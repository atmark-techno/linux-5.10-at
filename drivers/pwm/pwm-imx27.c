// SPDX-License-Identifier: GPL-2.0
/*
 * simple driver for PWM (Pulse Width Modulator) controller
 *
 * Derived from pxa PWM driver by eric miao <eric.miao@marvell.com>
 *
 * Limitations:
 * - When disabled the output is driven to 0 independent of the configured
 *   polarity.
 */

#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/slab.h>

#define MX3_PWMCR			0x00    /* PWM Control Register */
#define MX3_PWMSR			0x04    /* PWM Status Register */
#define MX3_PWMSAR			0x0C    /* PWM Sample Register */
#define MX3_PWMPR			0x10    /* PWM Period Register */
#define MX3_PWMCNR			0x14    /* PWM Counter Register */

#define MX3_PWMCR_FWM			GENMASK(27, 26)
#define MX3_PWMCR_STOPEN		BIT(25)
#define MX3_PWMCR_DOZEN			BIT(24)
#define MX3_PWMCR_WAITEN		BIT(23)
#define MX3_PWMCR_DBGEN			BIT(22)
#define MX3_PWMCR_BCTR			BIT(21)
#define MX3_PWMCR_HCTR			BIT(20)

#define MX3_PWMCR_POUTC			GENMASK(19, 18)
#define MX3_PWMCR_POUTC_NORMAL		0
#define MX3_PWMCR_POUTC_INVERTED	1
#define MX3_PWMCR_POUTC_OFF		2

#define MX3_PWMCR_CLKSRC		GENMASK(17, 16)
#define MX3_PWMCR_CLKSRC_OFF		0
#define MX3_PWMCR_CLKSRC_IPG		1
#define MX3_PWMCR_CLKSRC_IPG_HIGH	2
#define MX3_PWMCR_CLKSRC_IPG_32K	3

#define MX3_PWMCR_PRESCALER		GENMASK(15, 4)

#define MX3_PWMCR_SWR			BIT(3)

#define MX3_PWMCR_REPEAT		GENMASK(2, 1)
#define MX3_PWMCR_REPEAT_1X		0
#define MX3_PWMCR_REPEAT_2X		1
#define MX3_PWMCR_REPEAT_4X		2
#define MX3_PWMCR_REPEAT_8X		3

#define MX3_PWMCR_EN			BIT(0)

#define MX3_PWMSR_FWE			BIT(6)
#define MX3_PWMSR_CMP			BIT(5)
#define MX3_PWMSR_ROV			BIT(4)
#define MX3_PWMSR_FE			BIT(3)

#define MX3_PWMSR_FIFOAV		GENMASK(2, 0)
#define MX3_PWMSR_FIFOAV_EMPTY		0
#define MX3_PWMSR_FIFOAV_1WORD		1
#define MX3_PWMSR_FIFOAV_2WORDS		2
#define MX3_PWMSR_FIFOAV_3WORDS		3
#define MX3_PWMSR_FIFOAV_4WORDS		4

#define MX3_PWMCR_PRESCALER_SET(x)	FIELD_PREP(MX3_PWMCR_PRESCALER, (x) - 1)
#define MX3_PWMCR_PRESCALER_GET(x)	(FIELD_GET(MX3_PWMCR_PRESCALER, \
						   (x)) + 1)

#define MX3_PWM_SWR_LOOP		5

/* PWMPR register value of 0xffff has the same effect as 0xfffe */
#define MX3_PWMPR_MAX			0xfffe

struct pwm_imx27_chip {
	struct clk	*clk_ipg;
	struct clk	*clk_per;
	struct clk	*clk_32k;
	void __iomem	*mmio_base;
	struct pwm_chip	chip;

	/*
	 * The driver cannot read the current duty cycle from the hardware if
	 * the hardware is disabled. Cache the last programmed duty cycle
	 * value to return in that case.
	 */
	unsigned int duty_cycle;
};

#define to_pwm_imx27_chip(chip)	container_of(chip, struct pwm_imx27_chip, chip)

static int pwm_imx27_clk_prepare_enable(struct pwm_imx27_chip *imx)
{
	int ret;

	if (imx->clk_32k) {
		ret = clk_prepare_enable(imx->clk_32k);
		if (ret)
			goto err1;
	}

	ret = clk_prepare_enable(imx->clk_ipg);
	if (ret)
		goto err2;

	ret = clk_prepare_enable(imx->clk_per);
	if (ret)
		goto err3;

	return 0;
err3:
	clk_disable_unprepare(imx->clk_ipg);
err2:
	if (imx->clk_32k)
		clk_disable_unprepare(imx->clk_32k);
err1:
	return ret;
}

static void pwm_imx27_clk_disable_unprepare(struct pwm_imx27_chip *imx)
{
	clk_disable_unprepare(imx->clk_per);
	clk_disable_unprepare(imx->clk_ipg);
	if (imx->clk_32k)
		clk_disable_unprepare(imx->clk_32k);
}

static void pwm_imx27_get_state(struct pwm_chip *chip,
				struct pwm_device *pwm, struct pwm_state *state)
{
	struct pwm_imx27_chip *imx = to_pwm_imx27_chip(chip);
	u32 period, prescaler, pwm_clk, val;
	u64 tmp;
	int ret;

	ret = pwm_imx27_clk_prepare_enable(imx);
	if (ret < 0)
		return;

	val = readl(imx->mmio_base + MX3_PWMCR);

	if (val & MX3_PWMCR_EN)
		state->enabled = true;
	else
		state->enabled = false;

	switch (FIELD_GET(MX3_PWMCR_POUTC, val)) {
	case MX3_PWMCR_POUTC_NORMAL:
		state->polarity = PWM_POLARITY_NORMAL;
		break;
	case MX3_PWMCR_POUTC_INVERTED:
		state->polarity = PWM_POLARITY_INVERSED;
		break;
	default:
		dev_warn(chip->dev, "can't set polarity, output disconnected");
	}

	prescaler = MX3_PWMCR_PRESCALER_GET(val);
	pwm_clk = clk_get_rate(imx->clk_per);
	val = readl(imx->mmio_base + MX3_PWMPR);
	period = val >= MX3_PWMPR_MAX ? MX3_PWMPR_MAX : val;

	/* PWMOUT (Hz) = PWMCLK / (PWMPR + 2) */
	tmp = NSEC_PER_SEC * (u64)(period + 2) * prescaler;
	state->period = DIV_ROUND_UP_ULL(tmp, pwm_clk);

	/*
	 * PWMSAR can be read only if PWM is enabled. If the PWM is disabled,
	 * use the cached value.
	 */
	if (state->enabled)
		val = readl(imx->mmio_base + MX3_PWMSAR);
	else
		val = imx->duty_cycle;

	tmp = NSEC_PER_SEC * (u64)(val) * prescaler;
	state->duty_cycle = DIV_ROUND_UP_ULL(tmp, pwm_clk);

	pwm_imx27_clk_disable_unprepare(imx);
}

static void pwm_imx27_sw_reset(struct pwm_chip *chip)
{
	struct pwm_imx27_chip *imx = to_pwm_imx27_chip(chip);
	struct device *dev = chip->dev;
	int wait_count = 0;
	u32 cr;

	writel(MX3_PWMCR_SWR, imx->mmio_base + MX3_PWMCR);
	do {
		usleep_range(200, 1000);
		cr = readl(imx->mmio_base + MX3_PWMCR);
	} while ((cr & MX3_PWMCR_SWR) &&
		 (wait_count++ < MX3_PWM_SWR_LOOP));

	if (cr & MX3_PWMCR_SWR)
		dev_warn(dev, "software reset timeout\n");
}

static void pwm_imx27_wait_fifo_slot(struct pwm_chip *chip,
				     struct pwm_device *pwm)
{
	struct pwm_imx27_chip *imx = to_pwm_imx27_chip(chip);
	struct device *dev = chip->dev;
	unsigned int period_ms;
	int fifoav;
	u32 sr;

	sr = readl(imx->mmio_base + MX3_PWMSR);
	fifoav = FIELD_GET(MX3_PWMSR_FIFOAV, sr);
	if (fifoav == MX3_PWMSR_FIFOAV_4WORDS) {
		period_ms = DIV_ROUND_UP_ULL(pwm_get_period(pwm),
					 NSEC_PER_MSEC);
		msleep(period_ms);

		sr = readl(imx->mmio_base + MX3_PWMSR);
		if (fifoav == FIELD_GET(MX3_PWMSR_FIFOAV, sr))
			dev_warn(dev, "there is no free FIFO slot\n");
	}
}

static int pwm_imx27_apply(struct pwm_chip *chip, struct pwm_device *pwm,
			   const struct pwm_state *state)
{
	unsigned long period_cycles, duty_cycles, prescale, period_us, tmp;
	struct pwm_imx27_chip *imx = to_pwm_imx27_chip(chip);
	struct pwm_state cstate;
	unsigned long long c;
	unsigned long long clkrate;
	unsigned long flags;
	int val;
	int ret;
	u32 cr;

	pwm_get_state(pwm, &cstate);

	clkrate = clk_get_rate(imx->clk_per);
	c = clkrate * state->period;

	do_div(c, NSEC_PER_SEC);
	period_cycles = c;

	prescale = period_cycles / 0x10000 + 1;

	period_cycles /= prescale;
	c = clkrate * state->duty_cycle;
	do_div(c, NSEC_PER_SEC);
	duty_cycles = c;
	duty_cycles /= prescale;

	/*
	 * according to imx pwm RM, the real period value should be PERIOD
	 * value in PWMPR plus 2.
	 */
	if (period_cycles > 2)
		period_cycles -= 2;
	else
		period_cycles = 0;

	/*
	 * Wait for a free FIFO slot if the PWM is already enabled, and flush
	 * the FIFO if the PWM was disabled and is about to be enabled.
	 */
	if (cstate.enabled) {
		pwm_imx27_wait_fifo_slot(chip, pwm);
	} else {
		ret = pwm_imx27_clk_prepare_enable(imx);
		if (ret)
			return ret;

		pwm_imx27_sw_reset(chip);
	}

	val = readl(imx->mmio_base + MX3_PWMPR);
	val = val >= MX3_PWMPR_MAX ? MX3_PWMPR_MAX : val;
	cr = readl(imx->mmio_base + MX3_PWMCR);
	tmp = NSEC_PER_SEC * (u64)(val + 2) * MX3_PWMCR_PRESCALER_GET(cr);
	tmp = DIV_ROUND_UP_ULL(tmp, clkrate);
	period_us = DIV_ROUND_UP_ULL(tmp, 1000);

	/*
	 * ERR051198:
	 * PWM: PWM output may not function correctly if the FIFO is empty when
	 * a new SAR value is programmed
	 *
	 * Description:
	 * When the PWM FIFO is empty, a new value programmed to the PWM Sample
	 * register (PWM_PWMSAR) will be directly applied even if the current
	 * timer period has not expired.
	 *
	 * If the new SAMPLE value programmed in the PWM_PWMSAR register is
	 * less than the previous value, and the PWM counter register
	 * (PWM_PWMCNR) that contains the current COUNT value is greater than
	 * the new programmed SAMPLE value, the current period will not flip
	 * the level. This may result in an output pulse with a duty cycle of
	 * 100%.
	 *
	 * Consider a change from
	 *     ________
	 *    /        \______/
	 *    ^      *        ^
	 * to
	 *     ____
	 *    /    \__________/
	 *    ^               ^
	 * At the time marked by *, the new write value will be directly applied
	 * to SAR even the current period is not over if FIFO is empty.
	 *
	 *     ________        ____________________
	 *    /        \______/                    \__________/
	 *    ^               ^      *        ^               ^
	 *    |<-- old SAR -->|               |<-- new SAR -->|
	 *
	 * That is the output is active for a whole period.
	 *
	 * Workaround:
	 * Check new SAR less than old SAR and current counter is in errata
	 * windows, write extra old SAR into FIFO and new SAR will effect at
	 * next period.
	 *
	 * Sometime period is quite long, such as over 1 second. If add old SAR
	 * into FIFO unconditional, new SAR have to wait for next period. It
	 * may be too long.
	 *
	 * Turn off the interrupt to ensure that not IRQ and schedule happen
	 * during above operations. If any irq and schedule happen, counter
	 * in PWM will be out of data and take wrong action.
	 *
	 * Add a safety margin 1.5us because it needs some time to complete
	 * IO write.
	 *
	 * Use writel_relaxed() to minimize the interval between two writes to
	 * the SAR register to increase the fastest PWM frequency supported.
	 *
	 * When the PWM period is longer than 2us(or <500kHz), this workaround
	 * can solve this problem. No software workaround is available if PWM
	 * period is shorter than IO write. Just try best to fill old data
	 * into FIFO.
	 */
	c = clkrate * 1500;
	do_div(c, NSEC_PER_SEC);

	local_irq_save(flags);
	val = FIELD_GET(MX3_PWMSR_FIFOAV, readl_relaxed(imx->mmio_base + MX3_PWMSR));

	if (duty_cycles < imx->duty_cycle && (cr & MX3_PWMCR_EN)) {
		if (period_us < 2) { /* 2us = 500 kHz */
			/* Best effort attempt to fix up >500 kHz case */
			udelay(3 * period_us);
			writel_relaxed(imx->duty_cycle, imx->mmio_base + MX3_PWMSAR);
			writel_relaxed(imx->duty_cycle, imx->mmio_base + MX3_PWMSAR);
		} else if (val < MX3_PWMSR_FIFOAV_2WORDS) {
			val = readl_relaxed(imx->mmio_base + MX3_PWMCNR);
			/*
			 * If counter is close to period, controller may roll over when
			 * next IO write.
			 */
			if ((val + c >= duty_cycles && val < imx->duty_cycle) ||
			    val + c >= period_cycles)
				writel_relaxed(imx->duty_cycle, imx->mmio_base + MX3_PWMSAR);
		}
	}
	writel_relaxed(duty_cycles, imx->mmio_base + MX3_PWMSAR);
	local_irq_restore(flags);

	writel(period_cycles, imx->mmio_base + MX3_PWMPR);

	/*
	 * Store the duty cycle for future reference in cases where the
	 * MX3_PWMSAR register can't be read (i.e. when the PWM is disabled).
	 */
	imx->duty_cycle = duty_cycles;

	cr = MX3_PWMCR_PRESCALER_SET(prescale) |
	     MX3_PWMCR_STOPEN | MX3_PWMCR_DOZEN | MX3_PWMCR_WAITEN |
	     FIELD_PREP(MX3_PWMCR_CLKSRC, MX3_PWMCR_CLKSRC_IPG_HIGH) |
	     MX3_PWMCR_DBGEN;

	if (state->polarity == PWM_POLARITY_INVERSED)
		cr |= FIELD_PREP(MX3_PWMCR_POUTC,
				MX3_PWMCR_POUTC_INVERTED);

	if (state->enabled)
		cr |= MX3_PWMCR_EN;

	writel(cr, imx->mmio_base + MX3_PWMCR);

	if (!state->enabled)
		pwm_imx27_clk_disable_unprepare(imx);

	return 0;
}

static const struct pwm_ops pwm_imx27_ops = {
	.apply = pwm_imx27_apply,
	.get_state = pwm_imx27_get_state,
	.owner = THIS_MODULE,
};

static const struct of_device_id pwm_imx27_dt_ids[] = {
	{ .compatible = "fsl,imx27-pwm", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, pwm_imx27_dt_ids);

static int pwm_imx27_probe(struct platform_device *pdev)
{
	struct pwm_imx27_chip *imx;
	int ret;
	u32 pwmcr;

	imx = devm_kzalloc(&pdev->dev, sizeof(*imx), GFP_KERNEL);
	if (imx == NULL)
		return -ENOMEM;

	platform_set_drvdata(pdev, imx);

	imx->clk_ipg = devm_clk_get(&pdev->dev, "ipg");
	if (IS_ERR(imx->clk_ipg)) {
		int ret = PTR_ERR(imx->clk_ipg);

		if (ret != -EPROBE_DEFER)
			dev_err(&pdev->dev,
				"getting ipg clock failed with %d\n",
				ret);
		return ret;
	}

	imx->clk_per = devm_clk_get(&pdev->dev, "per");
	if (IS_ERR(imx->clk_per)) {
		int ret = PTR_ERR(imx->clk_per);

		if (ret != -EPROBE_DEFER)
			dev_err(&pdev->dev,
				"failed to get peripheral clock: %d\n",
				ret);

		return ret;
	}

	imx->clk_32k = devm_clk_get_optional(&pdev->dev, "32k");
	if (IS_ERR(imx->clk_32k)) {
		dev_err(&pdev->dev, "getting 32k clock failed with %ld\n",
				PTR_ERR(imx->clk_32k));
		return PTR_ERR(imx->clk_32k);
	}

	imx->chip.ops = &pwm_imx27_ops;
	imx->chip.dev = &pdev->dev;
	imx->chip.base = -1;
	imx->chip.npwm = 1;

	imx->chip.of_xlate = of_pwm_xlate_with_flags;
	imx->chip.of_pwm_n_cells = 3;

	imx->mmio_base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(imx->mmio_base))
		return PTR_ERR(imx->mmio_base);

	ret = pwm_imx27_clk_prepare_enable(imx);
	if (ret)
		return ret;

	/* keep clks on if pwm is running */
	pwmcr = readl(imx->mmio_base + MX3_PWMCR);
	if (!(pwmcr & MX3_PWMCR_EN))
		pwm_imx27_clk_disable_unprepare(imx);

	return pwmchip_add(&imx->chip);
}

static int pwm_imx27_remove(struct platform_device *pdev)
{
	struct pwm_imx27_chip *imx;

	imx = platform_get_drvdata(pdev);

	return pwmchip_remove(&imx->chip);
}

static struct platform_driver imx_pwm_driver = {
	.driver = {
		.name = "pwm-imx27",
		.of_match_table = pwm_imx27_dt_ids,
	},
	.probe = pwm_imx27_probe,
	.remove = pwm_imx27_remove,
};
module_platform_driver(imx_pwm_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Sascha Hauer <s.hauer@pengutronix.de>");
