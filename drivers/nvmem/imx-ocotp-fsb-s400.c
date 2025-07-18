// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2021 NXP
 */

#include <linux/of_address.h>
#include <linux/dev_printk.h>
#include <linux/errno.h>
#include <linux/firmware/imx/ele_base_msg.h>
#include <linux/firmware/imx/se_fw_inc.h>
#include <linux/io.h>
#include <linux/etherdevice.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/nvmem-consumer.h>
#include <linux/nvmem-provider.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#define LOCK_CFG	0x01
#define ECID		0x02
#define UNIQ_ID		0x07
#define OTFAD_CFG	0x17
#define MAPPING_SIZE	0x20
#define FUSE_ACC_DIS	0x28

enum soc_type {
	IMX8ULP,
	IMX93,
	IMX95,
};

struct bank_2_reg {
	unsigned int bank;
	unsigned int reg;
	bool flag;
};

struct imx_fsb_s400_hw {
	enum soc_type soc;
	unsigned int fsb_otp_shadow;
	const uint8_t se_pdev_name[20];
	const struct bank_2_reg fsb_bank_reg[MAPPING_SIZE];
	const uint16_t* nonecc_fuse_banks;
	bool oscca_fuse_read;
	const u8 *pf_mac_offset_list;
};

struct imx_fsb_s400_fuse {
	void __iomem *regs;
	struct nvmem_config config;
	struct mutex lock;
	struct device *se_dev;
	const struct imx_fsb_s400_hw *hw;
	bool fsb_read_dis;
	u8 pfn;
};

static int read_words_via_s400_api(u32 *buf, unsigned int fuse_base,
				   unsigned int num, struct device *se_dev)
{
	unsigned int i;
	int err = 0;

	for (i = 0; i < num; i++)
		err = read_common_fuse(se_dev, fuse_base + i, buf + i);

	return err;
}

static int read_words_via_fsb(void *priv, unsigned int bank, u32 *buf)
{
	struct imx_fsb_s400_fuse *fuse = priv;
	void __iomem *regs = fuse->regs + fuse->hw->fsb_otp_shadow;
	unsigned int i;
	unsigned int reg_id = UINT_MAX;
	unsigned int size = ARRAY_SIZE(fuse->hw->fsb_bank_reg);

	for (i = 0; i < size; i++) {
		if (fuse->hw->fsb_bank_reg[i].bank == bank) {
			reg_id = fuse->hw->fsb_bank_reg[i].reg;
			break;
		}
	}

	if (reg_id != UINT_MAX) {
		size = fuse->hw->fsb_bank_reg[i].flag ? 4 : 8;

		for (i = 0; i < size; i++) {
			if (size == 4) {
				*buf = readl_relaxed(regs + (reg_id + i/2) * 4);
				if (i%2)
					*buf >>= 16;
				*buf &= 0xFFFF;
			} else {
				*buf = readl_relaxed(regs + (reg_id + i) * 4);
			}
			buf = buf + 1;
		}
	}

	return 0;
}

static int read_nwords_via_fsb(void __iomem *regs, u32 *buf, u32 fuse_base, u32 num)
{
	unsigned int i;

	for (i = 0; i < num; i++) {
		*buf = readl_relaxed(regs + (fuse_base + i) * 4);
		buf = buf + 1;
	}

	return 0;
}

static int fsb_s400_fuse_read(void *priv, unsigned int offset, void *val,
			      size_t bytes)
{
	struct imx_fsb_s400_fuse *fuse = priv;
	void __iomem *regs = fuse->regs + fuse->hw->fsb_otp_shadow;
	unsigned int num_bytes, bank;
	u32 *buf;
	int err, i;

	num_bytes = round_up(2048, 4);
	buf = kzalloc(num_bytes, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	err = -EINVAL;

	mutex_lock(&fuse->lock);
	if (fuse->hw->soc == IMX8ULP) {
		for (bank = 0; bank < 63; bank++) {
			switch (bank) {
			case 0:
				break;
			case LOCK_CFG:
				err = read_words_via_s400_api(&buf[8], 8, 8,
							      fuse->se_dev);
				if (err)
					goto ret;
				break;
			case ECID:
				err = read_words_via_s400_api(&buf[16], 16, 8,
							      fuse->se_dev);
				if (err)
					goto ret;
				break;
			case UNIQ_ID:
				err = read_common_fuse(fuse->se_dev,
						       OTP_UNIQ_ID,
						       &buf[56]);
				if (err)
					goto ret;
				break;
			case OTFAD_CFG:
				err = read_common_fuse(fuse->se_dev,
						       OTFAD_CONFIG, &buf[184]);
				if (err)
					goto ret;
				break;
			case 15:
				err = read_words_via_s400_api(&buf[120], 120, 8,
							      fuse->se_dev);
				if (err)
					goto ret;
				break;
			case 25:
			case 26:
			case 27:
				err = read_words_via_s400_api(&buf[200], 200, 24,
							      fuse->se_dev);
				if (err)
					goto ret;
				break;
			case 32:
			case 33:
			case 34:
			case 35:
			case 36:
				err = read_words_via_s400_api(&buf[256], 256, 40,
							      fuse->se_dev);
				if (err)
					goto ret;
				break;
			case 49:
			case 50:
			case 51:
				err = read_words_via_s400_api(&buf[392], 392, 24,
							      fuse->se_dev);
				if (err)
					goto ret;
				break;
			default:
				err = read_words_via_fsb(priv, bank, &buf[bank * 8]);
				break;
			}
		}
	} else if (fuse->hw->soc == IMX93) {
		for (bank = 0; bank < 6; bank++) {
			if (fuse->fsb_read_dis)
				read_words_via_s400_api(&buf[bank * 8], bank * 8,
							8, fuse->se_dev);
			else
				read_nwords_via_fsb(regs, &buf[bank * 8], bank * 8, 8);
		}

		if (fuse->fsb_read_dis)
			read_words_via_s400_api(&buf[48], 48, 4, fuse->se_dev);
		else
			read_nwords_via_fsb(regs, &buf[48], 48, 4); /* OTP_UNIQ_ID */

		err = read_words_via_s400_api(&buf[63], 63, 1, fuse->se_dev);
		if (err)
			goto ret;

		err = read_words_via_s400_api(&buf[128], 128, 16, fuse->se_dev);
		if (err)
			goto ret;

		err = read_words_via_s400_api(&buf[182], 182, 1, fuse->se_dev);
		if (err)
			goto ret;

		err = read_words_via_s400_api(&buf[188], 188, 1, fuse->se_dev);
		if (err)
			goto ret;

		for (bank = 39; bank < 64; bank++) {
			if (fuse->fsb_read_dis)
				read_words_via_s400_api(&buf[bank * 8], bank * 8,
					       		8, fuse->se_dev);
			else
				read_nwords_via_fsb(regs, &buf[bank * 8], bank * 8, 8);
		}
	} else if (fuse->hw->soc == IMX95) {
		buf[0] = readl_relaxed(regs + 0 * 4) & 0xffff;
		buf[7] = readl_relaxed(regs + 7 * 4) & 0xffff;
		buf[9] = readl_relaxed(regs + 9 * 4) & 0xffff;
		buf[10] = readl_relaxed(regs + 10 * 4) & 0xffff;
		buf[11] = readl_relaxed(regs + 11 * 4) & 0xffff;
		for (i = 12; i < 36; i++)
			buf[i] = readl_relaxed(regs + i * 4);
		buf[36] = readl_relaxed(regs + 36 * 4) & 0xffff;
		buf[37] = readl_relaxed(regs + 37 * 4) & 0xffff;
		for (i = 38; i < 52; i++)
			buf[i] = readl_relaxed(regs + i * 4);
		buf[317] = readl_relaxed(regs + 317 * 4) & 0xffff;
		buf[318] = readl_relaxed(regs + 318 * 4) & 0xffff;
		for (i = 320; i < 327; i++)
			buf[i] = readl_relaxed(regs + i * 4);
		for (i = 328; i < 512; i++)
			buf[i] = readl_relaxed(regs + i * 4);

		read_words_via_s400_api(&buf[63], 63, 1, fuse->se_dev);
		read_words_via_s400_api(&buf[128], 128, 16, fuse->se_dev);
		read_words_via_s400_api(&buf[188], 188, 1, fuse->se_dev);

		err = 0;

		fuse->pfn = offset >> 12 & 0xf;
		offset = offset & 0xfff;
	}

	memcpy(val, (u8 *)(buf) + offset, bytes);

ret:
	kfree(buf);
	mutex_unlock(&fuse->lock);

	return err;
}

static int fsb_s400_fuse_write(void *priv, unsigned int offset, void *val, size_t bytes)
{
	struct imx_fsb_s400_fuse *fuse = priv;
	u32 *buf = val;
	u16 index, bank;
	bool lock = false;
	int ret;

	/* allow only writing one complete OTP word at a time,
	 * and avoid locking a fuse to 0 data */
	if (bytes != 4 || offset % 4 != 0 || *buf == 0)
		return -EINVAL;

	index = offset / 4;
	bank = index / 8;

        /* Lock 8ULP ECC fuse word, so second programming will return failure.
         * iMX9 OTP can protect ECC fuse, so does not need it
         */
	if (fuse->hw->nonecc_fuse_banks) {
		const u16 *nonecc_bank;

		for (nonecc_bank = fuse->hw->nonecc_fuse_banks;
		     *nonecc_bank != (u16)-1;
		     nonecc_bank++) {
			if (*nonecc_bank == bank)
				break;
		}
		if (*nonecc_bank == (u16)-1)
			lock = true;
	}

	dev_info(fuse->config.dev, "writing %x to bank %d word %d, lock %d\n",
		 *buf, bank, index % 8, lock);

	mutex_lock(&fuse->lock);
	ret = ele_write_fuse(fuse->se_dev, index, *buf, lock);
	mutex_unlock(&fuse->lock);

	return ret;
}

#define LIFECYCLE_OFFSET 0x41c
#define LIFECYCLE_OPEN 0x8
#define LIFECYCLE_CLOSE 0x20

static ssize_t secureboot_show(struct device *dev,
                struct device_attribute *attr, char *buf)
{
	struct imx_fsb_s400_fuse *fuse = dev_get_drvdata(dev);
	u32 lifecycle = readl_relaxed(fuse->regs + LIFECYCLE_OFFSET) & 0x3ff;
	int rc, len;
	u32 events[5], count = 5;

	switch (lifecycle) {
	case LIFECYCLE_OPEN:
		len = sprintf(buf, "open\n");
		break;
	case LIFECYCLE_CLOSE:
		len = sprintf(buf, "close\n");
		break;
	default:
		len = sprintf(buf, "unknown: %#x\n", lifecycle);
	}

	rc = ele_get_events(fuse->se_dev, events, &count);
	if (rc) {
		len += sprintf(buf+len, "could not get events: %d\n", rc);
		count = 0;
	}
	for (rc = 0; rc < count; rc++) {
		len += sprintf(buf+len, "%#x\n", events[rc]);
	}

	return len;
}

static ssize_t secureboot_store(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t len)
{
	struct imx_fsb_s400_fuse *fuse = dev_get_drvdata(dev);
	u32 lifecycle = readl_relaxed(fuse->regs + LIFECYCLE_OFFSET) & 0x3ff;

	if (!sysfs_streq(buf, "close")) {
		dev_err(dev, "Can only write 'close'\n");
		return -EINVAL;
	}

	if (lifecycle != LIFECYCLE_OPEN) {
		dev_err(dev, "Cannot close device not open (%#x)\n", lifecycle);
		return -EINVAL;
	}

	return ele_forward_lifecycle(fuse->se_dev, LIFECYCLE_OPEN);
}

static DEVICE_ATTR_ADMIN_RW(secureboot);

static struct attribute *secureboot_attributes[] = {
	&dev_attr_secureboot.attr,
	NULL
};

static const struct attribute_group secureboot_attr_group = {
	.attrs = secureboot_attributes,
};

static int imx_fsb_s400_fuse_probe(struct platform_device *pdev)
{
	struct imx_fsb_s400_fuse *fuse;
	struct nvmem_device *nvmem;
	struct device_node *np;
	void __iomem *reg;
	int rc;
	u32 v;

	fuse = devm_kzalloc(&pdev->dev, sizeof(*fuse), GFP_KERNEL);
	if (!fuse)
		return -ENOMEM;

	fuse->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(fuse->regs))
		return PTR_ERR(fuse->regs);

	fuse->config.dev = &pdev->dev;
	fuse->config.name = "fsb_s400_fuse";
	fuse->config.id = NVMEM_DEVID_AUTO;
	fuse->config.owner = THIS_MODULE;
	fuse->config.size = 2048; /* 64 Banks */
	fuse->config.reg_read = fsb_s400_fuse_read;
	fuse->config.reg_write = fsb_s400_fuse_write;
	fuse->config.priv = fuse;
	mutex_init(&fuse->lock);
	fuse->hw = of_device_get_match_data(&pdev->dev);

	if (fuse->hw->oscca_fuse_read) {
		np = of_find_compatible_node(NULL, NULL, "fsl,imx93-aonmix-ns-syscfg");
		if (!np)
			return -ENODEV;

		reg = of_iomap(np, 0);
		if (!reg)
			return -ENOMEM;

		v = readl_relaxed(reg + FUSE_ACC_DIS);
		if (v & BIT(0))
			fuse->fsb_read_dis = true;
		else
			fuse->fsb_read_dis = false;
	} else {
		fuse->fsb_read_dis = false;
	}

	nvmem = devm_nvmem_register(&pdev->dev, &fuse->config);
	if (IS_ERR(nvmem)) {
		dev_err(&pdev->dev, "failed to register fuse nvmem device\n");
		return PTR_ERR(nvmem);
	}

	fuse->se_dev = get_se_dev(fuse->hw->se_pdev_name);
	if (!fuse->se_dev) {
		dev_err(&pdev->dev, "No se device\n");
		return -ENODEV;
	}
	dev_dbg(&pdev->dev, "fuse nvmem device registered successfully\n");

	// create sysfs file for secureboot status (not a fuse)
	rc = devm_device_add_group(&pdev->dev, &secureboot_attr_group);
	if (rc) {
		dev_err(&pdev->dev, "Could not create secureboot attrs: %d\n", rc);
		return rc;
	}
	platform_set_drvdata(pdev, fuse);

	return 0;
}

static const u8 imx95_pf_mac_offset_list[] = { 0, 3, 6 };


static const u16 imx8ulp_nonecc_fuse_banks[] = {
	0, 1, 8, 12, 16, 22, 24, 25, 26, 27, 36, 41, 51, 56, -1
};

static const struct imx_fsb_s400_hw imx8ulp_fsb_s400_hw = {
	.soc = IMX8ULP,
	.fsb_otp_shadow = 0x800,
	.fsb_bank_reg = {
		[0] = { 3, 0 },
		[1] = { 4, 8 },
		[2] = { 5, 64 },
		[3] = { 6, 72 },
		[4] = { 8, 80, true },
		[5] = { 24, 84, true },
		[6] = { 26, 88, true },
		[7] = { 27, 92, true },
		[8] = { 28, 96 },
		[9] = { 29, 104 },
		[10] = { 30, 112 },
		[11] = { 31, 120 },
		[12] = { 37, 128 },
		[13] = { 38, 136 },
		[14] = { 39, 144 },
		[15] = { 40, 152 },
		[16] = { 41, 160 },
		[17] = { 42, 168 },
		[18] = { 43, 176 },
		[19] = { 44, 184 },
		[20] = { 45, 192 },
		[21] = { 46, 200 },
	},
	.nonecc_fuse_banks = imx8ulp_nonecc_fuse_banks,
	.oscca_fuse_read = false,
	.pf_mac_offset_list = NULL,
	.se_pdev_name = "se-fw2",
};

static const struct of_device_id imx_fsb_s400_fuse_match[] = {
	{ .compatible = "fsl,imx8ulp-ocotp", .data = &imx8ulp_fsb_s400_hw, },
	// imx93 has been removed because post process hook not supported in this kernel
	{},
};

static struct platform_driver imx_fsb_s400_fuse_driver = {
	.driver = {
		.name = "fsl-ocotp-fsb-s400",
		.of_match_table = imx_fsb_s400_fuse_match,
	},
	.probe = imx_fsb_s400_fuse_probe,
};
MODULE_DEVICE_TABLE(of, imx_fsb_s400_fuse_match);
module_platform_driver(imx_fsb_s400_fuse_driver);

MODULE_AUTHOR("NXP");
MODULE_DESCRIPTION("i.MX FSB/S400-API ocotp fuse box driver");
MODULE_LICENSE("GPL v2");
