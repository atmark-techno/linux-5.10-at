// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021 Atmark Techno, Inc. All Rights Reserved.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_fdt.h>

#define dtb_begin(f)    __dtb_##f##_begin
#define dtb_end(f)      __dtb_##f##_end
#define dtb_size(f)     (dtb_end(f) - dtb_begin(f))

#define extern_dtb(f)           \
extern uint8_t dtb_begin(f)[];  \
extern uint8_t dtb_end(f)[];

extern_dtb(armadillo_iotg_a6_rtc_nr3225sa);
extern_dtb(armadillo_iotg_a6_rtc_rv8803);

/* Atmark Techno Subboard A6 Revison Number */
#define SUBBOARD_REVISION_ATMARK_TECHNO_A6_5	(0x0005)

enum a6_subboard {
	RTC_UNKNOWN,
	RTC_NR3225SA,
	RTC_RV8803
};

static enum a6_subboard armadillo_iotg_a6_subboard_detect(struct device *dev)
{
	struct device_node *root;
	unsigned long revision;
	const char *system_revision;
	int ret;

	revision = ULONG_MAX;
	root = of_find_node_by_path("/");
	if (root) {
		ret = of_property_read_string(root, "revision-number",
					      &system_revision);
		if (ret == 0) {
			ret = kstrtoul(system_revision, 16, &revision);
			if (ret)
				revision = ULONG_MAX;
		}
	}

	dev_info(dev, "Armadillo-IoT Gateway A6 subboard\n");
	if (revision == ULONG_MAX) {
		dev_warn(dev, "revision undefined.");
		return RTC_NR3225SA;
	}

	dev_info(dev, "revision:%lu", revision);

	if (revision < SUBBOARD_REVISION_ATMARK_TECHNO_A6_5)
		return RTC_NR3225SA;

	return RTC_RV8803;
}

static int
armadillo_iotg_a6_subboard_apply_overlay(const enum a6_subboard subboard,
					 struct device *dev)
{
	void *begin;
	size_t size;
	int ovcs_id = 0;
	int ret;

	switch (subboard) {
	case RTC_NR3225SA:
		begin = dtb_begin(armadillo_iotg_a6_rtc_nr3225sa);
		size = dtb_size(armadillo_iotg_a6_rtc_nr3225sa);
		break;
	case RTC_RV8803:
		begin = dtb_begin(armadillo_iotg_a6_rtc_rv8803);
		size = dtb_size(armadillo_iotg_a6_rtc_rv8803);
		break;
	default:
		dev_warn(dev, "unknown subboard:%d\n", subboard);
		ret = -EINVAL;
		goto err;
	}

	ret = of_overlay_fdt_apply(begin, size, &ovcs_id);

err:
	return ret;
}

static int armadillo_iotg_a6_subboard_probe(struct platform_device *pdev)
{
	enum a6_subboard subboard;

	dev_dbg(&pdev->dev, "probe\n");

	subboard = armadillo_iotg_a6_subboard_detect(&pdev->dev);

	return armadillo_iotg_a6_subboard_apply_overlay(subboard, &pdev->dev);
}

static int armadillo_iotg_a6_subboard_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	dev_dbg(dev, "remove\n");

	return 0;
}

static struct of_device_id armadillo_iotg_a6_subboard_dt_ids[] = {
	{ .compatible = "armadillo_iotg_a6_subboard" },
	{ }
};

static struct platform_driver armadillo_iotg_a6_subboard_driver = {
	.driver		= {
		.name	= "armadillo_iotg_a6_subboard",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(armadillo_iotg_a6_subboard_dt_ids),
	},
	.probe		= armadillo_iotg_a6_subboard_probe,
	.remove		= armadillo_iotg_a6_subboard_remove,
};

static int __init armadillo_iotg_a6_subboard_init(void)
{
	int ret;

	ret = platform_driver_register(&armadillo_iotg_a6_subboard_driver);
	if (ret)
		printk(KERN_ERR "armadillo_iotg_a6_subboard: probe failed: %d\n", ret);

	return 0;
}
subsys_initcall_sync(armadillo_iotg_a6_subboard_init);

static void __exit armadillo_iotg_a6_subboard_exit(void)
{
	platform_driver_unregister(&armadillo_iotg_a6_subboard_driver);
}
module_exit(armadillo_iotg_a6_subboard_exit);

MODULE_AUTHOR("Atmark Techno, Inc.");
MODULE_DESCRIPTION("Armadillo-IoT Gateway A6 Subboard");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:armadillo_iotg_a6_subboard");
