// SPDX-License-Identifier: GPL-2.0
/*
 * GPIO-based Reset Driver for LWB5+ addon board of Armadillo 640
 *
 * Copyright (c) 2024 Atmark Techno, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>

struct lwb5p_reset_data {
	struct gpio_desc *pwr_en;
	struct gpio_desc *wlan_rst;
	struct gpio_desc *bt_rst;
	struct mutex gpio_lock;
	struct work_struct work;
};

/* delayed init to avoid blocking on probe() */
static void lwb5p_reset_delayed_init(struct work_struct *ws)
{
	struct lwb5p_reset_data *data =
		container_of(ws, struct lwb5p_reset_data, work);

	/* Let power settle before enabling bluetooth. Need at least 10ms in
	 * case the board pull-up brought the pins up faster than linux boot,
	 * make this 20ms.
	 */
	msleep(20);
	gpiod_set_value_cansleep(data->bt_rst, 0);

	/* Bringing wlan up together with bluetooth makes wlan occasionally
	 * fail. Wait long enough. (20ms fail, more tests to follow)
	 */
	msleep(100);
	gpiod_set_value_cansleep(data->wlan_rst, 0);
	mutex_unlock(&data->gpio_lock);
}

static ssize_t wlan_rst_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct lwb5p_reset_data *data = platform_get_drvdata(pdev);
	bool val;
	int ret;

	ret = kstrtobool(buf, &val);
	if (ret)
		return ret;

	mutex_lock(&data->gpio_lock);

	gpiod_set_value_cansleep(data->wlan_rst, 1);

	/* false-ish values just turn off */
	if (!val)
		goto out;

	msleep(20);
	gpiod_set_value_cansleep(data->wlan_rst, 0);

out:
	mutex_unlock(&data->gpio_lock);
	return count;
}

static ssize_t wlan_rst_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct lwb5p_reset_data *data = platform_get_drvdata(pdev);

	int val = gpiod_get_value_cansleep(data->wlan_rst);

	/* we show if it's enabled e.g. not reset */
	return sysfs_emit(buf, "%d\n", !val);
}
static DEVICE_ATTR_RW(wlan_rst);

static ssize_t bt_rst_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct lwb5p_reset_data *data = platform_get_drvdata(pdev);
	bool val;
	int ret;

	ret = kstrtobool(buf, &val);
	if (ret)
		return ret;

	mutex_lock(&data->gpio_lock);

	gpiod_set_value_cansleep(data->bt_rst, 1);

	/* false-ish values just turn off */
	if (!val)
		goto out;

	msleep(20);
	gpiod_set_value_cansleep(data->bt_rst, 0);

out:
	mutex_unlock(&data->gpio_lock);
	return count;
}

static ssize_t bt_rst_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct lwb5p_reset_data *data = platform_get_drvdata(pdev);

	int val = gpiod_get_value_cansleep(data->bt_rst);

	/* we show if it's enabled e.g. not reset */
	return sysfs_emit(buf, "%d\n", !val);
}
static DEVICE_ATTR_RW(bt_rst);

static struct attribute *lwb5p_reset_attrs[] = {
	&dev_attr_wlan_rst.attr,
	&dev_attr_bt_rst.attr,
	NULL
};

static struct attribute_group lwb5p_reset_attr_group = {
	.name = "reset",
	.attrs = lwb5p_reset_attrs,
};

static int lwb5p_reset_probe(struct platform_device *pdev)
{
	struct lwb5p_reset_data *data;
	int ret;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	platform_set_drvdata(pdev, data);
	mutex_init(&data->gpio_lock);

	data->wlan_rst = devm_gpiod_get(&pdev->dev, "lwb5p-wlan-rst", GPIOD_OUT_HIGH);
	if (IS_ERR(data->wlan_rst))
		return dev_err_probe(&pdev->dev, PTR_ERR(data->wlan_rst),
				     "lwb5p-wlan-rst property missing\n");

	data->bt_rst = devm_gpiod_get(&pdev->dev, "lwb5p-bt-rst", GPIOD_OUT_HIGH);
	if (IS_ERR(data->bt_rst))
		return dev_err_probe(&pdev->dev, PTR_ERR(data->bt_rst),
				     "lwb5p-bt-rst property missing\n");

	data->pwr_en = devm_gpiod_get(&pdev->dev, "lwb5p-pwr-en", GPIOD_OUT_HIGH);
	if (IS_ERR(data->pwr_en))
		return dev_err_probe(&pdev->dev, PTR_ERR(data->pwr_en),
				     "lwb5p-pwr-en property missing\n");

	ret = sysfs_create_group(&pdev->dev.kobj, &lwb5p_reset_attr_group);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "failed to create sysfs attrs\n");

	/* pullups will already have enabled both power and disabled reset
	 * pins by the time we get here, so we need a long-ish reset to
	 * get back in order: do this asynchronously through a work task
	 */
	INIT_WORK(&data->work, lwb5p_reset_delayed_init);
	mutex_lock(&data->gpio_lock);
	schedule_work(&data->work);

	return 0;
}

static const struct of_device_id lwb5p_reset_dt_ids[] = {
	{ .compatible = "lwb5p-reset" },
	{ }
};

static struct platform_driver lwb5p_reset_driver = {
	.probe = lwb5p_reset_probe,
	.driver = {
		.name = "lwb5p-reset",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(lwb5p_reset_dt_ids),
	},
};

static int __init lwb5p_reset_init(void)
{
	return platform_driver_register(&lwb5p_reset_driver);
}
module_init(lwb5p_reset_init);

static void __exit lwb5p_reset_exit(void)
{
	platform_driver_unregister(&lwb5p_reset_driver);
}
module_exit(lwb5p_reset_exit);

MODULE_AUTHOR("Atmark Techno, Inc.");
MODULE_DESCRIPTION("LWB5+ reset driver");
MODULE_LICENSE("GPL");
MODULE_DEVICE_TABLE(of, lwb5p_reset_dt_ids);
