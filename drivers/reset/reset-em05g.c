// SPDX-License-Identifier: GPL-2.0+
/*
 * GPIO-based Reset Driver for Quectel EM05-G
 *
 * Copyright (c) 2024 Atmark Techno, Inc. All Rights Reserved.
 *
 * Based on: gpio-reset.c
 *   Copyright 2013 Philipp Zabel, Pengutronix
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/reset-controller.h>
#include <linux/regulator/consumer.h>

#define EM05G_POWER_ON_WAIT_TIME_MS	(30)
#define EM05G_POWER_OFF_WAIT_TIME_MS	(100)
#define EM05G_RESET_ASSERT_TIME_MS	(460)

enum em05g_reset_power_ops {
	EM05G_POWER_TURN_ON = 0,
	EM05G_POWER_TURN_OFF,
};

enum em05g_reset_reset_ops {
	EM05G_RESET_DEASSERT = 0,
	EM05G_RESET_ASSERT,
};

struct em05g_reset_data {
	struct reset_controller_dev rcdev;
	struct device *dev;

	struct regulator *vbat;

	struct gpio_desc *reset; /* RESET# */
	struct gpio_desc *power; /* FULL_CARD_POWER_OFF# */

	struct mutex power_lock;
	struct mutex reset_lock;
};

#define to_em05g_reset_data(_rcdev)				\
	container_of(_rcdev, struct em05g_reset_data, rcdev)

static void em05g_reset_set_power(struct em05g_reset_data *data,
				  enum em05g_reset_power_ops ops)
{
	if (!data->power)
		return;

	dev_dbg(data->dev, "FULL_CARD_POWER_OFF: %s\n", !ops ? "on" : "off");

	gpiod_set_value_cansleep(data->power, !ops);
}

static void em05g_reset_set_reset(struct em05g_reset_data *data,
				  enum em05g_reset_reset_ops ops)
{
	if (!data->reset)
		return;

	dev_dbg(data->dev, "RESET: %s\n", !ops ? "de-assert" : "assert");

	gpiod_set_value_cansleep(data->reset, !ops);
}

static int em05g_reset_power_on(struct em05g_reset_data *data)
{
	int ret;

	dev_dbg(data->dev, "Power up start.\n");

	mutex_lock(&data->power_lock);

	ret = regulator_enable(data->vbat);
	if (ret) {
		dev_err(data->dev, "failed to enable vbat regulator\n");
		goto out;
	}

	em05g_reset_set_reset(data, EM05G_RESET_DEASSERT);
	msleep(EM05G_POWER_ON_WAIT_TIME_MS);
	em05g_reset_set_power(data, EM05G_POWER_TURN_ON);

out:
	mutex_unlock(&data->power_lock);

	return ret;
}

static int em05g_reset_power_off(struct em05g_reset_data *data)
{
	int ret;

	mutex_lock(&data->power_lock);

	em05g_reset_set_reset(data, EM05G_RESET_ASSERT);
	/* We need to wait for power off at least 100ms. */
	msleep(EM05G_POWER_OFF_WAIT_TIME_MS);
	em05g_reset_set_power(data, EM05G_POWER_TURN_OFF);

	ret = regulator_disable(data->vbat);
	if (ret)
		dev_err(data->dev, "failed to disable vbat regulator\n");

	mutex_unlock(&data->power_lock);

	dev_dbg(data->dev, "Power down end.\n");

	return ret;
}

static void em05g_reset_reset_assert(struct em05g_reset_data *data)
{
	dev_dbg(data->dev, "Reset start.\n");

	em05g_reset_set_reset(data, EM05G_RESET_ASSERT);
	msleep(EM05G_RESET_ASSERT_TIME_MS);
}

static void em05g_reset_reset_deassert(struct em05g_reset_data *data)
{
	em05g_reset_set_reset(data, EM05G_RESET_DEASSERT);

	dev_dbg(data->dev, "Reset end.\n");
}

static int em05g_reset_init_pdata(struct platform_device *pdev,
				  struct em05g_reset_data *data)
{
	data->vbat = devm_regulator_get(&pdev->dev, "vbat");
	if (IS_ERR(data->vbat))
		return dev_err_probe(&pdev->dev, PTR_ERR(data->vbat),
				     "vbat property missing\n");

	data->reset = devm_gpiod_get(&pdev->dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(data->reset))
		return dev_err_probe(&pdev->dev, PTR_ERR(data->reset),
				     "invalid reset gpio");

	data->power = devm_gpiod_get_optional(&pdev->dev, "fullcardpoweroff",
					      GPIOD_OUT_LOW);
	if (IS_ERR(data->power))
		return dev_err_probe(&pdev->dev, PTR_ERR(data->power),
				     "invalid power gpio");

	return 0;
}

static int em05g_reset_reset(struct reset_controller_dev *rcdev,
			     unsigned long id)
{
	struct em05g_reset_data *data = to_em05g_reset_data(rcdev);

	em05g_reset_reset_assert(data);
	em05g_reset_reset_deassert(data);

	return 0;
}

static struct reset_control_ops em05g_reset_ops = {
	.reset = em05g_reset_reset,
};

static ssize_t reset_store(struct device *dev,
			   struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct em05g_reset_data *data = platform_get_drvdata(pdev);
	int val, ret;

	ret = kstrtoint(buf, 0, &val);
	if (ret)
		return ret;

	if (val == 1) {
		em05g_reset_reset_assert(data);
		em05g_reset_reset_deassert(data);
	} else
		return -EINVAL;

	return ret ? : count;
}
static DEVICE_ATTR_WO(reset);

static ssize_t power_store(struct device *dev,
			   struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct em05g_reset_data *data = platform_get_drvdata(pdev);
	int val, ret;

	ret = kstrtoint(buf, 0, &val);
	if (ret)
		return ret;

	if (val)
		ret = em05g_reset_power_on(data);
	else
		ret = em05g_reset_power_off(data);

	return ret ? : count;
}
static DEVICE_ATTR_WO(power);

static struct attribute *em05g_reset_attrs[] = {
	&dev_attr_reset.attr,
	&dev_attr_power.attr,
	NULL
};

static struct attribute_group em05g_reset_attr_group = {
	.name = "reset",
	.attrs = em05g_reset_attrs,
};

static int of_em05g_reset_xlate(struct reset_controller_dev *rcdev,
				const struct of_phandle_args *reset_spec)
{
	if (WARN_ON(reset_spec->args_count != 0))
		return -EINVAL;

	return 0;
}

static int em05g_reset_probe(struct platform_device *pdev)
{
	struct em05g_reset_data *data;
	int ret;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	ret = em05g_reset_init_pdata(pdev, data);
	if (ret)
		return ret;

	mutex_init(&data->power_lock);
	mutex_init(&data->reset_lock);

	data->dev = &pdev->dev;
	platform_set_drvdata(pdev, data);

	data->rcdev.of_node = pdev->dev.of_node;
	data->rcdev.owner = THIS_MODULE;
	data->rcdev.nr_resets = 1;
	data->rcdev.ops = &em05g_reset_ops;
	data->rcdev.of_xlate = of_em05g_reset_xlate;
	ret = devm_reset_controller_register(&pdev->dev, &data->rcdev);
	if (ret) {
		dev_err(&pdev->dev, "failed to register device\n");
		return ret;
	}

	ret = sysfs_create_group(&pdev->dev.kobj, &em05g_reset_attr_group);
	if (ret) {
		dev_err(&pdev->dev, "failed to create sysfs attrs\n");
		return ret;
	}

	em05g_reset_power_on(data);

	return 0;
}

static int em05g_reset_remove(struct platform_device *pdev)
{
	struct em05g_reset_data *data = platform_get_drvdata(pdev);

	sysfs_remove_group(&pdev->dev.kobj, &em05g_reset_attr_group);
	em05g_reset_power_off(data);

	return 0;
}

static void em05g_reset_shutdown(struct platform_device *pdev)
{
	struct em05g_reset_data *data = platform_get_drvdata(pdev);

	em05g_reset_power_off(data);
}

static struct of_device_id em05g_reset_dt_ids[] = {
	{ .compatible = "em05g-reset" },
	{ }
};

static struct platform_driver em05g_reset_driver = {
	.probe = em05g_reset_probe,
	.remove = em05g_reset_remove,
	.shutdown = em05g_reset_shutdown,
	.driver = {
		.name = "em05g-reset",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(em05g_reset_dt_ids),
	},
};

static int __init em05g_reset_init(void)
{
	return platform_driver_register(&em05g_reset_driver);
}
arch_initcall(em05g_reset_init);

static void __exit em05g_reset_exit(void)
{
	platform_driver_unregister(&em05g_reset_driver);
}
module_exit(em05g_reset_exit);

MODULE_AUTHOR("Atmark Techno, Inc.");
MODULE_DESCRIPTION("Quectel EM05-G reset driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:em05g-reset");
MODULE_DEVICE_TABLE(of, em05g_reset_dt_ids);
