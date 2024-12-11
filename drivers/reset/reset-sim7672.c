// SPDX-License-Identifier: GPL-2.0
/*
 * GPIO-based Reset Driver for SIMCom SIM7672
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

#define SIM7672_PWRKEY_TURN_ON_ASSERT_TIME_MS	(50) /* Ton */
#define SIM7672_PWRKEY_TURN_OFF_ASSERT_TIME_MS	(2500) /* Toff */
#define SIM7672_RESET_OFF_ON_INTERVAL_MS	(2003) /* Toff(uart)+Toff-on */
#define SIM7672_STATUS_RUNNING_WAIT_TIME_MIN_MS	(320) /* Ton(status) */
#define SIM7672_STATUS_OFF_WAIT_TIME_MIN_US	(480)

/* These value were decided empirically */
#define SIM7672_VBAT_STABLE_TIME_MS		(30)
#define SIM7672_STATUS_RUNNING_TIMEOUT_MS	(2000)
#define SIM7672_STATUS_OFF_TIMEOUT_MS		(2000)
#define SIM7672_STATUS_POLL_MS			(10)

enum sim7672_reset_pwrkey_ops {
	SIM7672_PWRKEY_TURN_ON,
	SIM7672_PWRKEY_TURN_OFF,
};

enum sim7672_reset_vbus_ops {
	SIM7672_VBUS_TURN_ON,
	SIM7672_VBUS_TURN_OFF,
};

enum sim7672_reset_dtr_ops {
	SIM7672_DTR_LOW,
	SIM7672_DTR_HIGH,
};

enum sim7672_reset_status {
	SIM7672_STATUS_OFF = 0,
	SIM7672_STATUS_RUNNING,
};

enum sim7672_reset_action {
	SIM7672_ACTION_RESET,
	SIM7672_ACTION_POWER_ON,
	SIM7672_ACTION_POWER_OFF,
	SIM7672_ACTION_VBUS_ON,
	SIM7672_ACTION_VBUS_OFF,
	SIM7672_ACTION_DTR_HIGH,
	SIM7672_ACTION_DTR_LOW,
};

static const char *const sim7672_reset_action_str[] = {
	[SIM7672_ACTION_RESET]			= "reset",
	[SIM7672_ACTION_POWER_ON]		= "power on",
	[SIM7672_ACTION_POWER_OFF]		= "power off",
	[SIM7672_ACTION_VBUS_ON]		= "vbus on",
	[SIM7672_ACTION_VBUS_OFF]		= "vbus off",
	[SIM7672_ACTION_DTR_HIGH]		= "dtr high",
	[SIM7672_ACTION_DTR_LOW]		= "dtr low",
};

struct sim7672_reset_data {
	struct reset_controller_dev rcdev;
	struct device *dev;

	struct regulator *vbat;

	struct gpio_desc *pwrkey;
	struct gpio_desc *status;
	struct gpio_desc *vbus;
	struct gpio_desc *usb_boot;
	struct gpio_desc *dtr;
	bool vbus_active_low;
	bool dtr_active_low;

	struct mutex power_lock;
	struct mutex reset_lock;

	enum sim7672_reset_action action;
	struct work_struct work;
};

#define to_sim7672_reset_data(_rcdev)				\
	container_of(_rcdev, struct sim7672_reset_data, rcdev)

static void sim7672_reset_pwrkey(struct sim7672_reset_data *data,
				 enum sim7672_reset_pwrkey_ops ops)
{
	unsigned long delay;

	dev_dbg(data->dev, "PWRKEY: %s\n",
		ops == SIM7672_PWRKEY_TURN_ON ? "on" : "off");

	if (ops == SIM7672_PWRKEY_TURN_ON)
		delay = SIM7672_PWRKEY_TURN_ON_ASSERT_TIME_MS;
	else
		delay = SIM7672_PWRKEY_TURN_OFF_ASSERT_TIME_MS;

	gpiod_set_value_cansleep(data->pwrkey, 1);
	mdelay(delay);
	gpiod_set_value_cansleep(data->pwrkey, 0);
}

static void sim7672_reset_vbus(struct sim7672_reset_data *data,
			       enum sim7672_reset_vbus_ops ops)
{
	if (!data->vbus)
		return;

	dev_dbg(data->dev, "VBUS: %s\n",
		ops == SIM7672_VBUS_TURN_ON ? "on" : "off");

	if (ops == SIM7672_VBUS_TURN_ON)
		gpiod_set_value_cansleep(data->vbus, !data->vbus_active_low);
	else
		gpiod_set_value_cansleep(data->vbus, data->vbus_active_low);
}

static enum sim7672_reset_status sim7672_reset_status(struct sim7672_reset_data *data)
{
	if (regulator_is_enabled(data->vbat))
		return gpiod_get_value_cansleep(data->status);

	return SIM7672_STATUS_OFF;
}

static void sim7672_reset_set_dtr(struct sim7672_reset_data *data,
				  enum sim7672_reset_dtr_ops ops)
{
	if (!data->dtr)
		return;

	dev_dbg(data->dev, "DTR: %s\n",
		ops == SIM7672_DTR_HIGH ? "high" : "low");

	if (ops == SIM7672_DTR_HIGH)
		gpiod_set_value_cansleep(data->dtr, !data->dtr_active_low);
	else
		gpiod_set_value_cansleep(data->dtr, data->dtr_active_low);
}

static void sim7672_reset_power_on(struct sim7672_reset_data *data)
{
	unsigned long timeout;
	unsigned int tries;
	int ret = 0;

	dev_dbg(data->dev, "Power up start.\n");

	mutex_lock(&data->power_lock);

	if (sim7672_reset_status(data) == SIM7672_STATUS_RUNNING) {
		dev_dbg(data->dev, "Power is already up\n");
		goto out;
	}

	sim7672_reset_vbus(data, SIM7672_VBUS_TURN_ON);
	ret = regulator_enable(data->vbat);
	if (ret) {
		dev_err(data->dev, "failed to enable vbat regulator\n");
		goto out;
	}
	mdelay(SIM7672_VBAT_STABLE_TIME_MS);

	timeout = jiffies +
		msecs_to_jiffies(SIM7672_STATUS_RUNNING_WAIT_TIME_MIN_MS);

	sim7672_reset_pwrkey(data, SIM7672_PWRKEY_TURN_ON);

	/* We need to wait 320msec after VBAT stabilizes. */
	while (!time_after(jiffies, timeout))
		msleep(80);

	/* if USB_BOOT is set low, status pin will not be running. */
	if (gpiod_get_value_cansleep(data->usb_boot) == 0) {
		dev_info(data->dev, "firmware update mode.\n");
		goto out;
	}

	/* But max time is not defined. */
	tries = SIM7672_STATUS_RUNNING_TIMEOUT_MS / SIM7672_STATUS_POLL_MS;
	dev_dbg(data->dev, "Wait for the module state to turn on.\n");
	while (tries--) {
		if (sim7672_reset_status(data) == SIM7672_STATUS_RUNNING) {
			dev_dbg(data->dev, "Power up completed.\n");
			goto out;
		}
		msleep(SIM7672_STATUS_POLL_MS);
	}

	dev_err(data->dev, "sim7672 state does not turn on\n");
	regulator_disable(data->vbat);

out:
	mutex_unlock(&data->power_lock);
}

static void sim7672_reset_power_off(struct sim7672_reset_data *data)
{
	unsigned int tries;
	int ret;

	dev_dbg(data->dev, "Power down start. This may take a while...\n");

	mutex_lock(&data->power_lock);

	if (sim7672_reset_status(data) == SIM7672_STATUS_OFF) {
		if (regulator_is_enabled(data->vbat))
			regulator_disable(data->vbat);
		dev_dbg(data->dev, "Power is already down\n");
		goto out;
	}

	sim7672_reset_vbus(data, SIM7672_VBUS_TURN_OFF);
	sim7672_reset_pwrkey(data, SIM7672_PWRKEY_TURN_OFF);

	/* We need to wait for power off. */
	usleep_range(SIM7672_STATUS_OFF_WAIT_TIME_MIN_US,
		     SIM7672_STATUS_OFF_WAIT_TIME_MIN_US * 2);
	/* But max time is not defined. */
	tries = SIM7672_STATUS_OFF_TIMEOUT_MS / SIM7672_STATUS_POLL_MS;
	dev_dbg(data->dev, "Wait for the module state to turn off.\n");
	while (tries--) {
		if (sim7672_reset_status(data) == SIM7672_STATUS_OFF)
			break;
		msleep(SIM7672_STATUS_POLL_MS);
	}
	if (!tries)
		dev_err(data->dev, "sim7672 state does not turn off. did not shut down cleanly\n");

	ret = regulator_disable(data->vbat);
	if (ret)
		dev_err(data->dev, "failed to disable vbat regulator\n");

	if (!ret)
		dev_dbg(data->dev, "Power down completed.\n");

out:
	mutex_unlock(&data->power_lock);
}

static void sim7672_reset_work_func(struct work_struct *ws)
{
	struct sim7672_reset_data *data =
		container_of(ws, struct sim7672_reset_data, work);

	switch (data->action) {
	case SIM7672_ACTION_RESET:
		sim7672_reset_power_off(data);
		sim7672_reset_power_on(data);
		break;
	case SIM7672_ACTION_POWER_ON:
		sim7672_reset_power_on(data);
		break;
	case SIM7672_ACTION_POWER_OFF:
		sim7672_reset_power_off(data);
		break;
	case SIM7672_ACTION_VBUS_ON:
		sim7672_reset_vbus(data, SIM7672_VBUS_TURN_ON);
		break;
	case SIM7672_ACTION_VBUS_OFF:
		sim7672_reset_vbus(data, SIM7672_VBUS_TURN_OFF);
		break;
	case SIM7672_ACTION_DTR_HIGH:
		sim7672_reset_set_dtr(data, SIM7672_DTR_HIGH);
		break;
	case SIM7672_ACTION_DTR_LOW:
		sim7672_reset_set_dtr(data, SIM7672_DTR_LOW);
		break;
	default:
		WARN_ON(1);
	}
}

static int sim7672_reset_schedule_work(struct sim7672_reset_data *data,
				       enum sim7672_reset_action action)
{
	int ret = 0;

	mutex_lock(&data->reset_lock);

	if (!work_busy(&data->work)) {
		data->action = action;
		schedule_work(&data->work);
	} else {
		dev_warn(data->dev, "%s in progress. cannot %s.\n",
			 sim7672_reset_action_str[data->action],
			 sim7672_reset_action_str[action]);
		ret = -EBUSY;
	}

	mutex_unlock(&data->reset_lock);

	return ret;
}

static int sim7672_reset_init_pdata(struct platform_device *pdev,
				    struct sim7672_reset_data *data)
{
	struct device_node *np = pdev->dev.of_node;
	enum gpiod_flags flags;

	data->vbat = devm_regulator_get(&pdev->dev, "vbat");
	if (IS_ERR(data->vbat))
		return dev_err_probe(&pdev->dev, PTR_ERR(data->vbat),
				     "vbat property missing\n");

	data->pwrkey = devm_gpiod_get(&pdev->dev, "pwrkey", GPIOD_OUT_LOW);
	if (IS_ERR(data->pwrkey))
		return dev_err_probe(&pdev->dev, PTR_ERR(data->pwrkey),
				     "invalid pwrkey gpio");

	data->status = devm_gpiod_get(&pdev->dev, "status", GPIOD_IN);
	if (IS_ERR(data->status))
		return dev_err_probe(&pdev->dev, PTR_ERR(data->status),
				     "invalid status gpio");

	data->usb_boot = devm_gpiod_get(&pdev->dev, "usbboot", GPIOD_OUT_HIGH);
	if (IS_ERR(data->usb_boot))
		return dev_err_probe(&pdev->dev, PTR_ERR(data->usb_boot),
				     "invalid usb_boot gpio");

	if (of_property_read_bool(np, "dtr-active-low"))
		data->dtr_active_low = true;

	if (data->dtr_active_low)
		flags = GPIOD_OUT_HIGH;
	else
		flags = GPIOD_OUT_LOW;
	data->dtr = devm_gpiod_get_optional(&pdev->dev, "dtr", flags);
	if (IS_ERR(data->dtr))
		return dev_err_probe(&pdev->dev, PTR_ERR(data->dtr),
				     "invalid dtr gpio");

	if (of_property_read_bool(np, "vbus-active-low"))
		data->vbus_active_low = true;

	if (data->vbus_active_low)
		flags = GPIOD_OUT_HIGH;
	else
		flags = GPIOD_OUT_LOW;
	data->vbus = devm_gpiod_get_optional(&pdev->dev, "vbus", flags);
	if (IS_ERR(data->vbus))
		return dev_err_probe(&pdev->dev, PTR_ERR(data->vbus),
				     "invalid vbus gpio");

	return 0;
}

static int sim7672_reset_safe_reset(struct reset_controller_dev *rcdev,
				    unsigned long id)
{
	struct sim7672_reset_data *data = to_sim7672_reset_data(rcdev);

	return sim7672_reset_schedule_work(data, SIM7672_ACTION_RESET);
}

static struct reset_control_ops sim7672_reset_ops = {
	.reset = sim7672_reset_safe_reset,
};

static ssize_t reset_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct sim7672_reset_data *data = platform_get_drvdata(pdev);
	int ret;

	ret = sim7672_reset_schedule_work(data, SIM7672_ACTION_RESET);

	return ret ? : count;
}
static DEVICE_ATTR_WO(reset);

static ssize_t power_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct sim7672_reset_data *data = platform_get_drvdata(pdev);
	int val, ret;

	ret = kstrtoint(buf, 0, &val);
	if (ret)
		return ret;

	if (val)
		ret = sim7672_reset_schedule_work(data, SIM7672_ACTION_POWER_ON);
	else
		ret = sim7672_reset_schedule_work(data, SIM7672_ACTION_POWER_OFF);

	return ret ? : count;
}
static DEVICE_ATTR_WO(power);

static ssize_t status_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct sim7672_reset_data *data = platform_get_drvdata(pdev);
	char *s = sim7672_reset_status(data) == SIM7672_STATUS_RUNNING ?
		"running" : "off";

	return sysfs_emit(buf, "%s\n", s);
}
static DEVICE_ATTR_ADMIN_RO(status);

static ssize_t vbus_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct sim7672_reset_data *data = platform_get_drvdata(pdev);
	int val, ret;

	ret = kstrtoint(buf, 0, &val);
	if (ret)
		return ret;

	if (val)
		ret = sim7672_reset_schedule_work(data, SIM7672_ACTION_VBUS_ON);
	else
		ret = sim7672_reset_schedule_work(data, SIM7672_ACTION_VBUS_OFF);

	return ret ? : count;
}
static DEVICE_ATTR_WO(vbus);

static ssize_t dtr_store(struct device *dev,
			 struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct sim7672_reset_data *data = platform_get_drvdata(pdev);
	int val, ret;

	ret = kstrtoint(buf, 0, &val);
	if (ret)
		return ret;

	if (val)
		ret = sim7672_reset_schedule_work(data, SIM7672_ACTION_DTR_HIGH);
	else
		ret = sim7672_reset_schedule_work(data, SIM7672_ACTION_DTR_LOW);

	return ret ? : count;
}
static DEVICE_ATTR_WO(dtr);

static struct attribute *sim7672_reset_attrs[] = {
	&dev_attr_reset.attr,
	&dev_attr_power.attr,
	&dev_attr_status.attr,
	&dev_attr_vbus.attr,
	&dev_attr_dtr.attr,
	NULL
};

static struct attribute_group sim7672_reset_attr_group = {
	.name = "reset",
	.attrs = sim7672_reset_attrs,
};

static int of_sim7672_reset_xlate(struct reset_controller_dev *rcdev,
				  const struct of_phandle_args *reset_spec)
{
	if (WARN_ON(reset_spec->args_count != 0))
		return -EINVAL;

	return 0;
}

static int sim7672_reset_probe(struct platform_device *pdev)
{
	struct sim7672_reset_data *data;
	int ret;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	ret = sim7672_reset_init_pdata(pdev, data);
	if (ret)
		return ret;

	mutex_init(&data->power_lock);
	mutex_init(&data->reset_lock);

	data->dev = &pdev->dev;
	platform_set_drvdata(pdev, data);

	/* export USB BOOT */
	gpio_export(desc_to_gpio(data->usb_boot), false);
	gpio_export_link(&pdev->dev, "lte_usb_boot", desc_to_gpio(data->usb_boot));

	data->rcdev.of_node = pdev->dev.of_node;
	data->rcdev.owner = THIS_MODULE;
	data->rcdev.nr_resets = 1;
	data->rcdev.ops = &sim7672_reset_ops;
	data->rcdev.of_xlate = of_sim7672_reset_xlate;
	ret = devm_reset_controller_register(&pdev->dev, &data->rcdev);
	if (ret) {
		dev_err(&pdev->dev, "failed to register device\n");
		return ret;
	}

	ret = sysfs_create_group(&pdev->dev.kobj, &sim7672_reset_attr_group);
	if (ret) {
		dev_err(&pdev->dev, "failed to create sysfs attrs\n");
		return ret;
	}

	INIT_WORK(&data->work, sim7672_reset_work_func);
	sim7672_reset_schedule_work(data, SIM7672_ACTION_POWER_ON);

	return 0;
}

static int sim7672_reset_remove(struct platform_device *pdev)
{
	struct sim7672_reset_data *data = platform_get_drvdata(pdev);

	sysfs_remove_group(&pdev->dev.kobj, &sim7672_reset_attr_group);
	cancel_work_sync(&data->work);
	sim7672_reset_power_off(data);

	return 0;
}

static void sim7672_reset_shutdown(struct platform_device *pdev)
{
	struct sim7672_reset_data *data = platform_get_drvdata(pdev);

	cancel_work_sync(&data->work);
	sim7672_reset_power_off(data);
}

static const struct of_device_id sim7672_reset_dt_ids[] = {
	{ .compatible = "sim7672-reset" },
	{ }
};

static struct platform_driver sim7672_reset_driver = {
	.probe = sim7672_reset_probe,
	.remove = sim7672_reset_remove,
	.shutdown = sim7672_reset_shutdown,
	.driver = {
		.name = "sim7672-reset",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(sim7672_reset_dt_ids),
	},
};

static int __init sim7672_reset_init(void)
{
	return platform_driver_register(&sim7672_reset_driver);
}
module_init(sim7672_reset_init);

static void __exit sim7672_reset_exit(void)
{
	platform_driver_unregister(&sim7672_reset_driver);
}
module_exit(sim7672_reset_exit);

MODULE_AUTHOR("Atmark Techno, Inc.");
MODULE_DESCRIPTION("Quectel SIM7672 reset driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:sim7672-reset");
MODULE_DEVICE_TABLE(of, sim7672_reset_dt_ids);
