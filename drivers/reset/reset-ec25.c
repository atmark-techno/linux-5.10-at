/*
 * GPIO-based Reset Driver for Quectel EC25
 *
 * Copyright (c) 2022 Atmark Techno, Inc. All Rights Reserved.
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

#define EC25_VBAT_STABLE_TIME_MS		(30)
#define EC25_PWRKEY_TURN_ON_ASSERT_TIME_MS	(500)
#define EC25_PWRKEY_TURN_OFF_ASSERT_TIME_MS	(650)
#define EC25_RESET_ASSERT_TIME_MS		(460)
#define EC25_STATUS_RUNNING_WAIT_TIME_MIN_MS	(2500)
#define EC25_STATUS_OFF_WAIT_TIME_MIN_MS	(29500)

/* These value were decided empirically */
#define EC25_STATUS_RUNNING_TIMEOUT_MS		(10000) /*4 times the minimum*/
#define EC25_STATUS_OFF_TIMEOUT_MS		(177000) /*6 times the minimum*/
#define EC25_STATUS_POLL_MS			(100)

enum ec25_reset_pwrkey_ops {
	EC25_PWRKEY_TURN_ON,
	EC25_PWRKEY_TURN_OFF,
};

enum ec25_reset_vbus_ops {
	EC25_VBUS_TURN_ON,
	EC25_VBUS_TURN_OFF,
};

enum ec25_reset_status {
	EC25_STATUS_RUNNING = 0,
	EC25_STATUS_OFF,
};

enum ec25_reset_action {
	EC25_ACTION_RESET,
	EC25_ACTION_POWER_ON,
	EC25_ACTION_POWER_OFF,
};

static const char *const ec25_reset_action_str[] = {
	[EC25_ACTION_RESET]	= "reset",
	[EC25_ACTION_POWER_ON]	= "power on",
	[EC25_ACTION_POWER_OFF]	= "power off",
};

struct ec25_reset_data {
	struct reset_controller_dev rcdev;
	struct device *dev;

	struct regulator *vbat;

	struct gpio_desc *pwrkey;
	struct gpio_desc *reset;
	struct gpio_desc *status;
	struct gpio_desc *vbus;
	bool vbus_active_low;

	struct mutex power_lock;
	struct mutex reset_lock;

	enum ec25_reset_action action;
	struct work_struct work;
};

#define to_ec25_reset_data(_rcdev)				\
	container_of(_rcdev, struct ec25_reset_data, rcdev)

/*
 * Use RESET_N only when failed to turn off the module by PWRKEY pin.
 */
static int ec25_reset_emergency_reset(struct ec25_reset_data *data)
{
	int ret = 0;

	dev_warn(data->dev, "%s (%d): Emergency reset\n", current->comm, task_pid_nr(current));

	mutex_lock(&data->reset_lock);

	if (work_busy(&data->work)) {
		dev_warn(data->dev, "%s in progress. cannot emergency reset.\n",
			 ec25_reset_action_str[data->action]);
		ret = -EBUSY;
		goto out;
	}

	gpiod_set_value_cansleep(data->reset, 1);
	mdelay(EC25_RESET_ASSERT_TIME_MS);
	gpiod_set_value_cansleep(data->reset, 0);

out:
	mutex_unlock(&data->reset_lock);

	return ret;
}

static void ec25_reset_pwrkey(struct ec25_reset_data *data,
			      enum ec25_reset_pwrkey_ops ops)
{
	unsigned long delay;

	dev_dbg(data->dev, "PWRKEY: %s\n",
		ops == EC25_PWRKEY_TURN_ON ? "on" : "off");

	if (ops == EC25_PWRKEY_TURN_ON)
		delay = EC25_PWRKEY_TURN_ON_ASSERT_TIME_MS;
	else
		delay = EC25_PWRKEY_TURN_OFF_ASSERT_TIME_MS;

	gpiod_set_value_cansleep(data->pwrkey, 1);
	mdelay(delay);
	gpiod_set_value_cansleep(data->pwrkey, 0);
}

static void ec25_reset_vbus(struct ec25_reset_data *data,
			    enum ec25_reset_vbus_ops ops)
{
	if (!data->vbus)
		return;

	dev_dbg(data->dev, "VBUS: %s\n",
		ops == EC25_VBUS_TURN_ON ? "on" : "off");

	if (ops == EC25_VBUS_TURN_ON)
		gpiod_set_value_cansleep(data->vbus, !data->vbus_active_low);
	else
		gpiod_set_value_cansleep(data->vbus, data->vbus_active_low);
}

static enum ec25_reset_status ec25_reset_status(struct ec25_reset_data *data)
{
	if (regulator_is_enabled(data->vbat))
		return gpiod_get_value_cansleep(data->status);

	return EC25_STATUS_OFF;
}

static void ec25_reset_power_on(struct ec25_reset_data *data)
{
	unsigned long timeout;
	unsigned int tries;
	int ret = 0;

	dev_dbg(data->dev, "Power up start.\n");

	mutex_lock(&data->power_lock);

	if (ec25_reset_status(data) == EC25_STATUS_RUNNING) {
		dev_dbg(data->dev, "Power is already up\n");
		goto out;
	}

	ret = regulator_enable(data->vbat);
	if (ret) {
		dev_err(data->dev, "failed to enable vbat regulator\n");
		goto out;
	}
	mdelay(EC25_VBAT_STABLE_TIME_MS);

	timeout = jiffies +
		msecs_to_jiffies(EC25_STATUS_RUNNING_WAIT_TIME_MIN_MS);

	ec25_reset_vbus(data, EC25_VBUS_TURN_ON);
	ec25_reset_pwrkey(data, EC25_PWRKEY_TURN_ON);

	/* We need to wait 2.5s after VBAT stabilizes. */
	while (!time_after(jiffies, timeout))
		msleep(100);
	/* But max time is not defined. */
	tries = EC25_STATUS_RUNNING_TIMEOUT_MS / EC25_STATUS_POLL_MS;
	dev_dbg(data->dev, "Wait for the module state to turn on.\n");
	while (tries--) {
		if (ec25_reset_status(data) == EC25_STATUS_RUNNING) {
			dev_dbg(data->dev, "Power up completed.\n");
			goto out;
		}
		msleep(EC25_STATUS_POLL_MS);
	}

	dev_err(data->dev, "ec25 state does not turn on\n");
	regulator_disable(data->vbat);

out:
	mutex_unlock(&data->power_lock);
}

static void ec25_reset_power_off(struct ec25_reset_data *data)
{
	unsigned int tries;
	int ret;

	dev_dbg(data->dev, "Power down start. This may take a while...\n");

	mutex_lock(&data->power_lock);

	if (ec25_reset_status(data) == EC25_STATUS_OFF) {
		dev_dbg(data->dev, "Power is already down\n");
		goto out;
	}

	ec25_reset_pwrkey(data, EC25_PWRKEY_TURN_OFF);
	ec25_reset_vbus(data, EC25_VBUS_TURN_OFF);

	/* We need to wait for power off at least 29.5s. */
	msleep(EC25_STATUS_OFF_WAIT_TIME_MIN_MS);
	/* But max time is not defined. */
	tries = EC25_STATUS_OFF_TIMEOUT_MS / EC25_STATUS_POLL_MS;
	dev_dbg(data->dev, "Wait for the module state to turn off.\n");
	while (tries--) {
		if (ec25_reset_status(data) == EC25_STATUS_OFF)
			break;
		msleep(EC25_STATUS_POLL_MS);
	}
	if (!tries)
		dev_err(data->dev, "ec25 state does not turn off. did not shut down cleanly\n");

	ret = regulator_disable(data->vbat);
	if (ret)
		dev_err(data->dev, "failed to disable vbat regulator\n");

	if (!ret)
		dev_dbg(data->dev, "Power down completed.\n");

out:
	mutex_unlock(&data->power_lock);
}

static void ec25_reset_work_func(struct work_struct *ws)
{
	struct ec25_reset_data *data =
		container_of(ws, struct ec25_reset_data, work);

	switch (data->action) {
	case EC25_ACTION_RESET:
		ec25_reset_power_off(data);
		ec25_reset_power_on(data);
		break;
	case EC25_ACTION_POWER_ON:
		ec25_reset_power_on(data);
		break;
	case EC25_ACTION_POWER_OFF:
		ec25_reset_power_off(data);
		break;
	default:
		BUG();
	}
}

static int ec25_reset_schedule_work(struct ec25_reset_data *data,
				    enum ec25_reset_action action)
{
	int ret = 0;

	mutex_lock(&data->reset_lock);

	if (!work_busy(&data->work)) {
		data->action = action;
		schedule_work(&data->work);
	} else {
		dev_warn(data->dev, "%s in progress. cannot %s.\n",
			 ec25_reset_action_str[data->action],
			 ec25_reset_action_str[action]);
		ret = -EBUSY;
	}

	mutex_unlock(&data->reset_lock);

	return ret;
}

static int ec25_reset_init_pdata(struct platform_device *pdev,
				 struct ec25_reset_data *data)
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

	data->reset = devm_gpiod_get(&pdev->dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(data->reset))
		return dev_err_probe(&pdev->dev, PTR_ERR(data->reset),
				     "invalid reset gpio");

	data->status = devm_gpiod_get(&pdev->dev, "status", GPIOD_IN);
	if (IS_ERR(data->status))
		return dev_err_probe(&pdev->dev, PTR_ERR(data->status),
				     "invalid status gpio");

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

static int ec25_reset_safe_reset(struct reset_controller_dev *rcdev,
				 unsigned long id)
{
	struct ec25_reset_data *data = to_ec25_reset_data(rcdev);

	return ec25_reset_schedule_work(data, EC25_ACTION_RESET);
}

static struct reset_control_ops ec25_reset_ops = {
	.reset = ec25_reset_safe_reset,
};

static ssize_t ec25_reset_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct ec25_reset_data *data = platform_get_drvdata(pdev);
	int val, ret;

	ret = kstrtoint(buf, 0, &val);
	if (ret)
		return ret;

	if (val == 1)
		ret = ec25_reset_schedule_work(data, EC25_ACTION_RESET);
	else if (val == -1)
		ret = ec25_reset_emergency_reset(data);
	else
		return -EINVAL;

	return ret ? : count;
}
static DEVICE_ATTR(reset, S_IWUSR, NULL, ec25_reset_store);

static ssize_t ec25_power_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct ec25_reset_data *data = platform_get_drvdata(pdev);
	int val, ret;

	ret = kstrtoint(buf, 0, &val);
	if (ret)
		return ret;

	if (val)
		ret = ec25_reset_schedule_work(data, EC25_ACTION_POWER_ON);
	else
		ret = ec25_reset_schedule_work(data, EC25_ACTION_POWER_OFF);

	return ret ? : count;
}
static DEVICE_ATTR(power, S_IWUSR, NULL, ec25_power_store);

static ssize_t ec25_status_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct ec25_reset_data *data = platform_get_drvdata(pdev);
	char *s = ec25_reset_status(data) == EC25_STATUS_RUNNING ?
		"running" : "off";

	return sysfs_emit(buf, "%s\n", s);
}
static DEVICE_ATTR(status, S_IRUSR, ec25_status_show, NULL);

static struct attribute *ec25_reset_attrs[] = {
	&dev_attr_reset.attr,
	&dev_attr_power.attr,
	&dev_attr_status.attr,
	NULL
};

static struct attribute_group ec25_reset_attr_group = {
	.name = "reset",
	.attrs = ec25_reset_attrs,
};

static int of_ec25_reset_xlate(struct reset_controller_dev *rcdev,
			       const struct of_phandle_args *reset_spec)
{
	if (WARN_ON(reset_spec->args_count != 0))
		return -EINVAL;

	return 0;
}

static int ec25_reset_probe(struct platform_device *pdev)
{
	struct ec25_reset_data *data;
	int ret;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	ret = ec25_reset_init_pdata(pdev, data);
	if (ret)
		return ret;

	mutex_init(&data->power_lock);
	mutex_init(&data->reset_lock);

	data->dev = &pdev->dev;
	platform_set_drvdata(pdev, data);

	data->rcdev.of_node = pdev->dev.of_node;
	data->rcdev.owner = THIS_MODULE;
	data->rcdev.nr_resets = 1;
	data->rcdev.ops = &ec25_reset_ops;
	data->rcdev.of_xlate = of_ec25_reset_xlate;
	ret = devm_reset_controller_register(&pdev->dev, &data->rcdev);
	if (ret) {
		dev_err(&pdev->dev, "failed to register device\n");
		return ret;
	}

	ret = sysfs_create_group(&pdev->dev.kobj, &ec25_reset_attr_group);
	if (ret) {
		dev_err(&pdev->dev, "failed to create sysfs attrs\n");
		return ret;
	}

	INIT_WORK(&data->work, ec25_reset_work_func);
	ec25_reset_schedule_work(data, EC25_ACTION_POWER_ON);

	return 0;
}

static int ec25_reset_remove(struct platform_device *pdev)
{
	struct ec25_reset_data *data = platform_get_drvdata(pdev);

	sysfs_remove_group(&pdev->dev.kobj, &ec25_reset_attr_group);
	cancel_work_sync(&data->work);
	ec25_reset_power_off(data);

	return 0;
}

static void ec25_reset_shutdown(struct platform_device *pdev)
{
	struct ec25_reset_data *data = platform_get_drvdata(pdev);

	cancel_work_sync(&data->work);
	ec25_reset_power_off(data);
}

static struct of_device_id ec25_reset_dt_ids[] = {
	{ .compatible = "ec25-reset" },
	{ }
};

static struct platform_driver ec25_reset_driver = {
	.probe = ec25_reset_probe,
	.remove = ec25_reset_remove,
	.shutdown = ec25_reset_shutdown,
	.driver = {
		.name = "ec25-reset",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(ec25_reset_dt_ids),
	},
};

static int __init ec25_reset_init(void)
{
	return platform_driver_register(&ec25_reset_driver);
}
module_init(ec25_reset_init);

static void __exit ec25_reset_exit(void)
{
	platform_driver_unregister(&ec25_reset_driver);
}
module_exit(ec25_reset_exit);

MODULE_AUTHOR("Atmark Techno, Inc.");
MODULE_DESCRIPTION("Quectel EC25 reset driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:ec25-reset");
MODULE_DEVICE_TABLE(of, ec25_reset_dt_ids);
