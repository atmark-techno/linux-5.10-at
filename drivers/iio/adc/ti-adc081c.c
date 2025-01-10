// SPDX-License-Identifier: GPL-2.0-only
/*
 * TI ADC081C/ADC101C/ADC121C 8/10/12-bit ADC driver
 *
 * Copyright (C) 2012 Avionic Design GmbH
 * Copyright (C) 2016 Intel
 *
 * Datasheets:
 *	https://www.ti.com/lit/ds/symlink/adc081c021.pdf
 *	https://www.ti.com/lit/ds/symlink/adc101c021.pdf
 *	https://www.ti.com/lit/ds/symlink/adc121c021.pdf
 *
 * The devices have a very similar interface and differ mostly in the number of
 * bits handled. For the 8-bit and 10-bit models the least-significant 4 or 2
 * bits of value registers are reserved.
 */

#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/property.h>

#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/events.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/regulator/consumer.h>

struct adc081c {
	struct i2c_client *i2c;
	struct regulator *ref;

	/* 8, 10 or 12 */
	int bits;

	/* Ensure natural alignment of buffer elements */
	struct {
		u16 channel;
		s64 ts __aligned(8);
	} scan;

	struct mutex lock;
};

#define REG_CONV_RES	0x00
#define REG_ALERT_STAT	0x01
#define REG_CONF	0x02
#define REG_LO_LIMIT	0x03
#define REG_HI_LIMIT	0x04
/* FIXME: Hysteresis not support now */

#define REG_ALERT_STAT_RANGE_MASK	GENMASK(1, 0)
#define REG_ALERT_STAT_OVER_RANGE	BIT(1)
#define REG_ALERT_STAT_UNDER_RANGE	BIT(0)

#define REG_CONF_CYCLE_TIME_MASK	GENMASK(7, 5)
#define REG_CONF_CYCLE_TIME_SHIFT	(5)
#define REG_CONF_DEFAULT_CYCLE_TIME	(0x03) /* Tconvert * 128, 6.7ksps */
#define REG_CONF_ALERT_HOLD		BIT(4)
#define REG_CONF_ALERT_PIN_ENABLE	BIT(2)
#define REG_CONF_POLARITY		BIT(0)

static int adc081c_read_raw(struct iio_dev *iio,
			    struct iio_chan_spec const *channel, int *value,
			    int *shift, long mask)
{
	struct adc081c *adc = iio_priv(iio);
	int err;

	mutex_lock(&adc->lock);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		err = i2c_smbus_read_word_swapped(adc->i2c, REG_CONV_RES);
		if (err < 0)
			goto out;

		*value = (err & 0xFFF) >> (12 - adc->bits);
		err = IIO_VAL_INT;
		break;

	case IIO_CHAN_INFO_SCALE:
		err = regulator_get_voltage(adc->ref);
		if (err < 0)
			goto out;

		*value = err / 1000;
		*shift = adc->bits;

		err = IIO_VAL_FRACTIONAL_LOG2;
		break;

	default:
		err = -EINVAL;
		break;
	}

out:
	mutex_unlock(&adc->lock);

	return err;
}

static int adc081c_read_event(struct iio_dev *iio,
			      const struct iio_chan_spec *channel,
			      enum iio_event_type type,
			      enum iio_event_direction dir,
			      enum iio_event_info info, int *value, int *value2)
{
	struct adc081c *adc = iio_priv(iio);
	int err;

	mutex_lock(&adc->lock);

	switch (info) {
	case IIO_EV_INFO_VALUE:
		err = i2c_smbus_read_word_swapped(adc->i2c,
						  (dir == IIO_EV_DIR_RISING) ?
						  REG_HI_LIMIT : REG_LO_LIMIT);
		if (err < 0)
			break;

		*value = (err & 0xFFF) >> (12 - adc->bits);
		err = IIO_VAL_INT;
		break;
	default:
		err = -EINVAL;
		break;
	}

	mutex_unlock(&adc->lock);

	return err;
}

static int adc081c_write_event(struct iio_dev *iio,
			       const struct iio_chan_spec *channel,
			       enum iio_event_type type,
			       enum iio_event_direction dir,
			       enum iio_event_info info, int value, int value2)
{
	struct adc081c *adc = iio_priv(iio);
	u16 limit;
	int err;

	mutex_lock(&adc->lock);

	switch (info) {
	case IIO_EV_INFO_VALUE:
		limit = (value & 0xFFF) << (12 - adc->bits);
		err = i2c_smbus_write_word_swapped(adc->i2c, (dir == IIO_EV_DIR_RISING) ?
						   REG_HI_LIMIT : REG_LO_LIMIT, limit);
		break;
	default:
		err = -EINVAL;
	}

	mutex_unlock(&adc->lock);

	return err;
}

static int adc081c_read_event_config(struct iio_dev *iio,
				     const struct iio_chan_spec *channel,
				     enum iio_event_type type,
				     enum iio_event_direction dir)
{
	struct adc081c *adc = iio_priv(iio);
	int err;

	mutex_lock(&adc->lock);

	err = i2c_smbus_read_byte_data(adc->i2c, REG_CONF);
	if (err < 0)
		goto out;

	err = !!(err & REG_CONF_ALERT_PIN_ENABLE);

out:
	mutex_unlock(&adc->lock);

	return err;
}

static int adc081c_write_event_config(struct iio_dev *iio,
				      const struct iio_chan_spec *channel,
				      enum iio_event_type type,
				      enum iio_event_direction dir, int state)
{
	struct adc081c *adc = iio_priv(iio);
	int err;

	mutex_lock(&adc->lock);

	/* Clear ALERT status */
	err = i2c_smbus_write_byte_data(adc->i2c, REG_ALERT_STAT,
					REG_ALERT_STAT_OVER_RANGE |
					REG_ALERT_STAT_UNDER_RANGE);
	if (err < 0)
		goto err_mutex_unlock;

	/* Prevent from enabling both buffer and event at a time */
	err = iio_device_claim_direct_mode(iio);
	if (err < 0)
		goto err_mutex_unlock;

	err = i2c_smbus_read_byte_data(adc->i2c, REG_CONF);
	if (err < 0)
		goto err_release_direct_mode;
	if (state)
		err |= REG_CONF_ALERT_PIN_ENABLE;
	else
		err &= ~REG_CONF_ALERT_PIN_ENABLE;
	err = i2c_smbus_write_byte_data(adc->i2c, REG_CONF, err);

err_release_direct_mode:
	iio_device_release_direct_mode(iio);
err_mutex_unlock:
	mutex_unlock(&adc->lock);

	return err;
}


static irqreturn_t adc081c_event_handler(int irq, void *p)
{
	struct iio_dev *indio_dev = p;
	struct adc081c *data = iio_priv(indio_dev);
	enum iio_event_direction dir;
	int ret;

	/* Disables the ALERT output pin */
	ret = i2c_smbus_read_byte_data(data->i2c, REG_CONF);
	if (ret < 0)
		return IRQ_NONE;
	ret &= ~REG_CONF_ALERT_PIN_ENABLE;
	ret = i2c_smbus_write_byte_data(data->i2c, REG_CONF, ret);
	if (ret < 0)
		return IRQ_NONE;

	/* Check ALERT status */
	ret = i2c_smbus_read_byte_data(data->i2c, REG_ALERT_STAT);
	if (ret < 0)
		return IRQ_NONE;
	switch (ret & REG_ALERT_STAT_RANGE_MASK) {
	case REG_ALERT_STAT_OVER_RANGE:
		dir = IIO_EV_DIR_RISING;
		break;
	case REG_ALERT_STAT_UNDER_RANGE:
		dir = IIO_EV_DIR_FALLING;
		break;
	case REG_ALERT_STAT_OVER_RANGE | REG_ALERT_STAT_UNDER_RANGE:
		dir = IIO_EV_DIR_EITHER;
		break;
	default:
		dir = IIO_EV_DIR_NONE;
		break;
	}

	/* Clear ALERT status */
	ret = i2c_smbus_write_byte_data(data->i2c, REG_ALERT_STAT, ret);
	if (ret < 0)
		return IRQ_NONE;

	iio_push_event(indio_dev,
		       IIO_UNMOD_EVENT_CODE(IIO_VOLTAGE, 0,
					    IIO_EV_TYPE_THRESH, dir),
		       iio_get_time_ns(indio_dev));

	return IRQ_HANDLED;
}

static const struct iio_event_spec adc081c_events[] = {
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_RISING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE) |
				 BIT(IIO_EV_INFO_ENABLE),
	}, {
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_FALLING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE) |
				 BIT(IIO_EV_INFO_ENABLE),
	},
};

#define ADCxx1C_CHAN(_bits) {					\
	.type = IIO_VOLTAGE,					\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),	\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
	.scan_type = {						\
		.sign = 'u',					\
		.realbits = (_bits),				\
		.storagebits = 16,				\
		.shift = 12 - (_bits),				\
		.endianness = IIO_CPU,				\
	},							\
	.event_spec = adc081c_events,				\
	.num_event_specs = ARRAY_SIZE(adc081c_events),		\
}

#define DEFINE_ADCxx1C_CHANNELS(_name, _bits)				\
	static const struct iio_chan_spec _name ## _channels[] = {	\
		ADCxx1C_CHAN((_bits)),					\
		IIO_CHAN_SOFT_TIMESTAMP(1),				\
	};								\

#define ADC081C_NUM_CHANNELS 2

struct adcxx1c_model {
	const struct iio_chan_spec* channels;
	int bits;
};

#define ADCxx1C_MODEL(_name, _bits)					\
	{								\
		.channels = _name ## _channels,				\
		.bits = (_bits),					\
	}

DEFINE_ADCxx1C_CHANNELS(adc081c,  8);
DEFINE_ADCxx1C_CHANNELS(adc101c, 10);
DEFINE_ADCxx1C_CHANNELS(adc121c, 12);

/* Model ids are indexes in _models array */
enum adcxx1c_model_id {
	ADC081C = 0,
	ADC101C = 1,
	ADC121C = 2,
};

static struct adcxx1c_model adcxx1c_models[] = {
	ADCxx1C_MODEL(adc081c,  8),
	ADCxx1C_MODEL(adc101c, 10),
	ADCxx1C_MODEL(adc121c, 12),
};

static const struct iio_info adc081c_info = {
	.read_raw		= adc081c_read_raw,
	.read_event_value	= adc081c_read_event,
	.write_event_value	= adc081c_write_event,
	.read_event_config	= adc081c_read_event_config,
	.write_event_config	= adc081c_write_event_config,
};

static irqreturn_t adc081c_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct adc081c *data = iio_priv(indio_dev);
	int ret;

	ret = i2c_smbus_read_word_swapped(data->i2c, REG_CONV_RES);
	if (ret < 0)
		goto out;
	data->scan.channel = ret;
	iio_push_to_buffers_with_timestamp(indio_dev, &data->scan,
					   iio_get_time_ns(indio_dev));
out:
	iio_trigger_notify_done(indio_dev->trig);
	return IRQ_HANDLED;
}

static int adc081c_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct iio_dev *iio;
	struct adc081c *adc;
	const struct adcxx1c_model *model;
	int err;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_WORD_DATA))
		return -EOPNOTSUPP;

	if (dev_fwnode(&client->dev))
		model = device_get_match_data(&client->dev);
	else
		model = &adcxx1c_models[id->driver_data];

	iio = devm_iio_device_alloc(&client->dev, sizeof(*adc));
	if (!iio)
		return -ENOMEM;

	adc = iio_priv(iio);
	adc->i2c = client;
	adc->bits = model->bits;
	mutex_init(&adc->lock);

	adc->ref = devm_regulator_get(&client->dev, "vref");
	if (IS_ERR(adc->ref))
		return PTR_ERR(adc->ref);

	err = regulator_enable(adc->ref);
	if (err < 0)
		return err;

	iio->name = dev_name(&client->dev);
	iio->modes = INDIO_DIRECT_MODE;
	iio->info = &adc081c_info;

	iio->channels = model->channels;
	iio->num_channels = ADC081C_NUM_CHANNELS;

	err = iio_triggered_buffer_setup(iio, NULL, adc081c_trigger_handler, NULL);
	if (err < 0) {
		dev_err(&client->dev, "iio triggered buffer setup failed\n");
		goto err_regulator_disable;
	}

	/*
	 * Set default lower and upper threshold to min and max value
	 * respectively.
	 */
	err = i2c_smbus_write_word_swapped(adc->i2c, REG_LO_LIMIT, 0);
	if (err < 0)
		goto err_buffer_cleanup;
	err = i2c_smbus_write_word_swapped(adc->i2c, REG_HI_LIMIT,
					   GENMASK(12, (12 - adc->bits)));
	if (err < 0)
		goto err_buffer_cleanup;

	if (client->irq) {
		unsigned long irq_trig =
			irqd_get_trigger_type(irq_get_irq_data(client->irq));
		u8 conf = REG_CONF_ALERT_HOLD;
		u8 cycle_time;

		switch (irq_trig) {
		case IRQF_TRIGGER_FALLING:
		case IRQF_TRIGGER_LOW:
			break;
		case IRQF_TRIGGER_RISING:
		case IRQF_TRIGGER_HIGH:
			conf |= REG_CONF_POLARITY;
			break;
		default:
			err = -EINVAL;
			goto err_buffer_cleanup;
		}

		err = of_property_read_u8(client->dev.of_node, "ti,cycle-time", &cycle_time);
		if (err < 0)
			cycle_time = REG_CONF_DEFAULT_CYCLE_TIME;
		conf |= (cycle_time << REG_CONF_CYCLE_TIME_SHIFT) & REG_CONF_CYCLE_TIME_MASK;

		err = i2c_smbus_write_byte_data(adc->i2c, REG_CONF, conf);
		if (err < 0)
			goto err_buffer_cleanup;

		err = devm_request_threaded_irq(&client->dev, client->irq,
						NULL, adc081c_event_handler,
						irq_trig | IRQF_ONESHOT | IRQF_SHARED,
						client->name, iio);
		if (err)
			goto err_buffer_cleanup;
	}

	err = iio_device_register(iio);
	if (err < 0)
		goto err_buffer_cleanup;

	i2c_set_clientdata(client, iio);

	return 0;

err_buffer_cleanup:
	iio_triggered_buffer_cleanup(iio);
err_regulator_disable:
	regulator_disable(adc->ref);

	return err;
}

static int adc081c_remove(struct i2c_client *client)
{
	struct iio_dev *iio = i2c_get_clientdata(client);
	struct adc081c *adc = iio_priv(iio);

	iio_device_unregister(iio);
	iio_triggered_buffer_cleanup(iio);
	regulator_disable(adc->ref);

	return 0;
}

static const struct i2c_device_id adc081c_id[] = {
	{ "adc081c", ADC081C },
	{ "adc101c", ADC101C },
	{ "adc121c", ADC121C },
	{ }
};
MODULE_DEVICE_TABLE(i2c, adc081c_id);

static const struct acpi_device_id adc081c_acpi_match[] = {
	/* Used on some AAEON boards */
	{ "ADC081C", (kernel_ulong_t)&adcxx1c_models[ADC081C] },
	{ }
};
MODULE_DEVICE_TABLE(acpi, adc081c_acpi_match);

static const struct of_device_id adc081c_of_match[] = {
	{ .compatible = "ti,adc081c", .data = &adcxx1c_models[ADC081C] },
	{ .compatible = "ti,adc101c", .data = &adcxx1c_models[ADC101C] },
	{ .compatible = "ti,adc121c", .data = &adcxx1c_models[ADC121C] },
	{ }
};
MODULE_DEVICE_TABLE(of, adc081c_of_match);

static struct i2c_driver adc081c_driver = {
	.driver = {
		.name = "adc081c",
		.of_match_table = adc081c_of_match,
		.acpi_match_table = adc081c_acpi_match,
	},
	.probe = adc081c_probe,
	.remove = adc081c_remove,
	.id_table = adc081c_id,
};
module_i2c_driver(adc081c_driver);

MODULE_AUTHOR("Thierry Reding <thierry.reding@avionic-design.de>");
MODULE_DESCRIPTION("Texas Instruments ADC081C/ADC101C/ADC121C driver");
MODULE_LICENSE("GPL v2");
