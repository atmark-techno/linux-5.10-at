// SPDX-License-Identifier: GPL-2.0-only
/**
 * Copyright (C) 2024 Atmark Techno
 *
 * Driver for imx rpmsg
 */

#include <linux/init.h>
#include <linux/io.h>
#include <linux/imx_rpmsg.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pwm.h>
#include <linux/platform_device.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/rpmsg.h>
#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/regulator/consumer.h>

#define ADC_RPMSG_TIMEOUT_MS			500

#define ADC_RPMSG_VERSION			0x0002
#define ADC_RPMSG_TYPE_REQUEST			0x00
#define ADC_RPMSG_TYPE_RESPONSE			0x01 // SRTM_MessageTypeResponse
#define ADC_RPMSG_COMMAND_GET			0x00
#define ADC_RPMSG_COMMAND_INIT			0x01

struct fraction {
	uint8_t numerator;
	uint8_t denominator;
};

/* When the value of the CSCALE register is 0, the ADC input voltage level is
 * reduced by a factor of 30/64.
 */
static const struct fraction cscale_factor[] = {
	{
		.numerator = 30,
		.denominator = 64
	}, {
		.numerator = 1,
		.denominator = 1
	}
};

struct srtm_adc_init_payload {
	uint8_t adc_index;
	uint8_t adc_chan;
	uint8_t adc_side;
	uint8_t adc_scale;
	uint8_t adc_average;
} __packed;

struct adc_rpmsg_msg {
	struct imx_rpmsg_head header;

	/* Payload Start*/
	u8 request_id;
	u8 idx;
	union {
		struct srtm_adc_init_payload init;
		struct {
			u8 ret;
			u16 value;
		} __packed response;
	} __packed;
} __packed __aligned(1);

struct imx_rpmsg_adc {
	struct rpmsg_device *rpdev;
	struct completion cmd_complete;
	struct mutex lock;

	u8 last_retcode;
	u8 last_request_id;
	u16 inflight_request_id;
	u16 *requested_value;
	spinlock_t request_id_lock;
};

struct adc_channel_data {
	uint8_t adc_scale;
};
struct imx_rpmsg_adc_data {
	struct regulator *ref;
	struct adc_channel_data *channel_data;
	int num_channels;
};

/* rpmsg callback has a void *priv but it is not settable
 * when invoked with rpmsg_driver probe, so we need a global...
 */
static struct imx_rpmsg_adc adc_rpmsg;

#define IMX_RPMSG_VOLTAGE_CHANNEL(num, addr)                               \
	(struct iio_chan_spec){                                                            \
		.type = IIO_VOLTAGE,                                 \
		.address = (addr),                                 \
		.channel = (num),                                    \
		.indexed = 1,                                        \
		.scan_index = (num),                                 \
		.scan_type = {                                       \
			.sign = 'u',                                 \
			.realbits = 12,                              \
			.storagebits = 12,                           \
			.shift = 0,                                  \
			.endianness = IIO_CPU,                       \
		},                                                   \
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |       \
				BIT(IIO_CHAN_INFO_SCALE),\
	}

static int imx_rpmsg_adc_send_and_wait(struct adc_rpmsg_msg *msg, u16 *value)
{
	int ret;

	msg->header.cate = IMX_RPMSG_ADC;
	msg->header.major = ADC_RPMSG_VERSION;
	msg->header.minor = ADC_RPMSG_VERSION >> 8;
	msg->header.type = ADC_RPMSG_TYPE_REQUEST;

	mutex_lock(&adc_rpmsg.lock);
	reinit_completion(&adc_rpmsg.cmd_complete);
	if (value) {
		adc_rpmsg.requested_value = value;
	}

	/* We need the spin lock to ensure rpmsg cb does not set last_error
	 * after this timed out & another request started being processed */
	spin_lock_irq(&adc_rpmsg.request_id_lock);
	adc_rpmsg.inflight_request_id = adc_rpmsg.last_request_id++;
	spin_unlock_irq(&adc_rpmsg.request_id_lock);
	msg->request_id = adc_rpmsg.inflight_request_id;

	ret = rpmsg_send(adc_rpmsg.rpdev->ept, msg, sizeof(*msg));
	if (ret < 0) {
		dev_err(&adc_rpmsg.rpdev->dev, "rpmsg_send failed %d\n", ret);
		mutex_unlock(&adc_rpmsg.lock);
		return ret;
	}
	ret = wait_for_completion_timeout(&adc_rpmsg.cmd_complete,
					  msecs_to_jiffies(ADC_RPMSG_TIMEOUT_MS));
	mutex_unlock(&adc_rpmsg.lock);
	if (!ret) {
		/* don't process late replies - lock here ensures we're not completing
		 * the next call */
		spin_lock_irq(&adc_rpmsg.request_id_lock);
		adc_rpmsg.inflight_request_id = 0xffff;
		adc_rpmsg.requested_value = NULL;
		spin_unlock_irq(&adc_rpmsg.request_id_lock);
		dev_err(&adc_rpmsg.rpdev->dev, "rpmsg reply timeout\n");
		return -ETIMEDOUT;
	}
	return adc_rpmsg.last_retcode;
}

static int imx_rpmsg_adc_get(int idx, int *val)
{
	struct adc_rpmsg_msg msg = {
		.header.cmd = ADC_RPMSG_COMMAND_GET,
		.idx = idx,
	};
	int ret;
	u16 reply_val;

	if (!adc_rpmsg.rpdev)
		return -EINVAL;

	ret = imx_rpmsg_adc_send_and_wait(&msg, &reply_val);
	if (ret)
		return ret;

	*val = reply_val;

	return IIO_VAL_INT;
}

static int imx_rpmsg_read_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *channel, int *val,
			      int *val2, long mask)
{
	int cscale, ref_voltage_uv;
	struct imx_rpmsg_adc_data *data;

	data = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		return imx_rpmsg_adc_get(channel->channel, val);
	case IIO_CHAN_INFO_SCALE:
		ref_voltage_uv = regulator_get_voltage(data->ref);
		if (ref_voltage_uv < 0)
			return ref_voltage_uv;

		cscale = data->channel_data[channel->address].adc_scale;
		*val = ref_voltage_uv * cscale_factor[cscale].denominator / 1000;
		*val2 = (1 << channel->scan_type.realbits) * cscale_factor[cscale].numerator;
		return IIO_VAL_FRACTIONAL;
	default:
		return -EINVAL;
	}
}

static const struct iio_info imx_rpmsg_info = {
	.read_raw = imx_rpmsg_read_raw,
};

static int adc_rpmsg_cb(struct rpmsg_device *rpdev, void *data, int len,
			void *priv, u32 src)
{
	struct adc_rpmsg_msg *msg = data;

	if (!adc_rpmsg.rpdev) {
		dev_err(&rpdev->dev, "Ignoring message before init\n");
		return -EINVAL;
	}
	if (msg->header.type != ADC_RPMSG_TYPE_RESPONSE) {
		dev_err(&rpdev->dev, "bad type %x\n", msg->header.type);
		return -EINVAL;
	}
	spin_lock_irq(&adc_rpmsg.request_id_lock);
	if (msg->request_id != adc_rpmsg.inflight_request_id) {
		spin_unlock_irq(&adc_rpmsg.request_id_lock);
		dev_err(&rpdev->dev, "bad id %x (expected %x)\n",
			msg->request_id, adc_rpmsg.inflight_request_id);
		return -EINVAL;
	}
	/* don't process duplicates */
	adc_rpmsg.inflight_request_id = 0xffff;

	adc_rpmsg.last_retcode = msg->response.ret;
	if (adc_rpmsg.requested_value) {
		*adc_rpmsg.requested_value = msg->response.value;
		adc_rpmsg.requested_value = NULL;
	}
	spin_unlock_irq(&adc_rpmsg.request_id_lock);

	complete(&adc_rpmsg.cmd_complete);
	return 0;
}

#define READ_PROP_OR_RETURN(name) \
                ret = of_property_read_u32(chan_node, #name, &tmp); \
		if (!ret && tmp > 255) \
			ret = -ERANGE; \
                if (ret) { \
                        dev_err(dev, "%pOF: error reading %s: %d\n", \
                                chan_node, #name, ret); \
                        return ERR_PTR(ret); \
                } \
		msg.init. name = tmp

static struct iio_chan_spec const *
adc_rpmsg_init_remote(struct device *dev, int *num, struct imx_rpmsg_adc_data *data)
{
	struct adc_rpmsg_msg msg = {
		.header.cmd = ADC_RPMSG_COMMAND_INIT,
	};
	int count, ret, index;
	uint32_t tmp;
	struct iio_chan_spec *channels;
	struct device_node *chan_node;

	count = of_get_available_child_count(dev->of_node);
	if (count == 0) {
		dev_err(dev, "%pOF: no channel configured\n", dev->of_node);
		return ERR_PTR(-ENOENT);
	}

	channels = devm_kzalloc(dev, count * sizeof(struct iio_chan_spec), GFP_KERNEL);
	data->channel_data = devm_kzalloc(dev, count * sizeof(struct adc_channel_data), GFP_KERNEL);
	count = 0;

	for_each_available_child_of_node(dev->of_node, chan_node) {
		ret = of_property_read_u32(chan_node, "chan_index", &index);
		if (ret) {
			dev_err(dev, "%pOF: error reading chan_index: %d\n", chan_node, ret);
			continue;
		}
		msg.idx = index;
		READ_PROP_OR_RETURN(adc_index);
		READ_PROP_OR_RETURN(adc_chan);
		READ_PROP_OR_RETURN(adc_side);
		READ_PROP_OR_RETURN(adc_scale);
		READ_PROP_OR_RETURN(adc_average);

		if (msg.init.adc_scale >= ARRAY_SIZE(cscale_factor)) {
			dev_err(dev, "%pOF: error reading adc_scale(=%d) must be smaller than %lu.\n",
				chan_node, msg.init.adc_scale, ARRAY_SIZE(cscale_factor));
			continue;
		}

		ret = imx_rpmsg_adc_send_and_wait(&msg, NULL);
		if (ret) {
			dev_err(dev, "init of node %pOF failed: %d\n", chan_node, ret);
			continue;
		}

		channels[count] = IMX_RPMSG_VOLTAGE_CHANNEL(index, count);
		data->channel_data[count].adc_scale = msg.init.adc_scale;
		count++;
	}

	/* if we get here, some error messages were printed */
	if (count == 0)
		return ERR_PTR(-EINVAL);

	*num = count;
	data->num_channels = count;
	return channels;
}
#undef READ_PROP_OR_RETURN

static int adc_rpmsg_platform_probe(struct platform_device *pdev)
{
	struct iio_dev *indio_dev;
	struct imx_rpmsg_adc_data *data;
	int err;

	if (!adc_rpmsg.rpdev)
		return -EPROBE_DEFER;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*data));
	if (!indio_dev) {
		dev_err(&pdev->dev, "Failed to allocate IIO device\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, indio_dev);

	data = iio_priv(indio_dev);

	mutex_init(&adc_rpmsg.lock);
	spin_lock_init(&adc_rpmsg.request_id_lock);
	init_completion(&adc_rpmsg.cmd_complete);

	data->ref = devm_regulator_get(&pdev->dev, "vref");
	if (IS_ERR(data->ref))
		return PTR_ERR(data->ref);

	err = regulator_enable(data->ref);
	if (err < 0)
		return err;

	/* Initiate the Industrial I/O device */
	indio_dev->name = adc_rpmsg.rpdev->id.name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &imx_rpmsg_info;

	indio_dev->channels = adc_rpmsg_init_remote(&pdev->dev,
						    &indio_dev->num_channels,
						    data);
	if (IS_ERR(indio_dev->channels))
		return PTR_ERR(indio_dev->channels);

	dev_info(&pdev->dev, "Registering iio with %d channel(s)\n",
		 indio_dev->num_channels);
	/* TODO: trigger not implemented in m core */

	return devm_iio_device_register(&pdev->dev, indio_dev);
}

static int adc_rpmsg_platform_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev;
	struct imx_rpmsg_adc_data *data;

	indio_dev = platform_get_drvdata(pdev);
	data = iio_priv(indio_dev);

	regulator_disable(data->ref);

	dev_info(&pdev->dev, "rpmsg adc driver is removed\n");

	/* iio device is automatically freed, nothing to do? */

	return 0;
}

static const struct of_device_id adc_rpmsg_dt_ids[] = {
	{ .compatible = "fsl,imx-rpmsg-adc" },
	{ /* sentinel */ }
};

static struct platform_driver adc_rpmsg_platform_driver = {
	.driver = {
		.name = "imx-rpmsg-adc",
		.of_match_table = adc_rpmsg_dt_ids,
	},
	.probe = adc_rpmsg_platform_probe,
	.remove = adc_rpmsg_platform_remove,
};

static int adc_rpmsg_probe(struct rpmsg_device *rpdev)
{
	int rc;
	dev_info(&rpdev->dev, "new channel: 0x%x -> 0x%x!\n",
		 rpdev->src, rpdev->dst);

	rc = platform_driver_register(&adc_rpmsg_platform_driver);
	adc_rpmsg.rpdev = rpdev;
	return rc;
}

static void adc_rpmsg_remove(struct rpmsg_device *rpdev)
{
	dev_info(&rpdev->dev, "adc channel removed\n");
	platform_driver_unregister(&adc_rpmsg_platform_driver);
	adc_rpmsg.rpdev = NULL;
}

static struct rpmsg_device_id adc_rpmsg_id_table[] = {
	{ .name = "rpmsg-adc-channel" },
	{},
};

static struct rpmsg_driver adc_rpmsg_driver = {
	.drv.name = "adc_rpmsg",
	.drv.owner = THIS_MODULE,
	.id_table = adc_rpmsg_id_table,
	.probe = adc_rpmsg_probe,
	.callback = adc_rpmsg_cb,
	.remove = adc_rpmsg_remove,
};

static int __init imx_rpmsg_adc_init(void)
{
	return register_rpmsg_driver(&adc_rpmsg_driver);
}
static void __exit imx_rpmsg_adc_fini(void)
{
	unregister_rpmsg_driver(&adc_rpmsg_driver);
}
module_init(imx_rpmsg_adc_init);
module_exit(imx_rpmsg_adc_fini);


MODULE_AUTHOR("Dominique Martinet <dominique.martinet@atmark-techno.com>");
MODULE_DESCRIPTION("IMX RPMSG ADC driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
