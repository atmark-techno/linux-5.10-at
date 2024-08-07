// SPDX-License-Identifier: GPL-2.0-only
/**
 * Copyright (C) 2024 Atmark Techno
 *
 * Driver for imx rpmsg
 */

/*
 * The adc-rpmsg transfer protocol
 *   +---------------+-------------------------------+
 *   |  Byte Offset  |            Content            |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       0       |           Category            |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |     1 ~ 2     |           Version             |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       3       |             Type              |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       4       |           Command             |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       5       |           Reserved0           |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       6       |           Reserved1           |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       7       |           Reserved2           |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       8       |           Reserved3           |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       9       |           Reserved4           |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       10      |          Request ID           |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       11      |           ADC index           |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       12      |          Return code          |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       13      |           ADC value           |
 *   +---------------+---+---+---+---+---+---+---+---+
 *
 * Return values are
 *  0x00 success
 *  0x01 failed
 *  0x02 not supported (bad protocol or command)
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

#define ADC_RPMSG_TIMEOUT_MS                    500

#define ADC_RPMSG_VERSION                       0x0001
#define ADC_RPMSG_TYPE_REQUEST                  0x00
#define ADC_RPMSG_TYPE_RESPONSE                 0x01 // SRTM_MessageTypeResponse
#define ADC_RPMSG_COMMAND_GET                   0x00
#define ADC_RPMSG_COMMAND_SET                   0x01

struct adc_rpmsg_msg {
	struct imx_rpmsg_head header;

	/* Payload Start*/
	u8 request_id;
	u8 idx;
	u8 ret;
	u16 value;
} __packed __aligned(1);

struct imx_rpmsg {
	struct rpmsg_device *rpdev;
	struct adc_rpmsg_msg *msg;
	struct completion cmd_complete;
	struct mutex lock;

	u8 request_id;
};

/* rpmsg callback has a void *priv but it is not settable
 * when invoked with rpmsg_driver probe, so we need a global...
 */
static struct imx_rpmsg *adc_rpmsg;

#define IMX_RPMSG_VOLTAGE_CHANNEL(num)                               \
	{                                                            \
		.type = IIO_VOLTAGE,                                 \
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
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),        \
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),\
	}

static const struct iio_chan_spec imx_rpmsg_channels[] = {
	IMX_RPMSG_VOLTAGE_CHANNEL(0),
};

static int rpmsg_send_and_wait(struct imx_rpmsg *adc, struct adc_rpmsg_msg *msg)
{
	int ret;

	adc->request_id++;
	msg->request_id = adc->request_id;
	msg->header.cate = IMX_RPMSG_ADC;
	msg->header.major = ADC_RPMSG_VERSION;
	msg->header.minor = ADC_RPMSG_VERSION >> 8;
	msg->header.type = ADC_RPMSG_TYPE_REQUEST;

	ret = rpmsg_send(adc->rpdev->ept, msg, sizeof(*msg));
	if (ret < 0) {
		dev_err(&adc->rpdev->dev, "rpmsg_send failed %d\n", ret);
		return ret;
	}
	ret = wait_for_completion_timeout(&adc->cmd_complete,
					  msecs_to_jiffies(ADC_RPMSG_TIMEOUT_MS));
	if (!ret) {
		dev_err(&adc->rpdev->dev, "rpmsg reply timeout\n");
		return -ETIMEDOUT;
	}
	return 0;
}

static int imx_rpmsg_adc_get(struct imx_rpmsg *adc, int idx, int *val)
{
	struct adc_rpmsg_msg msg = {
		.header.cmd = ADC_RPMSG_COMMAND_GET,
		.idx = idx,
	};
	int ret;

	if (!adc || !adc->rpdev)
		return -EINVAL;

	ret = rpmsg_send_and_wait(adc, &msg);
	if (ret)
		return ret;

	*val = adc->msg->value;

	return IIO_VAL_INT;
}

static int imx_rpmsg_read_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *channel, int *val,
			      int *val2, long mask)
{
	struct imx_rpmsg *adc = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		/* TODO get idx */
		return imx_rpmsg_adc_get(adc, 0, val);
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

	if (!adc_rpmsg || !adc_rpmsg->rpdev) {
		dev_err(&rpdev->dev, "Ignoring message before init\n");
		return -EINVAL;
	}
	if (msg->header.type != ADC_RPMSG_TYPE_RESPONSE) {
		dev_err(&rpdev->dev, "bad type %x\n", msg->header.type);
		return -EINVAL;
	}
	if (msg->request_id != adc_rpmsg->request_id) {
		dev_err(&rpdev->dev, "bad id %x (expected %x)\n",
			msg->request_id, adc_rpmsg->request_id);
		return -EINVAL;
	}

	adc_rpmsg->msg = msg;
	complete(&adc_rpmsg->cmd_complete);
	return 0;
}

static int adc_rpmsg_probe(struct rpmsg_device *rpdev)
{
	struct iio_dev *indio_dev;
	struct imx_rpmsg *adc;

	if (adc_rpmsg) {
		// reinit
		adc_rpmsg->rpdev = rpdev;
		return 0;
	}

	indio_dev = devm_iio_device_alloc(&rpdev->dev, sizeof(*adc));
	if (!indio_dev) {
		dev_err(&rpdev->dev, "Failed to allocate IIO device\n");
		return -ENOMEM;
	}

	adc = iio_priv(indio_dev);
	adc->rpdev = rpdev;

	mutex_init(&adc->lock);
	init_completion(&adc->cmd_complete);

	/* Initiate the Industrial I/O device */
	indio_dev->name = rpdev->id.name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &imx_rpmsg_info;
	indio_dev->channels = imx_rpmsg_channels;
	indio_dev->num_channels = ARRAY_SIZE(imx_rpmsg_channels);

	/* TODO: get of by name ? other imx rpmsg drivers register both a rpmsg driver
	 * and a platform driver by dtb, but we don't need the dtb at all and double-register
	 * + static variable is horrible so just getting of node here is probably better..
	 */
	/* TODO: send configuration at probe time */
	/* TODO: trigger not implemented in m core */

	dev_info(&rpdev->dev, "new adc channel: 0x%x -> 0x%x!\n", rpdev->src, rpdev->dst);
	/* set global for cb */
	adc_rpmsg = adc;

	return devm_iio_device_register(&rpdev->dev, indio_dev);
}

static void adc_rpmsg_remove(struct rpmsg_device *rpdev)
{
	dev_info(&rpdev->dev, "removed adc channel\n");
	adc_rpmsg->rpdev = NULL;
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
device_initcall(imx_rpmsg_adc_init);

MODULE_AUTHOR("Dominique Martinet <dominique.martinet@atmark-techno.com>");
MODULE_DESCRIPTION("IMX RPMSG ADC driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
