// SPDX-License-Identifier: GPL-2.0-or-later
/**
 * Copyright (C) 2025 Atmark Techno
 *
 * Driver for imx rpmsg
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/err.h>

#include <linux/imx_rpmsg.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/rpmsg.h>
#include <linux/regulator/consumer.h>

#define DAC_RPMSG_TIMEOUT_MS	500
#define DAC_RPMSG_VERSION	0x0001
#define DAC_RPMSG_RESOLUTION	12

enum dac_rpmsg_header_type {
	DAC_RPMSG_TYPE_REQUEST,
	DAC_RPMSG_TYPE_RESPONSE,
};

enum dac_rpmsg_header_cmd {
	DAC_RPMSG_COMMAND_GET,
	DAC_RPMSG_COMMAND_SET,
	DAC_RPMSG_COMMAND_INIT,
};

struct dac_rpmsg_init_data
{
	u8 dac_vref;
	u8 dac_cref;
	u8 dac_speed;
} __packed;

struct dac_rpmsg_msg {
	struct imx_rpmsg_head header;

	/* Payload Start*/
	u8 request_id;
	u8 idx;
	union {
		u8 reserved;
		u8 retcode;
	} __packed;
	union {
		u32 value;
		struct dac_rpmsg_init_data init;
	} __packed;
} __packed;

struct imx_rpmsg_dac {
	struct rpmsg_device *rpdev;
	struct completion cmd_complete;
	struct mutex lock;

	u8 last_retcode;
	u8 last_request_id;
	u16 inflight_request_id;
	u16 *requested_value;
	spinlock_t request_id_lock;
};

/* rpmsg callback has a void *priv but it is not settable
 * when invoked with rpmsg_driver probe, so we need a global...
 */
static struct imx_rpmsg_dac dac_rpmsg;

struct imx_rpmsg_dac_data {
	u8 idx;
	struct regulator *ref;
};

static int imx_rpmsg_dac_send_and_wait(struct dac_rpmsg_msg *msg, u16 *value)
{
	int ret;

	msg->header.cate = IMX_RPMSG_DAC;
	msg->header.major = DAC_RPMSG_VERSION;
	msg->header.minor = DAC_RPMSG_VERSION >> 8;
	msg->header.type = DAC_RPMSG_TYPE_REQUEST;

	mutex_lock(&dac_rpmsg.lock);
	reinit_completion(&dac_rpmsg.cmd_complete);
	if (value) {
		dac_rpmsg.requested_value = value;
	}

	/* We need the spin lock to ensure rpmsg cb does not set last_error
	 * after this timed out & another request started being processed */
	spin_lock_irq(&dac_rpmsg.request_id_lock);
	dac_rpmsg.inflight_request_id = dac_rpmsg.last_request_id++;
	spin_unlock_irq(&dac_rpmsg.request_id_lock);
	msg->request_id = dac_rpmsg.inflight_request_id;

	ret = rpmsg_send(dac_rpmsg.rpdev->ept, msg, sizeof(*msg));
	if (ret < 0) {
		dev_err(&dac_rpmsg.rpdev->dev, "rpmsg_send failed %d\n", ret);
		mutex_unlock(&dac_rpmsg.lock);
		return ret;
	}
	ret = wait_for_completion_timeout(&dac_rpmsg.cmd_complete,
					  msecs_to_jiffies(DAC_RPMSG_TIMEOUT_MS));
	mutex_unlock(&dac_rpmsg.lock);
	if (!ret) {
		/* don't process late replies - lock here ensures we're not completing
		 * the next call */
		spin_lock_irq(&dac_rpmsg.request_id_lock);
		dac_rpmsg.inflight_request_id = 0xffff;
		dac_rpmsg.requested_value = NULL;
		spin_unlock_irq(&dac_rpmsg.request_id_lock);
		dev_err(&dac_rpmsg.rpdev->dev, "rpmsg reply timeout\n");
		return -ETIMEDOUT;
	}
	return dac_rpmsg.last_retcode;
}

static int imx_rpmsg_dac_get(int idx, int *val)
{
	struct dac_rpmsg_msg msg = {
		.header.cmd = DAC_RPMSG_COMMAND_GET,
		.idx = idx,
	};
	int ret;
	u16 reply_val;

	if (!dac_rpmsg.rpdev)
		return -EINVAL;

	ret = imx_rpmsg_dac_send_and_wait(&msg, &reply_val);
	if (ret)
		return ret;

	*val = reply_val;

	return IIO_VAL_INT;
}

static int imx_rpmsg_dac_set(int idx, const int val)
{
	struct dac_rpmsg_msg msg = {
		.header.cmd = DAC_RPMSG_COMMAND_SET,
		.idx = idx,
		.value = val,
	};

	if (!dac_rpmsg.rpdev)
		return -EINVAL;

	return imx_rpmsg_dac_send_and_wait(&msg, NULL);
}


static int imx_rpmsg_dac_read_raw(struct iio_dev *indio_dev,
				  struct iio_chan_spec const *chan,
				  int *val,
				  int *val2,
				  long m)
{
	struct imx_rpmsg_dac_data *data = iio_priv(indio_dev);
	int ret;

	switch (m) {
	case IIO_CHAN_INFO_RAW:
		return imx_rpmsg_dac_get(data->idx, val);
	case IIO_CHAN_INFO_SCALE:
		ret = regulator_get_voltage(data->ref);
		if (ret < 0)
			return ret;
		/* Corresponds to Vref / 2^(bits) */
		*val = ret / 1000;
		*val2 = DAC_RPMSG_RESOLUTION;
		return IIO_VAL_FRACTIONAL_LOG2;
	default:
		break;
	}

	return -EINVAL;
}

static int imx_rpmsg_dac_write_raw(struct iio_dev *indio_dev,
				   struct iio_chan_spec const *chan, int val, int val2, long mask)
{
	struct imx_rpmsg_dac_data *data = iio_priv(indio_dev);

	if (val2 != 0)
		return -EINVAL;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		return imx_rpmsg_dac_set(data->idx, val);
	default:
		break;
	}

	return -EINVAL;
}

static const struct iio_chan_spec imx_rpmsg_dac_channel = {
	.type = IIO_VOLTAGE,
	.indexed = 1,
	.output = 1,
	.channel = 0,
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
	BIT(IIO_CHAN_INFO_SCALE),
};

static const struct iio_info imx_rpmsg_dac_info = {
	.read_raw = imx_rpmsg_dac_read_raw,
	.write_raw = imx_rpmsg_dac_write_raw,
};

static int dac_rpmsg_cb(struct rpmsg_device *rpdev, void *data, int len,
			void *priv, u32 src)
{
	struct dac_rpmsg_msg *msg = data;

	if (!dac_rpmsg.rpdev) {
		dev_err(&rpdev->dev, "Ignoring message before init\n");
		return -EINVAL;
	}
	if (msg->header.type != DAC_RPMSG_TYPE_RESPONSE) {
		dev_err(&rpdev->dev, "bad type %x\n", msg->header.type);
		return -EINVAL;
	}
	spin_lock_irq(&dac_rpmsg.request_id_lock);
	if (msg->request_id != dac_rpmsg.inflight_request_id) {
		spin_unlock_irq(&dac_rpmsg.request_id_lock);
		dev_err(&rpdev->dev, "bad id %x (expected %x)\n",
			msg->request_id, dac_rpmsg.inflight_request_id);
		return -EINVAL;
	}
	/* don't process duplicates */
	dac_rpmsg.inflight_request_id = 0xffff;

	dac_rpmsg.last_retcode = msg->retcode;
	if (dac_rpmsg.requested_value) {
		*dac_rpmsg.requested_value = msg->value;
		dac_rpmsg.requested_value = NULL;
	}
	spin_unlock_irq(&dac_rpmsg.request_id_lock);

	complete(&dac_rpmsg.cmd_complete);
	return 0;
}

#define READ_PROP_OR_RETURN(name)				\
	ret = of_property_read_u32(dev->of_node, #name, &tmp);	\
	if (!ret && tmp > 255)					\
		ret = -ERANGE;					\
	if (ret) {						\
		dev_err(dev, "%pOF: error reading %s: %d\n",	\
			chan_node, #name, ret);			\
		return ret;					\
	}							\
	msg.init. name = tmp

static int dac_rpmsg_init_remote(struct device *dev, struct imx_rpmsg_dac_data *data)
{
	struct dac_rpmsg_msg msg = {
		.header.cmd = DAC_RPMSG_COMMAND_INIT,
	};
	int ret, index;
	uint32_t tmp;
	struct device_node *chan_node;

	ret = of_property_read_u32(dev->of_node, "dac_index", &index);
	if (ret) {
		dev_err(dev, "%pOF: error reading dac_index: %d\n", dev->of_node, ret);
	}
	msg.idx = index;
	READ_PROP_OR_RETURN(dac_vref);
	READ_PROP_OR_RETURN(dac_cref);
	READ_PROP_OR_RETURN(dac_speed);

	ret = imx_rpmsg_dac_send_and_wait(&msg, NULL);
	if (ret) {
		dev_err(dev, "init of node %pOF failed: %d\n", dev->of_node, ret);
	}

	data->idx = index;

	return ret;
}
#undef READ_PROP_OR_RETURN

static int dac_rpmsg_platform_probe(struct platform_device *pdev)
{
	struct iio_dev *indio_dev;
	struct imx_rpmsg_dac_data *data;
	int err;

	if (!dac_rpmsg.rpdev)
		return -EPROBE_DEFER;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*data));
	if (!indio_dev) {
		dev_err(&pdev->dev, "Failed to allocate IIO device\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, indio_dev);

	data = iio_priv(indio_dev);

	mutex_init(&dac_rpmsg.lock);
	spin_lock_init(&dac_rpmsg.request_id_lock);
	init_completion(&dac_rpmsg.cmd_complete);

	data->ref = devm_regulator_get(&pdev->dev, "vref");
	if (IS_ERR(data->ref))
		return PTR_ERR(data->ref);

	err = regulator_enable(data->ref);
	if (err < 0)
		return err;

	/* Initiate the Industrial I/O device */
	indio_dev->name = dac_rpmsg.rpdev->id.name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &imx_rpmsg_dac_info;
	indio_dev->num_channels = 1;
	indio_dev->channels = &imx_rpmsg_dac_channel;

	err = dac_rpmsg_init_remote(&pdev->dev, data);
	if (err < 0)
		return PTR_ERR(indio_dev->channels);

	dev_info(&pdev->dev, "Registering iio with %d channel(s)\n",
		 indio_dev->num_channels);
	/* TODO: trigger not implemented in m core */

	return devm_iio_device_register(&pdev->dev, indio_dev);
}

static int dac_rpmsg_platform_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev;
	struct imx_rpmsg_dac_data *data;

	indio_dev = platform_get_drvdata(pdev);
	data = iio_priv(indio_dev);

	regulator_disable(data->ref);

	dev_info(&pdev->dev, "rpmsg dac driver is removed\n");

	/* iio device is automatically freed, nothing to do? */

	return 0;
}

static const struct of_device_id dac_rpmsg_dt_ids[] = {
	{ .compatible = "fsl,imx-rpmsg-dac" },
	{ /* sentinel */ }
};

static struct platform_driver dac_rpmsg_platform_driver = {
	.driver = {
		.name = "imx-rpmsg-dac",
		.of_match_table = dac_rpmsg_dt_ids,
	},
	.probe = dac_rpmsg_platform_probe,
	.remove = dac_rpmsg_platform_remove,
};

static int dac_rpmsg_probe(struct rpmsg_device *rpdev)
{
	int rc;

	dev_info(&rpdev->dev, "new channel: 0x%x -> 0x%x!\n",
		 rpdev->src, rpdev->dst);

	rc = platform_driver_register(&dac_rpmsg_platform_driver);
	dac_rpmsg.rpdev = rpdev;

	return rc;
}

static void dac_rpmsg_remove(struct rpmsg_device *rpdev)
{
	dev_info(&rpdev->dev, "dac channel removed\n");
	platform_driver_unregister(&dac_rpmsg_platform_driver);
	dac_rpmsg.rpdev = NULL;
}


static struct rpmsg_device_id dac_rpmsg_id_table[] = {
	{ .name = "rpmsg-dac-channel" },
	{},
};

static struct rpmsg_driver dac_rpmsg_driver = {
	.drv.name = "dac_rpmsg",
	.drv.owner = THIS_MODULE,
	.id_table = dac_rpmsg_id_table,
	.probe = dac_rpmsg_probe,
	.callback = dac_rpmsg_cb,
	.remove = dac_rpmsg_remove,
};

static int __init imx_rpmsg_dac_init(void)
{
	return register_rpmsg_driver(&dac_rpmsg_driver);
}
static void __exit imx_rpmsg_dac_exit(void)
{
	unregister_rpmsg_driver(&dac_rpmsg_driver);
}
module_init(imx_rpmsg_dac_init);
module_exit(imx_rpmsg_dac_exit);

MODULE_AUTHOR("Daisuke Mizobuchi <mizo@atmark-techno.com>");
MODULE_DESCRIPTION("IMX RPMSG DAC driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
