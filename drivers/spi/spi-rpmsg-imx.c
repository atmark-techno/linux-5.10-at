/*
 * Copyright 2019 NXP
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/delay.h>
#include <linux/idr.h>
#include <linux/imx_rpmsg.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/rpmsg.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/time.h>
#include <linux/types.h>

#define SPI_RPMSG_TIMEOUT			500 /* unit: ms */

#define SPI_RPMSG_VERSION			0x0002

#define SPI_RPMSG_TYPE_REQUEST			0x00
#define SPI_RPMSG_TYPE_RESPONSE			0x01

enum spi_rpmsg_commands
{
	/* SPI Service Request Command definition */
	SPI_RPMSG_COMMAND_TRANSFER = 0U,
	SPI_RPMSG_COMMAND_INIT,
};

#define SPI_RPMSG_PRIORITY			0x01

#define SPI_RPMSG_M_STOP			0x0200

enum {
	SPI_TYPE_GPIO,
	SPI_TYPE_LPSPI,
};

struct spi_rpmsg_init_payload {
	uint32_t type;
	union {
		struct {
			uint32_t sck_pin;
			uint32_t miso_pin;
			uint32_t mosi_pin;
			uint32_t mode;
		} __packed gpio;
		struct {
			uint32_t spi_index;
			uint32_t mode;
		} __packed lpspi;
	} __packed;
} __packed;

/* spi_rpmsg_msg needs to fit within RPMSG_MAX_PAYLOAD_SIZE */
#define SPI_RPMSG_HDR_SIZE (IMX_RPMSG_HEAD_SIZE + 10)
#define RPMSG_MAX_SIZE  (RPMSG_MAX_PAYLOAD_SIZE - IMX_RPMSG_HEAD_SIZE)
#define SPI_RPMSG_MAX_BUF_SIZE	(RPMSG_MAX_PAYLOAD_SIZE - SPI_RPMSG_HDR_SIZE)
struct spi_rpmsg_msg {
	struct imx_rpmsg_head header;

	/* Payload Start*/
	u8 bus_id;
	u8 ret_val;
	u16 bits_per_word;
	u32 speed_hz;
	u16 len;
	union {
		u8 buf[SPI_RPMSG_MAX_BUF_SIZE];
		struct spi_rpmsg_init_payload init;
	} __packed;
} __packed __aligned(1);

struct spi_rpmsg_info {
	struct rpmsg_device *rpdev;
	struct device *dev;
	struct completion cmd_complete;
	struct mutex lock;
	struct ida ida;

	u8 ret_val;
	u8 bus_id;
	u16 len;
	u8 *rx_buf;
};

struct rpmsg_spi {
	u8 id;
};

static struct spi_rpmsg_info spi_rpmsg;

static int spi_rpmsg_cb(struct rpmsg_device *rpdev, void *data, int len,
			void *priv, u32 src)
{
	struct spi_rpmsg_msg *msg = (struct spi_rpmsg_msg *)data;

	if (msg->header.type != SPI_RPMSG_TYPE_RESPONSE)
		return -EINVAL;

	if (msg->bus_id != spi_rpmsg.bus_id) {
		dev_err(&rpdev->dev,
			"expected bus_id:%d, received bus_id:%d\n",
			spi_rpmsg.bus_id, msg->bus_id);
		return -EINVAL;
	}

	// this indirectly checks SPI_RPMSG_HDR_SIZE is valid
	BUILD_BUG_ON(sizeof(struct spi_rpmsg_msg) != RPMSG_MAX_PAYLOAD_SIZE);
	if (msg->len > len - SPI_RPMSG_HDR_SIZE) {
		dev_err(&rpdev->dev,
			"%s failed: data length greater than %ld, len=%d\n",
			__func__, len - SPI_RPMSG_HDR_SIZE, msg->len);
		return -EINVAL;
	}

	/* Receive Success */
	spi_rpmsg.ret_val = msg->ret_val;

	if (spi_rpmsg.rx_buf) {
		/* we could copy min, but mismatch is an error anyway */
		if (spi_rpmsg.len == msg->len)
			memcpy(spi_rpmsg.rx_buf, msg->buf, spi_rpmsg.len);
		spi_rpmsg.len = msg->len;
	}

	complete(&spi_rpmsg.cmd_complete);

	return 0;
}

static int rpmsg_xfer(struct spi_rpmsg_msg *rmsg, struct spi_rpmsg_info *info)
{
	int ret, size;

	reinit_completion(&info->cmd_complete);

	rmsg->header.cate = IMX_RPMSG_SPI;
	rmsg->header.major = SPI_RPMSG_VERSION;
	rmsg->header.minor = SPI_RPMSG_VERSION >> 8;
	rmsg->header.type = SPI_RPMSG_TYPE_REQUEST;
	rmsg->header.reserved[0] = SPI_RPMSG_PRIORITY;
	switch (rmsg->header.cmd) {
	case SPI_RPMSG_COMMAND_TRANSFER:
		size = sizeof(struct spi_rpmsg_msg) - SPI_RPMSG_MAX_BUF_SIZE + rmsg->len;
		break;
	case SPI_RPMSG_COMMAND_INIT:
		size = sizeof(struct spi_rpmsg_msg) - SPI_RPMSG_MAX_BUF_SIZE +
			sizeof(struct spi_rpmsg_init_payload);
		break;
	default:
		WARN_ON(1);
		return -EINVAL;
	}
	info->bus_id = rmsg->bus_id;

	ret = rpmsg_send(info->rpdev->ept, (void *)rmsg, size);
	if (ret < 0) {
		dev_err(&info->rpdev->dev, "rpmsg_send failed: %d\n", ret);
		return ret;
	}

	ret = wait_for_completion_timeout(&info->cmd_complete,
					msecs_to_jiffies(SPI_RPMSG_TIMEOUT));
	if (!ret) {
		dev_err(&info->rpdev->dev, "%s failed: timeout\n", __func__);
		return -ETIMEDOUT;
	}

	if (info->ret_val) {
		dev_dbg(&info->rpdev->dev,
			"%s failed: %d\n", __func__, info->ret_val);
		return -(info->ret_val);
	}

	return 0;
}

static int spi_rpmsg_transfer_one(struct spi_master *master,
				  struct spi_device *spi,
				  struct spi_transfer *transfer)
{
	struct rpmsg_spi *devdata = spi_master_get_devdata(master);
	struct spi_rpmsg_msg rmsg;
	int status = 0;

	if (transfer->len > SPI_RPMSG_MAX_BUF_SIZE) {
		dev_warn(&master->dev, "Failing transfer of %d bytes (%ld max)\n",
			 transfer->len, SPI_RPMSG_MAX_BUF_SIZE);
		status = -ERANGE;
		goto out;
	}

	if (!transfer->len)
		goto out;

	memset(&rmsg, 0, sizeof(SPI_RPMSG_HDR_SIZE));
	rmsg.header.cmd = SPI_RPMSG_COMMAND_TRANSFER;
	rmsg.bus_id = devdata->id;
	rmsg.bits_per_word = transfer->bits_per_word;
	rmsg.speed_hz = transfer->speed_hz;
	rmsg.len = transfer->len;
	memcpy(rmsg.buf, transfer->tx_buf, transfer->len);

	mutex_lock(&spi_rpmsg.lock);
	spi_rpmsg.rx_buf = transfer->rx_buf;
	spi_rpmsg.len = transfer->len;

	status = rpmsg_xfer(&rmsg, &spi_rpmsg);

	if (status)
		goto out_unlock;
	if (spi_rpmsg.len != transfer->len) {
		status = -EREMOTEIO;
		goto out_unlock;
	}
	// data already copied to rx buf in cb so nothing else to do here
	spi_rpmsg.rx_buf = NULL;

out_unlock:
	mutex_unlock(&spi_rpmsg.lock);
out:
	spi_finalize_current_transfer(master);

	return status;
}

static int spi_rpmsg_init_remote(struct device *dev, int bus_id)
{
	struct spi_rpmsg_msg rmsg = { 0 };
	/* need to build locally and memcpy for alignment */
	struct spi_rpmsg_init_payload init = { 0 };
	struct device_node *np = dev->of_node;
	int ret;

	rmsg.header.cmd = SPI_RPMSG_COMMAND_INIT;
	rmsg.bus_id = bus_id;
	rmsg.len = sizeof(struct spi_rpmsg_init_payload);

	ret = of_property_read_u32(np, "spi_type", &init.type);
	if (ret) {
		dev_err(dev, "%pOF: error reading spi_type: %d\n", np, ret);
		return ret;
	}

	switch (init.type) {
	case SPI_TYPE_GPIO:
		ret = of_property_read_u32(np, "spi_SCK_pin", &init.gpio.sck_pin);
		if (ret) {
			dev_err(dev, "%pOF: error reading spi_SCK_pin: %d\n", np, ret);
			return ret;
		}
		ret = of_property_read_u32(np, "spi_MISO_pin", &init.gpio.miso_pin);
		if (ret) {
			dev_err(dev, "%pOF: error reading spi_MISO_pin: %d\n", np, ret);
			return ret;
		}
		ret = of_property_read_u32(np, "spi_MOSI_pin", &init.gpio.mosi_pin);
		if (ret) {
			dev_err(dev, "%pOF: error reading spi_MOSI_pin: %d\n", np, ret);
			return ret;
		}
		init.gpio.mode = 0;
		break;
	case SPI_TYPE_LPSPI:
		ret = of_property_read_u32(np, "spi_index", &init.lpspi.spi_index);
		if (ret) {
			dev_err(dev, "%pOF: error reading spi_MOSI_pin: %d\n", np, ret);
			return ret;
		}
		init.lpspi.mode = 0;
		break;
	default:
		dev_err(dev, "%pOF: invalid spi_type %d\n", np, init.type);
		return -EINVAL;
	}

	memcpy(&rmsg.init, &init, sizeof(init));
	mutex_lock(&spi_rpmsg.lock);
	ret = rpmsg_xfer(&rmsg, &spi_rpmsg);
	mutex_unlock(&spi_rpmsg.lock);
	if (ret)
		dev_err(dev, "%pOF: register failed, check parameters: %d\n", np, ret);

	return ret;
}

static int spi_rpmsg_platform_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct spi_master *master;
	struct rpmsg_spi *devdata;
	int ret;

	if (!spi_rpmsg.rpdev)
		return -EPROBE_DEFER;

	master = devm_spi_alloc_master(dev, sizeof(*devdata));
	if (!master)
		return -ENOMEM;

	master->dev.of_node = np;
	master->bits_per_word_mask = SPI_BPW_RANGE_MASK(1, 32);
	master->mode_bits = 0;

	/* do not store it in bus_num to allow arbitrary alias */
	devdata = spi_master_get_devdata(master);
	devdata->id = ida_simple_get(&spi_rpmsg.ida, 0, 0, GFP_KERNEL);

	master->transfer_one = spi_rpmsg_transfer_one;

	ret = spi_rpmsg_init_remote(&pdev->dev, devdata->id);
	if (ret)
		goto error;


	ret = devm_spi_register_master(dev, master);
	if (ret < 0) {
		dev_err(dev, "failed to register SPI master: %d\n", ret);
		goto error;
	}

	dev_info(dev, "registered SPI%d driver successfully (%d)\n",
		 master->bus_num, devdata->id);

	return 0;

error:
	ida_free(&spi_rpmsg.ida, devdata->id);
	return ret;
}

static int spi_rpmsg_platform_remove(struct platform_device *pdev)
{
	// nothing to do thanks to devm

	return 0;
}

static const struct of_device_id imx_rpmsg_spi_dt_ids[] = {
	{ .compatible = "fsl,spi-rpmsg", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_rpmsg_spi_dt_ids);

static struct platform_driver imx_rpmsg_spi_platform_driver = {
	.driver = {
		.name	= "imx_rpmsg_spi",
		.of_match_table = imx_rpmsg_spi_dt_ids,
	},
	.probe		= spi_rpmsg_platform_probe,
	.remove		= spi_rpmsg_platform_remove
};

static int spi_rpmsg_rpmsg_probe(struct rpmsg_device *rpdev)
{
	int rc;

	if (!rpdev) {
		dev_info(&rpdev->dev, "%s failed, rpdev=NULL\n", __func__);
		return -EINVAL;
	}

	mutex_init(&spi_rpmsg.lock);
	init_completion(&spi_rpmsg.cmd_complete);
	ida_init(&spi_rpmsg.ida);

	dev_info(&rpdev->dev, "new channel: 0x%x -> 0x%x!\n",
		 rpdev->src, rpdev->dst);

	/* platform_driver_register() calls the platform probe immediately,
	 * but we need it to process rpmsg replies so only set rpdev last
	 * to make the first platform probe defer */
	rc = platform_driver_register(&(imx_rpmsg_spi_platform_driver));

	spi_rpmsg.rpdev = rpdev;
	return rc;
}

static void spi_rpmsg_rpmsg_remove(struct rpmsg_device *rpdev)
{
	platform_driver_unregister(&imx_rpmsg_spi_platform_driver);
	spi_rpmsg.rpdev = NULL;
	dev_info(&rpdev->dev, "spi rpmsg driver is removed\n");
	ida_destroy(&spi_rpmsg.ida);
}

static struct rpmsg_device_id spi_rpmsg_id_table[] = {
	{ .name	= "rpmsg-spi-channel" },
	{ },
};

static struct rpmsg_driver spi_rpmsg_driver = {
	.drv.name	= "spi-rpmsg",
	.drv.owner	= THIS_MODULE,
	.id_table	= spi_rpmsg_id_table,
	.probe		= spi_rpmsg_rpmsg_probe,
	.remove		= spi_rpmsg_rpmsg_remove,
	.callback	= spi_rpmsg_cb,
};

static int __init imx_rpmsg_spi_driver_init(void)
{
	return register_rpmsg_driver(&spi_rpmsg_driver);
}
subsys_initcall(imx_rpmsg_spi_driver_init);

MODULE_AUTHOR("Dominique Martinet <dominique.martinet@atmark-techno.com>");
MODULE_DESCRIPTION("Driver for spi over rpmsg");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:spi-rpmsg");
