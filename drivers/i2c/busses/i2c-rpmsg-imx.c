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

/* The i2c-rpmsg transfer protocol:
 *
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
 *   |       5       |           Priority            |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       6       |           Reserved1           |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       7       |           Reserved2           |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       8       |           Reserved3           |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       9       |           Reserved4           |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       10      |            BUS ID             |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       11      |         Return Value          |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |    12 ~ 13    |            BUS ID             |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |    14 ~ 15    |            Address            |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |    16 ~ 17    |           Data Len            |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |    18 ~ 33    |        16 Bytes Data          |
 *   +---------------+---+---+---+---+---+---+---+---+
 *
 * The definition of Return Value:
 * 0x00 = Success
 * 0x01 = Failed
 * 0x02 = Invalid parameter
 * 0x03 = Invalid message
 * 0x04 = Operate in invalid state
 * 0x05 = Memory allocation failed
 * 0x06 = Timeout when waiting for an event
 * 0x07 = Cannot add to list as node already in another list
 * 0x08 = Cannot remove from list as node not in list
 * 0x09 = Transfer timeout
 * 0x0A = Transfer failed due to peer core not ready
 * 0x0B = Transfer failed due to communication failure
 * 0x0C = Cannot find service for a request/notification
 * 0x0D = Service version cannot support the request/notification
 *
 */

#include <linux/idr.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/imx_rpmsg.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/rpmsg.h>

#define I2C_RPMSG_MAX_BUF_SIZE			(RPMSG_MAX_PAYLOAD_SIZE - 18)
#define I2C_RPMSG_TIMEOUT			500 /* unit: ms */

#define I2C_RPMSG_VERSION			0x0001

#define I2C_RPMSG_TYPE_REQUEST			0x00
#define I2C_RPMSG_TYPE_RESPONSE			0x01

#define I2C_RPMSG_COMMAND_READ			0x00
#define I2C_RPMSG_COMMAND_WRITE			0x01
#define I2C_RPMSG_COMMAND_INIT			0x02

#define I2C_RPMSG_PRIORITY			0x01

#define I2C_RPMSG_M_STOP			0x0200

enum {
	I2C_TYPE_LPI2C,
	I2C_TYPE_FLEXIO,
};

struct i2c_rpmsg_init_payload {
	uint32_t i2c_type;
	uint32_t i2c_index;
	uint32_t i2c_baudrate;
	union {
		/* nothing for lpi2c */
		struct {
			uint32_t scl_pin;
			uint32_t sda_pin;
		} __packed flexio;
	} __packed;
} __packed;

/* i2c_rpmsg_msg needs to fit within RPMSG_MAX_PAYLOAD_SIZE */
#define I2C_RPMSG_HDR_SIZE (IMX_RPMSG_HEAD_SIZE + 8)
#define RPMSG_MAX_SIZE  (RPMSG_MAX_PAYLOAD_SIZE - IMX_RPMSG_HEAD_SIZE)
struct i2c_rpmsg_msg {
	struct imx_rpmsg_head header;

	/* Payload Start*/
	u8 bus_id;
	u8 ret_val;
	u16 addr;
	u16 flags;
	u16 len;
	union {
		u8 buf[I2C_RPMSG_MAX_BUF_SIZE];
		struct i2c_rpmsg_init_payload init;
	} __packed;
} __packed __aligned(1);

struct i2c_rpmsg_info {
	struct rpmsg_device *rpdev;
	struct device *dev;
	struct completion cmd_complete;
	struct mutex lock;
	struct ida ida;

	u8 ret_val;
	u8 bus_id;
	u16 addr;
	u16 len;
	u8 *buf;
};

static struct i2c_rpmsg_info i2c_rpmsg;

struct imx_rpmsg_i2c_data {
	struct i2c_adapter adapter;
};

static int i2c_rpmsg_cb(struct rpmsg_device *rpdev, void *data, int len,
			   void *priv, u32 src)
{
	struct i2c_rpmsg_msg *msg = (struct i2c_rpmsg_msg *)data;

	if (msg->header.type != I2C_RPMSG_TYPE_RESPONSE)
		return -EINVAL;

	if (msg->bus_id != i2c_rpmsg.bus_id || msg->addr != i2c_rpmsg.addr) {
		dev_err(&rpdev->dev,
		"expected bus_id:%d, addr:%2x, received bus_id:%d, addr:%2x\n",
		i2c_rpmsg.bus_id, i2c_rpmsg.addr, msg->bus_id, msg->addr);
		return -EINVAL;
	}

	// this indirectly checks I2C_RPMSG_HDR_SIZE is valid
	BUILD_BUG_ON(sizeof(struct i2c_rpmsg_msg) != RPMSG_MAX_PAYLOAD_SIZE);
	if (msg->len > len - I2C_RPMSG_HDR_SIZE) {
		dev_err(&rpdev->dev,
		"%s failed: data length greater than %ld, len=%d\n",
		__func__, len - I2C_RPMSG_HDR_SIZE, msg->len);
		return -EINVAL;
	}

	/* Receive Success */
	i2c_rpmsg.ret_val = msg->ret_val;

	if (i2c_rpmsg.buf) {
		/* we could copy min, but mismatch is an error anyway */
		if (i2c_rpmsg.len == msg->len)
			memcpy(i2c_rpmsg.buf, msg->buf, i2c_rpmsg.len);
		i2c_rpmsg.len = msg->len;
	}

	complete(&i2c_rpmsg.cmd_complete);

	return 0;
}

static int rpmsg_xfer(struct i2c_rpmsg_msg *rmsg, struct i2c_rpmsg_info *info)
{
	int ret, size;

	reinit_completion(&info->cmd_complete);

	rmsg->header.cate = IMX_RPMSG_I2C;
	rmsg->header.major = I2C_RPMSG_VERSION;
	rmsg->header.minor = I2C_RPMSG_VERSION >> 8;
	rmsg->header.type = I2C_RPMSG_TYPE_REQUEST;
	rmsg->header.reserved[0] = I2C_RPMSG_PRIORITY;
	switch (rmsg->header.cmd) {
	case I2C_RPMSG_COMMAND_WRITE:
		size = sizeof(struct i2c_rpmsg_msg) - I2C_RPMSG_MAX_BUF_SIZE + rmsg->len;
		break;
	case I2C_RPMSG_COMMAND_READ:
		size = sizeof(struct i2c_rpmsg_msg) - I2C_RPMSG_MAX_BUF_SIZE;
		break;
	case I2C_RPMSG_COMMAND_INIT:
		size = sizeof(struct i2c_rpmsg_msg) - I2C_RPMSG_MAX_BUF_SIZE +
			sizeof(struct i2c_rpmsg_init_payload);
		break;
	default:
		WARN_ON(1);
		return -EINVAL;
	}
	info->bus_id = rmsg->bus_id;
	info->addr = rmsg->addr;

	ret = rpmsg_send(info->rpdev->ept, (void *)rmsg, size);
	if (ret < 0) {
		dev_err(&info->rpdev->dev, "rpmsg_send failed: %d\n", ret);
		return ret;
	}

	ret = wait_for_completion_timeout(&info->cmd_complete,
					msecs_to_jiffies(I2C_RPMSG_TIMEOUT));
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

static int i2c_rpmsg_read(struct i2c_msg *msg, struct i2c_rpmsg_info *info,
						int bus_id, bool is_last)
{
	int ret;
	struct i2c_rpmsg_msg rmsg;

	if (!info->rpdev)
		return -EINVAL;

	if (msg->len > I2C_RPMSG_MAX_BUF_SIZE) {
		dev_err(&info->rpdev->dev,
		"%s failed: data length greater than %d, len=%d\n",
		__func__, I2C_RPMSG_MAX_BUF_SIZE, msg->len);
		return -EINVAL;
	}

	memset(&rmsg, 0, sizeof(struct i2c_rpmsg_msg));
	rmsg.header.cmd = I2C_RPMSG_COMMAND_READ;
	rmsg.bus_id = bus_id;
	rmsg.addr = msg->addr;
	if (is_last)
		rmsg.flags = msg->flags | I2C_RPMSG_M_STOP;
	else
		rmsg.flags = msg->flags;
	rmsg.len = msg->len;

	info->buf = msg->buf;
	info->len = msg->len;

	ret = rpmsg_xfer(&rmsg, info);
	info->buf = NULL;
	if (ret)
		return ret;

	if ((info->len != msg->len)) {
		dev_err(&info->rpdev->dev, "%s failed: %d\n", __func__, -EPROTO);
		return -EPROTO;
	}

	return info->len;
}

int i2c_rpmsg_write(struct i2c_msg *msg, struct i2c_rpmsg_info *info,
						int bus_id, bool is_last)
{
	struct i2c_rpmsg_msg rmsg;

	if (!info || !info->rpdev)
		return -EINVAL;

	if (msg->len > I2C_RPMSG_MAX_BUF_SIZE) {
		dev_err(&info->rpdev->dev,
			"%s failed: data length greater than %d, len=%d\n",
			__func__, I2C_RPMSG_MAX_BUF_SIZE, msg->len);
		return -EINVAL;
	}

	memset(&rmsg, 0, sizeof(struct i2c_rpmsg_msg));
	rmsg.header.cmd = I2C_RPMSG_COMMAND_WRITE;
	rmsg.bus_id = bus_id;
	rmsg.addr = msg->addr;
	if (is_last)
		rmsg.flags = msg->flags | I2C_RPMSG_M_STOP;
	else
		rmsg.flags = msg->flags;
	rmsg.len = msg->len;

	memcpy(rmsg.buf, msg->buf, msg->len);

	return rpmsg_xfer(&rmsg, info);
}

static int i2c_rpmsg_probe(struct rpmsg_device *rpdev)
{
	int ret = 0;

	if (!rpdev) {
		dev_info(&rpdev->dev, "%s failed, rpdev=NULL\n", __func__);
		return -EINVAL;
	}

	mutex_init(&i2c_rpmsg.lock);
	init_completion(&i2c_rpmsg.cmd_complete);
	ida_init(&i2c_rpmsg.ida);

	dev_info(&rpdev->dev, "new channel: 0x%x -> 0x%x!\n",
		 rpdev->src, rpdev->dst);

	i2c_rpmsg.rpdev = rpdev;

	return ret;
}

static void i2c_rpmsg_remove(struct rpmsg_device *rpdev)
{
	i2c_rpmsg.rpdev = NULL;
	dev_info(&rpdev->dev, "i2c rpmsg driver is removed\n");
	ida_destroy(&i2c_rpmsg.ida);
}

static struct rpmsg_device_id i2c_rpmsg_id_table[] = {
	{ .name	= "rpmsg-i2c-channel" },
	{ },
};

static struct rpmsg_driver i2c_rpmsg_driver = {
	.drv.name	= "i2c-rpmsg",
	.drv.owner	= THIS_MODULE,
	.id_table	= i2c_rpmsg_id_table,
	.probe		= i2c_rpmsg_probe,
	.remove		= i2c_rpmsg_remove,
	.callback	= i2c_rpmsg_cb,
};


static int i2c_rpbus_xfer(struct i2c_adapter *adapter,
			  struct i2c_msg *msgs, int num)
{
	struct imx_rpmsg_i2c_data *rdata =
		container_of(adapter, struct imx_rpmsg_i2c_data, adapter);
	struct i2c_msg *pmsg;
	int i, ret;
	bool is_last = false;

	mutex_lock(&i2c_rpmsg.lock);

	for (i = 0; i < num; i++) {
		if (i == num - 1)
			is_last = true;

		pmsg = &msgs[i];

		if (pmsg->flags & I2C_M_RD) {
			ret = i2c_rpmsg_read(pmsg, &i2c_rpmsg,
						rdata->adapter.nr, is_last);
			if (ret < 0) {
				mutex_unlock(&i2c_rpmsg.lock);
				return ret;
			}

			pmsg->len = ret;
		} else {
			ret = i2c_rpmsg_write(pmsg, &i2c_rpmsg,
						rdata->adapter.nr, is_last);
			if (ret < 0) {
				mutex_unlock(&i2c_rpmsg.lock);
				return ret;
			}
		}
	}

	mutex_unlock(&i2c_rpmsg.lock);
	return num;
}

static u32 i2c_rpbus_func(struct i2c_adapter *a)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL
		| I2C_FUNC_SMBUS_READ_BLOCK_DATA;
}

static const struct i2c_algorithm i2c_rpbus_algorithm = {
	.master_xfer = i2c_rpbus_xfer,
	.functionality = i2c_rpbus_func,
};

static const struct i2c_adapter_quirks i2c_rpbus_quirks = {
	.max_write_len = I2C_RPMSG_MAX_BUF_SIZE,
	.max_read_len = I2C_RPMSG_MAX_BUF_SIZE,
};

static int i2c_rpbus_init_remote(struct device *dev, int bus_id)
{
	struct i2c_rpmsg_msg rmsg = { 0 };
	/* need to build locally and memcpy for alignment */
	struct i2c_rpmsg_init_payload init = { 0 };
	struct device_node *np = dev->of_node;
	int ret;

	rmsg.header.cmd = I2C_RPMSG_COMMAND_INIT;
	rmsg.bus_id = bus_id;
	rmsg.len = sizeof(struct i2c_rpmsg_init_payload);

	ret = of_property_read_u32(np, "i2c_type", &init.i2c_type);
	if (ret) {
		dev_err(dev, "%pOF: error reading i2c_type: %d\n", np, ret);
		return ret;
	}

	ret = of_property_read_u32(np, "i2c_index", &init.i2c_index);
	if (ret) {
		dev_err(dev, "%pOF: error reading i2c_index: %d\n", np, ret);
		return ret;
	}

	ret = of_property_read_u32(np, "i2c_baudrate", &init.i2c_baudrate);
	if (ret) {
		dev_err(dev, "%pOF: error reading i2c_baudrate: %d\n", np, ret);
		return ret;
	}

	switch (init.i2c_type) {
	case I2C_TYPE_LPI2C:
		break;
	case I2C_TYPE_FLEXIO:
		ret = of_property_read_u32(np, "i2c_SCL_pin", &init.flexio.scl_pin);
		if (ret) {
			dev_err(dev, "%pOF: error reading i2c_SCL_pin: %d\n", np, ret);
			return ret;
		}

		ret = of_property_read_u32(np, "i2c_SDA_pin", &init.flexio.sda_pin);
		if (ret) {
			dev_err(dev, "%pOF: error reading i2c_SDA_pin: %d\n", np, ret);
			return ret;
		}
		break;
	default:
		dev_err(dev, "%pOF: invalid i2c_type %d\n", np, init.i2c_type);
		return -EINVAL;
	}

	memcpy(&rmsg.init, &init, sizeof(init));
	mutex_lock(&i2c_rpmsg.lock);
	ret = rpmsg_xfer(&rmsg, &i2c_rpmsg);
	mutex_unlock(&i2c_rpmsg.lock);
	if (ret)
		dev_err(dev, "%pOF: register failed, check parameters: %d\n", np, ret);

	return ret;
}

static int i2c_rpbus_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct imx_rpmsg_i2c_data *rdata;
	struct i2c_adapter *adapter;
	int ret, of_id, id = -1;

	if (!i2c_rpmsg.rpdev)
		return -EPROBE_DEFER;

	rdata = devm_kzalloc(&pdev->dev, sizeof(*rdata), GFP_KERNEL);
	if (!rdata)
		return -ENOMEM;

	adapter = &rdata->adapter;
	/* setup i2c adapter description */
	adapter->owner = THIS_MODULE;
	adapter->class = I2C_CLASS_HWMON;
	adapter->algo = &i2c_rpbus_algorithm;
	adapter->dev.parent = dev;
	adapter->dev.of_node = np;
	of_id = of_alias_get_id(np, "i2c");
	if (of_id >= 0)
		id = ida_simple_get(&i2c_rpmsg.ida, of_id, of_id + 1, GFP_KERNEL);
	if (id < 0)
		id = ida_simple_get(&i2c_rpmsg.ida, 0, 0, GFP_KERNEL);
	if (of_id >= 0 && id != of_id)
		dev_warn(&pdev->dev, "Alias ID %d not available - got %d\n", of_id, id);
	adapter->nr = id;

	adapter->quirks = &i2c_rpbus_quirks;
	snprintf(rdata->adapter.name, sizeof(rdata->adapter.name), "%s",
							"i2c-rpmsg-adapter");
	platform_set_drvdata(pdev, rdata);

	ret = i2c_rpbus_init_remote(&pdev->dev, id);
	if (ret)
		goto error;


	ret = i2c_add_adapter(&rdata->adapter);
	if (ret < 0) {
		dev_err(dev, "failed to add I2C adapter: %d\n", ret);
		goto error;
	}

	dev_info(dev, "add I2C adapter %s successfully\n", rdata->adapter.name);

	return 0;

error:
	ida_free(&i2c_rpmsg.ida, id);
	return ret;
}

static int i2c_rpbus_remove(struct platform_device *pdev)
{
	struct imx_rpmsg_i2c_data *rdata = platform_get_drvdata(pdev);

	i2c_del_adapter(&rdata->adapter);

	return 0;
}

static const struct of_device_id imx_rpmsg_i2c_dt_ids[] = {
	{ .compatible = "fsl,i2c-rpbus", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_rpmsg_i2c_dt_ids);

static struct platform_driver imx_rpmsg_i2c_driver = {
	.driver = {
		.name	= "imx_rpmsg_i2c",
		.of_match_table = imx_rpmsg_i2c_dt_ids,
	},
	.probe		= i2c_rpbus_probe,
	.remove		= i2c_rpbus_remove
};

static int __init imx_rpmsg_i2c_driver_init(void)
{
	int ret = 0;

	ret = register_rpmsg_driver(&i2c_rpmsg_driver);
	if (ret < 0)
		return ret;

	return platform_driver_register(&(imx_rpmsg_i2c_driver));
}
subsys_initcall(imx_rpmsg_i2c_driver_init);

MODULE_AUTHOR("Clark Wang<xiaoning.wang@nxp.com>");
MODULE_DESCRIPTION("Driver for i2c over rpmsg");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:i2c-rpbus");
