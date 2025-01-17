// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2024 Atmark Techno, Inc. All Rights Reserved.
 *
 * Based on: drivers/rpmsg/imx_rpmsg_tty.c, Copyright 2019 NXP
 */

#include <linux/delay.h>
#include <linux/idr.h>
#include <linux/imx_rpmsg.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/rpmsg.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/virtio.h>

#define RPMSG_TIMEOUT   1000
/* Further improvement: could count compatible nodes with e.g.
 * `for_each_compatible_node(dn, NULL, "xxx") count++;`
 * and allocate dynamically */
#define IMX_RPMSG_UART_PORT_PER_SOC_MAX 4
#define TTY_RPMSG_MINOR 0
#define TTY_RPMSG_MAJOR 3

#define IMX_RPMSG_DEFAULT_BAUD 115200

enum tty_rpmsg_header_type {
	TTY_RPMSG_REQUEST,
	TTY_RPMSG_RESPONSE,
	TTY_RPMSG_NOTIFICATION,
};

enum tty_rpmsg_header_cmd {
	TTY_RPMSG_COMMAND_PAYLOAD,
	TTY_RPMSG_COMMAND_SET_CFLAG,
	TTY_RPMSG_COMMAND_NOTIFY,
	TTY_RPMSG_COMMAND_SET_WAKE,
	TTY_RPMSG_COMMAND_INIT,
};

enum tty_rpmsg_init_type {
	TTY_TYPE_LPUART,
	TTY_TYPE_CUSTOM,
};

struct srtm_tty_init_payload {
	uint32_t port_type;
	union {
		struct {
			uint32_t uart_index;
			uint32_t rs485_flags;
			uint32_t rs485_de_gpio;
			uint32_t suspend_wakeup_gpio;
			uint32_t cflag;
		} lpuart;
		struct {
			char name[32];
		} custom;
	};
};

/*
 * struct imx_rpmsg_port - Wrapper struct for imx rpmsg tty port.
 * @port:		TTY port data
 */
struct imx_rpmsg_port {
	struct tty_port		port;
	spinlock_t		rx_lock;
	struct rpmsg_device	*rpdev;
	struct platform_device  *pdev;
	struct tty_driver	*driver;

	struct mutex		tx_lock;
	struct completion	cmd_complete;
	u8			port_idx;
	u8			last_retcode;
	u8			last_request_id;
	u16			inflight_request_id;
	spinlock_t		request_id_lock;
};

/* imx_rpmsg_tty_msg needs to fit within RPMSG_MAX_PAYLOAD_SIZE */
#define RPMSG_MAX_SIZE	(RPMSG_MAX_PAYLOAD_SIZE - (IMX_RPMSG_HEAD_SIZE + 4))
struct imx_rpmsg_tty_msg {
	struct imx_rpmsg_head header;
	u8 port_idx;
	u8 request_id;
	u16 len;
	union {
		u8 buf[RPMSG_MAX_SIZE];
		u32 cflag;
		/* note: packed only by design, sanity is ensured by checking size */
		struct srtm_tty_init_payload init;
		u8 retcode;
	} __packed __aligned(1);
} __packed __aligned(1);

#define imx_rpmsg_uart_msg_size(s) (sizeof(struct imx_rpmsg_tty_msg) - \
				    RPMSG_MAX_SIZE + s)

struct rpmsg_uart {
	struct rpmsg_device *rpdev;
	struct imx_rpmsg_port *ports[IMX_RPMSG_UART_PORT_PER_SOC_MAX];
	struct ida ida;
};

/* We need these to be global because rpmsg_probe has no way to pass
 * values to the platform_probe */
static struct rpmsg_uart uart_rpmsg;

static int imx_rpmsg_uart_cb(struct rpmsg_device *rpdev, void *data, int len,
			     void *priv, u32 src)
{
	struct imx_rpmsg_tty_msg *msg = data;
	struct imx_rpmsg_port *port;
	unsigned char *buf;
	int space;

	dev_dbg(&rpdev->dev, "msg(<- src 0x%x) len %d\n", src, len);
	print_hex_dump_debug(__func__, DUMP_PREFIX_NONE, 16, 1,
			     data, len,  true);

	if (msg->header.major != TTY_RPMSG_MAJOR) {
		dev_err(&rpdev->dev, "invalid version\n");
		return 0;
	}
	if (msg->port_idx >= IMX_RPMSG_UART_PORT_PER_SOC_MAX
	    || !uart_rpmsg.ports[msg->port_idx]) {
		dev_err(&rpdev->dev, "port_idx %d too large\n", msg->port_idx);
		return 0;
	}
	port = uart_rpmsg.ports[msg->port_idx];

	if (msg->header.type == TTY_RPMSG_RESPONSE) {
		/* 1 for ret */
		if (len != imx_rpmsg_uart_msg_size(1)) {
			dev_err(&rpdev->dev, "got response size %d, expected %zd\n",
				len, imx_rpmsg_uart_msg_size(1));
			return 0;
		}
		/* ack for baudrate or tx */
		spin_lock_irq(&port->request_id_lock);
		if (msg->request_id != port->inflight_request_id) {
			/* non-wait message are always 0xffff */
			if (port->inflight_request_id != 0xffff)
				dev_warn(&rpdev->dev,
					 "received unexpected id %x (expected %x)\n",
					 msg->request_id, port->inflight_request_id);
			spin_unlock_irq(&port->request_id_lock);
			return 0;
		}
		/* don't process duplicates */
		port->inflight_request_id = 0xffff;
		port->last_retcode = msg->retcode;
		spin_unlock_irq(&port->request_id_lock);

		complete(&port->cmd_complete);
		return 0;
	}
	if (msg->header.type != TTY_RPMSG_NOTIFICATION) {
		/* invalid message */
		return 0;
	}
	if (len < imx_rpmsg_uart_msg_size(msg->len)) {
		dev_err(&rpdev->dev, "Short message, expected at least %zd, got %d\n",
			imx_rpmsg_uart_msg_size(msg->len), len);
		return 0;
	}

	spin_lock_bh(&port->rx_lock);
	space = tty_prepare_flip_string(&port->port, &buf, msg->len);
	if (space <= 0) {
		dev_err(&rpdev->dev, "No memory for tty_prepare_flip_string\n");
		spin_unlock_bh(&port->rx_lock);
		return 0;
	}

	memcpy(buf, msg->buf, msg->len);
	tty_flip_buffer_push(&port->port);
	spin_unlock_bh(&port->rx_lock);

	return 0;
}

static int tty_send_and_wait(struct imx_rpmsg_port *port, struct imx_rpmsg_tty_msg *msg,
			     const void *data, size_t len, bool wait)
{
	int err;

	BUILD_BUG_ON(sizeof(struct imx_rpmsg_tty_msg) > RPMSG_MAX_PAYLOAD_SIZE);
	BUG_ON(len > RPMSG_MAX_SIZE);

	msg->header.cate = IMX_RPMSG_TTY;
	msg->header.major = TTY_RPMSG_MAJOR;
	msg->header.minor = TTY_RPMSG_MINOR;
	msg->header.type = TTY_RPMSG_REQUEST;
	msg->len = len;
	msg->port_idx = port->port_idx;
	memcpy(msg->buf, data, len);

	reinit_completion(&port->cmd_complete);
	mutex_lock(&port->tx_lock);
	spin_lock_irq(&port->request_id_lock);
	port->inflight_request_id = port->last_request_id++;
	spin_unlock_irq(&port->request_id_lock);
	msg->request_id = port->inflight_request_id;
	if (!wait)
		port->inflight_request_id = 0xffff;

	err = rpmsg_send(port->rpdev->ept, msg,
				 imx_rpmsg_uart_msg_size(len));
	if (err) {
		dev_err(&port->rpdev->dev, "rpmsg_send failed: %d\n", err);
		goto err_out;
	}
	if (!wait) {
		mutex_unlock(&port->tx_lock);
		return 0;
	}
	err = wait_for_completion_timeout(&port->cmd_complete, msecs_to_jiffies(RPMSG_TIMEOUT));
	if (!err) {
		/* don't process late replies - lock here ensures
		 * we're not completing the next call */
		spin_lock_irq(&port->request_id_lock);
		port->inflight_request_id = 0xffff;
		spin_unlock_irq(&port->request_id_lock);
		dev_err(&port->rpdev->dev, "rpmsg_send timeout\n");
		err = -ETIMEDOUT;
		goto err_out;
	}
	if (port->last_retcode != 0) {
		dev_err(&port->rpdev->dev, "rpmsg error for %d: %d\n",
			msg->header.cmd, port->last_retcode);
		err = -EINVAL;
		goto err_out;
	}
	err = 0;

err_out:
	mutex_unlock(&port->tx_lock);

	return err;
}

static struct tty_port_operations  imx_rpmsg_port_ops = { };

static int imx_rpmsg_uart_install(struct tty_driver *driver,
				  struct tty_struct *tty)
{
	struct imx_rpmsg_port *port = driver->driver_state;

	return tty_port_install(&port->port, driver, tty);
}

static int imx_rpmsg_uart_open(struct tty_struct *tty, struct file *filp)
{
	return tty_port_open(tty->port, tty, filp);
}

static void imx_rpmsg_uart_close(struct tty_struct *tty, struct file *filp)
{
	return tty_port_close(tty->port, tty, filp);
}

static int imx_rpmsg_uart_write(struct tty_struct *tty,
				const unsigned char *buf, int total)
{
	struct imx_rpmsg_tty_msg msg = {
		.header.cmd = TTY_RPMSG_COMMAND_PAYLOAD,
	};
	int remain, ret = 0;
	const unsigned char *tbuf;
	int tlen;
	struct imx_rpmsg_port *port = container_of(tty->port,
						   struct imx_rpmsg_port, port);
	struct rpmsg_device *rpdev = port->rpdev;

	if (buf == NULL) {
		dev_err(&rpdev->dev, "buf shouldn't be null.\n");
		return -ENOMEM;
	}

	remain = total;
	tbuf = buf;
	do {
		tlen = remain > RPMSG_MAX_SIZE ? RPMSG_MAX_SIZE : remain;

		/* send a message to our remote processor */
		ret = tty_send_and_wait(port, &msg, tbuf, tlen, true);
		if (ret)
			return ret;

		if (remain > RPMSG_MAX_SIZE) {
			remain -= RPMSG_MAX_SIZE;
			tbuf += RPMSG_MAX_SIZE;
		} else {
			remain = 0;
		}
	} while (remain > 0);

	return total;
}

static int imx_rpmsg_uart_write_room(struct tty_struct *tty)
{
	/* report the space in the rpmsg buffer */
	return RPMSG_MAX_SIZE;
}

static int imx_rpmsg_uart_set_cflag(struct imx_rpmsg_port *port,
				    tcflag_t cflag)
{
	struct imx_rpmsg_tty_msg msg = {
		.header.cmd = TTY_RPMSG_COMMAND_SET_CFLAG,
	};

	return tty_send_and_wait(port, (void *)&msg, &cflag, sizeof(cflag), true);
}

static void imx_rpmsg_uart_set_termios(struct tty_struct *tty,
				       struct ktermios *old)
{
	struct imx_rpmsg_port *port = container_of(tty->port,
						   struct imx_rpmsg_port, port);
	unsigned int old_csize = old ? old->c_cflag & CSIZE : CS8;
	tcflag_t cflag = tty->termios.c_cflag;

	if (cflag == old->c_cflag)
		return;

	/*
	 * only support CS8 and CS7, and for CS7 must enable PE.
	 * supported mode:
	 *  - (7,e/o,1/2)
	 *  - (8,n,1/2)
	 *  - (8,m/s,1/2)
	 *  - (8,e/o,1/2)
	 */
	while ((cflag & CSIZE) != CS8 && (cflag & CSIZE) != CS7) {
		cflag &= ~CSIZE;
		cflag |= old_csize;
		old_csize = CS8;
	}

	if (cflag & CMSPAR) {
		if ((cflag & CSIZE) != CS8) {
			cflag &= ~CSIZE;
			cflag |= CS8;
		}
	}

	/* parity must be enabled when CS7 to match 8-bits format */
	if ((cflag & CSIZE) == CS7)
		cflag |= PARENB;

	imx_rpmsg_uart_set_cflag(port, cflag);
}

static const struct tty_operations imx_rpmsg_uart_ops = {
	.install		= imx_rpmsg_uart_install,
	.open			= imx_rpmsg_uart_open,
	.close			= imx_rpmsg_uart_close,
	.write			= imx_rpmsg_uart_write,
	.write_room		= imx_rpmsg_uart_write_room,
	.set_termios		= imx_rpmsg_uart_set_termios,
};

#define READ_PROP_OR_RETURN(name) \
		ret = of_property_read_u32(np, #name, &init.lpuart. name); \
		if (ret) { \
			dev_err(dev, "%pOF: error reading %s: %d\n", \
				np, #name, ret); \
			return ret; \
		}
static int imx_rpmsg_uart_init_remote(struct imx_rpmsg_port *port,
				      tcflag_t cflag)
{
	struct imx_rpmsg_tty_msg msg = {
		.header.cmd = TTY_RPMSG_COMMAND_INIT,
	};
	struct device *dev = &port->pdev->dev;
	struct device_node *np = dev->of_node;
	struct srtm_tty_init_payload init = { 0 };
	int ret;
	const char *str;

	ret = of_property_read_u32(np, "port_type", &init.port_type);
	if (ret) {
		dev_err(dev, "%pOF: error reading port_type: %d\n", np, ret);
		return ret;
	}

	switch (init.port_type) {
	case TTY_TYPE_LPUART:
		READ_PROP_OR_RETURN(uart_index);
		ret = of_property_read_u32(np, "rs485_flags",
					   &init.lpuart.rs485_flags);
		if (ret == 0) {
			/* not an error, rs485 disabled if missing */
			READ_PROP_OR_RETURN(rs485_de_gpio);
		}
		ret = of_property_read_u32(np, "suspend_wakeup_gpio",
					   &init.lpuart.suspend_wakeup_gpio);
		if (ret < 0) {
			dev_info(dev, "%pOF: no suspend wakeup gpio set, will not wake up\n",
					np);
			/* setting wakeup will error out m33-side and fail suspend */
			init.lpuart.suspend_wakeup_gpio = -1;
		}
		init.lpuart.cflag = cflag;
		break;
	case TTY_TYPE_CUSTOM:
		ret = of_property_read_string(np, "port_name", &str);
		if (ret) {
			dev_err(dev, "%pOF: error reading port_name: %d\n",
				np, ret);
			return ret;
		}
		ret = strscpy(init.custom.name, str, sizeof(init.custom.name));
		if (ret < 0) {
			dev_err(dev, "%pOF: port_name too long\n", np);
			return ret;
		}
		break;
	default:
		dev_err(dev, "%pOF: invalid port_type %d\n", np, init.port_type);
		return -EINVAL;
	}

	return tty_send_and_wait(port, (void *)&msg, &init, sizeof(init), true);
}
#undef READ_PROP_OR_RETURN

static int imx_rpmsg_uart_platform_probe(struct platform_device *pdev)
{
	int ret, of_id, id = -1;
	struct imx_rpmsg_port *port;
	struct tty_driver *driver;
	struct rpmsg_device *rpdev = uart_rpmsg.rpdev;

	/* defer probing until we can process rpmsg replies */
	if (!rpdev)
		return -EPROBE_DEFER;

	port = devm_kzalloc(&pdev->dev, sizeof(*port), GFP_KERNEL);
	if (!port)
		return -ENOMEM;

	driver = tty_alloc_driver(1, TTY_DRIVER_UNNUMBERED_NODE);
	if (IS_ERR(driver))
		return PTR_ERR(driver);

	of_id = of_alias_get_id(pdev->dev.of_node, "ttyrpmsg");
	if (of_id >= 0) {
		id = ida_simple_get(&uart_rpmsg.ida, of_id, of_id + 1, GFP_KERNEL);
		if (id < 0)
			dev_warn(&pdev->dev, "Alias ID %d not available\n", of_id);
	}
	if (id < 0)
		id = ida_simple_get(&uart_rpmsg.ida, 0, 0, GFP_KERNEL);
	if (id >= IMX_RPMSG_UART_PORT_PER_SOC_MAX) {
		dev_err(&pdev->dev, "id %d higher than max %d\n",
			id, IMX_RPMSG_UART_PORT_PER_SOC_MAX);
		ret = -ERANGE;
		goto error;
	}

	driver->driver_name = "imx_rpmsg";
	driver->name = kasprintf(GFP_KERNEL, "ttyrpmsg%d", id);
	driver->major = UNNAMED_MAJOR;
	driver->minor_start = 0;
	driver->type = TTY_DRIVER_TYPE_CONSOLE;
	driver->init_termios = tty_std_termios;
	tty_termios_encode_baud_rate(&driver->init_termios,
				     IMX_RPMSG_DEFAULT_BAUD,
				     IMX_RPMSG_DEFAULT_BAUD);

	tty_set_operations(driver, &imx_rpmsg_uart_ops);

	tty_port_init(&port->port);
	port->port.ops = &imx_rpmsg_port_ops;
	port->port.low_latency = port->port.flags | ASYNC_LOW_LATENCY;
	port->rpdev = rpdev;
	port->pdev = pdev;
	port->port_idx = id;
	driver->driver_state = port;
	port->driver = driver;
	spin_lock_init(&port->rx_lock);
	init_completion(&port->cmd_complete);
	mutex_init(&port->tx_lock);
	spin_lock_init(&port->request_id_lock);

	platform_set_drvdata(pdev, port);
	uart_rpmsg.ports[id] = port;

	/*
	 * set the baud rate to our remote processor's UART, and tell
	 * remote processor about this channel
	 */
	ret = imx_rpmsg_uart_init_remote(port, driver->init_termios.c_cflag);
	if (ret)
		goto error_port;

	ret = tty_register_driver(port->driver);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"Couldn't install rpmsg tty driver: ret %d\n", ret);
		goto error_port;
	} else {
		dev_info(&pdev->dev, "Install rpmsg tty driver!\n");
	}

	device_set_wakeup_capable(&pdev->dev, true);


	return 0;

error_port:
	tty_port_destroy(&port->port);
error:
	put_tty_driver(driver);
	port->driver = NULL;
	ida_free(&uart_rpmsg.ida, id);

	return ret;
}

static int imx_rpmsg_uart_platform_remove(struct platform_device *pdev)
{
	struct imx_rpmsg_port *port = platform_get_drvdata(pdev);

	dev_info(&pdev->dev, "rpmsg tty driver is removed\n");

	tty_unregister_driver(port->driver);
	kfree(port->driver->name);
	put_tty_driver(port->driver);
	tty_port_destroy(&port->port);
	port->driver = NULL;
	return 0;
}

static int __maybe_unused imx_rpmsg_uart_suspend(struct device *dev)
{
	struct imx_rpmsg_tty_msg msg = {
		.header.cmd = TTY_RPMSG_COMMAND_SET_WAKE,
	};
	struct imx_rpmsg_port *port = dev_get_drvdata(dev);
	bool enable = device_may_wakeup(dev);

	return tty_send_and_wait(port, (void *)&msg, &enable, sizeof(enable), true);
}

static int __maybe_unused imx_rpmsg_uart_resume(struct device *dev)
{
	/* Do nothing */
	return 0;
}

static SIMPLE_DEV_PM_OPS(imx_rpmsg_uart_pm_ops, imx_rpmsg_uart_suspend,
			 imx_rpmsg_uart_resume);

static const struct of_device_id imx_rpmsg_uart_dt_ids[] = {
	{ .compatible = "fsl,imx-rpmsg-tty-serial" },
	{ /* sentinel */ }
};

static struct platform_driver imx_rpmsg_uart_platform_driver = {
	.driver = {
		.name = "imx-rpmsg-tty-serial",
		.pm		= &imx_rpmsg_uart_pm_ops,
		.of_match_table = imx_rpmsg_uart_dt_ids,
	},
	.probe = imx_rpmsg_uart_platform_probe,
	.remove = imx_rpmsg_uart_platform_remove,
};

static int imx_rpmsg_uart_rpmsg_probe(struct rpmsg_device *rpdev)
{
	int rc;

	dev_info(&rpdev->dev, "new channel: 0x%x -> 0x%x!\n",
			rpdev->src, rpdev->dst);
	ida_init(&uart_rpmsg.ida);

	rc = platform_driver_register(&imx_rpmsg_uart_platform_driver);

	/* platform_driver_register() calls the platform probe immediately,
	 * but we need it to process rpmsg replies so only set rpdev last
	 * to make the first platform probe defer */
	uart_rpmsg.rpdev = rpdev;
	return rc;
}

static void imx_rpmsg_uart_rpmsg_remove(struct rpmsg_device *rpdev)
{
	dev_info(&rpdev->dev, "uart channel removed\n");
	platform_driver_unregister(&imx_rpmsg_uart_platform_driver);
	uart_rpmsg.rpdev = NULL;
	ida_destroy(&uart_rpmsg.ida);
}

static struct rpmsg_device_id tty_rpmsg_id_table[] = {
	{ .name = "rpmsg-tty-channel" },
	{ },
};

static struct rpmsg_driver imx_rpmsg_uart_rpmsg_driver = {
	.drv.name	= KBUILD_MODNAME,
	.drv.owner	= THIS_MODULE,
	.id_table	= tty_rpmsg_id_table,
	.probe		= imx_rpmsg_uart_rpmsg_probe,
	.callback	= imx_rpmsg_uart_cb,
	.remove		= imx_rpmsg_uart_rpmsg_remove,
};

static int __init imx_rpmsg_uart_init(void)
{
	return register_rpmsg_driver(&imx_rpmsg_uart_rpmsg_driver);
}

static void __exit imx_rpmsg_uart_fini(void)
{
	unregister_rpmsg_driver(&imx_rpmsg_uart_rpmsg_driver);
}
module_init(imx_rpmsg_uart_init);
module_exit(imx_rpmsg_uart_fini);

MODULE_AUTHOR("Atmark Techno, Inc.");
MODULE_DESCRIPTION("i.MX RPMSG tty driver");
MODULE_LICENSE("GPL v2");
