// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2024 Atmark Techno, Inc. All Rights Reserved.
 *
 * Based on: drivers/rpmsg/imx_rpmsg_tty.c, Copyright 2019 NXP
 */

#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/imx_rpmsg.h>
#include <linux/rpmsg.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/virtio.h>

#define RPMSG_TIMEOUT   1000
#define TTY_RPMSG_MINOR 0
#define TTY_RPMSG_MAJOR 2

enum tty_rpmsg_header_type {
	TTY_RPMSG_REQUEST,
	TTY_RPMSG_RESPONSE,
	TTY_RPMSG_NOTIFICATION,
};

enum tty_rpmsg_header_cmd {
	TTY_RPMSG_COMMAND_PAYLOAD,
	TTY_RPMSG_COMMAND_SET_BAUD,
	TTY_RPMSG_COMMAND_NOTIFY,
	TTY_RPMSG_COMMAND_SET_WAKE,
};

/*
 * struct imx_rpmsg_port - Wrapper struct for imx rpmsg tty port.
 * @port:		TTY port data
 */
struct imx_rpmsg_port {
	struct tty_port		port;
	spinlock_t		rx_lock;
	struct rpmsg_device	*rpdev;
	struct tty_driver	*driver;

	struct mutex		tx_lock;
	struct completion	cmd_complete;
	u8			last_retcode;
	u8			last_request_id;
	u16			inflight_request_id;
	spinlock_t		request_id_lock;
};

/* imx_rpmsg_tty_msg needs to fit within RPMSG_MAX_PAYLOAD_SIZE */
#define RPMSG_MAX_SIZE	(RPMSG_MAX_PAYLOAD_SIZE - (IMX_RPMSG_HEAD_SIZE + 3))
struct imx_rpmsg_tty_msg {
	struct imx_rpmsg_head header;
	u8 request_id;
	u16 len;
	union {
		u8 buf[RPMSG_MAX_SIZE];
		//uint32_t baudrate; // not actually used to avoid unaligned access.. & fails build_bug_on
		u8 retcode;
	};
} __packed __aligned(1);

#define imx_rpmsg_uart_msg_size(s) (sizeof(struct imx_rpmsg_tty_msg) - \
				    RPMSG_MAX_SIZE + s)

static int imx_rpmsg_uart_cb(struct rpmsg_device *rpdev, void *data, int len,
			     void *priv, u32 src)
{
	struct imx_rpmsg_tty_msg *msg = data;
	struct imx_rpmsg_port *port = dev_get_drvdata(&rpdev->dev);
	unsigned char *buf;
	int space;

	dev_dbg(&rpdev->dev, "msg(<- src 0x%x) len %d\n", src, len);
	print_hex_dump_debug(__func__, DUMP_PREFIX_NONE, 16, 1,
			     data, len,  true);

	if (msg->header.type == TTY_RPMSG_RESPONSE) {
		// ack for baudrate or tx
		spin_lock_irq(&port->request_id_lock);
		if (msg->request_id != port->inflight_request_id) {
			dev_warn(&rpdev->dev, "received unexpected id %x (expected %x)\n",
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
	/* XXX check length, version... */

	spin_lock_bh(&port->rx_lock);
	space = tty_prepare_flip_string(&port->port, &buf, msg->len);
	if (space <= 0) {
		dev_err(&rpdev->dev, "No memory for tty_prepare_flip_string\n");
		spin_unlock_bh(&port->rx_lock);
		return -ENOMEM;
	}

	memcpy(buf, msg->buf, msg->len);
	tty_flip_buffer_push(&port->port);
	spin_unlock_bh(&port->rx_lock);

	return 0;
}

static int tty_send_and_wait(struct imx_rpmsg_port *port, struct imx_rpmsg_tty_msg *msg,
			     const void *data, size_t len)
{
	int err;

	BUILD_BUG_ON(sizeof(struct imx_rpmsg_tty_msg) > RPMSG_MAX_PAYLOAD_SIZE);
	BUG_ON(len > RPMSG_MAX_SIZE);

	msg->header.cate = IMX_RPMSG_TTY;
	msg->header.major = TTY_RPMSG_MAJOR;
	msg->header.minor = TTY_RPMSG_MINOR;
	msg->header.type = TTY_RPMSG_REQUEST;
	msg->len = len;
	memcpy(msg->buf, data, len);

	reinit_completion(&port->cmd_complete);
	mutex_lock(&port->tx_lock);
	spin_lock_irq(&port->request_id_lock);
	port->inflight_request_id = port->last_request_id++;
	spin_unlock_irq(&port->request_id_lock);
	msg->request_id = port->inflight_request_id;

	err = rpmsg_send(port->rpdev->ept, msg,
				 imx_rpmsg_uart_msg_size(len));
	if (err) {
		dev_err(&port->rpdev->dev, "rpmsg_send failed: %d\n", err);
		goto err_out;
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
		ret = tty_send_and_wait(port, &msg, tbuf, tlen);
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

static void imx_rpmsg_uart_set_termios(struct tty_struct *tty,
				       struct ktermios *old)
{
	struct imx_rpmsg_tty_msg msg = {
		.header.cmd = TTY_RPMSG_COMMAND_SET_BAUD,
	};
	struct imx_rpmsg_port *port = container_of(tty->port,
						   struct imx_rpmsg_port, port);

	if (C_BAUD(tty) != (old->c_cflag & CBAUD)) {
		u32 baud = tty_get_baud_rate(tty);

		tty_send_and_wait(port, (void *)&msg, &baud, sizeof(baud));
	}
}

static const struct tty_operations imx_rpmsg_uart_ops = {
	.install		= imx_rpmsg_uart_install,
	.open			= imx_rpmsg_uart_open,
	.close			= imx_rpmsg_uart_close,
	.write			= imx_rpmsg_uart_write,
	.write_room		= imx_rpmsg_uart_write_room,
	.set_termios		= imx_rpmsg_uart_set_termios,
};

static int imx_rpmsg_uart_probe(struct rpmsg_device *rpdev)
{
	int ret;
	struct imx_rpmsg_port *port;
	struct tty_driver *driver;

	dev_info(&rpdev->dev, "new channel: 0x%x -> 0x%x!\n",
		 rpdev->src, rpdev->dst);

	port = devm_kzalloc(&rpdev->dev, sizeof(*port), GFP_KERNEL);
	if (!port)
		return -ENOMEM;

	driver = tty_alloc_driver(1, TTY_DRIVER_UNNUMBERED_NODE);
	if (IS_ERR(driver))
		return PTR_ERR(driver);

	driver->driver_name = "imx_rpmsg";
	// XXX add ida or alias if we support more than 1 in the future
	driver->name = kasprintf(GFP_KERNEL, "ttyrpmsg0");
	driver->major = UNNAMED_MAJOR;
	driver->minor_start = 0;
	driver->type = TTY_DRIVER_TYPE_CONSOLE;
	driver->init_termios = tty_std_termios;

	tty_set_operations(driver, &imx_rpmsg_uart_ops);

	tty_port_init(&port->port);
	port->port.ops = &imx_rpmsg_port_ops;
	port->port.low_latency = port->port.flags | ASYNC_LOW_LATENCY;
	port->rpdev = rpdev;
	dev_set_drvdata(&rpdev->dev, port);
	driver->driver_state = port;
	port->driver = driver;
	spin_lock_init(&port->rx_lock);
	init_completion(&port->cmd_complete);
	mutex_init(&port->tx_lock);
	spin_lock_init(&port->request_id_lock);

	ret = tty_register_driver(port->driver);
	if (ret < 0) {
		dev_err(&rpdev->dev,
			"Couldn't install rpmsg tty driver: ret %d\n", ret);
		goto error;
	} else {
		dev_info(&rpdev->dev, "Install rpmsg tty driver!\n");
	}

	device_set_wakeup_capable(&rpdev->dev, true);

	return 0;

error:
	put_tty_driver(port->driver);
	tty_port_destroy(&port->port);
	port->driver = NULL;

	return ret;
}

static void imx_rpmsg_uart_remove(struct rpmsg_device *rpdev)
{
	struct imx_rpmsg_port *port = dev_get_drvdata(&rpdev->dev);

	dev_info(&rpdev->dev, "rpmsg tty driver is removed\n");

	tty_unregister_driver(port->driver);
	kfree(port->driver->name);
	put_tty_driver(port->driver);
	tty_port_destroy(&port->port);
	port->driver = NULL;
}

static int __maybe_unused imx_rpmsg_uart_suspend(struct device *dev)
{
	struct imx_rpmsg_tty_msg msg = {
		.header.cmd = TTY_RPMSG_COMMAND_SET_WAKE,
	};
	struct imx_rpmsg_port *port = dev_get_drvdata(dev);
	bool enable = device_may_wakeup(dev);

	return tty_send_and_wait(port, (void *)&msg, &enable, sizeof(enable));
}

static int __maybe_unused imx_rpmsg_uart_resume(struct device *dev)
{
	/* Do nothing */
	return 0;
}

static SIMPLE_DEV_PM_OPS(imx_rpmsg_uart_pm_ops, imx_rpmsg_uart_suspend,
			 imx_rpmsg_uart_resume);

static struct rpmsg_device_id tty_rpmsg_id_table[] = {
	{ .name = "rpmsg-tty-channel" },
	{ },
};

static struct rpmsg_driver imx_rpmsg_uart_driver = {
	.drv.name	= KBUILD_MODNAME,
	.drv.pm		= &imx_rpmsg_uart_pm_ops,
	.drv.owner	= THIS_MODULE,
	.id_table	= tty_rpmsg_id_table,
	.probe		= imx_rpmsg_uart_probe,
	.callback	= imx_rpmsg_uart_cb,
	.remove		= imx_rpmsg_uart_remove,
};

static int __init imx_rpmsg_uart_init(void)
{
	return register_rpmsg_driver(&imx_rpmsg_uart_driver);
}

static void __exit imx_rpmsg_uart_fini(void)
{
	unregister_rpmsg_driver(&imx_rpmsg_uart_driver);
}
module_init(imx_rpmsg_uart_init);
module_exit(imx_rpmsg_uart_fini);

MODULE_AUTHOR("Atmark Techno, Inc.");
MODULE_DESCRIPTION("i.MX RPMSG tty driver");
MODULE_LICENSE("GPL v2");
