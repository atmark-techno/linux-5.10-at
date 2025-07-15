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
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/virtio.h>

#include "serial_mctrl_gpio.h"

#define DRIVER_NAME	"IMX-rpmsg-uart"
#define DEV_NAME	"ttyrpmsg"

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
	TTY_RPMSG_COMMAND_ACTIVATE,
};

enum tty_rpmsg_init_type {
	TTY_TYPE_LPUART,
	TTY_TYPE_CUSTOM,
	TTY_TYPE_M33_CONSOLE,
	TTY_TYPE_FLEXIO,
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
		// nothing for m33 console
		struct {
			uint32_t flexio_index;
			uint32_t flexio_rx;
			uint32_t flexio_tx;
			uint32_t suspend_wakeup_gpio;
			uint32_t cflag;
		} flexio;
	};
};

/*
 * struct imx_rpmsg_port - Wrapper struct for imx rpmsg tty port.
 * @port:		TTY port data
 */
struct imx_rpmsg_port {
	struct uart_port	port;
	spinlock_t		rx_lock;
	struct rpmsg_device	*rpdev;
	struct platform_device	*pdev;

	struct work_struct	tx_work;
	bool			force_end_work;

	struct mctrl_gpios	*gpios;

	struct mutex		tx_lock;
	struct completion	cmd_complete;
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

#define imx_rpmsg_uart_msg_size(s) (sizeof(struct imx_rpmsg_tty_msg) -	\
				    RPMSG_MAX_SIZE + s)

struct rpmsg_uart {
	struct rpmsg_device *rpdev;
	struct imx_rpmsg_port *rports[IMX_RPMSG_UART_PORT_PER_SOC_MAX];
	struct ida ida;
};

/* We need these to be global because rpmsg_probe has no way to pass
 * values to the platform_probe */
static struct rpmsg_uart uart_rpmsg;

static int imx_rpmsg_uart_cb(struct rpmsg_device *rpdev, void *data, int len,
			     void *priv, u32 src)
{
	struct imx_rpmsg_tty_msg *msg = data;
	struct imx_rpmsg_port *rport;
	struct uart_port *port;
	struct tty_port *tport;
	int copied;

	dev_dbg(&rpdev->dev, "msg(<- src 0x%x) len %d\n", src, len);
	print_hex_dump_debug(__func__, DUMP_PREFIX_NONE, 16, 1,
			     data, len, true);

	if (msg->header.major != TTY_RPMSG_MAJOR) {
		dev_err(&rpdev->dev, "invalid version\n");
		return 0;
	}
	if (msg->port_idx >= IMX_RPMSG_UART_PORT_PER_SOC_MAX
	    || !uart_rpmsg.rports[msg->port_idx]) {
		dev_err(&rpdev->dev, "port_idx %d too large\n", msg->port_idx);
		return 0;
	}
	rport = uart_rpmsg.rports[msg->port_idx];
	port = &rport->port;
	tport = &port->state->port;

	if (msg->header.type == TTY_RPMSG_RESPONSE) {
		/* 1 for ret */
		if (len != imx_rpmsg_uart_msg_size(1)) {
			dev_err(&rpdev->dev, "got response size %d, expected %zd\n",
				len, imx_rpmsg_uart_msg_size(1));
			return 0;
		}
		/* ack for baudrate or tx */
		spin_lock_irq(&rport->request_id_lock);
		if (msg->request_id != rport->inflight_request_id) {
			/* non-wait message are always 0xffff */
			if (rport->inflight_request_id != 0xffff)
				dev_warn(&rpdev->dev,
					 "received unexpected id %x (expected %x)\n",
					 msg->request_id, rport->inflight_request_id);
			spin_unlock_irq(&rport->request_id_lock);
			return 0;
		}
		/* don't process duplicates */
		rport->inflight_request_id = 0xffff;
		rport->last_retcode = msg->retcode;
		spin_unlock_irq(&rport->request_id_lock);

		complete(&rport->cmd_complete);
		return 0;
	}
	if (msg->header.type != TTY_RPMSG_NOTIFICATION) {
		/* invalid message */
		return 0;
	}
	if (len < imx_rpmsg_uart_msg_size(msg->len)) {
		dev_err(&rpdev->dev,
			"Short message, expected at least %zd, got %d\n",
			imx_rpmsg_uart_msg_size(msg->len), len);
		return 0;
	}

	spin_lock_bh(&rport->rx_lock);
	copied = tty_insert_flip_string(tport, msg->buf, msg->len);
	if (copied != msg->len) {
		dev_err(&rpdev->dev, "Only copied %d instead of %d bytes\n",
			copied, msg->len);
	}
	tty_flip_buffer_push(tport);
	port->icount.rx += copied;
	spin_unlock_bh(&rport->rx_lock);

	return 0;
}

static int imx_rpmsg_uart_send_and_wait(struct imx_rpmsg_port *rport,
					struct imx_rpmsg_tty_msg *msg,
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
	msg->port_idx = rport->port.line;
	memcpy(msg->buf, data, len);

	reinit_completion(&rport->cmd_complete);
	mutex_lock(&rport->tx_lock);
	spin_lock_irq(&rport->request_id_lock);
	rport->inflight_request_id = rport->last_request_id++;
	spin_unlock_irq(&rport->request_id_lock);
	msg->request_id = rport->inflight_request_id;
	if (!wait)
		rport->inflight_request_id = 0xffff;

	err = rpmsg_send(rport->rpdev->ept, msg, imx_rpmsg_uart_msg_size(len));
	if (err) {
		dev_err(&rport->rpdev->dev, "rpmsg_send failed: %d\n", err);
		goto err_out;
	}
	if (!wait) {
		mutex_unlock(&rport->tx_lock);
		return 0;
	}
	err = wait_for_completion_timeout(&rport->cmd_complete,
					  msecs_to_jiffies(RPMSG_TIMEOUT));
	if (!err) {
		/* don't process late replies - lock here ensures
		 * we're not completing the next call */
		spin_lock_irq(&rport->request_id_lock);
		rport->inflight_request_id = 0xffff;
		spin_unlock_irq(&rport->request_id_lock);
		dev_err(&rport->rpdev->dev, "rpmsg_send timeout\n");
		err = -ETIMEDOUT;
		goto err_out;
	}
	if (rport->last_retcode != 0) {
		dev_err(&rport->rpdev->dev, "rpmsg error for %d: %d\n",
			msg->header.cmd, rport->last_retcode);
		err = -EINVAL;
		goto err_out;
	}
	err = 0;

err_out:
	mutex_unlock(&rport->tx_lock);

	return err;
}

static void imx_rpmsg_uart_tx_work(struct work_struct *ws)
{
	struct imx_rpmsg_port *rport = container_of(ws, struct imx_rpmsg_port,
						    tx_work);
	struct uart_port *port = &rport->port;
	struct circ_buf *xmit = &port->state->xmit;
	struct imx_rpmsg_tty_msg msg = {
		.header.cmd = TTY_RPMSG_COMMAND_PAYLOAD,
	};
	int len;
	int ret;

	if (port->x_char) {
		/* Send next char */
		ret = imx_rpmsg_uart_send_and_wait(rport, &msg, &port->x_char,
						   1, true);
		if (ret)
			dev_err(&rport->rpdev->dev, "tx dropped XON/XOFF character\n");
		else
			port->icount.tx++;
		port->x_char = 0;
		return;
	}

	while (!uart_circ_empty(xmit) && !uart_tx_stopped(port) &&
	       !rport->force_end_work) {

		if (xmit->tail < xmit->head || xmit->head == 0)
			len = min(uart_circ_chars_pending(xmit), RPMSG_MAX_SIZE);
		else
			len = min(UART_XMIT_SIZE - xmit->tail, RPMSG_MAX_SIZE);

		/* send a message to our remote processor */
		ret = imx_rpmsg_uart_send_and_wait(rport, &msg,
						   &xmit->buf[xmit->tail],
						   len, true);
		if (ret) {
			/* The type of error cannot be determined, so
			 * the error count cannot be incremented. */
			dev_err(&rport->rpdev->dev, "tx dropped at most %d bytes\n",
				len);
		} else {
			/* Add data to send */
			port->icount.tx += len;
		}
		xmit->tail = (xmit->tail + len) & (UART_XMIT_SIZE - 1);
	}

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&rport->port);
}

static unsigned int imx_rpmsg_uart_tx_empty(struct uart_port *port)
{
	return TIOCSER_TEMT;
}

static void imx_rpmsg_uart_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	struct imx_rpmsg_port *rport = (struct imx_rpmsg_port *)port;

	mctrl_gpio_set(rport->gpios, mctrl);
}

static unsigned int imx_rpmsg_uart_get_mctrl(struct uart_port *port)
{
	struct imx_rpmsg_port *rport = (struct imx_rpmsg_port *)port;
	int ret = TIOCM_CTS | TIOCM_DSR | TIOCM_CAR;

	mctrl_gpio_get(rport->gpios, &ret);

	return ret;
}

static void imx_rpmsg_uart_stop_tx(struct uart_port *port)
{
	/* Nothing to do */
}

static void imx_rpmsg_uart_start_tx(struct uart_port *port)
{
	struct imx_rpmsg_port *rport = (struct imx_rpmsg_port *)port;

	schedule_work(&rport->tx_work);
}

static void imx_rpmsg_uart_stop_rx(struct uart_port *port)
{
	/* Nothing to do */
}

static void imx_rpmsg_uart_enable_ms(struct uart_port *port)
{
	struct imx_rpmsg_port *rport = (struct imx_rpmsg_port *)port;

	mctrl_gpio_enable_ms(rport->gpios);
}

static int _imx_rpmsg_uart_send_activate(struct imx_rpmsg_port *rport,
					 bool activate_)
{
	struct imx_rpmsg_tty_msg msg = {
		.header.cmd = TTY_RPMSG_COMMAND_ACTIVATE,
	};
	uint8_t activate = activate_;

	return imx_rpmsg_uart_send_and_wait(rport, (void *)&msg,
					    &activate, sizeof(activate), true);
}

static int imx_rpmsg_uart_startup(struct uart_port *port)
{
	struct imx_rpmsg_port *rport = (struct imx_rpmsg_port *)port;

	rport->force_end_work = false;

	return _imx_rpmsg_uart_send_activate(rport, true);
}

static void imx_rpmsg_uart_shutdown(struct uart_port *port)
{
	struct imx_rpmsg_port *rport = (struct imx_rpmsg_port *)port;

	rport->force_end_work = true;
	cancel_work_sync(&rport->tx_work);

	mctrl_gpio_disable_ms(rport->gpios);

	_imx_rpmsg_uart_send_activate(rport, false);
}

static void
imx_rpmsg_uart_set_termios(struct uart_port *port, struct ktermios *termios,
			   struct ktermios *old)
{
	struct imx_rpmsg_port *rport = (struct imx_rpmsg_port *)port;
	unsigned int old_csize = old ? old->c_cflag & CSIZE : CS8;
	tcflag_t cflag = termios->c_cflag;
	struct imx_rpmsg_tty_msg msg = {
		.header.cmd = TTY_RPMSG_COMMAND_SET_CFLAG,
	};

	if (!old || cflag == old->c_cflag)
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

	imx_rpmsg_uart_send_and_wait(rport, (void *)&msg, &cflag, sizeof(cflag),
				     true);
}

static const char *imx_rpmsg_uart_type(struct uart_port *port)
{
	return DRIVER_NAME;
}

static void imx_rpmsg_uart_config_port(struct uart_port *port, int flags)
{
	struct imx_rpmsg_port *rport = (struct imx_rpmsg_port *)port;

	if (flags & UART_CONFIG_TYPE)
		rport->port.type = PORT_IMX_RPMSG;
}

static const struct uart_ops imx_rpmsg_uart_ops = {
	.tx_empty	= imx_rpmsg_uart_tx_empty,
	.set_mctrl	= imx_rpmsg_uart_set_mctrl,
	.get_mctrl	= imx_rpmsg_uart_get_mctrl,
	.stop_tx	= imx_rpmsg_uart_stop_tx,
	.start_tx	= imx_rpmsg_uart_start_tx,
	.stop_rx	= imx_rpmsg_uart_stop_rx,
	.enable_ms	= imx_rpmsg_uart_enable_ms,
	.startup	= imx_rpmsg_uart_startup,
	.shutdown	= imx_rpmsg_uart_shutdown,
	.set_termios	= imx_rpmsg_uart_set_termios,
	.type		= imx_rpmsg_uart_type,
	.config_port	= imx_rpmsg_uart_config_port,
};

#define READ_PROP_OR_RETURN(group, name)				\
	ret = of_property_read_u32(np, #name, &init. group . name);	\
	if (ret) {							\
		dev_err(dev, "%pOF: error reading prop %s: %d\n",	\
			np, #name, ret);				\
		return ret;						\
	}
static int imx_rpmsg_uart_init_remote(struct imx_rpmsg_port *rport,
				      tcflag_t cflag)
{
	struct imx_rpmsg_tty_msg msg = {
		.header.cmd = TTY_RPMSG_COMMAND_INIT,
	};
	struct device *dev = &rport->pdev->dev;
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
		READ_PROP_OR_RETURN(lpuart, uart_index);
		ret = of_property_read_u32(np, "rs485_flags",
					   &init.lpuart.rs485_flags);
		if (ret == 0) {
			/* rs485 disabled if flags missing, but de_gpio is
			 * mandatory if enabled */
			READ_PROP_OR_RETURN(lpuart, rs485_de_gpio);
		}

		ret = of_property_read_u32(np, "suspend_wakeup_gpio",
					   &init.lpuart.suspend_wakeup_gpio);
		if (ret < 0) {
			dev_info(dev,
				 "%pOF: no suspend wakeup gpio set, will not wake up\n",
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
	case TTY_TYPE_M33_CONSOLE:
		// nothing to set
		break;
	case TTY_TYPE_FLEXIO:
		READ_PROP_OR_RETURN(flexio, flexio_index);
		READ_PROP_OR_RETURN(flexio, flexio_rx);
		READ_PROP_OR_RETURN(flexio, flexio_tx);
		ret = of_property_read_u32(np, "suspend_wakeup_gpio",
					   &init.flexio.suspend_wakeup_gpio);
		if (ret < 0) {
			dev_info(dev,
				 "%pOF: no suspend wakeup gpio set, will not wake up\n",
				 np);
			/* setting wakeup will error out m33-side and fail suspend */
			init.flexio.suspend_wakeup_gpio = -1;
		}
		init.flexio.cflag = cflag;
		break;
	default:
		dev_err(dev, "%pOF: invalid port_type %d\n", np, init.port_type);
		return -EINVAL;
	}

	return imx_rpmsg_uart_send_and_wait(rport, (void *)&msg, &init,
					    sizeof(init), true);
}
#undef READ_PROP_OR_RETURN

static struct uart_driver imx_rpmsg_uart_driver = {
	.owner          = THIS_MODULE,
	.driver_name    = DRIVER_NAME,
	.dev_name       = DEV_NAME,
	.major          = UNNAMED_MAJOR,
	.minor          = 0,
	.nr             = IMX_RPMSG_UART_PORT_PER_SOC_MAX,
};

static int imx_rpmsg_uart_platform_probe(struct platform_device *pdev)
{
	int of_id, id = -1;
	struct imx_rpmsg_port *rport;
	struct ktermios init_termios;
	struct rpmsg_device *rpdev = uart_rpmsg.rpdev;
	int ret;

	/* defer probing until we can process rpmsg replies */
	if (!rpdev)
		return -EPROBE_DEFER;

	rport = devm_kzalloc(&pdev->dev, sizeof(*rport), GFP_KERNEL);
	if (!rport)
		return -ENOMEM;

	of_id = of_alias_get_id(pdev->dev.of_node, "ttyrpmsg");
	if (of_id >= 0)
		id = ida_simple_get(&uart_rpmsg.ida, of_id, of_id + 1, GFP_KERNEL);
	if (id < 0)
		id = ida_simple_get(&uart_rpmsg.ida, 0, 0, GFP_KERNEL);
	if (of_id >= 0 && id != of_id)
		dev_warn(&pdev->dev, "Alias ID %d not available - got %d\n",
			 of_id, id);
	if (id >= IMX_RPMSG_UART_PORT_PER_SOC_MAX) {
		dev_err(&pdev->dev, "id %d higher than max %d\n",
			id, IMX_RPMSG_UART_PORT_PER_SOC_MAX);
		ret = -ERANGE;
		goto error;
	}
	rport->port.line = id;

	rport->port.dev = &pdev->dev;
	rport->port.type = PORT_IMX_RPMSG;
	rport->port.iotype = UPIO_MEM;
	rport->port.fifosize = 8;
	rport->port.ops = &imx_rpmsg_uart_ops;
	rport->port.flags = UPF_BOOT_AUTOCONF;

	rport->rpdev = rpdev;
	rport->pdev = pdev;

	spin_lock_init(&rport->rx_lock);
	init_completion(&rport->cmd_complete);
	mutex_init(&rport->tx_lock);
	spin_lock_init(&rport->request_id_lock);

	platform_set_drvdata(pdev, rport);
	uart_rpmsg.rports[id] = rport;

	/*
	 * set the baud rate to our remote processor's UART, and tell
	 * remote processor about this channel
	 */
	init_termios = tty_std_termios;
	tty_termios_encode_baud_rate(&init_termios,
				     IMX_RPMSG_DEFAULT_BAUD,
				     IMX_RPMSG_DEFAULT_BAUD);
	ret = imx_rpmsg_uart_init_remote(rport, init_termios.c_cflag);
	if (ret)
		goto error;

	if (of_property_read_bool(pdev->dev.of_node, "rpmsg-tty-no-echo"))
		init_termios.c_lflag &= ~ECHO;
	imx_rpmsg_uart_driver.tty_driver->init_termios = init_termios;

	INIT_WORK(&rport->tx_work, imx_rpmsg_uart_tx_work);
	ret = uart_add_one_port(&imx_rpmsg_uart_driver, &rport->port);
	if (ret)
		goto error;

	device_set_wakeup_capable(&pdev->dev, true);

	dev_info(&pdev->dev, "Initialized ttyrpmsg%d!\n", rport->port.line);

	return 0;

error:
	ida_free(&uart_rpmsg.ida, id);

	return ret;
}

static int imx_rpmsg_uart_platform_remove(struct platform_device *pdev)
{
	struct imx_rpmsg_port *rport = platform_get_drvdata(pdev);

	dev_info(&pdev->dev, "rpmsg tty driver is removed\n");

	return uart_add_one_port(&imx_rpmsg_uart_driver, &rport->port);
}

static int __maybe_unused imx_rpmsg_uart_suspend(struct device *dev)
{
	struct imx_rpmsg_tty_msg msg = {
		.header.cmd = TTY_RPMSG_COMMAND_SET_WAKE,
	};
	struct imx_rpmsg_port *rport = dev_get_drvdata(dev);
	bool enable = device_may_wakeup(dev);

	return imx_rpmsg_uart_send_and_wait(rport, (void *)&msg, &enable,
					    sizeof(enable), true);
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
	int ret;

	dev_info(&rpdev->dev, "new channel: 0x%x -> 0x%x!\n",
		 rpdev->src, rpdev->dst);

	ida_init(&uart_rpmsg.ida);

	ret = uart_register_driver(&imx_rpmsg_uart_driver);
	if (ret)
		return ret;

	ret = platform_driver_register(&imx_rpmsg_uart_platform_driver);
	if (ret != 0)
		goto err;

	/* platform_driver_register() calls the platform probe immediately,
	 * but we need it to process rpmsg replies so only set rpdev last
	 * to make the first platform probe defer */
	uart_rpmsg.rpdev = rpdev;

	return ret;

err:
	uart_unregister_driver(&imx_rpmsg_uart_driver);
	return ret;
}

static void imx_rpmsg_uart_rpmsg_remove(struct rpmsg_device *rpdev)
{
	dev_info(&rpdev->dev, "uart channel removed\n");

	uart_rpmsg.rpdev = NULL;
	platform_driver_unregister(&imx_rpmsg_uart_platform_driver);
	uart_unregister_driver(&imx_rpmsg_uart_driver);
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
