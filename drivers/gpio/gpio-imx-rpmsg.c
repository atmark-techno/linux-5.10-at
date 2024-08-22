/*
 * Copyright 2017 NXP
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/imx_rpmsg.h>
#include <linux/init.h>
#include <linux/irqdomain.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_qos.h>
#include <linux/rpmsg.h>
#include <linux/virtio.h>

#define RPMSG_TIMEOUT	1000
#define IMX_RPMSG_GPIO_PORT_PER_SOC_MAX	10
#define IMX_RPMSG_GPIO_VERSION_MAJOR 2
#define IMX_RPMSG_GPIO_VERSION_MINOR 0
#define GPIO_RPMSG_PINCTRL_UNSET 0xffffffff


enum gpio_input_trigger_type {
	GPIO_RPMSG_TRI_IGNORE,
	GPIO_RPMSG_TRI_RISING,
	GPIO_RPMSG_TRI_FALLING,
	GPIO_RPMSG_TRI_BOTH_EDGE,
	GPIO_RPMSG_TRI_LOW_LEVEL,
	GPIO_RPMSG_TRI_HIGH_LEVEL,
};

enum gpio_rpmsg_header_type {
	GPIO_RPMSG_SETUP,
	GPIO_RPMSG_REPLY,
	GPIO_RPMSG_NOTIFY,
};

enum gpio_rpmsg_header_cmd {
	GPIO_RPMSG_INPUT_INIT,
	GPIO_RPMSG_OUTPUT_INIT,
	GPIO_RPMSG_INPUT_GET,
	GPIO_RPMSG_OUTPUT_SET,
};

struct gpio_rpmsg_data {
	struct imx_rpmsg_head header;
	u8 pin_idx;
	u8 port_idx;
	union {
		struct {
			u8 event;
			u8 wakeup;
			u32 pinctrl;
		} __packed input_init;
		struct {
			u8 value;
			u32 pinctrl;
		} __packed output_init;
		/* no arg for input_get */
		struct {
			u8 value;
		} output_set;
		struct {
			u8 retcode;
			u8 value; /* only valid for input_get */
		} reply;
	};
} __packed __aligned(1);

struct imx_rpmsg_gpio_pin {
	u8 irq_shutdown;
	u8 irq_unmask;
	u8 irq_mask;
	u32 irq_wake_enable;
	u32 irq_type;
	u32 pinctrl;
};

struct imx_rpmsg_gpio_port {
	struct gpio_chip gc;
	struct irq_chip chip;
	int idx;
	struct imx_rpmsg_gpio_pin gpio_pins[];
};

struct imx_gpio_rpmsg_info {
	struct rpmsg_device *rpdev;
	struct gpio_rpmsg_data *notify_msg;
	struct gpio_rpmsg_data *reply_msg;
	struct pm_qos_request pm_qos_req;
	struct completion cmd_complete;
	struct imx_rpmsg_gpio_port *port_store[IMX_RPMSG_GPIO_PORT_PER_SOC_MAX];
	struct mutex lock;
};

static struct imx_gpio_rpmsg_info gpio_rpmsg;

static int gpio_send_message(struct imx_rpmsg_gpio_port *port,
			     struct gpio_rpmsg_data *msg,
			     struct imx_gpio_rpmsg_info *info,
			     u8 *value)
{
	int err;

	if (!info->rpdev) {
		dev_dbg(&info->rpdev->dev,
			"rpmsg channel not ready, m4 image ready?\n");
		return -EINVAL;
	}

	cpu_latency_qos_add_request(&info->pm_qos_req,
			0);

	reinit_completion(&info->cmd_complete);

	err = rpmsg_send(info->rpdev->ept, (void *)msg,
			    sizeof(struct gpio_rpmsg_data));

	if (err) {
		dev_err(&info->rpdev->dev, "rpmsg_send failed: %d\n", err);
		goto err_out;
	}

	if (value) {
		err = wait_for_completion_timeout(&info->cmd_complete,
					msecs_to_jiffies(RPMSG_TIMEOUT));
		if (!err) {
			dev_err(&info->rpdev->dev, "rpmsg_send timeout!\n");
			err = -ETIMEDOUT;
			goto err_out;
		}

		if (info->reply_msg->reply.retcode != 0) {
			dev_err(&info->rpdev->dev, "rpmsg error for %d / %d,%d: %d!\n",
				msg->header.cmd, msg->port_idx, msg->pin_idx,
				info->reply_msg->reply.retcode);
			err = -EINVAL;
			goto err_out;
		}

		if (info->reply_msg->pin_idx >= port->gc.ngpio) {
			dev_err(&info->rpdev->dev, "acked index %d above max %d!\n",
				info->reply_msg->pin_idx, port->gc.ngpio);
			err = -EINVAL;
			goto err_out;
		}

		/* copy the reply value */
		*value = info->reply_msg->reply.value;

		err = 0;
	}

err_out:
	cpu_latency_qos_remove_request(&info->pm_qos_req);

	return err;
}

static int gpio_rpmsg_cb(struct rpmsg_device *rpdev,
	void *data, int len, void *priv, u32 src)
{
	struct gpio_rpmsg_data *msg = (struct gpio_rpmsg_data *)data;
	unsigned long flags;

	if (msg->header.type == GPIO_RPMSG_REPLY) {
		/* TBD: Add irq request_id check for A core msg */
		gpio_rpmsg.reply_msg = msg;
		complete(&gpio_rpmsg.cmd_complete);
	} else if (msg->header.type == GPIO_RPMSG_NOTIFY) {
		gpio_rpmsg.notify_msg = msg;
		if (msg->port_idx >= IMX_RPMSG_GPIO_PORT_PER_SOC_MAX) {
			dev_err(&gpio_rpmsg.rpdev->dev, "port_idx %d too large\n", msg->port_idx);
			return 0;
		}
		local_irq_save(flags);
		generic_handle_irq(irq_find_mapping(gpio_rpmsg.port_store[msg->port_idx]->gc.irq.domain, msg->pin_idx));
		local_irq_restore(flags);
	} else
		dev_err(&gpio_rpmsg.rpdev->dev, "wrong command type!\n");

	return 0;
}

static inline void imx_rpmsg_gpio_msg_init(struct imx_rpmsg_gpio_port *port,
		unsigned int gpio, struct gpio_rpmsg_data *msg)
{
	msg->header.cate = IMX_RPMSG_GPIO;
	msg->header.major = IMX_RPMSG_GPIO_VERSION_MAJOR;
	msg->header.minor = IMX_RPMSG_GPIO_VERSION_MINOR;
	msg->header.type = GPIO_RPMSG_SETUP;
	msg->pin_idx = gpio;
	msg->port_idx = port->idx;
}

static int imx_rpmsg_gpio_get(struct gpio_chip *gc, unsigned int gpio)
{
	struct imx_rpmsg_gpio_port *port = gpiochip_get_data(gc);
	struct gpio_rpmsg_data msg = { 0 };
	int ret;
	u8 value;

	if (gpio >= port->gc.ngpio) {
		dev_err(&gpio_rpmsg.rpdev->dev, "index %d > max %d!\n",
			gpio, port->gc.ngpio);
		return -EINVAL;
	}

	mutex_lock(&gpio_rpmsg.lock);

	imx_rpmsg_gpio_msg_init(port, gpio, &msg);
	msg.header.cmd = GPIO_RPMSG_INPUT_GET;

	ret = gpio_send_message(port, &msg, &gpio_rpmsg, &value);
	if (!ret)
		ret = !!value;

	mutex_unlock(&gpio_rpmsg.lock);

	return ret;
}

static int imx_rpmsg_gpio_direction_input(struct gpio_chip *gc,
					  unsigned int gpio)
{
	struct imx_rpmsg_gpio_port *port = gpiochip_get_data(gc);
	struct gpio_rpmsg_data msg = { 0 };
	int ret;
	u8 wait;

	if (gpio >= port->gc.ngpio) {
		dev_err(&gpio_rpmsg.rpdev->dev, "index %d > max %d!\n",
			gpio, port->gc.ngpio);
		return -EINVAL;
	}

	mutex_lock(&gpio_rpmsg.lock);

	imx_rpmsg_gpio_msg_init(port, gpio, &msg);
	msg.header.cmd = GPIO_RPMSG_INPUT_INIT;
	msg.input_init.event = GPIO_RPMSG_TRI_IGNORE;
	msg.input_init.wakeup = 0;
	msg.input_init.pinctrl = port->gpio_pins[gpio].pinctrl;

	ret = gpio_send_message(port, &msg, &gpio_rpmsg, &wait);

	mutex_unlock(&gpio_rpmsg.lock);

	return ret;
}

static void imx_rpmsg_gpio_set(struct gpio_chip *gc, unsigned int gpio, int val)
{
	struct imx_rpmsg_gpio_port *port = gpiochip_get_data(gc);
	struct gpio_rpmsg_data msg = { 0 };
	u8 wait;

	if (gpio >= port->gc.ngpio) {
		dev_err(&gpio_rpmsg.rpdev->dev, "index %d > max %d!\n",
			gpio, port->gc.ngpio);
		return;
	}

	mutex_lock(&gpio_rpmsg.lock);

	imx_rpmsg_gpio_msg_init(port, gpio, &msg);
	msg.header.cmd = GPIO_RPMSG_OUTPUT_SET;
	msg.output_set.value = val;

	gpio_send_message(port, &msg, &gpio_rpmsg, &wait);

	mutex_unlock(&gpio_rpmsg.lock);
}

static int imx_rpmsg_gpio_direction_output(struct gpio_chip *gc,
					unsigned int gpio, int val)
{
	struct imx_rpmsg_gpio_port *port = gpiochip_get_data(gc);
	struct gpio_rpmsg_data msg = { 0 };
	int ret;
	u8 wait;

	if (gpio >= port->gc.ngpio) {
		dev_err(&gpio_rpmsg.rpdev->dev, "index %d > max %d!\n",
			gpio, port->gc.ngpio);
		return -EINVAL;
	}

	mutex_lock(&gpio_rpmsg.lock);

	imx_rpmsg_gpio_msg_init(port, gpio, &msg);
	msg.header.cmd = GPIO_RPMSG_OUTPUT_INIT;
	msg.output_init.value = val;
	msg.output_init.pinctrl = port->gpio_pins[gpio].pinctrl;

	ret = gpio_send_message(port, &msg, &gpio_rpmsg, &wait);

	mutex_unlock(&gpio_rpmsg.lock);

	return ret;
}

static int imx_rpmsg_irq_set_type(struct irq_data *d, u32 type)
{
	struct imx_rpmsg_gpio_port *port = irq_data_get_irq_chip_data(d);
	irq_flow_handler_t handler = handle_bad_irq;
	u32 gpio_idx = d->hwirq;
	int edge = 0;
	int ret = 0;

	switch (type) {
	case IRQ_TYPE_EDGE_RISING:
		edge = GPIO_RPMSG_TRI_RISING;
		handler = handle_edge_irq;
		break;
	case IRQ_TYPE_EDGE_FALLING:
		edge = GPIO_RPMSG_TRI_FALLING;
		handler = handle_edge_irq;
		break;
	case IRQ_TYPE_EDGE_BOTH:
		edge = GPIO_RPMSG_TRI_BOTH_EDGE;
		handler = handle_edge_irq;
		break;
	case IRQ_TYPE_LEVEL_LOW:
		edge = GPIO_RPMSG_TRI_LOW_LEVEL;
		handler = handle_level_irq;
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		edge = GPIO_RPMSG_TRI_HIGH_LEVEL;
		handler = handle_level_irq;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	port->gpio_pins[gpio_idx].irq_type = edge;
	irq_set_handler_locked(d, handler);
	return ret;
}

static int imx_rpmsg_irq_set_wake(struct irq_data *d, u32 enable)
{
	struct imx_rpmsg_gpio_port *port = irq_data_get_irq_chip_data(d);
	u32 gpio_idx = d->hwirq;

	if (gpio_idx >= port->gc.ngpio) {
		dev_err(&gpio_rpmsg.rpdev->dev, "index %d > max %d!\n",
			gpio_idx, port->gc.ngpio);
		return -EINVAL;
	}

	port->gpio_pins[gpio_idx].irq_wake_enable = enable;

	return 0;
}

/*
 * This function will be called at:
 *  - one interrupt setup.
 *  - the end of one interrupt happened
 * The gpio over rpmsg driver will not write the real register, so save
 * all infos before this function and then send all infos to M core in this
 * step.
 */
static void imx_rpmsg_unmask_irq(struct irq_data *d)
{
	struct imx_rpmsg_gpio_port *port = irq_data_get_irq_chip_data(d);
	u32 gpio_idx = d->hwirq;

	if (gpio_idx >= port->gc.ngpio) {
		dev_err(&gpio_rpmsg.rpdev->dev, "index %d > max %d!\n",
			gpio_idx, port->gc.ngpio);
		return;
	}

	port->gpio_pins[gpio_idx].irq_unmask = 1;
}

static void imx_rpmsg_mask_irq(struct irq_data *d)
{
	struct imx_rpmsg_gpio_port *port = irq_data_get_irq_chip_data(d);
	u32 gpio_idx = d->hwirq;

	if (gpio_idx >= port->gc.ngpio) {
		dev_err(&gpio_rpmsg.rpdev->dev, "index %d > max %d!\n",
			gpio_idx, port->gc.ngpio);
		return;
	}

	/*
	 * No need to implement the callback at A core side.
	 * M core will mask interrupt after a interrupt occurred, and then
	 * sends a notify to A core.
	 * After A core dealt with the notify, A core will send a rpmsg to
	 * M core to unmask this interrupt again.
	 */
	port->gpio_pins[gpio_idx].irq_mask = 1;
}

static void imx_rpmsg_irq_ack(struct irq_data *d)
{
}

static void imx_rpmsg_irq_shutdown(struct irq_data *d)
{
	struct imx_rpmsg_gpio_port *port = irq_data_get_irq_chip_data(d);
	u32 gpio_idx = d->hwirq;

	if (gpio_idx >= port->gc.ngpio) {
		dev_err(&gpio_rpmsg.rpdev->dev, "index %d > max %d!\n",
			gpio_idx, port->gc.ngpio);
		return;
	}

	port->gpio_pins[gpio_idx].irq_shutdown = 1;
}

static void imx_rpmsg_irq_bus_lock(struct irq_data *d)
{
	mutex_lock(&gpio_rpmsg.lock);
}

static void imx_rpmsg_irq_bus_sync_unlock(struct irq_data *d)
{
	struct imx_rpmsg_gpio_port *port = irq_data_get_irq_chip_data(d);
	struct gpio_rpmsg_data msg = { 0 };
	u32 gpio_idx = d->hwirq;

	if (port == NULL) {
		mutex_unlock(&gpio_rpmsg.lock);
		return;
	}

	if (gpio_idx >= port->gc.ngpio) {
		mutex_unlock(&gpio_rpmsg.lock);
		dev_err(&gpio_rpmsg.rpdev->dev, "index %d > max %d!\n",
			gpio_idx, port->gc.ngpio);
		return;
	}

	/*
	 * For mask irq, do nothing here.
	 * M core will mask interrupt after a interrupt occurred, and then
	 * sends a notify to A core.
	 * After A core dealt with the notify, A core will send a rpmsg to
	 * M core to unmask this interrupt again.
	 */

	if (port->gpio_pins[gpio_idx].irq_mask && !port->gpio_pins[gpio_idx].irq_unmask) {
		mutex_unlock(&gpio_rpmsg.lock);
		port->gpio_pins[gpio_idx].irq_mask = 0;
		return;
	}

	imx_rpmsg_gpio_msg_init(port, gpio_idx, &msg);
	msg.header.cmd = GPIO_RPMSG_INPUT_INIT;

	if (port->gpio_pins[gpio_idx].irq_shutdown) {
		msg.input_init.event = GPIO_RPMSG_TRI_IGNORE;
		msg.input_init.wakeup = 0;
		port->gpio_pins[gpio_idx].irq_shutdown = 0;
	} else {
		 /* if not set irq type, then use low level as trigger type */
		msg.input_init.event = port->gpio_pins[gpio_idx].irq_type;
		if (!msg.input_init.event)
			msg.input_init.event = GPIO_RPMSG_TRI_LOW_LEVEL;
		if (port->gpio_pins[gpio_idx].irq_unmask) {
			msg.input_init.wakeup = 0;
			port->gpio_pins[gpio_idx].irq_unmask = 0;
		} else /* irq set wake */
			msg.input_init.wakeup = port->gpio_pins[gpio_idx].irq_wake_enable;
	}

	gpio_send_message(port, &msg, &gpio_rpmsg, NULL);
	mutex_unlock(&gpio_rpmsg.lock);
}

static struct irq_chip imx_rpmsg_irq_chip = {
	.irq_mask = imx_rpmsg_mask_irq,
	.irq_unmask = imx_rpmsg_unmask_irq,
	.irq_ack = imx_rpmsg_irq_ack,
	.irq_set_wake = imx_rpmsg_irq_set_wake,
	.irq_set_type = imx_rpmsg_irq_set_type,
	.irq_shutdown = imx_rpmsg_irq_shutdown,
	.irq_bus_lock = imx_rpmsg_irq_bus_lock,
	.irq_bus_sync_unlock = imx_rpmsg_irq_bus_sync_unlock,
};

static int imx_rpmsg_gpio_get_one_pinctrl(struct imx_rpmsg_gpio_port *port, struct device_node *np)
{
	int count = of_property_count_elems_of_size(np, "imx-rpmsg,pins", sizeof(u32));
	int i, err = 0;

	if (count < 0) {
		dev_info(port->gc.parent, "No imx-rpmsg,pins node in gpio %d's pinctrl '%s'\n",
			 port->idx, np->name);
		/* just skip, not a hard error */
		return 0;
	}

	if (count % 2 != 0) {
		dev_err(port->gc.parent, "Expected an even number of elements in gpio %d's pinctrl '%s', got %d\n",
			 port->idx, np->name, count);
		return -EINVAL;
	}

	for (i = 0; i < count; i += 2) {
		uint32_t pin, pinctrl;

		err = of_property_read_u32_index(np, "imx-rpmsg,pins", i, &pin);
		if (err)
			break;
		err = of_property_read_u32_index(np, "imx-rpmsg,pins", i + 1, &pinctrl);
		if (err)
			break;

		/* pin should be within range.. */
		if ((pin >> 8) != port->idx) {
			dev_err(port->gc.parent, "gpio %d's pinctrl '%s', pin %#x not for this gpio\n",
				port->idx, np->name, pin);
		}
		pin = pin & 0xff;
		if (pin >= port->gc.ngpio) {
			dev_err(port->gc.parent, "gpio %d's pinctrl '%s', pin %#x > ngpio %#x\n",
				port->idx, np->name, pin, port->gc.ngpio);
		}
		if (port->gpio_pins[pin].pinctrl != GPIO_RPMSG_PINCTRL_UNSET) {
			dev_warn(port->gc.parent, "gpio %d's pin %#x was already set, overwriting it in '%s'\n",
				 port->idx, pin, np->name);
		}
		dev_dbg(port->gc.parent, "Set gpio %d pin %#x pinctrl to %#x\n", port->idx, pin, pinctrl);
		port->gpio_pins[pin].pinctrl = pinctrl;
	}
	if (err)
		dev_err(port->gc.parent, "Error in gpio %d's pinctrl '%s': %d\n",
			 port->idx, np->name, err);

	return err;
}

static int imx_rpmsg_gpio_get_pinctrl(struct imx_rpmsg_gpio_port *port)
{
	struct device_node *pinctrls, *child;
	int ret;

	pinctrls = of_get_child_by_name(port->gc.of_node, "pinctrl");
	if (!pinctrls) {
		dev_info(port->gc.parent, "No 'pinctrl' node for gpio %d\n", port->idx);
		return 0;
	}

	for_each_child_of_node(pinctrls, child) {
		ret = imx_rpmsg_gpio_get_one_pinctrl(port, child);
		if (ret) {
			of_node_put(child);
			break;
		}
	}

	of_node_put(pinctrls);

	return 0;
}

static int imx_rpmsg_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct imx_rpmsg_gpio_port *port;
	struct gpio_chip *gc;
	struct gpio_irq_chip *girq;
	int ngpio;
	int i;
	int ret;

	ret = of_property_read_u32(np, "gpio-count", &ngpio);
	if (ret) {
		// fallback to historic value
		ngpio = 32;
	}

	port = devm_kzalloc(&pdev->dev, sizeof(*port) + ngpio, GFP_KERNEL);
	if (!port)
		return -ENOMEM;

	ret = of_property_read_u32(np, "port_idx", &port->idx);
	if (ret)
		return ret;

	if (port->idx >= IMX_RPMSG_GPIO_PORT_PER_SOC_MAX) {
		dev_err(&pdev->dev, "port_idx %d too large\n", port->idx);
		return -EINVAL;
	}

	gpio_rpmsg.port_store[port->idx] = port;

	gc = &port->gc;
	gc->of_node = np;
	gc->parent = dev;
	gc->label = kasprintf(GFP_KERNEL, "imx-rpmsg-gpio-%d", port->idx);
	gc->ngpio = ngpio;
	gc->base = -1;

	for (i = 0; i < ngpio; i++) {
		port->gpio_pins[i].pinctrl = GPIO_RPMSG_PINCTRL_UNSET;
	}
	ret = imx_rpmsg_gpio_get_pinctrl(port);
	if (ret < 0)
		return ret;

	gc->direction_input = imx_rpmsg_gpio_direction_input;
	gc->direction_output = imx_rpmsg_gpio_direction_output;
	gc->get = imx_rpmsg_gpio_get;
	gc->set = imx_rpmsg_gpio_set;

	platform_set_drvdata(pdev, port);

	/* generate one new irq domain */
	port->chip = imx_rpmsg_irq_chip;
	port->chip.name = kasprintf(GFP_KERNEL, "rpmsg-irq-port-%d", port->idx);

	girq = &gc->irq;
	girq->chip = &port->chip;
	girq->default_type = IRQ_TYPE_NONE;
	girq->handler = handle_bad_irq;

	return devm_gpiochip_add_data(dev, gc, port);
}

static const struct of_device_id imx_rpmsg_gpio_dt_ids[] = {
	{ .compatible = "fsl,imx-rpmsg-gpio" },
	{ /* sentinel */ }
};

static struct platform_driver imx_rpmsg_gpio_driver = {
	.driver	= {
		.name = "gpio-imx-rpmsg",
		.of_match_table = imx_rpmsg_gpio_dt_ids,
	},
	.probe = imx_rpmsg_gpio_probe,
};

static int gpio_rpmsg_probe(struct rpmsg_device *rpdev)
{
	gpio_rpmsg.rpdev = rpdev;
	dev_info(&rpdev->dev, "new channel: 0x%x -> 0x%x!\n",
			rpdev->src, rpdev->dst);

	init_completion(&gpio_rpmsg.cmd_complete);
	mutex_init(&gpio_rpmsg.lock);

	return platform_driver_register(&imx_rpmsg_gpio_driver);
}

static void gpio_rpmsg_remove(struct rpmsg_device *rpdev)
{
	dev_info(&rpdev->dev, "gpio channel removed\n");
	platform_driver_unregister(&imx_rpmsg_gpio_driver);
	gpio_rpmsg.rpdev = NULL;
}

static struct rpmsg_device_id gpio_rpmsg_id_table[] = {
	{ .name = "rpmsg-io-channel" },
	{},
};

static struct rpmsg_driver gpio_rpmsg_driver = {
	.drv.name	= "gpio_rpmsg",
	.drv.owner	= THIS_MODULE,
	.id_table	= gpio_rpmsg_id_table,
	.probe		= gpio_rpmsg_probe,
	.callback	= gpio_rpmsg_cb,
	.remove		= gpio_rpmsg_remove,
};


static int __init gpio_imx_rpmsg_init(void)
{
	return register_rpmsg_driver(&gpio_rpmsg_driver);
}
device_initcall(gpio_imx_rpmsg_init);

MODULE_AUTHOR("NXP Semiconductor");
MODULE_DESCRIPTION("NXP i.MX7ULP rpmsg gpio driver");
MODULE_LICENSE("GPL v2");
