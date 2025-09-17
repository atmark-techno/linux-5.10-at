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
#include <linux/gpio/machine.h>
#include <linux/imx_rpmsg.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irqdomain.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/parser.h>
#include <linux/platform_device.h>
#include <linux/pm_qos.h>
#include <linux/rpmsg.h>
#include <linux/virtio.h>
#include <linux/workqueue.h>

#include "gpiolib.h"

#define RPMSG_TIMEOUT	1000
#define IMX_RPMSG_GPIO_PORT_PER_SOC_MAX	3
#define IMX_RPMSG_GPIO_VERSION_MAJOR 4
#define IMX_RPMSG_GPIO_VERSION_MINOR 0
#define GPIO_RPMSG_PINCTRL_UNSET 0xffffffff


enum gpio_input_trigger_type {
	GPIO_RPMSG_TRI_IGNORE = 0,
	GPIO_RPMSG_TRI_RISING,
	GPIO_RPMSG_TRI_FALLING,
	GPIO_RPMSG_TRI_BOTH_EDGE,
	GPIO_RPMSG_TRI_LOW_LEVEL,
	GPIO_RPMSG_TRI_HIGH_LEVEL,
	GPIO_RPMSG_TRI_DISABLE = 0xff,
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
	GPIO_RPMSG_PINCTRL,
};

struct gpio_rpmsg_data {
	struct imx_rpmsg_head header;
	u8 request_id;
	u8 pin_idx;
	u8 port_idx;
	union {
		struct {
			u8 event;
			u8 wakeup;
		} __packed input_init;
		struct {
			u8 value;
		} __packed output_init;
		/* no arg for input_get */
		struct {
			u8 value;
		} output_set;
		struct {
			u32 pinctrl[6];
		} __packed pinctrl;
		struct {
			u8 retcode;
			u8 value; /* only valid for input_get */
		} reply;
	};
} __packed __aligned(1);

/* used for irq_mask/unmask and initial gpio setup */
enum irq_state {
	IRQ_MASKED, /* masked, don't sent ack, start here. */
	IRQ_MASKED_JUSTSET, /* just masked, disable (and go back to masked) */
	IRQ_SHUTDOWN, /* just shutting down, disable (and go back to masked) */
	IRQ_UNMASKED, /* normal state, acks sent if irq_type is set */
};

struct imx_rpmsg_gpio_pin {
	u8 pin_idx;
	/* wakeup requested in setwake (irq_type used for wakeup) */
	u8 user_wakeup;
	/* edge setting (irq_set_type) for irq */
	u8 irq_type;
	/* mask/shutdown lifecycle */
	u8 irq_state; /* enum irq_state: using u8 to save space. */
	/* wakeup enabled in irq_set_wake */
	bool irq_wake; /* wakeup */
	/* used for irq_set_wake and irq_set_type:
	 * it cannot wait directly but we do the message sending & waiting
	 * in sync unlock immediately after setting wake or type, to
	 * ensure wakeup is set before suspend.
	 * We need to remember that state afterwards for any other irq
	 * that might come during that time
	 */
	bool irq_trigger_changed;
	/* for irq_state/irq_wake/irq_trigger_changed */
	spinlock_t state_lock;
	/* irq, gpio desc obtained before suspend for user wakeup */
	int irq;
	struct gpio_desc *desc;
	/* for async ack because sending a rpmsg might wait if tx queue is
	 * full and this is forbidden in irq context
	 */
	struct work_struct rpmsg_ack_work;
};

struct imx_rpmsg_gpio_port {
	struct gpio_chip gc;
	struct irq_chip chip;
	int idx;
	int wakeup_source;
	struct imx_rpmsg_gpio_pin gpio_pins[];
};

struct imx_gpio_rpmsg_info {
	struct rpmsg_device *rpdev;
	struct pm_qos_request pm_qos_req;
	struct completion cmd_complete;
	struct imx_rpmsg_gpio_port *port_store[IMX_RPMSG_GPIO_PORT_PER_SOC_MAX];
	struct mutex lock;
	struct workqueue_struct *rpmsg_ack_wq;

	u8 last_retcode;
	u8 last_request_id;
	u16 inflight_request_id;
	u8 *requested_value;
	spinlock_t request_id_lock;
};

static struct imx_gpio_rpmsg_info gpio_rpmsg;

static int gpio_send_message(struct imx_rpmsg_gpio_port *port,
			     struct gpio_rpmsg_data *msg,
			     struct imx_gpio_rpmsg_info *info,
			     u8 *value)
{
	int err;

	if (!info->rpdev) {
		dev_warn(&info->rpdev->dev,
			 "rpmsg channel not ready, m4 image ready?\n");
		return -EINVAL;
	}

	cpu_latency_qos_add_request(&info->pm_qos_req, 0);

	reinit_completion(&info->cmd_complete);
	if (value)
		info->requested_value = value;

	/* send are serialized, and if we wait for a reply the latest value
	 * will always be correct */
	/* We need the spin lock to ensure rpmsg cb does not set last_error
	 * after this timed out & another request started being processed */
	spin_lock_irq(&info->request_id_lock);
	info->inflight_request_id = info->last_request_id++;
	spin_unlock_irq(&info->request_id_lock);
	msg->request_id = info->inflight_request_id;

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
			/* don't process late replies - lock here ensures
			 * we're not completing the next call */
			spin_lock_irq(&info->request_id_lock);
			info->inflight_request_id = 0xffff;
			info->requested_value = NULL;
			spin_unlock_irq(&info->request_id_lock);
			dev_err(&info->rpdev->dev, "rpmsg_send timeout!\n");
			err = -ETIMEDOUT;
			goto err_out;
		}

		if (info->last_retcode != 0) {
			dev_err(&info->rpdev->dev, "rpmsg error for %d / %d,%d: %d!\n",
				msg->header.cmd, msg->port_idx, msg->pin_idx,
				info->last_retcode);
			err = -EINVAL;
			goto err_out;
		}

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
		spin_lock_irq(&gpio_rpmsg.request_id_lock);
		if (msg->request_id != gpio_rpmsg.inflight_request_id) {
			// obsolete reply that was not waited for -- not necessarily an error
			dev_dbg(&rpdev->dev, "unexpected id %x (expected %x)\n",
				msg->request_id, gpio_rpmsg.inflight_request_id);
			spin_unlock_irq(&gpio_rpmsg.request_id_lock);
			return 0;
		}
		/* don't process duplicates */
		gpio_rpmsg.inflight_request_id = 0xffff;

		gpio_rpmsg.last_retcode = msg->reply.retcode;
		if (gpio_rpmsg.requested_value) {
			*gpio_rpmsg.requested_value = msg->reply.value;
			gpio_rpmsg.requested_value = NULL;
		}
		spin_unlock_irq(&gpio_rpmsg.request_id_lock);

		complete(&gpio_rpmsg.cmd_complete);
	} else if (msg->header.type == GPIO_RPMSG_NOTIFY) {
		if (msg->port_idx >= IMX_RPMSG_GPIO_PORT_PER_SOC_MAX
		    || !gpio_rpmsg.port_store[msg->port_idx]) {
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
	dev_dbg(&gpio_rpmsg.rpdev->dev, "%s idx %d\n", __func__, gpio);

	imx_rpmsg_gpio_msg_init(port, gpio, &msg);
	msg.header.cmd = GPIO_RPMSG_INPUT_GET;

	mutex_lock(&gpio_rpmsg.lock);
	ret = gpio_send_message(port, &msg, &gpio_rpmsg, &value);
	mutex_unlock(&gpio_rpmsg.lock);

	if (!ret)
		ret = !!value;

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
	dev_dbg(&gpio_rpmsg.rpdev->dev, "%s idx %d\n", __func__, gpio);

	imx_rpmsg_gpio_msg_init(port, gpio, &msg);
	msg.header.cmd = GPIO_RPMSG_INPUT_INIT;
	msg.input_init.event = GPIO_RPMSG_TRI_IGNORE;
	msg.input_init.wakeup = 0;

	mutex_lock(&gpio_rpmsg.lock);
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
	dev_dbg(&gpio_rpmsg.rpdev->dev, "%s idx %d\n", __func__, gpio);

	imx_rpmsg_gpio_msg_init(port, gpio, &msg);
	msg.header.cmd = GPIO_RPMSG_OUTPUT_SET;
	msg.output_set.value = val;

	mutex_lock(&gpio_rpmsg.lock);
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
	dev_dbg(&gpio_rpmsg.rpdev->dev, "%s idx %d\n", __func__, gpio);

	imx_rpmsg_gpio_msg_init(port, gpio, &msg);
	msg.header.cmd = GPIO_RPMSG_OUTPUT_INIT;
	msg.output_init.value = val;

	mutex_lock(&gpio_rpmsg.lock);
	ret = gpio_send_message(port, &msg, &gpio_rpmsg, &wait);
	mutex_unlock(&gpio_rpmsg.lock);

	return ret;
}

static int imx_rpmsg_irq_set_type(struct irq_data *d, u32 type)
{
	struct imx_rpmsg_gpio_port *port = irq_data_get_irq_chip_data(d);
	struct imx_rpmsg_gpio_pin *pin;
	irq_flow_handler_t handler = handle_bad_irq;
	u32 gpio_idx = d->hwirq;
	int edge = 0;
	int ret = 0;
	unsigned long flags;

	dev_dbg(&gpio_rpmsg.rpdev->dev, "%s: irq %ld\n", __func__, d->hwirq);
	switch (type) {
	case IRQ_TYPE_NONE:
		edge = GPIO_RPMSG_TRI_IGNORE;
		break;
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

	pin = &port->gpio_pins[gpio_idx];
	pin->irq_type = edge;

	irq_set_handler_locked(d, handler);
	if (ret == 0) {
		spin_lock_irqsave(&pin->state_lock, flags);
		pin->irq_trigger_changed = true;
		spin_unlock_irqrestore(&pin->state_lock, flags);
	}
	return ret;
}

static int imx_rpmsg_irq_set_wake(struct irq_data *d, u32 enable)
{
	struct imx_rpmsg_gpio_port *port = irq_data_get_irq_chip_data(d);
	struct imx_rpmsg_gpio_pin *pin;
	u32 gpio_idx = d->hwirq;
	unsigned long flags;

	dev_dbg(&gpio_rpmsg.rpdev->dev, "%s: irq %ld @ %x\n", __func__, d->hwirq, enable);
	if (gpio_idx >= port->gc.ngpio) {
		dev_err(&gpio_rpmsg.rpdev->dev, "index %d > max %d!\n",
			gpio_idx, port->gc.ngpio);
		return -EINVAL;
	}

	pin = &port->gpio_pins[gpio_idx];

	spin_lock_irqsave(&pin->state_lock, flags);
	pin->irq_wake = enable;
	pin->irq_trigger_changed = true;
	spin_unlock_irqrestore(&pin->state_lock, flags);
	return 0;
}

static void imx_rpmsg_irq_bus_lock(struct irq_data *d)
{
	dev_dbg(&gpio_rpmsg.rpdev->dev, "%s: irq %ld\n", __func__, d->hwirq);
}

static void imx_rpmsg_irq_bus_sync_unlock(struct irq_data *d)
{
	struct imx_rpmsg_gpio_port *port = irq_data_get_irq_chip_data(d);
	struct imx_rpmsg_gpio_pin *pin;
	u32 gpio_idx = d->hwirq;
	struct gpio_rpmsg_data msg = { 0 };
	u8 wait;
	u8 wakeup;
	unsigned long flags;
	bool irq_trigger_changed;

	if (gpio_idx >= port->gc.ngpio) {
		dev_err(&gpio_rpmsg.rpdev->dev, "index %d > max %d!\n",
			gpio_idx, port->gc.ngpio);
		return;
	}
	dev_dbg(&gpio_rpmsg.rpdev->dev, "%s: irq %ld\n", __func__, d->hwirq);

	pin = &port->gpio_pins[gpio_idx];

	spin_lock_irqsave(&pin->state_lock, flags);
	wakeup = pin->irq_wake;
	irq_trigger_changed = pin->irq_trigger_changed;
	pin->irq_trigger_changed = false;
	spin_unlock_irqrestore(&pin->state_lock, flags);

	/* No change detected, skip update */
	if (!irq_trigger_changed)
		return;
	/* also can't set wake if no type */
	if (wakeup && !pin->irq_type)
		return;

	imx_rpmsg_gpio_msg_init(port, gpio_idx, &msg);
	msg.header.cmd = GPIO_RPMSG_INPUT_INIT;

	msg.input_init.event = pin->irq_type;
	msg.input_init.wakeup = wakeup;

	/* need to wait for the reply if enabling:
	 * if the reply comes after mailbox the subsystem suspended
	 * it'll abort suspending */
	mutex_lock(&gpio_rpmsg.lock);
	gpio_send_message(port, &msg, &gpio_rpmsg, wakeup ? &wait : NULL);
	mutex_unlock(&gpio_rpmsg.lock);
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
	struct imx_rpmsg_gpio_pin *pin;
	u32 gpio_idx = d->hwirq;
	unsigned long flags;

	dev_dbg(&gpio_rpmsg.rpdev->dev, "%s: irq %ld\n", __func__, d->hwirq);
	if (gpio_idx >= port->gc.ngpio) {
		dev_err(&gpio_rpmsg.rpdev->dev, "index %d > max %d!\n",
			gpio_idx, port->gc.ngpio);
		return;
	}

	pin = &port->gpio_pins[gpio_idx];

	spin_lock_irqsave(&pin->state_lock, flags);
	pin->irq_state = IRQ_UNMASKED;
	spin_unlock_irqrestore(&pin->state_lock, flags);
	queue_work(gpio_rpmsg.rpmsg_ack_wq, &pin->rpmsg_ack_work);
}

static void imx_rpmsg_mask_irq(struct irq_data *d)
{
	struct imx_rpmsg_gpio_port *port = irq_data_get_irq_chip_data(d);
	struct imx_rpmsg_gpio_pin *pin;
	u32 gpio_idx = d->hwirq;
	unsigned long flags;

	dev_dbg(&gpio_rpmsg.rpdev->dev, "%s: irq %ld\n", __func__, d->hwirq);
	if (gpio_idx >= port->gc.ngpio) {
		dev_err(&gpio_rpmsg.rpdev->dev, "index %d > max %d!\n",
			gpio_idx, port->gc.ngpio);
		return;
	}

	pin = &port->gpio_pins[gpio_idx];
	spin_lock_irqsave(&pin->state_lock, flags);
	pin->irq_state = IRQ_MASKED_JUSTSET;
	spin_unlock_irqrestore(&pin->state_lock, flags);
	queue_work(gpio_rpmsg.rpmsg_ack_wq, &pin->rpmsg_ack_work);
}

static void imx_rpmsg_irq_shutdown(struct irq_data *d)
{
	struct imx_rpmsg_gpio_port *port = irq_data_get_irq_chip_data(d);
	struct imx_rpmsg_gpio_pin *pin;
	u32 gpio_idx = d->hwirq;
	unsigned long flags;

	dev_dbg(&gpio_rpmsg.rpdev->dev, "%s: irq %ld\n", __func__, d->hwirq);
	if (gpio_idx >= port->gc.ngpio) {
		dev_err(&gpio_rpmsg.rpdev->dev, "index %d > max %d!\n",
			gpio_idx, port->gc.ngpio);
		return;
	}

	pin = &port->gpio_pins[gpio_idx];
	/* also reset type to avoid reusing next time we enable */
	port->gpio_pins[gpio_idx].irq_type = 0;
	spin_lock_irqsave(&pin->state_lock, flags);
	pin->irq_state = IRQ_SHUTDOWN;
	spin_unlock_irqrestore(&pin->state_lock, flags);
	queue_work(gpio_rpmsg.rpmsg_ack_wq, &pin->rpmsg_ack_work);
}

static void imx_rpmsg_irq_ack(struct irq_data *d)
{
	struct imx_rpmsg_gpio_port *port = irq_data_get_irq_chip_data(d);
	u32 gpio_idx = d->hwirq;

	dev_dbg(&gpio_rpmsg.rpdev->dev, "%s: irq %ld\n", __func__, d->hwirq);

	queue_work(gpio_rpmsg.rpmsg_ack_wq, &port->gpio_pins[gpio_idx].rpmsg_ack_work);
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

static void imx_rpmsg_gpio_send_ack(struct work_struct *work)
{
	struct imx_rpmsg_gpio_pin *pin =
		container_of(work, struct imx_rpmsg_gpio_pin, rpmsg_ack_work);
	u8 gpio_idx = pin->pin_idx, wakeup = 0;
	struct imx_rpmsg_gpio_port *port =
		container_of(pin, struct imx_rpmsg_gpio_port, gpio_pins[gpio_idx]);
	struct gpio_rpmsg_data msg = { 0 };
	unsigned long flags;
	enum irq_state state;

	/*
	 * For mask irq, do nothing here.
	 * M core will mask interrupt after a interrupt occurred, and then
	 * sends a notify to A core.
	 * After A core dealt with the notify, A core will send a rpmsg to
	 * M core to unmask this interrupt again.
	 */

	spin_lock_irqsave(&pin->state_lock, flags);
	state = pin->irq_state;
	if (state == IRQ_SHUTDOWN)
		pin->irq_state = IRQ_MASKED;
	if (state == IRQ_MASKED_JUSTSET)
		pin->irq_state = IRQ_MASKED;
	wakeup = pin->irq_wake;
	spin_unlock_irqrestore(&pin->state_lock, flags);

	if (state == IRQ_MASKED) {
		dev_dbg(&gpio_rpmsg.rpdev->dev, "%s: %d/%d masked\n",
			__func__, port->idx, gpio_idx);
		return;
	}
	if (state == IRQ_UNMASKED && !pin->irq_type) {
		// set_type not called yet, wait for it.
		dev_dbg(&gpio_rpmsg.rpdev->dev, "%s: %d/%d type not set\n",
			__func__, port->idx, gpio_idx)  ;
		return;
	}
	dev_dbg(&gpio_rpmsg.rpdev->dev, "%s: %d/%d\n", __func__, port->idx, gpio_idx);

	imx_rpmsg_gpio_msg_init(port, gpio_idx, &msg);
	msg.header.cmd = GPIO_RPMSG_INPUT_INIT;

	if (state == IRQ_SHUTDOWN || state == IRQ_MASKED_JUSTSET) {
		msg.input_init.event = GPIO_RPMSG_TRI_DISABLE;
		msg.input_init.wakeup = 0;
	} else {
		msg.input_init.event = pin->irq_type;
		msg.input_init.wakeup = wakeup;
	}

	mutex_lock(&gpio_rpmsg.lock);
	gpio_send_message(port, &msg, &gpio_rpmsg, NULL);
	mutex_unlock(&gpio_rpmsg.lock);
}


static int imx_rpmsg_gpio_get_one_pinctrl(struct imx_rpmsg_gpio_port *port, struct device_node *np)
{
	struct property *prop = of_find_property(np, "imx-rpmsg,pins", NULL);
	int i, count, err = 0;
	const __be32 *cur = NULL;
	struct gpio_rpmsg_data msg = { 0 };

	if (!prop || !prop->value) {
		dev_info(port->gc.parent, "No imx-rpmsg,pins node in %pOF\n", np);
		/* just skip, not a hard error */
		return 0;
	}
	if (prop->length % sizeof(u32) != 0) {
		dev_err(port->gc.parent, "Expected %pOF to contain u32s\n", np);
		return -EINVAL;
	}

	count = prop->length / sizeof(u32);
	if (count % 7 != 0) {
		dev_err(port->gc.parent, "Expected a multiple of 7 elements in %pOF, got %d\n",
			np, count);
		return -EINVAL;
	}

	imx_rpmsg_gpio_msg_init(port, 0, &msg);
	msg.header.cmd = GPIO_RPMSG_PINCTRL;

	for (i = 0; i < count / 7; i++) {
		int j;
		uint32_t pin;
		uint32_t *pinctrl = msg.pinctrl.pinctrl;
		u8 wait;

		cur = of_prop_next_u32(prop, cur, &pin);
		if (!cur)
			err = -ENOENT;
		for (j = 0; j < ARRAY_SIZE(msg.pinctrl.pinctrl); j++) {
			cur = of_prop_next_u32(prop, cur, &pinctrl[j]);
			if (!cur)
				err = -ENOENT;
		}
		if (err) {
			dev_err(port->gc.parent, "Short number of values in %pOF\n", np);
			break;
		}

		/* pin should be within range.. */
		if ((pin >> 8) != port->idx) {
			dev_err(port->gc.parent, "In %pOF, pin %#x not for this gpio, ignored.\n",
				np, pin);
			continue;
		}
		pin = pin & 0xff;
		if (pin >= port->gc.ngpio) {
			dev_err(port->gc.parent, "In %pOF, pin %#x > ngpio %#x: invalid binding used?\n",
				np, pin, port->gc.ngpio);
			continue;
		}
		msg.pin_idx = pin;

		dev_dbg(port->gc.parent, "Setting gpio %d pin %#x pinctrl to %#x/%#x/%#x/%#x/%#x/%#x\n",
			port->idx, pin, pinctrl[0], pinctrl[1], pinctrl[2],
			pinctrl[3], pinctrl[4], pinctrl[5]);
		if (gpio_send_message(port, &msg, &gpio_rpmsg, &wait) < 0) {
			dev_err(port->gc.parent, "Previous m33-side error was in %pOF\n", np);
		}
	}

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

static const match_table_t irq_types = {
	{ IRQ_TYPE_NONE, "disable" },
	{ IRQ_TYPE_EDGE_RISING, "rising" },
	{ IRQ_TYPE_EDGE_FALLING, "falling" },
	{ IRQ_TYPE_EDGE_BOTH, "both" },
	{ IRQ_TYPE_LEVEL_LOW, "low"},
	{ IRQ_TYPE_LEVEL_HIGH, "high"},
	{ IRQ_TYPE_DEFAULT, NULL },
};

static ssize_t setwake_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct imx_rpmsg_gpio_port *port = dev_get_drvdata(dev);
	int ret = count, tmp, irq;
	char *dup, *parse, *word;
	u8 irq_type = IRQ_TYPE_NONE, cur_irq_type;

	dup = kstrdup(buf, GFP_KERNEL);
	if (!dup)
		return -ENOMEM;

	parse = dup;
	/* format: list of words with either irq_type description or pin number.
	 * pin is set to the previous irq_type word.
	 * Anything not recognized is skipped
	 */
	while ((word = strsep(&parse, " \n")) != NULL) {
		substring_t args[MAX_OPT_ARGS];
		struct gpio_desc *desc;
		unsigned int pin;

		if (!word[0])
			continue;

		cur_irq_type = match_token(word, irq_types, args);

		/* irq_type word? */
		if (cur_irq_type != IRQ_TYPE_DEFAULT) {
			irq_type = cur_irq_type;
			continue;
		}

		if (irq_type == IRQ_TYPE_DEFAULT) {
			dev_warn(dev, "setwake: first word %s not one of disabled|rising|falling|both|low|high\n",
				 word);
			continue;
		}

		/* must be pin */
		tmp = kstrtouint(word, 10, &pin);
		if (tmp || pin >= port->gc.ngpio) {
			dev_warn(dev, "setwake: ignored word %s\n", word);
			/* keep first error */
			ret = (ret < 0) ? ret : -EINVAL;
			continue;
		}

		/* don't bother getting irq if disabled */
		if (irq_type == IRQ_TYPE_NONE) {
			WRITE_ONCE(port->gpio_pins[pin].user_wakeup, irq_type);
			continue;
		}

		desc = gpiochip_get_desc(&port->gc, pin);
		if (IS_ERR(desc))
			continue;

		irq = gpiod_to_irq(desc);
		if (irq < 0) {
			dev_warn(dev, "setwake: failed to translate to IRQ on gpio%d pin %d: irq %d\n",
				 port->idx, pin, irq);
			ret = (ret < 0) ? ret : -EINVAL;
			continue;
		}

		WRITE_ONCE(port->gpio_pins[pin].user_wakeup, irq_type);
		port->gpio_pins[pin].irq = irq;
	}

	kfree(dup);
	return ret;
}

static DEVICE_ATTR_WO(setwake);

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

	if (!gpio_rpmsg.rpdev)
		return -EPROBE_DEFER;

	ret = of_property_read_u32(np, "gpio-count", &ngpio);
	if (ret) {
		// fallback to historic value
		ngpio = 32;
	}

	port = devm_kzalloc(dev, sizeof(*port) + ngpio * sizeof(*port->gpio_pins),
			    GFP_KERNEL);
	if (!port)
		return -ENOMEM;

	ret = of_property_read_u32(np, "port_idx", &port->idx);
	if (ret)
		return ret;

	if (port->idx >= IMX_RPMSG_GPIO_PORT_PER_SOC_MAX) {
		dev_err(dev, "port_idx %d too large\n", port->idx);
		return -EINVAL;
	}

	port->wakeup_source = of_property_read_bool(np, "wakeup-source");

	gpio_rpmsg.port_store[port->idx] = port;

	gc = &port->gc;
	gc->of_node = np;
	gc->parent = dev;
	gc->label = kasprintf(GFP_KERNEL, "imx-rpmsg-gpio-%d", port->idx);
	gc->ngpio = ngpio;
	gc->base = -1;
	gc->can_sleep = true;

	for (i = 0; i < ngpio; i++) {
		port->gpio_pins[i].pin_idx = i;
		INIT_WORK(&port->gpio_pins[i].rpmsg_ack_work, imx_rpmsg_gpio_send_ack);
		spin_lock_init(&port->gpio_pins[i].state_lock);
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

	if (port->wakeup_source) {
		ret = device_create_file(dev, &dev_attr_setwake);
		if (ret < 0)
			return ret;

		device_init_wakeup(dev, 1);
	}

	return devm_gpiochip_add_data(dev, gc, port);
}

static int imx_rpmsg_gpio_remove(struct platform_device *pdev)
{
	struct imx_rpmsg_gpio_port *port = dev_get_drvdata(&pdev->dev);

	if (port->wakeup_source)
		device_remove_file(&pdev->dev, &dev_attr_setwake);

	return 0;
}

static int __maybe_unused
imx_rpmsg_gpio_enable_wakeup(struct imx_rpmsg_gpio_port *port)
{
	struct gpio_desc *desc;
	int err = 0;
	int i, irq;
	uint8_t wakeup_type;

	for (i = 0; i < port->gc.ngpio; i++) {
		wakeup_type = READ_ONCE(port->gpio_pins[i].user_wakeup);

		if (!wakeup_type)
			continue;

		desc = gpiochip_request_own_desc(&port->gc, i, "imx_rpmsg_gpio_wakeup",
						 GPIO_ACTIVE_HIGH, GPIOD_IN);

		if (IS_ERR(desc)) {
			dev_warn(port->gc.parent,
				 "GPIO pin %d, failed to request GPIO for pin: err %ld\n",
				 i, PTR_ERR(desc));
			goto fail_null_desc;
		}

		err = gpiochip_lock_as_irq(&port->gc, i);
		if (err) {
			dev_warn(port->gc.parent,
				 "GPIO pin %d, failed to lock as interrupt: err %d\n",
				 i, err);
			goto fail_free_desc;
		}

		irq = port->gpio_pins[i].irq;

		err = enable_irq_wake(irq);
		if (err) {
			dev_warn(port->gc.parent,
				 "GPIO pin %d, failed to configure IRQ %d as wakeup source: err %d\n",
				 i, irq, err);
			goto fail_unlock_irq;
		}

		err = irq_set_irq_type(irq, wakeup_type);
		if (err) {
			dev_warn(port->gc.parent,
				 "GPIO pin %d, failed to set wakeup trigger %d for IRQ %d: err %d\n",
				 i, port->gpio_pins[i].irq_type, irq, err);
			goto fail_disable_irq_type;
		}

		port->gpio_pins[i].desc = desc;

		continue;

fail_disable_irq_type:
		disable_irq_wake(irq);
fail_unlock_irq:
		gpiochip_unlock_as_irq(&port->gc, i);
fail_free_desc:
		gpiochip_free_own_desc(desc);
fail_null_desc:
		port->gpio_pins[i].desc = NULL;
	}

	return 0;
}

static void __maybe_unused
imx_rpmsg_gpio_disable_wakeup(struct imx_rpmsg_gpio_port *port)
{
	int i;

	for (i = 0; i < port->gc.ngpio; i++) {
		if (port->gpio_pins[i].desc == NULL)
			continue;

		disable_irq_wake(port->gpio_pins[i].irq);
		irq_set_irq_type(port->gpio_pins[i].irq, IRQ_TYPE_NONE);
		gpiochip_unlock_as_irq(&port->gc, i);
		gpiochip_free_own_desc(port->gpio_pins[i].desc);
		port->gpio_pins[i].desc = NULL;
	}
}

static int __maybe_unused imx_rpmsg_gpio_suspend(struct device *dev)
{
	struct imx_rpmsg_gpio_port *port = dev_get_drvdata(dev);
	int err;

	if (!device_may_wakeup(dev))
		return 0;

	err = imx_rpmsg_gpio_enable_wakeup(port);
	if (err)
		dev_warn(dev,
			 "could not enable wakeup of one or more pins for suspend\n");

	return 0;
}

static int __maybe_unused imx_rpmsg_gpio_resume(struct device *dev)
{
	struct imx_rpmsg_gpio_port *port = dev_get_drvdata(dev);

	if (!device_may_wakeup(dev))
		return 0;

	imx_rpmsg_gpio_disable_wakeup(port);

	return 0;
}

static SIMPLE_DEV_PM_OPS(imx_rpmsg_gpio_pm_ops, imx_rpmsg_gpio_suspend, imx_rpmsg_gpio_resume);

static const struct of_device_id imx_rpmsg_gpio_dt_ids[] = {
	{ .compatible = "fsl,imx-rpmsg-gpio" },
	{ /* sentinel */ }
};

static struct platform_driver imx_rpmsg_gpio_driver = {
	.driver	= {
		.name = "gpio-imx-rpmsg",
		.pm = &imx_rpmsg_gpio_pm_ops,
		.of_match_table = imx_rpmsg_gpio_dt_ids,
	},
	.probe = imx_rpmsg_gpio_probe,
	.remove = imx_rpmsg_gpio_remove,
};

static int gpio_rpmsg_probe(struct rpmsg_device *rpdev)
{
	int rc;
	dev_info(&rpdev->dev, "new channel: 0x%x -> 0x%x!\n",
			rpdev->src, rpdev->dst);

	init_completion(&gpio_rpmsg.cmd_complete);
	mutex_init(&gpio_rpmsg.lock);
	spin_lock_init(&gpio_rpmsg.request_id_lock);

	gpio_rpmsg.rpmsg_ack_wq = create_workqueue("imx_rpmsg_gpio_workqueue");
	if (!gpio_rpmsg.rpmsg_ack_wq) {
		dev_err(&rpdev->dev, "Failed to create imx_rpmsg_gpio_workqueue");
		return -ENOMEM;
	}

	rc = platform_driver_register(&imx_rpmsg_gpio_driver);
	gpio_rpmsg.rpdev = rpdev;
	return rc;
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
static void __exit gpio_imx_rpmsg_fini(void)
{
	unregister_rpmsg_driver(&gpio_rpmsg_driver);
}
module_init(gpio_imx_rpmsg_init);
module_exit(gpio_imx_rpmsg_fini);

MODULE_AUTHOR("NXP Semiconductor");
MODULE_DESCRIPTION("NXP i.MX7ULP rpmsg gpio driver");
MODULE_LICENSE("GPL v2");
