// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2025 Atmark Techno, Inc. All Rights Reserved.
 */

#include <dt-bindings/firmware/imx/rsrc.h>
#include <linux/bitfield.h>
#include <linux/can.h>
#include <linux/can/dev.h>
#include <linux/can/error.h>
#include <linux/can/led.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/imx_rpmsg.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_qos.h>
#include <linux/pm_runtime.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/rpmsg.h>

#define DRV_NAME "can-imx-rpmsg"

#define RPMSG_TIMEOUT	1000
#define CAN_RPMSG_MAJOR 1
#define CAN_RPMSG_MINOR 0

enum can_rpmsg_header_type {
	CAN_RPMSG_REQUEST,
	CAN_RPMSG_RESPONSE,
	CAN_RPMSG_NOTIFICATION,
};

enum can_rpmsg_header_cmd {
	CAN_RPMSG_COMMAND_PAYLOAD,
	CAN_RPMSG_COMMAND_OPEN,
	CAN_RPMSG_COMMAND_STOP,
	CAN_RPMSG_COMMAND_RESTART,
	CAN_RPMSG_COMMAND_GET_STATUS,
	CAN_RPMSG_COMMAND_NOTIFY,
	CAN_RPMSG_COMMAND_INIT,
	CAN_RPMSG_COMMAND_SET_WAKE,
};

struct imx_rpmsg_can_open_params {
	struct can_bittiming bittiming, data_bittiming;
	u32 ctrlmode;
} __packed __aligned(1);

struct imx_rpmsg_can_init_params {
	u32 suspend_wakeup_gpio;
} __packed __aligned(1);

struct imx_rpmsg_can_priv {
	struct can_priv can;
	struct device *dev;

	struct regulator *reg_xceiver;

	struct work_struct tx_work;
	struct sk_buff_head txq;
};

struct imx_rpmsg_can_info {
	struct rpmsg_device *rpdev;
	struct imx_rpmsg_can_priv *priv; /* Supports only one CAN device */

	struct pm_qos_request pm_qos_req;
	struct completion cmd_complete;
	struct mutex tx_lock;
	spinlock_t rx_lock;
	struct workqueue_struct *rpmsg_ack_wq;

	u8 last_retcode;
	u8 last_request_id;
	u16 inflight_request_id;
	void *requested_value;
	u16 requested_length;
	spinlock_t request_id_lock;
};

static struct imx_rpmsg_can_info can_rpmsg;

#define MAX(a, b)	((a) > (b) ? (a) : (b))
#define CAN_MTU_MAX	MAX(CAN_MTU, CANFD_MTU)
#define CAN_OPEN_PARAMS_SIZE	(sizeof(struct imx_rpmsg_can_open_params))
#define CAN_BUF_MAX	MAX(CAN_MTU_MAX, CAN_OPEN_PARAMS_SIZE)

/* imx_rpmsg_can_msg needs to fit within RPMSG_MAX_PAYLOAD_SIZE */
#define RPMSG_MAX_SIZE	(RPMSG_MAX_PAYLOAD_SIZE - (IMX_RPMSG_HEAD_SIZE + 4))
struct imx_rpmsg_can_msg {
	struct imx_rpmsg_head header;
	u8 request_id;
	u8 retcode;
	u16 len;
	u8 buf[CAN_BUF_MAX];
} __packed __aligned(1);

#define imx_rpmsg_can_msg_size(s) (sizeof(struct imx_rpmsg_can_msg) -	\
				   CAN_BUF_MAX + s)

/* Timing calculations are done by the m33 firmware. The
 * clock/bittiming of can defined here is dummy.
 */
#define imx_rpmsg_can_clock (24000000)

static const struct can_bittiming_const imx_rpmsg_can_bittiming_const = {
	.name = DRV_NAME,
	.tseg1_min = 4,
	.tseg1_max = 16,
	.tseg2_min = 2,
	.tseg2_max = 8,
	.sjw_max = 4,
	.brp_min = 1,
	.brp_max = 256,
	.brp_inc = 1,
};

static const struct can_bittiming_const imx_rpmsg_can_fd_bittiming_const = {
	.name = DRV_NAME,
	.tseg1_min = 2,
	.tseg1_max = 96,
	.tseg2_min = 2,
	.tseg2_max = 32,
	.sjw_max = 16,
	.brp_min = 1,
	.brp_max = 1024,
	.brp_inc = 1,
};

static const struct can_bittiming_const imx_rpmsg_can_fd_data_bittiming_const = {
	.name = DRV_NAME,
	.tseg1_min = 2,
	.tseg1_max = 39,
	.tseg2_min = 2,
	.tseg2_max = 8,
	.sjw_max = 4,
	.brp_min = 1,
	.brp_max = 1024,
	.brp_inc = 1,
};

static inline int imx_rpmsg_can_transceiver_enable(const struct imx_rpmsg_can_priv *priv)
{
	if (!priv->reg_xceiver)
		return 0;

	return regulator_enable(priv->reg_xceiver);
}

static inline int imx_rpmsg_can_transceiver_disable(const struct imx_rpmsg_can_priv *priv)
{
	if (!priv->reg_xceiver)
		return 0;

	return regulator_disable(priv->reg_xceiver);
}

static int rpmsg_can_send(struct imx_rpmsg_can_msg *msg, void *data, size_t len)
{
	int err;

	BUILD_BUG_ON(sizeof(struct imx_rpmsg_can_msg) > RPMSG_MAX_PAYLOAD_SIZE);
	BUG_ON(len > RPMSG_MAX_SIZE);

	msg->header.cate = IMX_RPMSG_CAN;
	msg->header.major = CAN_RPMSG_MAJOR;
	msg->header.minor = CAN_RPMSG_MINOR;
	msg->header.type = CAN_RPMSG_REQUEST;

	reinit_completion(&can_rpmsg.cmd_complete);
	mutex_lock(&can_rpmsg.tx_lock);
	spin_lock_irq(&can_rpmsg.request_id_lock);
	can_rpmsg.inflight_request_id = can_rpmsg.last_request_id++;
	spin_unlock_irq(&can_rpmsg.request_id_lock);
	msg->request_id = can_rpmsg.inflight_request_id;

	if (data) {
		can_rpmsg.requested_value = data;
		can_rpmsg.requested_length = len;
	}

	err = rpmsg_send(can_rpmsg.rpdev->ept, msg,
			 imx_rpmsg_can_msg_size(msg->len));
	if (err) {
		dev_err(&can_rpmsg.rpdev->dev, "rpmsg_send failed: %d\n", err);
		goto out_unlock;
	}

	err = wait_for_completion_timeout(&can_rpmsg.cmd_complete, msecs_to_jiffies(RPMSG_TIMEOUT));
	if (!err) {
		/* don't process late replies - lock here ensures
		 * we're not completing the next call
		 */
		spin_lock_irq(&can_rpmsg.request_id_lock);
		can_rpmsg.inflight_request_id = 0xffff;
		can_rpmsg.requested_value = NULL;
		spin_unlock_irq(&can_rpmsg.request_id_lock);
		dev_err(&can_rpmsg.rpdev->dev, "rpmsg_send timeout\n");
		err = -ETIMEDOUT;
		goto out_unlock;
	}
	if (can_rpmsg.last_retcode != 0) {
		dev_err(&can_rpmsg.rpdev->dev, "rpmsg error for %d: %d\n",
			msg->header.cmd, can_rpmsg.last_retcode);
		err = -EINVAL;
		goto out_unlock;
	}
	err = 0;

out_unlock:
	mutex_unlock(&can_rpmsg.tx_lock);

	return err;
}

static int imx_rpmsg_can_open(struct net_device *dev)
{
	struct imx_rpmsg_can_priv *priv = netdev_priv(dev);
	struct imx_rpmsg_can_open_params *params;
	struct imx_rpmsg_can_msg msg = {
		.header.cmd = CAN_RPMSG_COMMAND_OPEN,
		.len = sizeof(*params),
	};
	int ret;

	if ((priv->can.ctrlmode & CAN_CTRLMODE_3_SAMPLES) &&
	    (priv->can.ctrlmode & CAN_CTRLMODE_FD)) {
		netdev_err(dev, "Three Samples mode and CAN-FD mode can't be used together\n");
		return -EINVAL;
	}

	ret = open_candev(dev);
	if (ret)
		return ret;

	ret = imx_rpmsg_can_transceiver_enable(priv);
	if (ret)
		goto out_close;

	params = (struct imx_rpmsg_can_open_params *)msg.buf;
	params->bittiming = priv->can.bittiming;
	params->data_bittiming = priv->can.data_bittiming;
	params->ctrlmode = priv->can.ctrlmode;
	ret = rpmsg_can_send(&msg, NULL, 0);
	if (ret)
		goto out_transceiver_disable;

	can_led_event(dev, CAN_LED_EVENT_OPEN);

	netif_start_queue(dev);

	return 0;

out_transceiver_disable:
	imx_rpmsg_can_transceiver_disable(priv);
out_close:
	close_candev(dev);

	return ret;
}

static int imx_rpmsg_can_close(struct net_device *dev)
{
	struct imx_rpmsg_can_priv *priv = netdev_priv(dev);
	struct imx_rpmsg_can_msg msg = {
		.header.cmd = CAN_RPMSG_COMMAND_STOP,
		.len = 0,
	};
	int ret;

	netif_stop_queue(dev);
	priv->can.state = CAN_STATE_STOPPED;

	ret = rpmsg_can_send(&msg, NULL, 0);
	if (ret)
		return ret;

	imx_rpmsg_can_transceiver_disable(priv);
	close_candev(dev);
	pm_runtime_put(priv->dev);

	can_led_event(dev, CAN_LED_EVENT_STOP);

	return 0;
}

static void imx_rpmsg_can_transmit(struct work_struct *work)
{
	struct imx_rpmsg_can_priv *priv =
		container_of(work, struct imx_rpmsg_can_priv, tx_work);
	struct net_device *dev = priv->can.dev;
	struct net_device_stats *stats = &dev->stats;
	struct sk_buff *skb;
	struct imx_rpmsg_can_msg msg = {
		.header.cmd = CAN_RPMSG_COMMAND_PAYLOAD,
	};
	int ret;

	while ((skb = skb_dequeue(&priv->txq)) != NULL) {
		struct canfd_frame *cfd = (struct canfd_frame *)skb->data;

		if (can_is_canfd_skb(skb))
			msg.len = CANFD_MTU;
		else
			msg.len = CAN_MTU;

		memcpy(msg.buf, cfd, msg.len);

		ret = rpmsg_can_send(&msg, NULL, 0);
		if (ret) {
			stats->tx_errors++;
		} else {
			stats->tx_bytes += cfd->len;
			stats->tx_packets++;
		}
	}

	netif_wake_queue(dev);

	can_led_event(dev, CAN_LED_EVENT_TX);
}

static netdev_tx_t imx_rpmsg_can_start_xmit(struct sk_buff *skb,
					    struct net_device *dev)
{
	struct imx_rpmsg_can_priv *priv = netdev_priv(dev);

	if (can_dropped_invalid_skb(dev, skb))
		return NETDEV_TX_OK;

	/* ndo_start_xmit() called from atomic context. */
	skb_queue_tail(&priv->txq, skb);

	/* We're busy transmitting a packet... */
	netif_stop_queue(dev);
	schedule_work(&priv->tx_work);

	return NETDEV_TX_OK;
}

static int imx_rpmsg_can_set_mode(struct net_device *dev, enum can_mode mode)
{
	struct imx_rpmsg_can_msg msg = {
		.header.cmd = CAN_RPMSG_COMMAND_RESTART,
		.len = 0,
	};
	int ret;

	switch (mode) {
	case CAN_MODE_START:
		ret = rpmsg_can_send(&msg, NULL, 0);
		if (ret)
			return ret;

		netif_wake_queue(dev);
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static int imx_rpmsg_can_get_berr_counter(const struct net_device *dev,
					  struct can_berr_counter *bec)
{
	struct imx_rpmsg_can_msg msg = {
		.header.cmd = CAN_RPMSG_COMMAND_GET_STATUS,
		.len = 0,
	};
	u32 ecr;
	int ret;

	ret = rpmsg_can_send(&msg, &ecr, sizeof(ecr));
	if (ret)
		return ret;

	bec->txerr = (ecr >> 0) & 0xff;
	bec->rxerr = (ecr >> 8) & 0xff;

	return 0;
}

static int imx_rpmsg_can_init_remote(void)
{
	struct imx_rpmsg_can_init_params *params;
	struct imx_rpmsg_can_msg msg = {
		.header.cmd = CAN_RPMSG_COMMAND_INIT,
		.len = sizeof(*params),
	};
	struct device *device = can_rpmsg.priv->dev;
	struct device_node *np = device->of_node;
	int ret;

	params = (struct imx_rpmsg_can_init_params *)msg.buf;

	ret = of_property_read_u32(np, "suspend_wakeup_gpio",
				   &params->suspend_wakeup_gpio);
	if (ret < 0) {
		dev_info(device, "%pOF: no suspend wakeup gpio set, will not wake up\n",
			 np);
		/* setting wakeup will error out m33-side and fail suspend */
		params->suspend_wakeup_gpio = -1;
	}

	return rpmsg_can_send(&msg, NULL, 0);
}

static const struct net_device_ops imx_rpmsg_can_netdev_ops = {
	.ndo_open	= imx_rpmsg_can_open,
	.ndo_stop	= imx_rpmsg_can_close,
	.ndo_start_xmit	= imx_rpmsg_can_start_xmit,
	.ndo_change_mtu = can_change_mtu,
};

static int imx_rpmsg_can_probe(struct platform_device *pdev)
{
	struct net_device *dev;
	struct imx_rpmsg_can_priv *priv;
	struct regulator *reg_xceiver;
	int ret;

	BUILD_BUG_ON(RPMSG_MAX_SIZE < CAN_MTU_MAX);

	if (!can_rpmsg.rpdev)
		return -EPROBE_DEFER;

	reg_xceiver = devm_regulator_get_optional(&pdev->dev, "xceiver");
	if (PTR_ERR(reg_xceiver) == -EPROBE_DEFER)
		return -EPROBE_DEFER;
	else if (PTR_ERR(reg_xceiver) == -ENODEV)
		reg_xceiver = NULL;
	else if (IS_ERR(reg_xceiver))
		return PTR_ERR(reg_xceiver);

	dev = alloc_candev(sizeof(struct imx_rpmsg_can_priv), 1);
	if (!dev)
		return -ENOMEM;

	platform_set_drvdata(pdev, dev);
	SET_NETDEV_DEV(dev, &pdev->dev);

	dev->netdev_ops = &imx_rpmsg_can_netdev_ops;
	dev->flags |= IFF_ECHO;
	dev->features |= NETIF_F_LLTX;

	priv = netdev_priv(dev);

	priv->dev = &pdev->dev;
	priv->can.clock.freq = imx_rpmsg_can_clock;
	priv->can.do_set_mode = imx_rpmsg_can_set_mode;
	priv->can.do_get_berr_counter = imx_rpmsg_can_get_berr_counter;
	priv->can.ctrlmode_supported = CAN_CTRLMODE_LOOPBACK |
		CAN_CTRLMODE_LISTENONLY	| CAN_CTRLMODE_BERR_REPORTING;
	if (IS_ENABLED(CONFIG_CAN_IMX_RPMSG_SUPPORT_FD)) {
		priv->can.ctrlmode_supported |= CAN_CTRLMODE_FD |
			CAN_CTRLMODE_FD_NON_ISO;
		priv->can.bittiming_const = &imx_rpmsg_can_fd_bittiming_const;
		priv->can.data_bittiming_const =
			&imx_rpmsg_can_fd_data_bittiming_const;
	} else {
		priv->can.bittiming_const = &imx_rpmsg_can_bittiming_const;
	}
	priv->reg_xceiver = reg_xceiver;
	can_rpmsg.priv = priv;

	skb_queue_head_init(&priv->txq);
	INIT_WORK(&priv->tx_work, imx_rpmsg_can_transmit);

	ret = imx_rpmsg_can_init_remote();
	if (ret)
		goto out;

	ret = register_candev(dev);
	if (ret) {
		dev_err(&pdev->dev, "registering candev failed\n");
		goto out;
	}

	device_set_wakeup_capable(&pdev->dev, true);

	return 0;

out:
	free_candev(dev);
	return ret;
}

static int imx_rpmsg_can_remove(struct platform_device *pdev)
{
	struct net_device *dev = dev_get_drvdata(&pdev->dev);

	device_set_wakeup_enable(&pdev->dev, false);
	device_set_wakeup_capable(&pdev->dev, false);

	unregister_candev(dev);
	free_candev(dev);

	return 0;
}

static int __maybe_unused imx_rpmsg_can_suspend(struct device *device)
{
	struct net_device *dev = dev_get_drvdata(device);
	bool *enable;
	struct imx_rpmsg_can_msg msg = {
		.header.cmd = CAN_RPMSG_COMMAND_SET_WAKE,
		.len = sizeof(*enable),
	};

	enable = (bool *)msg.buf;
	*enable = device_may_wakeup(device);

	if (netif_running(dev)) {
		netif_stop_queue(dev);
		netif_device_detach(dev);
	}

	return rpmsg_can_send(&msg, NULL, 0);
}

static int __maybe_unused imx_rpmsg_can_resume(struct device *device)
{
	struct net_device *dev = dev_get_drvdata(device);

	if (netif_running(dev)) {
		netif_device_attach(dev);
		netif_start_queue(dev);
	}

	return 0;
}

static SIMPLE_DEV_PM_OPS(imx_rpmsg_can_pm_ops, imx_rpmsg_can_suspend,
			 imx_rpmsg_can_resume);

static const struct of_device_id imx_rpmsg_can_dt_ids[] = {
	{ .compatible = "fsl,imx-rpmsg-can" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_rpmsg_can_dt_ids);

static struct platform_driver imx_rpmsg_can_driver = {
	.driver	= {
		.name = DRV_NAME,
		.pm		= &imx_rpmsg_can_pm_ops,
		.of_match_table = imx_rpmsg_can_dt_ids,
	},
	.probe = imx_rpmsg_can_probe,
	.remove = imx_rpmsg_can_remove,
};

static int can_rpmsg_cb(struct rpmsg_device *rpdev, void *data, int len,
			void *priv, u32 src)
{
	struct imx_rpmsg_can_msg *msg = data;
	struct canfd_frame *cfd;
	struct sk_buff *skb;
	struct net_device *dev = can_rpmsg.priv->can.dev;
	struct net_device_stats *stats = &dev->stats;

	dev_dbg(&rpdev->dev, "msg(<- src 0x%x) len %d\n", src, len);
	print_hex_dump_debug(__func__, DUMP_PREFIX_NONE, 16, 1,
			     data, len,	 true);

	if (msg->header.type == CAN_RPMSG_RESPONSE) {
		// ack for baudrate or tx
		spin_lock_irq(&can_rpmsg.request_id_lock);
		if (msg->request_id != can_rpmsg.inflight_request_id) {
			// non-wait message are always 0xffff
			if (can_rpmsg.inflight_request_id != 0xffff)
				dev_warn(&rpdev->dev,
					 "received unexpected id %x (expected %x)\n",
					 msg->request_id, can_rpmsg.inflight_request_id);
			spin_unlock_irq(&can_rpmsg.request_id_lock);
			return 0;
		}
		/* don't process duplicates */
		can_rpmsg.inflight_request_id = 0xffff;
		can_rpmsg.last_retcode = msg->retcode;
		if (can_rpmsg.requested_value) {
			if (can_rpmsg.requested_length == msg->len)
				memcpy(can_rpmsg.requested_value, msg->buf, msg->len);
			else
				can_rpmsg.last_retcode = -EPROTO;
			can_rpmsg.requested_value = NULL;
		}
		spin_unlock_irq(&can_rpmsg.request_id_lock);

		complete(&can_rpmsg.cmd_complete);
		return 0;
	}
	if (msg->header.type != CAN_RPMSG_NOTIFICATION) {
		/* invalid message */
		return 0;
	}
	/* XXX check length, version... */

	spin_lock_bh(&can_rpmsg.rx_lock);

	if (can_rpmsg.priv->can.ctrlmode & CAN_CTRLMODE_FD)
		skb = alloc_canfd_skb(can_rpmsg.priv->can.dev, &cfd);
	else
		skb = alloc_can_skb(can_rpmsg.priv->can.dev, (struct can_frame **)&cfd);
	if (unlikely(!skb)) {
		stats->rx_dropped++;
		spin_unlock_bh(&can_rpmsg.rx_lock);

		return -ENOMEM;
	}

	memcpy(cfd, msg->buf, msg->len);

	spin_unlock_bh(&can_rpmsg.rx_lock);

	stats->rx_packets++;
	stats->rx_bytes += cfd->len;

	netif_receive_skb(skb);

	can_led_event(dev, CAN_LED_EVENT_RX);

	return 0;
}

static int can_rpmsg_probe(struct rpmsg_device *rpdev)
{
	init_completion(&can_rpmsg.cmd_complete);
	mutex_init(&can_rpmsg.tx_lock);
	spin_lock_init(&can_rpmsg.rx_lock);
	spin_lock_init(&can_rpmsg.request_id_lock);

	can_rpmsg.rpmsg_ack_wq = create_workqueue("imx_rpmsg_can_workqueue");
	if (!can_rpmsg.rpmsg_ack_wq) {
		dev_err(&rpdev->dev, "Failed to create imx_rpmsg_can_workqueue");
		return -ENOMEM;
	}

	can_rpmsg.rpdev = rpdev;

	dev_info(&rpdev->dev, "new channel: 0x%x -> 0x%x!\n",
		 rpdev->src, rpdev->dst);

	return 0;
}

static void can_rpmsg_remove(struct rpmsg_device *rpdev)
{
	can_rpmsg.rpdev = NULL;
	dev_info(&rpdev->dev, "can channel removed\n");
}

static struct rpmsg_device_id can_rpmsg_id_table[] = {
	{ .name = "rpmsg-can-channel" },
	{ },
};

static struct rpmsg_driver can_rpmsg_driver = {
	.drv.name	= KBUILD_MODNAME,
	.drv.owner	= THIS_MODULE,
	.id_table	= can_rpmsg_id_table,
	.probe		= can_rpmsg_probe,
	.callback	= can_rpmsg_cb,
	.remove		= can_rpmsg_remove,
};

static int __init imx_rpmsg_can_init(void)
{
	int ret;

	ret = register_rpmsg_driver(&can_rpmsg_driver);
	if (ret < 0)
		return ret;

	return platform_driver_register(&imx_rpmsg_can_driver);
}
module_init(imx_rpmsg_can_init);

MODULE_AUTHOR("Atmark Techno, Inc.");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("i.MX RPMSG can driver");
