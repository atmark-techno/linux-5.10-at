// SPDX-License-Identifier: GPL-2.0-only
/**
 * Copyright (C) 2024 Atmark Techno
 *
 * Driver for imx rpmsg
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
#include <linux/watchdog.h>


#define WDOG_RPMSG_TIMEOUT_MS                    500
#define WDOG_RPMSG_VERSION                       0x0001

#define WDOG_RPMSG_DEFAULT_TIME			60

// module param, defined later
static bool disable;
module_param(disable, bool, 0);
MODULE_PARM_DESC(timeout, "don't enable watchdog on module load");

static unsigned timeout;
module_param(timeout, uint, 0);
MODULE_PARM_DESC(timeout, "Watchdog timeout in seconds (default="
                                __MODULE_STRING(WDOG_RPMSG_DEFAULT_TIME) ")");

enum srtm_message_type
{
    SRTM_MessageTypeRequest = 0x00U,  /*!< Request message */
    SRTM_MessageTypeResponse,         /*!< Response message for certain Request */
    SRTM_MessageTypeNotification,     /*!< Notification message that doesn't require response */
    SRTM_MessageTypeCommLast,         /*!< Last value of communication message */

    SRTM_MessageTypeProcedure = 0x40, /*!< Local procedure */
    SRTM_MessageTypeRawData   = 0x41, /*!< Raw data message */
};

enum wdog_rpmsg_command {
	WDOG_RPMSG_ENABLE = 0,
	WDOG_RPMSG_PING,
	WDOG_RPMSG_NOTIFY,
};

enum wdog_rpmsg_retcode {
	WDOG_RPMSG_SUCCESS = 0,
	WDOG_RPMSG_ERROR,
	WDOG_RPMSG_UNSUPPORTED,
};

struct wdog_rpmsg_msg {
	struct imx_rpmsg_head header;
	u8 request_id;

	union {
		struct {
			bool enabled;
			u16 timeout;
		} __packed enable;
		// ping: no arg
		// notify: no arg
		struct {
			u8 retcode; // enum wdog_rpmsg_retcode
		} reply;
	};
} __packed __aligned(1);

struct imx_rpmsg_wdog {
	struct watchdog_device wdog;
	struct rpmsg_device *rpdev;
	struct completion cmd_complete;
	struct mutex lock;

	bool started;
	u8 last_retcode;
	u8 last_request_id;
	u16 inflight_request_id;
	spinlock_t request_id_lock;
};

/* rpmsg callback has a void *priv but it is not settable
 * when invoked with rpmsg_driver probe, so we need a global...
 */
static struct imx_rpmsg_wdog *wdog_rpmsg;

static int rpmsg_wdog_send_and_wait(struct imx_rpmsg_wdog *rpwdog,
				    struct wdog_rpmsg_msg *msg,
				    bool wait)
{
	int ret;

	if (!rpwdog || !rpwdog->rpdev)
		return -EINVAL;

	msg->header.cate = IMX_RPMSG_WDOG;
	msg->header.major = WDOG_RPMSG_VERSION;
	msg->header.minor = WDOG_RPMSG_VERSION >> 8;
	msg->header.type = SRTM_MessageTypeRequest;

	mutex_lock(&rpwdog->lock);
	reinit_completion(&rpwdog->cmd_complete);
	/* We need the spin lock to ensure rpmsg cb does not set last_error
	 * after this timed out & another request started being processed */
	spin_lock_irq(&rpwdog->request_id_lock);
	rpwdog->inflight_request_id = rpwdog->last_request_id++;
	spin_unlock_irq(&rpwdog->request_id_lock);
	msg->request_id = rpwdog->inflight_request_id;
	if (!wait)
		rpwdog->inflight_request_id = 0xffff;

	ret = rpmsg_send(rpwdog->rpdev->ept, msg, sizeof(*msg));
	if (ret < 0) {
		dev_err(&rpwdog->rpdev->dev, "rpmsg_send failed %d\n", ret);
		mutex_unlock(&rpwdog->lock);
		return ret;
	}
	if (!wait) {
		mutex_unlock(&rpwdog->lock);
		return 0;
	}
	ret = wait_for_completion_timeout(&rpwdog->cmd_complete,
					  msecs_to_jiffies(WDOG_RPMSG_TIMEOUT_MS));
	mutex_unlock(&rpwdog->lock);
	if (!ret) {
		/* don't process late replies - lock here ensures we're not completing
		 * the next call */
		spin_lock_irq(&rpwdog->request_id_lock);
		rpwdog->inflight_request_id = 0xffff;
		spin_unlock_irq(&rpwdog->request_id_lock);
		dev_err(&rpwdog->rpdev->dev, "rpmsg reply timeout\n");
		return -ETIMEDOUT;
	}
	if (rpwdog->last_retcode != WDOG_RPMSG_SUCCESS) {
		dev_err(&rpwdog->rpdev->dev, "rpmsg request failed? %d\n",
			rpwdog->last_retcode);
		return -EIO;
	}
	return 0;
}

static int rpmsg_wdog_enable(struct imx_rpmsg_wdog *rpwdog, bool wait)
{
	struct wdog_rpmsg_msg msg = {
		.header.cmd = WDOG_RPMSG_ENABLE,
	};

	if (!rpwdog)
		return -EINVAL;

	/* convert timeout in second to timeout in ms expected by driver */
	msg.enable.timeout = rpwdog->wdog.timeout * 1000;
	msg.enable.enabled = rpwdog->started;

	return rpmsg_wdog_send_and_wait(rpwdog, &msg, wait);
}


static int wdog_rpmsg_cb(struct rpmsg_device *rpdev, void *data, int len,
			void *priv, u32 src)
{
	struct wdog_rpmsg_msg *msg = data;

	if (!wdog_rpmsg || !wdog_rpmsg->rpdev) {
		dev_err(&rpdev->dev, "Ignoring message before init\n");
		return -EINVAL;
	}
	if (msg->header.type == SRTM_MessageTypeNotification
	    && msg->header.cmd == WDOG_RPMSG_NOTIFY) {
		// should normally use WDIOF_PRETIMEOUT, just error for now
		dev_err(&rpdev->dev, "Got notify from rpmsg rpwdog, aborting! will reboot!\n");
		wdog_rpmsg->started = false;
		return 0;
	}
	if (msg->header.type != SRTM_MessageTypeResponse) {
		dev_err(&rpdev->dev, "bad type %x\n", msg->header.type);
		return -EINVAL;
	}
	spin_lock_irq(&wdog_rpmsg->request_id_lock);
	if (msg->request_id != wdog_rpmsg->inflight_request_id) {
		dev_dbg(&rpdev->dev, "bad id %x (expected %x)\n",
			msg->request_id, wdog_rpmsg->inflight_request_id);
		spin_unlock_irq(&wdog_rpmsg->request_id_lock);
		return 0;
	}
	/* don't process duplicates */
	wdog_rpmsg->inflight_request_id = 0xffff;

	wdog_rpmsg->last_retcode = msg->reply.retcode;
	spin_unlock_irq(&wdog_rpmsg->request_id_lock);

	complete(&wdog_rpmsg->cmd_complete);

	return 0;
}

static int imx_rpmsg_wdt_ping(struct watchdog_device *wdog)
{
        struct imx_rpmsg_wdog *rpwdog = watchdog_get_drvdata(wdog);
	struct wdog_rpmsg_msg msg = {
		.header.cmd = WDOG_RPMSG_PING,
	};
	int rc;

	// skip if disabled
	if (!rpwdog->started)
		return 0;

	// consider us dead if this fails: don't send further pings
	rc = rpmsg_wdog_send_and_wait(rpwdog, &msg, true);
	if (rc) {
		dev_err(&rpwdog->rpdev->dev,
			"ping failed: %d; aborting further pings: will reboot!\n",
			rc);
		rpwdog->started = false;
	}
	return rc;
}


static int imx_rpmsg_wdt_start(struct watchdog_device *wdog)
{
        struct imx_rpmsg_wdog *rpwdog = watchdog_get_drvdata(wdog);
	int rc;

	/* already started */
	if (rpwdog->started)
		return -EBUSY;

	dev_info(&rpwdog->rpdev->dev, "enabling rpmsg watchdog\n");
	rc = rpmsg_wdog_enable(rpwdog, true);
	if (rc)
		return rc;

	rpwdog->started = true;
        set_bit(WDOG_HW_RUNNING, &wdog->status);

        return 0;
}

static int imx_rpmsg_wdt_set_timeout(struct watchdog_device *wdog,
				     unsigned int new_timeout)
{
        struct imx_rpmsg_wdog *rpwdog = watchdog_get_drvdata(wdog);

	if (!rpwdog->started)
		return -EBUSY;

	dev_info(&rpwdog->rpdev->dev, "new timeout %d\n", new_timeout);
	if (new_timeout > wdog->max_timeout)
		wdog->timeout = wdog->max_timeout;
	else
		wdog->timeout = new_timeout;
	return rpmsg_wdog_enable(rpwdog, true);
}


static const struct watchdog_info imx_rpmsg_wdt_info = {
        .identity = "imx rpmsg watchdog",
        .options = WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING | WDIOF_MAGICCLOSE,
};

static const struct watchdog_ops imx_rpmsg_wdt_ops = {
	.owner = THIS_MODULE,
	.start = imx_rpmsg_wdt_start,
	.ping = imx_rpmsg_wdt_ping,
	.set_timeout = imx_rpmsg_wdt_set_timeout,
};

static int wdog_rpmsg_probe(struct rpmsg_device *rpdev)
{
	struct imx_rpmsg_wdog *rpwdog;
	struct watchdog_device *wdog;
	int rc;

	if (wdog_rpmsg) {
		// reinit
		wdog_rpmsg->rpdev = rpdev;
		return 0;
	}

	rpwdog = devm_kzalloc(&rpdev->dev, sizeof(*rpwdog), GFP_KERNEL);

	rpwdog->rpdev = rpdev;
	init_completion(&rpwdog->cmd_complete);
	mutex_init(&rpwdog->lock);
	spin_lock_init(&rpwdog->request_id_lock);

	wdog = &rpwdog->wdog;
	wdog->info = &imx_rpmsg_wdt_info;
	wdog->ops = &imx_rpmsg_wdt_ops;
	wdog->min_timeout = 1;
	wdog->max_timeout = 0xffffU / 1000;
	wdog->timeout = WDOG_RPMSG_DEFAULT_TIME;
	wdog->max_hw_heartbeat_ms = WDOG_RPMSG_DEFAULT_TIME * 1000;
	wdog->parent = &rpdev->dev;

	dev_set_drvdata(&rpdev->dev, wdog);
	watchdog_set_drvdata(wdog, rpwdog);
	watchdog_init_timeout(wdog, timeout, &rpdev->dev);
	watchdog_stop_ping_on_suspend(wdog);
	wdog_rpmsg = rpwdog;

	if (!disable) {
		// async start - can't wait for reply in probe()
		rpwdog->started = true;
		rc = rpmsg_wdog_enable(rpwdog, false);
		if (rc)
			return rc;
		set_bit(WDOG_HW_RUNNING, &wdog->status);
	}

	return devm_watchdog_register_device(&rpdev->dev, wdog);
}

static void wdog_rpmsg_remove(struct rpmsg_device *rpdev)
{
	dev_info(&rpdev->dev, "removed wdog channel\n");
	wdog_rpmsg->rpdev = NULL;
}

static struct rpmsg_device_id wdog_rpmsg_id_table[] = {
	{ .name = "rpmsg-wdog-channel" },
	{},
};

static struct rpmsg_driver wdog_rpmsg_driver = {
	.drv.name = "wdog_rpmsg",
	.drv.owner = THIS_MODULE,
	.id_table = wdog_rpmsg_id_table,
	.probe = wdog_rpmsg_probe,
	.callback = wdog_rpmsg_cb,
	.remove = wdog_rpmsg_remove,
};

static int __init imx_rpmsg_wdog_init(void)
{
	return register_rpmsg_driver(&wdog_rpmsg_driver);
}
device_initcall(imx_rpmsg_wdog_init);

MODULE_AUTHOR("Dominique Martinet <dominique.martinet@atmark-techno.com>");
MODULE_DESCRIPTION("IMX RPMSG WDOG driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
