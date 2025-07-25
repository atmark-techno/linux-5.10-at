/** @file moal_main.h
 *
 * @brief This file contains wlan driver specific defines etc.
 *
 *
 * Copyright 2008-2025 NXP
 *
 * This software file (the File) is distributed by NXP
 * under the terms of the GNU General Public License Version 2, June 1991
 * (the License).  You may use, redistribute and/or modify the File in
 * accordance with the terms and conditions of the License, a copy of which
 * is available by writing to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or on the
 * worldwide web at http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt.
 *
 * THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE
 * ARE EXPRESSLY DISCLAIMED.  The License provides additional details about
 * this warranty disclaimer.
 *
 */

/********************************************************
Change log:
    10/21/2008: initial version
********************************************************/

#ifndef _MOAL_MAIN_H
#define _MOAL_MAIN_H

/* warnfix for FS redefination if any? */
#ifdef FS
#undef FS
#endif

/* Linux header files */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/version.h>
#include <linux/param.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/ip.h>
#include <linux/ipv6.h>
#include <linux/hashtable.h>
#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 10, 17)
#include <uapi/linux/sched/types.h>
#endif
#include <linux/timer.h>
#include <linux/ioport.h>
#include <linux/pci.h>
#include <linux/ctype.h>
#include <linux/proc_fs.h>
#include <linux/vmalloc.h>
#include <linux/ptrace.h>
#include <linux/string.h>
#include <linux/irqreturn.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 1, 0)
#include <linux/namei.h>
#include <linux/fs.h>
#endif
#include <linux/of.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 18)
#include <linux/config.h>
#endif

#ifdef USB
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 22)
#include <linux/freezer.h>
#endif
#include <linux/usb.h>
#endif /* USB */

/* ASM files */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)
#include <linux/semaphore.h>
#else
#include <asm/semaphore.h>
#endif
#include <asm/byteorder.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 4, 0)
#include <asm/switch_to.h>
#else
#include <asm/system.h>
#endif

#include <linux/spinlock.h>

/* Net header files */
#include <linux/netdevice.h>
#include <linux/net.h>
#include <linux/inet.h>
#include <linux/ip.h>
#include <linux/skbuff.h>
#include <linux/if_arp.h>
#include <linux/if_ether.h>
#include <linux/etherdevice.h>
#include <net/sock.h>
#include <net/arp.h>
#include <linux/rtnetlink.h>
#include <linux/inetdevice.h>

#include <linux/firmware.h>

#ifdef ANDROID_KERNEL
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 1, 0)
#include <linux/pm_wakeup.h>
#include <linux/device.h>
#else
#include <linux/wakelock.h>
#endif
#endif

#include <net/ieee80211_radiotap.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 6, 0)
#include <net/netdev_rx_queue.h>
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 9, 0)
#include <net/rps.h>
#endif

#include "mlan.h"
#include "moal_shim.h"
/* Wireless header */
#if defined(STA_CFG80211) || defined(UAP_CFG80211)
#if LINUX_VERSION_CODE <= KERNEL_VERSION(6, 12, 12)
#include <net/lib80211.h>
#endif
#include <net/cfg80211.h>
#include <net/ieee80211_radiotap.h>
#endif
#if defined(STA_WEXT) || defined(UAP_WEXT)
#include <linux/wireless.h>
#include <net/iw_handler.h>
#include "moal_wext.h"
#endif
#ifdef STA_WEXT
#include "moal_priv.h"
#endif

#ifdef IMX_SUPPORT
#include <linux/of_irq.h>
#include <linux/suspend.h>
#endif /* IMX_SUPPORT */

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 2, 0)
#include <linux/pm_qos.h>
#else
#include <linux/pm_qos_params.h>
#endif

#ifndef MIN
/** Find minimum */
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

/** Find maximum */
#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif

#define COMPAT_VERSION_CODE KERNEL_VERSION(0, 0, 0)
#define CFG80211_VERSION_CODE MAX(LINUX_VERSION_CODE, COMPAT_VERSION_CODE)

#define IMX_ANDROID_13 0
#define IMX_ANDROID_14 0
#define IMX_ANDROID_12_BACKPORT 0

#ifdef ANDROID_SDK_VERSION
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 15, 52)
#undef IMX_ANDROID_13
#define IMX_ANDROID_13 1
#ifdef ANDROID_14_SUPPORT
#undef IMX_ANDROID_14
#define IMX_ANDROID_14 1
#endif
#endif
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 15, 41)
#undef IMX_ANDROID_12_BACKPORT
#define IMX_ANDROID_12_BACKPORT 1
#endif
#else
#endif

/**
 * Reason Code 3: STA is leaving (or has left) IBSS or ESS
 */
#define DEF_DEAUTH_REASON_CODE (0x3)

/**
 * 802.1 Local Experimental 1.
 */

#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 24)
#define REFDATA __refdata
#else
#define REFDATA
#endif

/**
 * Linux Kernels later 3.9 use CONFIG_PM_RUNTIME instead of
 * CONFIG_USB_SUSPEND
 * Linux Kernels later 3.19 use CONFIG_PM instead of
 * CONFIG_PM_RUNTIME
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 19, 0)
#ifdef CONFIG_PM
#ifndef CONFIG_USB_SUSPEND
#define CONFIG_USB_SUSPEND
#endif
#ifndef CONFIG_PM_RUNTIME
#define CONFIG_PM_RUNTIME
#endif
#endif
#else /* LINUX_VERSION_CODE >= KERNEL_VERSION(3, 19, 0) */
#ifdef CONFIG_PM_RUNTIME
#ifndef CONFIG_USB_SUSPEND
#define CONFIG_USB_SUSPEND
#endif
#endif
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(3, 19, 0) */
#endif

/**
 * Linux kernel later 3.10 use strncasecmp instead of strnicmp
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 0, 0)
#define strnicmp strncasecmp
#endif

/**
 * Linux kernel later 4.7 use nl80211_band instead of ieee80211_band
 * Linux kernel later 4.7 use new macro
 */
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 7, 0)
#define ieee80211_band nl80211_band
#define IEEE80211_BAND_2GHZ NL80211_BAND_2GHZ
#define IEEE80211_BAND_5GHZ NL80211_BAND_5GHZ
#define IEEE80211_NUM_BANDS NUM_NL80211_BANDS
#endif

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 4, 0)
#define IEEE80211_BAND_6GHZ NL80211_BAND_6GHZ
#endif

/**
 * interface name
 */
#define default_mlan_name "mlan%d"
#define default_uap_name "uap%d"
#define default_wfd_name "wfd%d"
#define default_nan_name "nan%d"
#define default_mpl_name "mpl%d"
#define default_11p_name "ocb%d"
#define default_dfs_name "dfs%d"
#define mwiphy_name "mwiphy%d"

/** country txpower mode */
#define CNTRY_TXPOWER_MODE 1
/** country rgpower mode */
#define CNTRY_RGPOWER_MODE 2

#define DEF_NICE 20
/** Define BOOLEAN */
typedef t_u8 BOOLEAN;

#define INTF_CARDTYPE "----------%s-MM"

#define KERN_VERSION "6X"

#define V14 "14"
#define V15 "15"
#define V16 "16"
#define V17 "17"
#define V18 "18"

/** Chip Magic Value */
#define CHIP_MAGIC_VALUE 0x24
/** card type SD_UART */
#define CARD_TYPE_SD_UART 0
/** card type SD_SD */
#define CARD_TYPE_SD_SD 1
/** card type PCIE_PCIE */
#define CARD_TYPE_PCIE_PCIE 2
/** card type PCIE_UART */
#define CARD_TYPE_PCIE_UART 3
/** card type USB_UART */
#define CARD_TYPE_USB_UART 4
/** card type USB_USB */
#define CARD_TYPE_USB_USB 6
/** card type PCIE_USB */
#define CARD_TYPE_PCIE_USB 7
#ifdef SDAW693
/** card type SDAW693_UART */
#define CARD_TYPE_SDAW693_UART 1 // As per datasheet/SoC design
#endif
/** card type SD9177_UART */
#define CARD_TYPE_SD9177_UART 1 // As per datasheet/SoC design
/** card type SDIW624_UARTSPI */
#define CARD_TYPE_SDIW624_UARTSPI 0 // As per datasheet/SoC design
/** card type SDIW624_UARTUART */
#define CARD_TYPE_SDIW624_UARTUART 2 // As per datasheet/SoC design
/** card type PCIEIW624_USBUSB */
#define CARD_TYPE_PCIEIW624_USBUSB 4 // As per datasheet/SoC design
/** card type PCIEIW624_UARTUART */
#define CARD_TYPE_PCIEIW624_UARTUART 7 // As per datasheet/SoC design
/** card type PCIEIW624_UARTSPI */
#define CARD_TYPE_PCIEIW624_UARTSPI 5 // As per datasheet/SoC design
/** card type SDIW615_sd_uart_spi **/
#ifdef SDIW610
#define CARD_TYPE_SDIW610_UART                                                 \
	1 // As per datasheet/SoC design - Nighthawk, sd-uart strap = 0x3
#endif
#ifdef USBIW610
#define CARD_TYPE_USBIW610_USB                                                 \
	5 // As per datasheet/SoC design - Nighthawk, usb-usb strap = 0x5
#define CARD_TYPE_USBIW610_UART                                                \
	7 // As per datasheet/SoC design - Nighthawk, usb-uart strap = 0x7
#endif

/* Max buffer size */
#define MAX_BUF_LEN 512

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 38)
/* Max hostname length */
#define MAX_HOSTNAME_LEN 128

/* Max rtc time length */
#define MAX_TIME_LEN 128
#endif

/** Driver version */
extern char driver_version[MLAN_MAX_VER_STR_LEN];

extern struct semaphore AddRemoveCardSem;
extern int wifi_status;
extern int max_tx_buf;
extern int pcie_int_mode;

#ifdef STA_SUPPORT
#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 29)
extern const struct net_device_ops woal_netdev_ops;
#endif
#endif

#ifdef UAP_SUPPORT
#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 29)
extern const struct net_device_ops woal_uap_netdev_ops;
#endif
#endif

/** Global veriable for usb independent reset */
extern int fw_reload;

#ifdef MFG_CMD_SUPPORT
/** Mfg mode */
extern int mfg_mode;
#endif

/** rf_test mode */
extern int rf_test_mode;

#if defined(STA_CFG80211) || defined(UAP_CFG80211)
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 14, 0)
extern int fw_region;
#endif
#endif

#if defined(USB)
extern int skip_fwdnld;
#endif
#if defined(SDIO) || defined(PCIE)
typedef enum {
	RDWR_STATUS_SUCCESS = 0,
	RDWR_STATUS_FAILURE = 1,
	RDWR_STATUS_DONE = 2
} rdwr_status;
#endif

/** Private structure for MOAL */
typedef struct _moal_private moal_private, *pmoal_private;
/** Handle data structure for MOAL  */
typedef struct _moal_handle moal_handle, *pmoal_handle;

/** Hardware status codes */
typedef enum _MOAL_HARDWARE_STATUS {
	HardwareStatusReady,
	HardwareStatusInitializing,
	HardwareStatusFwReady,
	HardwareStatusReset,
	HardwareStatusClosing,
	HardwareStatusNotReady
} MOAL_HARDWARE_STATUS;

#define WIFI_STATUS_OK 0
#define WIFI_STATUS_FW_DNLD 1
#define WIFI_STATUS_FW_DNLD_COMPLETE 2
#define WIFI_STATUS_INIT_FW 3
#define WIFI_STATUS_DNLD_FW_FAIL 4
#define WIFI_STATUS_INIT_FW_FAIL 5
#define WIFI_STATUS_TX_TIMEOUT 6
#define WIFI_STATUS_WIFI_HANG 7
#define WIFI_STATUS_SCAN_TIMEOUT 8
#define WIFI_STATUS_FW_DUMP 9
#define WIFI_STATUS_FW_RELOAD 10
#define WIFI_STATUS_FW_RECOVERY_FAIL 11

/** fw cap info 11p */
#define FW_CAPINFO_80211P MBIT(24)
/** fw cap info bit26 for 0-DFS support */
#define FW_CAPINFO_ZERO_DFS MBIT(31)
/** fw cap info disable nan */
#define FW_CAPINFO_DISABLE_NAN MBIT(29)
/** fw cap info BGA */
#define FW_CAPINFO_80211BGA (MBIT(8) | MBIT(9) | MBIT(10))

/** moal_wait_option */
enum { MOAL_NO_WAIT, MOAL_IOCTL_WAIT, MOAL_IOCTL_WAIT_TIMEOUT };

/** moal_main_state */
enum {
	MOAL_STATE_IDLE,
	MOAL_RECV_INT,
	MOAL_ENTER_WORK_QUEUE,
	MOAL_START_MAIN_PROCESS,
	MOAL_END_MAIN_PROCESS
};

/** HostCmd_Header */
typedef struct _HostCmd_Header {
	/** Command */
	t_u16 command;
	/** Size */
	t_u16 size;
} HostCmd_Header;

/*
 * OS timer specific
 */

/** Timer structure */
typedef struct _moal_drv_timer {
	/** Timer list */
	struct timer_list tl;
	/** Timer function */
	void (*timer_function)(void *context);
	/** Timer function context */
	void *function_context;
	/** Time period */
	t_u32 time_period;
	/** Is timer periodic ? */
	t_u32 timer_is_periodic;
	/** Is timer cancelled ? */
	t_u32 timer_is_canceled;
} moal_drv_timer, *pmoal_drv_timer;

/** moal_802_11_action header */
typedef struct {
	/** Frame Cotrol */
	t_u16 frame_control;
	/** Duration */
	t_u16 duration;
	/** dest addr */
	t_u8 da[ETH_ALEN];
	/** source addr */
	t_u8 sa[ETH_ALEN];
	/** bssid */
	t_u8 bssid[ETH_ALEN];
	/** seq_ctrl */
	t_u16 seq_ctrl;
	/** category */
	t_u8 category;
} __attribute__((packed)) moal_802_11_action_header;

/**
 *  @brief Timer handler
 *
 *  @param fcontext	Timer context
 *
 *  @return		N/A
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 15, 0)
static inline void woal_timer_handler(struct timer_list *t)
{
	// Coverity violation raised for kernel's API
	// coverity[cert_arr39_c_violation:SUPPRESS]
	pmoal_drv_timer timer = from_timer(timer, t, tl);
#else
static inline void woal_timer_handler(unsigned long fcontext)
{
	pmoal_drv_timer timer = (pmoal_drv_timer)fcontext;
#endif

	if (!timer->timer_is_canceled)
		timer->timer_function(timer->function_context);

	if (timer->timer_is_periodic == MTRUE && !timer->timer_is_canceled) {
		mod_timer(&timer->tl,
			  jiffies + ((timer->time_period * HZ) / 1000));
	} else {
		timer->time_period = 0;
	}
}

/**
 *  @brief Initialize timer
 *
 *  @param timer		Timer structure
 *  @param TimerFunction	Timer function
 *  @param FunctionContext	Timer function context
 *
 *  @return			N/A
 */
static inline void woal_initialize_timer(pmoal_drv_timer timer,
					 void (*TimerFunction)(void *context),
					 void *FunctionContext)
{
	/* First, setup the timer to trigger the wlan_timer_handler proxy */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 15, 0)
	timer_setup(&timer->tl, woal_timer_handler, 0);
#else
	init_timer(&timer->tl);
	timer->tl.function = woal_timer_handler;
	timer->tl.data = (t_ptr)timer;
#endif

	/* Then tell the proxy which function to call and what to pass it */
	timer->timer_function = TimerFunction;
	timer->function_context = FunctionContext;
	timer->timer_is_canceled = MTRUE;
	timer->time_period = 0;
	timer->timer_is_periodic = MFALSE;
}

/**
 *  @brief Modify timer
 *
 *  @param timer		Timer structure
 *  @param millisecondperiod	Time period in millisecond
 *
 *  @return			N/A
 */
static inline void woal_mod_timer(pmoal_drv_timer timer,
				  t_u32 millisecondperiod)
{
	timer->time_period = millisecondperiod;
	mod_timer(&timer->tl, jiffies + (millisecondperiod * HZ) / 1000);
	timer->timer_is_canceled = MFALSE;
}

/**
 *  @brief Cancel timer
 *
 *  @param timer	Timer structure
 *
 *  @return		N/A
 */
static inline void woal_cancel_timer(moal_drv_timer *timer)
{
	if (timer->timer_is_periodic || in_atomic() || irqs_disabled())
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 15, 0)
		timer_delete(&timer->tl);
#else
		del_timer(&timer->tl);
#endif
	else
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 15, 0)
		timer_delete_sync(&timer->tl);
#else
		del_timer_sync(&timer->tl);
#endif
	timer->timer_is_canceled = MTRUE;
	timer->time_period = 0;
}

#ifdef REASSOCIATION
/*
 * OS Thread Specific
 */

#include <linux/kthread.h>

/** Kernel thread structure */
typedef struct _moal_thread {
	/** Task control structrue */
	struct task_struct *task;
	/** Pointer to wait_queue_head */
	wait_queue_head_t wait_q;
	/** PID */
	pid_t pid;
	/** Pointer to moal_handle */
	void *handle;
} moal_thread;

/**
 *  @brief Activate thread
 *
 *  @param thr			Thread structure
 *  @return			N/A
 */
static inline void woal_activate_thread(moal_thread *thr)
{
	/** Initialize the wait queue */
	init_waitqueue_head(&thr->wait_q);

	/** Record the thread pid */
	thr->pid = current->pid;
}

/**
 *  @brief De-activate thread
 *
 *  @param thr			Thread structure
 *  @return			N/A
 */
static inline void woal_deactivate_thread(moal_thread *thr)
{
	/* Reset the pid */
	thr->pid = 0;
}

/**
 *  @brief Create and run the thread
 *
 *  @param threadfunc		Thread function
 *  @param thr			Thread structure
 *  @param name			Thread name
 *  @return			N/A
 */
static inline void woal_create_thread(int (*threadfunc)(void *),
				      moal_thread *thr, char *name)
{
	/* Create and run the thread */
	thr->task = kthread_run(threadfunc, thr, "%s", name);
}
#endif /* REASSOCIATION */

/* The following macros are neccessary to retain compatibility
 * around the workqueue chenges happened in kernels >= 2.6.20:
 * - INIT_WORK changed to take 2 arguments and let the work function
 *   get its own data through the container_of macro
 * - delayed works have been split from normal works to save some
 *   memory usage in struct work_struct
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 20)
/** Work_queue work initialization */
#define MLAN_INIT_WORK(_work, _fun)                                            \
	INIT_WORK(_work, ((void (*)(void *))_fun), _work)
/** Work_queue delayed work initialization */
#define MLAN_INIT_DELAYED_WORK(_work, _fun)                                    \
	INIT_WORK(_work, ((void (*)(void *))_fun), _work)
/** Work_queue container parameter */
#define MLAN_DELAYED_CONTAINER_OF(_ptr, _type, _m) container_of(_ptr, _type, _m)
#else /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20) */
/** Work_queue work initialization */
#define MLAN_INIT_WORK INIT_WORK
/** Work_queue delayed work initialization */
#define MLAN_INIT_DELAYED_WORK INIT_DELAYED_WORK
/** Work_queue container parameter */
#define MLAN_DELAYED_CONTAINER_OF(_ptr, _type, _m)                             \
	container_of(_ptr, _type, _m.work)
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20) */

/**
 *  @brief Schedule timeout
 *
 *  @param millisec	Timeout duration in milli second
 *
 *  @return		N/A
 */
static inline void woal_sched_timeout(t_u32 millisec)
{
	set_current_state(TASK_INTERRUPTIBLE);

	schedule_timeout((millisec * HZ) / 1000);
}

/**
 *  @brief Schedule timeout uninterruptible
 *
 *  @param millisec	Timeout duration in milli second
 *
 *  @return		N/A
 */
static inline void woal_sched_timeout_uninterruptible(t_u32 millisec)
{
	set_current_state(TASK_UNINTERRUPTIBLE);
	schedule_timeout_uninterruptible((millisec * HZ) / 1000);
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 19)
#define IN6PTON_XDIGIT 0x00010000
#define IN6PTON_DIGIT 0x00020000
#define IN6PTON_COLON_MASK 0x00700000
#define IN6PTON_COLON_1 0x00100000 /* single : requested */
#define IN6PTON_COLON_2 0x00200000 /* second : requested */
#define IN6PTON_COLON_1_2 0x00400000 /* :: requested */
#define IN6PTON_DOT 0x00800000 /* . */
#define IN6PTON_DELIM 0x10000000
#define IN6PTON_NULL 0x20000000 /* first/tail */
#define IN6PTON_UNKNOWN 0x40000000

static inline int xdigit2bin(char c, int delim)
{
	if (c == delim || c == '\0')
		return IN6PTON_DELIM;
	if (c == ':')
		return IN6PTON_COLON_MASK;
	if (c == '.')
		return IN6PTON_DOT;
	if (c >= '0' && c <= '9')
		return IN6PTON_XDIGIT | IN6PTON_DIGIT | (c - '0');
	if (c >= 'a' && c <= 'f')
		return IN6PTON_XDIGIT | (c - 'a' + 10);
	if (c >= 'A' && c <= 'F')
		return IN6PTON_XDIGIT | (c - 'A' + 10);
	if (delim == -1)
		return IN6PTON_DELIM;
	return IN6PTON_UNKNOWN;
}

static inline int in4_pton(const char *src, int srclen, u8 *dst, int delim,
			   const char **end)
{
	const char *s;
	u8 *d;
	u8 dbuf[4];
	int ret = 0;
	int i;
	int w = 0;

	if (srclen < 0)
		srclen = strlen(src);
	s = src;
	d = dbuf;
	i = 0;
	while (1) {
		int c;
		c = xdigit2bin(srclen > 0 ? *s : '\0', delim);
		if (!(c & (IN6PTON_DIGIT | IN6PTON_DOT | IN6PTON_DELIM |
			   IN6PTON_COLON_MASK))) {
			goto out;
		}
		if (c & (IN6PTON_DOT | IN6PTON_DELIM | IN6PTON_COLON_MASK)) {
			if (w == 0)
				goto out;
			*d++ = w & 0xff;
			w = 0;
			i++;
			if (c & (IN6PTON_DELIM | IN6PTON_COLON_MASK)) {
				if (i != 4)
					goto out;
				break;
			}
			goto cont;
		}
		w = (w * 10) + c;
		if ((w & 0xffff) > 255)
			goto out;
	cont:
		if (i >= 4)
			goto out;
		s++;
		srclen--;
	}
	ret = 1;
	moal_memcpy_ext(NULL, dst, dbuf, sizeof(dbuf), sizeof(dbuf));
out:
	if (end)
		*end = s;
	return ret;
}
#endif /* < 2.6.19 */

#ifndef __ATTRIB_ALIGN__
#define __ATTRIB_ALIGN__ __attribute__((aligned(4)))
#endif

#ifndef __ATTRIB_PACK__
#define __ATTRIB_PACK__ __attribute__((packed))
#endif

/** Get module */
#define MODULE_GET try_module_get(THIS_MODULE)
/** Put module */
#define MODULE_PUT module_put(THIS_MODULE)

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 37)
/** Initialize semaphore */
#define MOAL_INIT_SEMAPHORE(x) init_MUTEX(x)
/** Initialize semaphore */
#define MOAL_INIT_SEMAPHORE_LOCKED(x) init_MUTEX_LOCKED(x)
#else
/** Initialize semaphore */
#define MOAL_INIT_SEMAPHORE(x) sema_init(x, 1)
/** Initialize semaphore */
#define MOAL_INIT_SEMAPHORE_LOCKED(x) sema_init(x, 0)
#endif

/** Acquire semaphore and with blocking */
#define MOAL_ACQ_SEMAPHORE_BLOCK(x) down_interruptible(x)
/** Acquire semaphore without blocking */
#define MOAL_ACQ_SEMAPHORE_NOBLOCK(x) down_trylock(x)
/** Release semaphore */
#define MOAL_REL_SEMAPHORE(x) up(x)

/** Request FW timeout in second */
#define REQUEST_FW_TIMEOUT 30

#if defined(USB) || defined(SYSKT)
/** Max loop count (* 100ms) for waiting device ready at init time */
#define MAX_WAIT_DEVICE_READY_COUNT 50
#endif

/** Default watchdog timeout */
#define MRVDRV_DEFAULT_WATCHDOG_TIMEOUT (10 * HZ)

#ifdef UAP_SUPPORT
/** Default watchdog timeout
    Increase the value to avoid kernel Tx timeout message in case
    station in PS mode or left.
    The default value of PS station ageout timer is 40 seconds.
    Hence, the watchdog timer is set to a value higher than it.
*/
#define MRVDRV_DEFAULT_UAP_WATCHDOG_TIMEOUT (41 * HZ)
#endif

/* IOCTL Timeout */
#define MOAL_IOCTL_TIMEOUT (20 * HZ)

#ifdef ANDROID_KERNEL
/** Wake lock timeout in msec */
#define WAKE_LOCK_TIMEOUT 3000
/** Roaming Wake lock timeout in msec */
#define ROAMING_WAKE_LOCK_TIMEOUT 10000
#endif

/** Threshold value of number of times the Tx timeout happened */
/* WAR For EDMAC Test */
#define NUM_TX_TIMEOUT_THRESHOLD 10
/** Custom event : DRIVER HANG */
#define CUS_EVT_DRIVER_HANG "EVENT=DRIVER_HANG"
/** Custom event : FW_DUMP */
#define CUS_EVT_FW_DUMP "EVENT=FW_DUMP"
/** Custom event : START FW RESET */
#define CUS_EVT_FW_RECOVER_START "EVENT=FW_RECOVER_START"
#define CUS_EVT_FW_RECOVER_SUCCESS "EVENT=FW_RECOVER_SUCCESS"
#define CUS_EVT_FW_RECOVER_FAIL "EVENT=FW_RECOVER_FAILURE"

/** TDLS connected event */
#define CUS_EVT_TDLS_CONNECTED "EVENT=TDLS_CONNECTED"
/** TDLS tear down event */
#define CUS_EVT_TDLS_TEARDOWN "EVENT=TDLS_TEARDOWN"
/** wmm info */
#define WMM_TYPE_INFO 0
/** wmm parameter */
#define WMM_TYPE_PARAMETER 1

/** AP connected event */
#define CUS_EVT_AP_CONNECTED "EVENT=AP_CONNECTED"

/** Custom event : BW changed */
#define CUS_EVT_BW_CHANGED "EVENT=BW_CHANGED"
/** Custom event : OBSS scan parameter */
#define CUS_EVT_OBSS_SCAN_PARAM "EVENT=OBSS_SCAN_PARAM"

/** Custom event : MIC failure, unicast */
#define CUS_EVT_MLME_MIC_ERR_UNI "MLME-MICHAELMICFAILURE.indication unicast"
/** Custom event : MIC failure, multicast */
#define CUS_EVT_MLME_MIC_ERR_MUL "MLME-MICHAELMICFAILURE.indication multicast"
/** Custom event : Beacon RSSI low */
#define CUS_EVT_BEACON_RSSI_LOW "EVENT=BEACON_RSSI_LOW"
/** Custom event : Beacon SNR low */
#define CUS_EVT_BEACON_SNR_LOW "EVENT=BEACON_SNR_LOW"
/** Custom event : Beacon RSSI high */
#define CUS_EVT_BEACON_RSSI_HIGH "EVENT=BEACON_RSSI_HIGH"
/** Custom event : Beacon SNR high */
#define CUS_EVT_BEACON_SNR_HIGH "EVENT=BEACON_SNR_HIGH"
/** Custom event : Max fail */
#define CUS_EVT_MAX_FAIL "EVENT=MAX_FAIL"
/** Custom event : Data RSSI low */
#define CUS_EVT_DATA_RSSI_LOW "EVENT=DATA_RSSI_LOW"
/** Custom event : Data SNR low */
#define CUS_EVT_DATA_SNR_LOW "EVENT=DATA_SNR_LOW"
/** Custom event : Data RSSI high */
#define CUS_EVT_DATA_RSSI_HIGH "EVENT=DATA_RSSI_HIGH"
/** Custom event : Data SNR high */
#define CUS_EVT_DATA_SNR_HIGH "EVENT=DATA_SNR_HIGH"
/** Custom event : Link Quality */
#define CUS_EVT_LINK_QUALITY "EVENT=LINK_QUALITY"
/** Custom event : Port Release */
#define CUS_EVT_PORT_RELEASE "EVENT=PORT_RELEASE"
/** Custom event : Pre-Beacon Lost */
#define CUS_EVT_PRE_BEACON_LOST "EVENT=PRE_BEACON_LOST"

/** Custom event : Deep Sleep awake */
#define CUS_EVT_DEEP_SLEEP_AWAKE "EVENT=DS_AWAKE"

#define CUS_EVT_ADDBA_TIMEOUT "EVENT=ADDBA_TIMEOUT"

#define CUS_EVT_TOD_TOA "EVENT=TOD-TOA"

/** Custom event : Host Sleep activated */
#define CUS_EVT_HS_ACTIVATED "HS_ACTIVATED"
/** Custom event : Host Sleep deactivated */
#define CUS_EVT_HS_DEACTIVATED "HS_DEACTIVATED"
/** Custom event : Host Sleep wakeup */
#define CUS_EVT_HS_WAKEUP "HS_WAKEUP"

/** Wakeup Reason */
typedef enum {
	NO_HSWAKEUP_REASON = 0, // 0.unknown
	BCAST_DATA_MATCHED, // 1. Broadcast data matched
	MCAST_DATA_MATCHED, // 2. Multicast data matched
	UCAST_DATA_MATCHED, // 3. Unicast data matched
	MASKTABLE_EVENT_MATCHED, // 4. Maskable event matched
	NON_MASKABLE_EVENT_MATCHED, // 5. Non-maskable event matched
	NON_MASKABLE_CONDITION_MATCHED, // 6. Non-maskable condition matched
					// (EAPoL rekey)
	MAGIC_PATTERN_MATCHED, // 7. Magic pattern matched
	CONTROL_FRAME_MATCHED, // 8. Control frame matched
	MANAGEMENT_FRAME_MATCHED, // 9. Management frame matched
	GTK_REKEY_FAILURE, // 10. GTK rekey failure
	MGMT_FRAME_FILTER_EXT_MATCHED, // 11. Management frame filter matched
	RESERVED // Others: reserved
} HSWakeupReason_t;

/** Custom event : Radar Detected */
#define CUS_EVT_RADAR_DETECTED "EVENT=RADAR_DETECTED"
/** Custom event : CAC finished */
#define CUS_EVT_CAC_FINISHED "EVENT=CAC_FINISHED"
/** Custom event : CAC start */
#define CUS_EVT_CAC_START "EVENT=CAC_START"
#ifdef UAP_SUPPORT
void woal_move_to_next_channel(moal_private *priv);
void woal_chan_event(moal_private *priv, t_u8 type, t_u8 channel, t_u8 radar);
void woal_process_chan_event(moal_private *priv, t_u8 type, t_u8 channel,
			     t_u8 radar);
mlan_status woal_do_dfs_cac(moal_private *priv,
			    mlan_ds_11h_chan_rep_req *ch_rpt_req);
#endif

/** Custom event : WEP ICV error */
#define CUS_EVT_WEP_ICV_ERR "EVENT=WEP_ICV_ERR"

/** Custom event : Channel Switch Announcment */
#define CUS_EVT_CHANNEL_SWITCH_ANN "EVENT=CHANNEL_SWITCH_ANN"

/** Custom event : Channel Switch complete */
#define CUS_EVT_CHAN_SWITCH_COMPLETE "EVENT=CHANNEL_SWITCH_COMPLETE"

/** Custom indiciation message sent to the application layer for WMM changes */
#define WMM_CONFIG_CHANGE_INDICATION "WMM_CONFIG_CHANGE.indication"

#define CUS_EVT_FW_DUMP_DONE "EVENT=FW_DUMP_DONE"

#ifdef UAP_SUPPORT
/** Custom event : STA connected */
#define CUS_EVT_STA_CONNECTED "EVENT=STA_CONNECTED"
/** Custom event : STA disconnected */
#define CUS_EVT_STA_DISCONNECTED "EVENT=STA_DISCONNECTED"
#endif
#define FW_DEBUG_INFO "EVENT=FW_DEBUG_INFO"

#define CUS_EVT_CSI "EVENT=MLAN_CSI"

/** 10 seconds */
#define MOAL_TIMER_10S 10000
/** 5 seconds */
#define MOAL_TIMER_5S 5000
/** 1 second */
#define MOAL_TIMER_1S 1000
/** 1 milisecond */
#define MOAL_TIMER_1MS 1
/**
 * Firmware dump to logger includes additional time delay to
 * complete dump. Duration shall be, 10 seconds if firmware
 * dump to log is enabled OR 5 seconds otherwise.
 *
 */
#ifdef FWDUMP_VIA_PRINT
/** 10 seconds */
#define MOAL_FW_DUMP_TIMER 10000
#else
/** 5 seconds */
#define MOAL_FW_DUMP_TIMER 5000
#endif
/** scan timeout set to 25 seconds */
#define SCAN_TIMEOUT_25S 25000

/** passive scan time */
#define PASSIVE_SCAN_CHAN_TIME 110
/** active scan time */
#define ACTIVE_SCAN_CHAN_TIME 110
/** specific scan time */
#define SPECIFIC_SCAN_CHAN_TIME 110
/** passive scan time */
#define INIT_PASSIVE_SCAN_CHAN_TIME 80
/** active scan time */
#define INIT_ACTIVE_SCAN_CHAN_TIME 80
/** specific scan time */
#define INIT_SPECIFIC_SCAN_CHAN_TIME 80
/** specific scan time after connected */
#define MIN_SPECIFIC_SCAN_CHAN_TIME 40

/* MAX SCAN TIMEOUT: 10 seconds, set to 9.5 seconds */
#define MAX_SCAN_TIMEOUT 9500

/** Default value of re-assoc timer */
#define REASSOC_TIMER_DEFAULT 500

/** Netlink protocol number */
#define NETLINK_NXP (MAX_LINKS - 1)
/** Netlink maximum payload size */
#define NL_MAX_PAYLOAD (3 * 1024)
/** Netlink multicast group number */
#define NL_MULTICAST_GROUP 1

#define MAX_RX_PENDING_THRHLD 50

/** high rx pending packets */
#define USB_HIGH_RX_PENDING 100
/** low rx pending packets */
#define USB_LOW_RX_PENDING 80

/** MAX Tx Pending count */
#define MAX_TX_PENDING 800

/** LOW Tx Pending count */
#define LOW_TX_PENDING 380

/** MAX Tx Pending count when multi-client scheduling is used  */
#define MCLIENT_MAX_TX_PENDING (128 * MAX_STA_COUNT)

/** LOW Tx Pending count when multi-client scheduling is used */
#define MCLIENT_LOW_TX_PENDING (MCLIENT_MAX_TX_PENDING * 3 / 4)

/** Offset for subcommand */
#define SUBCMD_OFFSET 4

/** default scan channel gap  */
#define DEF_SCAN_CHAN_GAP 50
/** default scan time per channel in miracast mode */
#define DEF_MIRACAST_SCAN_TIME 20
/** GAP value is optional */
#define GAP_FLAG_OPTIONAL MBIT(15)

#define AUTH_TX_DEFAULT_WAIT_TIME 2400

/** max retry count for wait_event_interupptible_xx while loop */
#define MAX_RETRY_CNT 300
/** wait_queue structure */
typedef struct _wait_queue {
	/** wait_queue_head */
	wait_queue_head_t wait;
	/** Wait condition */
	BOOLEAN condition;
	/** Start time */
	long start_time;
	/** Status from MLAN */
	mlan_status status;
	/** flag for wait_timeout */
	t_u8 wait_timeout;
	/** retry count */
	t_u16 retry;
} wait_queue, *pwait_queue;

/** Auto Rate */
#define AUTO_RATE 0xFF

#define STA_WEXT_MASK MBIT(0)
#define UAP_WEXT_MASK MBIT(1)
#define STA_CFG80211_MASK MBIT(2)
#define UAP_CFG80211_MASK MBIT(3)
#ifdef STA_CFG80211
#ifdef STA_SUPPORT
/** Is STA CFG80211 enabled in module param */
#define IS_STA_CFG80211(x) (x & STA_CFG80211_MASK)
#endif
#endif
#ifdef UAP_CFG80211
#ifdef UAP_SUPPORT
/** Is UAP CFG80211 enabled in module param */
#define IS_UAP_CFG80211(x) (x & UAP_CFG80211_MASK)
#endif
#endif
#if defined(STA_CFG80211) || defined(UAP_CFG80211)
/** Is UAP or STA CFG80211 enabled in module param */
#define IS_STA_OR_UAP_CFG80211(x) (x & (STA_CFG80211_MASK | UAP_CFG80211_MASK))
#endif

#ifdef STA_WEXT
/** Is STA WEXT enabled in module param */
#define IS_STA_WEXT(x) (x & STA_WEXT_MASK)
#endif /* STA_WEXT */
#ifdef UAP_WEXT
/** Is UAP WEXT enabled in module param */
#define IS_UAP_WEXT(x) (x & UAP_WEXT_MASK)
#endif /* UAP_WEXT */
#if defined(STA_WEXT) || defined(UAP_WEXT)
/** Is UAP or STA WEXT enabled in module param */
#define IS_STA_OR_UAP_WEXT(x) (x & (STA_WEXT_MASK | UAP_WEXT_MASK))
#endif

#ifdef STA_SUPPORT
/** Driver mode STA bit */
#define DRV_MODE_STA MBIT(0)
/** Maximum STA BSS */
#define MAX_STA_BSS 1
/** Default STA BSS */
#define DEF_STA_BSS 1
#endif
#ifdef UAP_SUPPORT
/** Driver mode uAP bit */
#define DRV_MODE_UAP MBIT(1)
/** Maximum uAP BSS */
#define MAX_UAP_BSS 2
#define MAX_UAP_BSS_DUAL_MAC 3
/** Default uAP BSS */
#define DEF_UAP_BSS 1

/** WACP Modes for uAP */
#define WACP_MODE_DEFAULT 0
#define WACP_MODE_1 1
#define WACP_MODE_2 2
#endif
#ifdef WIFI_DIRECT_SUPPORT
/** Driver mode WIFIDIRECT bit */
#define DRV_MODE_WIFIDIRECT MBIT(2)
/** Maximum WIFIDIRECT BSS */
#define MAX_WIFIDIRECT_BSS 2
/** Default WIFIDIRECT BSS */
#define DEF_WIFIDIRECT_BSS 1
#if defined(STA_CFG80211) && defined(UAP_CFG80211)
#define DEF_VIRTUAL_BSS 0
#endif
#endif /* WIFI_DIRECT_SUPPORT */
/** Driver mode NAN bit */
#define DRV_MODE_NAN MBIT(4)
/** Maximum NAN BSS */
#define MAX_NAN_BSS 1
/** Default NAN BSS */
#define DEF_NAN_BSS 1

/**Driver mode 0DFS bit**/
#define DRV_MODE_DFS MBIT(7)
/**Maxinmum DFS BSS**/
#define MAX_DFS_BSS 1
/**Default DFS BSS**/
#define DEF_DFS_BSS 1

#define DRV_MODE_WLAN (MBIT(0) | MBIT(1) | MBIT(2) | MBIT(3) | MBIT(4))

/**
 * the maximum number of adapter supported
 **/
#define MAX_MLAN_ADAPTER 4

typedef struct _moal_drv_mode {
	/** driver mode */
	t_u16 drv_mode;
	/** total number of interfaces */
	t_u16 intf_num;
	/** attribute of bss */
	mlan_bss_attr *bss_attr;
	/** name of firmware image */
	char *fw_name;
} moal_drv_mode;

extern moal_handle *m_handle[MAX_MLAN_ADAPTER];

/** Indicate if handle->info's address */
#define INFO_ADDR BIT(0)
#define IS_INFO_ADDR(attr) (attr & INFO_ADDR)
/** Indicate if handle's address */
#define HANDLE_ADDR BIT(1)
#define IS_HANDLE_ADDR(attr) (attr & HANDLE_ADDR)
/** Indicate if card's address */
#define CARD_ADDR BIT(2)
#define IS_CARD_ADDR(attr) (attr & CARD_ADDR)
/** indicate if priv's address */
#define PRIV_ADDR BIT(3)
#define IS_PRIV_ADDR(attr) (attr & PRIV_ADDR)

/** Debug data */
struct debug_data {
	/** Name */
	char name[32];
	/** Size */
	t_u32 size;
	/** Address */
	t_ptr addr;
	/** Attribute:
	0-7bit: start address for addr to add to, 0 means common(no specific)
	8-15bit: interface type, 0 means common(no interface specific)
	other: unused
	*/
	t_u32 attr;
};

/** Private debug data */
struct debug_data_priv {
	/** moal_private handle */
	moal_private *priv;
	/** Debug items */
	struct debug_data *items;
	/** numbre of item */
	int num_of_items;
};

/** Maximum IP address buffer length */
#define IPADDR_MAX_BUF 20
/** IP address operation: Remove */
#define IPADDR_OP_REMOVE 0

/* max hold of tcp ack pkt*/
#define TCP_ACK_MAX_HOLD 9
/* max num of tcp session */
#define TCP_ACK_MAX_SESS 100

#define DROP_TCP_ACK 1
#define HOLD_TCP_ACK 2
struct tcp_sess {
	struct list_head link;
	/** tcp session info */
	t_u32 src_ip_addr;
	t_u32 dst_ip_addr;
	t_u16 src_tcp_port;
	t_u16 dst_tcp_port;
	/** tx ack packet info */
	t_u32 ack_seq;
	/** tcp ack buffer */
	void *ack_skb;
	/** priv structure */
	void *priv;
	/** pmbuf */
	void *pmbuf;
	/** timer for ack */
	moal_drv_timer ack_timer __ATTRIB_ALIGN__;
	/** timer is set */
	atomic_t is_timer_set;
	/** last update time*/
	wifi_timeval update_time;
};

struct tx_status_info {
	struct list_head link;
	/** cookie */
	t_u64 tx_cookie;
	/** seq_num */
	t_u8 tx_seq_num;
	/** cancel remain on channel when receive tx status */
	t_u8 cancel_remain_on_channel;
	/** set to notify userspace tx duration expired */
	bool send_tx_expired;
	/**          skb */
	void *tx_skb;
};

/** woal event type */
enum woal_event_type {
	WOAL_EVENT_CHAN_SWITCH,
	WOAL_EVENT_RX_MGMT_PKT,
	WOAL_EVENT_BGSCAN_STOP,
#if defined(UAP_CFG80211) || defined(STA_CFG80211)
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
	WOAL_EVENT_DEAUTH,
	WOAL_EVENT_ASSOC_RESP,
	WOAL_EVENT_ASSOC_TIMEOUT,
#endif
#endif
	WOAL_EVENT_CHAN_RPT,
	WOAL_EVENT_RADAR,
#ifdef UAP_CFG80211
#if KERNEL_VERSION(3, 12, 0) <= CFG80211_VERSION_CODE
	WOAL_EVENT_CANCEL_CHANRPT,
#endif
#endif
	WOAL_EVENT_RGPWR_KEY_MISMATCH,
	WOAL_EVENT_RESET_WIFI,
};

/** chan_rpt_info */
typedef struct _chan_radar_info {
	/** channel */
	t_u8 channel;
	/** radar */
	t_u8 radar;
} chan_radar_info;

typedef struct _woal_evt_buf {
	/** Event len */
	t_u16 event_len;
	/** Event buffer */
	t_u8 event_buf[1500];
} woal_evt_buf;

/** woal event */
struct woal_event {
	/*list head */
	struct list_head link;
	/** type */
	enum woal_event_type type;
	/** priv pointer */
	void *priv;
	struct cfg80211_bss *assoc_bss;
	union {
		chan_band_info chan_info;
		woal_evt_buf evt;
		mlan_ds_assoc_info assoc_info;
		mlan_deauth_param deauth_info;
		chan_radar_info radar_info;
		t_u8 deauth_evt_cnt;
	};
};

#define MAX_NUM_ETHER_TYPE 8
typedef struct {
	/** number of protocols in protocol array*/
	t_u8 protocol_num;
	/** protocols supported */
	t_u16 protocols[MAX_NUM_ETHER_TYPE];
} __ATTRIB_PACK__ dot11_protocol;
typedef struct {
	/** Data rate in unit of 0.5Mbps */
	t_u16 datarate;
	/** Channel number to transmit the frame */
	t_u8 channel;
	/** Bandwidth to transmit the frame */
	t_u8 bw;
	/** Power to be used for transmission */
	t_u8 power;
	/** Priority of the packet to be transmitted */
	t_u8 priority;
	/** retry time of tx transmission*/
	t_u8 retry_limit;
	/** Reserved fields*/
	t_u8 reserved[1];
} __ATTRIB_PACK__ dot11_txcontrol;

typedef struct {
	/** Data rate of received paccket*/
	t_u16 datarate;
	/** Channel on which packet was received*/
	t_u8 channel;
	/** Rx antenna*/
	t_u8 antenna;
	/** RSSI */
	t_u8 rssi;
	/** Reserved */
	t_u8 reserved[3];
} __ATTRIB_PACK__ dot11_rxcontrol;

#define OKC_WAIT_TARGET_PMKSA_TIMEOUT (4 * HZ / 1000)
#define PMKID_LEN 16
struct pmksa_entry {
	struct list_head link;
	u8 bssid[ETH_ALEN];
	u8 pmkid[PMKID_LEN];
};

/** default rssi low threshold */
#define TDLS_RSSI_LOW_THRESHOLD 55
/** default rssi high threshold */
#define TDLS_RSSI_HIGH_THRESHOLD 50
/** TDLS idle time */
#define TDLS_IDLE_TIME (10 * HZ)
/** TDLS max failure count */
#define TDLS_MAX_FAILURE_COUNT 4
/** TDLS tear down reason */
#define TDLS_TEARN_DOWN_REASON_UNSPECIFIC 26

/** TDLS status */
typedef enum _tdlsStatus_e {
	TDLS_NOT_SETUP = 0,
	TDLS_SETUP_INPROGRESS,
	TDLS_SETUP_COMPLETE,
	TDLS_SETUP_FAILURE,
	TDLS_TEAR_DOWN,
	TDLS_SWITCHING_CHANNEL,
	TDLS_IN_BASE_CHANNEL,
	TDLS_IN_OFF_CHANNEL,
} tdlsStatus_e;

/** tdls peer_info */
struct tdls_peer {
	struct list_head link;
	/** MAC address information */
	t_u8 peer_addr[ETH_ALEN];
	/** rssi */
	int rssi;
	/** jiffies with rssi */
	long rssi_jiffies;
	/** link status */
	tdlsStatus_e link_status;
	/** num of set up failure */
	t_u8 num_failure;
};

/** mcast node */
struct mcast_node {
	struct list_head link;
	/** mcast address information */
	t_u8 mcast_addr[ETH_ALEN];
};

/** This is a flag for auto assoc/re-connect retry forever */
#define AUTO_ASSOC_RETRY_FOREVER 0xFFFF

typedef enum {
	AUTO_ASSOC_TYPE_NONE = 0,
	AUTO_ASSOC_TYPE_DRV_ASSOC,
	AUTO_ASSOC_TYPE_DRV_RECONN,
	AUTO_ASSOC_TYPE_FW_RECONN,
} AUTO_ASSOC_TYPE;

typedef struct {
	/** driver auto assoc retry count */
	t_u8 retry_count;
	/** driver auto assoc retry interval */
	t_u8 retry_interval;
	/** driver auto assoc status */
	t_u8 status;
} drv_auto_assoc;

typedef struct {
	/** Bitmap for auto assoc type on/off */
	t_u8 auto_assoc_type_on;
	/** flag of being triggered by drv auto assoc/re-connect  */
	t_u8 auto_assoc_trigger_flag;
	/** driver auto assoc info*/
	drv_auto_assoc drv_assoc;
	/** driver auto re-connect info*/
	drv_auto_assoc drv_reconnect;
} auto_assoc;

struct rf_test_mode_data {
	/* tx antenna num */
	t_u32 tx_antenna;
	/* rx antenna num */
	t_u32 rx_antenna;
	/* radio mode */
	t_u32 radio_mode[2];
	/* RF band */
	t_u32 band;
	/* RF bandwidth */
	t_u32 bandwidth;
	/* RF channel */
	t_u32 channel;
	/* Total Rx ucast/mcast/bcast pkt count */
	t_u32 rx_tot_pkt_count;
	/* Rx mcast/bcast pkt count */
	t_u32 rx_mcast_bcast_pkt_count;
	/* Rx fcs error count */
	t_u32 rx_pkt_fcs_err_count;
	/* Tx power config values */
	t_s32 tx_power_data[3];
	/* Tx continuous config values */
	t_u32 tx_cont_data[6];
	/* Tx frame config values */
	t_u32 tx_frame_data[22];
	/* HE TB Tx values */
	t_u32 he_tb_tx[4];
	t_s32 he_tb_tx_power[1];
	/* BSSID */
	t_u8 bssid[ETH_ALEN];
	/* Trigger frame config values */
	mfg_Cmd_IEEEtypes_CtlBasicTrigHdr_t mfg_tx_trigger_config;
	/* OTP frame data */
	mfg_cmd_otp_mac_addr_rd_wr_t mfg_otp_mac_addr_rd_wr;
	/* OTP CAL data */
	mfg_cmd_otp_cal_data_rd_wr_t mfg_otp_cal_data_rd_wr;
};

/** Number of samples in histogram (/proc/mwlan/adapterX/mlan0/histogram).*/
#define HIST_MAX_SAMPLES 1048576
#define RX_RATE_MAX 196

/** SRN MAX  */
#define SNR_MAX 256
/** NOISE FLR MAX  */
#define NOISE_FLR_MAX 256
/** SIG STRENTGH MAX */
#define SIG_STRENGTH_MAX 256
/** historgram data */
typedef struct _hgm_data {
	/** snr */
	atomic_t snr[SNR_MAX];
	/** noise flr */
	atomic_t noise_flr[NOISE_FLR_MAX];
	/** sig_str */
	atomic_t sig_str[SIG_STRENGTH_MAX];
	/** num sample */
	atomic_t num_samples;
	/** rx rate */
	atomic_t rx_rate[];
} hgm_data, *phgm_data;

/** max antenna number */
#define MAX_ANTENNA_NUM 4

/* wlan_hist_proc_data */
typedef struct _wlan_hist_proc_data {
	/** antenna */
	u8 ant_idx;
	/** Private structure */
	struct _moal_private *priv;
} wlan_hist_proc_data;

enum ring_id {
	VERBOSE_RING_ID,
	EVENT_RING_ID,
	RING_ID_MAX,
};

#define AUTO_DFS_ENABLE 0x1
#define AUTO_DFS_DISABLE 0x0
#define MAX_DFS_CHAN_LIST 16

/** Auto Zero DFS config structure */
typedef struct _auto_zero_dfs_cfg {
	/** 1: start 0: stop */
	t_u8 start_auto_zero_dfs;
	/** start channel for ZeroDFS */
	t_u8 cac_start_chan;
	/** cac timer */
	t_u32 cac_timer;
	/** bw: 0: 20MHz  1: 40Mz above  3: 40MHz below  4: Bandwidth 80MHz */
	t_u8 bw;
	/** enable uap chan switch after first CAC finished*/
	t_u8 uap_chan_switch;
	/** enable auto zero dfs */
	t_u8 multi_chan_dfs;
	/** num of chan */
	t_u8 num_of_chan;
	/** dfs channel list */
	t_u8 dfs_chan_list[MAX_DFS_CHAN_LIST];
} __ATTRIB_PACK__ auto_zero_dfs_cfg;

#if defined(UAP_CFG80211) || defined(STA_CFG80211)
/** station node */
typedef struct _station_node {
	/** station aid */
	t_u16 aid;
	/** station mac address */
	t_u8 peer_mac[MLAN_MAC_ADDR_LENGTH];
	/** net_device that station is bind to */
	struct net_device *netdev;
	/** is valid flag */
	t_u8 is_valid;
} station_node;

/** dhcp discover info */
struct dhcp_discover_info {
	/** link */
	struct list_head link;
	/** transaction id */
	t_u32 transaction_id;
	/** Time stamp when packet is received (seconds) */
	t_u32 in_ts_sec;
	/** Time stamp when packet is received (micro seconds) */
	t_u32 in_ts_usec;
};

#define UDP_PSEUDO_HEADER_SIZE 12
#define DHCP_ETH_HEADER_SIZE 14
#define DHCP_MIN_IP_HEADER_SIZE 20
#define DHCP_UDP_HEADER_SIZE 8
#define DHCP_IP_PROTO 0x0800
#define DHCP_IP_VERSION 0x04
#define DHCP_IP_TOS 0x00
#define DHCP_IP_TTL 0x40
#define DHCP_UDP_PROTO 0x11
#define DHCP_IPADDR_SIZE 4
#define DHCP_SRC_PORT 0x0044
#define DHCP_DST_PORT 0x0043

struct dhcp_pkt {
	/** DHCP op code or message type */
	t_u8 op;
	/** Hardware address type */
	t_u8 htype;
	/** Hardware address length */
	t_u8 hlen;
	/** Hops is optionally used by relay agents */
	t_u8 hops;
	/** Transaction identifier */
	t_u32 xid;
	/** Seconds till the client has started the DHCP process */
	t_u16 secs;
	/** Flags */
	t_u16 flags;
	/**
	 * IP address of the client  Only filled in if
	 * the client is in BOUND, RENEW or REBINDING
	 */
	t_u32 ciaddr;
	/**
	 * Your IP address offered to the client.
	 * Sent  by server during the DISCOVER and ACK
	 */
	t_u32 yiaddr;
	/**
	 *  IP address of the DHCP server to use next.
	 */
	t_u32 siaddr;
	/** Relay agent IP address */
	t_u32 giaddr;
	t_u8 chaddr[16];
	/** Null terminated server host name */
	t_u8 sname[64];
	t_u8 file[128];
	/** BOOTP magic cookie */
	t_u32 magic;
	t_u8 pOptions[1];
} __ATTRIB_PACK__;

/** IPv4 arp header */
struct arp_hdr {
	/** Hardware type */
	t_u16 htype;
	/** Protocol type */
	t_u16 ptype;
	/** Hardware address length */
	t_u8 addr_len;
	/** Protocol address length */
	t_u8 proto_len;
	/** Operation code */
	t_u16 op_code;
	/** Source mac address */
	t_u8 sender_mac[MLAN_MAC_ADDR_LENGTH];
	/** Sender IP address */
	t_u8 sender_ip[4];
	/** Destination mac address */
	t_u8 target_mac[MLAN_MAC_ADDR_LENGTH];
	/** Destination IP address */
	t_u8 target_ip[4];
} __ATTRIB_PACK__;

/** ARP request entry ageout of 10 secs */
#define ARP_REQ_AGEOUT_TIME (10 * HZ)

/** ARP request node */
struct arp_entry {
	t_u32 hash_key;
	struct hlist_node arp_hlist;
	t_u64 ageout_jiffies;
};

#define EASY_MESH_MULTI_AP_FH_BSS (t_u8)(0x20)
#define EASY_MESH_MULTI_AP_BH_BSS (t_u8)(0x40)
#define EASY_MESH_MULTI_AP_BH_AND_FH_BSS (t_u8)(0x60)

#define EASY_MESH_MULTI_AP_BSS_MODE_1 (t_u8)(0x01)
#define EASY_MESH_MULTI_AP_BSS_MODE_2 (t_u8)(0x02)
#define EASY_MESH_MULTI_AP_BSS_MODE_3 (t_u8)(0x03)
#endif

#ifdef STA_SUPPORT
enum scan_set_band {
	SCAN_SETBAND_AUTO = 0,
	SCAN_SETBAND_2G = BIT(0),
	SCAN_SETBAND_5G = BIT(1),
	SCAN_SETBAND_6G = BIT(2),
};
#endif

/** IPv6 address node */
struct ipv6addr_entry {
	/** list node link */
	struct list_head link;
	/** IPv6 address entry */
	t_u8 ipv6_addr[16];
};

/** Private structure for MOAL */
struct _moal_private {
	/** Handle structure */
	moal_handle *phandle;
	/** Tx timeout count */
	t_u32 num_tx_timeout;
	/** BSS index */
	t_u8 bss_index;
	/** BSS type */
	t_u8 bss_type;
	/** BSS role */
	t_u8 bss_role;
	/** bss virtual flag */
	t_u8 bss_virtual;
	/** MAC address information */
	t_u8 current_addr[ETH_ALEN];
	/** Media connection status */
	BOOLEAN media_connected;
	/** mclist work queue */
	struct workqueue_struct *mclist_workqueue;
	/** mclist work */
	struct work_struct mclist_work;
	/** Statistics of tcp ack tx dropped */
	t_u32 tcp_ack_drop_cnt;
	/** Statistics of tcp ack tx in total from kernel */
	t_u32 tcp_ack_cnt;
	/** Statistics of tcp ack with payload*/
	t_u32 tcp_ack_payload;
#ifdef UAP_SUPPORT
	/** uAP started or not */
	BOOLEAN bss_started;
	/** host based uap flag */
	BOOLEAN uap_host_based;
	/** target channel */
	t_u8 target_chan;
	/** backup channel */
	t_u8 backup_chan;
	/** channel mode for channel switch */
	t_u8 chan_mode;
	/** number of csa for channel switch */
	t_u8 chan_num_pkts;
	/** uAP skip CAC*/
	BOOLEAN skip_cac;
	/** tx block flag */
	BOOLEAN uap_tx_blocked;
	/** user cac period */
	t_u32 user_cac_period_msec;
	/** channel under nop */
	BOOLEAN chan_under_nop;
#if defined(STA_CFG80211) || defined(UAP_CFG80211)
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 17, 0)
	/** radar background */
	t_u8 radar_background;
	/** radar background channel */
	struct cfg80211_chan_def radar_background_chan;
#endif
#endif
	/** chan_rpt_req on Zero DFS interface */
	mlan_ds_11h_chan_rep_req chan_rpt_req;
	/** chan_rpt pending */
	t_u8 chan_rpt_pending;
	/** auto dfs cfg */
	auto_zero_dfs_cfg auto_dfs_cfg;
	/** index of cac */
	int curr_cac_idx;
#ifdef UAP_CFG80211
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
	/** current working channel */
	struct cfg80211_chan_def chan;
#endif
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 12, 0)
	/** switch channel */
	struct cfg80211_chan_def csa_chan;
	/** beacon after channel switch */
	struct cfg80211_beacon_data beacon_after;
	/** CSA work queue */
	struct workqueue_struct *csa_workqueue;
	/** csa work */
	struct delayed_work csa_work;
#endif
#endif
#endif
	/** IP addr type */
	t_u32 ip_addr_type;
	/** IP addr */
	t_u8 ip_addr[IPADDR_LEN];
	t_u8 ipv6_addr_configured;
	/** IPv6 addr count */
	t_u8 ipv6count;
	/** IPv6 addr Queue */
	struct list_head ipv6_addrses;
	/** Lock for IPv6 addrs */
	spinlock_t ipv6addr_lock;
#ifdef STA_SUPPORT
	/** scan type */
	t_u8 scan_type;

	/** set band for scan */
	t_u8 scan_setband_mask;

	/** extended capabilities */
	ExtCap_t extended_capabilities;
	/** bg_scan_start */
	t_u8 bg_scan_start;
	/** bg_scan reported */
	t_u8 bg_scan_reported;
	/** bg_scan config */
	wlan_bgscan_cfg scan_cfg;
	/** sched scaning flag */
	t_u8 sched_scanning;
	/** bgscan request id */
	t_u64 bg_scan_reqid;
#ifdef STA_CFG80211
	/** roaming enabled flag */
	t_u8 roaming_enabled;
	/** roaming required flag */
	t_u8 roaming_required;
#endif
#ifdef STA_CFG80211
	/** rssi low threshold */
	int rssi_low;
	/** channel for connect */
	struct ieee80211_channel conn_chan;
	/** bssid for connect */
	t_u8 conn_bssid[ETH_ALEN];
	/** ssid for connect */
	t_u8 conn_ssid[MLAN_MAX_SSID_LENGTH];
	/** length of ssid for connect */
	t_u8 conn_ssid_len;
	/** key data */
	t_u8 conn_wep_key[MAX_WEP_KEY_SIZE];
	/** connection param */
	struct cfg80211_connect_params sme_current;
	/** station info */
	struct station_info *sinfo;
	/* associcate bss */
	struct cfg80211_bss *assoc_bss;
#endif
	t_u8 wait_target_ap_pmkid;
	wait_queue_head_t okc_wait_q __ATTRIB_ALIGN__;
	struct list_head pmksa_cache_list;
	spinlock_t pmksa_list_lock;
	struct pmksa_entry *target_ap_pmksa;
	t_u8 okc_ie_len;
	t_u8 *okc_roaming_ie;
#endif
	/** Net device pointer */
	struct net_device *netdev;
	/** Net device statistics structure */
	struct net_device_stats stats;
#if defined(STA_CFG80211) || defined(UAP_CFG80211)
	/** Wireless device pointer */
	struct wireless_dev *wdev;
	/** Wireless device */
	struct wireless_dev w_dev;
	/** Net device pointer */
	struct net_device *pa_netdev;
	/** channel parameter for UAP/GO */
	t_u16 channel;
	/** bandwidth parameter for UAP/GO */
	t_u8 bandwidth;
#ifdef UAP_SUPPORT
	/** wep key */
	wep_key uap_wep_key[4];
	/** cipher */
	t_u32 cipher;
#endif
	/** pmk saved flag */
	t_u8 pmk_saved;
	/** pmk */
	mlan_pmk_t pmk;
	/** beacon ie index */
	t_u16 beacon_index;
	/** proberesp ie index */
	t_u16 proberesp_index;
	/** proberesp_p2p_index */
	t_u16 proberesp_p2p_index;
	/** assocresp ie index */
	t_u16 assocresp_index;
	/** assocresp qos map ie index */
	t_u16 assocresp_qos_map_index;
	/** probereq index for mgmt ie */
	t_u16 probereq_index;
	/** mgmt_subtype_mask */
	t_u32 mgmt_subtype_mask;
	/** beacon wps index for mgmt ie */
	t_u16 beacon_wps_index;
	/** beacon/proberesp vendor ie index */
	t_u16 beacon_vendor_index;
#endif
#ifdef STA_CFG80211
#ifdef STA_SUPPORT
	/** CFG80211 association bssid  */
	t_u8 cfg_bssid[ETH_ALEN];
	/** Disconnect request from CFG80211 */
	bool cfg_disconnect;
	/** connect request from CFG80211 */
	bool cfg_connect;
	/** lock for cfg connect */
	spinlock_t connect_lock;
	/** assoc status */
	t_u32 assoc_status;
	/** rssi_threshold */
	s32 cqm_rssi_thold;
	/** rssi_high_threshold */
	s32 cqm_rssi_high_thold;
	/** rssi hysteresis */
	u32 cqm_rssi_hyst;
	/** last rssi_low */
	u8 last_rssi_low;
	/** last rssi_high */
	u8 last_rssi_high;
	/** mrvl rssi threshold */
	u8 mrvl_rssi_low;
	/** last event */
	u32 last_event;
	/** fake scan flag */
	u8 fake_scan_complete;
	/**ft ie*/
	t_u8 ft_ie[MAX_IE_SIZE];
	/**ft ie len*/
	t_u8 ft_ie_len;
	/**ft ie*/
	t_u8 pre_ft_ie[MAX_IE_SIZE];
	/**ft ie len*/
	t_u8 pre_ft_ie_len;
	/**mobility domain value*/
	t_u16 ft_md;
	/**ft capability*/
	t_u8 ft_cap;
	/** set true when receive ft auth or action ft roaming */
	t_bool ft_pre_connect;
	/**ft roaming triggered by driver or not*/
	t_bool ft_roaming_triggered_by_driver;
	/**target ap mac address for Fast Transition*/
	t_u8 target_ap_bssid[ETH_ALEN];
	/** IOCTL wait queue for FT*/
	wait_queue_head_t ft_wait_q __ATTRIB_ALIGN__;
	/** ft wait condition */
	t_bool ft_wait_condition;
#endif /* STA_SUPPORT */
#endif /* STA_CFG80211 */
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
	/** flag for host_mlme */
	t_u8 host_mlme;
	/** flag for auth */
	t_u8 auth_flag;
	/** flag for auth algorithm */
	t_u16 auth_alg;
	/** auth tx cnt */
	t_u8 auth_tx_cnt;
	/** deauth evt cnt */
	t_u8 deauth_evt_cnt;
	/** delay deauth event */
	t_u8 delay_deauth_notify;
	/** notify bssid */
	t_u8 bssid_notify[ETH_ALEN];
#endif
#ifdef CONFIG_PROC_FS
	/** Proc entry */
	struct proc_dir_entry *proc_entry;
	/** Proc entry name */
	char proc_entry_name[IFNAMSIZ];
	/** proc entry for hist */
	struct proc_dir_entry *hist_entry;
	/** ant_hist_proc_data */
	wlan_hist_proc_data hist_proc[MAX_ANTENNA_NUM];
#endif /* CONFIG_PROC_FS */
#ifdef STA_SUPPORT
	/** Nickname */
	t_u8 nick_name[16];
	/** AdHoc link sensed flag */
	BOOLEAN is_adhoc_link_sensed;
	/** Current WEP key index */
	t_u16 current_key_index;
#ifdef REASSOCIATION
	mlan_ssid_bssid prev_ssid_bssid;
	/** Re-association required */
	BOOLEAN reassoc_required;
	/** Flag of re-association on/off */
	BOOLEAN reassoc_on;
	/** Set asynced essid flag */
	BOOLEAN set_asynced_essid_flag;
#endif /* REASSOCIATION */
	/** Report scan result */
	t_u8 report_scan_result;
	/** wpa_version */
	t_u8 wpa_version;
	/** key mgmt */
	t_u8 key_mgmt;
	/** rx_filter */
	t_u8 rx_filter;
#endif /* STA_SUPPORT */
	/** Rate index */
	t_u16 rate_index;
#if defined(STA_WEXT) || defined(UAP_WEXT)
	/** IW statistics */
	struct iw_statistics w_stats;
#endif
#ifdef UAP_WEXT
	/** Pairwise Cipher used for WPA/WPA2 mode */
	t_u16 pairwise_cipher;
	/** Group Cipher */
	t_u16 group_cipher;
	/** Protocol stored during uap wext configuratoin */
	t_u16 uap_protocol;
	/** Key Mgmt whether PSK or 1x */
	t_u16 uap_key_mgmt;
	/** Beacon IE length from hostapd */
	t_u16 bcn_ie_len;
	/** Beacon IE buffer from hostapd */
	t_u8 bcn_ie_buf[MAX_IE_SIZE];
#endif

	/** dscp mapping */
	t_u8 dscp_map[64];
	/** MLAN debug info */
	struct debug_data_priv items_priv;

	/** tcp session queue */
	struct list_head tcp_sess_queue;
	/** tcp session count */
	t_u8 tcp_sess_cnt;
	/** TCP Ack enhance flag */
	t_u8 enable_tcp_ack_enh;
	/** TCP Ack drop count */
	t_u8 tcp_ack_max_hold;
	/** TCP session spin lock */
	spinlock_t tcp_sess_lock;
	/** mcast spin lock */
	spinlock_t mcast_lock;
	/** mcast list */
	struct list_head mcast_list;
	/** num_mcast_addr */
	t_u32 num_mcast_addr;
	/** enable mc_aggr */
	t_u8 enable_mc_aggr;
	/** enable uc_nonaggr */
	t_u8 enable_uc_nonaggr;
	/** tcp list */
	struct list_head tdls_list;
	/** tdls spin lock */
	spinlock_t tdls_lock;
	/** auto tdls  flag */
	t_u8 enable_auto_tdls;
	/** check tx packet for tdls peer */
	t_u8 tdls_check_tx;
	auto_assoc auto_assoc_priv;
#if CFG80211_VERSION_CODE > KERNEL_VERSION(2, 6, 29)
	t_u32 max_tx_pending;
	t_u32 low_tx_pending;
	atomic_t wmm_tx_pending[4];
#endif
	struct sk_buff_head tx_q;
	/** per interface extra headroom */
	t_u16 extra_tx_head_len;
	/** TX status spin lock */
	spinlock_t tx_stat_lock;
	/** tx_seq_num */
	t_u8 tx_seq_num;
	/** tx status queue */
	struct list_head tx_stat_queue;
	/** rx hgm data */
	phgm_data hist_data[MAX_ANTENNA_NUM];
	t_u8 random_mac[MLAN_MAC_ADDR_LENGTH];
	BOOLEAN assoc_with_mac;
	t_u8 gtk_data_ready;
	mlan_ds_misc_gtk_rekey_data gtk_rekey_data;
	dot11_protocol tx_protocols;
	dot11_protocol rx_protocols;
	t_u8 enable_fils;
	t_u16 csi_seq;
	/** 0-disable, 1-enable */
	t_u16 csi_enable;
	/** default-ASCII, 1-binary */
	t_u8 csi_dump_format;
	/** total length of csi dump */
	t_u32 csi_dump_len;
	/** path name of csi dump */
	char csi_dump_path[64];
	/** CSI config */
	mlan_ds_csi_params csi_config;
	void *rings[RING_ID_MAX];
	t_u8 pkt_fate_monitor_enable;
	void *packet_filter;
#ifdef UAP_SUPPORT
#if defined(UAP_CFG80211) || defined(STA_CFG80211)
	t_u8 multi_ap_flag;
	station_node *vlan_sta_ptr;
	station_node *vlan_sta_list[MAX_STA_COUNT];
	moal_private *parent_priv;
#endif
#endif
	/** dhcp discover lock*/
	spinlock_t dhcp_discover_lock;
	/** DHCP DISCOVER Info queue */
	struct list_head dhcp_discover_queue;
	/** hash table for arp request */
	DECLARE_HASHTABLE(hlist, 8);
	/** arp request lock */
	spinlock_t arp_request_lock;
	/** txwatchdog disable */
	t_u8 txwatchdog_disable;

	/** secure boot uuid lower and higher 8 bytes */
	t_u64 uuid_lo;
	t_u64 uuid_hi;
	t_u16 auth_tx_wait_time;

	t_u32 rx_pkt_ac[MAX_AC_QUEUES];
};

#ifdef SDIO
#define DUMP_FW_SDIO_V2 2
#define DUMP_FW_SDIO_V3 3

#define DUMP_REG_MAX 13

typedef struct _dump_reg_t {
	t_u8 reg_table[DUMP_REG_MAX];
	t_u8 reg_table_size;
} dump_reg_t;
#endif

#define FW_NAMW_MAX_LEN 64

/** card info */
typedef struct _card_info {
	/** support embeded supp */
	t_bool embedded_supp;
	/** support drcs */
	t_bool drcs;
	/** support Go NOA*/
	t_bool go_noa;
	/** support V16_FW_API*/
	t_bool v16_fw_api;
	/** support V17_FW_API*/
	t_bool v17_fw_api;
	/** support pmic */
	t_bool pmic;
	/** support antcfg */
	t_bool antcfg;
	/** support cal_data_cfg */
	t_bool cal_data_cfg;
	/** support WLAN_LOW_POWER_ENABLE */
	t_bool low_power_enable;
	/** rx_rate_max for hist_data: 11N: 76  11AC:196 11AX: 412 */
	t_u16 rx_rate_max;
	t_u8 histogram_table_num;
	/* feature_control */
	t_u32 feature_control;
	/* Revision id register */
	t_u32 rev_id_reg;
	/* host interface selection reg*/
	t_u32 host_strap_reg;
	/* Chip Magic Register */
	t_u32 magic_reg;
	/** Chip boot mode reg */
	t_u32 boot_mode_reg;
	/* FW Name */
	char fw_name[FW_NAMW_MAX_LEN];
	char fw_name_wlan[FW_NAMW_MAX_LEN];
#ifdef SDIO
	t_u8 dump_fw_info;
	t_u8 dump_fw_ctrl_reg;
	t_u8 dump_fw_start_reg;
	t_u8 dump_fw_end_reg;
	t_u8 dump_fw_host_ready;
	dump_reg_t dump_reg;
	t_u8 scratch_reg;
	t_u8 func1_reg_start;
	t_u8 func1_reg_end;
	t_u32 slew_rate_reg;
	t_u8 slew_rate_bit_offset;
	t_u32 fw_winner_status_reg;
#endif
#if defined(SDIO) || defined(PCIE)
	t_u32 fw_stuck_code_reg;
	t_u32 fw_reset_reg;
	t_u8 fw_reset_val;
	t_u32 fw_wakeup_reg;
	t_u8 fw_wakeup_val;
#endif
	t_u8 sniffer_support;
	t_u8 per_pkt_cfg_support;
	t_u8 host_mlme_required;
} card_info;

/** channel_field.flags */
#define CHANNEL_FLAGS_TURBO 0x0010
#define CHANNEL_FLAGS_CCK 0x0020
#define CHANNEL_FLAGS_OFDM 0x0040
#define CHANNEL_FLAGS_2GHZ 0x0080
#define CHANNEL_FLAGS_5GHZ 0x0100
#define CHANNEL_FLAGS_ONLY_PASSIVSCAN_ALLOW 0x0200
#define CHANNEL_FLAGS_DYNAMIC_CCK_OFDM 0x0400
#define CHANNEL_FLAGS_GFSK 0x0800
struct channel_field {
	/** frequency */
	t_u16 frequency;
	/** flags */
	t_u16 flags;
} __packed;

/** mcs_field.known */
#define MCS_KNOWN_BANDWIDTH 0x01
#define MCS_KNOWN_MCS_INDEX_KNOWN 0x02
#define MCS_KNOWN_GUARD_INTERVAL 0x04
#define MCS_KNOWN_HT_FORMAT 0x08
#define MCS_KNOWN_FEC_TYPE 0x10
#define MCS_KNOWN_STBC_KNOWN 0x20
#define MCS_KNOWN_NESS_KNOWN 0x40
#define MCS_KNOWN_NESS_DATA 0x80
/** bandwidth */
#define RX_BW_20 0
#define RX_BW_40 1
#define RX_BW_20L 2
#define RX_BW_20U 3
#define RX_BW_80 4
#define RX_HE_BW_20 0
#define RX_HE_BW_40 1
#define RX_HE_BW_80 2
#define RX_HE_BW_160 3
/** mcs_field.flags
The flags field is any combination of the following:
0x03    bandwidth - 0: 20, 1: 40, 2: 20L, 3: 20U
0x04    guard interval - 0: long GI, 1: short GI
0x08    HT format - 0: mixed, 1: greenfield
0x10    FEC type - 0: BCC, 1: LDPC
0x60    Number of STBC streams
0x80    Ness - bit 0 (LSB) of Number of extension spatial streams */
struct mcs_field {
	/** known */
	t_u8 known;
	/** flags */
	t_u8 flags;
	/** mcs */
	t_u8 mcs;
} __packed;

/** vht_field.known */
#define VHT_KNOWN_STBC 0x0001
#define VHT_KNOWN_TXOP_PS_NA 0x0002
#define VHT_KNOWN_GI 0x0004
#define VHT_KNOWN_SGI_NSYM_DIS 0x0008
#define VHT_KNOWN_LDPC_EXTRA_OFDM_SYM 0x0010
#define VHT_KNOWN_BEAMFORMED 0x0020
#define VHT_KNOWN_BANDWIDTH 0x0040
#define VHT_KNOWN_GROUP_ID 0x0080
#define VHT_KNOWN_PARTIAL_AID 0x0100

/** vht_field.flags */
#define VHT_FLAG_STBC 0x01
#define VHT_FLAG_TXOP_PS_NA 0x02
#define VHT_FLAG_SGI 0x04
#define VHT_FLAG_SGI_NSYM_M10_9 0x08
#define VHT_FLAG_LDPC_EXTRA_OFDM_SYM 0x10
#define VHT_FLAG_BEAMFORMED 0x20

/** vht_field.coding */
#define VHT_CODING_LDPC_USER0 0x01
#define VHT_CODING_LDPC_USER1 0x02
#define VHT_CODING_LDPC_USER2 0x04
#define VHT_CODING_LDPC_USER3 0x08

/** vht_field */
struct vht_field {
	/** known */
	t_u16 known;
	/** flags */
	t_u8 flags;
	/** bandwidth */
	t_u8 bandwidth;
	/** mcs_nss for up to 4 users */
	t_u8 mcs_nss[4];
	/** coding for up to 4 users */
	t_u8 coding;
	/** group_id */
	t_u8 group_id;
	/** partial_aid */
	t_u16 partial_aid;
} __packed;

#define HE_BSS_COLOR_KNOWN 0x0002
#define HE_BEAM_CHANGE_KNOWN 0x0004
#define HE_UL_DL_KNOWN 0x0008
#define HE_MCS_KNOWN 0x0020
#define HE_DCM_KNOWN 0x0040
#define HE_CODING_KNOWN 0x0080
#define HE_BW_KNOWN 0x4000
#define HE_DATA_GI_KNOWN 0x0002
#define HE_MU_DATA 0x0002
#define HE_CODING_LDPC_USER0 0x2000
/** he_field - COCO */
struct he_field {
	t_u16 data1;
	t_u16 data2;
	t_u16 data3;
	t_u16 data4;
	t_u16 data5;
	t_u16 data6;
} __packed;

extern t_u8 ru_signal[16][9];
extern t_u8 ru_signal_106[14][9];
extern t_u8 ru_signal_52[9];

#define MLAN_20_BIT_CH1P 0xC0000000
#define MLAN_20_BIT_CH1S 0x0000003F
#define MLAN_20_BIT_CH2 0x007F8000
#define MLAN_80_CENTER_RU 0x00004000
#define MLAN_160_CENTER_RU 0x40000000
#define MLAN_20_BIT_CH3 0x00003FC0
#define MLAN_20_BIT_CH4 0x7F800000
#define MLAN_BIT_160_CH3 0x003FC000
#define MLAN_BIT_160_CH4 0x03FC0000

#define MLAN_DECODE_RU_SIGNALING_CH1(out, x, y)                                \
	{                                                                      \
		x = (((x << 8) & MLAN_20_BIT_CH1P)) >> 30;                     \
		out = x | ((y & MLAN_20_BIT_CH1S) << 2);                       \
	}

#define MLAN_DECODE_RU_SIGNALING_CH3(out, x, y)                                \
	{                                                                      \
		out = ((y & MLAN_20_BIT_CH3) >> 6);                            \
	}

#define MLAN_DECODE_RU_SIGNALING_CH2(out, x, y)                                \
	{                                                                      \
		out = ((y & MLAN_20_BIT_CH2) >> 15);                           \
	}

#define MLAN_DECODE_RU_SIGNALING_CH4(out, x, y)                                \
	{                                                                      \
		out = ((y & MLAN_20_BIT_CH4) >> 23);                           \
	}

#define MLAN_DECODING_160_RU_CH3(out, x, y)                                    \
	{                                                                      \
		out = ((y & MLAN_BIT_160_CH3) >> 5);                           \
	}

#define MLAN_DECODING_160_RU_CH4(out, x, y)                                    \
	{                                                                      \
		out = ((y & MLAN_BIT_160_CH4) >> 22);                          \
	}

#define RU_SIGNAL_52_TONE 112
#define TONE_MAX_USERS_52 4
#define TONE_MAX_USERS_242 3
#define RU_SIGNAL_26_TONE 0
#define TONE_MAX_USERS_26 8
#define RU_26_TONE_LIMIT 15
#define RU_TONE_LIMIT 96
#define RU_80_106_TONE 128
#define RU_40_242_TONE 192
#define RU_80_484_TONE 200
#define RU_160_996_TONE 208
#define RU_TONE_26 4
#define RU_TONE_52 5
#define RU_TONE_106 6
#define RU_TONE_242 7
#define RU_TONE_484 8
#define RU_TONE_996 9

#define MLAN_DECODE_RU_TONE(x, y, tone)                                           \
	{                                                                         \
		if ((x == RU_SIGNAL_52_TONE)) {                                   \
			if (((y + 1) <= TONE_MAX_USERS_52)) {                     \
				tone = RU_TONE_52;                                \
			} else {                                                  \
				y = (y + 1) - TONE_MAX_USERS_52;                  \
			}                                                         \
		} else if (x == RU_SIGNAL_26_TONE) {                              \
			if ((y + 1) <= TONE_MAX_USERS_26) {                       \
				tone = RU_TONE_26;                                \
			} else {                                                  \
				y = (y + 1) - TONE_MAX_USERS_26;                  \
			}                                                         \
		} else if (x <= RU_TONE_LIMIT) {                                  \
			t_u32 ru_arr_idx;                                         \
			ru_arr_idx = x > RU_26_TONE_LIMIT ? 1 : 0;                \
			if ((y + 1) > (ru_arr_idx ? ru_signal_106[x / 8][8] :     \
						    ru_signal[x][8])) {           \
				y = (y + 1) -                                     \
				    (ru_arr_idx ? ru_signal_106[x / 8][8] :       \
						  ru_signal[x][8]);               \
			} else {                                                  \
				t_u32 ind = 0;                                    \
				t_u32 idx = 0;                                    \
				while (ind < 8) {                                 \
					t_u32 tn =                                \
						ru_arr_idx ?                      \
							ru_signal_106[x / 8]      \
								     [7 - ind] :  \
							ru_signal[x][7 - ind];    \
					ind++;                                    \
					if (tn == 0x1 || tn == 0x0 ||             \
					    tn == 0x2) {                          \
						if (idx == y) {                   \
							tone = tn ? (tn ==        \
								     2) ?         \
								    RU_TONE_106 : \
								    RU_TONE_52 :  \
								    RU_TONE_26;   \
							break;                    \
						} else {                          \
							idx++;                    \
						}                                 \
					}                                         \
				}                                                 \
			}                                                         \
		} else if (x == RU_80_106_TONE) {                                 \
			if ((y + 1) > TONE_MAX_USERS_242) {                       \
				y = (y + 1) - TONE_MAX_USERS_242;                 \
			} else {                                                  \
				tone = (y == 2) ? RU_TONE_106 :                   \
				       (y == 1) ? 0 :                             \
						  RU_TONE_106;                    \
			}                                                         \
		} else if (x == RU_40_242_TONE) {                                 \
			if (!y) {                                                 \
				tone = RU_TONE_242;                               \
			} else {                                                  \
				y--;                                              \
			}                                                         \
		} else if (x == RU_80_484_TONE) {                                 \
			if (!y) {                                                 \
				tone = RU_TONE_484;                               \
			} else {                                                  \
				y--;                                              \
			}                                                         \
		} else if (x == RU_160_996_TONE) {                                \
			if (!y) {                                                 \
				tone = RU_TONE_996;                               \
			} else {                                                  \
				y--;                                              \
			}                                                         \
		}                                                                 \
	}

/** radiotap_body.flags */
#define RADIOTAP_FLAGS_DURING_CFG 0x01
#define RADIOTAP_FLAGS_SHORT_PREAMBLE 0x02
#define RADIOTAP_FLAGS_WEP_ENCRYPTION 0x04
#define RADIOTAP_FLAGS_WITH_FRAGMENT 0x08
#define RADIOTAP_FLAGS_INCLUDE_FCS 0x10
#define RADIOTAP_FLAGS_PAD_BTW_HEADER_PAYLOAD 0x20
#define RADIOTAP_FLAGS_FAILED_FCS_CHECK 0x40
#define RADIOTAP_FLAGS_USE_SGI_HT 0x80

struct radiotap_body {
	/** timestamp */
	t_u64 timestamp; /* 0~7 bytes */
	/** flags */
	t_u8 flags; /* 8th byte */
	/** rate for LG pkt, RATE flag will be present, it shows datarate in
	 * 500Kbps. For HT/VHT pkt, RATE flag will not be present, it is not
	 * used. */
	t_u8 rate; /* 9th byte */
	/** channel */
	struct channel_field channel; /* 10~13 bytes */
	/** antenna_signal */
	t_s8 antenna_signal; /* 14th byte */
	/** antenna_noise */
	t_s8 antenna_noise; /* 15th byte */
	/** antenna */
	t_u8 antenna; /* 16th byte */
	/** Required Alignment */
	t_u8 align_1; /* 17th byte */
	/** rx flags Required Alignment is 2 */
	t_u16 rx_flags; /* 18~19 bytes */
} __packed;

struct radiotap_header {
	struct ieee80211_radiotap_header hdr;
	struct radiotap_body body;
} __packed;

/** Roam offload config parameters */
typedef struct woal_priv_fw_roam_offload_cfg {
	/* User set passphrase*/
	t_u8 userset_passphrase;
	/* BSSID for fw roaming/auto_reconnect*/
	t_u8 bssid[MLAN_MAC_ADDR_LENGTH];
	/* Retry_count for fw roaming/auto_reconnect*/
	t_u8 retry_count;
	/* Condition to trigger roaming
	 * Bit0 : RSSI low trigger
	 * Bit1 : Pre-beacon lost trigger
	 * Bit2 : Link Lost trigger
	 * Bit3 : Deauth by ext-AP trigger
	 * Bit4 ~ Bit15 : Reserved
	 * value 0 : no trigger
	 * value 0xff : invalid
	 */
	t_u16 trigger_condition;
	/* SSID List(White list)*/
	mlan_ds_misc_ssid_list ssid_list;
	/* Black list(BSSID list)*/
	mlan_ds_misc_roam_offload_aplist black_list;

	/* RSSI paramters set flag*/
	t_u8 rssi_param_set_flag;
	/* MAX_RSSI for fw roaming*/
	t_u8 max_rssi;
	/*  MIN_RSSI for fw roaming*/
	t_u8 min_rssi;
	/*  Step_RSSI for fw roaming*/
	t_u8 step_rssi;

	/* BAND and RSSI_HYSTERESIS set flag*/
	t_u8 band_rssi_flag;
	mlan_ds_misc_band_rssi band_rssi;

	/* BGSCAN params set flag*/
	t_u8 bgscan_set_flag;
	mlan_ds_misc_bgscan_cfg bgscan_cfg;

	/* EES mode params set flag*/
	t_u8 ees_param_set_flag;
	mlan_ds_misc_ees_cfg ees_cfg;

	/* Beacon miss threshold*/
	t_u8 bcn_miss_threshold;

	/* Beacon miss threshold*/
	t_u8 pre_bcn_miss_threshold;

	/* scan repeat count*/
	t_u16 repeat_count;
} woal_roam_offload_cfg;
#ifdef STA_CFG80211
int woal_set_clear_pmk(moal_private *priv, t_u8 action);
#endif
int woal_config_fw_roaming(moal_private *priv, t_u8 cfg_mode,
			   woal_roam_offload_cfg *roam_offload_cfg);
int woal_enable_fw_roaming(moal_private *priv, int data);

#define GTK_REKEY_OFFLOAD_DISABLE 0
#define GTK_REKEY_OFFLOAD_ENABLE 1
#define GTK_REKEY_OFFLOAD_SUSPEND 2

/** Supported bandwidth for monitor mode */
enum {
	SNIFF_BW_20MHZ = 0,
	SNIFF_BW_40MHZ = 1,
	SNIFF_BW_40MHZ_ABOVE = 1,
	SNIFF_BW_40MHZ_BELOW = 3,
	SNIFF_BW_80MHZ = 4,
};

/** Monitor Band Channel Config */
typedef struct _netmon_band_chan_cfg {
	t_u32 band;
	t_u32 channel;
	t_u32 chan_bandwidth;
} netmon_band_chan_cfg;

#if defined(STA_CFG80211) && defined(UAP_CFG80211)
typedef struct _monitor_iface {
	/* The priv data of interface on which the monitor iface is based */
	moal_private *priv;
	struct wireless_dev wdev;
	int radiotap_enabled;
	/* The net_device on which the monitor iface is based. */
	struct net_device *base_ndev;
	struct net_device *mon_ndev;
	char ifname[IFNAMSIZ];
	int flag;
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
	struct cfg80211_chan_def chandef;
#endif
	/** Netmon Band Channel Config */
	netmon_band_chan_cfg band_chan_cfg;
	/** Monitor device statistics structure */
	struct net_device_stats stats;
} monitor_iface;
#endif

#define MAX_KEEP_ALIVE_ID 4
#define MAX_KEEP_ALIVE_RX_ID 4

/** Operation data structure for MOAL bus interfaces */
typedef struct _moal_if_ops {
	mlan_status (*register_dev)(moal_handle *handle);
	void (*unregister_dev)(moal_handle *handle);
	mlan_status (*read_reg)(moal_handle *handle, t_u32 reg, t_u32 *data);
	mlan_status (*write_reg)(moal_handle *handle, t_u32 reg, t_u32 data);
	mlan_status (*read_data_sync)(moal_handle *handle, mlan_buffer *pmbuf,
				      t_u32 port, t_u32 timeout);
	mlan_status (*write_data_sync)(moal_handle *handle, mlan_buffer *pmbuf,
				       t_u32 port, t_u32 timeout);
	mlan_status (*get_fw_name)(moal_handle *handle);
	void (*dump_fw_info)(moal_handle *handle);
	int (*dump_reg_info)(moal_handle *handle, t_u8 *buf);
	void (*card_reset)(moal_handle *handle);
	void (*reg_dbg)(moal_handle *handle);
	t_u8 (*is_second_mac)(moal_handle *handle);
} moal_if_ops;

#define WIFI_DIRECT_KERNEL_VERSION KERNEL_VERSION(2, 6, 39)

/**  Extended flags */
enum ext_mod_params {
	EXT_HW_TEST,
#ifdef CONFIG_OF
	EXT_DTS_ENABLE,
#endif
	EXT_REQ_FW_NOWAIT,
	EXT_FW_SERIAL,
	EXT_PM_KEEP_POWER,
#ifdef SDIO
	EXT_INTMODE,
#ifdef SDIO_SUSPEND_RESUME
	EXT_SHUTDOWN_HS,
#endif
#endif
	EXT_START_11AI_SCAN,
#if defined(USB)
	EXT_SKIP_FWDNLD,
#endif
	EXT_AGGR_CTRL,
	EXT_LOW_PW_MODE,
#ifdef SDIO
	EXT_SDIO_RX_AGGR,
#endif
	EXT_PMIC,
	EXT_DISCONNECT_ON_SUSPEND,
	EXT_HS_MIMO_SWITCH,
	EXT_FIX_BCN_BUF,
	EXT_NAPI,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 14, 0)
	EXT_DFS_OFFLOAD,
#endif
	EXT_CFG80211_DRCS,
	EXT_DISABLE_REGD_BY_DRIVER,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 14, 0)
	EXT_COUNTRY_IE_IGNORE,
	EXT_BEACON_HINTS,
#endif
	EXT_ROAMOFFLOAD_IN_HS,
#ifdef STA_CFG80211
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
	EXT_HOST_MLME,
#endif
#endif
	EXT_TX_WORK,
	EXT_TX_SKB_CLONE,
	EXT_PMQOS,
	EXT_CHAN_TRACK,
	EXT_DMCS,
	EXT_PREF_DBC,
#if CFG80211_VERSION_CODE > KERNEL_VERSION(4, 12, 14)
	EXT_CFG80211_EAPOL_OFFLOAD,
#endif
	EXT_MAX_PARAM,
};

/** Module parameter data structure for MOAL */
typedef struct _moal_mod_para {
	t_u8 ext_flgs[DIV_ROUND_UP(EXT_MAX_PARAM, 8)];
	/* BIT24 ~ BIT32 reserved */
	t_u8 flag;
	char *fw_name;
	int fw_reload;
	int auto_fw_reload;
	char *mac_addr;
#ifdef MFG_CMD_SUPPORT
	int mfg_mode;
#endif /* MFG_CMD_SUPPORT */
	int rf_test_mode;
	char *hw_name;
	int drv_mode;
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
	int mon_filter;
#endif
#ifdef DEBUG_LEVEL1
	int drvdbg;
#endif
#ifdef STA_SUPPORT
	int max_sta_bss;
	char *sta_name;
#endif /* STA_SUPPORT */
#ifdef UAP_SUPPORT
	int max_uap_bss;
	char *uap_name;
	int uap_max_sta;
	int wacp_mode;
#endif /* UAP_SUPPORT */
	unsigned int fw_data_cfg;
#ifdef WIFI_DIRECT_SUPPORT
	int max_wfd_bss;
	char *wfd_name;
#if defined(STA_CFG80211) && defined(UAP_CFG80211)
	int max_vir_bss;
#endif
#endif /* WIFI_DIRECT_SUPPORT */
	char *nan_name;
	int max_nan_bss;
	int auto_ds;
	int net_rx;
	int amsdu_deaggr;
	int tx_budget;
	int mclient_scheduling;
	int ext_scan;
	int bootup_cal_ctrl;
	int ps_mode;
	int p2a_scan;
	/** scan chan gap */
	int scan_chan_gap;
	int sched_scan;
	int max_tx_buf;
#if defined(SDIO)
	int gpiopin;
#endif
#if defined(STA_SUPPORT)
	int cfg_11d;
#endif
#if defined(UAP_SUPPORT)
	int custom_11d_bcn_country_ie_en;
#endif
#if defined(SDIO)
	int slew_rate;
#endif
	char *dpd_data_cfg;
	char *init_cfg;
	char *cal_data_cfg;
	char *txpwrlimit_cfg;
	int cntry_txpwr;
	char *init_hostcmd_cfg;
	char *band_steer_cfg;
	int cfg80211_wext;
	int wq_sched_prio;
	int wq_sched_policy;
	int rx_work;
#ifdef USB
	int usb_aggr;
#endif
#ifdef PCIE
	int pcie_int_mode;
	int ring_size;
#endif /* PCIE */
#ifdef ANDROID_KERNEL
	int wakelock_timeout;
#endif
	unsigned int dev_cap_mask;
	int pmic;
	int antcfg;
	/** dmcs*/
	int dmcs;
	/** pref_dbc*/
	int pref_dbc;
	unsigned int uap_oper_ctrl;
	int hs_wake_interval;
	int indication_gpio;
	int indrstcfg;
#ifdef WIFI_DIRECT_SUPPORT
	int GoAgeoutTime;
#endif
	int gtk_rekey_offload;
	t_u16 multi_dtim;
	t_u16 inact_tmo;
	int drcs_chantime_mode;
	char *reg_alpha2;
	int dfs53cfg;
	t_u8 mcs32;

#if defined(CONFIG_RPS)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 7, 0)
	/* rps module param */
	int rps;
#endif
#endif
	int edmac_ctrl;
	int keep_previous_scan;
	int auto_11ax;
	/** hs_auto_arp setting */
	int hs_auto_arp;
	/** Dual-BT **/
	int dual_nb;
	/* reject addba req config for HS or FW Auto-reconnect */
	t_u32 reject_addba_req;
	/** disable_11h_tpc setting */
	int disable_11h_tpc;
	/* tpe_ie_ignore setting */
	int tpe_ie_ignore;
	/* make_before_break during roam */
	int make_before_break;
} moal_mod_para;

void woal_tp_acnt_timer_func(void *context);
void woal_set_tp_state(moal_private *priv);
#define MAX_TP_ACCOUNT_DROP_POINT_NUM 5
#define RX_DROP_P1 (MAX_TP_ACCOUNT_DROP_POINT_NUM)
#define RX_DROP_P2 (MAX_TP_ACCOUNT_DROP_POINT_NUM + 1)
#define RX_DROP_P3 (MAX_TP_ACCOUNT_DROP_POINT_NUM + 2)
#define RX_DROP_P4 (MAX_TP_ACCOUNT_DROP_POINT_NUM + 3)
#define RX_DROP_P5 (MAX_TP_ACCOUNT_DROP_POINT_NUM + 4)
#define TXRX_MAX_SAMPLE 50
#define RX_TIME_PKT (MAX_TP_ACCOUNT_DROP_POINT_NUM + 5)
#define TX_TIME_PKT (MAX_TP_ACCOUNT_DROP_POINT_NUM + 6)

typedef struct _moal_tp_acnt_t {
	/* TX accounting */
	unsigned long tx_packets[MAX_TP_ACCOUNT_DROP_POINT_NUM];
	unsigned long tx_packets_last[MAX_TP_ACCOUNT_DROP_POINT_NUM];
	unsigned long tx_packets_rate[MAX_TP_ACCOUNT_DROP_POINT_NUM];
	unsigned long tx_bytes[MAX_TP_ACCOUNT_DROP_POINT_NUM];
	unsigned long tx_bytes_last[MAX_TP_ACCOUNT_DROP_POINT_NUM];
	unsigned long tx_bytes_rate[MAX_TP_ACCOUNT_DROP_POINT_NUM];
	unsigned long tx_amsdu_cnt;
	unsigned long tx_amsdu_cnt_last;
	unsigned long tx_amsdu_cnt_rate;
	unsigned long tx_amsdu_pkt_cnt;
	unsigned long tx_amsdu_pkt_cnt_last;
	unsigned long tx_amsdu_pkt_cnt_rate;
	unsigned long tx_intr_cnt;
	unsigned long tx_intr_last;
	unsigned long tx_intr_rate;
	unsigned long tx_pending;
	unsigned long tx_xmit_skb_realloc_cnt;
	unsigned long tx_stop_queue_cnt;
	unsigned long tx_delay_driver[TXRX_MAX_SAMPLE];
	/* drop_point1 to drop_point3 time */
	unsigned long tx_delay1_driver[TXRX_MAX_SAMPLE];

	/** RX accounting */
	unsigned long rx_packets[MAX_TP_ACCOUNT_DROP_POINT_NUM];
	unsigned long rx_packets_last[MAX_TP_ACCOUNT_DROP_POINT_NUM];
	unsigned long rx_packets_rate[MAX_TP_ACCOUNT_DROP_POINT_NUM];
	unsigned long rx_bytes[MAX_TP_ACCOUNT_DROP_POINT_NUM];
	unsigned long rx_bytes_last[MAX_TP_ACCOUNT_DROP_POINT_NUM];
	unsigned long rx_bytes_rate[MAX_TP_ACCOUNT_DROP_POINT_NUM];
	unsigned long rx_amsdu_cnt;
	unsigned long rx_amsdu_cnt_last;
	unsigned long rx_amsdu_cnt_rate;
	unsigned long rx_amsdu_pkt_cnt;
	unsigned long rx_amsdu_pkt_cnt_last;
	unsigned long rx_amsdu_pkt_cnt_rate;
	unsigned long rx_intr_cnt;
	unsigned long rx_intr_last;
	unsigned long rx_intr_rate;
	unsigned long rx_pending;
	unsigned long rx_paused_cnt;
	unsigned long rx_rdptr_full_cnt;
	unsigned long rx_delay1_driver[TXRX_MAX_SAMPLE];
	unsigned long rx_delay2_driver[TXRX_MAX_SAMPLE];
	unsigned long rx_delay_kernel[TXRX_MAX_SAMPLE];
	unsigned long rx_amsdu_delay[TXRX_MAX_SAMPLE];
	unsigned long rx_amsdu_copy_delay[TXRX_MAX_SAMPLE];
	t_u8 rx_amsdu_index;
	t_u8 rx_index;
	t_u8 tx_index;
	/* TP account mode 0-disable 1-enable */
	unsigned int on;
	/* drop point */
	unsigned int drop_point;
	/* periodic timer */
	moal_drv_timer timer;
} moal_tp_acnt_t;

/** Handle data structure for MOAL */
struct _moal_handle {
	/** MLAN adapter structure */
	t_void *pmlan_adapter;
	/** Private pointer */
	moal_private *priv[MLAN_MAX_BSS_NUM];
	/** Priv number */
	t_u8 priv_num;
	/** Bss attr */
	moal_drv_mode drv_mode;

#if defined(STA_CFG80211) && defined(UAP_CFG80211)
	/** Monitor interface */
	monitor_iface *mon_if;
#endif

	/** set mac address flag */
	t_u8 set_mac_addr;
	/** MAC address */
	t_u8 mac_addr[ETH_ALEN];
#ifdef CONFIG_PROC_FS
	/** Proc top level directory entry */
	struct proc_dir_entry *proc_wlan;
	/** Proc top level directory entry name */
	char proc_wlan_name[32];
#endif
#ifdef USB
	/** Firmware download skip flag */
	t_u8 skip_fw_dnld;
#endif /* USB */
	/** Firmware */
	const struct firmware *firmware;
	/** Firmware request start time */
	wifi_timeval req_fw_time;
	/** Init config file */
	const struct firmware *init_cfg_data;
	/** Init config file */
	const struct firmware *user_data;
	/** Init user configure wait queue token */
	t_u16 init_user_conf_wait_flag;
	/** Init user configure file wait queue */
	wait_queue_head_t init_user_conf_wait_q __ATTRIB_ALIGN__;
	/** dpd config file */
	const struct firmware *dpd_data;
	/** txpwr data file */
	const struct firmware *txpwr_data;
	/** Operation Mode PSD String */
	char mode_psd_string[64];
	/** Load time file name */
	char mode_psd_file[64];
	/** Hotplug device */
	struct device *hotplug_device;
	/** STATUS variables */
	MOAL_HARDWARE_STATUS hardware_status;
	BOOLEAN fw_reload;
	BOOLEAN fw_reseting;
	/** POWER MANAGEMENT AND PnP SUPPORT */
	BOOLEAN surprise_removed;
	/** Firmware release number */
	t_u32 fw_release_number;
	/** Firmware Hotfix version */
	t_u8 fw_hotfix_version;
	/** Firmware support bands */
	t_u16 fw_bands;
	/** ECSA support */
	t_u8 fw_ecsa_enable;
	/* Firmware support cmd_tx_data */
	t_u8 cmd_tx_data;
	/** FW support security key for rgpower table */
	t_u8 sec_rgpower;
	/** FW ROAMING support */
	t_u8 fw_roam_enable;
	/** FW ROAMING capability in fw */
	t_u8 fw_roaming_support;
	/** Retry count for auto reconnect based on FW ROAMING*/
	t_u16 auto_reconnect_retry_count;
	/** The SSID for auto reconnect FW ROAMING*/
	mlan_802_11_ssid auto_reconnect_ssid;
	/** The BSSID for auto reconnect FW ROAMING*/
	mlan_802_11_mac_addr auto_reconnect_bssid;
	/** The parameters for FW  ROAMING*/
	woal_roam_offload_cfg fw_roam_params;
	/** The keys for FW  ROAMING*/
	mlan_ds_passphrase ssid_passphrase[MAX_SEC_SSID_NUM];

	/** Getlog support */
	t_u8 fw_getlog_enable;
	/** Init wait queue token */
	t_u16 init_wait_q_woken;
	/** Init wait queue */
	wait_queue_head_t init_wait_q __ATTRIB_ALIGN__;
	/** Device suspend flag */
	BOOLEAN is_suspended;
#ifdef SDIO_SUSPEND_RESUME
	/** suspend notify flag */
	BOOLEAN suspend_notify_req;
	/** hs_shutdown in process flag  */
	BOOLEAN shutdown_hs_in_process;
#endif
	/** Suspend wait queue token */
	t_u16 suspend_wait_q_woken;
	/** Suspend wait queue */
	wait_queue_head_t suspend_wait_q __ATTRIB_ALIGN__;
	/** Host Sleep activated flag */
	t_u8 hs_activated;
	/** Host Sleep activated event wait queue token */
	t_u16 hs_activate_wait_q_woken;
	/** Host Sleep activated event wait queue */
	wait_queue_head_t hs_activate_wait_q __ATTRIB_ALIGN__;
	/** auto_arp and ipv6 offload enable/disable flag */
	t_u8 hs_auto_arp;
#ifdef IMX_SUPPORT
	/** wakeup irq number */
	int irq_oob_wakeup;
	/** wakeup notify flag */
	bool wake_by_wifi;
#endif /* IMX_SUPPORT */
	/** Card pointer */
	t_void *card;
	/** Rx pending in MLAN */
	atomic_t rx_pending;
	/** Tx packet pending count in mlan */
	atomic_t tx_pending;
	/** IOCTL pending count in mlan */
	atomic_t ioctl_pending;
	/** lock count */
	atomic_t lock_count;
	/** Malloc count */
	atomic_t malloc_count;
	/** vmalloc count */
	atomic_t vmalloc_count;
	/** mlan buffer alloc count */
	atomic_t mbufalloc_count;
#ifdef PCIE
	/** Malloc consistent count */
	atomic_t malloc_cons_count;
#endif
	/** hs skip count */
	t_u32 hs_skip_count;
	/** hs force count */
	t_u32 hs_force_count;
	/** suspend_fail flag */
	BOOLEAN suspend_fail;
#ifdef REASSOCIATION
	/** Re-association thread */
	moal_thread reassoc_thread;
	/** Re-association timer set flag */
	BOOLEAN is_reassoc_timer_set;
	/** Re-association timer */
	moal_drv_timer reassoc_timer __ATTRIB_ALIGN__;
	/**  */
	struct semaphore reassoc_sem;
	/** Bitmap for re-association on/off */
	t_u8 reassoc_on;
#endif /* REASSOCIATION */
	/** RTT capability */
	wifi_rtt_capabilities rtt_capa;
	/** RTT config */
	wifi_rtt_config_params_t rtt_params;
	/** Driver workqueue */
	struct workqueue_struct *workqueue;
	/** main work */
	struct work_struct main_work;
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
	/** host_mlme_priv */
	moal_private *host_mlme_priv;
	/** Host Mlme Work struct**/
	struct work_struct host_mlme_work;
#endif
	/** Driver event workqueue */
	struct workqueue_struct *evt_workqueue;
	/** event  work */
	struct work_struct evt_work;
#if defined(SDIO) || defined(USB)
	/** Driver workqueue */
	struct workqueue_struct *rx_workqueue;
	/** main work */
	struct work_struct rx_work;
#endif
#ifdef PCIE
	/** Driver pcie rx cmd resp workqueue */
	struct workqueue_struct *pcie_cmd_resp_workqueue;
	/** pcie rx cmd resp work */
	struct work_struct pcie_cmd_resp_work;
	/** pcie delayed work */
	struct delayed_work pcie_delayed_tx_work;
#ifdef TASKLET_SUPPORT
	/* pcie rx data tasklet */
	struct tasklet_struct pcie_rx_task;
	/* pcie tx complete tasklet */
	struct tasklet_struct pcie_tx_complete_task;
#else
	/** Driver pcie rx workqueue */
	struct workqueue_struct *pcie_rx_workqueue;
	/* pcie rx work */
	struct work_struct pcie_rx_work;
	/** Driver pcie tx complete workqueue */
	struct workqueue_struct *pcie_tx_complete_workqueue;
	/* pcie tx complete work */
	struct work_struct pcie_tx_complete_work;
#endif
#endif
	/** event spin lock */
	spinlock_t evt_lock;
	/** event queue */
	struct list_head evt_queue;
	/** tx workqueue */
	struct workqueue_struct *tx_workqueue;
	/** tx work */
	struct work_struct tx_work;

	/** remain on channel flag */
	t_u8 remain_on_channel;
	/** bss index for remain on channel */
	t_u8 remain_bss_index;
	/** wifi hal enabled flag */
	t_u8 wifi_hal_flag;
#if defined(STA_CFG80211) || defined(UAP_CFG80211)
	struct wiphy *wiphy;
	/** Country code for regulatory domain */
	t_u8 country_code[COUNTRY_CODE_LEN];
	/** dfs_region */
	t_u8 dfs_region;
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 0, 0)
	/** regulatory work */
	struct work_struct regulatory_work;
#endif
	/** band */
	enum ieee80211_band band;
	/** first scan done flag */
	t_u8 first_scan_done;
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(2, 6, 39)
	/** remain_on_channel timer set flag */
	BOOLEAN is_remain_timer_set;
	/** remani_on_channel_timer */
	moal_drv_timer remain_timer __ATTRIB_ALIGN__;
	/** ieee802_11_channel */
	struct ieee80211_channel chan;
#if CFG80211_VERSION_CODE < KERNEL_VERSION(3, 8, 0)
	/** channel type */
	enum nl80211_channel_type channel_type;
#endif
	/** cookie */
	t_u64 cookie;
#endif

#ifdef WIFI_DIRECT_SUPPORT
	/** NoA duration */
	t_u32 noa_duration;
	/** NoA interval */
	t_u32 noa_interval;
	/** miracast mode */
	t_u8 miracast_mode;
	/** scan time in miracast mode */
	t_u16 miracast_scan_time;
	/** GO timer set flag */
	BOOLEAN is_go_timer_set;
	/** GO timer */
	moal_drv_timer go_timer __ATTRIB_ALIGN__;
#endif
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 2, 0)
	/** cfg80211_suspend status */
	t_u8 cfg80211_suspend;
#endif
#endif
	/** FW debug flag */
	t_u8 fw_dbg;
	/** reg debug flag */
	t_u8 reg_dbg;
#ifdef SDIO
	/** sdio_blk_size */
	t_u32 sdio_blk_size;
#endif /* SDIO */
	/** Netlink kernel socket */
	struct sock *nl_sk;
	/** Netlink kernel socket number */
	t_u32 netlink_num;
	/** w_stats wait queue token */
	BOOLEAN meas_wait_q_woken;
	/** w_stats wait queue */
	wait_queue_head_t meas_wait_q __ATTRIB_ALIGN__;
	/** Measurement start jiffes */
	long meas_start_jiffies;
	/** CAC checking period flag */
	BOOLEAN cac_period;
	/** CAC timer jiffes */
	long cac_timer_jiffies;
	/** User NOP Period in sec */
	t_u16 usr_nop_period_sec;
	/** BSS START command delay executing flag */
	BOOLEAN delay_bss_start;
	/** SSID,BSSID parameter of delay executing */
	mlan_ssid_bssid delay_ssid_bssid;
#ifdef UAP_CFG80211
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 12, 0)
	/* CAC channel info */
	struct cfg80211_chan_def dfs_channel;
	/* time set flag*/
	BOOLEAN is_cac_timer_set;
	/** cac_timer */
	moal_drv_timer cac_timer __ATTRIB_ALIGN__;
	/** cac bss index */
	t_u8 cac_bss_index;
#endif
#endif
#if defined(UAP_SUPPORT)
	/** channel switch wait queue token */
	BOOLEAN chsw_wait_q_woken;
	/** channel switch wait queue */
	wait_queue_head_t chsw_wait_q __ATTRIB_ALIGN__;
#endif
	/** cac period length, valid only when dfs testing is enabled */
	long cac_period_jiffies;
	/** cac restart*/
	t_u8 cac_restart;
	/** handle index - for multiple card supports */
	t_u8 handle_idx;
#if defined(USB)
	/** Flag to indicate boot state */
	t_u8 boot_state;
#endif /* USB_NEW_FW_DNLD */
#ifdef SDIO_MMC_DEBUG
	/** cmd53 write state */
	u8 cmd53w;
	/** cmd53 read state */
	u8 cmd53r;
#endif
#ifdef STA_SUPPORT
	/** Scan pending on blocked flag */
	t_u8 scan_pending_on_block;
	/** Scan Private pointer */
	moal_private *scan_priv;
	/** Async scan semaphore */
	struct semaphore async_sem;
	/** scan channel gap */
	t_u16 scan_chan_gap;
	/** flag to check if specific scan time set by scancfg */
	t_u8 user_scan_cfg;
#ifdef STA_CFG80211
	/** CFG80211 scan request description */
	struct cfg80211_scan_request *scan_request;
	/** fake scan flag */
	u8 fake_scan_complete;
	/** Scan timeout work*/
	struct delayed_work scan_timeout_work;
	/** scan timeout time */
	t_u32 scan_timeout;

#endif
#endif
	/** main state */
	t_u8 main_state;
	/** driver status */
	t_u8 driver_status;
	/** driver state */
	t_u8 driver_state;
	/** ioctl timeout */
	t_u8 ioctl_timeout;
#ifdef DUMP_TO_PROC
	/** Pointer of fw dump buffer */
	t_u8 *drv_dump_buf;
	/** drv dump len */
	t_u32 drv_dump_len;
#endif
	/** FW dump state */
	t_u8 fw_dump;
	/** event fw dump */
	t_u8 event_fw_dump;
	/** Re-association timer set flag */
	BOOLEAN is_fw_dump_timer_set;
	/** Re-association timer */
	moal_drv_timer fw_dump_timer __ATTRIB_ALIGN__;
	/** fw dump buffer total len */
	t_u64 fw_dump_len;
	/** fw dump status for each chip, useful in multichip drive */
	BOOLEAN fw_dump_status;
#ifdef DUMP_TO_PROC
	/** Pointer of fw dump buffer */
	t_u8 *fw_dump_buf;
#endif
	/** FW dump full name */
	t_u8 firmware_dump_file[128];

#ifdef SDIO
	/** cmd52 function */
	t_u8 cmd52_func;
	/** cmd52 register */
	t_u8 cmd52_reg;
	/** cmd52 value */
	t_u8 cmd52_val;
#endif
#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 29)
	/** spinlock to stop_queue/wake_queue*/
	spinlock_t queue_lock;
#endif
	/** Driver spin lock */
	spinlock_t driver_lock;
	/** Driver ioctl spin lock */
	spinlock_t ioctl_lock;
	/** lock for scan_request */
	spinlock_t scan_req_lock;
	/** Interface type: 1 byte,  Card type: 1 byte */
	t_u16 card_type;
	/** card revsion */
	t_u8 card_rev;
	/** card info */
	card_info *card_info;
	/** Card specific driver version */
	t_s8 driver_version[MLAN_MAX_VER_STR_LEN];
	char *fwdump_fname;
#ifdef ANDROID_KERNEL
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 1, 0)
	struct wakeup_source ws;
#else
	struct wake_lock wake_lock;
#endif
#endif
	t_u16 dfs_repeater_mode;
	/* feature_control */
	t_u32 feature_control;
	struct notifier_block woal_notifier;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 1, 0)
#if IS_ENABLED(CONFIG_IPV6)
	struct notifier_block woal_inet6_notifier;
#endif
#endif
	mlan_ds_misc_keep_alive keep_alive[MAX_KEEP_ALIVE_ID];
	mlan_ds_misc_keep_alive_rx keep_alive_rx[MAX_KEEP_ALIVE_RX_ID];
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 14, 0)
	struct net_device *pnapi_dev;
#else
	struct net_device napi_dev;
#endif
	struct napi_struct napi_rx;
	/* bus interface operations */
	moal_if_ops ops;
	/* module parameter data */
	const struct firmware *param_data;
	/* wrapped module parameters */
	moal_mod_para params;
	/* debug info */
	mlan_debug_info debug_info;
	/* block id in module param config file */
	int blk_id;
	/** time when FW is active, time is get from boot time, in Nanosecond */
	t_u64 on_time;
	/** tx time, in usecs */
	t_u64 tx_time;
	/** systime when tx start */
	wifi_timeval tx_time_start;
	/** systime when tx end */
	wifi_timeval tx_time_end;
	/** rx time, in usecs */
	t_u64 rx_time;
	/** scan time, in usecs */
	t_u64 scan_time;
	/** systime when scan cmd response return success */
	wifi_timeval scan_time_start;
	/** systime when scan event has no more event */
	wifi_timeval scan_time_end;
	/** seecond mac flag */
	t_u8 second_mac;
	/** moal handle for another mac */
	void *pref_mac;
	/** RF test mode status */
	t_u8 rf_test_mode;
	/** pointer to rf test mode data struct */
	struct rf_test_mode_data *rf_data;
	/** TP accounting parameters */
	moal_tp_acnt_t tp_acnt;
	BOOLEAN is_tp_acnt_timer_set;

	t_u8 request_pm;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 35)
	struct pm_qos_request woal_pm_qos_req;
#endif
	t_u32 ips_ctrl;
	BOOLEAN is_edmac_enabled;
	bool driver_init;
};

/**
 *  @brief set extended flag in bitmap
 *
 *  @param dev		A pointer to moal_handle structure
 *  @param idx      extended flags id
 *  @return			N/A
 */
static inline void moal_extflg_set(moal_handle *handle, enum ext_mod_params idx)
{
	t_u8 *ext_fbyte;
	ext_fbyte = &handle->params.ext_flgs[idx / 8];
	*ext_fbyte |= MBIT(idx % 8);
}

/**
 *  @brief clear extended flag in bitmap
 *
 *  @param dev		A pointer to moal_handle structure
 *  @param idx      extended flags id
 *  @return			N/A
 */
static inline void moal_extflg_clear(moal_handle *handle,
				     enum ext_mod_params idx)
{
	t_u8 *ext_fbyte;
	ext_fbyte = &handle->params.ext_flgs[idx / 8];
	*ext_fbyte &= ~MBIT(idx % 8);
}

/**
 *  @brief check value of extended flag in bitmap
 *
 *  @param dev		A pointer to moal_handle structure
 *  @param idx      extended flags id
 *  @return			value of extended flag
 */
static inline t_u8 moal_extflg_isset(moal_handle *handle,
				     enum ext_mod_params idx)
{
	t_u8 ext_fbyte;
	ext_fbyte = handle->params.ext_flgs[idx / 8];
	return (ext_fbyte & MBIT(idx % 8)) != 0;
}

/**
 *  @brief set trans_start for each TX queue.
 *
 *  @param dev		A pointer to net_device structure
 *
 *  @return			N/A
 */
static inline void woal_set_trans_start(struct net_device *dev)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 31)
	unsigned int i;
	for (i = 0; i < dev->num_tx_queues; i++)
		netdev_get_tx_queue(dev, i)->trans_start = jiffies;
#endif
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 7, 0)
	dev->trans_start = jiffies;
#else
	netif_trans_update(dev);
#endif
}

/**
 *  @brief Start queue
 *
 *  @param dev		A pointer to net_device structure
 *
 *  @return			N/A
 */
static inline void woal_start_queue(struct net_device *dev)
{
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2, 6, 29)
	netif_start_queue(dev);
#else
	if (dev->reg_state == NETREG_REGISTERED)
		netif_tx_wake_all_queues(dev);
	else
		netif_tx_start_all_queues(dev);
#endif
}

/**
 *  @brief Stop queue
 *
 *  @param dev		A pointer to net_device structure
 *
 *  @return			N/A
 */
static inline void woal_stop_queue(struct net_device *dev)
{
#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 29)
	unsigned long flags;
	moal_private *priv = (moal_private *)netdev_priv(dev);
	spin_lock_irqsave(&priv->phandle->queue_lock, flags);
	woal_set_trans_start(dev);
	if (!netif_queue_stopped(dev))
		netif_tx_stop_all_queues(dev);
	spin_unlock_irqrestore(&priv->phandle->queue_lock, flags);
#else
	woal_set_trans_start(dev);
	if (!netif_queue_stopped(dev))
		netif_stop_queue(dev);
#endif
}

/**
 *  @brief wake queue
 *
 *  @param dev		A pointer to net_device structure
 *
 *  @return			N/A
 */
static inline void woal_wake_queue(struct net_device *dev)
{
#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 29)
	unsigned long flags;
	moal_private *priv = (moal_private *)netdev_priv(dev);
	spin_lock_irqsave(&priv->phandle->queue_lock, flags);
	if (netif_queue_stopped(dev))
		netif_tx_wake_all_queues(dev);
	spin_unlock_irqrestore(&priv->phandle->queue_lock, flags);
#else
	if (netif_queue_stopped(dev))
		netif_wake_queue(dev);
#endif
}

/** Debug Macro definition*/
#ifdef DEBUG_LEVEL1
extern t_u32 drvdbg;

#define LOG_CTRL(level) (0)

#ifdef DEBUG_LEVEL2
#define PRINTM_MINFO(msg...)                                                   \
	do {                                                                   \
		woal_print(MINFO, msg);                                        \
		if (drvdbg & MINFO)                                            \
			printk(KERN_DEBUG msg);                                \
	} while (0)
#define PRINTM_MWARN(msg...)                                                   \
	do {                                                                   \
		woal_print(MWARN, msg);                                        \
		if (drvdbg & MWARN)                                            \
			printk(KERN_DEBUG msg);                                \
	} while (0)
#define PRINTM_MENTRY(msg...)                                                  \
	do {                                                                   \
		woal_print(MENTRY, msg);                                       \
		if (drvdbg & MENTRY)                                           \
			printk(KERN_DEBUG msg);                                \
	} while (0)
#else
#define PRINTM_MINFO(msg...)                                                   \
	do {                                                                   \
	} while (0)
#define PRINTM_MWARN(msg...)                                                   \
	do {                                                                   \
	} while (0)
#define PRINTM_MENTRY(msg...)                                                  \
	do {                                                                   \
	} while (0)
#endif /* DEBUG_LEVEL2 */

#ifdef FWDUMP_VIA_PRINT
#define PRINTM_MFWDP_D(msg...)                                                 \
	do {                                                                   \
		woal_print(MFWDP_D, msg);                                      \
		if (drvdbg & MFWDP_D)                                          \
			printk(KERN_DEBUG msg);                                \
	} while (0)
#endif /*FWDUMP_VIA_PRINT*/
#define PRINTM_MFW_D(msg...)                                                   \
	do {                                                                   \
		woal_print(MFW_D, msg);                                        \
		if (drvdbg & MFW_D)                                            \
			printk(KERN_DEBUG msg);                                \
	} while (0)
#define PRINTM_MCMD_D(msg...)                                                  \
	do {                                                                   \
		woal_print(MCMD_D, msg);                                       \
		if (drvdbg & MCMD_D)                                           \
			printk(KERN_DEBUG msg);                                \
	} while (0)
#define PRINTM_MDAT_D(msg...)                                                  \
	do {                                                                   \
		woal_print(MDAT_D, msg);                                       \
		if (drvdbg & MDAT_D)                                           \
			printk(KERN_DEBUG msg);                                \
	} while (0)
#define PRINTM_MIF_D(msg...)                                                   \
	do {                                                                   \
		woal_print(MIF_D, msg);                                        \
		if (drvdbg & MIF_D)                                            \
			printk(KERN_DEBUG msg);                                \
	} while (0)

#define PRINTM_MREG(msg...)                                                    \
	do {                                                                   \
		woal_print(MREG, msg);                                         \
		if (drvdbg & MREG)                                             \
			printk(KERN_DEBUG msg);                                \
	} while (0)
#define PRINTM_MIOCTL(msg...)                                                  \
	do {                                                                   \
		woal_print(MIOCTL, msg);                                       \
		if (drvdbg & MIOCTL)                                           \
			printk(KERN_DEBUG msg);                                \
	} while (0)
#define PRINTM_MINTR(msg...)                                                   \
	do {                                                                   \
		woal_print(MINTR, msg);                                        \
		if (drvdbg & MINTR)                                            \
			printk(KERN_DEBUG msg);                                \
	} while (0)
#define PRINTM_MEVENT(msg...)                                                  \
	do {                                                                   \
		woal_print(MEVENT, msg);                                       \
		if (drvdbg & MEVENT)                                           \
			printk(msg);                                           \
	} while (0)
#define PRINTM_MCMND(msg...)                                                   \
	do {                                                                   \
		woal_print(MCMND, msg);                                        \
		if (drvdbg & MCMND)                                            \
			printk(KERN_DEBUG msg);                                \
	} while (0)
#define PRINTM_MDATA(msg...)                                                   \
	do {                                                                   \
		woal_print(MDATA, msg);                                        \
		if (drvdbg & MDATA)                                            \
			printk(KERN_DEBUG msg);                                \
	} while (0)
#define PRINTM_MERROR(msg...)                                                  \
	do {                                                                   \
		woal_print(MERROR, msg);                                       \
		if (drvdbg & MERROR)                                           \
			printk(KERN_ERR msg);                                  \
	} while (0)
#define PRINTM_MFATAL(msg...)                                                  \
	do {                                                                   \
		woal_print(MFATAL, msg);                                       \
		if (drvdbg & MFATAL)                                           \
			printk(KERN_ERR msg);                                  \
	} while (0)
#define PRINTM_MMSG(msg...)                                                    \
	do {                                                                   \
		woal_print(MMSG, msg);                                         \
		if (drvdbg & MMSG)                                             \
			printk(KERN_ALERT msg);                                \
	} while (0)

static inline void woal_print(t_u32 level, char *fmt, ...)
{
}

#define PRINTM(level, msg...) PRINTM_##level(msg)

#else

#define PRINTM(level, msg...)                                                  \
	do {                                                                   \
	} while (0)

#endif /* DEBUG_LEVEL1 */

/** Wait until a condition becomes true */
#define MASSERT(cond)                                                          \
	do {                                                                   \
		if (!(cond)) {                                                 \
			PRINTM(MFATAL, "ASSERT: %s: %i\n", __func__,           \
			       __LINE__);                                      \
			panic("Assert failed: Panic!");                        \
		}                                                              \
	} while (0)

/** Log entry point for debugging */
#define ENTER() PRINTM(MENTRY, "Enter: %s\n", __func__)
/** Log exit point for debugging */
#define LEAVE() PRINTM(MENTRY, "Leave: %s\n", __func__)

#ifdef DEBUG_LEVEL1
#define DBG_DUMP_BUF_LEN 64
#define MAX_DUMP_PER_LINE 16

static inline void hexdump(t_u32 level, char *prompt, const t_u8 *buf, int len)
{
	int i;
	char dbgdumpbuf[DBG_DUMP_BUF_LEN];
	char *ptr = dbgdumpbuf;

	if (drvdbg & level)
		printk(KERN_DEBUG "%s:\n", prompt);
	for (i = 1; i <= len; i++) {
		ptr += snprintf(ptr, 4, "%02x ", *buf);
		buf++;
		if (i % MAX_DUMP_PER_LINE == 0) {
			*ptr = 0;
			if (drvdbg & level)
				printk(KERN_DEBUG "%s\n", dbgdumpbuf);
			ptr = dbgdumpbuf;
		}
	}
	if (len % MAX_DUMP_PER_LINE) {
		*ptr = 0;
		if (drvdbg & level)
			printk(KERN_DEBUG "%s\n", dbgdumpbuf);
	}
}

#define DBG_HEXDUMP_MERROR(x, y, z)                                            \
	do {                                                                   \
		if ((drvdbg & MERROR) || LOG_CTRL(MERROR))                     \
			hexdump(MERROR, x, y, z);                              \
	} while (0)
#define DBG_HEXDUMP_MCMD_D(x, y, z)                                            \
	do {                                                                   \
		if ((drvdbg & MCMD_D) || LOG_CTRL(MCMD_D))                     \
			hexdump(MCMD_D, x, y, z);                              \
	} while (0)
#define DBG_HEXDUMP_MDAT_D(x, y, z)                                            \
	do {                                                                   \
		if ((drvdbg & MDAT_D) || LOG_CTRL(MDAT_D))                     \
			hexdump(MDAT_D, x, y, z);                              \
	} while (0)
#define DBG_HEXDUMP_MIF_D(x, y, z)                                             \
	do {                                                                   \
		if ((drvdbg & MIF_D) || LOG_CTRL(MIF_D))                       \
			hexdump(MIF_D, x, y, z);                               \
	} while (0)
#define DBG_HEXDUMP_MEVT_D(x, y, z)                                            \
	do {                                                                   \
		if ((drvdbg & MEVT_D) || LOG_CTRL(MEVT_D))                     \
			hexdump(MEVT_D, x, y, z);                              \
	} while (0)
#define DBG_HEXDUMP_MFW_D(x, y, z)                                             \
	do {                                                                   \
		if ((drvdbg & MFW_D) || LOG_CTRL(MFW_D))                       \
			hexdump(MFW_D, x, y, z);                               \
	} while (0)
#define DBG_HEXDUMP(level, x, y, z) DBG_HEXDUMP_##level(x, y, z)

#else
/** Do nothing since debugging is not turned on */
#define DBG_HEXDUMP(level, x, y, z)                                            \
	do {                                                                   \
	} while (0)
#endif

#ifdef DEBUG_LEVEL2
#define HEXDUMP(x, y, z)                                                       \
	do {                                                                   \
		if ((drvdbg & MINFO) || LOG_CTRL(MINFO))                       \
			hexdump(MINFO, x, y, z);                               \
	} while (0)
#else
/** Do nothing since debugging is not turned on */
#define HEXDUMP(x, y, z)                                                       \
	do {                                                                   \
	} while (0)
#endif

#ifdef BIG_ENDIAN_SUPPORT
/** Convert from 16 bit little endian format to CPU format */
#define woal_le16_to_cpu(x) le16_to_cpu(x)
/** Convert from 32 bit little endian format to CPU format */
#define woal_le32_to_cpu(x) le32_to_cpu(x)
/** Convert from 64 bit little endian format to CPU format */
#define woal_le64_to_cpu(x) le64_to_cpu(x)
/** Convert to 16 bit little endian format from CPU format */
#define woal_cpu_to_le16(x) cpu_to_le16(x)
/** Convert to 32 bit little endian format from CPU format */
#define woal_cpu_to_le32(x) cpu_to_le32(x)
/** Convert to 64 bit little endian format from CPU format */
#define woal_cpu_to_le64(x) cpu_to_le64(x)
#else
/** Do nothing */
#define woal_le16_to_cpu(x) x
/** Do nothing */
#define woal_le32_to_cpu(x) x
/** Do nothing */
#define woal_le64_to_cpu(x) x
/** Do nothing */
#define woal_cpu_to_le16(x) x
/** Do nothing */
#define woal_cpu_to_le32(x) x
/** Do nothing */
#define woal_cpu_to_le64(x) x
#endif

/**
 *  @brief This function returns first available priv
 *  based on the BSS role
 *
 *  @param handle    A pointer to moal_handle
 *  @param bss_role  BSS role or MLAN_BSS_ROLE_ANY
 *
 *  @return          Pointer to moal_private
 */
static inline moal_private *woal_get_priv(moal_handle *handle,
					  mlan_bss_role bss_role)
{
	int i;

	for (i = 0; i < MIN(handle->priv_num, MLAN_MAX_BSS_NUM); i++) {
		if (handle->priv[i]) {
			if (bss_role == MLAN_BSS_ROLE_ANY ||
			    GET_BSS_ROLE(handle->priv[i]) == bss_role)
				return handle->priv[i];
		}
	}
	return NULL;
}

/**
 *  @brief This function returns first available priv
 *  based on the BSS type
 *
 *  @param handle    A pointer to moal_handle
 *  @param bss_type  BSS type or MLAN_BSS_TYPE_ANY
 *
 *  @return          Pointer to moal_private
 */
static inline moal_private *woal_get_priv_bss_type(moal_handle *handle,
						   mlan_bss_type bss_type)
{
	int i;

	for (i = 0; i < MIN(handle->priv_num, MLAN_MAX_BSS_NUM); i++) {
		if (handle->priv[i]) {
			if (bss_type == MLAN_BSS_TYPE_ANY ||
			    handle->priv[i]->bss_type == bss_type)
				return handle->priv[i];
		}
	}
	return NULL;
}

static inline moal_private *woal_get_vir_priv_bss_type(moal_handle *handle,
						       mlan_bss_type bss_type)
{
	int i;

	for (i = 0; i < MIN(handle->priv_num, MLAN_MAX_BSS_NUM); i++) {
		if (handle->priv[i]) {
			if (handle->priv[i]->bss_type == bss_type &&
			    handle->priv[i]->bss_virtual)
				return handle->priv[i];
		}
	}
	return NULL;
}

#if defined(STA_CFG80211) || defined(UAP_CFG80211)
/** get any cfg80211 priv */
static inline moal_private *woal_get_priv_with_wdev(moal_handle *handle)
{
	int i;
	for (i = 0; i < MIN(handle->priv_num, MLAN_MAX_BSS_NUM); i++) {
		if (handle->priv[i]) {
			if (handle->priv[i]->wdev)
				return handle->priv[i];
		}
	}
	return NULL;
}
#endif

static inline void woal_get_monotonic_time(wifi_timeval *tv)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0)
	struct timespec64 ts;
#else
	struct timespec ts;
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0)
	ktime_get_raw_ts64(&ts);
#else
	getrawmonotonic(&ts);
#endif
	if (tv) {
		tv->time_sec = (t_u32)ts.tv_sec;
		tv->time_usec = (t_u32)ts.tv_nsec / 1000;
	}
}

/* CAC Measure report default time 60 seconds */
#define MEAS_REPORT_TIME (60 * HZ)

/** Max line length allowed in init config file */
#define MAX_LINE_LEN 256
/** Max MAC address string length allowed */
#define MAX_MAC_ADDR_LEN 18
/** Max register type/offset/value etc. parameter length allowed */
#define MAX_PARAM_LEN 12

/** HostCmd_CMD_DBGS_CFG for Debug cmd configuration */
#define HostCmd_CMD_DBGS_CFG 0x008b

/** Debug subcommand for enable hehtc */
#define DEBUG_SUBID_ENABLE_HEHTC 0x124

/** HostCmd_CMD_CFG_DATA for CAL data */
#define HostCmd_CMD_CFG_DATA 0x008f

/** HostCmd action set */
#define HostCmd_ACT_GEN_SET 0x0001
/** HostCmd CAL data header length */
#define CFG_DATA_HEADER_LEN 6

typedef struct _HostCmd_DS_GEN {
	t_u16 command;
	t_u16 size;
	t_u16 seq_num;
	t_u16 result;
} HostCmd_DS_GEN;

typedef struct _HostCmd_DS_802_11_CFG_DATA {
	/** Action */
	t_u16 action;
	/** Type */
	t_u16 type;
	/** Data length */
	t_u16 data_len;
	/** Data */
	t_u8 data[1];
} __ATTRIB_PACK__ HostCmd_DS_802_11_CFG_DATA;

typedef struct _HostCmd_DS_802_11_DBGS_CFG {
	/** hostcmd header */
	HostCmd_DS_GEN header;
	/** sub cmd action */
	t_u16 action;
	/** subid */
	t_u16 subid;
	/** Data */
	t_u8 data[];
} __ATTRIB_PACK__ HostCmd_DS_802_11_DBGS_CFG;

/** combo scan header */
#define WEXT_CSCAN_HEADER "CSCAN S\x01\x00\x00S\x00"
/** combo scan header size */
#define WEXT_CSCAN_HEADER_SIZE 12
/** combo scan ssid section */
#define WEXT_CSCAN_SSID_SECTION 'S'
/** commbo scan channel section */
#define WEXT_CSCAN_CHANNEL_SECTION 'C'
/** commbo scan passive dwell section */
#define WEXT_CSCAN_PASV_DWELL_SECTION 'P'
/** commbo scan home dwell section */
#define WEXT_CSCAN_HOME_DWELL_SECTION 'H'
/** BGSCAN RSSI section */
#define WEXT_BGSCAN_RSSI_SECTION 'R'
/** BGSCAN SCAN INTERVAL SECTION */
#define WEXT_BGSCAN_INTERVAL_SECTION 'T'
/** BGSCAN REPEAT SECTION */
#define WEXT_BGSCAN_REPEAT_SECTION 'E'
/** Min BGSCAN interval 30 second */
#define MIN_BGSCAN_INTERVAL 30000
/** default repeat count */
#define DEF_REPEAT_COUNT 6

/** default rssi low threshold */
#define DEFAULT_RSSI_LOW_THRESHOLD 70
/** RSSI HYSTERSIS */
#define RSSI_HYSTERESIS 6
/** lowest rssi threshold */
#define LOWEST_RSSI_THRESHOLD 82
/** delta rssi */
#define DELTA_RSSI 10

/** NL80211 scan configuration header */
#define NL80211_SCANCFG_HEADER "SCAN-CFG "
/** NL80211 scan configuration header length */
#define NL80211_SCANCFG_HEADER_SIZE 9
/** NL80211 scan configuration active scan section */
#define NL80211_SCANCFG_ACTV_DWELL_SECTION 'A'
/** NL80211 scan configuration passive scan section */
#define NL80211_SCANCFG_PASV_DWELL_SECTION 'P'
/** NL80211 scan configuration specific scan section */
#define NL80211_SCANCFG_SPCF_DWELL_SECTION 'S'

/** band AUTO */
#define WIFI_FREQUENCY_BAND_AUTO 0
/** band 5G */
#define WIFI_FREQUENCY_BAND_5GHZ 1
/** band 2G */
#define WIFI_FREQUENCY_BAND_2GHZ 2
/** All band */
#define WIFI_FREQUENCY_ALL_BAND 3

/** Rx filter: IPV4 multicast */
#define RX_FILTER_IPV4_MULTICAST 1
/** Rx filter: broadcast */
#define RX_FILTER_BROADCAST 2
/** Rx filter: unicast */
#define RX_FILTER_UNICAST 4
/** Rx filter: IPV6 multicast */
#define RX_FILTER_IPV6_MULTICAST 8

/**  Convert ASCII string to hex value */
int woal_ascii2hex(t_u8 *d, char *s, t_u32 dlen);
/** parse ie */
const t_u8 *woal_parse_ie_tlv(const t_u8 *ie, int len, t_u8 id);
/** parse extension ie */
const t_u8 *woal_parse_ext_ie_tlv(const t_u8 *ie, int len, t_u8 ext_id);
/**  Convert mac address from string to t_u8 buffer */
void woal_mac2u8(t_u8 *mac_addr, char *buf);
/**  Extract token from string */
char *woal_strsep(char **s, char delim, char esc);
/** Return int value of a given ASCII string */
mlan_status woal_atoi(int *data, char *a);
/** Return hex value of a given ASCII string */
int woal_atox(char *a);
/** Allocate buffer */
pmlan_buffer woal_alloc_mlan_buffer(moal_handle *handle, int size);
/** Allocate IOCTL request buffer */
pmlan_ioctl_req woal_alloc_mlan_ioctl_req(int size);
/** Free buffer */
void woal_free_mlan_buffer(moal_handle *handle, pmlan_buffer pmbuf);
/** Get private structure of a BSS by index */
moal_private *woal_bss_index_to_priv(moal_handle *handle, t_u32 bss_index);
/* Functions in init module */
/** init module parameters */
mlan_status woal_init_module_param(moal_handle *handle);
/** free module parameters */
void woal_free_module_param(moal_handle *handle);
/** init module parameters from device tree */
void woal_init_from_dev_tree(void);
/** initializes software */
mlan_status woal_init_sw(moal_handle *handle);
/** update the default firmware name */
void woal_update_firmware_name(moal_handle *handle);
mlan_status woal_set_rgpower_table(moal_handle *handle);
/** cancel all works in the queue */
void woal_terminate_workqueue(moal_handle *handle);
void woal_flush_workqueue(moal_handle *handle);
void woal_queue_rx_task(moal_handle *handle);
void woal_flush_evt_queue(moal_handle *handle);
/** initializes firmware */
mlan_status woal_init_fw(moal_handle *handle);
/** frees the structure of moal_handle */
void woal_free_moal_handle(moal_handle *handle);
/** shutdown fw */
mlan_status woal_shutdown_fw(moal_private *priv, t_u8 wait_option);
/* Functions in interface module */
#ifdef ANDROID_KERNEL
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0)
static inline void wakeup_source_init(struct wakeup_source *ws,
				      const char *name)
{
	ENTER();

	if (ws) {
		memset(ws, 0, sizeof(*ws));
		ws->name = name;
	}
	wakeup_source_add(ws);

	LEAVE();
}

static inline void wakeup_source_trash(struct wakeup_source *ws)
{
	ENTER();

	if (!ws) {
		PRINTM(MERROR, "ws is null!\n");
		return;
	}
	wakeup_source_remove(ws);
	__pm_relax(ws);

	LEAVE();
}
#endif
#endif
/** Add card */
moal_handle *woal_add_card(void *card, struct device *dev, moal_if_ops *if_ops,
			   t_u16 card_type);
/** Remove card */
mlan_status woal_remove_card(void *card);
/** broadcast event */
mlan_status woal_broadcast_event(moal_private *priv, t_u8 *payload, t_u32 len);
#ifdef CONFIG_PROC_FS
/** switch driver mode */
mlan_status woal_switch_drv_mode(moal_handle *handle, t_u32 mode);
#endif

int woal_check_media_connected(t_void *pmoal);
/** check if any interface is up */
t_u8 woal_is_any_interface_active(moal_handle *handle);
/** Get version */
void woal_get_version(moal_handle *handle, char *version, int maxlen);
/** Get Driver Version */
int woal_get_driver_version(moal_private *priv, struct ifreq *req);
/** Get extended driver version */
int woal_get_driver_verext(moal_private *priv, struct ifreq *ireq);
/** check driver status */
t_u8 woal_check_driver_status(moal_handle *handle);
/** Mgmt frame forward registration */
int woal_reg_rx_mgmt_ind(moal_private *priv, t_u16 action,
			 t_u32 *pmgmt_subtype_mask, t_u8 wait_option);
#ifdef DEBUG_LEVEL1
/** Set driver debug bit masks */
int woal_set_drvdbg(moal_private *priv, t_u32 drv_dbg);
#endif

mlan_status woal_set_get_tx_bf_cap(moal_private *priv, t_u16 action,
				   t_u32 *tx_bf_cap);
/** Set/Get TX beamforming configurations */
mlan_status woal_set_get_tx_bf_cfg(moal_private *priv, t_u16 action,
				   mlan_ds_11n_tx_bf_cfg *bf_cfg);
/** Request MAC address setting */
mlan_status woal_request_set_mac_address(moal_private *priv, t_u8 wait_option);
/** Request multicast list setting */
void woal_request_set_multicast_list(moal_private *priv,
				     struct net_device *dev);
/** Request IOCTL action */
mlan_status woal_request_ioctl(moal_private *priv, mlan_ioctl_req *req,
			       t_u8 wait_option);
/** Set/Get generic element */
mlan_status woal_set_get_gen_ie(moal_private *priv, t_u32 action,
				const t_u8 *ie, t_u8 *get_ie, int *ie_len,
				t_u8 wait_option);
#ifdef CONFIG_PROC_FS
mlan_status woal_request_soft_reset(moal_handle *handle);
#endif
int woal_request_fw_reload(moal_handle *phandle, t_u8 mode);

mlan_status woal_set_psk_11ai(moal_private *priv, t_u8 wait_option,
			      const t_u8 *addr, const t_u8 *key, int key_len);

/** Get debug information */
mlan_status woal_get_debug_info(moal_private *priv, t_u8 wait_option,
				mlan_debug_info *debug_info);
/** Set debug information */
mlan_status woal_set_debug_info(moal_private *priv, t_u8 wait_option,
				mlan_debug_info *debug_info);
/** Disconnect */
mlan_status woal_disconnect(moal_private *priv, t_u8 wait_option, t_u8 *mac,
			    t_u16 reason_code);
/** associate */
mlan_status woal_bss_start(moal_private *priv, t_u8 wait_option,
			   mlan_ssid_bssid *ssid_bssid);
/** Request firmware information */
mlan_status woal_request_get_fw_info(moal_private *priv, t_u8 wait_option,
				     mlan_fw_info *fw_info);
/** Get channel of active intf */
mlan_status woal_get_active_intf_channel(moal_private *priv,
					 chan_band_info *channel);
#ifdef STA_SUPPORT
/** Request Exented Capability information */
int woal_request_extcap(moal_private *priv, t_u8 *buf, t_u8 len);
#endif
mlan_status woal_set_get_dtim_period(moal_private *priv, t_u32 action,
				     t_u8 wait_option, t_u8 *value);
/** Set/get Host Sleep parameters */
mlan_status woal_set_get_hs_params(moal_private *priv, t_u16 action,
				   t_u8 wait_option, mlan_ds_hs_cfg *hscfg);
/** Cancel Host Sleep configuration */
mlan_status woal_cancel_hs(moal_private *priv, t_u8 wait_option);
/** Enable Host Sleep configuration */
int woal_enable_hs(moal_private *priv);
/** hs active timeout 2 second */
#define HS_ACTIVE_TIMEOUT (2 * HZ)
/** Get wakeup reason */
mlan_status woal_get_wakeup_reason(moal_private *priv,
				   mlan_ds_hs_wakeup_reason *wakeup_reason);
int woal_process_proc_hssetpara(moal_handle *handle, t_u8 *buf);
#ifdef DUMP_TO_PROC
#define FW_DUMP_INFO_LEN 0x280000
/** mem dump header */
typedef struct {
	/** seq number */
	t_u16 seq_num;
	/** resvered */
	t_u16 reserved;
	/** type */
	t_u16 type;
	/** len */
	t_u16 len;
	/** start addr */
	t_u32 start_addr;
} mem_dump_header;
int woal_save_dump_info_to_buf(moal_handle *phandle, t_u8 *src, t_u32 len,
			       t_u32 type);
void woal_append_end_block(moal_handle *phandle);
t_u8 *woal_dump_drv_info(moal_handle *phandle, t_u32 *dump_len);
#else
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 1, 0)
void woal_create_dump_dir(moal_handle *phandle, char *dir_buf, int buf_size);
#endif
mlan_status woal_save_dump_info_to_file(char *dir_name, char *file_name,
					t_u8 *buf, t_u32 buf_len);
void woal_dump_drv_info(moal_handle *phandle, t_u8 *dir_name);
#endif

#define FW_DUMP_TYPE_ENDED 0x002
#define FW_DUMP_TYPE_MEM_ITCM 0x004
#define FW_DUMP_TYPE_MEM_DTCM 0x005
#define FW_DUMP_TYPE_MEM_SQRAM 0x006
#define FW_DUMP_TYPE_MEM_IRAM 0x007
#define FW_DUMP_TYPE_REG_MAC 0x009
#define FW_DUMP_TYPE_REG_CIU 0x00E
#define FW_DUMP_TYPE_REG_APU 0x00F
#define FW_DUMP_TYPE_REG_ICU 0x014
#ifdef SDIO_MMC
void woal_dump_firmware_info(moal_handle *phandle);
void woal_dump_firmware_info_v2(moal_handle *phandle);
void woal_dump_firmware_info_v3(moal_handle *phandle);
#endif /* SDIO_MMC */
#ifdef FWDUMP_VIA_PRINT
/* Print FW dumps in kernel(dmesg) log */
t_void woal_print_firmware_dump(moal_handle *phandle, char *fwdp_fname);
#endif /*FWDUMP_VIA_PRINT*/
/* Store the FW dumps received from events in a file */
void woal_store_firmware_dump(moal_handle *phandle, pmlan_event pmevent);
void woal_send_fw_dump_complete_event(moal_private *priv);

#ifndef DUMP_TO_PROC
#if defined(PCIE)
void woal_store_ssu_dump(moal_handle *phandle, pmlan_event pmevent);
#endif /* SSU_SUPPORT */
/** save hostcmd response to file */
t_void woal_save_host_cmdresp(moal_handle *phandle, mlan_cmdresp_event *pevent);
#endif

int woal_pre_warmreset(moal_private *priv);
int woal_warmreset(moal_private *priv);

/** get deep sleep */
int woal_get_deep_sleep(moal_private *priv, t_u32 *data);
/** set deep sleep */
int woal_set_deep_sleep(moal_private *priv, t_u8 wait_option,
			BOOLEAN bdeep_sleep, t_u16 idletime);
/** process hang */
void woal_process_hang(moal_handle *handle);
/** Get BSS information */
mlan_status woal_get_bss_info(moal_private *priv, t_u8 wait_option,
			      mlan_bss_info *bss_info);
void woal_process_ioctl_resp(moal_private *priv, mlan_ioctl_req *req);
char *region_code_2_string(t_u8 region_code);
t_bool woal_is_etsi_country(t_u8 *country_code);
t_u8 woal_is_valid_alpha2(char *alpha2);
#ifdef STA_SUPPORT
void woal_send_disconnect_to_system(moal_private *priv, t_u16 reason_code);
void woal_send_mic_error_event(moal_private *priv, t_u32 event);
void woal_ioctl_get_bss_resp(moal_private *priv, mlan_ds_bss *bss);
void woal_ioctl_get_info_resp(moal_private *priv, mlan_ds_get_info *info);
mlan_status woal_get_assoc_rsp(moal_private *priv,
			       mlan_ds_misc_assoc_rsp *assoc_rsp,
			       t_u8 wait_option);
mlan_status woal_get_assoc_req(moal_private *priv,
			       mlan_ds_misc_assoc_req *assoc_req,
			       t_u8 wait_option);
mlan_status woal_get_prev_assoc_info(moal_private *priv,
				     mlan_ds_assoc_info *assoc_info,
				     t_u8 wait_option);
/** Get signal information */
mlan_status woal_get_signal_info(moal_private *priv, t_u8 wait_option,
				 mlan_ds_get_signal *signal);
/** Get mode */
t_u32 woal_get_mode(moal_private *priv, t_u8 wait_option);
mlan_status woal_get_sta_channel(moal_private *priv, t_u8 wait_option,
				 chan_band_info *channel);
#ifdef STA_WEXT
/** Get data rates */
mlan_status woal_get_data_rates(moal_private *priv, t_u8 wait_option,
				pmoal_802_11_rates m_rates);
void woal_send_iwevcustom_event(moal_private *priv, char *str);
/** Get channel list */
mlan_status woal_get_channel_list(moal_private *priv, t_u8 wait_option,
				  mlan_chan_list *chanlist);
#endif
mlan_status woal_11d_check_ap_channel(moal_private *priv, t_u8 wait_option,
				      mlan_ssid_bssid *ssid_bssid);
/** Set/Get retry count */
mlan_status woal_set_get_retry(moal_private *priv, t_u32 action,
			       t_u8 wait_option, int *value);
/** Set/Get RTS threshold */
mlan_status woal_set_get_rts(moal_private *priv, t_u32 action, t_u8 wait_option,
			     int *value);
/** Set/Get fragment threshold */
mlan_status woal_set_get_frag(moal_private *priv, t_u32 action,
			      t_u8 wait_option, int *value);
/** Set/Get TX power */
mlan_status woal_set_get_tx_power(moal_private *priv, t_u32 action,
				  mlan_power_cfg_t *pwr);
/** Set/Get power IEEE management */
mlan_status woal_set_get_power_mgmt(moal_private *priv, t_u32 action,
				    int *disabled, int type, t_u8 wait_option);
/** Get data rate */
mlan_status woal_set_get_data_rate(moal_private *priv, t_u8 action,
				   mlan_rate_cfg_t *datarate);
/** Request a network scan */
mlan_status woal_request_scan(moal_private *priv, t_u8 wait_option,
			      mlan_802_11_ssid *req_ssid);
/** Set radio on/off */
int woal_set_radio(moal_private *priv, t_u8 option);
/** Set region code */
mlan_status woal_set_region_code(moal_private *priv, char *region);
/** Set authentication mode */
mlan_status woal_set_auth_mode(moal_private *priv, t_u8 wait_option,
			       t_u32 auth_mode);
/** Set encryption mode */
mlan_status woal_set_encrypt_mode(moal_private *priv, t_u8 wait_option,
				  t_u32 encrypt_mode);
/** Enable wep key */
mlan_status woal_enable_wep_key(moal_private *priv, t_u8 wait_option);
/** Set WPA enable */
mlan_status woal_set_wpa_enable(moal_private *priv, t_u8 wait_option,
				t_u32 enable);
/** cancel scan command */
mlan_status woal_cancel_scan(moal_private *priv, t_u8 wait_option);
/** Find best network to connect */
mlan_status woal_find_best_network(moal_private *priv, t_u8 wait_option,
				   mlan_ssid_bssid *ssid_bssid);

/** Get scan table */
mlan_status woal_get_scan_table(moal_private *priv, t_u8 wait_option,
				mlan_scan_resp *scanresp);
/** Get authentication mode */
mlan_status woal_get_auth_mode(moal_private *priv, t_u8 wait_option,
			       t_u32 *auth_mode);
/** Get encryption mode */
mlan_status woal_get_encrypt_mode(moal_private *priv, t_u8 wait_option,
				  t_u32 *encrypt_mode);
/** Get WPA state */
mlan_status woal_get_wpa_enable(moal_private *priv, t_u8 wait_option,
				t_u32 *enable);
#endif /**STA_SUPPORT */

mlan_status woal_set_11d(moal_private *priv, t_u8 wait_option, t_u8 enable);

mlan_status woal_process_rf_test_mode(moal_handle *handle, t_u32 mode);
mlan_status woal_process_rf_test_mode_cmd(moal_handle *handle, t_u32 cmd,
					  const char *buffer, size_t len,
					  t_u32 action, t_u32 val);
#if defined(STA_SUPPORT) || defined(UAP_SUPPORT)
/** Get statistics information */
mlan_status woal_get_stats_info(moal_private *priv, t_u8 wait_option,
				pmlan_ds_get_stats stats);
#endif /**STA_SUPPORT||UAP_SUPPORT*/

mlan_status woal_set_wapi_enable(moal_private *priv, t_u8 wait_option,
				 t_u32 enable);

/** Initialize priv */
void woal_init_priv(moal_private *priv, t_u8 wait_option);
/** Reset interface(s) */
mlan_status woal_reset_intf(moal_private *priv, t_u8 wait_option, int all_intf);
#define TLV_TYPE_MGMT_IE (0x169)
#define MGMT_MASK_ASSOC_REQ 0x01
#define MGMT_MASK_REASSOC_REQ 0x04
#define MGMT_MASK_ASSOC_RESP 0x02
#define MGMT_MASK_REASSOC_RESP 0x08
#define MGMT_MASK_PROBE_REQ 0x10
#define MGMT_MASK_PROBE_RESP 0x20
#define MGMT_MASK_BEACON 0x100
#define MGMT_MASK_ASSOC_RESP_QOS_MAP 0x4000
#define MGMT_MASK_BEACON_WPS_P2P 0x8000
#define MLAN_CUSTOM_IE_DELETE_MASK 0x0
#define MLAN_CUSTOM_IE_NEW_MASK 0x8000
#define MRVL_PKT_TYPE_MGMT_FRAME 0xE5
/** common ioctl for uap, station */
int woal_custom_ie_ioctl(struct net_device *dev, struct ifreq *req);
#ifdef UAP_SUPPORT
int woal_priv_get_nonglobal_operclass_by_bw_channel(moal_private *priv,
						    t_u8 bandwidth,
						    t_u8 channel,
						    t_u8 *oper_class);
#endif
int woal_send_host_packet(struct net_device *dev, struct ifreq *req);
int woal_send_mon_if_packet(struct net_device *dev, struct ifreq *req);
/** Private command ID to pass mgmt frame */
#define WOAL_MGMT_FRAME_TX_IOCTL (SIOCDEVPRIVATE + 12)
/** common ioctl for TDLS */
int woal_tdls_config_ioctl(struct net_device *dev, struct ifreq *req);

int woal_get_bss_type(struct net_device *dev, struct ifreq *req);
#if defined(STA_WEXT) || defined(UAP_WEXT)
int woal_host_command(moal_private *priv, struct iwreq *wrq);
#endif
#if defined(STA_SUPPORT) && defined(UAP_SUPPORT)
mlan_status woal_bss_role_cfg(moal_private *priv, t_u8 action, t_u8 wait_option,
			      t_u8 *bss_role);
#if defined(STA_CFG80211) && defined(UAP_CFG80211)
void woal_go_timer_func(void *context);
#endif
#if defined(STA_WEXT) || defined(UAP_WEXT)
int woal_set_get_bss_role(moal_private *priv, struct iwreq *wrq);
#endif
#endif
#if defined(WIFI_DIRECT_SUPPORT) || defined(UAP_SUPPORT)
/** hostcmd ioctl for uap, wifidirect */
int woal_hostcmd_ioctl(struct net_device *dev, struct ifreq *req);
#endif

mlan_status woal_set_remain_channel_ioctl(moal_private *priv, t_u8 wait_option,
					  pmlan_ds_remain_chan pchan);
void woal_remain_timer_func(void *context);
#ifdef WIFI_DIRECT_SUPPORT
mlan_status woal_wifi_direct_mode_cfg(moal_private *priv, t_u16 action,
				      t_u16 *mode);
mlan_status woal_p2p_config(moal_private *priv, t_u32 action,
			    mlan_ds_wifi_direct_config *p2p_config);
#endif /* WIFI_DIRECT_SUPPORT */

int woal_11h_cancel_chan_report_ioctl(moal_private *priv, t_u8 wait_option);

#ifdef CONFIG_PROC_FS
/** Initialize /proc/mwlan */
mlan_status woal_root_proc_init(void);
/** Remove /proc/mwlan */
void woal_root_proc_remove(void);
/** Initialize proc fs */
void woal_proc_init(moal_handle *handle);
/** Clean up proc fs */
void woal_proc_exit(moal_handle *handle);
/** Create proc entry */
void woal_create_proc_entry(moal_private *priv);
/** Remove proc entry */
void woal_proc_remove(moal_private *priv);
/** string to number */
int woal_string_to_number(char *s);
#endif

/** Create debug proc fs */
void woal_debug_entry(moal_private *priv);
/** Remove debug proc fs */
void woal_debug_remove(moal_private *priv);

/** check pm info */
mlan_status woal_get_pm_info(moal_private *priv, mlan_ds_ps_info *pm_info);
/** get mlan debug info */
void woal_mlan_debug_info(moal_private *priv);

#ifdef USB
#ifdef CONFIG_USB_SUSPEND
/** Enter USB Suspend */
int woal_enter_usb_suspend(moal_handle *handle);
/** Exit from USB Suspend */
int woal_exit_usb_suspend(moal_handle *handle);
#endif /* CONFIG_USB_SUSPEND */
#endif

#ifdef REASSOCIATION
int woal_reassociation_thread(void *data);
void woal_reassoc_timer_func(void *context);
#endif /* REASSOCIATION */

void woal_fw_dump_timer_func(void *context);

#if defined(USB) || defined(SDIO)
t_void woal_rx_work_queue(struct work_struct *work);
#endif
t_void woal_main_work_queue(struct work_struct *work);
t_void woal_evt_work_queue(struct work_struct *work);
t_void woal_mclist_work_queue(struct work_struct *work);

#ifdef PCIE
t_void woal_pcie_cmd_resp_work_queue(struct work_struct *work);
t_void woal_pcie_delayed_tx_work(struct work_struct *work);
#ifndef TASKLET_SUPPORT
t_void woal_pcie_rx_work_queue(struct work_struct *work);
t_void woal_pcie_tx_complete_work_queue(struct work_struct *work);
#endif
#endif

#ifdef STA_CFG80211
t_void woal_scan_timeout_handler(struct work_struct *work);
#endif

netdev_tx_t woal_hard_start_xmit(struct sk_buff *skb, struct net_device *dev);
#ifdef STA_SUPPORT
mlan_status woal_init_sta_dev(struct net_device *dev, moal_private *priv);
#endif
#ifdef UAP_SUPPORT
mlan_status woal_init_uap_dev(struct net_device *dev, moal_private *priv);
#endif
mlan_status woal_update_drv_tbl(moal_handle *handle, int drv_mode_local);
void woal_fill_mlan_buffer(moal_private *priv, mlan_buffer *pmbuf,
			   struct sk_buff *skb);
moal_private *woal_add_interface(moal_handle *handle, t_u8 bss_num,
				 t_u8 bss_type);
void woal_clean_up(moal_handle *handle);
void woal_send_auto_recovery_complete_event(moal_handle *handle);
void woal_send_auto_recovery_failure_event(moal_handle *handle);
void woal_remove_interface(moal_handle *handle, t_u8 bss_index);
void woal_set_multicast_list(struct net_device *dev);
mlan_status woal_request_fw(moal_handle *handle);
mlan_status woal_ioctl_aggr_prio_tbl(moal_private *priv, t_u32 action,
				     mlan_ds_11n_aggr_prio_tbl *aggr_prio_tbl);
mlan_status woal_ioctl_addba_reject(moal_private *priv, t_u32 action,
				    t_u8 *addba_reject);
mlan_status woal_ioctl_tx_ampdu_prot_mode(moal_private *priv, t_u32 action,
					  t_u16 *prot_mode);
mlan_status woal_ioctl_addba_param(moal_private *priv, t_u32 action,
				   mlan_ds_11n_addba_param *addba_param);

int woal_11h_channel_check_ioctl(moal_private *priv, t_u8 wait_option);
void woal_cancel_cac_block(moal_private *priv);
void woal_moal_debug_info(moal_private *priv, moal_handle *handle, u8 flag);

#ifdef STA_SUPPORT
mlan_status woal_get_powermode(moal_private *priv, int *powermode);
mlan_status woal_set_scan_type(moal_private *priv, t_u32 scan_type);
mlan_status woal_get_scan_config(moal_private *priv, mlan_scan_cfg *scan_cfg);
mlan_status woal_enable_ext_scan(moal_private *priv, t_u8 enable);
mlan_status woal_set_powermode(moal_private *priv, char *powermode);
int woal_find_essid(moal_private *priv, mlan_ssid_bssid *ssid_bssid,
		    t_u8 wait_option);
mlan_status woal_find_bssid(moal_private *priv, mlan_802_11_mac_addr bssid);
mlan_status woal_request_userscan(moal_private *priv, t_u8 wait_option,
				  wlan_user_scan_cfg *scan_cfg);
mlan_status woal_do_scan(moal_private *priv, wlan_user_scan_cfg *scan_cfg);
int woal_set_combo_scan(moal_private *priv, char *buf, int length);
mlan_status woal_set_scan_time(moal_private *priv, t_u16 active_scan_time,
			       t_u16 passive_scan_time,
			       t_u16 specific_scan_time);
mlan_status woal_get_band(moal_private *priv, int *band);
mlan_status woal_set_band(moal_private *priv, char *pband);
mlan_status woal_add_rxfilter(moal_private *priv, char *rxfilter);
mlan_status woal_remove_rxfilter(moal_private *priv, char *rxfilter);
mlan_status woal_priv_qos_cfg(moal_private *priv, t_u32 action, char *qos_cfg);
mlan_status woal_set_sleeppd(moal_private *priv, char *psleeppd);
int woal_set_scan_cfg(moal_private *priv, char *buf, int length);
void woal_update_dscp_mapping(moal_private *priv);

/* EVENT: BCN_RSSI_LOW */
#define EVENT_BCN_RSSI_LOW 0x0001
/* EVENT: PRE_BCN_LOST */
#define EVENT_PRE_BCN_LOST 0x0002
mlan_status woal_set_rssi_low_threshold(moal_private *priv, char *rssi,
					t_u8 wait_option);
mlan_status woal_set_rssi_threshold(moal_private *priv, t_u32 event_id,
				    t_u8 wait_option);
/* EVENT: BG_SCAN_REPORT */
#define EVENT_BG_SCAN_REPORT 0x0004
mlan_status woal_set_bg_scan(moal_private *priv, char *buf, int length);
mlan_status woal_stop_bg_scan(moal_private *priv, t_u8 wait_option);
void woal_reconfig_bgscan(moal_handle *handle);
#ifdef STA_CFG80211
void woal_config_bgscan_and_rssi(moal_private *priv, t_u8 set_rssi);
void woal_start_roaming(moal_private *priv);
#endif
mlan_status woal_request_bgscan(moal_private *priv, t_u8 wait_option,
				wlan_bgscan_cfg *scan_cfg);
#endif
#ifdef STA_CFG80211
void woal_save_conn_params(moal_private *priv,
			   struct cfg80211_connect_params *sme);
void woal_clear_conn_params(moal_private *priv);
#endif

void woal_flush_tcp_sess_queue(moal_private *priv);
#ifdef STA_CFG80211
void woal_flush_tdls_list(moal_private *priv);
#endif
void wlan_scan_create_brief_table_entry(t_u8 **ppbuffer,
					BSSDescriptor_t *pbss_desc);
int wlan_get_scan_table_ret_entry(BSSDescriptor_t *pbss_desc, t_u8 **ppbuffer,
				  int *pspace_left);
BOOLEAN woal_ssid_valid(mlan_802_11_ssid *pssid);
int woal_is_connected(moal_private *priv, mlan_ssid_bssid *ssid_bssid);
int woal_priv_hostcmd(moal_private *priv, t_u8 *respbuf, t_u32 respbuflen,
		      t_u8 wait_option);
void woal_flush_tx_stat_queue(moal_private *priv);
struct tx_status_info *woal_get_tx_info(moal_private *priv, t_u8 tx_seq_num);
void woal_remove_tx_info(moal_private *priv, t_u8 tx_seq_num);

void woal_flush_mcast_list(moal_private *priv);
t_void woal_add_mcast_node(moal_private *priv, t_u8 *mcast_addr);
void woal_remove_mcast_node(moal_private *priv, t_u8 *mcast_addr);
t_u8 woal_find_mcast_node_tx(moal_private *priv, struct sk_buff *skb);

mlan_status woal_request_country_power_table(moal_private *priv, char *region,
					     t_u8 wait_option, t_u8 psd_mode);
mlan_status woal_mc_policy_cfg(moal_private *priv, t_u16 *enable,
			       t_u8 wait_option, t_u8 action);
#ifdef UAP_SUPPORT
void woal_check_mc_connection(moal_private *priv, t_u8 wait_option,
			      t_u8 new_channel);
#endif
mlan_status woal_set_low_pwr_mode(moal_handle *handle, t_u8 wait_option);
mlan_status woal_set_chan_track_mode(moal_handle *handle, t_u8 wait_option);
int woal_hexval(char chr);
mlan_status woal_pmic_configure(moal_handle *handle, t_u8 wait_option);
mlan_status woal_set_user_antcfg(moal_handle *handle, t_u8 wait_option);
void woal_hist_data_reset(moal_private *priv);
void woal_hist_do_reset(moal_private *priv, void *data);
void woal_hist_reset_table(moal_private *priv, t_u8 antenna);
void woal_hist_data_add(moal_private *priv, t_u16 rx_rate, t_s8 snr, t_s8 nflr,
			t_u8 antenna);
mlan_status woal_set_hotspotcfg(moal_private *priv, t_u8 wait_option,
				t_u32 hotspotcfg);

#if defined(STA_CFG80211)
mlan_status woal_multi_ap_cfg(moal_private *priv, t_u8 wait_option, t_u8 flag);
struct dhcp_discover_info *woal_get_dhcp_discover_info(moal_private *priv,
						       t_u32 transaction_id);
void woal_flush_dhcp_discover_queue(moal_private *priv);
t_u32 woal_get_dhcp_discover_transation_id(struct sk_buff *skb);
t_void woal_add_dhcp_discover_node(moal_private *priv, t_u32 transaction_id,
				   mlan_buffer *pmbuf);
t_void woal_add_arp_request_node(moal_private *priv, t_u32 hash_key);
t_u32 woal_generate_arp_request_hash(struct sk_buff *skb);
t_void woal_flush_arp_request_entry(moal_private *priv);
#endif

mlan_status woal_set_get_wowlan_config(moal_private *priv, t_u16 action,
				       t_u8 wait_option,
				       mlan_ds_misc_mef_flt_cfg *mefcfg);
mlan_status woal_set_auto_arp_ext(moal_handle *handle, t_u8 enable);
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
mlan_status woal_set_net_monitor(moal_private *priv, t_u8 wait_option,
				 t_u8 enable, t_u8 filter,
				 netmon_band_chan_cfg *band_chan_cfg);
#endif
#ifdef UAP_SUPPORT
mlan_status woal_send_bcn_country_ie_cmd_fw(moal_private *priv,
					    t_u8 wait_option);
#endif
mlan_status woal_delba_all(moal_private *priv, t_u8 wait_option);
#ifdef STA_CFG80211
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 14, 0)
int woal_mkeep_alive_vendor_event(moal_private *priv,
				  pmlan_ds_misc_keep_alive mkeep_alive);
#endif
#endif
int woal_start_mkeep_alive(moal_private *priv, t_u8 mkeep_alive_id,
			   t_u8 *ip_pkt, t_u16 ip_pkt_len, t_u8 *src_mac,
			   t_u8 *dst_mac, t_u32 period_msec,
			   t_u32 retry_interval, t_u8 retry_cnt);
int woal_stop_mkeep_alive(moal_private *priv, t_u8 mkeep_alive_id, t_u8 reset,
			  t_u8 *ip_pkt, t_u8 *pkt_len);
int woal_priv_save_cloud_keep_alive_params(
	moal_private *priv, t_u8 mkeep_alive_id, t_u8 enable, t_u16 ether_type,
	t_u8 *ip_pkt, t_u16 ip_pkt_len, t_u8 *src_mac, t_u8 *dst_mac,
	t_u32 period_msec, t_u32 retry_interval, t_u8 retry_cnt);
int woal_start_mkeep_alive_rx(moal_private *priv, t_u8 mkeep_alive_id,
			      t_u8 *ip_pkt, t_u16 ip_pkt_len, t_u8 *src_mac,
			      t_u8 *dst_mac);
int woal_stop_mkeep_alive_rx(moal_private *priv, t_u8 mkeep_alive_id,
			     t_u8 reset, t_u8 *ip_pkt, t_u8 *pkt_len);
int woal_priv_save_cloud_keep_alive_params_rx(moal_private *priv,
					      t_u8 mkeep_alive_id, t_u8 enable,
					      t_u16 ether_type, t_u8 *ip_pkt,
					      t_u16 ip_pkt_len, t_u8 *src_mac,
					      t_u8 *dst_mac);
void woal_channel_info_to_bandcfg(moal_private *priv,
				  wifi_channel_info *ch_info,
				  Band_Config_t *bandcfg);
void woal_bandcfg_to_channel_info(moal_private *priv, Band_Config_t *bandcfg,
				  t_u8 channel, wifi_channel_info *ch_info);
mlan_status woal_config_rtt(moal_private *priv, t_u8 wait_option,
			    wifi_rtt_config_params_t *rtt_params);
mlan_status woal_cancel_rtt(moal_private *priv, t_u8 wait_option,
			    t_u32 addr_num, t_u8 addr[][MLAN_MAC_ADDR_LENGTH]);
mlan_status woal_rtt_responder_cfg(moal_private *priv, t_u8 wait_option,
				   mlan_rtt_responder *rtt_rsp_cfg);
#ifdef UAP_SUPPORT
mlan_status woal_set_wacp_mode(moal_private *priv, t_u8 wait_option);
#endif
mlan_status woal_init_aggr_ctrl(moal_handle *handle, t_u8 wait_option);

#if defined(STA_CFG80211) && defined(UAP_CFG80211)
monitor_iface *woal_prepare_mon_if(moal_private *priv, const char *name,
				   unsigned char name_assign_type);
#endif

#if defined(STA_CFG80211) || defined(UAP_CFG80211)
#if KERNEL_VERSION(3, 14, 0) <= CFG80211_VERSION_CODE
void woal_cfg80211_vendor_event_fw_dump(moal_private *priv);
#endif
#endif

#ifdef STA_CFG80211
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 1, 0)
mlan_status woal_set_rekey_data(moal_private *priv,
				mlan_ds_misc_gtk_rekey_data *gtk_rekey,
				t_u8 action, t_u8 wait_option);
#endif
#endif

mlan_status woal_vdll_req_fw(moal_handle *handle);

void woal_ioctl_get_misc_conf(moal_private *priv, mlan_ds_misc_cfg *info);
t_u8 woal_get_second_channel_offset(moal_private *priv, int chan);

#ifdef IMX_SUPPORT
void woal_regist_oob_wakeup_irq(moal_handle *handle);
void woal_unregist_oob_wakeup_irq(moal_handle *handle);
void woal_disable_oob_wakeup_irq(moal_handle *handle);
void woal_enable_oob_wakeup_irq(moal_handle *handle);
irqreturn_t woal_oob_wakeup_irq_handler(int irq, void *priv);
#endif /* IMX_SUPPORT */

t_bool woal_secure_add(t_void *datain, t_s32 add, t_void *dataout,
		       data_type type);
t_bool woal_secure_sub(t_void *datain, t_s32 sub, t_void *dataout,
		       data_type type);

mlan_status woal_edmac_cfg(moal_private *priv, t_u8 *country_code);

#ifdef DUMP_TO_PROC
void woal_print_firmware_dump_buf(t_u8 *pfd_buf, t_u64 fwdump_len);
#endif

#if !defined(STA_CFG80211) && !defined(UAP_CFG80211)
unsigned int woal_classify8021d(struct sk_buff *skb);
#endif

#define ENUM_ELEMENT(name, id) name = id
#define ENUM_ELEMENT_LAST(name) name
enum host_error_code_id {
#include "ioctl_error_codes.h"
};
#undef ENUM_ELEMENT
#undef ENUM_ELEMENT_LAST

struct reflective_enum_element {
	int id;
	const char *name;
};

extern const char *wlan_errorcode_get_name(enum host_error_code_id id);

/*
 * Redefine GFP_KERNEL for coverity issue,
 * checker: MISRA C-2012 Rule 10.8,
 * msg: Cast from 16 bit width expression \"0x400U | 0x800U\" to a wider 32 bit
 * type., description: The value of a composite expression shall not be cast to
 * a different essential type category or a wider essential type.
 */
#ifdef __GFP_RECLAIM
#ifdef GFP_KERNEL
#undef GFP_KERNEL
#define GFP_KERNEL                                                             \
	((__GFP_DIRECT_RECLAIM | __GFP_KSWAPD_RECLAIM) | __GFP_IO | __GFP_FS)
#endif
#endif

mlan_status woal_ioctl_hostcmd_htc_cap(moal_private *priv, t_u16 action,
				       t_u8 *enable);
int woal_getset_regrdwr(moal_private *priv, t_u32 action, t_u32 type,
			t_u32 offset, t_u32 *value);

#endif /* _MOAL_MAIN_H */
