/** @file moal_ioctl.c
 *
 * @brief This file contains ioctl function to MLAN
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

#include "moal_main.h"
#include "moal_eth_ioctl.h"
#ifdef SDIO
#include "moal_sdio.h"
#endif
#ifdef USB
#include "moal_usb.h"
#endif
#ifdef UAP_SUPPORT
#include "moal_uap.h"
#endif
#include <linux/list.h>
#if defined(STA_CFG80211) || defined(UAP_CFG80211)
#include "moal_cfg80211.h"
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 14, 0)
#include "moal_cfg80211_util.h"
#endif
#endif

/********************************************************
			Local Variables
********************************************************/
#define MRVL_TLV_HEADER_SIZE 4
/* NXP Channel config TLV ID */
#define MRVL_CHANNELCONFIG_TLV_ID (0x0100 + 0x2a) /* 0x012a */

typedef struct _hostcmd_header {
	/** Command Header : Command */
	t_u16 command;
	/** Command Header : Size */
	t_u16 size;
	/** Command Header : Sequence number */
	t_u16 seq_num;
	/** Command Header : Result */
	t_u16 result;
	/** Command action */
	t_u16 action;
} hostcmd_header, *phostcmd_header;

/** Region code mapping */
typedef struct _region_code_mapping_t {
	/** Region */
	t_u8 region[COUNTRY_CODE_LEN];
	/** Code */
	t_u8 code;
} region_code_mapping_t;

#define EU_REGION_CODE 0x30

/** Region code mapping table */
static region_code_mapping_t region_code_mapping[] = {
	{"00", 0x00}, /* World       */
	{"US", 0x10}, /* US FCC      */
	{"CA", 0x20}, /* IC Canada   */
	{"SG", 0x10}, /* Singapore   */
	{"EU", 0x30}, /* ETSI        */
	{"AU", 0x30}, /* Australia   */
	{"KR", 0x30}, /* Republic Of Korea */
	{"JP", 0x40}, /* Japan       */
	{"CN", 0x50}, /* China       */
	{"BR", 0x09}, /* Brazil      */
	{"RU", 0x0f}, /* Russia      */
	{"IN", 0x06}, /* India       */
	{"MY", 0x06}, /* Malaysia    */
	{"MX", 0x07}, /* Mexico    */
	{"NE", 0x30}, /* New Zeland  */
};

/** EEPROM Region code mapping table */
static region_code_mapping_t hw_region_code_mapping[] = {
	{"00 ", 0x00}, /* World       */
	{"US ", 0x10}, /* US FCC      */
	{"CA ", 0x20}, /* IC Canada   */
	{"KR ", 0x30}, /* Korea       */
	{"CN ", 0x50}, /* China       */
	{"ES ", 0x31}, /* Spain       */
	{"FR ", 0x32}, /* France      */
	{"JP ", 0x40}, /* Japan       */
	{"JP ", 0x41}, /* Japan       */
};

/** Country code for ETSI */
static t_u8 eu_country_code_table[][COUNTRY_CODE_LEN] = {
	"AL", "AD", "AT", "AU", "BE", "BA", "BG", "HR", "CY", "CZ", "DK", "EE",
	"FI", "FR", "MK", "DE", "GR", "HU", "IS", "IE", "IT", "LV", "LI", "LT",
	"LU", "MT", "MD", "MC", "ME", "NL", "NO", "PL", "RO", "SM", "RS", "SI",
	"SK", "ES", "SE", "CH", "TR", "UA", "UK", "GB", "NE", "NZ", "DZ", "AO",
	"AM", "AW", "BH", "BD", "BT", "BO", "BQ", "BW", "VG", "BF", "BI", "KH",
	"CL", "KM", "CG", "CD", "CW", "EG", "FO", "GF", "PF", "GE", "GI", "GP",
	"HK", "IN", "ID", "IM", "IL", "JE", "KE", "XK", "KW", "LA", "LR", "MW",
	"MV", "MQ", "MR", "YT", "MA", "MZ", "MM", "NA", "NC", "NG", "OM", "PS",
	"PT", "QA", "RW", "RE", "BL", "MF", "VC", "SA", "SC", "ZA", "SZ", "SY",
	"TZ", "TG", "TN", "AE", "VA", "EH", "YE", "ZM", "ZW"};

/********************************************************
			Global Variables
********************************************************/

#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 29)
#ifdef UAP_SUPPORT
/** Network device handlers for uAP */
extern const struct net_device_ops woal_uap_netdev_ops;
#endif
#ifdef STA_SUPPORT
/** Network device handlers for STA */
extern const struct net_device_ops woal_netdev_ops;
#endif
#endif

/********************************************************
			Local Functions
********************************************************/
/**
 *  @brief This function converts region string to region code
 *
 *  @param country_code     Region string
 *
 *  @return                 Region code
 */
static t_bool woal_is_country_code_supported(t_u8 *country_code)
{
	t_u8 i;
	ENTER();

	for (i = 0; i < ARRAY_SIZE(region_code_mapping); i++) {
		if (!memcmp(country_code, region_code_mapping[i].region,
			    COUNTRY_CODE_LEN - 1)) {
			PRINTM(MIOCTL,
			       "found country code in region_code table\n");
			LEAVE();
			return MTRUE;
		}
	}

	LEAVE();
	return MFALSE;
}
/**
 *  @brief This function converts region string to region code
 *
 *  @param region_string    Region string
 *
 *  @return                 Region code
 */
static t_u8 region_string_2_region_code(char *region_string)
{
	t_u8 i;

	ENTER();
	for (i = 0; i < ARRAY_SIZE(region_code_mapping); i++) {
		if (!memcmp(region_string, region_code_mapping[i].region,
			    strlen(region_string))) {
			LEAVE();
			return region_code_mapping[i].code;
		}
	}

	/* If still not found, look for code in EU country code table */
	for (i = 0; i < ARRAY_SIZE(eu_country_code_table); i++) {
		if (!memcmp(region_string, eu_country_code_table[i],
			    COUNTRY_CODE_LEN - 1)) {
			PRINTM(MIOCTL, "found region code=%d in EU table\n",
			       EU_REGION_CODE);
			LEAVE();
			return EU_REGION_CODE;
		}
	}

	/* Default is US */
	LEAVE();
	return region_code_mapping[0].code;
}

/**
 *  @brief This function converts region string to region code
 *
 *  @param country_code     Region string
 *
 *  @return                 Region code
 */
t_bool woal_is_etsi_country(t_u8 *country_code)
{
	t_u8 i;
	ENTER();

	for (i = 0; i < ARRAY_SIZE(eu_country_code_table); i++) {
		if (!memcmp(country_code, eu_country_code_table[i],
			    COUNTRY_CODE_LEN - 1)) {
			PRINTM(MIOCTL, "found region code=%d in EU table\n",
			       EU_REGION_CODE);
			LEAVE();
			return MTRUE;
		}
	}

	LEAVE();
	return MFALSE;
}

/**
 *  @brief This function converts region string to region code
 *
 *  @param region_code      region code
 *
 *  @return                 Region string or NULL
 */
char *region_code_2_string(t_u8 region_code)
{
	t_u8 i;

	ENTER();
	for (i = 0; i < ARRAY_SIZE(hw_region_code_mapping); i++) {
		if (hw_region_code_mapping[i].code == region_code) {
			LEAVE();
			return hw_region_code_mapping[i].region;
		}
	}
	LEAVE();
	return NULL;
}

t_u8 woal_is_valid_alpha2(char *alpha2)
{
	if (!alpha2 || strlen(alpha2) < 2)
		return MFALSE;
	if (isalpha(alpha2[0]) && isalpha(alpha2[1]))
		return MTRUE;
	return MFALSE;
}

/**
 * @brief Get second channel offset
 *
 * @param priv         A pointer to moal_private structure
 * @param chan             channel num
 * @return                second channel offset
 */
t_u8 woal_get_second_channel_offset(moal_private *priv, int chan)
{
	t_u8 chan2Offset = SEC_CHAN_NONE;
	mlan_bss_info bss_info;

	memset(&bss_info, 0, sizeof(bss_info));
	/* Special Case: 20Mhz-only Channel */
	if (MLAN_STATUS_SUCCESS !=
	    woal_get_bss_info(priv, MOAL_IOCTL_WAIT, &bss_info))
		return chan2Offset;
	if (bss_info.region_code != COUNTRY_CODE_US && chan == 165)
		return chan2Offset;

	switch (chan) {
	case 36:
	case 44:
	case 52:
	case 60:
	case 100:
	case 108:
	case 116:
	case 124:
	case 132:
	case 140:
	case 149:
	case 157:
	case 165:
	case 173:
		chan2Offset = SEC_CHAN_ABOVE;
		break;
	case 40:
	case 48:
	case 56:
	case 64:
	case 104:
	case 112:
	case 120:
	case 128:
	case 136:
	case 144:
	case 153:
	case 161:
	case 169:
	case 177:
		chan2Offset = SEC_CHAN_BELOW;
		break;
	}
	return chan2Offset;
}

/**
 *  @brief Copy mc address to the mlist
 *
 *  @param mlist    A pointer to mlan_multicast_list structure
 *  @param mac      mc address
 *
 *  @return         N/A
 */
static inline void woal_copy_mc_addr(mlan_multicast_list *mlist,
				     mlan_802_11_mac_addr mac)
{
	t_u32 i = 0;
	for (i = 0; i < mlist->num_multicast_addr; i++) {
		if (!memcmp(&mlist->mac_list[i], mac, ETH_ALEN))
			return;
	}
	if (mlist->num_multicast_addr < MLAN_MAX_MULTICAST_LIST_SIZE)
		moal_memcpy_ext(NULL,
				&mlist->mac_list[mlist->num_multicast_addr],
				mac, ETH_ALEN, sizeof(mlan_802_11_mac_addr));
	mlist->num_multicast_addr++;
	return;
}

/**
 *  @brief Copy multicast table
 *
 *  @param mlist    A pointer to mlan_multicast_list structure
 *  @param dev      A pointer to net_device structure
 *
 *  @return         Number of multicast addresses
 */
static inline int woal_copy_mcast_addr(mlan_multicast_list *mlist,
				       struct net_device *dev)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 35)
	struct dev_mc_list *mcptr = dev->mc_list;
	int i = 0;
#else
	struct netdev_hw_addr *mcptr = NULL;
#endif /* < 2.6.35 */

	ENTER();
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 35)
	for (i = 0; i < dev->mc_count && mcptr; i++) {
		woal_copy_mc_addr(mlist, mcptr->dmi_addr);
		mcptr = mcptr->next;
	}
#else
	// Coverity violation raised for kernel's API
	// coverity[cert_arr39_c_violation:SUPPRESS]
	netdev_for_each_mc_addr (mcptr, dev)
		woal_copy_mc_addr(mlist, mcptr->addr);
#endif /* < 2.6.35 */
	LEAVE();
	return mlist->num_multicast_addr;
}

/**
 *  @brief copy mc list from all the active interface
 *
 *  @param handle  A pointer to moal_handle
 *  @param mlist  A pointer to multicast list
 *
 *  @return       total_mc_count
 */
static int woal_copy_all_mc_list(moal_handle *handle,
				 mlan_multicast_list *mlist)
{
	int i;
	moal_private *priv = NULL;
#ifdef STA_SUPPORT
	int mc_count = 0;
#endif
	ENTER();
	for (i = 0; i < handle->priv_num && (priv = handle->priv[i]); i++) {
#ifdef STA_SUPPORT
		if (GET_BSS_ROLE(handle->priv[i]) == MLAN_BSS_ROLE_STA) {
			if (handle->priv[i]->media_connected == MTRUE) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 35)
				mc_count = priv->netdev->mc_count;
#else
				mc_count = netdev_mc_count(priv->netdev);
#endif
				if (mc_count)
					woal_copy_mcast_addr(mlist,
							     priv->netdev);
			}
		}
#endif
	}
	PRINTM(MIOCTL, "total mc_count=%d\n", mlist->num_multicast_addr);
	LEAVE();
	return mlist->num_multicast_addr;
}

/**
 *  @brief Fill in wait queue
 *
 *  @param priv         A pointer to moal_private structure
 *  @param wait         A pointer to wait_queue structure
 *  @param wait_option  Wait option
 *
 *  @return             N/A
 */
static inline void woal_fill_wait_queue(moal_private *priv, wait_queue *wait,
					t_u8 wait_option)
{
	ENTER();
	wait->start_time = jiffies;
	wait->condition = MFALSE;
	wait->wait_timeout = MFALSE;
	switch (wait_option) {
	case MOAL_NO_WAIT:
		break;
	case MOAL_IOCTL_WAIT:
		init_waitqueue_head(&wait->wait);
		break;
	case MOAL_IOCTL_WAIT_TIMEOUT:
		init_waitqueue_head(&wait->wait);
		wait->wait_timeout = MTRUE;
		break;
	}
	LEAVE();
	return;
}

/**
 *  @brief Wait mlan ioctl complete
 *
 *  @param priv         A pointer to moal_private structure
 *  @param req          A pointer to mlan_ioctl_req structure
 *  @param wait_option  Wait option
 *
 *  @return             N/A
 */
static inline mlan_status woal_wait_ioctl_complete(moal_private *priv,
						   mlan_ioctl_req *req,
						   t_u8 wait_option)
{
	mlan_status status;
	wait_queue *wait = (wait_queue *)req->reserved_1;
	unsigned long flags;

	ENTER();

	priv->phandle->ioctl_timeout = MFALSE;

	switch (wait_option) {
	case MOAL_NO_WAIT:
		break;
	case MOAL_IOCTL_WAIT:
		while (wait_event_interruptible_exclusive(
			       wait->wait, wait->condition) == -ERESTARTSYS &&
		       wait->retry < MAX_RETRY_CNT) {
			wait->retry++;
		}
		break;
	case MOAL_IOCTL_WAIT_TIMEOUT:
		wait_event_timeout(wait->wait, wait->condition,
				   MOAL_IOCTL_TIMEOUT);
		break;
	}
	spin_lock_irqsave(&priv->phandle->driver_lock, flags);
	if (wait->condition == MFALSE) {
		if (wait_option == MOAL_IOCTL_WAIT_TIMEOUT) {
			priv->phandle->ioctl_timeout = MTRUE;
			PRINTM(MMSG,
			       "wlan: IOCTL timeout %p id=0x%x, sub_id=0x%x, wait_option=%d, action=%d\n",
			       req, req->req_id, (*(t_u32 *)req->pbuf),
			       wait_option, (int)req->action);
		} else {
			PRINTM(MMSG,
			       "wlan: IOCTL by signal %p id=0x%x, sub_id=0x%x, wait_option=%d, action=%d\n",
			       req, req->req_id, (*(t_u32 *)req->pbuf),
			       wait_option, (int)req->action);
		}
		req->reserved_1 = 0;
		status = MLAN_STATUS_PENDING;
	} else {
		status = wait->status;
	}
	spin_unlock_irqrestore(&priv->phandle->driver_lock, flags);

	LEAVE();
	return status;
}

/**
 *  @brief CAC period block cmd handler
 *
 *  @param priv     A pointer to moal_private structure
 *  @param req      A pointer to mlan_ioctl_req buffer
 *
 *  @return         MTRUE/MFALSE
 */
static inline t_bool woal_cac_period_block_cmd(moal_private *priv,
					       pmlan_ioctl_req req)
{
	mlan_status ret = MFALSE;
	t_u32 sub_command;

	ENTER();
	if (req == NULL || req->pbuf == NULL)
		goto done;

	sub_command = *(t_u32 *)req->pbuf;

	switch (req->req_id) {
	case MLAN_IOCTL_SCAN:
		if (sub_command == MLAN_OID_SCAN_NORMAL ||
		    sub_command == MLAN_OID_SCAN_SPECIFIC_SSID ||
		    sub_command == MLAN_OID_SCAN_USER_CONFIG)
			ret = MTRUE;
		break;
	case MLAN_IOCTL_BSS:
		if (sub_command == MLAN_OID_BSS_STOP ||
		    sub_command == MLAN_OID_BSS_CHANNEL
		    /* sub_command == MLAN_OID_BSS_ROLE */)
			ret = MTRUE;
#ifdef UAP_SUPPORT
		else if (sub_command == MLAN_OID_UAP_BSS_CONFIG) {
			mlan_ds_bss *bss = (mlan_ds_bss *)req->pbuf;
			if (bss->param.bss_config.channel)
				ret = MTRUE;
			else
				ret = MFALSE;
		}
#endif
		break;
	case MLAN_IOCTL_RADIO_CFG:
		if (sub_command == MLAN_OID_BAND_CFG ||
		    sub_command == MLAN_OID_REMAIN_CHAN_CFG)
			ret = MTRUE;
		break;
	case MLAN_IOCTL_SNMP_MIB:
		if (sub_command == MLAN_OID_SNMP_MIB_DOT11D)
			ret = MTRUE;
#if defined(UAP_SUPPORT)
		if (sub_command == MLAN_OID_SNMP_MIB_DOT11H)
			ret = MTRUE;
#endif
		break;
	case MLAN_IOCTL_11D_CFG:
#ifdef STA_SUPPORT
		if (sub_command == MLAN_OID_11D_CFG_ENABLE)
			ret = MTRUE;
#endif
#ifdef UAP_SUPPORT
		if (sub_command == MLAN_OID_11D_DOMAIN_INFO)
			ret = MTRUE;
#endif
		if (sub_command == MLAN_OID_11D_DOMAIN_INFO_EXT)
			ret = MTRUE;
		break;
	case MLAN_IOCTL_MISC_CFG:
		if (sub_command == MLAN_OID_MISC_REGION)
			ret = MTRUE;
		if (sub_command == MLAN_OID_MISC_HOST_CMD) {
			phostcmd_header phostcmd;
			t_u8 *ptlv_buf;
			t_u16 tag, length;

			phostcmd =
				(phostcmd_header)((pmlan_ds_misc_cfg)req->pbuf)
					->param.hostcmd.cmd;
			ptlv_buf = (t_u8 *)phostcmd + sizeof(hostcmd_header);
			if (phostcmd->action == MLAN_ACT_SET) {
				while (ptlv_buf <
				       (t_u8 *)phostcmd + phostcmd->size) {
					tag = *(t_u16 *)ptlv_buf;
					length = *(t_u16 *)(ptlv_buf + 2);
					/* Check Blocking TLV here, should add
					 * more... */
					if (tag == MRVL_CHANNELCONFIG_TLV_ID) {
						ret = MTRUE;
						break;
					}
					ptlv_buf +=
						(length + MRVL_TLV_HEADER_SIZE);
				}
			}
		}
		break;
	case MLAN_IOCTL_11H_CFG:
		/* Prevent execute more than once */
		if (sub_command == MLAN_OID_11H_CHANNEL_CHECK)
			ret = MTRUE;
		break;
	default:
		ret = MFALSE;
		break;
	}

done:
	LEAVE();
	return ret;
}

/********************************************************
			Global Functions
********************************************************/

/**
 *  @brief Send ioctl request to MLAN
 *
 *  @param priv          A pointer to moal_private structure
 *  @param req           A pointer to mlan_ioctl_req buffer
 *  @param wait_option   Wait option (MOAL_WAIT or MOAL_NO_WAIT)
 *
 *  @return              MLAN_STATUS_SUCCESS/MLAN_STATUS_PENDING
 *                          -- success, otherwise fail
 */
mlan_status woal_request_ioctl(moal_private *priv, mlan_ioctl_req *req,
			       t_u8 wait_option)
{
	wait_queue *wait = NULL;
	mlan_status status;
	unsigned long flags;
	t_u32 sub_command = 0;

	ENTER();

	if (!priv || !priv->phandle || !priv->phandle->pmlan_adapter || !req) {
		PRINTM(MINFO,
		       "priv or priv->phandle or priv->phandle->pmlan_adapter or req is null\n");
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}

	sub_command = *(t_u32 *)req->pbuf;

	if (sub_command != MLAN_OID_GET_DEBUG_INFO &&
	    sub_command != MLAN_OID_MISC_WARM_RESET) {
		if (priv->phandle->surprise_removed == MTRUE ||
		    priv->phandle->driver_status) {
			PRINTM(MCMND,
			       "IOCTL is not allowed while the device is not present or hang\n");
			LEAVE();
			return MLAN_STATUS_FAILURE;
		}
		if (priv->phandle->is_suspended == MTRUE) {
			PRINTM(MCMND, "IOCTL is not allowed while suspended\n");
			LEAVE();
			return MLAN_STATUS_FAILURE;
		}
#ifdef MFG_CMD_SUPPORT
		if (mfg_mode && sub_command != MLAN_OID_MISC_HOST_CMD) {
			PRINTM(MCMND, "Only HOST_CMD allowed in MFG mode\n");
			LEAVE();
			return MLAN_STATUS_FAILURE;
		}
#endif
	}
	/* For MLAN_OID_MISC_HOST_CMD, action is 0, "action set" is checked
	 * later */
	if ((req->action == MLAN_ACT_SET || req->action == 0) &&
	    priv->phandle->cac_period == MTRUE) {
		/* CAC checking period left to complete jiffies */
		long cac_left_jiffies;

		/* cac_left_jiffies will be negative if and only if
		 * event MLAN_EVENT_ID_DRV_MEAS_REPORT recieved from FW
		 * after CAC measure period ends,
		 * usually this could be considered as a FW bug
		 */
		cac_left_jiffies =
			priv->phandle->cac_timer_jiffies -
			(jiffies - priv->phandle->meas_start_jiffies);
		if (priv->phandle->cac_period_jiffies) {
			cac_left_jiffies =
				priv->phandle->cac_period_jiffies -
				(jiffies - priv->phandle->meas_start_jiffies);
		}
		if (priv->phandle->cac_restart)
			cac_left_jiffies = DEF_CAC_DWELL_TIME * HZ / 1000;
		if (cac_left_jiffies < 0) {
			/* Avoid driver hang in FW died during CAC measure
			 * period */
			priv->phandle->cac_period = MFALSE;
			PRINTM(MERROR,
			       "CAC measure period spends longer than scheduled time "
			       "or meas done event never received\n");
			status = MLAN_STATUS_FAILURE;
#ifdef UAP_SUPPORT
#if defined(STA_CFG80211) || defined(UAP_CFG80211)
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 14, 0)
			if (priv->uap_host_based &&
			    moal_extflg_isset(priv->phandle, EXT_DFS_OFFLOAD))
				woal_cfg80211_dfs_vendor_event(
					priv, event_dfs_cac_aborted,
					&priv->chan);
#endif
#endif
#endif

			goto done;
		}

		/* Check BSS START first */
		if (sub_command == MLAN_OID_BSS_START) {
			mlan_ds_bss *bss;
			bss = (mlan_ds_bss *)req->pbuf;
			/*
			 * Bss delay start after channel report received,
			 * not block the driver by delay executing. This is
			 * because a BSS_START cmd is always executed right
			 * after channel check issued.
			 */
			if (priv->phandle->delay_bss_start == MFALSE) {
				PRINTM(MMSG,
				       "Received BSS Start command during CAC period, delay executing %ld seconds\n",
				       cac_left_jiffies / HZ);
				priv->phandle->delay_bss_start = MTRUE;
				moal_memcpy_ext(
					priv->phandle,
					&priv->phandle->delay_ssid_bssid,
					&bss->param.ssid_bssid,
					sizeof(mlan_ssid_bssid),
					sizeof(mlan_ssid_bssid));
				/* TODO: return success to allow the half below
				 * of routines of which calling BSS start to
				 * execute
				 */
				status = MLAN_STATUS_SUCCESS;
				goto done;
			} else {
				/* TODO: not blocking it, just return failure */
				PRINTM(MMSG,
				       "Only one BSS Start command allowed for delay executing!\n");
				status = MLAN_STATUS_FAILURE;
				goto done;
			}
		}
		if (woal_cac_period_block_cmd(priv, req)) {
			priv->phandle->meas_wait_q_woken = MFALSE;
			PRINTM(MMSG,
			       "CAC check is on going... Blocking Command %ld seconds\n",
			       cac_left_jiffies / HZ);
			/* blocking timeout set to 1.5 * CAC checking period
			 * left time */
			wait_event_interruptible_timeout(
				priv->phandle->meas_wait_q,
				priv->phandle->meas_wait_q_woken,
				cac_left_jiffies * 3 / 2);
		}
	}
#ifdef UAP_CFG80211
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 12, 0)
	else if (priv->phandle->is_cac_timer_set &&
		 (req->action == MLAN_ACT_SET || req->action == 0)) {
		if (woal_cac_period_block_cmd(priv, req)) {
			PRINTM(MMSG,
			       "CAC check is on going... Blocking Command\n");
			status = MLAN_STATUS_FAILURE;
			goto done;
		}
	}
#endif
#endif
	else if (priv->phandle->cac_period) {
		PRINTM(MINFO, "Operation during CAC check period.\n");
	}
	wait = (wait_queue *)req->reserved_1;
	req->bss_index = priv->bss_index;
	if (wait_option)
		woal_fill_wait_queue(priv, wait, wait_option);
	else
		req->reserved_1 = 0;

	/* Call MLAN ioctl handle */
	atomic_inc(&priv->phandle->ioctl_pending);
	spin_lock_irqsave(&priv->phandle->ioctl_lock, flags);
	status = mlan_ioctl(priv->phandle->pmlan_adapter, req);
	spin_unlock_irqrestore(&priv->phandle->ioctl_lock, flags);
	switch (status) {
	case MLAN_STATUS_PENDING:
		if (wait_option == MOAL_NO_WAIT)
			PRINTM(MIOCTL, "IOCTL MOAL_NO_WAIT: %p\n", req);
		else
			PRINTM(MIOCTL,
			       "IOCTL pending: %p id=0x%x, sub_id=0x%x wait_option=%d, action=%d\n",
			       req, req->req_id, (*(t_u32 *)req->pbuf),
			       wait_option, (int)req->action);
		/* Status pending, wake up main process */
		queue_work(priv->phandle->workqueue, &priv->phandle->main_work);

		/* Wait for completion */
		if (wait_option)
			status = woal_wait_ioctl_complete(priv, req,
							  wait_option);
		break;
	case MLAN_STATUS_SUCCESS:
	case MLAN_STATUS_FAILURE:
	case MLAN_STATUS_RESOURCE:
		if (req)
			PRINTM(MIOCTL,
			       "IOCTL: %p id=0x%x, sub_id=0x%x wait_option=%d, action=%d status=%d\n",
			       req, req->req_id, (*(t_u32 *)req->pbuf),
			       wait_option, (int)req->action, status);
		atomic_dec(&priv->phandle->ioctl_pending);
		break;
	default:
		atomic_dec(&priv->phandle->ioctl_pending);
		break;
	}

done:
	LEAVE();
	return status;
}

/**
 *  @brief Send set MAC address request to MLAN
 *
 *  @param priv   A pointer to moal_private structure
 *  @param wait_option wait option
 *
 *  @return       MLAN_STATUS_SUCCESS/MLAN_STATUS_PENDING
 *                  -- success, otherwise fail
 */
mlan_status woal_request_set_mac_address(moal_private *priv, t_u8 wait_option)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_bss *bss = NULL;
	mlan_status status;
	ENTER();

	/* Allocate an IOCTL request buffer */
	req = (mlan_ioctl_req *)woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_bss));
	if (req == NULL) {
		status = MLAN_STATUS_FAILURE;
		goto done;
	}

	/* Fill request buffer */
	bss = (mlan_ds_bss *)req->pbuf;
	bss->sub_command = MLAN_OID_BSS_MAC_ADDR;
	moal_memcpy_ext(priv->phandle, &bss->param.mac_addr, priv->current_addr,
			sizeof(mlan_802_11_mac_addr),
			sizeof(mlan_802_11_mac_addr));
	req->req_id = MLAN_IOCTL_BSS;
	req->action = MLAN_ACT_SET;

	/* Send IOCTL request to MLAN */
	status = woal_request_ioctl(priv, req, wait_option);
	if (status == MLAN_STATUS_FAILURE) {
		PRINTM(MERROR,
		       "set mac address failed! status=%d, error_code=0x%x\n",
		       status, req->status_code);
	} else {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 16, 0)
		eth_hw_addr_set(priv->netdev, priv->current_addr);
#else
		moal_memcpy_ext(priv->phandle, priv->netdev->dev_addr,
				priv->current_addr, ETH_ALEN, ETH_ALEN);
#endif
		HEXDUMP("priv->MacAddr:", priv->current_addr, ETH_ALEN);
	}
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return status;
}

/**
 *  @brief Set PSK
 *
 *  @param priv                 A pointer to moal_private structure
 *  @param wait_option          Wait option
 *  @param ssid_bssid           A pointer to mlan_ssid_bssid structure
 *
 *  @return                     MLAN_STATUS_SUCCESS/MLAN_STATUS_PENDING --
 * success, otherwise fail
 */

mlan_status woal_set_psk_11ai(moal_private *priv, t_u8 wait_option,
			      const t_u8 *addr, const t_u8 *key, int key_len)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_sec_cfg *sec = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	if (key_len) {
		/* Allocate an IOCTL request buffer */
		req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_sec_cfg));
		if (req == NULL) {
			status = MLAN_STATUS_FAILURE;
			goto error;
		}
		/* Fill request buffer */
		sec = (mlan_ds_sec_cfg *)req->pbuf;
		sec->sub_command = MLAN_OID_SEC_CFG_PASSPHRASE;
		req->req_id = MLAN_IOCTL_SEC_CFG;
		req->action = MLAN_ACT_SET;

		/* Try Get All */
		memset(&sec->param.passphrase, 0, sizeof(mlan_ds_passphrase));

		if (addr)
			moal_memcpy_ext(priv->phandle,
					&sec->param.passphrase.bssid, addr,
					MLAN_MAC_ADDR_LENGTH,
					sizeof(mlan_802_11_mac_addr));

		sec->param.passphrase.psk_type = MLAN_PSK_PMK;

		moal_memcpy_ext(priv->phandle,
				&(sec->param.passphrase.psk.pmk.pmk), key,
				MLAN_MAX_KEY_LENGTH,
				sizeof(sec->param.passphrase.psk.pmk.pmk));

		/* Send IOCTL request to MLAN */
		status = woal_request_ioctl(priv, req, wait_option);
		if (status != MLAN_STATUS_SUCCESS) {
			PRINTM(MERROR, "11AI: Failed to set PSK\n");
			goto error;
		}
	}

error:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return status;
}

/**
 *  @brief Send multicast list request to MLAN
 *
 *  @param priv   A pointer to moal_private structure
 *  @param dev    A pointer to net_device structure
 *
 *  @return       N/A
 */
void woal_request_set_multicast_list(moal_private *priv, struct net_device *dev)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_bss *bss = NULL;
	mlan_status status;
	int mc_count = 0;

	ENTER();

	/* Allocate an IOCTL request buffer */
	req = (mlan_ioctl_req *)woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_bss));
	if (req == NULL) {
		PRINTM(MERROR, "%s:Fail to allocate ioctl req buffer\n",
		       __func__);
		goto done;
	}

	/* Fill request buffer */
	bss = (mlan_ds_bss *)req->pbuf;
	bss->sub_command = MLAN_OID_BSS_MULTICAST_LIST;
	req->req_id = MLAN_IOCTL_BSS;
	req->action = MLAN_ACT_SET;
	if (dev->flags & IFF_ALLMULTI) {
		bss->param.multicast_list.mode = MLAN_ALL_MULTI_MODE;
	} else {
		if (dev->flags & IFF_PROMISC)
			bss->param.multicast_list.mode = MLAN_PROMISC_MODE;
		bss->param.multicast_list.mode |= MLAN_MULTICAST_MODE;
		mc_count = woal_copy_all_mc_list(priv->phandle,
						 &bss->param.multicast_list);
		if (mc_count > MLAN_MAX_MULTICAST_LIST_SIZE)
			bss->param.multicast_list.mode = MLAN_ALL_MULTI_MODE;
	}
	PRINTM(MCMND, "%s set multicast_list\n", dev->name);
	/* Send IOCTL request to MLAN */
	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
done:
	LEAVE();
	return;
}

/**
 *  @brief Send deauth command to MLAN
 *
 *  @param priv          A pointer to moal_private structure
 *  @param wait_option          Wait option
 *  @param mac           MAC address to deauthenticate
 *  @param reason code   reason code to deauthenticate
 *  @return              MLAN_STATUS_SUCCESS/MLAN_STATUS_PENDING -- success,
 * otherwise fail
 */
mlan_status woal_disconnect(moal_private *priv, t_u8 wait_option, t_u8 *mac,
			    t_u16 reason_code)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_bss *bss = NULL;
	mlan_status status;

	ENTER();

	/* Allocate an IOCTL request buffer */
	req = (mlan_ioctl_req *)woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_bss));
	if (req == NULL) {
		status = MLAN_STATUS_FAILURE;
		goto done;
	}

	/* Fill request buffer */
	bss = (mlan_ds_bss *)req->pbuf;
	bss->sub_command = MLAN_OID_BSS_STOP;
	if (mac)
		moal_memcpy_ext(priv->phandle,
				&bss->param.deauth_param.mac_addr, mac,
				sizeof(mlan_802_11_mac_addr),
				sizeof(bss->param.deauth_param.mac_addr));
	bss->param.deauth_param.reason_code = reason_code;
	req->req_id = MLAN_IOCTL_BSS;
	req->action = MLAN_ACT_SET;

	/* Send IOCTL request to MLAN */
	status = woal_request_ioctl(priv, req, wait_option);

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
#ifdef REASSOCIATION
	priv->reassoc_required = MFALSE;
#endif /* REASSOCIATION */
	priv->auto_assoc_priv.drv_assoc.status = MFALSE;
	priv->auto_assoc_priv.drv_reconnect.status = MFALSE;
	LEAVE();
	return status;
}

#if defined(UAP_SUPPORT)
/**
 * @brief               Get non-global oper class
 *
 * @param priv          Pointer to moal_private structure
 * @param bw            bandwidth
 * @param channel       channel
 * @param oper_class    pointer to oper_class

 *  @return             non-global operclass
 */
int woal_priv_get_nonglobal_operclass_by_bw_channel(moal_private *priv,
						    t_u8 bandwidth,
						    t_u8 channel,
						    t_u8 *oper_class)
{
	int ret = 0;
	mlan_ioctl_req *ioctl_req = NULL;
	mlan_ds_misc_cfg *misc = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	ioctl_req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (ioctl_req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	misc = (mlan_ds_misc_cfg *)ioctl_req->pbuf;
	misc->sub_command = MLAN_OID_MISC_OPER_CLASS;
	ioctl_req->req_id = MLAN_IOCTL_MISC_CFG;
	ioctl_req->action = MLAN_ACT_GET;
	misc->param.bw_chan_oper.bandwidth = bandwidth;
	misc->param.bw_chan_oper.channel = channel;

	status = woal_request_ioctl(priv, ioctl_req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}
	*oper_class = misc->param.bw_chan_oper.oper_class;

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(ioctl_req);

	LEAVE();
	return ret;
}
#endif

#ifdef UAP_SUPPORT
/**
 *  @brief Check current uap/go connection status
 *         Need handle channel switch if current channel is DFS channel
 *
 *  @param priv          A pointer to moal_private structure
 *  @param wait_option          Wait option
 *  @param new channel          new channel
 *  @return              N/A
 */
static void woal_check_uap_dfs_status(moal_private *priv, t_u8 wait_option,
				      t_u8 new_channel)
{
	chan_band_info channel;
	mlan_bss_info bss_info;
#if defined(UAP_SUPPORT)
	IEEEtypes_ChanSwitchAnn_t *chan_switch = NULL;
	IEEEtypes_ExtChanSwitchAnn_t *ext_chan_switch = NULL;
	custom_ie *pcust_chansw_ie = NULL;
	mlan_ioctl_req *ioctl_req = NULL;
	mlan_ds_misc_cfg *misc = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;
	t_u8 bw = 0, oper_class = 0;
#endif

	/* Get BSS information */
	if (MLAN_STATUS_SUCCESS !=
	    woal_get_bss_info(priv, wait_option, &bss_info))
		goto done;
	if (MLAN_STATUS_SUCCESS !=
	    woal_set_get_ap_channel(priv, MLAN_ACT_GET, wait_option, &channel))
		goto done;
	PRINTM(MCMND, "is_11h_active=%d dfs_check_channel=%d\n",
	       bss_info.is_11h_active, bss_info.dfs_check_channel);
	PRINTM(MCMND, "uap current channel=%d new_channel=%d\n",
	       channel.channel, new_channel);
#if defined(UAP_SUPPORT)
	if (new_channel == channel.channel)
		goto done;
	if (bss_info.is_11h_active &&
	    (bss_info.dfs_check_channel == channel.channel)) {
		if (new_channel < MAX_BG_CHANNEL) {
			bw = 20;
		} else {
			switch (channel.bandcfg.chanWidth) {
			case CHAN_BW_20MHZ:
				bw = 20;
				break;
			case CHAN_BW_40MHZ:
				bw = 40;
				break;
			case CHAN_BW_80MHZ:
				bw = 80;
				break;
			default:
				break;
			}
		}
		woal_priv_get_nonglobal_operclass_by_bw_channel(
			priv, bw, new_channel, &oper_class);
		PRINTM(MCMND,
		       "Switch the uap channel from %d to %d, oper_class=%d bw=%d\n",
		       channel.channel, new_channel, oper_class, bw);
		ioctl_req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
		if (ioctl_req) {
			misc = (mlan_ds_misc_cfg *)ioctl_req->pbuf;
			misc->sub_command = MLAN_OID_MISC_CUSTOM_IE;
			ioctl_req->req_id = MLAN_IOCTL_MISC_CFG;
			ioctl_req->action = MLAN_ACT_SET;
			misc->param.cust_ie.type = TLV_TYPE_MGMT_IE;
			misc->param.cust_ie.len =
				sizeof(custom_ie) - MAX_IE_SIZE;
			pcust_chansw_ie = (custom_ie *)&misc->param.cust_ie
						  .ie_data_list[0];
			pcust_chansw_ie->ie_index = 0xffff; /*Auto index */
			if (!oper_class) {
				pcust_chansw_ie->ie_length =
					sizeof(IEEEtypes_ChanSwitchAnn_t);
				pcust_chansw_ie->mgmt_subtype_mask =
					MGMT_MASK_BEACON |
					MGMT_MASK_PROBE_RESP; /*Add IE for
								 BEACON/probe
								 resp*/
				chan_switch =
					(IEEEtypes_ChanSwitchAnn_t *)
						pcust_chansw_ie->ie_buffer;
				chan_switch->element_id = CHANNEL_SWITCH_ANN;
				chan_switch->len =
					sizeof(IEEEtypes_ChanSwitchAnn_t) -
					sizeof(IEEEtypes_Header_t);
				chan_switch->chan_switch_mode = 1; /* STA should
								      not
								      transmit
								    */
				chan_switch->new_channel_num = new_channel;
				chan_switch->chan_switch_count =
					DEF_CHAN_SWITCH_COUNT;
				DBG_HEXDUMP(MCMD_D, "CSA IE",
					    (t_u8 *)pcust_chansw_ie->ie_buffer,
					    pcust_chansw_ie->ie_length);
			} else {
				pcust_chansw_ie->ie_length =
					sizeof(IEEEtypes_ExtChanSwitchAnn_t);
				pcust_chansw_ie->mgmt_subtype_mask =
					MGMT_MASK_BEACON |
					MGMT_MASK_PROBE_RESP; /*Add IE for
								 BEACON/probe
								 resp*/
				ext_chan_switch =
					(IEEEtypes_ExtChanSwitchAnn_t *)
						pcust_chansw_ie->ie_buffer;
				ext_chan_switch->element_id =
					EXTEND_CHANNEL_SWITCH_ANN;
				ext_chan_switch->len =
					sizeof(IEEEtypes_ExtChanSwitchAnn_t) -
					sizeof(IEEEtypes_Header_t);
				ext_chan_switch->chan_switch_mode =
					1; /* STA should not transmit */
				ext_chan_switch->new_channel_num = new_channel;
				ext_chan_switch->chan_switch_count =
					DEF_CHAN_SWITCH_COUNT;
				ext_chan_switch->new_oper_class = oper_class;
				DBG_HEXDUMP(MCMD_D, "ECSA IE",
					    (t_u8 *)pcust_chansw_ie->ie_buffer,
					    pcust_chansw_ie->ie_length);
			}
			status = woal_request_ioctl(priv, ioctl_req,
						    wait_option);
			if (status != MLAN_STATUS_SUCCESS) {
				PRINTM(MERROR, "Failed to set CSA IE\n");
				goto done;
			}
			PRINTM(MCMND, "CSA/ECSA ie index=%d\n",
			       pcust_chansw_ie->ie_index);
			priv->phandle->chsw_wait_q_woken = MFALSE;
			/* wait for channel switch to complete  */
			wait_event_interruptible_timeout(
				priv->phandle->chsw_wait_q,
				priv->phandle->chsw_wait_q_woken,
				(u32)HZ * (DEF_CHAN_SWITCH_COUNT + 2) * 110 /
					1000);

			pcust_chansw_ie->ie_index = 0xffff;
			pcust_chansw_ie->mgmt_subtype_mask =
				MLAN_CUSTOM_IE_DELETE_MASK;
			status = woal_request_ioctl(priv, ioctl_req,
						    MOAL_IOCTL_WAIT);
			if (status != MLAN_STATUS_SUCCESS) {
				PRINTM(MERROR, "Failed to clear CSA/ECSA IE\n");
			}
		}
	}
#endif
done:
#if defined(UAP_SUPPORT)
	if (status != MLAN_STATUS_PENDING)
		kfree(ioctl_req);
#endif
	return;
}

/**
 *  @brief Check current multi-channel connections
 *
 *  @param priv          A pointer to moal_private structure
 *  @param wait_option          Wait option
 *  @param new channel  new channel
 *
 *  @return              N/A
 */
void woal_check_mc_connection(moal_private *priv, t_u8 wait_option,
			      t_u8 new_channel)
{
	moal_handle *handle = priv->phandle;
#ifdef UAP_SUPPORT
	int i;
#endif
	t_u16 enable = 0;
	if (priv->phandle->card_info->drcs &&
	    moal_extflg_isset(priv->phandle, EXT_CFG80211_DRCS)) {
		if (woal_mc_policy_cfg(priv, &enable, wait_option,
				       MLAN_ACT_GET)) {
			PRINTM(MERROR, "Get multi-channel policy failed\n");
		}
	}

	if (!enable)
		return;
#ifdef UAP_SUPPORT
	for (i = 0; i < handle->priv_num; i++) {
		if (handle->priv[i] &&
		    GET_BSS_ROLE(handle->priv[i]) == MLAN_BSS_ROLE_UAP) {
			if (handle->priv[i]->bss_started == MTRUE)
				woal_check_uap_dfs_status(handle->priv[i],
							  wait_option,
							  new_channel);
		}
	}
#endif
	return;
}
#endif

/**
 *  @brief Send bss_start command to MLAN
 *
 *  @param priv          A pointer to moal_private structure
 *  @param wait_option          Wait option
 *  @param ssid_bssid    A point to mlan_ssid_bssid structure
 *
 *  @return              MLAN_STATUS_SUCCESS/MLAN_STATUS_PENDING -- success,
 * otherwise fail
 */
mlan_status woal_bss_start(moal_private *priv, t_u8 wait_option,
			   mlan_ssid_bssid *ssid_bssid)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_bss *bss = NULL;
	mlan_status status;
#ifdef UAP_SUPPORT
	mlan_ssid_bssid *temp_ssid_bssid = NULL;
#endif

	ENTER();

	/* Stop the O.S. TX queue When we are roaming */
	woal_stop_queue(priv->netdev);
	if (priv->media_connected == MFALSE) {
		if (netif_carrier_ok(priv->netdev))
			netif_carrier_off(priv->netdev);
	}
#ifdef UAP_SUPPORT
	if (!ssid_bssid) {
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}
	temp_ssid_bssid = kzalloc(sizeof(mlan_ssid_bssid), GFP_ATOMIC);
	if (!temp_ssid_bssid) {
		PRINTM(MERROR, "Fail to allocate ssid_bssid buffer\n");
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}

	moal_memcpy_ext(priv->phandle, temp_ssid_bssid, ssid_bssid,
			sizeof(mlan_ssid_bssid), sizeof(mlan_ssid_bssid));
	if (MLAN_STATUS_SUCCESS ==
	    woal_find_best_network(priv, wait_option, temp_ssid_bssid))
		woal_check_mc_connection(priv, wait_option,
					 temp_ssid_bssid->channel);
	kfree(temp_ssid_bssid);
#endif

	if (MOAL_ACQ_SEMAPHORE_BLOCK(&priv->phandle->async_sem)) {
		PRINTM(MERROR, "Acquire semaphore error, woal_bss_start\n");
		LEAVE();
		return -EBUSY;
	}
	/* Allocate an IOCTL request buffer */
	req = (mlan_ioctl_req *)woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_bss));
	if (req == NULL) {
		status = MLAN_STATUS_FAILURE;
		goto done;
	}

	/* Fill request buffer */
	bss = (mlan_ds_bss *)req->pbuf;
	bss->sub_command = MLAN_OID_BSS_START;
	if (ssid_bssid)
		moal_memcpy_ext(priv->phandle, &bss->param.ssid_bssid,
				ssid_bssid, sizeof(mlan_ssid_bssid),
				sizeof(mlan_ssid_bssid));
	req->req_id = MLAN_IOCTL_BSS;
	req->action = MLAN_ACT_SET;

	/* Send IOCTL request to MLAN */
	status = woal_request_ioctl(priv, req, wait_option);
	if (ssid_bssid)
		moal_memcpy_ext(priv->phandle, ssid_bssid,
				&bss->param.ssid_bssid, sizeof(mlan_ssid_bssid),
				sizeof(mlan_ssid_bssid));
#ifdef STA_CFG80211
#ifdef STA_SUPPORT
	if (status != MLAN_STATUS_PENDING)
		priv->assoc_status = req->status_code;
#endif
#endif
done:
	MOAL_REL_SEMAPHORE(&priv->phandle->async_sem);
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return status;
}

/**
 *  @brief Get BSS info
 *
 *  @param priv                 A pointer to moal_private structure
 *  @param wait_option          Wait option
 *  @param bss_info             A pointer to mlan_bss_info structure
 *
 *  @return                     MLAN_STATUS_SUCCESS/MLAN_STATUS_PENDING --
 * success, otherwise fail
 */
mlan_status woal_get_bss_info(moal_private *priv, t_u8 wait_option,
			      mlan_bss_info *bss_info)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_get_info *info = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;
	ENTER();

	/* Allocate an IOCTL request buffer */
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_get_info));
	if (req == NULL) {
		PRINTM(MERROR,
		       "Fail to allocate the buffer for get bss_info\n");
		status = MLAN_STATUS_FAILURE;
		goto done;
	}

	/* Fill request buffer */
	info = (mlan_ds_get_info *)req->pbuf;
	info->sub_command = MLAN_OID_GET_BSS_INFO;
	req->req_id = MLAN_IOCTL_GET_INFO;
	req->action = MLAN_ACT_GET;

	/* Send IOCTL request to MLAN */
	status = woal_request_ioctl(priv, req, wait_option);
	if (status == MLAN_STATUS_SUCCESS) {
		if (bss_info)
			moal_memcpy_ext(priv->phandle, bss_info,
					&info->param.bss_info,
					sizeof(mlan_bss_info),
					sizeof(mlan_bss_info));
	}
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return status;
}

/**
 *  @brief Set/Get generic IE
 *
 *  @param priv         A pointer to moal_private structure
 *  @param action       Action set or get
 *  @param ie           Information element to set
 *  @param get_ie       Information element to get
 *  @param ie_len       Length of the IE
 *  @param wait_option  wait option
 *
 *  @return             MLAN_STATUS_SUCCESS -- success, otherwise fail
 */
mlan_status woal_set_get_gen_ie(moal_private *priv, t_u32 action,
				const t_u8 *ie, t_u8 *get_ie, int *ie_len,
				t_u8 wait_option)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_ds_misc_cfg *misc = NULL;
	mlan_ioctl_req *req = NULL;

	ENTER();

	if ((action == MLAN_ACT_GET) && (get_ie == NULL || ie_len == NULL)) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	if (action == MLAN_ACT_SET && *ie_len > MAX_IE_SIZE) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	misc = (mlan_ds_misc_cfg *)req->pbuf;
	misc->sub_command = MLAN_OID_MISC_GEN_IE;
	req->req_id = MLAN_IOCTL_MISC_CFG;
	req->action = action;
	misc->param.gen_ie.type = MLAN_IE_TYPE_GEN_IE;

	if (action == MLAN_ACT_SET) {
		misc->param.gen_ie.len = *ie_len;
		if (*ie_len)
			moal_memcpy_ext(priv->phandle,
					misc->param.gen_ie.ie_data, ie, *ie_len,
					MAX_IE_SIZE);
	}

	ret = woal_request_ioctl(priv, req, wait_option);
	if (ret != MLAN_STATUS_SUCCESS)
		goto done;

	if (action == MLAN_ACT_GET) {
		*ie_len = misc->param.gen_ie.len;
		if (*ie_len)
			moal_memcpy_ext(priv->phandle, get_ie,
					misc->param.gen_ie.ie_data, *ie_len,
					*ie_len);
	}

done:
	if (ret != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

#ifdef STA_SUPPORT
/**
 *  @brief Set/Get retry count
 *
 *  @param priv                 A pointer to moal_private structure
 *  @param action               Action set or get
 *  @param wait_option          Wait option
 *  @param value                Retry value
 *
 *  @return                     MLAN_STATUS_SUCCESS/MLAN_STATUS_PENDING --
 * success, otherwise fail
 */
mlan_status woal_set_get_retry(moal_private *priv, t_u32 action,
			       t_u8 wait_option, int *value)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_snmp_mib *mib = NULL;
	mlan_status ret = MLAN_STATUS_SUCCESS;

	ENTER();

	/* Allocate an IOCTL request buffer */
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_snmp_mib));
	if (req == NULL) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	/* Fill request buffer */
	mib = (mlan_ds_snmp_mib *)req->pbuf;
	mib->sub_command = MLAN_OID_SNMP_MIB_RETRY_COUNT;
	req->req_id = MLAN_IOCTL_SNMP_MIB;
	req->action = action;

	if (action == MLAN_ACT_SET) {
		if (*value < MLAN_TX_RETRY_MIN || *value > MLAN_TX_RETRY_MAX) {
			ret = MLAN_STATUS_FAILURE;
			goto done;
		}
		mib->param.retry_count = *value;
	}

	/* Send IOCTL request to MLAN */
	ret = woal_request_ioctl(priv, req, wait_option);
	if (ret == MLAN_STATUS_SUCCESS && action == MLAN_ACT_GET)
		*value = mib->param.retry_count;
#ifdef STA_CFG80211
	/* If set is invoked from other than iw i.e iwconfig,
	 * wiphy retry count should be updated as well */
	if (IS_STA_CFG80211(priv->phandle->params.cfg80211_wext) &&
	    priv->wdev && priv->wdev->wiphy &&
	    (GET_BSS_ROLE(priv) == MLAN_BSS_ROLE_STA) &&
	    (action == MLAN_ACT_SET)) {
		priv->wdev->wiphy->retry_long = (t_u8)*value;
		priv->wdev->wiphy->retry_short = (t_u8)*value;
	}
#endif

done:
	if (ret != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 * @brief Performs pre-warm-reset
 *
 * @param priv         A pointer to moal_private structure
 *
 * @return         0 if successful else negative value
 */
int woal_pre_warmreset(moal_private *priv)
{
	moal_handle *handle = priv->phandle;
	int ret = 0;
	int intf_num;
#ifdef WIFI_DIRECT_SUPPORT
#if defined(STA_SUPPORT) && defined(UAP_SUPPORT)
#if defined(STA_WEXT) || defined(UAP_WEXT)
	t_u8 bss_role = MLAN_BSS_ROLE_STA;
#endif
#endif
#endif /* WIFI_DIRECT_SUPPORT */

	ENTER();
#ifdef USB
#ifdef CONFIG_USB_SUSPEND
	if (IS_USB(handle->card_type) && handle->is_suspended &&
	    woal_exit_usb_suspend(handle)) {
		PRINTM(MERROR, "Failed to resume the suspended device\n");
		LEAVE();
		return -EFAULT;
	}
#endif /* CONFIG_USB_SUSPEND */
#endif
	woal_cancel_cac_block(priv);
	/* Reset all interfaces */
	woal_reset_intf(priv, MOAL_IOCTL_WAIT, MTRUE);
	/* Initialize private structures */
	for (intf_num = 0; intf_num < handle->priv_num; intf_num++) {
		if (!handle->priv[intf_num])
			continue;

		woal_init_priv(handle->priv[intf_num], MOAL_IOCTL_WAIT);
#ifdef WIFI_DIRECT_SUPPORT
#if defined(STA_SUPPORT) && defined(UAP_SUPPORT)
#if defined(STA_WEXT) || defined(UAP_WEXT)
		if ((handle->priv[intf_num]->bss_type ==
		     MLAN_BSS_TYPE_WIFIDIRECT) &&
		    (GET_BSS_ROLE(handle->priv[intf_num]) ==
		     MLAN_BSS_ROLE_UAP)) {
			if (MLAN_STATUS_SUCCESS !=
			    woal_bss_role_cfg(handle->priv[intf_num],
					      MLAN_ACT_SET, MOAL_IOCTL_WAIT,
					      &bss_role)) {
				ret = -EFAULT;
				goto done;
			}
		}
#endif /* STA_WEXT || UAP_WEXT */
#endif /* STA_SUPPORT && UAP_SUPPORT */
#endif /* WIFI_DIRECT_SUPPORT */
	}
	woal_shutdown_fw(priv, MOAL_IOCTL_WAIT);
#ifdef WIFI_DIRECT_SUPPORT
#if defined(STA_SUPPORT) && defined(UAP_SUPPORT)
#if defined(STA_WEXT) || defined(UAP_WEXT)
done:
#endif /* STA_WEXT || UAP_WEXT */
#endif /* STA_SUPPORT && UAP_SUPPORT */
#endif /* WIFI_DIRECT_SUPPORT */
	LEAVE();
	return ret;
}

/**
 * @brief warm reset
 *
 * @param priv         A pointer to moal_private structure
 *
 * @return        0 success, otherwise failure
 */
int woal_warmreset(moal_private *priv)
{
	moal_handle *handle = priv->phandle;
	mlan_ioctl_req *req = NULL;
	mlan_ds_misc_cfg *misc = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;
	int ret = 0;
	int intf_num;

	/* Restart the firmware */
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req) {
		misc = (mlan_ds_misc_cfg *)req->pbuf;
		misc->sub_command = MLAN_OID_MISC_WARM_RESET;
		misc->param.fw_reload = MTRUE;
		req->req_id = MLAN_IOCTL_MISC_CFG;
		req->action = MLAN_ACT_SET;
		status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
		if (status != MLAN_STATUS_SUCCESS) {
			PRINTM(MERROR, "warm reset failure!\n");
			ret = -EFAULT;
			if (status != MLAN_STATUS_PENDING)
				kfree(req);
			goto done;
		}
		kfree(req);
	}
#ifdef USB
	if (IS_USB(handle->card_type) && handle->params.usb_aggr == 1) {
		/* Enable USB aggregation in FW */
		if (woal_usb_aggr_init(handle)) {
			PRINTM(MERROR, "usb aggr init fail\n");
			ret = -EFAULT;
			goto done;
		}
	}
#endif

	if (moal_extflg_isset(handle, EXT_AGGR_CTRL)) {
		/* Enable aggregation in FW */
		if (woal_init_aggr_ctrl(handle, MOAL_IOCTL_WAIT)) {
			PRINTM(MERROR, "Fail to init aggr ctrl\n");
			ret = -EFAULT;
			goto done;
		}
	}

	/* Enable interfaces */
	for (intf_num = 0; intf_num < handle->priv_num; intf_num++) {
		if (handle->priv[intf_num]) {
			netif_device_attach(handle->priv[intf_num]->netdev);
			woal_start_queue(handle->priv[intf_num]->netdev);
		}
	}
done:
	LEAVE();
	return ret;
}

/**
 *  @brief Set/Get RTS threshold
 *
 *  @param priv                 A pointer to moal_private structure
 *  @param action               Action set or get
 *  @param wait_option          Wait option
 *  @param value                RTS threshold value
 *
 *  @return                     MLAN_STATUS_SUCCESS/MLAN_STATUS_PENDING --
 * success, otherwise fail
 */
mlan_status woal_set_get_rts(moal_private *priv, t_u32 action, t_u8 wait_option,
			     int *value)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_snmp_mib *mib = NULL;
	mlan_status ret = MLAN_STATUS_SUCCESS;

	ENTER();

	/* Allocate an IOCTL request buffer */
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_snmp_mib));
	if (req == NULL) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	/* Fill request buffer */
	mib = (mlan_ds_snmp_mib *)req->pbuf;
	mib->sub_command = MLAN_OID_SNMP_MIB_RTS_THRESHOLD;
	req->req_id = MLAN_IOCTL_SNMP_MIB;
	req->action = action;

	if (action == MLAN_ACT_SET) {
		if (*value < MLAN_RTS_MIN_VALUE ||
		    *value > MLAN_RTS_MAX_VALUE) {
			ret = MLAN_STATUS_FAILURE;
			goto done;
		}
		mib->param.rts_threshold = *value;
	}

	/* Send IOCTL request to MLAN */
	ret = woal_request_ioctl(priv, req, wait_option);
	if (ret == MLAN_STATUS_SUCCESS && action == MLAN_ACT_GET)
		*value = mib->param.rts_threshold;
#ifdef STA_CFG80211
	/* If set is invoked from other than iw i.e iwconfig,
	 * wiphy RTS threshold should be updated as well */
	if (IS_STA_CFG80211(priv->phandle->params.cfg80211_wext) &&
	    priv->wdev && priv->wdev->wiphy &&
	    (GET_BSS_ROLE(priv) == MLAN_BSS_ROLE_STA) &&
	    (action == MLAN_ACT_SET))
		priv->wdev->wiphy->rts_threshold = *value;
#endif

done:
	if (ret != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Set/Get Fragment threshold
 *
 *  @param priv                 A pointer to moal_private structure
 *  @param action               Action set or get
 *  @param wait_option          Wait option
 *  @param value                Fragment threshold value
 *
 *  @return                     MLAN_STATUS_SUCCESS/MLAN_STATUS_PENDING --
 * success, otherwise fail
 */
mlan_status woal_set_get_frag(moal_private *priv, t_u32 action,
			      t_u8 wait_option, int *value)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_snmp_mib *mib = NULL;
	mlan_status ret = MLAN_STATUS_SUCCESS;

	ENTER();

	/* Allocate an IOCTL request buffer */
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_snmp_mib));
	if (req == NULL) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	/* Fill request buffer */
	mib = (mlan_ds_snmp_mib *)req->pbuf;
	mib->sub_command = MLAN_OID_SNMP_MIB_FRAG_THRESHOLD;
	req->req_id = MLAN_IOCTL_SNMP_MIB;
	req->action = action;

	if (action == MLAN_ACT_SET) {
		if (*value < MLAN_FRAG_MIN_VALUE ||
		    *value > MLAN_FRAG_MAX_VALUE) {
			ret = MLAN_STATUS_FAILURE;
			goto done;
		}
		mib->param.frag_threshold = *value;
	}

	/* Send IOCTL request to MLAN */
	ret = woal_request_ioctl(priv, req, wait_option);
	if (ret == MLAN_STATUS_SUCCESS && action == MLAN_ACT_GET)
		*value = mib->param.frag_threshold;
#ifdef STA_CFG80211
	/* If set is invoked from other than iw i.e iwconfig,
	 * wiphy fragment threshold should be updated as well */
	if (IS_STA_CFG80211(priv->phandle->params.cfg80211_wext) &&
	    priv->wdev && priv->wdev->wiphy &&
	    (GET_BSS_ROLE(priv) == MLAN_BSS_ROLE_STA) &&
	    (action == MLAN_ACT_SET))
		priv->wdev->wiphy->frag_threshold = *value;
#endif

done:
	if (ret != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Set/Get TX power
 *
 *  @param priv                 A pointer to moal_private structure
 *  @param action               Action set or get
 *  @param power_cfg            A pinter to mlan_power_cfg_t structure
 *
 *  @return                     MLAN_STATUS_SUCCESS/MLAN_STATUS_PENDING --
 * success, otherwise fail
 */
mlan_status woal_set_get_tx_power(moal_private *priv, t_u32 action,
				  mlan_power_cfg_t *power_cfg)
{
	mlan_ds_power_cfg *pcfg = NULL;
	mlan_ioctl_req *req = NULL;
	mlan_status ret = MLAN_STATUS_SUCCESS;
	ENTER();

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_power_cfg));
	if (req == NULL) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	pcfg = (mlan_ds_power_cfg *)req->pbuf;
	pcfg->sub_command = MLAN_OID_POWER_CFG;
	req->req_id = MLAN_IOCTL_POWER_CFG;
	req->action = action;
	if (action == MLAN_ACT_SET && power_cfg)
		moal_memcpy_ext(priv->phandle, &pcfg->param.power_cfg,
				power_cfg, sizeof(mlan_power_cfg_t),
				sizeof(mlan_power_cfg_t));

	ret = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (ret != MLAN_STATUS_SUCCESS)
		goto done;

	if (ret == MLAN_STATUS_SUCCESS && power_cfg)
		moal_memcpy_ext(priv->phandle, power_cfg,
				&pcfg->param.power_cfg,
				sizeof(mlan_power_cfg_t),
				sizeof(mlan_power_cfg_t));

done:
	if (ret != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Set/Get IEEE power management
 *
 *  @param priv                 A pointer to moal_private structure
 *  @param action               Action set or get
 *  @param disabled             A pointer to disabled flag
 *  @param power_type           IEEE power type
 *  @param wait_option          wait option
 *
 *  @return                     MLAN_STATUS_SUCCESS -- success, otherwise fail
 */
mlan_status woal_set_get_power_mgmt(moal_private *priv, t_u32 action,
				    int *disabled, int power_type,
				    t_u8 wait_option)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_ioctl_req *req = NULL;
	mlan_ds_pm_cfg *pm_cfg = NULL;

	ENTER();

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_pm_cfg));
	if (req == NULL) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	pm_cfg = (mlan_ds_pm_cfg *)req->pbuf;
	pm_cfg->sub_command = MLAN_OID_PM_CFG_IEEE_PS;
	req->req_id = MLAN_IOCTL_PM_CFG;
	req->action = action;

	if (action == MLAN_ACT_SET) {
		PRINTM(MINFO, "PS_MODE set power disabled=%d power type=%#x\n",
		       *disabled, power_type);
		if (*disabled)
			pm_cfg->param.ps_mode = 0;
		else {
			/* Check not support case only (vwrq->disabled == FALSE)
			 */
			if ((power_type & MW_POWER_TYPE) == MW_POWER_TIMEOUT) {
				PRINTM(MERROR,
				       "Setting power timeout is not supported\n");
				ret = MLAN_STATUS_FAILURE;
				goto done;
			} else if ((power_type & MW_POWER_TYPE) ==
				   MW_POWER_PERIOD) {
				PRINTM(MERROR,
				       "Setting power period is not supported\n");
				ret = MLAN_STATUS_FAILURE;
				goto done;
			}
			pm_cfg->param.ps_mode = 1;
		}
	}

	ret = woal_request_ioctl(priv, req, wait_option);
	if (ret != MLAN_STATUS_SUCCESS)
		goto done;

	if (ret == MLAN_STATUS_SUCCESS && action == MLAN_ACT_GET)
		*disabled = pm_cfg->param.ps_mode;

#ifdef STA_CFG80211
	/* If set is invoked from other than iw i.e iwconfig,
	 * wiphy IEEE power save mode should be updated */
	if (IS_STA_CFG80211(priv->phandle->params.cfg80211_wext) &&
	    priv->wdev && (GET_BSS_ROLE(priv) == MLAN_BSS_ROLE_STA) &&
	    (action == MLAN_ACT_SET)) {
		if (*disabled)
			priv->wdev->ps = MFALSE;
		else
			priv->wdev->ps = MTRUE;
	}
#endif

done:
	if (ret != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 * @brief Set Country Code
 *
 * @param priv     A pointer to moal_private structure
 * @param region   A pointer to region string
 *
 * @return         MLAN_STATUS_SUCCESS/MLAN_STATUS_PENDING --success, otherwise
 * fail
 */
static int woal_set_countrycode(moal_private *priv, char *country)
{
	int ret = 0;
	mlan_ioctl_req *req = NULL;
	mlan_ds_misc_cfg *pcfg_misc = NULL;
	mlan_ds_misc_country_code *country_code = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	/* Allocate an IOCTL request buffer */
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	/* Fill request buffer */
	pcfg_misc = (mlan_ds_misc_cfg *)req->pbuf;
	country_code = &pcfg_misc->param.country_code;
	pcfg_misc->sub_command = MLAN_OID_MISC_COUNTRY_CODE;
	req->req_id = MLAN_IOCTL_MISC_CFG;

	memset(country_code->country_code, 0, COUNTRY_CODE_LEN);
	moal_memcpy_ext(priv->phandle, country_code->country_code, country,
			COUNTRY_CODE_LEN - 1, COUNTRY_CODE_LEN - 1);
	req->action = MLAN_ACT_SET;

	/* Send IOCTL request to MLAN */
	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);

	LEAVE();
	return ret;
}

/**
 * @brief Set region code
 *
 * @param priv     A pointer to moal_private structure
 * @param region   A pointer to region string
 *
 * @return         MLAN_STATUS_SUCCESS/MLAN_STATUS_PENDING --success, otherwise
 * fail
 */
mlan_status woal_set_region_code(moal_private *priv, char *region)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_ds_misc_cfg *cfg = NULL;
	mlan_ioctl_req *req = NULL;

	ENTER();
	if (woal_is_country_code_supported(region))
		return woal_set_countrycode(priv, region);

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	cfg = (mlan_ds_misc_cfg *)req->pbuf;
	cfg->sub_command = MLAN_OID_MISC_REGION;
	req->req_id = MLAN_IOCTL_MISC_CFG;
	req->action = MLAN_ACT_SET;
	cfg->param.region_code = region_string_2_region_code(region);
	ret = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);

done:
	if (ret != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Set/Get data rate
 *
 *  @param priv                 A pointer to moal_private structure
 *  @param action               Action set or get
 *  @param datarate             A pointer to mlan_rate_cfg_t structure
 *
 *  @return                     MLAN_STATUS_SUCCESS -- success, otherwise fail
 */
mlan_status woal_set_get_data_rate(moal_private *priv, t_u8 action,
				   mlan_rate_cfg_t *datarate)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_ds_rate *rate = NULL;
	mlan_ioctl_req *req = NULL;

	ENTER();

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_rate));
	if (req == NULL) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	rate = (mlan_ds_rate *)req->pbuf;
	rate->param.rate_cfg.rate_type = MLAN_RATE_VALUE;
	rate->sub_command = MLAN_OID_RATE_CFG;
	req->req_id = MLAN_IOCTL_RATE;
	req->action = action;

	if (datarate && (action == MLAN_ACT_SET))
		moal_memcpy_ext(priv->phandle, &rate->param.rate_cfg, datarate,
				sizeof(mlan_rate_cfg_t),
				sizeof(mlan_rate_cfg_t));

	ret = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (ret == MLAN_STATUS_SUCCESS && datarate && action == MLAN_ACT_GET)
		moal_memcpy_ext(priv->phandle, datarate, &rate->param.rate_cfg,
				sizeof(mlan_rate_cfg_t),
				sizeof(mlan_rate_cfg_t));

done:
	if (ret != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Get assoc_resp buffer
 *
 *  @param priv                 A pointer to moal_private structure
 *  @param assoc_rsp            A pointer to mlan_ds_misc_assoc_rsp structure
 *  @param wait_option          wait option
 *
 *  @return                     MLAN_STATUS_SUCCESS -- success, otherwise fail
 */
mlan_status woal_get_assoc_rsp(moal_private *priv,
			       mlan_ds_misc_assoc_rsp *assoc_rsp,
			       t_u8 wait_option)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_ds_misc_cfg *misc = NULL;
	mlan_ioctl_req *req = NULL;

	ENTER();

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		PRINTM(MERROR, "Fail to allocate buffer for get assoc resp\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	req->req_id = MLAN_IOCTL_MISC_CFG;
	misc = (pmlan_ds_misc_cfg)req->pbuf;
	misc->sub_command = MLAN_OID_MISC_ASSOC_RSP;
	req->action = MLAN_ACT_GET;

	ret = woal_request_ioctl(priv, req, wait_option);
	if (ret == MLAN_STATUS_SUCCESS && assoc_rsp)
		moal_memcpy_ext(priv->phandle, assoc_rsp,
				&misc->param.assoc_resp,
				sizeof(mlan_ds_misc_assoc_rsp),
				sizeof(mlan_ds_misc_assoc_rsp));

done:
	if (ret != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Get assoc_req IEs buffer
 *
 *  @param priv                 A pointer to moal_private structure
 *  @param assoc_rsp            A pointer to mlan_ds_misc_assoc_rsp structure
 *  @param wait_option          wait option
 *
 *  @return                     MLAN_STATUS_SUCCESS -- success, otherwise fail
 */
mlan_status woal_get_assoc_req(moal_private *priv,
			       mlan_ds_misc_assoc_req *assoc_req,
			       t_u8 wait_option)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_ds_misc_cfg *misc = NULL;
	mlan_ioctl_req *req = NULL;

	ENTER();

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		PRINTM(MERROR,
		       "Fail to allocate buffer for get assoc request\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	req->req_id = MLAN_IOCTL_MISC_CFG;
	misc = (pmlan_ds_misc_cfg)req->pbuf;
	misc->sub_command = MLAN_OID_MISC_ASSOC_REQ;
	req->action = MLAN_ACT_GET;

	ret = woal_request_ioctl(priv, req, wait_option);
	if (ret == MLAN_STATUS_SUCCESS && assoc_req)
		moal_memcpy_ext(priv->phandle, assoc_req,
				&misc->param.assoc_req,
				sizeof(mlan_ds_misc_assoc_req),
				sizeof(mlan_ds_misc_assoc_req));

done:
	if (ret != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Get previous AP's assoc_info
 *
 *  @param priv                 A pointer to moal_private structure
 *  @param assoc_info            A pointer to mlan_ds_assoc_info structure
 *  @param wait_option          wait option
 *
 *  @return                     MLAN_STATUS_SUCCESS -- success, otherwise fail
 */
mlan_status woal_get_prev_assoc_info(moal_private *priv,
				     mlan_ds_assoc_info *assoc_info,
				     t_u8 wait_option)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_ds_misc_cfg *misc = NULL;
	mlan_ioctl_req *req = NULL;

	ENTER();

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		PRINTM(MERROR,
		       "Fail to allocate buffer for get assoc info request\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	req->req_id = MLAN_IOCTL_MISC_CFG;
	misc = (pmlan_ds_misc_cfg)req->pbuf;
	misc->sub_command = MLAN_OID_MISC_PREV_ASSOC_INFO;
	req->action = MLAN_ACT_GET;

	ret = woal_request_ioctl(priv, req, wait_option);
	if (ret == MLAN_STATUS_SUCCESS && assoc_info)
		moal_memcpy_ext(priv->phandle, assoc_info,
				&misc->param.assoc_info,
				sizeof(mlan_ds_assoc_info),
				sizeof(mlan_ds_assoc_info));

done:
	if (ret != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}
#endif

/**
 *  @brief Send get FW info request to MLAN
 *
 *  @param priv             A pointer to moal_private structure
 *  @param wait_option      Wait option
 *  @param fw_info          FW information
 *
 *  @return                 MLAN_STATUS_SUCCESS/MLAN_STATUS_PENDING -- success,
 * otherwise fail
 */
mlan_status woal_request_get_fw_info(moal_private *priv, t_u8 wait_option,
				     mlan_fw_info *fw_info)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_get_info *info;
	mlan_status status;
	ENTER();
	memset(priv->current_addr, 0xff, ETH_ALEN);

	/* Allocate an IOCTL request buffer */
	req = (mlan_ioctl_req *)woal_alloc_mlan_ioctl_req(
		sizeof(mlan_ds_get_info));
	if (req == NULL) {
		status = MLAN_STATUS_FAILURE;
		goto done;
	}

	/* Fill request buffer */
	info = (mlan_ds_get_info *)req->pbuf;
	req->req_id = MLAN_IOCTL_GET_INFO;
	req->action = MLAN_ACT_GET;
	info->sub_command = MLAN_OID_GET_FW_INFO;

	/* Send IOCTL request to MLAN */
	status = woal_request_ioctl(priv, req, wait_option);
	if (status == MLAN_STATUS_SUCCESS) {
		priv->phandle->fw_release_number = info->param.fw_info.fw_ver;
		priv->phandle->fw_hotfix_version =
			info->param.fw_info.hotfix_version;
		priv->phandle->fw_ecsa_enable = info->param.fw_info.ecsa_enable;
		priv->phandle->fw_bands = info->param.fw_info.fw_bands;
		priv->phandle->cmd_tx_data = info->param.fw_info.cmd_tx_data;
		priv->phandle->sec_rgpower = info->param.fw_info.sec_rgpower;
		priv->phandle->fw_getlog_enable =
			info->param.fw_info.getlog_enable;
		priv->phandle->fw_roaming_support =
			info->param.fw_info.fw_roaming_support;
		if (priv->current_addr[0] == 0xff)
			moal_memcpy_ext(priv->phandle, priv->current_addr,
					&info->param.fw_info.mac_addr,
					sizeof(mlan_802_11_mac_addr), ETH_ALEN);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 16, 0)
		eth_hw_addr_set(priv->netdev, priv->current_addr);
#else
		moal_memcpy_ext(priv->phandle, priv->netdev->dev_addr,
				priv->current_addr, ETH_ALEN, ETH_ALEN);
#endif
		if (fw_info)
			moal_memcpy_ext(priv->phandle, fw_info,
					&info->param.fw_info,
					sizeof(mlan_fw_info),
					sizeof(mlan_fw_info));
		DBG_HEXDUMP(MCMD_D, "mac", priv->current_addr, 6);
	} else if (status != MLAN_STATUS_PENDING)
		PRINTM(MERROR,
		       "get fw info failed! status=%d, error_code=0x%x\n",
		       status, req->status_code);
	else
		PRINTM(MERROR, "get fw info failed! status=%d", status);
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return status;
}

/**
 *  @brief Get current channel of active interface
 *
 *  @param priv        A pointer to moal_private
 *  @param channel 	   A pointer to chan_band_info structure
 *
 *  @return            MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status woal_get_active_intf_channel(moal_private *priv,
					 chan_band_info *channel)
{
	moal_handle *handle = priv->phandle;
	int i;
	for (i = 0; i < handle->priv_num; i++) {
		if (!handle->priv[i])
			continue;
#ifdef STA_SUPPORT
		if (GET_BSS_ROLE(handle->priv[i]) == MLAN_BSS_ROLE_STA) {
			if (handle->priv[i]->media_connected == MTRUE)
				return woal_get_sta_channel(handle->priv[i],
							    MOAL_IOCTL_WAIT,
							    channel);
		}
#endif
#ifdef UAP_SUPPORT
		if (GET_BSS_ROLE(handle->priv[i]) == MLAN_BSS_ROLE_UAP) {
			if (handle->priv[i]->bss_started == MTRUE)
				return woal_set_get_ap_channel(handle->priv[i],
							       MLAN_ACT_GET,
							       MOAL_IOCTL_WAIT,
							       channel);
		}
#endif
	}
	return MLAN_STATUS_FAILURE;
}

#ifdef STA_SUPPORT
/**
 *  @brief Send get ext cap info request to MLAN
 *
 *  @param priv             A pointer to moal_private structure
 *  @param buf              data buffer
 *  @param len              data buffer length
 *
 *  @return                 number of bytes of extended capability -- success,
 * otherwise error
 */
int woal_request_extcap(moal_private *priv, t_u8 *buf, t_u8 len)
{
	int ret = 0;
	mlan_ioctl_req *req = NULL;
	mlan_ds_misc_cfg *cfg = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();
	if (buf == NULL || len <= 0 || len < sizeof(ExtCap_t)) {
		ret = -EINVAL;
		goto out;
	}
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto out;
	}
	cfg = (mlan_ds_misc_cfg *)req->pbuf;
	cfg->sub_command = MLAN_OID_MISC_EXT_CAP_CFG;
	req->req_id = MLAN_IOCTL_MISC_CFG;
	req->action = MLAN_ACT_GET;
	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto out;
	}
	memset(buf, 0, len);
	moal_memcpy_ext(priv->phandle, buf, &cfg->param.ext_cap,
			sizeof(ExtCap_t), len);
	ret = sizeof(ExtCap_t);
out:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}
#endif

/**
 *  @brief Get debug info
 *
 *  @param priv                 A pointer to moal_private structure
 *  @param wait_option          Wait option
 *  @param debug_info           A pointer to mlan_debug_info structure
 *
 *  @return                     MLAN_STATUS_SUCCESS/MLAN_STATUS_PENDING --
 * success, otherwise fail
 */
mlan_status woal_get_debug_info(moal_private *priv, t_u8 wait_option,
				mlan_debug_info *debug_info)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_get_info *info = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;
	ENTER();

	/* Allocate an IOCTL request buffer */
	req = woal_alloc_mlan_ioctl_req(sizeof(t_u32) +
					sizeof(mlan_debug_info));
	if (req == NULL) {
		status = MLAN_STATUS_FAILURE;
		goto done;
	}

	/* Fill request buffer */
	info = (mlan_ds_get_info *)req->pbuf;
	info->sub_command = MLAN_OID_GET_DEBUG_INFO;
	req->req_id = MLAN_IOCTL_GET_INFO;
	req->action = MLAN_ACT_GET;

	/* Send IOCTL request to MLAN */
	status = woal_request_ioctl(priv, req, wait_option);
	if (status == MLAN_STATUS_SUCCESS) {
		if (debug_info) {
			moal_memcpy_ext(priv->phandle, debug_info,
					&info->param.debug_info,
					sizeof(mlan_debug_info),
					sizeof(mlan_debug_info));
		}
	}
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return status;
}

#if defined(STA_WEXT) || defined(UAP_WEXT)
/**
 *  @brief host command ioctl function
 *
 *  @param priv     A pointer to moal_private structure
 *  @param wrq      A pointer to iwreq structure
 *  @return         0 --success, otherwise fail
 */
int woal_host_command(moal_private *priv, struct iwreq *wrq)
{
	HostCmd_Header cmd_header;
	int ret = 0;
	mlan_ds_misc_cfg *misc = NULL;
	mlan_ioctl_req *req = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	/* Sanity check */
	if (wrq->u.data.pointer == NULL) {
		PRINTM(MERROR, "hostcmd IOCTL corrupt data\n");
		ret = -EINVAL;
		goto done;
	}
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}
	misc = (mlan_ds_misc_cfg *)req->pbuf;
	memset(&cmd_header, 0, sizeof(cmd_header));

	/* get command header */
	if (copy_from_user(&cmd_header, wrq->u.data.pointer,
			   sizeof(HostCmd_Header))) {
		PRINTM(MERROR, "copy from user failed: Host command header\n");
		ret = -EFAULT;
		goto done;
	}
	misc->param.hostcmd.len = woal_le16_to_cpu(cmd_header.size);

	PRINTM(MINFO, "Host command len = %u\n", misc->param.hostcmd.len);

	if (!misc->param.hostcmd.len ||
	    misc->param.hostcmd.len > MRVDRV_SIZE_OF_CMD_BUFFER) {
		PRINTM(MERROR, "Invalid data buffer length\n");
		ret = -EINVAL;
		goto done;
	}

	/* get the whole command from user */
	if (copy_from_user(misc->param.hostcmd.cmd, wrq->u.data.pointer,
			   woal_le16_to_cpu(cmd_header.size))) {
		PRINTM(MERROR, "copy from user failed\n");
		ret = -EFAULT;
		goto done;
	}
	misc->sub_command = MLAN_OID_MISC_HOST_CMD;
	req->req_id = MLAN_IOCTL_MISC_CFG;

	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}
	if (copy_to_user(wrq->u.data.pointer, (t_u8 *)misc->param.hostcmd.cmd,
			 misc->param.hostcmd.len)) {
		ret = -EFAULT;
		goto done;
	}
	wrq->u.data.length = misc->param.hostcmd.len;
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}
#endif

#if defined(WIFI_DIRECT_SUPPORT) || defined(UAP_SUPPORT)
/**
 *  @brief host command ioctl function
 *
 *  @param dev      A pointer to net_device structure
 *  @param req      A pointer to ifreq structure
 *  @return         0 --success, otherwise fail
 */
/*********  format of ifr_data *************/
/*    buf_len + Hostcmd_body               */
/*    buf_len: 4 bytes                     */
/*             the length of the buf which */
/*             can be used to return data  */
/*             to application              */
/*    Hostcmd_body                         */
/*******************************************/
int woal_hostcmd_ioctl(struct net_device *dev, struct ifreq *req)
{
	moal_private *priv = (moal_private *)netdev_priv(dev);
	t_u32 buf_len = 0;
	HostCmd_Header cmd_header;
	int ret = 0;
	mlan_ds_misc_cfg *misc = NULL;
	mlan_ioctl_req *ioctl_req = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	/* Sanity check */
	if (req->ifr_data == NULL) {
		PRINTM(MERROR, "uap_hostcmd_ioctl() corrupt data\n");
		ret = -EFAULT;
		goto done;
	}

	if (copy_from_user(&buf_len, req->ifr_data, sizeof(buf_len))) {
		PRINTM(MERROR, "Copy from user failed\n");
		ret = -EFAULT;
		goto done;
	}

	memset(&cmd_header, 0, sizeof(cmd_header));

	/* get command header */
	if (copy_from_user(&cmd_header,
			   (void *)((t_u8 *)(req->ifr_data) + sizeof(buf_len)),
			   sizeof(HostCmd_Header))) {
		PRINTM(MERROR, "Copy from user failed\n");
		ret = -EFAULT;
		goto done;
	}

	PRINTM(MINFO, "Host command len = %d\n",
	       woal_le16_to_cpu(cmd_header.size));

	if (woal_le16_to_cpu(cmd_header.size) > MRVDRV_SIZE_OF_CMD_BUFFER) {
		ret = -EINVAL;
		goto done;
	}

	ioctl_req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (ioctl_req == NULL) {
		ret = -ENOMEM;
		goto done;
	}
	misc = (mlan_ds_misc_cfg *)ioctl_req->pbuf;

	misc->param.hostcmd.len = woal_le16_to_cpu(cmd_header.size);

	/* get the whole command from user */
	if (copy_from_user(misc->param.hostcmd.cmd,
			   (void *)((t_u8 *)(req->ifr_data) + sizeof(buf_len)),
			   misc->param.hostcmd.len)) {
		PRINTM(MERROR, "copy from user failed\n");
		ret = -EFAULT;
		goto done;
	}
	misc->sub_command = MLAN_OID_MISC_HOST_CMD;
	ioctl_req->req_id = MLAN_IOCTL_MISC_CFG;

	status = woal_request_ioctl(priv, ioctl_req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}
	if (misc->param.hostcmd.len > buf_len) {
		PRINTM(MERROR,
		       "buf_len is too small, resp_len=%d, buf_len=%d\n",
		       (int)misc->param.hostcmd.len, (int)buf_len);
		ret = -EFAULT;
		goto done;
	}
	if (copy_to_user((void *)((t_u8 *)(req->ifr_data) + sizeof(buf_len)),
			 (t_u8 *)misc->param.hostcmd.cmd,
			 misc->param.hostcmd.len)) {
		ret = -EFAULT;
		goto done;
	}
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(ioctl_req);
	LEAVE();
	return ret;
}
#endif

/**
 *  @brief CUSTOM_IE ioctl handler
 *
 *  @param dev      A pointer to net_device structure
 *  @param req      A pointer to ifreq structure
 *  @return         0 --success, otherwise fail
 */
int woal_custom_ie_ioctl(struct net_device *dev, struct ifreq *req)
{
	moal_private *priv = (moal_private *)netdev_priv(dev);
	mlan_ioctl_req *ioctl_req = NULL;
	mlan_ds_misc_cfg *misc = NULL;
	mlan_ds_misc_custom_ie *pcustom_ie = NULL;
	int ret = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;
	gfp_t flag;

	ENTER();

	/* Sanity check */
	if (req->ifr_data == NULL) {
		PRINTM(MERROR, "woal_custom_ie_ioctl() corrupt data\n");
		ret = -EFAULT;
		goto done;
	}

	flag = (in_atomic() || irqs_disabled()) ? GFP_ATOMIC : GFP_KERNEL;
	pcustom_ie = kzalloc(sizeof(mlan_ds_misc_custom_ie), flag);
	if (!pcustom_ie) {
		ret = -ENOMEM;
		goto done;
	}

	if (copy_from_user(pcustom_ie, req->ifr_data,
			   sizeof(mlan_ds_misc_custom_ie))) {
		PRINTM(MERROR, "Copy from user failed\n");
		ret = -EFAULT;
		goto done;
	}

	ioctl_req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (ioctl_req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	misc = (mlan_ds_misc_cfg *)ioctl_req->pbuf;
	misc->sub_command = MLAN_OID_MISC_CUSTOM_IE;
	ioctl_req->req_id = MLAN_IOCTL_MISC_CFG;
	if ((pcustom_ie->len == 0) ||
	    (pcustom_ie->len == sizeof(pcustom_ie->ie_data_list[0].ie_index)))
		ioctl_req->action = MLAN_ACT_GET;
	else
		ioctl_req->action = MLAN_ACT_SET;

	moal_memcpy_ext(priv->phandle, &misc->param.cust_ie, pcustom_ie,
			sizeof(mlan_ds_misc_custom_ie),
			sizeof(mlan_ds_misc_custom_ie));

	status = woal_request_ioctl(priv, ioctl_req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	if (ioctl_req->action == MLAN_ACT_GET) {
		if (copy_to_user(req->ifr_data, &misc->param.cust_ie,
				 sizeof(mlan_ds_misc_custom_ie))) {
			PRINTM(MERROR, "Copy to user failed!\n");
			ret = -EFAULT;
			goto done;
		}
	} else if (ioctl_req->status_code == MLAN_ERROR_IOCTL_FAIL) {
		/* send a separate error code to indicate error from driver */
		ret = EFAULT;
	}

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(ioctl_req);
	kfree(pcustom_ie);
	LEAVE();
	return ret;
}

/**
 *  @brief send mgmt packet through ioctl
 *
 *  @param priv     A pointer to moal_private structure
 *  @param pmbuf    A pointer to mlan_buffer which hold mgmt packet
 *  @return         N/A
 */
static mlan_status woal_send_mgmt_packet(moal_private *priv, pmlan_buffer pmbuf)
{
	mlan_ioctl_req *ioctl_req = NULL;
	mlan_ds_misc_cfg *misc = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;
	ENTER();
	ioctl_req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (ioctl_req == NULL)
		goto done;
	misc = (mlan_ds_misc_cfg *)ioctl_req->pbuf;
	misc->sub_command = MLAN_OID_MISC_TX_FRAME;
	ioctl_req->req_id = MLAN_IOCTL_MISC_CFG;
	ioctl_req->action = MLAN_ACT_SET;
	misc->param.tx_frame.data_len = pmbuf->data_len;
	misc->param.tx_frame.buf_type = MLAN_BUF_TYPE_RAW_DATA;
	misc->param.tx_frame.priority = 7;
	if (pmbuf->tx_seq_num)
		misc->param.tx_frame.flags = MLAN_BUF_FLAG_TX_STATUS;
	misc->param.tx_frame.tx_seq_num = pmbuf->tx_seq_num;
	moal_memcpy_ext(priv->phandle, misc->param.tx_frame.tx_buf,
			pmbuf->pbuf + pmbuf->data_offset, pmbuf->data_len,
			MRVDRV_SIZE_OF_CMD_BUFFER);
	status = woal_request_ioctl(priv, ioctl_req, MOAL_IOCTL_WAIT);
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(ioctl_req);
	LEAVE();
	return status;
}

/**
 *  @brief function to send raw data packet from monitor mode interface
 *
 *  @param dev      A pointer to net_device structure
 *  @param req      A pointer to ifreq structure
 *  @return         0 --success, otherwise fail
 */
int woal_send_mon_if_packet(struct net_device *dev, struct ifreq *req)
{
	moal_private *priv = (moal_private *)netdev_priv(dev);
	t_u32 packet_len = 0;
	int ret = 0;
	pmlan_buffer pmbuf = NULL;
	mlan_status status;

	ENTER();

	if (!priv || !priv->phandle) {
		PRINTM(MERROR, "priv or handle is NULL\n");
		ret = -EFAULT;
		goto done;
	}

	/* Sanity check */
	if (req->ifr_data == NULL) {
		PRINTM(MERROR, "woal_send_mon_if_packet() corrupt data\n");
		ret = -EFAULT;
		goto done;
	}

	moal_memcpy_ext(NULL, &packet_len, req->ifr_data, sizeof(packet_len),
			sizeof(packet_len));
#define PACKET_HEADER_LEN 8
#define FRAME_LEN 2
#define MV_ETH_FRAME_LEN 1514
	if (packet_len > MV_ETH_FRAME_LEN) {
		PRINTM(MERROR, "Invalid packet length %d\n", packet_len);
		ret = -EFAULT;
		goto done;
	}
	pmbuf = woal_alloc_mlan_buffer(
		priv->phandle, (int)(MLAN_MIN_DATA_HEADER_LEN +
				     (int)packet_len + PACKET_HEADER_LEN));
	if (!pmbuf) {
		PRINTM(MERROR, "Fail to allocate mlan_buffer\n");
		ret = -ENOMEM;
		goto done;
	}
	pmbuf->data_offset = MLAN_MIN_DATA_HEADER_LEN;

	moal_memcpy_ext(NULL, pmbuf->pbuf + pmbuf->data_offset,
			(void *)((t_u8 *)(req->ifr_data) + sizeof(packet_len)),
			PACKET_HEADER_LEN + packet_len,
			PACKET_HEADER_LEN + packet_len);

	pmbuf->data_len = PACKET_HEADER_LEN + packet_len;
	pmbuf->buf_type = MLAN_BUF_TYPE_RAW_DATA;
	pmbuf->bss_index = priv->bss_index;

	status = mlan_send_packet(priv->phandle->pmlan_adapter, pmbuf);
	switch (status) {
	case MLAN_STATUS_PENDING:
		atomic_inc(&priv->phandle->tx_pending);
		queue_work(priv->phandle->workqueue, &priv->phandle->main_work);
		break;
	case MLAN_STATUS_SUCCESS:
		woal_free_mlan_buffer(priv->phandle, pmbuf);
		break;
	case MLAN_STATUS_FAILURE:
	default:
		woal_free_mlan_buffer(priv->phandle, pmbuf);
		ret = -EFAULT;
		break;
	}
done:
	LEAVE();
	return ret;
}

/**
 *  @brief send raw data packet ioctl function
 *
 *  @param dev      A pointer to net_device structure
 *  @param req      A pointer to ifreq structure
 *  @return         0 --success, otherwise fail
 */
int woal_send_host_packet(struct net_device *dev, struct ifreq *req)
{
	moal_private *priv = (moal_private *)netdev_priv(dev);
	t_u32 packet_len = 0;
	int ret = 0;
	pmlan_buffer pmbuf = NULL;
	IEEE80211_MGMT *mgmt = NULL;
	mlan_status status;

	ENTER();

	if (!priv || !priv->phandle) {
		PRINTM(MERROR, "priv or handle is NULL\n");
		ret = -EFAULT;
		goto done;
	}

	/* Sanity check */
	if (req->ifr_data == NULL) {
		PRINTM(MERROR, "woal_send_host_packet() corrupt data\n");
		ret = -EFAULT;
		goto done;
	}

	if (copy_from_user(&packet_len, req->ifr_data, sizeof(packet_len))) {
		PRINTM(MERROR, "Copy from user failed\n");
		ret = -EFAULT;
		goto done;
	}
#define PACKET_HEADER_LEN 8
#define FRAME_LEN 2
#define MV_ETH_FRAME_LEN 1514
	if (packet_len > MV_ETH_FRAME_LEN) {
		PRINTM(MERROR, "Invalid packet length %d\n", packet_len);
		ret = -EFAULT;
		goto done;
	}
	pmbuf = woal_alloc_mlan_buffer(
		priv->phandle, (int)(MLAN_MIN_DATA_HEADER_LEN +
				     (int)packet_len + PACKET_HEADER_LEN));
	if (!pmbuf) {
		PRINTM(MERROR, "Fail to allocate mlan_buffer\n");
		ret = -ENOMEM;
		goto done;
	}
	pmbuf->data_offset = MLAN_MIN_DATA_HEADER_LEN;

	/* get whole packet and header */
	if (copy_from_user(
		    pmbuf->pbuf + pmbuf->data_offset,
		    (void *)((t_u8 *)(req->ifr_data) + sizeof(packet_len)),
		    PACKET_HEADER_LEN + packet_len)) {
		PRINTM(MERROR, "Copy from user failed\n");
		ret = -EFAULT;
		woal_free_mlan_buffer(priv->phandle, pmbuf);
		goto done;
	}
	pmbuf->data_len = PACKET_HEADER_LEN + packet_len;
	pmbuf->buf_type = MLAN_BUF_TYPE_RAW_DATA;
	pmbuf->bss_index = priv->bss_index;

	mgmt = (IEEE80211_MGMT *)(pmbuf->pbuf + pmbuf->data_offset +
				  PACKET_HEADER_LEN + FRAME_LEN);

	if (priv->phandle->cmd_tx_data &&
	    ((mgmt->frame_control & IEEE80211_FC_MGMT_FRAME_TYPE_MASK) == 0)) {
		status = woal_send_mgmt_packet(priv, pmbuf);
		woal_free_mlan_buffer(priv->phandle, pmbuf);
		goto done;
	}
	status = mlan_send_packet(priv->phandle->pmlan_adapter, pmbuf);
	switch (status) {
	case MLAN_STATUS_PENDING:
		atomic_inc(&priv->phandle->tx_pending);
		queue_work(priv->phandle->workqueue, &priv->phandle->main_work);
		break;
	case MLAN_STATUS_SUCCESS:
		woal_free_mlan_buffer(priv->phandle, pmbuf);
		break;
	case MLAN_STATUS_FAILURE:
	default:
		woal_free_mlan_buffer(priv->phandle, pmbuf);
		ret = -EFAULT;
		break;
	}
done:
	LEAVE();
	return ret;
}

#if defined(UAP_WEXT)
/**
 *  @brief Set/Get CUSTOM_IE ioctl handler
 *
 *  @param priv         A pointer to moal_private structure
 *  @param mask         Mask to set or clear from caller
 *  @param ie           IE buffer to set for beacon
 *  @param ie_len       Length of the IE
 *
 *  @return         0 --success, otherwise fail
 */
int woal_set_get_custom_ie(moal_private *priv, t_u16 mask, t_u8 *ie, int ie_len)
{
	mlan_ioctl_req *ioctl_req = NULL;
	mlan_ds_misc_cfg *misc = NULL;
	mlan_ds_misc_custom_ie *misc_ie = NULL;
	int ret = 0;
	custom_ie *pcust_bcn_ie = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	ioctl_req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (ioctl_req == NULL) {
		LEAVE();
		return -ENOMEM;
	}

	misc = (mlan_ds_misc_cfg *)ioctl_req->pbuf;
	misc->sub_command = MLAN_OID_MISC_CUSTOM_IE;
	ioctl_req->req_id = MLAN_IOCTL_MISC_CFG;
	ioctl_req->action = MLAN_ACT_SET;
	misc_ie = &misc->param.cust_ie;

	misc_ie->type = TLV_TYPE_MGMT_IE;
	misc_ie->len = (sizeof(custom_ie) - MAX_IE_SIZE) + ie_len;
	pcust_bcn_ie = misc_ie->ie_data_list;
	pcust_bcn_ie->ie_index = 0xffff;
	pcust_bcn_ie->mgmt_subtype_mask = mask;
	pcust_bcn_ie->ie_length = ie_len;
	moal_memcpy_ext(priv->phandle, pcust_bcn_ie->ie_buffer, ie, ie_len,
			MAX_IE_SIZE);

	status = woal_request_ioctl(priv, ioctl_req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS)
		ret = -EFAULT;

	if (status != MLAN_STATUS_PENDING)
		kfree(ioctl_req);
	LEAVE();
	return ret;
}
#endif /* defined(HOST_TXRX_MGMT_FRAME) && defined(UAP_WEXT) */

/**
 *  @brief TDLS configuration ioctl handler
 *
 *  @param dev      A pointer to net_device structure
 *  @param req      A pointer to ifreq structure
 *  @return         0 --success, otherwise fail
 */
int woal_tdls_config_ioctl(struct net_device *dev, struct ifreq *req)
{
	moal_private *priv = (moal_private *)netdev_priv(dev);
	mlan_ioctl_req *ioctl_req = NULL;
	mlan_ds_misc_cfg *misc = NULL;
	mlan_ds_misc_tdls_config *tdls_data = NULL;
	int ret = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;
	gfp_t flag;

	ENTER();

	/* Sanity check */
	if (req->ifr_data == NULL) {
		PRINTM(MERROR, "woal_tdls_config_ioctl() corrupt data\n");
		ret = -EFAULT;
		goto done;
	}
	flag = (in_atomic() || irqs_disabled()) ? GFP_ATOMIC : GFP_KERNEL;
	tdls_data = kzalloc(sizeof(mlan_ds_misc_tdls_config), flag);
	if (!tdls_data) {
		ret = -ENOMEM;
		goto done;
	}

	if (copy_from_user(tdls_data, req->ifr_data,
			   sizeof(mlan_ds_misc_tdls_config))) {
		PRINTM(MERROR, "Copy from user failed\n");
		ret = -EFAULT;
		goto done;
	}

	ioctl_req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (ioctl_req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	misc = (mlan_ds_misc_cfg *)ioctl_req->pbuf;
	misc->sub_command = MLAN_OID_MISC_TDLS_CONFIG;
	ioctl_req->req_id = MLAN_IOCTL_MISC_CFG;
	if (tdls_data->tdls_action == WLAN_TDLS_DISCOVERY_REQ ||
	    tdls_data->tdls_action == WLAN_TDLS_LINK_STATUS)
		ioctl_req->action = MLAN_ACT_GET;
	else
		ioctl_req->action = MLAN_ACT_SET;

	moal_memcpy_ext(priv->phandle, &misc->param.tdls_config, tdls_data,
			sizeof(mlan_ds_misc_tdls_config),
			sizeof(mlan_ds_misc_tdls_config));

	status = woal_request_ioctl(priv, ioctl_req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	if (tdls_data->tdls_action == WLAN_TDLS_DISCOVERY_REQ ||
	    tdls_data->tdls_action == WLAN_TDLS_LINK_STATUS) {
		if (copy_to_user(req->ifr_data, &misc->param.tdls_config,
				 sizeof(mlan_ds_misc_tdls_config))) {
			PRINTM(MERROR, "Copy to user failed!\n");
			ret = -EFAULT;
			goto done;
		}
	}

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(ioctl_req);
	kfree(tdls_data);
	LEAVE();
	return ret;
}

/**
 *  @brief ioctl function get BSS type
 *
 *  @param dev      A pointer to net_device structure
 *  @param req      A pointer to ifreq structure
 *  @return         0 --success, otherwise fail
 */
int woal_get_bss_type(struct net_device *dev, struct ifreq *req)
{
	int ret = 0;
	moal_private *priv = (moal_private *)netdev_priv(dev);
	int bss_type;

	ENTER();

	bss_type = (int)priv->bss_type;
	if (copy_to_user(req->ifr_data, &bss_type, sizeof(int))) {
		PRINTM(MINFO, "Copy to user failed!\n");
		ret = -EFAULT;
	}

	LEAVE();
	return ret;
}

#if defined(STA_SUPPORT) && defined(UAP_SUPPORT)
/**
 * @brief Swithces BSS role of interface
 *
 * @param priv          A pointer to moal_private structure
 * @param action        Action: set or get
 * @param wait_option   Wait option (MOAL_WAIT or MOAL_NO_WAIT)
 * @param bss_role      A pointer to bss role
 *
 * @return         0 --success, otherwise fail
 */
mlan_status woal_bss_role_cfg(moal_private *priv, t_u8 action, t_u8 wait_option,
			      t_u8 *bss_role)
{
	int ret = 0;
	mlan_ds_bss *bss = NULL;
	mlan_ioctl_req *req = NULL;
	struct net_device *dev = priv->netdev;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_bss));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}
	bss = (mlan_ds_bss *)req->pbuf;
	bss->sub_command = MLAN_OID_BSS_ROLE;
	req->req_id = MLAN_IOCTL_BSS;
	req->action = action;
	if (action == MLAN_ACT_SET) {
		if (priv->bss_role == *bss_role) {
			PRINTM(MWARN, "BSS is in desired role already\n");
			goto done;
		} else {
			bss->param.bss_role = *bss_role;
		}
	}
	status = woal_request_ioctl(priv, req, wait_option);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	if (action == MLAN_ACT_GET) {
		*bss_role = bss->param.bss_role;
	} else {
		/* Update moal_private */
		priv->bss_role = *bss_role;
		if (priv->bss_type == MLAN_BSS_TYPE_UAP)
			priv->bss_type = MLAN_BSS_TYPE_STA;
		else if (priv->bss_type == MLAN_BSS_TYPE_STA)
			priv->bss_type = MLAN_BSS_TYPE_UAP;

		if (*bss_role == MLAN_BSS_ROLE_UAP) {
			/* Switch: STA -> uAP */
			/* Setup the OS Interface to our functions */
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2, 6, 29)
			dev->do_ioctl = woal_uap_do_ioctl;
			dev->set_multicast_list = woal_uap_set_multicast_list;
#else
			dev->netdev_ops = &woal_uap_netdev_ops;
#endif
#ifdef UAP_WEXT
			if (IS_UAP_WEXT(priv->phandle->params.cfg80211_wext)) {
#if WIRELESS_EXT < 21
				dev->get_wireless_stats =
					woal_get_uap_wireless_stats;
#endif
				dev->wireless_handlers =
					(struct iw_handler_def
						 *)&woal_uap_handler_def;
			}
#endif /* UAP_WEXT */
		} else if (*bss_role == MLAN_BSS_ROLE_STA) {
			/* Switch: uAP -> STA */
			/* Setup the OS Interface to our functions */
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2, 6, 29)
			dev->do_ioctl = woal_do_ioctl;
			dev->set_multicast_list = woal_set_multicast_list;
#else
			dev->netdev_ops = &woal_netdev_ops;
#endif
#ifdef STA_WEXT
			if (IS_STA_WEXT(priv->phandle->params.cfg80211_wext)) {
#if WIRELESS_EXT < 21
				dev->get_wireless_stats =
					woal_get_wireless_stats;
#endif
				dev->wireless_handlers =
					(struct iw_handler_def
						 *)&woal_handler_def;
			}
#endif /* STA_WEXT */
		}
	}

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

#if defined(STA_WEXT) || defined(UAP_WEXT)
/**
 * @brief Set/Get BSS role
 *
 * @param priv     A pointer to moal_private structure
 * @param wrq      A pointer to iwreq structure
 *
 * @return         0 --success, otherwise fail
 */
int woal_set_get_bss_role(moal_private *priv, struct iwreq *wrq)
{
	int ret = 0;
	t_u32 bss_role = 0;
	t_u8 action = MLAN_ACT_GET;

	ENTER();

	if (wrq->u.data.length) {
		if (copy_from_user(&bss_role, wrq->u.data.pointer,
				   sizeof(int))) {
			PRINTM(MERROR, "Copy from user failed\n");
			ret = -EFAULT;
			goto done;
		}
		if ((bss_role != MLAN_BSS_ROLE_STA &&
		     bss_role != MLAN_BSS_ROLE_UAP)
#ifdef WIFI_DIRECT_SUPPORT
		    || (priv->bss_type != MLAN_BSS_TYPE_WIFIDIRECT)
#endif
		) {
			PRINTM(MWARN, "Invalid BSS role\n");
			ret = -EINVAL;
			goto done;
		}
		if (bss_role == GET_BSS_ROLE(priv)) {
			PRINTM(MWARN, "Already BSS is in desired role\n");
			ret = -EINVAL;
			goto done;
		}
		action = MLAN_ACT_SET;
		/* Reset interface */
		if (MLAN_STATUS_SUCCESS !=
		    woal_reset_intf(priv, MOAL_IOCTL_WAIT, MFALSE)) {
			PRINTM(MERROR, "%s: reset_intf failed \n", __func__);
			ret = -EFAULT;
			goto done;
		}
	}

	if (MLAN_STATUS_SUCCESS != woal_bss_role_cfg(priv, action,
						     MOAL_IOCTL_WAIT,
						     (t_u8 *)&bss_role)) {
		ret = -EFAULT;
		goto done;
	}

	if (!wrq->u.data.length) {
		if (copy_to_user(wrq->u.data.pointer, &bss_role, sizeof(int))) {
			ret = -EFAULT;
			goto done;
		}
		wrq->u.data.length = 1;
	} else {
#if defined(STA_CFG80211) || defined(UAP_CFG80211)
		if (IS_STA_OR_UAP_CFG80211(priv->phandle->params.cfg80211_wext))
			woal_clear_all_mgmt_ies(priv, MOAL_IOCTL_WAIT);
#endif
		/* Initialize private structures */
		woal_init_priv(priv, MOAL_IOCTL_WAIT);

		/* Enable interfaces */
		netif_device_attach(priv->netdev);
		woal_start_queue(priv->netdev);
	}

done:
	LEAVE();
	return ret;
}
#endif /* STA_WEXT || UAP_WEXT */
#endif /* STA_SUPPORT && UAP_SUPPORT */

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 1, 0)
/**
 *  @brief Enable IPv6 Router Advertisement offload
 *
 *  @param handle  A pointer to moal_handle structure
 *  @param enable  enable or disable
 *
 *  @return        MLAN_STATUS_SUCCESS/MLAN_STATUS_PENDING -- success, otherwise
 * fail
 */
static mlan_status woal_set_ipv6_ra_offload(moal_handle *handle, t_u8 enable)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	moal_private *priv = NULL;
	mlan_ds_misc_cfg *misc = NULL;
	mlan_ioctl_req *req = NULL;
	t_u8 *ipv6_ra;
	int i = 0;
	unsigned long flags;
	struct ipv6addr_entry *ipv6_entry = NULL;

	ENTER();

	for (i = 0; i < handle->priv_num && (priv = handle->priv[i]); i++) {
		if (priv->ipv6_addr_configured)
			break;
	}

	if (!priv || !priv->ipv6_addr_configured) {
		PRINTM(MIOCTL, "No IPv6 address configured\n");
		goto done;
	}

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		PRINTM(MIOCTL, "IOCTL req allocated failed!\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	misc = (mlan_ds_misc_cfg *)req->pbuf;
	misc->sub_command = MLAN_OID_MISC_IPV6_RA_OFFLOAD;
	req->req_id = MLAN_IOCTL_MISC_CFG;
	req->action = MLAN_ACT_SET;

	misc->param.ipv6_ra_offload.enable = enable;
	misc->param.ipv6_ra_offload.ipv6_addrs_count = priv->ipv6count;
	spin_lock_irqsave(&priv->ipv6addr_lock, flags);
	ipv6_ra = misc->param.ipv6_ra_offload.ipv6_addrs;
	// Coverity violation raised for kernel's API
	// coverity[cert_arr39_c_violation:SUPPRESS]
	list_for_each_entry (ipv6_entry, &priv->ipv6_addrses, link) {
		moal_memcpy_ext(priv->phandle, ipv6_ra, ipv6_entry->ipv6_addr,
				IPADDR_LEN, IPADDR_LEN);
		ipv6_ra += IPADDR_LEN;
	}
	spin_unlock_irqrestore(&priv->ipv6addr_lock, flags);

	ret = woal_request_ioctl(woal_get_priv(handle, MLAN_BSS_ROLE_STA), req,
				 MOAL_NO_WAIT);
	if (ret != MLAN_STATUS_SUCCESS && ret != MLAN_STATUS_PENDING)
		PRINTM(MIOCTL, "Set IPv6 RA offload failed\n");

done:
	if (ret != MLAN_STATUS_PENDING && req != NULL)
		kfree(req);
	LEAVE();
	return ret;
}
#endif

static mlan_status woal_set_wake_on_mdns(moal_handle *handle, t_u8 enable)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_ds_misc_cfg *misc = NULL;
	mlan_ioctl_req *req = NULL;
	mlan_ds_misc_mef_flt_cfg *mef_cfg = NULL;
	mef_entry_t *entry = NULL;
	mef_filter_t *filter = NULL;

	ENTER();

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		PRINTM(MIOCTL, "IOCTL req allocated failed!\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	misc = (mlan_ds_misc_cfg *)req->pbuf;
	misc->sub_command = MLAN_OID_MISC_MEF_FLT_CFG;
	req->req_id = MLAN_IOCTL_MISC_CFG;
	req->action = MLAN_ACT_SET;

	mef_cfg = (mlan_ds_misc_mef_flt_cfg *)(&misc->param.mef_flt_cfg);
	mef_cfg->mef_act_type = MEF_ACT_MDNS;
	mef_cfg->criteria = MBIT(3);

	entry = (mef_entry_t *)&mef_cfg->mef_entry;
	entry->mode = MBIT(0);
	entry->action = 3;

	filter = (mef_filter_t *)entry->filter_item;
	filter->fill_flag = (FILLING_TYPE | FILLING_REPEAT | FILLING_OFFSET |
			     FILLING_BYTE_SEQ);
	filter->type = TYPE_BYTE_EQ;
	filter->repeat = 1;
	filter->offset = 20;
	filter->num_byte_seq = 2;
	moal_memcpy_ext(handle, filter->byte_seq, "\x08\x00", 2,
			sizeof(filter->byte_seq));
	entry->rpn[1] = RPN_TYPE_AND;

	filter++;
	filter->fill_flag = (FILLING_TYPE | FILLING_REPEAT | FILLING_OFFSET |
			     FILLING_BYTE_SEQ);
	filter->type = TYPE_BYTE_EQ;
	filter->repeat = 1;
	filter->offset = 38;
	filter->num_bytes = 4;
	moal_memcpy_ext(handle, filter->byte_seq, "\xe0\x00\x00\xfb", 4,
			sizeof(filter->byte_seq));
	entry->rpn[2] = RPN_TYPE_AND;
	filter++;
	filter->fill_flag = (FILLING_TYPE | FILLING_PATTERN | FILLING_OFFSET |
			     FILLING_NUM_BYTES);
	filter->type = TYPE_BYTE_EQ + 1;
	filter->pattern = 17;
	filter->offset = 31;
	filter->num_bytes = 1;
	entry->rpn[3] = RPN_TYPE_AND;
	filter++;
	filter->fill_flag = (FILLING_TYPE | FILLING_PATTERN | FILLING_OFFSET |
			     FILLING_NUM_BYTES);
	filter->type = TYPE_BYTE_EQ + 1;
	filter->pattern = 5353;
	filter->offset = 44;
	filter->num_bytes = 2;
	filter++;
	entry->filter_num = 4;
	if (enable) {
		mef_cfg->op_code = MLAN_OP_ADD_MDNS;
	} else {
		mef_cfg->op_code = MLAN_IPADDR_OP_IP_REMOVE;
	}
	ret = woal_request_ioctl(woal_get_priv(handle, MLAN_BSS_ROLE_ANY), req,
				 MOAL_NO_WAIT);
	if (ret != MLAN_STATUS_SUCCESS && ret != MLAN_STATUS_PENDING)
		PRINTM(MIOCTL, "Set Mdns wake up failed! ret=%d\n", ret);
done:
	if (ret != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

#define ICMPV6_HEADER_TYPE "\x3a"
#define IPV6_HEADER_TYPE "\x86\xdd"
#define TYPE_NS "\x87"
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 1, 0)
/**
 *  @brief Enable IPv6 Neighbor Solicitation offload
 *
 *  @param handle  A pointer to moal_handle structure
 *  @param enable  enable or disable mef entry
 *
 *  @return        MLAN_STATUS_SUCCESS/MLAN_STATUS_PENDING -- success, otherwise
 * fail
 */
static mlan_status woal_set_ipv6_ns_offload(moal_handle *handle, t_u8 enable)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_ds_misc_cfg *misc = NULL;
	mlan_ioctl_req *req = NULL;
	mlan_ds_misc_mef_flt_cfg *mef_cfg = NULL;
	mef_entry_t *entry = NULL;
	mef_filter_t *filter = NULL;

	ENTER();

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		PRINTM(MIOCTL, "IOCTL req allocated failed!\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	misc = (mlan_ds_misc_cfg *)req->pbuf;
	misc->sub_command = MLAN_OID_MISC_MEF_FLT_CFG;
	req->req_id = MLAN_IOCTL_MISC_CFG;
	req->action = MLAN_ACT_SET;

	mef_cfg = (mlan_ds_misc_mef_flt_cfg *)(&misc->param.mef_flt_cfg);
	mef_cfg->mef_act_type = MEF_ACT_IPV6_NS;
	mef_cfg->criteria = (MBIT(1) | MBIT(3));

	entry = (mef_entry_t *)&mef_cfg->mef_entry;
	entry->mode = MBIT(0);
	entry->action = 0x40;
	filter = (mef_filter_t *)entry->filter_item;
	filter->fill_flag = (FILLING_TYPE | FILLING_REPEAT | FILLING_OFFSET |
			     FILLING_BYTE_SEQ);
	filter->type = TYPE_BYTE_EQ;
	filter->repeat = 1;
	filter->offset = 20;
	filter->num_byte_seq = 2;
	moal_memcpy_ext(handle, filter->byte_seq, IPV6_HEADER_TYPE, 2,
			sizeof(filter->byte_seq));
	entry->rpn[1] = RPN_TYPE_AND;

	filter++;
	filter->fill_flag = (FILLING_TYPE | FILLING_REPEAT | FILLING_OFFSET |
			     FILLING_BYTE_SEQ);
	filter->type = TYPE_BYTE_EQ;
	filter->repeat = 1;
	filter->offset = 28;
	filter->num_byte_seq = 1;
	moal_memcpy_ext(handle, filter->byte_seq, ICMPV6_HEADER_TYPE, 1,
			sizeof(filter->byte_seq));
	entry->rpn[2] = RPN_TYPE_AND;

	filter++;
	filter->fill_flag = (FILLING_TYPE | FILLING_REPEAT | FILLING_OFFSET |
			     FILLING_BYTE_SEQ);
	filter->type = TYPE_BYTE_EQ;
	filter->repeat = 1;
	filter->offset = 62;
	filter->num_byte_seq = 1;
	moal_memcpy_ext(handle, filter->byte_seq, TYPE_NS, 1,
			sizeof(filter->byte_seq));
	entry->filter_num = 3;
	if (enable) {
		mef_cfg->op_code = MLAN_OP_ADD_IPV6_NS;
	} else {
		mef_cfg->op_code = MLAN_IPADDR_OP_IP_REMOVE;
	}
	ret = woal_request_ioctl(woal_get_priv(handle, MLAN_BSS_ROLE_ANY), req,
				 MOAL_NO_WAIT);
	if (ret != MLAN_STATUS_SUCCESS && ret != MLAN_STATUS_PENDING)
		PRINTM(MIOCTL, "Set ipv6 ns offload failed! ret=%d\n", ret);

done:
	if (ret != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}
#endif

/**
 *  @brief Set auto arp resp
 *
 *  @param handle         A pointer to moal_handle structure
 *  @param enable         enable/disable
 *
 *  @return               MLAN_STATUS_SUCCESS/MLAN_STATUS_PENDING -- success,
 * otherwise fail
 */
static mlan_status woal_set_auto_arp(moal_handle *handle, t_u8 enable)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	int i = 0;
	moal_private *priv = NULL;
	mlan_ds_misc_cfg *misc = NULL;
	mlan_ioctl_req *req = NULL;
	mlan_ds_misc_ipaddr_cfg ipaddr_cfg;

	ENTER();

	memset(&ipaddr_cfg, 0, sizeof(ipaddr_cfg));
	for (i = 0; i < handle->priv_num && (priv = handle->priv[i]); i++) {
		if (priv && priv->ip_addr_type != IPADDR_TYPE_NONE) {
			moal_memcpy_ext(
				handle,
				ipaddr_cfg.ip_addr[ipaddr_cfg.ip_addr_num],
				priv->ip_addr, IPADDR_LEN, IPADDR_LEN);
			ipaddr_cfg.ip_addr_num++;
		}
	}
	if (ipaddr_cfg.ip_addr_num == 0) {
		PRINTM(MIOCTL, "No IP addr configured.\n");
		goto done;
	}

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		PRINTM(MIOCTL, "IOCTL req allocated failed!\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	misc = (mlan_ds_misc_cfg *)req->pbuf;
	misc->sub_command = MLAN_OID_MISC_IP_ADDR;
	req->req_id = MLAN_IOCTL_MISC_CFG;
	req->action = MLAN_ACT_SET;
	moal_memcpy_ext(handle, &misc->param.ipaddr_cfg, &ipaddr_cfg,
			sizeof(ipaddr_cfg), sizeof(misc->param.ipaddr_cfg));
	if (enable) {
		misc->param.ipaddr_cfg.op_code = MLAN_IPADDR_OP_ARP_FILTER |
						 MLAN_IPADDR_OP_AUTO_ARP_RESP;
		misc->param.ipaddr_cfg.ip_addr_type = IPADDR_TYPE_IPV4;
	} else {
		/** remove ip */
		misc->param.ipaddr_cfg.op_code = MLAN_IPADDR_OP_IP_REMOVE;
	}
	ret = woal_request_ioctl(woal_get_priv(handle, MLAN_BSS_ROLE_ANY), req,
				 MOAL_NO_WAIT);
	if (ret != MLAN_STATUS_SUCCESS && ret != MLAN_STATUS_PENDING)
		PRINTM(MIOCTL, "Set auto arp IOCTL failed!\n");
done:
	if (ret != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Set auto arp resp with enhancement method
 *
 *  @param handle         A pointer to moal_handle structure
 *  @param enable         enable/disable
 *
 *  @return               MLAN_STATUS_SUCCESS/MLAN_STATUS_PENDING -- success,
 * otherwise fail
 */
mlan_status woal_set_auto_arp_ext(moal_handle *handle, t_u8 enable)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	int i = 0, ip_addr_num = 0;
	moal_private *priv = NULL;
	mlan_ds_misc_cfg *misc = NULL;
	mlan_ioctl_req *req = NULL;
	mlan_ds_misc_mef_flt_cfg *mef_cfg = NULL;
	mef_entry_t *entry = NULL;
	mef_filter_t *filter = NULL;
	t_u8 ip_addr[MAX_IPADDR][IPADDR_LEN];

	ENTER();

	memset(&mef_cfg, 0, sizeof(mef_cfg));
	for (i = 0; i < handle->priv_num && (priv = handle->priv[i]); i++) {
		if (priv && priv->ip_addr_type != IPADDR_TYPE_NONE) {
			moal_memcpy_ext(handle, ip_addr[ip_addr_num],
					priv->ip_addr, IPADDR_LEN, IPADDR_LEN);
			ip_addr_num++;
		}
	}
	if (ip_addr_num == 0) {
		PRINTM(MIOCTL, "No IP addr configured.\n");
		goto done;
	}

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		PRINTM(MIOCTL, "IOCTL req allocated failed!\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	misc = (mlan_ds_misc_cfg *)req->pbuf;
	misc->sub_command = MLAN_OID_MISC_MEF_FLT_CFG;
	req->req_id = MLAN_IOCTL_MISC_CFG;
	req->action = MLAN_ACT_SET;

	mef_cfg = (mlan_ds_misc_mef_flt_cfg *)(&(misc->param.mef_flt_cfg));
	mef_cfg->mef_act_type = MEF_ACT_AUTOARP;
	mef_cfg->criteria = (MBIT(0) | MBIT(1));

	entry = (mef_entry_t *)&mef_cfg->mef_entry;
	entry->mode = MBIT(0);
	entry->action = 0x10;

	filter = (mef_filter_t *)(entry->filter_item);
	filter->fill_flag = (FILLING_TYPE | FILLING_REPEAT | FILLING_OFFSET |
			     FILLING_BYTE_SEQ);
	filter->type = TYPE_BYTE_EQ;
	filter->repeat = 1;
	filter->offset = 20;
	filter->num_byte_seq = 2;
	moal_memcpy_ext(handle, filter->byte_seq, "\x08\x06", 2,
			sizeof(filter->byte_seq));
	entry->rpn[1] = RPN_TYPE_AND;

	for (i = 0; i < ip_addr_num; i++) {
		filter++;
		filter->fill_flag = (FILLING_TYPE | FILLING_REPEAT |
				     FILLING_OFFSET | FILLING_BYTE_SEQ);
		filter->type = TYPE_BYTE_EQ;
		filter->repeat = 1;
		filter->offset = 46;
		filter->num_byte_seq = 4;
		moal_memcpy_ext(handle, filter->byte_seq, &ip_addr[i],
				sizeof(t_u32), sizeof(filter->byte_seq));
		if (i > 1)
			entry->rpn[i] = RPN_TYPE_OR;
	}
	entry->filter_num = ip_addr_num + 1;

	if (enable)
		mef_cfg->op_code = MLAN_IPADDR_OP_ARP_FILTER |
				   MLAN_IPADDR_OP_AUTO_ARP_RESP;
	else
		/** remove ip */
		mef_cfg->op_code = MLAN_IPADDR_OP_IP_REMOVE;
	ret = woal_request_ioctl(woal_get_priv(handle, MLAN_BSS_ROLE_ANY), req,
				 MOAL_NO_WAIT);
	if (ret != MLAN_STATUS_SUCCESS && ret != MLAN_STATUS_PENDING)
		PRINTM(MIOCTL, "Set auto arp IOCTL failed!\n");
done:
	if (ret != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Set/Get DTIM period
 *
 *  @param priv                 A pointer to moal_private structure
 *  @param action               Action set or get
 *  @param wait_option          Wait option
 *  @param value                DTIM period
 *
 *  @return                     MLAN_STATUS_SUCCESS/MLAN_STATUS_PENDING --
 * success, otherwise fail
 */
mlan_status woal_set_get_dtim_period(moal_private *priv, t_u32 action,
				     t_u8 wait_option, t_u8 *value)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_snmp_mib *mib = NULL;
	mlan_status ret = MLAN_STATUS_SUCCESS;

	ENTER();

	/* Allocate an IOCTL request buffer */
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_snmp_mib));
	if (req == NULL) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	/* Fill request buffer */
	mib = (mlan_ds_snmp_mib *)req->pbuf;
	mib->sub_command = MLAN_OID_SNMP_MIB_DTIM_PERIOD;
	req->req_id = MLAN_IOCTL_SNMP_MIB;
	req->action = action;

	if (action == MLAN_ACT_SET)
		mib->param.dtim_period = *value;

	/* Send IOCTL request to MLAN */
	ret = woal_request_ioctl(priv, req, wait_option);
	if (ret == MLAN_STATUS_SUCCESS && action == MLAN_ACT_GET)
		*value = (t_u8)mib->param.dtim_period;

done:
	if (ret != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Get Host Sleep parameters
 *
 *  @param priv         A pointer to moal_private structure
 *  @param action       Action: set or get
 *  @param wait_option  Wait option (MOAL_WAIT or MOAL_NO_WAIT)
 *  @param hscfg        A pointer to mlan_ds_hs_cfg structure
 *
 *  @return             MLAN_STATUS_SUCCESS/MLAN_STATUS_PENDING -- success,
 * otherwise fail
 */
mlan_status woal_set_get_hs_params(moal_private *priv, t_u16 action,
				   t_u8 wait_option, mlan_ds_hs_cfg *hscfg)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_ds_pm_cfg *pmcfg = NULL;
	mlan_ioctl_req *req = NULL;

	ENTER();

	/* Allocate an IOCTL request buffer */
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_pm_cfg));
	if (req == NULL) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	/* Fill request buffer */
	pmcfg = (mlan_ds_pm_cfg *)req->pbuf;
	pmcfg->sub_command = MLAN_OID_PM_CFG_HS_CFG;
	req->req_id = MLAN_IOCTL_PM_CFG;
	req->action = action;
	if (action == MLAN_ACT_SET)
		moal_memcpy_ext(priv->phandle, &pmcfg->param.hs_cfg, hscfg,
				sizeof(mlan_ds_hs_cfg), sizeof(mlan_ds_hs_cfg));

	/* Send IOCTL request to MLAN */
	ret = woal_request_ioctl(priv, req, wait_option);
	if (ret == MLAN_STATUS_SUCCESS) {
		if (hscfg && action == MLAN_ACT_GET) {
			moal_memcpy_ext(priv->phandle, hscfg,
					&pmcfg->param.hs_cfg,
					sizeof(mlan_ds_hs_cfg),
					sizeof(mlan_ds_hs_cfg));
		}
	}
done:
	if (ret != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Get wakeup reason
 *
 *  @param priv         A pointer to moal_private structure
 *  @param wakeup_reason        A pointer to mlan_ds_hs_wakeup_reason  structure
 *
 *  @return             MLAN_STATUS_SUCCESS/MLAN_STATUS_PENDING -- success,
 * otherwise fail
 */
mlan_status woal_get_wakeup_reason(moal_private *priv,
				   mlan_ds_hs_wakeup_reason *wakeup_reason)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_ds_pm_cfg *pmcfg = NULL;
	mlan_ioctl_req *req = NULL;

	ENTER();

	/* Allocate an IOCTL request buffer */
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_pm_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	/* Fill request buffer */
	pmcfg = (mlan_ds_pm_cfg *)req->pbuf;
	pmcfg->sub_command = MLAN_OID_PM_HS_WAKEUP_REASON;
	req->req_id = MLAN_IOCTL_PM_CFG;
	req->action = MLAN_ACT_GET;

	/* Send IOCTL request to MLAN */
	ret = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT_TIMEOUT);
	if (ret == MLAN_STATUS_SUCCESS) {
		wakeup_reason->hs_wakeup_reason =
			pmcfg->param.wakeup_reason.hs_wakeup_reason;
	}
done:
	if (ret != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Set or Get wowlan config
 *
 *  @param priv         A pointer to moal_private structure
 *  @param action       Action: action to config wowlan
 *  @param wait_option  Wait option (MOAL_WAIT or MOAL_NO_WAIT)
 *  @param mefcfg        A pointer to mlan_ds_misc_mef_flt_cfg structure
 *
 *  @return             MLAN_STATUS_SUCCESS/MLAN_STATUS_FAILURE
 */
mlan_status woal_set_get_wowlan_config(moal_private *priv, t_u16 action,
				       t_u8 wait_option,
				       mlan_ds_misc_mef_flt_cfg *mefcfg)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_ds_misc_cfg *misccfg = NULL;
	mlan_ioctl_req *req = NULL;

	ENTER();

	/** Allocate an IOCTL request buffer*/
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}
	/** Fill request buffer */
	misccfg = (mlan_ds_misc_cfg *)req->pbuf;
	misccfg->sub_command = MLAN_OID_MISC_MEF_FLT_CFG;
	req->req_id = MLAN_IOCTL_MISC_CFG;
	req->action = action;
	moal_memcpy_ext(priv->phandle, &misccfg->param.mef_flt_cfg, mefcfg,
			sizeof(mlan_ds_misc_mef_flt_cfg),
			sizeof(mlan_ds_misc_mef_flt_cfg));
	/* Send IOCTL request to MLAN */
	ret = woal_request_ioctl(priv, req, wait_option);
	if (ret == MLAN_STATUS_SUCCESS) {
		if (mefcfg && action == MLAN_ACT_GET) {
			moal_memcpy_ext(priv->phandle, mefcfg,
					&misccfg->param.mef_flt_cfg,
					sizeof(mlan_ds_misc_mef_flt_cfg),
					sizeof(mlan_ds_misc_mef_flt_cfg));
		}
	}
	if (ret != MLAN_STATUS_SUCCESS && ret != MLAN_STATUS_PENDING)
		PRINTM(MIOCTL, "Set Get wowlan IOCTL failed!\n");
done:
	if (ret != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Cancel Host Sleep configuration
 *
 *  @param priv             A pointer to moal_private structure
 *  @param wait_option      wait option
 *
 *  @return                 MLAN_STATUS_SUCCESS, MLAN_STATUS_PENDING,
 *                              or MLAN_STATUS_FAILURE
 */
mlan_status woal_cancel_hs(moal_private *priv, t_u8 wait_option)
{
#ifdef STA_CFG80211
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 1, 0)
	moal_handle *handle = NULL;
#endif
#endif
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_ds_hs_cfg hscfg;
#ifdef STA_CFG80211
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 1, 0)
	int i;
#endif
#endif
	ENTER();

	memset(&hscfg, 0, sizeof(mlan_ds_hs_cfg));

	if (!priv) {
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}
	/* Cancel Host Sleep */
	hscfg.conditions = HOST_SLEEP_CFG_CANCEL;
	hscfg.is_invoke_hostcmd = MTRUE;
	ret = woal_set_get_hs_params(priv, MLAN_ACT_SET, wait_option, &hscfg);
	if (ret != MLAN_STATUS_SUCCESS && ret != MLAN_STATUS_PENDING) {
		PRINTM(MERROR, "%s: woal_set_get_hs_params failed \n",
		       __func__);
		LEAVE();
		return ret;
	}
	if (moal_extflg_isset(priv->phandle, EXT_ROAMOFFLOAD_IN_HS)) {
		/*Disable firmware roaming*/
		woal_enable_fw_roaming(priv, 0);
	}
	if (priv->phandle->fw_roam_enable == ROAM_OFFLOAD_WITH_BSSID ||
	    priv->phandle->fw_roam_enable == ROAM_OFFLOAD_WITH_SSID ||
	    priv->phandle->fw_roam_enable == AUTO_RECONNECT)
		if (woal_config_fw_roaming(priv, ROAM_OFFLOAD_RESUME_CFG, NULL))
			PRINTM(MERROR,
			       "config fw roaming ROAM_OFFLOAD_RESUME_CFG failed\n");
#ifdef STA_CFG80211
	if (priv->phandle->fw_roam_enable == AUTO_RECONNECT)
		woal_set_clear_pmk(priv, MLAN_ACT_CLEAR);
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 1, 0)
#if IS_ENABLED(CONFIG_IPV6)
	if (priv->phandle->hs_auto_arp) {
		PRINTM(MIOCTL, "Cancel Host Sleep... remove ipv6 offload\n");
		/** Set ipv6 router advertisement message offload */
		woal_set_ipv6_ra_offload(priv->phandle, MFALSE);
	}
	/** Set Neighbor Solitation message offload */
	woal_set_ipv6_ns_offload(priv->phandle, MFALSE);
#endif
#endif

	if (priv->phandle->hs_auto_arp) {
		PRINTM(MIOCTL, "Cancel Host Sleep... remove Mdns wake up\n");
		/** Set ipv6 router advertisement message offload */
		woal_set_wake_on_mdns(priv->phandle, MFALSE);
	}

	if (priv->phandle->hs_auto_arp) {
		PRINTM(MIOCTL, "Cancel Host Sleep... remove FW auto arp\n");
		/* remove auto arp from FW */
		woal_set_auto_arp(priv->phandle, MFALSE);
		/* remove auto arp from FW */
		woal_set_auto_arp_ext(priv->phandle, MFALSE);
	}

#ifdef STA_CFG80211
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 1, 0)
	handle = priv->phandle;
	if (GTK_REKEY_OFFLOAD_SUSPEND == handle->params.gtk_rekey_offload) {
		PRINTM(MIOCTL,
		       "Cancel Host Sleep... clear gtk rekey offload of FW\n");
		for (i = 0; i < handle->priv_num; i++) {
			if (handle->priv[i] &&
			    handle->priv[i]->gtk_data_ready) {
				PRINTM(MCMND, "clear GTK in resume\n");
				if (MLAN_STATUS_SUCCESS !=
				    woal_set_rekey_data(handle->priv[i], NULL,
							MLAN_ACT_CLEAR,
							wait_option))
					PRINTM(MERROR,
					       "%s: clear GTK in resume failed \n",
					       __func__);
			}
		}
	}
#endif
#endif

	LEAVE();
	return ret;
}

/**  @brief This function config fw roaming parameters
 *
 *  @param priv     A Pointer to the moal_private structure
 *  @return         MTRUE or MFALSE
 */
static int woal_set_fw_roaming_params(moal_private *priv)
{
	int ret = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;
	mlan_ioctl_req *req = NULL;
	mlan_ds_sec_cfg *sec = NULL;
	woal_roam_offload_cfg roam_offload_cfg;
#ifdef STA_CFG80211
	t_u8 zero[MLAN_MAX_KEY_LENGTH] = {0};
#endif

	/*Enable fw roaming*/
	if (woal_config_fw_roaming(priv, ROAM_OFFLOAD_ENABLE, NULL))
		PRINTM(MERROR,
		       "config fw roaming ROAM_OFFLOAD_ENABLE failed\n");
	/*Download fw roaming parameters*/
	if (woal_config_fw_roaming(priv, ROAM_OFFLOAD_PARAM_CFG,
				   &priv->phandle->fw_roam_params))
		PRINTM(MERROR,
		       "config fw roaming ROAM_OFFLOAD_PARAM_CFG failed\n");

		/*Download userset passphrase key and current connection's PMK*/
#ifdef STA_CFG80211
	if (!priv->phandle->fw_roam_params.userset_passphrase) {
		woal_set_clear_pmk(priv, MLAN_ACT_SET);
		goto done;
	}
#endif

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_sec_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	req->req_id = MLAN_IOCTL_SEC_CFG;
	sec = (mlan_ds_sec_cfg *)req->pbuf;
	sec->sub_command = MLAN_OID_SEC_CFG_PASSPHRASE;
	sec->multi_passphrase = 1;
	req->action = MLAN_ACT_SET;

	/*Copy user set passphrase*/
	moal_memcpy_ext(priv->phandle, (char *)sec->param.roam_passphrase,
			(char *)priv->phandle->ssid_passphrase,
			MAX_SEC_SSID_NUM * sizeof(mlan_ds_passphrase),
			MAX_SEC_SSID_NUM * sizeof(mlan_ds_passphrase));
	roam_offload_cfg.userset_passphrase =
		priv->phandle->fw_roam_params.userset_passphrase;
#ifdef STA_CFG80211
	if (memcmp(priv->pmk.pmk, zero, MLAN_MAX_KEY_LENGTH)) {
		/*Download current connection PMK*/
		if (priv->pmk_saved) {
			woal_set_clear_pmk(priv, MLAN_ACT_SET);
			priv->pmk_saved = false;
		}
	}
#endif
	/*Set userset to mlan adapter*/
	if (woal_config_fw_roaming(priv, ROAM_OFFLOAD_ENABLE,
				   &roam_offload_cfg))
		PRINTM(MERROR,
		       "config fw roaming ROAM_OFFLOAD_ENABLE failed\n");

	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**  @brief This function enable/disable fw roaming
 *
 *  @param priv     A Pointer to the moal_private structure
 *  @param enable   Enable/disable fw roaming
 *  @return         MTRUE or MFALSE
 */
int woal_enable_fw_roaming(moal_private *priv, int data)
{
	mlan_ioctl_req *ioctl_req = NULL;
	mlan_ds_misc_cfg *misc = NULL;
	mlan_ds_misc_roam_offload *roam = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;
	int ret = 0;

	ENTER();

	if (!priv || !priv->phandle) {
		PRINTM(MERROR, "priv or handle is null\n");
		ret = -EFAULT;
		goto done;
	}

	ioctl_req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (ioctl_req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	if (!data && !priv->phandle->fw_roam_enable) {
		PRINTM(MIOCTL, "Fw roaming already disabled\n");
		goto done;
	}

	misc = (mlan_ds_misc_cfg *)ioctl_req->pbuf;
	misc->sub_command = MLAN_OID_MISC_ROAM_OFFLOAD;
	ioctl_req->req_id = MLAN_IOCTL_MISC_CFG;

	roam = (mlan_ds_misc_roam_offload *)&misc->param.roam_offload;
	roam->aplist.ap_num = 0;
	/* SET operation */
	ioctl_req->action = MLAN_ACT_SET;
	roam->enable = data;
	roam->config_mode = ROAM_OFFLOAD_ENABLE;

	if (moal_extflg_isset(priv->phandle, EXT_ROAMOFFLOAD_IN_HS) && data) {
		priv->phandle->fw_roam_enable = data;
		goto done;
	}

	status = woal_request_ioctl(priv, ioctl_req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}
	priv->phandle->fw_roam_enable = data;
	if (!data) {
		memset((char *)&priv->phandle->fw_roam_params, 0,
		       sizeof(woal_roam_offload_cfg));
		memset((char *)&priv->phandle->ssid_passphrase, 0,
		       MAX_SEC_SSID_NUM * sizeof(mlan_ds_passphrase));
	}
#ifdef STA_CFG80211
	else if (priv->media_connected && priv->pmk_saved) {
		woal_set_clear_pmk(priv, MLAN_ACT_SET);
		priv->pmk_saved = false;
	}
#endif
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(ioctl_req);

	LEAVE();
	return ret;
}

/**  @brief This function set rgpower table
 *
 *  @param priv     A Pointer to the moal_private structure
 *  @return         MTRUE or MFALSE
 */
mlan_status woal_set_rgpower_table(moal_handle *handle)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_ioctl_req *req = NULL;
	mlan_ds_misc_cfg *misc = NULL;

	ENTER();
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req) {
		misc = (mlan_ds_misc_cfg *)req->pbuf;
		misc->sub_command = MLAN_OID_MISC_REGION_POWER_CFG;
		req->req_id = MLAN_IOCTL_MISC_CFG;
		req->action = MLAN_ACT_SET;
		ret = woal_request_ioctl(woal_get_priv(handle,
						       MLAN_BSS_ROLE_ANY),
					 req, MOAL_IOCTL_WAIT);
	}
	if (ret != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**  @brief This function enables the host sleep
 *
 *  @param priv     A Pointer to the moal_private structure
 *  @return         MTRUE or MFALSE
 */
int woal_enable_hs(moal_private *priv)
{
	mlan_ds_hs_cfg hscfg;
	moal_handle *handle = NULL;
	int hs_actived = MFALSE;
	int timeout = 0;
	int i;
#ifdef SDIO_SUSPEND_RESUME
	mlan_ds_ps_info pm_info;
#endif
	pmlan_ds_misc_keep_alive keep_alive = NULL;
	pmlan_ds_misc_keep_alive_rx keep_alive_rx = NULL;
	t_u8 media_connected = MFALSE;

	ENTER();

	if (priv == NULL) {
		PRINTM(MERROR, "Invalid priv\n");
		goto done;
	}
	handle = priv->phandle;
	if (handle->hs_activated == MTRUE) {
		PRINTM(MIOCTL, "HS Already actived\n");
		hs_actived = MTRUE;
		goto done;
	}
	for (i = 0; i < MIN(handle->priv_num, MLAN_MAX_BSS_NUM); i++) {
		if (handle->priv[i] &&
		    (GET_BSS_ROLE(handle->priv[i]) == MLAN_BSS_ROLE_STA)) {
			if (moal_extflg_isset(handle,
					      EXT_DISCONNECT_ON_SUSPEND) &&
			    handle->priv[i]->media_connected == MTRUE) {
				PRINTM(MIOCTL, "disconnect on suspend\n");
				if (MLAN_STATUS_SUCCESS !=
				    woal_disconnect(handle->priv[i],
						    MOAL_NO_WAIT, NULL,
						    DEF_DEAUTH_REASON_CODE)) {
					PRINTM(MIOCTL,
					       "disconnect on suspend failed\n");
				}
			}
		}
		if (handle->priv[i]) {
			PRINTM(MIOCTL, "woal_delba_all on priv[%d]\n", i);
			woal_delba_all(handle->priv[i], MOAL_NO_WAIT);
		}
	}

#if defined(STA_CFG80211) || defined(UAP_CFG80211)
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(2, 6, 39)
	if (priv->phandle->is_remain_timer_set) {
		woal_cancel_timer(&priv->phandle->remain_timer);
		woal_remain_timer_func(priv->phandle);
	}
	/* cancel pending remain on channel */
	if (priv->phandle->remain_on_channel) {
		t_u8 channel_status;
		moal_private *remain_priv =
			priv->phandle->priv[priv->phandle->remain_bss_index];
		if (remain_priv) {
			if (woal_cfg80211_remain_on_channel_cfg(
				    remain_priv, MOAL_NO_WAIT, MTRUE,
				    &channel_status, NULL, 0, 0)) {
				PRINTM(MIOCTL,
				       "Failed to cancel pending remain on channel\n");
			}
			if (priv->phandle->cookie) {
				cfg80211_remain_on_channel_expired(
#if CFG80211_VERSION_CODE < KERNEL_VERSION(3, 6, 0)
					remain_priv->netdev,
#else
					remain_priv->wdev,
#endif
					priv->phandle->cookie,
					&priv->phandle->chan,
#if CFG80211_VERSION_CODE < KERNEL_VERSION(3, 8, 0)
					priv->phandle->channel_type,
#endif
					GFP_ATOMIC);
				priv->phandle->cookie = 0;
			}
		}
		priv->phandle->remain_on_channel = MFALSE;
	}
#endif
#endif

#ifdef STA_SUPPORT
	woal_reconfig_bgscan(priv->phandle);
#endif

	if (moal_extflg_isset(priv->phandle, EXT_ROAMOFFLOAD_IN_HS) &&
	    handle->fw_roam_enable) {
		woal_set_fw_roaming_params(priv);
	}
	if (handle->fw_roam_enable == ROAM_OFFLOAD_WITH_BSSID ||
	    handle->fw_roam_enable == ROAM_OFFLOAD_WITH_SSID ||
	    handle->fw_roam_enable == AUTO_RECONNECT) {
		if (woal_config_fw_roaming(priv, ROAM_OFFLOAD_SUSPEND_CFG,
					   NULL))
			PRINTM(MERROR,
			       "config fw roaming ROAM_OFFLOAD_SUSPEND_CFG failed\n");
#ifdef STA_CFG80211
		if (priv->phandle->fw_roam_enable == AUTO_RECONNECT)
			woal_set_clear_pmk(priv, MLAN_ACT_SET);
#endif
	}
	media_connected = woal_check_media_connected(handle);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 1, 0)
#if IS_ENABLED(CONFIG_IPV6)
	if (handle->hs_auto_arp && media_connected) {
		PRINTM(MIOCTL, "Host Sleep enabled... set ipv6 offload\n");
		/** Set ipv6 router advertisement message offload */
		woal_set_ipv6_ra_offload(handle, MTRUE);
		/** Set Neighbor Solitation message offload */
		woal_set_ipv6_ns_offload(handle, MTRUE);
	}
#endif
#endif

	if (handle->hs_auto_arp) {
		PRINTM(MIOCTL, "Host Sleep enabled... set mdns wake up\n");
		/**MDNS wake up**/
		woal_set_wake_on_mdns(handle, MTRUE);
	}

	if (handle->hs_auto_arp && media_connected) {
		PRINTM(MIOCTL, "Host Sleep enabled... set FW auto arp\n");
		/* Set auto arp response configuration to Fw */
		woal_set_auto_arp(handle, MTRUE);
		/* Set auto arp response configuration to Fw */
		woal_set_auto_arp_ext(handle, MTRUE);
	}

#ifdef STA_CFG80211
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 1, 0)
	if (GTK_REKEY_OFFLOAD_SUSPEND == handle->params.gtk_rekey_offload) {
		PRINTM(MIOCTL,
		       "Host Sleep enabled... set gtk rekey offload to FW\n");
		for (i = 0; i < handle->priv_num; i++) {
			if (handle->priv[i] &&
			    handle->priv[i]->gtk_data_ready) {
				PRINTM(MCMND, "set GTK before suspend\n");
				if (MLAN_STATUS_SUCCESS !=
				    woal_set_rekey_data(
					    handle->priv[i],
					    &handle->priv[i]->gtk_rekey_data,
					    MLAN_ACT_SET, MOAL_NO_WAIT))
					PRINTM(MERROR,
					       "%s: set GTR before suspend failed \n",
					       __func__);
			}
		}
	}
#endif
#endif

	for (i = 0; i < MAX_KEEP_ALIVE_ID; i++) {
		keep_alive = &handle->keep_alive[i];
		if (keep_alive && keep_alive->cached && keep_alive->enable) {
			keep_alive->cached = false;
			woal_start_mkeep_alive(
				woal_get_priv(handle, MLAN_BSS_ROLE_ANY),
				keep_alive->mkeep_alive_id, keep_alive->packet,
				keep_alive->pkt_len, keep_alive->src_mac,
				keep_alive->dst_mac, keep_alive->send_interval,
				keep_alive->retry_interval,
				keep_alive->retry_count);
			keep_alive->pkt_len = 0;
			memset(keep_alive->packet, 0, MKEEP_ALIVE_IP_PKT_MAX);
		}
	}
	for (i = 0; i < MAX_KEEP_ALIVE_RX_ID; i++) {
		keep_alive_rx = &handle->keep_alive_rx[i];
		if (keep_alive_rx && keep_alive_rx->cached &&
		    keep_alive_rx->enable) {
			keep_alive_rx->cached = MFALSE;
			woal_start_mkeep_alive_rx(
				woal_get_priv(handle, MLAN_BSS_ROLE_ANY),
				keep_alive_rx->mkeep_alive_id,
				keep_alive_rx->packet, keep_alive_rx->pkt_len,
				keep_alive_rx->src_mac, keep_alive_rx->dst_mac);
			keep_alive_rx->pkt_len = 0;
			memset(keep_alive_rx->packet, 0,
			       MKEEP_ALIVE_ACK_PKT_MAX);
		}
	}
	/* Enable Host Sleep */
	handle->hs_activate_wait_q_woken = MFALSE;
	memset(&hscfg, 0, sizeof(mlan_ds_hs_cfg));
	hscfg.is_invoke_hostcmd = MTRUE;
	if (woal_set_get_hs_params(priv, MLAN_ACT_SET, MOAL_NO_WAIT, &hscfg) ==
	    MLAN_STATUS_FAILURE) {
		PRINTM(MIOCTL, "IOCTL request HS enable failed\n");
		goto done;
	}
	timeout = wait_event_timeout(handle->hs_activate_wait_q,
				     handle->hs_activate_wait_q_woken,
				     HS_ACTIVE_TIMEOUT);
#ifdef SDIO_MMC
	if (IS_SD(handle->card_type)) {
		sdio_claim_host(((sdio_mmc_card *)handle->card)->func);
	}
#endif

#ifdef SDIO_SUSPEND_RESUME
	memset(&pm_info, 0, sizeof(mlan_ds_ps_info));
#endif
	if ((handle->hs_activated == MTRUE) ||
	    (handle->is_suspended == MTRUE)) {
		PRINTM(MCMND, "suspend success! force=%u skip=%u\n",
		       handle->hs_force_count, handle->hs_skip_count);
		hs_actived = MTRUE;
	}
#ifdef SDIO_SUSPEND_RESUME
	else if (IS_SD(handle->card_type)) {
		handle->suspend_fail = MTRUE;
		if (woal_get_pm_info(priv, &pm_info))
			PRINTM(MCMND, "get pm info failed\n");
		if (pm_info.is_suspend_allowed == MTRUE) {
#ifdef MMC_PM_FUNC_SUSPENDED
			woal_wlan_is_suspended(priv->phandle);
#endif
			handle->hs_force_count++;
			PRINTM(MCMND, "suspend allowed! force=%u skip=%u\n",
			       handle->hs_force_count, handle->hs_skip_count);
			hs_actived = MTRUE;
		}
	}
#endif /* SDIO_SUSPEND_RESUME*/
#ifdef SDIO_MMC
	if (IS_SD(handle->card_type)) {
		sdio_release_host(((sdio_mmc_card *)handle->card)->func);
	}
#endif
	if (hs_actived != MTRUE) {
		handle->hs_skip_count++;
#ifdef SDIO_SUSPEND_RESUME
		if (IS_SD(handle->card_type)) {
			PRINTM(MCMND,
			       "suspend skipped! timeout=%d allow=%d force=%u skip=%u\n",
			       timeout, (int)pm_info.is_suspend_allowed,
			       handle->hs_force_count, handle->hs_skip_count);
		}
#else
		PRINTM(MCMND, "suspend skipped! timeout=%d skip=%u\n", timeout,
		       handle->hs_skip_count);
#endif
		woal_cancel_hs(priv, MOAL_NO_WAIT);
	}
done:
	LEAVE();
	return hs_actived;
}

#ifdef CONFIG_PROC_FS
/**
 *  @brief This function send soft_reset command to firmware
 *
 *  @param handle   A pointer to moal_handle structure
 *  @return         MLAN_STATUS_SUCCESS/MLAN_STATUS_PENDING on success,
 *                      otherwise failure code
 */
mlan_status woal_request_soft_reset(moal_handle *handle)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_ioctl_req *req = NULL;
	mlan_ds_misc_cfg *misc = NULL;

	ENTER();
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req) {
		misc = (mlan_ds_misc_cfg *)req->pbuf;
		misc->sub_command = MLAN_OID_MISC_SOFT_RESET;
		req->req_id = MLAN_IOCTL_MISC_CFG;
		req->action = MLAN_ACT_SET;
		ret = woal_request_ioctl(woal_get_priv(handle,
						       MLAN_BSS_ROLE_ANY),
					 req, MOAL_IOCTL_WAIT);
	}

	handle->surprise_removed = MTRUE;
	woal_sched_timeout(5);
	if (ret != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}
#endif /* CONFIG_PROC_FS */

/**
 *  @brief Set wapi enable
 *
 *  @param priv                 A pointer to moal_private structure
 *  @param wait_option          Wait option
 *  @param enable               MTRUE or MFALSE
 *
 *  @return                     MLAN_STATUS_SUCCESS/MLAN_STATUS_PENDING --
 * success, otherwise fail
 */
mlan_status woal_set_wapi_enable(moal_private *priv, t_u8 wait_option,
				 t_u32 enable)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_sec_cfg *sec = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;
	ENTER();

	/* Allocate an IOCTL request buffer */
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_sec_cfg));
	if (req == NULL) {
		status = MLAN_STATUS_FAILURE;
		goto done;
	}

	/* Fill request buffer */
	sec = (mlan_ds_sec_cfg *)req->pbuf;
	sec->sub_command = MLAN_OID_SEC_CFG_WAPI_ENABLED;
	req->req_id = MLAN_IOCTL_SEC_CFG;
	req->action = MLAN_ACT_SET;
	sec->param.wapi_enabled = enable;

	/* Send IOCTL request to MLAN */
	status = woal_request_ioctl(priv, req, wait_option);
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return status;
}

/**
 *  @brief Get version
 *
 *  @param handle       A pointer to moal_handle structure
 *  @param version      A pointer to version buffer
 *  @param max_len      max length of version buffer
 *
 *  @return             N/A
 */
void woal_get_version(moal_handle *handle, char *version, int max_len)
{
	t_u8 hotfix_ver = 0;
	union {
		t_u32 l;
		t_u8 c[4];
	} ver;
	char fw_ver[32];

	ENTER();

	hotfix_ver = handle->fw_hotfix_version;
	ver.l = handle->fw_release_number;

	if (hotfix_ver) {
		if (snprintf(fw_ver, sizeof(fw_ver), "%u.%u.%u.p%u.%u",
			     ver.c[2], ver.c[1], ver.c[0], ver.c[3],
			     hotfix_ver) <= 0)
			PRINTM(MERROR,
			       "Failed to print hotfix fw version in buffer\n");

	} else {
		if (snprintf(fw_ver, sizeof(fw_ver), "%u.%u.%u.p%u", ver.c[2],
			     ver.c[1], ver.c[0], ver.c[3]) <= 0)
			PRINTM(MERROR,
			       "Failed to print fw version in buffer\n");
	}

	if (snprintf(version, max_len, handle->driver_version, fw_ver) <= 0)
		PRINTM(MERROR, "Failed to print driver version in buffer\n");

	LEAVE();
}

#if defined(STA_WEXT) || defined(UAP_WEXT)
/**
 *  @brief Get Driver Version
 *
 *  @param priv         A pointer to moal_private structure
 *  @param req          A pointer to ifreq structure
 *
 *  @return             0 --success, otherwise fail
 */
int woal_get_driver_version(moal_private *priv, struct ifreq *req)
{
	struct iwreq *wrq = (struct iwreq *)req;
	int len;
	char buf[MLAN_MAX_VER_STR_LEN];
	ENTER();

	woal_get_version(priv->phandle, buf, sizeof(buf) - 1);

	len = strlen(buf);
	if (wrq->u.data.pointer) {
		if (copy_to_user(wrq->u.data.pointer, buf, len)) {
			PRINTM(MERROR, "Copy to user failed\n");
			LEAVE();
			return -EFAULT;
		}
		wrq->u.data.length = len;
	}
	PRINTM(MINFO, "MOAL VERSION: %s\n", buf);
	LEAVE();
	return 0;
}

/**
 *  @brief Get extended driver version
 *
 *  @param priv         A pointer to moal_private structure
 *  @param ireq         A pointer to ifreq structure
 *
 *  @return             0 --success, otherwise fail
 */
int woal_get_driver_verext(moal_private *priv, struct ifreq *ireq)
{
	struct iwreq *wrq = (struct iwreq *)ireq;
	mlan_ds_get_info *info = NULL;
	mlan_ioctl_req *req = NULL;
	int ret = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_get_info));
	if (req == NULL) {
		LEAVE();
		return -ENOMEM;
	}

	info = (mlan_ds_get_info *)req->pbuf;
	info->sub_command = MLAN_OID_GET_VER_EXT;
	req->req_id = MLAN_IOCTL_GET_INFO;
	req->action = MLAN_ACT_GET;

	if (!wrq->u.data.flags) {
		info->param.ver_ext.version_str_sel =
			*((int *)(wrq->u.name + SUBCMD_OFFSET));
	} else {
		if (copy_from_user(
			    &info->param.ver_ext.version_str_sel,
			    wrq->u.data.pointer,
			    sizeof(info->param.ver_ext.version_str_sel))) {
			PRINTM(MERROR, "Copy from user failed\n");
			ret = -EFAULT;
			goto done;
		} else {
			if (((t_s32)(info->param.ver_ext.version_str_sel)) <
			    0) {
				PRINTM(MERROR, "Invalid arguments!\n");
				ret = -EINVAL;
				goto done;
			}
		}
	}

	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	if (wrq->u.data.pointer) {
		if (copy_to_user(wrq->u.data.pointer,
				 info->param.ver_ext.version_str,
				 strlen(info->param.ver_ext.version_str))) {
			PRINTM(MERROR, "Copy to user failed\n");
			ret = -EFAULT;
			goto done;
		}
		wrq->u.data.length = strlen(info->param.ver_ext.version_str);
	}

	PRINTM(MINFO, "MOAL EXTENDED VERSION: %s\n",
	       info->param.ver_ext.version_str);
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);

	LEAVE();
	return ret;
}
#endif

#ifdef DEBUG_LEVEL1
/**
 *  @brief Set driver debug bit masks to mlan in order to enhance performance
 *
 *  @param priv         A pointer to moal_private structure
 *  @param drv_dbg       Driver debug level
 *
 *  @return             0 --success, otherwise fail
 */
int woal_set_drvdbg(moal_private *priv, t_u32 drv_dbg)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_misc_cfg *misc = NULL;
	int ret = 0;

	ENTER();

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		LEAVE();
		return -ENOMEM;
	}

	misc = (mlan_ds_misc_cfg *)req->pbuf;
	misc->sub_command = MLAN_OID_MISC_DRVDBG;
	req->req_id = MLAN_IOCTL_MISC_CFG;
	req->action = MLAN_ACT_SET;
	misc->param.drvdbg = drv_dbg;

	ret = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);

	if (ret != MLAN_STATUS_PENDING)
		kfree(req);

	LEAVE();
	return ret;
}
#endif

/**
 *  @brief Mgmt frame forward registration
 *
 *  @param priv         A pointer to moal_private structure
 *  @param action       Action: set or get
 *  @param pmgmt_subtype_mask   A Pointer to mgmt frame subtype mask
 *  @param wait_option  wait option (MOAL_WAIT or MOAL_NO_WAIT)
 *
 *  @return             0 --success, otherwise fail
 */
int woal_reg_rx_mgmt_ind(moal_private *priv, t_u16 action,
			 t_u32 *pmgmt_subtype_mask, t_u8 wait_option)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_misc_cfg *misc = NULL;
	int ret = 0;

	ENTER();

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		LEAVE();
		return -ENOMEM;
	}

	misc = (mlan_ds_misc_cfg *)req->pbuf;
	misc->sub_command = MLAN_OID_MISC_RX_MGMT_IND;
	req->req_id = MLAN_IOCTL_MISC_CFG;
	req->action = action;
	misc->param.mgmt_subtype_mask = *pmgmt_subtype_mask;
	if (req->action == MLAN_ACT_SET)
		moal_memcpy_ext(priv->phandle, &misc->param.mgmt_subtype_mask,
				pmgmt_subtype_mask,
				sizeof(misc->param.mgmt_subtype_mask),
				sizeof(misc->param.mgmt_subtype_mask));

	ret = woal_request_ioctl(priv, req, wait_option);

	if ((ret == MLAN_STATUS_SUCCESS) && (req->action == MLAN_ACT_GET)) {
		moal_memcpy_ext(priv->phandle, pmgmt_subtype_mask,
				&misc->param.mgmt_subtype_mask,
				sizeof(misc->param.mgmt_subtype_mask),
				sizeof(misc->param.mgmt_subtype_mask));
	}

	if (ret != MLAN_STATUS_PENDING)
		kfree(req);

	LEAVE();
	return ret;
}

/**
 *  @brief Set/Get Transmit beamforming capabilities
 *
 *  @param priv     A pointer to moal_private structure
 *  @param action       Action: set or get
 *  @param tx_bf_cap    A pointer to tx_buf_cap buffer
 *
 *  @return         MLAN_STATUS_SUCCESS -- success, otherwise fail
 */
mlan_status woal_set_get_tx_bf_cap(moal_private *priv, t_u16 action,
				   t_u32 *tx_bf_cap)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_11n_cfg *bf_cfg = NULL;
	mlan_status ret = MLAN_STATUS_SUCCESS;

	ENTER();

	if (!tx_bf_cap) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	/* Allocate an IOCTL request buffer */
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_11n_cfg));
	if (req == NULL) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	/* Fill request buffer */
	bf_cfg = (mlan_ds_11n_cfg *)req->pbuf;
	req->req_id = MLAN_IOCTL_11N_CFG;
	bf_cfg->sub_command = MLAN_OID_11N_CFG_TX_BF_CAP;
	req->action = action;
	if (action == MLAN_ACT_SET)
		bf_cfg->param.tx_bf_cap = *tx_bf_cap;

	/* Send IOCTL request to MLAN */
	ret = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (ret != MLAN_STATUS_SUCCESS) {
		goto done;
	}

	if (action == MLAN_ACT_GET)
		*tx_bf_cap = bf_cfg->param.tx_bf_cap;

done:
	if (ret != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Set/Get Transmit beamforming configuration
 *
 *  @param priv         A pointer to moal_private structure
 *  @param action       Action: set or get
 *  @param tx_bf_cfg    A pointer to tx_bf_cfg structure
 *
 *  @return         MLAN_STATUS_SUCCESS -- success, otherwise fail
 */
mlan_status woal_set_get_tx_bf_cfg(moal_private *priv, t_u16 action,
				   mlan_ds_11n_tx_bf_cfg *tx_bf_cfg)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_ioctl_req *req = NULL;
	mlan_ds_11n_cfg *bf_cfg = NULL;

	ENTER();

	/* Sanity test */
	if (tx_bf_cfg == NULL) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	/* Allocate an IOCTL request buffer */
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_11n_cfg));
	if (req == NULL) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	/* Fill request buffer */
	bf_cfg = (mlan_ds_11n_cfg *)req->pbuf;
	req->req_id = MLAN_IOCTL_11N_CFG;
	bf_cfg->sub_command = MLAN_OID_11N_CFG_TX_BF_CFG;

	req->action = action;
	moal_memcpy_ext(priv->phandle, &bf_cfg->param.tx_bf, tx_bf_cfg,
			sizeof(mlan_ds_11n_tx_bf_cfg),
			sizeof(mlan_ds_11n_tx_bf_cfg));

	/* Send IOCTL request to MLAN */
	ret = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (ret != MLAN_STATUS_SUCCESS)
		goto done;

	if (action == MLAN_ACT_GET)
		moal_memcpy_ext(priv->phandle, tx_bf_cfg, &bf_cfg->param.tx_bf,
				sizeof(mlan_ds_11n_tx_bf_cfg),
				sizeof(mlan_ds_11n_tx_bf_cfg));

done:
	if (ret != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Handle ioctl resp
 *
 *  @param priv     Pointer to moal_private structure
 *  @param req      Pointer to mlan_ioctl_req structure
 *
 *  @return         N/A
 */
void woal_process_ioctl_resp(moal_private *priv, mlan_ioctl_req *req)
{
	int cfg80211_wext;

	ENTER();

	if (priv == NULL) {
		LEAVE();
		return;
	}
	cfg80211_wext = priv->phandle->params.cfg80211_wext;
	switch (req->req_id) {
	case MLAN_IOCTL_GET_INFO:
#ifdef STA_WEXT
#ifdef STA_SUPPORT
		if (IS_STA_WEXT(cfg80211_wext) &&
		    GET_BSS_ROLE(priv) == MLAN_BSS_ROLE_STA)
			woal_ioctl_get_info_resp(priv,
						 (mlan_ds_get_info *)req->pbuf);
#endif
#endif
#ifdef UAP_WEXT
#ifdef UAP_SUPPORT
		if (IS_UAP_WEXT(cfg80211_wext) &&
		    GET_BSS_ROLE(priv) == MLAN_BSS_ROLE_UAP)
			woal_ioctl_get_uap_info_resp(
				priv, (mlan_ds_get_info *)req->pbuf);
#endif
#endif
		break;
#ifdef STA_WEXT
#ifdef STA_SUPPORT
	case MLAN_IOCTL_BSS:
		if (IS_STA_WEXT(cfg80211_wext) &&
		    GET_BSS_ROLE(priv) == MLAN_BSS_ROLE_STA)
			woal_ioctl_get_bss_resp(priv, (mlan_ds_bss *)req->pbuf);
		break;
#endif
#endif
	case MLAN_IOCTL_MISC_CFG:
		woal_ioctl_get_misc_conf(priv, (mlan_ds_misc_cfg *)req->pbuf);
	default:
		break;
	}

	LEAVE();
	return;
}

/**
 *  @brief Get PM info
 *
 *  @param priv                 A pointer to moal_private structure
 *  @param pm_info              A pointer to mlan_ds_ps_info structure
 *
 *  @return                     MLAN_STATUS_SUCCESS/MLAN_STATUS_PENDING --
 * success, otherwise fail
 */
mlan_status woal_get_pm_info(moal_private *priv, mlan_ds_ps_info *pm_info)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_ds_pm_cfg *pmcfg = NULL;
	mlan_ioctl_req *req = NULL;

	ENTER();

	/* Allocate an IOCTL request buffer */
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_pm_cfg));
	if (req == NULL) {
		PRINTM(MERROR, "Fail to alloc mlan_ds_pm_cfg buffer\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	/* Fill request buffer */
	pmcfg = (mlan_ds_pm_cfg *)req->pbuf;
	pmcfg->sub_command = MLAN_OID_PM_INFO;
	req->req_id = MLAN_IOCTL_PM_CFG;
	req->action = MLAN_ACT_GET;

	/* Send IOCTL request to MLAN */
	ret = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (ret == MLAN_STATUS_SUCCESS) {
		if (pm_info) {
			moal_memcpy_ext(priv->phandle, pm_info,
					&pmcfg->param.ps_info,
					sizeof(mlan_ds_ps_info),
					sizeof(mlan_ds_ps_info));
		}
	}
done:
	if (ret != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Get Deep Sleep
 *
 *  @param priv      Pointer to the moal_private driver data struct
 *  @param data      Pointer to return deep_sleep setting
 *
 *  @return          0 --success, otherwise fail
 */
int woal_get_deep_sleep(moal_private *priv, t_u32 *data)
{
	int ret = 0;
	mlan_ioctl_req *req = NULL;
	mlan_ds_pm_cfg *pm = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_pm_cfg));
	if (req == NULL) {
		LEAVE();
		return -ENOMEM;
	}
	pm = (mlan_ds_pm_cfg *)req->pbuf;
	pm->sub_command = MLAN_OID_PM_CFG_DEEP_SLEEP;
	req->req_id = MLAN_IOCTL_PM_CFG;

	req->action = MLAN_ACT_GET;
	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}
	*data = pm->param.auto_deep_sleep.auto_ds;
	*(data + 1) = pm->param.auto_deep_sleep.idletime;

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Set Deep Sleep
 *
 *  @param priv         Pointer to the moal_private driver data struct
 *  @param wait_option  wait option
 *  @param bdeep_sleep  TRUE--enalbe deepsleep, FALSE--disable deepsleep
 *  @param idletime     Idle time for optimized PS API
 *
 *  @return             0 --success, otherwise fail
 */
int woal_set_deep_sleep(moal_private *priv, t_u8 wait_option,
			BOOLEAN bdeep_sleep, t_u16 idletime)
{
	int ret = 0;
	mlan_ioctl_req *req = NULL;
	mlan_ds_pm_cfg *pm = NULL;

	ENTER();

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_pm_cfg));
	if (req == NULL) {
		LEAVE();
		return -ENOMEM;
	}
	pm = (mlan_ds_pm_cfg *)req->pbuf;
	pm->sub_command = MLAN_OID_PM_CFG_DEEP_SLEEP;
	req->req_id = MLAN_IOCTL_PM_CFG;

	req->action = MLAN_ACT_SET;
	if (bdeep_sleep == MTRUE) {
		PRINTM(MIOCTL, "Deep Sleep: sleep\n");
		pm->param.auto_deep_sleep.auto_ds = DEEP_SLEEP_ON;
		if (idletime)
			pm->param.auto_deep_sleep.idletime = idletime;
		ret = woal_request_ioctl(priv, req, wait_option);
		if (ret != MLAN_STATUS_SUCCESS && ret != MLAN_STATUS_PENDING) {
			ret = -EFAULT;
			goto done;
		}
	} else {
		PRINTM(MIOCTL, "%lu : Deep Sleep: wakeup\n", jiffies);
		pm->param.auto_deep_sleep.auto_ds = DEEP_SLEEP_OFF;
		ret = woal_request_ioctl(priv, req, wait_option);
		if (ret != MLAN_STATUS_SUCCESS && ret != MLAN_STATUS_PENDING) {
			ret = -EFAULT;
			goto done;
		}
	}
done:
	if (ret != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Cancel CAC period block
 *
 *  @param priv         A pointer to moal_private structure
 *
 *  @return         N/A
 */
void woal_cancel_cac_block(moal_private *priv)
{
	ENTER();
	/* if during CAC period, wake up wait queue */
	if (priv->phandle->cac_period == MTRUE) {
		priv->phandle->cac_period = MFALSE;
		/* Make sure Chan Report is cancelled */
		if (woal_11h_cancel_chan_report_ioctl(priv, MOAL_IOCTL_WAIT))
			PRINTM(MERROR,
			       "%s: Cancelling CAC chan report in FW failed \n",
			       __func__);
		priv->phandle->meas_start_jiffies = 0;
		if (priv->phandle->delay_bss_start == MTRUE)
			priv->phandle->delay_bss_start = MFALSE;
		if (priv->phandle->meas_wait_q_woken == MFALSE) {
			priv->phandle->meas_wait_q_woken = MTRUE;
			wake_up_interruptible(&priv->phandle->meas_wait_q);
		}
#ifdef UAP_SUPPORT
#if defined(STA_CFG80211) || defined(UAP_CFG80211)
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 14, 0)
		if (priv->uap_host_based &&
		    moal_extflg_isset(priv->phandle, EXT_DFS_OFFLOAD))
			woal_cfg80211_dfs_vendor_event(
				priv, event_dfs_cac_aborted, &priv->chan);
#endif
#endif
#endif
	}
	LEAVE();
}

/** MEAS report timeout value in seconds */

/**
 *  @brief Issue MLAN_OID_11H_CHANNEL_CHECK ioctl
 *
 *  @param priv     Pointer to the moal_private driver data struct
 *  @param wait_option wait option
 *
 *  @return         0 --success, otherwise fail
 */
int woal_11h_channel_check_ioctl(moal_private *priv, t_u8 wait_option)
{
	int ret = 0;
	mlan_ioctl_req *req = NULL;
	mlan_ds_11h_cfg *ds_11hcfg = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;
#ifdef UAP_SUPPORT
	chan_band_info chan;
	chan_band_info uapchan;
#endif

	ENTER();

#ifdef UAP_SUPPORT
	if (priv->skip_cac) {
		LEAVE();
		return ret;
	}
#endif

	/* Skip sending request/report query when DFS_REPEATER_MODE is on. This
	 * would get rid of CAC timers before starting BSSes in
	 * DFS_REPEATER_MODE
	 */
	if (priv->phandle->dfs_repeater_mode) {
		LEAVE();
		return ret;
	}

	if (woal_is_any_interface_active(priv->phandle)) {
#ifdef UAP_SUPPORT
		/* When any other interface is active
		 * Get rid of CAC timer when drcs is disabled */
		t_u16 enable = 0;
		if (priv->phandle->card_info->drcs)
			ret = woal_mc_policy_cfg(priv, &enable, wait_option,
						 MLAN_ACT_GET);
		if (!enable) {
			LEAVE();
			return ret;
		} else {
			woal_get_active_intf_channel(priv, &chan);
			woal_set_get_ap_channel(priv, MLAN_ACT_GET,
						MOAL_IOCTL_WAIT, &uapchan);
			if (chan.channel != uapchan.channel) {
				if (uapchan.is_dfs_chan) {
					PRINTM(MERROR,
					       "DFS channel is not allowed when another connection exists on different channel\n");
					PRINTM(MERROR,
					       "Another connection's channel=%d, dfs channel=%d\n",
					       chan.channel, uapchan.channel);
					return -EINVAL;
				} else {
					// check if we need move first uap0 from
					// DFS channel to new non dfs channel
					woal_check_mc_connection(
						priv, wait_option,
						uapchan.channel);
				}
			}
		}
#else
		LEAVE();
		return status;
#endif
	}

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_11h_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}
	ds_11hcfg = (mlan_ds_11h_cfg *)req->pbuf;

	ds_11hcfg->sub_command = MLAN_OID_11H_CHANNEL_CHECK;
	ds_11hcfg->param.chan_rpt_req.host_based = MFALSE;
	req->req_id = MLAN_IOCTL_11H_CFG;
	req->action = MLAN_ACT_SET;
	/* Send Channel Check command and wait until the report is ready */
	status = woal_request_ioctl(priv, req, wait_option);
	if (status != MLAN_STATUS_SUCCESS) {
		goto done;
	}

	/* set flag from here */
	priv->phandle->cac_period = MTRUE;
	priv->phandle->meas_start_jiffies = jiffies;
	priv->phandle->cac_timer_jiffies =
		ds_11hcfg->param.chan_rpt_req.millisec_dwell_time * HZ / 1000;

#ifdef UAP_SUPPORT
#if defined(STA_CFG80211) || defined(UAP_CFG80211)
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 14, 0)
	if (priv->uap_host_based &&
	    moal_extflg_isset(priv->phandle, EXT_DFS_OFFLOAD))
		woal_cfg80211_dfs_vendor_event(priv, event_dfs_cac_started,
					       &priv->chan);
#endif
#endif
#endif
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Issue MLAN_OID_11H_CHAN_REPORT_REQUEST ioctl to cancel dozer
 *
 *  @param priv     Pointer to the moal_private driver data struct
 *  @param wait_option wait option
 *
 *  @return         0 --success, otherwise fail
 */
int woal_11h_cancel_chan_report_ioctl(moal_private *priv, t_u8 wait_option)
{
	int ret = 0;
	mlan_ioctl_req *req = NULL;
	mlan_ds_11h_cfg *ds_11hcfg = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();
	PRINTM(MCMND, "wlan: woal_11h_cancel_chan_report\n");
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_11h_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}
	ds_11hcfg = (mlan_ds_11h_cfg *)req->pbuf;

	ds_11hcfg->sub_command = MLAN_OID_11H_CHAN_REPORT_REQUEST;
	req->req_id = MLAN_IOCTL_11H_CFG;
	req->action = MLAN_ACT_SET;
	ds_11hcfg->param.chan_rpt_req.millisec_dwell_time = 0;
	/* Send Channel Check command and wait until the report is ready */
	status = woal_request_ioctl(priv, req, wait_option);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief set remain channel
 *
 *  @param priv         A pointer to moal_private structure
 *  @param wait_option  Wait option
 *  @param pchan        A pointer to mlan_ds_remain_chan structure
 *
 *  @return             MLAN_STATUS_SUCCESS -- success, otherwise fail
 */
mlan_status woal_set_remain_channel_ioctl(moal_private *priv, t_u8 wait_option,
					  mlan_ds_remain_chan *pchan)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_ioctl_req *req = NULL;
	mlan_ds_radio_cfg *radio_cfg = NULL;

	ENTER();
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_radio_cfg));
	if (req == NULL) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	radio_cfg = (mlan_ds_radio_cfg *)req->pbuf;
	radio_cfg->sub_command = MLAN_OID_REMAIN_CHAN_CFG;
	req->req_id = MLAN_IOCTL_RADIO_CFG;

	req->action = MLAN_ACT_SET;
	moal_memcpy_ext(priv->phandle, &radio_cfg->param.remain_chan, pchan,
			sizeof(mlan_ds_remain_chan),
			sizeof(mlan_ds_remain_chan));
	ret = woal_request_ioctl(priv, req, wait_option);
	if (ret == MLAN_STATUS_SUCCESS) {
		moal_memcpy_ext(priv->phandle, pchan,
				&radio_cfg->param.remain_chan,
				sizeof(mlan_ds_remain_chan),
				sizeof(mlan_ds_remain_chan));
		PRINTM(MCMND,
		       "Remain_chan_cfg: remove=%d, channel=%d, remain_period=%d\n",
		       pchan->remove, pchan->channel, pchan->remain_period);
	}
done:
	if (ret != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

#ifdef WIFI_DIRECT_SUPPORT
/**
 *  @brief set/get wifi direct mode
 *
 *  @param priv         A pointer to moal_private structure
 *  @param action       set or get
 *  @param mode         A pointer to wifi direct mode
 *
 *  @return             MLAN_STATUS_SUCCESS -- success, otherwise fail
 */
mlan_status woal_wifi_direct_mode_cfg(moal_private *priv, t_u16 action,
				      t_u16 *mode)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_ioctl_req *req = NULL;
	mlan_ds_bss *bss = NULL;

	ENTER();
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_bss));
	if (req == NULL) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	bss = (mlan_ds_bss *)req->pbuf;
	bss->sub_command = MLAN_OID_WIFI_DIRECT_MODE;
	req->req_id = MLAN_IOCTL_BSS;

	req->action = action;
	if (action == MLAN_ACT_SET)
		bss->param.wfd_mode = *mode;
	ret = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (ret == MLAN_STATUS_SUCCESS) {
		*mode = bss->param.wfd_mode;
		PRINTM(MIOCTL, "ACT=%d, wifi_direct_mode=%d\n", action, *mode);
	}
done:
	if (ret != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Set p2p config
 *
 *  @param priv         A pointer to moal_private structure
 *  @param action       Action set or get
 *  @param p2p_config   A pointer to  mlan_ds_wifi_direct_config structure
 *
 *  @return             MLAN_STATUS_SUCCESS -- success, otherwise fail
 */
mlan_status woal_p2p_config(moal_private *priv, t_u32 action,
			    mlan_ds_wifi_direct_config *p2p_config)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_ioctl_req *req = NULL;
	mlan_ds_misc_cfg *misc_cfg = NULL;

	ENTER();
	if (!p2p_config) {
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	misc_cfg = (mlan_ds_misc_cfg *)req->pbuf;
	misc_cfg->sub_command = MLAN_OID_MISC_WIFI_DIRECT_CONFIG;
	req->req_id = MLAN_IOCTL_MISC_CFG;
	req->action = action;
	moal_memcpy_ext(priv->phandle, &misc_cfg->param.p2p_config, p2p_config,
			sizeof(mlan_ds_wifi_direct_config),
			sizeof(mlan_ds_wifi_direct_config));
	ret = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (ret == MLAN_STATUS_SUCCESS) {
		if (action == MLAN_ACT_GET)
			moal_memcpy_ext(priv->phandle, p2p_config,
					&misc_cfg->param.p2p_config,
					sizeof(mlan_ds_wifi_direct_config),
					sizeof(mlan_ds_wifi_direct_config));
	}
done:
	if (ret != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}
#endif /* WIFI_DIRECT_SUPPORT */

#ifdef STA_SUPPORT
/**
 *  @brief Get STA Channel Info
 *
 *  @param priv             A pointer to moal_private structure
 *  @param wait_option      Wait option
 *  @param channel          A pointer to channel info
 *
 *  @return                     MLAN_STATUS_SUCCESS/MLAN_STATUS_PENDING --
 * success, otherwise fail
 */
mlan_status woal_get_sta_channel(moal_private *priv, t_u8 wait_option,
				 chan_band_info *channel)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_bss *bss = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;
	ENTER();

	/* Allocate an IOCTL request buffer */
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_bss));
	if (req == NULL) {
		PRINTM(MERROR, "woal_get_sta_channel req alloc fail\n");
		status = MLAN_STATUS_FAILURE;
		goto done;
	}

	/* Fill request buffer */
	bss = (mlan_ds_bss *)req->pbuf;
	bss->sub_command = MLAN_OID_BSS_CHAN_INFO;
	req->req_id = MLAN_IOCTL_BSS;
	req->action = MLAN_ACT_GET;

	/* Send IOCTL request to MLAN */
	status = woal_request_ioctl(priv, req, wait_option);
	if (status == MLAN_STATUS_SUCCESS && channel)
		moal_memcpy_ext(priv->phandle, channel,
				&(bss->param.sta_channel),
				sizeof(chan_band_info), sizeof(chan_band_info));

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return status;
}

/**
 *  @brief Get RSSI info
 *
 *  @param priv         A pointer to moal_private structure
 *  @param wait_option  Wait option
 *  @param signal       A pointer tp mlan_ds_get_signal structure
 *
 *  @return             MLAN_STATUS_SUCCESS/MLAN_STATUS_PENDING -- success,
 * otherwise fail
 */
mlan_status woal_get_signal_info(moal_private *priv, t_u8 wait_option,
				 mlan_ds_get_signal *signal)
{
	mlan_ds_get_info *info = NULL;
	mlan_ioctl_req *req = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	/* Allocate an IOCTL request buffer */
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_get_info));
	if (req == NULL) {
		status = MLAN_STATUS_FAILURE;
		goto done;
	}

	/* Fill request buffer */
	info = (mlan_ds_get_info *)req->pbuf;
	info->sub_command = MLAN_OID_GET_SIGNAL;
	info->param.signal.selector = ALL_RSSI_INFO_MASK;
	req->req_id = MLAN_IOCTL_GET_INFO;
	req->action = MLAN_ACT_GET;

	/* Send IOCTL request to MLAN */
	status = woal_request_ioctl(priv, req, wait_option);
	if (status == MLAN_STATUS_SUCCESS) {
		if (signal)
			moal_memcpy_ext(priv->phandle, signal,
					&info->param.signal,
					sizeof(mlan_ds_get_signal),
					sizeof(mlan_ds_get_signal));
#ifdef STA_WEXT
		if (IS_STA_WEXT(priv->phandle->params.cfg80211_wext)) {
			if (info->param.signal.selector & BCN_RSSI_AVG_MASK)
				priv->w_stats.qual.level =
					info->param.signal.bcn_rssi_avg;
			if (info->param.signal.selector & BCN_NF_AVG_MASK)
				priv->w_stats.qual.noise =
					info->param.signal.bcn_nf_avg;
		}
#endif
	}
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return status;
}

/**
 *  @brief Get scan table
 *
 *  @param priv         A pointer to moal_private structure
 *  @param wait_option  Wait option
 *  @param scan_resp    A pointer to mlan_scan_resp structure
 *
 *  @return             MLAN_STATUS_SUCCESS/MLAN_STATUS_PENDING -- success,
 * otherwise fail
 */
mlan_status woal_get_scan_table(moal_private *priv, t_u8 wait_option,
				mlan_scan_resp *scan_resp)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_scan *scan = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;
	ENTER();

	if (!scan_resp) {
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}

	/* Allocate an IOCTL request buffer */
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_scan));
	if (req == NULL) {
		status = MLAN_STATUS_FAILURE;
		goto done;
	}

	/* Fill request buffer */
	scan = (mlan_ds_scan *)req->pbuf;
	scan->sub_command = MLAN_OID_SCAN_NORMAL;
	req->req_id = MLAN_IOCTL_SCAN;
	req->action = MLAN_ACT_GET;
	moal_memcpy_ext(priv->phandle, (void *)&scan->param.scan_resp,
			(void *)scan_resp, sizeof(mlan_scan_resp),
			sizeof(mlan_scan_resp));

	/* Send IOCTL request to MLAN */
	status = woal_request_ioctl(priv, req, wait_option);
	if (status == MLAN_STATUS_SUCCESS) {
		if (scan_resp) {
			moal_memcpy_ext(priv->phandle, scan_resp,
					&scan->param.scan_resp,
					sizeof(mlan_scan_resp),
					sizeof(mlan_scan_resp));
		}
	}

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return status;
}

/**
 *  @brief Request a scan
 *
 *  @param priv                 A pointer to moal_private structure
 *  @param wait_option          Wait option
 *  @param req_ssid             A pointer to mlan_802_11_ssid structure
 *
 *  @return                     MLAN_STATUS_SUCCESS -- success, otherwise fail
 */
mlan_status woal_request_scan(moal_private *priv, t_u8 wait_option,
			      mlan_802_11_ssid *req_ssid)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	moal_handle *handle = priv->phandle;
	mlan_ioctl_req *ioctl_req = NULL;
	mlan_ds_scan *scan = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;
	ENTER();

	if (MOAL_ACQ_SEMAPHORE_BLOCK(&handle->async_sem)) {
		PRINTM(MERROR, "Acquire semaphore error, request_scan\n");
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}
	handle->scan_pending_on_block = MTRUE;
	handle->scan_priv = priv;

	/* Allocate an IOCTL request buffer */
	ioctl_req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_scan));
	if (ioctl_req == NULL) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	scan = (mlan_ds_scan *)ioctl_req->pbuf;

	if (req_ssid && req_ssid->ssid_len != 0) {
		/* Specific SSID scan */
		ioctl_req->req_id = MLAN_IOCTL_SCAN;
		ioctl_req->action = MLAN_ACT_SET;

		scan->sub_command = MLAN_OID_SCAN_SPECIFIC_SSID;

		moal_memcpy_ext(handle, scan->param.scan_req.scan_ssid.ssid,
				req_ssid->ssid, req_ssid->ssid_len,
				MLAN_MAX_SSID_LENGTH);
		scan->param.scan_req.scan_ssid.ssid_len =
			MIN(MLAN_MAX_SSID_LENGTH, req_ssid->ssid_len);
	} else {
		/* Normal scan */
		ioctl_req->req_id = MLAN_IOCTL_SCAN;
		ioctl_req->action = MLAN_ACT_SET;

		scan->sub_command = MLAN_OID_SCAN_NORMAL;
	}
	/* Send IOCTL request to MLAN */
	status = woal_request_ioctl(priv, ioctl_req, wait_option);

	if (status == MLAN_STATUS_FAILURE) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(ioctl_req);

	if (ret == MLAN_STATUS_FAILURE) {
		handle->scan_pending_on_block = MFALSE;
		handle->scan_priv = NULL;
		MOAL_REL_SEMAPHORE(&handle->async_sem);
	}
	LEAVE();
	return ret;
}

/**
 *  @brief Find the best network to associate
 *
 *  @param priv             A pointer to moal_private structure
 *  @param wait_option      Wait option
 *  @param ssid_bssid       A pointer to mlan_ssid_bssid structure
 *
 *  @return                     MLAN_STATUS_SUCCESS/MLAN_STATUS_PENDING --
 * success, otherwise fail
 */
mlan_status woal_find_best_network(moal_private *priv, t_u8 wait_option,
				   mlan_ssid_bssid *ssid_bssid)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_bss *bss = NULL;
	mlan_status ret = MLAN_STATUS_SUCCESS;

	ENTER();

	if (!ssid_bssid) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	/* Allocate an IOCTL request buffer */
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_bss));
	if (req == NULL) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	/* Fill request buffer */
	bss = (mlan_ds_bss *)req->pbuf;
	req->req_id = MLAN_IOCTL_BSS;
	req->action = MLAN_ACT_GET;
	bss->sub_command = MLAN_OID_BSS_FIND_BSS;

	moal_memcpy_ext(priv->phandle, &bss->param.ssid_bssid, ssid_bssid,
			sizeof(mlan_ssid_bssid), sizeof(mlan_ssid_bssid));

	/* Send IOCTL request to MLAN */
	ret = woal_request_ioctl(priv, req, wait_option);
	if (ret == MLAN_STATUS_SUCCESS) {
		moal_memcpy_ext(priv->phandle, ssid_bssid,
				&bss->param.ssid_bssid, sizeof(mlan_ssid_bssid),
				sizeof(mlan_ssid_bssid));
		PRINTM(MINFO, "Find network: ssid=%s, " MACSTR ", idx=%d\n",
		       ssid_bssid->ssid.ssid,
		       MAC2STR((t_u8 *)&ssid_bssid->bssid),
		       (int)ssid_bssid->idx);
	}

done:
	if (ret != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Find the best network to associate
 *
 *  @param priv             A pointer to moal_private structure
 *  @param bssid       		A pointer to bssid
 *
 *  @return                     MLAN_STATUS_SUCCESS/MLAN_STATUS_PENDING --
 * success, otherwise fail
 */
mlan_status woal_find_bssid(moal_private *priv, mlan_802_11_mac_addr bssid)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_bss *bss = NULL;
	mlan_status ret = MLAN_STATUS_SUCCESS;

	ENTER();
	/* Allocate an IOCTL request buffer */
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_bss));
	if (req == NULL) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	/* Fill request buffer */
	bss = (mlan_ds_bss *)req->pbuf;
	req->req_id = MLAN_IOCTL_BSS;
	req->action = MLAN_ACT_GET;
	bss->sub_command = MLAN_OID_BSS_FIND_BSSID;

	moal_memcpy_ext(priv->phandle, &bss->param.bssid, bssid,
			sizeof(mlan_802_11_mac_addr),
			sizeof(mlan_802_11_mac_addr));

	/* Send IOCTL request to MLAN */
	ret = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
done:
	if (ret != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Check if AP channel is valid for STA Region
 *
 *  @param priv             A pointer to moal_private structure
 *  @param wait_option      Wait option
 *  @param ssid_bssid       A pointer to mlan_ssid_bssid structure
 *
 *  @return                 MLAN_STATUS_SUCCESS/MLAN_STATUS_PENDING -- success,
 * otherwise fail
 */
mlan_status woal_11d_check_ap_channel(moal_private *priv, t_u8 wait_option,
				      mlan_ssid_bssid *ssid_bssid)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_bss *bss = NULL;
	mlan_status ret = MLAN_STATUS_SUCCESS;

	ENTER();

	if (!ssid_bssid) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	/* Allocate an IOCTL request buffer */
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_bss));
	if (req == NULL) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	/* Fill request buffer */
	bss = (mlan_ds_bss *)req->pbuf;
	req->req_id = MLAN_IOCTL_BSS;
	req->action = MLAN_ACT_GET;
	bss->sub_command = MLAN_OID_BSS_11D_CHECK_CHANNEL;

	moal_memcpy_ext(priv->phandle, &bss->param.ssid_bssid, ssid_bssid,
			sizeof(mlan_ssid_bssid), sizeof(mlan_ssid_bssid));

	PRINTM(MINFO, "ssid=%s, " MACSTR ", idx=%d\n", ssid_bssid->ssid.ssid,
	       MAC2STR((t_u8 *)&ssid_bssid->bssid), (int)ssid_bssid->idx);

	/* Send IOCTL request to MLAN */
	ret = woal_request_ioctl(priv, req, wait_option);

done:
	if (ret != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Get authentication mode
 *
 *  @param priv             A pointer to moal_private structure
 *  @param wait_option      Wait option
 *  @param auth_mode        A pointer to authentication mode
 *
 *  @return                     MLAN_STATUS_SUCCESS/MLAN_STATUS_PENDING --
 * success, otherwise fail
 */
mlan_status woal_get_auth_mode(moal_private *priv, t_u8 wait_option,
			       t_u32 *auth_mode)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_sec_cfg *sec = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;
	ENTER();

	/* Allocate an IOCTL request buffer */
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_sec_cfg));
	if (req == NULL) {
		status = MLAN_STATUS_FAILURE;
		goto done;
	}

	/* Fill request buffer */
	sec = (mlan_ds_sec_cfg *)req->pbuf;
	sec->sub_command = MLAN_OID_SEC_CFG_AUTH_MODE;
	req->req_id = MLAN_IOCTL_SEC_CFG;
	req->action = MLAN_ACT_GET;

	/* Send IOCTL request to MLAN */
	status = woal_request_ioctl(priv, req, wait_option);
	if (status == MLAN_STATUS_SUCCESS && auth_mode)
		*auth_mode = sec->param.auth_mode;
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return status;
}

/**
 *  @brief Get encrypt mode
 *
 *  @param priv                 A pointer to moal_private structure
 *  @param wait_option          Wait option
 *  @param encrypt_mode         A pointer to encrypt mode
 *
 *  @return                     MLAN_STATUS_SUCCESS/MLAN_STATUS_PENDING --
 * success, otherwise fail
 */
mlan_status woal_get_encrypt_mode(moal_private *priv, t_u8 wait_option,
				  t_u32 *encrypt_mode)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_sec_cfg *sec = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	/* Allocate an IOCTL request buffer */
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_sec_cfg));
	if (req == NULL) {
		status = MLAN_STATUS_FAILURE;
		goto done;
	}

	/* Fill request buffer */
	sec = (mlan_ds_sec_cfg *)req->pbuf;
	sec->sub_command = MLAN_OID_SEC_CFG_ENCRYPT_MODE;
	req->req_id = MLAN_IOCTL_SEC_CFG;
	req->action = MLAN_ACT_GET;

	/* Send IOCTL request to MLAN */
	status = woal_request_ioctl(priv, req, wait_option);
	if (status == MLAN_STATUS_SUCCESS && encrypt_mode)
		*encrypt_mode = sec->param.encrypt_mode;
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return status;
}

/**
 *  @brief Get WPA enable
 *
 *  @param priv             A pointer to moal_private structure
 *  @param wait_option      Wait option
 *  @param enable           A pointer to wpa enable status
 *
 *  @return                     MLAN_STATUS_SUCCESS/MLAN_STATUS_PENDING --
 * success, otherwise fail
 */
mlan_status woal_get_wpa_enable(moal_private *priv, t_u8 wait_option,
				t_u32 *enable)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_sec_cfg *sec = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	/* Allocate an IOCTL request buffer */
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_sec_cfg));
	if (req == NULL) {
		status = MLAN_STATUS_FAILURE;
		goto done;
	}

	/* Fill request buffer */
	sec = (mlan_ds_sec_cfg *)req->pbuf;
	sec->sub_command = MLAN_OID_SEC_CFG_WPA_ENABLED;
	req->req_id = MLAN_IOCTL_SEC_CFG;
	req->action = MLAN_ACT_GET;

	/* Send IOCTL request to MLAN */
	status = woal_request_ioctl(priv, req, wait_option);
	if (status == MLAN_STATUS_SUCCESS && enable)
		*enable = sec->param.wpa_enabled;
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return status;
}

/**
 *  @brief Set authentication mode
 *
 *  @param priv             A pointer to moal_private structure
 *  @param wait_option      Wait option
 *  @param auth_mode        Authentication mode
 *
 *  @return                     MLAN_STATUS_SUCCESS/MLAN_STATUS_PENDING --
 * success, otherwise fail
 */
mlan_status woal_set_auth_mode(moal_private *priv, t_u8 wait_option,
			       t_u32 auth_mode)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_sec_cfg *sec = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;
	ENTER();

	/* Allocate an IOCTL request buffer */
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_sec_cfg));
	if (req == NULL) {
		status = MLAN_STATUS_FAILURE;
		goto done;
	}

	/* Fill request buffer */
	sec = (mlan_ds_sec_cfg *)req->pbuf;
	sec->sub_command = MLAN_OID_SEC_CFG_AUTH_MODE;
	req->req_id = MLAN_IOCTL_SEC_CFG;
	req->action = MLAN_ACT_SET;
	sec->param.auth_mode = auth_mode;

	/* Send IOCTL request to MLAN */
	status = woal_request_ioctl(priv, req, wait_option);
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return status;
}

/**
 *  @brief Set encrypt mode
 *
 *  @param priv                 A pointer to moal_private structure
 *  @param wait_option          Wait option
 *  @param encrypt_mode         Encryption mode
 *
 *  @return                     MLAN_STATUS_SUCCESS/MLAN_STATUS_PENDING --
 * success, otherwise fail
 */
mlan_status woal_set_encrypt_mode(moal_private *priv, t_u8 wait_option,
				  t_u32 encrypt_mode)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_sec_cfg *sec = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;
	ENTER();

	/* Allocate an IOCTL request buffer */
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_sec_cfg));
	if (req == NULL) {
		status = MLAN_STATUS_FAILURE;
		goto done;
	}

	/* Fill request buffer */
	sec = (mlan_ds_sec_cfg *)req->pbuf;
	sec->sub_command = MLAN_OID_SEC_CFG_ENCRYPT_MODE;
	req->req_id = MLAN_IOCTL_SEC_CFG;
	req->action = MLAN_ACT_SET;
	sec->param.encrypt_mode = encrypt_mode;

	/* Send IOCTL request to MLAN */
	status = woal_request_ioctl(priv, req, wait_option);
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return status;
}

/**
 *  @brief Set wpa enable
 *
 *  @param priv                 A pointer to moal_private structure
 *  @param wait_option          Wait option
 *  @param enable               MTRUE or MFALSE
 *
 *  @return                     MLAN_STATUS_SUCCESS/MLAN_STATUS_PENDING --
 * success, otherwise fail
 */
mlan_status woal_set_wpa_enable(moal_private *priv, t_u8 wait_option,
				t_u32 enable)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_sec_cfg *sec = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;
	ENTER();

	/* Allocate an IOCTL request buffer */
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_sec_cfg));
	if (req == NULL) {
		status = MLAN_STATUS_FAILURE;
		goto done;
	}

	/* Fill request buffer */
	sec = (mlan_ds_sec_cfg *)req->pbuf;
	sec->sub_command = MLAN_OID_SEC_CFG_WPA_ENABLED;
	req->req_id = MLAN_IOCTL_SEC_CFG;
	req->action = MLAN_ACT_SET;
	sec->param.wpa_enabled = enable;

	/* Send IOCTL request to MLAN */
	status = woal_request_ioctl(priv, req, wait_option);
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return status;
}

/**
 *  @brief enable wep key
 *
 *  @param priv                 A pointer to moal_private structure
 *  @param wait_option          Wait option
 *
 *  @return                     MLAN_STATUS_SUCCESS/MLAN_STATUS_PENDING --
 * success, otherwise fail
 */
mlan_status woal_enable_wep_key(moal_private *priv, t_u8 wait_option)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_sec_cfg *sec = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;
	ENTER();

	/* Allocate an IOCTL request buffer */
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_sec_cfg));
	if (req == NULL) {
		status = MLAN_STATUS_FAILURE;
		goto done;
	}

	/* Fill request buffer */
	sec = (mlan_ds_sec_cfg *)req->pbuf;
	sec->sub_command = MLAN_OID_SEC_CFG_ENCRYPT_KEY;
	req->req_id = MLAN_IOCTL_SEC_CFG;
	req->action = MLAN_ACT_SET;
	sec->param.encrypt_key.key_disable = MFALSE;
	sec->param.encrypt_key.key_index = MLAN_KEY_INDEX_DEFAULT;
	sec->param.encrypt_key.is_current_wep_key = MTRUE;

	/* Send IOCTL request to MLAN */
	status = woal_request_ioctl(priv, req, wait_option);
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return status;
}

/**
 *  @brief Request user scan
 *
 *  @param priv                 A pointer to moal_private structure
 *  @param wait_option          Wait option
 *  @param scan_cfg             A pointer to wlan_user_scan_config structure
 *
 *  @return                     MLAN_STATUS_SUCCESS -- success, otherwise fail
 */
mlan_status woal_request_userscan(moal_private *priv, t_u8 wait_option,
				  wlan_user_scan_cfg *scan_cfg)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_ioctl_req *ioctl_req = NULL;
	mlan_ds_scan *scan = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;
	moal_handle *handle = priv->phandle;
	ENTER();

	if (MOAL_ACQ_SEMAPHORE_BLOCK(&handle->async_sem)) {
		PRINTM(MERROR, "Acquire semaphore error, request_scan\n");
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}
	handle->scan_pending_on_block = MTRUE;
	handle->scan_priv = priv;

	/* Allocate an IOCTL request buffer */
	ioctl_req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_scan) +
					      sizeof(wlan_user_scan_cfg));
	if (ioctl_req == NULL) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	scan = (mlan_ds_scan *)ioctl_req->pbuf;
	scan->sub_command = MLAN_OID_SCAN_USER_CONFIG;
	ioctl_req->req_id = MLAN_IOCTL_SCAN;
	ioctl_req->action = MLAN_ACT_SET;
	moal_memcpy_ext(handle, scan->param.user_scan.scan_cfg_buf, scan_cfg,
			sizeof(wlan_user_scan_cfg), sizeof(wlan_user_scan_cfg));
	/* Send IOCTL request to MLAN */
	status = woal_request_ioctl(priv, ioctl_req, wait_option);
	if (status == MLAN_STATUS_FAILURE) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(ioctl_req);
	else if (wait_option != MOAL_NO_WAIT) {
		PRINTM(MMSG, "scan interrupted by Signal, Cancel it...");
		woal_cancel_scan(priv, MOAL_IOCTL_WAIT_TIMEOUT);
	}
	if (ret == MLAN_STATUS_FAILURE) {
		handle->scan_pending_on_block = MFALSE;
		handle->scan_priv = NULL;
		MOAL_REL_SEMAPHORE(&handle->async_sem);
	}
	LEAVE();
	return ret;
}

/**
 *  @brief woal_get_scan_config
 *
 *  @param priv                 A pointer to moal_private structure
 *  @param scan_cfg             A pointer to scan_cfg structure
 *
 *  @return                     MLAN_STATUS_SUCCESS -- success, otherwise fail
 */
mlan_status woal_get_scan_config(moal_private *priv, mlan_scan_cfg *scan_cfg)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_ds_scan *scan = NULL;
	mlan_ioctl_req *req = NULL;

	ENTER();

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_scan));
	if (req == NULL) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	scan = (mlan_ds_scan *)req->pbuf;
	scan->sub_command = MLAN_OID_SCAN_CONFIG;
	req->req_id = MLAN_IOCTL_SCAN;
	req->action = MLAN_ACT_GET;
	memset(&scan->param.scan_cfg, 0, sizeof(mlan_scan_cfg));
	ret = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (ret == MLAN_STATUS_SUCCESS && scan_cfg)
		moal_memcpy_ext(priv->phandle, scan_cfg, &scan->param.scan_cfg,
				sizeof(mlan_scan_cfg), sizeof(mlan_scan_cfg));

done:
	if (ret != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief set scan time
 *
 *  @param priv                 A pointer to moal_private structure
 *  @param active_scan_time     Active scan time
 *  @param passive_scan_time    Passive scan time
 *  @param specific_scan_time   Specific scan time
 *
 *  @return                     MLAN_STATUS_SUCCESS -- success, otherwise fail
 */
mlan_status woal_set_scan_time(moal_private *priv, t_u16 active_scan_time,
			       t_u16 passive_scan_time,
			       t_u16 specific_scan_time)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_ds_scan *scan = NULL;
	mlan_ioctl_req *req = NULL;
	mlan_scan_cfg scan_cfg;

	ENTER();

	memset(&scan_cfg, 0, sizeof(scan_cfg));
	if (MLAN_STATUS_SUCCESS != woal_get_scan_config(priv, &scan_cfg)) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_scan));
	if (req == NULL) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	scan = (mlan_ds_scan *)req->pbuf;
	scan->sub_command = MLAN_OID_SCAN_CONFIG;
	req->req_id = MLAN_IOCTL_SCAN;
	req->action = MLAN_ACT_SET;
	memset(&scan->param.scan_cfg, 0, sizeof(mlan_scan_cfg));
	scan_cfg.scan_time.active_scan_time = active_scan_time;
	scan_cfg.scan_time.specific_scan_time = specific_scan_time;
	scan_cfg.scan_time.passive_scan_time = passive_scan_time;
	PRINTM(MIOCTL, "Set specific=%d, active=%d, passive=%d\n",
	       (int)active_scan_time, (int)passive_scan_time,
	       (int)specific_scan_time);
	moal_memcpy_ext(priv->phandle, &scan->param.scan_cfg, &scan_cfg,
			sizeof(mlan_scan_cfg), sizeof(mlan_scan_cfg));
	ret = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);

done:
	if (ret != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief request scan
 *
 *  @param priv                 A pointer to moal_private structure
 *  @param scan_cfg             A pointer to wlan_user_scan_cfg structure
 *
 *  @return                     MLAN_STATUS_SUCCESS -- success, otherwise fail
 */
mlan_status woal_do_scan(moal_private *priv, wlan_user_scan_cfg *scan_cfg)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	moal_handle *handle = priv->phandle;

	ENTER();
	if (handle->scan_pending_on_block == MTRUE) {
		PRINTM(MINFO, "scan already in processing...\n");
		LEAVE();
		return ret;
	}
#ifdef REASSOCIATION
	if (MOAL_ACQ_SEMAPHORE_BLOCK(&handle->reassoc_sem)) {
		PRINTM(MERROR, "Acquire semaphore error, woal_do_combo_scan\n");
		LEAVE();
		return -EBUSY;
	}
#endif /* REASSOCIATION */
	priv->report_scan_result = MTRUE;

	if (!scan_cfg)
		ret = woal_request_scan(priv, MOAL_NO_WAIT, NULL);
	else
		ret = woal_request_userscan(priv, MOAL_NO_WAIT, scan_cfg);

#ifdef REASSOCIATION
	MOAL_REL_SEMAPHORE(&handle->reassoc_sem);
#endif
	LEAVE();
	return ret;
}

/**
 *  @brief Cancel pending scan
 *
 *  @param priv                 A pointer to moal_private structure
 *  @return                     MLAN_STATUS_SUCCESS -- success, otherwise fail
 */
mlan_status woal_cancel_scan(moal_private *priv, t_u8 wait_option)
{
	mlan_ioctl_req *req = NULL;
	mlan_status ret = MLAN_STATUS_SUCCESS;
	moal_handle *handle = priv->phandle;
	moal_private *scan_priv = handle->scan_priv;
#ifdef STA_CFG80211
	unsigned long flags;
#endif

#ifdef STA_CFG80211
	// cancel scan timeout
	if (IS_STA_CFG80211(handle->params.cfg80211_wext) &&
	    handle->scan_request)
		cancel_delayed_work(&handle->scan_timeout_work);
#endif
	/* If scan is in process, cancel the scan command */
	if (!handle->scan_pending_on_block || !scan_priv) {
#ifdef STA_CFG80211
		spin_lock_irqsave(&handle->scan_req_lock, flags);
		if (IS_STA_CFG80211(handle->params.cfg80211_wext) &&
		    handle->scan_request) {
			/* some supplicant cannot handle SCAN abort event */
			if (scan_priv &&
			    (scan_priv->bss_type == MLAN_BSS_TYPE_STA))
				woal_cfg80211_scan_done(handle->scan_request,
							MTRUE);
			else
				woal_cfg80211_scan_done(handle->scan_request,
							MFALSE);
			handle->scan_request = NULL;
			handle->fake_scan_complete = MFALSE;
		}
		spin_unlock_irqrestore(&handle->scan_req_lock, flags);
#endif
		return ret;
	}
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_scan));
	if (req == NULL) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	req->req_id = MLAN_IOCTL_SCAN;
	req->action = MLAN_ACT_SET;
	((mlan_ds_scan *)req->pbuf)->sub_command = MLAN_OID_SCAN_CANCEL;
	ret = woal_request_ioctl(scan_priv, req, wait_option);
	handle->scan_pending_on_block = MFALSE;
	MOAL_REL_SEMAPHORE(&handle->async_sem);
#ifdef STA_CFG80211
	spin_lock_irqsave(&handle->scan_req_lock, flags);
	if (IS_STA_CFG80211(handle->params.cfg80211_wext) &&
	    handle->scan_request) {
		/** some supplicant can not handle SCAN abort event */
		if (scan_priv->bss_type == MLAN_BSS_TYPE_STA)
			woal_cfg80211_scan_done(handle->scan_request, MTRUE);
		else
			woal_cfg80211_scan_done(handle->scan_request, MFALSE);
		handle->scan_request = NULL;
	}
	spin_unlock_irqrestore(&handle->scan_req_lock, flags);
#endif
	/* add 300ms delay, incase firmware delay 0x7f event after scan cancel
	 * command response */
	woal_sched_timeout(300);
	// coverity[LOCK_EVASION:SUPPRESS]
	handle->scan_priv = NULL;
done:
	if (ret != MLAN_STATUS_PENDING)
		kfree(req);
	return ret;
}

/**
 *  @brief find ssid in scan_table
 *
 *  @param priv         A pointer to moal_private
 *  @param ssid_bssid   A pointer to mlan_ssid_bssid structure
 *
 *
 *  @return             MLAN_STATUS_SUCCESS/MLAN_STATUS_FAILURE
 */
int woal_find_essid(moal_private *priv, mlan_ssid_bssid *ssid_bssid,
		    t_u8 wait_option)
{
	int ret = 0;
	mlan_scan_resp scan_resp;
	wifi_timeval t;
	ENTER();

	memset(&scan_resp, 0, sizeof(scan_resp));

	if (MLAN_STATUS_SUCCESS !=
	    woal_get_scan_table(priv, wait_option, &scan_resp)) {
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}
#ifdef STA_CFG80211
	if (priv->ft_pre_connect || priv->ft_ie_len) {
		/** skip check the scan age out */
		ret = woal_find_best_network(priv, wait_option, ssid_bssid);
		LEAVE();
		return ret;
	}
#endif
	woal_get_monotonic_time(&t);
/** scan result timeout value */
#define SCAN_RESULT_AGEOUT 10
	if (t.time_sec > (scan_resp.age_in_secs + SCAN_RESULT_AGEOUT)) {
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}
	ret = woal_find_best_network(priv, wait_option, ssid_bssid);
	LEAVE();
	return ret;
}

/**
 * @brief                    auto reconnection configure
 *
 * @param priv               Pointer to moal_private structure
 * @param cfg_mode           configure mode
 * @param roam_offload_cfg   Pointer to woal_roam_offload_cfg structure
 *
 *  @return                  0-success, negative for failure.
 */
int woal_config_fw_roaming(moal_private *priv, t_u8 cfg_mode,
			   woal_roam_offload_cfg *roam_offload_cfg)
{
	mlan_ioctl_req *ioctl_req = NULL;
	mlan_ds_misc_cfg *misc = NULL;
	mlan_ds_misc_roam_offload *roam = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;
	int ret = 0;

	ENTER();

	if (!priv || !priv->phandle) {
		PRINTM(MERROR, "priv or handle is null\n");
		ret = -EFAULT;
		goto done;
	}

	ioctl_req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (ioctl_req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	misc = (mlan_ds_misc_cfg *)ioctl_req->pbuf;
	misc->sub_command = MLAN_OID_MISC_ROAM_OFFLOAD;
	ioctl_req->req_id = MLAN_IOCTL_MISC_CFG;

	roam = (mlan_ds_misc_roam_offload *)&misc->param.roam_offload;
	roam->aplist.ap_num = 0;
	ioctl_req->action = MLAN_ACT_SET;
	roam->enable = priv->phandle->fw_roam_enable;
	roam->config_mode = cfg_mode;

	if ((roam->config_mode == ROAM_OFFLOAD_ENABLE) && roam_offload_cfg) {
		roam->userset_passphrase = roam_offload_cfg->userset_passphrase;
		if (roam->userset_passphrase)
			roam->enable = 0;
	}
	if (roam->config_mode == ROAM_OFFLOAD_PARAM_CFG) {
		moal_memcpy_ext(priv->phandle, (t_u8 *)&roam->bssid_reconnect,
				(t_u8 *)&roam_offload_cfg->bssid,
				MLAN_MAC_ADDR_LENGTH,
				sizeof(roam->bssid_reconnect));
		if (roam_offload_cfg->ssid_list.ssid_num) {
			moal_memcpy_ext(priv->phandle, (t_u8 *)&roam->ssid_list,
					(t_u8 *)&roam_offload_cfg->ssid_list,
					sizeof(mlan_ds_misc_ssid_list),
					sizeof(mlan_ds_misc_ssid_list));
		}
		if (roam_offload_cfg->black_list.ap_num) {
			moal_memcpy_ext(
				priv->phandle, (t_u8 *)&roam->black_list,
				(t_u8 *)&roam_offload_cfg->black_list,
				sizeof(mlan_ds_misc_roam_offload_aplist),
				sizeof(mlan_ds_misc_roam_offload_aplist));
		}
		roam->trigger_condition = roam_offload_cfg->trigger_condition;
		roam->retry_count = roam_offload_cfg->retry_count;
		if (roam_offload_cfg->rssi_param_set_flag) {
			roam->para_rssi.set_flag = 1;
			roam->para_rssi.max_rssi = roam_offload_cfg->max_rssi;
			roam->para_rssi.min_rssi = roam_offload_cfg->min_rssi;
			roam->para_rssi.step_rssi = roam_offload_cfg->step_rssi;
		}
		if (roam_offload_cfg->band_rssi_flag) {
			roam->band_rssi_flag = roam_offload_cfg->band_rssi_flag;
			moal_memcpy_ext(priv->phandle, (t_u8 *)&roam->band_rssi,
					(t_u8 *)&roam_offload_cfg->band_rssi,
					sizeof(mlan_ds_misc_band_rssi),
					sizeof(mlan_ds_misc_band_rssi));
		}
		if (roam_offload_cfg->bgscan_set_flag) {
			roam->bgscan_set_flag =
				roam_offload_cfg->bgscan_set_flag;
			moal_memcpy_ext(priv->phandle,
					(t_u8 *)&roam->bgscan_cfg,
					(t_u8 *)&roam_offload_cfg->bgscan_cfg,
					sizeof(mlan_ds_misc_bgscan_cfg),
					sizeof(mlan_ds_misc_bgscan_cfg));
		}
		if (roam_offload_cfg->ees_param_set_flag) {
			roam->ees_param_set_flag =
				roam_offload_cfg->ees_param_set_flag;
			moal_memcpy_ext(priv->phandle, (t_u8 *)&roam->ees_cfg,
					(t_u8 *)&roam_offload_cfg->ees_cfg,
					sizeof(mlan_ds_misc_ees_cfg),
					sizeof(mlan_ds_misc_ees_cfg));
		}
		roam->bcn_miss_threshold = roam_offload_cfg->bcn_miss_threshold;
		roam->pre_bcn_miss_threshold =
			roam_offload_cfg->pre_bcn_miss_threshold;
		roam->repeat_count = roam_offload_cfg->repeat_count;
	}
	if (roam->config_mode == ROAM_OFFLOAD_SUSPEND_CFG) {
		moal_memcpy_ext(priv->phandle, roam->bssid_reconnect,
				priv->phandle->auto_reconnect_bssid,
				MLAN_MAC_ADDR_LENGTH,
				sizeof(roam->bssid_reconnect));
		roam->ssid_list.ssid_num = 1;
		moal_memcpy_ext(
			priv->phandle, (t_u8 *)&roam->ssid_list.ssids[0].ssid,
			(t_u8 *)&priv->phandle->auto_reconnect_ssid.ssid,
			priv->phandle->auto_reconnect_ssid.ssid_len,
			MLAN_MAX_SSID_LENGTH);
		roam->retry_count = priv->phandle->auto_reconnect_retry_count;
	}

	status = woal_request_ioctl(priv, ioctl_req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(ioctl_req);

	LEAVE();
	return ret;
}

/**
 *  @brief Request user scan
 *
 *  @param priv                 A pointer to moal_private structure
 *  @param wait_option          Wait option
 *  @param scan_cfg             A pointer to wlan_bgscan_cfg structure
 *
 *  @return                     MLAN_STATUS_SUCCESS -- success, otherwise fail
 */
mlan_status woal_request_bgscan(moal_private *priv, t_u8 wait_option,
				wlan_bgscan_cfg *scan_cfg)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_ioctl_req *ioctl_req = NULL;
	mlan_ds_scan *scan = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;
	ENTER();

	/* Allocate an IOCTL request buffer */
	ioctl_req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_scan) +
					      sizeof(wlan_bgscan_cfg));
	if (ioctl_req == NULL) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	scan = (mlan_ds_scan *)ioctl_req->pbuf;
	scan->sub_command = MLAN_OID_SCAN_BGSCAN_CONFIG;
	ioctl_req->req_id = MLAN_IOCTL_SCAN;
	ioctl_req->action = MLAN_ACT_SET;
	moal_memcpy_ext(priv->phandle, scan->param.user_scan.scan_cfg_buf,
			scan_cfg, sizeof(wlan_bgscan_cfg),
			sizeof(wlan_bgscan_cfg));

	/* Send IOCTL request to MLAN */
	status = woal_request_ioctl(priv, ioctl_req, wait_option);
	if (status == MLAN_STATUS_FAILURE) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(ioctl_req);
	LEAVE();
	return ret;
}

/**
 *  @brief set bgscan config
 *
 *  @param priv                 A pointer to moal_private structure
 *  @param buf                  A pointer to scan command buf
 *  @param length               buf length
 *
 *  @return                     MLAN_STATUS_SUCCESS -- success, otherwise fail
 */
mlan_status woal_set_bg_scan(moal_private *priv, char *buf, int length)
{
	t_u8 *ptr = buf + strlen("BGSCAN-CONFIG") + 1;
	int buf_left = length - (strlen("BGSCAN-CONFIG") + 1);
	int band = 0;
	int num_ssid = 0;
	int ssid_len = 0;
	mlan_status ret = MLAN_STATUS_FAILURE;

	ENTER();
	memset(&priv->scan_cfg, 0, sizeof(priv->scan_cfg));
	priv->scan_cfg.report_condition =
		BG_SCAN_SSID_MATCH | BG_SCAN_WAIT_ALL_CHAN_DONE;
	while (buf_left >= 2) {
		switch (*ptr) {
		case WEXT_CSCAN_SSID_SECTION:
			ssid_len = *(ptr + 1);
			if ((buf_left < (ssid_len + 2)) ||
			    (ssid_len > MLAN_MAX_SSID_LENGTH)) {
				PRINTM(MERROR,
				       "Invalid ssid, buf_left=%d, ssid_len=%d\n",
				       buf_left, ssid_len);
				buf_left = 0;
				break;
			}
			if (ssid_len &&
			    (num_ssid < (MRVDRV_MAX_SSID_LIST_LENGTH - 1))) {
				strncpy(priv->scan_cfg.ssid_list[num_ssid].ssid,
					ptr + 2, ssid_len);
				priv->scan_cfg.ssid_list[num_ssid].max_len = 0;
				PRINTM(MIOCTL, "BG scan: ssid=%s\n",
				       priv->scan_cfg.ssid_list[num_ssid].ssid);
				num_ssid++;
			}
			buf_left -= ssid_len + 2;
			ptr += ssid_len + 2;
			break;
		case WEXT_BGSCAN_RSSI_SECTION:
			priv->scan_cfg.report_condition =
				BG_SCAN_SSID_RSSI_MATCH |
				BG_SCAN_WAIT_ALL_CHAN_DONE;
			priv->scan_cfg.rssi_threshold = ptr[1];
			PRINTM(MIOCTL, "BG scan: rssi_threshold=%d\n",
			       (int)priv->scan_cfg.rssi_threshold);
			ptr += 2;
			buf_left -= 2;
			break;
		case WEXT_BGSCAN_REPEAT_SECTION:
			priv->scan_cfg.repeat_count = (t_u16)ptr[1];
			PRINTM(MIOCTL, "BG scan: repeat_count=%d\n",
			       (int)priv->scan_cfg.repeat_count);
			ptr += 2;
			buf_left -= 2;
			break;
		case WEXT_BGSCAN_INTERVAL_SECTION:
			if (buf_left < 3) {
				PRINTM(MERROR,
				       "Invalid scan_interval, buf_left=%d\n",
				       buf_left);
				buf_left = 0;
				break;
			}
			priv->scan_cfg.scan_interval =
				(ptr[2] << 8 | ptr[1]) * 1000;
			PRINTM(MIOCTL, "BG scan: scan_interval=%d\n",
			       (int)priv->scan_cfg.scan_interval);
			ptr += 3;
			buf_left -= 3;
			break;
		default:
			buf_left = 0;
			break;
		}
	}
	/** set bgscan when ssid_num > 0 */
	if (num_ssid) {
		if (MLAN_STATUS_SUCCESS != woal_get_band(priv, &band)) {
			ret = MLAN_STATUS_FAILURE;
			goto done;
		}
		switch (band) {
		case WIFI_FREQUENCY_BAND_2GHZ:
			priv->scan_cfg.chan_list[0].radio_type =
				0 | BAND_SPECIFIED;
			break;
		case WIFI_FREQUENCY_BAND_5GHZ:
			priv->scan_cfg.chan_list[0].radio_type =
				1 | BAND_SPECIFIED;
			break;
		default:
			break;
		}
		priv->scan_cfg.bss_type = MLAN_BSS_MODE_INFRA;
		priv->scan_cfg.action = BG_SCAN_ACT_SET;
		priv->scan_cfg.enable = MTRUE;
		moal_memcpy_ext(priv->phandle, priv->scan_cfg.random_mac,
				priv->random_mac, ETH_ALEN,
				sizeof(priv->scan_cfg.random_mac));
		ret = woal_request_bgscan(priv, MOAL_IOCTL_WAIT,
					  &priv->scan_cfg);
	}
done:
	LEAVE();
	return ret;
}

#ifdef STA_CFG80211
/**
 *  @brief set bgscan and new rssi_low_threshold
 *
 *  @param priv                 A pointer to moal_private structure
 *  @param set_rssi             flag for set rssi_low_threshold
 *
 *  @return                     N/A
 */
void woal_config_bgscan_and_rssi(moal_private *priv, t_u8 set_rssi)
{
	char rssi_low[11];
	mlan_bss_info bss_info;
	int band = 0;

	ENTER();
	if (!priv->rssi_low) {
		LEAVE();
		return;
	}
	memset(&bss_info, 0, sizeof(bss_info));
	if (MLAN_STATUS_SUCCESS !=
	    woal_get_bss_info(priv, MOAL_IOCTL_WAIT, &bss_info)) {
		PRINTM(MIOCTL, "Get BSS Info Failed\n");
		LEAVE();
		return;
	}
	if (!bss_info.media_connected) {
		PRINTM(MIOCTL, "We already lost connection\n");
		LEAVE();
		return;
	}
	memset(&priv->scan_cfg, 0, sizeof(priv->scan_cfg));
	strncpy(priv->scan_cfg.ssid_list[0].ssid, bss_info.ssid.ssid,
		bss_info.ssid.ssid_len);
	priv->scan_cfg.ssid_list[0].max_len = 0;

	priv->scan_cfg.report_condition =
		BG_SCAN_SSID_RSSI_MATCH | BG_SCAN_WAIT_ALL_CHAN_DONE;
	priv->scan_cfg.rssi_threshold = priv->rssi_low - RSSI_HYSTERESIS;
	priv->scan_cfg.repeat_count = DEF_REPEAT_COUNT;
	priv->scan_cfg.scan_interval = MIN_BGSCAN_INTERVAL;
	if (MLAN_STATUS_SUCCESS != woal_get_band(priv, &band)) {
		PRINTM(MERROR, "woal get band fail\n");
		LEAVE();
		return;
	}
	switch (band) {
	case WIFI_FREQUENCY_BAND_2GHZ:
		priv->scan_cfg.chan_list[0].radio_type = 0 | BAND_SPECIFIED;
		break;
	case WIFI_FREQUENCY_BAND_5GHZ:
		priv->scan_cfg.chan_list[0].radio_type = 1 | BAND_SPECIFIED;
		break;
	default:
		break;
	}
	priv->scan_cfg.bss_type = MLAN_BSS_MODE_INFRA;
	priv->scan_cfg.action = BG_SCAN_ACT_SET;
	priv->scan_cfg.enable = MTRUE;
	moal_memcpy_ext(priv->phandle, priv->scan_cfg.random_mac,
			priv->random_mac, ETH_ALEN,
			sizeof(priv->scan_cfg.random_mac));
	if (MLAN_STATUS_FAILURE ==
	    woal_request_bgscan(priv, MOAL_NO_WAIT, &priv->scan_cfg)) {
		PRINTM(MERROR, "request bgscan fail\n");
		LEAVE();
		return;
	}
	if (set_rssi &&
	    ((priv->rssi_low + RSSI_HYSTERESIS) <= LOWEST_RSSI_THRESHOLD)) {
		priv->rssi_low += RSSI_HYSTERESIS;
		snprintf(rssi_low, sizeof(rssi_low), "%d", priv->rssi_low);
		if (MLAN_STATUS_FAILURE ==
		    woal_set_rssi_low_threshold(priv, rssi_low, MOAL_NO_WAIT))
			PRINTM(MERROR, "set_rssi_low_threshold fail\n");
	}
	LEAVE();
}
#endif

/**
 *  @brief stop bg scan
 *
 *  @param priv                 A pointer to moal_private structure
 *  @param wait_option          wait option
 *
 *  @return                     MLAN_STATUS_SUCCESS -- success, otherwise fail
 */
mlan_status woal_stop_bg_scan(moal_private *priv, t_u8 wait_option)
{
	wlan_bgscan_cfg *scan_cfg;
	mlan_status ret = MLAN_STATUS_SUCCESS;
	ENTER();

	scan_cfg = kmalloc(sizeof(wlan_bgscan_cfg), GFP_KERNEL);
	if (!scan_cfg) {
		LEAVE();
		return ret;
	}
	memset(scan_cfg, 0, sizeof(wlan_bgscan_cfg));
	scan_cfg->action = BG_SCAN_ACT_SET;
	scan_cfg->enable = MFALSE;
	ret = woal_request_bgscan(priv, wait_option, scan_cfg);
	kfree(scan_cfg);
	LEAVE();
	return ret;
}

/**
 *  @brief set bgscan config
 *
 *  @param handle               A pointer to moal_handle structure
 *
 *  @return                     MLAN_STATUS_SUCCESS -- success, otherwise fail
 */
void woal_reconfig_bgscan(moal_handle *handle)
{
	int i;
	for (i = 0; i < MIN(handle->priv_num, MLAN_MAX_BSS_NUM); i++) {
		if (handle->priv[i] &&
		    (GET_BSS_ROLE(handle->priv[i]) == MLAN_BSS_ROLE_STA)) {
			if (handle->priv[i]->bg_scan_start &&
			    handle->priv[i]->bg_scan_reported) {
				PRINTM(MIOCTL, "Reconfig BGSCAN\n");
				woal_request_bgscan(handle->priv[i],
						    MOAL_NO_WAIT,
						    &handle->priv[i]->scan_cfg);
				handle->priv[i]->bg_scan_reported = MFALSE;
			}
		}
	}
}

/**
 *  @brief set rssi low threshold
 *
 *  @param priv                 A pointer to moal_private structure
 *  @param rssi                 A pointer to low rssi
 *  @param wait_option          Wait option
 *
 *  @return                     MLAN_STATUS_SUCCESS -- success, otherwise fail
 */
mlan_status woal_set_rssi_low_threshold(moal_private *priv, char *rssi,
					t_u8 wait_option)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_ioctl_req *req = NULL;
	mlan_ds_misc_cfg *misc = NULL;
	int low_rssi = 0;

	ENTER();
	if (priv->media_connected == MFALSE)
		goto done;

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	misc = (mlan_ds_misc_cfg *)req->pbuf;
	req->req_id = MLAN_IOCTL_MISC_CFG;
	misc->sub_command = MLAN_OID_MISC_SUBSCRIBE_EVENT;
	req->action = MLAN_ACT_SET;
	misc->param.subscribe_event.evt_action = SUBSCRIBE_EVT_ACT_BITWISE_SET;
	misc->param.subscribe_event.evt_bitmap = SUBSCRIBE_EVT_RSSI_LOW;
	misc->param.subscribe_event.evt_bitmap |= SUBSCRIBE_EVT_PRE_BEACON_LOST;
	misc->param.subscribe_event.pre_beacon_miss = DEFAULT_PRE_BEACON_MISS;

	if (MLAN_STATUS_SUCCESS != woal_atoi(&low_rssi, rssi)) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
#ifdef STA_CFG80211
	priv->mrvl_rssi_low = low_rssi;
#endif
	misc->param.subscribe_event.low_rssi = low_rssi;
	misc->param.subscribe_event.low_rssi_freq = 0;
	ret = woal_request_ioctl(priv, req, wait_option);
	if (ret == MLAN_STATUS_FAILURE) {
		PRINTM(MERROR, "request set rssi_low_threshold fail!\n");
		goto done;
	}
done:
	if (ret != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

#if defined(STA_CFG80211)
/**
 *  @brief set rssi low threshold
 *
 *  @param priv                 A pointer to moal_private structure
 *  @param event_id             event id.
 *  @param wait_option          wait option
 *
 *  @return                     MLAN_STATUS_SUCCESS -- success, otherwise fail
 */
mlan_status woal_set_rssi_threshold(moal_private *priv, t_u32 event_id,
				    t_u8 wait_option)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_ioctl_req *req = NULL;
	mlan_ds_misc_cfg *misc = NULL;

	ENTER();

	if (priv->mrvl_rssi_low || !priv->cqm_rssi_thold)
		goto done;
	if (event_id == MLAN_EVENT_ID_FW_BCN_RSSI_LOW) {
		if (priv->last_rssi_low < 100)
			priv->last_rssi_low += priv->cqm_rssi_hyst;
		priv->last_rssi_high = abs(priv->cqm_rssi_high_thold);
	} else if (event_id == MLAN_EVENT_ID_FW_BCN_RSSI_HIGH) {
		priv->last_rssi_low = abs(priv->cqm_rssi_thold);
		if (priv->last_rssi_high > priv->cqm_rssi_hyst)
			priv->last_rssi_high -= priv->cqm_rssi_hyst;
	} else {
		priv->last_rssi_low = abs(priv->cqm_rssi_thold);
		priv->last_rssi_high = abs(priv->cqm_rssi_high_thold);
	}

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}
	misc = (mlan_ds_misc_cfg *)req->pbuf;
	req->req_id = MLAN_IOCTL_MISC_CFG;
	misc->sub_command = MLAN_OID_MISC_SUBSCRIBE_EVENT;
	req->action = MLAN_ACT_SET;
	if (!event_id && !priv->cqm_rssi_thold && !priv->cqm_rssi_hyst)
		misc->param.subscribe_event.evt_action =
			SUBSCRIBE_EVT_ACT_BITWISE_CLR;
	else
		misc->param.subscribe_event.evt_action =
			SUBSCRIBE_EVT_ACT_BITWISE_SET;
	misc->param.subscribe_event.evt_bitmap =
		SUBSCRIBE_EVT_RSSI_LOW | SUBSCRIBE_EVT_RSSI_HIGH;
	misc->param.subscribe_event.low_rssi_freq = 0;
	misc->param.subscribe_event.low_rssi = priv->last_rssi_low;
	misc->param.subscribe_event.high_rssi_freq = 0;
	misc->param.subscribe_event.high_rssi = priv->last_rssi_high;
	PRINTM(MIOCTL, "rssi_low=%d, rssi_high=%d action=%d\n",
	       (int)priv->last_rssi_low, (int)priv->last_rssi_high,
	       misc->param.subscribe_event.evt_action);
	ret = woal_request_ioctl(priv, req, wait_option);
done:
	if (ret != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}
#endif

/**
 *  @brief  Get power mode
 *
 *  @param priv                 A pointer to moal_private structure
 *  @param powermode            A pointer to powermode buf
 *
 *  @return                     MLAN_STATUS_SUCCESS -- success, otherwise fail
 */
mlan_status woal_get_powermode(moal_private *priv, int *powermode)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	int ps_mode;

	ENTER();

	if (MLAN_STATUS_SUCCESS != woal_set_get_power_mgmt(priv, MLAN_ACT_GET,
							   &ps_mode, 0,
							   MOAL_IOCTL_WAIT)) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	if (ps_mode)
		*powermode = MFALSE;
	else
		*powermode = MTRUE;

done:
	LEAVE();
	return ret;
}

/**
 *  @brief set scan type
 *
 *  @param priv                 A pointer to moal_private structure
 *  @param scan_type            MLAN_SCAN_TYPE_ACTIVE/MLAN_SCAN_TYPE_PASSIVE
 *
 *  @return                     MLAN_STATUS_SUCCESS -- success, otherwise fail
 */
mlan_status woal_set_scan_type(moal_private *priv, t_u32 scan_type)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_ds_scan *scan = NULL;
	mlan_ioctl_req *req = NULL;
	mlan_scan_cfg scan_cfg;

	ENTER();
	memset(&scan_cfg, 0, sizeof(scan_cfg));
	if (MLAN_STATUS_SUCCESS != woal_get_scan_config(priv, &scan_cfg)) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_scan));
	if (req == NULL) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	scan = (mlan_ds_scan *)req->pbuf;
	scan->sub_command = MLAN_OID_SCAN_CONFIG;
	req->req_id = MLAN_IOCTL_SCAN;
	req->action = MLAN_ACT_SET;
	memset(&scan->param.scan_cfg, 0, sizeof(mlan_scan_cfg));
	scan_cfg.scan_type = scan_type;
	PRINTM(MIOCTL, "Set scan_type=%d\n", (int)scan_type);
	moal_memcpy_ext(priv->phandle, &scan->param.scan_cfg, &scan_cfg,
			sizeof(mlan_scan_cfg), sizeof(mlan_scan_cfg));
	ret = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);

done:
	if (ret != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief enable/disable ext_scan
 *
 *  @param priv                 A pointer to moal_private structure
 *  @param enable               MTRUE -- enable, MFALSE --disable
 *
 *  @return                     MLAN_STATUS_SUCCESS -- success, otherwise fail
 */
mlan_status woal_enable_ext_scan(moal_private *priv, t_u8 enable)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_ds_scan *scan = NULL;
	mlan_ioctl_req *req = NULL;
	mlan_scan_cfg scan_cfg;

	ENTER();
	memset(&scan_cfg, 0, sizeof(scan_cfg));
	if (MLAN_STATUS_SUCCESS != woal_get_scan_config(priv, &scan_cfg)) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_scan));
	if (req == NULL) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	scan = (mlan_ds_scan *)req->pbuf;
	scan->sub_command = MLAN_OID_SCAN_CONFIG;
	req->req_id = MLAN_IOCTL_SCAN;
	req->action = MLAN_ACT_SET;
	memset(&scan->param.scan_cfg, 0, sizeof(mlan_scan_cfg));
	scan_cfg.ext_scan = enable;
	PRINTM(MIOCTL, "Set ext_scan=%d\n", (int)enable);
	moal_memcpy_ext(priv->phandle, &scan->param.scan_cfg, &scan_cfg,
			sizeof(mlan_scan_cfg), sizeof(mlan_scan_cfg));
	ret = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);

done:
	if (ret != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Set FILS PSK
 *
 *  @param priv                 A pointer to moal_private structure
 *  @param data          	 A pointer to a buffer
 *
 *  @return                     MLAN_STATUS_SUCCESS/MLAN_STATUS_PENDING --
 * success, otherwise fail
 */
mlan_status woal_set_fils_psk(moal_private *priv, char *data)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_802_11_mac_addr mac = {0x00};
	t_u8 key[MLAN_MAX_KEY_LENGTH] = {0};
	char *begin, *end, *opt;

	ENTER();
	begin = data;

	/* Parse command arguments */
	while (begin) {
		end = woal_strsep(&begin, ';', '/');
		if (!end || !end[0])
			break;

		opt = woal_strsep(&end, '=', '/');
		if (!opt || !end) {
			PRINTM(MERROR, "Invalid option\n");
			LEAVE();
			return -EINVAL;
		}

		if (!strncmp(opt, FILS_KEY, strlen(FILS_KEY))) {
			memset(key, 0, sizeof(key));
			woal_ascii2hex((t_u8 *)key, end, MLAN_MAX_KEY_LENGTH);
		} else if (!strncmp(opt, FILS_BSSID, strlen(FILS_BSSID))) {
			woal_mac2u8(mac, end);
		}
	}

	ret = woal_set_psk_11ai(priv, MOAL_IOCTL_WAIT, (t_u8 *)mac, (t_u8 *)key,
				MLAN_MAX_KEY_LENGTH);

	priv->enable_fils = MTRUE;
	return ret;
}

#ifdef UAP_SUPPORT
/**
 *  @brief Set FILS IP Config
 *
 *  @param priv                 A pointer to moal_private structure
 *  @param data                 A pointer to a buffer
 *
 *  @return                     MLAN_STATUS_SUCCESS on success, otherwise fail
 */
mlan_status woal_set_fils_ip_cfg(moal_private *priv, char *data)
{
	char *begin, *end, *opt;
	ip_addr_cfg_t ip_cfg;
	int maxClients = 0;
	u8 ip[MAX_IPADDR];
	u32 ip_address;
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_ioctl_req *req = NULL;
	mlan_ds_bss *bss = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();
	begin = data;

	memset(&ip_cfg, 0, sizeof(ip_cfg));

	/* Parse command arguments */
	while (begin) {
		end = woal_strsep(&begin, ';', '/');
		if (!end || !end[0])
			break;

		opt = woal_strsep(&end, '=', '/');
		if (!opt || !end) {
			PRINTM(MERROR, "Invalid option\n");
			LEAVE();
			return -EINVAL;
		}

		if (!strncmp(opt, FILS_COUNT_STR, strlen(FILS_COUNT_STR))) {
			maxClients = ip_cfg.maxClients;
			if (woal_atoi(&maxClients, end) !=
			    MLAN_STATUS_SUCCESS) {
				LEAVE();
				return -EINVAL;
			}
			continue;
		}

		/* Parse and store the IP address */
		in4_pton(end, strlen(end), ip, ' ', NULL);

		ip_address =
			ip[3] | (ip[2] << 8) | (ip[1] << 16) | (ip[0] << 24);

		if (!strncmp(opt, FILS_IP_STR, strlen(FILS_IP_STR))) {
			ip_cfg.ip = ip_address;
		} else if (!strncmp(opt, FILS_MASK_STR,
				    strlen(FILS_MASK_STR))) {
			ip_cfg.subnetMask = ip_address;
		} else if (!strncmp(opt, FILS_BASE_IP_STR,
				    strlen(FILS_BASE_IP_STR))) {
			ip_cfg.baseIp = ip_address;
		} else if (!strncmp(opt, FILS_DNS_STR, strlen(FILS_DNS_STR))) {
			ip_cfg.dnsIp = ip_address;
		}
	}

	/* Allocate an IOCTL request buffer */
	req = (mlan_ioctl_req *)woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_bss));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	/* Fill request buffer */
	bss = (mlan_ds_bss *)req->pbuf;
	bss->sub_command = MLAN_OID_FILS_IP_CFG;
	req->req_id = MLAN_IOCTL_BSS;
	req->action = MLAN_ACT_SET;

	/* Fill IP configuration */
	moal_memcpy_ext(priv->phandle, &(bss->param.bss_config.ip_cfg), &ip_cfg,
			sizeof(ip_cfg), sizeof(bss->param.bss_config.ip_cfg));

	/* Send IOCTL request to MLAN */
	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	priv->enable_fils = MTRUE;

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}
#endif

/**
 *  @brief set power mode
 *
 *  @param priv                 A pointer to moal_private structure
 *  @param powermode            A pointer to powermode string.
 *
 *  @return                     MLAN_STATUS_SUCCESS -- success, otherwise fail
 */
mlan_status woal_set_powermode(moal_private *priv, char *powermode)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	int disabled;

	ENTER();

	if (*powermode == '1') {
		PRINTM(MIOCTL, "Disable power save\n");
		disabled = 1;
	} else if (*powermode == '0') {
		PRINTM(MIOCTL, "Enable power save\n");
		disabled = 0;
	} else {
		PRINTM(MERROR, "unsupported power mode\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	if (MLAN_STATUS_SUCCESS != woal_set_get_power_mgmt(priv, MLAN_ACT_SET,
							   &disabled, 0,
							   MOAL_IOCTL_WAIT))
		ret = MLAN_STATUS_FAILURE;

done:
	LEAVE();
	return ret;
}

/**
 *  @brief set combo scan
 *
 *  @param priv                 A pointer to moal_private structure
 *  @param buf                  A pointer to scan command buf
 *  @param length               buf length
 *
 *  @return                     0 -- success, otherwise fail
 */
int woal_set_combo_scan(moal_private *priv, char *buf, int length)
{
	int ret = 0;
	wlan_user_scan_cfg *scan_cfg;
	t_u8 *ptr = buf + WEXT_CSCAN_HEADER_SIZE;
	int buf_left = length - WEXT_CSCAN_HEADER_SIZE;
	int num_ssid = 0;
	int num_chan = 0;
	int ssid_len = 0;
	int i = 0;
	t_u16 passive_scan_time = 0;
	t_u16 specific_scan_time = 0;

	ENTER();

	scan_cfg = (wlan_user_scan_cfg *)kmalloc(sizeof(wlan_user_scan_cfg),
						 GFP_KERNEL);
	if (!scan_cfg) {
		PRINTM(MERROR, "Malloc buffer failed\n");
		LEAVE();
		return -ENOMEM;
	}

	memset(scan_cfg, 0, sizeof(wlan_user_scan_cfg));
	while (buf_left >= 2) {
		switch (*ptr) {
		case WEXT_CSCAN_SSID_SECTION:
			ssid_len = *(ptr + 1);
			if ((buf_left < (ssid_len + 2)) ||
			    (ssid_len > MLAN_MAX_SSID_LENGTH)) {
				PRINTM(MERROR,
				       "Invalid ssid, buf_left=%d, ssid_len=%d\n",
				       buf_left, ssid_len);
				buf_left = 0;
				break;
			}
			if (ssid_len &&
			    (num_ssid < (MRVDRV_MAX_SSID_LIST_LENGTH - 1))) {
				strncpy(scan_cfg->ssid_list[num_ssid].ssid,
					ptr + 2, ssid_len);
				scan_cfg->ssid_list[num_ssid].max_len = 0;
				PRINTM(MIOCTL, "Combo scan: ssid=%s\n",
				       scan_cfg->ssid_list[num_ssid].ssid);
				num_ssid++;
			}
			buf_left -= ssid_len + 2;
			ptr += ssid_len + 2;
			break;
		case WEXT_CSCAN_CHANNEL_SECTION:
			num_chan = ptr[1];
			if ((buf_left < (num_chan + 2)) ||
			    (num_chan > WLAN_USER_SCAN_CHAN_MAX)) {
				PRINTM(MERROR,
				       "Invalid channel list, buf_left=%d, num_chan=%d\n",
				       buf_left, num_chan);
				buf_left = 0;
				break;
			}
			for (i = 0; i < num_chan; i++) {
				scan_cfg->chan_list[i].chan_number = ptr[2 + i];
				PRINTM(MIOCTL, "Combo scan: chan=%d\n",
				       scan_cfg->chan_list[i].chan_number);
			}
			buf_left -= 2 + num_chan;
			ptr += 2 + num_chan;
			break;
		case WEXT_CSCAN_PASV_DWELL_SECTION:
			if (buf_left < 3) {
				PRINTM(MERROR,
				       "Invalid PASV_DWELL_SECTION, buf_left=%d\n",
				       buf_left);
				buf_left = 0;
				break;
			}
			passive_scan_time = ptr[2] << 8 | ptr[1];
			ptr += 3;
			buf_left -= 3;
			break;
		case WEXT_CSCAN_HOME_DWELL_SECTION:
			if (buf_left < 3) {
				PRINTM(MERROR,
				       "Invalid HOME_DWELL_SECTION, buf_left=%d\n",
				       buf_left);
				buf_left = 0;
				break;
			}
			specific_scan_time = ptr[2] << 8 | ptr[1];
			ptr += 3;
			buf_left -= 3;
			break;
		default:
			buf_left = 0;
			break;
		}
	}
	if (passive_scan_time || specific_scan_time) {
		PRINTM(MIOCTL,
		       "Set passive_scan_time=%d specific_scan_time=%d\n",
		       passive_scan_time, specific_scan_time);
		if (MLAN_STATUS_FAILURE ==
		    woal_set_scan_time(priv, 0, passive_scan_time,
				       specific_scan_time)) {
			ret = -EFAULT;
			goto done;
		}
	}
	if (num_ssid || num_chan) {
		if (num_ssid) {
			/* Add broadcast scan to ssid_list */
			scan_cfg->ssid_list[num_ssid].max_len = 0xff;
			if (priv->scan_type == MLAN_SCAN_TYPE_PASSIVE)
				woal_set_scan_type(priv, MLAN_SCAN_TYPE_ACTIVE);
		}
		if (MLAN_STATUS_FAILURE == woal_do_scan(priv, scan_cfg))
			ret = -EFAULT;
		if (num_ssid && (priv->scan_type == MLAN_SCAN_TYPE_PASSIVE))
			woal_set_scan_type(priv, MLAN_SCAN_TYPE_PASSIVE);
	} else {
		/* request broadcast scan */
		if (MLAN_STATUS_FAILURE == woal_do_scan(priv, NULL))
			ret = -EFAULT;
	}
done:
	kfree(scan_cfg);
	LEAVE();
	return ret;
}

/**
 *  @brief  Get band
 *
 *  @param priv                 A pointer to moal_private structure
 *  @param band                 A pointer to band buf
 *
 *  @return                     MLAN_STATUS_SUCCESS -- success, otherwise fail
 */
mlan_status woal_get_band(moal_private *priv, int *band)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_ioctl_req *req = NULL;
	mlan_ds_radio_cfg *radio_cfg = NULL;
	int support_band = 0;

	ENTER();

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_radio_cfg));
	if (req == NULL) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	radio_cfg = (mlan_ds_radio_cfg *)req->pbuf;
	radio_cfg->sub_command = MLAN_OID_BAND_CFG;
	req->req_id = MLAN_IOCTL_RADIO_CFG;
	/* Get config_bands, adhoc_start_band and adhoc_channel values from MLAN
	 */
	req->action = MLAN_ACT_GET;
	ret = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (ret != MLAN_STATUS_SUCCESS)
		goto done;

	if (radio_cfg->param.band_cfg.config_bands &
	    (BAND_B | BAND_G | BAND_GN))
		support_band |= WIFI_FREQUENCY_BAND_2GHZ;
	if (radio_cfg->param.band_cfg.config_bands & (BAND_A | BAND_AN))
		support_band |= WIFI_FREQUENCY_BAND_5GHZ;
	*band = support_band;
	if (support_band == WIFI_FREQUENCY_ALL_BAND)
		*band = WIFI_FREQUENCY_BAND_AUTO;
done:
	if (ret != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief set band
 *
 *  @param priv             A pointer to moal_private structure
 *  @param pband            A pointer to band string.
 *
 *  @return                 MLAN_STATUS_SUCCESS -- success, otherwise fail
 */
mlan_status woal_set_band(moal_private *priv, char *pband)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	int band = 0;
	mlan_ioctl_req *req = NULL;
	mlan_ds_radio_cfg *radio_cfg = NULL;
#if defined(STA_CFG80211) || defined(UAP_CFG80211)
	int cfg80211_wext = priv->phandle->params.cfg80211_wext;
#endif

	ENTER();
	if (priv->media_connected == MTRUE) {
		PRINTM(MERROR, "Set band is not allowed in connected state\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_radio_cfg));
	if (req == NULL) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	radio_cfg = (mlan_ds_radio_cfg *)req->pbuf;
	radio_cfg->sub_command = MLAN_OID_BAND_CFG;
	req->req_id = MLAN_IOCTL_RADIO_CFG;

	/* Get fw supported values from MLAN */
	req->action = MLAN_ACT_GET;
	ret = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (ret != MLAN_STATUS_SUCCESS)
		goto done;

	if (*pband == '0') {
		PRINTM(MIOCTL, "Set band to AUTO\n");
		band = radio_cfg->param.band_cfg.fw_bands;
#if defined(STA_CFG80211) || defined(UAP_CFG80211)
		if (IS_STA_OR_UAP_CFG80211(cfg80211_wext) && priv->wdev &&
		    priv->wdev->wiphy) {
			if (radio_cfg->param.band_cfg.fw_bands & BAND_A)
				priv->wdev->wiphy->bands[IEEE80211_BAND_5GHZ] =
					&cfg80211_band_5ghz;
			priv->wdev->wiphy->bands[IEEE80211_BAND_2GHZ] =
				&cfg80211_band_2ghz;
		}
#endif
	} else if (*pband == '1') {
		PRINTM(MIOCTL, "Set band to 5G\n");
		if (!(radio_cfg->param.band_cfg.fw_bands & BAND_A)) {
			PRINTM(MERROR, "Don't support 5G band\n");
			ret = MLAN_STATUS_FAILURE;
			goto done;
		}
		band = BAND_A;
		if (radio_cfg->param.band_cfg.fw_bands & BAND_AN)
			band |= BAND_AN;
		if (radio_cfg->param.band_cfg.fw_bands & BAND_AAC)
			band |= BAND_AAC;
#if defined(STA_CFG80211) || defined(UAP_CFG80211)
		if (IS_STA_OR_UAP_CFG80211(cfg80211_wext) && priv->wdev &&
		    priv->wdev->wiphy) {
			priv->wdev->wiphy->bands[IEEE80211_BAND_5GHZ] =
				&cfg80211_band_5ghz;
			priv->wdev->wiphy->bands[IEEE80211_BAND_2GHZ] = NULL;
		}
#endif
	} else if (*pband == '2') {
		PRINTM(MIOCTL, "Set band to 2G\n");
		band = BAND_B | BAND_G;
		band |= BAND_GN;
#if defined(STA_CFG80211) || defined(UAP_CFG80211)
		if (IS_STA_OR_UAP_CFG80211(cfg80211_wext) && priv->wdev &&
		    priv->wdev->wiphy) {
			priv->wdev->wiphy->bands[IEEE80211_BAND_5GHZ] = NULL;
			priv->wdev->wiphy->bands[IEEE80211_BAND_2GHZ] =
				&cfg80211_band_2ghz;
		}
#endif
	} else {
		PRINTM(MERROR, "unsupported band\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	/* Set config_bands to MLAN */
	req->action = MLAN_ACT_SET;
	memset(&radio_cfg->param.band_cfg, 0, sizeof(mlan_ds_band_cfg));
	radio_cfg->param.band_cfg.config_bands = band;
	ret = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);

done:
	if (ret != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Add RX Filter
 *
 *  @param priv                 A pointer to moal_private structure
 *  @param rxfilter             A pointer to rxfilter string.
 *
 *  @return                     MLAN_STATUS_SUCCESS -- success, otherwise fail
 */
mlan_status woal_add_rxfilter(moal_private *priv, char *rxfilter)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	ENTER();
	/*  Android command:
	    "DRIVER RXFILTER-ADD 0"
	    "DRIVER RXFILTER-ADD 1"
	    "DRIVER RXFILTER-ADD 3" */
	if (*rxfilter == '0') {
		PRINTM(MIOCTL, "Add IPV4 multicast filter\n");
		priv->rx_filter |= RX_FILTER_IPV4_MULTICAST;
	} else if (*rxfilter == '1') {
		PRINTM(MIOCTL, "Add broadcast filter\n");
		priv->rx_filter |= RX_FILTER_BROADCAST;
	} else if (*rxfilter == '2') {
		PRINTM(MIOCTL, "Add unicast filter\n");
		priv->rx_filter |= RX_FILTER_UNICAST;
	} else if (*rxfilter == '3') {
		PRINTM(MIOCTL, "Add IPV6 multicast fitler\n");
		priv->rx_filter |= RX_FILTER_IPV6_MULTICAST;
	} else {
		PRINTM(MERROR, "unsupported rx fitler\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
done:
	LEAVE();
	return ret;
}

/**
 *  @brief Remove RX Filter
 *
 *  @param priv                 A pointer to moal_private structure
 *  @param rxfilter             A pointer to rxfilter string.
 *
 *  @return                     MLAN_STATUS_SUCCESS -- success, otherwise fail
 */
mlan_status woal_remove_rxfilter(moal_private *priv, char *rxfilter)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	ENTER();
	if (*rxfilter == '0') {
		PRINTM(MIOCTL, "Remove IPV4 multicast filter\n");
		priv->rx_filter &= ~RX_FILTER_IPV4_MULTICAST;
	} else if (*rxfilter == '1') {
		PRINTM(MIOCTL, "Remove broadcast filter\n");
		priv->rx_filter &= ~RX_FILTER_BROADCAST;
	} else if (*rxfilter == '2') {
		PRINTM(MIOCTL, "Remove unicast filter\n");
		priv->rx_filter &= ~RX_FILTER_UNICAST;
	} else if (*rxfilter == '3') {
		PRINTM(MIOCTL, "Remove IPV6 multicast fitler\n");
		priv->rx_filter &= ~RX_FILTER_IPV6_MULTICAST;
	} else {
		PRINTM(MERROR, "unsupported rx fitler\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
done:
	LEAVE();
	return ret;
}

/**
 * @brief Set/Get WMM IE QoS configuration
 *
 * @param priv     A pointer to moal_private structure
 *  @param action  Action set or get
 * @param qos_cfg  A pointer to QoS configuration structure
 *
 * @return         MLAN_STATUS_SUCCESS -- success, otherwise fail
 */
mlan_status woal_priv_qos_cfg(moal_private *priv, t_u32 action, char *qos_cfg)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_ds_wmm_cfg *cfg = NULL;
	mlan_ioctl_req *req = NULL;
	int qosinfo = 0;

	ENTER();

	if (qos_cfg == NULL) {
		PRINTM(MERROR, "QOS info buffer is null\n");
		return MLAN_STATUS_FAILURE;
	}
	if ((action == MLAN_ACT_SET) &&
	    (MLAN_STATUS_SUCCESS != woal_atoi(&qosinfo, qos_cfg))) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_wmm_cfg));
	if (req == NULL) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	cfg = (mlan_ds_wmm_cfg *)req->pbuf;
	cfg->sub_command = MLAN_OID_WMM_CFG_QOS;
	req->req_id = MLAN_IOCTL_WMM_CFG;
	req->action = action;
	if (action == MLAN_ACT_SET) {
		cfg->param.qos_cfg = (t_u8)qosinfo;
		PRINTM(MIOCTL, "set qosinfo=%d\n", qosinfo);
	}
	ret = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);

	if (action == MLAN_ACT_GET)
		*qos_cfg = cfg->param.qos_cfg;
done:
	if (ret != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 * @brief Set sleep period
 *
 * @param priv     A pointer to moal_private structure
 * @param psleeppd A pointer to sleep period configuration structure
 *
 * @return         MLAN_STATUS_SUCCESS -- success, otherwise fail
 */
mlan_status woal_set_sleeppd(moal_private *priv, char *psleeppd)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_ds_pm_cfg *pm_cfg = NULL;
	mlan_ioctl_req *req = NULL;
	int sleeppd = 0;

	ENTER();

	if (MLAN_STATUS_SUCCESS != woal_atoi(&sleeppd, psleeppd)) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	PRINTM(MIOCTL, "set sleeppd=%d\n", sleeppd);
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_pm_cfg));
	if (req == NULL) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	pm_cfg = (mlan_ds_pm_cfg *)req->pbuf;
	pm_cfg->sub_command = MLAN_OID_PM_CFG_SLEEP_PD;
	req->req_id = MLAN_IOCTL_PM_CFG;
	if ((sleeppd <= MAX_SLEEP_PERIOD && sleeppd >= MIN_SLEEP_PERIOD) ||
	    (sleeppd == 0) || (sleeppd == SLEEP_PERIOD_RESERVED_FF)) {
		req->action = MLAN_ACT_SET;
		pm_cfg->param.sleep_period = sleeppd;
	} else {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	ret = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);

done:
	if (ret != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 * @brief  Set scan period function
 *
 * @param priv      A pointer to moal_private structure
 * @param buf       A pointer to scan command buf
 * @param length    buf length
 *
 * @return          MLAN_STATUS_SUCCESS -- success, otherwise fail
 */
int woal_set_scan_cfg(moal_private *priv, char *buf, int length)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	t_u8 *ptr = buf + NL80211_SCANCFG_HEADER_SIZE;
	int buf_left = length - NL80211_SCANCFG_HEADER_SIZE;
	t_u16 active_scan_time = 0;
	t_u16 passive_scan_time = 0;
	t_u16 specific_scan_time = 0;

	ENTER();
	while (buf_left >= 2) {
		switch (*ptr) {
		case NL80211_SCANCFG_ACTV_DWELL_SECTION:
			if (buf_left < 3) {
				PRINTM(MERROR,
				       "Invalid ACTV_DWELL_SECTION, buf_left=%d\n",
				       buf_left);
				buf_left = 0;
				break;
			}
			active_scan_time = ptr[2] << 8 | ptr[1];
			ptr += 3;
			buf_left -= 3;
			break;
		case NL80211_SCANCFG_PASV_DWELL_SECTION:
			if (buf_left < 3) {
				PRINTM(MERROR,
				       "Invalid PASV_DWELL_SECTION, buf_left=%d\n",
				       buf_left);
				buf_left = 0;
				break;
			}
			passive_scan_time = ptr[2] << 8 | ptr[1];
			ptr += 3;
			buf_left -= 3;
			break;
		case NL80211_SCANCFG_SPCF_DWELL_SECTION:
			if (buf_left < 3) {
				PRINTM(MERROR,
				       "Invalid SPCF_DWELL_SECTION, buf_left=%d\n",
				       buf_left);
				buf_left = 0;
				break;
			}
			specific_scan_time = ptr[2] << 8 | ptr[1];
			ptr += 3;
			buf_left -= 3;
			break;
		default:
			buf_left = 0;
			break;
		}
	}

	if (active_scan_time || passive_scan_time || specific_scan_time) {
		PRINTM(MIOCTL,
		       "Set active_scan_time= %d passive_scan_time=%d specific_scan_time=%d\n",
		       active_scan_time, passive_scan_time, specific_scan_time);
		if (MLAN_STATUS_FAILURE ==
		    woal_set_scan_time(priv, active_scan_time,
				       passive_scan_time, specific_scan_time)) {
			ret = -EFAULT;
		}
	}

	LEAVE();
	return ret;
}

/**
 *  @brief Set Radio On/OFF
 *
 *  @param priv                 A pointer to moal_private structure
 *  @param option               Radio Option
 *
 *  @return                     0 --success, otherwise fail
 */
int woal_set_radio(moal_private *priv, t_u8 option)
{
	int ret = 0;
	mlan_ds_radio_cfg *radio = NULL;
	mlan_ioctl_req *req = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;
	ENTER();
	if ((option != 0) && (option != 1)) {
		ret = -EINVAL;
		goto done;
	}
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_radio_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}
	radio = (mlan_ds_radio_cfg *)req->pbuf;
	radio->sub_command = MLAN_OID_RADIO_CTRL;
	req->req_id = MLAN_IOCTL_RADIO_CFG;
	req->action = MLAN_ACT_SET;
	radio->param.radio_on_off = option;
	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS)
		ret = -EFAULT;

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

#endif /* STA_SUPPORT */

#ifdef USB
/**
 * @brief Initialize USB packet aggregation control
 *
 * @param handle    A pointer to moal_private structure
 *
 * @return          MLAN_STATUS_SUCCESS -- success, otherwise fail
 */
mlan_status woal_usb_aggr_init(moal_handle *handle)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	moal_private *priv = NULL;
	mlan_ioctl_req *req = NULL;
	mlan_ds_misc_cfg *pcfg_misc = NULL;
	mlan_ds_misc_usb_aggr_ctrl *aggr_param = NULL;
	t_u8 usb_aggr_enable;
	struct usb_card_rec *cardp = NULL;
	int i = 0, resubmit_urbs = 0;

	ENTER();

	if (!handle) {
		PRINTM(MERROR, "Handle is null\n");
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}
	priv = woal_get_priv(handle, MLAN_BSS_ROLE_STA);
	if (!priv) {
		PRINTM(MERROR, "STA priv is null\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	if (handle->card == NULL) {
		PRINTM(MERROR, "Card is null in %s()\n", __func__);
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	cardp = (struct usb_card_rec *)handle->card;

	/* Check module parameter */
	if (handle->params.usb_aggr == 0 || handle->params.usb_aggr == 2) {
		usb_aggr_enable = MFALSE;
	} else if (handle->params.usb_aggr == 1) {
		usb_aggr_enable = MTRUE;
	} else {
		PRINTM(MWARN,
		       "Invalid module param (usb_aggr) value %d, "
		       "using MLAN default\n",
		       handle->params.usb_aggr);
		handle->params.usb_aggr = 0;
		usb_aggr_enable = MFALSE;
	}

	/* Allocate an IOCTL request buffer */
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	/* Fill request buffer */
	pcfg_misc = (mlan_ds_misc_cfg *)req->pbuf;
	pcfg_misc->sub_command = MLAN_OID_MISC_USB_AGGR_CTRL;
	req->req_id = MLAN_IOCTL_MISC_CFG;
	req->action = MLAN_ACT_SET;

	aggr_param = &pcfg_misc->param.usb_aggr_params;
	/* Initialize TX aggr default values */
	aggr_param->tx_aggr_ctrl.enable = usb_aggr_enable;
	aggr_param->tx_aggr_ctrl.aggr_mode = MLAN_USB_AGGR_MODE_NUM;
	aggr_param->tx_aggr_ctrl.aggr_align =
		MAX(handle->params.max_tx_buf, MLAN_USB_TX_AGGR_ALIGN);
	aggr_param->tx_aggr_ctrl.aggr_max = MLAN_USB_TX_MAX_AGGR_NUM;
	aggr_param->tx_aggr_ctrl.aggr_tmo =
		MLAN_USB_TX_AGGR_TIMEOUT_MSEC * 1000;

	/* Initialize RX deaggr default values */
	aggr_param->rx_deaggr_ctrl.enable = usb_aggr_enable;
	aggr_param->rx_deaggr_ctrl.aggr_mode = MLAN_USB_AGGR_MODE_NUM;
	aggr_param->rx_deaggr_ctrl.aggr_align = MLAN_USB_RX_ALIGN_SIZE;
	aggr_param->rx_deaggr_ctrl.aggr_max = MLAN_USB_RX_MAX_AGGR_NUM;
	aggr_param->rx_deaggr_ctrl.aggr_tmo = MLAN_USB_RX_DEAGGR_TIMEOUT_USEC;

	/* Do not fail driver init due to command failure */
	ret = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	/* Disable the feature if FW return failure/unsupported */
	if (req->status_code)
		cardp->tx_aggr_ctrl.enable = MFALSE;
	else
		moal_memcpy_ext(handle, &cardp->tx_aggr_ctrl,
				&aggr_param->tx_aggr_ctrl,
				sizeof(usb_aggr_ctrl_cfg),
				sizeof(usb_aggr_ctrl_cfg));

	if (req->status_code) {
		/* Disable the feature if FW return failure/unsupported */
		cardp->rx_deaggr_ctrl.enable = MFALSE;
	} else {
		/* Default is disable, resubmit URBs only for enable */
		if (cardp->rx_deaggr_ctrl.enable != usb_aggr_enable)
			resubmit_urbs = 1;
	}
	if (resubmit_urbs) {
		/* Indicate resubmition from here */
		cardp->resubmit_urbs = 1;
		/* Rx SG feature is disabled due to FW failure/unsupported,
		 * kill the URBs, they will be resubmitted after saving the
		 * parameters to USB card */
		if (atomic_read(&cardp->rx_data_urb_pending)) {
			for (i = 0; i < MVUSB_RX_DATA_URB; i++) {
				if (cardp->rx_data_list[i].urb) {
					usb_kill_urb(
						cardp->rx_data_list[i].urb);
				}
			}
		}

		/* Default is disable, update only for enable case */
		moal_memcpy_ext(handle, &cardp->rx_deaggr_ctrl,
				&aggr_param->rx_deaggr_ctrl,
				sizeof(usb_aggr_ctrl_cfg),
				sizeof(usb_aggr_ctrl_cfg));

		/* Ensure the next data URBs will use the modified parameters */
		if (!atomic_read(&cardp->rx_data_urb_pending)) {
			/* Submit multiple Rx data URBs */
			woal_usb_submit_rx_data_urbs(handle);
		}
		cardp->resubmit_urbs = 0;
	}

done:
	if (ret != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}
#endif

/**
 * @brief Set/Get configure multi-channel policy
 *
 * @param priv		A pointer to moal_private structure
 * @param enable	A pointer to enable
 * @param wait_option	wait_option of ioctl
 * @param action	action of ioctl
 *
 * @return          MLAN_STATUS_SUCCESS -- success, otherwise fail
 */
mlan_status woal_mc_policy_cfg(moal_private *priv, t_u16 *enable,
			       t_u8 wait_option, t_u8 action)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_misc_cfg *cfg = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		status = MLAN_STATUS_FAILURE;
		goto done;
	}

	cfg = (mlan_ds_misc_cfg *)req->pbuf;
	cfg->sub_command = MLAN_OID_MISC_MULTI_CHAN_POLICY;
	req->req_id = MLAN_IOCTL_MISC_CFG;
	req->action = action;
	if (MLAN_ACT_SET == action)
		cfg->param.multi_chan_policy = *enable;
	status = woal_request_ioctl(priv, req, wait_option);
	if (status != MLAN_STATUS_SUCCESS)
		goto done;
	if (MLAN_ACT_GET == action)
		*enable = cfg->param.multi_chan_policy;

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return status;
}

#if defined(STA_CFG80211) || defined(UAP_CFG80211)
/**
 * @brief Set multi ap flag to mlan layer
 *
 * @param priv		A pointer to moal_private structure
 * @param wait_option	wait_option of ioctl
 * @param flag	    multi ap flag
 *
 * @return          MLAN_STATUS_SUCCESS -- success, otherwise fail
 */
mlan_status woal_multi_ap_cfg(moal_private *priv, t_u8 wait_option, t_u8 flag)
{
	mlan_status status = MLAN_STATUS_SUCCESS;
	mlan_ioctl_req *req = NULL;
	mlan_ds_misc_cfg *cfg = NULL;

	ENTER();

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		status = MLAN_STATUS_FAILURE;
		goto done;
	}

	cfg = (mlan_ds_misc_cfg *)req->pbuf;
	cfg->param.multi_ap_flag = flag;
	cfg->sub_command = MLAN_OID_MISC_MULTI_AP_CFG;
	req->req_id = MLAN_IOCTL_MISC_CFG;
	req->action = MLAN_ACT_SET;

	status = woal_request_ioctl(priv, req, wait_option);
	if (status != MLAN_STATUS_SUCCESS)
		goto done;

	PRINTM(MCMND, "%s: %s 4addr mode\n", priv->netdev->name,
	       flag ? "Enable" : "Disable");

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return status;
}
#endif

/**
 *  @brief Set hotspot configuration value to mlan layer
 *
 *  @param priv         A pointer to moal_private structure
 *  @param wait_option  wait option
 *  @param hotspotcfg   Hotspot configuration value
 *
 *  @return             MLAN_STATUS_SUCCESS/MLAN_STATUS_PENDING -- success,
 * otherwise fail
 */
mlan_status woal_set_hotspotcfg(moal_private *priv, t_u8 wait_option,
				t_u32 hotspotcfg)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_ioctl_req *req = NULL;
	mlan_ds_misc_cfg *cfg = NULL;

	ENTER();

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	cfg = (mlan_ds_misc_cfg *)req->pbuf;
	cfg->param.hotspot_cfg = hotspotcfg;
	req->action = MLAN_ACT_SET;

	cfg->sub_command = MLAN_OID_MISC_HOTSPOT_CFG;
	req->req_id = MLAN_IOCTL_MISC_CFG;

	ret = woal_request_ioctl(priv, req, wait_option);
	if (ret != MLAN_STATUS_SUCCESS)
		goto done;

done:
	if (ret != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
/**
 * @brief               Set/Get network monitor configurations
 *
 * @param priv          Pointer to moal_private structure
 * @param wait_option  wait option
 * @param enable	    Enable/Disable
 * @param filter	    Filter flag - Management/Control/Data
 * @param band_chan_cfg           Network monitor band channel config
 *
 * @return             MLAN_STATUS_SUCCESS -- success, otherwise fail
 */
mlan_status woal_set_net_monitor(moal_private *priv, t_u8 wait_option,
				 t_u8 enable, t_u8 filter,
				 netmon_band_chan_cfg *band_chan_cfg)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_ioctl_req *req = NULL;
	mlan_ds_misc_cfg *misc = NULL;
	mlan_ds_misc_net_monitor *net_mon = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	misc = (mlan_ds_misc_cfg *)req->pbuf;
	net_mon = (mlan_ds_misc_net_monitor *)&misc->param.net_mon;
	misc->sub_command = MLAN_OID_MISC_NET_MONITOR;
	req->req_id = MLAN_IOCTL_MISC_CFG;
	req->action = MLAN_ACT_SET;
	net_mon->enable_net_mon = enable;
	if (net_mon->enable_net_mon) {
		net_mon->filter_flag = filter;
		if (band_chan_cfg && band_chan_cfg->channel) {
			net_mon->band = band_chan_cfg->band;
			net_mon->channel = band_chan_cfg->channel;
			net_mon->chan_bandwidth = band_chan_cfg->chan_bandwidth;
		}
	}

	status = woal_request_ioctl(priv, req, wait_option);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	/* update chan band values from response in band_chan_cfg */
	if (net_mon->enable_net_mon) {
		if (band_chan_cfg) {
			band_chan_cfg->band = net_mon->band;
			band_chan_cfg->channel = net_mon->channel;
			band_chan_cfg->chan_bandwidth = net_mon->chan_bandwidth;
		}
	}
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);

	LEAVE();
	return ret;
}
#endif

/**
 *  @brief Send delelte all BA command to MLAN
 *
 *  @param priv          A pointer to moal_private structure
 *  @param wait_option          Wait option
 *
 *  @return              MLAN_STATUS_SUCCESS/MLAN_STATUS_PENDING -- success,
 * otherwise fail
 */
mlan_status woal_delba_all(moal_private *priv, t_u8 wait_option)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_11n_cfg *cfg_11n = NULL;
	mlan_ds_11n_delba *del_ba = NULL;
	t_u8 zero_mac[MLAN_MAC_ADDR_LENGTH] = {0};
	mlan_status status = MLAN_STATUS_SUCCESS;
	int ret = 0;

	req = (mlan_ioctl_req *)woal_alloc_mlan_ioctl_req(
		sizeof(mlan_ds_11n_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}
	cfg_11n = (mlan_ds_11n_cfg *)req->pbuf;
	req->req_id = MLAN_IOCTL_11N_CFG;
	cfg_11n->sub_command = MLAN_OID_11N_CFG_DELBA;

	del_ba = &cfg_11n->param.del_ba;
	memset(del_ba, 0, sizeof(mlan_ds_11n_delba));
	del_ba->direction = DELBA_RX | DELBA_TX;
	del_ba->tid = DELBA_ALL_TIDS;
	moal_memcpy_ext(priv->phandle, del_ba->peer_mac_addr, zero_mac,
			ETH_ALEN, sizeof(del_ba->peer_mac_addr));

	status = woal_request_ioctl(priv, req, wait_option);

	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief config RTT to mlan layer
 *
 *  @param priv         A pointer to moal_private structure
 *  @param ch_info   A pointer to wifi_channel_info
 *  @param bandcfg  A pointer to Band_Config_t
 *
 *  @return            void
 */
void woal_channel_info_to_bandcfg(moal_private *priv,
				  wifi_channel_info *ch_info,
				  Band_Config_t *bandcfg)
{
	t_u8 channel = 0;

	if (!ch_info || !bandcfg)
		return;
#if defined(STA_CFG80211) || defined(UAP_CFG80211)
	channel = ieee80211_frequency_to_channel(ch_info->center_freq);
#endif

	switch (ch_info->width) {
	case WIFI_CHAN_WIDTH_10:
		bandcfg->chanWidth = CHAN_BW_10MHZ;
		break;
	case WIFI_CHAN_WIDTH_20:
		bandcfg->chanWidth = CHAN_BW_20MHZ;
		break;
	case WIFI_CHAN_WIDTH_40:
		bandcfg->chanWidth = CHAN_BW_40MHZ;
		break;
	case WIFI_CHAN_WIDTH_80:
		bandcfg->chanWidth = CHAN_BW_80MHZ;
		break;
	default:
		bandcfg->chanWidth = CHAN_BW_20MHZ;
		break;
	}
	bandcfg->chan2Offset = SEC_CHAN_NONE;
	if (bandcfg->chanWidth == CHAN_BW_40MHZ) {
		if (ch_info->center_freq0 < ch_info->center_freq)
			bandcfg->chan2Offset = SEC_CHAN_BELOW;
		else
			bandcfg->chan2Offset = SEC_CHAN_ABOVE;
	} else if (bandcfg->chanWidth == CHAN_BW_80MHZ)
		bandcfg->chan2Offset =
			woal_get_second_channel_offset(priv, channel);
	bandcfg->chanBand = (channel <= MAX_BG_CHANNEL) ? BAND_2GHZ : BAND_5GHZ;
	bandcfg->scanMode = SCAN_MODE_MANUAL;

	return;
}

/**
 *  @brief config RTT to mlan layer
 *
 *  @param priv         A pointer to moal_private structure
 *  @param ch_info   A pointer to wifi_channel_info
 *  @param bandcfg  A pointer to Band_Config_t
 *
 *  @return            void
 */
void woal_bandcfg_to_channel_info(moal_private *priv, Band_Config_t *bandcfg,
				  t_u8 channel, wifi_channel_info *ch_info)
{
	if (!ch_info || !bandcfg)
		return;
#if defined(STA_CFG80211) || defined(UAP_CFG80211)
	ch_info->center_freq = ieee80211_channel_to_frequency(
		channel, (channel <= MAX_BG_CHANNEL) ? NL80211_BAND_2GHZ :
						       NL80211_BAND_5GHZ);
#endif

	switch (bandcfg->chanWidth) {
	case CHAN_BW_10MHZ:
		ch_info->width = WIFI_CHAN_WIDTH_10;
		break;
	case CHAN_BW_20MHZ:
		ch_info->width = WIFI_CHAN_WIDTH_20;
		break;
	case CHAN_BW_40MHZ:
		ch_info->width = WIFI_CHAN_WIDTH_40;
		break;
	case CHAN_BW_80MHZ:
		ch_info->width = WIFI_CHAN_WIDTH_80;
		break;
	default:
		ch_info->width = WIFI_CHAN_WIDTH_20;
		break;
	}

	return;
}

/**
 *  @brief config RTT to mlan layer
 *
 *  @param priv         A pointer to moal_private structure
 *  @param wait_option  wait option
 *  @param hotspotcfg   A pointer to rtt_config_params_t
 *
 *  @return             MLAN_STATUS_SUCCESS/MLAN_STATUS_PENDING -- success,
 * otherwise fail
 */
mlan_status woal_config_rtt(moal_private *priv, t_u8 wait_option,
			    wifi_rtt_config_params_t *rtt_params_in)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_ioctl_req *req = NULL;
	mlan_ds_misc_cfg *misc = NULL;
	mlan_rtt_config_params *rtt_params = NULL;
	t_u32 i = 0;

	ENTER();

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	req->action = MLAN_ACT_SET;
	req->req_id = MLAN_IOCTL_MISC_CFG;

	misc = (mlan_ds_misc_cfg *)req->pbuf;
	misc->sub_command = MLAN_OID_MISC_CONFIG_RTT;
	rtt_params = &(misc->param.rtt_params);
	rtt_params->rtt_config_num = rtt_params_in->rtt_config_num;
	for (i = 0; i < MIN(rtt_params->rtt_config_num, MAX_RTT_CONFIG_NUM);
	     i++) {
		moal_memcpy_ext(priv->phandle, rtt_params->rtt_config[i].addr,
				rtt_params_in->rtt_config[i].addr,
				sizeof(rtt_params->rtt_config[i].addr),
				sizeof(rtt_params->rtt_config[i].addr));
		rtt_params->rtt_config[i].type =
			rtt_params_in->rtt_config[i].type;
		rtt_params->rtt_config[i].peer =
			rtt_params_in->rtt_config[i].peer;
#if defined(STA_CFG80211) || defined(UAP_CFG80211)
		rtt_params->rtt_config[i]
			.channel = ieee80211_frequency_to_channel(
			rtt_params_in->rtt_config[i].channel.center_freq);
#endif
		woal_channel_info_to_bandcfg(
			priv, &(rtt_params_in->rtt_config[i].channel),
			&(rtt_params->rtt_config[i].bandcfg));
		rtt_params->rtt_config[i].burst_period =
			rtt_params_in->rtt_config[i].burst_period;
		rtt_params->rtt_config[i].num_burst =
			rtt_params_in->rtt_config[i].num_burst;
		rtt_params->rtt_config[i].num_frames_per_burst =
			rtt_params_in->rtt_config[i].num_frames_per_burst;
		rtt_params->rtt_config[i].num_retries_per_rtt_frame =
			rtt_params_in->rtt_config[i].num_retries_per_rtt_frame;
		rtt_params->rtt_config[i].num_retries_per_ftmr =
			rtt_params_in->rtt_config[i].num_retries_per_ftmr;
		rtt_params->rtt_config[i].LCI_request =
			rtt_params_in->rtt_config[i].LCI_request;
		rtt_params->rtt_config[i].LCR_request =
			rtt_params_in->rtt_config[i].LCR_request;
		rtt_params->rtt_config[i].burst_duration =
			rtt_params_in->rtt_config[i].burst_duration;
		rtt_params->rtt_config[i].preamble =
			rtt_params_in->rtt_config[i].preamble;
		rtt_params->rtt_config[i].bw = rtt_params_in->rtt_config[i].bw;
	}

	ret = woal_request_ioctl(priv, req, wait_option);
	if (ret != MLAN_STATUS_SUCCESS)
		goto done;

done:
	if (ret != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief cancel RTT to mlan layer
 *
 *  @param priv         A pointer to moal_private structure
 *  @param wait_option  wait option
 *  @param hotspotcfg   A pointer to rtt_config_params_t
 *
 *  @return             MLAN_STATUS_SUCCESS/MLAN_STATUS_PENDING -- success,
 * otherwise fail
 */
mlan_status woal_cancel_rtt(moal_private *priv, t_u8 wait_option,
			    t_u32 addr_num, t_u8 addr[][MLAN_MAC_ADDR_LENGTH])
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_ioctl_req *req = NULL;
	mlan_ds_misc_cfg *misc = NULL;
	mlan_rtt_cancel_params *rtt_cancel = NULL;

	ENTER();

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	req->action = MLAN_ACT_SET;
	req->req_id = MLAN_IOCTL_MISC_CFG;

	misc = (mlan_ds_misc_cfg *)req->pbuf;
	misc->sub_command = MLAN_OID_MISC_CANCEL_RTT;
	rtt_cancel = &(misc->param.rtt_cancel);
	rtt_cancel->rtt_cancel_num = addr_num;
	moal_memcpy_ext(priv->phandle, rtt_cancel->rtt_cancel, addr,
			sizeof(rtt_cancel->rtt_cancel[0]) *
				rtt_cancel->rtt_cancel_num,
			sizeof(rtt_cancel->rtt_cancel[0]) * MAX_RTT_CONFIG_NUM);
	ret = woal_request_ioctl(priv, req, wait_option);
	if (ret != MLAN_STATUS_SUCCESS)
		goto done;

done:
	if (ret != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief cancel RTT to mlan layer
 *
 *  @param priv         A pointer to moal_private structure
 *  @param wait_option  wait option
 *  @param hotspotcfg   A pointer to rtt_config_params_t
 *
 *  @return             MLAN_STATUS_SUCCESS/MLAN_STATUS_PENDING -- success,
 * otherwise fail
 */
mlan_status woal_rtt_responder_cfg(moal_private *priv, t_u8 wait_option,
				   mlan_rtt_responder *rtt_rsp_cfg)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_ioctl_req *req = NULL;
	mlan_ds_misc_cfg *misc = NULL;

	ENTER();

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	req->action = MLAN_ACT_SET;
	req->req_id = MLAN_IOCTL_MISC_CFG;

	misc = (mlan_ds_misc_cfg *)req->pbuf;
	misc->sub_command = MLAN_OID_MISC_RTT_RESPONDER_CFG;
	moal_memcpy_ext(priv->phandle, &(misc->param.rtt_rsp_cfg), rtt_rsp_cfg,
			sizeof(misc->param.rtt_rsp_cfg),
			sizeof(misc->param.rtt_rsp_cfg));

	ret = woal_request_ioctl(priv, req, wait_option);
	if (ret != MLAN_STATUS_SUCCESS)
		goto done;
	moal_memcpy_ext(priv->phandle, rtt_rsp_cfg, &(misc->param.rtt_rsp_cfg),
			sizeof(*rtt_rsp_cfg), sizeof(*rtt_rsp_cfg));

done:
	if (ret != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Send 11d enable/disable command to firmware.
 *
 *  @param priv             A pointer to moal_private structure
 *  @param wait_option      Wait option
 *  @param enable           Enable/Disable 11d
 *
 *  @return                 MLAN_STATUS_SUCCESS/MLAN_STATUS_PENDING -- success,
 * otherwise fail
 */
mlan_status woal_set_11d(moal_private *priv, t_u8 wait_option, t_u8 enable)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_snmp_mib *snmp = NULL;
	mlan_status ret = MLAN_STATUS_SUCCESS;

	ENTER();

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_snmp_mib));
	if (req == NULL) {
		LEAVE();
		return -ENOMEM;
	}

	req->action = MLAN_ACT_SET;
	req->req_id = MLAN_IOCTL_SNMP_MIB;

	snmp = (mlan_ds_snmp_mib *)req->pbuf;
	snmp->sub_command = MLAN_OID_SNMP_MIB_DOT11D;
	snmp->param.oid_value = enable;

	ret = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (ret != MLAN_STATUS_PENDING)
		kfree(req);

	LEAVE();
	return ret;
}

/**
 *  @brief Handle miscellaneous ioctls for asynchronous command response
 *
 *  @param priv     Pointer to moal_private structure
 *  @param info     Pointer to mlan_ds_misc_cfg structure
 *
 *  @return         N/A
 */
void woal_ioctl_get_misc_conf(moal_private *priv, mlan_ds_misc_cfg *info)
{
	ENTER();
	switch (info->sub_command) {
	default:
		break;
	}
}

#ifdef CONFIG_PROC_FS
#define RADIO_MODE_STR_LEN 20
#define TX_PWR_STR_LEN 20
#define TX_CONT_STR_LEN 50
#define TX_FRAME_STR_LEN 200
#define TRIGGER_FRAME_STR_LEN 250
#define HE_TB_TX_STR_LEN 30
#define MAX_RADIO_MODE 21
#define OTP_RDWR_LEN 50

/*
 *  @brief Parse mfg cmd radio mode string
 *
 *  @param s        A pointer to user buffer
 *  @param len      Length of user buffer
 *  @param d        A pointer to mfg_cmd_generic_cfg struct
 *  @return         0 on success, -EINVAL otherwise
 */
static int parse_radio_mode_string(const char *s, size_t len,
				   struct mfg_cmd_generic_cfg *d)
{
	int ret = MLAN_STATUS_SUCCESS;
	char *string = NULL;
	char *tmp = NULL;
	char *pos = NULL;
	gfp_t flag;
	t_u32 copy_len = 0;

	ENTER();
	if (!s || !d) {
		LEAVE();
		return -EINVAL;
	}
	flag = (in_atomic() || irqs_disabled()) ? GFP_ATOMIC : GFP_KERNEL;
	string = kzalloc(RADIO_MODE_STR_LEN + 1, flag);
	if (string == NULL) {
		LEAVE();
		return -ENOMEM;
	}

	copy_len = MIN(len - strlen("radio_mode="), RADIO_MODE_STR_LEN - 1);
	moal_memcpy_ext(NULL, string, s + strlen("radio_mode="), copy_len,
			copy_len);
	string[RADIO_MODE_STR_LEN] = '\0';

	tmp = string;
	string = strstrip(string);

	/* radio mode index 0 */
	pos = strsep(&string, " \t");
	if (pos)
		d->data1 = (t_u32)woal_string_to_number(pos);

	/* radio mode index 1 */
	pos = strsep(&string, " \t");
	if (pos)
		d->data2 = (t_u32)woal_string_to_number(pos);

	if ((d->data1 > MAX_RADIO_MODE) || (d->data2 > MAX_RADIO_MODE))
		ret = -EINVAL;

	kfree(tmp);
	LEAVE();
	return ret;
}

/*
 *  @brief PowerLevelToDUT11Bits
 *
 *  @param Pwr		A user txpwr values of type int
 *  @param PowerLevel	A Pointer of uint32 type for converted txpwr vals
 *  @return		nothing just exit
 */

static void PowerLevelToDUT11Bits(int Pwr, t_u32 *PowerLevel)
{
	int Z = 0;

	if ((Pwr > 64) || (Pwr < -64))
		return;

	Z = (int)(Pwr * 16);
	if (Z < 0) {
		Z = Z + (1 << 11);
	}
	(*PowerLevel) = (t_u32)Z;

	return;
}

/*
 *  @brief Parse mfg cmd tx pwr string
 *
 *  @param handle   A pointer to moal_handle structure
 *  @param s        A pointer to user buffer
 *  @param len      Length of user buffer
 *  @param d        A pointer to mfg_cmd_generic_cfg struct
 *  @return         0 on success, -EINVAL otherwise
 */

static int parse_tx_pwr_string(moal_handle *handle, const char *s, size_t len,
			       struct mfg_cmd_generic_cfg *d)
{
	int ret = MLAN_STATUS_SUCCESS;
	char *string = NULL;
	char *tmp = NULL;
	char *pos = NULL;
	gfp_t flag;
	t_u32 tx_pwr_converted = 0xffffffff;
	int tx_pwr_local = 0;
	t_u8 pow_conv = MFALSE;
	t_u32 pow_limit = 24;
	t_u8 card_type;
	t_u32 copy_len = 0;

	ENTER();
	if (!s || !d) {
		LEAVE();
		return -EINVAL;
	}

	card_type = (handle->card_type) & 0xff;
	if ((card_type == CARD_TYPE_9098) || (card_type == CARD_TYPE_9097) ||
	    (card_type == CARD_TYPE_9177) || (card_type == CARD_TYPE_IW624) ||
	    (card_type == CARD_TYPE_AW693) || (card_type == CARD_TYPE_IW610))
		pow_conv = MTRUE;

	flag = (in_atomic() || irqs_disabled()) ? GFP_ATOMIC : GFP_KERNEL;
	string = kzalloc(TX_PWR_STR_LEN + 1, flag);
	if (string == NULL) {
		LEAVE();
		return -ENOMEM;
	}

	copy_len = MIN(len - strlen("tx_power="), TX_PWR_STR_LEN - 1);
	moal_memcpy_ext(NULL, string, s + strlen("tx_power="), copy_len,
			copy_len);
	string[TX_PWR_STR_LEN] = '\0';

	tmp = string;
	string = strstrip(string);

	/* tx power value */
	pos = strsep(&string, " \t");
	if (pow_conv && pos) {
		/* We need to convert user power vals including -ve vals to
		 * 1/16dbm resolution*/
		tx_pwr_local = woal_string_to_number(pos);
		if (tx_pwr_local != 0xffffffff) {
			PowerLevelToDUT11Bits(tx_pwr_local, &tx_pwr_converted);
			d->data1 = tx_pwr_converted;
			pow_limit = 384;
		} else
			d->data1 = (t_u32)tx_pwr_local;
	} else if (pos) {
		d->data1 = (t_u32)woal_string_to_number(pos);
	}
	/* modulation */
	pos = strsep(&string, " \t");
	if (pos)
		d->data2 = (t_u32)woal_string_to_number(pos);

	/* path id */
	pos = strsep(&string, " \t");
	if (pos)
		d->data3 = (t_u32)woal_string_to_number(pos);

	if (((d->data1 > pow_limit) && (d->data1 <= 0xfffffff0)) ||
	    (d->data2 > 2))
		ret = -EINVAL;

	kfree(tmp);
	LEAVE();
	return ret;
}
/*
 *  @brief Parse mfg cmd tx cont string
 *
 *  @param s        A pointer to user buffer
 *  @param len      Length of user buffer
 *  @param d        A pointer to mfg_cmd_tx_cont struct
 *  @return         0 on success, -EINVAL otherwise
 */
static int parse_tx_cont_string(const char *s, size_t len,
				struct mfg_cmd_tx_cont *d)
{
	int ret = MLAN_STATUS_SUCCESS;
	char *string = NULL;
	char *tmp = NULL;
	char *pos = NULL;
	gfp_t flag;
	t_u32 copy_len = 0;

	ENTER();
	if (!s || !d) {
		LEAVE();
		return -EINVAL;
	}
	flag = (in_atomic() || irqs_disabled()) ? GFP_ATOMIC : GFP_KERNEL;
	string = kzalloc(TX_CONT_STR_LEN + 1, flag);
	if (string == NULL) {
		LEAVE();
		return -ENOMEM;
	}

	copy_len = MIN(len - strlen("tx_continuous="), TX_CONT_STR_LEN - 1);
	moal_memcpy_ext(NULL, string, s + strlen("tx_continuous="), copy_len,
			copy_len);
	string[TX_CONT_STR_LEN] = '\0';

	tmp = string;
	string = strstrip(string);

	pos = strsep(&string, " \t");
	if (pos)
		d->enable_tx = (t_u32)woal_string_to_number(pos);

	pos = strsep(&string, " \t");
	if (pos)
		d->cw_mode = (t_u32)woal_string_to_number(pos);

	if (d->enable_tx == MFALSE)
		goto done;

	pos = strsep(&string, " \t");
	if (pos)
		d->payload_pattern = (t_u32)woal_string_to_number(pos);

	pos = strsep(&string, " \t");
	if (pos)
		d->cs_mode = (t_u32)woal_string_to_number(pos);

	pos = strsep(&string, " \t");
	if (pos)
		d->act_sub_ch = (t_u32)woal_string_to_number(pos);

	pos = strsep(&string, " \t");
	if (pos)
		d->tx_rate = (t_u32)woal_string_to_number(pos);

	if ((d->enable_tx > 1) || (d->cw_mode > 1) || (d->cs_mode > 1) ||
	    (d->act_sub_ch == 2 || d->act_sub_ch > 3))
		ret = -EINVAL;
done:
	kfree(tmp);
	LEAVE();
	return ret;
}
/*
 *  @brief Parse mfg cmd trigger string
 *
 *  @param s        A pointer to user buffer
 *  @param len      Length of user buffer
 *  @param d        A pointer to mfg_cmd_tx_frame2 struct
 *  @return         0 on success, -EINVAL otherwise
 */
static int parse_trigger_frame_string(const char *s, size_t len,
				      mfg_Cmd_IEEEtypes_CtlBasicTrigHdr_t *d)
{
	int ret = MLAN_STATUS_SUCCESS;
	char *string = NULL;
	char *tmp = NULL;
	char *pos = NULL;
	gfp_t flag;
	t_u32 copy_len = 0;

	ENTER();
	if (!s || !d) {
		LEAVE();
		return -EINVAL;
	}
	flag = (in_atomic() || irqs_disabled()) ? GFP_ATOMIC : GFP_KERNEL;
	string = kzalloc(TRIGGER_FRAME_STR_LEN + 1, flag);
	if (string == NULL)
		return -ENOMEM;

	copy_len =
		MIN(len - strlen("trigger_frame="), TRIGGER_FRAME_STR_LEN - 1);
	moal_memcpy_ext(NULL, string, s + strlen("trigger_frame="), copy_len,
			copy_len);
	string[TRIGGER_FRAME_STR_LEN] = '\0';

	tmp = string;
	string = strstrip(string);

	pos = strsep(&string, " \t");
	if (pos)
		d->enable_tx = (t_u32)woal_string_to_number(pos);

	if (d->enable_tx == MFALSE)
		goto done;

	pos = strsep(&string, " \t");
	if (pos)
		d->standalone_hetb = (t_u32)woal_string_to_number(pos);

	pos = strsep(&string, " \t");
	if (pos)
		d->frmCtl.type = (t_u32)woal_string_to_number(pos);

	pos = strsep(&string, " \t");
	if (pos)
		d->frmCtl.sub_type = (t_u32)woal_string_to_number(pos);

	pos = strsep(&string, " \t");
	if (pos)
		d->duration = (t_u32)woal_string_to_number(pos);

	pos = strsep(&string, " \t");
	if (pos)
		d->trig_common_field.trigger_type =
			(t_u32)woal_string_to_number(pos);

	pos = strsep(&string, " \t");
	if (pos)
		d->trig_common_field.ul_len = (t_u32)woal_string_to_number(pos);

	pos = strsep(&string, " \t");
	if (pos)
		d->trig_common_field.more_tf =
			(t_u32)woal_string_to_number(pos);

	pos = strsep(&string, " \t");
	if (pos)
		d->trig_common_field.cs_required =
			(t_u32)woal_string_to_number(pos);

	pos = strsep(&string, " \t");
	if (pos)
		d->trig_common_field.ul_bw = (t_u32)woal_string_to_number(pos);

	pos = strsep(&string, " \t");
	if (pos)
		d->trig_common_field.ltf_type =
			(t_u32)woal_string_to_number(pos);

	pos = strsep(&string, " \t");
	if (pos)
		d->trig_common_field.ltf_mode =
			(t_u32)woal_string_to_number(pos);

	pos = strsep(&string, " \t");
	if (pos)
		d->trig_common_field.ltf_symbol =
			(t_u32)woal_string_to_number(pos);

	pos = strsep(&string, " \t");
	if (pos)
		d->trig_common_field.ul_stbc =
			(t_u32)woal_string_to_number(pos);

	pos = strsep(&string, " \t");
	if (pos)
		d->trig_common_field.ldpc_ess =
			(t_u32)woal_string_to_number(pos);

	pos = strsep(&string, " \t");
	if (pos)
		d->trig_common_field.ap_tx_pwr =
			(t_u32)woal_string_to_number(pos);

	pos = strsep(&string, " \t");
	if (pos)
		d->trig_common_field.pre_fec_pad_fct =
			(t_u32)woal_string_to_number(pos);

	pos = strsep(&string, " \t");
	if (pos)
		d->trig_common_field.pe_disambig =
			(t_u32)woal_string_to_number(pos);

	pos = strsep(&string, " \t");
	if (pos)
		d->trig_common_field.spatial_reuse =
			(t_u32)woal_string_to_number(pos);

	pos = strsep(&string, " \t");
	if (pos)
		d->trig_common_field.doppler =
			(t_u32)woal_string_to_number(pos);

	pos = strsep(&string, " \t");
	if (pos)
		d->trig_common_field.he_sig2 =
			(t_u32)woal_string_to_number(pos);

	pos = strsep(&string, " \t");
	if (pos)
		d->trig_user_info_field.aid12 =
			(t_u32)woal_string_to_number(pos);

	pos = strsep(&string, " \t");
	if (pos)
		d->trig_user_info_field.ru_alloc_reg =
			(t_u32)woal_string_to_number(pos);

	pos = strsep(&string, " \t");
	if (pos)
		d->trig_user_info_field.ru_alloc =
			(t_u32)woal_string_to_number(pos);

	pos = strsep(&string, " \t");
	if (pos)
		d->trig_user_info_field.ul_coding_type =
			(t_u32)woal_string_to_number(pos);

	pos = strsep(&string, " \t");
	if (pos)
		d->trig_user_info_field.ul_mcs =
			(t_u32)woal_string_to_number(pos);

	pos = strsep(&string, " \t");
	if (pos)
		d->trig_user_info_field.ul_dcm =
			(t_u32)woal_string_to_number(pos);

	pos = strsep(&string, " \t");
	if (pos)
		d->trig_user_info_field.ss_alloc =
			(t_u32)woal_string_to_number(pos);

	pos = strsep(&string, " \t");
	if (pos)
		d->trig_user_info_field.ul_target_rssi =
			(t_u32)woal_string_to_number(pos);

	pos = strsep(&string, " \t");
	if (pos)
		d->basic_trig_user_info.mpdu_mu_sf =
			(t_u32)woal_string_to_number(pos);

	pos = strsep(&string, " \t");
	if (pos)
		d->basic_trig_user_info.tid_al =
			(t_u32)woal_string_to_number(pos);

	pos = strsep(&string, " \t");
	if (pos)
		d->basic_trig_user_info.ac_pl =
			(t_u32)woal_string_to_number(pos);

	pos = strsep(&string, " \t");
	if (pos)
		d->basic_trig_user_info.pref_ac =
			(t_u32)woal_string_to_number(pos);

	if (d->enable_tx > 1)
		ret = -EINVAL;

done:
	kfree(tmp);
	LEAVE();
	return ret;
}

/*
 *  @brief Parse mfg cmd tx frame string
 *
 *  @param s        A pointer to user buffer
 *  @param len      Length of user buffer
 *  @param d        A pointer to mfg_cmd_tx_frame2 struct
 *  @return         0 on success, -EINVAL otherwise
 */
static int parse_tx_frame_string(moal_handle *handle, const char *s, size_t len,
				 struct mfg_cmd_tx_frame2 *d)
{
	int ret = MLAN_STATUS_SUCCESS;
	char *string = NULL;
	char *tmp = NULL;
	char *pos = NULL;
	int i;
	gfp_t flag;
	t_u8 card_type;
	t_u32 copy_len = 0;
	BOOLEAN dot11ax = MFALSE;

	ENTER();
	if (!s || !d) {
		LEAVE();
		return -EINVAL;
	}
	flag = (in_atomic() || irqs_disabled()) ? GFP_ATOMIC : GFP_KERNEL;
	string = kzalloc(TX_FRAME_STR_LEN + 1, flag);
	if (string == NULL)
		return -ENOMEM;

	card_type = (handle->card_type) & 0xff;
	if ((card_type >= CARD_TYPE_9098) && (card_type != CARD_TYPE_8801))
		dot11ax = MTRUE;

	/*Initialize the parameters to default values to be used*/
	d->data_rate = 0x1100;
	d->frame_pattern = 0xB496DEB6;
	d->frame_length = 0x400;
	d->enable = 0;
	d->short_preamble = -1;
	d->short_gi = 0;
	d->adv_coding = -1;
	d->tx_bf = 0;
	d->gf_mode = 0;
	d->stbc = 0;
	d->adjust_burst_sifs = 0;
	d->burst_sifs_in_us = 0;
	d->signal_bw = -1;
	d->NumPkt = -1;
	d->MaxPE = -1;
	d->BeamChange = -1;
	d->Dcm = -1;
	d->Doppler = -1;
	d->MidP = -1;
	d->QNum = -1;

	for (i = 0; i < ETH_ALEN; i++) {
		d->bssid[i] = 0xff;
	}

	copy_len = MIN(len - strlen("tx_frame="), TX_FRAME_STR_LEN - 1);
	moal_memcpy_ext(NULL, string, s + strlen("tx_frame="), copy_len,
			copy_len);
	string[TX_FRAME_STR_LEN] = '\0';

	tmp = string;
	string = strstrip(string);

	pos = strsep(&string, " \t");
	if (pos)
		d->enable = (t_u32)woal_string_to_number(pos);

	if (d->enable == MFALSE)
		goto done;

	pos = strsep(&string, " \t");
	if (pos)
		d->data_rate = (t_u32)woal_string_to_number(pos);

	pos = strsep(&string, " \t");
	if (pos)
		d->frame_pattern = (t_u32)woal_string_to_number(pos);

	pos = strsep(&string, " \t");
	if (pos)
		d->frame_length = (t_u32)woal_string_to_number(pos);

	pos = strsep(&string, " \t");
	if (pos)
		d->adjust_burst_sifs = (t_u16)woal_string_to_number(pos);

	pos = strsep(&string, " \t");
	if (pos)
		d->burst_sifs_in_us = (t_u32)woal_string_to_number(pos);

	pos = strsep(&string, " \t");
	if (pos)
		d->short_preamble = (t_u32)woal_string_to_number(pos);

	pos = strsep(&string, " \t");
	if (pos)
		d->act_sub_ch = (t_u32)woal_string_to_number(pos);

	pos = strsep(&string, " \t");
	if (pos)
		d->short_gi = (t_u32)woal_string_to_number(pos);

	pos = strsep(&string, " \t");
	if (pos)
		d->adv_coding = (t_u32)woal_string_to_number(pos);

	pos = strsep(&string, " \t");
	if (pos)
		d->tx_bf = (t_u32)woal_string_to_number(pos);

	pos = strsep(&string, " \t");
	if (pos)
		d->gf_mode = (t_u32)woal_string_to_number(pos);

	pos = strsep(&string, " \t");
	if (pos)
		d->stbc = (t_u32)woal_string_to_number(pos);

	pos = strsep(&string, " \t");
	if (pos)
		d->signal_bw = (t_u32)woal_string_to_number(pos);

	/* DOT 11AX parameters*/
	if (dot11ax) {
		pos = strsep(&string, " \t");
		if (pos)
			d->NumPkt = (t_u32)woal_string_to_number(pos);

		pos = strsep(&string, " \t");
		if (pos)
			d->MaxPE = (t_u32)woal_string_to_number(pos);

		pos = strsep(&string, " \t");
		if (pos)
			d->BeamChange = (t_u32)woal_string_to_number(pos);

		pos = strsep(&string, " \t");
		if (pos)
			d->Dcm = (t_u32)woal_string_to_number(pos);

		pos = strsep(&string, " \t");
		if (pos)
			d->Doppler = (t_u32)woal_string_to_number(pos);

		pos = strsep(&string, " \t");
		if (pos)
			d->MidP = (t_u32)woal_string_to_number(pos);

		pos = strsep(&string, " \t");
		if (pos)
			d->QNum = (t_u32)woal_string_to_number(pos);
	}

	pos = strsep(&string, " \t");
	if (pos) {
		char *begin, *end;
		begin = pos;
		for (i = 0; i < ETH_ALEN; i++) {
			end = woal_strsep(&begin, ':', '/');
			if (end)
				d->bssid[i] = woal_atox(end);
		}
	}

	if (d->enable > 1)
		ret = -EINVAL;
done:
	kfree(tmp);
	LEAVE();
	return ret;
}

/*
 *  @brief Parse mfg cmd HE TB Tx string
 *
 *  @param s        A pointer to user buffer
 *  @param len      Length of user buffer
 *  @param d        A pointer to mfg_Cmd_HE_TBTx_t struct
 *  @return         0 on success, -EINVAL otherwise
 */
static int parse_he_tb_tx_string(const char *s, size_t len,
				 struct mfg_Cmd_HE_TBTx_t *d)
{
	int ret = MLAN_STATUS_SUCCESS;
	char *string = NULL;
	char *pos = NULL;
	char *tmp = NULL;
	gfp_t flag;

	ENTER();
	if (!s || !d) {
		LEAVE();
		return -EINVAL;
	}
	flag = (in_atomic() || irqs_disabled()) ? GFP_ATOMIC : GFP_KERNEL;
	string = kzalloc(HE_TB_TX_STR_LEN, flag);
	if (string == NULL)
		return -ENOMEM;

	moal_memcpy_ext(NULL, string, s + strlen("he_tb_tx="),
			len - strlen("he_tb_tx="), HE_TB_TX_STR_LEN - 1);

	tmp = string;
	pos = strsep(&string, " \t");
	if (pos)
		d->enable = (t_u32)woal_string_to_number(pos);

	if (d->enable == MFALSE)
		goto done;
	pos = strsep(&string, " \t");
	if (pos)
		d->qnum = (t_u32)woal_string_to_number(pos);

	pos = strsep(&string, " \t");
	if (pos)
		d->aid = (t_u32)woal_string_to_number(pos);

	pos = strsep(&string, " \t");
	if (pos)
		d->axq_mu_timer = (t_u32)woal_string_to_number(pos);

	pos = strsep(&string, " \t");
	if (pos)
		d->tx_power = (t_u16)woal_string_to_number(pos);

	if (d->enable > 1)
		ret = -EINVAL;

done:
	kfree(tmp);
	LEAVE();
	return ret;
}
/*
 *  @brief This function enables/disables RF test mode in firmware
 *
 *  @param handle   A pointer to moal_handle structure
 *  @return         MLAN_STATUS_SUCCESS/MLAN_STATUS_PENDING on success,
 *                  otherwise failure code
 */
mlan_status woal_process_rf_test_mode(moal_handle *handle, t_u32 mode)
{
	mlan_status ret = MLAN_STATUS_FAILURE;
	mlan_ioctl_req *req = NULL;
	mlan_ds_misc_cfg *misc = NULL;
	t_u32 flag = 0;

	ENTER();
#ifdef MFG_CMD_SUPPORT
	if (mfg_mode) {
		LEAVE();
		return ret;
	}
#endif
	if (mode != MFG_CMD_SET_TEST_MODE && mode != MFG_CMD_UNSET_TEST_MODE) {
		LEAVE();
		return ret;
	}

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req) {
		misc = (mlan_ds_misc_cfg *)req->pbuf;
		misc->sub_command = MLAN_OID_MISC_RF_TEST_GENERIC;
		req->req_id = MLAN_IOCTL_MISC_CFG;
		req->action = MLAN_ACT_SET;
		if (mode == MFG_CMD_SET_TEST_MODE)
			misc->param.mfg_generic_cfg.mfg_cmd =
				MFG_CMD_SET_TEST_MODE;
		ret = woal_request_ioctl(woal_get_priv(handle,
						       MLAN_BSS_ROLE_ANY),
					 req, MOAL_IOCTL_WAIT);
	}

	if (ret == MLAN_STATUS_SUCCESS && mode == MFG_CMD_SET_TEST_MODE &&
	    handle->rf_data == NULL) {
		flag = (in_atomic() || irqs_disabled()) ? GFP_ATOMIC :
							  GFP_KERNEL;
		/* Allocate memory to hold RF test mode data */
		handle->rf_data =
			kzalloc(sizeof(struct rf_test_mode_data), flag);
		if (!handle->rf_data)
			PRINTM(MERROR,
			       "Couldn't allocate memory for RF test mode\n");
		handle->rf_test_mode = MTRUE;
		if (handle->rf_data) {
			/* antenna is set to 1 by default */
			handle->rf_data->tx_antenna = 1;
			handle->rf_data->rx_antenna = 1;
		}
	} else if (mode == MFG_CMD_UNSET_TEST_MODE) {
		if (handle->rf_data) {
			/* Free RF test mode data memory */
			kfree(handle->rf_data);
			handle->rf_data = NULL;
		}
		handle->rf_test_mode = MFALSE;
	}
	if (ret != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/*
 *  @brief Parse mfg cmd otp rdwr
 *
 *  @param handle   A pointer to moal_handle structure
 *  @param s        A pointer to user buffer
 *  @param len      Length of user buffer
 *  @param d        A pointer to mfg_cmd_generic_cfg struct
 *  @return         0 on success, -EINVAL otherwise
 */

static int parse_otp_mac_addr_rd_wr_string(const char *s, size_t len,
					   mfg_cmd_otp_mac_addr_rd_wr_t *d)
{
	int ret = MLAN_STATUS_SUCCESS;
	char *string = NULL;
	char *tmp = NULL;
	char *pos = NULL;
	gfp_t flag;
	int i = 0;

	ENTER();
	if (!s || !d) {
		LEAVE();
		return -EINVAL;
	}

	flag = (in_atomic() || irqs_disabled()) ? GFP_ATOMIC : GFP_KERNEL;
	string = kzalloc(OTP_RDWR_LEN, flag);
	if (string == NULL) {
		LEAVE();
		return -ENOMEM;
	}

	moal_memcpy_ext(NULL, string, s + strlen("otp_mac_addr_rd_wr="),
			len - strlen("otp_mac_addr_rd_wr="), OTP_RDWR_LEN - 1);

	tmp = string;
	pos = strsep(&string, " \t");
	d->action = (t_u16)woal_string_to_number(pos);
	if (d->action == MFALSE)
		goto done;

	pos = strsep(&string, " \t");
	if (pos) {
		char *begin, *end;
		begin = pos;
		for (i = 0; i < MLAN_MAC_ADDR_LENGTH; i++) {
			end = woal_strsep(&begin, ':', '/');
			if (end)
				d->mac_addr[i] = woal_atox(end);
		}
	}

	if (d->action > 1)
		ret = -EINVAL;

done:
	kfree(tmp);
	LEAVE();
	return ret;
}

/*
 *  @brief Parse mfg cmd otp cal data rdwr
 *
 *  @param handle   A pointer to moal_handle structure
 *  @param s        A pointer to user buffer
 *  @param len      Length of user buffer
 *  @param d        A pointer to mfg_cmd_generic_cfg struct
 *  @return         0 on success, -EINVAL otherwise
 */

static int parse_otp_cal_data_rd_wr_string(const char *s, size_t len,
					   mfg_cmd_otp_cal_data_rd_wr_t *d)
{
	int ret = MLAN_STATUS_SUCCESS;
	char *string = NULL;
	char *pos = NULL;
	int i = 0;

	ENTER();
	if (!s || !d) {
		LEAVE();
		return -EINVAL;
	}
	// Typecasting is done to read the value
	// coverity[misra_c_2012_rule_11_8_violation:SUPPRESS]
	string = (char *)&(s[strlen("otp_cal_data_rd_wr=")]);
	pos = strsep(&string, " \t");
	d->action = (t_u16)woal_string_to_number(pos);
	if (d->action == MFALSE)
		goto done;

	for (i = 0; i < CAL_DATA_LEN; i++) {
		pos = strsep(&string, " \t");
		if (pos) {
			d->cal_data[i] = (t_u16)woal_string_to_number(pos);
			d->cal_data_len++;
		}
	}
	if (d->action > 1)
		ret = -EINVAL;
done:
	LEAVE();
	return ret;
}

/**
 *  @brief This function sends RF test mode command in firmware
 *
 *  @param handle   A pointer to moal_handle structure
 *  @return         MLAN_STATUS_SUCCESS/MLAN_STATUS_PENDING on success,
 *                      otherwise failure code
 */
mlan_status woal_process_rf_test_mode_cmd(moal_handle *handle, t_u32 cmd,
					  const char *buffer, size_t len,
					  t_u32 action, t_u32 val)
{
	mlan_status ret = MLAN_STATUS_FAILURE;
	mlan_ioctl_req *req = NULL;
	mlan_ds_misc_cfg *misc = NULL;
	int err = MFALSE;
	int i;

	ENTER();

	if (!handle->rf_test_mode || !handle->rf_data)
		goto done;

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (!req)
		goto done;

	misc = (mlan_ds_misc_cfg *)req->pbuf;
	req->req_id = MLAN_IOCTL_MISC_CFG;
	misc->sub_command = MLAN_OID_MISC_RF_TEST_GENERIC;
	req->action = action;

	switch (cmd) {
	case MFG_CMD_TX_ANT:
		if (val != 1 && val != 2 && val != 3)
			err = MTRUE;
		break;
	case MFG_CMD_RX_ANT:
		if (val != 1 && val != 2 && val != 3)
			err = MTRUE;
		break;
	case MFG_CMD_RF_BAND_AG:
		if (val != 0 && val != 1 && val != 2)
			err = MTRUE;
		break;
	case MFG_CMD_RF_CHANNELBW:
		if (val != 0 && val != 1 &&
		    (val != 4 ||
		     (val == 4 && handle->rf_data->band == BAND_2GHZ)))
			err = MTRUE;
		break;
	case MFG_CMD_RF_CHAN:
		break;
	case MFG_CMD_CLR_RX_ERR:
		break;
	case MFG_CMD_RADIO_MODE_CFG:
		if (parse_radio_mode_string(buffer, len,
					    &misc->param.mfg_generic_cfg))
			err = MTRUE;
		break;
	case MFG_CMD_RFPWR:
		if (parse_tx_pwr_string(handle, buffer, len,
					&misc->param.mfg_generic_cfg))
			err = MTRUE;
		break;
	case MFG_CMD_TX_CONT:
		misc->sub_command = MLAN_OID_MISC_RF_TEST_TX_CONT;
		if (parse_tx_cont_string(buffer, len, &misc->param.mfg_tx_cont))
			err = MTRUE;
		break;
	case MFG_CMD_TX_FRAME:
		misc->sub_command = MLAN_OID_MISC_RF_TEST_TX_FRAME;
		if (parse_tx_frame_string(handle, buffer, len,
					  &misc->param.mfg_tx_frame2))
			err = MTRUE;
		break;
	case MFG_CMD_CONFIG_MAC_HE_TB_TX:
		misc->sub_command = MLAN_OID_MISC_RF_TEST_HE_POWER;
		if (parse_he_tb_tx_string(buffer, len,
					  &misc->param.mfg_he_power))
			err = MTRUE;
		break;
	case MFG_CMD_CONFIG_TRIGGER_FRAME:
		misc->sub_command = MLAN_OID_MISC_RF_TEST_CONFIG_TRIGGER_FRAME;
		if (parse_trigger_frame_string(
			    buffer, len, &misc->param.mfg_tx_trigger_config))
			err = MTRUE;
		break;
	case MFG_CMD_OTP_MAC_ADD:
		misc->sub_command = MLAN_OID_MISC_OTP_MAC_RD_WR;
		if (parse_otp_mac_addr_rd_wr_string(
			    buffer, len, &misc->param.mfg_otp_mac_addr_rd_wr)) {
			err = MTRUE;
		}
		break;
	case MFG_CMD_OTP_CAL_DATA:
		misc->sub_command = MLAN_OID_MISC_OTP_CAL_DATA_RD_WR;
		if (parse_otp_cal_data_rd_wr_string(
			    buffer, len, &misc->param.mfg_otp_cal_data_rd_wr)) {
			err = MTRUE;
		}
		break;
	default:
		err = MTRUE;
	}

	if (!err) {
		misc->param.mfg_generic_cfg.mfg_cmd = (t_u32)cmd;
		if (cmd != MFG_CMD_RFPWR &&
		    misc->sub_command == MLAN_OID_MISC_RF_TEST_GENERIC)
			misc->param.mfg_generic_cfg.data1 = val;

		ret = woal_request_ioctl(woal_get_priv(handle,
						       MLAN_BSS_ROLE_ANY),
					 req, MOAL_IOCTL_WAIT);
	}
	if (ret != MLAN_STATUS_SUCCESS)
		goto done;

	switch (cmd) {
	case MFG_CMD_TX_ANT:
		handle->rf_data->tx_antenna = misc->param.mfg_generic_cfg.data1;
		break;
	case MFG_CMD_RX_ANT:
		handle->rf_data->rx_antenna = misc->param.mfg_generic_cfg.data1;
		break;
	case MFG_CMD_RADIO_MODE_CFG:
		handle->rf_data->radio_mode[0] =
			misc->param.mfg_generic_cfg.data1;
		handle->rf_data->radio_mode[1] =
			misc->param.mfg_generic_cfg.data2;
		break;
	case MFG_CMD_RF_BAND_AG:
		handle->rf_data->band = misc->param.mfg_generic_cfg.data1;
		/* set fw default bw and channel config on band change */
		handle->rf_data->bandwidth = CHANNEL_BW_20MHZ;
		if (handle->rf_data->band == BAND_2GHZ)
			handle->rf_data->channel = 6;
		else if (handle->rf_data->band == BAND_5GHZ)
			handle->rf_data->channel = 36;
		else if (handle->rf_data->band == BAND_6GHZ)
			handle->rf_data->channel = 1;
		break;
	case MFG_CMD_RF_CHANNELBW:
		handle->rf_data->bandwidth = misc->param.mfg_generic_cfg.data1;
		break;
	case MFG_CMD_RF_CHAN:
		handle->rf_data->channel = misc->param.mfg_generic_cfg.data1;
		break;
	case MFG_CMD_CLR_RX_ERR:
		handle->rf_data->rx_tot_pkt_count =
			misc->param.mfg_generic_cfg.data1;
		handle->rf_data->rx_mcast_bcast_pkt_count =
			misc->param.mfg_generic_cfg.data2;
		handle->rf_data->rx_pkt_fcs_err_count =
			misc->param.mfg_generic_cfg.data3;
		break;
	case MFG_CMD_RFPWR:
		handle->rf_data->tx_power_data[0] =
			misc->param.mfg_generic_cfg.data1;
		handle->rf_data->tx_power_data[1] =
			misc->param.mfg_generic_cfg.data2;
		handle->rf_data->tx_power_data[2] =
			misc->param.mfg_generic_cfg.data3;
		break;
	case MFG_CMD_TX_CONT:
		handle->rf_data->tx_cont_data[0] =
			misc->param.mfg_tx_cont.enable_tx;
		handle->rf_data->tx_cont_data[1] =
			misc->param.mfg_tx_cont.cw_mode;
		handle->rf_data->tx_cont_data[2] =
			misc->param.mfg_tx_cont.payload_pattern;
		handle->rf_data->tx_cont_data[3] =
			misc->param.mfg_tx_cont.cs_mode;
		handle->rf_data->tx_cont_data[4] =
			misc->param.mfg_tx_cont.act_sub_ch;
		handle->rf_data->tx_cont_data[5] =
			misc->param.mfg_tx_cont.tx_rate;
		break;
	case MFG_CMD_TX_FRAME:
		handle->rf_data->tx_frame_data[0] =
			misc->param.mfg_tx_frame2.enable;
		handle->rf_data->tx_frame_data[1] =
			misc->param.mfg_tx_frame2.data_rate;
		handle->rf_data->tx_frame_data[2] =
			misc->param.mfg_tx_frame2.frame_pattern;
		handle->rf_data->tx_frame_data[3] =
			misc->param.mfg_tx_frame2.frame_length;
		handle->rf_data->tx_frame_data[4] =
			misc->param.mfg_tx_frame2.adjust_burst_sifs;
		handle->rf_data->tx_frame_data[5] =
			misc->param.mfg_tx_frame2.burst_sifs_in_us;
		handle->rf_data->tx_frame_data[6] =
			misc->param.mfg_tx_frame2.short_preamble;
		handle->rf_data->tx_frame_data[7] =
			misc->param.mfg_tx_frame2.act_sub_ch;
		handle->rf_data->tx_frame_data[8] =
			misc->param.mfg_tx_frame2.short_gi;
		handle->rf_data->tx_frame_data[9] =
			misc->param.mfg_tx_frame2.adv_coding;
		handle->rf_data->tx_frame_data[10] =
			misc->param.mfg_tx_frame2.tx_bf;
		handle->rf_data->tx_frame_data[11] =
			misc->param.mfg_tx_frame2.gf_mode;
		handle->rf_data->tx_frame_data[12] =
			misc->param.mfg_tx_frame2.stbc;
		handle->rf_data->tx_frame_data[13] =
			misc->param.mfg_tx_frame2.signal_bw;
		handle->rf_data->tx_frame_data[14] =
			misc->param.mfg_tx_frame2.NumPkt;
		handle->rf_data->tx_frame_data[15] =
			misc->param.mfg_tx_frame2.MaxPE;
		handle->rf_data->tx_frame_data[16] =
			misc->param.mfg_tx_frame2.BeamChange;
		handle->rf_data->tx_frame_data[17] =
			misc->param.mfg_tx_frame2.Dcm;
		handle->rf_data->tx_frame_data[18] =
			misc->param.mfg_tx_frame2.Doppler;
		handle->rf_data->tx_frame_data[19] =
			misc->param.mfg_tx_frame2.MidP;
		handle->rf_data->tx_frame_data[20] =
			misc->param.mfg_tx_frame2.QNum;
		for (i = 0; i < ETH_ALEN; i++) {
			handle->rf_data->bssid[i] =
				misc->param.mfg_tx_frame2.bssid[i];
		}
		break;
	case MFG_CMD_CONFIG_MAC_HE_TB_TX:
		handle->rf_data->he_tb_tx[0] = misc->param.mfg_he_power.enable;
		handle->rf_data->he_tb_tx[1] = misc->param.mfg_he_power.qnum;
		handle->rf_data->he_tb_tx[2] = misc->param.mfg_he_power.aid;
		handle->rf_data->he_tb_tx[3] =
			misc->param.mfg_he_power.axq_mu_timer;
		handle->rf_data->he_tb_tx_power[0] =
			misc->param.mfg_he_power.tx_power;
		break;
	case MFG_CMD_CONFIG_TRIGGER_FRAME:
		handle->rf_data->mfg_tx_trigger_config.enable_tx =
			misc->param.mfg_tx_trigger_config.enable_tx;
		handle->rf_data->mfg_tx_trigger_config.standalone_hetb =
			misc->param.mfg_tx_trigger_config.standalone_hetb;
		handle->rf_data->mfg_tx_trigger_config.frmCtl.type =
			misc->param.mfg_tx_trigger_config.frmCtl.type;
		handle->rf_data->mfg_tx_trigger_config.frmCtl.sub_type =
			misc->param.mfg_tx_trigger_config.frmCtl.sub_type;
		handle->rf_data->mfg_tx_trigger_config.duration =
			misc->param.mfg_tx_trigger_config.duration;
		handle->rf_data->mfg_tx_trigger_config.trig_common_field =
			misc->param.mfg_tx_trigger_config.trig_common_field;
		handle->rf_data->mfg_tx_trigger_config.trig_user_info_field =
			misc->param.mfg_tx_trigger_config.trig_user_info_field;
		handle->rf_data->mfg_tx_trigger_config.basic_trig_user_info =
			misc->param.mfg_tx_trigger_config.basic_trig_user_info;
		break;
	case MFG_CMD_OTP_MAC_ADD:
		handle->rf_data->mfg_otp_mac_addr_rd_wr.action =
			misc->param.mfg_otp_mac_addr_rd_wr.action;
		for (i = 0; i < ETH_ALEN; i++) {
			handle->rf_data->mfg_otp_mac_addr_rd_wr.mac_addr[i] =
				misc->param.mfg_otp_mac_addr_rd_wr.mac_addr[i];
		}
		break;
	case MFG_CMD_OTP_CAL_DATA:
		handle->rf_data->mfg_otp_cal_data_rd_wr.action =
			misc->param.mfg_otp_cal_data_rd_wr.action;
		for (i = 0; i < misc->param.mfg_otp_cal_data_rd_wr.cal_data_len;
		     i++) {
			handle->rf_data->mfg_otp_cal_data_rd_wr.cal_data[i] =
				misc->param.mfg_otp_cal_data_rd_wr.cal_data[i];
		}
		break;
	}
done:
	if (err || ret != MLAN_STATUS_PENDING)
		kfree(req);

	LEAVE();
	return ret;
}
#endif /* RF_TEST_MODE */

/**
 *  @brief Configures edmac parameters based on region
 *
 *  @param priv         A pointer to moal_private structure
 *  @param country_code A pointer to country code
 *  @return             MLAN_STATUS_SUCCESS/MLAN_STATUS_PENDING on success,
 *                      otherwise failure code
 */
mlan_status woal_edmac_cfg(moal_private *priv, t_u8 *country_code)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_misc_cfg *cfg = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;
	BOOLEAN is_etsi = MFALSE;

	ENTER();

	/* If WACP enabled, EDMAC / EU Adaptivity enabled (all countries /
	 * regions) */
	if (priv->phandle->params.wacp_mode)
		is_etsi = MTRUE;
	else {
		is_etsi = woal_is_etsi_country(country_code);

		if (is_etsi == MFALSE &&
		    priv->phandle->is_edmac_enabled == MFALSE)
			return MLAN_STATUS_SUCCESS;

		if (is_etsi == MTRUE &&
		    priv->phandle->is_edmac_enabled == MTRUE)
			return MLAN_STATUS_SUCCESS;
	}

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		status = MLAN_STATUS_FAILURE;
		goto done;
	}

	cfg = (mlan_ds_misc_cfg *)req->pbuf;
	cfg->sub_command = MLAN_OID_MISC_EDMAC_CONFIG;
	req->req_id = MLAN_IOCTL_MISC_CFG;

	if (is_etsi)
		req->action = MLAN_ACT_SET;
	else
		req->action = MLAN_ACT_CLEAR;

	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		PRINTM(MMSG, "Failed to configure edmac\n");
		goto done;
	}

	if (is_etsi)
		priv->phandle->is_edmac_enabled = MTRUE;
	else
		priv->phandle->is_edmac_enabled = MFALSE;
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return status;
}

/**
 *  @brief Configures htc_cap by hostcmd
 *
 *  @param priv         A pointer to moal_private structure
 *  @param action       MLAN_ACT_SET or MLAN_ACT_GET
 *  @param enable       A pointer to enable status
 *  @return             MLAN_STATUS_SUCCESS/MLAN_STATUS_PENDING on success,
 *                      otherwise failure code
 */
mlan_status woal_ioctl_hostcmd_htc_cap(moal_private *priv, t_u16 action,
				       t_u8 *enable)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_misc_cfg *misc = NULL;
	HostCmd_DS_802_11_DBGS_CFG *debug_cmd = NULL;
	mlan_status ret = MLAN_STATUS_SUCCESS;

	ENTER();

	/* Allocate an IOCTL request buffer */
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}
	/* Fill request buffer */
	misc = (mlan_ds_misc_cfg *)req->pbuf;
	misc->sub_command = MLAN_OID_MISC_HOST_CMD;
	req->req_id = MLAN_IOCTL_MISC_CFG;
	req->action = action;

	debug_cmd = (HostCmd_DS_802_11_DBGS_CFG *)misc->param.hostcmd.cmd;
	debug_cmd->header.command = HostCmd_CMD_DBGS_CFG;
	debug_cmd->header.size =
		sizeof(HostCmd_DS_802_11_DBGS_CFG) + sizeof(t_u8) /* value */;
	debug_cmd->action = action;
	debug_cmd->subid = DEBUG_SUBID_ENABLE_HEHTC;
	misc->param.hostcmd.len = debug_cmd->header.size;
	if (action == MLAN_ACT_SET)
		debug_cmd->data[0] = *enable;
	/* Send IOCTL request to MLAN */
	ret = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (ret != MLAN_STATUS_SUCCESS)
		goto done;
	if (action == MLAN_ACT_GET)
		*enable = debug_cmd->data[0];
done:
	if (ret != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}
