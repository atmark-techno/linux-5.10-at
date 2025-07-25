/** @file  moal_wext.c
 *
 * @brief This file contains wireless extension standard ioctl functions
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

/************************************************************************
Change log:
    10/21/2008: initial version
************************************************************************/

#include "moal_main.h"

#ifdef STA_SUPPORT
/** Approximate amount of data needed to pass a scan result back to iwlist */
#define MAX_SCAN_CELL_SIZE                                                     \
	(IW_EV_ADDR_LEN + MLAN_MAX_SSID_LENGTH + IW_EV_UINT_LEN +              \
	 IW_EV_FREQ_LEN + IW_EV_QUAL_LEN + MLAN_MAX_SSID_LENGTH +              \
	 IW_EV_PARAM_LEN + 40) /* 40 for WPAIE */

/********************************************************
			Local Variables
********************************************************/
/**
 * iwpriv ioctl handlers
 */
static const struct iw_priv_args woal_private_args[] = {
	{WOAL_SETONEINT_GETWORDCHAR, IW_PRIV_TYPE_INT | 1,
	 IW_PRIV_TYPE_CHAR | 128, ""},
	{WOAL_VERSION, IW_PRIV_TYPE_INT | 1, IW_PRIV_TYPE_CHAR | 128,
	 "version"},
	{WOAL_VEREXT, IW_PRIV_TYPE_INT | 1, IW_PRIV_TYPE_CHAR | 128, "verext"},
	{WOAL_SETNONE_GETNONE, IW_PRIV_TYPE_NONE, IW_PRIV_TYPE_NONE, ""},
	{WOAL_WARMRESET, IW_PRIV_TYPE_NONE, IW_PRIV_TYPE_NONE, "warmreset"},
#ifdef CONFIG_USB_SUSPEND
	{WOAL_USB_SUSPEND, IW_PRIV_TYPE_NONE, IW_PRIV_TYPE_NONE, "usbsuspend"},
	{WOAL_USB_RESUME, IW_PRIV_TYPE_NONE, IW_PRIV_TYPE_NONE, "usbresume"},
#endif /* CONFIG_USB_SUSPEND */
	{WOAL_SETONEINT_GETONEINT, IW_PRIV_TYPE_INT | 1, IW_PRIV_TYPE_INT | 1,
	 ""},
	{WOAL_SET_GET_TXRATE, IW_PRIV_TYPE_INT | 1, IW_PRIV_TYPE_INT | 1,
	 "txratecfg"},
	{WOAL_SET_GET_REGIONCODE, IW_PRIV_TYPE_INT | 1, IW_PRIV_TYPE_INT | 1,
	 "regioncode"},
	{WOAL_SET_RADIO, IW_PRIV_TYPE_INT | 1, IW_PRIV_TYPE_INT | 1,
	 "radioctrl"},
	{WOAL_WMM_ENABLE, IW_PRIV_TYPE_INT | 1, IW_PRIV_TYPE_INT | 1, "wmmcfg"},
	{WOAL_11D_ENABLE, IW_PRIV_TYPE_INT | 1, IW_PRIV_TYPE_INT | 1, "11dcfg"},
	{WOAL_11D_CLR_CHAN_TABLE, IW_PRIV_TYPE_NONE, IW_PRIV_TYPE_NONE,
	 "11dclrtbl"},
	{WOAL_SET_GET_QOS_CFG, IW_PRIV_TYPE_INT | 1, IW_PRIV_TYPE_INT | 1,
	 "qoscfg"},
#ifndef OPCHAN
	{WOAL_SET_GET_WWS_CFG, IW_PRIV_TYPE_INT | 1, IW_PRIV_TYPE_INT | 1,
	 "wwscfg"},
#endif
#if defined(REASSOCIATION)
	{WOAL_SET_GET_REASSOC, IW_PRIV_TYPE_INT | 1, IW_PRIV_TYPE_INT | 1,
	 "reassoctrl"},
#endif
	{WOAL_TXBUF_CFG, IW_PRIV_TYPE_INT | 1, IW_PRIV_TYPE_INT | 1,
	 "txbufcfg"},
	{WOAL_SLEEP_PD, IW_PRIV_TYPE_INT | 1, IW_PRIV_TYPE_INT | 1, "sleeppd"},
	{WOAL_FW_WAKEUP_METHOD, IW_PRIV_TYPE_INT | 1, IW_PRIV_TYPE_INT | 1,
	 "fwwakeupmethod"},
	{WOAL_AUTH_TYPE, IW_PRIV_TYPE_INT | 1, IW_PRIV_TYPE_INT | 1,
	 "authtype"},
	{WOAL_PORT_CTRL, IW_PRIV_TYPE_INT | 1, IW_PRIV_TYPE_INT | 1,
	 "port_ctrl"},
#ifdef WIFI_DIRECT_SUPPORT
#if defined(STA_SUPPORT) && defined(UAP_SUPPORT)
	{WOAL_SET_GET_BSS_ROLE, IW_PRIV_TYPE_INT | 1, IW_PRIV_TYPE_INT | 1,
	 "bssrole"},
#endif
#endif
	{WOAL_SET_GET_11H_LOCAL_PWR_CONSTRAINT, IW_PRIV_TYPE_INT | 1,
	 IW_PRIV_TYPE_INT | 1, "powercons"},
	{WOAL_HT_STREAM_CFG, IW_PRIV_TYPE_INT | 1, IW_PRIV_TYPE_INT | 1,
	 "htstreamcfg"},
	{WOAL_MAC_CONTROL, IW_PRIV_TYPE_INT | 1, IW_PRIV_TYPE_INT | 1,
	 "macctrl"},
	{WOAL_THERMAL, IW_PRIV_TYPE_INT | 1, IW_PRIV_TYPE_INT | 1, "thermal"},
	{WOAL_CFG_HOTSPOT, IW_PRIV_TYPE_INT | 1, IW_PRIV_TYPE_INT | 1,
	 "hotspotcfg"},
	{WOAL_SET_GET_SIXTEEN_INT, IW_PRIV_TYPE_INT | 16, IW_PRIV_TYPE_INT | 16,
	 ""},
	{WOAL_TX_POWERCFG, IW_PRIV_TYPE_INT | 16, IW_PRIV_TYPE_INT | 16,
	 "txpowercfg"},
#ifdef DEBUG_LEVEL1
	{WOAL_DRV_DBG, IW_PRIV_TYPE_INT | 16, IW_PRIV_TYPE_INT | 16, "drvdbg"},
#endif
	{WOAL_BEACON_INTERVAL, IW_PRIV_TYPE_INT | 16, IW_PRIV_TYPE_INT | 16,
	 "bcninterval"},
	{WOAL_SIGNAL, IW_PRIV_TYPE_INT | 16, IW_PRIV_TYPE_INT | 16,
	 "getsignal"},
	{
		WOAL_DEEP_SLEEP,
		IW_PRIV_TYPE_INT | 16,
		IW_PRIV_TYPE_INT | 16,
		"deepsleep",
	},
	{WOAL_11N_TX_CFG, IW_PRIV_TYPE_INT | 16, IW_PRIV_TYPE_INT | 16,
	 "httxcfg"},
	{WOAL_11N_HTCAP_CFG, IW_PRIV_TYPE_INT | 16, IW_PRIV_TYPE_INT | 16,
	 "htcapinfo"},
	{WOAL_PRIO_TBL, IW_PRIV_TYPE_INT | 16, IW_PRIV_TYPE_INT | 16,
	 "aggrpriotbl"},
	{WOAL_11N_AMSDU_AGGR_CTRL, IW_PRIV_TYPE_INT | 16, IW_PRIV_TYPE_INT | 16,
	 "amsduaggrctrl"},
	{WOAL_ADDBA_UPDT, IW_PRIV_TYPE_INT | 16, IW_PRIV_TYPE_INT | 16,
	 "addbapara"},
	{WOAL_ADDBA_REJECT, IW_PRIV_TYPE_INT | 16, IW_PRIV_TYPE_INT | 16,
	 "addbareject"},
	{WOAL_TX_BF_CAP, IW_PRIV_TYPE_INT | 16, IW_PRIV_TYPE_INT | 16,
	 "httxbfcap"},
	{WOAL_HS_CFG, IW_PRIV_TYPE_INT | 16, IW_PRIV_TYPE_INT | 16, "hscfg"},
	{WOAL_HS_SETPARA, IW_PRIV_TYPE_INT | 16, IW_PRIV_TYPE_INT | 16,
	 "hssetpara"},
	{WOAL_REG_READ_WRITE, IW_PRIV_TYPE_INT | 16, IW_PRIV_TYPE_INT | 16,
	 "regrdwr"},
	{WOAL_BAND_CFG, IW_PRIV_TYPE_INT | 16, IW_PRIV_TYPE_INT | 16,
	 "bandcfg"},
	{WOAL_INACTIVITY_TIMEOUT_EXT, IW_PRIV_TYPE_INT | 16,
	 IW_PRIV_TYPE_INT | 16, "inactivityto"},
#ifdef SDIO
	{WOAL_SDIO_CLOCK, IW_PRIV_TYPE_INT | 16, IW_PRIV_TYPE_INT | 16,
	 "sdioclock"},
	{WOAL_CMD_52RDWR, IW_PRIV_TYPE_INT | 16, IW_PRIV_TYPE_INT | 16,
	 "sdcmd52rw"},
#endif
	{WOAL_SCAN_CFG, IW_PRIV_TYPE_INT | 16, IW_PRIV_TYPE_INT | 16,
	 "scancfg"},
	{WOAL_PS_CFG, IW_PRIV_TYPE_INT | 16, IW_PRIV_TYPE_INT | 16, "pscfg"},
	{WOAL_MEM_READ_WRITE, IW_PRIV_TYPE_INT | 16, IW_PRIV_TYPE_INT | 16,
	 "memrdwr"},
#ifdef SDIO
	{WOAL_SDIO_MPA_CTRL, IW_PRIV_TYPE_INT | 16, IW_PRIV_TYPE_INT | 16,
	 "mpactrl"},
#endif
	{WOAL_SLEEP_PARAMS, IW_PRIV_TYPE_INT | 16, IW_PRIV_TYPE_INT | 16,
	 "sleepparams"},
	{WOAL_NET_MONITOR, IW_PRIV_TYPE_INT | 16, IW_PRIV_TYPE_INT | 16,
	 "netmon"},
	{WOAL_DFS_TESTING, IW_PRIV_TYPE_INT | 16, IW_PRIV_TYPE_INT | 16,
	 "dfstesting"},
	{WOAL_MGMT_FRAME_CTRL, IW_PRIV_TYPE_INT | 16, IW_PRIV_TYPE_INT | 16,
	 "mgmtframectrl"},
	{WOAL_CFP_CODE, IW_PRIV_TYPE_INT | 16, IW_PRIV_TYPE_INT | 16,
	 "cfpcode"},
	{WOAL_SET_GET_TX_RX_ANT, IW_PRIV_TYPE_INT | 16, IW_PRIV_TYPE_INT | 16,
	 "antcfg"},
	{WOAL_IND_RST_CFG, IW_PRIV_TYPE_INT | 16, IW_PRIV_TYPE_INT | 16,
	 "indrstcfg"},
	{WOALGETLOG, IW_PRIV_TYPE_NONE, IW_PRIV_TYPE_CHAR | GETLOG_BUFSIZE,
	 "getlog"},
	{WOAL_SETADDR_GETNONE, IW_PRIV_TYPE_ADDR | 1, IW_PRIV_TYPE_NONE, ""},
	{WOAL_DEAUTH, IW_PRIV_TYPE_ADDR | 1, IW_PRIV_TYPE_NONE, "deauth"},
	{WOAL_SET_GET_256_CHAR, IW_PRIV_TYPE_CHAR | 256,
	 IW_PRIV_TYPE_CHAR | 256, ""},
	{WOAL_PASSPHRASE, IW_PRIV_TYPE_CHAR | 256, IW_PRIV_TYPE_CHAR | 256,
	 "passphrase"},
	{WOAL_GET_KEY, IW_PRIV_TYPE_CHAR | 256, IW_PRIV_TYPE_CHAR | 256,
	 "getkey"},
	{WOAL_ASSOCIATE, IW_PRIV_TYPE_CHAR | 256, IW_PRIV_TYPE_CHAR | 256,
	 "associate"},
	{WOAL_WMM_QUEUE_STATUS, IW_PRIV_TYPE_CHAR | 256,
	 IW_PRIV_TYPE_CHAR | 256, "qstatus"},
	{WOAL_WMM_TS_STATUS, IW_PRIV_TYPE_CHAR | 256, IW_PRIV_TYPE_CHAR | 256,
	 "ts_status"},
	{WOAL_IP_ADDRESS, IW_PRIV_TYPE_CHAR | 256, IW_PRIV_TYPE_CHAR | 256,
	 "ipaddr"},
	{WOAL_TX_BF_CFG, IW_PRIV_TYPE_CHAR | 256, IW_PRIV_TYPE_CHAR | 256,
	 "httxbfcfg"},
	{WOAL_SETNONE_GETTWELVE_CHAR, IW_PRIV_TYPE_NONE, IW_PRIV_TYPE_CHAR | 12,
	 ""},
	{WOAL_WPS_SESSION, IW_PRIV_TYPE_NONE, IW_PRIV_TYPE_CHAR | 12,
	 "wpssession"},
	{WOAL_SETNONE_GET_FOUR_INT, IW_PRIV_TYPE_NONE, IW_PRIV_TYPE_INT | 4,
	 ""},
	{WOAL_DATA_RATE, IW_PRIV_TYPE_NONE, IW_PRIV_TYPE_INT | 4,
	 "getdatarate"},
	{WOAL_ESUPP_MODE, IW_PRIV_TYPE_NONE, IW_PRIV_TYPE_INT | 4, "esuppmode"},
	{WOAL_SET_GET_64_INT, IW_PRIV_TYPE_INT | 64, IW_PRIV_TYPE_INT | 64, ""},
	{WOAL_ECL_SYS_CLOCK, IW_PRIV_TYPE_INT | 64, IW_PRIV_TYPE_INT | 64,
	 "sysclock"},
	{WOAL_HOST_CMD, IW_PRIV_TYPE_BYTE | 2047, IW_PRIV_TYPE_BYTE | 2047,
	 "hostcmd"},
	{WOAL_ARP_FILTER, IW_PRIV_TYPE_BYTE | 2047, IW_PRIV_TYPE_BYTE | 2047,
	 "arpfilter"},
	{WOAL_SET_INTS_GET_CHARS, IW_PRIV_TYPE_INT | 16,
	 IW_PRIV_TYPE_BYTE | 256, ""},
	{WOAL_READ_EEPROM, IW_PRIV_TYPE_INT | 16, IW_PRIV_TYPE_BYTE | 256,
	 "rdeeprom"},
	{WOAL_SET_GET_2K_BYTES, IW_PRIV_TYPE_BYTE | WOAL_2K_BYTES,
	 IW_PRIV_TYPE_BYTE | WOAL_2K_BYTES, ""},
#if defined(SDIO)
	{WOAL_CMD_53RDWR, IW_PRIV_TYPE_BYTE | WOAL_2K_BYTES,
	 IW_PRIV_TYPE_BYTE | WOAL_2K_BYTES, "sdcmd53rw"},
#endif
	{WOAL_SET_USER_SCAN, IW_PRIV_TYPE_BYTE | WOAL_2K_BYTES,
	 IW_PRIV_TYPE_BYTE | WOAL_2K_BYTES, "setuserscan"},
	{WOAL_GET_SCAN_TABLE, IW_PRIV_TYPE_BYTE | WOAL_2K_BYTES,
	 IW_PRIV_TYPE_BYTE | WOAL_2K_BYTES, "getscantable"},
	{WOAL_SET_USER_SCAN_EXT, IW_PRIV_TYPE_BYTE | WOAL_2K_BYTES,
	 IW_PRIV_TYPE_BYTE | WOAL_2K_BYTES, "setuserscanext"},
	{WOAL_WMM_ADDTS, IW_PRIV_TYPE_BYTE | WOAL_2K_BYTES,
	 IW_PRIV_TYPE_BYTE | WOAL_2K_BYTES, "addts"},
	{WOAL_WMM_DELTS, IW_PRIV_TYPE_BYTE | WOAL_2K_BYTES,
	 IW_PRIV_TYPE_BYTE | WOAL_2K_BYTES, "delts"},
	{WOAL_WMM_QUEUE_CONFIG, IW_PRIV_TYPE_BYTE | WOAL_2K_BYTES,
	 IW_PRIV_TYPE_BYTE | WOAL_2K_BYTES, "qconfig"},
	{WOAL_WMM_QUEUE_STATS, IW_PRIV_TYPE_BYTE | WOAL_2K_BYTES,
	 IW_PRIV_TYPE_BYTE | WOAL_2K_BYTES, "qstats"},
	{WOAL_BYPASSED_PACKET, IW_PRIV_TYPE_BYTE | WOAL_2K_BYTES,
	 IW_PRIV_TYPE_BYTE | WOAL_2K_BYTES, "pb_bypass"},
#ifdef UAP_WEXT
	{WOAL_FROYO_START, IW_PRIV_TYPE_NONE, IW_PRIV_TYPE_NONE, "START"},
	{WOAL_FROYO_STOP, IW_PRIV_TYPE_NONE, IW_PRIV_TYPE_NONE, "STOP"},
	{WOAL_FROYO_WL_FW_RELOAD, IW_PRIV_TYPE_CHAR | 256,
	 IW_PRIV_TYPE_CHAR | 256, "WL_FW_RELOAD"},
#endif
};

/********************************************************
			Local Functions
********************************************************/

/**
 *  @brief Sort Channels
 *
 *  @param freq                 A pointer to iw_freq structure
 *  @param num                  Number of Channels
 *
 *  @return                     N/A
 */
static inline void woal_sort_channels(struct iw_freq *freq, int num)
{
	int i, j;
	struct iw_freq temp;

	for (i = 0; i < num; i++)
		for (j = i + 1; j < num; j++)
			if (freq[i].i > freq[j].i) {
				temp.i = freq[i].i;
				temp.m = freq[i].m;

				freq[i].i = freq[j].i;
				freq[i].m = freq[j].m;

				freq[j].i = temp.i;
				freq[j].m = temp.m;
			}
}

/**
 *  @brief Convert RSSI to quality
 *
 *  @param rssi     RSSI in dBm
 *
 *  @return         Quality of the link (0-5)
 */
static t_u8 woal_rssi_to_quality(t_s16 rssi)
{
/** Macro for RSSI range */
#define MOAL_RSSI_NO_SIGNAL -90
#define MOAL_RSSI_VERY_LOW -80
#define MOAL_RSSI_LOW -70
#define MOAL_RSSI_GOOD -60
#define MOAL_RSSI_VERY_GOOD -50
#define MOAL_RSSI_INVALID 0
	if (rssi <= MOAL_RSSI_NO_SIGNAL || rssi == MOAL_RSSI_INVALID)
		return 0;
	else if (rssi <= MOAL_RSSI_VERY_LOW)
		return 1;
	else if (rssi <= MOAL_RSSI_LOW)
		return 2;
	else if (rssi <= MOAL_RSSI_GOOD)
		return 3;
	else if (rssi <= MOAL_RSSI_VERY_GOOD)
		return 4;
	else
		return 5;
}

/**
 *  @brief Set Adapter Node Name
 *
 *  @param dev                  A pointer to net_device structure
 *  @param info                 A pointer to iw_request_info structure
 *  @param wrqu                 A pointer to iwreq_data union
 *  @param extra                A pointer to extra data buf
 *
 *  @return                     0 --success, otherwise fail
 */
static int woal_set_nick(struct net_device *dev, struct iw_request_info *info,
			 union iwreq_data *wrqu, char *extra)
{
	moal_private *priv = (moal_private *)netdev_priv(dev);
	struct iw_point *dwrq = &wrqu->data;
	ENTER();
	/*
	 * Check the size of the string
	 */
	if (dwrq->length > 16) {
		LEAVE();
		return -E2BIG;
	}
	memset(priv->nick_name, 0, sizeof(priv->nick_name));
	moal_memcpy_ext(priv->phandle, priv->nick_name, extra, dwrq->length,
			sizeof(priv->nick_name));
	LEAVE();
	return 0;
}

/**
 *  @brief Get Adapter Node Name
 *
 *  @param dev                  A pointer to net_device structure
 *  @param info                 A pointer to iw_request_info structure
 *  @param wrqu                 A pointer to iwreq_data union
 *  @param extra                A pointer to extra data buf
 *
 *  @return                     0 --success
 */
static int woal_get_nick(struct net_device *dev, struct iw_request_info *info,
			 union iwreq_data *wrqu, char *extra)
{
	moal_private *priv = (moal_private *)netdev_priv(dev);
	struct iw_point *dwrq = &wrqu->data;
	ENTER();
	/*
	 * Get the Nick Name saved
	 */
	strncpy(extra, (char *)priv->nick_name, 16);
	extra[16] = '\0';
	/*
	 * If none, we may want to get the one that was set
	 */

	/*
	 * Push it out !
	 */
	dwrq->length = strlen(extra) + 1;
	LEAVE();
	return 0;
}

/**
 *  @brief Commit handler: called after a bunch of SET operations
 *
 *  @param dev          A pointer to net_device structure
 *  @param info         A pointer to iw_request_info structure
 *  @param cwrq         A pointer to char buffer
 *  @param extra        A pointer to extra data buf
 *
 *  @return             0 --success
 */
static int woal_config_commit(struct net_device *dev,
			      struct iw_request_info *info,
			      union iwreq_data *cwrq, char *extra)
{
	ENTER();

	LEAVE();
	return 0;
}

/**
 *  @brief Get name
 *
 *  @param dev          A pointer to net_device structure
 *  @param info         A pointer to iw_request_info structure
 *  @param cwrq         A pointer to char buffer
 *  @param extra        A pointer to extra data buf
 *
 *  @return             0 --success
 */
static int woal_get_name(struct net_device *dev, struct iw_request_info *info,
			 union iwreq_data *wrqu, char *extra)
{
	char *cwrq = wrqu->name;
	ENTER();
	strncpy(cwrq, "IEEE 802.11-DS", IFNAMSIZ);
	LEAVE();
	return 0;
}

/**
 *  @brief Set frequency
 *
 *  @param dev                  A pointer to net_device structure
 *  @param info                 A pointer to iw_request_info structure
 *  @param fwrq                 A pointer to iw_freq structure
 *  @param extra                A pointer to extra data buf
 *
 *  @return                     0 --success, otherwise fail
 */
static int woal_set_freq(struct net_device *dev, struct iw_request_info *info,
			 union iwreq_data *wrqu, char *extra)
{
	int ret = 0;
	moal_private *priv = (moal_private *)netdev_priv(dev);
	struct iw_freq *fwrq = &wrqu->freq;
	mlan_ds_bss *bss = NULL;
	mlan_ioctl_req *req = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_bss));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}
	bss = (mlan_ds_bss *)req->pbuf;
	/*
	 * If setting by frequency, convert to a channel
	 */
	if (fwrq->e == 1) {
		long f = fwrq->m / 100000;
		bss->param.bss_chan.freq = f;
	} else
		bss->param.bss_chan.channel = fwrq->m;

	bss->sub_command = MLAN_OID_BSS_CHANNEL;
	req->req_id = MLAN_IOCTL_BSS;
	req->action = MLAN_ACT_SET;
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
 *  @brief Get frequency
 *
 *  @param dev                  A pointer to net_device structure
 *  @param info                 A pointer to iw_request_info structure
 *  @param fwrq                 A pointer to iw_freq structure
 *  @param extra                A pointer to extra data buf
 *
 *  @return                     0 --success, otherwise fail
 */
static int woal_get_freq(struct net_device *dev, struct iw_request_info *info,
			 union iwreq_data *wrqu, char *extra)
{
	int ret = 0;
	moal_private *priv = (moal_private *)netdev_priv(dev);
	struct iw_freq *fwrq = &wrqu->freq;
	mlan_ds_bss *bss = NULL;
	mlan_ioctl_req *req = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_bss));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}
	bss = (mlan_ds_bss *)req->pbuf;
	bss->sub_command = MLAN_OID_BSS_CHANNEL;
	req->req_id = MLAN_IOCTL_BSS;
	req->action = MLAN_ACT_GET;
	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}
	fwrq->m = (long)bss->param.bss_chan.freq;
	fwrq->i = (long)bss->param.bss_chan.channel;
	fwrq->e = 6;
	fwrq->flags = IW_FREQ_FIXED;
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Set wlan mode
 *
 *  @param dev                  A pointer to net_device structure
 *  @param info                 A pointer to iw_request_info structure
 *  @param uwrq                 Wireless mode to set
 *  @param extra                A pointer to extra data buf
 *
 *  @return                     0 --success, otherwise fail
 */
static int woal_set_bss_mode(struct net_device *dev,
			     struct iw_request_info *info,
			     union iwreq_data *wrqu, char *extra)
{
	int ret = 0;
	moal_private *priv = (moal_private *)netdev_priv(dev);
	t_u32 *uwrq = &wrqu->mode;
	mlan_ds_bss *bss = NULL;
	mlan_ioctl_req *req = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_bss));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}
	bss = (mlan_ds_bss *)req->pbuf;
	bss->sub_command = MLAN_OID_BSS_MODE;
	req->req_id = MLAN_IOCTL_BSS;
	req->action = MLAN_ACT_SET;

	switch (*uwrq) {
	case IW_MODE_INFRA:
		bss->param.bss_mode = MLAN_BSS_MODE_INFRA;
		break;
	case IW_MODE_AUTO:
		bss->param.bss_mode = MLAN_BSS_MODE_AUTO;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	if (ret)
		goto done;
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
 *  @brief Get current BSSID
 *
 *  @param dev          A pointer to net_device structure
 *  @param info         A pointer to iw_request_info structure
 *  @param awrq         A pointer to sockaddr structure
 *  @param extra        A pointer to extra data buf
 *
 *  @return             0 --success
 */
static int woal_get_wap(struct net_device *dev, struct iw_request_info *info,
			union iwreq_data *wrqu, char *extra)
{
	int ret = 0;
	moal_private *priv = (moal_private *)netdev_priv(dev);
	struct sockaddr *awrq = &wrqu->addr;
	mlan_bss_info bss_info;

	ENTER();

	memset(&bss_info, 0, sizeof(bss_info));

	if (MLAN_STATUS_SUCCESS !=
	    woal_get_bss_info(priv, MOAL_IOCTL_WAIT, &bss_info)) {
		ret = -EFAULT;
		goto done;
	}

	if (bss_info.media_connected == MTRUE)
		moal_memcpy_ext(priv->phandle, awrq->sa_data, &bss_info.bssid,
				MLAN_MAC_ADDR_LENGTH,
				sizeof(awrq->sa_data_min));
	else
		memset(awrq->sa_data, 0, MLAN_MAC_ADDR_LENGTH);
	awrq->sa_family = ARPHRD_ETHER;
done:
	LEAVE();
	return ret;
}

/**
 *  @brief Connect to the AP or Ad-hoc Network with specific bssid
 *
 * NOTE: Scan should be issued by application before this function is called
 *
 *  @param dev          A pointer to net_device structure
 *  @param info         A pointer to iw_request_info structure
 *  @param awrq         A pointer to iw_param structure
 *  @param extra        A pointer to extra data buf
 *
 *  @return             0 --success, otherwise fail
 */
static int woal_set_wap(struct net_device *dev, struct iw_request_info *info,
			union iwreq_data *wrqu, char *extra)
{
	int ret = 0;
	const t_u8 bcast[MLAN_MAC_ADDR_LENGTH] = {255, 255, 255, 255, 255, 255};
	const t_u8 zero_mac[MLAN_MAC_ADDR_LENGTH] = {0, 0, 0, 0, 0, 0};
	moal_private *priv = (moal_private *)netdev_priv(dev);
	struct sockaddr *awrq = &wrqu->addr;
	mlan_ssid_bssid *ssid_bssid = NULL;
	mlan_bss_info bss_info;

	ENTER();

	if (awrq->sa_family != ARPHRD_ETHER) {
		ret = -EINVAL;
		goto done;
	}

	PRINTM(MINFO, "ASSOC: WAP: sa_data: " MACSTR "\n",
	       MAC2STR((t_u8 *)awrq->sa_data));

	if (MLAN_STATUS_SUCCESS !=
	    woal_get_bss_info(priv, MOAL_IOCTL_WAIT, &bss_info)) {
		ret = -EFAULT;
		goto done;
	}

#ifdef REASSOCIATION
	/* Cancel re-association */
	priv->reassoc_required = MFALSE;
#endif

	/* zero_mac means disconnect */
	if (!memcmp(zero_mac, awrq->sa_data, MLAN_MAC_ADDR_LENGTH)) {
		woal_disconnect(priv, MOAL_IOCTL_WAIT, NULL,
				DEF_DEAUTH_REASON_CODE);
		goto done;
	}

	/* Broadcast MAC means search for best network */
	ssid_bssid = kzalloc(sizeof(mlan_ssid_bssid), GFP_ATOMIC);
	if (!ssid_bssid) {
		PRINTM(MERROR, "Fail to allocate ssid_bssid buffer\n");
		ret = -ENOMEM;
		goto done;
	}
	if (memcmp(bcast, awrq->sa_data, MLAN_MAC_ADDR_LENGTH)) {
		/* Check if we are already assoicated to the AP */
		if (bss_info.media_connected == MTRUE) {
			if (!memcmp(awrq->sa_data, &bss_info.bssid, ETH_ALEN))
				goto done;
		}
		moal_memcpy_ext(priv->phandle, &ssid_bssid->bssid,
				awrq->sa_data, ETH_ALEN,
				sizeof(ssid_bssid->bssid));
	}

	if (MLAN_STATUS_SUCCESS !=
	    woal_find_best_network(priv, MOAL_IOCTL_WAIT, ssid_bssid)) {
		PRINTM(MERROR,
		       "ASSOC: WAP: MAC address not found in BSSID List\n");
		ret = -ENETUNREACH;
		goto done;
	}
	/* Zero SSID implies use BSSID to connect */
	memset(&ssid_bssid->ssid, 0, sizeof(mlan_802_11_ssid));
	if (MLAN_STATUS_SUCCESS !=
	    woal_bss_start(priv, MOAL_IOCTL_WAIT, ssid_bssid)) {
		ret = -EFAULT;
		goto done;
	}

#ifdef REASSOCIATION
	memset(&bss_info, 0, sizeof(bss_info));
	if (MLAN_STATUS_SUCCESS !=
	    woal_get_bss_info(priv, MOAL_IOCTL_WAIT, &bss_info)) {
		ret = -EFAULT;
		goto done;
	}
	moal_memcpy_ext(priv->phandle, &priv->prev_ssid_bssid.ssid,
			&bss_info.ssid, sizeof(mlan_802_11_ssid),
			sizeof(priv->prev_ssid_bssid.ssid));
	moal_memcpy_ext(priv->phandle, &priv->prev_ssid_bssid.bssid,
			&bss_info.bssid, MLAN_MAC_ADDR_LENGTH,
			sizeof(priv->prev_ssid_bssid.bssid));
#endif /* REASSOCIATION */

done:
	if (ssid_bssid)
		kfree(ssid_bssid);
	LEAVE();
	return ret;
}

/**
 *  @brief Get wlan mode
 *
 *  @param dev                  A pointer to net_device structure
 *  @param info                 A pointer to iw_request_info structure
 *  @param uwrq                 A pointer to t_u32 string
 *  @param extra                A pointer to extra data buf
 *
 *  @return                     0 --success
 */
static int woal_get_bss_mode(struct net_device *dev,
			     struct iw_request_info *info,
			     union iwreq_data *wrqu, char *extra)
{
	moal_private *priv = (moal_private *)netdev_priv(dev);
	t_u32 *uwrq = &wrqu->mode;
	ENTER();
	*uwrq = woal_get_mode(priv, MOAL_IOCTL_WAIT);
	LEAVE();
	return 0;
}

/**
 *  @brief Set sensitivity
 *
 *  @param dev                  A pointer to net_device structure
 *  @param info                 A pointer to iw_request_info structure
 *  @param vwrq                 A pointer to iw_param structure
 *  @param extra                A pointer to extra data buf
 *
 *  @return                     0 --success
 */
static int woal_set_sens(struct net_device *dev, struct iw_request_info *info,
			 union iwreq_data *wrqu, char *extra)
{
	int ret = 0;

	ENTER();

	LEAVE();
	return ret;
}

/**
 *  @brief Get sensitivity
 *
 *  @param dev                  A pointer to net_device structure
 *  @param info                 A pointer to iw_request_info structure
 *  @param vwrq                 A pointer to iw_param structure
 *  @param extra                A pointer to extra data buf
 *
 *  @return                     -1
 */
static int woal_get_sens(struct net_device *dev, struct iw_request_info *info,
			 union iwreq_data *wrqu, char *extra)
{
	int ret = -1;

	ENTER();

	LEAVE();
	return ret;
}

/**
 *  @brief Set Tx power
 *
 *  @param dev                  A pointer to net_device structure
 *  @param info                 A pointer to iw_request_info structure
 *  @param vwrq                 A pointer to iwreq_data structure
 *  @param extra                A pointer to extra data buf
 *
 *  @return                     0 --success, otherwise fail
 */
static int woal_set_txpow(struct net_device *dev, struct iw_request_info *info,
			  union iwreq_data *vwrq, char *extra)
{
	int ret = 0;
	struct iw_param *vwrq_ = (struct iw_param *)vwrq;
	moal_private *priv = (moal_private *)netdev_priv(dev);
	mlan_power_cfg_t power_cfg;

	ENTER();
	memset(&power_cfg, 0, sizeof(mlan_power_cfg_t));
	if (vwrq_->disabled) {
		woal_set_radio(priv, 0);
		goto done;
	}
	woal_set_radio(priv, 1);

	if (!vwrq_->fixed)
		power_cfg.is_power_auto = 1;
	else {
		power_cfg.is_power_auto = 0;
		power_cfg.power_level = vwrq_->value;
	}

	if (MLAN_STATUS_SUCCESS !=
	    woal_set_get_tx_power(priv, MLAN_ACT_SET, &power_cfg)) {
		ret = -EFAULT;
		goto done;
	}

done:
	LEAVE();
	return ret;
}

/**
 *  @brief Get Tx power
 *
 *  @param dev                  A pointer to net_device structure
 *  @param info                 A pointer to iw_request_info structure
 *  @param vwrq                 A pointer to iwreq_data structure
 *  @param extra                A pointer to extra data buf
 *
 *  @return                     0 --success, otherwise fail
 */
static int woal_get_txpow(struct net_device *dev, struct iw_request_info *info,
			  union iwreq_data *vwrq, char *extra)
{
	int ret = 0;
	struct iw_param *vwrq_ = (struct iw_param *)vwrq;
	moal_private *priv = (moal_private *)netdev_priv(dev);
	mlan_power_cfg_t power_cfg;
	mlan_bss_info bss_info;

	ENTER();

	memset(&power_cfg, 0, sizeof(mlan_power_cfg_t));
	memset(&bss_info, 0, sizeof(bss_info));
	if (MLAN_STATUS_SUCCESS !=
	    woal_get_bss_info(priv, MOAL_IOCTL_WAIT, &bss_info)) {
		ret = -EFAULT;
		goto done;
	}

	if (MLAN_STATUS_SUCCESS !=
	    woal_set_get_tx_power(priv, MLAN_ACT_GET, &power_cfg)) {
		ret = -EFAULT;
		goto done;
	}

	vwrq_->value = power_cfg.power_level;
	if (power_cfg.is_power_auto)
		vwrq_->fixed = 0;
	else
		vwrq_->fixed = 1;
	if (bss_info.radio_on) {
		vwrq_->disabled = 0;
		vwrq_->flags = IW_TXPOW_DBM;
	} else {
		vwrq_->disabled = 1;
	}

done:
	LEAVE();
	return ret;
}

/**
 *  @brief  Set power management
 *
 *  @param dev                  A pointer to net_device structure
 *  @param info                 A pointer to iw_request_info structure
 *  @param wrqu                 A pointer to iwreq_data union
 *  @param extra                A pointer to extra data buf
 *
 *  @return                     MLAN_STATUS_SUCCESS --success, otherwise fail
 */
static int woal_set_power(struct net_device *dev, struct iw_request_info *info,
			  union iwreq_data *wrqu, char *extra)
{
	int ret = 0, disabled;
	struct iw_param *vwrq = &wrqu->param;
	moal_private *priv = (moal_private *)netdev_priv(dev);

	ENTER();

	if (moal_extflg_isset(priv->phandle, EXT_HW_TEST)) {
		PRINTM(MIOCTL, "block set power in hw_test mode\n");
		LEAVE();
		return ret;
	}
	disabled = vwrq->disabled;

	if (MLAN_STATUS_SUCCESS !=
	    woal_set_get_power_mgmt(priv, MLAN_ACT_SET, &disabled, vwrq->flags,
				    MOAL_IOCTL_WAIT)) {
		ret = -EFAULT;
	}

	LEAVE();
	return ret;
}

/**
 *  @brief  Get power management
 *
 *  @param dev                  A pointer to net_device structure
 *  @param info                 A pointer to iw_request_info structure
 *  @param wrqu                 A pointer to iwreq_data union
 *  @param extra                A pointer to extra data buf
 *
 *  @return                     MLAN_STATUS_SUCCESS --success, otherwise fail
 */
static int woal_get_power(struct net_device *dev, struct iw_request_info *info,
			  union iwreq_data *wrqu, char *extra)
{
	int ret = 0, ps_mode = 0;
	struct iw_param *vwrq = &wrqu->param;
	moal_private *priv = (moal_private *)netdev_priv(dev);

	ENTER();

	if (MLAN_STATUS_SUCCESS != woal_set_get_power_mgmt(priv, MLAN_ACT_GET,
							   &ps_mode, 0,
							   MOAL_IOCTL_WAIT)) {
		ret = -EFAULT;
	}

	if (ps_mode)
		vwrq->disabled = 0;
	else
		vwrq->disabled = 1;

	vwrq->value = 0;

	LEAVE();
	return ret;
}

/**
 *  @brief Set Tx retry count
 *
 *  @param dev                  A pointer to net_device structure
 *  @param info                 A pointer to iw_request_info structure
 *  @param wrqu                 A pointer to iwreq_data union
 *  @param extra                A pointer to extra data buf
 *
 *  @return                     0 --success, otherwise fail
 */
static int woal_set_retry(struct net_device *dev, struct iw_request_info *info,
			  union iwreq_data *wrqu, char *extra)
{
	struct iw_param *vwrq = &wrqu->retry;
	int ret = 0, retry_val = vwrq->value;
	moal_private *priv = (moal_private *)netdev_priv(dev);

	ENTER();

	if (vwrq->flags == IW_RETRY_LIMIT) {
		/*
		 * The MAC has a 4-bit Total_Tx_Count register
		 * Total_Tx_Count = 1 + Tx_Retry_Count
		 */

		if (MLAN_STATUS_SUCCESS !=
		    woal_set_get_retry(priv, MLAN_ACT_SET, MOAL_IOCTL_WAIT,
				       &retry_val)) {
			ret = -EFAULT;
			goto done;
		}
	} else {
		ret = -EOPNOTSUPP;
		goto done;
	}

done:
	LEAVE();
	return ret;
}

/**
 *  @brief Get Tx retry count
 *
 *  @param dev                  A pointer to net_device structure
 *  @param info                 A pointer to iw_request_info structure
 *  @param wrqu                 A pointer to iwreq_data union
 *  @param extra                A pointer to extra data buf
 *
 *  @return                     0 --success, otherwise fail
 */
static int woal_get_retry(struct net_device *dev, struct iw_request_info *info,
			  union iwreq_data *wrqu, char *extra)
{
	int retry_val, ret = 0;
	struct iw_param *vwrq = &wrqu->retry;
	moal_private *priv = (moal_private *)netdev_priv(dev);

	ENTER();

	if (MLAN_STATUS_SUCCESS != woal_set_get_retry(priv, MLAN_ACT_GET,
						      MOAL_IOCTL_WAIT,
						      &retry_val)) {
		ret = -EFAULT;
		goto done;
	}

	vwrq->disabled = 0;
	if (!vwrq->flags) {
		vwrq->flags = IW_RETRY_LIMIT;
		/* Get Tx retry count */
		vwrq->value = retry_val;
	}

done:
	LEAVE();
	return ret;
}

/**
 *  @brief Set encryption key
 *
 *  @param dev                  A pointer to net_device structure
 *  @param info                 A pointer to iw_request_info structure
 *  @param wrqu                 A pointer to iwreq_data union
 *  @param extra                A pointer to extra data buf
 *
 *  @return                     0 --success, otherwise fail
 */
static int woal_set_encode(struct net_device *dev, struct iw_request_info *info,
			   union iwreq_data *wrqu, char *extra)
{
	int ret = 0;
	moal_private *priv = (moal_private *)netdev_priv(dev);
	struct iw_point *dwrq = &wrqu->encoding;
	mlan_ds_sec_cfg *sec = NULL;
	mlan_ioctl_req *req = NULL;
	int index = 0;
	t_u32 auth_mode = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_sec_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}
	sec = (mlan_ds_sec_cfg *)req->pbuf;
	sec->sub_command = MLAN_OID_SEC_CFG_ENCRYPT_KEY;
	req->req_id = MLAN_IOCTL_SEC_CFG;
	req->action = MLAN_ACT_SET;

	/* Check index */
	index = (dwrq->flags & IW_ENCODE_INDEX) - 1;
	if (index > 3) {
		PRINTM(MERROR, "Key index #%d out of range\n", index);
		ret = -EINVAL;
		goto done;
	}

	sec->param.encrypt_key.key_len = 0;
	if (!(dwrq->flags & IW_ENCODE_NOKEY) && dwrq->length) {
		if (dwrq->length > MAX_WEP_KEY_SIZE) {
			PRINTM(MERROR, "Key length (%d) out of range\n",
			       dwrq->length);
			ret = -EINVAL;
			goto done;
		}
		if (index < 0)
			sec->param.encrypt_key.key_index =
				MLAN_KEY_INDEX_DEFAULT;
		else
			sec->param.encrypt_key.key_index = index;
		moal_memcpy_ext(priv->phandle,
				sec->param.encrypt_key.key_material, extra,
				dwrq->length,
				sizeof(sec->param.encrypt_key.key_material));
		/* Set the length */
		if (dwrq->length > MIN_WEP_KEY_SIZE)
			sec->param.encrypt_key.key_len = MAX_WEP_KEY_SIZE;
		else
			sec->param.encrypt_key.key_len = MIN_WEP_KEY_SIZE;
	} else {
		/*
		 * No key provided so it is either enable key,
		 * on or off
		 */
		if (dwrq->flags & IW_ENCODE_DISABLED) {
			PRINTM(MINFO, "*** iwconfig mlanX key off ***\n");
			sec->param.encrypt_key.key_disable = MTRUE;
		} else {
			/*
			 * iwconfig mlanX key [n]
			 * iwconfig mlanX key on
			 * iwconfig mlanX key open
			 * iwconfig mlanX key restricted
			 * Do we want to just set the transmit key index ?
			 */
			if (index < 0) {
				PRINTM(MINFO,
				       "*** iwconfig mlanX key on ***\n");
				sec->param.encrypt_key.key_index =
					MLAN_KEY_INDEX_DEFAULT;
			} else
				sec->param.encrypt_key.key_index = index;
			sec->param.encrypt_key.is_current_wep_key = MTRUE;
		}
	}
	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}
	if (dwrq->flags & (IW_ENCODE_RESTRICTED | IW_ENCODE_OPEN)) {
		switch (dwrq->flags & 0xf000) {
		case IW_ENCODE_RESTRICTED:
			/* iwconfig mlanX restricted key [1] */
			auth_mode = MLAN_AUTH_MODE_SHARED;
			PRINTM(MINFO, "Auth mode restricted!\n");
			break;
		case IW_ENCODE_OPEN:
			/* iwconfig mlanX key [2] open */
			auth_mode = MLAN_AUTH_MODE_OPEN;
			PRINTM(MINFO, "Auth mode open!\n");
			break;
		case IW_ENCODE_RESTRICTED | IW_ENCODE_OPEN:
		default:
			/* iwconfig mlanX key [2] open restricted */
			auth_mode = MLAN_AUTH_MODE_AUTO;
			PRINTM(MINFO, "Auth mode auto!\n");
			break;
		}
		if (MLAN_STATUS_SUCCESS !=
		    woal_set_auth_mode(priv, MOAL_IOCTL_WAIT, auth_mode))
			ret = -EFAULT;
	}
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Get encryption key
 *
 *  @param dev                  A pointer to net_device structure
 *  @param info                 A pointer to iw_request_info structure
 *  @param wrqu                 A pointer to iwreq_data union
 *  @param extra                A pointer to extra data buf
 *
 *  @return                     0 --success, otherwise fail
 */
static int woal_get_encode(struct net_device *dev, struct iw_request_info *info,
			   union iwreq_data *wrqu, char *extra)
{
	int ret = 0;
	moal_private *priv = (moal_private *)netdev_priv(dev);
	struct iw_point *dwrq = &wrqu->data;
	mlan_ds_sec_cfg *sec = NULL;
	mlan_ioctl_req *req = NULL;
	t_u32 auth_mode;
	int index = (dwrq->flags & IW_ENCODE_INDEX);
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();
	if (index < 0 || index > 4) {
		PRINTM(MERROR, "Key index #%d out of range\n", index);
		ret = -EINVAL;
		goto done;
	}
	if (MLAN_STATUS_SUCCESS !=
	    woal_get_auth_mode(priv, MOAL_IOCTL_WAIT, &auth_mode)) {
		ret = -EFAULT;
		goto done;
	}
	dwrq->flags = 0;
	/*
	 * Check encryption mode
	 */
	switch (auth_mode) {
	case MLAN_AUTH_MODE_OPEN:
		dwrq->flags = IW_ENCODE_OPEN;
		break;

	case MLAN_AUTH_MODE_SHARED:
	case MLAN_AUTH_MODE_NETWORKEAP:
		dwrq->flags = IW_ENCODE_RESTRICTED;
		break;

	case MLAN_AUTH_MODE_SAE:
		dwrq->flags = IW_ENCODE_RESTRICTED;
		break;
	case MLAN_AUTH_MODE_OWE:
		dwrq->flags = IW_ENCODE_OPEN;
		break;

	case MLAN_AUTH_MODE_AUTO:
		dwrq->flags = IW_ENCODE_OPEN | IW_ENCODE_RESTRICTED;
		break;

	default:
		dwrq->flags = IW_ENCODE_DISABLED | IW_ENCODE_OPEN;
		break;
	}
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_sec_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	sec = (mlan_ds_sec_cfg *)req->pbuf;
	sec->sub_command = MLAN_OID_SEC_CFG_ENCRYPT_KEY;
	req->req_id = MLAN_IOCTL_SEC_CFG;
	req->action = MLAN_ACT_GET;

	if (!index)
		sec->param.encrypt_key.key_index = MLAN_KEY_INDEX_DEFAULT;
	else
		sec->param.encrypt_key.key_index = index - 1;

	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}
	memset(extra, 0, 16);
	if (sec->param.encrypt_key.key_len) {
		moal_memcpy_ext(priv->phandle, extra,
				sec->param.encrypt_key.key_material,
				sec->param.encrypt_key.key_len,
				sec->param.encrypt_key.key_len);
		dwrq->length = sec->param.encrypt_key.key_len;
		dwrq->flags |= (sec->param.encrypt_key.key_index + 1);
		dwrq->flags &= ~IW_ENCODE_DISABLED;
	} else if (sec->param.encrypt_key.key_disable)
		dwrq->flags |= IW_ENCODE_DISABLED;
	else
		dwrq->flags &= ~IW_ENCODE_DISABLED;

	dwrq->flags |= IW_ENCODE_NOKEY;
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Set data rate
 *
 *  @param dev          A pointer to net_device structure
 *  @param info         A pointer to iw_request_info structure
 *  @param wrqu         A pointer to iwreq_data union
 *  @param extra        A pointer to extra data buf
 *
 *  @return             0 --success, otherwise fail
 */
static int woal_set_rate(struct net_device *dev, struct iw_request_info *info,
			 union iwreq_data *wrqu, char *extra)
{
	int ret = 0;
	moal_private *priv = (moal_private *)netdev_priv(dev);
	struct iw_param *vwrq = &wrqu->bitrate;
	mlan_rate_cfg_t rate_cfg;

	ENTER();

	memset(&rate_cfg, 0, sizeof(mlan_rate_cfg_t));

	if (vwrq->value == -1) {
		rate_cfg.is_rate_auto = 1;
	} else {
		rate_cfg.is_rate_auto = 0;
		rate_cfg.rate_type = MLAN_RATE_VALUE;
		rate_cfg.rate = vwrq->value / 500000;
	}
	if (MLAN_STATUS_SUCCESS !=
	    woal_set_get_data_rate(priv, MLAN_ACT_SET, &rate_cfg)) {
		ret = -EFAULT;
	}

	LEAVE();
	return ret;
}

/**
 *  @brief Get data rate
 *
 *  @param dev          A pointer to net_device structure
 *  @param info         A pointer to iw_request_info structure
 *  @param wrqu         A pointer to iwreq_data union
 *  @param extra        A pointer to extra data buf
 *
 *  @return             0 --success, otherwise fail
 */
static int woal_get_rate(struct net_device *dev, struct iw_request_info *info,
			 union iwreq_data *wrqu, char *extra)
{
	int ret = 0;
	struct iw_param *vwrq = &wrqu->bitrate;
	moal_private *priv = (moal_private *)netdev_priv(dev);
	mlan_rate_cfg_t rate_cfg;

	ENTER();

	if (MLAN_STATUS_SUCCESS !=
	    woal_set_get_data_rate(priv, MLAN_ACT_GET, &rate_cfg)) {
		ret = -EFAULT;
		goto done;
	}

	if (rate_cfg.is_rate_auto)
		vwrq->fixed = 0;
	else
		vwrq->fixed = 1;
	vwrq->value = rate_cfg.rate * 500000;
done:
	LEAVE();
	return ret;
}

/**
 *  @brief Set RTS threshold
 *
 *  @param dev                  A pointer to net_device structure
 *  @param info                 A pointer to iw_request_info structure
 *  @param wrqu                 A pointer to iwreq_data union
 *  @param extra                A pointer to extra data buf
 *
 *  @return                     0 --success, otherwise fail
 */
static int woal_set_rts(struct net_device *dev, struct iw_request_info *info,
			union iwreq_data *wrqu, char *extra)
{
	int ret = 0;
	struct iw_param *vwrq = &wrqu->rts;
	moal_private *priv = (moal_private *)netdev_priv(dev);
	int rthr = vwrq->value;

	ENTER();

	if (vwrq->disabled) {
		rthr = MLAN_RTS_MAX_VALUE;
	} else {
		if (rthr < MLAN_RTS_MIN_VALUE || rthr > MLAN_RTS_MAX_VALUE) {
			ret = -EINVAL;
			goto done;
		}
	}

	if (MLAN_STATUS_SUCCESS !=
	    woal_set_get_rts(priv, MLAN_ACT_SET, MOAL_IOCTL_WAIT, &rthr)) {
		ret = -EFAULT;
		goto done;
	}

done:
	LEAVE();
	return ret;
}

/**
 *  @brief Get RTS threshold
 *
 *  @param dev                  A pointer to net_device structure
 *  @param info                 A pointer to iw_request_info structure
 *  @param wrqu                 A pointer to iwreq_data union
 *  @param extra                A pointer to extra data buf
 *
 *  @return                     0 --success, otherwise fail
 */
static int woal_get_rts(struct net_device *dev, struct iw_request_info *info,
			union iwreq_data *wrqu, char *extra)
{
	int rthr, ret = 0;
	struct iw_param *vwrq = &wrqu->rts;
	moal_private *priv = (moal_private *)netdev_priv(dev);

	ENTER();

	if (MLAN_STATUS_SUCCESS !=
	    woal_set_get_rts(priv, MLAN_ACT_GET, MOAL_IOCTL_WAIT, &rthr)) {
		ret = -EFAULT;
		goto done;
	}

	vwrq->value = rthr;
	vwrq->disabled = ((vwrq->value < MLAN_RTS_MIN_VALUE) ||
			  (vwrq->value > MLAN_RTS_MAX_VALUE));
	vwrq->fixed = 1;

done:
	LEAVE();
	return ret;
}

/**
 *  @brief Set Fragment threshold
 *
 *  @param dev                  A pointer to net_device structure
 *  @param info                 A pointer to iw_request_info structure
 *  @param wrqu                 A pointer to iwreq_data union
 *  @param extra                A pointer to extra data buf
 *
 *  @return                     0 --success, otherwise fail
 */
static int woal_set_frag(struct net_device *dev, struct iw_request_info *info,
			 union iwreq_data *wrqu, char *extra)
{
	int ret = 0;
	struct iw_param *vwrq = &wrqu->frag;
	moal_private *priv = (moal_private *)netdev_priv(dev);
	int fthr = vwrq->value;

	ENTER();

	if (vwrq->disabled) {
		fthr = MLAN_FRAG_MAX_VALUE;
	} else {
		if (fthr < MLAN_FRAG_MIN_VALUE || fthr > MLAN_FRAG_MAX_VALUE) {
			ret = -EINVAL;
			goto done;
		}
	}

	if (MLAN_STATUS_SUCCESS !=
	    woal_set_get_frag(priv, MLAN_ACT_SET, MOAL_IOCTL_WAIT, &fthr)) {
		ret = -EFAULT;
		goto done;
	}

done:
	LEAVE();
	return ret;
}

/**
 *  @brief Get Fragment threshold
 *
 *  @param dev                  A pointer to net_device structure
 *  @param info                 A pointer to iw_request_info structure
 *  @param wrqu                 A pointer to iwreq_data union
 *  @param extra                A pointer to extra data buf
 *
 *  @return                     0 --success, otherwise fail
 */
static int woal_get_frag(struct net_device *dev, struct iw_request_info *info,
			 union iwreq_data *wrqu, char *extra)
{
	int ret = 0, fthr;
	moal_private *priv = (moal_private *)netdev_priv(dev);
	struct iw_param *vwrq = &wrqu->frag;

	ENTER();

	if (MLAN_STATUS_SUCCESS !=
	    woal_set_get_frag(priv, MLAN_ACT_GET, MOAL_IOCTL_WAIT, &fthr)) {
		ret = -EFAULT;
		goto done;
	}

	vwrq->value = fthr;
	vwrq->disabled = ((vwrq->value < MLAN_FRAG_MIN_VALUE) ||
			  (vwrq->value > MLAN_FRAG_MAX_VALUE));
	vwrq->fixed = 1;

done:
	LEAVE();
	return ret;
}

#if (WIRELESS_EXT >= 18)
/**
 *  @brief Get IE
 *
 *  @param dev                  A pointer to net_device structure
 *  @param info                 A pointer to iw_request_info structure
 *  @param dwrq                 A pointer to iw_point structure
 *  @param extra                A pointer to extra data buf
 *
 *  @return                     0 --success, otherwise fail
 */
static int woal_get_gen_ie(struct net_device *dev, struct iw_request_info *info,
			   union iwreq_data *wrqu, char *extra)
{
	int ret = 0;
	moal_private *priv = (moal_private *)netdev_priv(dev);
	struct iw_point *dwrq = &wrqu->data;
	int copy_size = 0, ie_len;
	t_u8 ie[MAX_IE_SIZE];

	ENTER();

	if (MLAN_STATUS_SUCCESS != woal_set_get_gen_ie(priv, MLAN_ACT_GET, NULL,
						       ie, &ie_len,
						       MOAL_IOCTL_WAIT)) {
		ret = -EFAULT;
		goto done;
	}

	copy_size = MIN(ie_len, dwrq->length);
	moal_memcpy_ext(priv->phandle, extra, ie, copy_size, copy_size);
	dwrq->length = copy_size;

done:
	LEAVE();
	return ret;
}

/**
 *  @brief Set IE
 *
 *  Pass an opaque block of data, expected to be IEEE IEs, to the driver
 *    for eventual passthrough to the firmware in an associate/join
 *    (and potentially start) command.
 *
 *  @param dev                  A pointer to net_device structure
 *  @param info                 A pointer to iw_request_info structure
 *  @param dwrq                 A pointer to iw_point structure
 *  @param extra                A pointer to extra data buf
 *
 *  @return                     0 --success, otherwise fail
 */
static int woal_set_gen_ie(struct net_device *dev, struct iw_request_info *info,
			   union iwreq_data *wrqu, char *extra)
{
	int ret = 0;
	moal_private *priv = (moal_private *)netdev_priv(dev);
	struct iw_point *dwrq = &wrqu->data;
	int ie_len = dwrq->length;
	const t_u8 wps_oui[] = {0x00, 0x50, 0xf2, 0x04};
	mlan_ds_wps_cfg *pwps = NULL;
	mlan_ioctl_req *req = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	/* extra + 2 to skip element id and length */
	if (!memcmp((t_u8 *)(extra + 2), wps_oui, sizeof(wps_oui))) {
		req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_wps_cfg));
		if (req == NULL) {
			ret = -ENOMEM;
			goto done;
		}

		pwps = (mlan_ds_wps_cfg *)req->pbuf;
		req->req_id = MLAN_IOCTL_WPS_CFG;
		req->action = MLAN_ACT_SET;
		pwps->sub_command = MLAN_OID_WPS_CFG_SESSION;
		pwps->param.wps_session = MLAN_WPS_CFG_SESSION_START;

		status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
		if (status != MLAN_STATUS_SUCCESS) {
			ret = -EFAULT;
			goto done;
		}
	}

	if (MLAN_STATUS_SUCCESS !=
	    woal_set_get_gen_ie(priv, MLAN_ACT_SET, (t_u8 *)extra, NULL,
				&ie_len, MOAL_IOCTL_WAIT)) {
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
 *  @brief  Extended version of encoding configuration
 *
 *  @param dev          A pointer to net_device structure
 *  @param info         A pointer to iw_request_info structure
 *  @param dwrq         A pointer to iw_point structure
 *  @param extra        A pointer to extra data buf
 *
 *  @return              0 --success, otherwise fail
 */
static int woal_set_encode_ext(struct net_device *dev,
			       struct iw_request_info *info,
			       union iwreq_data *wrqu, char *extra)
{
	struct iw_encode_ext *ext = (struct iw_encode_ext *)extra;
	moal_private *priv = (moal_private *)netdev_priv(dev);
	struct iw_point *dwrq = &wrqu->data;
	int key_index;
	t_u8 *pkey_material = NULL;
	mlan_ioctl_req *req = NULL;
	mlan_ds_sec_cfg *sec = NULL;
	int ret = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();
	key_index = (dwrq->flags & IW_ENCODE_INDEX) - 1;
	if (key_index < 0 || key_index > 5) {
		ret = -EINVAL;
		goto done;
	}
	if (ext->key_len > (dwrq->length - sizeof(struct iw_encode_ext))) {
		ret = -EINVAL;
		goto done;
	}
	if (ext->key_len > (MLAN_MAX_KEY_LENGTH)) {
		ret = -EINVAL;
		goto done;
	}
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_sec_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}
	sec = (mlan_ds_sec_cfg *)req->pbuf;
	sec->sub_command = MLAN_OID_SEC_CFG_ENCRYPT_KEY;
	req->req_id = MLAN_IOCTL_SEC_CFG;
	req->action = MLAN_ACT_SET;
	pkey_material = (t_u8 *)(ext + 1);
	sec->param.encrypt_key.key_len = ext->key_len;
	moal_memcpy_ext(priv->phandle, sec->param.encrypt_key.mac_addr,
			(u8 *)ext->addr.sa_data, ETH_ALEN,
			sizeof(sec->param.encrypt_key.mac_addr));
	/* Disable and Remove Key */
	if ((dwrq->flags & IW_ENCODE_DISABLED) && !ext->key_len) {
		sec->param.encrypt_key.key_remove = MTRUE;
		sec->param.encrypt_key.key_index = key_index;
		sec->param.encrypt_key.key_flags = KEY_FLAG_REMOVE_KEY;
		PRINTM(MIOCTL,
		       "Remove key key_index=%d, dwrq->flags=0x%x " MACSTR "\n",
		       key_index, dwrq->flags,
		       MAC2STR(sec->param.encrypt_key.mac_addr));
	} else if (ext->key_len <= MAX_WEP_KEY_SIZE) {
		/* Set WEP key */
		sec->param.encrypt_key.key_index = key_index;
		sec->param.encrypt_key.key_flags = ext->ext_flags;
		moal_memcpy_ext(priv->phandle,
				sec->param.encrypt_key.key_material,
				pkey_material, ext->key_len,
				sizeof(sec->param.encrypt_key.key_material));
		if (ext->ext_flags & IW_ENCODE_EXT_SET_TX_KEY)
			sec->param.encrypt_key.is_current_wep_key = MTRUE;
	} else {
		/* Set WPA key */
		sec->param.encrypt_key.key_index = key_index;
		sec->param.encrypt_key.key_flags = ext->ext_flags;
		if (ext->ext_flags & IW_ENCODE_EXT_RX_SEQ_VALID) {
			moal_memcpy_ext(priv->phandle,
					sec->param.encrypt_key.pn,
					(t_u8 *)ext->rx_seq, SEQ_MAX_SIZE,
					sizeof(sec->param.encrypt_key.pn));
			DBG_HEXDUMP(MCMD_D, "Rx PN", sec->param.encrypt_key.pn,
				    SEQ_MAX_SIZE);
		}
		if (ext->ext_flags & IW_ENCODE_EXT_TX_SEQ_VALID) {
			moal_memcpy_ext(priv->phandle,
					sec->param.encrypt_key.pn,
					(t_u8 *)ext->tx_seq, SEQ_MAX_SIZE,
					sizeof(sec->param.encrypt_key.pn));
			DBG_HEXDUMP(MCMD_D, "Tx PN", sec->param.encrypt_key.pn,
				    SEQ_MAX_SIZE);
		}
		moal_memcpy_ext(priv->phandle,
				sec->param.encrypt_key.key_material,
				pkey_material, ext->key_len,
				sizeof(sec->param.encrypt_key.key_material));
		PRINTM(MIOCTL,
		       "set wpa key key_index=%d, key_len=%d key_flags=0x%x " MACSTR
		       "\n",
		       key_index, ext->key_len,
		       sec->param.encrypt_key.key_flags,
		       MAC2STR(sec->param.encrypt_key.mac_addr));
		DBG_HEXDUMP(MCMD_D, "wpa key", pkey_material, ext->key_len);
#define IW_ENCODE_ALG_AES_CMAC 5
		if (ext->alg == IW_ENCODE_ALG_AES_CMAC)
			sec->param.encrypt_key.key_flags |=
				KEY_FLAG_AES_MCAST_IGTK;
#define IW_ENCODE_ALG_SMS4 0x20
		/* Set WAPI key */
		if (ext->alg == IW_ENCODE_ALG_SMS4) {
			sec->param.encrypt_key.is_wapi_key = MTRUE;
			moal_memcpy_ext(priv->phandle,
					sec->param.encrypt_key.pn,
					(t_u8 *)ext->tx_seq, SEQ_MAX_SIZE,
					sizeof(sec->param.encrypt_key.pn));
			moal_memcpy_ext(
				priv->phandle,
				&sec->param.encrypt_key.pn[SEQ_MAX_SIZE],
				(t_u8 *)ext->rx_seq, SEQ_MAX_SIZE,
				sizeof(sec->param.encrypt_key.pn) -
					SEQ_MAX_SIZE);
			DBG_HEXDUMP(MCMD_D, "WAPI PN",
				    sec->param.encrypt_key.pn, PN_SIZE);
		}
	}
	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS)
		ret = -EFAULT;
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief  Extended version of encoding configuration
 *
 *  @param dev          A pointer to net_device structure
 *  @param info         A pointer to iw_request_info structure
 *  @param dwrq         A pointer to iw_point structure
 *  @param extra        A pointer to extra data buf
 *
 *  @return             -EOPNOTSUPP
 */
static int woal_get_encode_ext(struct net_device *dev,
			       struct iw_request_info *info,
			       union iwreq_data *wrqu, char *extra)
{
	ENTER();
	LEAVE();
	return -EOPNOTSUPP;
}

/**
 *  @brief  Request MLME operation
 *
 *  @param dev          A pointer to net_device structure
 *  @param info         A pointer to iw_request_info structure
 *  @param dwrq         A pointer to iw_point structure
 *  @param extra        A pointer to extra data buf
 *
 *  @return             0--success, otherwise fail
 */
static int woal_set_mlme(struct net_device *dev, struct iw_request_info *info,
			 union iwreq_data *wrqu, char *extra)
{
	struct iw_mlme *mlme = (struct iw_mlme *)extra;
	moal_private *priv = (moal_private *)netdev_priv(dev);
	int ret = 0;

	ENTER();
	if ((mlme->cmd == IW_MLME_DEAUTH) || (mlme->cmd == IW_MLME_DISASSOC)) {
		if (MLAN_STATUS_SUCCESS !=
		    woal_disconnect(priv, MOAL_IOCTL_WAIT,
				    (t_u8 *)mlme->addr.sa_data,
				    DEF_DEAUTH_REASON_CODE))
			ret = -EFAULT;
	}
	LEAVE();
	return ret;
}

/** @brief Set authentication mode parameters
 *
 *  @param dev                  A pointer to net_device structure
 *  @param info                 A pointer to iw_request_info structure
 *  @param vwrq                 A pointer to iw_param structure
 *  @param extra                A pointer to extra data buf
 *
 *  @return                     0 --success, otherwise fail
 */
static int woal_set_auth(struct net_device *dev, struct iw_request_info *info,
			 union iwreq_data *wrqu, char *extra)
{
	int ret = 0;
	moal_private *priv = (moal_private *)netdev_priv(dev);
	struct iw_param *vwrq = &wrqu->param;
	t_u32 auth_mode = 0;
	t_u32 encrypt_mode = 0;
	ENTER();

	switch (vwrq->flags & IW_AUTH_INDEX) {
	case IW_AUTH_CIPHER_PAIRWISE:
	case IW_AUTH_CIPHER_GROUP:
		if (vwrq->value & IW_AUTH_CIPHER_NONE)
			encrypt_mode = MLAN_ENCRYPTION_MODE_NONE;
		else if (vwrq->value & IW_AUTH_CIPHER_WEP40)
			encrypt_mode = MLAN_ENCRYPTION_MODE_WEP40;
		else if (vwrq->value & IW_AUTH_CIPHER_WEP104)
			encrypt_mode = MLAN_ENCRYPTION_MODE_WEP104;
		else if (vwrq->value & IW_AUTH_CIPHER_TKIP)
			encrypt_mode = MLAN_ENCRYPTION_MODE_TKIP;
		else if (vwrq->value & IW_AUTH_CIPHER_CCMP)
			encrypt_mode = MLAN_ENCRYPTION_MODE_CCMP;
		if (MLAN_STATUS_SUCCESS !=
		    woal_set_encrypt_mode(priv, MOAL_IOCTL_WAIT, encrypt_mode))
			ret = -EFAULT;
		break;
	case IW_AUTH_80211_AUTH_ALG:
		switch (vwrq->value) {
		case IW_AUTH_ALG_SHARED_KEY:
			PRINTM(MINFO, "Auth mode shared key!\n");
			auth_mode = MLAN_AUTH_MODE_SHARED;
			break;
		case IW_AUTH_ALG_LEAP:
			PRINTM(MINFO, "Auth mode LEAP!\n");
			auth_mode = MLAN_AUTH_MODE_NETWORKEAP;
			break;
		case IW_AUTH_ALG_OPEN_SYSTEM:
			PRINTM(MINFO, "Auth mode open!\n");
			auth_mode = MLAN_AUTH_MODE_OPEN;
			break;
		case IW_AUTH_ALG_SHARED_KEY | IW_AUTH_ALG_OPEN_SYSTEM:
		default:
			PRINTM(MINFO, "Auth mode auto!\n");
			auth_mode = MLAN_AUTH_MODE_AUTO;
			break;
		}
		if (MLAN_STATUS_SUCCESS !=
		    woal_set_auth_mode(priv, MOAL_IOCTL_WAIT, auth_mode))
			ret = -EFAULT;
		break;
	case IW_AUTH_WPA_ENABLED:
		if (MLAN_STATUS_SUCCESS !=
		    woal_set_wpa_enable(priv, MOAL_IOCTL_WAIT, vwrq->value))
			ret = -EFAULT;
		break;
#define IW_AUTH_WAPI_ENABLED 0x20
	case IW_AUTH_WAPI_ENABLED:
		if (MLAN_STATUS_SUCCESS !=
		    woal_set_wapi_enable(priv, MOAL_IOCTL_WAIT, vwrq->value))
			ret = -EFAULT;
		break;
	case IW_AUTH_WPA_VERSION:
		/* set WPA_VERSION_DISABLED/VERSION_WPA/VERSION_WP2 */
		priv->wpa_version = vwrq->value;
		break;
	case IW_AUTH_KEY_MGMT:
		/* set KEY_MGMT_802_1X/KEY_MGMT_PSK */
		priv->key_mgmt = vwrq->value;
		break;
	case IW_AUTH_TKIP_COUNTERMEASURES:
	case IW_AUTH_DROP_UNENCRYPTED:
	case IW_AUTH_RX_UNENCRYPTED_EAPOL:
	case IW_AUTH_ROAMING_CONTROL:
	case IW_AUTH_PRIVACY_INVOKED:
#if CFG80211_VERSION_CODE > KERNEL_VERSION(2, 6, 29)
	case IW_AUTH_MFP:
#endif
		break;
	default:
		ret = -EOPNOTSUPP;
		break;
	}
	LEAVE();
	return ret;
}

/**
 *  @brief Get authentication mode parameters
 *
 *  @param dev                  A pointer to net_device structure
 *  @param info                 A pointer to iw_request_info structure
 *  @param vwrq                 A pointer to iw_param structure
 *  @param extra                A pointer to extra data buf
 *
 *  @return                     0 --success, otherwise fail
 */
static int woal_get_auth(struct net_device *dev, struct iw_request_info *info,
			 union iwreq_data *wrqu, char *extra)
{
	int ret = 0;
	moal_private *priv = (moal_private *)netdev_priv(dev);
	struct iw_param *vwrq = &wrqu->param;
	t_u32 encrypt_mode = 0;
	t_u32 auth_mode;
	t_u32 wpa_enable;
	ENTER();
	switch (vwrq->flags & IW_AUTH_INDEX) {
	case IW_AUTH_CIPHER_PAIRWISE:
	case IW_AUTH_CIPHER_GROUP:
		if (MLAN_STATUS_SUCCESS !=
		    woal_get_encrypt_mode(priv, MOAL_IOCTL_WAIT, &encrypt_mode))
			ret = -EFAULT;
		else {
			if (encrypt_mode == MLAN_ENCRYPTION_MODE_NONE)
				vwrq->value = IW_AUTH_CIPHER_NONE;
			else if (encrypt_mode == MLAN_ENCRYPTION_MODE_WEP40)
				vwrq->value = IW_AUTH_CIPHER_WEP40;
			else if (encrypt_mode == MLAN_ENCRYPTION_MODE_TKIP)
				vwrq->value = IW_AUTH_CIPHER_TKIP;
			else if (encrypt_mode == MLAN_ENCRYPTION_MODE_CCMP)
				vwrq->value = IW_AUTH_CIPHER_CCMP;
			else if (encrypt_mode == MLAN_ENCRYPTION_MODE_WEP104)
				vwrq->value = IW_AUTH_CIPHER_WEP104;
		}
		break;
	case IW_AUTH_80211_AUTH_ALG:
		if (MLAN_STATUS_SUCCESS !=
		    woal_get_auth_mode(priv, MOAL_IOCTL_WAIT, &auth_mode))
			ret = -EFAULT;
		else {
			if (auth_mode == MLAN_AUTH_MODE_SHARED)
				vwrq->value = IW_AUTH_ALG_SHARED_KEY;
			else if (auth_mode == MLAN_AUTH_MODE_NETWORKEAP)
				vwrq->value = IW_AUTH_ALG_LEAP;
			else
				vwrq->value = IW_AUTH_ALG_OPEN_SYSTEM;
		}
		break;
	case IW_AUTH_WPA_ENABLED:
		if (MLAN_STATUS_SUCCESS !=
		    woal_get_wpa_enable(priv, MOAL_IOCTL_WAIT, &wpa_enable))
			ret = -EFAULT;
		else
			vwrq->value = wpa_enable;
		break;
	case IW_AUTH_WPA_VERSION:
		vwrq->value = priv->wpa_version;
		break;
	case IW_AUTH_KEY_MGMT:
		vwrq->value = priv->key_mgmt;
		break;
	case IW_AUTH_TKIP_COUNTERMEASURES:
	case IW_AUTH_DROP_UNENCRYPTED:
	case IW_AUTH_RX_UNENCRYPTED_EAPOL:
	case IW_AUTH_ROAMING_CONTROL:
	case IW_AUTH_PRIVACY_INVOKED:
	default:
		ret = -EOPNOTSUPP;
		goto done;
	}

done:
	LEAVE();
	return ret;
}

/**
 *  @brief Set PMKSA Cache
 *
 *  @param dev      A pointer to net_device structure
 *  @param info     A pointer to iw_request_info structure
 *  @param wrqu     A pointer to iwreq_data union
 *  @param extra    A pointer to extra data buf
 *
 *  @return         -EOPNOTSUPP
 */
static int woal_set_pmksa(struct net_device *dev, struct iw_request_info *info,
			  union iwreq_data *wrqu, char *extra)
{
	ENTER();
	LEAVE();
	return -EOPNOTSUPP;
}

#endif /* WE >= 18 */

/* Data rate listing
 *      MULTI_BANDS:
 *              abg             a       b       b/g
 *  Infra       G(12)           A(8)    B(4)    G(12)
 *  Adhoc       A+B(12)         A(8)    B(4)    B(4)
 *      non-MULTI_BANDS:
										b       b/g
 *  Infra                               B(4)    G(12)
 *  Adhoc                               B(4)    B(4)
 */
/**
 *  @brief Get Range Info
 *
 *  @param dev                  A pointer to net_device structure
 *  @param info                 A pointer to iw_request_info structure
 *  @param dwrq                 A pointer to iw_point structure
 *  @param extra                A pointer to extra data buf
 *
 *  @return                     0 --success, otherwise fail
 */
static int woal_get_range(struct net_device *dev, struct iw_request_info *info,
			  union iwreq_data *wrqu, char *extra)
{
	int i;
	moal_private *priv = (moal_private *)netdev_priv(dev);
	struct iw_point *dwrq = &wrqu->data;
	struct iw_range *range = (struct iw_range *)extra;
	moal_802_11_rates rates;
	mlan_chan_list *pchan_list = NULL;
	mlan_bss_info bss_info;
	gfp_t flag;
	int ret = 0;

	ENTER();

	flag = (in_atomic() || irqs_disabled()) ? GFP_ATOMIC : GFP_KERNEL;
	pchan_list = kzalloc(sizeof(mlan_chan_list), flag);
	if (!pchan_list) {
		LEAVE();
		return -ENOMEM;
	}

	dwrq->length = sizeof(struct iw_range);
	memset(range, 0, sizeof(struct iw_range));

	range->min_nwid = 0;
	range->max_nwid = 0;

	memset(&rates, 0, sizeof(rates));
	woal_get_data_rates(priv, MOAL_IOCTL_WAIT, &rates);
	range->num_bitrates = rates.num_of_rates;

	for (i = 0;
	     i < MIN(range->num_bitrates, IW_MAX_BITRATES) && rates.rates[i];
	     i++) {
		range->bitrate[i] = (rates.rates[i] & 0x7f) * 500000;
	}
	range->num_bitrates = i;
	PRINTM(MINFO, "IW_MAX_BITRATES=%d num_bitrates=%d\n", IW_MAX_BITRATES,
	       range->num_bitrates);

	range->num_frequency = 0;

	woal_get_channel_list(priv, MOAL_IOCTL_WAIT, pchan_list);

	range->num_frequency = MIN(pchan_list->num_of_chan, IW_MAX_FREQUENCIES);

	for (i = 0; i < range->num_frequency; i++) {
		range->freq[i].i = (long)pchan_list->cf[i].channel;
		range->freq[i].m = (long)pchan_list->cf[i].freq * 100000;
		range->freq[i].e = 1;
	}
	kfree(pchan_list);

	PRINTM(MINFO, "IW_MAX_FREQUENCIES=%d num_frequency=%d\n",
	       IW_MAX_FREQUENCIES, range->num_frequency);

	range->num_channels = range->num_frequency;

	woal_sort_channels(&range->freq[0], range->num_frequency);

	/*
	 * Set an indication of the max TCP throughput in bit/s that we can
	 * expect using this interface
	 */
	if (i > 2)
		range->throughput = 5000 * 1000;
	else
		range->throughput = 1500 * 1000;

	range->min_rts = MLAN_RTS_MIN_VALUE;
	range->max_rts = MLAN_RTS_MAX_VALUE;
	range->min_frag = MLAN_FRAG_MIN_VALUE;
	range->max_frag = MLAN_FRAG_MAX_VALUE;

	range->encoding_size[0] = 5;
	range->encoding_size[1] = 13;
	range->num_encoding_sizes = 2;
	range->max_encoding_tokens = 4;

/** Minimum power period */
#define IW_POWER_PERIOD_MIN 1000000 /* 1 sec */
/** Maximum power period */
#define IW_POWER_PERIOD_MAX 120000000 /* 2 min */
/** Minimum power timeout value */
#define IW_POWER_TIMEOUT_MIN 1000 /* 1 ms  */
/** Maximim power timeout value */
#define IW_POWER_TIMEOUT_MAX 1000000 /* 1 sec */

	/* Power Management duration & timeout */
	range->min_pmp = IW_POWER_PERIOD_MIN;
	range->max_pmp = IW_POWER_PERIOD_MAX;
	range->min_pmt = IW_POWER_TIMEOUT_MIN;
	range->max_pmt = IW_POWER_TIMEOUT_MAX;
	range->pmp_flags = IW_POWER_PERIOD;
	range->pmt_flags = IW_POWER_TIMEOUT;
	range->pm_capa = IW_POWER_PERIOD | IW_POWER_TIMEOUT | IW_POWER_ALL_R;

	/*
	 * Minimum version we recommend
	 */
	range->we_version_source = 15;

	/*
	 * Version we are compiled with
	 */
	range->we_version_compiled = WIRELESS_EXT;

	range->retry_capa = IW_RETRY_LIMIT;
	range->retry_flags = IW_RETRY_LIMIT | IW_RETRY_MAX;

	range->min_retry = MLAN_TX_RETRY_MIN;
	range->max_retry = MLAN_TX_RETRY_MAX;

	/*
	 * Set the qual, level and noise range values
	 */
	/*
	 * need to put the right values here
	 */
/** Maximum quality percentage */
#define IW_MAX_QUAL_PERCENT 5
/** Average quality percentage */
#define IW_AVG_QUAL_PERCENT 3
	range->max_qual.qual = IW_MAX_QUAL_PERCENT;
	range->max_qual.level = 0;
	range->max_qual.noise = 0;

	range->avg_qual.qual = IW_AVG_QUAL_PERCENT;
	range->avg_qual.level = 0;
	range->avg_qual.noise = 0;

	range->sensitivity = 0;

	/*
	 * Setup the supported power level ranges
	 */
	memset(range->txpower, 0, sizeof(range->txpower));

	memset(&bss_info, 0, sizeof(bss_info));

	if (MLAN_STATUS_SUCCESS !=
	    woal_get_bss_info(priv, MOAL_IOCTL_WAIT, &bss_info)) {
		ret = -EFAULT;
		goto done;
	}

	range->txpower[0] = bss_info.min_power_level;
	range->txpower[1] = bss_info.max_power_level;
	range->num_txpower = 2;
	range->txpower_capa = IW_TXPOW_DBM | IW_TXPOW_RANGE;

#if (WIRELESS_EXT >= 18)
	range->enc_capa = IW_ENC_CAPA_WPA | IW_ENC_CAPA_WPA2 |
			  IW_ENC_CAPA_CIPHER_CCMP | IW_ENC_CAPA_CIPHER_TKIP;
#endif
done:
	LEAVE();
	return ret;
}

#ifdef MEF_CFG_RX_FILTER
/**
 *  @brief Enable/disable Rx broadcast/multicast filter in non-HS mode
 *
 *  @param priv                 A pointer to moal_private structure
 *  @param enable               MTRUE/MFALSE: enable/disable
 *
 *  @return                     0 -- success, otherwise fail
 */
static int woal_set_rxfilter(moal_private *priv, BOOLEAN enable)
{
	int ret = 0;
	mlan_ioctl_req *req = NULL;
	mlan_ds_misc_cfg *misc = NULL;
	mlan_ds_misc_mef_cfg *mef_cfg = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}
	misc = (mlan_ds_misc_cfg *)req->pbuf;
	mef_cfg = &misc->param.mef_cfg;
	req->req_id = MLAN_IOCTL_MISC_CFG;
	misc->sub_command = MLAN_OID_MISC_MEF_CFG;
	req->action = MLAN_ACT_SET;

	mef_cfg->sub_id = (enable ? MEF_CFG_RX_FILTER_ENABLE : MEF_CFG_DISABLE);
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
#endif

/**
 *  @brief Set priv command
 *
 *  @param dev          A pointer to net_device structure
 *  @param info         A pointer to iw_request_info structure
 *  @param dwrq         A pointer to iw_point structure
 *  @param extra        A pointer to extra data buf
 *
 *  @return                     0 --success, otherwise fail
 */
static int woal_set_priv(struct net_device *dev, struct iw_request_info *info,
			 union iwreq_data *wrqu, char *extra)
{
	int ret = 0;
	moal_private *priv = (moal_private *)netdev_priv(dev);
	struct iw_point *dwrq = &wrqu->data;
	char *buf = NULL;
	int power_mode = 0;
	int band = 0;
	char *pband = NULL;
	mlan_bss_info bss_info;
	mlan_ds_get_signal signal;
	mlan_rate_cfg_t rate;
	char *pdata = NULL;
	t_u8 country_code[COUNTRY_CODE_LEN];
	int len = 0;
	gfp_t flag;
	unsigned int link_speed = 0;

	ENTER();
	if (!priv || !priv->phandle) {
		PRINTM(MERROR, "priv or handle is NULL\n");
		ret = -EFAULT;
		goto done;
	}
	flag = (in_atomic() || irqs_disabled()) ? GFP_ATOMIC : GFP_KERNEL;
	buf = kzalloc(dwrq->length + 1, flag);
	if (!buf) {
		ret = -ENOMEM;
		goto done;
	}
	if (copy_from_user(buf, dwrq->pointer, dwrq->length)) {
		ret = -EFAULT;
		goto done;
	}
	buf[dwrq->length] = '\0';
	PRINTM(MIOCTL, "SIOCSIWPRIV request = %s\n", buf);
	if (strncmp(buf, "RSSILOW-THRESHOLD", strlen("RSSILOW-THRESHOLD")) ==
	    0) {
		if (dwrq->length > strlen("RSSILOW-THRESHOLD") + 1) {
			pdata = buf + strlen("RSSILOW-THRESHOLD") + 1;
			if (MLAN_STATUS_SUCCESS !=
			    woal_set_rssi_low_threshold(priv, pdata,
							MOAL_IOCTL_WAIT)) {
				ret = -EFAULT;
				goto done;
			}
			len = snprintf(buf, MAX_BUF_LEN, "OK\n") + 1;
		} else {
			ret = -EFAULT;
			goto done;
		}
	} else if (strncmp(buf, "RSSI", strlen("RSSI")) == 0) {
		if (MLAN_STATUS_SUCCESS !=
		    woal_get_bss_info(priv, MOAL_IOCTL_WAIT, &bss_info)) {
			ret = -EFAULT;
			goto done;
		}
		if (bss_info.media_connected) {
			if (MLAN_STATUS_SUCCESS !=
			    woal_get_signal_info(priv, MOAL_IOCTL_WAIT,
						 &signal)) {
				ret = -EFAULT;
				goto done;
			}
			len = snprintf(buf, MAX_BUF_LEN, "%s rssi %d\n",
				       bss_info.ssid.ssid,
				       signal.bcn_rssi_avg) +
			      1;
		} else {
			len = snprintf(buf, MAX_BUF_LEN, "OK\n") + 1;
		}
	} else if (strncmp(buf, "LINKSPEED", strlen("LINKSPEED")) == 0) {
		if (MLAN_STATUS_SUCCESS !=
		    woal_set_get_data_rate(priv, MLAN_ACT_GET, &rate)) {
			ret = -EFAULT;
			goto done;
		}
		PRINTM(MIOCTL, "tx rate=%d\n", (int)rate.rate);
		link_speed = rate.rate * 500000 / 1000000;
		len = snprintf(buf, MAX_BUF_LEN, "LinkSpeed %d\n",
			       (int)link_speed) +
		      1;
	} else if (strncmp(buf, "MACADDR", strlen("MACADDR")) == 0) {
		len = snprintf(buf, MAX_BUF_LEN,
			       "Macaddr = %02X:%02X:%02X:%02X:%02X:%02X\n",
			       priv->current_addr[0], priv->current_addr[1],
			       priv->current_addr[2], priv->current_addr[3],
			       priv->current_addr[4], priv->current_addr[5]) +
		      1;
	} else if (strncmp(buf, "GETPOWER", strlen("GETPOWER")) == 0) {
		if (MLAN_STATUS_SUCCESS !=
		    woal_get_powermode(priv, &power_mode)) {
			ret = -EFAULT;
			goto done;
		}
		len = snprintf(buf, MAX_BUF_LEN, "powermode = %d\n",
			       power_mode) +
		      1;
	} else if (strncmp(buf, "SCAN-ACTIVE", strlen("SCAN-ACTIVE")) == 0) {
		if (MLAN_STATUS_SUCCESS !=
		    woal_set_scan_type(priv, MLAN_SCAN_TYPE_ACTIVE)) {
			ret = -EFAULT;
			goto done;
		}
		priv->scan_type = MLAN_SCAN_TYPE_ACTIVE;
		PRINTM(MIOCTL, "Set Active Scan\n");
		len = snprintf(buf, MAX_BUF_LEN, "OK\n") + 1;
	} else if (strncmp(buf, "SCAN-PASSIVE", strlen("SCAN-PASSIVE")) == 0) {
		if (MLAN_STATUS_SUCCESS !=
		    woal_set_scan_type(priv, MLAN_SCAN_TYPE_PASSIVE)) {
			ret = -EFAULT;
			goto done;
		}
		priv->scan_type = MLAN_SCAN_TYPE_PASSIVE;
		PRINTM(MIOCTL, "Set Passive Scan\n");
		len = snprintf(buf, MAX_BUF_LEN, "OK\n") + 1;
	} else if (strncmp(buf, "POWERMODE", strlen("POWERMODE")) == 0) {
		if (dwrq->length > strlen("POWERMODE") + 1) {
			pdata = buf + strlen("POWERMODE") + 1;
			if (!moal_extflg_isset(priv->phandle, EXT_HW_TEST)) {
				if (MLAN_STATUS_SUCCESS !=
				    woal_set_powermode(priv, pdata)) {
					ret = -EFAULT;
					goto done;
				}
			}
			len = snprintf(buf, MAX_BUF_LEN, "OK\n") + 1;
		} else {
			ret = -EFAULT;
			goto done;
		}
	} else if (strncmp(buf, "COUNTRY", strlen("COUNTRY")) == 0) {
		memset(country_code, 0, sizeof(country_code));
		if ((strlen(buf) - strlen("COUNTRY") - 1) > COUNTRY_CODE_LEN) {
			ret = -EFAULT;
			goto done;
		}
		moal_memcpy_ext(priv->phandle, country_code,
				buf + strlen("COUNTRY") + 1,
				COUNTRY_CODE_LEN - 1, sizeof(country_code) - 1);
		PRINTM(MIOCTL, "Set COUNTRY %s\n", country_code);
		if (MLAN_STATUS_SUCCESS !=
		    woal_set_region_code(priv, country_code)) {
			ret = -EFAULT;
			goto done;
		}
		len = snprintf(buf, MAX_BUF_LEN, "OK\n") + 1;
	} else if (memcmp(buf, WEXT_CSCAN_HEADER, strlen(WEXT_CSCAN_HEADER)) ==
		   0) {
		PRINTM(MIOCTL, "Set Combo Scan\n");
		if (MLAN_STATUS_SUCCESS !=
		    woal_set_combo_scan(priv, buf, dwrq->length)) {
			ret = -EFAULT;
			goto done;
		}
		len = snprintf(buf, MAX_BUF_LEN, "OK\n") + 1;
	} else if (strncmp(buf, "GETBAND", strlen("GETBAND")) == 0) {
		if (MLAN_STATUS_SUCCESS != woal_get_band(priv, &band)) {
			ret = -EFAULT;
			goto done;
		}
		len = snprintf(buf, MAX_BUF_LEN, "Band %d\n", band) + 1;
	} else if (strncmp(buf, "SETBAND", strlen("SETBAND")) == 0) {
		pband = buf + strlen("SETBAND") + 1;
		if (MLAN_STATUS_SUCCESS != woal_set_band(priv, pband)) {
			ret = -EFAULT;
			goto done;
		}
		len = snprintf(buf, MAX_BUF_LEN, "OK\n") + 1;
	} else if (strncmp(buf, "START", strlen("START")) == 0) {
		len = snprintf(buf, MAX_BUF_LEN, "OK\n") + 1;
	} else if (strncmp(buf, "STOP", strlen("STOP")) == 0) {
		len = snprintf(buf, MAX_BUF_LEN, "OK\n") + 1;
	} else if (strncmp(buf, "SETSUSPENDOPT", strlen("SETSUSPENDOPT")) ==
		   0) {
		/* it will be done by GUI */
		len = snprintf(buf, MAX_BUF_LEN, "OK\n") + 1;
	} else if (strncmp(buf, "BTCOEXMODE", strlen("BTCOEXMODE")) == 0) {
		len = snprintf(buf, MAX_BUF_LEN, "OK\n") + 1;
	} else if (strncmp(buf, "BTCOEXSCAN-START",
			   strlen("BTCOEXSCAN-START")) == 0) {
		len = snprintf(buf, MAX_BUF_LEN, "OK\n") + 1;
	} else if (strncmp(buf, "BTCOEXSCAN-STOP", strlen("BTCOEXSCAN-STOP")) ==
		   0) {
		len = snprintf(buf, MAX_BUF_LEN, "OK\n") + 1;
	} else if (strncmp(buf, "BGSCAN-START", strlen("BGSCAN-START")) == 0) {
		len = snprintf(buf, MAX_BUF_LEN, "OK\n") + 1;
	} else if (strncmp(buf, "BGSCAN-CONFIG", strlen("BGSCAN-CONFIG")) ==
		   0) {
		if (MLAN_STATUS_SUCCESS !=
		    woal_set_bg_scan(priv, buf, dwrq->length)) {
			ret = -EFAULT;
			goto done;
		}
		priv->bg_scan_start = MTRUE;
		priv->bg_scan_reported = MFALSE;
		len = snprintf(buf, MAX_BUF_LEN, "OK\n") + 1;
	} else if (strncmp(buf, "BGSCAN-STOP", strlen("BGSCAN-STOP")) == 0) {
		if (priv->bg_scan_start && !priv->scan_cfg.rssi_threshold) {
			if (MLAN_STATUS_SUCCESS !=
			    woal_stop_bg_scan(priv, MOAL_IOCTL_WAIT)) {
				ret = -EFAULT;
				goto done;
			}
			priv->bg_scan_start = MFALSE;
			priv->bg_scan_reported = MFALSE;
		}
		len = snprintf(buf, MAX_BUF_LEN, "OK\n") + 1;
	} else if (strncmp(buf, "RXFILTER-START", strlen("RXFILTER-START")) ==
		   0) {
#ifdef MEF_CFG_RX_FILTER
		ret = woal_set_rxfilter(priv, MTRUE);
		if (ret)
			goto done;
#endif
		len = snprintf(buf, MAX_BUF_LEN, "OK\n") + 1;
	} else if (strncmp(buf, "RXFILTER-STOP", strlen("RXFILTER-STOP")) ==
		   0) {
#ifdef MEF_CFG_RX_FILTER
		ret = woal_set_rxfilter(priv, MFALSE);
		if (ret)
			goto done;
#endif
		len = snprintf(buf, MAX_BUF_LEN, "OK\n") + 1;
	} else if (strncmp(buf, "RXFILTER-ADD", strlen("RXFILTER-ADD")) == 0) {
		if (dwrq->length > strlen("RXFILTER-ADD") + 1) {
			pdata = buf + strlen("RXFILTER-ADD") + 1;
			if (MLAN_STATUS_SUCCESS !=
			    woal_add_rxfilter(priv, pdata)) {
				ret = -EFAULT;
				goto done;
			}
			len = snprintf(buf, MAX_BUF_LEN, "OK\n") + 1;
		} else {
			ret = -EFAULT;
			goto done;
		}
	} else if (strncmp(buf, "RXFILTER-REMOVE", strlen("RXFILTER-REMOVE")) ==
		   0) {
		if (dwrq->length > strlen("RXFILTER-REMOVE") + 1) {
			pdata = buf + strlen("RXFILTER-REMOVE") + 1;
			if (MLAN_STATUS_SUCCESS !=
			    woal_remove_rxfilter(priv, pdata)) {
				ret = -EFAULT;
				goto done;
			}
			len = snprintf(buf, MAX_BUF_LEN, "OK\n") + 1;
		} else {
			ret = -EFAULT;
			goto done;
		}
	} else if (strncmp(buf, "QOSINFO", strlen("QOSINFO")) == 0) {
		if (dwrq->length > strlen("QOSINFO") + 1) {
			pdata = buf + strlen("QOSINFO") + 1;
			if (MLAN_STATUS_SUCCESS !=
			    woal_priv_qos_cfg(priv, MLAN_ACT_SET, pdata)) {
				ret = -EFAULT;
				goto done;
			}
			len = snprintf(buf, MAX_BUF_LEN, "OK\n") + 1;
		} else {
			ret = -EFAULT;
			goto done;
		}
	} else if (strncmp(buf, "SLEEPPD", strlen("SLEEPPD")) == 0) {
		if (dwrq->length > strlen("SLEEPPD") + 1) {
			pdata = buf + strlen("SLEEPPD") + 1;
			if (MLAN_STATUS_SUCCESS !=
			    woal_set_sleeppd(priv, pdata)) {
				ret = -EFAULT;
				goto done;
			}
			len = snprintf(buf, MAX_BUF_LEN, "OK\n") + 1;
		} else {
			ret = -EFAULT;
			goto done;
		}
	} else {
		PRINTM(MIOCTL, "Unknow PRIVATE command: %s, ignored\n", buf);
		ret = -EFAULT;
		goto done;
	}
	PRINTM(MIOCTL, "PRIV Command return: %s, length=%d\n", buf, len);
	dwrq->length = (t_u16)len;
	if (copy_to_user((void __user *)dwrq->pointer, buf, dwrq->length))
		ret = -EFAULT;
done:
	kfree(buf);
	LEAVE();
	return ret;
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
static mlan_status woal_wext_request_scan(moal_private *priv, t_u8 wait_option,
					  mlan_802_11_ssid *req_ssid)
{
	wlan_user_scan_cfg *scan_req;
	mlan_scan_cfg scan_cfg;
	mlan_status status;
	ENTER();
	if (!woal_is_any_interface_active(priv->phandle)) {
		LEAVE();
		return woal_request_scan(priv, wait_option, req_ssid);
	}
	scan_req = (wlan_user_scan_cfg *)kmalloc(sizeof(wlan_user_scan_cfg),
						 GFP_KERNEL);
	if (!scan_req) {
		PRINTM(MERROR, "Malloc buffer failed\n");
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}

	memset(&scan_cfg, 0, sizeof(scan_cfg));
	memset(scan_req, 0, sizeof(wlan_user_scan_cfg));
	if (req_ssid && req_ssid->ssid_len != 0) {
		moal_memcpy_ext(priv->phandle, scan_req->ssid_list[0].ssid,
				req_ssid->ssid, req_ssid->ssid_len,
				MLAN_MAX_SSID_LENGTH);
		scan_req->ssid_list[0].max_len = 0;
	}
	if (MLAN_STATUS_SUCCESS != woal_get_scan_config(priv, &scan_cfg)) {
		PRINTM(MERROR, "Unable to get scan configuration\n");
	}
	if (scan_cfg.scan_chan_gap)
		scan_req->scan_chan_gap = scan_cfg.scan_chan_gap;
	else
		scan_req->scan_chan_gap = priv->phandle->scan_chan_gap;
	/** indicate FW, gap is optional */
	if (scan_req->scan_chan_gap && priv->phandle->pref_mac)
		scan_req->scan_chan_gap |= GAP_FLAG_OPTIONAL;
	status = woal_request_userscan(priv, wait_option, scan_req);
	kfree(scan_req);
	LEAVE();
	return status;
}

/**
 *  @brief Scan Network
 *
 *  @param dev          A pointer to net_device structure
 *  @param info         A pointer to iw_request_info structure
 *  @param vwrq         A pointer to iw_param structure
 *  @param extra        A pointer to extra data buf
 *
 *  @return             0--success, otherwise fail
 */
static int woal_set_scan(struct net_device *dev, struct iw_request_info *info,
			 union iwreq_data *wrqu, char *extra)
{
	int ret = 0;
	moal_private *priv = (moal_private *)netdev_priv(dev);
	struct iw_param *vwrq = &wrqu->param;
	moal_handle *handle = priv->phandle;
#if WIRELESS_EXT >= 18
	struct iw_scan_req *req;
	struct iw_point *dwrq = (struct iw_point *)vwrq;
#endif
	mlan_802_11_ssid req_ssid;

	ENTER();
	if (handle->scan_pending_on_block == MTRUE) {
		PRINTM(MINFO, "scan already in processing...\n");
		LEAVE();
		return ret;
	}
#ifdef REASSOCIATION
	if (MOAL_ACQ_SEMAPHORE_BLOCK(&handle->reassoc_sem)) {
		PRINTM(MERROR, "Acquire semaphore error, woal_set_scan\n");
		LEAVE();
		return -EBUSY;
	}
#endif /* REASSOCIATION */
	priv->report_scan_result = MTRUE;

	memset(&req_ssid, 0x00, sizeof(mlan_802_11_ssid));

#if WIRELESS_EXT >= 18
	if ((dwrq->flags & IW_SCAN_THIS_ESSID) &&
	    (dwrq->length == sizeof(struct iw_scan_req))) {
		req = (struct iw_scan_req *)extra;

		if (req->essid_len <= MLAN_MAX_SSID_LENGTH) {
			req_ssid.ssid_len = req->essid_len;
			moal_memcpy_ext(priv->phandle, req_ssid.ssid,
					(t_u8 *)req->essid, req->essid_len,
					sizeof(req_ssid.ssid));
			if (MLAN_STATUS_SUCCESS !=
			    woal_wext_request_scan(priv, MOAL_NO_WAIT,
						   &req_ssid)) {
				ret = -EFAULT;
				goto done;
			}
		}
	} else {
#endif
		if (MLAN_STATUS_SUCCESS !=
		    woal_wext_request_scan(priv, MOAL_NO_WAIT, &req_ssid)) {
			ret = -EFAULT;
			goto done;
		}
#if WIRELESS_EXT >= 18
	}
#endif

	if (priv->phandle->surprise_removed) {
		ret = -EFAULT;
		goto done;
	}

done:
#ifdef REASSOCIATION
	MOAL_REL_SEMAPHORE(&handle->reassoc_sem);
#endif

	LEAVE();
	return ret;
}

/**
 * @brief Request scan based on connect parameter
 *
 * @param priv            A pointer to moal_private structure
 * @param ssid_bssid      A pointer to mlan_ssid_bssid structure
 *
 * @return                0 -- success, otherwise fail
 */
static int woal_owe_specific_scan(moal_private *priv,
				  mlan_ssid_bssid *ssid_bssid)
{
	moal_handle *handle = priv->phandle;
	int ret = 0;
	wlan_user_scan_cfg *scan_req;
	ENTER();
	if (handle->scan_pending_on_block == MTRUE) {
		PRINTM(MINFO, "scan already in processing...\n");
		LEAVE();
		return ret;
	}
	scan_req = (wlan_user_scan_cfg *)kmalloc(sizeof(wlan_user_scan_cfg),
						 GFP_KERNEL);
	if (!scan_req) {
		PRINTM(MERROR, "Malloc buffer failed\n");
		LEAVE();
		return -ENOMEM;
	}

	priv->report_scan_result = MTRUE;
	memset(scan_req, 0x00, sizeof(wlan_user_scan_cfg));
	scan_req->keep_previous_scan = MTRUE;
	moal_memcpy_ext(priv->phandle, scan_req->ssid_list[0].ssid,
			ssid_bssid->trans_ssid.ssid,
			MIN(ssid_bssid->trans_ssid.ssid_len,
			    MLAN_MAX_SSID_LENGTH),
			sizeof(scan_req->ssid_list[0].ssid));
	scan_req->ssid_list[0].max_len = 0;
	scan_req->chan_list[0].chan_number = ssid_bssid->channel;
	if (ssid_bssid->bss_band == BAND_A)
		scan_req->chan_list[0].radio_type = BAND_5GHZ;
	else
		scan_req->chan_list[0].radio_type = BAND_2GHZ;
	scan_req->chan_list[0].scan_time = 0;
	// TODO need set to PASSIVE TO ACTIVE on DFS channel
	scan_req->chan_list[0].scan_type = MLAN_SCAN_TYPE_ACTIVE;

	moal_memcpy_ext(priv->phandle, scan_req->random_mac, priv->random_mac,
			ETH_ALEN, sizeof(scan_req->random_mac));
	ret = woal_request_userscan(priv, MOAL_IOCTL_WAIT, scan_req);
	kfree(scan_req);
	LEAVE();
	return ret;
}

/**
 *  @brief Set essid
 *
 *  @param dev          A pointer to net_device structure
 *  @param info         A pointer to iw_request_info structure
 *  @param wrqu         A pointer to iwreq_data union
 *  @param extra        A pointer to extra data buf
 *
 *  @return             0--success, otherwise fail
 */
static int woal_set_essid(struct net_device *dev, struct iw_request_info *info,
			  union iwreq_data *wrqu, char *extra)
{
	moal_private *priv = (moal_private *)netdev_priv(dev);
	struct iw_point *dwrq = &wrqu->data;
	mlan_802_11_ssid req_ssid;
	mlan_ssid_bssid *ssid_bssid = NULL;
	mlan_ssid_bssid *owe_ssid_bssid = NULL;
#ifdef REASSOCIATION
	moal_handle *handle = priv->phandle;
	mlan_bss_info bss_info;
#endif
	int ret = 0;
	t_u32 mode = 0;

	ENTER();

#ifdef REASSOCIATION
	/* Cancel re-association */
	priv->reassoc_required = MFALSE;

	if (MOAL_ACQ_SEMAPHORE_BLOCK(&handle->reassoc_sem)) {
		PRINTM(MERROR, "Acquire semaphore error, woal_set_essid\n");
		LEAVE();
		return -EBUSY;
	}
#endif /* REASSOCIATION */

	/* Check the size of the string */
	if (dwrq->length > IW_ESSID_MAX_SIZE + 1) {
		ret = -E2BIG;
		goto setessid_ret;
	}
	if (priv->scan_type == MLAN_SCAN_TYPE_PASSIVE)
		woal_set_scan_type(priv, MLAN_SCAN_TYPE_ACTIVE);
	memset(&req_ssid, 0, sizeof(mlan_802_11_ssid));
	ssid_bssid = kzalloc(sizeof(mlan_ssid_bssid), GFP_ATOMIC);
	if (!ssid_bssid) {
		PRINTM(MERROR, "Fail to allocate ssid_bssid buffer\n");
		ret = -ENOMEM;
		goto setessid_ret;
	}

#if WIRELESS_EXT > 20
	req_ssid.ssid_len = dwrq->length;
#else
	req_ssid.ssid_len = dwrq->length - 1;
#endif

	/*
	 * Check if we asked for `any' or 'particular'
	 */
	if (!dwrq->flags) {
#ifdef REASSOCIATION
		if (!req_ssid.ssid_len) {
			memset(&priv->prev_ssid_bssid.ssid, 0x00,
			       sizeof(mlan_802_11_ssid));
			memset(&priv->prev_ssid_bssid.bssid, 0x00,
			       MLAN_MAC_ADDR_LENGTH);
			goto setessid_ret;
		}
#endif
		/* Do normal SSID scanning */
		if (MLAN_STATUS_SUCCESS !=
		    woal_request_scan(priv, MOAL_IOCTL_WAIT, NULL)) {
			ret = -EFAULT;
			goto setessid_ret;
		}
	} else {
		/* Set the SSID */
		moal_memcpy_ext(priv->phandle, req_ssid.ssid, extra,
				MIN(req_ssid.ssid_len, MLAN_MAX_SSID_LENGTH),
				sizeof(req_ssid.ssid));
		if (!req_ssid.ssid_len ||
		    (MFALSE == woal_ssid_valid(&req_ssid))) {
			PRINTM(MERROR, "Invalid SSID - aborting set_essid\n");
			ret = -EINVAL;
			goto setessid_ret;
		}

		PRINTM(MINFO, "Requested new SSID = %s\n",
		       (char *)req_ssid.ssid);
		moal_memcpy_ext(priv->phandle, &ssid_bssid->ssid, &req_ssid,
				sizeof(mlan_802_11_ssid),
				sizeof(mlan_802_11_ssid));
		if (MTRUE == woal_is_connected(priv, ssid_bssid)) {
			PRINTM(MIOCTL, "Already connect to the network\n");
			goto setessid_ret;
		}

		priv->auto_assoc_priv.drv_assoc.status = MFALSE;
		priv->auto_assoc_priv.drv_reconnect.status = MFALSE;
#ifdef REASSOCIATION
		if (priv->reassoc_on == MTRUE) {
			if (priv->auto_assoc_priv.auto_assoc_type_on &
			    (0x1 << (AUTO_ASSOC_TYPE_DRV_ASSOC - 1))) {
				if (priv->scan_type == MLAN_SCAN_TYPE_PASSIVE)
					woal_set_scan_type(
						priv, MLAN_SCAN_TYPE_PASSIVE);
				MOAL_REL_SEMAPHORE(&handle->reassoc_sem);
				moal_memcpy_ext(
					priv->phandle,
					&priv->prev_ssid_bssid.ssid, &req_ssid,
					sizeof(mlan_802_11_ssid),
					sizeof(priv->prev_ssid_bssid.ssid));
				priv->auto_assoc_priv.auto_assoc_trigger_flag =
					AUTO_ASSOC_TYPE_DRV_ASSOC;
				priv->auto_assoc_priv.drv_assoc.status = MTRUE;
				priv->reassoc_required = MTRUE;
				priv->phandle->is_reassoc_timer_set = MTRUE;
				PRINTM(MINFO,
				       " auto assoc: trigger driver auto assoc\n");
				woal_mod_timer(&priv->phandle->reassoc_timer,
					       0);
				ret = MLAN_STATUS_SUCCESS;

				goto setessid_ret;
			}
		}
#endif

		if (dwrq->flags != 0xFFFF) {
			if (MLAN_STATUS_SUCCESS !=
			    woal_find_essid(priv, ssid_bssid,
					    MOAL_IOCTL_WAIT)) {
				/* Do specific SSID scanning */
				if (MLAN_STATUS_SUCCESS !=
				    woal_request_scan(priv, MOAL_IOCTL_WAIT,
						      &req_ssid)) {
					ret = -EFAULT;
					goto setessid_ret;
				}
			}
		}
	}

	mode = woal_get_mode(priv, MOAL_IOCTL_WAIT);
	if (mode == IW_MODE_ADHOC)
		/* disconnect before try to associate */
		if (MLAN_STATUS_SUCCESS !=
		    woal_disconnect(priv, MOAL_IOCTL_WAIT, NULL,
				    DEF_DEAUTH_REASON_CODE)) {
			PRINTM(MERROR,
			       "Disconnect before association failed\n");
		}

	if (mode != IW_MODE_ADHOC) {
		if (MLAN_STATUS_SUCCESS !=
		    woal_find_best_network(priv, MOAL_IOCTL_WAIT, ssid_bssid)) {
			ret = -EFAULT;
			goto setessid_ret;
		}
		if (ssid_bssid->trans_ssid.ssid_len &&
		    (ssid_bssid->owe_transition_mode == OWE_TRANS_MODE_OPEN)) {
			// We need scan for OWE AP
			owe_ssid_bssid = (mlan_ssid_bssid *)kmalloc(
				sizeof(mlan_ssid_bssid), GFP_KERNEL);
			if (!owe_ssid_bssid) {
				PRINTM(MERROR, "Malloc buffer failed\n");
				ret = -ENOMEM;
				goto setessid_ret;
			}
			woal_owe_specific_scan(priv, ssid_bssid);
			memset(owe_ssid_bssid, 0, sizeof(mlan_ssid_bssid));
			moal_memcpy_ext(priv->phandle, &owe_ssid_bssid->ssid,
					&ssid_bssid->trans_ssid,
					sizeof(mlan_802_11_ssid),
					sizeof(owe_ssid_bssid->ssid));
			moal_memcpy_ext(priv->phandle, &owe_ssid_bssid->bssid,
					&ssid_bssid->trans_bssid,
					sizeof(mlan_802_11_mac_addr),
					sizeof(owe_ssid_bssid->bssid));
			if (MLAN_STATUS_SUCCESS ==
			    woal_find_essid(priv, owe_ssid_bssid,
					    MOAL_IOCTL_WAIT))
				moal_memcpy_ext(priv->phandle, ssid_bssid,
						owe_ssid_bssid,
						sizeof(mlan_ssid_bssid),
						sizeof(mlan_ssid_bssid));
		}
		if (MLAN_STATUS_SUCCESS !=
		    woal_11d_check_ap_channel(priv, MOAL_IOCTL_WAIT,
					      ssid_bssid)) {
			PRINTM(MERROR,
			       "The AP's channel is invalid for current region\n");
			ret = -EFAULT;
			goto setessid_ret;
		}
	} else if (MLAN_STATUS_SUCCESS !=
		   woal_find_best_network(priv, MOAL_IOCTL_WAIT, ssid_bssid))
		/* Adhoc start, Check the channel command */
		woal_11h_channel_check_ioctl(priv, MOAL_IOCTL_WAIT);

#ifdef REASSOCIATION
	if (priv->reassoc_on == MTRUE && req_ssid.ssid_len) {
		moal_memcpy_ext(priv->phandle, &priv->prev_ssid_bssid.ssid,
				&req_ssid, sizeof(mlan_802_11_ssid),
				sizeof(priv->prev_ssid_bssid.ssid));
		memset(&priv->prev_ssid_bssid.bssid, 0x00,
		       MLAN_MAC_ADDR_LENGTH);
	}
#endif /* REASSOCIATION */

	/* Connect to BSS by ESSID */
	memset(&ssid_bssid->bssid, 0, MLAN_MAC_ADDR_LENGTH);

	if (MLAN_STATUS_SUCCESS !=
	    woal_bss_start(priv, MOAL_IOCTL_WAIT, ssid_bssid)) {
		ret = -EFAULT;
		goto setessid_ret;
	}

#ifdef REASSOCIATION
	memset(&bss_info, 0, sizeof(bss_info));
	if (MLAN_STATUS_SUCCESS !=
	    woal_get_bss_info(priv, MOAL_IOCTL_WAIT, &bss_info)) {
		ret = -EFAULT;
		goto setessid_ret;
	}
	moal_memcpy_ext(priv->phandle, &priv->prev_ssid_bssid.ssid,
			&bss_info.ssid, sizeof(mlan_802_11_ssid),
			sizeof(priv->prev_ssid_bssid.ssid));
	moal_memcpy_ext(priv->phandle, &priv->prev_ssid_bssid.bssid,
			&bss_info.bssid, MLAN_MAC_ADDR_LENGTH,
			sizeof(priv->prev_ssid_bssid.bssid));
#endif /* REASSOCIATION */

setessid_ret:
	if (ssid_bssid)
		kfree(ssid_bssid);
	if (priv->scan_type == MLAN_SCAN_TYPE_PASSIVE)
		woal_set_scan_type(priv, MLAN_SCAN_TYPE_PASSIVE);
#ifdef REASSOCIATION
	MOAL_REL_SEMAPHORE(&handle->reassoc_sem);
#endif
	if (owe_ssid_bssid)
		kfree(owe_ssid_bssid);
	LEAVE();
	return ret;
}

/**
 *  @brief Get current essid
 *
 *  @param dev      A pointer to net_device structure
 *  @param info     A pointer to iw_request_info structure
 *  @param wrqu     A pointer to iwreq_data union
 *  @param extra    A pointer to extra data buf
 *
 *  @return         0--success, otherwise fail
 */
static int woal_get_essid(struct net_device *dev, struct iw_request_info *info,
			  union iwreq_data *wrqu, char *extra)
{
	moal_private *priv = (moal_private *)netdev_priv(dev);
	struct iw_point *dwrq = &wrqu->essid;
	mlan_bss_info bss_info;
	int ret = 0;

	ENTER();

	memset(&bss_info, 0, sizeof(bss_info));

	if (MLAN_STATUS_SUCCESS !=
	    woal_get_bss_info(priv, MOAL_IOCTL_WAIT, &bss_info)) {
		ret = -EFAULT;
		goto done;
	}

	if (bss_info.media_connected) {
		dwrq->length = MIN(dwrq->length, bss_info.ssid.ssid_len);
		moal_memcpy_ext(priv->phandle, extra, bss_info.ssid.ssid,
				dwrq->length, dwrq->length);
	} else
		dwrq->length = 0;

	if (bss_info.scan_table_idx)
		dwrq->flags = (bss_info.scan_table_idx + 1) & IW_ENCODE_INDEX;
	else
		dwrq->flags = 1;

done:
	LEAVE();
	return ret;
}

/**
 *  @brief  Retrieve the scan table entries via wireless tools IOCTL call
 *
 *  @param dev          A pointer to net_device structure
 *  @param info         A pointer to iw_request_info structure
 *  @param wrqu         A pointer to iwreq_data union
 *  @param extra        A pointer to extra data buf
 *
 *  @return             0--success, otherwise fail
 */
static int woal_get_scan(struct net_device *dev, struct iw_request_info *info,
			 union iwreq_data *wrqu, char *extra)
{
	moal_private *priv = (moal_private *)netdev_priv(dev);
	struct iw_point *dwrq = &wrqu->data;
	int ret = 0;
	char *current_ev = extra;
	char *end_buf = extra + IW_SCAN_MAX_DATA;
	char *current_val; /* For rates */
	struct iw_event iwe; /* Temporary buffer */
	unsigned int i;
	unsigned int j;
	mlan_scan_resp scan_resp;
	mlan_bss_info bss_info;
	BSSDescriptor_t *scan_table;
	t_u16 buf_size = 16 + 256 * 2;
	char *buf = NULL;
	char *ptr;
#if WIRELESS_EXT >= 18
	t_u8 *praw_data;
#endif
	int beacon_size;
	t_u8 *pbeacon;
	IEEEtypes_ElementId_e element_id;
	t_u8 element_len;
	gfp_t flag;
	int event_len = 0;

	ENTER();

	if (priv->phandle->scan_pending_on_block == MTRUE) {
		LEAVE();
		return -EAGAIN;
	}
	flag = (in_atomic() || irqs_disabled()) ? GFP_ATOMIC : GFP_KERNEL;
	buf = kzalloc((buf_size), flag);
	if (!buf) {
		PRINTM(MERROR, "Cannot allocate buffer!\n");
		ret = -EFAULT;
		goto done;
	}

	memset(&bss_info, 0, sizeof(bss_info));
	if (MLAN_STATUS_SUCCESS !=
	    woal_get_bss_info(priv, MOAL_IOCTL_WAIT, &bss_info)) {
		ret = -EFAULT;
		goto done;
	}
	memset(&scan_resp, 0, sizeof(scan_resp));
	if (MLAN_STATUS_SUCCESS !=
	    woal_get_scan_table(priv, MOAL_IOCTL_WAIT, &scan_resp)) {
		ret = -EFAULT;
		goto done;
	}
	scan_table = (BSSDescriptor_t *)scan_resp.pscan_table;
	if (dwrq->length)
		end_buf = extra + dwrq->length;
	if (priv->media_connected == MTRUE)
		PRINTM(MINFO, "Current Ssid: %-32s\n", bss_info.ssid.ssid);
	PRINTM(MINFO, "Scan: Get: NumInScanTable = %d\n",
	       (int)scan_resp.num_in_scan_table);

#if WIRELESS_EXT > 13
	/* The old API using SIOCGIWAPLIST had a hard limit of
	 * IW_MAX_AP. The new API using SIOCGIWSCAN is only
	 * limited by buffer size WE-14 -> WE-16 the buffer is
	 * limited to IW_SCAN_MAX_DATA bytes which is 4096.
	 */
	for (i = 0; i < MIN(scan_resp.num_in_scan_table, 64); i++) {
		if ((current_ev + MAX_SCAN_CELL_SIZE) >= end_buf) {
			PRINTM(MINFO,
			       "i=%d break out: current_ev=%p end_buf=%p "
			       "MAX_SCAN_CELL_SIZE=%d\n",
			       i, current_ev, end_buf,
			       (t_u32)MAX_SCAN_CELL_SIZE);
			ret = -E2BIG;
			break;
		}
		if (!scan_table[i].freq) {
			PRINTM(MWARN, "Invalid channel number %d\n",
			       (int)scan_table[i].channel);
			continue;
		}
		PRINTM(MINFO, "i=%d  Ssid: %-32s\n", i,
		       scan_table[i].ssid.ssid);

		/* check ssid is valid or not, ex. hidden ssid will be filter
		 * out */
		if (woal_ssid_valid(&scan_table[i].ssid) == MFALSE)
			continue;

		/* First entry *MUST* be the AP MAC address */
		iwe.cmd = SIOCGIWAP;
		iwe.u.ap_addr.sa_family = ARPHRD_ETHER;
		moal_memcpy_ext(priv->phandle, iwe.u.ap_addr.sa_data,
				&scan_table[i].mac_address, ETH_ALEN,
				sizeof(iwe.u.ap_addr.sa_data_min));

		iwe.len = IW_EV_ADDR_LEN;
		current_ev = IWE_STREAM_ADD_EVENT(info, current_ev, end_buf,
						  &iwe, iwe.len);

		/* Add the ESSID */
		iwe.u.data.length = scan_table[i].ssid.ssid_len;

		if (iwe.u.data.length > 32)
			iwe.u.data.length = 32;

		iwe.cmd = SIOCGIWESSID;
		iwe.u.essid.flags = (i + 1) & IW_ENCODE_INDEX;
		iwe.len = IW_EV_POINT_LEN + iwe.u.data.length;
		current_ev =
			IWE_STREAM_ADD_POINT(info, current_ev, end_buf, &iwe,
					     (char *)scan_table[i].ssid.ssid);

		/* Add mode */
		iwe.cmd = SIOCGIWMODE;
		if (scan_table[i].bss_mode == MLAN_BSS_MODE_INFRA)
			iwe.u.mode = IW_MODE_MASTER;
		else
			iwe.u.mode = IW_MODE_AUTO;

		iwe.len = IW_EV_UINT_LEN;
		current_ev = IWE_STREAM_ADD_EVENT(info, current_ev, end_buf,
						  &iwe, iwe.len);

		/* Frequency */
		iwe.cmd = SIOCGIWFREQ;
		iwe.u.freq.m = (long)scan_table[i].freq;
		iwe.u.freq.e = 6;
		iwe.u.freq.flags = IW_FREQ_FIXED;
		iwe.len = IW_EV_FREQ_LEN;
		current_ev = IWE_STREAM_ADD_EVENT(info, current_ev, end_buf,
						  &iwe, iwe.len);

		memset(&iwe, 0, sizeof(iwe));
		/* Add quality statistics */
		iwe.cmd = IWEVQUAL;
		iwe.u.qual.level = SCAN_RSSI(scan_table[i].rssi);
		if (!bss_info.bcn_nf_last)
			iwe.u.qual.noise = MRVDRV_NF_DEFAULT_SCAN_VALUE;
		else
			iwe.u.qual.noise = bss_info.bcn_nf_last;

		iwe.u.qual.qual =
			woal_rssi_to_quality((t_s16)(iwe.u.qual.level - 0x100));
		iwe.len = IW_EV_QUAL_LEN;
		current_ev = IWE_STREAM_ADD_EVENT(info, current_ev, end_buf,
						  &iwe, iwe.len);

		/* Add encryption capability */
		iwe.cmd = SIOCGIWENCODE;
		if (scan_table[i].privacy)
			iwe.u.data.flags = IW_ENCODE_ENABLED | IW_ENCODE_NOKEY;
		else
			iwe.u.data.flags = IW_ENCODE_DISABLED;

		iwe.u.data.length = 0;
		iwe.len = IW_EV_POINT_LEN + iwe.u.data.length;
		current_ev = IWE_STREAM_ADD_POINT(info, current_ev, end_buf,
						  &iwe, NULL);

		current_val = current_ev + IW_EV_LCP_LEN;

		iwe.cmd = SIOCGIWRATE;

		iwe.u.bitrate.fixed = 0;
		iwe.u.bitrate.disabled = 0;
		iwe.u.bitrate.value = 0;

		/* Bit rate given in 500 kb/s units (+ 0x80) */
		for (j = 0; j < sizeof(scan_table[i].supported_rates); j++) {
			if (!scan_table[i].supported_rates[j])
				break;

			iwe.u.bitrate.value =
				(scan_table[i].supported_rates[j] & 0x7f) *
				500000;
			iwe.len = IW_EV_PARAM_LEN;
			current_val = IWE_STREAM_ADD_VALUE(info, current_ev,
							   current_val, end_buf,
							   &iwe, iwe.len);
		}

		/* Check if an event is added */
		event_len = current_val - current_ev;
		if ((unsigned int)event_len >= IW_EV_PARAM_LEN)
			current_ev = current_val;

		/* Beacon Interval */
		memset(&iwe, 0, sizeof(iwe));
		ptr = buf;
		ptr += snprintf(ptr, MAX_BUF_LEN, "Beacon interval=%d",
				scan_table[i].beacon_period);

		iwe.u.data.length = strlen(buf);
		iwe.cmd = IWEVCUSTOM;
		iwe.len = IW_EV_POINT_LEN + iwe.u.data.length;
		current_ev = IWE_STREAM_ADD_POINT(info, current_ev, end_buf,
						  &iwe, buf);
		current_val = current_ev + IW_EV_LCP_LEN + strlen(buf);

		/* Parse and send the IEs */
		pbeacon = scan_table[i].pbeacon_buf;
		beacon_size = scan_table[i].beacon_buf_size;

		/* Skip time stamp, beacon interval and capability */
		if (pbeacon) {
			pbeacon += sizeof(scan_table[i].beacon_period) +
				   sizeof(scan_table[i].time_stamp) +
				   sizeof(scan_table[i].cap_info);
			beacon_size -= sizeof(scan_table[i].beacon_period) +
				       sizeof(scan_table[i].time_stamp) +
				       sizeof(scan_table[i].cap_info);

			while ((unsigned int)beacon_size >=
			       sizeof(IEEEtypes_Header_t)) {
				element_id = (IEEEtypes_ElementId_e)(*(
					t_u8 *)pbeacon);
				element_len = *((t_u8 *)pbeacon + 1);
				if ((unsigned int)beacon_size <
				    (unsigned int)element_len +
					    sizeof(IEEEtypes_Header_t)) {
					PRINTM(MERROR,
					       "Get scan: Error in processing IE, "
					       "bytes left < IE length\n");
					break;
				}

				switch (element_id) {
#if WIRELESS_EXT >= 18
				case VENDOR_SPECIFIC_221:
				case RSN_IE:
				case WAPI_IE:
					praw_data = (t_u8 *)pbeacon;
					memset(&iwe, 0, sizeof(iwe));
					memset(buf, 0, buf_size);
					ptr = buf;
					moal_memcpy_ext(
						priv->phandle, buf, praw_data,
						element_len +
							sizeof(IEEEtypes_Header_t),
						buf_size);
					iwe.cmd = IWEVGENIE;
					iwe.u.data.length =
						element_len +
						sizeof(IEEEtypes_Header_t);
					iwe.len = IW_EV_POINT_LEN +
						  iwe.u.data.length;
					current_ev = IWE_STREAM_ADD_POINT(
						info, current_ev, end_buf, &iwe,
						buf);
					break;
#endif
				default:
					break;
				}
				pbeacon += element_len +
					   sizeof(IEEEtypes_Header_t);
				beacon_size -= element_len +
					       sizeof(IEEEtypes_Header_t);
			}
		}

#if WIRELESS_EXT > 14
		memset(&iwe, 0, sizeof(iwe));
		memset(buf, 0, buf_size);
		ptr = buf;
		ptr += snprintf(ptr, MAX_BUF_LEN, "band=");
		memset(&iwe, 0, sizeof(iwe));
		if (scan_table[i].bss_band == BAND_A)
			snprintf(ptr, MAX_BUF_LEN, "a");
		else
			snprintf(ptr, MAX_BUF_LEN, "bg");
		iwe.u.data.length = strlen(buf);
		PRINTM(MINFO, "iwe.u.data.length %d\n", iwe.u.data.length);
		PRINTM(MINFO, "BUF: %s\n", buf);
		iwe.cmd = IWEVCUSTOM;
		iwe.len = IW_EV_POINT_LEN + iwe.u.data.length;
		current_ev = IWE_STREAM_ADD_POINT(info, current_ev, end_buf,
						  &iwe, buf);
#endif
		current_val = current_ev + IW_EV_LCP_LEN;

		/*
		 * Check if we added any event
		 */
		event_len = current_val - current_ev;
		if ((unsigned int)event_len > IW_EV_LCP_LEN)
			current_ev = current_val;
	}

	dwrq->length = (current_ev - extra);
	dwrq->flags = 0;
#endif

done:
	kfree(buf);
	LEAVE();
	return ret;
}

/**
 * iwconfig settable callbacks
 */
static const iw_handler woal_handler[] = {
	(iw_handler)woal_config_commit, /* SIOCSIWCOMMIT */
	(iw_handler)woal_get_name, /* SIOCGIWNAME */
	(iw_handler)NULL, /* SIOCSIWNWID */
	(iw_handler)NULL, /* SIOCGIWNWID */
	(iw_handler)woal_set_freq, /* SIOCSIWFREQ */
	(iw_handler)woal_get_freq, /* SIOCGIWFREQ */
	(iw_handler)woal_set_bss_mode, /* SIOCSIWMODE */
	(iw_handler)woal_get_bss_mode, /* SIOCGIWMODE */
	(iw_handler)woal_set_sens, /* SIOCSIWSENS */
	(iw_handler)woal_get_sens, /* SIOCGIWSENS */
	(iw_handler)NULL, /* SIOCSIWRANGE */
	(iw_handler)woal_get_range, /* SIOCGIWRANGE */
	(iw_handler)woal_set_priv, /* SIOCSIWPRIV */
	(iw_handler)NULL, /* SIOCGIWPRIV */
	(iw_handler)NULL, /* SIOCSIWSTATS */
	(iw_handler)NULL, /* SIOCGIWSTATS */
#if WIRELESS_EXT > 15
#ifdef CONFIG_WEXT_SPY
	iw_handler_set_spy, /* SIOCSIWSPY */
	iw_handler_get_spy, /* SIOCGIWSPY */
	iw_handler_set_thrspy, /* SIOCSIWTHRSPY */
	iw_handler_get_thrspy, /* SIOCGIWTHRSPY */
#else
	(iw_handler)NULL, /* -- hole -- */
	(iw_handler)NULL, /* -- hole -- */
	(iw_handler)NULL, /* -- hole -- */
	(iw_handler)NULL, /* -- hole -- */
#endif
#else /* WIRELESS_EXT > 15 */
	(iw_handler)NULL, /* -- hole -- */
	(iw_handler)NULL, /* -- hole -- */
	(iw_handler)NULL, /* -- hole -- */
	(iw_handler)NULL, /* -- hole -- */
#endif /* WIRELESS_EXT > 15 */
	(iw_handler)woal_set_wap, /* SIOCSIWAP */
	(iw_handler)woal_get_wap, /* SIOCGIWAP */
#if WIRELESS_EXT >= 18
	(iw_handler)woal_set_mlme, /* SIOCSIWMLME  */
#else
	(iw_handler)NULL, /* -- hole -- */
#endif
	/* (iw_handler) wlan_get_aplist, */ /* SIOCGIWAPLIST */
	NULL, /* SIOCGIWAPLIST */
#if WIRELESS_EXT > 13
	(iw_handler)woal_set_scan, /* SIOCSIWSCAN */
	(iw_handler)woal_get_scan, /* SIOCGIWSCAN */
#else /* WIRELESS_EXT > 13 */
	(iw_handler)NULL, /* SIOCSIWSCAN */
	(iw_handler)NULL, /* SIOCGIWSCAN */
#endif /* WIRELESS_EXT > 13 */
	(iw_handler)woal_set_essid, /* SIOCSIWESSID */
	(iw_handler)woal_get_essid, /* SIOCGIWESSID */
	(iw_handler)woal_set_nick, /* SIOCSIWNICKN */
	(iw_handler)woal_get_nick, /* SIOCGIWNICKN */
	(iw_handler)NULL, /* -- hole -- */
	(iw_handler)NULL, /* -- hole -- */
	(iw_handler)woal_set_rate, /* SIOCSIWRATE */
	(iw_handler)woal_get_rate, /* SIOCGIWRATE */
	(iw_handler)woal_set_rts, /* SIOCSIWRTS */
	(iw_handler)woal_get_rts, /* SIOCGIWRTS */
	(iw_handler)woal_set_frag, /* SIOCSIWFRAG */
	(iw_handler)woal_get_frag, /* SIOCGIWFRAG */
	(iw_handler)woal_set_txpow, /* SIOCSIWTXPOW */
	(iw_handler)woal_get_txpow, /* SIOCGIWTXPOW */
	(iw_handler)woal_set_retry, /* SIOCSIWRETRY */
	(iw_handler)woal_get_retry, /* SIOCGIWRETRY */
	(iw_handler)woal_set_encode, /* SIOCSIWENCODE */
	(iw_handler)woal_get_encode, /* SIOCGIWENCODE */
	(iw_handler)woal_set_power, /* SIOCSIWPOWER */
	(iw_handler)woal_get_power, /* SIOCGIWPOWER */
#if (WIRELESS_EXT >= 18)
	(iw_handler)NULL, /* -- hole -- */
	(iw_handler)NULL, /* -- hole -- */
	(iw_handler)woal_set_gen_ie, /* SIOCSIWGENIE */
	(iw_handler)woal_get_gen_ie, /* SIOCGIWGENIE */
	(iw_handler)woal_set_auth, /* SIOCSIWAUTH  */
	(iw_handler)woal_get_auth, /* SIOCGIWAUTH  */
	(iw_handler)woal_set_encode_ext, /* SIOCSIWENCODEEXT */
	(iw_handler)woal_get_encode_ext, /* SIOCGIWENCODEEXT */
	(iw_handler)woal_set_pmksa, /* SIOCSIWPMKSA */
#endif /* WIRELESSS_EXT >= 18 */
};

/**
 * iwpriv settable callbacks
 */
static const iw_handler woal_private_handler[] = {
	NULL, /* SIOCIWFIRSTPRIV */
};
#endif /* STA_SUPPORT */

/********************************************************
			Global Functions
********************************************************/

#if WIRELESS_EXT > 14

/**
 *  @brief This function sends customized event to application.
 *
 *  @param priv    A pointer to moal_private structure
 *  @param str     A pointer to event string
 *
 *  @return        N/A
 */
void woal_send_iwevcustom_event(moal_private *priv, char *str)
{
	union iwreq_data iwrq;
	char buf[256];

	ENTER();
	/* Check register_netdevice is completed before sending*/
	if (priv->netdev->reg_state != NETREG_REGISTERED) {
		LEAVE();
		return;
	}

	memset(&iwrq, 0, sizeof(union iwreq_data));
	memset(buf, 0, sizeof(buf));

	if (snprintf(buf, sizeof(buf) - 1, "%s", str) <= 0) {
		PRINTM(MERROR, "Failed to write custom event in iwreq buf\n");
		LEAVE();
		return;
	}

	iwrq.data.pointer = (t_u8 __user *)buf;
	iwrq.data.length = strlen(buf) + 1;

	/* Send Event to upper layer */
	wireless_send_event(priv->netdev, IWEVCUSTOM, &iwrq, buf);
	PRINTM(MINFO, "Wireless event %s is sent to application\n", str);

	LEAVE();
	return;
}
#endif

#if WIRELESS_EXT >= 18
/**
 *  @brief This function sends mic error event to application.
 *
 *  @param priv    A pointer to moal_private structure
 *  @param event   MIC MERROR EVENT.
 *
 *  @return        N/A
 */
void woal_send_mic_error_event(moal_private *priv, t_u32 event)
{
	union iwreq_data iwrq;
	struct iw_michaelmicfailure mic;

	ENTER();

	memset(&iwrq, 0, sizeof(iwrq));
	memset(&mic, 0, sizeof(mic));
	if (event == MLAN_EVENT_ID_FW_MIC_ERR_UNI)
		mic.flags = IW_MICFAILURE_PAIRWISE;
	else
		mic.flags = IW_MICFAILURE_GROUP;
	iwrq.data.pointer = (t_u8 __user *)&mic;
	iwrq.data.length = sizeof(mic);

	wireless_send_event(priv->netdev, IWEVMICHAELMICFAILURE, &iwrq,
			    (char *)&mic);

	LEAVE();
	return;
}
#endif

// clang-format off
#ifdef STA_SUPPORT
/** wlan_handler_def */
struct iw_handler_def woal_handler_def = {
	.num_standard = ARRAY_SIZE(woal_handler),
	.num_private = ARRAY_SIZE(woal_private_handler),
	.num_private_args = ARRAY_SIZE(woal_private_args),
	.standard = (const iw_handler *)woal_handler,
	.private = (const iw_handler *)woal_private_handler,
	.private_args = (const struct iw_priv_args *)woal_private_args,
#if WIRELESS_EXT > 20
	.get_wireless_stats = woal_get_wireless_stats,
#endif
};
// clang-format on

/**
 *  @brief Get wireless statistics
 *
 *  @param dev          A pointer to net_device structure
 *
 *  @return             A pointer to iw_statistics buf
 */
struct iw_statistics *woal_get_wireless_stats(struct net_device *dev)
{
	moal_private *priv = (moal_private *)netdev_priv(dev);
	t_u16 wait_option = MOAL_IOCTL_WAIT;

	ENTER();

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 31)
	/*
	 * Since schedule() is not allowed from an atomic context
	 * such as when dev_base_lock for netdevices is acquired
	 * for reading/writing in kernel before this call, HostCmd
	 * is issued in non-blocking way in such contexts and
	 * blocking in other cases.
	 */
	if (in_atomic() || !write_can_lock(&dev_base_lock))
		wait_option = MOAL_NO_WAIT;
#endif

	priv->w_stats.status = woal_get_mode(priv, wait_option);
	priv->w_stats.discard.retries = priv->stats.tx_errors;
	priv->w_stats.qual.qual = 0;

	/* Send RSSI command to get beacon RSSI/NF, valid only if associated */
	if (priv->media_connected == MTRUE) {
		if (MLAN_STATUS_SUCCESS ==
		    woal_get_signal_info(priv, wait_option, NULL))
			priv->w_stats.qual.qual = woal_rssi_to_quality(
				(t_s16)(priv->w_stats.qual.level - 0x100));
	}
#if WIRELESS_EXT > 18
	priv->w_stats.qual.updated |= (IW_QUAL_ALL_UPDATED | IW_QUAL_DBM);
#else
	priv->w_stats.qual.updated |= 7;
#endif
	if (!priv->w_stats.qual.noise && priv->media_connected == MTRUE)
		priv->w_stats.qual.noise = MRVDRV_NF_DEFAULT_SCAN_VALUE;

	PRINTM(MINFO, "Link Quality = %#x\n", priv->w_stats.qual.qual);
	PRINTM(MINFO, "Signal Level = %#x\n", priv->w_stats.qual.level);
	PRINTM(MINFO, "Noise = %#x\n", priv->w_stats.qual.noise);
	priv->w_stats.discard.code = 0;
	if (MLAN_STATUS_SUCCESS != woal_get_stats_info(priv, wait_option, NULL))
		PRINTM(MERROR, "Error getting stats information\n");

	LEAVE();
	return &priv->w_stats;
}
#endif /* STA_SUPPORT */
