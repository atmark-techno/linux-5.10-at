/** @file moal_uap.h
 *
 * @brief This file contains uap driver specific defines etc.
 *
 *
 * Copyright 2008-2025, NXP
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
    02/02/2009: initial version
********************************************************/

#ifndef _MOAL_UAP_H
#define _MOAL_UAP_H

/** Private command ID to send ioctl */
#define UAP_IOCTL_CMD (SIOCDEVPRIVATE + 2)
/** Updating ADDBA variables */
#define UAP_ADDBA_PARA 0
/** Updating priority table for AMPDU/AMSDU */
#define UAP_AGGR_PRIOTBL 1
/** Updating addbareject table */

#define UAP_ADDBA_REJECT 2
/** Get FW INFO */
#define UAP_FW_INFO 4
/** Updating Deep sleep variables */
#define UAP_DEEP_SLEEP 3
/** Tx data pause subcommand */
#define UAP_TX_DATA_PAUSE 5
#ifdef SDIO
/** sdcmd52 read write subcommand */
#define UAP_SDCMD52_RW 6
#endif
/** snmp mib subcommand */
#define UAP_SNMP_MIB 7
/** domain info subcommand */
#define UAP_DOMAIN_INFO 8
/** TX beamforming configuration */
#define UAP_TX_BF_CFG 9
/** dfs testing subcommand */
#define UAP_DFS_TESTING 10
/** sub command ID to set/get Host Sleep configuration */
#define UAP_HS_CFG 11
/** sub command ID to set/get Host Sleep Parameters */
#define UAP_HS_SET_PARA 12

/** Management Frame Control Mask */
#define UAP_MGMT_FRAME_CONTROL 13

#define UAP_TX_RATE_CFG 14

/** Subcommand ID to set/get antenna configuration */
#define UAP_ANTENNA_CFG 15

#define UAP_DFS_REPEATER_MODE 16

#define UAP_CAC_TIMER_STATUS 17

/** Skip CAC */
#define UAP_SKIP_CAC 18

#define UAP_HT_TX_CFG 19

#define UAP_VHT_CFG 20

#define UAP_HT_STREAM_CFG 21

#define UAP_OPERATION_CTRL 22

#define UAP_CHAN_SWITCH_COUNT_CFG 23
#define UAP_BAND_STEER 24

#define UAP_BEACON_STUCK_DETECT 25

/** wacp_mode Config */
#define UAP_WACP_MODE 26

/** Private command ID to Power Mode */
#define UAP_POWER_MODE (SIOCDEVPRIVATE + 3)

/** Private command id to start/stop/reset bss */
#define UAP_BSS_CTRL (SIOCDEVPRIVATE + 4)
/** BSS START */
#define UAP_BSS_START 0
/** BSS STOP */
#define UAP_BSS_STOP 1
/** BSS RESET */
#define UAP_BSS_RESET 2

/* HE MAC Capabilities Information field BIT 1 for TWT Req */
#define HE_MAC_CAP_TWT_REQ_SUPPORT MBIT(1)
/* HE MAC Capabilities Information field BIT 2 for TWT Resp*/
#define HE_MAC_CAP_TWT_RESP_SUPPORT MBIT(2)

/** wapi_msg */
typedef struct _wapi_msg {
	/** message type */
	t_u16 msg_type;
	/** message len */
	t_u16 msg_len;
	/** message */
	t_u8 msg[96];
} wapi_msg;

/* wapi key msg */
typedef struct _wapi_key_msg {
	/** mac address */
	t_u8 mac_addr[MLAN_MAC_ADDR_LENGTH];
	/** pad */
	t_u8 pad;
	/** key id */
	t_u8 key_id;
	/** key */
	t_u8 key[32];
} wapi_key_msg;

/** Private command ID to set wapi info */
#define UAP_WAPI_MSG (SIOCDEVPRIVATE + 10)
/** set wapi flag */
#define P80211_PACKET_WAPIFLAG 0x0001
/** set wapi key */
#define P80211_PACKET_SETKEY 0x0003
/** wapi mode psk */
#define WAPI_MODE_PSK 0x04
/** wapi mode certificate */
#define WAPI_MODE_CERT 0x08

typedef struct _tx_rate_cfg_t {
	/** sub command */
	int subcmd;
	/** Action */
	int action;
	/** Rate format */
	int rate_format;
	/** Rate configured */
	int rate;
	/** nss */
	int nss;
	/** user_data_cnt */
	int user_data_cnt;
	/** Rate bitmap */
	t_u16 bitmap_rates[MAX_BITMAP_RATES_SIZE];
	/** Rate Setting */
	t_u16 rate_setting;
	/** Only set auto tx fix rate */
	t_u16 auto_null_fixrate_enable;

} tx_rate_cfg_t;

/** ant_cfg structure */
typedef struct _ant_cfg_t {
	/** Subcommand */
	int subcmd;
	/** Action */
	int action;
	/** TX mode configured */
	int tx_mode;
	/** RX mode configured */
	int rx_mode;
	/** TX mode 6G configured */
	t_u8 tx_mode_6g;
	/** RX mode 6G configured */
	t_u8 rx_mode_6g;
} ant_cfg_t;

/** htstream_cfg structure */
typedef struct _htstream_cfg_t {
	/** Subcommand */
	int subcmd;
	/** Action */
	int action;
	/** HT stream configuration */
	t_u32 stream_cfg;
} htstream_cfg_t;

/* dfs repeater mode */
typedef struct _dfs_repeater_mode {
	/** subcmd */
	t_u32 subcmd;
	/** set/get */
	t_u32 action;
	/** mode */
	t_u32 mode;
} dfs_repeater_mode;

/* */
typedef struct _cac_timer_status {
	/** subcmd */
	t_u32 subcmd;
	/** set/get */
	t_u32 action;
	/** mode */
	t_u32 mode;
} cac_timer_status;

/** skip_cac parameters */
typedef struct _skip_cac_para {
	/** subcmd */
	t_u32 subcmd;
	/** Set */
	t_u32 action;
	/** enable/disable skip cac*/
	t_u16 skip_cac;
	/** channel */
	t_u8 channel;
	/** bandwidth */
	t_u8 bw;
} skip_cac_para;

typedef struct _wacp_mode_para {
	/** Action */
	t_u32 subcmd;
	/** Action */
	t_u32 action;
	/** TLV type*/
	t_u16 type;
	/** TLV length */
	t_u16 len;
	/** wacp_mode */
	t_u8 wacp_mode;
} wacp_mode_para;

/** radio control command */
#define UAP_RADIO_CTL (SIOCDEVPRIVATE + 5)

/** Private command ID to BSS config */
#define UAP_BSS_CONFIG (SIOCDEVPRIVATE + 6)

/** deauth station */
#define UAP_STA_DEAUTH (SIOCDEVPRIVATE + 7)

/** enable UAP report mic error */
#define UAP_REPORT_MIC_ERR (SIOCDEVPRIVATE + 8)
/** uap set key */
#define UAP_SET_KEY (SIOCDEVPRIVATE + 9)
/** encrypt key */
typedef struct _encrypt_key {
	/** Key index */
	t_u32 key_index;
	/** Key length */
	t_u32 key_len;
	/** Key */
	t_u8 key_material[MLAN_MAX_KEY_LENGTH];
	/** mac address */
	t_u8 mac_addr[MLAN_MAC_ADDR_LENGTH];
} encrypt_key;

/** pkt_header */
typedef struct _pkt_header {
	/** pkt_len */
	u32 pkt_len;
	/** pkt_type */
	u32 TxPktType;
	/** tx control */
	u32 TxControl;
} pkt_header;
/** uap get station list */
#define UAP_GET_STA_LIST (SIOCDEVPRIVATE + 11)
/** Packet inject command ioctl number */
#define UAPHOSTPKTINJECT WOAL_MGMT_FRAME_TX_IOCTL

/** Private command ID to set/get custom IE buffer */
#define UAP_CUSTOM_IE (SIOCDEVPRIVATE + 13)

/** HS WAKE UP event id */
#define UAP_EVENT_ID_HS_WAKEUP 0x80000001
/** HS_ACTIVATED event id */
#define UAP_EVENT_ID_DRV_HS_ACTIVATED 0x80000002
/** HS DEACTIVATED event id */
#define UAP_EVENT_ID_DRV_HS_DEACTIVATED 0x80000003

/** Host sleep flag set */
#define HS_CFG_FLAG_GET 0
/** Host sleep flag get */
#define HS_CFG_FLAG_SET 1
/** Host sleep flag for condition */
#define HS_CFG_FLAG_CONDITION 2
/** Host sleep flag for GPIO */
#define HS_CFG_FLAG_GPIO 4
/** Host sleep flag for Gap */
#define HS_CFG_FLAG_GAP 8
/** Host sleep flag for all */
#define HS_CFG_FLAG_ALL 0x0f
/** Host sleep mask to get condition */
#define HS_CFG_CONDITION_MASK 0x0f

/** ds_hs_cfg */
typedef struct _ds_hs_cfg {
	/** subcmd */
	t_u32 subcmd;
	/** Bit0: 0 - Get, 1 Set
	 *  Bit1: 1 - conditions is valid
	 *  Bit2: 2 - gpio is valid
	 *  Bit3: 3 - gap is valid
	 */
	t_u32 flags;
	/** Host sleep config condition */
	/** Bit0: non-unicast data
	 *  Bit1: unicast data
	 *  Bit2: mac events
	 *  Bit3: magic packet
	 */
	t_u32 conditions;
	/** GPIO */
	t_u32 gpio;
	/** Gap in milliseconds */
	t_u32 gap;
} ds_hs_cfg;

/** Private command ID to get BSS type */
#define UAP_GET_BSS_TYPE (SIOCDEVPRIVATE + 15)

/** addba_param */
typedef struct _uap_addba_param {
	/** subcmd */
	t_u32 subcmd;
	/** Set/Get */
	t_u32 action;
	/** block ack timeout for ADDBA request */
	t_u32 timeout;
	/** Buffer size for ADDBA request */
	t_u32 txwinsize;
	/** Buffer size for ADDBA response */
	t_u32 rxwinsize;
	/** amsdu for ADDBA request */
	t_u8 txamsdu;
	/** amsdu for ADDBA response */
	t_u8 rxamsdu;
} uap_addba_param;

/** aggr_prio_tbl */
typedef struct _uap_aggr_prio_tbl {
	/** subcmd */
	t_u32 subcmd;
	/** Set/Get */
	t_u32 action;
	/** ampdu priority table */
	t_u8 ampdu[MAX_NUM_TID];
	/** amsdu priority table */
	t_u8 amsdu[MAX_NUM_TID];
} uap_aggr_prio_tbl;

/** addba_reject parameters */
typedef struct _addba_reject_para {
	/** subcmd */
	t_u32 subcmd;
	/** Set/Get */
	t_u32 action;
	/** BA Reject paramters */
	t_u8 addba_reject[MAX_NUM_TID];
} addba_reject_para;

/** fw_info */
typedef struct _uap_fw_info {
	/** subcmd */
	t_u32 subcmd;
	/** Get */
	t_u32 action;
	/** Firmware release number */
	t_u32 fw_release_number;
	/** Device support for MIMO abstraction of MCSs */
	t_u8 hw_dev_mcs_support;
	/** fw_bands*/
	t_u8 fw_bands;
	/** Region Code */
	t_u16 region_code;
	/** 802.11n device capabilities */
	t_u32 hw_dot_11n_dev_cap;
} uap_fw_info;

typedef struct _ht_tx_cfg_para_hdr {
	/** Sub command */
	t_u32 subcmd;
	/** Action: Set/Get */
	t_u32 action;
} ht_tx_cfg_para_hdr;

typedef struct _tx_bf_cfg_para_hdr {
	/** Sub command */
	t_u32 subcmd;
	/** Action: Set/Get */
	t_u32 action;
} tx_bf_cfg_para_hdr;

typedef struct _vht_cfg_para_hdr {
	/** Sub command */
	t_u32 subcmd;
	/** Action: Set/Get */
	t_u32 action;
} vht_cfg_para_hdr;

typedef struct _uap_oper_para_hdr {
	/** Sub command */
	t_u32 subcmd;
	/** Action: Set/Get */
	t_u32 action;
} uap_oper_para_hdr;

#ifdef SDIO
/** sdcmd52rw parameters */
typedef struct _sdcmd52_para {
	/** subcmd */
	t_u32 subcmd;
	/** Write /Read */
	t_u32 action;
	/** Command 52 paramters */
	t_u8 cmd52_params[3];
} sdcmd52_para;
#endif

/** deep_sleep parameters */
typedef struct _deep_sleep_para {
	/** subcmd */
	t_u32 subcmd;
	/** Set/Get */
	t_u32 action;
	/** enable/disable deepsleep*/
	t_u16 deep_sleep;
	/** idle_time */
	t_u16 idle_time;
} deep_sleep_para;

/** band_steering parameters */
typedef struct _band_steer_para {
	/** subcmd */
	t_u32 subcmd;
	/** Set/Get */
	t_u8 action;
	/** enable/disable band steering*/
	t_u8 state;
	/** Probe Response will be blocked to 2G channel for first
	 * block_2g_prb_req probe requests*/
	t_u8 block_2g_prb_req;
	/** When band steering is enabled, limit the btm request sent to STA at
	 * <max_btm_req_allowed>*/
	t_u8 max_btm_req_allowed;

} band_steer_para;

/** beacon stuck detect mechanism parameters */
typedef struct _beacon_stuck_detect_para {
	/** subcmd */
	t_u32 subcmd;
	/** Set/Get */
	t_u8 action;
	/** No of beacon interval after which firmware will check if beacon Tx
	 * is going fine */
	t_u8 beacon_stuck_detect_count;
	/** Upon performing MAC reset, no of beacon interval after which
	 * firmware will check if recovery was successful */
	t_u8 recovery_confirm_count;
} beacon_stuck_detect_para;

/** tx_data_pause parameters */
typedef struct _tx_data_pause_para {
	/** subcmd */
	t_u32 subcmd;
	/** Set/Get */
	t_u32 action;
	/** enable/disable Tx data pause*/
	t_u16 txpause;
	/** Max number of TX buffer allowed for all PS client*/
	t_u16 txbufcnt;
} tx_data_pause_para;

/** mgmt_frame_ctrl */
typedef struct _mgmt_frame_ctrl {
	/** subcmd */
	t_u32 subcmd;
	/** Set/Get */
	t_u32 action;
	/** mask */
	t_u32 mask;
} mgmt_frame_ctrl;

typedef struct _snmp_mib_para {
	/** subcmd */
	t_u32 subcmd;
	/** Set/Get */
	t_u32 action;
	/** oid to set/get */
	t_u16 oid;
	/** length of oid value */
	t_u16 oid_val_len;
	/** oid value to set/get */
	t_u8 oid_value[];
} snmp_mib_para;

/** Max length for oid_value field */
#define MAX_SNMP_VALUE_SIZE 128

/** Oid for 802.11D enable/disable */
#define OID_80211D_ENABLE 0x0009
/** Oid for 802.11H enable/disable */
#define OID_80211H_ENABLE 0x000a

int woal_uap_11h_ctrl(moal_private *priv, t_u32 enable);

/** dfs_testing parameters */
typedef struct _dfs_testing_param {
	/** subcmd */
	t_u32 subcmd;
	/** Set/Get */
	t_u32 action;
	/** user CAC period (msec) */
	t_u32 usr_cac_period;
	/** user NOP period (sec) */
	t_u16 usr_nop_period;
	/** don't change channel on radar */
	t_u8 no_chan_change;
	/** fixed channel to change to on radar */
	t_u8 fixed_new_chan;
	/** CAC restart */
	t_u8 cac_restart;
} dfs_testing_para;

/** Channel switch count config */
typedef struct _cscount_cfg_t {
	/** subcmd */
	t_u32 subcmd;
	/** Set/Get */
	t_u32 action;
	/** user channel switch count */
	t_u8 cs_count;
} cscount_cfg_t;

/** domain_info parameters */
typedef struct _domain_info_param {
	/** subcmd */
	t_u32 subcmd;
	/** Set/Get */
	t_u32 action;
	/** domain_param TLV (incl. header) */
	t_u8 tlv[];
} domain_info_para;

/** DOMAIN_INFO param sizes */
#define TLV_HEADER_LEN (2 + 2)
#define SUB_BAND_LEN 3
#define MAX_SUB_BANDS 40

/** MAX domain TLV length */
#define MAX_DOMAIN_TLV_LEN                                                     \
	(TLV_HEADER_LEN + COUNTRY_CODE_LEN + (SUB_BAND_LEN * MAX_SUB_BANDS))

/** DOMAIN_INFO param size of dfs_region */
#define DFS_REGION_LEN 1
/** MAX reg domain TLV length*/
#define MAX_REG_DOMAIN_TLV_LEN (TLV_HEADER_LEN + DFS_REGION_LEN)

/** Get/Set channel DFS state */
int woal_11h_chan_dfs_state(moal_private *priv, t_u8 action,
			    mlan_ds_11h_chan_dfs_state *ch_dfs_state);
#ifdef UAP_CFG80211
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 14, 0)
void woal_update_channels_dfs_state(moal_private *priv, t_u8 channel,
				    t_u8 bandwidth, t_u8 dfs_state);
void woal_update_uap_channel_dfs_state(moal_private *priv);
#endif
#endif

mlan_status woal_set_get_uap_power_mode(moal_private *priv, t_u32 action,
					mlan_ds_ps_mgmt *ps_mgmt);
void woal_uap_set_multicast_list(struct net_device *dev);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0)
int woal_uap_do_ioctl(struct net_device *dev, struct ifreq *req,
		      void __user *data, int cmd);
#else
int woal_uap_do_ioctl(struct net_device *dev, struct ifreq *req, int cmd);
#endif

int woal_uap_bss_ctrl(moal_private *priv, t_u8 wait_option, int data);

int woal_uap_get_channel_nop_info(moal_private *priv, t_u8 wait_option,
				  pmlan_ds_11h_chan_nop_info ch_info);

mlan_status woal_set_get_ap_channel(moal_private *priv, t_u16 action,
				    t_u8 wait_option,
				    chan_band_info *uap_channel);
#ifdef CONFIG_PROC_FS
void woal_uap_get_version(moal_private *priv, char *version, int max_len);
#endif
mlan_status woal_uap_get_stats(moal_private *priv, t_u8 wait_option,
			       pmlan_ds_uap_stats ustats);
#if defined(UAP_WEXT) || defined(UAP_CFG80211)
extern struct iw_handler_def woal_uap_handler_def;
struct iw_statistics *woal_get_uap_wireless_stats(struct net_device *dev);
/** IOCTL function for wireless private IOCTLs */
int woal_uap_do_priv_ioctl(struct net_device *dev, struct ifreq *req, int cmd);
#endif
/** Set invalid data for each member of mlan_uap_bss_param */
void woal_set_sys_config_invalid_data(pmlan_uap_bss_param config);
/** Set/Get system configuration parameters */
mlan_status woal_set_get_sys_config(moal_private *priv, t_u16 action,
				    t_u8 wait_option,
				    mlan_uap_bss_param *sys_cfg);
/** Set get AP wmm parameter */
mlan_status woal_set_get_ap_wmm_para(moal_private *priv, t_u16 action,
				     wmm_parameter_t *ap_wmm_para);
int woal_uap_set_ap_cfg(moal_private *priv, t_u8 *data, int len);

#if defined(UAP_CFG80211)
#if defined(STA_WEXT) || defined(UAP_WEXT)
int woal_uap_set_get_multi_ap_mode(moal_private *priv, struct iwreq *wrq);
#endif
#endif

int woal_uap_set_11ac_status(moal_private *priv, t_u8 action, t_u8 band,
			     t_u8 vht20_40,
			     const IEEEtypes_VHTCap_t *vhtcap_ie);
int woal_11ax_cfg(moal_private *priv, t_u8 action, mlan_ds_11ax_he_cfg *he_cfg,
		  t_u8 wait_option);
int woal_uap_set_11ax_status(moal_private *priv, t_u8 action, t_u8 band,
			     const IEEEtypes_HECap_t *hecap_ie);
int woal_set_uap_ht_tx_cfg(moal_private *priv, Band_Config_t bandcfg,
			   t_u16 ht_cap, t_u8 en);
mlan_status woal_uap_set_11n_status(moal_private *priv,
				    mlan_uap_bss_param *sys_cfg, t_u8 action);
#ifdef UAP_WEXT
void woal_ioctl_get_uap_info_resp(moal_private *priv, pmlan_ds_get_info info);
int woal_set_get_custom_ie(moal_private *priv, t_u16 mask, t_u8 *ie,
			   int ie_len);
#endif /* UAP_WEXT */

#endif /* _MOAL_UAP_H */
