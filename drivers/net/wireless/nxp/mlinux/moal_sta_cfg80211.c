/** @file moal_sta_cfg80211.c
 *
 * @brief This file contains the functions for STA CFG80211.
 *
 *
 * Copyright 2011-2025 NXP
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

#include "moal_cfg80211.h"
#include "moal_cfg80211_util.h"
#include "moal_sta_cfg80211.h"
#include "moal_eth_ioctl.h"
#ifdef UAP_SUPPORT
#include "moal_uap_cfg80211.h"
#endif
#include <linux/sort.h>

#if defined(STA_CFG80211) || defined(UAP_CFG80211)
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 14, 0)
extern int fw_region;
#endif
#endif
/* Supported crypto cipher suits to be advertised to cfg80211 */
static const u32 cfg80211_cipher_suites[] = {
	WLAN_CIPHER_SUITE_WEP40,	WLAN_CIPHER_SUITE_WEP104,
	WLAN_CIPHER_SUITE_TKIP,		WLAN_CIPHER_SUITE_CCMP,
	WLAN_CIPHER_SUITE_SMS4,		WLAN_CIPHER_SUITE_AES_CMAC,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 0, 0)
	WLAN_CIPHER_SUITE_BIP_GMAC_128, WLAN_CIPHER_SUITE_BIP_GMAC_256,
#endif
	WLAN_CIPHER_SUITE_FILS_PSK,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 6, 0)
	WLAN_CIPHER_SUITE_GCMP,
#endif
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 0, 0)
	WLAN_CIPHER_SUITE_GCMP_256,
#endif
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 0, 0)
	WLAN_CIPHER_SUITE_CCMP_256,
#endif
};

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 1, 0)
static const u32 cfg80211_akm_suites[] = {
	WLAN_AKM_SUITE_8021X,
	WLAN_AKM_SUITE_PSK,
	WLAN_AKM_SUITE_SAE,
	WLAN_AKM_SUITE_FT_OVER_SAE,
	WLAN_AKM_SUITE_8021X_SHA256,
	WLAN_AKM_SUITE_PSK_SHA256,
	WLAN_AKM_SUITE_TDLS,
	WLAN_AKM_SUITE_FT_8021X,
	WLAN_AKM_SUITE_FT_PSK,
	WLAN_AKM_SUITE_8021X_SUITE_B,
	WLAN_AKM_SUITE_8021X_SUITE_B_192,
	WLAN_AKM_SUITE_FILS_SHA256,
	WLAN_AKM_SUITE_FILS_SHA384,
	WLAN_AKM_SUITE_FT_FILS_SHA256,
	WLAN_AKM_SUITE_FT_FILS_SHA384,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 7, 0)
	WLAN_AKM_SUITE_OWE,
#endif
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 8, 0)
	WLAN_AKM_SUITE_FT_PSK_SHA384,
	WLAN_AKM_SUITE_PSK_SHA384,
	WLAN_AKM_SUITE_FILS_SHA384,
	WLAN_AKM_SUITE_FT_FILS_SHA384,
#endif
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 11, 0)
	WLAN_AKM_SUITE_WFA_DPP,
#endif
};
#endif

#define AP_MODE_IND 0
#define AP_MODE_SP 1
#define AP_MODE_VLP 2

#define HE_OPER_CTRL_MASK 0x38

/**
 * @brief Band: 6G Region: US STA-Mode-PSD Table
 */
mode_psd_t mode_psd_sta_FCC_6G[] = {
	{"indoor_", "minus1"},
	{"sp_", ""},
	{"vlp_", "minus7"},
};

/**
 * @brief Band: 6G, Region: EU STA-Mode-PSD Table
 */
mode_psd_t mode_psd_sta_EU_6G[] = {
	{"indoor_", "minus1"},
	{"sp_", ""},
	{"vlp_", "minus10"},
};

/**
 * @brief The 6GHz STA Region-Mode-PSD Table
 */
rmp_table_t rmp_table_sta_6G[] = {
	{
		0x10, /* FCC region */
		mode_psd_sta_FCC_6G,
	},
	{
		0x30, /* ETSI region */
		mode_psd_sta_EU_6G,
	},
};

#ifdef UAP_SUPPORT
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
static int woal_cfg80211_set_monitor_channel(struct wiphy *wiphy,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(6, 13, 0)
					     struct net_device *dev,
#endif
					     struct cfg80211_chan_def *chandef);
#endif
#endif
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 9, 0)
static void
#else
static int
#endif
woal_cfg80211_reg_notifier(struct wiphy *wiphy,
			   struct regulatory_request *request);

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 6, 0)
static int woal_cfg80211_scan(struct wiphy *wiphy,
			      struct cfg80211_scan_request *request);
#else
static int woal_cfg80211_scan(struct wiphy *wiphy, struct net_device *dev,
			      struct cfg80211_scan_request *request);
#endif
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 5, 0)
static void woal_cfg80211_abort_scan(struct wiphy *wiphy,
				     struct wireless_dev *wdev);
#endif
static int woal_cfg80211_connect(struct wiphy *wiphy, struct net_device *dev,
				 struct cfg80211_connect_params *sme);

#if CFG80211_VERSION_CODE > KERNEL_VERSION(4, 12, 14)
static int woal_cfg80211_set_psk(moal_private *priv,
				 struct cfg80211_connect_params *sme);
#endif
static int woal_cfg80211_disconnect(struct wiphy *wiphy, struct net_device *dev,
				    t_u16 reason_code);

static int woal_cfg80211_get_station(struct wiphy *wiphy,
				     struct net_device *dev,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 16, 0)
				     const u8 *mac,
#else
				     u8 *mac,
#endif
				     struct station_info *sinfo);

static int woal_cfg80211_dump_station(struct wiphy *wiphy,
				      struct net_device *dev, int idx,
				      t_u8 *mac, struct station_info *sinfo);

static int woal_cfg80211_dump_survey(struct wiphy *wiphy,
				     struct net_device *dev, int idx,
				     struct survey_info *survey);
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
static int woal_cfg80211_get_channel(struct wiphy *wiphy,
				     struct wireless_dev *wdev,
#if ((CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 19, 2)) ||                    \
     (defined(ANDROID_SDK_VERSION) && ANDROID_SDK_VERSION >= 31))
				     unsigned int link_id,
#endif
				     struct cfg80211_chan_def *chandef);
#endif
static int woal_cfg80211_set_power_mgmt(struct wiphy *wiphy,
					struct net_device *dev, bool enabled,
					int timeout);
#if CFG80211_VERSION_CODE > KERNEL_VERSION(2, 6, 35)
static int woal_cfg80211_set_cqm_rssi_config(struct wiphy *wiphy,
					     struct net_device *dev,
					     s32 rssi_thold, u32 rssi_hyst);
#endif

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(2, 6, 31)
static int woal_cfg80211_get_tx_power(struct wiphy *wiphy,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
				      struct wireless_dev *wdev,
#endif
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(6, 14, 0)
				      unsigned int link_id,
#endif
				      int *dbm);

static int woal_cfg80211_set_tx_power(struct wiphy *wiphy,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
				      struct wireless_dev *wdev,
#endif
#if CFG80211_VERSION_CODE < KERNEL_VERSION(2, 6, 36)
				      enum tx_power_setting type,
#else
				      enum nl80211_tx_power_setting type,
#endif
				      int mbm);
#endif

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(2, 6, 39)
static int woal_cfg80211_mgmt_tx_cancel_wait(struct wiphy *wiphy,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 6, 0)
					     struct wireless_dev *wdev,
#else
					     struct net_device *dev,
#endif
					     u64 cookie);

static int
woal_cfg80211_remain_on_channel(struct wiphy *wiphy,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 6, 0)
				struct wireless_dev *wdev,
#else
				struct net_device *dev,
#endif
				struct ieee80211_channel *chan,
#if CFG80211_VERSION_CODE < KERNEL_VERSION(3, 8, 0)
				enum nl80211_channel_type channel_type,
#endif
				unsigned int duration, u64 *cookie);

static int woal_cfg80211_cancel_remain_on_channel(struct wiphy *wiphy,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 6, 0)
						  struct wireless_dev *wdev,
#else
						  struct net_device *dev,
#endif
						  u64 cookie);
#endif /* KERNEL_VERSION */

#ifdef CONFIG_NL80211_TESTMODE
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 12, 0)
static int woal_testmode_cmd(struct wiphy *wiphy, struct wireless_dev *wdev,
			     void *data, int len);
#else
static int woal_testmode_cmd(struct wiphy *wiphy, void *data, int len);
#endif
#endif

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 2, 0)
int woal_cfg80211_sched_scan_start(struct wiphy *wiphy, struct net_device *dev,
				   struct cfg80211_sched_scan_request *request);
int woal_cfg80211_sched_scan_stop(struct wiphy *wiphy, struct net_device *dev
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 12, 0)
				  ,
				  u64 reqid
#endif
);
#endif

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 2, 0)
int woal_cfg80211_resume(struct wiphy *wiphy);
int woal_cfg80211_suspend(struct wiphy *wiphy, struct cfg80211_wowlan *wow);
#endif
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 5, 0)
static void woal_cfg80211_set_wakeup(struct wiphy *wiphy, bool enabled);
#endif

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 2, 0)
void woal_check_auto_tdls(struct wiphy *wiphy, struct net_device *dev);
int woal_cfg80211_tdls_oper(struct wiphy *wiphy, struct net_device *dev,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 16, 0)
			    const u8 *peer,
#else
			    u8 *peer,
#endif
			    enum nl80211_tdls_operation oper);
int woal_cfg80211_tdls_mgmt(struct wiphy *wiphy, struct net_device *dev,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 16, 0)
			    const u8 *peer,
#else
			    u8 *peer,
#endif
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(6, 5, 0)
			    int link_id,
#endif
			    u8 action_code, u8 dialog_token, u16 status_code,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 15, 0)
			    u32 peer_capability,
#endif
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 17, 0)
			    bool initiator,
#endif
			    const u8 *extra_ies, size_t extra_ies_len);
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 19, 0)
static int woal_cfg80211_tdls_channel_switch(struct wiphy *wiphy,
					     struct net_device *dev,
					     const u8 *addr, u8 oper_class,
					     struct cfg80211_chan_def *chandef);

void woal_cfg80211_tdls_cancel_channel_switch(struct wiphy *wiphy,
					      struct net_device *dev,
					      const u8 *addr);
#endif
#endif
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 2, 0)
static int woal_cfg80211_change_station(struct wiphy *wiphy,
					struct net_device *dev,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 16, 0)
					const u8 *mac,
#else
					u8 *mac,
#endif
					struct station_parameters *params);
#endif
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
int woal_cfg80211_update_ft_ies(struct wiphy *wiphy, struct net_device *dev,
				struct cfg80211_update_ft_ies_params *ftie);
#endif
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
static mlan_status woal_get_common_rates(struct net_device *dev,
					 struct cfg80211_auth_request *req,
					 t_u32 *tx_control);

static int woal_cfg80211_authenticate(struct wiphy *wiphy,
				      struct net_device *dev,
				      struct cfg80211_auth_request *req);

static int woal_cfg80211_associate(struct wiphy *wiphy, struct net_device *dev,
				   struct cfg80211_assoc_request *req);
#ifdef UAP_SUPPORT
int woal_cfg80211_uap_add_station(struct wiphy *wiphy, struct net_device *dev,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 16, 0)
				  const u8 *mac,
#else
				  u8 *mac,
#endif
				  struct station_parameters *params);
#endif
#endif

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 2, 0)
#ifdef UAP_SUPPORT
static int woal_cfg80211_add_station(struct wiphy *wiphy,
				     struct net_device *dev,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 16, 0)
				     const u8 *mac,
#else
				     u8 *mac,
#endif
				     struct station_parameters *params);
#endif
#endif

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 4, 0)
static int woal_cfg80211_deauthenticate(struct wiphy *wiphy,
					struct net_device *dev,
					struct cfg80211_deauth_request *req);

static int woal_cfg80211_disassociate(struct wiphy *wiphy,
				      struct net_device *dev,
				      struct cfg80211_disassoc_request *req);
#endif

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
static int woal_cfg80211_add_tx_ts(struct wiphy *wiphy, struct net_device *dev,
				   u8 tsid, const u8 *peer, u8 user_prio,
				   u16 admitted_time);
static int woal_cfg80211_del_tx_ts(struct wiphy *wiphy, struct net_device *dev,
				   u8 tsid, const u8 *peer);
#endif /* KERNEL_VERSION(3, 8, 0) */

static mlan_status woal_cfg80211_dump_station_info(moal_private *priv,
						   struct station_info *sinfo);

/** cfg80211 operations */
static struct cfg80211_ops woal_cfg80211_ops = {
	.change_virtual_intf = woal_cfg80211_change_virtual_intf,
	.scan = woal_cfg80211_scan,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 5, 0)
	.abort_scan = woal_cfg80211_abort_scan,
#endif
	.connect = woal_cfg80211_connect,
	.disconnect = woal_cfg80211_disconnect,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 4, 0)
	.deauth = woal_cfg80211_deauthenticate,
	.disassoc = woal_cfg80211_disassociate,
#endif
	.get_station = woal_cfg80211_get_station,
	.dump_station = woal_cfg80211_dump_station,
	.dump_survey = woal_cfg80211_dump_survey,
	.set_wiphy_params = woal_cfg80211_set_wiphy_params,
#if CFG80211_VERSION_CODE < KERNEL_VERSION(3, 5, 0)
	.set_channel = woal_cfg80211_set_channel,
#endif
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
	.get_channel = woal_cfg80211_get_channel,
#endif
	.add_key = woal_cfg80211_add_key,
	.del_key = woal_cfg80211_del_key,
	.set_default_key = woal_cfg80211_set_default_key,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(2, 6, 30)
	.set_default_mgmt_key = woal_cfg80211_set_default_mgmt_key,
#endif
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 10, 0)
	.set_default_beacon_key = woal_cfg80211_set_default_beacon_key,
#endif

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 1, 0)
	.set_rekey_data = woal_cfg80211_set_rekey_data,
#endif
	.set_pmksa = woal_cfg80211_set_pmksa,
	.del_pmksa = woal_cfg80211_del_pmksa,
	.flush_pmksa = woal_cfg80211_flush_pmksa,
	.set_power_mgmt = woal_cfg80211_set_power_mgmt,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(2, 6, 31)
	.set_tx_power = woal_cfg80211_set_tx_power,
	.get_tx_power = woal_cfg80211_get_tx_power,
#endif
	.set_bitrate_mask = woal_cfg80211_set_bitrate_mask,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 2, 0)
	.sched_scan_start = woal_cfg80211_sched_scan_start,
	.sched_scan_stop = woal_cfg80211_sched_scan_stop,
#endif
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 2, 0)
	.suspend = woal_cfg80211_suspend,
	.resume = woal_cfg80211_resume,
#endif
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 5, 0)
	.set_wakeup = woal_cfg80211_set_wakeup,
#endif
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(2, 6, 38)
	.set_antenna = woal_cfg80211_set_antenna,
	.get_antenna = woal_cfg80211_get_antenna,
#endif
#if CFG80211_VERSION_CODE > KERNEL_VERSION(2, 6, 35)
	.set_cqm_rssi_config = woal_cfg80211_set_cqm_rssi_config,
#endif
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 2, 0)
	.tdls_oper = woal_cfg80211_tdls_oper,
	.tdls_mgmt = woal_cfg80211_tdls_mgmt,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 19, 0)
	.tdls_channel_switch = woal_cfg80211_tdls_channel_switch,
	.tdls_cancel_channel_switch = woal_cfg80211_tdls_cancel_channel_switch,
#endif
#endif
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 2, 0)
	.change_station = woal_cfg80211_change_station,
#endif
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
	.update_ft_ies = woal_cfg80211_update_ft_ies,
#endif
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 14, 0)
	.set_qos_map = woal_cfg80211_set_qos_map,
#endif
#ifdef UAP_CFG80211
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 12, 0)
	.set_coalesce = woal_cfg80211_set_coalesce,
#endif
	.add_virtual_intf = woal_cfg80211_add_virtual_intf,
	.del_virtual_intf = woal_cfg80211_del_virtual_intf,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 4, 0)
	.start_ap = woal_cfg80211_add_beacon,
	.change_beacon = woal_cfg80211_set_beacon,
	.stop_ap = woal_cfg80211_del_beacon,
#else
	.add_beacon = woal_cfg80211_add_beacon,
	.set_beacon = woal_cfg80211_set_beacon,
	.del_beacon = woal_cfg80211_del_beacon,
#endif
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
	.change_bss = woal_cfg80211_change_bss,
#endif
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 2, 0)
	.add_station = woal_cfg80211_add_station,
#endif
	.del_station = woal_cfg80211_del_station,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 1, 0)
	.set_txq_params = woal_cfg80211_set_txq_params,
#endif
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 9, 0)
	.set_mac_acl = woal_cfg80211_set_mac_acl,
#endif
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 12, 0)
	.start_radar_detection = woal_cfg80211_start_radar_detection,

	.channel_switch = woal_cfg80211_channel_switch,
#endif
#endif
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(2, 6, 37)
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 8, 0)
	.update_mgmt_frame_registrations = woal_cfg80211_mgmt_frame_register,
#else
	.mgmt_frame_register = woal_cfg80211_mgmt_frame_register,
#endif
	.mgmt_tx = woal_cfg80211_mgmt_tx,
#endif

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(2, 6, 39)
	.mgmt_tx_cancel_wait = woal_cfg80211_mgmt_tx_cancel_wait,
	.remain_on_channel = woal_cfg80211_remain_on_channel,
	.cancel_remain_on_channel = woal_cfg80211_cancel_remain_on_channel,
#endif

#ifdef UAP_SUPPORT
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
	.set_monitor_channel = woal_cfg80211_set_monitor_channel,
#endif
#endif
#ifdef CONFIG_NL80211_TESTMODE
	.testmode_cmd = woal_testmode_cmd,
#endif
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
	.add_tx_ts = woal_cfg80211_add_tx_ts,
	.del_tx_ts = woal_cfg80211_del_tx_ts,
#endif /* KERNEL_VERSION(3, 8, 0) */

};

/** Region code mapping */
typedef struct _region_code_t {
	/** Region */
	t_u8 region[COUNTRY_CODE_LEN];
} region_code_t;

static const struct ieee80211_regdomain mrvl_regdom = {
	.n_reg_rules = 4,
	.alpha2 = "99",
	.reg_rules = {
		/* IEEE 802.11b/g, channels 1..11 */
		REG_RULE(2412 - 10, 2472 + 10, 40, 6, 20, 0),
		/* If any */
		/* IEEE 802.11 channel 14 - Only JP enables
		 * this and for 802.11b only
		 */
		REG_RULE(2484 - 10, 2484 + 10, 20, 6, 20, 0),
		/* IEEE 802.11a, channel 36..64 */
		REG_RULE(5150 - 10, 5350 + 10, 80, 6, 20, 0),
		/* IEEE 802.11a, channel 100..165 */
		REG_RULE(5470 - 10, 5850 + 10, 80, 6, 20, 0),
	}};

/********************************************************
				Local Variables
********************************************************/
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(2, 6, 37)
// clang-format off
static const struct ieee80211_txrx_stypes
	ieee80211_mgmt_stypes[NUM_NL80211_IFTYPES] = {
		[NL80211_IFTYPE_STATION] = {
			.tx = MBIT(IEEE80211_STYPE_ACTION >> 4) |
			      MBIT(IEEE80211_STYPE_PROBE_RESP >> 4)|
			      MBIT(IEEE80211_STYPE_AUTH >> 4)|
			      MBIT(IEEE80211_STYPE_DEAUTH >> 4),
			.rx = MBIT(IEEE80211_STYPE_ACTION >> 4) |
			      MBIT(IEEE80211_STYPE_PROBE_REQ >> 4)|
			      MBIT(IEEE80211_STYPE_AUTH >> 4),
		},
		[NL80211_IFTYPE_AP] = {
			.tx = 0xffff,
			.rx = MBIT(IEEE80211_STYPE_ASSOC_REQ >> 4) |
			      MBIT(IEEE80211_STYPE_REASSOC_REQ >> 4) |
			      MBIT(IEEE80211_STYPE_PROBE_REQ >> 4) |
			      MBIT(IEEE80211_STYPE_DISASSOC >> 4) |
			      MBIT(IEEE80211_STYPE_AUTH >> 4) |
			      MBIT(IEEE80211_STYPE_DEAUTH >> 4) |
			      MBIT(IEEE80211_STYPE_ACTION >> 4),
		},
		[NL80211_IFTYPE_AP_VLAN] = {
			.tx = 0x0000,
			.rx = 0x0000,
		},
#ifdef WIFI_DIRECT_SUPPORT
#if CFG80211_VERSION_CODE >= WIFI_DIRECT_KERNEL_VERSION
		[NL80211_IFTYPE_P2P_CLIENT] = {
			.tx = MBIT(IEEE80211_STYPE_ACTION >> 4) |
			      MBIT(IEEE80211_STYPE_PROBE_RESP >> 4),
			.rx = MBIT(IEEE80211_STYPE_ACTION >> 4) |
			      MBIT(IEEE80211_STYPE_PROBE_REQ >> 4),
		},
		[NL80211_IFTYPE_P2P_GO] = {
			.tx = MBIT(IEEE80211_STYPE_ACTION >> 4) |
			      MBIT(IEEE80211_STYPE_AUTH >> 4) |
			      MBIT(IEEE80211_STYPE_ASSOC_RESP >> 4) |
			      MBIT(IEEE80211_STYPE_REASSOC_RESP >> 4) |
			      MBIT(IEEE80211_STYPE_PROBE_RESP >> 4),
			.rx = MBIT(IEEE80211_STYPE_ASSOC_REQ >> 4) |
			      MBIT(IEEE80211_STYPE_REASSOC_REQ >> 4) |
			      MBIT(IEEE80211_STYPE_PROBE_REQ >> 4) |
			      MBIT(IEEE80211_STYPE_DISASSOC >> 4) |
			      MBIT(IEEE80211_STYPE_AUTH >> 4) |
			      MBIT(IEEE80211_STYPE_DEAUTH >> 4) |
			      MBIT(IEEE80211_STYPE_ACTION >> 4),
		},
#endif
#endif
		[NL80211_IFTYPE_MESH_POINT] = {
			.tx = 0x0000,
			.rx = 0x0000,
		},

};
// clang-format on
#endif

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 0, 0)
/**
 * NOTE: types in all the sets must be equals to the
 * initial value of wiphy->interface_modes
 */
static const struct ieee80211_iface_limit cfg80211_ap_sta_limits[] = {
	{.max = 4,
	 .types = MBIT(NL80211_IFTYPE_STATION)
#ifdef UAP_CFG80211
		  | MBIT(NL80211_IFTYPE_AP) | MBIT(NL80211_IFTYPE_MONITOR)
#endif
#ifdef WIFI_DIRECT_SUPPORT
#if CFG80211_VERSION_CODE >= WIFI_DIRECT_KERNEL_VERSION
		  | MBIT(NL80211_IFTYPE_P2P_GO) |
		  MBIT(NL80211_IFTYPE_P2P_CLIENT)
#endif
#endif
	}};

static struct ieee80211_iface_combination cfg80211_iface_comb_ap_sta = {
	.limits = cfg80211_ap_sta_limits,
	.num_different_channels = 1,
	.n_limits = ARRAY_SIZE(cfg80211_ap_sta_limits),
	.max_interfaces = 4,
	.beacon_int_infra_match = MTRUE,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 12, 0)
	.radar_detect_widths =
		MBIT(NL80211_CHAN_WIDTH_20_NOHT) | MBIT(NL80211_CHAN_WIDTH_20),
#endif
};
#endif

extern pmoal_handle m_handle[];

#ifdef CONFIG_PM
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
static const struct wiphy_wowlan_support wowlan_support = {
	.flags = WIPHY_WOWLAN_ANY | WIPHY_WOWLAN_MAGIC_PKT,
	.n_patterns = MAX_NUM_FILTERS,
	.pattern_min_len = 1,
	.pattern_max_len = WOWLAN_MAX_PATTERN_LEN,
	.max_pkt_offset = WOWLAN_MAX_OFFSET_LEN,
};
static const struct wiphy_wowlan_support wowlan_support_with_gtk = {
	.flags = WIPHY_WOWLAN_ANY | WIPHY_WOWLAN_MAGIC_PKT |
		 WIPHY_WOWLAN_SUPPORTS_GTK_REKEY |
		 WIPHY_WOWLAN_GTK_REKEY_FAILURE,
	.n_patterns = MAX_NUM_FILTERS,
	.pattern_min_len = 1,
	.pattern_max_len = WOWLAN_MAX_PATTERN_LEN,
	.max_pkt_offset = WOWLAN_MAX_OFFSET_LEN,
};
#endif
#endif

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 12, 0)
static const struct wiphy_coalesce_support coalesce_support = {
	.n_rules = COALESCE_MAX_RULES,
	.max_delay = MAX_COALESCING_DELAY,
	.n_patterns = COALESCE_MAX_FILTERS,
	.pattern_min_len = 1,
	.pattern_max_len = MAX_PATTERN_LEN,
	.max_pkt_offset = MAX_OFFSET_LEN,
};
#endif

/********************************************************
				Global Variables
********************************************************/

/********************************************************
				Local Functions
********************************************************/
#ifdef UAP_SUPPORT
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
static int woal_cfg80211_set_monitor_channel(struct wiphy *wiphy,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(6, 13, 0)
					     struct net_device *dev,
#endif
					     struct cfg80211_chan_def *chandef)
{
	moal_handle *handle = (moal_handle *)woal_get_wiphy_priv(wiphy);
	moal_private *priv =
		(moal_private *)woal_get_priv(handle, MLAN_BSS_ROLE_STA);
	netmon_band_chan_cfg band_chan_cfg;
	t_u32 bandwidth = 0;
	int ret = -EFAULT;

	ENTER();

	if (!priv) {
		ret = -EFAULT;
		goto done;
	}
	if (handle->mon_if) {
		if (cfg80211_chandef_identical(&handle->mon_if->chandef,
					       chandef)) {
			ret = 0;
			goto done;
		}
		if (woal_is_any_interface_active(handle)) {
			PRINTM(MERROR,
			       "Cannot change monitor channel for an active"
			       " interface\n");
			goto done;
		}
		memset(&band_chan_cfg, 0x00, sizeof(band_chan_cfg));
		/* Set channel */
		band_chan_cfg.channel = ieee80211_frequency_to_channel(
			chandef->chan->center_freq);
		/* Set band */
		if (chandef->chan->band == IEEE80211_BAND_2GHZ)
			band_chan_cfg.band = BAND_2GHZ;
		else if (chandef->chan->band == IEEE80211_BAND_5GHZ)
			band_chan_cfg.band = BAND_5GHZ;
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 4, 0)
		else if (chandef->chan->band == IEEE80211_BAND_6GHZ)
			band_chan_cfg.band = BAND_6GHZ;
#endif

		/* Set bandwidth */
		if (chandef->width == NL80211_CHAN_WIDTH_20)
			bandwidth = CHANNEL_BW_20MHZ;
		else if (chandef->width == NL80211_CHAN_WIDTH_40)
			bandwidth = chandef->center_freq1 >
						    chandef->chan->center_freq ?
					    CHANNEL_BW_40MHZ_ABOVE :
					    CHANNEL_BW_40MHZ_BELOW;
		else if (chandef->width == NL80211_CHAN_WIDTH_80)
			bandwidth = CHANNEL_BW_80MHZ;
		band_chan_cfg.chan_bandwidth = bandwidth;

		if (MLAN_STATUS_SUCCESS !=
		    woal_set_net_monitor(priv, MOAL_IOCTL_WAIT, MTRUE,
					 handle->mon_if->flag,
					 &band_chan_cfg)) {
			PRINTM(MERROR, "%s: woal_set_net_monitor fail\n",
			       __func__);
			ret = -EFAULT;
			goto done;
		}

		moal_memcpy_ext(priv->phandle, &handle->mon_if->band_chan_cfg,
				&band_chan_cfg,
				sizeof(handle->mon_if->band_chan_cfg),
				sizeof(handle->mon_if->band_chan_cfg));
		handle->mon_if->chandef = *chandef;

		if (handle->mon_if->chandef.chan)
			PRINTM(MINFO,
			       "set_monitor_channel+++ chan[band=%d center_freq=%d hw_value=%d] width=%d center_freq1=%d center_freq2=%d\n",
			       handle->mon_if->chandef.chan->band,
			       handle->mon_if->chandef.chan->center_freq,
			       handle->mon_if->chandef.chan->hw_value,
			       handle->mon_if->chandef.width,
			       handle->mon_if->chandef.center_freq1,
			       handle->mon_if->chandef.center_freq2);
		PRINTM(MINFO,
		       "set_monitor_channel+++ band=%x channel=%d bandwidth=%d\n",
		       handle->mon_if->band_chan_cfg.band,
		       handle->mon_if->band_chan_cfg.channel,
		       handle->mon_if->band_chan_cfg.chan_bandwidth);
		ret = 0;
	}

done:
	LEAVE();
	return ret;
}
#endif
#endif

/**
 *  @brief This function check cfg80211 special region code.
 *
 *  @param region_string         Region string
 *
 *  @return     MTRUE/MFALSE
 */
t_u8 is_cfg80211_special_region_code(t_u8 *region_string)
{
	t_u8 i;
	region_code_t cfg80211_special_region_code[] = {
		{"00 "}, {"99 "}, {"98 "}, {"97 "}};

	for (i = 0; i < COUNTRY_CODE_LEN && region_string[i]; i++)
		region_string[i] = toupper(region_string[i]);

	for (i = 0; i < ARRAY_SIZE(cfg80211_special_region_code); i++) {
		if (!memcmp(region_string,
			    cfg80211_special_region_code[i].region,
			    COUNTRY_CODE_LEN)) {
			PRINTM(MIOCTL, "special region code=%s\n",
			       region_string);
			return MTRUE;
		}
	}
	return MFALSE;
}

/**
 * @brief Get the encryption mode from cipher
 *
 * @param cipher        Cipher cuite
 * @param wpa_enabled   WPA enable or disable
 *
 * @return              MLAN_ENCRYPTION_MODE_*
 */
static int woal_cfg80211_get_encryption_mode(t_u32 cipher, int *wpa_enabled)
{
	int encrypt_mode;

	ENTER();

	*wpa_enabled = 0;
	switch (cipher) {
	case MW_AUTH_CIPHER_NONE:
		encrypt_mode = MLAN_ENCRYPTION_MODE_NONE;
		break;
	case WLAN_CIPHER_SUITE_WEP40:
		encrypt_mode = MLAN_ENCRYPTION_MODE_WEP40;
		break;
	case WLAN_CIPHER_SUITE_WEP104:
		encrypt_mode = MLAN_ENCRYPTION_MODE_WEP104;
		break;
	case WLAN_CIPHER_SUITE_TKIP:
		encrypt_mode = MLAN_ENCRYPTION_MODE_TKIP;
		*wpa_enabled = 1;
		break;
	case WLAN_CIPHER_SUITE_CCMP:
		encrypt_mode = MLAN_ENCRYPTION_MODE_CCMP;
		*wpa_enabled = 1;
		break;
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 0, 0)
	case WLAN_CIPHER_SUITE_CCMP_256:
		encrypt_mode = MLAN_ENCRYPTION_MODE_CCMP_256;
		*wpa_enabled = 1;
		break;
#endif

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 6, 0)
	case WLAN_CIPHER_SUITE_GCMP:
		encrypt_mode = MLAN_ENCRYPTION_MODE_GCMP;
		*wpa_enabled = 1;
		break;
#endif
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 0, 0)
	case WLAN_CIPHER_SUITE_GCMP_256:
		encrypt_mode = MLAN_ENCRYPTION_MODE_GCMP_256;
		*wpa_enabled = 1;
		break;
#endif
	default:
		encrypt_mode = -1;
	}

	LEAVE();
	return encrypt_mode;
}

/**
 *  @brief get associate failure status code
 *
 *  @param priv     Pointer to the moal_private driver data struct
 *
 *  @return         IEEE status code
 */
static int woal_get_assoc_status(moal_private *priv)
{
	int ret = WLAN_STATUS_UNSPECIFIED_FAILURE;
	t_u16 status = (t_u16)(priv->assoc_status & 0xffff);
	t_u16 cap = (t_u16)(priv->assoc_status >> 16);

	switch (cap) {
	case 0xfffd:
	case 0xfffe:
		ret = status;
		break;
	case 0xfffc:
		ret = WLAN_STATUS_AUTH_TIMEOUT;
		break;
	default:
		break;
	}
	PRINTM(MCMND, "Assoc fail: status=%d, cap=0x%x, IEEE status=%d\n",
	       status, cap, ret);
	return ret;
}

/**
 *  @brief Check the pairwise or group cipher for
 *  WEP enabled or not
 *
 *  @param cipher       MLAN Cipher cuite
 *
 *  @return             1 -- enable or 0 -- disable
 */
static int woal_cfg80211_is_alg_wep(t_u32 cipher)
{
	int alg = 0;
	ENTER();

	if (cipher == MLAN_ENCRYPTION_MODE_WEP40 ||
	    cipher == MLAN_ENCRYPTION_MODE_WEP104)
		alg = 1;

	LEAVE();
	return alg;
}

/**
 *  @brief Convert NL80211 interface type to MLAN_BSS_MODE_*
 *
 *  @param iftype   Interface type of NL80211
 *
 *  @return         Driver bss mode
 */
static t_u32 woal_nl80211_iftype_to_mode(enum nl80211_iftype iftype)
{
	switch (iftype) {
	case NL80211_IFTYPE_STATION:
		return MLAN_BSS_MODE_INFRA;
	case NL80211_IFTYPE_UNSPECIFIED:
	default:
		return MLAN_BSS_MODE_AUTO;
	}
}

/**
 *  @brief Control WPS Session Enable/Disable
 *
 *  @param priv     Pointer to the moal_private driver data struct
 *  @param enable   enable/disable flag
 *
 *  @return          0 --success, otherwise fail
 */
static int woal_wps_cfg(moal_private *priv, int enable)
{
	int ret = 0;
	mlan_ds_wps_cfg *pwps = NULL;
	mlan_ioctl_req *req = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	PRINTM(MINFO, "WOAL_WPS_SESSION\n");

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_wps_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	pwps = (mlan_ds_wps_cfg *)req->pbuf;
	req->req_id = MLAN_IOCTL_WPS_CFG;
	req->action = MLAN_ACT_SET;
	pwps->sub_command = MLAN_OID_WPS_CFG_SESSION;
	if (enable)
		pwps->param.wps_session = MLAN_WPS_CFG_SESSION_START;
	else
		pwps->param.wps_session = MLAN_WPS_CFG_SESSION_END;

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
 * @brief configure ASSOC IE
 *
 * @param priv				A pointer to moal private structure
 * @param ie				A pointer to ie data
 * @param ie_len			The length of ie data
 * @param wait_option       wait option
 *
 * @return                  0 -- success, otherwise fail
 */
static int woal_cfg80211_assoc_ies_cfg(moal_private *priv, const t_u8 *ie,
				       size_t ie_len, t_u8 wait_option)
{
	int bytes_left = ie_len;
	const t_u8 *pcurrent_ptr = ie;
	int total_ie_len;
	t_u8 element_len;
	int ret = MLAN_STATUS_SUCCESS;
	IEEEtypes_ElementId_e element_id;
	const IEEEtypes_VendorSpecific_t *pvendor_ie;
	t_u8 wps_oui[] = {0x00, 0x50, 0xf2, 0x04};
	t_u8 hs20_oui[] = {0x50, 0x6f, 0x9a, 0x10};

	t_u8 multiap_oui[] = {0x50, 0x6f, 0x9a, 0x1b};
	t_u8 multiap_flag = 0;

	while (bytes_left >= 2) {
		element_id =
			(IEEEtypes_ElementId_e)(*((const t_u8 *)pcurrent_ptr));
		element_len = *((const t_u8 *)pcurrent_ptr + 1);
		total_ie_len = element_len + sizeof(IEEEtypes_Header_t);
		if (bytes_left < total_ie_len) {
			PRINTM(MERROR,
			       "InterpretIE: Error in processing IE, bytes left < IE length\n");
			bytes_left = 0;
			continue;
		}
		switch (element_id) {
		case RSN_IE:
			if (MLAN_STATUS_SUCCESS !=
			    woal_set_get_gen_ie(priv, MLAN_ACT_SET,
						pcurrent_ptr, NULL,
						&total_ie_len, wait_option)) {
				PRINTM(MERROR, "Fail to set RSN IE\n");
				ret = -EFAULT;
				goto done;
			}
			PRINTM(MIOCTL, "Set RSN IE\n");
			break;
		case VENDOR_SPECIFIC_221:
			pvendor_ie = (const IEEEtypes_VendorSpecific_t *)
				pcurrent_ptr;
			if (!memcmp(pvendor_ie->vend_hdr.oui, wps_oui,
				    sizeof(pvendor_ie->vend_hdr.oui)) &&
			    (pvendor_ie->vend_hdr.oui_type == wps_oui[3])) {
				PRINTM(MIOCTL, "Enable WPS session\n");
				if (woal_wps_cfg(priv, MTRUE)) {
					PRINTM(MERROR,
					       "%s: Enable WPS session failed\n",
					       __func__);
					ret = -EFAULT;
					goto done;
				}
			}

			if (!memcmp(pvendor_ie->vend_hdr.oui, multiap_oui,
				    sizeof(pvendor_ie->vend_hdr.oui)) &&
			    (pvendor_ie->vend_hdr.oui_type == multiap_oui[3])) {
				multiap_flag = pvendor_ie->data[0];
				if (MLAN_STATUS_SUCCESS !=
				    woal_multi_ap_cfg(priv, wait_option,
						      multiap_flag)) {
					PRINTM(MERROR,
					       "%s: failed to configure multi ap\n",
					       __func__);
					ret = -EFAULT;
					goto done;
				}
			}

			if (!memcmp(pvendor_ie->vend_hdr.oui, hs20_oui,
				    sizeof(pvendor_ie->vend_hdr.oui)) &&
			    (pvendor_ie->vend_hdr.oui_type == hs20_oui[3])) {
				PRINTM(MIOCTL,
				       "Hotspot2.0 is enabled for this bss\n");
				if (MLAN_STATUS_SUCCESS !=
				    woal_set_hotspotcfg(priv, wait_option,
							(HOTSPOT_BY_SUPPLICANT |
							 HOTSPOT_ENABLED))) {
					PRINTM(MERROR,
					       "Fail to enable hotspot 2.0\n");
					ret = -EFAULT;
					goto done;
				}
			}
			if (MLAN_STATUS_SUCCESS !=
			    woal_set_get_gen_ie(priv, MLAN_ACT_SET,
						pcurrent_ptr, NULL,
						&total_ie_len, wait_option)) {
				PRINTM(MERROR,
				       "Fail to Set VENDOR SPECIFIC IE\n");
				ret = -EFAULT;
				goto done;
			}
			PRINTM(MIOCTL,
			       "Set VENDOR SPECIFIC IE, OUI: %02x:%02x:%02x:%02x\n",
			       pvendor_ie->vend_hdr.oui[0],
			       pvendor_ie->vend_hdr.oui[1],
			       pvendor_ie->vend_hdr.oui[2],
			       pvendor_ie->vend_hdr.oui_type);
			break;
		case MOBILITY_DOMAIN:
			break;
		case FAST_BSS_TRANSITION:
			if (MLAN_STATUS_SUCCESS !=
			    woal_set_get_gen_ie(priv, MLAN_ACT_SET,
						pcurrent_ptr, NULL,
						&total_ie_len, wait_option)) {
				PRINTM(MERROR, "Fail to set"
					       "FAST_BSS_TRANSITION IE\n");
				ret = -EFAULT;
				goto done;
			}
			PRINTM(MIOCTL, "Set FAST_BSS_TRANSITION IE\n");
			break;
		case RIC:
			if (MLAN_STATUS_SUCCESS !=
			    woal_set_get_gen_ie(priv, MLAN_ACT_SET,
						pcurrent_ptr, NULL,
						&total_ie_len, wait_option)) {
				PRINTM(MERROR,
				       "Fail to set"
				       "RESOURCE INFORMATION CONTAINER IE\n");
				ret = -EFAULT;
				goto done;
			}
			PRINTM(MIOCTL,
			       "Set RESOURCE INFORMATION CONTAINER IE\n");
			break;
		case EXT_CAPABILITY:
			if (MLAN_STATUS_SUCCESS !=
			    woal_set_get_gen_ie(priv, MLAN_ACT_SET,
						pcurrent_ptr, NULL,
						&total_ie_len, wait_option)) {
				PRINTM(MERROR,
				       "Fail to set Extended Capabilites IE\n");
				ret = -EFAULT;
				goto done;
			}
			PRINTM(MIOCTL, "Set Extended Capabilities IE\n");
			break;
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
		case EXTENSION:
			if (MLAN_STATUS_SUCCESS !=
			    woal_set_get_gen_ie(priv, MLAN_ACT_SET,
						pcurrent_ptr, NULL,
						&total_ie_len, wait_option)) {
				PRINTM(MERROR, "Fail to set Extension IE\n");
				ret = -EFAULT;
				goto done;
			}
			PRINTM(MIOCTL, "Set Extension IE\n");
			break;
		case FRAGMENT:
			if (MLAN_STATUS_SUCCESS !=
			    woal_set_get_gen_ie(priv, MLAN_ACT_SET,
						pcurrent_ptr, NULL,
						&total_ie_len, wait_option)) {
				PRINTM(MERROR, "Fail to set Fragmented IE\n");
				ret = -EFAULT;
				goto done;
			}
			PRINTM(MIOCTL, "Set Fragmented IE\n");
			break;
#endif
		default:
			if (MLAN_STATUS_SUCCESS !=
			    woal_set_get_gen_ie(priv, MLAN_ACT_SET,
						pcurrent_ptr, NULL,
						&total_ie_len, wait_option)) {
				PRINTM(MERROR, "Fail to set GEN IE\n");
				// coverity[misra_c_2012_rule_11_8_violation:SUPPRESS]
				DBG_HEXDUMP(MCMD_D, "GEN IE",
					    (t_u8 *)pcurrent_ptr, total_ie_len);
				ret = -EFAULT;
				goto done;
			}
			PRINTM(MIOCTL, "Set GEN IE\n");
			break;
		}
		pcurrent_ptr += element_len + 2;
		/* Need to account for IE ID and IE Len */
		bytes_left -= (element_len + 2);
	}
done:
	return ret;
}

#ifdef CONFIG_NL80211_TESTMODE
enum moal_tm_attr {
	__MOAL_TM_ATTR_INVALID = 0,
	MOAL_TM_ATTR_CMD = 1,
	MOAL_TM_ATTR_DATA = 2,

	/* keep last */
	__MOAL_TM_ATTR_AFTER_LAST,
	MOAL_TM_ATTR_MAX = __MOAL_TM_ATTR_AFTER_LAST - 1,
};

static const struct nla_policy moal_tm_policy[MOAL_TM_ATTR_MAX + 1] = {
	[MOAL_TM_ATTR_CMD] = {.type = NLA_U32},
	[MOAL_TM_ATTR_DATA] = {.type = NLA_BINARY,
			       .len = MRVDRV_SIZE_OF_CMD_BUFFER},
};

enum moal_tm_command {
	MOAL_TM_CMD_HOSTCMD = 0,
};

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 12, 0)
static int woal_testmode_cmd(struct wiphy *wiphy, struct wireless_dev *wdev,
			     void *data, int len)
#else
static int woal_testmode_cmd(struct wiphy *wiphy, void *data, int len)
#endif
{
	moal_handle *handle = (moal_handle *)woal_get_wiphy_priv(wiphy);
	moal_private *priv =
		(moal_private *)woal_get_priv(handle, MLAN_BSS_ROLE_ANY);
	mlan_ioctl_req *req = NULL;
	mlan_ds_misc_cfg *misc_cfg = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;
	struct nlattr *tb[MOAL_TM_ATTR_MAX + 1];
	struct sk_buff *skb;
	int err;

	if (!priv)
		return -EINVAL;

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 12, 0)
	err = nla_parse(tb, MOAL_TM_ATTR_MAX, data, len, moal_tm_policy, NULL);
#else
	err = nla_parse(tb, MOAL_TM_ATTR_MAX, data, len, moal_tm_policy);
#endif
	if (err)
		return err;

	if (!tb[MOAL_TM_ATTR_CMD])
		return -EINVAL;

	switch (nla_get_u32(tb[MOAL_TM_ATTR_CMD])) {
	case MOAL_TM_CMD_HOSTCMD:
		if (!tb[MOAL_TM_ATTR_DATA])
			return -EINVAL;
		req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
		if (req == NULL)
			return -ENOMEM;
		misc_cfg = (mlan_ds_misc_cfg *)req->pbuf;
		misc_cfg->sub_command = MLAN_OID_MISC_HOST_CMD;
		req->req_id = MLAN_IOCTL_MISC_CFG;
		req->action = MLAN_ACT_SET;
		misc_cfg->param.hostcmd.len = nla_len(tb[MOAL_TM_ATTR_DATA]);
		moal_memcpy_ext(priv->phandle, misc_cfg->param.hostcmd.cmd,
				nla_data(tb[MOAL_TM_ATTR_DATA]),
				misc_cfg->param.hostcmd.len,
				MRVDRV_SIZE_OF_CMD_BUFFER);
		status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
		if (status != MLAN_STATUS_SUCCESS) {
			err = -EFAULT;
			goto error;
		}
		/* process hostcmd response*/
		skb = cfg80211_testmode_alloc_reply_skb(
			wiphy, misc_cfg->param.hostcmd.len);
		if (!skb) {
			kfree(req);
			return -ENOMEM;
		}
		err = nla_put(skb, MOAL_TM_ATTR_DATA,
			      misc_cfg->param.hostcmd.len,
			      misc_cfg->param.hostcmd.cmd);
		if (err) {
			kfree(req);
			kfree_skb(skb);
			return -EMSGSIZE;
		}
		err = cfg80211_testmode_reply(skb);
		kfree(req);
		return err;
	default:
		return -EOPNOTSUPP;
	}
error:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	return err;
}
#endif

/**
 * @brief Send domain info command to FW
 *
 * @param priv      A pointer to moal_private structure
 * @param wait_option  wait option
 *
 * @return          MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status woal_send_domain_info_cmd_fw(moal_private *priv,
						t_u8 wait_option)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	enum ieee80211_band band;
	struct ieee80211_supported_band *sband = NULL;
	struct ieee80211_channel *channel = NULL;
	t_u8 no_of_sub_band = 0;
	t_u8 no_of_parsed_chan = 0;
	t_u8 first_chan = 0, next_chan = 0, max_pwr = 0;
	t_u8 i, flag = 0;
	mlan_ds_11d_cfg *cfg_11d = NULL;
	mlan_ioctl_req *req = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	if (!priv->wdev || !priv->wdev->wiphy) {
		PRINTM(MERROR, "No wdev or wiphy in priv\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	band = priv->phandle->band;
	if (!priv->wdev->wiphy->bands[band]) {
		PRINTM(MERROR, "11D: setting domain info in FW failed band=%d",
		       band);
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	PRINTM(MCMD_D, "Send domain info: country=%c%c band=%d dfs_region=%d\n",
	       priv->phandle->country_code[0], priv->phandle->country_code[1],
	       band, priv->phandle->dfs_region);
	/* Allocate an IOCTL request buffer */
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_11d_cfg));
	if (req == NULL) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	cfg_11d = (mlan_ds_11d_cfg *)req->pbuf;
	cfg_11d->sub_command = MLAN_OID_11D_DOMAIN_INFO_EXT;
	req->req_id = MLAN_IOCTL_11D_CFG;
	req->action = MLAN_ACT_SET;
	cfg_11d->param.domain_info.dfs_region = priv->phandle->dfs_region;
	if (is_cfg80211_special_region_code(priv->phandle->country_code)) {
		/* Set country code */
		cfg_11d->param.domain_info.country_code[0] = 'W';
		cfg_11d->param.domain_info.country_code[1] = 'W';
	} else {
		/* Set country code */
		cfg_11d->param.domain_info.country_code[0] =
			priv->phandle->country_code[0];
		cfg_11d->param.domain_info.country_code[1] =
			priv->phandle->country_code[1];
	}
	cfg_11d->param.domain_info.country_code[2] = ' ';
	cfg_11d->param.domain_info.band = woal_ieee_band_to_radio_type(band);

	sband = priv->wdev->wiphy->bands[band];
	for (i = 0; (i < sband->n_channels) &&
		    (no_of_sub_band < MRVDRV_MAX_SUBBAND_802_11D);
	     i++) {
		channel = &sband->channels[i];
		if (channel->flags & IEEE80211_CHAN_DISABLED)
			continue;

		if (!flag) {
			flag = 1;
			next_chan = first_chan = (t_u32)channel->hw_value;
			max_pwr = channel->max_power;
			no_of_parsed_chan = 1;
			continue;
		}

		if (channel->hw_value == next_chan + 1 &&
		    channel->max_power == max_pwr) {
			next_chan++;
			no_of_parsed_chan++;
		} else {
			cfg_11d->param.domain_info.sub_band[no_of_sub_band]
				.first_chan = first_chan;
			cfg_11d->param.domain_info.sub_band[no_of_sub_band]
				.no_of_chan = no_of_parsed_chan;
			cfg_11d->param.domain_info.sub_band[no_of_sub_band]
				.max_tx_pwr = max_pwr;
			no_of_sub_band++;
			next_chan = first_chan = (t_u32)channel->hw_value;
			max_pwr = channel->max_power;
			no_of_parsed_chan = 1;
		}
	}

	if (flag && (no_of_sub_band < MRVDRV_MAX_SUBBAND_802_11D)) {
		cfg_11d->param.domain_info.sub_band[no_of_sub_band].first_chan =
			first_chan;
		cfg_11d->param.domain_info.sub_band[no_of_sub_band].no_of_chan =
			no_of_parsed_chan;
		cfg_11d->param.domain_info.sub_band[no_of_sub_band].max_tx_pwr =
			max_pwr;
		no_of_sub_band++;
	}
	cfg_11d->param.domain_info.no_of_sub_band = no_of_sub_band;

	PRINTM(MCMND, "CFG80211: Country=%c%c, band=%d, no_of_sub_band=%d\n",
	       priv->phandle->country_code[0], priv->phandle->country_code[1],
	       priv->phandle->band, cfg_11d->param.domain_info.no_of_sub_band);

	/* skip download the command to FW when “no_of_sub_band = 0 */
	if (!no_of_sub_band)
		goto done;

	/* Send domain info command to FW */
	status = woal_request_ioctl(priv, req, wait_option);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = MLAN_STATUS_FAILURE;
		PRINTM(MERROR, "11D: Error setting domain info in FW\n");
		goto done;
	}

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 * @brief Send channel attributes to the FW
 *
 * @param priv      A pointer to moal_private structure
 * @param is6g      whether its a 6g table
 * @param wait_option  wait option
 *
 * @return          MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status woal_dnld_chan_attr(moal_private *priv, t_bool is6g,
				       t_u8 wait_option)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	struct ieee80211_supported_band *sband = NULL;
	struct wiphy *wiphy = NULL;
	t_u8 i, c = 0;
	mlan_ds_misc_cfg *misc = NULL;
	mlan_ds_chan_attr *ca = NULL;
	mlan_ioctl_req *req = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	if (!priv || !priv->wdev || !priv->wdev->wiphy) {
		PRINTM(MERROR, "No priv or no wdev or wiphy in priv\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	wiphy = priv->wdev->wiphy;

	/* Allocate an IOCTL request buffer */
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	misc = (mlan_ds_misc_cfg *)req->pbuf;
	misc->sub_command = MLAN_OID_MISC_GET_CHAN_REGION_CFG;
	req->req_id = MLAN_IOCTL_MISC_CFG;
	memset(&misc->param.chan_attr_cfg, 0,
	       sizeof(misc->param.chan_attr_cfg));

	ca = (mlan_ds_chan_attr *)&misc->param.chan_attr_cfg;

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 4, 0)
	if (is6g) {
		req->action = MLAN_ACT_SET_6G_CFP_TBL;
		sband = wiphy->bands[NL80211_BAND_6GHZ];
		if (sband) {
			for (i = 0; i < sband->n_channels; i++, c++) {
				ca->chan_attr[c].channel =
					sband->channels[i].hw_value;
				if (sband->channels[i].flags &
				    IEEE80211_CHAN_DISABLED)
					ca->chan_attr[c].flags |=
						NXP_CHANNEL_DISABLED;
				if (sband->channels[i].flags &
				    IEEE80211_CHAN_NO_IR)
					ca->chan_attr[c].flags |=
						NXP_CHANNEL_PASSIVE;
				if (sband->channels[i].flags &
				    IEEE80211_CHAN_RADAR)
					ca->chan_attr[c].flags |=
						NXP_CHANNEL_DFS;
				if ((sband->channels[i].flags &
				     IEEE80211_CHAN_NO_HT40MINUS) &&
				    (sband->channels[i].flags &
				     IEEE80211_CHAN_NO_HT40PLUS))
					ca->chan_attr[c].flags |=
						NXP_CHANNEL_NOHT40;
				if (sband->channels[i].flags &
				    IEEE80211_CHAN_NO_80MHZ)
					ca->chan_attr[c].flags |=
						NXP_CHANNEL_NOHT80;
			}
		}
	} else
#endif
	{
		req->action = MLAN_ACT_SET;
		sband = wiphy->bands[NL80211_BAND_2GHZ];
		if (sband) {
			for (i = 0; i < sband->n_channels; i++, c++) {
				ca->chan_attr[c].channel =
					sband->channels[i].hw_value;
				if (sband->channels[i].flags &
				    IEEE80211_CHAN_DISABLED)
					ca->chan_attr[c].flags |=
						NXP_CHANNEL_DISABLED;
				if (sband->channels[i].flags &
				    IEEE80211_CHAN_NO_IR)
					ca->chan_attr[c].flags |=
						NXP_CHANNEL_PASSIVE;
				if (sband->channels[i].flags &
				    IEEE80211_CHAN_RADAR)
					ca->chan_attr[c].flags |=
						NXP_CHANNEL_DFS;
				if ((sband->channels[i].flags &
				     IEEE80211_CHAN_NO_HT40MINUS) &&
				    (sband->channels[i].flags &
				     IEEE80211_CHAN_NO_HT40PLUS))
					ca->chan_attr[c].flags |=
						NXP_CHANNEL_NOHT40;
				if (sband->channels[i].flags &
				    IEEE80211_CHAN_NO_80MHZ)
					ca->chan_attr[c].flags |=
						NXP_CHANNEL_NOHT80;
			}
		}
		sband = wiphy->bands[NL80211_BAND_5GHZ];
		if (sband) {
			for (i = 0; i < sband->n_channels; i++, c++) {
				ca->chan_attr[c].channel =
					sband->channels[i].hw_value;
				if (sband->channels[i].flags &
				    IEEE80211_CHAN_DISABLED)
					ca->chan_attr[c].flags |=
						NXP_CHANNEL_DISABLED;
				if (sband->channels[i].flags &
				    IEEE80211_CHAN_NO_IR)
					ca->chan_attr[c].flags |=
						NXP_CHANNEL_PASSIVE;
				if (sband->channels[i].flags &
				    IEEE80211_CHAN_RADAR)
					ca->chan_attr[c].flags |=
						NXP_CHANNEL_DFS;
				if ((sband->channels[i].flags &
				     IEEE80211_CHAN_NO_HT40MINUS) &&
				    (sband->channels[i].flags &
				     IEEE80211_CHAN_NO_HT40PLUS))
					ca->chan_attr[c].flags |=
						NXP_CHANNEL_NOHT40;
				if (sband->channels[i].flags &
				    IEEE80211_CHAN_NO_80MHZ)
					ca->chan_attr[c].flags |=
						NXP_CHANNEL_NOHT80;
			}
		}
	}
	c = (c > MLAN_MAX_CHANNEL_NUM) ? MLAN_MAX_CHANNEL_NUM : c;
	ca->data_len = c * sizeof(chan_attr_t);

	/* Send chan attr command to FW */
	if (c)
		status = woal_request_ioctl(priv, req, wait_option);

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}
/**
 * @brief Request the driver to change the channel and
 * change domain info according to that channel
 *
 * @param priv            A pointer to moal_private structure
 * @param chan            A pointer to ieee80211_channel structure
 * @param channel_type    Channel type of nl80211_channel_type
 * @param wait_option     wait option
 *
 * @return                0 -- success, otherwise fail
 */
int woal_set_rf_channel(moal_private *priv, struct ieee80211_channel *chan,
			enum nl80211_channel_type channel_type,
			t_u8 wait_option)
{
	int ret = 0;
	t_u32 mode, config_bands = 0;
	mlan_ioctl_req *req = NULL;
	mlan_ds_radio_cfg *radio_cfg = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	if (!chan) {
		LEAVE();
		return -EINVAL;
	}
	mode = woal_nl80211_iftype_to_mode(priv->wdev->iftype);
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_radio_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}
	radio_cfg = (mlan_ds_radio_cfg *)req->pbuf;
	radio_cfg->sub_command = MLAN_OID_BAND_CFG;
	req->req_id = MLAN_IOCTL_RADIO_CFG;
	/* Get config_bands, adhoc_start_band and adhoc_channel values from MLAN
	 */
	req->action = MLAN_ACT_GET;
	status = woal_request_ioctl(priv, req, wait_option);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}
	req->action = MLAN_ACT_SET;
	priv->phandle->band = chan->band;
	/* Set appropriate bands */
	if (chan->band == IEEE80211_BAND_2GHZ)
		config_bands = BAND_B | BAND_G | BAND_GN;
	else {
		config_bands = BAND_AN | BAND_A;
	}

	status = woal_request_ioctl(priv, req, wait_option);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}
	woal_send_domain_info_cmd_fw(priv, wait_option);

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Set ewpa mode
 *
 *  @param priv                 A pointer to moal_private structure
 *  @param wait_option          Wait option
 *  @param ssid_bssid           A pointer to mlan_ssid_bssid structure
 *
 *  @return                     MLAN_STATUS_SUCCESS/MLAN_STATUS_PENDING --
 * success, otherwise fail
 */
static mlan_status woal_set_ewpa_mode(moal_private *priv, t_u8 wait_option,
				      mlan_ssid_bssid *ssid_bssid)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_sec_cfg *sec = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	if (!priv->phandle->card_info->embedded_supp)
		goto error;

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
	req->action = MLAN_ACT_GET;

	/* Try Get All */
	memset(&sec->param.passphrase, 0, sizeof(mlan_ds_passphrase));
	moal_memcpy_ext(priv->phandle, &sec->param.passphrase.ssid,
			&ssid_bssid->ssid, sizeof(sec->param.passphrase.ssid),
			sizeof(sec->param.passphrase.ssid));
	moal_memcpy_ext(priv->phandle, &sec->param.passphrase.bssid,
			&ssid_bssid->bssid, MLAN_MAC_ADDR_LENGTH,
			sizeof(sec->param.passphrase.bssid));
	sec->param.passphrase.psk_type = MLAN_PSK_QUERY;

	/* Send IOCTL request to MLAN */
	status = woal_request_ioctl(priv, req, wait_option);
	if (status != MLAN_STATUS_SUCCESS)
		goto error;
	sec->param.ewpa_enabled = MFALSE;
	if (sec->param.passphrase.psk_type == MLAN_PSK_PASSPHRASE) {
		if (sec->param.passphrase.psk.passphrase.passphrase_len > 0)
			sec->param.ewpa_enabled = MTRUE;
	} else if (sec->param.passphrase.psk_type == MLAN_PSK_PMK)
		sec->param.ewpa_enabled = MTRUE;

	sec->sub_command = MLAN_OID_SEC_CFG_EWPA_ENABLED;
	req->action = MLAN_ACT_SET;

	/* Send IOCTL request to MLAN */
	status = woal_request_ioctl(priv, req, wait_option);

error:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return status;
}

/**
 * @brief Set encryption mode and enable WPA
 *
 * @param priv          A pointer to moal_private structure
 * @param encrypt_mode  Encryption mode
 * @param wpa_enabled   WPA enable or not
 * @param wait_option   wait option
 *
 * @return              0 -- success, otherwise fail
 */
static int woal_cfg80211_set_auth(moal_private *priv, int encrypt_mode,
				  int wpa_enabled, t_u8 wait_option)
{
	int ret = 0;

	ENTER();

	if (MLAN_STATUS_SUCCESS !=
	    woal_set_encrypt_mode(priv, wait_option, encrypt_mode))
		ret = -EFAULT;

	if (wpa_enabled) {
		if (MLAN_STATUS_SUCCESS !=
		    woal_set_wpa_enable(priv, wait_option, 1))
			ret = -EFAULT;
	}

	LEAVE();
	return ret;
}

/**
 * @brief Reset the wifi
 *
 * @param handle        A pointer to moal_handle structure
 * @param cnt           wifi reset count
 * @param reason        wifi reset reason
 *
 * @return              MLAN_STATUS_SUCCESS or MLAN_STATUS_PENDING
 */
mlan_status woal_reset_wifi(moal_handle *handle, t_u8 cnt, char *reason)
{
	static wifi_timeval reset_time;
	wifi_timeval ts;
	t_u64 diff;
	t_u8 intf_num;

	/* Disconnect all interfaces */
	for (intf_num = 0; intf_num < handle->priv_num; intf_num++) {
		if (handle->priv[intf_num] &&
		    handle->priv[intf_num]->media_connected == MTRUE) {
			if (woal_disconnect(handle->priv[intf_num],
					    MOAL_IOCTL_WAIT, NULL,
					    DEF_DEAUTH_REASON_CODE))
				PRINTM(MERROR, "woal_disconnect failed\n");
		}
	}
#define MAX_WIFI_RESET_INTERVAL 15 * 60 * 1000000 // 15 minute
	woal_get_monotonic_time(&ts);
	diff = (t_u64)(timeval_to_usec(ts) - timeval_to_usec(reset_time));
	PRINTM(MERROR, "WiFi Reset diff %lld\n", diff);
	if (reset_time.time_sec == 0 || diff >= MAX_WIFI_RESET_INTERVAL) {
		reset_time = ts;
		PRINTM(MERROR, "WiFi Reset due to %s cnt %d\n", reason, cnt);
		/* Do wifi independent reset */
		woal_process_hang(handle);
		return MLAN_STATUS_SUCCESS;
	}
	return MLAN_STATUS_PENDING;
}

/**
 * @brief Informs the CFG802.11 subsystem of a new BSS connection.
 *
 * The following information are sent to the CFG802.11 subsystem
 * to register the new BSS connection. If we do not register the new BSS,
 * a kernel panic will result.
 *      - MAC address
 *      - Capabilities
 *      - Beacon period
 *      - RSSI value
 *      - Channel
 *      - Supported rates IE
 *      - Extended capabilities IE
 *      - DS parameter set IE
 *      - HT Capability IE
 *      - Vendor Specific IE (221)
 *      - WPA IE
 *      - RSN IE
 *
 * @param priv            A pointer to moal_private structure
 * @param ssid_bssid      A pointer to A pointer to mlan_ssid_bssid structure
 * @param wait_option     wait_option
 *
 * @return          MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status woal_inform_bss_from_scan_result(moal_private *priv,
					     mlan_ssid_bssid *ssid_bssid,
					     t_u8 wait_option)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	struct ieee80211_channel *chan;
	mlan_scan_resp scan_resp;
	BSSDescriptor_t *scan_table;
	t_u64 ts = 0;
	u16 cap_info = 0;
	int i = 0;
	struct cfg80211_bss *pub = NULL;

	ENTER();
	if (!priv->wdev || !priv->wdev->wiphy) {
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}

	memset(&scan_resp, 0, sizeof(scan_resp));
	if (MLAN_STATUS_SUCCESS !=
	    woal_get_scan_table(priv, wait_option, &scan_resp)) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	if (scan_resp.num_in_scan_table) {
		scan_table = (BSSDescriptor_t *)scan_resp.pscan_table;
		for (i = 0; i < (int)scan_resp.num_in_scan_table; i++) {
			if (ssid_bssid) {
				/* Inform specific BSS only */
				if (memcmp(ssid_bssid->ssid.ssid,
					   scan_table[i].ssid.ssid,
					   ssid_bssid->ssid.ssid_len) ||
				    memcmp(ssid_bssid->bssid,
					   scan_table[i].mac_address, ETH_ALEN))
					continue;
			}
			if (!scan_table[i].freq) {
				scan_table[i].freq =
					ieee80211_channel_to_frequency(
						(int)scan_table[i].channel
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(2, 6, 39)
						,
						woal_band_cfg_to_ieee_band(
							scan_table[i].bss_band)
#endif
					);
			}
			chan = ieee80211_get_channel(priv->wdev->wiphy,
						     scan_table[i].freq);
			if (!chan) {
				PRINTM(MCMND,
				       "Fail to get chan with freq: channel=%d freq=%d\n",
				       (int)scan_table[i].channel,
				       (int)scan_table[i].freq);
				continue;
			}
#if defined(WIFI_DIRECT_SUPPORT)
#if CFG80211_VERSION_CODE >= WIFI_DIRECT_KERNEL_VERSION
			if (priv->bss_type == MLAN_BSS_TYPE_WIFIDIRECT &&
			    !ssid_bssid) {
				if (!strncmp(scan_table[i].ssid.ssid, "DIRECT-",
					     strlen("DIRECT-"))) {
					PRINTM(MCMND,
					       "wlan: P2P device " MACSTR
					       " found, channel=%d\n",
					       MAC2STR(scan_table[i]
							       .mac_address),
					       (int)chan->hw_value);
				}
			}
#endif
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 39)
			/** Andorid's Location service is expecting timestamp to
			 * be local time (in microsecond) since boot; and not
			 * the TSF found in the beacon. */
			ts = ktime_to_us(ktime_get_boottime());
#else
			moal_memcpy_ext(priv->phandle, &ts,
					scan_table[i].time_stamp, sizeof(ts),
					sizeof(ts));
#endif
			moal_memcpy_ext(priv->phandle, &cap_info,
					&scan_table[i].cap_info,
					sizeof(cap_info), sizeof(cap_info));
			pub = cfg80211_inform_bss(
				priv->wdev->wiphy, chan,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 18, 0)
				CFG80211_BSS_FTYPE_UNKNOWN,
#endif
				scan_table[i].mac_address, ts, cap_info,
				scan_table[i].beacon_period,
				scan_table[i].pbeacon_buf +
					WLAN_802_11_FIXED_IE_SIZE,
				scan_table[i].beacon_buf_size -
					WLAN_802_11_FIXED_IE_SIZE,
				-RSSI_DBM_TO_MDM(scan_table[i].rssi),
				GFP_KERNEL);
			if (pub) {
#if CFG80211_VERSION_CODE < KERNEL_VERSION(3, 8, 0)
				pub->len_information_elements =
					pub->len_beacon_ies;
#endif
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 9, 0)
				cfg80211_put_bss(priv->wdev->wiphy, pub);
#else
				cfg80211_put_bss(pub);
#endif
			}
		}
	}
done:
	LEAVE();
	return ret;
}

/**
 * @brief Process country IE before assoicate
 *
 * @param priv            A pointer to moal_private structure
 * @param bss             A pointer to cfg80211_bss structure
 *
 * @return                0 -- success, otherwise fail
 */
static int woal_process_country_ie(moal_private *priv, struct cfg80211_bss *bss)
{
	const u8 *country_ie;
	u8 country_ie_len;
	int ret = 0;
	mlan_ioctl_req *req = NULL;
	mlan_ds_11d_cfg *cfg_11d = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();
	rcu_read_lock();
	country_ie = (const u8 *)ieee80211_bss_get_ie(bss, WLAN_EID_COUNTRY);
	if (!country_ie) {
		rcu_read_unlock();
		PRINTM(MIOCTL, "No country IE found!\n");
		woal_send_domain_info_cmd_fw(priv, MOAL_IOCTL_WAIT);
		LEAVE();
		return 0;
	}

	country_ie_len = country_ie[1];
	if (country_ie_len < IEEE80211_COUNTRY_IE_MIN_LEN) {
		rcu_read_unlock();
		PRINTM(MIOCTL, "Wrong Country IE length!\n");
		woal_send_domain_info_cmd_fw(priv, MOAL_IOCTL_WAIT);
		LEAVE();
		return 0;
	}
	priv->phandle->country_code[0] = country_ie[2];
	priv->phandle->country_code[1] = country_ie[3];
	priv->phandle->country_code[2] = ' ';
	if (is_cfg80211_special_region_code(priv->phandle->country_code)) {
		rcu_read_unlock();
		PRINTM(MIOCTL, "Skip special region code in CountryIE");
		LEAVE();
		return 0;
	}
	if (MLAN_STATUS_SUCCESS !=
	    woal_set_region_code(priv, priv->phandle->country_code))
		PRINTM(MERROR, "Set country code failed!\n");

	/* Allocate an IOCTL request buffer */
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_11d_cfg));
	if (req == NULL) {
		rcu_read_unlock();
		PRINTM(MERROR, "Fail to allocate mlan_ds_11d_cfg buffer\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	cfg_11d = (mlan_ds_11d_cfg *)req->pbuf;
	cfg_11d->sub_command = MLAN_OID_11D_DOMAIN_INFO_EXT;
	req->req_id = MLAN_IOCTL_11D_CFG;
	req->action = MLAN_ACT_SET;

	cfg_11d->param.domain_info.dfs_region = NXP_DFS_UNKNOWN;
	/* Set country code */
	cfg_11d->param.domain_info.country_code[0] =
		priv->phandle->country_code[0];
	cfg_11d->param.domain_info.country_code[1] =
		priv->phandle->country_code[1];
	cfg_11d->param.domain_info.country_code[2] = ' ';

	/** IEEE80211_BAND_2GHZ or IEEE80211_BAND_5GHZ */
	cfg_11d->param.domain_info.band = priv->phandle->band;

	country_ie_len -= COUNTRY_CODE_LEN;
	cfg_11d->param.domain_info.no_of_sub_band = MIN(
		MRVDRV_MAX_SUBBAND_802_11D,
		(country_ie_len / sizeof(struct ieee80211_country_ie_triplet)));
	moal_memcpy_ext(priv->phandle,
			(u8 *)cfg_11d->param.domain_info.sub_band,
			&country_ie[2] + COUNTRY_CODE_LEN,
			cfg_11d->param.domain_info.no_of_sub_band *
				sizeof(mlan_ds_subband_set_t),
			sizeof(cfg_11d->param.domain_info.sub_band));

	PRINTM(MCMND, "11D: Country IE: %c%c band=%d no_of_sub_band=%d\n",
	       country_ie[2], country_ie[3], priv->phandle->band,
	       cfg_11d->param.domain_info.no_of_sub_band);
	rcu_read_unlock();

	/* Send domain info command to FW */
	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = MLAN_STATUS_FAILURE;
		PRINTM(MERROR, "11D: Error setting domain info in FW\n");
		goto done;
	}
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 * @brief Request scan based on connect parameter
 *
 * @param priv            A pointer to moal_private structure
 * @param conn_param      A pointer to connect parameters
 * @param wait_option     wait option
 *
 * @return                0 -- success, otherwise fail
 */
static int
woal_cfg80211_connect_scan(moal_private *priv,
			   struct cfg80211_connect_params *conn_param,
			   t_u8 wait_option)
{
	moal_handle *handle = priv->phandle;
	int ret = 0;
	wlan_user_scan_cfg *scan_req;
	enum ieee80211_band band;
	struct ieee80211_supported_band *sband;
	struct ieee80211_channel *ch;
	int chan_idx = 0, i;

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
	scan_req = (wlan_user_scan_cfg *)kmalloc(sizeof(wlan_user_scan_cfg),
						 GFP_KERNEL);
	if (!scan_req) {
		PRINTM(MERROR, "Malloc buffer failed\n");
		LEAVE();
		return -ENOMEM;
	}

	priv->report_scan_result = MTRUE;
	memset(scan_req, 0x00, sizeof(wlan_user_scan_cfg));
	moal_memcpy_ext(priv->phandle, scan_req->ssid_list[0].ssid,
			conn_param->ssid, conn_param->ssid_len,
			sizeof(scan_req->ssid_list[0].ssid));
	scan_req->ssid_list[0].max_len = 0;
	if (conn_param->channel) {
		scan_req->chan_list[0].chan_number =
			conn_param->channel->hw_value;
		scan_req->chan_list[0].radio_type =
			woal_ieee_band_to_radio_type(conn_param->channel->band);
		if (conn_param->channel->flags & IEEE80211_CHAN_PASSIVE_SCAN)
			scan_req->chan_list[0].scan_type =
				MLAN_SCAN_TYPE_PASSIVE;
		else if (conn_param->channel->flags & IEEE80211_CHAN_RADAR)
			scan_req->chan_list[0].scan_type =
				MLAN_SCAN_TYPE_PASSIVE_TO_ACTIVE;
		else
			scan_req->chan_list[0].scan_type =
				MLAN_SCAN_TYPE_ACTIVE;
		scan_req->chan_list[0].scan_time = 0;
	} else {
		for (band = 0; (band < IEEE80211_NUM_BANDS); band++) {
			if (!priv->wdev->wiphy->bands[band])
				continue;
			sband = priv->wdev->wiphy->bands[band];
			for (i = 0; (i < sband->n_channels); i++) {
				ch = &sband->channels[i];
				if (ch->flags & IEEE80211_CHAN_DISABLED)
					continue;
				scan_req->chan_list[chan_idx].radio_type =
					woal_ieee_band_to_radio_type(band);
				if (ch->flags & IEEE80211_CHAN_PASSIVE_SCAN)
					scan_req->chan_list[chan_idx].scan_type =
						MLAN_SCAN_TYPE_PASSIVE;
				else if (ch->flags & IEEE80211_CHAN_RADAR)
					scan_req->chan_list[chan_idx].scan_type =
						MLAN_SCAN_TYPE_PASSIVE_TO_ACTIVE;
				else
					scan_req->chan_list[chan_idx].scan_type =
						MLAN_SCAN_TYPE_ACTIVE;
				scan_req->chan_list[chan_idx].chan_number =
					(u32)ch->hw_value;
				chan_idx++;
			}
		}
	}
	moal_memcpy_ext(priv->phandle, scan_req->random_mac, priv->random_mac,
			ETH_ALEN, sizeof(scan_req->random_mac));
	ret = woal_request_userscan(priv, wait_option, scan_req);
	kfree(scan_req);
#ifdef REASSOCIATION
	MOAL_REL_SEMAPHORE(&handle->reassoc_sem);
#endif
	LEAVE();
	return ret;
}

/**
 *  @brief This function check if scan is allowed o/n specified band
 *
 *  @param priv     A pointer to moal_private structure
 *  @param chan     ieee80211_channel
 *
 *  @return           MTRUE/MFALSE
 */
static t_u8 is_scan_band_allowed(moal_private *priv,
				 struct ieee80211_channel *chan)
{
	t_u8 ret = MTRUE;
	t_u8 band_mask = 0;

	ENTER();
	if (!priv->scan_setband_mask) {
		LEAVE();
		return ret;
	}

	switch (chan->band) {
	case IEEE80211_BAND_5GHZ:
		band_mask = SCAN_SETBAND_5G;
		break;
	case IEEE80211_BAND_2GHZ:
		band_mask = SCAN_SETBAND_2G;
		break;
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 8, 0)
	case IEEE80211_BAND_6GHZ:
		band_mask = SCAN_SETBAND_6G;
		break;
#endif
	default:
		break;
	}

	if (band_mask & priv->scan_setband_mask) {
		ret = MTRUE;
	} else {
		PRINTM(MINFO, "is_scan_band_allowed: Avoid scan on band %d\n",
		       chan->band);
		ret = MFALSE;
	}

	LEAVE();
	return ret;
}

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
/**
 * @brief Save assoc parameters for roaming
 *
 * @param priv            A pointer to moal_private
 * @param req             A pointer to cfg80211_assoc_request structure
 */
static void woal_save_assoc_params(moal_private *priv,
				   struct cfg80211_assoc_request *req,
				   mlan_ssid_bssid *ssid_bssid)
{
	ENTER();

	priv->assoc_bss = req->bss;
	if (req->bss->channel) {
		priv->sme_current.channel = &priv->conn_chan;
		moal_memcpy_ext(priv->phandle, priv->sme_current.channel,
				req->bss->channel,
				sizeof(struct ieee80211_channel),
				sizeof(struct ieee80211_channel));
	}
	moal_memcpy_ext(priv->phandle, (void *)priv->conn_bssid,
			req->bss->bssid, MLAN_MAC_ADDR_LENGTH,
			MLAN_MAC_ADDR_LENGTH);
	priv->sme_current.bssid = priv->conn_bssid;
	if (req->ie && req->ie_len) {
		priv->sme_current.ie = kzalloc(req->ie_len, GFP_ATOMIC);
		if (!priv->sme_current.ie) {
			PRINTM(MERROR,
			       "Failed to allocate memory for sme params\n");
			LEAVE();
			return;
		}
		priv->sme_current.ie_len = req->ie_len;
		/* doing memcpy in memory allocated just above */
		// coverity[misra_c_2012_rule_11_8_violation:SUPPRESS]
		moal_memcpy_ext(priv->phandle, (void *)priv->sme_current.ie,
				req->ie, req->ie_len, priv->sme_current.ie_len);
	}
	moal_memcpy_ext(priv->phandle, &priv->sme_current.crypto, &req->crypto,
			sizeof(struct cfg80211_crypto_settings),
			sizeof(struct cfg80211_crypto_settings));
	priv->sme_current.flags = req->flags;
	moal_memcpy_ext(priv->phandle, &priv->sme_current.ht_capa,
			&req->ht_capa, sizeof(struct ieee80211_ht_cap),
			sizeof(struct ieee80211_ht_cap));
	moal_memcpy_ext(priv->phandle, &priv->sme_current.ht_capa_mask,
			&req->ht_capa_mask, sizeof(struct ieee80211_ht_cap),
			sizeof(struct ieee80211_ht_cap));
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
	moal_memcpy_ext(priv->phandle, &priv->sme_current.vht_capa,
			&req->vht_capa, sizeof(struct ieee80211_vht_cap),
			sizeof(struct ieee80211_vht_cap));
	moal_memcpy_ext(priv->phandle, &priv->sme_current.vht_capa_mask,
			&req->vht_capa_mask, sizeof(struct ieee80211_vht_cap),
			sizeof(struct ieee80211_vht_cap));
#endif
	if (ssid_bssid && ssid_bssid->ssid.ssid_len) {
		memset(priv->conn_ssid, 0, MLAN_MAX_SSID_LENGTH);
		moal_memcpy_ext(priv->phandle, (void *)priv->conn_ssid,
				ssid_bssid->ssid.ssid,
				ssid_bssid->ssid.ssid_len,
				sizeof(priv->conn_ssid));
		priv->sme_current.ssid = priv->conn_ssid;
		priv->sme_current.ssid_len = ssid_bssid->ssid.ssid_len;
		priv->conn_ssid_len = ssid_bssid->ssid.ssid_len;
	}
	if (priv->sinfo)
		memset(priv->sinfo, 0, sizeof(struct station_info));
	else
		priv->sinfo = kzalloc(sizeof(struct station_info), GFP_ATOMIC);
	LEAVE();
}

/**
 * @brief Save auth parameters for roaming
 *
 * @param priv            A pointer to moal_private
 * @param req             A pointer to struct cfg80211_auth_request
 */
static void woal_save_auth_params(moal_private *priv,
				  struct cfg80211_auth_request *req)
{
	ENTER();
	woal_clear_conn_params(priv);
	priv->assoc_bss = req->bss;
	priv->sme_current.auth_type = req->auth_type;
	priv->sme_current.key_idx = req->key_idx;
	priv->sme_current.key_len = req->key_len;
	if (req->key && req->key_len && (req->key_len <= MAX_WEP_KEY_SIZE)) {
		priv->sme_current.key = priv->conn_wep_key;
		// coverity[misra_c_2012_rule_11_8_violation:SUPPRESS]
		moal_memcpy_ext(priv->phandle, (t_u8 *)priv->sme_current.key,
				req->key, req->key_len,
				sizeof(priv->conn_wep_key));
	}
	LEAVE();
}

/**
 * @brief Request scan based on auth_request parameter
 *
 * @param priv            A pointer to moal_private structure
 * @param req             A pointer to cfg80211_auth_request
 * @param wait_option     wait option
 *
 * @return                0 -- success, otherwise fail
 */
static int woal_cfg80211_auth_scan(moal_private *priv,
				   struct cfg80211_auth_request *req,
				   t_u8 wait_option)
{
	moal_handle *handle = priv->phandle;
	int ret = 0;
	wlan_user_scan_cfg *scan_req;
	enum ieee80211_band band;
	struct ieee80211_supported_band *sband;
	struct ieee80211_channel *ch;
	int chan_idx = 0, i;
	const u8 *ssid;

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
	scan_req = (wlan_user_scan_cfg *)kmalloc(sizeof(wlan_user_scan_cfg),
						 GFP_KERNEL);
	if (!scan_req) {
		PRINTM(MERROR, "Malloc buffer failed\n");
		LEAVE();
		return -ENOMEM;
	}

	priv->report_scan_result = MTRUE;
	memset(scan_req, 0x00, sizeof(wlan_user_scan_cfg));
	rcu_read_lock();
	ssid = ieee80211_bss_get_ie(req->bss, WLAN_EID_SSID);
	if (ssid) {
		moal_memcpy_ext(priv->phandle, scan_req->ssid_list[0].ssid,
				ssid + 2, ssid[1],
				sizeof(scan_req->ssid_list[0].ssid));
		scan_req->ssid_list[0].max_len = 0;
	}
	rcu_read_unlock();
	moal_memcpy_ext(priv->phandle, scan_req->specific_bssid,
			req->bss->bssid, ETH_ALEN, ETH_ALEN);
	if (req->bss->channel) {
		scan_req->chan_list[0].chan_number =
			req->bss->channel->hw_value;
		scan_req->chan_list[0].radio_type =
			woal_ieee_band_to_radio_type(req->bss->channel->band);
		if (req->bss->channel->flags & IEEE80211_CHAN_PASSIVE_SCAN)
			scan_req->chan_list[0].scan_type =
				MLAN_SCAN_TYPE_PASSIVE;
		else if (req->bss->channel->flags & IEEE80211_CHAN_RADAR)
			scan_req->chan_list[0].scan_type =
				MLAN_SCAN_TYPE_PASSIVE_TO_ACTIVE;
		else
			scan_req->chan_list[0].scan_type =
				MLAN_SCAN_TYPE_ACTIVE;
		scan_req->chan_list[0].scan_time = 0;
	} else {
		for (band = 0; (band < IEEE80211_NUM_BANDS); band++) {
			if (!priv->wdev->wiphy->bands[band])
				continue;
			sband = priv->wdev->wiphy->bands[band];
			for (i = 0; (i < sband->n_channels); i++) {
				ch = &sband->channels[i];
				if (ch->flags & IEEE80211_CHAN_DISABLED)
					continue;
				scan_req->chan_list[chan_idx].radio_type =
					woal_ieee_band_to_radio_type(band);
				if (ch->flags & IEEE80211_CHAN_PASSIVE_SCAN)
					scan_req->chan_list[chan_idx].scan_type =
						MLAN_SCAN_TYPE_PASSIVE;
				else if (ch->flags & IEEE80211_CHAN_RADAR)
					scan_req->chan_list[chan_idx].scan_type =
						MLAN_SCAN_TYPE_PASSIVE_TO_ACTIVE;
				else
					scan_req->chan_list[chan_idx].scan_type =
						MLAN_SCAN_TYPE_ACTIVE;
				scan_req->chan_list[chan_idx].chan_number =
					(u32)ch->hw_value;
				chan_idx++;
			}
		}
	}
	moal_memcpy_ext(priv->phandle, scan_req->random_mac, priv->random_mac,
			ETH_ALEN, sizeof(scan_req->random_mac));
	ret = woal_request_userscan(priv, wait_option, scan_req);
	kfree(scan_req);
#ifdef REASSOCIATION
	MOAL_REL_SEMAPHORE(&handle->reassoc_sem);
#endif
	LEAVE();
	return ret;
}

/**
 *  @brief Send set host_mlme request to MLAN
 *
 *  @param priv   A pointer to moal_private structure
 *
 *  @return       MLAN_STATUS_SUCCESS/MLAN_STATUS_PENDING
 *                  -- success, otherwise fail
 */
mlan_status woal_request_set_host_mlme(moal_private *priv, t_u8 *bssid)
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
	bss->sub_command = MLAN_OID_BSS_HOST_MLME;
	req->req_id = MLAN_IOCTL_BSS;
	req->action = MLAN_ACT_SET;
	if (bssid) {
		moal_memcpy_ext(priv->phandle, &bss->param.bssid, bssid,
				MLAN_MAC_ADDR_LENGTH, MLAN_MAC_ADDR_LENGTH);
	}
	/* Send IOCTL request to MLAN */
	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return status;
}

/**
 *  @brief This function calculates common rates supported
 *         by AP and STA and updates tx_control
 *         value.
 *
 *  @param dev         A pointer to net_device
 *
 *  @param req         A pointer to cfg80211_auth_request
 *
 *  @param tx_control  Output: Updated value of tx_control
 *
 *  @return            0 -- success, otherwise fail
 */

static mlan_status woal_get_common_rates(struct net_device *dev,
					 struct cfg80211_auth_request *req,
					 t_u32 *tx_control)
{
	moal_private *priv = (moal_private *)woal_get_netdev_priv(dev);
	mlan_scan_resp scan_resp;
	BSSDescriptor_t *scan_table;
	int i, k, j;
	t_u8 *prate = {0};
	t_u8 rateIndex = 0xff;
	t_u8 baserates[] = {0x82, 0x84, 0x8b, 0x96, 0x8c, 0x98, 0xb0};

	ENTER();
	memset(&scan_resp, 0, sizeof(scan_resp));
	if (MLAN_STATUS_SUCCESS !=
	    woal_get_scan_table(priv, MOAL_NO_WAIT, &scan_resp)) {
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}

	// Fetch supported rates of AP from scan table
	if (scan_resp.num_in_scan_table) {
		scan_table = (BSSDescriptor_t *)scan_resp.pscan_table;
		for (i = 0; i < (int)scan_resp.num_in_scan_table; i++) {
			if (req->bss) {
				if (!memcmp(req->bss->bssid,
					    scan_table[i].mac_address,
					    ETH_ALEN)) {
					prate = (t_u8 *)&(
						scan_table[i].supported_rates);
					break;
				}
			}
		}
	} else {
		PRINTM(MMSG, "bssid not found in scan list\n");
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}
	// Calculate rate index value for rate defined in baserates
	// which is supported by both AP and STA
	for (j = 0; (j < WLAN_SUPPORTED_RATES && prate[j] != 0); j++) {
		if (prate[j] >= 0x82) {
			for (k = 0; k < sizeof(baserates); k++) {
				if (prate[j] == baserates[k] && k < rateIndex)
					rateIndex = k;
			}
		}
	}
	if (rateIndex == 0xff) {
		PRINTM(MMSG, "AP and STA have no rates as common\n");
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}
	*tx_control |= 1 << 15;
	switch (baserates[rateIndex]) {
	case 0x82:
		*tx_control |= 0 << 16;
		break;
	case 0x84:
		*tx_control |= 1 << 16;
		break;
	case 0x8b:
		*tx_control |= 2 << 16;
		break;
	case 0x96:
		*tx_control |= 3 << 16;
		break;
	case 0x8c:
		*tx_control |= 5 << 16;
		break;
	case 0x98:
		*tx_control |= 7 << 16;
		break;
	case 0xb0:
		*tx_control |= 9 << 16;
		break;
	default:
		*tx_control = 0;
		PRINTM(MMSG, "Not support the base rates");
		break;
	}
	LEAVE();
	return MLAN_STATUS_SUCCESS;
}
/**
 *  @brief This function is authentication handler when host MLME
 *          enable.
 *          In this case driver will prepare and send Auth Req.
 *
 *  @param wiphy       A pointer to wiphy.
 *
 *  @param dev         A pointer to net_device
 *
 *  @param req         A pointer to cfg80211_auth_request
 *
 *  @return            0 -- success, otherwise fail
 */
static int woal_cfg80211_authenticate(struct wiphy *wiphy,
				      struct net_device *dev,
				      struct cfg80211_auth_request *req)
{
	moal_private *priv = (moal_private *)woal_get_netdev_priv(dev);
	IEEE80211_MGMT *mgmt = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;
	pmlan_buffer pmbuf = NULL;
	mlan_ioctl_req *ioctl_req = NULL;
	mlan_ds_misc_cfg *misc = NULL;
	t_u8 *pbuf = NULL;
	t_u32 pkt_type, tx_control;
	t_u16 packet_len = 0, auth_alg;
	t_u16 pkt_len;
	t_u8 addr[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
	int ret = 0;
	t_u8 trans = 1, status_code = 0;
	t_u8 *varptr = NULL;
	moal_handle *handle = priv->phandle;
	mlan_ssid_bssid *ssid_bssid;
	int i;

	ENTER();

	if (!is_scan_band_allowed(priv, req->bss->channel)) {
		LEAVE();
		return -EFAULT;
	}
#ifdef REASSOCIATION
	// disable reassoc_on
	handle->reassoc_on &= ~MBIT(priv->bss_index);
	priv->reassoc_on = MFALSE;
	priv->reassoc_required = MFALSE;
	if (!handle->reassoc_on && handle->is_reassoc_timer_set == MTRUE) {
		woal_cancel_timer(&handle->reassoc_timer);
		handle->is_reassoc_timer_set = MFALSE;
	}
#endif

	priv->cfg_disconnect = MFALSE;
	priv->delay_deauth_notify = MFALSE;
#ifdef UAP_CFG80211
	if (GET_BSS_ROLE(priv) == MLAN_BSS_ROLE_UAP) {
		PRINTM(MERROR, "ERR: Role is AP\n");
		LEAVE();
		return -EFAULT;
	}
#endif
	if (priv->wdev->iftype != NL80211_IFTYPE_STATION
#ifdef WIFI_DIRECT_SUPPORT
	    && priv->wdev->iftype != NL80211_IFTYPE_P2P_CLIENT

#endif /* WIFI_DIRECT_SUPPORT */
	) {
		PRINTM(MERROR,
		       "Received infra auth request when interface not in infra mode\n");
		LEAVE();
		return -EINVAL;
	}

	/** cancel pending scan */
	woal_cancel_scan(priv, MOAL_IOCTL_WAIT);

	ssid_bssid = kzalloc(sizeof(mlan_ssid_bssid), GFP_ATOMIC);
	if (!ssid_bssid) {
		PRINTM(MERROR, "Fail to allocate ssid_bssid buffer\n");
		LEAVE();
		return -ENOMEM;
	}
	moal_memcpy_ext(priv->phandle, ssid_bssid->bssid, req->bss->bssid,
			ETH_ALEN, sizeof(ssid_bssid->bssid));
	/* Not allowed to connect to the same AP which is already connected
	with other interface */
	for (i = 0; i < handle->priv_num; i++) {
		if (handle->priv[i] != priv &&
		    MTRUE == woal_is_connected(handle->priv[i], ssid_bssid)) {
			PRINTM(MMSG,
			       "wlan: already connected with other interface, bssid " MACSTR
			       "\n",
			       MAC2STR(handle->priv[i]->cfg_bssid));
			kfree(ssid_bssid);
			LEAVE();
			return -EINVAL;
		}
	}
	if (MLAN_STATUS_SUCCESS != woal_find_bssid(priv, req->bss->bssid)) {
		woal_cfg80211_auth_scan(priv, req, MOAL_IOCTL_WAIT);
		if (MLAN_STATUS_SUCCESS !=
		    woal_find_bssid(priv, req->bss->bssid)) {
			PRINTM(MMSG, "bssid not found in scan list\n");
			kfree(ssid_bssid);
			LEAVE();
			/* Supplicannt has provision to retry Auth,
			 * if error is returned as ENOENT.
			 * it does Auth scan on specific channel
			 * to sync cfg80211 and DRV scan table entry
			 */
			return -ENOENT;
		}
	}
	kfree(ssid_bssid);

	if (MLAN_STATUS_SUCCESS !=
	    woal_get_common_rates(dev, req, &tx_control)) {
		tx_control = 0;
	}
	if ((priv->auth_alg != WLAN_AUTH_SAE) &&
	    (priv->auth_flag & HOST_MLME_AUTH_PENDING)) {
		PRINTM(MERROR, "pending auth on going\n");
		LEAVE();
		return -EBUSY;
	}
#ifdef WIFI_DIRECT_SUPPORT
	if (priv->bss_type == MLAN_BSS_TYPE_WIFIDIRECT &&
	    (priv->wdev->iftype == NL80211_IFTYPE_STATION ||
	     priv->wdev->iftype == NL80211_IFTYPE_P2P_CLIENT)) {
		/* if bsstype == wifi direct, and iftype == station or p2p
		 * client, that means wpa_supplicant wants to enable wifi direct
		 * functionality, so we should init p2p client.
		 *
		 * Note that due to kernel iftype check, ICS wpa_supplicant
		 * could not updaet iftype to init p2p client, so we have to
		 * done it here.
		 * */
		if (MLAN_STATUS_SUCCESS !=
		    woal_cfg80211_init_p2p_client(priv)) {
			PRINTM(MERROR,
			       "Init p2p client for wpa_supplicant failed.\n");
			ret = -EFAULT;
			LEAVE();
			return ret;
		}
	}
	if (priv->bss_type == MLAN_BSS_TYPE_WIFIDIRECT) {
		/* WAR for P2P connection with vendor TV */
		woal_sched_timeout(200);
	}
#endif

	/*enable auth register frame*/
	if (priv->auth_flag == 0) {
		woal_mgmt_frame_register(priv, IEEE80211_STYPE_AUTH, MTRUE);
		woal_mgmt_frame_register(priv, IEEE80211_STYPE_DEAUTH, MTRUE);
		woal_mgmt_frame_register(priv, IEEE80211_STYPE_DISASSOC, MTRUE);
	}

	if (MLAN_STATUS_SUCCESS !=
	    woal_cfg80211_set_key(priv, 0, 0, NULL, 0, NULL, 0,
				  KEY_INDEX_CLEAR_ALL, NULL, 1, 0,
				  MOAL_IOCTL_WAIT)) {
		/* Disable keys and clear all previous security settings */
		PRINTM(MERROR, "Fail to clear previous keys\n");
		ret = -EFAULT;
		goto done;
	}

	switch (req->auth_type) {
	case NL80211_AUTHTYPE_OPEN_SYSTEM:
		auth_alg = WLAN_AUTH_OPEN;
		break;
	case NL80211_AUTHTYPE_SHARED_KEY:
		auth_alg = WLAN_AUTH_SHARED_KEY;
		break;
	case NL80211_AUTHTYPE_FT:
		auth_alg = WLAN_AUTH_FT;
		break;
	case NL80211_AUTHTYPE_NETWORK_EAP:
		auth_alg = WLAN_AUTH_LEAP;
		break;
	case NL80211_AUTHTYPE_SAE:
		auth_alg = WLAN_AUTH_SAE;
		break;
	default:
		PRINTM(MERROR, "Unsupported auth type=%d\n", req->auth_type);
		ret = -EOPNOTSUPP;
		break;
	}
	if (ret)
		goto done;
	if (MLAN_STATUS_SUCCESS !=
	    woal_set_auth_mode(priv, MOAL_IOCTL_WAIT, auth_alg)) {
		PRINTM(MERROR, "Fail to set auth mode\n");
		ret = -EFAULT;
		goto done;
	}

	if (req->key && ((auth_alg == WLAN_AUTH_OPEN) ||
			 (auth_alg == WLAN_AUTH_SHARED_KEY))) {
		PRINTM(MMSG, "Setting wep encryption with key len %d\n",
		       req->key_len);
		/* Set the WEP key */
		if (MLAN_STATUS_SUCCESS !=
		    woal_cfg80211_set_wep_keys(priv, req->key, req->key_len,
					       req->key_idx, MOAL_IOCTL_WAIT)) {
			PRINTM(MERROR, "Fail to set wep key idx %d\n",
			       req->key_idx);
			ret = -EFAULT;
			goto done;
		}
		/* Enable the WEP key by key index */
		if (MLAN_STATUS_SUCCESS !=
		    woal_cfg80211_set_wep_keys(priv, NULL, 0, req->key_idx,
					       MOAL_IOCTL_WAIT)) {
			PRINTM(MERROR, "Fail to enable wep key idx %d\n",
			       req->key_idx);
			ret = -EFAULT;
			goto done;
		}
	}

	if (priv->auth_flag == 0) {
		/*
		 * The priority of auth req is highest so we need to cancel
		 * current exist remain on channel for same function.
		 * EX: If wfd0 supplicant often scans, mlan0 is difficult to
		 * get the chance to request ROC.
		 * Risk: If exist one is another auth, it will be replaced
		 * with new auth ROC request.
		 * Avoid: if using woal_cfg80211_mgmt_tx_cancel_wait(), we
		 * cannot sure the result when the API reports cancel event
		 * to cfg80211 if priv->phandle->cookie != 0.
		 */
		if (priv->phandle->remain_on_channel) {
			moal_private *remain_priv = NULL;
			remain_priv =
				priv->phandle
					->priv[priv->phandle->remain_bss_index];
			if (!remain_priv) {
				/* cannot find its priv, weird! keep
				 * continuing... */
				PRINTM(MERROR,
				       "mgmt_tx_cancel_wait: Wrong remain_bss_index=%d\n",
				       priv->phandle->remain_bss_index);
			} else {
				if (woal_cfg80211_remain_on_channel_cfg(
					    remain_priv, MOAL_IOCTL_WAIT, MTRUE,
					    (t_u8 *)&status, NULL, 0, 0)) {
					/* fail to cancel current one! keep
					 * continuing... */
					PRINTM(MERROR,
					       "mgmt_tx_cancel_wait: Fail to cancel remain on channel\n");
				} else {
					/* only cancel is successfully then
					 * change the flag */
					priv->phandle->remain_on_channel =
						MFALSE;
				}
			}
		}

		if (woal_cfg80211_remain_on_channel_cfg(
			    priv, MOAL_IOCTL_WAIT, MFALSE, (t_u8 *)&status,
			    req->bss->channel, 0, priv->auth_tx_wait_time)) {
			PRINTM(MERROR, "Fail to configure remain on channel\n");
			ret = -EFAULT;
			goto done;
		}
		if (status == MLAN_STATUS_SUCCESS) {
			priv->phandle->remain_on_channel = MTRUE;
			moal_memcpy_ext(priv->phandle, &(priv->phandle->chan),
					req->bss->channel,
					sizeof(struct ieee80211_channel),
					sizeof(priv->phandle->chan));
		} else {
			PRINTM(MERROR,
			       "HostMlme %s: Set remain on Channel: with status=%d\n",
			       dev->name, status);
		}
	}
#define HEADER_SIZE 8
	// frmctl + durationid + addr1 + addr2 + addr3 + seqctl + addr4
#define MGMT_HEADER_LEN (2 + 2 + 6 + 6 + 6 + 2 + 6)
	// 6   = auth_alg + auth_transaction +auth_status
#define AUTH_BODY_LEN 6
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 10, 0)
	packet_len = (t_u16)req->ie_len + req->auth_data_len + MGMT_HEADER_LEN +
		     AUTH_BODY_LEN;
#else
	packet_len = (t_u16)req->ie_len + req->sae_data_len + MGMT_HEADER_LEN +
		     AUTH_BODY_LEN;
#endif
	if (priv->phandle->cmd_tx_data) {
		ioctl_req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
		if (ioctl_req == NULL) {
			ret = -ENOMEM;
			goto done;
		}
		misc = (mlan_ds_misc_cfg *)ioctl_req->pbuf;
		misc->sub_command = MLAN_OID_MISC_TX_FRAME;
		ioctl_req->req_id = MLAN_IOCTL_MISC_CFG;
		ioctl_req->action = MLAN_ACT_SET;
		misc->param.tx_frame.bandcfg.chanBand =
			woal_ieee_band_to_radio_type(req->bss->channel->band);
		misc->param.tx_frame.channel = req->bss->channel->hw_value;
		pbuf = misc->param.tx_frame.tx_buf;
	} else {
		pmbuf = woal_alloc_mlan_buffer(
			priv->phandle, MLAN_MIN_DATA_HEADER_LEN + HEADER_SIZE +
					       packet_len + sizeof(packet_len));

		if (!pmbuf) {
			PRINTM(MERROR, "Fail to allocate mlan_buffer\n");
			ret = -ENOMEM;
			goto done;
		}
		pmbuf->data_offset = MLAN_MIN_DATA_HEADER_LEN;
		pbuf = pmbuf->pbuf + pmbuf->data_offset;
	}
	pkt_type = MRVL_PKT_TYPE_MGMT_FRAME;
	/* Add pkt_type and tx_control */
	moal_memcpy_ext(priv->phandle, pbuf, &pkt_type, sizeof(pkt_type),
			sizeof(pkt_type));
	moal_memcpy_ext(priv->phandle, pbuf + sizeof(pkt_type), &tx_control,
			sizeof(tx_control), sizeof(tx_control));

	mgmt = (IEEE80211_MGMT *)(pbuf + HEADER_SIZE + sizeof(packet_len));
	memset(mgmt, 0, MGMT_HEADER_LEN);
	/**Authentication Frame: Frame Control*/
	mgmt->frame_control =
		woal_cpu_to_le16(IEEE80211_FTYPE_MGMT | IEEE80211_STYPE_AUTH);
	/**Authentication Frame: Destination Address*/
	moal_memcpy_ext(priv->phandle, mgmt->da, req->bss->bssid, ETH_ALEN,
			sizeof(mgmt->da));
	/**Authentication Frame: Source Address*/
	moal_memcpy_ext(priv->phandle, mgmt->sa, priv->current_addr, ETH_ALEN,
			sizeof(mgmt->sa));
	/**Authentication Frame: BSSID*/
	moal_memcpy_ext(priv->phandle, mgmt->bssid, req->bss->bssid, ETH_ALEN,
			sizeof(mgmt->bssid));
	moal_memcpy_ext(priv->phandle, mgmt->addr4, addr, ETH_ALEN,
			sizeof(mgmt->addr4));

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 10, 0)
	if (req->auth_data_len >= 4) {
		if (req->auth_type == NL80211_AUTHTYPE_SAE) {
			const __le16 *pos = (const __le16 *)req->auth_data;

			trans = le16_to_cpu(pos[0]);
			status_code = le16_to_cpu(pos[1]);
		}
		moal_memcpy_ext(priv->phandle, (t_u8 *)(&mgmt->u.auth.variable),
				req->auth_data + 4, req->auth_data_len - 4,
				req->auth_data_len - 4);
		varptr = (t_u8 *)&mgmt->u.auth.variable +
			 (req->auth_data_len - 4);
		packet_len -= 4;
	}
#else
	if (req->sae_data_len >= 4) {
		if (req->auth_type == NL80211_AUTHTYPE_SAE) {
			__le16 *pos = (__le16 *)req->sae_data;

			trans = le16_to_cpu(pos[0]);
			status_code = le16_to_cpu(pos[1]);
		}
		moal_memcpy_ext(priv->phandle, (t_u8 *)(&mgmt->u.auth.variable),
				req->sae_data + 4, req->sae_data_len - 4,
				req->sae_data_len - 4);
		varptr = (t_u8 *)&mgmt->u.auth.variable +
			 (req->sae_data_len - 4);
		packet_len -= 4;
	}
#endif
	/*Add packet len*/
	pkt_len = woal_cpu_to_le16(packet_len);
	moal_memcpy_ext(priv->phandle, pbuf + HEADER_SIZE, &pkt_len,
			sizeof(pkt_len), sizeof(pkt_len));

	/**Authentication Frame: Authentication Alg*/
	mgmt->u.auth.auth_alg = woal_cpu_to_le16(auth_alg);
	mgmt->u.auth.auth_transaction = woal_cpu_to_le16(trans);
	/**Authentication Frame: Status code*/
	mgmt->u.auth.status_code = woal_cpu_to_le16(status_code);

	if (req->ie && req->ie_len) {
		if (!varptr) {
			varptr = (t_u8 *)&mgmt->u.auth.variable;
		}
		moal_memcpy_ext(priv->phandle, (t_u8 *)varptr, req->ie,
				req->ie_len, req->ie_len);
	}

	priv->host_mlme = MTRUE;
	priv->auth_flag = HOST_MLME_AUTH_PENDING;
	priv->auth_alg = woal_cpu_to_le16(auth_alg);
	woal_save_auth_params(priv, req);
	woal_request_set_host_mlme(priv, req->bss->bssid);

	PRINTM(MMSG, "wlan: HostMlme %s send auth to bssid " MACSTR "\n",
	       dev->name, MAC2STR(req->bss->bssid));
	DBG_HEXDUMP(MDAT_D, "Auth:", pbuf,
		    HEADER_SIZE + packet_len + sizeof(packet_len));
	if (priv->phandle->cmd_tx_data) {
		misc->param.tx_frame.data_len =
			HEADER_SIZE + packet_len + sizeof(packet_len);
		misc->param.tx_frame.buf_type = MLAN_BUF_TYPE_RAW_DATA;
		misc->param.tx_frame.priority = 7;
		status = woal_request_ioctl(priv, ioctl_req, MOAL_IOCTL_WAIT);
		if (status != MLAN_STATUS_SUCCESS) {
			priv->host_mlme = MFALSE;
			priv->auth_flag = 0;
			priv->auth_alg = 0xFFFF;
			ret = -EFAULT;
			PRINTM(MERROR, "Fail to send packet status=%d\n",
			       status);
		}
	} else {
		pmbuf->data_len = HEADER_SIZE + packet_len + sizeof(packet_len);
		pmbuf->buf_type = MLAN_BUF_TYPE_RAW_DATA;
		pmbuf->bss_index = priv->bss_index;
		pmbuf->priority = 7;

		status = mlan_send_packet(priv->phandle->pmlan_adapter, pmbuf);

		switch (status) {
		case MLAN_STATUS_PENDING:
			atomic_inc(&priv->phandle->tx_pending);
			queue_work(priv->phandle->workqueue,
				   &priv->phandle->main_work);
#define MAX_AUTH_COUNTER 5
			priv->auth_tx_cnt++;
			if (priv->auth_tx_cnt >= MAX_AUTH_COUNTER) {
				if (woal_reset_wifi(priv->phandle,
						    priv->auth_tx_cnt,
						    "auth timeout") ==
				    MLAN_STATUS_SUCCESS) {
					priv->auth_tx_cnt = 0;
				}
			}
			break;
		case MLAN_STATUS_SUCCESS:
			woal_free_mlan_buffer(priv->phandle, pmbuf);
			break;
		case MLAN_STATUS_FAILURE:
		default:
			woal_free_mlan_buffer(priv->phandle, pmbuf);
			priv->host_mlme = MFALSE;
			priv->auth_flag = 0;
			priv->auth_alg = 0xFFFF;
			ret = -EFAULT;
			PRINTM(MERROR, "Fail to send packet status=%d\n",
			       status);
			break;
		}
	}
done:
	if (priv->phandle->cmd_tx_data) {
		if (status != MLAN_STATUS_PENDING)
			kfree(ioctl_req);
	}
	if (ret) {
		woal_mgmt_frame_register(priv, IEEE80211_STYPE_AUTH, MFALSE);
		if (priv->phandle->remain_on_channel) {
			if (woal_cfg80211_remain_on_channel_cfg(
				    priv, MOAL_IOCTL_WAIT, MTRUE,
				    (t_u8 *)&status, NULL, 0, 0)) {
				PRINTM(MERROR,
				       "Fail to cancel remain on channel\n");
			}
			priv->phandle->remain_on_channel = MFALSE;
		}
	}
	LEAVE();
	return ret;
}

/**
 *  @brief This workqueue function handles association response in host mlme
 * case
 *
 *  @param work    A pointer to work_struct
 *
 *  @return        N/A
 */
void woal_host_mlme_work_queue(struct work_struct *work)
{
	// Coverity violation raised for kernel's API
	// coverity[cert_arr39_c_violation:SUPPRESS]
	moal_handle *handle = container_of(work, moal_handle, host_mlme_work);
	moal_private *priv = (moal_private *)handle->host_mlme_priv;
	mlan_status status = MLAN_STATUS_SUCCESS;

	if (priv) {
		if (priv->auth_flag & HOST_MLME_AUTH_DONE) {
			priv->auth_flag = 0;
			woal_mgmt_frame_register(priv, IEEE80211_STYPE_AUTH,
						 MFALSE);

			if (priv->phandle->remain_on_channel) {
				if (woal_cfg80211_remain_on_channel_cfg(
					    priv, MOAL_IOCTL_WAIT, MTRUE,
					    (t_u8 *)&status, NULL, 0, 0)) {
					PRINTM(MERROR,
					       "failed to cancel remain on channel\n");
				}
				priv->phandle->remain_on_channel = MFALSE;
			}
			PRINTM(MCMND, "wlan: HostMlme %s auth success\n",
			       priv->netdev->name);
		}
	}
}

/**
 *  @brief This workqueue function handles association timeout event in event
 * queue case
 *
 *  @param priv         pointer to moal_private
 *  @param assoc_info   pointer to cfg80211_bss
 *
 *  @return        N/A
 */

void woal_host_mlme_process_assoc_timeout(moal_private *priv,
					  struct cfg80211_bss *bss)
{
#if (CFG80211_VERSION_CODE >= KERNEL_VERSION(6, 0, 0) ||                       \
     (defined(ANDROID_SDK_VERSION) && ANDROID_SDK_VERSION >= 33 &&             \
      CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 15, 74)))
	struct cfg80211_assoc_failure data;
#endif
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 15, 0)
	mlan_ds_assoc_info *assoc_info;
	struct cfg80211_roam_info roam_info;
	chan_band_info channel;

	assoc_info = kzalloc(sizeof(mlan_ds_assoc_info), GFP_ATOMIC);
	if (!assoc_info)
		return;
	memset(&roam_info, 0, sizeof(roam_info));
	memset(&channel, 0, sizeof(channel));

	woal_inform_bss_from_scan_result(priv, NULL, MOAL_IOCTL_WAIT);
	woal_get_prev_assoc_info(priv, assoc_info, MOAL_IOCTL_WAIT);

	/* Fallback to prior AP */
	if (priv->media_connected && priv->phandle->params.make_before_break &&
	    priv->sme_current.auth_type != NL80211_AUTHTYPE_SHARED_KEY &&
	    priv->sme_current.auth_type != NL80211_AUTHTYPE_FT &&
	    assoc_info->assoc_req_len && assoc_info->assoc_resp_len) {
#if ((CFG80211_VERSION_CODE >= KERNEL_VERSION(6, 0, 0)) ||                     \
     (defined(ANDROID_SDK_VERSION) && ANDROID_SDK_VERSION >= 31))
		roam_info.links[0].bssid = assoc_info->bssid;
#else
		roam_info.bssid = assoc_info->bssid;
#endif
		roam_info.req_ie = assoc_info->assoc_req_buf;
		roam_info.req_ie_len = assoc_info->assoc_req_len;
		roam_info.resp_ie = assoc_info->assoc_resp_buf;
		roam_info.resp_ie_len = assoc_info->assoc_resp_len;

		cfg80211_roamed(priv->netdev, &roam_info, GFP_KERNEL);
		priv->cfg_disconnect = MFALSE;
		priv->host_mlme = MTRUE;
		priv->auth_flag |= HOST_MLME_ASSOC_DONE;
		moal_memcpy_ext(priv->phandle, priv->cfg_bssid,
				assoc_info->bssid, MLAN_MAC_ADDR_LENGTH,
				MLAN_MAC_ADDR_LENGTH);

		if (MLAN_STATUS_SUCCESS !=
		    woal_get_sta_channel(priv, MOAL_IOCTL_WAIT, &channel)) {
			PRINTM(MERROR,
			       "Assoc timeout:get sta channel failed\n");
		} else {
			if (MLAN_STATUS_FAILURE ==
			    woal_chandef_create(priv, &priv->chan, &channel)) {
				PRINTM(MERROR,
				       "Asssoc timeout:create chandef failed\n");
			}
			priv->channel = channel.channel;
			priv->bandwidth = channel.bandcfg.chanWidth;
		}
		PRINTM(MMSG, "wlan: HostMlme fallback to AP " MACSTR "\n",
		       MAC2STR(assoc_info->bssid));
		kfree(assoc_info);
		return;
	}
	kfree(assoc_info);
#endif
	/* Send Assoc Failure with Timeout to CFG80211 */
	priv->host_mlme = MFALSE;
	priv->auth_flag = 0;
#if (CFG80211_VERSION_CODE >= KERNEL_VERSION(6, 0, 0) ||                       \
     (defined(ANDROID_SDK_VERSION) && ANDROID_SDK_VERSION >= 33 &&             \
      CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 15, 74)))
	memset(&data, 0, sizeof(struct cfg80211_assoc_failure));
	data.timeout = 1;
	data.bss[0] = bss;
	PRINTM(MEVENT, "wlan: HostMlme assoc failure\n");
	/*
	 * 100ms delay to report to CFG to sync redv_assoc and assoc_failure
	 * API in CFG to hold/unhold BSS.
	 */
	woal_sched_timeout(100);
	cfg80211_assoc_failure(priv->netdev, &data);
#else
	PRINTM(MEVENT, "wlan: HostMlme assoc timeout\n");
	woal_sched_timeout(100);
	cfg80211_assoc_timeout(priv->netdev, bss);
#endif
	memset(priv->cfg_bssid, 0, ETH_ALEN);
	woal_clear_conn_params(priv);
}

/**
 *  @brief This workqueue function handles association response in event queue
 * case
 *
 *  @param priv  	pointer to moal_private
 *  @param assoc_info	pointer to mlan_ds_assoc_info
 *
 *  @return        N/A
 */
void woal_host_mlme_process_assoc_resp(moal_private *priv,
				       mlan_ds_assoc_info *assoc_info)
{
	struct cfg80211_bss *bss = NULL;
	unsigned long flags;
	t_u8 qos_cfg = 0;
	int uapsd_queues = -1;
	u8 *assoc_req_buf = NULL;
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(6, 7, 0)
	struct cfg80211_rx_assoc_resp_data resp = {
		.uapsd_queues = -1,
	};
#elif ((CFG80211_VERSION_CODE >= KERNEL_VERSION(6, 0, 0)) ||                   \
       (defined(ANDROID_SDK_VERSION) && ANDROID_SDK_VERSION >= 34))
	struct cfg80211_rx_assoc_resp resp = {
		.uapsd_queues = -1,
	};
#endif

	if (priv) {
		if (priv->auth_flag & HOST_MLME_ASSOC_DONE) {
			priv->auth_flag = 0;
			bss = priv->assoc_bss;
			if (!bss) {
				PRINTM(MERROR,
				       "HostMlme %s:assoc_bss is null\n",
				       priv->netdev->name);
				return;
			}

			if (assoc_info->assoc_resp_len) {
				PRINTM(MCMND,
				       "HostMlme: %s assoc_resp_len=%d, frame_control=0x%x\n",
				       priv->netdev->name,
				       assoc_info->assoc_resp_len,
				       ((struct ieee80211_mgmt *)
						assoc_info->assoc_resp_buf)
					       ->frame_control);
				if (ieee80211_is_assoc_resp(
					    ((struct ieee80211_mgmt *)
						     assoc_info->assoc_resp_buf)
						    ->frame_control) ||
				    ieee80211_is_reassoc_resp(
					    ((struct ieee80211_mgmt *)
						     assoc_info->assoc_resp_buf)
						    ->frame_control)) {
					spin_lock_irqsave(&priv->connect_lock,
							  flags);
					if (le16_to_cpu(
						    ((struct ieee80211_mgmt
							      *)assoc_info
							     ->assoc_resp_buf)
							    ->u.assoc_resp
							    .status_code) !=
					    WLAN_STATUS_SUCCESS) {
						memset(priv->cfg_bssid, 0,
						       ETH_ALEN);
						woal_clear_conn_params(priv);
					} else {
						priv->cfg_disconnect = MFALSE;
					}
					spin_unlock_irqrestore(
						&priv->connect_lock, flags);
					/*Populate Assoc req buf only if len is
					 * non zero . i.e. we received assoc req
					 * buffer from fw.*/
					if (assoc_info->assoc_req_len)
						assoc_req_buf =
							assoc_info
								->assoc_req_buf;
					if (!woal_priv_qos_cfg(priv,
							       MLAN_ACT_GET,
							       &qos_cfg) &&
					    qos_cfg != 0) {
						int ac;
						uapsd_queues = 0;
						for (ac = WMM_AC_BK;
						     ac <= WMM_AC_VO; ac++) {
							if (qos_cfg &
							    MBIT(WMM_AC_VO -
								 ac))
								uapsd_queues |=
									MBIT(ac);
						}
					}

#if ((CFG80211_VERSION_CODE >= KERNEL_VERSION(6, 0, 0)) ||                     \
     (defined(ANDROID_SDK_VERSION) && ANDROID_SDK_VERSION >= 34))
					resp.links[0].bss = bss;
					resp.buf = assoc_info->assoc_resp_buf;
					resp.len = assoc_info->assoc_resp_len;
					resp.req_ies = assoc_req_buf;
					resp.req_ies_len =
						assoc_info->assoc_req_len;
					resp.uapsd_queues = uapsd_queues;
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(6, 7, 0)
					wiphy_lock(priv->wdev->wiphy);
					cfg80211_rx_assoc_resp(priv->netdev,
							       &resp);
					wiphy_unlock(priv->wdev->wiphy);
#else
					mutex_lock(&priv->wdev->mtx);
					cfg80211_rx_assoc_resp(priv->netdev,
							       &resp);
					mutex_unlock(&priv->wdev->mtx);
#endif
#else
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 1, 0)
					mutex_lock(&priv->wdev->mtx);
					cfg80211_rx_assoc_resp(
						priv->netdev, bss,
						assoc_info->assoc_resp_buf,
						assoc_info->assoc_resp_len,
						uapsd_queues, assoc_req_buf,
						assoc_info->assoc_req_len);
					mutex_unlock(&priv->wdev->mtx);
#else
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 18, 0)
					mutex_lock(&priv->wdev->mtx);
					cfg80211_rx_assoc_resp(
						priv->netdev, bss,
						assoc_info->assoc_resp_buf,
						assoc_info->assoc_resp_len,
						uapsd_queues);
					mutex_unlock(&priv->wdev->mtx);
#else
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 11, 0)
					mutex_lock(&priv->wdev->mtx);
					cfg80211_rx_assoc_resp(
						priv->netdev, bss,
						assoc_info->assoc_resp_buf,
						assoc_info->assoc_resp_len);
					mutex_unlock(&priv->wdev->mtx);
#else
					cfg80211_send_rx_assoc(
						priv->netdev, bss,
						assoc_info->assoc_resp_buf,
						assoc_info->assoc_resp_len);
#endif
#endif
#endif
#endif
				}
			}
		}
	}
}

/**
 * @brief   Handle assoc timeout event (When AP do not respond to (Re)Assoc
 * Request)
 *
 * @param priv          A pointer moal_private structure
 * @param pchan_info    A pointer to cfg80211_assoc_request structre
 *
 * @return          N/A
 */

static void woal_assoc_timeout_event(moal_private *priv,
				     struct cfg80211_assoc_request *req)
{
	struct woal_event *evt;
	unsigned long flags;
	moal_handle *handle = priv->phandle;
	evt = kzalloc(sizeof(struct woal_event), GFP_ATOMIC);
	if (evt) {
		evt->priv = priv;
		evt->type = WOAL_EVENT_ASSOC_TIMEOUT;
		evt->assoc_bss = req->bss;
		INIT_LIST_HEAD(&evt->link);
		spin_lock_irqsave(&handle->evt_lock, flags);
		list_add_tail(&evt->link, &handle->evt_queue);
		spin_unlock_irqrestore(&handle->evt_lock, flags);
		queue_work(handle->evt_workqueue, &handle->evt_work);
	}
	// coverity[leaked_storage:SUPPRESS]
}

/**
 * @brief   Handle assoc response event
 *
 * @param priv          A pointer moal_private structure
 * @param pchan_info    A pointer to mlan_ds_misc_assoc_rsp structure
 *
 * @return          N/A
 */

static void woal_assoc_resp_event(moal_private *priv,
				  mlan_ds_misc_assoc_rsp *passoc_rsp)
{
	struct woal_event *evt;
	unsigned long flags;
	moal_handle *handle = priv->phandle;
	mlan_ds_misc_assoc_req *assoc_req = NULL;

	assoc_req = kzalloc(sizeof(mlan_ds_misc_assoc_req), GFP_ATOMIC);
	if (!assoc_req) {
		PRINTM(MERROR,
		       "Fail to allocate mlan_ds_misc_assoc_req buffer\n");
		return;
	}

	woal_get_assoc_req(priv, assoc_req, MOAL_IOCTL_WAIT);

	evt = kzalloc(sizeof(struct woal_event), GFP_ATOMIC);
	if (evt) {
		evt->priv = priv;
		evt->type = WOAL_EVENT_ASSOC_RESP;
		moal_memcpy_ext(priv->phandle, evt->assoc_info.assoc_resp_buf,
				passoc_rsp->assoc_resp_buf,
				passoc_rsp->assoc_resp_len, ASSOC_RSP_BUF_SIZE);
		evt->assoc_info.assoc_resp_len =
			MIN(passoc_rsp->assoc_resp_len, ASSOC_RSP_BUF_SIZE);
		moal_memcpy_ext(priv->phandle, evt->assoc_info.assoc_req_buf,
				assoc_req->assoc_req_buf,
				assoc_req->assoc_req_len, ASSOC_RSP_BUF_SIZE);
		evt->assoc_info.assoc_req_len =
			MIN(assoc_req->assoc_req_len, ASSOC_RSP_BUF_SIZE);

		INIT_LIST_HEAD(&evt->link);
		spin_lock_irqsave(&handle->evt_lock, flags);
		list_add_tail(&evt->link, &handle->evt_queue);
		spin_unlock_irqrestore(&handle->evt_lock, flags);
		queue_work(handle->evt_workqueue, &handle->evt_work);
	}
	kfree(assoc_req);
	return;
}

/**
 *  @brief This function is association handler when host MLME
 *          enable.
 *          In this case driver will prepare and send Assoc Req.
 *
 *  @param wiphy       A pointer to wiphy.
 *
 *  @param dev         A pointer to net_device
 *
 *  @param req         A pointer to cfg80211_assoc_request
 *
 *  @return            0 -- success, otherwise fail
 */
static int woal_cfg80211_associate(struct wiphy *wiphy, struct net_device *dev,
				   struct cfg80211_assoc_request *req)
{
	moal_private *priv = (moal_private *)woal_get_netdev_priv(dev);
	int ret = 0;
	int ie_len;
	mlan_ssid_bssid *ssid_bssid = NULL;
	unsigned long flags;
	const u8 *ssid_ie;
	int wpa_enabled = 0, group_enc_mode = 0, pairwise_enc_mode = 0;
	mlan_bss_info bss_info;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();
	if (!priv->auth_flag && !priv->host_mlme) {
		PRINTM(MINFO,
		       "wlan: HostMlme %s can not proceed with this Assoc, as Auth is not done yet \n",
		       priv->netdev->name);
		LEAVE();
		return -EFAULT;
	}

	priv->cfg_disconnect = MFALSE;

	ssid_bssid = kmalloc(sizeof(mlan_ssid_bssid), GFP_KERNEL);
	if (!ssid_bssid) {
		LEAVE();
		return -EFAULT;
	}

	if (priv->auth_alg == WLAN_AUTH_SAE) {
		priv->auth_flag = HOST_MLME_AUTH_DONE;

		woal_mgmt_frame_register(priv, IEEE80211_STYPE_AUTH, MFALSE);
		PRINTM(MINFO, "wlan: HostMlme %s auth exchange successful\n",
		       priv->netdev->name);

		if (priv->phandle->remain_on_channel) {
			if (woal_cfg80211_remain_on_channel_cfg(
				    priv, MOAL_IOCTL_WAIT, MTRUE,
				    (t_u8 *)&status, NULL, 0, 0)) {
				PRINTM(MERROR,
				       "Failed to cancel remain on channel\n");
				ret = -EFAULT;
				goto done;
			}
			priv->phandle->remain_on_channel = MFALSE;
		}
	}

	if (priv->auth_flag && !(priv->auth_flag & HOST_MLME_AUTH_DONE)) {
		kfree(ssid_bssid);
		LEAVE();
		return -EBUSY;
	}

	if (!req || !req->bss) {
		ret = -EINVAL;
		goto done;
	}

	/** cancel pending scan */
	woal_cancel_scan(priv, MOAL_IOCTL_WAIT);

	priv->cfg_connect = MTRUE;
	priv->assoc_status = 0;
	priv->auth_alg = 0xFFFF;

	memset(ssid_bssid, 0, sizeof(mlan_ssid_bssid));
	rcu_read_lock();

	ssid_ie = ieee80211_bss_get_ie(req->bss, WLAN_EID_SSID);
	moal_memcpy_ext(priv->phandle, ssid_bssid->bssid, req->bss->bssid,
			ETH_ALEN, sizeof(ssid_bssid->bssid));

	if (!ssid_ie) {
		rcu_read_unlock();
		ret = -EINVAL;
		goto done;
	}

	moal_memcpy_ext(priv->phandle, ssid_bssid->ssid.ssid, ssid_ie + 2,
			ssid_ie[1], sizeof(ssid_bssid->ssid.ssid));
	ssid_bssid->ssid.ssid_len = ssid_ie[1];
	rcu_read_unlock();

	if (ssid_bssid->ssid.ssid_len > MW_ESSID_MAX_SIZE) {
		PRINTM(MERROR, "Invalid SSID - aborting\n");
		ret = -EINVAL;
		goto done;
	}

	if (!ssid_bssid->ssid.ssid_len) {
		PRINTM(MERROR, "Invalid SSID - aborting\n");
		ret = -EINVAL;
		goto done;
	}

	if (req->bss) {
		if ((!priv->phandle->params.reg_alpha2 ||
		     strncmp(priv->phandle->params.reg_alpha2, "99",
			     strlen("99")))
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 14, 0)
		    &&
		    (!moal_extflg_isset(priv->phandle, EXT_COUNTRY_IE_IGNORE))
#endif
		)
			woal_process_country_ie(priv, req->bss);
	}

#ifdef STA_WEXT
	if (IS_STA_WEXT(priv->phandle->params.cfg80211_wext)) {
		switch (req->crypto.wpa_versions) {
		case NL80211_WPA_VERSION_2:
			priv->wpa_version = IW_AUTH_WPA_VERSION_WPA2;
			break;
		case NL80211_WPA_VERSION_1:
			priv->wpa_version = IW_AUTH_WPA_VERSION_WPA;
			break;
		default:
			priv->wpa_version = 0;
			break;
		}
		if (req->crypto.n_akm_suites) {
			switch (req->crypto.akm_suites[0]) {
			case WLAN_AKM_SUITE_PSK:
				priv->key_mgmt = IW_AUTH_KEY_MGMT_PSK;
				break;
			case WLAN_AKM_SUITE_8021X:
				priv->key_mgmt = IW_AUTH_KEY_MGMT_802_1X;
				break;
			default:
				priv->key_mgmt = 0;
				break;
			}
		}
	}
#endif

	if (req->ie && req->ie_len) { /* Set the IE */
		if (MLAN_STATUS_SUCCESS !=
		    // Casting is done to read the value
		    // coverity[misra_c_2012_rule_11_8_violation:SUPPRESS]
		    woal_cfg80211_assoc_ies_cfg(priv, (t_u8 *)req->ie,
						req->ie_len, MOAL_IOCTL_WAIT)) {
			PRINTM(MINFO, "Fail to woal_cfg80211_assoc_ies_cfg\n");
		}
	}

	if (req->crypto.n_ciphers_pairwise) {
		pairwise_enc_mode = woal_cfg80211_get_encryption_mode(
			req->crypto.ciphers_pairwise[0], &wpa_enabled);
		ret = woal_cfg80211_set_auth(priv, pairwise_enc_mode,
					     wpa_enabled, MOAL_IOCTL_WAIT);
		if (ret)
			goto done;
	}

	if (req->crypto.cipher_group) {
		group_enc_mode = woal_cfg80211_get_encryption_mode(
			req->crypto.cipher_group, &wpa_enabled);
		ret = woal_cfg80211_set_auth(priv, group_enc_mode, wpa_enabled,
					     MOAL_IOCTL_WAIT);
		if (ret)
			goto done;
	}
	ssid_bssid->host_mlme = priv->host_mlme;

	if (req->bss->channel) {
		ssid_bssid->channel_flags = req->bss->channel->flags;
		ssid_bssid->channel_flags |= CHAN_FLAGS_MAX;
		PRINTM(MCMND, "channel flags=0x%x\n", req->bss->channel->flags);
	}
	if (req->prev_bssid) {
		moal_memcpy_ext(priv->phandle, ssid_bssid->prev_bssid,
				req->prev_bssid, ETH_ALEN,
				sizeof(ssid_bssid->prev_bssid));
	}

	PRINTM(MCMND, "wlan: HostMlme %s send assoicate to bssid " MACSTR "\n",
	       priv->netdev->name, MAC2STR(req->bss->bssid));
	if (MLAN_STATUS_SUCCESS !=
	    woal_bss_start(priv, MOAL_IOCTL_WAIT_TIMEOUT, ssid_bssid)) {
		PRINTM(MERROR, "HostMlme %s: bss_start Fails\n",
		       priv->netdev->name);
		ret = -EFAULT;
	}

done:

	if (!ret) {
		struct station_info sinfo;
		priv->rssi_low = DEFAULT_RSSI_LOW_THRESHOLD;

		woal_save_assoc_params(priv, req, ssid_bssid);
		memset(&sinfo, 0, sizeof(sinfo));
		if (MLAN_STATUS_SUCCESS !=
		    woal_cfg80211_dump_station_info(priv, &sinfo)) {
			PRINTM(MERROR, "Failed to get station info\n");
		}

		memset(&bss_info, 0, sizeof(bss_info));
		if (MLAN_STATUS_SUCCESS !=
		    woal_get_bss_info(priv, MOAL_IOCTL_WAIT, &bss_info)) {
			PRINTM(MERROR,
			       "woal_get_bss_info Fails to get bss info\n");
		}
		priv->channel = bss_info.bss_chan;
	} else {
		/* clear the encryption mode */
		if (MLAN_STATUS_SUCCESS !=
		    woal_cfg80211_set_auth(priv, MLAN_ENCRYPTION_MODE_NONE,
					   MFALSE, MOAL_IOCTL_WAIT)) {
			PRINTM(MERROR, "Could not clear encryption \n");
			ret = -EFAULT;
		}
		/* clear IE */
		ie_len = 0;
		if (MLAN_STATUS_SUCCESS !=
		    woal_set_get_gen_ie(priv, MLAN_ACT_SET, NULL, NULL, &ie_len,
					MOAL_IOCTL_WAIT)) {
			PRINTM(MERROR, "Could not clear RSN IE\n");
			ret = -EFAULT;
		}
	}

	spin_lock_irqsave(&priv->connect_lock, flags);
	priv->cfg_connect = MFALSE;
	if (!ret && priv->media_connected) {
		PRINTM(MMSG,
		       "wlan: HostMlme %s Connected to bssid " MACSTR
		       " successfully\n",
		       priv->netdev->name, MAC2STR(priv->cfg_bssid));
		spin_unlock_irqrestore(&priv->connect_lock, flags);
		if (ssid_bssid->assoc_rsp.assoc_resp_len) {
			/* Parse the HE Operation IE from Assoc Response and
			 * download the 6E PSD table as per the ex-AP Operation
			 * mode */
			PRINTM(MCMND, "In/Out: priv->conn_chan.band: %d",
			       priv->conn_chan.band);
			if ((priv->phandle->fw_bands & BAND_6G) &&
			    (priv->conn_chan.band == NL80211_BAND_6GHZ))
				woal_dnld_sta_6e_psd_table(
					priv,
					ssid_bssid->assoc_rsp.assoc_resp_buf,
					ssid_bssid->assoc_rsp.assoc_resp_len,
					NULL);
			else
				memset(priv->phandle->mode_psd_string, 0,
				       sizeof(priv->phandle->mode_psd_string));
		}
	} else {
		PRINTM(MERROR,
		       "wlan: HostMlme %s Failed to connect to bssid " MACSTR
		       "\n",
		       priv->netdev->name, MAC2STR(req->bss->bssid));
		if (ssid_bssid->assoc_rsp.assoc_resp_len &&
		    ssid_bssid->assoc_rsp.assoc_resp_len >
			    (sizeof(IEEEtypes_MgmtHdr_t) +
			     sizeof(IEEEtypes_AssocRsp_t))) {
			// save the connection param when send assoc_resp to
			// kernel
			woal_save_assoc_params(priv, req, ssid_bssid);
			ret = 0;
			priv->host_mlme = MFALSE;
			priv->auth_flag = 0;
		} else {
			ssid_bssid->assoc_rsp.assoc_resp_len = 0;
			ret = 0;
			woal_assoc_timeout_event(priv, req);
		}
		spin_unlock_irqrestore(&priv->connect_lock, flags);
	}
	/*Association Response should also be send when ret is non-zero.
	  We also need to return success when we have association response
	  available*/
	if (ssid_bssid->assoc_rsp.assoc_resp_len) {
		priv->auth_flag |= HOST_MLME_ASSOC_DONE;
		woal_assoc_resp_event(priv, &ssid_bssid->assoc_rsp);
	}
	kfree(ssid_bssid);
	LEAVE();
	return ret;
}
#endif

/**
 * @brief Request the driver for (re)association
 *
 * @param priv            A pointer to moal_private structure
 * @param sme             A pointer to connect parameters
 * @param wait_option     wait option
 * @param assoc_resp      A pointer to assoc_rsp structure;
 *
 * @return                0 -- success, otherwise fail
 */
int woal_cfg80211_assoc(moal_private *priv, void *sme, t_u8 wait_option,
			mlan_ds_misc_assoc_rsp *assoc_rsp)
{
	struct cfg80211_connect_params *conn_param = NULL;
	mlan_802_11_ssid req_ssid;
	mlan_ssid_bssid *ssid_bssid = NULL;
	mlan_ioctl_req *req = NULL;
	int ret = 0;
	t_u32 auth_type = 0, mode;
	int wpa_enabled = 0;
	int group_enc_mode = 0, pairwise_enc_mode = 0;
	int alg_is_wep = 0;
	t_u8 ssid_len = 0;
	const t_u8 *bssid, *ssid;
	const t_u8 *ie = NULL;
	int ie_len = 0;
	struct ieee80211_channel *channel = NULL;
	bool privacy;
	struct cfg80211_bss *pub = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	if (!sme) {
		LEAVE();
		return -EFAULT;
	}
	ssid_bssid = kmalloc(sizeof(mlan_ssid_bssid), GFP_KERNEL);
	if (!ssid_bssid) {
		LEAVE();
		return -EFAULT;
	}

	mode = woal_nl80211_iftype_to_mode(priv->wdev->iftype);

	{
		conn_param = (struct cfg80211_connect_params *)sme;
		ssid = (const t_u8 *)conn_param->ssid;
		ssid_len = conn_param->ssid_len;
		bssid = (const t_u8 *)conn_param->bssid;
		channel = conn_param->channel;
		if (channel)
			priv->phandle->band = channel->band;
		if (conn_param->ie_len)
			ie = (const t_u8 *)conn_param->ie;
		ie_len = conn_param->ie_len;
		privacy = conn_param->privacy;
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 1, 0)
		pub = cfg80211_get_bss(priv->wdev->wiphy, channel, bssid, ssid,
				       ssid_len, IEEE80211_BSS_TYPE_ESS,
				       IEEE80211_PRIVACY_ANY);
#else
		pub = cfg80211_get_bss(priv->wdev->wiphy, channel, bssid, ssid,
				       ssid_len, WLAN_CAPABILITY_ESS,
				       WLAN_CAPABILITY_ESS);
#endif
		if (pub) {
			if ((!priv->phandle->params.reg_alpha2 ||
			     strncmp(priv->phandle->params.reg_alpha2, "99",
				     strlen("99")))
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 14, 0)
			    && (!moal_extflg_isset(priv->phandle,
						   EXT_COUNTRY_IE_IGNORE))
#endif
			)
				woal_process_country_ie(priv, pub);
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 9, 0)
			cfg80211_put_bss(priv->wdev->wiphy, pub);
#else
			cfg80211_put_bss(pub);
#endif
		} else
			woal_send_domain_info_cmd_fw(priv, wait_option);
#ifdef STA_WEXT
		if (IS_STA_WEXT(priv->phandle->params.cfg80211_wext)) {
			switch (conn_param->crypto.wpa_versions) {
			case NL80211_WPA_VERSION_2:
				priv->wpa_version = IW_AUTH_WPA_VERSION_WPA2;
				break;
			case NL80211_WPA_VERSION_1:
				priv->wpa_version = IW_AUTH_WPA_VERSION_WPA;
				break;
			default:
				priv->wpa_version = 0;
				break;
			}
			if (conn_param->crypto.n_akm_suites) {
				switch (conn_param->crypto.akm_suites[0]) {
				case WLAN_AKM_SUITE_PSK:
					priv->key_mgmt = IW_AUTH_KEY_MGMT_PSK;
					break;
				case WLAN_AKM_SUITE_8021X:
					priv->key_mgmt =
						IW_AUTH_KEY_MGMT_802_1X;
					break;
				default:
					priv->key_mgmt = 0;
					break;
				}
			}
		}
#endif
	}

	memset(&req_ssid, 0, sizeof(mlan_802_11_ssid));
	memset(ssid_bssid, 0, sizeof(mlan_ssid_bssid));

	req_ssid.ssid_len = ssid_len;
	if (ssid_len > MW_ESSID_MAX_SIZE) {
		PRINTM(MERROR, "Invalid SSID - aborting\n");
		ret = -EINVAL;
		goto done;
	}

	moal_memcpy_ext(priv->phandle, req_ssid.ssid, ssid, ssid_len,
			sizeof(req_ssid.ssid));
	if (!req_ssid.ssid_len || req_ssid.ssid[0] < 0x20) {
		PRINTM(MERROR, "Invalid SSID - aborting\n");
		ret = -EINVAL;
		goto done;
	}

	if (priv->phandle->card_info->embedded_supp)
		if (MLAN_STATUS_SUCCESS !=
		    woal_set_ewpa_mode(priv, wait_option, ssid_bssid)) {
			ret = -EFAULT;
			goto done;
		}

	if (MLAN_STATUS_SUCCESS !=
	    woal_cfg80211_set_key(priv, 0, 0, NULL, 0, NULL, 0,
				  KEY_INDEX_CLEAR_ALL, NULL, 1, 0,
				  wait_option)) {
		/* Disable keys and clear all previous security settings */
		ret = -EFAULT;
		goto done;
	}

#if CFG80211_VERSION_CODE > KERNEL_VERSION(4, 12, 14)
	if (conn_param && conn_param->crypto.psk &&
	    priv->phandle->card_info->embedded_supp) {
		if (MLAN_STATUS_SUCCESS !=
		    woal_cfg80211_set_psk(priv, conn_param)) {
			PRINTM(MERROR, "Embedded supplicant: set psk failed\n");
			ret = -EFAULT;
			goto done;
		}
	}
#endif

#ifdef STA_CFG80211
	if (IS_STA_CFG80211(priv->phandle->params.cfg80211_wext)) {
		/** Check if current roaming support OKC offload roaming */
		if (conn_param && conn_param->crypto.n_akm_suites &&
		    conn_param->crypto.akm_suites[0] == WLAN_AKM_SUITE_8021X) {
			if (priv->okc_roaming_ie && priv->okc_ie_len) {
				ie = priv->okc_roaming_ie;
				ie_len = priv->okc_ie_len;
			}
		}
	}
#endif

	if ((priv->ft_pre_connect ||
	     (conn_param && conn_param->auth_type == NL80211_AUTHTYPE_FT)) &&
	    priv->ft_ie_len) {
		ie = priv->ft_ie;
		ie_len = priv->ft_ie_len;
		priv->ft_ie_len = 0;
	}
	if (ie && ie_len) { /* Set the IE */
		if (MLAN_STATUS_SUCCESS !=
		    woal_cfg80211_assoc_ies_cfg(priv, ie, ie_len,
						wait_option)) {
			PRINTM(MINFO, "Fail to woal_cfg80211_assoc_ies_cfg\n");
		}
	}

	if (conn_param && mode != MLAN_BSS_MODE_IBSS) {
		/* These parameters are only for managed mode */
		if (conn_param->auth_type == NL80211_AUTHTYPE_OPEN_SYSTEM)
			auth_type = MLAN_AUTH_MODE_OPEN;
		else if (conn_param->auth_type == NL80211_AUTHTYPE_SHARED_KEY)
			auth_type = MLAN_AUTH_MODE_SHARED;
		else if (conn_param->auth_type == NL80211_AUTHTYPE_NETWORK_EAP)
			auth_type = MLAN_AUTH_MODE_NETWORKEAP;
		else if (conn_param->auth_type == NL80211_AUTHTYPE_FT)
			auth_type = MLAN_AUTH_MODE_FT;
		else
			auth_type = MLAN_AUTH_MODE_AUTO;
		if (priv->ft_pre_connect)
			auth_type = MLAN_AUTH_MODE_FT;
		/* Set FILS auth mode */
		if (priv->enable_fils)
			auth_type = MLAN_AUTH_MODE_FILS;
		if (MLAN_STATUS_SUCCESS !=
		    woal_set_auth_mode(priv, wait_option, auth_type)) {
			ret = -EFAULT;
			goto done;
		}

		if (conn_param->crypto.n_ciphers_pairwise) {
			pairwise_enc_mode = woal_cfg80211_get_encryption_mode(
				conn_param->crypto.ciphers_pairwise[0],
				&wpa_enabled);
			ret = woal_cfg80211_set_auth(priv, pairwise_enc_mode,
						     wpa_enabled, wait_option);
			if (ret)
				goto done;
		}

		if (conn_param->crypto.cipher_group) {
			group_enc_mode = woal_cfg80211_get_encryption_mode(
				conn_param->crypto.cipher_group, &wpa_enabled);
			ret = woal_cfg80211_set_auth(priv, group_enc_mode,
						     wpa_enabled, wait_option);
			if (ret)
				goto done;
		}

		if (conn_param->key) {
			alg_is_wep =
				woal_cfg80211_is_alg_wep(pairwise_enc_mode) |
				woal_cfg80211_is_alg_wep(group_enc_mode);
			if (alg_is_wep) {
				PRINTM(MINFO,
				       "Setting wep encryption with key len %d\n",
				       conn_param->key_len);
				/* Set the WEP key */
				if (MLAN_STATUS_SUCCESS !=
				    woal_cfg80211_set_wep_keys(
					    priv, conn_param->key,
					    conn_param->key_len,
					    conn_param->key_idx, wait_option)) {
					ret = -EFAULT;
					goto done;
				}
				/* Enable the WEP key by key index */
				if (MLAN_STATUS_SUCCESS !=
				    woal_cfg80211_set_wep_keys(
					    priv, NULL, 0, conn_param->key_idx,
					    wait_option)) {
					ret = -EFAULT;
					goto done;
				}
			}
		}
	}

	moal_memcpy_ext(priv->phandle, &ssid_bssid->ssid, &req_ssid,
			sizeof(mlan_802_11_ssid), sizeof(ssid_bssid->ssid));
	if (bssid)
		moal_memcpy_ext(priv->phandle, &ssid_bssid->bssid, bssid,
				ETH_ALEN, sizeof(ssid_bssid->bssid));
	if (MLAN_STATUS_SUCCESS !=
	    woal_find_essid(priv, ssid_bssid, wait_option)) {
		/* Do specific SSID scanning */
		if (mode != MLAN_BSS_MODE_IBSS)
			ret = woal_cfg80211_connect_scan(priv, conn_param,
							 wait_option);
		else
			ret = woal_request_scan(priv, wait_option, &req_ssid);
		if (ret) {
			ret = -EFAULT;
			goto done;
		}
	}

	/* Disconnect before try to associate */
	if (mode == MLAN_BSS_MODE_IBSS)
		woal_disconnect(priv, wait_option, NULL,
				DEF_DEAUTH_REASON_CODE);

	if (mode != MLAN_BSS_MODE_IBSS) {
		if (MLAN_STATUS_SUCCESS !=
		    woal_find_best_network(priv, wait_option, ssid_bssid)) {
			ret = -EFAULT;
			goto done;
		}
		/* Inform the BSS information to kernel, otherwise
		 * kernel will give a panic after successful assoc */
		if (MLAN_STATUS_SUCCESS !=
		    woal_inform_bss_from_scan_result(priv, ssid_bssid,
						     wait_option)) {
			ret = -EFAULT;
			goto done;
		}
	} else if (MLAN_STATUS_SUCCESS !=
		   woal_find_best_network(priv, wait_option, ssid_bssid))
		/* Adhoc start, Check the channel command */
		woal_11h_channel_check_ioctl(priv, wait_option);

	PRINTM(MINFO, "Trying to associate to %s and bssid " MACSTR "\n",
	       (char *)req_ssid.ssid, MAC2STR(ssid_bssid->bssid));

	/* Zero SSID implies use BSSID to connect */
	if (bssid)
		memset(&ssid_bssid->ssid, 0, sizeof(mlan_802_11_ssid));
	else /* Connect to BSS by ESSID */
		memset(&ssid_bssid->bssid, 0, MLAN_MAC_ADDR_LENGTH);
	if (channel) {
		ssid_bssid->channel_flags = channel->flags;
		ssid_bssid->channel_flags |= CHAN_FLAGS_MAX;
		PRINTM(MCMND, "channel flags=0x%x\n", channel->flags);
	}
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 7, 0)
	if (conn_param && conn_param->prev_bssid) {
		moal_memcpy_ext(priv->phandle, ssid_bssid->prev_bssid,
				conn_param->prev_bssid, ETH_ALEN,
				sizeof(ssid_bssid->prev_bssid));
	}
#endif

	if (MLAN_STATUS_SUCCESS !=
	    woal_bss_start(priv, MOAL_IOCTL_WAIT_TIMEOUT, ssid_bssid)) {
		ret = -EFAULT;
		goto done;
	}

	if (assoc_rsp) {
		moal_memcpy_ext(priv->phandle, assoc_rsp,
				&ssid_bssid->assoc_rsp,
				sizeof(mlan_ds_misc_assoc_rsp),
				sizeof(mlan_ds_misc_assoc_rsp));
		PRINTM(MCMND, "assoc_rsp ie len=%d\n",
		       assoc_rsp->assoc_resp_len);
	}
done:
	kfree(ssid_bssid);
	if (ret) {
		/* clear the encryption mode */
		if (MLAN_STATUS_SUCCESS !=
		    woal_cfg80211_set_auth(priv, MLAN_ENCRYPTION_MODE_NONE,
					   MFALSE, wait_option)) {
			PRINTM(MERROR, "Could not clear encryption \n");
			ret = -EFAULT;
		}
		/* clear IE */
		ie_len = 0;
		if (MLAN_STATUS_SUCCESS !=
		    woal_set_get_gen_ie(priv, MLAN_ACT_SET, NULL, NULL, &ie_len,
					wait_option)) {
			PRINTM(MERROR, "Could not clear RSN IE\n");
			ret = -EFAULT;
		}
	}
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 1, 18)
/**
 * @brief This function convert he_gi to nl80211_he_gi
 *
 * @param he_gi  0/1/2/3
 *
 *
 * @return  0: NL80211_RATE_INFO_HE_GI_0_8
 *          1: NL80211_RATE_INFO_HE_GI_1_6
 *          2: NL80211_RATE_INFO_HE_GI_3_2
 */
static t_u8 woal_he_gi_to_nl80211_he_gi(t_u8 he_gi)
{
	t_u8 cfg_he_gi = 0;
	switch (he_gi) {
	case 3:
		cfg_he_gi = NL80211_RATE_INFO_HE_GI_3_2;
		break;
	case 2:
		cfg_he_gi = NL80211_RATE_INFO_HE_GI_1_6;
		break;
	case 0:
	case 1:
	default:
		cfg_he_gi = NL80211_RATE_INFO_HE_GI_0_8;
		break;
	}
	return cfg_he_gi;
}
#endif

/**
 * @brief Request the driver to fill the tx/rx rate info
 *
 * @param priv            A pointer to moal_private structure
 * @param sinfo           A pointer to station_info structure
 *
 * @return                0 -- success, otherwise fail
 */
static void woal_cfg80211_fill_rate_info(moal_private *priv,
					 struct station_info *sinfo)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_ioctl_req *req = NULL;
	mlan_ds_rate *rate = NULL;
	t_u16 Rates[12] = {0x02, 0x04, 0x0B, 0x16, 0x0C, 0x12,
			   0x18, 0x24, 0x30, 0x48, 0x60, 0x6c};
	ENTER();
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_rate));
	if (req == NULL) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	rate = (mlan_ds_rate *)req->pbuf;
	rate->sub_command = MLAN_OID_GET_DATA_RATE;
	req->req_id = MLAN_IOCTL_RATE;
	req->action = MLAN_ACT_GET;
	ret = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (ret != MLAN_STATUS_SUCCESS)
		goto done;
	if (rate->param.data_rate.tx_rate_format != MLAN_RATE_FORMAT_LG) {
		if (rate->param.data_rate.tx_rate_format ==
		    MLAN_RATE_FORMAT_HT) {
			sinfo->txrate.flags = RATE_INFO_FLAGS_MCS;
			if (rate->param.data_rate.tx_ht_bw == MLAN_HT_BW40)
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 0, 0)
				sinfo->txrate.bw = RATE_INFO_BW_40;
#else
				sinfo->txrate.flags |=
					RATE_INFO_FLAGS_40_MHZ_WIDTH;
#endif
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 0, 0)
			else
				sinfo->txrate.bw = RATE_INFO_BW_20;
#endif
		}
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
		else if (rate->param.data_rate.tx_rate_format ==
			 MLAN_RATE_FORMAT_VHT) {
			sinfo->txrate.flags = RATE_INFO_FLAGS_VHT_MCS;
			sinfo->txrate.nss = rate->param.data_rate.tx_nss + 1;
			if (rate->param.data_rate.tx_ht_bw == MLAN_VHT_BW80)
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 0, 0)
				sinfo->txrate.bw = RATE_INFO_BW_80;
#else
				sinfo->txrate.flags |=
					RATE_INFO_FLAGS_80_MHZ_WIDTH;
#endif
			else if (rate->param.data_rate.tx_ht_bw == MLAN_HT_BW40)
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 0, 0)
				sinfo->txrate.bw = RATE_INFO_BW_40;
#else
				sinfo->txrate.flags |=
					RATE_INFO_FLAGS_40_MHZ_WIDTH;
#endif
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 0, 0)
			else
				sinfo->txrate.bw = RATE_INFO_BW_20;
#endif
		}
#endif
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 1, 18)
		else if (rate->param.data_rate.tx_rate_format ==
			 MLAN_RATE_FORMAT_HE) {
			sinfo->txrate.flags = RATE_INFO_FLAGS_HE_MCS;
			sinfo->txrate.nss = rate->param.data_rate.tx_nss + 1;
			sinfo->txrate.mcs = rate->param.data_rate.tx_mcs_index;
			sinfo->txrate.he_gi = woal_he_gi_to_nl80211_he_gi(
				rate->param.data_rate.tx_ht_gi);
			if (rate->param.data_rate.tx_ht_bw == MLAN_VHT_BW80)
				sinfo->txrate.bw = RATE_INFO_BW_80;
			else if (rate->param.data_rate.tx_ht_bw == MLAN_HT_BW40)
				sinfo->txrate.bw = RATE_INFO_BW_40;
			else
				sinfo->txrate.bw = RATE_INFO_BW_20;
		}
#endif
		if (rate->param.data_rate.tx_ht_gi == MLAN_HT_SGI)
			sinfo->txrate.flags |= RATE_INFO_FLAGS_SHORT_GI;
		sinfo->txrate.mcs = rate->param.data_rate.tx_mcs_index;
	} else {
		/* Bit rate is in 500 kb/s units. Convert it to 100kb/s units */
		if (rate->param.data_rate.tx_data_rate < 12) {
			sinfo->txrate.legacy =
				Rates[rate->param.data_rate.tx_data_rate] * 5;
		} else
			sinfo->txrate.legacy = Rates[0] * 5;
	}
	// Fill Rx rate
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 0, 0)
	if (rate->param.data_rate.rx_rate_format != MLAN_RATE_FORMAT_LG) {
		if (rate->param.data_rate.rx_rate_format ==
		    MLAN_RATE_FORMAT_HT) {
			sinfo->rxrate.flags = RATE_INFO_FLAGS_MCS;
			if (rate->param.data_rate.rx_ht_bw == MLAN_HT_BW40)
				sinfo->rxrate.bw = RATE_INFO_BW_40;
			else
				sinfo->rxrate.bw = RATE_INFO_BW_20;
		} else if (rate->param.data_rate.rx_rate_format ==
			   MLAN_RATE_FORMAT_VHT) {
			sinfo->rxrate.flags = RATE_INFO_FLAGS_VHT_MCS;
			sinfo->rxrate.nss = rate->param.data_rate.rx_nss + 1;
			if (rate->param.data_rate.rx_ht_bw == MLAN_VHT_BW80)
				sinfo->rxrate.bw = RATE_INFO_BW_80;
			else if (rate->param.data_rate.rx_ht_bw == MLAN_HT_BW40)
				sinfo->rxrate.bw = RATE_INFO_BW_40;
			else
				sinfo->rxrate.bw = RATE_INFO_BW_20;
		}
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 1, 18)
		else if (rate->param.data_rate.rx_rate_format ==
			 MLAN_RATE_FORMAT_HE) {
			sinfo->rxrate.flags = RATE_INFO_FLAGS_HE_MCS;
			sinfo->rxrate.nss = rate->param.data_rate.rx_nss + 1;
			sinfo->rxrate.mcs = rate->param.data_rate.rx_mcs_index;
			sinfo->rxrate.he_gi = woal_he_gi_to_nl80211_he_gi(
				rate->param.data_rate.rx_ht_gi);
			if (rate->param.data_rate.rx_ht_bw == MLAN_VHT_BW80)
				sinfo->rxrate.bw = RATE_INFO_BW_80;
			else if (rate->param.data_rate.rx_ht_bw == MLAN_HT_BW40)
				sinfo->rxrate.bw = RATE_INFO_BW_40;
			else
				sinfo->rxrate.bw = RATE_INFO_BW_20;
		}
#endif
		if (rate->param.data_rate.rx_ht_gi == MLAN_HT_SGI)
			sinfo->rxrate.flags |= RATE_INFO_FLAGS_SHORT_GI;
		sinfo->rxrate.mcs = rate->param.data_rate.rx_mcs_index;
	} else {
		/* Bit rate is in 500 kb/s units. Convert it to 100kb/s units */
		if (rate->param.data_rate.rx_data_rate < 12) {
			sinfo->rxrate.legacy =
				Rates[rate->param.data_rate.rx_data_rate] * 5;
		} else
			sinfo->rxrate.legacy = 0;
	}
#endif
done:
	if (ret != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return;
}
/**
 * @brief Request the driver to dump the station information
 *
 * @param priv            A pointer to moal_private structure
 * @param sinfo           A pointer to station_info structure
 *
 * @return                0 -- success, otherwise fail
 */
static mlan_status woal_cfg80211_dump_station_info(moal_private *priv,
						   struct station_info *sinfo)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_ds_get_signal signal;
	mlan_ds_get_stats stats;
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 0, 0)
	mlan_bss_info bss_info;
	t_u8 dtim_period = 0;
#endif

	ENTER();

	if (priv->phandle->scan_pending_on_block) {
		if (priv->sinfo)
			moal_memcpy_ext(priv->phandle, sinfo, priv->sinfo,
					sizeof(struct station_info),
					sizeof(struct station_info));
		LEAVE();
		return ret;
	}
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 0, 0)
	sinfo->filled = MBIT(NL80211_STA_INFO_RX_BYTES) |
			MBIT(NL80211_STA_INFO_TX_BYTES) |
			MBIT(NL80211_STA_INFO_RX_PACKETS) |
			MBIT(NL80211_STA_INFO_TX_PACKETS) |
			MBIT(NL80211_STA_INFO_SIGNAL) |
			MBIT(NL80211_STA_INFO_SIGNAL_AVG) |
			MBIT(NL80211_STA_INFO_TX_BITRATE) |
			MBIT(NL80211_STA_INFO_RX_BITRATE);
#else
	sinfo->filled = STATION_INFO_RX_BYTES | STATION_INFO_TX_BYTES |
			STATION_INFO_RX_PACKETS | STATION_INFO_TX_PACKETS |
			STATION_INFO_SIGNAL | STATION_INFO_TX_BITRATE;
#endif

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(2, 6, 37)
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 0, 0)
	sinfo->filled |= MBIT(NL80211_STA_INFO_TX_FAILED);
#else
	sinfo->filled |= STATION_INFO_TX_FAILED;
#endif
#endif

	/* Get signal information from the firmware */
	memset(&signal, 0, sizeof(mlan_ds_get_signal));
	if (MLAN_STATUS_SUCCESS !=
	    woal_get_signal_info(priv, MOAL_IOCTL_WAIT, &signal)) {
		PRINTM(MERROR, "Error getting signal information\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	/* Get stats information from the firmware */
	memset(&stats, 0, sizeof(mlan_ds_get_stats));
	if (MLAN_STATUS_SUCCESS !=
	    woal_get_stats_info(priv, MOAL_IOCTL_WAIT, &stats)) {
		PRINTM(MERROR, "Error getting stats information\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	sinfo->rx_bytes = priv->stats.rx_bytes;
	sinfo->tx_bytes = priv->stats.tx_bytes;
	sinfo->rx_packets = priv->stats.rx_packets;
	sinfo->tx_packets = priv->stats.tx_packets;
	sinfo->signal = signal.bcn_rssi_last;
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(2, 6, 38)
	sinfo->signal_avg = signal.bcn_rssi_avg;
#endif
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(2, 6, 37)
	sinfo->tx_failed = stats.failed;
#endif

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 0, 0)
	/* Update BSS information */
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 0, 0)
	sinfo->filled |= MBIT(NL80211_STA_INFO_BSS_PARAM);
#else
	sinfo->filled |= STATION_INFO_BSS_PARAM;
#endif
	sinfo->bss_param.flags = 0;
	ret = woal_get_bss_info(priv, MOAL_IOCTL_WAIT, &bss_info);
	if (ret)
		goto done;
	if (bss_info.capability_info & WLAN_CAPABILITY_SHORT_PREAMBLE)
		sinfo->bss_param.flags |= BSS_PARAM_FLAGS_SHORT_PREAMBLE;
	if (bss_info.capability_info & WLAN_CAPABILITY_SHORT_SLOT_TIME)
		sinfo->bss_param.flags |= BSS_PARAM_FLAGS_SHORT_SLOT_TIME;
	sinfo->bss_param.beacon_interval = bss_info.beacon_interval;
	/* Get DTIM period */
	ret = woal_set_get_dtim_period(priv, MLAN_ACT_GET, MOAL_IOCTL_WAIT,
				       &dtim_period);
	if (ret) {
		PRINTM(MERROR, "Get DTIM period failed\n");
		goto done;
	}
	sinfo->bss_param.dtim_period = dtim_period;
#endif
	woal_cfg80211_fill_rate_info(priv, sinfo);
	if (priv->sinfo)
		moal_memcpy_ext(priv->phandle, priv->sinfo, sinfo,
				sizeof(struct station_info),
				sizeof(struct station_info));

done:
	LEAVE();
	return ret;
}

/********************************************************
				Global Functions
********************************************************/
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 0, 0)

/**
 *  @brief This function will be used by sort, to compare LHS & RHS
 *
 *  @param lhs              LHS value
 *  @param rhs              RHS value
 *  @return                 0
 */
static int compare(const void *lhs, const void *rhs)
{
	const chan_freq_power_t *lhs_cfp = (const chan_freq_power_t *)(lhs);
	const chan_freq_power_t *rhs_cfp = (const chan_freq_power_t *)(rhs);

	if (lhs_cfp->channel < rhs_cfp->channel)
		return -1;
	if (lhs_cfp->channel > rhs_cfp->channel)
		return 1;

	return 0;
}

/**
 *  @brief This function get channel reg_rule flags
 *
 *  @param buf              Buffer containing channel region config
 *  @param num_chan         Length of buffer
 *  @param regd             ieee80211_regdomain to be updated
 *
 *  @return                 N/A
 */
static t_u32 woal_get_chan_rule_flags(mlan_ds_custom_reg_domain *custom_reg,
				      t_u8 channel)
{
	t_u16 num_chan = 0;
	t_u32 flags = 0;
	int idx;
	t_u16 chflags;

	num_chan = custom_reg->num_bg_chan;
	num_chan += custom_reg->num_a_chan;
	for (idx = 0; idx < num_chan; idx++) {
		chflags = custom_reg->cfp_tbl[idx].dynamic.flags;
		if (chflags & NXP_CHANNEL_DISABLED)
			continue;
		if (custom_reg->cfp_tbl[idx].channel == channel) {
			if (chflags & NXP_CHANNEL_PASSIVE)
				flags |= NL80211_RRF_NO_IR;
			if (chflags & NXP_CHANNEL_DFS)
				flags |= NL80211_RRF_DFS;
			if (chflags & NXP_CHANNEL_NO_OFDM)
				flags |= NL80211_RRF_NO_OFDM;
			break;
		}
	}
	return flags;
}

/**
 *  @brief This function update the beaconng flags of channel
 *
 *  @param custom_reg       pointer to mlan_ds_custom_reg_domain
 *
 *  @return                 pointer to ieee80211_regdomain
 */

static void
woal_reg_apply_beaconing_flags(struct wiphy *wiphy,
			       mlan_ds_custom_reg_domain *custom_reg)
{
	struct ieee80211_supported_band *sband;
	struct ieee80211_channel *ch;
	int band, i;
	t_u32 rule_flags = 0;

	for (band = 0; band < IEEE80211_NUM_BANDS; band++) {
		sband = wiphy->bands[band];
		if (!sband)
			continue;

		for (i = 0; i < sband->n_channels; i++) {
			ch = &sband->channels[i];

			if (ch->flags &
			    (IEEE80211_CHAN_DISABLED | IEEE80211_CHAN_RADAR))
				continue;

			rule_flags = woal_get_chan_rule_flags(custom_reg,
							      ch->hw_value);

			if (!(rule_flags & NL80211_RRF_NO_IR))
				ch->flags &= ~IEEE80211_CHAN_NO_IR;
		}
	}
}

/**
 *  @brief This function create the custom regdomain
 *
 *  @param priv       pointer to moal_private
 *  @param custom_reg       pointer to mlan_ds_custom_reg_domain
 *
 *  @return                 pointer to ieee80211_regdomain
 */
static struct ieee80211_regdomain *
create_custom_regdomain(moal_private *priv,
			mlan_ds_custom_reg_domain *custom_reg)
{
	struct ieee80211_reg_rule *rule;
	bool new_rule;
	int idx, freq, prev_freq = 0;
	t_u8 chan;
	t_u16 num_chan = 0;
	t_u16 pwr, prev_pwr = 0;
	t_u32 bw, prev_bw = 0;
	t_u16 chflags, prev_chflags = 0, valid_rules = 0;
	struct ieee80211_regdomain *regd = NULL;
	int regd_size;
	const struct ieee80211_freq_range *freq_range = NULL;

	num_chan = custom_reg->num_bg_chan;
	num_chan += custom_reg->num_a_chan;

	sort(&custom_reg->cfp_tbl[custom_reg->num_bg_chan],
	     custom_reg->num_a_chan, sizeof(chan_freq_power_t), &compare, NULL);
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 4, 0)
	num_chan += custom_reg->num_6g_chan;
#endif
	regd_size = sizeof(struct ieee80211_regdomain) +
		    num_chan * sizeof(struct ieee80211_reg_rule);

	regd = kzalloc(regd_size, GFP_KERNEL);
	if (!regd) {
		return NULL;
	}
	for (idx = 0; idx < num_chan; idx++) {
		enum ieee80211_band band;

		chan = custom_reg->cfp_tbl[idx].channel;
		if (!chan) {
			if (regd)
				kfree(regd);
			return NULL;
		}
		chflags = custom_reg->cfp_tbl[idx].dynamic.flags;
		band = (chan <= 14) ? IEEE80211_BAND_2GHZ : IEEE80211_BAND_5GHZ;
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 4, 0)
		if (idx >= (custom_reg->num_bg_chan + custom_reg->num_a_chan))
			band = IEEE80211_BAND_6GHZ;
#endif
		freq = ieee80211_channel_to_frequency(chan, band);
		PRINTM(MINFO, "chan=%d freq=%d chan_flag=0x%x\n", chan, freq,
		       chflags);
		new_rule = false;

		if (chflags & NXP_CHANNEL_DISABLED) {
			prev_chflags = chflags;
			continue;
		}

		if (band == IEEE80211_BAND_5GHZ
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 4, 0)
		    || band == IEEE80211_BAND_6GHZ
#endif
		) {
			if (!(chflags & NXP_CHANNEL_NOHT80))
				bw = MHZ_TO_KHZ(80);
			else if (!(chflags & NXP_CHANNEL_NOHT40))
				bw = MHZ_TO_KHZ(40);
			else
				bw = MHZ_TO_KHZ(20);
		} else {
			if (!(chflags & NXP_CHANNEL_NOHT40))
				bw = MHZ_TO_KHZ(40);
			else
				bw = MHZ_TO_KHZ(20);
		}

		pwr = custom_reg->cfp_tbl[idx].max_tx_power;

		if (idx == 0 || prev_chflags != chflags || prev_bw != bw ||
		    freq - prev_freq > 20) {
			valid_rules++;
			new_rule = true;
		}
		if (!new_rule && pwr != prev_pwr) {
			valid_rules++;
			new_rule = true;
		}

		rule = &regd->reg_rules[valid_rules - 1];

		rule->freq_range.end_freq_khz = MHZ_TO_KHZ(freq + 10);

		prev_chflags = chflags;
		prev_freq = freq;
		prev_bw = bw;

		if (!new_rule)
			continue;

		rule->freq_range.start_freq_khz = MHZ_TO_KHZ(freq - 10);
		rule->power_rule.max_eirp = DBM_TO_MBM(pwr);
		prev_pwr = pwr;

		rule->flags = 0;

		if (chflags & NXP_CHANNEL_PASSIVE)
			rule->flags |= NL80211_RRF_NO_IR;
		if (chflags & NXP_CHANNEL_DFS)
			rule->flags |= NL80211_RRF_DFS;
		if (chflags & NXP_CHANNEL_NO_OFDM)
			rule->flags |= NL80211_RRF_NO_OFDM;
		rule->freq_range.max_bandwidth_khz = bw;
	}

	regd->n_reg_rules = valid_rules;

	if (custom_reg->region.country_code[0] == 'W' &&
	    custom_reg->region.country_code[1] == 'W') {
		regd->alpha2[0] = '0';
		regd->alpha2[1] = '0';
	} else {
		/* set alpha2 from FW. */
		regd->alpha2[0] = custom_reg->region.country_code[0];
		regd->alpha2[1] = custom_reg->region.country_code[1];
	}

	switch (custom_reg->region.dfs_region) {
	case 1:
		regd->dfs_region = NL80211_DFS_FCC;
		break;
	case 2:
		regd->dfs_region = NL80211_DFS_ETSI;
		break;
	case 3:
		regd->dfs_region = NL80211_DFS_JP;
		break;
	default:
		regd->dfs_region = NL80211_DFS_UNSET;
		break;
	}
	priv->phandle->dfs_region = regd->dfs_region;
	PRINTM(MCMND, "create_custom_regdomain: %c%c rules=%d dfs_region=%d\n",
	       regd->alpha2[0], regd->alpha2[1], valid_rules, regd->dfs_region);
	for (idx = 0; idx < (int)regd->n_reg_rules; idx++) {
		rule = &regd->reg_rules[idx];
		freq_range = &rule->freq_range;
		PRINTM(MCMND,
		       "flags=0x%x star_freq=%d end_freq=%d freq_diff=%d max_bandwidth=%d\n",
		       rule->flags, freq_range->start_freq_khz,
		       freq_range->end_freq_khz,
		       freq_range->end_freq_khz - freq_range->start_freq_khz,
		       freq_range->max_bandwidth_khz);
	}
	if (!regd->n_reg_rules) {
		kfree(regd);
		regd = NULL;
	}
	return regd;
}

/**
 *  @brief get channel region config(0x242),update otp_region including
 * force_reg
 *
 *  @param priv         A pointer to moal_private structure
 *
 *  @return		        0-success, otherwise failure
 */

static int woal_get_chan_region_cfg(moal_private *priv)
{
	mlan_ds_misc_cfg *misc = NULL;
	mlan_ioctl_req *req = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;
	t_u8 country_code[COUNTRY_CODE_LEN];
	int ret = 0;

	ENTER();

	memset(country_code, 0, sizeof(country_code));
	country_code[0] = priv->phandle->country_code[0];
	country_code[1] = priv->phandle->country_code[1];

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}
	misc = (mlan_ds_misc_cfg *)req->pbuf;
	misc->sub_command = MLAN_OID_MISC_GET_CHAN_REGION_CFG;
	req->req_id = MLAN_IOCTL_MISC_CFG;
	req->action = MLAN_ACT_GET;
	memset(&misc->param.custom_reg_domain, 0,
	       sizeof(misc->param.custom_reg_domain));

	misc->param.custom_reg_domain.region.country_code[0] = country_code[0];
	misc->param.custom_reg_domain.region.country_code[1] = country_code[1];

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
 *  @brief create custom channel regulatory config
 *
 *  @param priv         A pointer to moal_private structure
 *
 *  @return		        0-success, otherwise failure
 */
static int woal_update_custom_regdomain(moal_private *priv, struct wiphy *wiphy)
{
	mlan_ds_misc_cfg *misc = NULL;
	mlan_ioctl_req *req = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;
	t_u8 country_code[COUNTRY_CODE_LEN];

	int ret = 0;
	struct ieee80211_regdomain *regd = NULL;

	ENTER();

	if (!priv || !wiphy) {
		LEAVE();
		return -EFAULT;
	}

	memset(country_code, 0, sizeof(country_code));
	if (MTRUE ==
	    is_cfg80211_special_region_code(priv->phandle->country_code)) {
		country_code[0] = 'W';
		country_code[1] = 'W';
	} else {
		country_code[0] = priv->phandle->country_code[0];
		country_code[1] = priv->phandle->country_code[1];
	}

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		ret = -EFAULT;
		goto done;
	}
	misc = (mlan_ds_misc_cfg *)req->pbuf;
	misc->sub_command = MLAN_OID_MISC_GET_CHAN_REGION_CFG;
	req->req_id = MLAN_IOCTL_MISC_CFG;
	req->action = MLAN_ACT_GET;
	memset(&misc->param.custom_reg_domain, 0,
	       sizeof(misc->param.custom_reg_domain));

	misc->param.custom_reg_domain.region.country_code[0] = country_code[0];
	misc->param.custom_reg_domain.region.country_code[1] = country_code[1];

	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}
	if (misc->param.custom_reg_domain.region.country_code[0] == '\0' ||
	    misc->param.custom_reg_domain.region.country_code[1] == '\0') {
		PRINTM(MCMND, "FW country code not valid\n");
		ret = -EFAULT;
		goto done;
	}
	if (misc->param.custom_reg_domain.region.country_code[0] !=
		    country_code[0] ||
	    misc->param.custom_reg_domain.region.country_code[1] !=
		    country_code[1]) {
		PRINTM(MCMND, "FW country code %c%c not match %c%c\n",
		       misc->param.custom_reg_domain.region.country_code[0],
		       misc->param.custom_reg_domain.region.country_code[1],
		       country_code[0], country_code[1]);
	}
	regd = create_custom_regdomain(priv, &misc->param.custom_reg_domain);
	if (regd) {
		PRINTM(MMSG, "call regulatory_set_wiphy_regd %c%c\n",
		       misc->param.custom_reg_domain.region.country_code[0],
		       misc->param.custom_reg_domain.region.country_code[1]);
		wiphy->regulatory_flags &=
			~(REGULATORY_STRICT_REG | REGULATORY_CUSTOM_REG);
		wiphy->regulatory_flags |= REGULATORY_WIPHY_SELF_MANAGED;
		rtnl_lock();
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 12, 0)
		ret = regulatory_set_wiphy_regd_sync(wiphy, regd);
#else
		ret = regulatory_set_wiphy_regd_sync_rtnl(wiphy, regd);
#endif
		rtnl_unlock();
		kfree(regd);
		if (!ret)
			woal_reg_apply_beaconing_flags(
				wiphy, &misc->param.custom_reg_domain);
	}
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief This workqueue handles create customer regulatory
 * case
 *
 *  @param work    A pointer to work_struct
 *
 *  @return        N/A
 */
void woal_regulatory_work_queue(struct work_struct *work)
{
	// Coverity violation raised for kernel's API
	// coverity[cert_arr39_c_violation:SUPPRESS]
	moal_handle *handle = container_of(work, moal_handle, regulatory_work);
	struct wiphy *wiphy = handle->wiphy;
	moal_private *priv = woal_get_priv(handle, MLAN_BSS_ROLE_ANY);
	enum ieee80211_band band;

	if (priv && wiphy) {
		woal_update_custom_regdomain(priv, wiphy);
		band = priv->phandle->band;
		priv->phandle->band = IEEE80211_BAND_2GHZ;
		woal_send_domain_info_cmd_fw(priv, MOAL_IOCTL_WAIT);
		if (priv->phandle->fw_bands & BAND_A) {
			priv->phandle->band = IEEE80211_BAND_5GHZ;
			woal_send_domain_info_cmd_fw(priv, MOAL_IOCTL_WAIT);
		}
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 4, 0)
		if (priv->phandle->fw_bands & BAND_6G) {
			priv->phandle->band = IEEE80211_BAND_6GHZ;
			woal_send_domain_info_cmd_fw(priv, MOAL_IOCTL_WAIT);
		}
#endif
		priv->phandle->band = band;
	}
}
#endif

/**
 * @brief Request the driver to change regulatory domain
 *
 * @param wiphy           A pointer to wiphy structure
 * @param request         A pointer to regulatory_request structure
 *
 * @return                0
 */
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 9, 0)
static void
#else
static int
#endif
woal_cfg80211_reg_notifier(struct wiphy *wiphy,
			   struct regulatory_request *request)
{
	moal_private *priv = NULL;
	moal_handle *handle = (moal_handle *)woal_get_wiphy_priv(wiphy);
	t_u8 region[COUNTRY_CODE_LEN];
	enum ieee80211_band band;
#if CFG80211_VERSION_CODE < KERNEL_VERSION(3, 9, 0)
	int ret = 0;
#endif
	t_u8 load_power_table = MFALSE;
	mlan_fw_info fw_info;
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 0, 0)
	char *reg_alpha2 = NULL;
#endif
	t_u8 dfs_region = NXP_DFS_UNKNOWN;
	struct ieee80211_supported_band *bands;

	ENTER();

	// channel 14 is not supported by the hardware, force disable it
	// even if regdb would accept it
	bands = wiphy->bands[NL80211_BAND_2GHZ];
	if (bands) {
		int i;
		for (i = 0; i < bands->n_channels; i++) {
			// channel 14 center freq has no define...
			if (bands->channels[i].center_freq == 2484) {
				bands->channels[i].flags |= IEEE80211_CHAN_DISABLED;
			}
		}
	}

	priv = woal_get_priv(handle, MLAN_BSS_ROLE_ANY);
	if (!priv || handle->driver_status || handle->surprise_removed) {
		PRINTM(MERROR, "Blocking reg_notifier in %s()\n", __func__);
		LEAVE();
#if CFG80211_VERSION_CODE < KERNEL_VERSION(3, 9, 0)
		return -EINVAL;
#else
		return;
#endif
	}
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 3, 0)
	dfs_region = request->dfs_region;
#endif
	PRINTM(MCMND,
	       "cfg80211 regulatory domain callback "
	       "%c%c initiator=%d dfs_region=%d\n",
	       request->alpha2[0], request->alpha2[1], request->initiator,
	       dfs_region);

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 0, 0)
	if (!(wiphy->regulatory_flags & REGULATORY_WIPHY_SELF_MANAGED))
#endif
		handle->dfs_region = dfs_region;

	memset(&fw_info, 0, sizeof(mlan_fw_info));
	woal_request_get_fw_info(priv, MOAL_IOCTL_WAIT, &fw_info);
	if (fw_info.force_reg) {
		PRINTM(MINFO,
		       "Regulatory domain is enforced in the on-chip OTP\n");
		LEAVE();
#if CFG80211_VERSION_CODE < KERNEL_VERSION(3, 9, 0)
		return -EINVAL;
#else
		return;
#endif
	}

	memset(region, 0, sizeof(region));
	moal_memcpy_ext(priv->phandle, region, request->alpha2,
			sizeof(request->alpha2), sizeof(region));
	region[2] = ' ';
	if ((handle->country_code[0] != request->alpha2[0]) ||
	    (handle->country_code[1] != request->alpha2[1])) {
		if (handle->params.cntry_txpwr) {
			t_u8 country_code[COUNTRY_CODE_LEN];
			handle->country_code[0] = request->alpha2[0];
			handle->country_code[1] = request->alpha2[1];
			handle->country_code[2] = ' ';
			memset(country_code, 0, sizeof(country_code));
			if (MTRUE == is_cfg80211_special_region_code(region)) {
				country_code[0] = 'W';
				country_code[1] = 'W';
			} else {
				country_code[0] = request->alpha2[0];
				country_code[1] = request->alpha2[1];
			}
			if (MLAN_STATUS_SUCCESS !=
			    woal_request_country_power_table(
				    priv, country_code, MOAL_IOCTL_WAIT, 0)) {
#if CFG80211_VERSION_CODE < KERNEL_VERSION(3, 9, 0)
				return -EFAULT;
#else
				return;
#endif
			}
			load_power_table = MTRUE;
		}
	}
	if (!handle->params.cntry_txpwr) {
		handle->country_code[0] = request->alpha2[0];
		handle->country_code[1] = request->alpha2[1];
		handle->country_code[2] = ' ';
		if (MTRUE != is_cfg80211_special_region_code(region)) {
			if (MLAN_STATUS_SUCCESS !=
			    woal_set_region_code(priv, handle->country_code))
				PRINTM(MERROR, "Set country code failed!\n");
		}
	}
	switch (request->initiator) {
	case NL80211_REGDOM_SET_BY_DRIVER:
		PRINTM(MCMND, "Regulatory domain BY_DRIVER\n");
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 0, 0)
		reg_alpha2 = priv->phandle->params.reg_alpha2;
		if ((handle->params.cntry_txpwr == CNTRY_RGPOWER_MODE) &&
		    !handle->params.txpwrlimit_cfg && load_power_table &&
		    reg_alpha2 && woal_is_valid_alpha2(reg_alpha2))
			queue_work(handle->evt_workqueue,
				   &handle->regulatory_work);
#endif
		break;
	case NL80211_REGDOM_SET_BY_CORE:
		PRINTM(MCMND, "Regulatory domain BY_CORE\n");
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 0, 0)
		if (handle->params.cntry_txpwr == CNTRY_RGPOWER_MODE &&
		    load_power_table && !handle->params.txpwrlimit_cfg)
			queue_work(handle->evt_workqueue,
				   &handle->regulatory_work);
#endif
		break;
	case NL80211_REGDOM_SET_BY_USER:
		PRINTM(MCMND, "Regulatory domain BY_USER\n");
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 0, 0)
		if (handle->params.cntry_txpwr == CNTRY_RGPOWER_MODE &&
		    load_power_table && !handle->params.txpwrlimit_cfg)
			queue_work(handle->evt_workqueue,
				   &handle->regulatory_work);
#endif
		break;
	case NL80211_REGDOM_SET_BY_COUNTRY_IE:
		PRINTM(MCMND, "Regulatory domain BY_COUNTRY_IE\n");
		break;
	}

	if (priv->wdev && priv->wdev->wiphy &&
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 0, 0)
	    !(wiphy->regulatory_flags & REGULATORY_WIPHY_SELF_MANAGED) &&
#endif
	    (request->initiator != NL80211_REGDOM_SET_BY_COUNTRY_IE)) {
		band = priv->phandle->band;
		priv->phandle->band = IEEE80211_BAND_2GHZ;
		woal_send_domain_info_cmd_fw(priv, MOAL_IOCTL_WAIT);
		if (priv->phandle->fw_bands & BAND_A) {
			priv->phandle->band = IEEE80211_BAND_5GHZ;
			woal_send_domain_info_cmd_fw(priv, MOAL_IOCTL_WAIT);
		}
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 4, 0)
		if (priv->phandle->fw_bands & BAND_6G) {
			priv->phandle->band = IEEE80211_BAND_6GHZ;
			woal_send_domain_info_cmd_fw(priv, MOAL_IOCTL_WAIT);
		}
#endif
		priv->phandle->band = band;
	}

	if (priv->wdev && priv->wdev->wiphy &&
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 0, 0)
	    !(wiphy->regulatory_flags & REGULATORY_WIPHY_SELF_MANAGED)
#endif
	) {
		woal_dnld_chan_attr(priv, MFALSE, MOAL_IOCTL_WAIT);
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 4, 0)
		woal_dnld_chan_attr(priv, MTRUE, MOAL_IOCTL_WAIT);
#endif
	}
	if (handle->params.edmac_ctrl)
		woal_edmac_cfg(priv, priv->phandle->country_code);

	LEAVE();
#if CFG80211_VERSION_CODE < KERNEL_VERSION(3, 9, 0)
	return ret;
#endif
}

static int woal_find_wps_ie_in_probereq(const t_u8 *ie, size_t len)
{
	int left_len = len;
	const t_u8 *pos = ie;
	t_u8 ie_id, ie_len;
	const IEEEtypes_VendorSpecific_t *pvendor_ie = NULL;
	const u8 wps_oui[4] = {0x00, 0x50, 0xf2, 0x04};

	while (left_len >= 2) {
		ie_id = *pos;
		ie_len = *(pos + 1);
		if ((ie_len + 2) > left_len)
			break;
		if (ie_id == VENDOR_SPECIFIC_221) {
			pvendor_ie = (const IEEEtypes_VendorSpecific_t *)pos;
			if (!memcmp(pvendor_ie->vend_hdr.oui, wps_oui,
				    sizeof(pvendor_ie->vend_hdr.oui)) &&
			    pvendor_ie->vend_hdr.oui_type == wps_oui[3])
				return MTRUE;
		}

		pos += (ie_len + 2);
		left_len -= (ie_len + 2);
	}

	return MFALSE;
}

#ifdef UAP_CFG80211
/** scan result expired value */
#define SCAN_RESULT_EXPIRTED 1
/**
 *  @brief check if the scan result expired
 *
 *  @param priv         A pointer to moal_private
 *
 *
 *  @return             MTRUE/MFALSE;
 */
static t_u8 woal_is_uap_scan_result_expired(moal_private *priv)
{
	mlan_scan_resp scan_resp;
	wifi_timeval t;
	ENTER();

	memset(&scan_resp, 0, sizeof(scan_resp));

	if (MLAN_STATUS_SUCCESS !=
	    woal_get_scan_table(priv, MOAL_IOCTL_WAIT, &scan_resp)) {
		LEAVE();
		return MTRUE;
	}
	if (!scan_resp.num_in_scan_table) {
		LEAVE();
		return MTRUE;
	}
	woal_get_monotonic_time(&t);
	if (t.time_sec > (scan_resp.age_in_secs + SCAN_RESULT_EXPIRTED)) {
		LEAVE();
		return MTRUE;
	}
	LEAVE();
	return MFALSE;
}
#endif

/**
 *  @brief check if the scan result ageout
 *
 *  @param priv         A pointer to moal_private
 *
 *
 *  @return             MTRUE/MFALSE;
 */
static t_u8 wlan_check_scan_table_ageout(moal_private *priv)
{
	mlan_scan_resp scan_resp;
	wifi_timeval t;
	ENTER();
	memset(&scan_resp, 0, sizeof(scan_resp));
	if (MLAN_STATUS_SUCCESS !=
	    woal_get_scan_table(priv, MOAL_IOCTL_WAIT, &scan_resp)) {
		LEAVE();
		return MFALSE;
	}
	woal_get_monotonic_time(&t);
#define CFG80211_SCAN_RESULT_AGEOUT 20
	if (t.time_sec >
	    (scan_resp.age_in_secs + CFG80211_SCAN_RESULT_AGEOUT)) {
		LEAVE();
		return MFALSE;
	}
	PRINTM(MCMND, "Scan: Keep Previous result\n");
	LEAVE();
	return MTRUE;
}

/**
 * @brief Cancel remain on channel before scan request process

 * @param priv            A pointer to moal_private
 *
 * @return                none
 */
static void woal_cancel_remain_on_channel(moal_private *priv)
{
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(2, 6, 39)
	if (priv->phandle->remain_on_channel) {
		t_u8 channel_status;
		moal_private *remain_priv;
		remain_priv =
			priv->phandle->priv[priv->phandle->remain_bss_index];
		if (remain_priv) {
			PRINTM(MMSG,
			       "Cancel Remain on Channel before scan request\n");
			if (woal_cfg80211_remain_on_channel_cfg(
				    remain_priv, MOAL_IOCTL_WAIT, MTRUE,
				    &channel_status, NULL, 0, 0))
				PRINTM(MERROR,
				       "Fail to cancel remain on channel %s %d\n",
				       __func__, __LINE__);
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
			priv->phandle->remain_on_channel = MFALSE;
		}
	}
#endif
}

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 6, 0)
/**
 * @brief Request the driver to do a scan. Always returning
 * zero meaning that the scan request is given to driver,
 * and will be valid until passed to cfg80211_scan_done().
 * To inform scan results, call cfg80211_inform_bss().
 *
 * @param wiphy           A pointer to wiphy structure
 * @param request         A pointer to cfg80211_scan_request structure
 *
 * @return                0 -- success, otherwise fail
 */
static int woal_cfg80211_scan(struct wiphy *wiphy,
			      struct cfg80211_scan_request *request)
#else
/**
 * @brief Request the driver to do a scan. Always returning
 * zero meaning that the scan request is given to driver,
 * and will be valid until passed to cfg80211_scan_done().
 * To inform scan results, call cfg80211_inform_bss().
 *
 * @param wiphy           A pointer to wiphy structure
 * @param dev             A pointer to net_device structure
 * @param request         A pointer to cfg80211_scan_request structure
 *
 * @return                0 -- success, otherwise fail
 */
static int woal_cfg80211_scan(struct wiphy *wiphy, struct net_device *dev,
			      struct cfg80211_scan_request *request)
#endif
{
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 6, 0)
	struct net_device *dev = request->wdev->netdev;
#endif
	moal_private *priv = (moal_private *)woal_get_netdev_priv(dev);
	wlan_user_scan_cfg *scan_req = NULL;
	mlan_bss_info bss_info;
	mlan_scan_cfg scan_cfg;
	struct ieee80211_channel *chan;
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 19, 0)
	t_u8 buf[ETH_ALEN];
#endif
	int ret = 0, i, num_chans;
	unsigned long flags;
	t_u16 max_gap = 0;
	t_u16 total_scan_time = 0;

	ENTER();

	PRINTM(MINFO, "Received scan request on %s\n", dev->name);
	if (priv->phandle->driver_status || priv->phandle->surprise_removed) {
		PRINTM(MERROR,
		       "Block woal_cfg80211_scan in abnormal driver state\n");
		LEAVE();
		return -EFAULT;
	}
	if (priv->phandle->scan_pending_on_block == MTRUE)
		woal_sched_timeout(200);
	if (priv->phandle->scan_pending_on_block == MTRUE) {
		PRINTM(MCMND, "scan already in processing...\n");
		LEAVE();
		return -EAGAIN;
	}
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 2, 0)
	if (priv->last_event & EVENT_BG_SCAN_REPORT) {
		PRINTM(MCMND, "block scan while pending BGSCAN result\n");
		priv->last_event = 0;
		LEAVE();
		return -EAGAIN;
	}
#endif
#if defined(STA_CFG80211) || defined(UAP_CFG80211)
#ifdef WIFI_DIRECT_SUPPORT
#if CFG80211_VERSION_CODE >= WIFI_DIRECT_KERNEL_VERSION
	if (priv->phandle->is_go_timer_set &&
	    priv->wdev->iftype != NL80211_IFTYPE_P2P_GO) {
		PRINTM(MCMND, "block scan in go timer....\n");
		LEAVE();
		return -EAGAIN;
	}
#endif
#endif
#endif
	cancel_delayed_work(&priv->phandle->scan_timeout_work);
	priv->phandle->fake_scan_complete = priv->fake_scan_complete;
	if (priv->fake_scan_complete) {
		PRINTM(MEVENT, "fake scan complete flag is on\n");
		spin_lock_irqsave(&priv->phandle->scan_req_lock, flags);
		priv->phandle->scan_request = request;
		spin_unlock_irqrestore(&priv->phandle->scan_req_lock, flags);
		queue_delayed_work(priv->phandle->evt_workqueue,
				   &priv->phandle->scan_timeout_work,
				   msecs_to_jiffies(1000));
		return ret;
	}

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
	/* This check is to prevent the situation where
	 * new scan request comes while Auth is not completed */
	if (priv->auth_flag & HOST_MLME_AUTH_PENDING) {
		PRINTM(MCMND, "Block scan as auth is pending\n");
		LEAVE();
		return -EAGAIN;
	}
#endif

	/** Cance remain on channel */
	woal_cancel_remain_on_channel(priv);

	memset(&bss_info, 0, sizeof(bss_info));
	if (MLAN_STATUS_SUCCESS ==
	    woal_get_bss_info(priv, MOAL_IOCTL_WAIT, &bss_info)) {
		if (bss_info.scan_block) {
			PRINTM(MEVENT,
			       "Block scan in mlan module for scan_request %p\n",
			       request);
			spin_lock_irqsave(&priv->phandle->scan_req_lock, flags);
			priv->phandle->fake_scan_complete = MTRUE;
			priv->phandle->scan_request = request;
			spin_unlock_irqrestore(&priv->phandle->scan_req_lock,
					       flags);
			queue_delayed_work(priv->phandle->evt_workqueue,
					   &priv->phandle->scan_timeout_work,
					   msecs_to_jiffies(1000));
			return ret;
		}
	}
	if (priv->phandle->scan_request &&
	    priv->phandle->scan_request != request) {
		PRINTM(MCMND,
		       "different scan_request is coming before previous one is finished on %s...\n",
		       dev->name);
		LEAVE();
		return -EBUSY;
	}
	spin_lock_irqsave(&priv->phandle->scan_req_lock, flags);
	priv->phandle->scan_request = request;
	spin_unlock_irqrestore(&priv->phandle->scan_req_lock, flags);
	if (is_zero_timeval(priv->phandle->scan_time_start)) {
		woal_get_monotonic_time(&priv->phandle->scan_time_start);
		PRINTM(MINFO, "%s : start_timeval=%d:%d \n", __func__,
		       priv->phandle->scan_time_start.time_sec,
		       priv->phandle->scan_time_start.time_usec);
	}
	scan_req = kmalloc(sizeof(wlan_user_scan_cfg), GFP_KERNEL);
	if (!scan_req) {
		PRINTM(MERROR, "Failed to alloc memory for scan_req\n");
		LEAVE();
		return -ENOMEM;
	}
	memset(scan_req, 0x00, sizeof(wlan_user_scan_cfg));
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 7, 0)
	if (!is_broadcast_ether_addr(request->bssid)) {
		moal_memcpy_ext(priv->phandle, scan_req->specific_bssid,
				request->bssid, ETH_ALEN,
				sizeof(scan_req->specific_bssid));
		PRINTM(MIOCTL, "scan: bssid=" MACSTR "\n",
		       MAC2STR(scan_req->specific_bssid));
	}
#endif

	memset(&scan_cfg, 0, sizeof(mlan_scan_cfg));
	if (MLAN_STATUS_SUCCESS != woal_get_scan_config(priv, &scan_cfg)) {
		PRINTM(MERROR, "Fail to get scan request IE\n");
	}
#ifdef WIFI_DIRECT_SUPPORT
	if (priv->phandle->miracast_mode)
		scan_req->scan_chan_gap = priv->phandle->scan_chan_gap;
	else {
#endif
		if (scan_cfg.scan_chan_gap)
			scan_req->scan_chan_gap = scan_cfg.scan_chan_gap;
		else if (woal_is_any_interface_active(priv->phandle))
			scan_req->scan_chan_gap = priv->phandle->scan_chan_gap;
		else
			scan_req->scan_chan_gap = 0;
#ifdef WIFI_DIRECT_SUPPORT
	}
#endif

	scan_req->scan_cfg_only = MTRUE;
	if (scan_cfg.ext_scan == 3)
		scan_req->ext_scan_type = EXT_SCAN_ENHANCE;

	for (i = 0; i < priv->phandle->scan_request->n_ssids; i++) {
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 10, 0)
		if (request->scan_6ghz) {
			if (i &&
			    !priv->phandle->scan_request->ssids[i].ssid_len)
				continue;
		}
#endif
		// coverity[cert_arr30_c_violation: SUPPRESS]
		moal_memcpy_ext(priv->phandle, scan_req->ssid_list[i].ssid,
				priv->phandle->scan_request->ssids[i].ssid,
				priv->phandle->scan_request->ssids[i].ssid_len,
				sizeof(scan_req->ssid_list[i].ssid));
		if (priv->phandle->scan_request->ssids[i].ssid_len)
			scan_req->ssid_list[i].max_len = 0;
		else
			scan_req->ssid_list[i].max_len = 0xff;
		PRINTM(MIOCTL, "scan: ssid=%s\n", scan_req->ssid_list[i].ssid);
	}
#if defined(WIFI_DIRECT_SUPPORT)
#if CFG80211_VERSION_CODE >= WIFI_DIRECT_KERNEL_VERSION
	if (priv->bss_type == MLAN_BSS_TYPE_WIFIDIRECT &&
	    priv->phandle->scan_request->n_ssids) {
		if (!memcmp(scan_req->ssid_list[0].ssid, "DIRECT-", 7))
			scan_req->ssid_list[0].max_len = 0xfe;
	}
#endif
#endif
	if (priv->scan_setband_mask) {
		PRINTM(MCMD_D,
		       "cfg80211_scan: scan_setband mask is set to %d\n",
		       priv->scan_setband_mask);
	}
	for (i = 0, num_chans = 0;
	     i < (int)MIN(WLAN_USER_SCAN_CHAN_MAX,
			  priv->phandle->scan_request->n_channels);
	     i++) {
		chan = priv->phandle->scan_request->channels[i];
		if (MFALSE == is_scan_band_allowed(priv, chan))
			continue;
		scan_req->chan_list[num_chans].chan_number = chan->hw_value;
		scan_req->chan_list[num_chans].radio_type =
			woal_ieee_band_to_radio_type(chan->band);
		if ((chan->flags & IEEE80211_CHAN_PASSIVE_SCAN) ||
		    !priv->phandle->scan_request->n_ssids)
			scan_req->chan_list[num_chans].scan_type =
				MLAN_SCAN_TYPE_PASSIVE;
		else if (chan->flags & IEEE80211_CHAN_RADAR)
			scan_req->chan_list[num_chans].scan_type =
				MLAN_SCAN_TYPE_PASSIVE_TO_ACTIVE;
		else
			scan_req->chan_list[num_chans].scan_type =
				MLAN_SCAN_TYPE_ACTIVE;
		PRINTM(MCMD_D, "cfg80211_scan: chan=%d chan->flag=0x%x\n",
		       chan->hw_value, chan->flags);
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 8, 0)
		scan_req->chan_list[num_chans].scan_time =
			priv->phandle->scan_request->duration;
#else
		scan_req->chan_list[num_chans].scan_time = 0;
#endif
#if defined(WIFI_DIRECT_SUPPORT)
#if CFG80211_VERSION_CODE >= WIFI_DIRECT_KERNEL_VERSION
		if (priv->bss_type == MLAN_BSS_TYPE_WIFIDIRECT &&
		    priv->phandle->scan_request->n_ssids) {
			if (!memcmp(scan_req->ssid_list[0].ssid, "DIRECT-", 7))
				scan_req->chan_list[num_chans].scan_time =
					MIN_SPECIFIC_SCAN_CHAN_TIME;
		}
#endif
#endif
#ifdef WIFI_DIRECT_SUPPORT
		if (priv->phandle->miracast_mode)
			scan_req->chan_list[num_chans].scan_time =
				priv->phandle->miracast_scan_time;
		else if (woal_is_any_interface_active(priv->phandle)) {
			if (chan->flags & IEEE80211_CHAN_PASSIVE_SCAN)
				scan_req->chan_list[num_chans].scan_time =
					INIT_PASSIVE_SCAN_CHAN_TIME;
			else if (priv->bss_type == MLAN_BSS_TYPE_STA &&
				 scan_req->chan_list[num_chans].scan_type ==
					 MLAN_SCAN_TYPE_PASSIVE) {
				/*
				 * Set passive scan time to 110ms to discover
				 * all nearby AP's, Current 40ms passive scan
				 * time does not scan all AP's. As beacon
				 * interval is 100ms and dwell time on channel
				 * is 40ms There are issues with 40ms scan time:
				 * 1. Passive scan does list limited nearby
				 * AP's. This change is limited to below
				 * scenario only:
				 * 1. STA is in connected state
				 * 2. Scan type is passive
				 */
				scan_req->chan_list[num_chans].scan_time =
					PASSIVE_SCAN_CHAN_TIME;
			} else {
				scan_req->chan_list[num_chans].scan_time = MIN(
					MIN_SPECIFIC_SCAN_CHAN_TIME,
					scan_cfg.scan_time.specific_scan_time);
			}
		}
#endif
#ifdef UAP_CFG80211
		if (GET_BSS_ROLE(priv) == MLAN_BSS_ROLE_UAP) {
			if (priv->bss_type != MLAN_BSS_TYPE_UAP)
				scan_req->scan_chan_gap = 0;

			if (!woal_is_uap_scan_result_expired(priv))
				scan_req->chan_list[num_chans].scan_time =
					MIN_SPECIFIC_SCAN_CHAN_TIME;

			else
				scan_req->chan_list[num_chans].scan_time =
					PASSIVE_SCAN_CHAN_TIME;
		}
#endif
		if (scan_req->chan_list[num_chans].scan_time)
			total_scan_time +=
				scan_req->chan_list[num_chans].scan_time;
		else
			total_scan_time += PASSIVE_SCAN_CHAN_TIME;
		num_chans++;
	}

	if ((total_scan_time < MAX_SCAN_TIMEOUT) && num_chans)
		max_gap = (MAX_SCAN_TIMEOUT - total_scan_time) / num_chans;
	if (scan_req->scan_chan_gap)
		scan_req->scan_chan_gap = MIN(max_gap, scan_req->scan_chan_gap);
	/** indicate FW, gap is optional */
	if (scan_req->scan_chan_gap && priv->phandle->pref_mac)
		scan_req->scan_chan_gap |= GAP_FLAG_OPTIONAL;

	if (priv->phandle->scan_request->ie &&
	    priv->phandle->scan_request->ie_len) {
		if (woal_find_wps_ie_in_probereq(
			    (const t_u8 *)priv->phandle->scan_request->ie,
			    priv->phandle->scan_request->ie_len)) {
			PRINTM(MIOCTL,
			       "Notify firmware only keep probe response\n");
			scan_req->proberesp_only = MTRUE;
		}
		if (MLAN_STATUS_SUCCESS !=
		    woal_cfg80211_mgmt_frame_ie(
			    priv, NULL, 0, NULL, 0, NULL, 0,
			    (const t_u8 *)priv->phandle->scan_request->ie,
			    priv->phandle->scan_request->ie_len,
			    MGMT_MASK_PROBE_REQ, MOAL_IOCTL_WAIT)) {
			PRINTM(MERROR, "Fail to set scan request IE\n");
			ret = -EFAULT;
			goto done;
		}
	} else {
		/** Clear SCAN IE in Firmware */
		if (priv->probereq_index != MLAN_CUSTOM_IE_AUTO_IDX_MASK) {
			if (MLAN_STATUS_SUCCESS !=
			    woal_cfg80211_mgmt_frame_ie(
				    priv, NULL, 0, NULL, 0, NULL, 0, NULL, 0,
				    MGMT_MASK_PROBE_REQ, MOAL_IOCTL_WAIT)) {
				PRINTM(MERROR,
				       "Fail to clear scan request IE\n");
				ret = -EFAULT;
				goto done;
			}
		}
	}
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 19, 0)
	if (request->flags & NL80211_SCAN_FLAG_RANDOM_ADDR) {
		PRINTM(MIOCTL, "NL80211_SCAN_FLAG_RANDOM_ADDR is set\n");
		get_random_bytes(buf, ETH_ALEN);
		for (i = 0; i < ETH_ALEN; i++) {
			buf[i] &= ~request->mac_addr_mask[i];
			buf[i] |= request->mac_addr[i] &
				  request->mac_addr_mask[i];
		}
		moal_memcpy_ext(priv->phandle, scan_req->random_mac, buf,
				ETH_ALEN, sizeof(scan_req->random_mac));
	} else
#endif
		moal_memcpy_ext(priv->phandle, scan_req->random_mac,
				priv->random_mac, ETH_ALEN,
				sizeof(scan_req->random_mac));

	PRINTM(MCMND, "wlan:random_mac " MACSTR "\n",
	       MAC2STR(scan_req->random_mac));
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 10, 0)
	if (request->scan_6ghz && request->n_6ghz_params) {
		u8 channel_idx = 0;
		for (i = 0; i < request->n_6ghz_params; i++) {
			channel_idx = request->scan_6ghz_params[i].channel_idx;
			if (channel_idx >
			    MIN(WLAN_USER_SCAN_CHAN_MAX, request->n_channels))
				continue;
			scan_req->chan_list[channel_idx].rnr_flag = MTRUE;
			if (scan_req->num_6g_scan_params <
			    WLAN_MAX_6G_SCAN_PARAMS_LIST) {
				scan_req->scan_param_list
					[scan_req->num_6g_scan_params]
						.channel =
					scan_req->chan_list[channel_idx]
						.chan_number;
				scan_req->scan_param_list
					[scan_req->num_6g_scan_params]
						.short_ssid =
					request->scan_6ghz_params[i].short_ssid;
				moal_memcpy_ext(
					priv->phandle,
					scan_req->scan_param_list
						[scan_req->num_6g_scan_params]
							.bssid,
					request->scan_6ghz_params[i].bssid,
					ETH_ALEN, ETH_ALEN);
				if (request->scan_6ghz_params[i]
					    .short_ssid_valid)
					scan_req->scan_param_list
						[scan_req->num_6g_scan_params]
							.flags |=
						SHORT_SSID_VALID;
				if (request->scan_6ghz_params[i]
					    .unsolicited_probe)
					scan_req->scan_param_list
						[scan_req->num_6g_scan_params]
							.flags |=
						UNSOLICITED_PROBE;
				scan_req->num_6g_scan_params++;
			}
		}
	}
#endif
	if (priv->phandle->params.keep_previous_scan)
		scan_req->keep_previous_scan =
			wlan_check_scan_table_ageout(priv);

	if (MLAN_STATUS_SUCCESS != woal_do_scan(priv, scan_req)) {
		PRINTM(MERROR, "woal_do_scan fails!\n");
		ret = -EAGAIN;
		goto done;
	}
done:
	if (ret) {
		spin_lock_irqsave(&priv->phandle->scan_req_lock, flags);
		woal_cfg80211_scan_done(request, MTRUE);
		priv->phandle->scan_request = NULL;
		priv->phandle->scan_priv = NULL;
		spin_unlock_irqrestore(&priv->phandle->scan_req_lock, flags);
	} else {
		PRINTM(MMSG, "wlan: %s START SCAN\n", dev->name);
		queue_delayed_work(
			priv->phandle->evt_workqueue,
			&priv->phandle->scan_timeout_work,
			msecs_to_jiffies(priv->phandle->scan_timeout));
	}
	kfree(scan_req);
	LEAVE();
	return ret;
}
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 5, 0)
static void woal_cfg80211_abort_scan(struct wiphy *wiphy,
				     struct wireless_dev *wdev)
{
	moal_private *priv = (moal_private *)woal_get_netdev_priv(wdev->netdev);
	ENTER();
	PRINTM(MMSG, "wlan: ABORT SCAN start\n");
	woal_cancel_scan(priv, MOAL_IOCTL_WAIT);
	LEAVE();
	return;
}
#endif
/**
 * @brief construct and send ft action request
 *
 *  @param priv     A pointer to moal_private structure
 * @param ie       A pointer to ft ie
 * @param le       Value of ie len
 * @param bssid    A pointer to target ap bssid
 * @
 * @return         0 -- success, otherwise fail
 */
static int woal_send_ft_action_requst(moal_private *priv, t_u8 *ie, t_u8 len,
				      t_u8 *bssid, t_u8 *target_ap)
{
	IEEE80211_MGMT *mgmt = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;
	pmlan_buffer pmbuf = NULL;
	t_u32 pkt_type;
	t_u32 tx_control;
	t_u16 packet_len = 0;
	t_u16 pkt_len = 0;
	t_u8 addr[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
	int ret = 0;

	ENTER();

	/* pkt_type + tx_control */
#define HEADER_SIZE 8
	/* frmctl + durationid + addr1 + addr2 + addr3 + seqctl + addr4*/
#define MGMT_HEADER_LEN (2 + 2 + 6 + 6 + 6 + 2 + 6)
	/* 14   = category + action + sta addr + target ap */
#define FT_REQUEST_LEN 14
	packet_len = (t_u16)len + MGMT_HEADER_LEN + FT_REQUEST_LEN;
	pmbuf = woal_alloc_mlan_buffer(priv->phandle,
				       MLAN_MIN_DATA_HEADER_LEN + HEADER_SIZE +
					       packet_len + sizeof(packet_len));
	if (!pmbuf) {
		PRINTM(MERROR, "Fail to allocate mlan_buffer\n");
		ret = -ENOMEM;
		goto done;
	}

	pmbuf->data_offset = MLAN_MIN_DATA_HEADER_LEN;
	pkt_type = MRVL_PKT_TYPE_MGMT_FRAME;
	tx_control = 0;
	/* Add pkt_type and tx_control */
	moal_memcpy_ext(priv->phandle, pmbuf->pbuf + pmbuf->data_offset,
			&pkt_type, sizeof(pkt_type), sizeof(pkt_type));
	moal_memcpy_ext(priv->phandle,
			pmbuf->pbuf + pmbuf->data_offset + sizeof(pkt_type),
			&tx_control, sizeof(tx_control), sizeof(tx_control));
	/*Add packet len*/
	pkt_len = woal_cpu_to_le16(packet_len);
	moal_memcpy_ext(priv->phandle,
			pmbuf->pbuf + pmbuf->data_offset + HEADER_SIZE,
			&pkt_len, sizeof(pkt_len), sizeof(pkt_len));

	mgmt = (IEEE80211_MGMT *)(pmbuf->pbuf + pmbuf->data_offset +
				  HEADER_SIZE + sizeof(packet_len));
	memset(mgmt, 0, MGMT_HEADER_LEN);
	mgmt->frame_control =
		woal_cpu_to_le16(IEEE80211_FTYPE_MGMT | IEEE80211_STYPE_ACTION);
	moal_memcpy_ext(priv->phandle, mgmt->da, bssid, ETH_ALEN,
			sizeof(mgmt->da));
	moal_memcpy_ext(priv->phandle, mgmt->sa, priv->current_addr, ETH_ALEN,
			sizeof(mgmt->sa));
	moal_memcpy_ext(priv->phandle, mgmt->bssid, bssid, ETH_ALEN,
			sizeof(mgmt->bssid));
	moal_memcpy_ext(priv->phandle, mgmt->addr4, addr, ETH_ALEN,
			sizeof(mgmt->addr4));

	mgmt->u.ft_req.category = 0x06; /**ft action code 0x6*/
	mgmt->u.ft_req.action = 0x1; /**ft action request*/
	moal_memcpy_ext(priv->phandle, mgmt->u.ft_req.sta_addr,
			priv->current_addr, ETH_ALEN,
			sizeof(mgmt->u.ft_req.sta_addr));
	moal_memcpy_ext(priv->phandle, mgmt->u.ft_req.target_ap_addr, target_ap,
			ETH_ALEN, sizeof(mgmt->u.ft_req.target_ap_addr));

	if (ie && len)
		moal_memcpy_ext(priv->phandle,
				(t_u8 *)(&mgmt->u.ft_req.variable), ie, len,
				len);

	pmbuf->data_len = HEADER_SIZE + packet_len + sizeof(packet_len);
	pmbuf->buf_type = MLAN_BUF_TYPE_RAW_DATA;
	pmbuf->bss_index = priv->bss_index;
	pmbuf->priority = 7;

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
 * @brief construct and send ft auth request
 *
 *  @param priv     A pointer to moal_private structure
 * @param ie       A pointer to ft ie
 * @param le       Value of ie len
 * @param bssid    A pointer to target ap bssid
 * @
 * @return         0 -- success, otherwise fail
 */
static int woal_send_ft_auth_requst(moal_private *priv, t_u8 *ie, t_u8 len,
				    t_u8 *bssid)
{
	IEEE80211_MGMT *mgmt = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;
	pmlan_buffer pmbuf = NULL;
	t_u32 pkt_type;
	t_u32 tx_control;
	t_u16 packet_len = 0;
	t_u16 pkt_len;
	t_u8 addr[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
	int ret = 0;

	ENTER();
	/* pkt_type + tx_control */
#define HEADER_SIZE 8
	/* frmctl + durationid + addr1 + addr2 + addr3 + seqctl + addr4*/
#define MGMT_HEADER_LEN (2 + 2 + 6 + 6 + 6 + 2 + 6)
	/* 6   = auth_alg + auth_transaction +auth_status*/
#define AUTH_BODY_LEN 6
	packet_len = (t_u16)len + MGMT_HEADER_LEN + AUTH_BODY_LEN;
	pmbuf = woal_alloc_mlan_buffer(priv->phandle,
				       MLAN_MIN_DATA_HEADER_LEN + HEADER_SIZE +
					       packet_len + sizeof(packet_len));
	if (!pmbuf) {
		PRINTM(MERROR, "Fail to allocate mlan_buffer\n");
		ret = -ENOMEM;
		goto done;
	}

	pmbuf->data_offset = MLAN_MIN_DATA_HEADER_LEN;
	pkt_type = MRVL_PKT_TYPE_MGMT_FRAME;
	tx_control = 0;
	/* Add pkt_type and tx_control */
	moal_memcpy_ext(priv->phandle, pmbuf->pbuf + pmbuf->data_offset,
			&pkt_type, sizeof(pkt_type), sizeof(pkt_type));
	moal_memcpy_ext(priv->phandle,
			pmbuf->pbuf + pmbuf->data_offset + sizeof(pkt_type),
			&tx_control, sizeof(tx_control), sizeof(tx_control));
	/*Add packet len*/
	pkt_len = woal_cpu_to_le16(packet_len);
	moal_memcpy_ext(priv->phandle,
			pmbuf->pbuf + pmbuf->data_offset + HEADER_SIZE,
			&pkt_len, sizeof(pkt_len), sizeof(pkt_len));

	mgmt = (IEEE80211_MGMT *)(pmbuf->pbuf + pmbuf->data_offset +
				  HEADER_SIZE + sizeof(packet_len));
	memset(mgmt, 0, MGMT_HEADER_LEN);
	mgmt->frame_control =
		woal_cpu_to_le16(IEEE80211_FTYPE_MGMT | IEEE80211_STYPE_AUTH);
	moal_memcpy_ext(priv->phandle, mgmt->da, bssid, ETH_ALEN,
			sizeof(mgmt->da));
	moal_memcpy_ext(priv->phandle, mgmt->sa, priv->current_addr, ETH_ALEN,
			sizeof(mgmt->sa));
	moal_memcpy_ext(priv->phandle, mgmt->bssid, bssid, ETH_ALEN,
			sizeof(mgmt->bssid));
	moal_memcpy_ext(priv->phandle, mgmt->addr4, addr, ETH_ALEN,
			sizeof(mgmt->addr4));

	mgmt->u.auth.auth_alg = woal_cpu_to_le16(WLAN_AUTH_FT);
	mgmt->u.auth.auth_transaction = woal_cpu_to_le16(1);
	mgmt->u.auth.status_code = woal_cpu_to_le16(0);
	if (ie && len)
		moal_memcpy_ext(priv->phandle, (t_u8 *)(&mgmt->u.auth.variable),
				ie, len, len);

	pmbuf->data_len = HEADER_SIZE + packet_len + sizeof(packet_len);
	pmbuf->buf_type = MLAN_BUF_TYPE_RAW_DATA;
	pmbuf->bss_index = priv->bss_index;
	pmbuf->priority = 7;

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
 * @brief connect the AP through ft over air.
 *
 * @param priv            A pointer to moal_private structure
 * @param bssid           A pointer to bssid
 * @param chan            struct ieee80211_channel
 *
 * @return                0 -- success, otherwise fail
 */
static int woal_connect_ft_over_air(moal_private *priv, t_u8 *bssid,
				    struct ieee80211_channel *chan)
{
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(2, 6, 39)
	t_u8 status = 0;
#endif

	t_u8 wait_option = MOAL_IOCTL_WAIT;
	int ret = 0;
	long timeout = 0;

	ENTER();

	if (!bssid) {
		PRINTM(MERROR,
		       "Invalid bssid, unable to connect AP to through FT\n");
		LEAVE();
		return -EFAULT;
	}

	/*enable auth register frame*/
	woal_mgmt_frame_register(priv, IEEE80211_STYPE_AUTH, MTRUE);

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(2, 6, 39)
	if (woal_cfg80211_remain_on_channel_cfg(priv, wait_option, MFALSE,
						&status, chan, 0,
						priv->auth_tx_wait_time)) {
		PRINTM(MERROR, "Failed remain on channel config\n");
	}
#endif

	/*construct auth request and send out*/
	woal_send_ft_auth_requst(priv, priv->ft_ie, priv->ft_ie_len, bssid);
	PRINTM(MMSG, "wlan: send out FT auth,wait for auth response\n");
	/*wait until received auth response*/
	priv->ft_wait_condition = MFALSE;
	timeout = wait_event_timeout(priv->ft_wait_q, priv->ft_wait_condition,
				     1 * HZ);
	if (!timeout) {
		/*connet fail */
		if (!priv->ft_roaming_triggered_by_driver) {
			woal_inform_bss_from_scan_result(priv, NULL,
							 wait_option);
			cfg80211_connect_result(priv->netdev, priv->cfg_bssid,
						NULL, 0, NULL, 0,
						WLAN_STATUS_SUCCESS,
						GFP_KERNEL);
		}
		priv->ft_roaming_triggered_by_driver = MFALSE;
		PRINTM(MMSG, "wlan: keep connected to bssid " MACSTR "\n",
		       MAC2STR(priv->cfg_bssid));
	} else {
		PRINTM(MMSG, "wlan: FT auth received \n");
		moal_memcpy_ext(priv->phandle, priv->target_ap_bssid, bssid,
				ETH_ALEN, sizeof(priv->target_ap_bssid));
	}

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(2, 6, 39)
	if (woal_cfg80211_remain_on_channel_cfg(priv, wait_option, MTRUE,
						&status, NULL, 0, 0)) {
		PRINTM(MERROR, "Failed to cancel remain on channel\n");
	}
#endif

	woal_mgmt_frame_register(priv, IEEE80211_STYPE_AUTH, MFALSE);

	LEAVE();
	return ret;
}

/**
 * @brief connect the AP through ft over DS.
 *
 * @param priv            A pointer to moal_private structure
 * @param bssid           A pointer to bssid
 * @param chan            struct ieee80211_channel
 *
 * @return                0 -- success, otherwise fail
 */
static int woal_connect_ft_over_ds(moal_private *priv, t_u8 *bssid,
				   struct ieee80211_channel *pchan)
{
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(2, 6, 39)
	t_u8 status = 0;
#endif
	t_u8 wait_option = MOAL_IOCTL_WAIT;
	int ret = 0;
	long timeout = 0;

	ENTER();

	if (priv->media_connected) {
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(2, 6, 39)
		if (woal_cfg80211_remain_on_channel_cfg(priv, wait_option,
							MFALSE, &status, pchan,
							0, 1200)) {
			PRINTM(MERROR,
			       "Failed to configure remain on channel\n");
		}
#endif
		/*construct ft action request and send out*/
		woal_send_ft_action_requst(priv, priv->ft_ie, priv->ft_ie_len,
					   (t_u8 *)priv->cfg_bssid, bssid);
		PRINTM(MMSG,
		       "wlan: send out FT request,wait for FT response\n");
		/*wait until received auth response*/
		priv->ft_wait_condition = MFALSE;
		timeout = wait_event_timeout(priv->ft_wait_q,
					     priv->ft_wait_condition, 1 * HZ);
		if (!timeout) {
			/*go over air, as current AP may be unreachable */
			PRINTM(MMSG, "wlan: go over air\n");
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(2, 6, 39)
			if (woal_cfg80211_remain_on_channel_cfg(
				    priv, wait_option, MTRUE, &status, NULL, 0,
				    0)) {
				PRINTM(MERROR,
				       "Failed to cancel remain on channel\n");
			}
#endif
			woal_connect_ft_over_air(priv, bssid, pchan);
			LEAVE();
			return ret;
		} else {
			PRINTM(MMSG, "wlan: received FT response\n");
			moal_memcpy_ext(priv->phandle, priv->target_ap_bssid,
					bssid, ETH_ALEN,
					sizeof(priv->target_ap_bssid));
		}
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(2, 6, 39)
		if (woal_cfg80211_remain_on_channel_cfg(
			    priv, wait_option, MTRUE, &status, NULL, 0, 0)) {
			PRINTM(MERROR, "Failed to cancel remain on channel\n");
		}
#endif
	}

	LEAVE();
	return ret;
}

/**
 * @brief start FT Roaming.
 *
 * @param priv               A pointer to moal_private structure
 * @param ssid_bssid         A pointer to mlan_ssid_bssid structure
 *
 *
 * @return                   0 -- success, otherwise fail
 */
static int woal_start_ft_roaming(moal_private *priv,
				 mlan_ssid_bssid *ssid_bssid)
{
	struct ieee80211_channel chan;
	int ret = 0;

	ENTER();

	PRINTM(MEVENT, "Try to start FT roaming......\n");
	chan.band = (ssid_bssid->channel < 36) ? IEEE80211_BAND_2GHZ :
						 IEEE80211_BAND_5GHZ;
	chan.center_freq = ieee80211_channel_to_frequency(ssid_bssid->channel
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(2, 6, 39)
							  ,
							  chan.band
#endif
	);

	if (!(priv->last_event & EVENT_PRE_BCN_LOST) &&
	    (ssid_bssid->ft_cap & MBIT(0))) {
		woal_connect_ft_over_ds(priv, (t_u8 *)&ssid_bssid->bssid,
					&chan);
	} else {
		/*if pre beacon lost, it need to send auth request instead ft
		 * action request when ft over ds */
		woal_connect_ft_over_air(priv, (t_u8 *)&ssid_bssid->bssid,
					 &chan);
	}

	LEAVE();
	return ret;
}

#if CFG80211_VERSION_CODE > KERNEL_VERSION(4, 12, 14)
/**
 * @brief Set psk to firmware in embedded supplicant mode
 *
 * @param wiphy           A pointer to priv structure
 * @param sme             A pointer to cfg80211_connect_params structure
 *
 * @return                0 -- success, otherwise fail
 */
static int woal_cfg80211_set_psk(moal_private *priv,
				 struct cfg80211_connect_params *sme)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_sec_cfg *sec = NULL;
	int ret = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	if (!priv->phandle->card_info->embedded_supp) {
		PRINTM(MERROR, "Not supported cmd on this card\n");
		ret = -EOPNOTSUPP;
		goto done;
	}

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_sec_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	req->req_id = MLAN_IOCTL_SEC_CFG;
	sec = (mlan_ds_sec_cfg *)req->pbuf;
	sec->sub_command = MLAN_OID_SEC_CFG_PASSPHRASE;
	req->action = MLAN_ACT_SET;

	sec->param.passphrase.ssid.ssid_len = sme->ssid_len;
	memcpy(sec->param.passphrase.ssid.ssid, sme->ssid, sme->ssid_len);

	memcpy((t_u8 *)(sec->param.passphrase.psk.pmk.pmk), sme->crypto.psk,
	       MLAN_PMK_HEXSTR_LENGTH / 2);
	sec->param.passphrase.psk_type = MLAN_PSK_PMK;

	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS)
		ret = -EFAULT;
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);

	LEAVE();
	return ret;
}
#endif

/**
 * @brief Request the driver to connect to the ESS with
 * the specified parameters from kernel
 *
 * @param wiphy           A pointer to wiphy structure
 * @param dev             A pointer to net_device structure
 * @param sme             A pointer to cfg80211_connect_params structure
 *
 * @return                0 -- success, otherwise fail
 */
static int woal_cfg80211_connect(struct wiphy *wiphy, struct net_device *dev,
				 struct cfg80211_connect_params *sme)
{
	moal_private *priv = (moal_private *)woal_get_netdev_priv(dev);
	int ret = 0;
	mlan_bss_info bss_info;
	unsigned long flags;
	mlan_ds_misc_assoc_rsp *assoc_rsp = NULL;
	IEEEtypes_AssocRsp_t *passoc_rsp = NULL;
	mlan_ds_misc_assoc_req *assoc_req = NULL;

	mlan_ssid_bssid *ssid_bssid = NULL;
	moal_handle *handle = priv->phandle;
	int i;

	ENTER();

	PRINTM(MINFO, "Received association request on %s\n", dev->name);
	priv->cfg_disconnect = MFALSE;
#ifdef UAP_CFG80211
	if (GET_BSS_ROLE(priv) == MLAN_BSS_ROLE_UAP) {
		LEAVE();
		return 0;
	}
#endif
	if (priv->wdev->iftype != NL80211_IFTYPE_STATION
#ifdef WIFI_DIRECT_SUPPORT
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(2, 6, 37)
	    && priv->wdev->iftype != NL80211_IFTYPE_P2P_CLIENT
#endif /* KERNEL_VERSION */
#endif /* WIFI_DIRECT_SUPPORT */
	) {
		PRINTM(MERROR,
		       "Received infra assoc request when station not in infra mode\n");
		LEAVE();
		return -EINVAL;
	}
	ssid_bssid = kmalloc(sizeof(mlan_ssid_bssid), GFP_KERNEL);
	if (!ssid_bssid) {
		LEAVE();
		return -EFAULT;
	}
	memset(ssid_bssid, 0, sizeof(mlan_ssid_bssid));
	moal_memcpy_ext(priv->phandle, &ssid_bssid->ssid.ssid, sme->ssid,
			sme->ssid_len, sizeof(ssid_bssid->ssid.ssid));
	ssid_bssid->ssid.ssid_len = sme->ssid_len;
	if (sme->bssid)
		moal_memcpy_ext(priv->phandle, &ssid_bssid->bssid, sme->bssid,
				ETH_ALEN, sizeof(ssid_bssid->bssid));
	/* Not allowed to connect to the same AP which is already connected
		with other interface */
	for (i = 0; i < handle->priv_num; i++) {
		if (handle->priv[i] != priv &&
		    MTRUE == woal_is_connected(handle->priv[i], ssid_bssid)) {
			PRINTM(MMSG,
			       "wlan: already connected with other interface, bssid " MACSTR
			       "\n",
			       MAC2STR(handle->priv[i]->cfg_bssid));
			kfree(ssid_bssid);
			LEAVE();
			return -EINVAL;
		}
	}

	/** cancel pending scan */
	woal_cancel_scan(priv, MOAL_IOCTL_WAIT);

#ifdef WIFI_DIRECT_SUPPORT
#if CFG80211_VERSION_CODE >= WIFI_DIRECT_KERNEL_VERSION
	if (priv->bss_type == MLAN_BSS_TYPE_WIFIDIRECT &&
	    (priv->wdev->iftype == NL80211_IFTYPE_STATION ||
	     priv->wdev->iftype == NL80211_IFTYPE_P2P_CLIENT)) {
		/* if bsstype == wifi direct, and iftype == station or p2p
		 * client, that means wpa_supplicant wants to enable wifi direct
		 * functionality, so we should init p2p client.
		 *
		 * Note that due to kernel iftype check, ICS wpa_supplicant
		 * could not updaet iftype to init p2p client, so we have to
		 * done it here.
		 * */
		if (MLAN_STATUS_SUCCESS !=
		    woal_cfg80211_init_p2p_client(priv)) {
			PRINTM(MERROR,
			       "Init p2p client for wpa_supplicant failed.\n");
			ret = -EFAULT;

			kfree(ssid_bssid);
			LEAVE();
			return ret;
		}
	}
	if (priv->bss_type == MLAN_BSS_TYPE_WIFIDIRECT) {
		/* WAR for P2P connection with vendor TV */
		woal_sched_timeout(200);
	}
#endif
#endif
	/*11r roaming triggered by supplicant */
	if (priv->media_connected && priv->ft_ie_len &&
	    !(priv->ft_cap & MBIT(0))) {
		/** get current bss info */
		memset(&bss_info, 0, sizeof(bss_info));
		if (MLAN_STATUS_SUCCESS !=
		    woal_get_bss_info(priv, MOAL_IOCTL_WAIT, &bss_info)) {
			PRINTM(MERROR, "Fail to get bss info\n");
		}
		/** get target bss info */
		if (MLAN_STATUS_SUCCESS !=
		    woal_find_essid(priv, ssid_bssid, MOAL_IOCTL_WAIT)) {
			ret = woal_cfg80211_connect_scan(priv, sme,
							 MOAL_IOCTL_WAIT);
			if (!ret) {
				if (MLAN_STATUS_SUCCESS !=
				    woal_find_best_network(priv,
							   MOAL_IOCTL_WAIT,
							   ssid_bssid)) {
					PRINTM(MERROR,
					       "can't find targe AP \n");
					// LEAVE();
					// return -EFAULT;
				}
			}
		}
		if (bss_info.mdid == ssid_bssid->ft_md &&
		    bss_info.ft_cap == ssid_bssid->ft_cap) {
			ret = woal_start_ft_roaming(priv, ssid_bssid);
			kfree(ssid_bssid);
			LEAVE();
			return 0;
		}
	}

	priv->cfg_connect = MTRUE;
	if (priv->scan_type == MLAN_SCAN_TYPE_PASSIVE)
		woal_set_scan_type(priv, MLAN_SCAN_TYPE_ACTIVE);
	priv->assoc_status = 0;
	assoc_rsp = kzalloc(sizeof(mlan_ds_misc_assoc_rsp), GFP_ATOMIC);
	if (!assoc_rsp) {
		PRINTM(MERROR, "Failed to allocate memory for assoc_rsp\n");
		ret = -ENOMEM;
		kfree(ssid_bssid);
		LEAVE();
		return ret;
	}
	ret = woal_cfg80211_assoc(priv, (void *)sme, MOAL_IOCTL_WAIT,
				  assoc_rsp);

	if (priv->scan_type == MLAN_SCAN_TYPE_PASSIVE)
		woal_set_scan_type(priv, MLAN_SCAN_TYPE_PASSIVE);
	if (!ret) {
		passoc_rsp = (IEEEtypes_AssocRsp_t *)assoc_rsp->assoc_resp_buf;
		priv->rssi_low = DEFAULT_RSSI_LOW_THRESHOLD;
		woal_save_conn_params(priv, sme);
		memset(&bss_info, 0, sizeof(bss_info));
		woal_get_bss_info(priv, MOAL_IOCTL_WAIT, &bss_info);
		priv->channel = bss_info.bss_chan;
		if (!ssid_bssid->ft_md) {
			priv->ft_ie_len = 0;
			priv->ft_pre_connect = MFALSE;
			priv->ft_md = 0;
			priv->ft_cap = 0;
		}
		assoc_req = kzalloc(sizeof(mlan_ds_misc_assoc_req), GFP_ATOMIC);
		if (assoc_req)
			woal_get_assoc_req(priv, assoc_req, MOAL_IOCTL_WAIT);
	}
	spin_lock_irqsave(&priv->connect_lock, flags);
	priv->cfg_connect = MFALSE;
	if (!ret && priv->media_connected) {
		PRINTM(MMSG,
		       "wlan: Connected to bssid " MACSTR " successfully\n",
		       MAC2STR(priv->cfg_bssid));
		spin_unlock_irqrestore(&priv->connect_lock, flags);
		cfg80211_connect_result(
			priv->netdev, priv->cfg_bssid, assoc_req->assoc_req_buf,
			assoc_req->assoc_req_len, passoc_rsp->ie_buffer,
			assoc_rsp->assoc_resp_len - ASSOC_RESP_FIXED_SIZE,
			WLAN_STATUS_SUCCESS, GFP_KERNEL);
	} else {
		PRINTM(MINFO, "wlan: Failed to connect to bssid " MACSTR "\n",
		       MAC2STR(priv->cfg_bssid));
		memset(priv->cfg_bssid, 0, ETH_ALEN);
		spin_unlock_irqrestore(&priv->connect_lock, flags);
		cfg80211_connect_result(priv->netdev, priv->cfg_bssid, NULL, 0,
					NULL, 0, woal_get_assoc_status(priv),
					GFP_KERNEL);
	}
	kfree(ssid_bssid);
	kfree(assoc_req);
	kfree(assoc_rsp);
	assoc_rsp = NULL;
	LEAVE();
	return 0;
}

/**
 *  @brief This function will print diconnect reason code according
 *  to IEEE 802.11 spec
 *
 *  @param reason_code    reason code for the deauth/disaccoc
 *                        received from firmware
 *  @return        N/A
 */
static void woal_print_disconnect_reason(t_u16 reason_code)
{
	ENTER();

	switch (reason_code) {
	case MLAN_REASON_UNSPECIFIED:
		PRINTM(MMSG, "wlan: REASON: Unspecified reason\n");
		break;
	case MLAN_REASON_PREV_AUTH_NOT_VALID:
		PRINTM(MMSG,
		       "wlan: REASON: Previous authentication no longer valid\n");
		break;
	case MLAN_REASON_DEAUTH_LEAVING:
		PRINTM(MMSG,
		       "wlan: REASON: (Deauth) Sending STA is leaving (or has left) IBSS or ESS\n");
		break;
	case MLAN_REASON_DISASSOC_DUE_TO_INACTIVITY:
		PRINTM(MMSG,
		       "wlan: REASON: Disassociated due to inactivity \n");
		break;
	case MLAN_REASON_DISASSOC_AP_BUSY:
		PRINTM(MMSG,
		       "wlan: REASON: (Disassociated) AP unable to handle all connected STAs\n");
		break;
	case MLAN_REASON_CLASS2_FRAME_FROM_NOAUTH_STA:
		PRINTM(MMSG,
		       "wlan: REASON: Class 2 frame was received from nonauthenticated STA\n");
		break;
	case MLAN_REASON_CLASS3_FRAME_FROM_NOASSOC_STA:
		PRINTM(MMSG,
		       "wlan: REASON: Class 3 frame was received from nonassociated STA\n");
		break;
	case MLAN_REASON_DISASSOC_STA_HAS_LEFT:
		PRINTM(MMSG,
		       "wlan: REASON: (Disassocated) Sending STA is leaving (or has left) BSS\n");
		break;
	case MLAN_REASON_STA_REQ_ASSOC_WITHOUT_AUTH:
		PRINTM(MMSG,
		       "wlan: REASON: STA requesting (re)assoc is not authenticated with responding STA\n");
		break;
	default:
		break;
	}

	LEAVE();
	return;
}

/**
 * @brief Request the driver to disconnect
 *
 * @param wiphy           A pointer to wiphy structure
 * @param dev             A pointer to net_device structure
 * @param reason_code     Reason code
 *
 * @return                0 -- success, otherwise fail
 */
static int woal_cfg80211_disconnect(struct wiphy *wiphy, struct net_device *dev,
				    t_u16 reason_code)
{
	moal_private *priv = (moal_private *)woal_get_netdev_priv(dev);
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();
	PRINTM(MMSG,
	       "wlan: Received disassociation request on %s, reason: %u\n",
	       dev->name, reason_code);
	woal_print_disconnect_reason(reason_code);
#ifdef UAP_CFG80211
	if (GET_BSS_ROLE(priv) == MLAN_BSS_ROLE_UAP) {
		LEAVE();
		return 0;
	}
#endif
	if (priv->phandle->driver_status || priv->phandle->surprise_removed) {
		PRINTM(MERROR,
		       "Block woal_cfg80211_disconnect in abnormal driver state\n");
		LEAVE();
		return -EFAULT;
	}

	if (priv->media_connected == MFALSE) {
		PRINTM(MMSG, " Already disconnected\n");
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 11, 0)
#if ((CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 19, 2)) ||                    \
     (defined(ANDROID_SDK_VERSION) && ANDROID_SDK_VERSION >= 31))
		if (priv->wdev->connected &&
#else
		if (priv->wdev->current_bss &&
#endif
		    (priv->wdev->iftype == NL80211_IFTYPE_STATION ||
		     priv->wdev->iftype == NL80211_IFTYPE_P2P_CLIENT)) {
			priv->cfg_disconnect = MTRUE;
			cfg80211_disconnected(priv->netdev, 0, NULL, 0,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 2, 0)
					      true,
#endif
					      GFP_KERNEL);
		}
#endif
		LEAVE();
		return 0;
	}

	if (priv->cfg_disconnect) {
		PRINTM(MERROR, "Disassociation already in progress\n");
		LEAVE();
		return 0;
	}

	if (priv->phandle->remain_on_channel) {
		if (woal_cfg80211_remain_on_channel_cfg(priv, MOAL_IOCTL_WAIT,
							MTRUE, (t_u8 *)&status,
							NULL, 0, 0)) {
			PRINTM(MERROR, "Fail to cancel remain on channel\n");
		}
		priv->phandle->remain_on_channel = MFALSE;
	}

	/** cancel pending scan */
	woal_cancel_scan(priv, MOAL_IOCTL_WAIT);
	priv->cfg_disconnect = MTRUE;
	if (woal_disconnect(priv, MOAL_IOCTL_WAIT_TIMEOUT, priv->cfg_bssid,
			    reason_code) != MLAN_STATUS_SUCCESS) {
		priv->cfg_disconnect = MFALSE;
		LEAVE();
		return -EFAULT;
	}

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 11, 0)
	if (priv->wdev->iftype == NL80211_IFTYPE_STATION ||
	    priv->wdev->iftype == NL80211_IFTYPE_P2P_CLIENT)
		cfg80211_disconnected(priv->netdev, 0, NULL, 0,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 2, 0)
				      true,
#endif
				      GFP_KERNEL);
#endif

	memset(priv->cfg_bssid, 0, ETH_ALEN);
	woal_clear_conn_params(priv);
	priv->channel = 0;

	LEAVE();
	return 0;
}

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 4, 0)
/**
 *  @brief This function is deauthentication handler when host MLME
 *          enable.
 *          In this case driver will prepare and send Deauth Req.
 *
 *  @param wiphy       A pointer to wiphy.
 *
 *  @param dev         A pointer to net_device
 *
 *  @param req         A pointer to cfg80211_deauth_request
 *
 *  @return            0 -- success, otherwise fail
 */

static int woal_cfg80211_deauthenticate(struct wiphy *wiphy,
					struct net_device *dev,
					struct cfg80211_deauth_request *req)
{
	int ret = 0;
	moal_private *priv = (moal_private *)woal_get_netdev_priv(dev);
	if (priv->phandle->driver_status || priv->phandle->surprise_removed) {
		PRINTM(MERROR,
		       "Block woal_cfg80211_deauthenticate in abnormal driver state\n");
		return ret;
	}

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
	if (priv->host_mlme) {
		priv->delay_deauth_notify = MTRUE;
		moal_memcpy_ext(priv->phandle, priv->bssid_notify, req->bssid,
				MLAN_MAC_ADDR_LENGTH, MLAN_MAC_ADDR_LENGTH);
	}
#endif

	ret = woal_cfg80211_disconnect(wiphy, dev, req->reason_code);
#if CFG80211_VERSION_CODE < KERNEL_VERSION(3, 11, 0)
	if (priv->wdev->iftype == NL80211_IFTYPE_STATION ||
	    priv->wdev->iftype == NL80211_IFTYPE_P2P_CLIENT)
		cfg80211_disconnected(priv->netdev, 0, NULL, 0, GFP_KERNEL);
#endif
	if (priv->media_connected)
		woal_send_disconnect_to_system(priv, DEF_DEAUTH_REASON_CODE);
	return ret;
}

/**
 *  @brief This function is disassociation handler when host MLME
 *          enable.
 *          In this case driver will prepare and send Disassoc frame.
 *
 *  @param wiphy       A pointer to wiphy.
 *
 *  @param dev         A pointer to net_device
 *
 *  @param req         A pointer to cfg80211_disassoc_request
 *
 *  @return            0 -- success, otherwise fail
 */
static int woal_cfg80211_disassociate(struct wiphy *wiphy,
				      struct net_device *dev,
				      struct cfg80211_disassoc_request *req)
{
	int ret = 0;
	moal_private *priv = (moal_private *)woal_get_netdev_priv(dev);

	if (priv->phandle->driver_status || priv->phandle->surprise_removed) {
		PRINTM(MERROR,
		       "Block woal_cfg80211_disassociate in abnormal driver state\n");
		return ret;
	}

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
	if (priv->host_mlme) {
		priv->delay_deauth_notify = MTRUE;
#if (CFG80211_VERSION_CODE >= KERNEL_VERSION(6, 0, 0) ||                       \
     (defined(ANDROID_SDK_VERSION) && ANDROID_SDK_VERSION >= 33 &&             \
      CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 15, 74)))
		moal_memcpy_ext(priv->phandle, priv->bssid_notify, req->ap_addr,
				MLAN_MAC_ADDR_LENGTH, MLAN_MAC_ADDR_LENGTH);
#else
		moal_memcpy_ext(priv->phandle, priv->bssid_notify,
				req->bss->bssid, MLAN_MAC_ADDR_LENGTH,
				MLAN_MAC_ADDR_LENGTH);
#endif
	}
#endif

	ret = woal_cfg80211_disconnect(wiphy, dev, req->reason_code);
#if CFG80211_VERSION_CODE < KERNEL_VERSION(3, 11, 0)
	if (priv->wdev->iftype == NL80211_IFTYPE_STATION ||
	    priv->wdev->iftype == NL80211_IFTYPE_P2P_CLIENT)
		cfg80211_disconnected(priv->netdev, 0, NULL, 0, GFP_KERNEL);
#endif
	if (priv->media_connected)
		woal_send_disconnect_to_system(priv, DEF_DEAUTH_REASON_CODE);

	return ret;
}
#endif

/**
 * @brief Request the driver to get the station information
 *
 * @param wiphy           A pointer to wiphy structure
 * @param dev             A pointer to net_device structure
 * @param mac             MAC address of the station
 * @param sinfo           A pointer to station_info structure
 *
 * @return                0 -- success, otherwise fail
 */
static int woal_cfg80211_get_station(struct wiphy *wiphy,
				     struct net_device *dev,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 16, 0)
				     const u8 *mac,
#else
				     u8 *mac,
#endif
				     struct station_info *sinfo)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	moal_private *priv = (moal_private *)woal_get_netdev_priv(dev);

	ENTER();

#ifdef UAP_CFG80211
	if (GET_BSS_ROLE(priv) == MLAN_BSS_ROLE_UAP) {
		LEAVE();
		return woal_uap_cfg80211_get_station(wiphy, dev, mac, sinfo);
	}
#endif
	if (priv->media_connected == MFALSE) {
		PRINTM(MINFO, "cfg80211: Media not connected!\n");
		LEAVE();
		return -ENOENT;
	}

	if (MLAN_STATUS_SUCCESS !=
	    woal_cfg80211_dump_station_info(priv, sinfo)) {
		PRINTM(MERROR, "cfg80211: Failed to get station info\n");
		ret = -EFAULT;
	}
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 2, 0)
	woal_check_auto_tdls(wiphy, dev);
#endif
	LEAVE();
	return ret;
}

/**
 * @brief Request the driver to dump the station information
 *
 * @param wiphy           A pointer to wiphy structure
 * @param dev             A pointer to net_device structure
 * @param idx             Station index
 * @param mac             MAC address of the station
 * @param sinfo           A pointer to station_info structure
 *
 * @return                0 -- success, otherwise fail
 */
static int woal_cfg80211_dump_station(struct wiphy *wiphy,
				      struct net_device *dev, int idx,
				      t_u8 *mac, struct station_info *sinfo)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	moal_private *priv = (moal_private *)woal_get_netdev_priv(dev);

	ENTER();

#ifdef UAP_CFG80211
	if (GET_BSS_ROLE(priv) == MLAN_BSS_ROLE_UAP) {
		LEAVE();
		return woal_uap_cfg80211_dump_station(wiphy, dev, idx, mac,
						      sinfo);
	}
#endif

	if (!priv->media_connected || idx != 0) {
		PRINTM(MINFO,
		       "cfg80211: Media not connected or not for this station!\n");
		LEAVE();
		return -ENOENT;
	}

	moal_memcpy_ext(priv->phandle, mac, priv->cfg_bssid, ETH_ALEN,
			ETH_ALEN);

	if (MLAN_STATUS_SUCCESS !=
	    woal_cfg80211_dump_station_info(priv, sinfo)) {
		PRINTM(MERROR, "cfg80211: Failed to get station info\n");
		ret = -EFAULT;
	}

	LEAVE();
	return ret;
}

/**
 *  @brief Convert driver band configuration to IEEE band type
 *
 *  @param band     Driver band configuration
 *
 *  @return         IEEE band type
 */
static t_u8 woal_bandcfg_to_ieee_band(Band_Config_t bandcfg)
{
	t_u8 ret_radio_type = 0;

	ENTER();

	switch (bandcfg.chanBand) {
	case BAND_5GHZ:
		ret_radio_type = IEEE80211_BAND_5GHZ;
		break;
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 8, 0)
	case BAND_6GHZ:
		ret_radio_type = IEEE80211_BAND_6GHZ;
		break;
#endif
	case BAND_2GHZ:
	default:
		ret_radio_type = IEEE80211_BAND_2GHZ;
		break;
	}
	LEAVE();
	return ret_radio_type;
}

/**
 * @brief Request the driver to dump survey info
 *
 * @param wiphy           A pointer to wiphy structure
 * @param dev             A pointer to net_device structure
 * @param idx             Station index
 * @param survey          A pointer to survey_info structure
 *
 * @return                0 -- success, otherwise fail
 */
static int woal_cfg80211_dump_survey(struct wiphy *wiphy,
				     struct net_device *dev, int idx,
				     struct survey_info *survey)
{
	int ret = -ENOENT;
	moal_private *priv = (moal_private *)woal_get_netdev_priv(dev);
	enum ieee80211_band band;
	ChanStatistics_t *pchan_stats = NULL;
	mlan_scan_resp scan_resp;

	ENTER();
	PRINTM(MIOCTL, "dump_survey idx=%d\n", idx);

	memset(&scan_resp, 0, sizeof(scan_resp));
	if (MLAN_STATUS_SUCCESS !=
	    woal_get_scan_table(priv, MOAL_IOCTL_WAIT, &scan_resp)) {
		ret = -EFAULT;
		goto done;
	}
	pchan_stats = (ChanStatistics_t *)scan_resp.pchan_stats;
	if (idx > (int)scan_resp.num_in_chan_stats || idx < 0) {
		ret = -EFAULT;
		goto done;
	}
	if (idx == (int)scan_resp.num_in_chan_stats ||
	    !pchan_stats[idx].cca_scan_duration)
		goto done;
	ret = 0;
	memset(survey, 0, sizeof(*survey));
	band = woal_bandcfg_to_ieee_band(pchan_stats[idx].bandcfg);
	survey->channel = ieee80211_get_channel(
		wiphy, ieee80211_channel_to_frequency(pchan_stats[idx].chan_num
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(2, 6, 39)
						      ,
						      band
#endif
						      ));
	survey->filled = SURVEY_INFO_NOISE_DBM;
	survey->noise = pchan_stats[idx].noise;
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(2, 6, 37)
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 0, 0)
	survey->filled |= SURVEY_INFO_TIME | SURVEY_INFO_TIME_BUSY;
	survey->time = pchan_stats[idx].cca_scan_duration;
	survey->time_busy = pchan_stats[idx].cca_busy_duration;
#else
	survey->filled |=
		SURVEY_INFO_CHANNEL_TIME | SURVEY_INFO_CHANNEL_TIME_BUSY;
	survey->channel_time = pchan_stats[idx].cca_scan_duration;
	survey->channel_time_busy = pchan_stats[idx].cca_busy_duration;
#endif
#endif
done:
	LEAVE();
	return ret;
}

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
static int woal_cfg80211_get_channel(struct wiphy *wiphy,
				     struct wireless_dev *wdev,
#if ((CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 19, 2)) ||                    \
     (defined(ANDROID_SDK_VERSION) && ANDROID_SDK_VERSION >= 31))
				     unsigned int link_id,
#endif
				     struct cfg80211_chan_def *chandef)
{
	moal_private *priv = (moal_private *)woal_get_netdev_priv(wdev->netdev);
	chan_band_info channel;
#ifdef UAP_SUPPORT
	moal_handle *handle = (moal_handle *)woal_get_wiphy_priv(wiphy);
#endif

	memset(&channel, 0x00, sizeof(channel));

#ifdef UAP_SUPPORT
	if (wdev->iftype == NL80211_IFTYPE_MONITOR) {
		if ((handle->mon_if) &&
		    (handle->mon_if->mon_ndev == wdev->netdev)) {
			*chandef = handle->mon_if->chandef;
			return 0;
		}
		return -EFAULT;
	}
#endif

#ifdef UAP_SUPPORT
	if (GET_BSS_ROLE(priv) == MLAN_BSS_ROLE_UAP) {
		if (priv->bss_started == MTRUE) {
			if (MLAN_STATUS_SUCCESS !=
			    woal_set_get_ap_channel(priv, MLAN_ACT_GET,
						    MOAL_IOCTL_WAIT,
						    &channel)) {
				PRINTM(MERROR, "Fail to get ap channel \n");
				return -EFAULT;
			}
		} else {
			PRINTM(MIOCTL, "get_channel when AP is not started\n");
			return -EFAULT;
		}
	} else
#endif
		if (GET_BSS_ROLE(priv) == MLAN_BSS_ROLE_STA) {
		if (priv->media_connected == MTRUE) {
			if (MLAN_STATUS_SUCCESS !=
			    woal_get_sta_channel(priv, MOAL_IOCTL_WAIT,
						 &channel)) {
				PRINTM(MERROR, "Fail to get sta channel \n");
				return -EFAULT;
			}
		} else {
			PRINTM(MIOCTL,
			       "get_channel when STA is not connected\n");
			return -EFAULT;
		}
	} else {
		PRINTM(MERROR, "BssRole not support %d.\n", GET_BSS_ROLE(priv));
		return -EFAULT;
	}

	if (MLAN_STATUS_FAILURE == woal_chandef_create(priv, chandef, &channel))
		return -EFAULT;
	else
		return 0;
}
#endif

/**
 * @brief Request the driver to change the IEEE power save
 * mdoe
 *
 * @param wiphy           A pointer to wiphy structure
 * @param dev             A pointer to net_device structure
 * @param enabled         Enable or disable
 * @param timeout         Timeout value
 *
 * @return                0 -- success, otherwise fail
 */
static int woal_cfg80211_set_power_mgmt(struct wiphy *wiphy,
					struct net_device *dev, bool enabled,
					int timeout)
{
	int ret = 0, disabled;
	moal_private *priv = (moal_private *)woal_get_netdev_priv(dev);

	ENTER();
	if (moal_extflg_isset(priv->phandle, EXT_HW_TEST) ||
	    (priv->phandle->params.ps_mode == MLAN_INIT_PARA_DISABLED)) {
		PRINTM(MIOCTL, "block set power hw_test=%d ps_mode=%d\n",
		       moal_extflg_isset(priv->phandle, EXT_HW_TEST),
		       priv->phandle->params.ps_mode);
		LEAVE();
		return -EFAULT;
	}

	if (priv->phandle->driver_status || priv->phandle->surprise_removed) {
		PRINTM(MERROR,
		       "Block woal_cfg80211_set_power_mgmt in abnormal driver state\n");
		LEAVE();
		return -EFAULT;
	}
#ifdef WIFI_DIRECT_SUPPORT
#if CFG80211_VERSION_CODE >= WIFI_DIRECT_KERNEL_VERSION
	if (priv->bss_type == MLAN_BSS_TYPE_WIFIDIRECT) {
		PRINTM(MIOCTL, "skip set power for p2p interface\n");
		LEAVE();
		return ret;
	}
#endif
#endif
	if (enabled)
		disabled = 0;
	else
		disabled = 1;

	if (MLAN_STATUS_SUCCESS != woal_set_get_power_mgmt(priv, MLAN_ACT_SET,
							   &disabled, timeout,
							   MOAL_IOCTL_WAIT)) {
		ret = -EOPNOTSUPP;
	}

	LEAVE();
	return ret;
}

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(2, 6, 31)
/**
 * @brief Request the driver to get the transmit power info
 *
 * @param wiphy           A pointer to wiphy structure
 * @param type            TX power adjustment type
 * @param dbm             TX power in dbm
 *
 * @return                0 -- success, otherwise fail
 */
static int woal_cfg80211_get_tx_power(struct wiphy *wiphy,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
				      struct wireless_dev *wdev,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(6, 14, 0)
				      unsigned int link_id,
#endif
#endif
				      int *dbm)
{
	int ret = 0;
	moal_private *priv = NULL;
	mlan_power_cfg_t power_cfg;
	moal_handle *handle = (moal_handle *)woal_get_wiphy_priv(wiphy);

	ENTER();

	if (!handle) {
		PRINTM(MFATAL, "Unable to get handle\n");
		LEAVE();
		return -EFAULT;
	}

	priv = woal_get_priv(handle, MLAN_BSS_ROLE_ANY);

	if (!priv) {
		PRINTM(MFATAL, "Unable to get priv in %s()\n", __func__);
		LEAVE();
		return -EFAULT;
	}

	if (MLAN_STATUS_SUCCESS !=
	    woal_set_get_tx_power(priv, MLAN_ACT_GET, &power_cfg)) {
		LEAVE();
		return -EFAULT;
	}

	*dbm = power_cfg.power_level;

	LEAVE();
	return ret;
}
/**
 * @brief Request the driver to change the transmit power
 *
 * @param wiphy           A pointer to wiphy structure
 * @param type            TX power adjustment type
 * @param mbm             TX power in mbm
 *
 * @return                0 -- success, otherwise fail
 */
static int woal_cfg80211_set_tx_power(struct wiphy *wiphy,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
				      struct wireless_dev *wdev,
#endif
#if CFG80211_VERSION_CODE < KERNEL_VERSION(2, 6, 36)
				      enum tx_power_setting type,
#else
				      enum nl80211_tx_power_setting type,
#endif
				      int mbm)
{
	int ret = 0;
	moal_private *priv = NULL;
	moal_handle *handle = (moal_handle *)woal_get_wiphy_priv(wiphy);
	mlan_power_cfg_t power_cfg;
	int dbm = MBM_TO_DBM(mbm);

	ENTER();
	memset(&power_cfg, 0, sizeof(power_cfg));

	priv = woal_get_priv(handle, MLAN_BSS_ROLE_ANY);
	if (!priv) {
		PRINTM(MFATAL, "Unable to get priv in %s()\n", __func__);
		LEAVE();
		return -EFAULT;
	}

	if (type) {
		power_cfg.is_power_auto = 0;
		if (mbm < 0 || (mbm % 100)) {
			PRINTM(MERROR, "Invalid tx power value %d mbm\n", mbm);
			LEAVE();
			return -EOPNOTSUPP;
		}
		power_cfg.power_level = dbm;
	} else
		power_cfg.is_power_auto = 1;

	if (MLAN_STATUS_SUCCESS !=
	    woal_set_get_tx_power(priv, MLAN_ACT_SET, &power_cfg))
		ret = -EFAULT;

	LEAVE();
	return ret;
}
#endif

#if CFG80211_VERSION_CODE > KERNEL_VERSION(2, 6, 35)
/**
 * CFG802.11 operation handler for connection quality monitoring.
 *
 * @param wiphy           A pointer to wiphy structure
 * @param dev             A pointer to net_device structure
 * @param rssi_thold	  rssi threshold
 * @param rssi_hyst		  rssi hysteresis
 */
static int woal_cfg80211_set_cqm_rssi_config(struct wiphy *wiphy,
					     struct net_device *dev,
					     s32 rssi_thold, u32 rssi_hyst)
{
	int ret = 0;
	moal_private *priv = (moal_private *)woal_get_netdev_priv(dev);
	ENTER();
	priv->cqm_rssi_thold = rssi_thold;
	priv->cqm_rssi_high_thold = rssi_thold;
	priv->cqm_rssi_hyst = rssi_hyst;
	priv->mrvl_rssi_low = 0;

	PRINTM(MIOCTL, "rssi_thold=%d rssi_hyst=%d\n", (int)rssi_thold,
	       (int)rssi_hyst);
	if (MLAN_STATUS_SUCCESS !=
	    woal_set_rssi_threshold(priv, 0, MOAL_IOCTL_WAIT)) {
		PRINTM(MERROR, "Fail to set rssi thresold.\n");
	}
	LEAVE();
	return ret;
}
#endif

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(2, 6, 39)
/**
 * @brief remain on channel config
 *
 * @param priv              A pointer to moal_private structure
 * @param wait_option       Wait option
 * @param cancel			cancel remain on channel flag
 * @param status            A pointer to status, success, in process or reject
 * @param chan              A pointer to ieee80211_channel structure
 * @param channel_type      channel_type,
 * @param duration          Duration wait to receive frame
 *
 * @return                  0 -- success, otherwise fail
 */
int woal_cfg80211_remain_on_channel_cfg(moal_private *priv, t_u8 wait_option,
					t_u8 remove, t_u8 *status,
					struct ieee80211_channel *chan,
					enum mlan_channel_type channel_type,
					t_u32 duration)
{
	mlan_ds_remain_chan chan_cfg;
	int ret = 0;

	ENTER();

	if (!status || (!chan && !remove)) {
		PRINTM(MERROR,
		       "Invalid parameter status=%p, chan=%p, remove=%d\n",
		       status, chan, remove);
		LEAVE();
		return -EFAULT;
	}
	memset(&chan_cfg, 0, sizeof(mlan_ds_remain_chan));
	if (remove) {
		chan_cfg.remove = MTRUE;
	} else {
#ifdef WIFI_DIRECT_SUPPORT
		if (priv->phandle->is_go_timer_set) {
			PRINTM(MINFO,
			       "block remain on channel while go timer is on\n");
			LEAVE();
			return -EBUSY;
		}
#endif
		chan_cfg.bandcfg.chanBand =
			woal_ieee_band_to_radio_type(chan->band);
		switch (channel_type) {
		case CHAN_HT40MINUS:
			chan_cfg.bandcfg.chan2Offset = SEC_CHAN_BELOW;
			chan_cfg.bandcfg.chanWidth = CHAN_BW_40MHZ;
			break;
		case CHAN_HT40PLUS:
			chan_cfg.bandcfg.chan2Offset = SEC_CHAN_ABOVE;
			chan_cfg.bandcfg.chanWidth = CHAN_BW_40MHZ;
			break;
		case CHAN_VHT80:
			chan_cfg.bandcfg.chanWidth = CHAN_BW_80MHZ;
			break;
		case CHAN_NO_HT:
		case CHAN_HT20:
		default:
			break;
		}
		chan_cfg.channel =
			ieee80211_frequency_to_channel(chan->center_freq);
		chan_cfg.remain_period = duration;
		PRINTM(MCMND,
		       "Remain on Channel: chan=%d, offset=%d width=%d\n",
		       chan_cfg.channel, chan_cfg.bandcfg.chan2Offset,
		       chan_cfg.bandcfg.chanWidth);
	}
	if (MLAN_STATUS_SUCCESS ==
	    woal_set_remain_channel_ioctl(priv, wait_option, &chan_cfg))
		*status = chan_cfg.status;
	else
		ret = -EFAULT;

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
	if (remove) {
		if (priv->host_mlme &&
		    (priv->auth_flag & HOST_MLME_AUTH_PENDING)) {
			/* abort the pending auth exchange */
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 11, 0)
			if (priv->assoc_bss) {
				PRINTM(MEVENT,
				       "wlan: HostMlme auth remain on channel cancelled\n");
				cfg80211_auth_timeout(priv->netdev,
						      priv->assoc_bss->bssid);
			}
#endif
			priv->auth_flag = 0;
			priv->host_mlme = MFALSE;
			priv->auth_alg = 0xFFFF;
		}
	}
#endif

	LEAVE();
	return ret;
}

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 6, 0)
/**
 * @brief tx mgmt frame
 *
 * @param wiphy                 A pointer to wiphy structure
 * @param wdev                  A pointer to wireless_dev structure
 * @param cookie                A pointer to frame cookie
 *
 * @return                0 -- success, otherwise fail
 */
static int woal_cfg80211_mgmt_tx_cancel_wait(struct wiphy *wiphy,
					     struct wireless_dev *wdev,
					     u64 cookie)
#else
/**
 * @brief tx mgmt frame
 *
 * @param wiphy                 A pointer to wiphy structure
 * @param dev                   A pointer to net_device structure
 * @param cookie                A pointer to frame cookie
 *
 * @return                0 -- success, otherwise fail
 */
static int woal_cfg80211_mgmt_tx_cancel_wait(struct wiphy *wiphy,
					     struct net_device *dev, u64 cookie)
#endif
{
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 6, 0)
	struct net_device *dev = wdev->netdev;
#endif
	moal_private *priv = (moal_private *)woal_get_netdev_priv(dev);
	int ret = 0;
	t_u8 status = 1;
	moal_private *remain_priv = NULL;

	ENTER();

	if (priv->phandle->remain_on_channel) {
		remain_priv =
			priv->phandle->priv[priv->phandle->remain_bss_index];
		if (!remain_priv) {
			PRINTM(MERROR,
			       "mgmt_tx_cancel_wait: Wrong remain_bss_index=%d\n",
			       priv->phandle->remain_bss_index);
			ret = -EFAULT;
			goto done;
		}
		if (woal_cfg80211_remain_on_channel_cfg(remain_priv,
							MOAL_IOCTL_WAIT, MTRUE,
							&status, NULL, 0, 0)) {
			PRINTM(MERROR,
			       "mgmt_tx_cancel_wait: Fail to cancel remain on channel\n");
			ret = -EFAULT;
			goto done;
		}
		if (priv->phandle->cookie) {
			cfg80211_remain_on_channel_expired(
#if CFG80211_VERSION_CODE < KERNEL_VERSION(3, 6, 0)
				remain_priv->netdev,
#else
				remain_priv->wdev,
#endif
				priv->phandle->cookie, &priv->phandle->chan,
#if CFG80211_VERSION_CODE < KERNEL_VERSION(3, 8, 0)
				priv->phandle->channel_type,
#endif
				GFP_ATOMIC);
			priv->phandle->cookie = 0;
		}
		priv->phandle->remain_on_channel = MFALSE;
	}

done:
	LEAVE();
	return ret;
}

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 6, 0)
/**
 * @brief Make chip remain on channel
 *
 * @param wiphy                 A pointer to wiphy structure
 * @param wdev                  A pointer to wireless_dev structure
 * @param chan                  A pointer to ieee80211_channel structure
 * @param channel_type          Channel type
 * @param duration              Duration for timer
 * @param cookie                A pointer to timer cookie
 *
 * @return                  0 -- success, otherwise fail
 */
static int
woal_cfg80211_remain_on_channel(struct wiphy *wiphy, struct wireless_dev *wdev,
				struct ieee80211_channel *chan,
#if CFG80211_VERSION_CODE < KERNEL_VERSION(3, 8, 0)
				enum nl80211_channel_type channel_type,
#endif
				unsigned int duration, u64 *cookie)
#else
/**
 * @brief Make chip remain on channel
 *
 * @param wiphy                 A pointer to wiphy structure
 * @param dev                   A pointer to net_device structure
 * @param chan                  A pointer to ieee80211_channel structure
 * @param channel_type          Channel type
 * @param duration              Duration for timer
 * @param cookie                A pointer to timer cookie
 *
 * @return                  0 -- success, otherwise fail
 */
static int
woal_cfg80211_remain_on_channel(struct wiphy *wiphy, struct net_device *dev,
				struct ieee80211_channel *chan,
				enum nl80211_channel_type channel_type,
				unsigned int duration, u64 *cookie)
#endif
{
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 6, 0)
	struct net_device *dev = wdev->netdev;
#endif
	moal_private *priv = (moal_private *)woal_get_netdev_priv(dev);
	int ret = 0;
	t_u8 status = 1;
	moal_private *remain_priv = NULL;

	ENTER();

	if (!chan || !cookie) {
		PRINTM(MERROR, "Invalid parameter for remain on channel\n");
		ret = -EFAULT;
		goto done;
	}
	/** cancel previous remain on channel */
	if (priv->phandle->remain_on_channel &&
	    ((priv->phandle->chan.center_freq != chan->center_freq))) {
		remain_priv =
			priv->phandle->priv[priv->phandle->remain_bss_index];
		if (!remain_priv) {
			PRINTM(MERROR,
			       "remain_on_channel: Wrong remain_bss_index=%d\n",
			       priv->phandle->remain_bss_index);
			ret = -EFAULT;
			goto done;
		}
		if (woal_cfg80211_remain_on_channel_cfg(remain_priv,
							MOAL_IOCTL_WAIT, MTRUE,
							&status, NULL, 0, 0)) {
			PRINTM(MERROR,
			       "remain_on_channel: Fail to cancel remain on channel\n");
			ret = -EFAULT;
			goto done;
		}
		priv->phandle->cookie = 0;
		priv->phandle->remain_on_channel = MFALSE;
	}
	if (MLAN_STATUS_SUCCESS !=
	    woal_cfg80211_remain_on_channel_cfg(priv, MOAL_IOCTL_WAIT, MFALSE,
						&status, chan,
#if CFG80211_VERSION_CODE < KERNEL_VERSION(3, 8, 0)
						channel_type,
#else
						0,
#endif
						(t_u32)duration)) {
		ret = -EFAULT;
		goto done;
	}

	if (status) {
		PRINTM(MMSG,
		       "%s: Set remain on Channel: channel=%d with status=%d\n",
		       dev->name,
		       ieee80211_frequency_to_channel(chan->center_freq),
		       status);
		if (!priv->phandle->remain_on_channel) {
			priv->phandle->is_remain_timer_set = MTRUE;
			woal_mod_timer(&priv->phandle->remain_timer, duration);
		}
	}

	/* remain on channel operation success */
	/* we need update the value cookie */
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 8, 0)
	*cookie = (u64)random32() | 1;
#else
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0)
	*cookie = (u64)prandom_u32() | 1;
#else
	*cookie = (u64)get_random_u32() | 1;
#endif
#endif
	priv->phandle->remain_on_channel = MTRUE;
	priv->phandle->remain_bss_index = priv->bss_index;
	priv->phandle->cookie = *cookie;
#if CFG80211_VERSION_CODE < KERNEL_VERSION(3, 8, 0)
	priv->phandle->channel_type = channel_type;
#endif
	moal_memcpy_ext(priv->phandle, &priv->phandle->chan, chan,
			sizeof(struct ieee80211_channel),
			sizeof(priv->phandle->chan));

	if (status == 0)
		PRINTM(MIOCTL,
		       "%s: Set remain on Channel: channel=%d cookie = %#llx\n",
		       dev->name,
		       ieee80211_frequency_to_channel(chan->center_freq),
		       priv->phandle->cookie);

	cfg80211_ready_on_channel(
#if CFG80211_VERSION_CODE < KERNEL_VERSION(3, 6, 0)
		dev,
#else
		priv->wdev,
#endif
		*cookie, chan,
#if CFG80211_VERSION_CODE < KERNEL_VERSION(3, 8, 0)
		channel_type,
#endif
		duration, GFP_KERNEL);

done:
	LEAVE();
	return ret;
}

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 6, 0)
/**
 * @brief Cancel remain on channel
 *
 * @param wiphy                 A pointer to wiphy structure
 * @param wdev                  A pointer to wireless_dev structure
 * @param cookie                A pointer to timer cookie
 *
 * @return                  0 -- success, otherwise fail
 */
static int woal_cfg80211_cancel_remain_on_channel(struct wiphy *wiphy,
						  struct wireless_dev *wdev,
						  u64 cookie)
#else
/**
 * @brief Cancel remain on channel
 *
 * @param wiphy                 A pointer to wiphy structure
 * @param dev                   A pointer to net_device structure
 * @param cookie                A pointer to timer cookie
 *
 * @return                  0 -- success, otherwise fail
 */
static int woal_cfg80211_cancel_remain_on_channel(struct wiphy *wiphy,
						  struct net_device *dev,
						  u64 cookie)
#endif
{
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 6, 0)
	struct net_device *dev = wdev->netdev;
#endif
	moal_private *priv = (moal_private *)woal_get_netdev_priv(dev);
	moal_private *remain_priv = NULL;
	int ret = 0;
	t_u8 status = 1;

	ENTER();
	PRINTM(MIOCTL, "Cancel remain on Channel: cookie = %#llx\n", cookie);
	remain_priv = priv->phandle->priv[priv->phandle->remain_bss_index];
	if (!remain_priv) {
		PRINTM(MERROR,
		       "cancel_remain_on_channel: Wrong remain_bss_index=%d\n",
		       priv->phandle->remain_bss_index);
		ret = -EFAULT;
		goto done;
	}
	if (woal_cfg80211_remain_on_channel_cfg(remain_priv, MOAL_IOCTL_WAIT,
						MTRUE, &status, NULL, 0, 0)) {
		PRINTM(MERROR,
		       "cancel_remain_on_channel: Fail to cancel remain on channel\n");
		ret = -EFAULT;
		goto done;
	}

	priv->phandle->remain_on_channel = MFALSE;
	if (priv->phandle->cookie)
		priv->phandle->cookie = 0;
done:
	LEAVE();
	return ret;
}
#endif /* KERNEL_VERSION */

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 2, 0)
/**
 * @brief start sched scan
 *
 * @param wiphy                 A pointer to wiphy structure
 * @param dev                   A pointer to net_device structure
 * @param request               A pointer to struct cfg80211_sched_scan_request
 *
 * @return                  0 -- success, otherwise fail
 */
int woal_cfg80211_sched_scan_start(struct wiphy *wiphy, struct net_device *dev,
				   struct cfg80211_sched_scan_request *request)
{
	struct ieee80211_channel *chan = NULL;
	int i = 0;
	int ret = 0;
	moal_private *priv = (moal_private *)woal_get_netdev_priv(dev);
	struct cfg80211_ssid *ssid = NULL;
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 19, 0)
	t_u8 buf[ETH_ALEN];
#endif
	int index = 0;
	int j = 0;
	struct cfg80211_ssid *match_ssid = NULL;
#ifdef WIFI_DIRECT_SUPPORT
	mlan_scan_cfg scan_cfg;
#endif

	ENTER();

#ifdef UAP_CFG80211
	if (GET_BSS_ROLE(priv) == MLAN_BSS_ROLE_UAP) {
		LEAVE();
		return -EFAULT;
	}
#endif

	memset(&priv->scan_cfg, 0, sizeof(priv->scan_cfg));
	if (!request) {
		PRINTM(MERROR, "Invalid sched_scan req parameter\n");
		LEAVE();
		return -EINVAL;
	}
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 4, 0)
	PRINTM(MCMND,
	       "%s sched scan: n_ssids=%d n_match_sets=%d n_channels=%d interval=%d iterations=%d ie_len=%d\n",
	       priv->netdev->name, request->n_ssids, request->n_match_sets,
	       request->n_channels, request->scan_plans[0].interval,
	       request->scan_plans[0].iterations, (int)request->ie_len);
#else
	PRINTM(MCMND,
	       "%s sched scan: n_ssids=%d n_match_sets=%d n_channels=%d interval=%d ie_len=%d\n",
	       priv->netdev->name, request->n_ssids, request->n_match_sets,
	       request->n_channels, request->interval, (int)request->ie_len);
#endif
	/** We have pending scan, start bgscan later */
	if (priv->phandle->scan_pending_on_block)
		priv->scan_cfg.start_later = MTRUE;
	for (i = 0; i < request->n_ssids; i++) {
		ssid = &request->ssids[i];
		strncpy(priv->scan_cfg.ssid_list[index].ssid, ssid->ssid,
			ssid->ssid_len);
		if (ssid->ssid_len) {
			priv->scan_cfg.ssid_list[index].max_len = 0;
			index++;
		}
		PRINTM(MCMND, "sched scan: ssid=%s\n", ssid->ssid);
	}

	for (i = 0; i < request->n_match_sets; i++) {
		match_ssid = &request->match_sets[i].ssid;
		for (j = 0; j < request->n_ssids; j++) {
			ssid = &request->ssids[j];
			if (ssid->ssid_len == match_ssid->ssid_len &&
			    !strncmp(ssid->ssid, match_ssid->ssid,
				     ssid->ssid_len))
				break;
		}
		if (j == request->n_ssids) {
			strncpy(priv->scan_cfg.ssid_list[index].ssid,
				match_ssid->ssid, match_ssid->ssid_len);
			priv->scan_cfg.ssid_list[index].max_len =
				match_ssid->ssid_len;
			index++;
		}
		PRINTM(MCMND, "sched scan: match_ssid=%s\n", match_ssid->ssid);
	}
	/** Add broadcast scan, when ssid_list is empty */
	if (!index)
		priv->scan_cfg.ssid_list[0].max_len = 0xff;
	for (i = 0; i < (int)MIN(WLAN_USER_SCAN_CHAN_MAX, request->n_channels);
	     i++) {
		chan = request->channels[i];
		priv->scan_cfg.chan_list[i].chan_number = chan->hw_value;
		priv->scan_cfg.chan_list[i].radio_type =
			woal_ieee_band_to_radio_type(chan->band);
		if (chan->flags &
		    (IEEE80211_CHAN_PASSIVE_SCAN | IEEE80211_CHAN_RADAR))
			priv->scan_cfg.chan_list[i].scan_type =
				MLAN_SCAN_TYPE_PASSIVE;
		else
			priv->scan_cfg.chan_list[i].scan_type =
				MLAN_SCAN_TYPE_ACTIVE;
		PRINTM(MCMD_D, "cfg80211_sched_scan: chan=%d chan->flag=0x%x\n",
		       chan->hw_value, chan->flags);
		priv->scan_cfg.chan_list[i].scan_time = 0;
#ifdef WIFI_DIRECT_SUPPORT
		if (priv->phandle->miracast_mode)
			priv->scan_cfg.chan_list[i].scan_time =
				priv->phandle->miracast_scan_time;
#endif
	}
	priv->scan_cfg.chan_per_scan =
		MIN(WLAN_USER_SCAN_CHAN_MAX, request->n_channels);

	/** set scan request IES */
	if (request->ie && request->ie_len) {
		if (MLAN_STATUS_SUCCESS !=
		    woal_cfg80211_mgmt_frame_ie(
			    priv, NULL, 0, NULL, 0, NULL, 0,
			    (const t_u8 *)request->ie, request->ie_len,
			    MGMT_MASK_PROBE_REQ, MOAL_IOCTL_WAIT)) {
			PRINTM(MERROR, "Fail to set sched scan IE\n");
			ret = -EFAULT;
			goto done;
		}
	} else {
		/** Clear SCAN IE in Firmware */
		if (priv->probereq_index != MLAN_CUSTOM_IE_AUTO_IDX_MASK) {
			if (MLAN_STATUS_SUCCESS !=
			    woal_cfg80211_mgmt_frame_ie(
				    priv, NULL, 0, NULL, 0, NULL, 0, NULL, 0,
				    MGMT_MASK_PROBE_REQ, MOAL_IOCTL_WAIT)) {
				PRINTM(MERROR, "Fail to clear sched scan IE\n");
				ret = -EFAULT;
				goto done;
			}
		}
	}

	/* Interval between scan cycles in milliseconds,supplicant set to 10
	 * second */
	/* We want to use 30 second for per scan cycle */
	priv->scan_cfg.scan_interval = MIN_BGSCAN_INTERVAL;
	priv->scan_cfg.repeat_count = 0;

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 4, 0)
	/* interval in seconds */
	if (request->scan_plans[0].interval)
		priv->scan_cfg.scan_interval =
			request->scan_plans[0].interval * 1000;
	priv->scan_cfg.repeat_count = request->scan_plans[0].iterations;
	if (request->n_scan_plans >= 2) {
		priv->scan_cfg.config_ees = MTRUE;
		priv->scan_cfg.ees_mode =
			MBIT(EES_MODE_HIGH) | MBIT(EES_MODE_MID);
		/*High scan interval in milliseconds*/
		priv->scan_cfg.high_period =
			request->scan_plans[0].interval * 1000;
		priv->scan_cfg.high_period_count =
			request->scan_plans[0].iterations;
		/*Mid scan interval in seconds*/
		priv->scan_cfg.mid_period = request->scan_plans[1].interval;
		priv->scan_cfg.mid_period_count =
			request->scan_plans[1].iterations;
		if (request->n_scan_plans == 3) {
			priv->scan_cfg.ees_mode |= MBIT(EES_MODE_LOW);
			/*low scan interval in seconds*/
			priv->scan_cfg.low_period =
				request->scan_plans[2].interval;
			priv->scan_cfg.low_period_count =
				request->scan_plans[2].iterations;
		}
	}
#else
	/* interval in miliseconds */
	if (request->interval)
		priv->scan_cfg.scan_interval = request->interval;
#endif
	priv->scan_cfg.report_condition =
		BG_SCAN_SSID_MATCH | BG_SCAN_WAIT_ALL_CHAN_DONE;
	priv->scan_cfg.bss_type = MLAN_BSS_MODE_INFRA;
	priv->scan_cfg.action = BG_SCAN_ACT_SET;
	priv->scan_cfg.enable = MTRUE;
#ifdef WIFI_DIRECT_SUPPORT
	memset(&scan_cfg, 0, sizeof(mlan_scan_cfg));
	if (MLAN_STATUS_SUCCESS != woal_get_scan_config(priv, &scan_cfg)) {
		PRINTM(MERROR, "Fail to get scan request IE\n");
	}
	if (priv->phandle->miracast_mode) {
		priv->scan_cfg.scan_chan_gap = priv->phandle->scan_chan_gap;
	} else {
		if (scan_cfg.scan_chan_gap)
			priv->scan_cfg.scan_chan_gap = scan_cfg.scan_chan_gap;
		else if (woal_is_any_interface_active(priv->phandle))
			priv->scan_cfg.scan_chan_gap =
				priv->phandle->scan_chan_gap;
		else
			priv->scan_cfg.scan_chan_gap = 0;
	}
#endif

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 19, 0)
	if (request->flags & NL80211_SCAN_FLAG_RANDOM_ADDR) {
		PRINTM(MIOCTL, "NL80211_SCAN_FLAG_RANDOM_ADDR is set\n");
		get_random_bytes(buf, ETH_ALEN);
		for (i = 0; i < ETH_ALEN; i++) {
			buf[i] &= ~request->mac_addr_mask[i];
			buf[i] |= request->mac_addr[i] &
				  request->mac_addr_mask[i];
		}
		moal_memcpy_ext(priv->phandle, priv->scan_cfg.random_mac, buf,
				ETH_ALEN, sizeof(priv->scan_cfg.random_mac));
	} else
#endif
		moal_memcpy_ext(priv->phandle, priv->scan_cfg.random_mac,
				priv->random_mac, ETH_ALEN,
				sizeof(priv->scan_cfg.random_mac));

	PRINTM(MCMND, "wlan:random_mac " MACSTR "\n",
	       MAC2STR(priv->scan_cfg.random_mac));
	if (MLAN_STATUS_SUCCESS ==
	    woal_request_bgscan(priv, MOAL_IOCTL_WAIT, &priv->scan_cfg)) {
		PRINTM(MMSG, "wlan: sched scan start\n");
		priv->sched_scanning = MTRUE;
		priv->bg_scan_start = MTRUE;
		priv->bg_scan_reported = MFALSE;
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 12, 0)
		priv->bg_scan_reqid = request->reqid;
#endif
	} else
		ret = -EFAULT;
done:
	LEAVE();
	return ret;
}

/**
 * @brief stop sched scan
 *
 * @param wiphy                 A pointer to wiphy structure
 * @param dev                   A pointer to net_device structure
 *
 * @return                      0 -- success, otherwise fail
 */
int woal_cfg80211_sched_scan_stop(struct wiphy *wiphy, struct net_device *dev
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 12, 0)
				  ,
				  u64 reqid
#endif
)
{
	moal_private *priv = (moal_private *)woal_get_netdev_priv(dev);
	ENTER();
	PRINTM(MMSG, "wlan: sched scan stop\n");
	priv->sched_scanning = MFALSE;
	woal_stop_bg_scan(priv, MOAL_NO_WAIT);
	priv->bg_scan_start = MFALSE;
	priv->bg_scan_reported = MFALSE;
	LEAVE();
	return 0;
}
#endif

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 2, 0)
/**
 * @brief cfg80211_resume handler
 *
 * @param wiphy                 A pointer to wiphy structure
 *
 * @return                      0 -- success, otherwise fail
 */
int woal_cfg80211_resume(struct wiphy *wiphy)
{
	moal_handle *handle = (moal_handle *)woal_get_wiphy_priv(wiphy);
	moal_private *priv = woal_get_priv(handle, MLAN_BSS_ROLE_ANY);
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 11, 0) && defined(CONFIG_PM)
	struct cfg80211_wowlan_wakeup wakeup_report;
#endif
	mlan_ds_hs_wakeup_reason wakeup_reason;
	int i;

	PRINTM(MCMND, "<--- Enter woal_cfg80211_resume --->\n");

	if (!priv) {
		PRINTM(MERROR, "woal_cfg80211_resume: priv is NULL\n");
		goto done;
	}
	handle->cfg80211_suspend = MFALSE;
	for (i = 0; i < MIN(handle->priv_num, MLAN_MAX_BSS_NUM); i++) {
		if (handle->priv[i] &&
		    (GET_BSS_ROLE(handle->priv[i]) == MLAN_BSS_ROLE_STA)) {
			if (handle->priv[i]->last_event &
			    EVENT_BG_SCAN_REPORT) {
				if (handle->priv[i]->sched_scanning) {
					woal_inform_bss_from_scan_result(
						handle->priv[i], NULL,
						MOAL_IOCTL_WAIT);
#if KERNEL_VERSION(3, 2, 0) <= CFG80211_VERSION_CODE
					woal_report_sched_scan_result(
						handle->priv[i]);
					woal_sched_timeout(50);
					woal_bgscan_stop_event(handle->priv[i]);
#endif
					handle->priv[i]->last_event = 0;
					PRINTM(MCMND,
					       "Report sched scan result in cfg80211 resume\n");
				}
				if (!moal_extflg_isset(handle, EXT_HW_TEST) &&
				    handle->priv[i]->roaming_enabled) {
					handle->priv[i]->roaming_required =
						MTRUE;
#ifdef ANDROID_KERNEL
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 1, 0)
					__pm_wakeup_event(
						&handle->ws,
						ROAMING_WAKE_LOCK_TIMEOUT);
#else
					wake_lock_timeout(
						&handle->wake_lock,
						msecs_to_jiffies(
							ROAMING_WAKE_LOCK_TIMEOUT));
#endif
#endif
#ifdef REASSOCIATION
					wake_up_interruptible(
						&handle->reassoc_thread.wait_q);
#endif
				}
			}
		}
	}

	memset((t_u8 *)&wakeup_reason, 0, sizeof(wakeup_reason));
	if (MLAN_STATUS_SUCCESS !=
	    woal_get_wakeup_reason(priv, &wakeup_reason)) {
		PRINTM(MERROR, "%s: get_wakeup_reason failed \n", __func__);
		goto done;
	}
#ifdef STA_CFG80211
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 14, 0)
	if (IS_STA_CFG80211(priv->phandle->params.cfg80211_wext))
		woal_wake_reason_logger(priv, wakeup_reason);
#endif
#endif

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 11, 0) && defined(CONFIG_PM)
	memset(&wakeup_report, 0, sizeof(struct cfg80211_wowlan_wakeup));
	wakeup_report.pattern_idx = -1;

	switch (wakeup_reason.hs_wakeup_reason) {
	case NO_HSWAKEUP_REASON:
		break;
	case BCAST_DATA_MATCHED:
		break;
	case MCAST_DATA_MATCHED:
		break;
	case UCAST_DATA_MATCHED:
		break;
	case MASKTABLE_EVENT_MATCHED:
		break;
	case NON_MASKABLE_EVENT_MATCHED:
		break;
	case NON_MASKABLE_CONDITION_MATCHED:
		if (wiphy->wowlan_config && wiphy->wowlan_config->disconnect)
			wakeup_report.disconnect = true;
		break;
	case MAGIC_PATTERN_MATCHED:
		if (wiphy->wowlan_config && wiphy->wowlan_config->magic_pkt)
			wakeup_report.magic_pkt = true;
		if (wiphy->wowlan_config && wiphy->wowlan_config->n_patterns)
			wakeup_report.pattern_idx = 1;
		break;
	case CONTROL_FRAME_MATCHED:
		break;
	case MANAGEMENT_FRAME_MATCHED:
		break;
	case GTK_REKEY_FAILURE:
		if (wiphy->wowlan_config &&
		    wiphy->wowlan_config->gtk_rekey_failure)
			wakeup_report.gtk_rekey_failure = true;
		break;
	case MGMT_FRAME_FILTER_EXT_MATCHED:
		break;
	default:
		break;
	}

	if ((wakeup_reason.hs_wakeup_reason > 0) &&
	    (wakeup_reason.hs_wakeup_reason <= 11)) {
		cfg80211_report_wowlan_wakeup(priv->wdev, &wakeup_report,
					      GFP_KERNEL);
	}
#endif

done:
	woal_queue_rx_task(handle);
	PRINTM(MCMND, "<--- Leave woal_cfg80211_resume --->\n");
	return 0;
}

/**
 * @brief is_wowlan_pattern_supported
 *
 * @param priv                 A pointer to moal_private
 * @param pat                 A pointer to wowlan pattern
 * @param byte_seq       A pointer to byte_seq
 *
 * @return                      1 -- support, 0 -- not support
 */
static t_bool is_wowlan_pattern_supported(moal_private *priv,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 12, 0)
					  struct cfg80211_pkt_pattern *pat,
#else
					  struct cfg80211_wowlan_trig_pkt_pattern
						  *pat,
#endif
					  s8 *byte_seq)
{
	int j, k, valid_byte_cnt = 0;
	t_bool dont_care_byte = MFALSE;

	for (j = 0; j < DIV_ROUND_UP(pat->pattern_len, 8); j++) {
		for (k = 0; k < 8; k++) {
			if (pat->mask[j] & 1 << k) {
				moal_memcpy_ext(priv->phandle,
						byte_seq + valid_byte_cnt,
						&pat->pattern[j * 8 + k], 1, 1);
				valid_byte_cnt++;
				if (dont_care_byte)
					return MFALSE;
			} else {
				if (valid_byte_cnt)
					dont_care_byte = MTRUE;
			}

			if (valid_byte_cnt > MAX_NUM_BYTE_SEQ)
				return MFALSE;
		}
	}

	byte_seq[MAX_NUM_BYTE_SEQ] = valid_byte_cnt;

	return MTRUE;
}

/**
 * @brief cfg80211_suspend handler
 *
 * @param wiphy                 A pointer to wiphy structure
 * @param wow                   A pointer to cfg80211_wowlan
 *
 * @return                      0 -- success, otherwise fail
 */
int woal_cfg80211_suspend(struct wiphy *wiphy, struct cfg80211_wowlan *wow)
{
	moal_handle *handle = (moal_handle *)woal_get_wiphy_priv(wiphy);
	int i;
	int ret = 0;
	moal_private *priv = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;
	mlan_ds_misc_mef_flt_cfg mef_cfg;
	mef_entry_t *mef_entry = NULL;
	int filt_num = 0;
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 9, 0)
	t_bool first_pat = MTRUE;
#endif
	t_u8 byte_seq[MAX_NUM_BYTE_SEQ + 1];
	const t_u8 ipv4_mc_mac[] = {0x33, 0x33};
	const t_u8 ipv6_mc_mac[] = {0x01, 0x00, 0x5e};
	mlan_ds_hs_cfg hscfg;
	priv = woal_get_priv(handle, MLAN_BSS_ROLE_STA);
	if (!priv) {
		return 0;
	}

	PRINTM(MCMND, "<--- Enter woal_cfg80211_suspend --->\n");
	for (i = 0; i < MIN(handle->priv_num, MLAN_MAX_BSS_NUM); i++) {
		if (handle->priv[i]) {
			if (handle->scan_request) {
				PRINTM(MIOCTL,
				       "Cancel pending scan in woal_cfg80211_suspend\n");
				woal_cancel_scan(handle->priv[i],
						 MOAL_IOCTL_WAIT);
			}
			handle->priv[i]->last_event = 0;
		}
	}

	handle->cfg80211_suspend = MTRUE;
	if (!wow) {
		PRINTM(MEVENT,
		       "None of the WOWLAN triggers enabled in suspend\n");
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
		if (priv->delay_deauth_notify) {
			priv->delay_deauth_notify = MFALSE;
			priv->host_mlme = MFALSE;
			priv->auth_flag = 0;
			priv->auth_alg = 0xFFFF;
			/*send deauth packet to notify disconnection to
			 * wpa_supplicant */
			woal_deauth_event(priv, MLAN_REASON_DEAUTH_LEAVING,
					  priv->bssid_notify);
		}
#endif
		ret = 0;
		goto done;
	}

	if ((!priv || ((priv->bss_role == MLAN_BSS_ROLE_STA) &&
		       !priv->media_connected)) ||
	    ((priv->bss_role == MLAN_BSS_ROLE_UAP) && !priv->bss_started)) {
		PRINTM(MERROR,
		       "Can not configure WOWLAN in disconnected state\n");
		ret = 0;
		goto done;
	}

	PRINTM(MCMND, "wow->n_patterns=%d\n", wow->n_patterns);
	PRINTM(MCMND, "wow->any=%d\n", wow->any);
	PRINTM(MCMND, "wow->disconnect=%d\n", wow->disconnect);
	PRINTM(MCMND, "wow->magic_pkt=%d\n", wow->magic_pkt);
	PRINTM(MCMND, "wow->gtk_rekey_failure=%d\n", wow->gtk_rekey_failure);
	PRINTM(MCMND, "wow->eap_identity_req=%d\n", wow->eap_identity_req);
	PRINTM(MCMND, "wow->four_way_handshake=%d\n", wow->four_way_handshake);
	PRINTM(MCMND, "wow->rfkill_release=%d\n", wow->rfkill_release);

	if (!(wow->n_patterns) && !(wow->magic_pkt)) {
		PRINTM(MCMND, "No pattern or magic packet configured\n");
		ret = 0;
		goto done;
	}

	memset(&mef_cfg, 0, sizeof(mef_cfg));
	mef_cfg.mef_act_type = MEF_ACT_WOWLAN;
	mef_entry = &mef_cfg.mef_entry;

	mef_entry->mode = MEF_MODE_HOST_SLEEP;
	mef_entry->action = MEF_ACTION_ALLOW_AND_WAKEUP_HOST;

	for (i = 0; i < wow->n_patterns; i++) {
		memset(byte_seq, 0, sizeof(byte_seq));
		if (!is_wowlan_pattern_supported(priv, &wow->patterns[i],
						 byte_seq)) {
			PRINTM(MERROR, "Pattern not supported\n");
			ret = -EOPNOTSUPP;
			goto done;
		}

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 9, 0)
		if (!wow->patterns[i].pkt_offset) {
#endif
			if (!(byte_seq[0] & 0x01) &&
			    (byte_seq[MAX_NUM_BYTE_SEQ] == 1)) {
				mef_cfg.criteria |= CRITERIA_UNICAST;
				continue;
			} else if (is_broadcast_ether_addr(byte_seq)) {
				mef_cfg.criteria |= CRITERIA_BROADCAST;
				continue;
			} else if ((!memcmp(byte_seq, ipv4_mc_mac, 2) &&
				    (byte_seq[MAX_NUM_BYTE_SEQ] == 2)) ||
				   (!memcmp(byte_seq, ipv6_mc_mac, 3) &&
				    (byte_seq[MAX_NUM_BYTE_SEQ] == 3))) {
				mef_cfg.criteria |= CRITERIA_MULTICAST;
				continue;
			}
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 9, 0)
		}

		mef_entry->filter_item[filt_num].fill_flag =
			(FILLING_TYPE | FILLING_REPEAT | FILLING_BYTE_SEQ |
			 FILLING_OFFSET);
		mef_entry->filter_item[filt_num].repeat = 1;
		mef_entry->filter_item[filt_num].offset =
			wow->patterns[i].pkt_offset;
		moal_memcpy_ext(
			priv->phandle,
			mef_entry->filter_item[filt_num].byte_seq, byte_seq,
			MAX_NUM_BYTE_SEQ,
			sizeof(mef_entry->filter_item[filt_num].byte_seq));
		mef_entry->filter_item[filt_num].num_byte_seq =
			byte_seq[MAX_NUM_BYTE_SEQ];
		mef_entry->filter_item[filt_num].type = TYPE_BYTE_EQ;

		if (first_pat)
			first_pat = MFALSE;
		else
			mef_entry->rpn[filt_num] = RPN_TYPE_OR;

		filt_num++;
#endif
	}

	if (wow->magic_pkt) {
		mef_cfg.criteria |= CRITERIA_UNICAST | CRITERIA_BROADCAST |
				    CRITERIA_MULTICAST;
		mef_entry->filter_item[filt_num].fill_flag =
			(FILLING_TYPE | FILLING_REPEAT | FILLING_BYTE_SEQ |
			 FILLING_OFFSET);
		mef_entry->filter_item[filt_num].repeat = 16;
		moal_memcpy_ext(
			priv->phandle,
			mef_entry->filter_item[filt_num].byte_seq,
			priv->current_addr, ETH_ALEN,
			sizeof(mef_entry->filter_item[filt_num].byte_seq));
		mef_entry->filter_item[filt_num].num_byte_seq = ETH_ALEN;
		mef_entry->filter_item[filt_num].offset = 56;
		mef_entry->filter_item[filt_num].type = TYPE_BYTE_EQ;
		if (filt_num)
			mef_entry->rpn[filt_num] = RPN_TYPE_OR;
		filt_num++;
		mef_entry->filter_item[filt_num].fill_flag =
			(FILLING_TYPE | FILLING_REPEAT | FILLING_BYTE_SEQ |
			 FILLING_OFFSET);
		mef_entry->filter_item[filt_num].repeat = 16;
		moal_memcpy_ext(
			priv->phandle,
			mef_entry->filter_item[filt_num].byte_seq,
			priv->current_addr, ETH_ALEN,
			sizeof(mef_entry->filter_item[filt_num].byte_seq));
		mef_entry->filter_item[filt_num].num_byte_seq = ETH_ALEN;
		mef_entry->filter_item[filt_num].offset = 28;
		mef_entry->filter_item[filt_num].type = TYPE_BYTE_EQ;
		if (filt_num)
			mef_entry->rpn[filt_num] = RPN_TYPE_OR;
		filt_num++;
	}

	mef_entry->filter_num = filt_num;

	if (!mef_cfg.criteria)
		mef_cfg.criteria = CRITERIA_BROADCAST | CRITERIA_UNICAST |
				   CRITERIA_MULTICAST;

	status = woal_set_get_wowlan_config(priv, MLAN_ACT_SET, MOAL_IOCTL_WAIT,
					    &mef_cfg);
	if (status != MLAN_STATUS_SUCCESS) {
		PRINTM(MERROR, "woal_set_get_wowlan_config fail!\n");
		ret = -EFAULT;
		goto done;
	}

	memset(&hscfg, 0, sizeof(mlan_ds_hs_cfg));
	status = woal_set_get_hs_params(priv, MLAN_ACT_GET, MOAL_IOCTL_WAIT,
					&hscfg);
	if (status != MLAN_STATUS_SUCCESS) {
		PRINTM(MERROR,
		       "Fail to get HS parameter in woal_cfg80211_suspend: 0x%x 0x%x 0x%x\n",
		       hscfg.conditions, hscfg.gap, hscfg.gpio);
		ret = -EFAULT;
		goto done;
	}
	hscfg.is_invoke_hostcmd = MFALSE;
	if (wow->n_patterns || wow->magic_pkt)
		hscfg.conditions = 0;
	status = woal_set_get_hs_params(priv, MLAN_ACT_SET, MOAL_IOCTL_WAIT,
					&hscfg);
	if (status != MLAN_STATUS_SUCCESS) {
		PRINTM(MERROR,
		       "Fail to set HS parameter in woal_cfg80211_suspend: 0x%x 0x%x 0x%x\n",
		       hscfg.conditions, hscfg.gap, hscfg.gpio);
		ret = -EFAULT;
		goto done;
	}

done:
	PRINTM(MCMND, "<--- Leave woal_cfg80211_suspend --->\n");
	return ret;
}
#endif

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 5, 0)
static void woal_cfg80211_set_wakeup(struct wiphy *wiphy, bool enabled)
{
	moal_handle *handle = (moal_handle *)woal_get_wiphy_priv(wiphy);

	device_set_wakeup_enable(handle->hotplug_device, enabled);
}
#endif

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 2, 0)
/**
 *  @brief TDLS operation ioctl handler
 *
 *  @param priv     A pointer to moal_private structure
 *  @param peer     A pointer to peer mac
 *  @apram action   action for TDLS
 *  @return         0 --success, otherwise fail
 */
static int woal_tdls_oper(moal_private *priv, const u8 *peer, t_u8 action)
{
	mlan_ioctl_req *ioctl_req = NULL;
	mlan_ds_misc_cfg *misc = NULL;
	int ret = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	ioctl_req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (ioctl_req == NULL) {
		ret = -ENOMEM;
		goto done;
	}
	misc = (mlan_ds_misc_cfg *)ioctl_req->pbuf;
	misc->sub_command = MLAN_OID_MISC_TDLS_OPER;
	ioctl_req->req_id = MLAN_IOCTL_MISC_CFG;
	ioctl_req->action = MLAN_ACT_SET;
	misc->param.tdls_oper.tdls_action = action;
	moal_memcpy_ext(priv->phandle, misc->param.tdls_oper.peer_mac, peer,
			ETH_ALEN, sizeof(misc->param.tdls_oper.peer_mac));
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
 *  @brief TDLS operation ioctl handler
 *
 *  @param priv         A pointer to moal_private structure
 *  @param peer         A pointer to peer mac
 *  @param tdls_ies     A pointer to mlan_ds_misc_tdls_ies structure
 *  @param flags        TDLS ie flags
 *
 *  @return         0 --success, otherwise fail
 */
static int woal_tdls_get_ies(moal_private *priv, const u8 *peer,
			     mlan_ds_misc_tdls_ies *tdls_ies, t_u16 flags)
{
	mlan_ioctl_req *ioctl_req = NULL;
	mlan_ds_misc_cfg *misc = NULL;
	int ret = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	ioctl_req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (ioctl_req == NULL) {
		ret = -ENOMEM;
		goto done;
	}
	misc = (mlan_ds_misc_cfg *)ioctl_req->pbuf;
	misc->sub_command = MLAN_OID_MISC_GET_TDLS_IES;
	ioctl_req->req_id = MLAN_IOCTL_MISC_CFG;
	ioctl_req->action = MLAN_ACT_GET;
	misc->param.tdls_ies.flags = flags;
	moal_memcpy_ext(priv->phandle, misc->param.tdls_ies.peer_mac, peer,
			ETH_ALEN, sizeof(misc->param.tdls_ies.peer_mac));
	status = woal_request_ioctl(priv, ioctl_req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}
	if (tdls_ies)
		moal_memcpy_ext(priv->phandle, tdls_ies, &misc->param.tdls_ies,
				sizeof(mlan_ds_misc_tdls_ies),
				sizeof(mlan_ds_misc_tdls_ies));
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(ioctl_req);
	LEAVE();
	return ret;
}

/**
 * @brief append tdls ext_capability
 *
 * @param skb                   A pointer to sk_buff structure
 *
 * @return                      N/A
 */
static void woal_tdls_add_ext_capab(moal_private *priv, struct sk_buff *skb,
				    mlan_ds_misc_tdls_ies *tdls_ies)
{
	u8 *pos = NULL;
	if (tdls_ies->ext_cap[0] == WLAN_EID_EXT_CAPABILITY) {
		pos = (void *)skb_put(skb, sizeof(IEEEtypes_ExtCap_t));
		moal_memcpy_ext(priv->phandle, pos, tdls_ies->ext_cap,
				sizeof(IEEEtypes_ExtCap_t),
				sizeof(IEEEtypes_ExtCap_t));
	} else {
		PRINTM(MERROR, "Fail to append tdls ext_capability\n");
	}
}

/**
 * @brief append supported rates
 *
 * @param priv                  A pointer to moal_private structure
 * @param skb                   A pointer to sk_buff structure
 * @param band                  AP's band
 *
 * @return                      N/A
 */
static void woal_add_supported_rates_ie(moal_private *priv, struct sk_buff *skb,
					enum ieee80211_band band)
{
	t_u8 basic_rates[] = {0x82, 0x84, 0x8b, 0x96, 0x0c, 0x12, 0x18, 0x24};
	t_u8 basic_rates_5G[] = {0x0c, 0x12, 0x18, 0x24,
				 0x30, 0x48, 0x60, 0x6c};
	t_u8 *pos;
	t_u8 rate_num = 0;
	if (band == IEEE80211_BAND_2GHZ)
		rate_num = sizeof(basic_rates);
	else
		rate_num = sizeof(basic_rates_5G);

	if (skb_tailroom(skb) < rate_num + 2)
		return;

	pos = skb_put(skb, rate_num + 2);
	*pos++ = WLAN_EID_SUPP_RATES;
	*pos++ = rate_num;
	if (band == IEEE80211_BAND_2GHZ)
		moal_memcpy_ext(priv->phandle, pos, basic_rates, rate_num,
				rate_num);
	else
		moal_memcpy_ext(priv->phandle, pos, basic_rates_5G, rate_num,
				rate_num);
	return;
}

/**
 * @brief append ext_supported rates
 *
 * @param priv                  A pointer to moal_private structure
 * @param skb                   A pointer to sk_buff structure
 * @param band                  AP's band
 *
 * @return                      N/A
 */
static void woal_add_ext_supported_rates_ie(moal_private *priv,
					    struct sk_buff *skb,
					    enum ieee80211_band band)
{
	t_u8 ext_rates[] = {0x0c, 0x12, 0x18, 0x60};
	t_u8 *pos;
	t_u8 rate_num = sizeof(ext_rates);

	if (band != IEEE80211_BAND_2GHZ)
		return;

	if (skb_tailroom(skb) < rate_num + 2)
		return;

	pos = skb_put(skb, rate_num + 2);
	*pos++ = WLAN_EID_EXT_SUPP_RATES;
	*pos++ = rate_num;
	moal_memcpy_ext(priv->phandle, pos, ext_rates, rate_num, rate_num);
	return;
}

/**
 * @brief append wmm ie
 *
 * @param priv                  A pointer to moal_private structure
 * @param skb                   A pointer to sk_buff structure
 * @param wmm_type         WMM_TYPE_INFO/WMM_TYPE_PARAMETER
 * @param pQosInfo           A pointer to qos info
 *
 * @return                      N/A
 */
static void woal_add_wmm_ie(moal_private *priv, struct sk_buff *skb,
			    t_u8 wmm_type, t_u8 *pQosInfo)
{
	t_u8 wmmInfoElement[] = {0x00, 0x50, 0xf2, 0x02, 0x00, 0x01};
	t_u8 wmmParamElement[] = {0x00, 0x50, 0xf2, 0x02, 0x01, 0x01};
	t_u8 ac_vi[] = {0x42, 0x43, 0x5e, 0x00};
	t_u8 ac_vo[] = {0x62, 0x32, 0x2f, 0x00};
	t_u8 ac_be[] = {0x03, 0xa4, 0x00, 0x00};
	t_u8 ac_bk[] = {0x27, 0xa4, 0x00, 0x00};
	t_u8 qosInfo = 0x0;
	t_u8 reserved = 0;
	t_u8 wmm_id = 221;
	t_u8 wmmParamIe_len = 25;
	t_u8 wmmInfoIe_len = 7;
	t_u8 len = 0;
	t_u8 *pos;

	qosInfo = (pQosInfo == NULL) ? 0xf : (*pQosInfo);
	/*wmm parameter*/
	if (wmm_type == WMM_TYPE_PARAMETER) {
		if (skb_tailroom(skb) < (wmmParamIe_len + 2))
			return;
		pos = skb_put(skb, wmmParamIe_len + 2);
		len = wmmParamIe_len;
	} else {
		if (skb_tailroom(skb) < (wmmInfoIe_len + 2))
			return;
		pos = skb_put(skb, wmmInfoIe_len + 2);
		len = wmmInfoIe_len;
	}

	*pos++ = wmm_id;
	*pos++ = len;
	/*wmm parameter*/
	if (wmm_type == WMM_TYPE_PARAMETER) {
		moal_memcpy_ext(priv->phandle, pos, wmmParamElement,
				sizeof(wmmParamElement),
				sizeof(wmmParamElement));
		pos += sizeof(wmmParamElement);
	} else {
		moal_memcpy_ext(priv->phandle, pos, wmmInfoElement,
				sizeof(wmmInfoElement), sizeof(wmmInfoElement));
		pos += sizeof(wmmInfoElement);
	}
	*pos++ = qosInfo;
	/*wmm parameter*/
	if (wmm_type == WMM_TYPE_PARAMETER) {
		*pos++ = reserved;
		moal_memcpy_ext(priv->phandle, pos, ac_be, sizeof(ac_be),
				sizeof(ac_be));
		pos += sizeof(ac_be);
		moal_memcpy_ext(priv->phandle, pos, ac_bk, sizeof(ac_bk),
				sizeof(ac_bk));
		pos += sizeof(ac_bk);
		moal_memcpy_ext(priv->phandle, pos, ac_vi, sizeof(ac_vi),
				sizeof(ac_vi));
		pos += sizeof(ac_vi);
		moal_memcpy_ext(priv->phandle, pos, ac_vo, sizeof(ac_vo),
				sizeof(ac_vo));
	}
	return;
}

/**
 * @brief update tdls peer status
 *
 * @param priv                  A pointer to moal_private structure
 * @param peer_addr             A point to peer mac address
 * @param link_status           link status
 *
 * @return                      N/A
 */
static t_void woal_updata_peer_status(moal_private *priv, const t_u8 *peer_addr,
				      tdlsStatus_e link_status)
{
	struct tdls_peer *peer = NULL;
	unsigned long flags;
	if (priv && priv->enable_auto_tdls) {
		spin_lock_irqsave(&priv->tdls_lock, flags);
		// Coverity violation raised for kernel's API
		// coverity[cert_arr39_c_violation:SUPPRESS]
		list_for_each_entry (peer, &priv->tdls_list, link) {
			if (!memcmp(peer->peer_addr, peer_addr, ETH_ALEN)) {
				if ((link_status == TDLS_NOT_SETUP) &&
				    (peer->link_status ==
				     TDLS_SETUP_INPROGRESS))
					peer->num_failure++;
				else if (link_status == TDLS_SETUP_COMPLETE)
					peer->num_failure = 0;
				peer->link_status = link_status;
				break;
			}
		}
		spin_unlock_irqrestore(&priv->tdls_lock, flags);
	}
}

/**
 * @brief add tdls peer
 *
 * @param priv                  A pointer to moal_private structure
 * @param peer                  A point to peer address
 *
 * @return                      N/A
 */
static t_void woal_add_tdls_peer(moal_private *priv, const t_u8 *peer)
{
	struct tdls_peer *tdls_peer = NULL;
	unsigned long flags;
	t_u8 find_peer = MFALSE;
	if (priv && priv->enable_auto_tdls) {
		spin_lock_irqsave(&priv->tdls_lock, flags);
		// Coverity violation raised for kernel's API
		// coverity[cert_arr39_c_violation:SUPPRESS]
		list_for_each_entry (tdls_peer, &priv->tdls_list, link) {
			if (!memcmp(tdls_peer->peer_addr, peer, ETH_ALEN)) {
				tdls_peer->link_status = TDLS_SETUP_INPROGRESS;
				tdls_peer->rssi_jiffies = jiffies;
				find_peer = MTRUE;
				break;
			}
		}
		if (!find_peer) {
			/* create new TDLS peer */
			tdls_peer =
				kzalloc(sizeof(struct tdls_peer), GFP_ATOMIC);
			if (tdls_peer) {
				moal_memcpy_ext(priv->phandle,
						tdls_peer->peer_addr, peer,
						ETH_ALEN,
						sizeof(tdls_peer->peer_addr));
				tdls_peer->link_status = TDLS_SETUP_INPROGRESS;
				tdls_peer->rssi_jiffies = jiffies;
				INIT_LIST_HEAD(&tdls_peer->link);
				list_add_tail(&tdls_peer->link,
					      &priv->tdls_list);
				PRINTM(MCMND,
				       "Add to TDLS list: peer=" MACSTR "\n",
				       MAC2STR(peer));
			}
		}
		spin_unlock_irqrestore(&priv->tdls_lock, flags);
	}
}

/**
 * @brief check auto tdls
 *
 * @param wiphy                 A pointer to wiphy structure
 * @param dev                   A pointer to net_device structure
 *
 * @return                      N/A
 */
void woal_check_auto_tdls(struct wiphy *wiphy, struct net_device *dev)
{
	t_u8 bcast_addr[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
	struct tdls_peer *tdls_peer = NULL;
	unsigned long flags;
	t_u8 tdls_discovery = MFALSE;
	moal_private *priv = (moal_private *)woal_get_netdev_priv(dev);

	ENTER();
	if (priv && priv->enable_auto_tdls) {
		priv->tdls_check_tx = MFALSE;
		spin_lock_irqsave(&priv->tdls_lock, flags);
		// Coverity violation raised for kernel's API
		// coverity[cert_arr39_c_violation:SUPPRESS]
		list_for_each_entry (tdls_peer, &priv->tdls_list, link) {
			if ((jiffies - tdls_peer->rssi_jiffies) >
			    TDLS_IDLE_TIME) {
				tdls_peer->rssi = 0;
				if (tdls_peer->num_failure <
				    TDLS_MAX_FAILURE_COUNT)
					tdls_discovery = MTRUE;
			}
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
			if (tdls_peer->rssi &&
			    (tdls_peer->rssi >= TDLS_RSSI_LOW_THRESHOLD)) {
				if (tdls_peer->link_status ==
				    TDLS_SETUP_COMPLETE) {
					tdls_peer->link_status = TDLS_TEAR_DOWN;
					PRINTM(MMSG,
					       "Wlan: Tear down TDLS link, peer=" MACSTR
					       " rssi=%d\n",
					       MAC2STR(tdls_peer->peer_addr),
					       -tdls_peer->rssi);
					cfg80211_tdls_oper_request(
						dev, tdls_peer->peer_addr,
						NL80211_TDLS_TEARDOWN,
						TDLS_TEARN_DOWN_REASON_UNSPECIFIC,
						GFP_ATOMIC);
				}
			} else if (tdls_peer->rssi &&
				   (tdls_peer->rssi <=
				    TDLS_RSSI_HIGH_THRESHOLD)) {
				if ((tdls_peer->link_status ==
				     TDLS_NOT_SETUP) &&
				    (tdls_peer->num_failure <
				     TDLS_MAX_FAILURE_COUNT)) {
					priv->tdls_check_tx = MTRUE;
					PRINTM(MCMND,
					       "Wlan: Find TDLS peer=" MACSTR
					       " rssi=%d\n",
					       MAC2STR(tdls_peer->peer_addr),
					       -tdls_peer->rssi);
				}
			}
#endif
		}
		spin_unlock_irqrestore(&priv->tdls_lock, flags);
	}
	if (tdls_discovery)
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 15, 0)
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(6, 5, 0)
		woal_cfg80211_tdls_mgmt(wiphy, dev, bcast_addr, 0,
					TDLS_DISCOVERY_REQUEST, 1, 0, 0, 0,
					NULL, 0);
#else
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 17, 0)
		woal_cfg80211_tdls_mgmt(wiphy, dev, bcast_addr,
					TDLS_DISCOVERY_REQUEST, 1, 0, 0, 0,
					NULL, 0);
#else
		woal_cfg80211_tdls_mgmt(wiphy, dev, bcast_addr,
					TDLS_DISCOVERY_REQUEST, 1, 0, 0, NULL,
					0);
#endif
#endif
#else
		woal_cfg80211_tdls_mgmt(wiphy, dev, bcast_addr,
					TDLS_DISCOVERY_REQUEST, 1, 0, NULL, 0);
#endif
	LEAVE();
}

/**
 * @brief woal construct tdls data frame
 *
 * @param priv                  A pointer to moal_private structure
 * @param peer                  A pointer to peer mac
 * @param action_code           tdls action code
 * @param dialog_token          dialog_token
 * @param status_code           status_code
 * @param skb                   skb buffer
 *
 * @return                      0 -- success, otherwise fail
 */
static int woal_construct_tdls_data_frame(moal_private *priv, const t_u8 *peer,
					  t_u8 action_code, t_u8 dialog_token,
					  t_u16 status_code,
					  struct sk_buff *skb)
{
	struct ieee80211_tdls_data *tdata;
	t_u16 capability;
	IEEEtypes_HTCap_t *HTcap;
	IEEEtypes_HTInfo_t *HTInfo;
	IEEEtypes_2040BSSCo_t *BSSCo;
	IEEEtypes_VHTCap_t *VHTcap;
	IEEEtypes_VHTOprat_t *vht_oprat;
	IEEEtypes_AID_t *AidInfo;
	IEEEtypes_Header_t *ieee_hdr;
	t_u8 *skb_data;
	t_u8 len = 0;
	IEEEtypes_Generic_t *pSupp_chan = NULL, *pRegulatory_class = NULL;
	mlan_ds_misc_tdls_ies *tdls_ies = NULL;
	int ret = 0;
	mlan_bss_info bss_info;
	enum ieee80211_band band;
	mlan_fw_info fw_info;
	t_u16 setup_flag = 0;
	t_u16 confirm_flag = 0;

	ENTER();

	memset(&bss_info, 0, sizeof(bss_info));
	if (MLAN_STATUS_SUCCESS !=
	    woal_get_bss_info(priv, MOAL_IOCTL_WAIT, &bss_info)) {
		PRINTM(MERROR, "Fail to get bss info\n");
		LEAVE();
		return -EFAULT;
	}
	band = woal_band_cfg_to_ieee_band(bss_info.bss_band);
	tdls_ies = kzalloc(sizeof(mlan_ds_misc_tdls_ies), GFP_KERNEL);
	if (!tdls_ies) {
		PRINTM(MERROR, "Fail to alloc memory for tdls_ies\n");
		LEAVE();
		return -ENOMEM;
	}

	capability = 0x2421;

	memset(&fw_info, 0, sizeof(mlan_fw_info));
	tdata = (void *)skb_put(skb, offsetof(struct ieee80211_tdls_data, u));
	moal_memcpy_ext(priv->phandle, tdata->da, peer, ETH_ALEN,
			sizeof(tdata->da));
	moal_memcpy_ext(priv->phandle, tdata->sa, priv->current_addr, ETH_ALEN,
			sizeof(tdata->sa));
	tdata->ether_type = cpu_to_be16(MLAN_ETHER_PKT_TYPE_TDLS_ACTION);
	tdata->payload_type = WLAN_TDLS_SNAP_RFTYPE;
	woal_request_get_fw_info(priv, MOAL_IOCTL_WAIT, &fw_info);

	setup_flag = TDLS_IE_FLAGS_EXTCAP | TDLS_IE_FLAGS_HTCAP |
		     TDLS_IE_FLAGS_SUPP_CS_IE;
	confirm_flag = TDLS_IE_FLAGS_EXTCAP | TDLS_IE_FLAGS_HTINFO |
		       TDLS_IE_FLAGS_QOS_INFO;
	if (fw_info.fw_bands & BAND_AAC) {
		setup_flag |= (TDLS_IE_FLAGS_VHTCAP | TDLS_IE_FLAGS_AID);
		confirm_flag |= TDLS_IE_FLAGS_VHTOPRAT;
	}
	if (fw_info.fw_bands & BAND_AAX) {
		setup_flag |= (TDLS_IE_FLAGS_VHTCAP | TDLS_IE_FLAGS_AID |
			       TDLS_IE_FLAGS_HECAP);
		confirm_flag |= (TDLS_IE_FLAGS_VHTOPRAT | TDLS_IE_FLAGS_HEOP);
	}
	if (fw_info.fw_bands & BAND_GAX) {
		setup_flag |= TDLS_IE_FLAGS_HECAP;
		confirm_flag |= TDLS_IE_FLAGS_HEOP;
	}
	switch (action_code) {
	case WLAN_TDLS_SETUP_REQUEST:
		setup_flag |= TDLS_IE_FLAGS_SETUP;
		if (woal_tdls_get_ies(priv, peer, tdls_ies, setup_flag)) {
			PRINTM(MERROR, "%s: woal_tdls_get_ies failed \n",
			       __func__);
			ret = -EFAULT;
			goto done;
		}
		tdata->category = WLAN_CATEGORY_TDLS;
		tdata->action_code = WLAN_TDLS_SETUP_REQUEST;
		skb_put(skb, sizeof(tdata->u.setup_req));
		tdata->u.setup_req.dialog_token = dialog_token;
		tdata->u.setup_req.capability = cpu_to_le16(capability);
		woal_add_supported_rates_ie(priv, skb, band);
		woal_add_ext_supported_rates_ie(priv, skb, band);
		break;
	case WLAN_TDLS_SETUP_RESPONSE:
		if (woal_tdls_get_ies(priv, peer, tdls_ies, setup_flag)) {
			PRINTM(MERROR, "%s: woal_tdls_get_ies failed \n",
			       __func__);
			ret = -EFAULT;
			goto done;
		}
		tdata->category = WLAN_CATEGORY_TDLS;
		tdata->action_code = WLAN_TDLS_SETUP_RESPONSE;

		skb_put(skb, sizeof(tdata->u.setup_resp));
		tdata->u.setup_resp.status_code = cpu_to_le16(status_code);
		tdata->u.setup_resp.dialog_token = dialog_token;
		tdata->u.setup_resp.capability = cpu_to_le16(capability);

		woal_add_supported_rates_ie(priv, skb, band);
		woal_add_ext_supported_rates_ie(priv, skb, band);
		break;
	case WLAN_TDLS_SETUP_CONFIRM:
		if (woal_tdls_get_ies(priv, peer, tdls_ies, confirm_flag)) {
			PRINTM(MERROR, "%s: woal_tdls_get_ies failed \n",
			       __func__);
			ret = -EFAULT;
			goto done;
		}
		tdata->category = WLAN_CATEGORY_TDLS;
		tdata->action_code = WLAN_TDLS_SETUP_CONFIRM;

		skb_put(skb, sizeof(tdata->u.setup_cfm));
		tdata->u.setup_cfm.status_code = cpu_to_le16(status_code);
		tdata->u.setup_cfm.dialog_token = dialog_token;

		break;
	case WLAN_TDLS_TEARDOWN:
		tdata->category = WLAN_CATEGORY_TDLS;
		tdata->action_code = WLAN_TDLS_TEARDOWN;

		skb_put(skb, sizeof(tdata->u.teardown));
		tdata->u.teardown.reason_code = cpu_to_le16(status_code);
		break;
	case WLAN_TDLS_DISCOVERY_REQUEST:
		tdata->category = WLAN_CATEGORY_TDLS;
		tdata->action_code = WLAN_TDLS_DISCOVERY_REQUEST;

		skb_put(skb, sizeof(tdata->u.discover_req));
		tdata->u.discover_req.dialog_token = dialog_token;
		break;
	default:
		ret = -EINVAL;
		goto done;
	}

	if (action_code == WLAN_TDLS_SETUP_REQUEST ||
	    action_code == WLAN_TDLS_SETUP_RESPONSE) {
		/* supported chanel ie*/
		if (tdls_ies->supp_chan[0] == SUPPORTED_CHANNELS) {
			pSupp_chan = (void *)skb_put(
				skb, sizeof(IEEEtypes_Header_t) +
					     tdls_ies->supp_chan[1]);
			memset(pSupp_chan, 0,
			       sizeof(IEEEtypes_Header_t) +
				       tdls_ies->supp_chan[1]);
			moal_memcpy_ext(priv->phandle, pSupp_chan,
					tdls_ies->supp_chan,
					sizeof(IEEEtypes_Header_t) +
						tdls_ies->supp_chan[1],
					sizeof(IEEEtypes_Header_t) +
						tdls_ies->supp_chan[1]);
		}
		/* supported regulatory class ie*/
		if (tdls_ies->regulatory_class[0] == REGULATORY_CLASS) {
			pRegulatory_class = (void *)skb_put(
				skb, sizeof(IEEEtypes_Header_t) +
					     tdls_ies->regulatory_class[1]);
			memset(pRegulatory_class, 0,
			       sizeof(IEEEtypes_Header_t) +
				       tdls_ies->regulatory_class[1]);
			moal_memcpy_ext(priv->phandle, pRegulatory_class,
					tdls_ies->regulatory_class,
					sizeof(IEEEtypes_Header_t) +
						tdls_ies->regulatory_class[1],
					sizeof(IEEEtypes_Header_t) +
						tdls_ies->regulatory_class[1]);
		}
		woal_tdls_add_ext_capab(priv, skb, tdls_ies);
	}

	/* TODO we should fill in ht_cap and htinfo with correct value */
	switch (action_code) {
	case WLAN_TDLS_SETUP_REQUEST:
	case WLAN_TDLS_SETUP_RESPONSE:
		/*HT capability*/
		if (tdls_ies->ht_cap[0] == HT_CAPABILITY) {
			HTcap = (void *)skb_put(skb, sizeof(IEEEtypes_HTCap_t));
			memset(HTcap, 0, sizeof(IEEEtypes_HTCap_t));
			moal_memcpy_ext(priv->phandle, HTcap, tdls_ies->ht_cap,
					sizeof(IEEEtypes_HTCap_t),
					sizeof(IEEEtypes_HTCap_t));
		} else {
			PRINTM(MIOCTL, "No TDLS HT capability\n");
		}

		/*20_40_bss_coexist*/
		BSSCo = (void *)skb_put(skb, sizeof(IEEEtypes_2040BSSCo_t));
		memset(BSSCo, 0, sizeof(IEEEtypes_2040BSSCo_t));
		BSSCo->ieee_hdr.element_id = BSSCO_2040;
		BSSCo->ieee_hdr.len = sizeof(IEEEtypes_2040BSSCo_t) -
				      sizeof(IEEEtypes_Header_t);
		BSSCo->bss_co_2040.bss_co_2040_value = 0x01;

		/* VHT capability */
		if (tdls_ies->vht_cap[0] == VHT_CAPABILITY) {
			VHTcap = (void *)skb_put(skb,
						 sizeof(IEEEtypes_VHTCap_t));
			memset(VHTcap, 0, sizeof(IEEEtypes_VHTCap_t));
			moal_memcpy_ext(priv->phandle, VHTcap,
					tdls_ies->vht_cap,
					sizeof(IEEEtypes_VHTCap_t),
					sizeof(IEEEtypes_VHTCap_t));
		} else {
			PRINTM(MIOCTL, "NO TDLS VHT capability\n");
		}
		/* AID info */
		if (tdls_ies->aid_info[0] == AID_INFO) {
			AidInfo = (void *)skb_put(skb, sizeof(IEEEtypes_AID_t));
			memset(AidInfo, 0, sizeof(IEEEtypes_AID_t));
			moal_memcpy_ext(priv->phandle, AidInfo,
					tdls_ies->aid_info,
					sizeof(IEEEtypes_AID_t),
					sizeof(IEEEtypes_AID_t));
		} else {
			PRINTM(MIOCTL, "No TDLS AID info\n");
		}
		/* HE capability */
		if (tdls_ies->he_cap[2] == HE_CAPABILITY) {
			ieee_hdr = (IEEEtypes_Header_t *)tdls_ies->he_cap;
			len = sizeof(IEEEtypes_Header_t) + ieee_hdr->len;
			skb_data = (void *)skb_put(skb, len);
			memset(skb_data, 0, len);
			moal_memcpy_ext(priv->phandle, skb_data,
					tdls_ies->he_cap, len, len);
		} else {
			PRINTM(MIOCTL, "NO TDLS HE Capability IE\n");
		}
		break;
	case WLAN_TDLS_SETUP_CONFIRM:
		/*HT information*/
		if (tdls_ies->ht_info[0] == HT_OPERATION) {
			HTInfo = (void *)skb_put(skb,
						 sizeof(IEEEtypes_HTInfo_t));
			memset(HTInfo, 0, sizeof(IEEEtypes_HTInfo_t));
			moal_memcpy_ext(priv->phandle, HTInfo,
					tdls_ies->ht_info,
					sizeof(IEEEtypes_HTInfo_t),
					sizeof(IEEEtypes_HTInfo_t));
		} else
			PRINTM(MIOCTL, "No TDLS HT information\n");
		/** VHT operation */
		if (tdls_ies->vht_oprat[0] == VHT_OPERATION) {
			vht_oprat = (void *)skb_put(
				skb, sizeof(IEEEtypes_VHTOprat_t));
			memset(vht_oprat, 0, sizeof(IEEEtypes_VHTOprat_t));
			moal_memcpy_ext(priv->phandle, vht_oprat,
					tdls_ies->vht_oprat,
					sizeof(IEEEtypes_VHTOprat_t),
					sizeof(IEEEtypes_VHTOprat_t));
		} else
			PRINTM(MIOCTL, "NO TDLS VHT Operation IE\n");
		/** HE operation */
		if (tdls_ies->he_op[2] == HE_OPERATION) {
			ieee_hdr = (IEEEtypes_Header_t *)tdls_ies->he_op;
			len = sizeof(IEEEtypes_Header_t) + ieee_hdr->len;
			skb_data = (void *)skb_put(skb, len);
			memset(skb_data, 0, len);
			moal_memcpy_ext(priv->phandle, skb_data,
					tdls_ies->he_op, len, len);
		} else
			PRINTM(MIOCTL, "NO TDLS HE Operation IE\n");
		break;
	default:
		break;
	}

	if (action_code == WLAN_TDLS_SETUP_REQUEST ||
	    action_code == WLAN_TDLS_SETUP_RESPONSE) {
		/*wmm info*/
		woal_add_wmm_ie(priv, skb, WMM_TYPE_INFO, NULL);
	} else if (action_code == WLAN_TDLS_SETUP_CONFIRM) {
		/*wmm parameter*/
		woal_add_wmm_ie(priv, skb, WMM_TYPE_PARAMETER,
				&tdls_ies->QosInfo);
	}

done:
	kfree(tdls_ies);
	return ret;
}

/**
 * @brief woal construct tdls action frame
 *
 * @param priv                  A pointer to moal_private structure
 * @param peer                  A pointer to peer mac
 * @param action_code           tdls action code
 * @param dialog_token          dialog_token
 * @param status_code           status_code
 * @param skb                   skb buffer
 *
 * @return                      0 -- success, otherwise fail
 */
static int woal_construct_tdls_action_frame(moal_private *priv,
					    const t_u8 *peer, t_u8 action_code,
					    t_u8 dialog_token,
					    t_u16 status_code,
					    struct sk_buff *skb)
{
	struct ieee80211_mgmt *mgmt;
	t_u8 addr[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
	t_u16 capability;
	t_u8 *pos = NULL;
	mlan_ds_misc_tdls_ies *tdls_ies = NULL;
	mlan_bss_info bss_info;
	enum ieee80211_band band;
	IEEEtypes_Generic_t *pSupp_chan = NULL, *pRegulatory_class = NULL;

	int ret = 0;

	ENTER();

	memset(&bss_info, 0, sizeof(bss_info));
	if (MLAN_STATUS_SUCCESS !=
	    woal_get_bss_info(priv, MOAL_IOCTL_WAIT, &bss_info)) {
		PRINTM(MERROR, "Fail to get bss info\n");
		LEAVE();
		return -EFAULT;
	}
	band = woal_band_cfg_to_ieee_band(bss_info.bss_band);

	tdls_ies = kzalloc(sizeof(mlan_ds_misc_tdls_ies), GFP_KERNEL);
	if (!tdls_ies) {
		PRINTM(MERROR, "Fail to alloc memory for tdls_ies\n");
		LEAVE();
		return -ENOMEM;
	}

	mgmt = (void *)skb_put(skb, 24);
	memset(mgmt, 0, 24);
	moal_memcpy_ext(priv->phandle, mgmt->da, peer, ETH_ALEN,
			sizeof(mgmt->da));
	moal_memcpy_ext(priv->phandle, mgmt->sa, priv->current_addr, ETH_ALEN,
			sizeof(mgmt->sa));
	moal_memcpy_ext(priv->phandle, mgmt->bssid, priv->cfg_bssid, ETH_ALEN,
			sizeof(mgmt->bssid));

	mgmt->frame_control = cpu_to_le16(IEEE80211_FTYPE_MGMT) |
			      cpu_to_le16(IEEE80211_STYPE_ACTION);
	/* add address 4*/
	pos = skb_put(skb, ETH_ALEN);

	capability = 0x2421;

	switch (action_code) {
	case WLAN_PUB_ACTION_TDLS_DISCOVER_RES:
		if (woal_tdls_get_ies(priv, peer, tdls_ies,
				      TDLS_IE_FLAGS_EXTCAP |
					      TDLS_IE_FLAGS_SUPP_CS_IE)) {
			PRINTM(MERROR, "%s: woal_tdls_get_ies failed \n",
			       __func__);
			if (tdls_ies)
				kfree(tdls_ies);
			LEAVE();
			return -EFAULT;
		}
		skb_put(skb, 1 + sizeof(mgmt->u.action.u.tdls_discover_resp));
		mgmt->u.action.category = WLAN_CATEGORY_PUBLIC;
		mgmt->u.action.u.tdls_discover_resp.action_code =
			WLAN_PUB_ACTION_TDLS_DISCOVER_RES;
		mgmt->u.action.u.tdls_discover_resp.dialog_token = dialog_token;
		mgmt->u.action.u.tdls_discover_resp.capability =
			cpu_to_le16(capability);
		/* move back for addr4 */
		memmove(pos + ETH_ALEN, &mgmt->u.action,
			1 + sizeof(mgmt->u.action.u.tdls_discover_resp));
		/** init address 4 */
		moal_memcpy_ext(priv->phandle, pos, addr, ETH_ALEN, ETH_ALEN);

		woal_add_supported_rates_ie(priv, skb, band);
		woal_add_ext_supported_rates_ie(priv, skb, band);
		woal_tdls_add_ext_capab(priv, skb, tdls_ies);
		/* supported chanel ie*/
		if (tdls_ies->supp_chan[0] == SUPPORTED_CHANNELS) {
			pSupp_chan = (void *)skb_put(
				skb, sizeof(IEEEtypes_Header_t) +
					     tdls_ies->supp_chan[1]);
			memset(pSupp_chan, 0,
			       sizeof(IEEEtypes_Header_t) +
				       tdls_ies->supp_chan[1]);
			moal_memcpy_ext(priv->phandle, pSupp_chan,
					tdls_ies->supp_chan,
					sizeof(IEEEtypes_Header_t) +
						tdls_ies->supp_chan[1],
					sizeof(IEEEtypes_Header_t) +
						tdls_ies->supp_chan[1]);
		}
		/* supported regulatory class ie*/
		if (tdls_ies->regulatory_class[0] == REGULATORY_CLASS) {
			pRegulatory_class = (void *)skb_put(
				skb, sizeof(IEEEtypes_Header_t) +
					     tdls_ies->regulatory_class[1]);
			memset(pRegulatory_class, 0,
			       sizeof(IEEEtypes_Header_t) +
				       tdls_ies->regulatory_class[1]);
			moal_memcpy_ext(priv->phandle, pRegulatory_class,
					tdls_ies->regulatory_class,
					sizeof(IEEEtypes_Header_t) +
						tdls_ies->regulatory_class[1],
					sizeof(IEEEtypes_Header_t) +
						tdls_ies->regulatory_class[1]);
		}

		break;
	default:
		ret = -EINVAL;
		break;
	}
	if (tdls_ies)
		kfree(tdls_ies);
	return ret;
}

/**
 * @brief woal add tdls link identifier ie
 *
 * @param skb                   skb buffer
 * @param src_addr              source address
 * @param peer                  peer address
 * @param bssid                 AP's bssid
 *
 * @return                      NA
 */
static void woal_tdls_add_link_ie(moal_private *priv, struct sk_buff *skb,
				  const u8 *src_addr, const u8 *peer, u8 *bssid)
{
	struct ieee80211_tdls_lnkie *lnkid;

	lnkid = (void *)skb_put(skb, sizeof(struct ieee80211_tdls_lnkie));

	lnkid->ie_type = WLAN_EID_LINK_ID;
	lnkid->ie_len = sizeof(struct ieee80211_tdls_lnkie) - 2;

	moal_memcpy_ext(priv->phandle, lnkid->bssid, bssid, ETH_ALEN,
			sizeof(lnkid->bssid));
	moal_memcpy_ext(priv->phandle, lnkid->init_sta, src_addr, ETH_ALEN,
			sizeof(lnkid->init_sta));
	moal_memcpy_ext(priv->phandle, lnkid->resp_sta, peer, ETH_ALEN,
			sizeof(lnkid->resp_sta));
}

/**
 * @brief woal send tdls action frame
 *
 * @param wiphy                 A pointer to wiphy structure
 * @param dev                   A pointer to net_device structure
 * @param peer                  A pointer to peer mac
 * @param action_code           tdls action code
 * @param dialog_token          dialog_token
 * @param status_code           status_code
 * @param extra_ies              A pointer to extra ie buffer
 * @param extra_ies_len          etra ie len
 * @param skb                   skb buffer
 *
 * @return                      0 -- success, otherwise fail
 */
static int woal_send_tdls_action_frame(struct wiphy *wiphy,
				       struct net_device *dev, const t_u8 *peer,
				       u8 action_code, t_u8 dialog_token,
				       t_u16 status_code, const t_u8 *extra_ies,
				       size_t extra_ies_len)
{
	moal_private *priv = (moal_private *)woal_get_netdev_priv(dev);
	pmlan_buffer pmbuf = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;
	struct sk_buff *skb = NULL;
	t_u8 *pos;
	t_u32 pkt_type;
	t_u32 tx_control;
	t_u16 pkt_len;
	t_u16 packet_len;
	int ret = 0;
	t_u16 buf_size = 0;

	ENTER();

#define HEADER_SIZE 8 /* pkt_type + tx_control */

	woal_secure_add(&buf_size, MLAN_MIN_DATA_HEADER_LEN, &buf_size,
			TYPE_UINT32);
	woal_secure_add(&buf_size, HEADER_SIZE, &buf_size, TYPE_UINT32);
	woal_secure_add(&buf_size, sizeof(pkt_len), &buf_size, TYPE_UINT32);
	woal_secure_add(&buf_size,
			max(sizeof(struct ieee80211_mgmt),
			    sizeof(struct ieee80211_tdls_data)),
			&buf_size, TYPE_UINT32);
	/* supported rates */
	woal_secure_add(&buf_size, 50, &buf_size, TYPE_UINT32);
	/* ext capab */
	woal_secure_add(&buf_size, sizeof(IEEEtypes_ExtCap_t), &buf_size,
			TYPE_UINT32);
	woal_secure_add(&buf_size, extra_ies_len, &buf_size, TYPE_UINT32);
	woal_secure_add(&buf_size, sizeof(IEEEtypes_tdls_linkie), &buf_size,
			TYPE_UINT32);
	pmbuf = woal_alloc_mlan_buffer(priv->phandle, buf_size);

	if (!pmbuf) {
		PRINTM(MERROR, "Fail to allocate mlan_buffer\n");
		ret = -ENOMEM;
		goto done;
	}

	skb = (struct sk_buff *)pmbuf->pdesc;

	skb_put(skb, MLAN_MIN_DATA_HEADER_LEN);

	pos = skb_put(skb, HEADER_SIZE + sizeof(pkt_len));
	pkt_type = MRVL_PKT_TYPE_MGMT_FRAME;
	tx_control = 0;
	memset(pos, 0, HEADER_SIZE + sizeof(pkt_len));
	moal_memcpy_ext(priv->phandle, pos, &pkt_type, sizeof(pkt_type),
			sizeof(pkt_type));
	moal_memcpy_ext(priv->phandle, pos + sizeof(pkt_type), &tx_control,
			sizeof(tx_control), sizeof(tx_control));

	woal_construct_tdls_action_frame(priv, peer, action_code, dialog_token,
					 status_code, skb);

	if (extra_ies_len)
		moal_memcpy_ext(priv->phandle, skb_put(skb, extra_ies_len),
				extra_ies, extra_ies_len, extra_ies_len);

	/* the TDLS link IE is always added last */
	/* we are the responder */
	woal_tdls_add_link_ie(priv, skb, peer, priv->current_addr,
			      priv->cfg_bssid);

	/*
	 * According to 802.11z: Setup req/resp are sent in AC_BK, otherwise
	 * we should default to AC_VI.
	 */
	skb_set_queue_mapping(skb, WMM_AC_VI);
	skb->priority = 5;

	pmbuf->data_offset = MLAN_MIN_DATA_HEADER_LEN;
	pmbuf->data_len = skb->len - pmbuf->data_offset;
	pmbuf->priority = skb->priority;
	pmbuf->buf_type = MLAN_BUF_TYPE_RAW_DATA;
	pmbuf->bss_index = priv->bss_index;

	pkt_len = pmbuf->data_len - HEADER_SIZE - sizeof(pkt_len);
	packet_len = woal_cpu_to_le16(pkt_len);
	moal_memcpy_ext(priv->phandle,
			pmbuf->pbuf + pmbuf->data_offset + HEADER_SIZE,
			&packet_len, sizeof(packet_len), sizeof(packet_len));

	DBG_HEXDUMP(MDAT_D, "TDLS action:", pmbuf->pbuf + pmbuf->data_offset,
		    pmbuf->data_len);

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
 * @brief woal send tdls data frame
 *
 * @param wiphy                 A pointer to wiphy structure
 * @param dev                   A pointer to net_device structure
 * @param peer                  A pointer to peer mac
 * @param action_code           tdls action code
 * @param dialog_token          dialog_token
 * @param status_code           status_code
 * @param extra_ies              A pointer to extra ie buffer
 * @param extra_ies_len          etra ie len
 * @param skb                   skb buffer
 *
 * @return                      0 -- success, otherwise fail
 */
static int woal_send_tdls_data_frame(struct wiphy *wiphy,
				     struct net_device *dev, const t_u8 *peer,
				     u8 action_code, t_u8 dialog_token,
				     t_u16 status_code, const t_u8 *extra_ies,
				     size_t extra_ies_len)
{
	moal_private *priv = (moal_private *)woal_get_netdev_priv(dev);
	pmlan_buffer pmbuf = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;
	struct sk_buff *skb = NULL;
	int ret = 0;
#if CFG80211_VERSION_CODE > KERNEL_VERSION(2, 6, 29)
	t_u32 index = 0;
#endif

	ENTER();

	skb = dev_alloc_skb(
		priv->extra_tx_head_len + MLAN_MIN_DATA_HEADER_LEN +
		sizeof(mlan_buffer) +
		max(sizeof(struct ieee80211_mgmt),
		    sizeof(struct ieee80211_tdls_data)) +
		50 + /* supported rates */
		sizeof(IEEEtypes_ExtCap_t) + /* ext capab */
		3 + /* Qos Info */
		sizeof(IEEEtypes_WmmParameter_t) + /*wmm ie*/
		sizeof(IEEEtypes_HTCap_t) + sizeof(IEEEtypes_2040BSSCo_t) +
		sizeof(IEEEtypes_HTInfo_t) + sizeof(IEEEtypes_VHTCap_t) +
		sizeof(IEEEtypes_VHTOprat_t) + sizeof(IEEEtypes_AID_t) +
		sizeof(IEEEtypes_HECap_t) + sizeof(IEEEtypes_HeOp_t) +
		extra_ies_len + sizeof(IEEEtypes_tdls_linkie));
	if (!skb)
		return -ENOMEM;

	skb_reserve(skb, MLAN_MIN_DATA_HEADER_LEN + sizeof(mlan_buffer) +
				 priv->extra_tx_head_len);

	woal_construct_tdls_data_frame(priv, peer, action_code, dialog_token,
				       status_code, skb);

	if (extra_ies_len)
		moal_memcpy_ext(priv->phandle, skb_put(skb, extra_ies_len),
				extra_ies, extra_ies_len, extra_ies_len);

	/* the TDLS link IE is always added last */
	switch (action_code) {
	case WLAN_TDLS_SETUP_REQUEST:
	case WLAN_TDLS_SETUP_CONFIRM:
	case WLAN_TDLS_TEARDOWN:
	case WLAN_TDLS_DISCOVERY_REQUEST:
		/* we are the initiator */
		woal_tdls_add_link_ie(priv, skb, priv->current_addr, peer,
				      priv->cfg_bssid);
		break;
	case WLAN_TDLS_SETUP_RESPONSE:
		/* we are the responder */
		woal_tdls_add_link_ie(priv, skb, peer, priv->current_addr,
				      priv->cfg_bssid);
		break;
	default:
		ret = -ENOTSUPP;
		goto fail;
	}

	/*
	 * According to 802.11z: Setup req/resp are sent in AC_BK, otherwise
	 * we should default to AC_VI.
	 */
	switch (action_code) {
	case WLAN_TDLS_SETUP_REQUEST:
	case WLAN_TDLS_SETUP_RESPONSE:
		skb_set_queue_mapping(skb, WMM_AC_BK);
		skb->priority = 2;
		break;
	default:
		skb_set_queue_mapping(skb, WMM_AC_VI);
		skb->priority = 5;
		break;
	}

	pmbuf = (mlan_buffer *)skb->head;
	memset((t_u8 *)pmbuf, 0, sizeof(mlan_buffer));
	pmbuf->bss_index = priv->bss_index;
	pmbuf->pdesc = skb;
	pmbuf->pbuf = skb->head + sizeof(mlan_buffer);

	pmbuf->data_offset = skb->data - (skb->head + sizeof(mlan_buffer));
	pmbuf->data_len = skb->len;
	pmbuf->priority = skb->priority;
	pmbuf->buf_type = MLAN_BUF_TYPE_DATA;

	DBG_HEXDUMP(MDAT_D, "TDLS data:", pmbuf->pbuf + pmbuf->data_offset,
		    pmbuf->data_len);

#if CFG80211_VERSION_CODE > KERNEL_VERSION(2, 6, 29)
	index = skb_get_queue_mapping(skb);
#endif
	status = mlan_send_packet(priv->phandle->pmlan_adapter, pmbuf);

	switch (status) {
	case MLAN_STATUS_PENDING:
		atomic_inc(&priv->phandle->tx_pending);
#if CFG80211_VERSION_CODE > KERNEL_VERSION(2, 6, 29)
		atomic_inc(&priv->wmm_tx_pending[index]);
#endif
		queue_work(priv->phandle->workqueue, &priv->phandle->main_work);
		/*delay 10 ms to guarantee the teardown/confirm frame can be
		 * sent out before disalbe/enable tdls link if we don't delay
		 * and return immediately, wpa_supplicant will call
		 * disalbe/enable tdls link this may cause tdls link
		 * disabled/enabled before teardown/confirm frame sent out */
		if (action_code == WLAN_TDLS_TEARDOWN ||
		    action_code == WLAN_TDLS_SETUP_CONFIRM)
			woal_sched_timeout(10);
		break;
	case MLAN_STATUS_SUCCESS:
		dev_kfree_skb(skb);
		break;
	case MLAN_STATUS_FAILURE:
	default:
		dev_kfree_skb(skb);
		ret = -ENOTSUPP;
		break;
	}

	LEAVE();
	return ret;
fail:
	dev_kfree_skb(skb);
	return ret;
}

/**
 * @brief Tx TDLS packet
 *
 * @param wiphy                 A pointer to wiphy structure
 * @param dev                   A pointer to net_device structure
 * @param peer                  A pointer to peer mac
 * @param action_code           tdls action code
 * @param dialog_token          dialog_token
 * @param status_code           status_code
 * @param peer_capability       peer capability
 * @param initiator             initiator
 * @param extra_ies              A pointer to extra ie buffer
 * @param extra_ies_len          etra ie len
 *
 * @return                      0 -- success, otherwise fail
 */
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(6, 5, 0)
int woal_cfg80211_tdls_mgmt(struct wiphy *wiphy, struct net_device *dev,
			    const t_u8 *peer, int link_id, u8 action_code,
			    t_u8 dialog_token, t_u16 status_code,
			    t_u32 peer_capability, bool initiator,
			    const t_u8 *extra_ies, size_t extra_ies_len)
#else
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 17, 0)
int woal_cfg80211_tdls_mgmt(struct wiphy *wiphy, struct net_device *dev,
			    const t_u8 *peer, u8 action_code, t_u8 dialog_token,
			    t_u16 status_code, t_u32 peer_capability,
			    bool initiator, const t_u8 *extra_ies,
			    size_t extra_ies_len)
#else
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 15, 0)
/**
 * @brief Tx TDLS packet
 *
 * @param wiphy                 A pointer to wiphy structure
 * @param dev                   A pointer to net_device structure
 * @param peer                  A pointer to peer mac
 * @param action_code           tdls action code
 * @param dialog_token          dialog_token
 * @param status_code           status_code
 * @param peer_capability       peer capability
 * @param extra_ies              A pointer to extra ie buffer
 * @param extra_ies_len          etra ie len
 *
 * @return                      0 -- success, otherwise fail
 */
int woal_cfg80211_tdls_mgmt(struct wiphy *wiphy, struct net_device *dev,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 16, 0)
			    const t_u8 *peer,
#else
			    t_u8 *peer,
#endif
			    u8 action_code, t_u8 dialog_token,
			    t_u16 status_code, t_u32 peer_capability,
			    const t_u8 *extra_ies, size_t extra_ies_len)
#else
/**
 * @brief Tx TDLS packet
 *
 * @param wiphy                 A pointer to wiphy structure
 * @param dev                   A pointer to net_device structure
 * @param peer                  A pointer to peer mac
 * @param action_code           tdls action code
 * @param dialog_token          dialog_token
 * @param status_code           status_code
 * @param extra_ies              A pointer to extra ie buffer
 * @param extra_ies_len          etra ie len
 *
 * @return                      0 -- success, otherwise fail
 */
int woal_cfg80211_tdls_mgmt(struct wiphy *wiphy, struct net_device *dev,
			    t_u8 *peer, u8 action_code, t_u8 dialog_token,
			    t_u16 status_code, const t_u8 *extra_ies,
			    size_t extra_ies_len)
#endif
#endif
#endif
{
	moal_private *priv = (moal_private *)woal_get_netdev_priv(dev);
	int ret = 0;
	mlan_bss_info bss_info;

	ENTER();

	if (!(wiphy->flags & WIPHY_FLAG_SUPPORTS_TDLS)) {
		LEAVE();
		return -ENOTSUPP;
	}
	/* make sure we are not in uAP mode and Go mode */
	if (priv->bss_type != MLAN_BSS_TYPE_STA) {
		LEAVE();
		return -ENOTSUPP;
	}

	/* check if AP prohited TDLS */
	memset(&bss_info, 0, sizeof(bss_info));
	if (MLAN_STATUS_SUCCESS !=
	    woal_get_bss_info(priv, MOAL_IOCTL_WAIT, &bss_info)) {
		PRINTM(MERROR, "WLAN, %s: Failed to get bss info.\n", __func__);
	}
	if (IS_EXTCAP_TDLS_PROHIBITED(bss_info.ext_cap)) {
		PRINTM(MMSG, "TDLS is prohibited by AP\n");
		LEAVE();
		return -ENOTSUPP;
	}

	switch (action_code) {
	case TDLS_SETUP_REQUEST:
		woal_add_tdls_peer(priv, (const t_u8 *)peer);
		PRINTM(MMSG,
		       "wlan: Send TDLS Setup Request to " MACSTR
		       " status_code=%d\n",
		       MAC2STR(peer), status_code);
		ret = woal_send_tdls_data_frame(wiphy, dev, (const t_u8 *)peer,
						action_code, dialog_token,
						status_code, extra_ies,
						extra_ies_len);
		break;
	case TDLS_SETUP_RESPONSE:
		PRINTM(MMSG,
		       "wlan: Send TDLS Setup Response to " MACSTR
		       " status_code=%d\n",
		       MAC2STR(peer), status_code);
		ret = woal_send_tdls_data_frame(wiphy, dev, (const t_u8 *)peer,
						action_code, dialog_token,
						status_code, extra_ies,
						extra_ies_len);
		break;
	case TDLS_SETUP_CONFIRM:
		PRINTM(MMSG,
		       "wlan: Send TDLS Confirm to " MACSTR " status_code=%d\n",
		       MAC2STR(peer), status_code);
		ret = woal_send_tdls_data_frame(wiphy, dev, (const t_u8 *)peer,
						action_code, dialog_token,
						status_code, extra_ies,
						extra_ies_len);
		break;
	case TDLS_TEARDOWN:
		PRINTM(MMSG, "wlan: Send TDLS Tear down to " MACSTR "\n",
		       MAC2STR(peer));
		ret = woal_send_tdls_data_frame(wiphy, dev, (const t_u8 *)peer,
						action_code, dialog_token,
						status_code, extra_ies,
						extra_ies_len);
		break;
	case TDLS_DISCOVERY_REQUEST:
		PRINTM(MMSG,
		       "wlan: Send TDLS Discovery Request to " MACSTR "\n",
		       MAC2STR(peer));
		ret = woal_send_tdls_data_frame(wiphy, dev, (const t_u8 *)peer,
						action_code, dialog_token,
						status_code, extra_ies,
						extra_ies_len);
		break;
	case TDLS_DISCOVERY_RESPONSE:
		PRINTM(MMSG,
		       "wlan: Send TDLS Discovery Response to " MACSTR "\n",
		       MAC2STR(peer));
		ret = woal_send_tdls_action_frame(
			wiphy, dev, (const t_u8 *)peer, action_code,
			dialog_token, status_code, extra_ies, extra_ies_len);
		break;
	default:
		break;
	}

	LEAVE();
	return ret;
}

/**
 * @brief cfg80211_tdls_oper handler
 *
 * @param wiphy                 A pointer to wiphy structure
 * @param dev                   A pointer to net_device structure
 * @param peer                  tdls peer mac
 * @param oper                  tdls operation code
 *
 * @return                  	0 -- success, otherwise fail
 */
int woal_cfg80211_tdls_oper(struct wiphy *wiphy, struct net_device *dev,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 16, 0)
			    const u8 *peer,
#else
			    u8 *peer,
#endif
			    enum nl80211_tdls_operation oper)
{
	moal_private *priv = (moal_private *)woal_get_netdev_priv(dev);
	t_u8 action;
	int ret = 0;
	t_u8 event_buf[32];
	int custom_len = 0;

	ENTER();

	if (!(wiphy->flags & WIPHY_FLAG_SUPPORTS_TDLS))
		return -ENOTSUPP;

	if (!(wiphy->flags & WIPHY_FLAG_TDLS_EXTERNAL_SETUP))
		return -ENOTSUPP;
	/* make sure we are in managed mode, and associated */
	if (priv->bss_type != MLAN_BSS_TYPE_STA)
		return -ENOTSUPP;

	PRINTM(MIOCTL, "wlan: TDLS peer=" MACSTR ", oper=%d\n", MAC2STR(peer),
	       oper);
	switch (oper) {
	case NL80211_TDLS_ENABLE_LINK:
		/*Configure TDLS link first*/
		woal_tdls_oper(priv, (const u8 *)peer, WLAN_TDLS_CONFIG_LINK);
		woal_updata_peer_status(priv, (const t_u8 *)peer,
					TDLS_SETUP_COMPLETE);
		PRINTM(MMSG, "wlan: TDLS_ENABLE_LINK: peer=" MACSTR "\n",
		       MAC2STR(peer));
		action = WLAN_TDLS_ENABLE_LINK;
		memset(event_buf, 0, sizeof(event_buf));
		custom_len = strlen(CUS_EVT_TDLS_CONNECTED);
		moal_memcpy_ext(priv->phandle, event_buf,
				CUS_EVT_TDLS_CONNECTED, custom_len,
				sizeof(event_buf));
		moal_memcpy_ext(priv->phandle, event_buf + custom_len, peer,
				ETH_ALEN, sizeof(event_buf) - custom_len);
		woal_broadcast_event(priv, event_buf, custom_len + ETH_ALEN);
		break;
	case NL80211_TDLS_DISABLE_LINK:
		woal_updata_peer_status(priv, (const t_u8 *)peer,
					TDLS_NOT_SETUP);
		PRINTM(MMSG, "wlan: TDLS_DISABLE_LINK: peer=" MACSTR "\n",
		       MAC2STR(peer));
		action = WLAN_TDLS_DISABLE_LINK;
		memset(event_buf, 0, sizeof(event_buf));
		custom_len = strlen(CUS_EVT_TDLS_TEARDOWN);
		moal_memcpy_ext(priv->phandle, event_buf, CUS_EVT_TDLS_TEARDOWN,
				custom_len, sizeof(event_buf));
		moal_memcpy_ext(priv->phandle, event_buf + custom_len, peer,
				ETH_ALEN, sizeof(event_buf) - custom_len);
		woal_broadcast_event(priv, event_buf, custom_len + ETH_ALEN);
		break;
	case NL80211_TDLS_TEARDOWN:
	case NL80211_TDLS_SETUP:
	case NL80211_TDLS_DISCOVERY_REQ:
		return 0;

	default:
		return -ENOTSUPP;
	}
	ret = woal_tdls_oper(priv, (const u8 *)peer, action);

	LEAVE();
	return ret;
}

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 19, 0)
/**
 * @brief tdls channel switch
 *
 * @param wiphy                 A pointer to wiphy structure
 * @param dev                   A pointer to net_device structure
 * @param addr                  A pointer to peer addr
 * @param oper_class            The operating class
 * @param chandef               A pointer to cfg80211_chan_def structure
 *
 * @return                      0 -- success, otherwise fail
 */
static int woal_cfg80211_tdls_channel_switch(struct wiphy *wiphy,
					     struct net_device *dev,
					     const u8 *addr, u8 oper_class,
					     struct cfg80211_chan_def *chandef)
{
	moal_private *priv = (moal_private *)woal_get_netdev_priv(dev);
	mlan_ioctl_req *ioctl_req = NULL;
	mlan_ds_misc_cfg *misc = NULL;
	mlan_ds_misc_tdls_config *tdls_data = NULL;
	tdls_all_config *tdls_all_cfg = NULL;
	int ret = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;
	mlan_bss_info bss_info;

	ENTER();

	/* check if AP prohited TDLS channel switch */
	memset(&bss_info, 0, sizeof(bss_info));
	if (MLAN_STATUS_SUCCESS !=
	    woal_get_bss_info(priv, MOAL_IOCTL_WAIT, &bss_info)) {
		PRINTM(MERROR, "WLAN, %s: Failed to get bss info.\n", __func__);
	}
	if (IS_EXTCAP_TDLS_CHLSWITCHPROHIB(bss_info.ext_cap)) {
		PRINTM(MMSG, "TDLS Channel Switching is prohibited by AP\n");
		LEAVE();
		return -ENOTSUPP;
	}

	ioctl_req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (ioctl_req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	misc = (mlan_ds_misc_cfg *)ioctl_req->pbuf;
	misc->sub_command = MLAN_OID_MISC_TDLS_OPER;
	ioctl_req->req_id = MLAN_IOCTL_MISC_CFG;
	ioctl_req->action = MLAN_ACT_SET;

	tdls_data = &misc->param.tdls_config;
	tdls_data->tdls_action = WLAN_TDLS_INIT_CHAN_SWITCH;

	tdls_all_cfg = (tdls_all_config *)tdls_data->tdls_data;
	moal_memcpy_ext(priv->phandle,
			tdls_all_cfg->u.tdls_chan_switch.peer_mac_addr, addr,
			ETH_ALEN,
			sizeof(tdls_all_cfg->u.tdls_chan_switch.peer_mac_addr));
	tdls_all_cfg->u.tdls_chan_switch.primary_channel =
		chandef->chan->hw_value;
	tdls_all_cfg->u.tdls_chan_switch.band = chandef->chan->band;
	tdls_all_cfg->u.tdls_chan_switch.regulatory_class = oper_class;

	status = woal_request_ioctl(priv, ioctl_req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		PRINTM(MERROR, "TDLS channel switch request failed.\n");
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
 * @brief tdls cancel channel switch
 *
 * @param wiphy                 A pointer to wiphy structure
 * @param dev                   A pointer to net_device structure
 * @param addr                  A pointer to peer addr
 *
 */
void woal_cfg80211_tdls_cancel_channel_switch(struct wiphy *wiphy,
					      struct net_device *dev,
					      const u8 *addr)
{
	moal_private *priv = (moal_private *)woal_get_netdev_priv(dev);
	mlan_ioctl_req *ioctl_req = NULL;
	mlan_ds_misc_cfg *misc = NULL;
	mlan_ds_misc_tdls_config *tdls_data = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	if (!priv || !priv->phandle) {
		PRINTM(MERROR, "priv or handle is null\n");
		status = MLAN_STATUS_FAILURE;
		goto done;
	}

	ioctl_req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (ioctl_req == NULL) {
		status = MLAN_STATUS_FAILURE;
		goto done;
	}

	misc = (mlan_ds_misc_cfg *)ioctl_req->pbuf;
	misc->sub_command = MLAN_OID_MISC_TDLS_CONFIG;
	ioctl_req->req_id = MLAN_IOCTL_MISC_CFG;
	ioctl_req->action = MLAN_ACT_SET;

	tdls_data = &misc->param.tdls_config;
	tdls_data->tdls_action = WLAN_TDLS_STOP_CHAN_SWITCH;
	moal_memcpy_ext(priv->phandle, tdls_data->tdls_data, addr, ETH_ALEN,
			sizeof(tdls_data->tdls_data));

	status = woal_request_ioctl(priv, ioctl_req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS)
		goto done;

	PRINTM(MIOCTL, "Tdls channel switch stop!\n");
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(ioctl_req);

	LEAVE();
}
#endif
#endif
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 2, 0)
/**
 * @brief change station info
 *
 * @param wiphy                 A pointer to wiphy structure
 * @param dev                   A pointer to net_device structure
 * @param mac                   A pointer to peer mac
 * @param params                station parameters
 *
 * @return                      0 -- success, otherwise fail
 */
static int woal_cfg80211_change_station(struct wiphy *wiphy,
					struct net_device *dev,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 16, 0)
					const u8 *mac,
#else
					u8 *mac,
#endif
					struct station_parameters *params)
{
	int ret = 0;
#ifdef UAP_SUPPORT
	moal_private *priv = (moal_private *)woal_get_netdev_priv(dev);
	moal_private *vlan_priv = NULL;
	station_node *sta_node = NULL;
	int i = 0;
#endif

	ENTER();
#ifdef UAP_SUPPORT
	/** Bind the station to uap virtual interface and
	save the station info in moal_private */
	if (params->vlan) {
		if (params->vlan->ieee80211_ptr &&
		    params->vlan->ieee80211_ptr->iftype ==
			    NL80211_IFTYPE_AP_VLAN) {
			vlan_priv = (moal_private *)woal_get_netdev_priv(
				params->vlan);
			for (i = 0; i < MAX_STA_COUNT; i++) {
				sta_node = priv->vlan_sta_list[i];
				if (sta_node &&
				    !moal_memcmp(priv->phandle,
						 sta_node->peer_mac, mac,
						 MLAN_MAC_ADDR_LENGTH)) {
					PRINTM(MCMND,
					       "wlan: Easymesh change station aid=%d\n",
					       sta_node->aid);
					sta_node->netdev = params->vlan;
					sta_node->is_valid = MTRUE;
					vlan_priv->vlan_sta_ptr = sta_node;
					break;
				}
			}
		}
	}
#endif
	/**do nothing*/

	LEAVE();
	return ret;
}
#endif

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 2, 0)
#ifdef UAP_SUPPORT
/**
 * @brief add station
 *
 * @param wiphy                 A pointer to wiphy structure
 * @param dev                   A pointer to net_device structure
 * @param mac                  A pointer to peer mac
 * @param params           	station parameters
 *
 * @return                  	0 -- success, otherwise fail
 */
static int woal_cfg80211_add_station(struct wiphy *wiphy,
				     struct net_device *dev,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 16, 0)
				     const u8 *mac,
#else
				     u8 *mac,
#endif
				     struct station_parameters *params)
{
	moal_private *priv = (moal_private *)woal_get_netdev_priv(dev);
	int ret = 0;
	station_node *sta_node = NULL;

	ENTER();
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
#ifdef UAP_SUPPORT
	if (moal_extflg_isset(priv->phandle, EXT_HOST_MLME) &&
	    (priv->bss_role == MLAN_BSS_ROLE_UAP)) {
		sta_node = kmalloc(sizeof(station_node), GFP_KERNEL);
		if (!sta_node) {
			PRINTM(MERROR,
			       "Failed to alloc memory for station node\n");
			LEAVE();
			return -ENOMEM;
		}
		memset(sta_node, 0, sizeof(*sta_node));
		moal_memcpy_ext(priv->phandle, sta_node->peer_mac, mac,
				MLAN_MAC_ADDR_LENGTH, ETH_ALEN);
		sta_node->netdev = dev;
		sta_node->aid = params->aid;
		sta_node->is_valid = MFALSE;
		/** AID should start from 1 to MAX_STA_COUNT */
		priv->vlan_sta_list[(params->aid - 1) % MAX_STA_COUNT] =
			sta_node;
		ret = woal_cfg80211_uap_add_station(wiphy, dev, mac, params);
		LEAVE();
		return ret;
	}
#endif
#endif
	if (!(params->sta_flags_set & MBIT(NL80211_STA_FLAG_TDLS_PEER)))
		goto done;
	/* make sure we are in connected mode */
	if ((priv->bss_type != MLAN_BSS_TYPE_STA) ||
	    (priv->media_connected == MFALSE)) {
		ret = -ENOTSUPP;
		goto done;
	}
	PRINTM(MMSG, "wlan: TDLS add peer station, address =" MACSTR "\n",
	       MAC2STR(mac));
	ret = woal_tdls_oper(priv, (const u8 *)mac, WLAN_TDLS_CREATE_LINK);
done:
	LEAVE();
	return ret;
}
#endif
#endif

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
/**
 * @brief Update ft ie for Fast BSS Transition
 *
 * @param wiphy           A pointer to wiphy structure
 * @param dev             A pointer to net_device structure
 * @param ftie           A pointer to cfg80211_update_ft_ies_params structure
 *
 * @return                0 success , other failure
 */
int woal_cfg80211_update_ft_ies(struct wiphy *wiphy, struct net_device *dev,
				struct cfg80211_update_ft_ies_params *ftie)
{
	moal_private *priv = (moal_private *)woal_get_netdev_priv(dev);
	const IEEEtypes_MobilityDomain_t *md_ie = NULL;
	int ret = 0;
	mlan_ds_misc_assoc_rsp *assoc_rsp = NULL;
	IEEEtypes_AssocRsp_t *passoc_rsp = NULL;
	mlan_bss_info bss_info;

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 12, 0)
	struct cfg80211_roam_info roam_info = {};
#endif

	ENTER();

	if (!ftie) {
		LEAVE();
		return ret;
	}
	assoc_rsp = kmalloc(sizeof(mlan_ds_misc_assoc_rsp), GFP_KERNEL);
	if (!assoc_rsp) {
		LEAVE();
		return ret;
	}
#ifdef MLAN_64BIT
	PRINTM(MINFO, "==>woal_cfg80211_update_ft_ies %lx \n", ftie->ie_len);
#else
	PRINTM(MINFO, "==>woal_cfg80211_update_ft_ies %x \n", ftie->ie_len);
#endif
	md_ie = (const IEEEtypes_MobilityDomain_t *)woal_parse_ie_tlv(
		ftie->ie, (int)ftie->ie_len, MOBILITY_DOMAIN);
	if (!md_ie) {
		PRINTM(MERROR, "No Mobility domain IE\n");
		kfree(assoc_rsp);
		LEAVE();
		return ret;
	}
	priv->ft_cap = md_ie->ft_cap;
	if (priv->ft_ie_len) {
		priv->pre_ft_ie_len = priv->ft_ie_len;
		moal_memcpy_ext(priv->phandle, priv->pre_ft_ie, priv->ft_ie,
				priv->ft_ie_len, MAX_IE_SIZE);
	}
	memset(priv->ft_ie, 0, MAX_IE_SIZE);
	moal_memcpy_ext(priv->phandle, priv->ft_ie, ftie->ie,
			MIN(ftie->ie_len, MAX_IE_SIZE), sizeof(priv->ft_ie));
	priv->ft_ie_len = ftie->ie_len;
	priv->ft_md = ftie->md;

	if (!priv->ft_pre_connect) {
		kfree(assoc_rsp);
		LEAVE();
		return ret;
	}
	/* check if is different AP */
	if (!memcmp(&priv->target_ap_bssid, priv->cfg_bssid,
		    MLAN_MAC_ADDR_LENGTH)) {
		PRINTM(MMSG, "This is the same AP, no Fast bss transition\n");
		kfree(assoc_rsp);
		priv->ft_pre_connect = MFALSE;
		priv->ft_ie_len = 0;
		LEAVE();
		return 0;
	}

	/* start fast BSS transition to target AP */
	priv->assoc_status = 0;

	moal_memcpy_ext(priv->phandle, (void *)priv->conn_bssid,
			&priv->target_ap_bssid, MLAN_MAC_ADDR_LENGTH,
			sizeof(priv->conn_bssid));
	priv->sme_current.bssid = priv->conn_bssid;

	memset(assoc_rsp, 0, sizeof(mlan_ds_misc_assoc_rsp));
	ret = woal_cfg80211_assoc(priv, (void *)&priv->sme_current,
				  MOAL_IOCTL_WAIT, assoc_rsp);

	if ((priv->ft_cap & MBIT(0)) || priv->ft_roaming_triggered_by_driver) {
		if (!ret) {
			woal_inform_bss_from_scan_result(priv, NULL,
							 MOAL_IOCTL_WAIT);
			passoc_rsp = (IEEEtypes_AssocRsp_t *)
					     assoc_rsp->assoc_resp_buf;
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 12, 0)
#if ((CFG80211_VERSION_CODE >= KERNEL_VERSION(6, 0, 0)) ||                     \
     (defined(ANDROID_SDK_VERSION) && ANDROID_SDK_VERSION >= 31))
			roam_info.links[0].bssid = priv->cfg_bssid;
#else
			roam_info.bssid = priv->cfg_bssid;
#endif
			roam_info.req_ie = priv->sme_current.ie;
			roam_info.req_ie_len = priv->sme_current.ie_len;
			roam_info.resp_ie = passoc_rsp->ie_buffer;
			roam_info.resp_ie_len = assoc_rsp->assoc_resp_len -
						ASSOC_RESP_FIXED_SIZE;
			cfg80211_roamed(priv->netdev, &roam_info, GFP_KERNEL);
#else
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 0, 0)
			cfg80211_roamed(priv->netdev, NULL, priv->cfg_bssid,
					priv->sme_current.ie,
					priv->sme_current.ie_len,
					passoc_rsp->ie_buffer,
					assoc_rsp->assoc_resp_len -
						ASSOC_RESP_FIXED_SIZE,
					GFP_KERNEL);
#else
			cfg80211_roamed(priv->netdev, priv->cfg_bssid,
					priv->sme_current.ie,
					priv->sme_current.ie_len,
					passoc_rsp->ie_buffer,
					assoc_rsp->assoc_resp_len -
						ASSOC_RESP_FIXED_SIZE,
					GFP_KERNEL);
#endif
#endif
			PRINTM(MMSG,
			       "Fast BSS transition to bssid " MACSTR
			       " successfully\n",
			       MAC2STR(priv->cfg_bssid));
		} else {
			PRINTM(MMSG,
			       "Fast BSS transition failed, keep connect to " MACSTR
			       " \n",
			       MAC2STR(priv->cfg_bssid));
			moal_memcpy_ext(priv->phandle, (void *)priv->conn_bssid,
					&priv->cfg_bssid, MLAN_MAC_ADDR_LENGTH,
					sizeof(priv->conn_bssid));
			priv->sme_current.bssid = priv->conn_bssid;
			priv->ft_ie_len = priv->pre_ft_ie_len;
			moal_memcpy_ext(priv->phandle, priv->ft_ie,
					priv->pre_ft_ie, priv->pre_ft_ie_len,
					MAX_IE_SIZE);
		}
		priv->ft_roaming_triggered_by_driver = MFALSE;

	} else {
		if (!ret) {
			memset(assoc_rsp, 0, sizeof(mlan_ds_misc_assoc_rsp));
			woal_get_assoc_rsp(priv, assoc_rsp, MOAL_IOCTL_WAIT);
			passoc_rsp = (IEEEtypes_AssocRsp_t *)
					     assoc_rsp->assoc_resp_buf;
			cfg80211_connect_result(priv->netdev, priv->cfg_bssid,
						NULL, 0, passoc_rsp->ie_buffer,
						assoc_rsp->assoc_resp_len -
							ASSOC_RESP_FIXED_SIZE,
						WLAN_STATUS_SUCCESS,
						GFP_KERNEL);
			PRINTM(MMSG,
			       "wlan: Fast Bss transition to bssid " MACSTR
			       " successfully\n",
			       MAC2STR(priv->cfg_bssid));

			memset(&bss_info, 0, sizeof(bss_info));
			woal_get_bss_info(priv, MOAL_IOCTL_WAIT, &bss_info);
			priv->channel = bss_info.bss_chan;
		} else {
			PRINTM(MMSG,
			       "wlan: Failed to connect to bssid " MACSTR "\n",
			       MAC2STR(priv->target_ap_bssid));
			cfg80211_connect_result(priv->netdev,
						priv->target_ap_bssid, NULL, 0,
						NULL, 0,
						woal_get_assoc_status(priv),
						GFP_KERNEL);
			moal_memcpy_ext(priv->phandle, (void *)priv->conn_bssid,
					&priv->cfg_bssid, MLAN_MAC_ADDR_LENGTH,
					sizeof(priv->conn_bssid));
			priv->sme_current.bssid = priv->conn_bssid;
			memset(priv->target_ap_bssid, 0, ETH_ALEN);
			priv->ft_ie_len = priv->pre_ft_ie_len;
			moal_memcpy_ext(priv->phandle, priv->ft_ie,
					priv->pre_ft_ie, priv->pre_ft_ie_len,
					MAX_IE_SIZE);
			// priv->ft_ie_len = 0;
		}
	}
	kfree(assoc_rsp);
	priv->ft_pre_connect = MFALSE;
	LEAVE();
	return 0;
}
#endif

/**
 * @brief Save connect parameters for roaming
 *
 * @param priv            A pointer to moal_private
 * @param sme             A pointer to cfg80211_connect_params structure
 */
void woal_save_conn_params(moal_private *priv,
			   struct cfg80211_connect_params *sme)
{
	t_u8 *pIe = NULL;

	ENTER();
	woal_clear_conn_params(priv);
	moal_memcpy_ext(priv->phandle, &priv->sme_current, sme,
			sizeof(struct cfg80211_connect_params),
			sizeof(priv->sme_current));
	if (sme->channel) {
		priv->sme_current.channel = &priv->conn_chan;
		moal_memcpy_ext(priv->phandle, priv->sme_current.channel,
				sme->channel, sizeof(struct ieee80211_channel),
				sizeof(priv->conn_chan));
	}
	if (sme->bssid) {
		moal_memcpy_ext(priv->phandle, (void *)priv->conn_bssid,
				sme->bssid, MLAN_MAC_ADDR_LENGTH,
				sizeof(priv->conn_bssid));
		priv->sme_current.bssid = priv->conn_bssid;
	}
	if (sme->ssid && sme->ssid_len) {
		memset(priv->conn_ssid, 0, MLAN_MAX_SSID_LENGTH);
		moal_memcpy_ext(priv->phandle, (void *)priv->conn_ssid,
				sme->ssid, sme->ssid_len,
				sizeof(priv->conn_ssid));
		priv->sme_current.ssid = priv->conn_ssid;
	}
	if (sme->ie && sme->ie_len) {
		pIe = kzalloc(sme->ie_len, GFP_KERNEL);
		if (!pIe) {
			PRINTM(MERROR,
			       "Failed to allocate memory for sme params\n");
			LEAVE();
			return;
		}
		moal_memcpy_ext(priv->phandle, (void *)pIe, sme->ie,
				sme->ie_len, sme->ie_len);
		priv->sme_current.ie = pIe;
	}
	if (sme->key && sme->key_len && (sme->key_len <= MAX_WEP_KEY_SIZE)) {
		moal_memcpy_ext(priv->phandle, (t_u8 *)priv->conn_wep_key,
				sme->key, sme->key_len,
				sizeof(priv->conn_wep_key));
		priv->sme_current.key = priv->conn_wep_key;
	}
	if (priv->sinfo)
		memset(priv->sinfo, 0, sizeof(struct station_info));
	else
		priv->sinfo = kzalloc(sizeof(struct station_info), GFP_KERNEL);
}

/**
 * @brief clear connect parameters for ing
 *
 * @param priv            A pointer to moal_private
 */
void woal_clear_conn_params(moal_private *priv)
{
	ENTER();
	if (priv->sme_current.ie_len)
		kfree(priv->sme_current.ie);
	memset(&priv->sme_current, 0, sizeof(struct cfg80211_connect_params));
	priv->roaming_required = MFALSE;
	priv->assoc_bss = NULL;
	if (priv->sinfo) {
		kfree(priv->sinfo);
		priv->sinfo = NULL;
	}
	LEAVE();
}

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 2, 0)
/**
 * @brief Build new roaming connect ie for okc
 *
 * @param priv            A pointer to moal_private
 * @param entry           A pointer to pmksa_entry
 **/
static int woal_update_okc_roaming_ie(moal_private *priv,
				      struct pmksa_entry *entry)
{
	struct cfg80211_connect_params *sme = &priv->sme_current;
	int ret = MLAN_STATUS_SUCCESS;
	const t_u8 *sme_pos, *sme_ptr;
	t_u8 *okc_ie_pos;
	t_u8 id, ie_len;
	int left_len;

	ENTER();

	if (!sme->ie || !sme->ie_len) {
		PRINTM(MERROR, "No connect ie saved in driver\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	if (!entry) {
		PRINTM(MERROR, "No roaming ap pmkid\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	if (!priv->okc_roaming_ie) {
		t_u32 okc_ie_len =
			MIN(UINT_MAX, sme->ie_len + sizeof(t_u16) + PMKID_LEN);

		/** Alloc new buffer for okc roaming ie */
		priv->okc_roaming_ie = kzalloc(okc_ie_len, GFP_KERNEL);
		if (!priv->okc_roaming_ie) {
			PRINTM(MERROR, "Fail to allocate assoc req ie\n");
			ret = MLAN_STATUS_FAILURE;
			goto done;
		}
	}

	/* Build OKC RSN IE with PMKID list
	 * Format of RSN IE: length(bytes) and container
	 * | 1| 1 |   2   |          4            |           2               |
	 * |id|len|version|group data cipher suite|pairwise cipher suite count|
	 * |          4 * m           |       2       |    4 * n     |   2    |
	 * |pairwise cipher suite list|AKM suite count|AKM suite list|RSN Cap |
	 * |    2     |  16 * s  |              4              |
	 * |PMKIDCount|PMKID List|Group Management Cipher Suite|
	 */
#define PAIRWISE_CIPHER_COUNT_OFFSET 8
#define AKM_SUITE_COUNT_OFFSET(n) (10 + (n)*4)
#define PMKID_COUNT_OFFSET(n) (14 + (n)*4)

	sme_pos = sme->ie;
	left_len = sme->ie_len;
	okc_ie_pos = priv->okc_roaming_ie;
	priv->okc_ie_len = 0;

	while (left_len >= 2) {
		id = *sme_pos;
		ie_len = *(sme_pos + 1);
		if ((ie_len + 2) > left_len) {
			PRINTM(MERROR, "Invalid ie len %d\n", ie_len);
			ret = MLAN_STATUS_FAILURE;
			goto done;
		}

		if (id == RSN_IE) {
			t_u16 pairwise_count, akm_count;
			t_u8 *rsn_ie_len;
			int rsn_offset;

			pairwise_count =
				*(const t_u16 *)(sme_pos +
						 PAIRWISE_CIPHER_COUNT_OFFSET);
			akm_count = *(const t_u16 *)(sme_pos +
						     AKM_SUITE_COUNT_OFFSET(
							     pairwise_count));
			rsn_offset =
				PMKID_COUNT_OFFSET(pairwise_count + akm_count);
			sme_ptr = (const t_u8 *)(sme_pos + rsn_offset);

			moal_memcpy_ext(priv->phandle, okc_ie_pos, sme_pos,
					rsn_offset, rsn_offset);
			rsn_ie_len = okc_ie_pos + 1;
			okc_ie_pos += rsn_offset;
			*(t_u16 *)okc_ie_pos = 1;
			okc_ie_pos += sizeof(t_u16);
			moal_memcpy_ext(priv->phandle, okc_ie_pos, entry->pmkid,
					PMKID_LEN, PMKID_LEN);
			okc_ie_pos += PMKID_LEN;
			priv->okc_ie_len +=
				rsn_offset + sizeof(t_u16) + PMKID_LEN;
			*rsn_ie_len =
				rsn_offset - 2 + sizeof(t_u16) + PMKID_LEN;

			if ((ie_len + 2) > rsn_offset) {
				/** Previous conn ie include pmkid list */
				u16 pmkid_count = *(const t_u16 *)sme_ptr;
				rsn_offset += (sizeof(t_u16) +
					       PMKID_LEN * pmkid_count);
				if ((ie_len + 2) > rsn_offset) {
					sme_ptr += (sizeof(t_u16) +
						    PMKID_LEN * pmkid_count);
					moal_memcpy_ext(
						priv->phandle, okc_ie_pos,
						sme_ptr,
						(ie_len + 2 - rsn_offset),
						(ie_len + 2 - rsn_offset));
					okc_ie_pos += (ie_len + 2 - rsn_offset);
					priv->okc_ie_len +=
						(ie_len + 2 - rsn_offset);
					*rsn_ie_len +=
						(ie_len + 2 - rsn_offset);
				}
			}
		} else {
			moal_memcpy_ext(priv->phandle, okc_ie_pos, sme_pos,
					ie_len + 2, ie_len + 2);
			okc_ie_pos += ie_len + 2;
			priv->okc_ie_len += ie_len + 2;
		}

		sme_pos += (ie_len + 2);
		left_len -= (ie_len + 2);
	}

done:
	if (ret != MLAN_STATUS_SUCCESS) {
		if (priv->okc_roaming_ie) {
			kfree(priv->okc_roaming_ie);
			priv->okc_roaming_ie = NULL;
			priv->okc_ie_len = 0;
		}
	}

	LEAVE();
	return ret;
}
#endif

/**
 * @brief Start roaming: driver handle roaming
 *
 * @param priv      A pointer to moal_private structure
 *
 * @return          N/A
 */
void woal_start_roaming(moal_private *priv)
{
	mlan_ds_get_signal signal;
	mlan_ssid_bssid *ssid_bssid = NULL;
	char rssi_low[10];
	int ret = 0;
	mlan_ds_misc_assoc_rsp *assoc_rsp = NULL;
	IEEEtypes_AssocRsp_t *passoc_rsp = NULL;
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 12, 0)
	struct cfg80211_roam_info roam_info = {};
#endif

	ENTER();
	if (priv->ft_roaming_triggered_by_driver) {
		PRINTM(MIOCTL, "FT roaming is in processing ...... \n");
		LEAVE();
		return;
	}
	ssid_bssid = kmalloc(sizeof(mlan_ssid_bssid), GFP_KERNEL);
	if (!ssid_bssid) {
		LEAVE();
		return;
	}

	if (priv->last_event & EVENT_BG_SCAN_REPORT) {
		woal_inform_bss_from_scan_result(priv, NULL, MOAL_IOCTL_WAIT);
		PRINTM(MIOCTL, "Report bgscan result\n");
	}
	if (priv->media_connected == MFALSE || !priv->sme_current.ssid_len) {
		PRINTM(MIOCTL, "Not connected, ignore roaming\n");
		kfree(ssid_bssid);
		LEAVE();
		return;
	}

	/* Get signal information from the firmware */
	memset(&signal, 0, sizeof(mlan_ds_get_signal));
	if (MLAN_STATUS_SUCCESS !=
	    woal_get_signal_info(priv, MOAL_IOCTL_WAIT, &signal)) {
		PRINTM(MERROR, "Error getting signal information\n");
		ret = -EFAULT;
		goto done;
	}
	memset(ssid_bssid, 0, sizeof(mlan_ssid_bssid));
	ssid_bssid->ssid.ssid_len = priv->sme_current.ssid_len;
	moal_memcpy_ext(priv->phandle, ssid_bssid->ssid.ssid,
			priv->sme_current.ssid, priv->sme_current.ssid_len,
			sizeof(ssid_bssid->ssid.ssid));
	if (MLAN_STATUS_SUCCESS !=
	    woal_find_best_network(priv, MOAL_IOCTL_WAIT, ssid_bssid)) {
		PRINTM(MIOCTL, "Can not find better network\n");
		ret = -EFAULT;
		goto done;
	}
	/* check if we found different AP */
	if (!memcmp(&ssid_bssid->bssid, priv->cfg_bssid,
		    MLAN_MAC_ADDR_LENGTH)) {
		PRINTM(MIOCTL, "This is the same AP, no roaming\n");
		ret = -EFAULT;
		goto done;
	}
	PRINTM(MIOCTL, "Find AP: bssid=" MACSTR ", signal=%d\n",
	       MAC2STR(ssid_bssid->bssid), ssid_bssid->rssi);
	/* check signal */
	if (!(priv->last_event & EVENT_PRE_BCN_LOST)) {
		if ((abs(signal.bcn_rssi_avg) - abs(ssid_bssid->rssi)) <
		    DELTA_RSSI) {
			PRINTM(MERROR, "New AP's signal is not good too.\n");
			ret = -EFAULT;
			goto done;
		}
	}
	/**check if need start FT Roaming*/
	if (priv->ft_ie_len && (priv->ft_md == ssid_bssid->ft_md) &&
	    (priv->ft_cap == ssid_bssid->ft_cap)) {
		priv->ft_roaming_triggered_by_driver = MTRUE;
		woal_start_ft_roaming(priv, ssid_bssid);
		goto done;
	}
	/* start roaming to new AP */
	moal_memcpy_ext(priv->phandle, (void *)priv->conn_bssid,
			&ssid_bssid->bssid, MLAN_MAC_ADDR_LENGTH,
			sizeof(priv->conn_bssid));
	priv->sme_current.bssid = priv->conn_bssid;
#ifdef STA_CFG80211
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 2, 0)
	if (IS_STA_CFG80211(priv->phandle->params.cfg80211_wext)) {
		/** Check if current roaming support OKC offload roaming */
		if (priv->sme_current.crypto.n_akm_suites &&
		    priv->sme_current.crypto.akm_suites[0] ==
			    WLAN_AKM_SUITE_8021X) {
			struct pmksa_entry *entry = NULL;

			/** Get OKC PMK Cache entry
			 *  Firstly try to get pmksa from cfg80211
			 */
			priv->wait_target_ap_pmkid = MTRUE;
			cfg80211_pmksa_candidate_notify(priv->netdev, 0,
							priv->sme_current.bssid,
							MTRUE, GFP_ATOMIC);
			if (wait_event_interruptible_timeout(
				    priv->okc_wait_q,
				    !priv->wait_target_ap_pmkid,
				    OKC_WAIT_TARGET_PMKSA_TIMEOUT)) {
				PRINTM(MIOCTL, "OKC Roaming is ready\n");
				entry = priv->target_ap_pmksa;
			} else {
				/** Try to get pmksa from pmksa list */
				priv->wait_target_ap_pmkid = MFALSE;
				entry = woal_get_pmksa_entry(
					priv, priv->sme_current.bssid);
			}
			/** Build okc roaming ie */
			woal_update_okc_roaming_ie(priv, entry);
			priv->target_ap_pmksa = NULL;
		}
	}
#endif
#endif
	assoc_rsp = kzalloc(sizeof(mlan_ds_misc_assoc_rsp), GFP_ATOMIC);
	if (!assoc_rsp) {
		PRINTM(MERROR, "Failed to allocate memory for assoc_rsp\n");
		ret = -ENOMEM;
		goto done;
	}
	ret = woal_cfg80211_assoc(priv, (void *)&priv->sme_current,
				  MOAL_IOCTL_WAIT, assoc_rsp);
	if (!ret) {
		const t_u8 *ie;
		int ie_len;

		woal_inform_bss_from_scan_result(priv, NULL, MOAL_IOCTL_WAIT);
		passoc_rsp = (IEEEtypes_AssocRsp_t *)assoc_rsp->assoc_resp_buf;

		/** Update connect ie in roam event */
		ie = priv->sme_current.ie;
		ie_len = priv->sme_current.ie_len;
#ifdef STA_CFG80211
		if (IS_STA_CFG80211(priv->phandle->params.cfg80211_wext)) {
			/** Check if current roaming support OKC offload roaming
			 */
			if (priv->sme_current.crypto.n_akm_suites &&
			    priv->sme_current.crypto.akm_suites[0] ==
				    WLAN_AKM_SUITE_8021X) {
				if (priv->okc_roaming_ie && priv->okc_ie_len) {
					ie = priv->okc_roaming_ie;
					ie_len = priv->okc_ie_len;
				}
			}
		}
#endif
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 12, 0)
#if ((CFG80211_VERSION_CODE >= KERNEL_VERSION(6, 0, 0)) ||                     \
     (defined(ANDROID_SDK_VERSION) && ANDROID_SDK_VERSION >= 31))
		roam_info.links[0].bssid = priv->cfg_bssid;
#else
		roam_info.bssid = priv->cfg_bssid;
#endif
		roam_info.req_ie = ie;
		roam_info.req_ie_len = ie_len;
		roam_info.resp_ie = passoc_rsp->ie_buffer;
		roam_info.resp_ie_len =
			assoc_rsp->assoc_resp_len - ASSOC_RESP_FIXED_SIZE;
		cfg80211_roamed(priv->netdev, &roam_info, GFP_KERNEL);
#else
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 0, 0)
		cfg80211_roamed(priv->netdev, NULL, priv->cfg_bssid, ie, ie_len,
				passoc_rsp->ie_buffer,
				assoc_rsp->assoc_resp_len -
					ASSOC_RESP_FIXED_SIZE,
				GFP_KERNEL);
#else
		cfg80211_roamed(priv->netdev, priv->cfg_bssid, ie, ie_len,
				passoc_rsp->ie_buffer,
				assoc_rsp->assoc_resp_len -
					ASSOC_RESP_FIXED_SIZE,
				GFP_KERNEL);
#endif
#endif
		PRINTM(MMSG, "Roamed to bssid " MACSTR " successfully\n",
		       MAC2STR(priv->cfg_bssid));
	} else {
		PRINTM(MIOCTL, "Roaming to bssid " MACSTR " failed\n",
		       MAC2STR(ssid_bssid->bssid));
	}
	kfree(assoc_rsp);
done:
	kfree(ssid_bssid);
	/* config rssi low threshold again */
	priv->last_event = 0;
	priv->rssi_low = DEFAULT_RSSI_LOW_THRESHOLD;
	if (snprintf(rssi_low, sizeof(rssi_low), "%d", priv->rssi_low) <= 0)
		PRINTM(MERROR, "Fail to print rssi low threshold in buffer\n");
	if (MLAN_STATUS_FAILURE ==
	    woal_set_rssi_low_threshold(priv, rssi_low, MOAL_IOCTL_WAIT))
		PRINTM(MERROR, "set_rssi_low_threshold fail\n");
	LEAVE();
	return;
}

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
#ifdef UAP_SUPPORT
/**
 * @brief add uap station
 *
 * @param wiphy                 A pointer to wiphy structure
 * @param dev                   A pointer to net_device structure
 * @param mac                  A pointer to peer mac
 * @param params           	station parameters
 *
 * @return                  	0 -- success, otherwise fail
 */
int woal_cfg80211_uap_add_station(struct wiphy *wiphy, struct net_device *dev,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 16, 0)
				  const u8 *mac,
#else
				  u8 *mac,
#endif
				  struct station_parameters *params)
{
	moal_private *priv = (moal_private *)woal_get_netdev_priv(dev);
	mlan_ioctl_req *req = NULL;
	t_u32 req_len = 0;
	mlan_ds_bss *bss = NULL;
	t_u8 *pos;
	t_u8 qosinfo;
	MrvlIEtypes_Data_t *tlv;
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 20, 0)
	MrvlExtIEtypes_Data_t *ext_tlv;
#endif
	mlan_status status;
	int ret = 0;

	ENTER();

	req_len = sizeof(mlan_ds_bss);
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 9, 0)
	if (params->ext_capab_len)
		req_len += sizeof(MrvlIEtypesHeader_t) + params->ext_capab_len;
#endif
#if ((CFG80211_VERSION_CODE >= KERNEL_VERSION(6, 0, 0)) ||                     \
     (defined(ANDROID_SDK_VERSION) && ANDROID_SDK_VERSION >= 34))
	if (params->link_sta_params.supported_rates_len)
		req_len += sizeof(MrvlIEtypesHeader_t) +
			   params->link_sta_params.supported_rates_len;
#else
	if (params->supported_rates_len)
		req_len += sizeof(MrvlIEtypesHeader_t) +
			   params->supported_rates_len;
#endif
	if (params->uapsd_queues || params->max_sp)
		req_len += sizeof(MrvlIEtypesHeader_t) + sizeof(qosinfo);
#if ((CFG80211_VERSION_CODE >= KERNEL_VERSION(6, 0, 0)) ||                     \
     (defined(ANDROID_SDK_VERSION) && ANDROID_SDK_VERSION >= 34))
	if (params->link_sta_params.ht_capa)
#else
	if (params->ht_capa)
#endif
		req_len += sizeof(MrvlIEtypesHeader_t) +
			   sizeof(struct ieee80211_ht_cap);
#if ((CFG80211_VERSION_CODE >= KERNEL_VERSION(6, 0, 0)) ||                     \
     (defined(ANDROID_SDK_VERSION) && ANDROID_SDK_VERSION >= 34))
	if (params->link_sta_params.vht_capa)
#else
	if (params->vht_capa)
#endif
		req_len += sizeof(MrvlIEtypesHeader_t) +
			   sizeof(struct ieee80211_vht_cap);
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 14, 0)
#if ((CFG80211_VERSION_CODE >= KERNEL_VERSION(6, 0, 0)) ||                     \
     (defined(ANDROID_SDK_VERSION) && ANDROID_SDK_VERSION >= 34))
	if (params->link_sta_params.opmode_notif_used)
		req_len += sizeof(MrvlIEtypesHeader_t) + sizeof(u8);
#else
	if (params->opmode_notif_used)
		req_len += sizeof(MrvlIEtypesHeader_t) + sizeof(u8);
#endif
#endif

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 20, 0)
#if ((CFG80211_VERSION_CODE >= KERNEL_VERSION(6, 0, 0)) ||                     \
     (defined(ANDROID_SDK_VERSION) && ANDROID_SDK_VERSION >= 34))
	if (params->link_sta_params.he_capa_len)
		req_len += sizeof(MrvlExtIEtypesHeader_t) +
			   params->link_sta_params.he_capa_len;
	if (params->link_sta_params.he_6ghz_capa)
		req_len += sizeof(MrvlExtIEtypesHeader_t) +
			   sizeof(params->link_sta_params.he_6ghz_capa->capa);
#else
	if (params->he_capa_len)
		req_len += sizeof(MrvlExtIEtypesHeader_t) + params->he_capa_len;

#if (CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 8, 0))
	if (params->he_6ghz_capa)
		req_len += sizeof(MrvlExtIEtypesHeader_t) +
			   sizeof(params->he_6ghz_capa->capa);
#endif

#endif
#endif
	req = woal_alloc_mlan_ioctl_req(req_len);
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}
	bss = (mlan_ds_bss *)req->pbuf;
	bss->sub_command = MLAN_OID_UAP_ADD_STATION;
	req->req_id = MLAN_IOCTL_BSS;
	req->action = MLAN_ACT_SET;
	bss->param.sta_info.listen_interval = params->listen_interval;
	bss->param.sta_info.aid = params->aid;
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 9, 0)
	bss->param.sta_info.cap_info = params->capability;
#else
	bss->param.sta_info.cap_info = 0;
#endif
	bss->param.sta_info.tlv_len = 0;
	bss->param.sta_info.sta_flags = params->sta_flags_set;
	moal_memcpy_ext(priv->phandle, bss->param.sta_info.peer_mac, mac,
			MLAN_MAC_ADDR_LENGTH,
			sizeof(bss->param.sta_info.peer_mac));
	PRINTM(MMSG, "wlan: UAP/GO add peer station, address =" MACSTR "\n",
	       MAC2STR(mac));
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 9, 0)
	PRINTM(MCMND,
	       "sta_flags=0x%x listen_interval=%d aid=%d cap_info=0x%x\n",
	       params->sta_flags_set, params->listen_interval, params->aid,
	       params->capability);
#else
	PRINTM(MCMND, "sta_flags=0x%x listen_interval=%d aid=%d\n",
	       params->sta_flags_set, params->listen_interval, params->aid);
#endif
	pos = &bss->param.sta_info.tlv[0];
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 9, 0)
	if (params->ext_capab_len) {
		tlv = (MrvlIEtypes_Data_t *)pos;
		tlv->header.type = EXT_CAPABILITY;
		tlv->header.len = params->ext_capab_len;
		moal_memcpy_ext(priv->phandle, tlv->data, params->ext_capab,
				tlv->header.len, tlv->header.len);
		pos += sizeof(MrvlIEtypesHeader_t) + tlv->header.len;
		bss->param.sta_info.tlv_len +=
			sizeof(MrvlIEtypesHeader_t) + tlv->header.len;
		tlv = (MrvlIEtypes_Data_t *)pos;
	}
#endif
#if ((CFG80211_VERSION_CODE >= KERNEL_VERSION(6, 0, 0)) ||                     \
     (defined(ANDROID_SDK_VERSION) && ANDROID_SDK_VERSION >= 34))
	if (params->link_sta_params.supported_rates_len) {
#else
	if (params->supported_rates_len) {
#endif
		tlv = (MrvlIEtypes_Data_t *)pos;
		tlv->header.type = SUPPORTED_RATES;
#if ((CFG80211_VERSION_CODE >= KERNEL_VERSION(6, 0, 0)) ||                     \
     (defined(ANDROID_SDK_VERSION) && ANDROID_SDK_VERSION >= 34))
		tlv->header.len = params->link_sta_params.supported_rates_len;
#else
		tlv->header.len = params->supported_rates_len;
#endif
		moal_memcpy_ext(priv->phandle, tlv->data,
#if ((CFG80211_VERSION_CODE >= KERNEL_VERSION(6, 0, 0)) ||                     \
     (defined(ANDROID_SDK_VERSION) && ANDROID_SDK_VERSION >= 34))
				params->link_sta_params.supported_rates,
				tlv->header.len,
#else
				params->supported_rates, tlv->header.len,
#endif
				tlv->header.len);
		pos += sizeof(MrvlIEtypesHeader_t) + tlv->header.len;
		bss->param.sta_info.tlv_len +=
			sizeof(MrvlIEtypesHeader_t) + tlv->header.len;
		tlv = (MrvlIEtypes_Data_t *)pos;
	}
	if (params->uapsd_queues || params->max_sp) {
		tlv = (MrvlIEtypes_Data_t *)pos;
		tlv->header.type = QOS_INFO;
		tlv->header.len = sizeof(qosinfo);
		qosinfo = params->uapsd_queues | (params->max_sp << 5);
		moal_memcpy_ext(priv->phandle, tlv->data, &qosinfo,
				tlv->header.len, tlv->header.len);
		pos += sizeof(MrvlIEtypesHeader_t) + tlv->header.len;
		bss->param.sta_info.tlv_len +=
			sizeof(MrvlIEtypesHeader_t) + tlv->header.len;
		tlv = (MrvlIEtypes_Data_t *)pos;
	}
#if ((CFG80211_VERSION_CODE >= KERNEL_VERSION(6, 0, 0)) ||                     \
     (defined(ANDROID_SDK_VERSION) && ANDROID_SDK_VERSION >= 34))
	if (params->link_sta_params.ht_capa) {
#else
	if (params->ht_capa) {
#endif
		tlv = (MrvlIEtypes_Data_t *)pos;
		tlv->header.type = HT_CAPABILITY;
		tlv->header.len = sizeof(struct ieee80211_ht_cap);
#if ((CFG80211_VERSION_CODE >= KERNEL_VERSION(6, 0, 0)) ||                     \
     (defined(ANDROID_SDK_VERSION) && ANDROID_SDK_VERSION >= 34))
		moal_memcpy_ext(priv->phandle, tlv->data,
				params->link_sta_params.ht_capa,
#else
		moal_memcpy_ext(priv->phandle, tlv->data, params->ht_capa,
#endif
				tlv->header.len, tlv->header.len);
		pos += sizeof(MrvlIEtypesHeader_t) + tlv->header.len;
		bss->param.sta_info.tlv_len +=
			sizeof(MrvlIEtypesHeader_t) + tlv->header.len;
		tlv = (MrvlIEtypes_Data_t *)pos;
	}
#if ((CFG80211_VERSION_CODE >= KERNEL_VERSION(6, 0, 0)) ||                     \
     (defined(ANDROID_SDK_VERSION) && ANDROID_SDK_VERSION >= 34))
	if (params->link_sta_params.vht_capa) {
#else
	if (params->vht_capa) {
#endif
		tlv = (MrvlIEtypes_Data_t *)pos;
		tlv->header.type = VHT_CAPABILITY;
		tlv->header.len = sizeof(struct ieee80211_vht_cap);
#if ((CFG80211_VERSION_CODE >= KERNEL_VERSION(6, 0, 0)) ||                     \
     (defined(ANDROID_SDK_VERSION) && ANDROID_SDK_VERSION >= 34))
		moal_memcpy_ext(priv->phandle, tlv->data,
				params->link_sta_params.vht_capa,
#else
		moal_memcpy_ext(priv->phandle, tlv->data, params->vht_capa,
#endif
				tlv->header.len, tlv->header.len);
		pos += sizeof(MrvlIEtypesHeader_t) + tlv->header.len;
		bss->param.sta_info.tlv_len +=
			sizeof(MrvlIEtypesHeader_t) + tlv->header.len;
		tlv = (MrvlIEtypes_Data_t *)pos;
	}
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 14, 0)
#if ((CFG80211_VERSION_CODE >= KERNEL_VERSION(6, 0, 0)) ||                     \
     (defined(ANDROID_SDK_VERSION) && ANDROID_SDK_VERSION >= 34))
	if (params->link_sta_params.opmode_notif_used) {
#else
	if (params->opmode_notif_used) {
#endif
		tlv = (MrvlIEtypes_Data_t *)pos;
		tlv->header.type = OPER_MODE_NTF;
		tlv->header.len = sizeof(u8);
#if ((CFG80211_VERSION_CODE >= KERNEL_VERSION(6, 0, 0)) ||                     \
     (defined(ANDROID_SDK_VERSION) && ANDROID_SDK_VERSION >= 34))
		moal_memcpy_ext(priv->phandle, tlv->data,
				&params->link_sta_params.opmode_notif,
#else
		moal_memcpy_ext(priv->phandle, tlv->data, &params->opmode_notif,
#endif
				tlv->header.len, tlv->header.len);
		pos += sizeof(MrvlIEtypesHeader_t) + tlv->header.len;
		bss->param.sta_info.tlv_len +=
			sizeof(MrvlIEtypesHeader_t) + tlv->header.len;
		tlv = (MrvlIEtypes_Data_t *)pos;
	}
#endif
#if ((CFG80211_VERSION_CODE >= KERNEL_VERSION(6, 0, 0)) ||                     \
     (defined(ANDROID_SDK_VERSION) && ANDROID_SDK_VERSION >= 34))
	if (params->link_sta_params.he_capa_len) {
		ext_tlv = (MrvlExtIEtypes_Data_t *)pos;
		ext_tlv->header.type = EXTENSION;
		ext_tlv->header.len =
			params->link_sta_params.he_capa_len + sizeof(u8);
		ext_tlv->header.ext_id = HE_CAPABILITY;
		moal_memcpy_ext(priv->phandle, ext_tlv->data,
				(u8 *)params->link_sta_params.he_capa,
				params->link_sta_params.he_capa_len,
				params->link_sta_params.he_capa_len);
		pos += sizeof(MrvlExtIEtypesHeader_t) +
		       params->link_sta_params.he_capa_len;
		bss->param.sta_info.tlv_len +=
			sizeof(MrvlExtIEtypesHeader_t) +
			params->link_sta_params.he_capa_len;
		tlv = (MrvlIEtypes_Data_t *)pos;
	}
	if (params->link_sta_params.he_6ghz_capa) {
		const t_size he_6g_cap_size =
			sizeof(params->link_sta_params.he_6ghz_capa->capa);

		ext_tlv = (MrvlExtIEtypes_Data_t *)pos;
		ext_tlv->header.type = EXTENSION;
		ext_tlv->header.len = he_6g_cap_size + sizeof(u8);
		ext_tlv->header.ext_id = HE_6G_CAPABILITY;
		moal_memcpy_ext(
			priv->phandle, ext_tlv->data,
			(u8 *)&params->link_sta_params.he_6ghz_capa->capa,
			he_6g_cap_size, he_6g_cap_size);
		pos += sizeof(MrvlExtIEtypesHeader_t) + he_6g_cap_size;
		bss->param.sta_info.tlv_len +=
			sizeof(MrvlExtIEtypesHeader_t) + he_6g_cap_size;
		tlv = (MrvlIEtypes_Data_t *)pos;
	}

#elif CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 20, 0)
	if (params->he_capa_len) {
		ext_tlv = (MrvlExtIEtypes_Data_t *)pos;
		ext_tlv->header.type = EXTENSION;
		ext_tlv->header.len = params->he_capa_len + sizeof(u8);
		ext_tlv->header.ext_id = HE_CAPABILITY;
		moal_memcpy_ext(priv->phandle, ext_tlv->data,
				(const u8 *)params->he_capa,
				params->he_capa_len, params->he_capa_len);
		pos += sizeof(MrvlExtIEtypesHeader_t) + params->he_capa_len;
		bss->param.sta_info.tlv_len +=
			sizeof(MrvlExtIEtypesHeader_t) + params->he_capa_len;
		tlv = (MrvlIEtypes_Data_t *)pos;
	}

#if (CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 8, 0))
	if (params->he_6ghz_capa) {
		const t_size he_6g_cap_size =
			sizeof(params->he_6ghz_capa->capa);

		ext_tlv = (MrvlExtIEtypes_Data_t *)pos;
		ext_tlv->header.type = EXTENSION;
		ext_tlv->header.len = he_6g_cap_size + sizeof(u8);
		ext_tlv->header.ext_id = HE_6G_CAPABILITY;
		moal_memcpy_ext(priv->phandle, ext_tlv->data,
				(u8 *)&params->he_6ghz_capa->capa,
				he_6g_cap_size, he_6g_cap_size);
		pos += sizeof(MrvlExtIEtypesHeader_t) + he_6g_cap_size;
		bss->param.sta_info.tlv_len +=
			sizeof(MrvlExtIEtypesHeader_t) + he_6g_cap_size;
		tlv = (MrvlIEtypes_Data_t *)pos;
	}
#endif

#endif
	DBG_HEXDUMP(MCMD_D, "sta tlv", &bss->param.sta_info.tlv[0],
		    bss->param.sta_info.tlv_len);
	/* Send IOCTL request to MLAN */
	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
#if KERNEL_VERSION(3, 8, 0) <= CFG80211_VERSION_CODE
	if (moal_extflg_isset(priv->phandle, EXT_HOST_MLME)) {
		struct station_info *sinfo = NULL;
		sinfo = kzalloc(sizeof(struct station_info), GFP_KERNEL);
		if (sinfo) {
			cfg80211_new_sta(dev, mac, sinfo, GFP_KERNEL);
			kfree(sinfo);
		}
	}
#endif
done:
	LEAVE();
	return ret;
}

/**
 *  @brief This function is probe client handle.
 *
 *  @param wiphy       A pointer to wiphy.
 *
 *  @param dev         A pointer to net_device
 *
 *  @param peer        A pointer to peer
 *
 *  @param cookie      A pointer to cookie
 *
 *  @return            0 -- success, otherwise fail
 */
static int woal_cfg80211_probe_client(struct wiphy *wiphy,
				      struct net_device *dev, const u8 *peer,
				      u64 *cookie)
{
	return -1;
}
#endif

/**
 *  @brief Sends deauth packet to kernel
 *
 *  @param priv A pointer to moal_private struct
 *  @param sa   A pointer to source address
 *  @param reason_code  disconnect reason code
 *  @return     N/A
 */
void woal_host_mlme_disconnect(moal_private *priv, u16 reason_code, u8 *sa)
{
	t_u8 broadcast_addr[ETH_ALEN] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
	t_u8 frame_buf[100];
	struct ieee80211_mgmt *mgmt = (struct ieee80211_mgmt *)frame_buf;
	ENTER();

	memset(frame_buf, 0, sizeof(frame_buf));
	mgmt->frame_control =
		(__force __le16)cpu_to_le16(IEEE80211_STYPE_DEAUTH);
	mgmt->duration = 0;
	mgmt->seq_ctrl = 0;
	mgmt->u.deauth.reason_code = (__force __le16)cpu_to_le16(reason_code);
	if (GET_BSS_ROLE(priv) == MLAN_BSS_ROLE_STA) {
		moal_memcpy_ext(priv->phandle, mgmt->da, broadcast_addr,
				ETH_ALEN, sizeof(mgmt->da));
		moal_memcpy_ext(priv->phandle, mgmt->sa,
				priv->sme_current.bssid, ETH_ALEN,
				sizeof(mgmt->sa));
		moal_memcpy_ext(priv->phandle, mgmt->bssid, sa, ETH_ALEN,
				sizeof(mgmt->bssid));
		priv->host_mlme = MFALSE;
		priv->auth_flag = 0;
	} else {
		moal_memcpy_ext(priv->phandle, mgmt->da, priv->current_addr,
				ETH_ALEN, sizeof(mgmt->da));
		moal_memcpy_ext(priv->phandle, mgmt->sa, sa, ETH_ALEN,
				sizeof(mgmt->sa));
		moal_memcpy_ext(priv->phandle, mgmt->bssid, priv->current_addr,
				ETH_ALEN, sizeof(mgmt->bssid));
		PRINTM(MMSG,
		       "wlan: hostmlme notify deauth station " MACSTR "\n",
		       MAC2STR(sa));
	}

	if (GET_BSS_ROLE(priv) != MLAN_BSS_ROLE_UAP) {
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(6, 7, 0)
		wiphy_lock(priv->wdev->wiphy);
		cfg80211_rx_mlme_mgmt(priv->netdev, frame_buf, 26);
		wiphy_unlock(priv->wdev->wiphy);
#elif CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 11, 0)
		mutex_lock(&priv->wdev->mtx);
		cfg80211_rx_mlme_mgmt(priv->netdev, frame_buf, 26);
		mutex_unlock(&priv->wdev->mtx);
#else
		cfg80211_send_deauth(priv->netdev, frame_buf, 26);
#endif

	} else {
		int freq = ieee80211_channel_to_frequency(
			priv->channel
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(2, 6, 39)
			,
			(priv->channel <= 14 ? IEEE80211_BAND_2GHZ :
					       IEEE80211_BAND_5GHZ)
#endif
		);
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 4, 0)
		cfg80211_rx_mgmt(
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 6, 0)
			priv->wdev,
#else
			priv->netdev,
#endif
			freq, 0, frame_buf, 26
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 12, 0)
			,
			0
#endif
#if CFG80211_VERSION_CODE < KERNEL_VERSION(3, 18, 0)
			,
			GFP_ATOMIC
#endif
		);
#else
		cfg80211_rx_mgmt(priv->netdev, freq, frame_buf, 26, GFP_ATOMIC);
#endif
	}

	LEAVE();
	return;
}
#endif

/**
 * @brief Register the device with cfg80211
 *
 * @param dev       A pointer to net_device structure
 * @param bss_type  BSS type
 *
 * @return          MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status woal_register_sta_cfg80211(struct net_device *dev, t_u8 bss_type)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	moal_private *priv = (moal_private *)netdev_priv(dev);
	struct wireless_dev *wdev = NULL;
	int psmode = 0;
	enum ieee80211_band band;

	ENTER();

	wdev = (struct wireless_dev *)&priv->w_dev;
	memset(wdev, 0, sizeof(struct wireless_dev));
	wdev->wiphy = priv->phandle->wiphy;
	if (!wdev->wiphy) {
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}
	if (bss_type == MLAN_BSS_TYPE_STA) {
		wdev->iftype = NL80211_IFTYPE_STATION;
		priv->roaming_enabled = MFALSE;
		priv->roaming_required = MFALSE;
	}
#ifdef WIFI_DIRECT_SUPPORT
#if CFG80211_VERSION_CODE >= WIFI_DIRECT_KERNEL_VERSION
	if (bss_type == MLAN_BSS_TYPE_WIFIDIRECT)
		wdev->iftype = NL80211_IFTYPE_STATION;
#endif
#endif
	if (bss_type == MLAN_BSS_TYPE_NAN)
		wdev->iftype = NL80211_IFTYPE_STATION;
	dev_net_set(dev, wiphy_net(wdev->wiphy));
	dev->ieee80211_ptr = wdev;
	SET_NETDEV_DEV(dev, wiphy_dev(wdev->wiphy));
	priv->wdev = wdev;
	/* Get IEEE power save mode */
	if (MLAN_STATUS_SUCCESS == woal_set_get_power_mgmt(priv, MLAN_ACT_GET,
							   &psmode, 0,
							   MOAL_IOCTL_WAIT)) {
		/* Save the IEEE power save mode to wiphy, because after
		 * warmreset wiphy power save should be updated instead
		 * of using the last saved configuration */
		if (psmode)
			priv->wdev->ps = MTRUE;
		else
			priv->wdev->ps = MFALSE;
	}
	if (priv->phandle->country_code[0] && priv->phandle->country_code[1]) {
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 0, 0)
		if (priv->phandle->params.cntry_txpwr == CNTRY_RGPOWER_MODE &&
		    !priv->phandle->params.txpwrlimit_cfg)
			queue_work(priv->phandle->evt_workqueue,
				   &priv->phandle->regulatory_work);
#endif
		band = priv->phandle->band;
		priv->phandle->band = IEEE80211_BAND_2GHZ;
		woal_send_domain_info_cmd_fw(priv, MOAL_IOCTL_WAIT);
		if (priv->phandle->fw_bands & BAND_A) {
			priv->phandle->band = IEEE80211_BAND_5GHZ;
			woal_send_domain_info_cmd_fw(priv, MOAL_IOCTL_WAIT);
		}
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 4, 0)
		if (priv->phandle->fw_bands & BAND_6G) {
			priv->phandle->band = IEEE80211_BAND_6GHZ;
			woal_send_domain_info_cmd_fw(priv, MOAL_IOCTL_WAIT);
		}
#endif
		priv->phandle->band = band;
	}
	LEAVE();
	return ret;
}

/**
 * @brief Initialize the wiphy
 *
 * @param priv            A pointer to moal_private structure
 * @param wiphy 		  A pointer to structure wiphy
 * @param fw_info         A pointer to mlan_fw_info
 * @param wait_option     Wait option
 * @return                MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status woal_cfg80211_init_wiphy(moal_private *priv,
					    struct wiphy *wiphy,
					    mlan_fw_info *fw_info,
					    t_u8 wait_option)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	int retry_count, rts_thr, frag_thr;
	mlan_ioctl_req *req = NULL;
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(2, 6, 38)
	mlan_ds_radio_cfg *radio = NULL;
#endif
	pmlan_ds_11n_cfg cfg_11n = NULL;
	t_u32 hw_dev_cap;
#ifdef UAP_SUPPORT
	pmlan_uap_bss_param sys_cfg = NULL;
#endif
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 0, 0)
	t_u16 enable = 0;
#endif
	int mcs_supp = 0;
	char countryOTP[3];

	ENTER();
	countryOTP[0] = fw_info->fw_country_code & 0x00FF;
	countryOTP[1] = (fw_info->fw_country_code & 0xFF00) >> 8;
	countryOTP[2] = '\0';

	/**check for OTP country code */
	if (!fw_info->force_reg && countryOTP[0] && countryOTP[1] &&
	    (priv->phandle->params.cntry_txpwr == CNTRY_RGPOWER_MODE)) {
		priv->phandle->country_code[0] = countryOTP[0];
		priv->phandle->country_code[1] = countryOTP[1];

		/**download OTP country code rgpower_xx.bin file*/
		if (MLAN_STATUS_SUCCESS !=
		    woal_request_country_power_table(priv, countryOTP,
						     MOAL_IOCTL_WAIT, 0)) {
			PRINTM(MCMND, "rgpower_xx downloading fail \n");
#if CFG80211_VERSION_CODE < KERNEL_VERSION(3, 9, 0)
			return -EFAULT;
#else
			return MLAN_STATUS_FAILURE;
#endif
		}
		woal_get_chan_region_cfg(priv);
	}
	/* Get 11n tx parameters from MLAN */
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_11n_cfg));
	if (req == NULL) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	cfg_11n = (mlan_ds_11n_cfg *)req->pbuf;
	cfg_11n->sub_command = MLAN_OID_11N_HTCAP_CFG;
	req->req_id = MLAN_IOCTL_11N_CFG;
	req->action = MLAN_ACT_GET;
	cfg_11n->param.htcap_cfg.hw_cap_req = MTRUE;

	ret = woal_request_ioctl(priv, req, wait_option);
	if (ret != MLAN_STATUS_SUCCESS)
		goto done;
	hw_dev_cap = cfg_11n->param.htcap_cfg.htcap;

	/* Get supported MCS sets */
	memset(req->pbuf, 0, sizeof(mlan_ds_11n_cfg));
	cfg_11n->sub_command = MLAN_OID_11N_CFG_SUPPORTED_MCS_SET;
	req->req_id = MLAN_IOCTL_11N_CFG;
	req->action = MLAN_ACT_GET;

	ret = woal_request_ioctl(priv, req, wait_option);
	if (ret != MLAN_STATUS_SUCCESS)
		goto done;

	/* Initialize parameters for 2GHz and 5GHz bands */
	if (wiphy->bands[IEEE80211_BAND_2GHZ]) {
		if (IS_CARD9098(priv->phandle->card_type) ||
		    IS_CARD9097(priv->phandle->card_type) ||
		    IS_CARDAW693(priv->phandle->card_type)) {
			mcs_supp = priv->phandle->params.antcfg & 0xf;
			if (mcs_supp != 3 && mcs_supp != 0)
				cfg_11n->param.supported_mcs_set[1] = 0;
		}
		woal_cfg80211_setup_ht_cap(
			&wiphy->bands[IEEE80211_BAND_2GHZ]->ht_cap, hw_dev_cap,
			cfg_11n->param.supported_mcs_set,
			fw_info->hw_mpdu_density);
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 6, 0)
		woal_cfg80211_setup_vht_cap(
			priv, &wiphy->bands[IEEE80211_BAND_2GHZ]->vht_cap);
#endif
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 20, 0)
		woal_cfg80211_setup_he_cap(priv,
					   wiphy->bands[IEEE80211_BAND_2GHZ]);
#endif
	}
	/* For 2.4G band only card, this shouldn't be set */
	if (wiphy->bands[IEEE80211_BAND_5GHZ]) {
		if (IS_CARD9098(priv->phandle->card_type) ||
		    IS_CARD9097(priv->phandle->card_type) ||
		    IS_CARDAW693(priv->phandle->card_type)) {
			mcs_supp = (priv->phandle->params.antcfg & 0xf00) >> 8;
			if (mcs_supp != 3 && mcs_supp != 0)
				cfg_11n->param.supported_mcs_set[1] = 0;
		}
		woal_cfg80211_setup_ht_cap(
			&wiphy->bands[IEEE80211_BAND_5GHZ]->ht_cap, hw_dev_cap,
			cfg_11n->param.supported_mcs_set,
			fw_info->hw_mpdu_density);
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 6, 0)
		woal_cfg80211_setup_vht_cap(
			priv, &wiphy->bands[IEEE80211_BAND_5GHZ]->vht_cap);
#endif
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 20, 0)
		woal_cfg80211_setup_he_cap(priv,
					   wiphy->bands[IEEE80211_BAND_5GHZ]);
#endif
	}

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 8, 0)
	if (wiphy->bands[IEEE80211_BAND_6GHZ]) {
		woal_cfg80211_setup_he_cap(priv,
					   wiphy->bands[IEEE80211_BAND_6GHZ]);
	}
#endif
	kfree(req);

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(2, 6, 38)
	/* Get antenna modes */
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_radio_cfg));
	if (req == NULL) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	radio = (mlan_ds_radio_cfg *)req->pbuf;
	radio->sub_command = MLAN_OID_ANT_CFG;
	req->req_id = MLAN_IOCTL_RADIO_CFG;
	req->action = MLAN_ACT_GET;

	ret = woal_request_ioctl(priv, req, wait_option);
	if (ret != MLAN_STATUS_SUCCESS)
		goto done;
		/* Set available antennas to wiphy */
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 17, 0)
	if (priv->phandle->params.drv_mode & DRV_MODE_DFS) {
		radio->param.ant_cfg.tx_antenna = 0x101;
		radio->param.ant_cfg.rx_antenna = 0x101;
	}
#endif
	if (IS_CARD9098(priv->phandle->card_type) ||
	    IS_CARD9097(priv->phandle->card_type) ||
	    IS_CARDIW624(priv->phandle->card_type) ||
	    IS_CARDAW693(priv->phandle->card_type)) {
		woal_cfg80211_notify_antcfg(priv, wiphy, radio);
	}
	wiphy->available_antennas_tx = radio->param.ant_cfg.tx_antenna;
	wiphy->available_antennas_rx = radio->param.ant_cfg.rx_antenna;
#endif /* CFG80211_VERSION_CODE */

	/* Set retry limit count to wiphy */
	if (GET_BSS_ROLE(priv) == MLAN_BSS_ROLE_STA) {
		if (MLAN_STATUS_SUCCESS !=
		    woal_set_get_retry(priv, MLAN_ACT_GET, wait_option,
				       &retry_count)) {
			ret = MLAN_STATUS_FAILURE;
			goto done;
		}
	}
#ifdef UAP_SUPPORT
	else {
		sys_cfg = kzalloc(sizeof(mlan_uap_bss_param), GFP_ATOMIC);
		if (!sys_cfg) {
			PRINTM(MERROR,
			       "Fail to alloc memory for mlan_uap_bss_param\n");
			ret = MLAN_STATUS_FAILURE;
			kfree(sys_cfg);
			goto done;
		}
		if (MLAN_STATUS_SUCCESS !=
		    woal_set_get_sys_config(priv, MLAN_ACT_GET, wait_option,
					    sys_cfg)) {
			ret = MLAN_STATUS_FAILURE;
			kfree(sys_cfg);
			goto done;
		}
		retry_count = sys_cfg->retry_limit;
		kfree(sys_cfg);
	}
#endif
	wiphy->retry_long = (t_u8)retry_count;
	wiphy->retry_short = (t_u8)retry_count;
	wiphy->max_scan_ie_len = MAX_IE_SIZE;

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(2, 6, 37)
	wiphy->mgmt_stypes = ieee80211_mgmt_stypes;
#endif
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(2, 6, 39)
	wiphy->max_remain_on_channel_duration = MAX_REMAIN_ON_CHANNEL_DURATION;
#endif /* KERNEL_VERSION */

	/* Set RTS threshold to wiphy */
	if (MLAN_STATUS_SUCCESS !=
	    woal_set_get_rts(priv, MLAN_ACT_GET, wait_option, &rts_thr)) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	if (rts_thr < MLAN_RTS_MIN_VALUE || rts_thr > MLAN_RTS_MAX_VALUE)
		rts_thr = MLAN_FRAG_RTS_DISABLED;
	wiphy->rts_threshold = (t_u32)rts_thr;

	/* Set fragment threshold to wiphy */
	if (MLAN_STATUS_SUCCESS !=
	    woal_set_get_frag(priv, MLAN_ACT_GET, wait_option, &frag_thr)) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	if (frag_thr < MLAN_RTS_MIN_VALUE || frag_thr > MLAN_RTS_MAX_VALUE)
		frag_thr = MLAN_FRAG_RTS_DISABLED;
	wiphy->frag_threshold = (t_u32)frag_thr;
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 0, 0)
	/* Enable multi-channel by default if multi-channel is supported */
	if (cfg80211_iface_comb_ap_sta.num_different_channels > 1) {
		if (priv->phandle->card_info->drcs &&
		    moal_extflg_isset(priv->phandle, EXT_CFG80211_DRCS)) {
			enable = 1;
			ret = woal_mc_policy_cfg(priv, &enable, wait_option,
						 MLAN_ACT_SET);
		}
	}
#endif

done:
	LEAVE();
	if (ret != MLAN_STATUS_PENDING)
		kfree(req);
	return ret;
}

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 9, 0)
/**
 * @brief Update channel flag
 *
 * @param wiphy          A pointer to wiphy structure
 *
 * @return                N/A
 */
static void woal_update_channel_flag(struct wiphy *wiphy, mlan_fw_info *fw_info)
{
	enum ieee80211_band band;
	struct ieee80211_supported_band *sband;
	int i;
	for (band = 0; band < IEEE80211_NUM_BANDS; band++) {
		sband = wiphy->bands[band];
		if (!sband)
			continue;
		if (sband->band == IEEE80211_BAND_5GHZ &&
		    fw_info->prohibit_80mhz) {
			for (i = 0; i < sband->n_channels; i++) {
				sband->channels[i].flags |=
					IEEE80211_CHAN_NO_80MHZ;
				PRINTM(MCMND, "hw_value=%d channel flag %x\n",
				       sband->channels[i].hw_value,
				       sband->channels[i].flags);
			}
		}
	}
}
#endif

/*
 * This function registers the device with CFG802.11 subsystem.
 *
 * @param priv       A pointer to moal_private
 *
 */
mlan_status woal_register_cfg80211(moal_private *priv)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	struct wiphy *wiphy;
	void *wdev_priv = NULL;
	mlan_fw_info fw_info;
	char *country = NULL, *reg_alpha2 = NULL;
	int index = 0;

	ENTER();

	memset(&fw_info, 0, sizeof(mlan_fw_info));
	woal_request_get_fw_info(priv, MOAL_IOCTL_WAIT, &fw_info);
	reg_alpha2 = priv->phandle->params.reg_alpha2;
	wiphy = wiphy_new(&woal_cfg80211_ops, sizeof(moal_handle *));
	if (!wiphy) {
		PRINTM(MERROR, "Could not allocate wiphy device\n");
		ret = MLAN_STATUS_FAILURE;
		goto err_wiphy;
	}
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
	if (moal_extflg_isset(priv->phandle, EXT_HOST_MLME)) {
		woal_cfg80211_ops.auth = woal_cfg80211_authenticate;
		woal_cfg80211_ops.assoc = woal_cfg80211_associate;
		woal_cfg80211_ops.disconnect = NULL;
		woal_cfg80211_ops.connect = NULL;
#ifdef UAP_SUPPORT
		woal_cfg80211_ops.probe_client = woal_cfg80211_probe_client;
#endif
	}
#endif
#ifdef CONFIG_PM
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 11, 0)
	if (fw_info.fw_supplicant_support)
		wiphy->wowlan = &wowlan_support_with_gtk;
	else
		wiphy->wowlan = &wowlan_support;
#else
	wiphy->wowlan.flags = WIPHY_WOWLAN_ANY | WIPHY_WOWLAN_MAGIC_PKT;
	if (fw_info.fw_supplicant_support) {
		wiphy->wowlan.flags |= WIPHY_WOWLAN_SUPPORTS_GTK_REKEY |
				       WIPHY_WOWLAN_GTK_REKEY_FAILURE;
	}
	wiphy->wowlan.n_patterns = MAX_NUM_FILTERS;
	wiphy->wowlan.pattern_min_len = 1;
	wiphy->wowlan.pattern_max_len = WOWLAN_MAX_PATTERN_LEN;
	wiphy->wowlan.max_pkt_offset = WOWLAN_MAX_OFFSET_LEN;
#endif
#endif
#endif
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 12, 0)
	wiphy->coalesce = &coalesce_support;
#endif
	wiphy->max_scan_ssids = MRVDRV_MAX_SSID_LIST_LENGTH;
	wiphy->max_scan_ie_len = MAX_IE_SIZE;
	wiphy->interface_modes = 0;
	wiphy->interface_modes = MBIT(NL80211_IFTYPE_STATION) |
				 MBIT(NL80211_IFTYPE_AP_VLAN) |
				 MBIT(NL80211_IFTYPE_AP);
	wiphy->interface_modes |= MBIT(NL80211_IFTYPE_MONITOR);

#ifdef WIFI_DIRECT_SUPPORT
#if CFG80211_VERSION_CODE >= WIFI_DIRECT_KERNEL_VERSION
	wiphy->interface_modes |=
		MBIT(NL80211_IFTYPE_P2P_GO) | MBIT(NL80211_IFTYPE_P2P_CLIENT);
#endif
#endif

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 14, 0)
	woal_register_cfg80211_vendor_command(wiphy);
#endif
	/* Make this wiphy known to this driver only */
	wiphy->privid = mrvl_wiphy_privid;

	if (!fw_info.fw_bands)
		fw_info.fw_bands = BAND_B | BAND_G;
	if (fw_info.fw_bands & BAND_A) {
		wiphy->bands[IEEE80211_BAND_5GHZ] =
			woal_setup_wiphy_bands(IEEE80211_BAND_5GHZ);
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 9, 0)
		woal_update_channel_flag(wiphy, &fw_info);
#endif
		priv->phandle->band = IEEE80211_BAND_5GHZ;
	}
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 8, 0)
	if (fw_info.fw_bands & BAND_6G) {
		wiphy->bands[IEEE80211_BAND_6GHZ] =
			woal_setup_wiphy_bands(IEEE80211_BAND_6GHZ);
	}
#endif

	/* Supported bands */
	if (fw_info.fw_bands & (BAND_B | BAND_G | BAND_GN | BAND_GAC)) {
		wiphy->bands[IEEE80211_BAND_2GHZ] =
			woal_setup_wiphy_bands(IEEE80211_BAND_2GHZ);
		/* If 2.4G enable, it will overwrite default to 2.4G*/
		priv->phandle->band = IEEE80211_BAND_2GHZ;
	}

	if (fw_info.fw_bands & BAND_A) {
		/** reduce scan time from 110ms to 80ms */
		woal_set_scan_time(priv, INIT_ACTIVE_SCAN_CHAN_TIME,
				   INIT_PASSIVE_SCAN_CHAN_TIME,
				   INIT_SPECIFIC_SCAN_CHAN_TIME);
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 12, 0)
		cfg80211_iface_comb_ap_sta.radar_detect_widths |=
			MBIT(NL80211_CHAN_WIDTH_40);
		if (fw_info.fw_bands & BAND_AAC)
			cfg80211_iface_comb_ap_sta.radar_detect_widths |=
				MBIT(NL80211_CHAN_WIDTH_80);
#endif
	} else
		woal_set_scan_time(priv, ACTIVE_SCAN_CHAN_TIME,
				   PASSIVE_SCAN_CHAN_TIME,
				   SPECIFIC_SCAN_CHAN_TIME);

	/* Initialize cipher suits */
	wiphy->cipher_suites = cfg80211_cipher_suites;
	wiphy->n_cipher_suites = ARRAY_SIZE(cfg80211_cipher_suites);
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 1, 0)
	wiphy->akm_suites = cfg80211_akm_suites;
	wiphy->n_akm_suites = ARRAY_SIZE(cfg80211_akm_suites);
#endif

#ifdef UAP_CFG80211
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 9, 0)
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
	if (!moal_extflg_isset(priv->phandle, EXT_HOST_MLME))
#endif
		wiphy->max_acl_mac_addrs = MAX_MAC_FILTER_NUM;
#endif
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 15, 0)
	if (fw_info.max_ap_assoc_sta) {
		wiphy->max_ap_assoc_sta =
			MAX(fw_info.max_ap_assoc_sta,
			    priv->phandle->params.uap_max_sta);
		PRINTM(MCMND, "Set wiphy max_ap_assoc_sta=%d\n",
		       wiphy->max_ap_assoc_sta);
	}
#endif
#endif

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 0, 0)
	cfg80211_iface_comb_ap_sta.num_different_channels = 1;
	if ((moal_extflg_isset(priv->phandle, EXT_CFG80211_DRCS) &&
	     priv->phandle->card_info->drcs) ||
	    IS_CARD9098(priv->phandle->card_type) ||
	    IS_CARDAW693(priv->phandle->card_type)) {
		cfg80211_iface_comb_ap_sta.num_different_channels = 2;
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 12, 0)
		cfg80211_iface_comb_ap_sta.radar_detect_widths = 0;
#endif
	}
	/* Initialize interface combinations */
	wiphy->iface_combinations = &cfg80211_iface_comb_ap_sta;
	wiphy->n_iface_combinations = 1;
#endif

	moal_memcpy_ext(priv->phandle, wiphy->perm_addr, priv->current_addr,
			ETH_ALEN, sizeof(wiphy->perm_addr));
	wiphy->signal_type = CFG80211_SIGNAL_TYPE_MBM;

	wiphy->flags = 0;
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(2, 6, 33)
	wiphy->flags |= WIPHY_FLAG_PS_ON_BY_DEFAULT;
	wiphy->flags |= WIPHY_FLAG_NETNS_OK;
#endif
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 3, 0)
	wiphy->flags |=
		WIPHY_FLAG_HAS_REMAIN_ON_CHANNEL | WIPHY_FLAG_OFFCHAN_TX;
	wiphy->flags |= WIPHY_FLAG_AP_UAPSD | WIPHY_FLAG_AP_PROBE_RESP_OFFLOAD;

	wiphy->probe_resp_offload = NL80211_PROBE_RESP_OFFLOAD_SUPPORT_WPS |
				    NL80211_PROBE_RESP_OFFLOAD_SUPPORT_WPS2 |
				    NL80211_PROBE_RESP_OFFLOAD_SUPPORT_P2P;

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
	if (moal_extflg_isset(priv->phandle, EXT_HOST_MLME))
		wiphy->flags |= WIPHY_FLAG_REPORTS_OBSS;
	else
#endif
		wiphy->flags |= WIPHY_FLAG_HAVE_AP_SME;
#endif
#ifdef ANDROID_KERNEL
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
	if (!moal_extflg_isset(priv->phandle, EXT_HOST_MLME))
#endif
		wiphy->flags |= WIPHY_FLAG_HAVE_AP_SME;
#endif

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 2, 0)
	if (priv->phandle->params.sched_scan) {
#if CFG80211_VERSION_CODE < KERNEL_VERSION(4, 12, 0)
		wiphy->flags |= WIPHY_FLAG_SUPPORTS_SCHED_SCAN;
#else
		wiphy->max_sched_scan_reqs = 1;
#endif
		wiphy->max_sched_scan_ssids = MRVDRV_MAX_SSID_LIST_LENGTH;
		wiphy->max_sched_scan_ie_len = MAX_IE_SIZE;
		wiphy->max_match_sets = MRVDRV_MAX_SSID_LIST_LENGTH;
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 4, 0)
		wiphy->max_sched_scan_plans = 3;
		wiphy->max_sched_scan_plan_iterations = 100;
#endif
	}
#endif
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 18, 0)
	wiphy->features |= NL80211_FEATURE_TX_POWER_INSERTION;
#endif
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 4, 0)
	wiphy->features |= NL80211_FEATURE_INACTIVITY_TIMER;
#endif
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 2, 0)
	wiphy->flags |=
		WIPHY_FLAG_SUPPORTS_TDLS | WIPHY_FLAG_TDLS_EXTERNAL_SETUP;
#endif
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 19, 0)
	wiphy->features |= NL80211_FEATURE_TDLS_CHANNEL_SWITCH;
#endif

#if CFG80211_VERSION_CODE > KERNEL_VERSION(4, 12, 14)
	/* Enable support for offloading EAPOL handshakes for WPA/WPA2. */
	if (!moal_extflg_isset(priv->phandle, EXT_HOST_MLME) &&
	    priv->phandle->card_info->embedded_supp &&
	    moal_extflg_isset(priv->phandle, EXT_CFG80211_EAPOL_OFFLOAD)) {
		wiphy_ext_feature_set(
			wiphy, NL80211_EXT_FEATURE_4WAY_HANDSHAKE_STA_PSK);
	}
#endif

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 0, 0)
#define WLAN_EXT_FEATURE_DFS_OFFLOAD 25
	if (moal_extflg_isset(priv->phandle, EXT_DFS_OFFLOAD)) {
		if (NUM_NL80211_EXT_FEATURES > WLAN_EXT_FEATURE_DFS_OFFLOAD) {
			PRINTM(MCMND,
			       "wlan: Set NL80211_EXT_FEATURE_DFS_OFFLOAD\n");
			wiphy_ext_feature_set(wiphy,
					      WLAN_EXT_FEATURE_DFS_OFFLOAD);
		}
	}
#endif

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 6, 0)
	if (moal_extflg_isset(priv->phandle, EXT_HOST_MLME))
		wiphy_ext_feature_set(wiphy, NL80211_EXT_FEATURE_RRM);
#endif
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
	if (moal_extflg_isset(priv->phandle, EXT_HOST_MLME))
		wiphy->features |= NL80211_FEATURE_SAE;
#endif
	wiphy->flags |= WIPHY_FLAG_4ADDR_AP;
	wiphy->flags |= WIPHY_FLAG_4ADDR_STATION;

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
	wiphy->features |= NL80211_FEATURE_NEED_OBSS_SCAN;
#endif

	wiphy->reg_notifier = woal_cfg80211_reg_notifier;
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 12, 0)
	wiphy->flags |= WIPHY_FLAG_HAS_CHANNEL_SWITCH;
#endif
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 16, 0)
	/* Indicate to cfg80211 that the driver can support
	 * CSA and ESCA,i.e., both types of channel switch
	 * Applications like hostapd 2.6 will append CSA IE
	 * and ECSA IE and expect the driver to advertise 2
	 * in max_num_csa_counters to successfully issue a
	 * channel switch
	 */
	wiphy->max_num_csa_counters = MAX_CSA_COUNTERS_NUM;
#endif
	wiphy->flags |= WIPHY_FLAG_CONTROL_PORT_PROTOCOL;
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 19, 0)
	wiphy->features |= NL80211_FEATURE_SCAN_RANDOM_MAC_ADDR;
	wiphy->features |= NL80211_FEATURE_SCHED_SCAN_RANDOM_MAC_ADDR;
#endif

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 19, 0)
	wiphy->features |= NL80211_FEATURE_SUPPORTS_WMM_ADMISSION;
#endif /* KERNEL_VERSION(3, 19, 0) */

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 11, 0)
	wiphy_ext_feature_set(wiphy, NL80211_EXT_FEATURE_MGMT_TX_RANDOM_TA);
	wiphy_ext_feature_set(wiphy,
			      NL80211_EXT_FEATURE_MGMT_TX_RANDOM_TA_CONNECTED);
#endif

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 8, 0)
	wiphy_ext_feature_set(wiphy, NL80211_EXT_FEATURE_SET_SCAN_DWELL);
#endif
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
	if (moal_extflg_isset(priv->phandle, EXT_HOST_MLME))
		wiphy->features |= NL80211_FEATURE_SK_TX_STATUS;
#endif

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 10, 0)
	if (fw_info.fw_beacon_prot) {
		wiphy_ext_feature_set(wiphy,
				      NL80211_EXT_FEATURE_BEACON_PROTECTION);
		wiphy_ext_feature_set(
			wiphy, NL80211_EXT_FEATURE_BEACON_PROTECTION_CLIENT);
	}
#endif
#ifdef UAP_CFG80211
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 17, 0)
	if (priv->phandle->params.drv_mode & DRV_MODE_DFS) {
		wiphy_ext_feature_set(wiphy,
				      NL80211_EXT_FEATURE_RADAR_BACKGROUND);
		woal_cfg80211_ops.set_radar_background =
			woal_cfg80211_set_radar_background;
	}
#endif
#endif

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)
	wiphy_ext_feature_set(wiphy, NL80211_EXT_FEATURE_FILS_DISCOVERY);
	wiphy_ext_feature_set(wiphy,
			      NL80211_EXT_FEATURE_UNSOL_BCAST_PROBE_RESP);
#endif
	/* Set struct moal_handle pointer in wiphy_priv */
	wdev_priv = wiphy_priv(wiphy);
	*(unsigned long *)wdev_priv = (unsigned long)priv->phandle;

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(2, 6, 39)
	set_wiphy_dev(wiphy, (struct device *)priv->phandle->hotplug_device);
#endif
	/* Set phy name*/
	for (index = 0; index < MAX_MLAN_ADAPTER; index++) {
		if (m_handle[index] == priv->phandle) {
			dev_set_name(&wiphy->dev, mwiphy_name, index);
			break;
		}
	}

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 14, 0)
	if (moal_extflg_isset(priv->phandle, EXT_BEACON_HINTS)) {
		/* REGULATORY_DISABLE_BEACON_HINTS: NO-IR flag won't be removed
		 * on chn where an AP is visible! */
		wiphy->regulatory_flags |= REGULATORY_DISABLE_BEACON_HINTS;
	}
	if (moal_extflg_isset(priv->phandle, EXT_COUNTRY_IE_IGNORE)) {
		PRINTM(MIOCTL, "Don't follow countryIE provided by AP.\n");
		wiphy->regulatory_flags |= REGULATORY_COUNTRY_IE_IGNORE;
	} else {
		PRINTM(MIOCTL, "Follow countryIE provided by AP.\n");
	}
#endif
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 19, 0)
#if CFG80211_VERSION_CODE <= KERNEL_VERSION(6, 1, 38)
	/*REGULATORY_IGNORE_STALE_KICKOFF: the regulatory core will _not_ make
	 * sure all interfaces on this wiphy reside on allowed channels. If this
	 * flag is not set, upon a regdomain change, the interfaces are given a
	 * grace period (currently 60 seconds) to disconnect or move to an
	 * allowed channel.*/
	wiphy->regulatory_flags |= REGULATORY_IGNORE_STALE_KICKOFF;
#endif
#endif

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(6, 2, 0)
#if CFG80211_VERSION_CODE <= KERNEL_VERSION(6, 3, 12)
	/*REGULATORY_IGNORE_STALE_KICKOFF: the regulatory core will _not_ make
	 * sure all interfaces on this wiphy reside on allowed channels. If this
	 * flag is not set, upon a regdomain change, the interfaces are given a
	 * grace period (currently 60 seconds) to disconnect or move to an
	 * allowed channel.*/
	wiphy->regulatory_flags |= REGULATORY_IGNORE_STALE_KICKOFF;
#endif
#endif
	memset(&priv->phandle->country_code, 0,
	       sizeof(priv->phandle->country_code));
	priv->phandle->dfs_region = NXP_DFS_UNKNOWN;

	if (reg_alpha2 && !strncmp(reg_alpha2, "99", strlen("99"))) {
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 14, 0)
		wiphy->regulatory_flags |= REGULATORY_CUSTOM_REG |
					   REGULATORY_DISABLE_BEACON_HINTS |
					   REGULATORY_COUNTRY_IE_IGNORE;
#else
		wiphy->flags |= WIPHY_FLAG_CUSTOM_REGULATORY;
#endif
		wiphy_apply_custom_regulatory(wiphy, &mrvl_regdom);
	}

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 9, 0)
	if (woal_request_extcap(priv, (t_u8 *)&priv->extended_capabilities,
				sizeof(priv->extended_capabilities)) < 0)
		PRINTM(MERROR,
		       "Failed to get driver extended capability, use default\n");
	DBG_HEXDUMP(MCMD_D, "wiphy ext cap",
		    (t_u8 *)&priv->extended_capabilities,
		    sizeof(priv->extended_capabilities));
	wiphy->extended_capabilities = (t_u8 *)&priv->extended_capabilities;
	wiphy->extended_capabilities_mask =
		(t_u8 *)&priv->extended_capabilities;
	wiphy->extended_capabilities_len = sizeof(priv->extended_capabilities);
#endif

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 10, 0)
	if (fw_info.fw_bands & BAND_6G) {
		wiphy->flags |= WIPHY_FLAG_SPLIT_SCAN_6GHZ;
		PRINTM(MIOCTL, "Enable 6E out of band discovery.\n");
	}
#endif

	ret = woal_cfg80211_init_wiphy(priv, wiphy, &fw_info, MOAL_IOCTL_WAIT);
	if (ret == MLAN_STATUS_FAILURE) {
		PRINTM(MERROR, "Wiphy device initialization failed!\n");
		ret = MLAN_STATUS_FAILURE;
		goto err_wiphy;
	}
	if (wiphy_register(wiphy) < 0) {
		PRINTM(MERROR, "Wiphy device registration failed!\n");
		ret = MLAN_STATUS_FAILURE;
		goto err_wiphy;
	}

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 0, 0)
	if (fw_info.force_reg) {
		PRINTM(MCMND, "FW region_code=%d force_reg=%d\n",
		       fw_info.region_code, fw_info.force_reg);
		country = region_code_2_string(fw_info.region_code);
		if (country) {
			moal_memcpy_ext(priv->phandle,
					priv->phandle->country_code, country, 2,
					2);
			queue_work(priv->phandle->evt_workqueue,
				   &priv->phandle->regulatory_work);
		}
	}
#endif
	if ((!reg_alpha2 || strncmp(reg_alpha2, "99", strlen("99")))

	) {
		/** we will try driver parameter first */
		if (reg_alpha2 && woal_is_valid_alpha2(reg_alpha2)) {
			PRINTM(MIOCTL, "Notify reg_alpha2 %c%c\n",
			       reg_alpha2[0], reg_alpha2[1]);
			if (!moal_extflg_isset(priv->phandle,
					       EXT_DISABLE_REGD_BY_DRIVER))
				regulatory_hint(wiphy, reg_alpha2);
		} else {
			country = region_code_2_string(fw_info.region_code);
			if (country) {
				if (fw_info.region_code != 0) {
					PRINTM(MIOCTL,
					       "Notify hw region code=%d %c%c\n",
					       fw_info.region_code, country[0],
					       country[1]);
					if (!moal_extflg_isset(
						    priv->phandle,
						    EXT_DISABLE_REGD_BY_DRIVER))
						regulatory_hint(wiphy, country);
				}
			} else
				PRINTM(MCMND,
				       "hw region code=%d not supported\n",
				       fw_info.region_code);
		}
	}
	priv->phandle->wiphy = wiphy;
	return ret;
err_wiphy:
	if (wiphy) {
		woal_cfg80211_free_bands(wiphy);
		wiphy_free(wiphy);
	}
	LEAVE();
	return ret;
}

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
/**
 * @brief Add WMM tspec TS

 * @param wiphy           A pointer to wiphy structure
 * @param dev             A pointer to net_device structure
 * @param tsid            A u8 tsid value for the TS
 * @param peer            A pointer to peer MAC address
 * @param user_prio       A u8 user priority value
 * @param admitted_time   A u16 admitted time for the TS
 *
 * @return                0 -- success, otherwise fail
 */
static int woal_cfg80211_add_tx_ts(struct wiphy *wiphy, struct net_device *dev,
				   u8 tsid, const u8 *peer, u8 user_prio,
				   u16 admitted_time)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_tx_addts_cfg *ts_cfg = NULL;
	mlan_ds_wmm_cfg *pwmm = NULL;
	int ret = 0;
	mlan_status status = MLAN_STATUS_FAILURE;
	moal_private *priv = (moal_private *)woal_get_netdev_priv(dev);

	ENTER();

	if (peer == NULL || tsid >= MAX_NUM_TID || user_prio >= MAX_NUM_TID) {
		PRINTM(MERROR, "ADDTS: Invalid parameters\n");
		ret = -EINVAL;
		goto done;
	}
	if (memcmp(peer, priv->cfg_bssid, MLAN_MAC_ADDR_LENGTH)) {
		PRINTM(MERROR, "ADDTS: Invalid peer Address\n");
		ret = -EINVAL;
		goto done;
	}

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_wmm_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	pwmm = (mlan_ds_wmm_cfg *)req->pbuf;
	pwmm->sub_command = MLAN_OID_WMM_CFG_HOST_ADDTS;
	ts_cfg = &pwmm->param.host_addts;
	ts_cfg->tsid = tsid;
	ts_cfg->user_prio = user_prio;
	ts_cfg->admitted_time = admitted_time;
	moal_memcpy_ext(priv->phandle, ts_cfg->peer, peer, sizeof(ts_cfg->peer),
			MLAN_MAC_ADDR_LENGTH);

	req->action = MLAN_ACT_SET;
	req->req_id = MLAN_IOCTL_WMM_CFG;

	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	PRINTM(MMSG,
	       "WMM AC - ADDTS tsid=%u peer=" MACSTR
	       " up=%u admitted_time=%u\n",
	       tsid, MAC2STR(peer), user_prio, admitted_time);

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 * @brief Set del tx tspec parameters

 * @param wiphy           A pointer to wiphy structure
 * @param dev             A pointer to net_device structure
 * @param params		  A pointer to ieee80211_txq_params structure
 *
 * @return                0 -- success, otherwise fail
 */
static int woal_cfg80211_del_tx_ts(struct wiphy *wiphy, struct net_device *dev,
				   u8 tsid, const u8 *peer)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_wmm_cfg *pwmm = NULL;
	mlan_ds_tx_delts_cfg *ts_cfg = NULL;
	int ret = 0;
	mlan_status status = MLAN_STATUS_FAILURE;
	moal_private *priv = (moal_private *)woal_get_netdev_priv(dev);

	ENTER();

	if (peer == NULL || tsid >= MAX_NUM_TID) {
		PRINTM(MERROR, "DELTS: Invalid parameters \n");
		ret = -EINVAL;
		goto done;
	}

	if (memcmp(peer, priv->cfg_bssid, MLAN_MAC_ADDR_LENGTH)) {
		PRINTM(MERROR, "DELTS: Invalid peer Address\n");
		ret = -EINVAL;
		goto done;
	}
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_wmm_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}
	pwmm = (mlan_ds_wmm_cfg *)req->pbuf;
	pwmm->sub_command = MLAN_OID_WMM_CFG_HOST_DELTS;
	ts_cfg = &pwmm->param.host_delts;
	ts_cfg->tsid = tsid;
	moal_memcpy_ext(priv->phandle, ts_cfg->peer, peer, sizeof(ts_cfg->peer),
			MLAN_MAC_ADDR_LENGTH);

	req->action = MLAN_ACT_SET;
	req->req_id = MLAN_IOCTL_WMM_CFG;

	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	PRINTM(MMSG, "WMM AC - DELTS tsid=%u peer=" MACSTR "\n", tsid,
	       MAC2STR(peer));
done:

	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}
#endif /* KERNEL_VERSION(3, 8, 0) */

/**
 *  @brief This function downloads the STA 6E PSD table based on
 *		   ex-AP operation mode
 *  @param priv  		pointer to moal_private
 *  @param resp_buf		pointer to assoc response buffer
 *  @param resp_len		length of assoc response
 *
 *  @return N/A
 */
void woal_dnld_sta_6e_psd_table(moal_private *priv, t_u8 *resp_buf,
				t_u32 resp_len,
				chan_band_reginfo_t *psta_reg_info)
{
	const IEEEtypes_HeOp_t *heoper_ie = NULL;
	t_u8 *ie_buffer = NULL;
	mode_psd_t *mode_psd_6G = NULL;
	t_u8 country_code[COUNTRY_CODE_LEN];
	t_u8 reg_info = 0;

	ENTER();
	/* Set the Country Code */
	country_code[0] = priv->phandle->country_code[0];
	country_code[1] = priv->phandle->country_code[1];
	country_code[2] = '\0';

	/* Memset the "mode_psd_string" */
	memset(priv->phandle->mode_psd_string, 0,
	       sizeof(priv->phandle->mode_psd_string));

	/* Parse the HE operation IE */
	if (resp_buf != NULL) {
		ie_buffer = resp_buf + sizeof(IEEEtypes_MgmtHdr_t) +
			    ASSOC_RESP_FIXED_SIZE;
		if (resp_len >=
		    (sizeof(IEEEtypes_MgmtHdr_t) + ASSOC_RESP_FIXED_SIZE))
			heoper_ie =
				(const IEEEtypes_HeOp_t *)woal_parse_ext_ie_tlv(
					ie_buffer,
					resp_len -
						((sizeof(IEEEtypes_MgmtHdr_t) +
						  ASSOC_RESP_FIXED_SIZE)),
					HE_OPERATION);
		if (heoper_ie)
			DBG_HEXDUMP(MCMD_D, "HE Oper", (const t_u8 *)heoper_ie,
				    14);

		/* Download the correct PSD table based on AP mode */
		if (heoper_ie && heoper_ie->he_op_param.he_6g_op_info_present) {
			reg_info =
				(heoper_ie->option[1] & HE_OPER_CTRL_MASK) >> 3;
			PRINTM(MCMND, "===== 6E Reg Mode: %x =====", reg_info);
		}
	}

	/* Parse the Regulatory Info */
	if (psta_reg_info != NULL) {
		reg_info = psta_reg_info->regInfo;
		PRINTM(MCMND, "===== Ex-AP 6E Reg Mode: %d =====", reg_info);
	}

	switch (reg_info) {
		/* Indoor Mode */
	case AP_MODE_IND: {
		/* Copy the initial Reg power string */
		strncpy(priv->phandle->mode_psd_string,
			"region_pwr_cfg_6G_PSD_",
			strlen("region_pwr_cfg_6G_PSD_") + 1);

		/* Prepare the 6E operation mode/psd based string */
		switch (priv->phandle->dfs_region) {
		case NXP_DFS_FCC: {
			mode_psd_6G = rmp_table_sta_6G[NXP_DFS_FCC - 1].mp_ptr;
			break;
		}
		case NXP_DFS_ETSI: {
			mode_psd_6G = rmp_table_sta_6G[NXP_DFS_ETSI - 1].mp_ptr;
			break;
		}
		default:
			PRINTM(MERROR, "Unsupported DFS Region selected\n");
			goto done;
		}
		strncat(priv->phandle->mode_psd_string,
			mode_psd_6G[AP_MODE_IND].op_mode,
			strlen(mode_psd_6G[AP_MODE_IND].op_mode));
		strncat(priv->phandle->mode_psd_string,
			mode_psd_6G[AP_MODE_IND].psd_dbm,
			strlen(mode_psd_6G[AP_MODE_IND].psd_dbm));
		break;
	}
	/* Standard Power Mode */
	case AP_MODE_SP: {
		/* Copy the initial Reg power string */
		strncpy(priv->phandle->mode_psd_string,
			"region_pwr_cfg_6G_PSD_",
			strlen("region_pwr_cfg_6G_PSD_") + 1);

		/* Prepare the 6E operation mode/psd based string */
		switch (priv->phandle->dfs_region) {
		case NXP_DFS_FCC: {
			mode_psd_6G = rmp_table_sta_6G[NXP_DFS_FCC - 1].mp_ptr;
			break;
		}
		case NXP_DFS_ETSI: {
			mode_psd_6G = rmp_table_sta_6G[NXP_DFS_ETSI - 1].mp_ptr;
			break;
		}
		default:
			PRINTM(MERROR, "Unsupported DFS Region selected\n");
			goto done;
		}
		strncat(priv->phandle->mode_psd_string,
			mode_psd_6G[AP_MODE_SP].op_mode,
			strlen(mode_psd_6G[AP_MODE_SP].op_mode));
		strncat(priv->phandle->mode_psd_string,
			mode_psd_6G[AP_MODE_SP].psd_dbm,
			strlen(mode_psd_6G[AP_MODE_SP].psd_dbm));
		break;
	}
	/* Very Low Power Mode */
	case AP_MODE_VLP: {
		/* Copy the initial Reg power string */
		strncpy(priv->phandle->mode_psd_string,
			"region_pwr_cfg_6G_PSD_",
			strlen("region_pwr_cfg_6G_PSD_") + 1);

		/* Prepare the 6E operation mode/psd based string */
		switch (priv->phandle->dfs_region) {
		case NXP_DFS_FCC: {
			mode_psd_6G = rmp_table_sta_6G[NXP_DFS_FCC - 1].mp_ptr;
			break;
		}
		case NXP_DFS_ETSI: {
			mode_psd_6G = rmp_table_sta_6G[NXP_DFS_ETSI - 1].mp_ptr;
			break;
		}
		default:
			PRINTM(MERROR, "Unsupported DFS Region selected\n");
			goto done;
		}
		strncat(priv->phandle->mode_psd_string,
			mode_psd_6G[AP_MODE_VLP].op_mode,
			strlen(mode_psd_6G[AP_MODE_VLP].op_mode));
		strncat(priv->phandle->mode_psd_string,
			mode_psd_6G[AP_MODE_VLP].psd_dbm,
			strlen(mode_psd_6G[AP_MODE_VLP].psd_dbm));
		break;
	}
	default:
		PRINTM(MERROR, "Incorrect 6E AP Operation Mode\n");
		goto done;
	}

	/* Download the ex-AP mode specific PSD table */
	PRINTM(MCMND, "DFS region = %d Opmode string = %s\n",
	       priv->phandle->dfs_region, priv->phandle->mode_psd_string);
	if (MLAN_STATUS_SUCCESS !=
	    woal_request_country_power_table(priv, country_code, MOAL_NO_WAIT,
					     1)) {
		PRINTM(MERROR, "Failed to get country power table\n");
	}
done:
	LEAVE();
	return;
}

/**
 *  @brief Download default 6E table in case of disconnect/link_loss from Ex-AP
 *
 *  @param priv   A pointer to moal_private structure
 *
 *  @return       MLAN_STATUS_SUCCESS/MLAN_STATUS_PENDING
 *                  -- success, otherwise fail
 */
mlan_status woal_dnld_default_6e_psd_table(moal_private *priv)
{
	t_u8 country_code[COUNTRY_CODE_LEN];
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	/* Set the Country Code */
	country_code[0] = priv->phandle->country_code[0];
	country_code[1] = priv->phandle->country_code[1];
	country_code[2] = '\0';

	memset(priv->phandle->mode_psd_string, 0,
	       sizeof(priv->phandle->mode_psd_string));

	/* Copy the initial Reg power string */
	strncpy(priv->phandle->mode_psd_string, "region_pwr_cfg_6G",
		strlen("region_pwr_cfg_6G") + 1);

	PRINTM(MINFO, "Opmode string = %s\n", priv->phandle->mode_psd_string);

	if (MLAN_STATUS_SUCCESS !=
	    woal_request_country_power_table(priv, country_code, MOAL_NO_WAIT,
					     1)) {
		PRINTM(MERROR, "Failed to get country power table\n");
		status = MLAN_STATUS_FAILURE;
	}

	LEAVE();
	return status;
}
