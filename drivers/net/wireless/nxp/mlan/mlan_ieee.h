/** @file mlan_ieee.h
 *
 *  @brief This file contains IEEE information element related
 *  definitions used in MLAN and MOAL module.
 *
 *
 *  Copyright 2008-2024 NXP
 *
 *  This software file (the File) is distributed by NXP
 *  under the terms of the GNU General Public License Version 2, June 1991
 *  (the License).  You may use, redistribute and/or modify the File in
 *  accordance with the terms and conditions of the License, a copy of which
 *  is available by writing to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or on the
 *  worldwide web at http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt.
 *
 *  THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE
 *  ARE EXPRESSLY DISCLAIMED.  The License provides additional details about
 *  this warranty disclaimer.
 *
 */

/******************************************************
Change log:
    11/03/2008: initial version
******************************************************/

#ifndef _MLAN_IEEE_H_
#define _MLAN_IEEE_H_

/* center frequency description */
struct center_freq_desc {
	t_u8 pri_chan;
	t_u8 ch_40;
	t_u8 ch_80;
	t_u8 ch_160;
};

/* List of 5GHZ channels */
static const struct center_freq_desc center_freq_idx_map_5g[] = {
	{.pri_chan = 36, .ch_40 = 38, .ch_80 = 42, .ch_160 = 50},
	{.pri_chan = 40, .ch_40 = 38, .ch_80 = 42, .ch_160 = 50},
	{.pri_chan = 44, .ch_40 = 46, .ch_80 = 42, .ch_160 = 50},
	{.pri_chan = 48, .ch_40 = 46, .ch_80 = 42, .ch_160 = 50},
	{.pri_chan = 52, .ch_40 = 54, .ch_80 = 58, .ch_160 = 50},
	{.pri_chan = 56, .ch_40 = 54, .ch_80 = 58, .ch_160 = 50},
	{.pri_chan = 60, .ch_40 = 62, .ch_80 = 58, .ch_160 = 50},
	{.pri_chan = 64, .ch_40 = 62, .ch_80 = 58, .ch_160 = 50},
	{.pri_chan = 68, .ch_40 = 70, .ch_80 = 74, .ch_160 = 0},
	{.pri_chan = 72, .ch_40 = 70, .ch_80 = 74, .ch_160 = 0},
	{.pri_chan = 76, .ch_40 = 78, .ch_80 = 74, .ch_160 = 0},
	{.pri_chan = 80, .ch_40 = 78, .ch_80 = 74, .ch_160 = 0},
	{.pri_chan = 84, .ch_40 = 86, .ch_80 = 90, .ch_160 = 0},
	{.pri_chan = 88, .ch_40 = 86, .ch_80 = 90, .ch_160 = 0},
	{.pri_chan = 92, .ch_40 = 94, .ch_80 = 90, .ch_160 = 0},
	{.pri_chan = 96, .ch_40 = 94, .ch_80 = 90, .ch_160 = 0},
	{.pri_chan = 100, .ch_40 = 102, .ch_80 = 106, .ch_160 = 114},
	{.pri_chan = 104, .ch_40 = 102, .ch_80 = 106, .ch_160 = 114},
	{.pri_chan = 108, .ch_40 = 110, .ch_80 = 106, .ch_160 = 114},
	{.pri_chan = 112, .ch_40 = 110, .ch_80 = 106, .ch_160 = 114},
	{.pri_chan = 116, .ch_40 = 118, .ch_80 = 122, .ch_160 = 114},
	{.pri_chan = 120, .ch_40 = 118, .ch_80 = 122, .ch_160 = 114},
	{.pri_chan = 124, .ch_40 = 126, .ch_80 = 122, .ch_160 = 114},
	{.pri_chan = 128, .ch_40 = 126, .ch_80 = 122, .ch_160 = 114},
	{.pri_chan = 132, .ch_40 = 134, .ch_80 = 138, .ch_160 = 0},
	{.pri_chan = 136, .ch_40 = 134, .ch_80 = 138, .ch_160 = 0},
	{.pri_chan = 140, .ch_40 = 142, .ch_80 = 138, .ch_160 = 0},
	{.pri_chan = 144, .ch_40 = 142, .ch_80 = 138, .ch_160 = 0},
	{.pri_chan = 149, .ch_40 = 151, .ch_80 = 155, .ch_160 = 163},
	{.pri_chan = 153, .ch_40 = 151, .ch_80 = 155, .ch_160 = 163},
	{.pri_chan = 157, .ch_40 = 159, .ch_80 = 155, .ch_160 = 163},
	{.pri_chan = 161, .ch_40 = 159, .ch_80 = 155, .ch_160 = 163},
	{.pri_chan = 165, .ch_40 = 167, .ch_80 = 171, .ch_160 = 163},
	{.pri_chan = 169, .ch_40 = 167, .ch_80 = 171, .ch_160 = 163},
	{.pri_chan = 173, .ch_40 = 175, .ch_80 = 171, .ch_160 = 163},
	{.pri_chan = 177, .ch_40 = 175, .ch_80 = 171, .ch_160 = 163},
	{.pri_chan = 184, .ch_40 = 186, .ch_80 = 190, .ch_160 = 0},
	{.pri_chan = 188, .ch_40 = 186, .ch_80 = 190, .ch_160 = 0},
	{.pri_chan = 192, .ch_40 = 194, .ch_80 = 190, .ch_160 = 0},
	{.pri_chan = 196, .ch_40 = 194, .ch_80 = 190, .ch_160 = 0},
	{.pri_chan = 0, .ch_40 = 42 /* terminator with default cfreq */}};

/* List of 6GHZ channels */
static const struct center_freq_desc center_freq_idx_map_6g[] = {
	{.pri_chan = 1, .ch_40 = 3, .ch_80 = 7, .ch_160 = 15},
	{.pri_chan = 5, .ch_40 = 3, .ch_80 = 7, .ch_160 = 15},
	{.pri_chan = 9, .ch_40 = 11, .ch_80 = 7, .ch_160 = 15},
	{.pri_chan = 13, .ch_40 = 11, .ch_80 = 7, .ch_160 = 15},
	{.pri_chan = 17, .ch_40 = 19, .ch_80 = 23, .ch_160 = 15},
	{.pri_chan = 21, .ch_40 = 19, .ch_80 = 23, .ch_160 = 15},
	{.pri_chan = 25, .ch_40 = 27, .ch_80 = 23, .ch_160 = 15},
	{.pri_chan = 29, .ch_40 = 27, .ch_80 = 23, .ch_160 = 15},
	{.pri_chan = 33, .ch_40 = 35, .ch_80 = 39, .ch_160 = 47},
	{.pri_chan = 37, .ch_40 = 35, .ch_80 = 39, .ch_160 = 47},
	{.pri_chan = 41, .ch_40 = 43, .ch_80 = 39, .ch_160 = 47},
	{.pri_chan = 45, .ch_40 = 43, .ch_80 = 39, .ch_160 = 47},
	{.pri_chan = 49, .ch_40 = 51, .ch_80 = 55, .ch_160 = 47},
	{.pri_chan = 53, .ch_40 = 51, .ch_80 = 55, .ch_160 = 47},
	{.pri_chan = 57, .ch_40 = 59, .ch_80 = 55, .ch_160 = 47},
	{.pri_chan = 61, .ch_40 = 59, .ch_80 = 55, .ch_160 = 47},
	{.pri_chan = 65, .ch_40 = 67, .ch_80 = 71, .ch_160 = 79},
	{.pri_chan = 69, .ch_40 = 67, .ch_80 = 71, .ch_160 = 79},
	{.pri_chan = 73, .ch_40 = 75, .ch_80 = 71, .ch_160 = 79},
	{.pri_chan = 77, .ch_40 = 75, .ch_80 = 71, .ch_160 = 79},
	{.pri_chan = 81, .ch_40 = 83, .ch_80 = 87, .ch_160 = 79},
	{.pri_chan = 85, .ch_40 = 83, .ch_80 = 87, .ch_160 = 79},
	{.pri_chan = 89, .ch_40 = 91, .ch_80 = 87, .ch_160 = 79},
	{.pri_chan = 93, .ch_40 = 91, .ch_80 = 87, .ch_160 = 79},
	{.pri_chan = 97, .ch_40 = 99, .ch_80 = 103, .ch_160 = 111},
	{.pri_chan = 101, .ch_40 = 99, .ch_80 = 103, .ch_160 = 111},
	{.pri_chan = 105, .ch_40 = 107, .ch_80 = 103, .ch_160 = 111},
	{.pri_chan = 109, .ch_40 = 107, .ch_80 = 103, .ch_160 = 111},
	{.pri_chan = 113, .ch_40 = 115, .ch_80 = 119, .ch_160 = 111},
	{.pri_chan = 117, .ch_40 = 115, .ch_80 = 119, .ch_160 = 111},
	{.pri_chan = 121, .ch_40 = 123, .ch_80 = 119, .ch_160 = 111},
	{.pri_chan = 125, .ch_40 = 123, .ch_80 = 119, .ch_160 = 111},
	{.pri_chan = 129, .ch_40 = 131, .ch_80 = 135, .ch_160 = 143},
	{.pri_chan = 133, .ch_40 = 131, .ch_80 = 135, .ch_160 = 143},
	{.pri_chan = 137, .ch_40 = 139, .ch_80 = 135, .ch_160 = 143},
	{.pri_chan = 141, .ch_40 = 139, .ch_80 = 135, .ch_160 = 143},
	{.pri_chan = 145, .ch_40 = 147, .ch_80 = 151, .ch_160 = 143},
	{.pri_chan = 149, .ch_40 = 147, .ch_80 = 151, .ch_160 = 143},
	{.pri_chan = 153, .ch_40 = 155, .ch_80 = 151, .ch_160 = 143},
	{.pri_chan = 157, .ch_40 = 155, .ch_80 = 151, .ch_160 = 143},
	{.pri_chan = 161, .ch_40 = 163, .ch_80 = 167, .ch_160 = 175},
	{.pri_chan = 165, .ch_40 = 163, .ch_80 = 167, .ch_160 = 175},
	{.pri_chan = 169, .ch_40 = 171, .ch_80 = 167, .ch_160 = 175},
	{.pri_chan = 173, .ch_40 = 171, .ch_80 = 167, .ch_160 = 175},
	{.pri_chan = 177, .ch_40 = 179, .ch_80 = 183, .ch_160 = 175},
	{.pri_chan = 181, .ch_40 = 179, .ch_80 = 183, .ch_160 = 175},
	{.pri_chan = 185, .ch_40 = 187, .ch_80 = 183, .ch_160 = 175},
	{.pri_chan = 189, .ch_40 = 187, .ch_80 = 183, .ch_160 = 175},
	{.pri_chan = 193, .ch_40 = 195, .ch_80 = 199, .ch_160 = 207},
	{.pri_chan = 197, .ch_40 = 195, .ch_80 = 199, .ch_160 = 207},
	{.pri_chan = 201, .ch_40 = 203, .ch_80 = 199, .ch_160 = 207},
	{.pri_chan = 205, .ch_40 = 203, .ch_80 = 199, .ch_160 = 207},
	{.pri_chan = 209, .ch_40 = 211, .ch_80 = 215, .ch_160 = 207},
	{.pri_chan = 213, .ch_40 = 211, .ch_80 = 215, .ch_160 = 207},
	{.pri_chan = 217, .ch_40 = 219, .ch_80 = 215, .ch_160 = 207},
	{.pri_chan = 221, .ch_40 = 219, .ch_80 = 215, .ch_160 = 207},
	{.pri_chan = 225, .ch_40 = 227, .ch_80 = 0, .ch_160 = 0},
	{.pri_chan = 229, .ch_40 = 227, .ch_80 = 0, .ch_160 = 0},
	{.pri_chan = 233, .ch_40 = 0, .ch_80 = 0, .ch_160 = 0},
	{.pri_chan = 0, .ch_40 = 37 /* terminator with default cfreq */}};

/** WLAN header size */
#define IEEE80211_HEADER_SIZE 24

/** FIX IES size in beacon buffer */
#define WLAN_802_11_FIXED_IE_SIZE 12
/** WLAN supported rates */
#define WLAN_SUPPORTED_RATES 14

/** WLAN supported rates extension */
#define WLAN_SUPPORTED_RATES_EXT 60

/** Enumeration definition*/
/** WLAN_802_11_NETWORK_TYPE */
typedef enum _WLAN_802_11_NETWORK_TYPE {
	Wlan802_11FH,
	Wlan802_11DS,
	/* Defined as upper bound*/
	Wlan802_11NetworkTypeMax
} WLAN_802_11_NETWORK_TYPE;

#ifdef BIG_ENDIAN_SUPPORT
/** Frame control: Type Mgmt frame */
#define IEEE80211_FC_MGMT_FRAME_TYPE_MASK 0x0c00
/** Frame control: SubType Mgmt frame */
#define IEEE80211_GET_FC_MGMT_FRAME_SUBTYPE(fc) (((fc)&0xF000) >> 12)
#else
/** Frame control: Type Mgmt frame */
#define IEEE80211_FC_MGMT_FRAME_TYPE_MASK 0x000C
/** Frame control: SubType Mgmt frame */
#define IEEE80211_GET_FC_MGMT_FRAME_SUBTYPE(fc) (((fc)&0x00F0) >> 4)
#endif

#ifdef PRAGMA_PACK
#pragma pack(push, 1)
#endif

/* Reason codes */
#define IEEE_80211_REASONCODE_UNSPECIFIED 1

typedef enum _IEEEtypes_Ext_ElementId_e {
	HE_CAPABILITY = 35,
	HE_OPERATION = 36,
	MU_EDCA_PARAM_SET = 38,
	MBSSID_CONFIG = 55,
	NON_INHERITANCE = 56,
	HE_6G_CAPABILITY = 59
} IEEEtypes_Ext_ElementId_e;

/** IEEE Type definitions  */
typedef MLAN_PACK_START enum _IEEEtypes_ElementId_e {
	SSID = 0,
	SUPPORTED_RATES = 1,

	FH_PARAM_SET = 2,
	DS_PARAM_SET = 3,
	CF_PARAM_SET = 4,

	COUNTRY_INFO = 7,
	POWER_CONSTRAINT = 32,
	POWER_CAPABILITY = 33,
	TPC_REQUEST = 34,
	TPC_REPORT = 35,
	CHANNEL_SWITCH_ANN = 37,
	EXTEND_CHANNEL_SWITCH_ANN = 60,
	QUIET = 40,
	MEASUREMENT_REQUEST = 38,
	MEASUREMENT_REPORT = 39,
	SUPPORTED_CHANNELS = 36,
	REGULATORY_CLASS = 59,
	HT_CAPABILITY = 45,
	QOS_INFO = 46,
	HT_OPERATION = 61,
	MULTI_BSSID = 71,
	BSSCO_2040 = 72,
	OVERLAPBSSSCANPARAM = 74,
	NONTX_BSSID_CAP = 83,
	MBSSID_INDEX = 85,
	EXT_CAPABILITY = 127,
	LINK_ID = 101,
	/*IEEE802.11r*/
	MOBILITY_DOMAIN = 54,
	FAST_BSS_TRANSITION = 55,
	TIMEOUT_INTERVAL = 56,
	RIC = 57,
	QOS_MAPPING = 110,
	VHT_CAPABILITY = 191,
	VHT_OPERATION = 192,
	EXT_BSS_LOAD = 193,
	BW_CHANNEL_SWITCH = 194,
	VHT_TX_POWER_ENV = 195,
	EXT_POWER_CONSTR = 196,
	AID_INFO = 197,
	QUIET_CHAN = 198,
	OPER_MODE_NTF = 199,
	RNR = 201,
	FILS_SESSION = 210,
	FILS_PMKID_LIST = 211,
	FILS_IP_REQ = 212,
	FILS_IP_RESP = 213,
	FILS_KEY_AUTH = 214,
	FILS_KEY_DELIVERY = 215,
	FILS_INDICATION = 240,

	ERP_INFO = 42,

	EXTENDED_SUPPORTED_RATES = 50,

	VENDOR_SPECIFIC_221 = 221,
	WMM_IE = VENDOR_SPECIFIC_221,

	WPS_IE = VENDOR_SPECIFIC_221,
	/* WPA */
	WPA_IE = VENDOR_SPECIFIC_221,
	/* WPA2 */
	RSN_IE = 48,
	VS_IE = VENDOR_SPECIFIC_221,
	WAPI_IE = 68,
	FRAGMENT = 242,
	RSNX_IE = 244,
	EXTENSION = 255
} MLAN_PACK_END IEEEtypes_ElementId_e;

/** IEEE IE header */
typedef MLAN_PACK_START struct _IEEEtypes_Header_t {
	/** Element ID */
	t_u8 element_id;
	/** Length */
	t_u8 len;
} MLAN_PACK_END IEEEtypes_Header_t, *pIEEEtypes_Header_t;

/** Vendor specific IE header */
typedef MLAN_PACK_START struct _IEEEtypes_VendorHeader_t {
	/** Element ID */
	t_u8 element_id;
	/** Length */
	t_u8 len;
	/** OUI */
	t_u8 oui[3];
	/** OUI type */
	t_u8 oui_type;
	/** OUI subtype */
	t_u8 oui_subtype;
	/** Version */
	t_u8 version;
} MLAN_PACK_END IEEEtypes_VendorHeader_t, *pIEEEtypes_VendorHeader_t;

/** Vendor specific IE */
typedef MLAN_PACK_START struct _IEEEtypes_VendorSpecific_t {
	/** Vendor specific IE header */
	IEEEtypes_VendorHeader_t vend_hdr;
	/** IE Max - size of previous fields */
	t_u8 data[IEEE_MAX_IE_SIZE - sizeof(IEEEtypes_VendorHeader_t)];
} MLAN_PACK_END IEEEtypes_VendorSpecific_t, *pIEEEtypes_VendorSpecific_t;

/** IEEE IE */
typedef MLAN_PACK_START struct _IEEEtypes_Generic_t {
	/** Generic IE header */
	IEEEtypes_Header_t ieee_hdr;
	/** IE Max - size of previous fields */
	t_u8 data[IEEE_MAX_IE_SIZE - sizeof(IEEEtypes_Header_t)];
} MLAN_PACK_END IEEEtypes_Generic_t, *pIEEEtypes_Generic_t;

#define MEASURE_TYPE_CLI 8
#define MEASURE_TYPE_LOCATION_CIVIC 9

/** Measurement Report IE */
typedef MLAN_PACK_START struct _IEEEtypes_MeasurementReport_t {
	/** Generic IE header */
	IEEEtypes_Header_t ieee_hdr;
	/** Measurement Token */
	t_u8 ms_token;
	/** Measurement Report Mode */
	t_u8 ms_rp_mode;
	/** Measurement Type, value in MEASURE_TYPE_XXX */
	t_u8 ms_type;
	/** variable */
	t_u8 variable[];
} MLAN_PACK_END IEEEtypes_MeasurementReport_t;

/** Report */
typedef MLAN_PACK_START struct _IEEEtypes_Report_t {
	/** Subelement ID */
	t_u8 subelement_id;
	/** length */
	t_u8 length;
	/** variable */
	t_u8 variable[];
} MLAN_PACK_END IEEEtypes_Report_t;

/**ft capability policy*/
typedef MLAN_PACK_START struct _IEEEtypes_FtCapPolicy_t {
#ifdef BIG_ENDIAN_SUPPORT
	/** Reserved */
	t_u8 reserved : 6;
	/** RIC support */
	t_u8 ric : 1;
	/** FT over the DS capable */
	t_u8 ft_over_ds : 1;
#else
	/** FT over the DS capable */
	t_u8 ft_over_ds : 1;
	/** RIC support */
	t_u8 ric : 1;
	/** Reserved */
	t_u8 reserved : 6;
#endif
} MLAN_PACK_END IEEEtypes_FtCapPolicy_t;

/** Mobility domain IE */
typedef MLAN_PACK_START struct _IEEEtypes_MobilityDomain_t {
	/** Generic IE header */
	IEEEtypes_Header_t ieee_hdr;
	/** Mobility Domain ID */
	t_u16 mdid;
	/** FT Capability policy */
	t_u8 ft_cap;
} MLAN_PACK_END IEEEtypes_MobilityDomain_t;

/**FT MIC Control*/
typedef MLAN_PACK_START struct _IEEEtypes_FT_MICControl_t {
	/** reserved */
	t_u8 reserved;
	/** element count */
	t_u8 element_count;
} MLAN_PACK_END IEEEtypes_FT_MICControl_t;

/** FTIE MIC LEN */
#define FTIE_MIC_LEN 16

/**FT IE*/
typedef MLAN_PACK_START struct _IEEEtypes_FastBssTransElement_t {
	/** Generic IE header */
	IEEEtypes_Header_t ieee_hdr;
	/** mic control */
	IEEEtypes_FT_MICControl_t mic_control;
	/** mic */
	t_u8 mic[FTIE_MIC_LEN];
	/** ANonce */
	t_u8 a_nonce[32];
	/** SNonce */
	t_u8 s_nonce[32];
	/** sub element */
	t_u8 sub_element[];
} MLAN_PACK_END IEEEtypes_FastBssTransElement_t;

/*Category for FT*/
#define FT_CATEGORY 6
/** FT ACTION request */
#define FT_ACTION_REQUEST 1
/** FT ACTION response */
#define FT_ACTION_RESPONSE 2

/*FT response and FT ack*/
typedef MLAN_PACK_START struct {
	/** category */
	t_u8 category;
	/** action */
	t_u8 action;
	/** sta address */
	t_u8 sta_addr[MLAN_MAC_ADDR_LENGTH];
	/** target ap address */
	t_u8 target_ap_addr[MLAN_MAC_ADDR_LENGTH];
	/** status code */
	t_u16 status_code;
	/** varible */
	t_u8 variable[];
} MLAN_PACK_END IEEEtypes_Ft_action_response;

/**FT request */
typedef MLAN_PACK_START struct {
	/** category */
	t_u8 category;
	/** action */
	t_u8 action;
	/** sta address */
	t_u8 sta_addr[MLAN_MAC_ADDR_LENGTH];
	/** target ap address */
	t_u8 target_ap_addr[MLAN_MAC_ADDR_LENGTH];
	/** varible */
	t_u8 variable[];
} MLAN_PACK_END IEEEtypes_Ft_action_request;

/** auth frame body*/
typedef MLAN_PACK_START struct {
	/** auth alg */
	t_u16 auth_alg;
	/** auth transaction */
	t_u16 auth_transaction;
	/** status code */
	t_u16 status_code;
	/** variable */
	t_u8 variable[];
} MLAN_PACK_END IEEEtypes_Auth_framebody;

/** associate request frame */
typedef MLAN_PACK_START struct {
	t_u16 capab_info;
	t_u16 listen_interval;
	/** followed by SSID and Supported rates */
	t_u8 variablep[];
} MLAN_PACK_END IEEEtypes_assoc_req;

/** Assoc Request */
#define SUBTYPE_ASSOC_REQUEST 0
/** Assoc Response */
#define SUBTYPE_ASSOC_RESP 1
/** ReAssoc Request */
#define SUBTYPE_REASSOC_REQUEST 2
/** ReAssoc Request */
#define SUBTYPE_REASSOC_RESP 3
/** Probe Resp */
#define SUBTYPE_PROBE_RESP 5
/** Disassoc Request */
#define SUBTYPE_DISASSOC 10
/** Auth Request */
#define SUBTYPE_AUTH 11
/** Deauth Request */
#define SUBTYPE_DEAUTH 12
/** Action frame */
#define SUBTYPE_ACTION 13
/** beacon */
#define SUBTYPE_BEACON 8

/*Mgmt frame*/
typedef MLAN_PACK_START struct {
	/** frame control */
	t_u16 frame_control;
	/** duration */
	t_u16 duration;
	/** dest address */
	t_u8 da[MLAN_MAC_ADDR_LENGTH];
	/** source address */
	t_u8 sa[MLAN_MAC_ADDR_LENGTH];
	/** bssid */
	t_u8 bssid[MLAN_MAC_ADDR_LENGTH];
	/** seq control */
	t_u16 seq_ctrl;
	/** address 4 */
	t_u8 addr4[MLAN_MAC_ADDR_LENGTH];
	union {
		IEEEtypes_Auth_framebody auth;
		IEEEtypes_assoc_req assoc_req;
		IEEEtypes_Ft_action_response ft_resp;
		IEEEtypes_Ft_action_request ft_req;
	} u;
} MLAN_PACK_END IEEE80211_MGMT;

/** TLV header */
typedef MLAN_PACK_START struct _TLV_Generic_t {
	/** Type */
	t_u16 type;
	/** Length */
	t_u16 len;
} MLAN_PACK_END TLV_Generic_t, *pTLV_Generic_t;

/** Capability information mask */
#define CAPINFO_MASK (~(MBIT(15) | MBIT(14) | MBIT(11) | MBIT(9)))

/** Capability Bit Map*/
#ifdef BIG_ENDIAN_SUPPORT
typedef MLAN_PACK_START struct _IEEEtypes_CapInfo_t {
	t_u8 rsrvd1 : 2;
	t_u8 dsss_ofdm : 1;
	t_u8 radio_measurement : 1;
	t_u8 rsvrd2 : 1;
	t_u8 short_slot_time : 1;
	t_u8 rsrvd3 : 1;
	t_u8 spectrum_mgmt : 1;
	t_u8 chan_agility : 1;
	t_u8 pbcc : 1;
	t_u8 short_preamble : 1;
	t_u8 privacy : 1;
	t_u8 cf_poll_rqst : 1;
	t_u8 cf_pollable : 1;
	t_u8 rsrvd4 : 1;
	t_u8 ess : 1;
} MLAN_PACK_END IEEEtypes_CapInfo_t, *pIEEEtypes_CapInfo_t;
#else
typedef MLAN_PACK_START struct _IEEEtypes_CapInfo_t {
	/** Capability Bit Map : ESS */
	t_u8 ess : 1;
	t_u8 rsrvd4 : 1;
	/** Capability Bit Map : CF pollable */
	t_u8 cf_pollable : 1;
	/** Capability Bit Map : CF poll request */
	t_u8 cf_poll_rqst : 1;
	/** Capability Bit Map : privacy */
	t_u8 privacy : 1;
	/** Capability Bit Map : Short preamble */
	t_u8 short_preamble : 1;
	/** Capability Bit Map : PBCC */
	t_u8 pbcc : 1;
	/** Capability Bit Map : Channel agility */
	t_u8 chan_agility : 1;
	/** Capability Bit Map : Spectrum management */
	t_u8 spectrum_mgmt : 1;
	/** Capability Bit Map : Reserved */
	t_u8 rsrvd3 : 1;
	/** Capability Bit Map : Short slot time */
	t_u8 short_slot_time : 1;
	/** Capability Bit Map : APSD */
	t_u8 Apsd : 1;
	/** Capability Bit Map : Reserved */
	t_u8 rsvrd2 : 1;
	/** Capability Bit Map : DSS OFDM */
	t_u8 dsss_ofdm : 1;
	/** Capability Bit Map : Reserved */
	t_u8 rsrvd1 : 2;
} MLAN_PACK_END IEEEtypes_CapInfo_t, *pIEEEtypes_CapInfo_t;
#endif /* BIG_ENDIAN_SUPPORT */

/** IEEEtypes_Ssid_t */
typedef MLAN_PACK_START struct _IEEEtypes_Ssid_t {
	/** SSID: Element ID */
	t_u8 element_id;
	/** SSID : Length */
	t_u8 len;
	/** ssid */
	t_u8 ssid[MLAN_MAX_SSID_LENGTH];
} MLAN_PACK_END IEEEtypes_Ssid_t, *pIEEEtypes_Ssid_t;

/** IEEEtypes_CfParamSet_t */
typedef MLAN_PACK_START struct _IEEEtypes_CfParamSet_t {
	/** CF peremeter : Element ID */
	t_u8 element_id;
	/** CF peremeter : Length */
	t_u8 len;
	/** CF peremeter : Count */
	t_u8 cfp_cnt;
	/** CF peremeter : Period */
	t_u8 cfp_period;
	/** CF peremeter : Maximum duration */
	t_u16 cfp_max_duration;
	/** CF peremeter : Remaining duration */
	t_u16 cfp_duration_remaining;
} MLAN_PACK_END IEEEtypes_CfParamSet_t, *pIEEEtypes_CfParamSet_t;

/** IEEEtypes_SsParamSet_t */
typedef MLAN_PACK_START union _IEEEtypes_SsParamSet_t {
	/** SS parameter : CF parameter set */
	IEEEtypes_CfParamSet_t cf_param_set;
} MLAN_PACK_END IEEEtypes_SsParamSet_t, *pIEEEtypes_SsParamSet_t;

/** IEEEtypes_FhParamSet_t */
typedef MLAN_PACK_START struct _IEEEtypes_FhParamSet_t {
	/** FH parameter : Element ID */
	t_u8 element_id;
	/** FH parameter : Length */
	t_u8 len;
	/** FH parameter : Dwell time in milliseconds */
	t_u16 dwell_time;
	/** FH parameter : Hop set */
	t_u8 hop_set;
	/** FH parameter : Hop pattern */
	t_u8 hop_pattern;
	/** FH parameter : Hop index */
	t_u8 hop_index;
} MLAN_PACK_END IEEEtypes_FhParamSet_t, *pIEEEtypes_FhParamSet_t;

/** IEEEtypes_DsParamSet_t */
typedef MLAN_PACK_START struct _IEEEtypes_DsParamSet_t {
	/** DS parameter : Element ID */
	t_u8 element_id;
	/** DS parameter : Length */
	t_u8 len;
	/** DS parameter : Current channel */
	t_u8 current_chan;
} MLAN_PACK_END IEEEtypes_DsParamSet_t, *pIEEEtypes_DsParamSet_t;

/** IEEEtypes_PhyParamSet_t */
typedef MLAN_PACK_START union _IEEEtypes_PhyParamSet_t {
	/** FH parameter set */
	IEEEtypes_FhParamSet_t fh_param_set;
	/** DS parameter set */
	IEEEtypes_DsParamSet_t ds_param_set;
} MLAN_PACK_END IEEEtypes_PhyParamSet_t, *pIEEEtypes_PhyParamSet_t;

/** IEEEtypes_ERPInfo_t */
typedef MLAN_PACK_START struct _IEEEtypes_ERPInfo_t {
	/** Element ID */
	t_u8 element_id;
	/** Length */
	t_u8 len;
	/** ERP flags */
	t_u8 erp_flags;
} MLAN_PACK_END IEEEtypes_ERPInfo_t, *pIEEEtypes_ERPInfo_t;

/** IEEEtypes_AId_t */
typedef t_u16 IEEEtypes_AId_t;

/** IEEEtypes_StatusCode_t */
typedef t_u16 IEEEtypes_StatusCode_t;

/** Fixed size in assoc_resp */
#define ASSOC_RESP_FIXED_SIZE 6

/** IEEEtypes_SeqCtl_t */
typedef MLAN_PACK_START struct _IEEEtypes_SeqCtl_t {
	/** Fragment Number */
	t_u16 FragNum : 4;
	/** Sequence Number */
	t_u16 SeqNum : 12;
} MLAN_PACK_END IEEEtypes_SeqCtl_t;

/** IEEEtypes_MgmtHdr_t */
typedef MLAN_PACK_START struct _IEEEtypes_MgmtHdr_t {
	/** FrmCtl*/
	t_u16 FrmCtl;
	/** Duration*/
	t_u16 Duration;
	/** Destination Addr*/
	t_u8 DestAddr[6];
	/** Source Addr*/
	t_u8 SrcAddr[6];
	/** BSSID */
	t_u8 BssId[6];
	/** IEEEtypes_SeqCtl_t */
	IEEEtypes_SeqCtl_t SeqCtl;
} MLAN_PACK_END IEEEtypes_MgmtHdr_t;

/** IEEEtypes_AssocRsp_t */
typedef MLAN_PACK_START struct _IEEEtypes_AssocRsp_t {
	/** Capability information */
	IEEEtypes_CapInfo_t capability;
	/** Association response status code */
	IEEEtypes_StatusCode_t status_code;
	/** Association ID */
	IEEEtypes_AId_t a_id;
	/** IE data buffer */
	t_u8 ie_buffer[];
} MLAN_PACK_END IEEEtypes_AssocRsp_t, *pIEEEtypes_AssocRsp_t;

/** 802.11 supported rates */
typedef t_u8 WLAN_802_11_RATES[WLAN_SUPPORTED_RATES];

/** cipher TKIP */
#define WPA_CIPHER_TKIP 2
/** cipher AES */
#define WPA_CIPHER_AES_CCM 4
/** AKM: 8021x */
#define RSN_AKM_8021X 1
/** AKM: PSK */
#define RSN_AKM_PSK 2
/** AKM: PSK SHA256 */
#define RSN_AKM_PSK_SHA256 6

/** AKM: PSK SHA256 */
#define RSN_AKM_SAE 8
/** AKM: PSK SHA256 */
#define RSN_AKM_OWE 18

#if defined(STA_SUPPORT)
/** Pairwise Cipher Suite length */
#define PAIRWISE_CIPHER_SUITE_LEN 4
/** AKM Suite length */
#define AKM_SUITE_LEN 4
/** MFPC bit in RSN capability */
#define MFPC_BIT 7
/** MFPR bit in RSN capability */
#define MFPR_BIT 6
/** PMF ORing mask */
#define PMF_MASK 0x00c0
#endif

/** wpa_suite_t */
typedef MLAN_PACK_START struct _wpa_suite_t {
	/** OUI */
	t_u8 oui[3];
	/** tyep */
	t_u8 type;
} MLAN_PACK_END wpa_suite, wpa_suite_mcast_t;

/** wpa_suite_ucast_t */
typedef MLAN_PACK_START struct {
	/* count */
	t_u16 count;
	/** wpa_suite list */
	wpa_suite list[];
} MLAN_PACK_END wpa_suite_ucast_t, wpa_suite_auth_key_mgmt_t;

/** IEEEtypes_Rsn_t */
typedef MLAN_PACK_START struct _IEEEtypes_Rsn_t {
	/** Rsn : Element ID */
	t_u8 element_id;
	/** Rsn : Length */
	t_u8 len;
	/** Rsn : version */
	t_u16 version;
	/** Rsn : group cipher */
	wpa_suite_mcast_t group_cipher;
	/** Rsn : pairwise cipher */
	wpa_suite_ucast_t pairwise_cipher;
} MLAN_PACK_END IEEEtypes_Rsn_t, *pIEEEtypes_Rsn_t;

/** IEEEtypes_Wpa_t */
typedef MLAN_PACK_START struct _IEEEtypes_Wpa_t {
	/** Wpa : Element ID */
	t_u8 element_id;
	/** Wpa : Length */
	t_u8 len;
	/** Wpa : oui */
	t_u8 oui[4];
	/** version */
	t_u16 version;
	/** Wpa : group cipher */
	wpa_suite_mcast_t group_cipher;
	/** Wpa : pairwise cipher */
	wpa_suite_ucast_t pairwise_cipher;
} MLAN_PACK_END IEEEtypes_Wpa_t, *pIEEEtypes_Wpa_t;

/** Data structure of WMM QoS information */
typedef MLAN_PACK_START struct _IEEEtypes_WmmQosInfo_t {
#ifdef BIG_ENDIAN_SUPPORT
	/** QoS UAPSD */
	t_u8 qos_uapsd : 1;
	/** Reserved */
	t_u8 reserved : 3;
	/** Parameter set count */
	t_u8 para_set_count : 4;
#else
	/** Parameter set count */
	t_u8 para_set_count : 4;
	/** Reserved */
	t_u8 reserved : 3;
	/** QoS UAPSD */
	t_u8 qos_uapsd : 1;
#endif /* BIG_ENDIAN_SUPPORT */
} MLAN_PACK_END IEEEtypes_WmmQosInfo_t, *pIEEEtypes_WmmQosInfo_t;

/** Data structure of WMM Aci/Aifsn */
typedef MLAN_PACK_START struct _IEEEtypes_WmmAciAifsn_t {
#ifdef BIG_ENDIAN_SUPPORT
	/** Reserved */
	t_u8 reserved : 1;
	/** Aci */
	t_u8 aci : 2;
	/** Acm */
	t_u8 acm : 1;
	/** Aifsn */
	t_u8 aifsn : 4;
#else
	/** Aifsn */
	t_u8 aifsn : 4;
	/** Acm */
	t_u8 acm : 1;
	/** Aci */
	t_u8 aci : 2;
	/** Reserved */
	t_u8 reserved : 1;
#endif /* BIG_ENDIAN_SUPPORT */
} MLAN_PACK_END IEEEtypes_WmmAciAifsn_t, *pIEEEtypes_WmmAciAifsn_t;

/** Data structure of WMM ECW */
typedef MLAN_PACK_START struct _IEEEtypes_WmmEcw_t {
#ifdef BIG_ENDIAN_SUPPORT
	/** Maximum Ecw */
	t_u8 ecw_max : 4;
	/** Minimum Ecw */
	t_u8 ecw_min : 4;
#else
	/** Minimum Ecw */
	t_u8 ecw_min : 4;
	/** Maximum Ecw */
	t_u8 ecw_max : 4;
#endif /* BIG_ENDIAN_SUPPORT */
} MLAN_PACK_END IEEEtypes_WmmEcw_t, *pIEEEtypes_WmmEcw_t;

/** Data structure of WMM AC parameters  */
typedef MLAN_PACK_START struct _IEEEtypes_WmmAcParameters_t {
	IEEEtypes_WmmAciAifsn_t aci_aifsn; /**< AciAifSn */
	IEEEtypes_WmmEcw_t ecw; /**< Ecw */
	t_u16 tx_op_limit; /**< Tx op limit */
} MLAN_PACK_END IEEEtypes_WmmAcParameters_t, *pIEEEtypes_WmmAcParameters_t;

/** Data structure of WMM Info IE  */
typedef MLAN_PACK_START struct _IEEEtypes_WmmInfo_t {
	/**
	 * WMM Info IE - Vendor Specific Header:
	 *   element_id  [221/0xdd]
	 *   Len         [7]
	 *   Oui         [00:50:f2]
	 *   OuiType     [2]
	 *   OuiSubType  [0]
	 *   Version     [1]
	 */
	IEEEtypes_VendorHeader_t vend_hdr;

	/** QoS information */
	IEEEtypes_WmmQosInfo_t qos_info;

} MLAN_PACK_END IEEEtypes_WmmInfo_t, *pIEEEtypes_WmmInfo_t;

/** Data structure of WMM parameter IE  */
typedef MLAN_PACK_START struct _IEEEtypes_WmmParameter_t {
	/**
	 * WMM Parameter IE - Vendor Specific Header:
	 *   element_id  [221/0xdd]
	 *   Len         [24]
	 *   Oui         [00:50:f2]
	 *   OuiType     [2]
	 *   OuiSubType  [1]
	 *   Version     [1]
	 */
	IEEEtypes_VendorHeader_t vend_hdr;

	/** QoS information */
	IEEEtypes_WmmQosInfo_t qos_info;
	/** Reserved */
	t_u8 reserved;

	/** AC Parameters Record WMM_AC_BE, WMM_AC_BK, WMM_AC_VI, WMM_AC_VO */
	IEEEtypes_WmmAcParameters_t ac_params[MAX_AC_QUEUES];
} MLAN_PACK_END IEEEtypes_WmmParameter_t, *pIEEEtypes_WmmParameter_t;

/** Enumerator for TSPEC direction */
typedef MLAN_PACK_START enum _IEEEtypes_WMM_TSPEC_TS_Info_Direction_e {

	TSPEC_DIR_UPLINK = 0,
	TSPEC_DIR_DOWNLINK = 1,
	/* 2 is a reserved value */
	TSPEC_DIR_BIDIRECT = 3,

} MLAN_PACK_END IEEEtypes_WMM_TSPEC_TS_Info_Direction_e;

/** Enumerator for TSPEC PSB */
typedef MLAN_PACK_START enum _IEEEtypes_WMM_TSPEC_TS_Info_PSB_e {

	TSPEC_PSB_LEGACY = 0,
	TSPEC_PSB_TRIG = 1,

} MLAN_PACK_END IEEEtypes_WMM_TSPEC_TS_Info_PSB_e;

/** Enumerator for TSPEC Ack Policy */
typedef MLAN_PACK_START enum _IEEEtypes_WMM_TSPEC_TS_Info_AckPolicy_e {

	TSPEC_ACKPOLICY_NORMAL = 0,
	TSPEC_ACKPOLICY_NOACK = 1,
	/* 2 is reserved */
	TSPEC_ACKPOLICY_BLOCKACK = 3,

} MLAN_PACK_END IEEEtypes_WMM_TSPEC_TS_Info_AckPolicy_e;

/** Enumerator for TSPEC Trafffice type */
typedef MLAN_PACK_START enum _IEEEtypes_WMM_TSPEC_TS_TRAFFIC_TYPE_e {

	TSPEC_TRAFFIC_APERIODIC = 0,
	TSPEC_TRAFFIC_PERIODIC = 1,

} MLAN_PACK_END IEEEtypes_WMM_TSPEC_TS_TRAFFIC_TYPE_e;

/** Data structure of WMM TSPEC information */
typedef MLAN_PACK_START struct {
#ifdef BIG_ENDIAN_SUPPORT
	t_u8 Reserved17_23 : 7; /* ! Reserved */
	t_u8 Schedule : 1;
	IEEEtypes_WMM_TSPEC_TS_Info_AckPolicy_e AckPolicy : 2;
	t_u8 UserPri : 3; /* ! 802.1d User Priority */
	// IEEEtypes_WMM_TSPEC_TS_Info_PSB_e PowerSaveBehavior : 1; /*
	// !Legacy/Trigg*/
	t_u8 PowerSaveBehavior : 1;
	t_u8 Aggregation : 1; /* ! Reserved */
	t_u8 AccessPolicy2 : 1; /* ! */
	t_u8 AccessPolicy1 : 1; /* ! */
	IEEEtypes_WMM_TSPEC_TS_Info_Direction_e Direction : 2;
	t_u8 TID : 4; /* ! Unique identifier */
	// IEEEtypes_WMM_TSPEC_TS_TRAFFIC_TYPE_e TrafficType : 1;
	t_u8 TrafficType : 1;
#else
	// IEEEtypes_WMM_TSPEC_TS_TRAFFIC_TYPE_e TrafficType : 1;
	t_u8 TrafficType : 1;
	t_u8 TID : 4; /* ! Unique identifier */
	IEEEtypes_WMM_TSPEC_TS_Info_Direction_e Direction : 2;
	t_u8 AccessPolicy1 : 1; /* ! */
	t_u8 AccessPolicy2 : 1; /* ! */
	t_u8 Aggregation : 1; /* ! Reserved */
	// IEEEtypes_WMM_TSPEC_TS_Info_PSB_e PowerSaveBehavior : 1; /* !
	// Legacy/Trigg*/
	t_u8 PowerSaveBehavior : 1;
	t_u8 UserPri : 3; /* ! 802.1d User Priority */
	IEEEtypes_WMM_TSPEC_TS_Info_AckPolicy_e AckPolicy : 2;
	t_u8 Schedule : 1;
	t_u8 Reserved17_23 : 7; /* ! Reserved */
#endif
} MLAN_PACK_END IEEEtypes_WMM_TSPEC_TS_Info_t;

/** Data structure of WMM TSPEC Nominal Size */
typedef MLAN_PACK_START struct {
#ifdef BIG_ENDIAN_SUPPORT
	t_u16 Fixed : 1; /* ! 1: Fixed size given in Size, 0: Var, size is
			    nominal */
	t_u16 Size : 15; /* ! Nominal size in octets */
#else
	t_u16 Size : 15; /* ! Nominal size in octets */
	t_u16 Fixed : 1; /* ! 1: Fixed size given in Size, 0: Var, size is
			    nominal */
#endif
} MLAN_PACK_END IEEEtypes_WMM_TSPEC_NomMSDUSize_t;

/** Data structure of WMM TSPEC SBWA */
typedef MLAN_PACK_START struct {
#ifdef BIG_ENDIAN_SUPPORT
	t_u16 Whole : 3; /* ! Whole portion */
	t_u16 Fractional : 13; /* ! Fractional portion */
#else
	t_u16 Fractional : 13; /* ! Fractional portion */
	t_u16 Whole : 3; /* ! Whole portion */
#endif
} MLAN_PACK_END IEEEtypes_WMM_TSPEC_SBWA;

/** Data structure of WMM TSPEC Body */
typedef MLAN_PACK_START struct {
	IEEEtypes_WMM_TSPEC_TS_Info_t TSInfo;
	IEEEtypes_WMM_TSPEC_NomMSDUSize_t NomMSDUSize;
	t_u16 MaximumMSDUSize;
	t_u32 MinServiceInterval;
	t_u32 MaxServiceInterval;
	t_u32 InactivityInterval;
	t_u32 SuspensionInterval;
	t_u32 ServiceStartTime;
	t_u32 MinimumDataRate;
	t_u32 MeanDataRate;
	t_u32 PeakDataRate;
	t_u32 MaxBurstSize;
	t_u32 DelayBound;
	t_u32 MinPHYRate;
	IEEEtypes_WMM_TSPEC_SBWA SurplusBWAllowance;
	t_u16 MediumTime;
} MLAN_PACK_END IEEEtypes_WMM_TSPEC_Body_t;

/** Data structure of WMM TSPEC all elements */
typedef MLAN_PACK_START struct {
	t_u8 ElementId;
	t_u8 Len;
	t_u8 OuiType[4]; /* 00:50:f2:02 */
	t_u8 OuiSubType; /* 01 */
	t_u8 Version;

	IEEEtypes_WMM_TSPEC_Body_t TspecBody;

} MLAN_PACK_END IEEEtypes_WMM_TSPEC_t;

/** WMM Action Category values */
typedef MLAN_PACK_START enum _IEEEtypes_ActionCategory_e {

	IEEE_MGMT_ACTION_CATEGORY_SPECTRUM_MGMT = 0,
	IEEE_MGMT_ACTION_CATEGORY_QOS = 1,
	IEEE_MGMT_ACTION_CATEGORY_DLS = 2,
	IEEE_MGMT_ACTION_CATEGORY_BLOCK_ACK = 3,
	IEEE_MGMT_ACTION_CATEGORY_PUBLIC = 4,
	IEEE_MGMT_ACTION_CATEGORY_RADIO_RSRC = 5,
	IEEE_MGMT_ACTION_CATEGORY_FAST_BSS_TRANS = 6,
	IEEE_MGMT_ACTION_CATEGORY_HT = 7,

	IEEE_MGMT_ACTION_CATEGORY_WNM = 10,
	IEEE_MGMT_ACTION_CATEGORY_UNPROTECT_WNM = 11,

	IEEE_MGMT_ACTION_CATEGORY_WMM_TSPEC = 17

} MLAN_PACK_END IEEEtypes_ActionCategory_e;

/** WMM TSPEC operations */
typedef MLAN_PACK_START enum _IEEEtypes_WMM_Tspec_Action_e {

	TSPEC_ACTION_CODE_ADDTS_REQ = 0,
	TSPEC_ACTION_CODE_ADDTS_RSP = 1,
	TSPEC_ACTION_CODE_DELTS = 2,

} MLAN_PACK_END IEEEtypes_WMM_Tspec_Action_e;

/** NAN SDF vendor oui size */
#define NAN_SDF_VENDOR_SIZE 4
/** NAN service descriptor attribute offset */
#define NAN_SDA_OFFSET 5
/** NAN service control type offset */
#define NAN_SRVC_CTRL_OFFSET 11
/** Service control field */
#define NAN_SRV_CTRL_TYPE_MASK (BIT(0) | BIT(1))
/** NAN service control type */
#define NAN_PUBLISH 0
#define NAN_FOLLOW_UP 2

/** NAN Attribute ID list */
typedef MLAN_PACK_START enum _Nan_AttrId_e {
	NAN_ATTR_SDA = 0x03
} MLAN_PACK_END Nan_AttrId_e;

/** Public Action Codes */
typedef MLAN_PACK_START enum _IEEEtypes_Public_ActionCategory_e {
	IEEE_PUBLIC_ACTION_CATEGORY_VENDOR_SPECIFIC = 9
} MLAN_PACK_END IEEEtypes_Public_ActionCategory_e;

/** WMM TSPEC Category Action Base */
typedef MLAN_PACK_START struct {
	IEEEtypes_ActionCategory_e category;
	IEEEtypes_WMM_Tspec_Action_e action;
	t_u8 dialogToken;

} MLAN_PACK_END IEEEtypes_WMM_Tspec_Action_Base_Tspec_t;

/** WMM TSPEC AddTS request structure */
typedef MLAN_PACK_START struct {
	IEEEtypes_WMM_Tspec_Action_Base_Tspec_t tspecAct;
	t_u8 statusCode;
	IEEEtypes_WMM_TSPEC_t tspecIE;

	/* Place holder for additional elements after the TSPEC */
	t_u8 subElem[256];

} MLAN_PACK_END IEEEtypes_Action_WMM_AddTsReq_t;

/** WMM TSPEC AddTS response structure */
typedef MLAN_PACK_START struct {
	IEEEtypes_WMM_Tspec_Action_Base_Tspec_t tspecAct;
	t_u8 statusCode;
	IEEEtypes_WMM_TSPEC_t tspecIE;

	/* Place holder for additional elements after the TSPEC */
	t_u8 subElem[256];

} MLAN_PACK_END IEEEtypes_Action_WMM_AddTsRsp_t;

/** WMM TSPEC DelTS structure */
typedef MLAN_PACK_START struct {
	IEEEtypes_WMM_Tspec_Action_Base_Tspec_t tspecAct;
	t_u8 reasonCode;
	IEEEtypes_WMM_TSPEC_t tspecIE;

} MLAN_PACK_END IEEEtypes_Action_WMM_DelTs_t;

/** union of WMM TSPEC structures */
typedef MLAN_PACK_START union {
	IEEEtypes_WMM_Tspec_Action_Base_Tspec_t tspecAct;

	IEEEtypes_Action_WMM_AddTsReq_t addTsReq;
	IEEEtypes_Action_WMM_AddTsRsp_t addTsRsp;
	IEEEtypes_Action_WMM_DelTs_t delTs;

} MLAN_PACK_END IEEEtypes_Action_WMMAC_t;

/** union of WMM TSPEC & Action category */
typedef MLAN_PACK_START union {
	IEEEtypes_ActionCategory_e category;

	IEEEtypes_Action_WMMAC_t wmmAc;

} MLAN_PACK_END IEEEtypes_ActionFrame_t;

/** Data structure for subband set */
typedef MLAN_PACK_START struct _IEEEtypes_SubbandSet_t {
	/** First channel */
	t_u8 first_chan;
	/** Number of channels */
	t_u8 no_of_chan;
	/** Maximum Tx power in dBm */
	t_u8 max_tx_pwr;
} MLAN_PACK_END IEEEtypes_SubbandSet_t, *pIEEEtypes_SubbandSet_t;

#ifdef STA_SUPPORT
/** Data structure for Country IE */
typedef MLAN_PACK_START struct _IEEEtypes_CountryInfoSet_t {
	/** Element ID */
	t_u8 element_id;
	/** Length */
	t_u8 len;
	/** Country code */
	t_u8 country_code[COUNTRY_CODE_LEN];
	/** Set of subbands */
	IEEEtypes_SubbandSet_t sub_band[];
} MLAN_PACK_END IEEEtypes_CountryInfoSet_t, *pIEEEtypes_CountryInfoSet_t;

/** Data structure for Country IE full set */
typedef MLAN_PACK_START struct _IEEEtypes_CountryInfoFullSet_t {
	/** Element ID */
	t_u8 element_id;
	/** Length */
	t_u8 len;
	/** Country code */
	t_u8 country_code[COUNTRY_CODE_LEN];
	/** Set of subbands */
	IEEEtypes_SubbandSet_t sub_band[MRVDRV_MAX_SUBBAND_802_11D];
} MLAN_PACK_END IEEEtypes_CountryInfoFullSet_t,
	*pIEEEtypes_CountryInfoFullSet_t;

#endif /* STA_SUPPORT */

/** Data structure for Link ID */
typedef MLAN_PACK_START struct _IEEEtypes_LinkIDElement_t {
	/** Element ID */
	t_u8 element_id;
	/** Length */
	t_u8 len;
	/** bssid */
	t_u8 bssid[MLAN_MAC_ADDR_LENGTH];
	/** initial sta address */
	t_u8 init_sta[MLAN_MAC_ADDR_LENGTH];
	/** respose sta address */
	t_u8 resp_sta[MLAN_MAC_ADDR_LENGTH];
} MLAN_PACK_END IEEEtypes_LinkIDElement_t, *pIEEEtypes_LinkIDElement_t;

/** HT Capabilities Data */
typedef struct MLAN_PACK_START _HTCap_t {
	/** HT Capabilities Info field */
	t_u16 ht_cap_info;
	/** A-MPDU Parameters field */
	t_u8 ampdu_param;
	/** Supported MCS Set field */
	t_u8 supported_mcs_set[16];
	/** HT Extended Capabilities field */
	t_u16 ht_ext_cap;
	/** Transmit Beamforming Capabilities field */
	t_u32 tx_bf_cap;
	/** Antenna Selection Capability field */
	t_u8 asel;
} MLAN_PACK_END HTCap_t, *pHTCap_t;

/** HT Information Data */
typedef struct MLAN_PACK_START _HTInfo_t {
	/** Primary channel */
	t_u8 pri_chan;
	/** Field 2 */
	t_u8 field2;
	/** Field 3 */
	t_u16 field3;
	/** Field 4 */
	t_u16 field4;
	/** Bitmap indicating MCSs supported by all HT STAs in the BSS */
	t_u8 basic_mcs_set[16];
} MLAN_PACK_END HTInfo_t, *pHTInfo_t;

/** 20/40 BSS Coexistence Data */
typedef struct MLAN_PACK_START _BSSCo2040_t {
	/** 20/40 BSS Coexistence value */
	t_u8 bss_co_2040_value;
} MLAN_PACK_END BSSCo2040_t, *pBSSCo2040_t;

#define MAX_DSCP_EXCEPTION_NUM 21
/** DSCP Range */
typedef struct MLAN_PACK_START _DSCP_Exception_t {
	/* DSCP value 0 to 63 or ff */
	t_u8 dscp_value;
	/* user priority 0-7*/
	t_u8 user_priority;
} MLAN_PACK_END DSCP_Exception_t, *pDSCP_Exception_t;

/** DSCP Range */
typedef struct MLAN_PACK_START _DSCP_Range_t {
	/* DSCP low value */
	t_u8 dscp_low_value;
	/* DSCP high value */
	t_u8 dscp_high_value;
} MLAN_PACK_END DSCP_Range_t, *pDSCP_Range_t;

/** Overlapping BSS Scan Parameters Data */
typedef struct MLAN_PACK_START _OverlapBSSScanParam_t {
	/** OBSS Scan Passive Dwell in milliseconds */
	t_u16 obss_scan_passive_dwell;
	/** OBSS Scan Active Dwell in milliseconds */
	t_u16 obss_scan_active_dwell;
	/** BSS Channel Width Trigger Scan Interval in seconds */
	t_u16 bss_chan_width_trigger_scan_int;
	/** OBSS Scan Passive Total Per Channel */
	t_u16 obss_scan_passive_total;
	/** OBSS Scan Active Total Per Channel */
	t_u16 obss_scan_active_total;
	/** BSS Width Channel Transition Delay Factor */
	t_u16 bss_width_chan_trans_delay;
	/** OBSS Scan Activity Threshold */
	t_u16 obss_scan_active_threshold;
} MLAN_PACK_END OBSSScanParam_t, *pOBSSScanParam_t;

/** HT Capabilities IE */
typedef MLAN_PACK_START struct _IEEEtypes_HTCap_t {
	/** Generic IE header */
	IEEEtypes_Header_t ieee_hdr;
	/** HTCap struct */
	HTCap_t ht_cap;
} MLAN_PACK_END IEEEtypes_HTCap_t, *pIEEEtypes_HTCap_t;

/** HT Information IE */
typedef MLAN_PACK_START struct _IEEEtypes_HTInfo_t {
	/** Generic IE header */
	IEEEtypes_Header_t ieee_hdr;
	/** HTInfo struct */
	HTInfo_t ht_info;
} MLAN_PACK_END IEEEtypes_HTInfo_t, *pIEEEtypes_HTInfo_t;

/** the AP which send the multi_bssid IE */
#define MULTI_BSSID_AP 1
/** the AP which don't send beacon */
#define MULTI_BSSID_SUB_AP 2
/** IEEEtypes_NotxBssCap_t */
typedef MLAN_PACK_START struct _IEEEtypes_NotxBssCap_t {
	/** Nontransmitted BSSID Capability: Element ID */
	t_u8 element_id;
	/** Nontransmitted BSSID Capability : Length */
	t_u8 len;
	/** capability */
	t_u16 cap;
} MLAN_PACK_END IEEEtypes_NotxBssCap_t, *pIEEEtypes_NotxBssCap_t;

/** Multi BSSID IE */
typedef MLAN_PACK_START struct _IEEEtypes_MultiBSSIDIndex_t {
	/** Generic IE header */
	IEEEtypes_Header_t ieee_hdr;
	/** BSSID Index */
	t_u8 bssid_index;
	/** DTIM Period (Optional, not Present in ProbeRsp) */
	t_u8 dtim_period;
	/** DTIM Count (Optional, not Present in ProbeRsp) */
	t_u8 dtim_count;
} MLAN_PACK_END IEEEtypes_MultiBSSIDIndex_t, *pIEEEtypes_MultiBSSIDIndex_t;

/** NonTransmitted BSSID Profile Subelement IE */
/** SUBID for IEEEtypes_NonTransBSSIDCap_t */
#define NONTRANS_BSSID_PROFILE_SUBELEM_ID 0

/** NonTransmitted BSSID Capability IE */
typedef MLAN_PACK_START struct _IEEEtypes_NonTransBSSIDProfile_t {
	/** Generic IE header */
	IEEEtypes_Header_t ieee_hdr;
	t_u8 profile_data[];
} MLAN_PACK_END IEEEtypes_NonTransBSSIDProfile_t,
	*pIEEEtypes_NonTransBSSIDProfile_t;

/** Multi BSSID IE */
typedef MLAN_PACK_START struct _IEEEtypes_MultiBSSID_t {
	/** Generic IE header */
	IEEEtypes_Header_t ieee_hdr;
	/** Max BSSID Indicator */
	t_u8 max_bssid_indicator;
	/** Optional Subelement data*/
	t_u8 sub_elem_data[];
} MLAN_PACK_END IEEEtypes_MultiBSSID_t, *pIEEEtypes_MultiBSSID_t;

/** Multi BSSID Configuration IE */
typedef MLAN_PACK_START struct _IEEEtypes_MBSSID_Config_t {
	/** Generic IE header */
	IEEEtypes_Header_t ieee_hdr;
	/** Element id extension */
	t_u8 ext_id;
	/** BSSID Count */
	t_u8 bssid_cnt;
	/** Full Set Rx Periodicity */
	t_u8 fs_rx_periodicity;
} MLAN_PACK_END IEEEtypes_MBSSID_Config_t, *pIEEEtypes_MBSSID_Config_t;
/** 20/40 BSS Coexistence IE */
typedef MLAN_PACK_START struct _IEEEtypes_2040BSSCo_t {
	/** Generic IE header */
	IEEEtypes_Header_t ieee_hdr;
	/** BSSCo2040_t struct */
	BSSCo2040_t bss_co_2040;
} MLAN_PACK_END IEEEtypes_2040BSSCo_t, *pIEEEtypes_2040BSSCo_t;

/** Extended Capabilities IE */
typedef MLAN_PACK_START struct _IEEEtypes_ExtCap_t {
	/** Generic IE header */
	IEEEtypes_Header_t ieee_hdr;
	/** ExtCap_t struct */
	ExtCap_t ext_cap;
} MLAN_PACK_END IEEEtypes_ExtCap_t, *pIEEEtypes_ExtCap_t;

/** Overlapping BSS Scan Parameters IE */
typedef MLAN_PACK_START struct _IEEEtypes_OverlapBSSScanParam_t {
	/** Generic IE header */
	IEEEtypes_Header_t ieee_hdr;
	/** OBSSScanParam_t struct */
	OBSSScanParam_t obss_scan_param;
} MLAN_PACK_END IEEEtypes_OverlapBSSScanParam_t,
	*pIEEEtypes_OverlapBSSScanParam_t;

/** VHT MCS rate set field, refer to 802.11ac */
typedef MLAN_PACK_START struct _VHT_MCS_set {
	t_u16 rx_mcs_map;
	t_u16 rx_max_rate; /* bit 29-31 reserved */
	t_u16 tx_mcs_map;
	t_u16 tx_max_rate; /* bit 61-63 reserved */
} MLAN_PACK_END VHT_MCS_set_t, *pVHT_MCS_set_t;

/** VHT Capabilities info field, reference 802.11ac D1.4 p89 */
typedef MLAN_PACK_START struct _VHT_capa {
#if 0
#ifdef BIG_ENDIAN_SUPPORT
    t_u8 mpdu_max_len:2;
    t_u8 chan_width:2;
    t_u8 rx_LDPC:1;
    t_u8 sgi_80:1;
    t_u8 sgi_160:1;
    t_u8 tx_STBC:1;
    t_u8 rx_STBC:3;
    t_u8 SU_beamformer_capa:1;
    t_u8 SU_beamformee_capa:1;
    t_u8 beamformer_ante_num:3;
    t_u8 sounding_dim_num:3;
    t_u8 MU_beamformer_capa:1;
    t_u8 MU_beamformee_capa:1;
    t_u8 VHT_TXOP_ps:1;
    t_u8 HTC_VHT_capa:1;
    t_u8 max_ampdu_len:3;
    t_u8 link_apapt_capa:2;
    t_u8 reserved_1:4;
#else
    t_u8 reserved_1:4;
    t_u8 link_apapt_capa:2;
    t_u8 max_ampdu_len:3;
    t_u8 HTC_VHT_capa:1;
    t_u8 VHT_TXOP_ps:1;
    t_u8 MU_beamformee_capa:1;
    t_u8 MU_beamformer_capa:1;
    t_u8 sounding_dim_num:3;
    t_u8 beamformer_ante_num:3;
    t_u8 SU_beamformee_capa:1;
    t_u8 SU_beamformer_capa:1;
    t_u8 rx_STBC:3;
    t_u8 tx_STBC:1;
    t_u8 sgi_160:1;
    t_u8 sgi_80:1;
    t_u8 rx_LDPC:1;
    t_u8 chan_width:2;
    t_u8 mpdu_max_len:2;
#endif /* BIG_ENDIAN_SUPPORT */
#endif
	t_u32 vht_cap_info;
	VHT_MCS_set_t mcs_sets;
} MLAN_PACK_END VHT_capa_t, *pVHT_capa_t;

/** VHT Capabilities IE */
typedef MLAN_PACK_START struct _IEEEtypes_VHTCap_t {
	/** Generic IE header */
	IEEEtypes_Header_t ieee_hdr;
	VHT_capa_t vht_cap;
} MLAN_PACK_END IEEEtypes_VHTCap_t, *pIEEEtypes_VHTCap_t;

#define VHT_CAP_CHWD_80MHZ 0
#define VHT_CAP_CHWD_160MHZ 1
#define VHT_CAP_CHWD_80_80MHZ 2

/** VHT Operations IE */
typedef MLAN_PACK_START struct _IEEEtypes_VHTOprat_t {
	/** Generic IE header */
	IEEEtypes_Header_t ieee_hdr;
	t_u8 chan_width;
	t_u8 chan_center_freq_1;
	t_u8 chan_center_freq_2;
	/** Basic MCS set map, each 2 bits stands for a Nss */
	t_u16 basic_MCS_map;
} MLAN_PACK_END IEEEtypes_VHTOprat_t, *pIEEEtypes_VHTOprat_t;

#define VHT_OPER_CHWD_20_40MHZ 0
#define VHT_OPER_CHWD_80MHZ 1
#define VHT_OPER_CHWD_160MHZ 2
#define VHT_OPER_CHWD_80_80MHZ 3

/** VHT Transmit Power Envelope IE */
typedef MLAN_PACK_START struct _IEEEtypes_VHTtxpower_t {
	/** Generic IE header */
	IEEEtypes_Header_t ieee_hdr;
	t_u8 max_tx_power;
	t_u8 chan_center_freq;
	t_u8 chan_width;
} MLAN_PACK_END IEEEtypes_VHTtxpower_t, *pIEEEtypes_VHTtxpower_t;

/** Extended Power Constraint IE */
typedef MLAN_PACK_START struct _IEEEtypes_ExtPwerCons_t {
	/** Generic IE header */
	IEEEtypes_Header_t ieee_hdr;
	/** channel width */
	t_u8 chan_width;
	/** local power constraint */
	t_u8 local_power_cons;
} MLAN_PACK_END IEEEtypes_ExtPwerCons_t, *pIEEEtypes_ExtPwerCons_t;

/** Extended BSS Load IE */
typedef MLAN_PACK_START struct _IEEEtypes_ExtBSSload_t {
	/** Generic IE header */
	IEEEtypes_Header_t ieee_hdr;
	t_u8 MU_MIMO_capa_count;
	t_u8 stream_underutilization;
	t_u8 VHT40_util;
	t_u8 VHT80_util;
	t_u8 VHT160_util;
} MLAN_PACK_END IEEEtypes_ExtBSSload_t, *pIEEEtypes_ExtBSSload_t;

/** Quiet Channel IE */
typedef MLAN_PACK_START struct _IEEEtypes_QuietChan_t {
	/** Generic IE header */
	IEEEtypes_Header_t ieee_hdr;
	t_u8 AP_quiet_mode;
	t_u8 quiet_count;
	t_u8 quiet_period;
	t_u16 quiet_dur;
	t_u16 quiet_offset;
} MLAN_PACK_END IEEEtypes_QuietChan_t, *pIEEEtypes_QuietChan_t;

/** Wide Bandwidth Channel Switch IE */
typedef MLAN_PACK_START struct _IEEEtypes_BWSwitch_t {
	/** Generic IE header */
	IEEEtypes_Header_t ieee_hdr;
	t_u8 new_chan_width;
	t_u8 new_chan_center_freq_1;
	t_u8 new_chan_center_freq_2;
} MLAN_PACK_END IEEEtypes_BWSwitch_t, *pIEEEtypes_BWSwitch_t;

/** AID IE */
typedef MLAN_PACK_START struct _IEEEtypes_AID_t {
	/** Generic IE header */
	IEEEtypes_Header_t ieee_hdr;
	/** AID number */
	t_u16 AID;
} MLAN_PACK_END IEEEtypes_AID_t, *pIEEEtypes_AID_t;

/** Operating Mode Notificaton IE */
typedef MLAN_PACK_START struct _IEEEtypes_OperModeNtf_t {
	/** Generic IE header */
	IEEEtypes_Header_t ieee_hdr;
	/** Operating Mode */
	t_u8 oper_mode;
} MLAN_PACK_END IEEEtypes_OperModeNtf_t, *pIEEEtypes_OperModeNtf_t;

typedef MLAN_PACK_START struct _IEEEtypes_Extension_t {
	/** Generic IE header */
	IEEEtypes_Header_t ieee_hdr;
	/** Element id extension */
	t_u8 ext_id;
	/** payload */
	t_u8 data[];
} MLAN_PACK_END IEEEtypes_Extension_t, *pIEEEtypes_Extension_t;

typedef MLAN_PACK_START struct _IEEEtypes_HeMcsMap_t {
#ifdef BIG_ENDIAN_SUPPORT
	/** Max HE-MAC for 8 SS */
	t_u8 max_mcs_8ss : 2;
	/** Max HE-MAC for 7 SS */
	t_u8 max_mcs_7ss : 2;
	/** Max HE-MAC for 6 SS */
	t_u8 max_mcs_6ss : 2;
	/** Max HE-MAC for 5 SS */
	t_u8 max_mcs_5ss : 2;
	/** Max HE-MAC for 4 SS */
	t_u8 max_mcs_4ss : 2;
	/** Max HE-MAC for 3 SS */
	t_u8 max_mcs_3ss : 2;
	/** Max HE-MAC for 2 SS */
	t_u8 max_mcs_2ss : 2;
	/** Max HE-MAC for 1 SS */
	t_u8 max_mcs_1ss : 2;
#else
	/** Max HE-MAC for 1 SS */
	t_u8 max_mcs_1ss : 2;
	/** Max HE-MAC for 2 SS */
	t_u8 max_mcs_2ss : 2;
	/** Max HE-MAC for 3 SS */
	t_u8 max_mcs_3ss : 2;
	/** Max HE-MAC for 4 SS */
	t_u8 max_mcs_4ss : 2;
	/** Max HE-MAC for 5 SS */
	t_u8 max_mcs_5ss : 2;
	/** Max HE-MAC for 6 SS */
	t_u8 max_mcs_6ss : 2;
	/** Max HE-MAC for 7 SS */
	t_u8 max_mcs_7ss : 2;
	/** Max HE-MAC for 8 SS */
	t_u8 max_mcs_8ss : 2;
#endif
} MLAN_PACK_END IEEEtypes_HeMcsMap_t, *pIEEEtypes_HeMcsMap_t;

typedef MLAN_PACK_START struct _IEEEtypes_HeMcsNss_t {
	/** HE Rx MCS and NSS Set */
	t_u16 rx_mcs;
	/** HE Tx MCS and NSS Set*/
	t_u16 tx_mcs;
} MLAN_PACK_END IEEEtypes_HeMcsNss_t, *pIEEEtypes_HeMcsNss_t;

typedef MLAN_PACK_START struct _IEEEtypes_HECap_t {
	/** Generic IE header */
	IEEEtypes_Header_t ieee_hdr;
	/** Element id extension */
	t_u8 ext_id;
	/** he mac capability info */
	t_u8 he_mac_cap[6];
	/** he phy capability info */
	t_u8 he_phy_cap[11];
	/** he txrx mcs support (for 80 MHz) */
	t_u8 he_txrx_mcs_support[4];
	/** Optional Field, including he_txrx_mcs_support for 160 and 80+80 MHz,
	 * and PPE Thresholds */
	t_u8 option[28];
} MLAN_PACK_END IEEEtypes_HECap_t, *pIEEEtypes_HECap_t;

typedef MLAN_PACK_START struct _IEEEtypes_HeOpParam_t {
#ifdef BIG_ENDIAN_SUPPORT
	/** Reserved, including 6G Operation Info Pressent (bit17) */
	t_u8 reserved : 6; /* bit 18-23 */
	/* 6g operation info present */
	t_u8 he_6g_op_info_present : 1; /* bit 17 */
	/** ER SU Disable */
	t_u8 er_su_disable : 1; /* bit 16 */
	/** Co-Hosted BSS */
	t_u16 co_located_bss : 1; /* bit 15 */
	/** VHT Operation Info Present */
	t_u16 vht_op_info_present : 1; /* bit 14 */
	/** TXOP Duration RTS Threshold */
	t_u16 txop_dur_rts_threshold : 10; /* bit 4-13 */
	/** TWT Required */
	t_u16 twt_req : 1; /* bit 3 */
	/** Default PE Duration */
	t_u16 default_pe_dur : 3; /* bit 0-2 */
#else
	/** Default PE Duration */
	t_u16 default_pe_dur : 3; /* bit 0-2 */
	/** TWT Required */
	t_u16 twt_req : 1; /* bit 3 */
	/** TXOP Duration RTS Threshold */
	t_u16 txop_dur_rts_threshold : 10; /* bit 4-13 */
	/** VHT Operation Info Present */
	t_u16 vht_op_info_present : 1; /* bit 14 */
	/** Co-Hosted BSS */
	t_u16 co_located_bss : 1; /* bit 15 */
	/** ER SU Disable */
	t_u8 er_su_disable : 1; /* bit 16 */
	/* 6g operation info present */
	t_u8 he_6g_op_info_present : 1; /* bit 17 */
	/** Reserved bit 18-23 */
	t_u8 reserved : 6; /* bit 18-23 */
#endif
} MLAN_PACK_END IEEEtypes_HeOpParam_t;

typedef MLAN_PACK_START struct _IEEEtypes_HeBssColorInfo_t {
#ifdef BIG_ENDIAN_SUPPORT
	/** BSS Color Disabled */
	t_u8 bss_color_disabled : 1; /* bit 7 */
	/** Partial BSS Color */
	t_u8 partial_bss_color : 1; /* bit 6 */
	/** BSS Color */
	t_u8 bss_color : 6; /* bit 0-5 */
#else
	/** BSS Color */
	t_u8 bss_color : 6; /* bit 0-5 */
	/** Partial BSS Color */
	t_u8 partial_bss_color : 1; /* bit 6 */
	/** BSS Color Disabled */
	t_u8 bss_color_disabled : 1; /* bit 7 */
#endif
} MLAN_PACK_END IEEEtypes_HeBssColorInfo_t;

typedef MLAN_PACK_START struct _IEEEtypes_HeOp_t {
	/** Generic IE header */
	IEEEtypes_Header_t ieee_hdr;
	/** Element id extension */
	t_u8 ext_id;
	/** HE Operation Parameters */
	IEEEtypes_HeOpParam_t he_op_param;
	/** BSS Color Info */
	IEEEtypes_HeBssColorInfo_t bss_color_info;
	/** Basic HE-MCS and NSS Set */
	IEEEtypes_HeMcsMap_t basic_he_mcs_nss;
	/** Optional Field, including VHT Operation Info Max Co-Hosted BSSID
	 * Indicator, and 6Ghz Operation Info  */
	t_u8 option[9];
} MLAN_PACK_END IEEEtypes_HeOp_t;

/** MU EDCA Parameter Set */
typedef MLAN_PACK_START struct _IEEEtypes_MUEDCAParamSet_t {
	/** Generic IE header */
	IEEEtypes_Header_t ieee_hdr;
	/** Extended Tag */
	t_u8 ext_tag;
	/** QOS Information */
	t_u8 qos_info;
	/** MUAC BE Paramter Record */
	t_u8 muac_be[3];
	/** MUAC BK Paramter Record */
	t_u8 muac_bk[3];
	/** MUAC VI Paramter Record */
	t_u8 muac_vi[3];
	/** MUAC VO Paramter Record */
	t_u8 muac_vo[3];
} MLAN_PACK_END IEEEtypes_MUEDCAParamSet_t, *pIEEEtypes_MUEDCAParamSet_t;

/** IEEE format IE */
typedef MLAN_PACK_START struct _IEEEtypes_Element_t {
	/** Generic IE header */
	IEEEtypes_Header_t ieee_hdr;
	/** IE data */
	t_u8 data[];
} MLAN_PACK_END IEEEtypes_Element_t, *pIEEEtypes_Element_t;

typedef MLAN_PACK_START struct _IEEEtypes_6G_control_t {
#ifdef BIG_ENDIAN_SUPPORT
	/** reserved */
	t_u8 reserved : 2; /* bit 6-7 */
	/** regulatory info */
	t_u8 regulatory_info : 3; /* bit 3-5 */
	/** duplicate_beacon */
	t_u8 duplicate_beacon : 1; /* bit 2 */
	/** channel width */
	t_u8 channel_width : 2; /* bit 0-1 */
#else
	/** channel width */
	t_u8 channel_width : 2; /* bit 0-1 */
	/** duplicate_beacon */
	t_u8 duplicate_beacon : 1; /* bit 2 */
	/** regulatory info */
	t_u8 regulatory_info : 3; /* bit 3-5 */
	/** reserved */
	t_u8 reserved : 2; /* bit 6-7 */
#endif
} MLAN_PACK_END IEEEtypes_6G_control_t;

/* IEEEtypes_He6GOpInfo_t */
typedef MLAN_PACK_START struct _IEEEtypes_He6GOpInfo_t {
	/** primary channel */
	t_u8 primary_channel;
	/** control */
	IEEEtypes_6G_control_t control;
	/** center freq0 */
	t_u8 channel_center_freq0;
	/** center freq1 */
	t_u8 channel_center_freq1;
	/** minimum rate */
	t_u8 minimum_rate;
} MLAN_PACK_END IEEEtypes_He6GOpInfo_t, *pIEEEtypes_He6GOpInfo_t;

/* IEEEtypes_HE6GCap_t */
typedef MLAN_PACK_START struct _IEEEtypes_HE6GCap_t {
	/** Generic IE header */
	IEEEtypes_Header_t ieee_hdr;
	/** Element id extension */
	t_u8 ext_id;
	/** he 6g capability */
	t_u16 capa;
} MLAN_PACK_END IEEEtypes_HE6GCap_t, *pIEEEtypes_HE6GCap_t;

/** default channel switch count */
#define DEF_CHAN_SWITCH_COUNT 5

/*  IEEE Channel Switch Announcement Element (7.3.2.20) */
/**
 *  Provided in beacons and probe responses.  Used to advertise when
 *    and to which channel it is changing to.  Only starting STAs in
 *    an IBSS and APs are allowed to originate a chan switch element.
 */
typedef MLAN_PACK_START struct {
	t_u8 element_id; /**< IEEE Element ID = 37 */
	t_u8 len; /**< Element length after id and len */
	t_u8 chan_switch_mode; /**< STA should not transmit any frames if 1 */
	t_u8 new_channel_num; /**< Channel # that AP/IBSS is moving to */
	t_u8 chan_switch_count; /**< # of TBTTs before channel switch */

} MLAN_PACK_END IEEEtypes_ChanSwitchAnn_t;

/** data structure for extended channel switch */
typedef MLAN_PACK_START struct {
	/** IEEE element ID = 60 */
	t_u8 element_id;
	/** Element length after id and len, set to 4 */
	t_u8 len;
	/** STA should not transmit any frames if 1 */
	t_u8 chan_switch_mode;
	/** Operate class # that AP/IBSS is moving to */
	t_u8 new_oper_class;
	/** Channel # that AP/IBSS is moving to */
	t_u8 new_channel_num;
	/** of TBTTs before channel switch */
	t_u8 chan_switch_count;
} MLAN_PACK_END IEEEtypes_ExtChanSwitchAnn_t;

/** Maximum number of subbands in the IEEEtypes_SupportedChannels_t structure */
#define WLAN_11H_MAX_SUBBANDS 6

/**  IEEE Power Constraint element (7.3.2.15) */
typedef MLAN_PACK_START struct {
	t_u8 element_id; /**< IEEE Element ID = 32 */
	t_u8 len; /**< Element length after id and len */
	t_u8 local_constraint; /**< Local power constraint applied to 11d
				chan info */
} MLAN_PACK_END IEEEtypes_PowerConstraint_t;

/**  IEEE Power Capability element (7.3.2.16) */
typedef MLAN_PACK_START struct {
	t_u8 element_id; /**< IEEE Element ID = 33 */
	t_u8 len; /**< Element length after id and len */
	t_s8 min_tx_power_capability; /**< Minimum Transmit power (dBm) */
	t_s8 max_tx_power_capability; /**< Maximum Transmit power (dBm) */
} MLAN_PACK_END IEEEtypes_PowerCapability_t;

/**  IEEE TPC Report element (7.3.2.18) */
typedef MLAN_PACK_START struct {
	t_u8 element_id; /**< IEEE Element ID = 35 */
	t_u8 len; /**< Element length after id and len */
	t_s8 tx_power; /**< Max power used to transmit the TPC Report frame
			  (dBm) */
	t_s8 link_margin; /**< Link margin when TPC Request received (dB) */
} MLAN_PACK_END IEEEtypes_TPCReport_t;

/*  IEEE Supported Channel sub-band description (7.3.2.19) */
/**
 *  Sub-band description used in the supported channels element.
 */
typedef MLAN_PACK_START struct {
	t_u8 start_chan; /**< Starting channel in the subband */
	t_u8 num_chans; /**< Number of channels in the subband */

} MLAN_PACK_END IEEEtypes_SupportChan_Subband_t;

/*  IEEE Supported Channel element (7.3.2.19) */
/**
 *  Sent in association requests. Details the sub-bands and number
 *    of channels supported in each subband
 */
typedef MLAN_PACK_START struct {
	t_u8 element_id; /**< IEEE Element ID = 36 */
	t_u8 len; /**< Element length after id and len */

	/** Configured sub-bands information in the element */
	IEEEtypes_SupportChan_Subband_t subband[WLAN_11H_MAX_SUBBANDS];

} MLAN_PACK_END IEEEtypes_SupportedChannels_t;

/*  IEEE Wide Bandwidth Channel Switch Element */
/**
 *  Provided in beacons and probe responses.  Used to advertise when
 *    and to which channel it is changing to.  Only starting STAs in
 *    an IBSS and APs are allowed to originate a wide bandwidth chan
 *    switch element.
 */
typedef MLAN_PACK_START struct {
	/** Generic IE header IEEE Element ID = 194*/
	IEEEtypes_Header_t ieee_hdr;
	t_u8 new_channel_width;
	t_u8 new_channel_center_freq0;
	t_u8 new_channel_center_freq1;
} MLAN_PACK_END IEEEtypes_WideBWChanSwitch_t;

/*  IEEE VHT Transmit Power Envelope Element */
/**
 *  Provided in beacons and probe responses.  Used to advertise the max
 *    TX power in sepeate bandwidth and as a sub element of Channel Switch
 *    Wrapper IE.
 */
typedef MLAN_PACK_START struct {
	/** Generic IE header IEEE Element ID = 195*/
	IEEEtypes_Header_t ieee_hdr;
	t_u8 tpc_info; /**< Transmit Power Information>*/
	t_u8 local_max_tp_20mhz; /**< Local Maximum Transmit Power for 20 MHZ>*/
	t_u8 local_max_tp_40mhz; /**< Local Maximum Transmit Power for 40 MHZ>*/
	t_u8 local_max_tp_80mhz; /**< Local Maximum Transmit Power for 80 MHZ>*/
	t_u8 local_max_tp_160mhz_80_80mhz; /**< Local Maximum Transmit Power for
					      160/80+80 MHZ>*/
} MLAN_PACK_END IEEEtypes_VhtTpcEnvelope_t;

/*  IEEE Quiet Period Element (7.3.2.23) */
/**
 *  Provided in beacons and probe responses.  Indicates times during
 *    which the STA should not be transmitting data.  Only starting STAs in
 *    an IBSS and APs are allowed to originate a quiet element.
 */
typedef MLAN_PACK_START struct {
	t_u8 element_id; /**< IEEE Element ID = 40 */
	t_u8 len; /**< Element length after id and len */
	t_u8 quiet_count; /**< Number of TBTTs until beacon with the quiet
			     period */
	t_u8 quiet_period; /**< Regular quiet period, # of TBTTS between periods
			    */
	t_u16 quiet_duration; /**< Duration of the quiet period in TUs */
	t_u16 quiet_offset; /**< Offset in TUs from the TBTT for the quiet
			       period */

} MLAN_PACK_END IEEEtypes_Quiet_t;

/**
***  @brief Map octet of the basic measurement report (7.3.2.22.1)
**/
typedef MLAN_PACK_START struct {
#ifdef BIG_ENDIAN_SUPPORT
	/**< Reserved */
	t_u8 rsvd5_7 : 3;
	/**< Channel is unmeasured */
	t_u8 unmeasured : 1;
	/**< Radar detected on channel */
	t_u8 radar : 1;
	/**< Unidentified signal found on channel */
	t_u8 unidentified_sig : 1;
	/**< OFDM preamble detected on channel */
	t_u8 ofdm_preamble : 1;
	/**< At least one valid MPDU received on channel */
	t_u8 bss : 1;
#else
	/**< At least one valid MPDU received on channel */
	t_u8 bss : 1;
	/**< OFDM preamble detected on channel */
	t_u8 ofdm_preamble : 1;
	/**< Unidentified signal found on channel */
	t_u8 unidentified_sig : 1;
	/**< Radar detected on channel */
	t_u8 radar : 1;
	/**< Channel is unmeasured */
	t_u8 unmeasured : 1;
	/**< Reserved */
	t_u8 rsvd5_7 : 3;
#endif /* BIG_ENDIAN_SUPPORT */

} MLAN_PACK_END MeasRptBasicMap_t;

/*  IEEE DFS Channel Map field (7.3.2.24) */
/**
 *  Used to list supported channels and provide a octet "map" field which
 *    contains a basic measurement report for that channel in the
 *    IEEEtypes_IBSS_DFS_t element
 */
typedef MLAN_PACK_START struct {
	t_u8 channel_number; /**< Channel number */
	MeasRptBasicMap_t rpt_map; /**< Basic measurement report for the channel
				    */

} MLAN_PACK_END IEEEtypes_ChannelMap_t;

/* 802.11h BSS information kept for each BSSID received in scan results */
/**
 * IEEE BSS information needed from scan results for later processing in
 *    join commands
 */
typedef struct {
	t_u8 sensed_11h; /**< Capability bit set or 11h IE found in this BSS */

	IEEEtypes_PowerConstraint_t power_constraint; /**< Power Constraint IE
						       */
	IEEEtypes_PowerCapability_t power_capability; /**< Power Capability IE
						       */
	IEEEtypes_TPCReport_t tpc_report; /**< TPC Report IE */
	IEEEtypes_ChanSwitchAnn_t chan_switch_ann; /**< Channel Switch
						      Announcement IE */
	IEEEtypes_Quiet_t quiet; /**< Quiet IE */

} wlan_11h_bss_info_t;

/** Ethernet packet type for TDLS */
#define MLAN_ETHER_PKT_TYPE_TDLS_ACTION (0x890D)

/*802.11z  TDLS action frame type and strcuct */
typedef MLAN_PACK_START struct {
	/*link indentifier ie =101*/
	t_u8 element_id;
	/*len = 18*/
	t_u8 len;
	/** bssid */
	t_u8 bssid[MLAN_MAC_ADDR_LENGTH];
	/** init sta mac address */
	t_u8 init_sta[MLAN_MAC_ADDR_LENGTH];
	/** resp sta mac address */
	t_u8 resp_sta[MLAN_MAC_ADDR_LENGTH];
} MLAN_PACK_END IEEEtypes_tdls_linkie;

/** action code for tdls setup request */
#define TDLS_SETUP_REQUEST 0
/** action code for tdls setup response */
#define TDLS_SETUP_RESPONSE 1
/** action code for tdls setup confirm */
#define TDLS_SETUP_CONFIRM 2
/** action code for tdls tear down */
#define TDLS_TEARDOWN 3
/** action code for tdls traffic indication */
#define TDLS_PEER_TRAFFIC_INDICATION 4
/** action code for tdls channel switch request */
#define TDLS_CHANNEL_SWITCH_REQUEST 5
/** action code for tdls channel switch response */
#define TDLS_CHANNEL_SWITCH_RESPONSE 6
/** action code for tdls psm request */
#define TDLS_PEER_PSM_REQUEST 7
/** action code for tdls psm response */
#define TDLS_PEER_PSM_RESPONSE 8
/** action code for tdls traffic response */
#define TDLS_PEER_TRAFFIC_RESPONSE 9
/** action code for tdls discovery request */
#define TDLS_DISCOVERY_REQUEST 10
/** action code for TDLS discovery response */
#define TDLS_DISCOVERY_RESPONSE 14
/** category public */
#define CATEGORY_PUBLIC 4

/** action code for 20/40 BSS Coexsitence Management frame */
#define BSS_20_40_COEX 0

#ifdef STA_SUPPORT
/** Macro for maximum size of scan response buffer */
#define MAX_SCAN_RSP_BUF (16 * 1024)

/** Maximum number of channels that can be sent in user scan config */
#define WLAN_USER_SCAN_CHAN_MAX 109
/** Maximum length of SSID list */
#define MRVDRV_MAX_SSID_LIST_LENGTH 10

/** Maximum length of BSSID list */
#define MAX_BSSID_FILTER_LIST 5

/** Scan all the channels in specified band */
#define BAND_SPECIFIED 0x80

/**
 *  IOCTL SSID List sub-structure sent in wlan_ioctl_user_scan_cfg
 *
 *  Used to specify SSID specific filters as well as SSID pattern matching
 *    filters for scan result processing in firmware.
 */
typedef MLAN_PACK_START struct _wlan_user_scan_ssid {
	/** SSID */
	t_u8 ssid[MLAN_MAX_SSID_LENGTH + 1];
	/** Maximum length of SSID */
	t_u8 max_len;
} MLAN_PACK_END wlan_user_scan_ssid;

/**
 *  @brief IOCTL channel sub-structure sent in wlan_ioctl_user_scan_cfg
 *
 *  Multiple instances of this structure are included in the IOCTL command
 *   to configure a instance of a scan on the specific channel.
 */
typedef MLAN_PACK_START struct _wlan_user_scan_chan {
	/** Channel Number to scan */
	t_u8 chan_number;
	/** Radio type: 'B/G' Band = 0, 'A' Band = 1 */
	t_u8 radio_type;
	/** Scan type: Active = 1, Passive = 2 */
	t_u8 scan_type;
	/** rnr_flag */
	t_u8 rnr_flag;
	/** Scan duration in milliseconds; if 0 default used */
	t_u32 scan_time;
} MLAN_PACK_END wlan_user_scan_chan;

/** channel statictics */
typedef MLAN_PACK_START struct _ChanStatistics_t {
	/** channle number */
	t_u8 chan_num;
	/** band info */
	Band_Config_t bandcfg;
	/** flags */
	t_u8 flags;
	/** noise */
	t_s8 noise;
	/** total network */
	t_u16 total_networks;
	/** scan duration */
	t_u16 cca_scan_duration;
	/** busy duration */
	t_u16 cca_busy_duration;
	/** min rss */
	t_u8 min_rss;
	/** max rssi */
	t_u8 max_rss;
} MLAN_PACK_END ChanStatistics_t;

#define MLAN_6G_CHAN_MAX 59
#define MLAN_RNR_COLOC_AP_MAX MLAN_6G_CHAN_MAX
#define WLAN_MAX_6G_SCAN_PARAMS_LIST 20
#define SHORT_SSID_VALID MBIT(0)
#define UNSOLICITED_PROBE MBIT(1)

/**
 *  IOCTL 6g scan params List sub-structure sent in wlan_ioctl_user_scan_cfg
 */
typedef MLAN_PACK_START struct _wlan_6g_scan_params {
	/** scan channel */
	t_u8 channel;
	/** bit0: short_ssid_valid, bit1: unsolicited_probe */
	t_u16 flags;
	/** short ssid */
	t_u32 short_ssid;
	/** bssid */
	t_u8 bssid[6];
} MLAN_PACK_END wlan_6g_scan_params;

/*
 * Reduced Neighbor Report(RNR), based on 802.11ax-2021,
 * section 9.4.2.170.
 */
#define IEEE80211_RNR_TBTT_INFO_OFFSET_BSSID_BSSPARAMS 9
#define IEEE80211_RNR_TBTT_INFO_OFFSET_BSSID_SSSID_BSSPARAMS 13

/** Get count of TBTT Information (bit 7:4) */
#define GET_RNR_TBTT_INFO_HDR_COUNT(hdr) ((hdr >> 4) & 0x0f)
#define GET_RNR_TBTT_INFO_BSSPARAMS_BIT(bss_params, bit)                       \
	(bss_params >> bit) & 0x01

/** RNR BSS parameters */
typedef MLAN_PACK_START struct _IEEEtypes_RnrBssParams_t {
#ifdef BIG_ENDIAN_SUPPORT
	/** Reserved */
	t_u8 rsrvd : 1;
	/** Co-Located AP */
	t_u8 colocated : 1;
	/** Unsolicited Probe Responses Active */
	t_u8 unsolicited_probe : 1;
	/** Member Of ESS With 2.4/5 GHz Co-Located AP */
	t_u8 colocated_ess : 1;
	/** Transmitted BSSID */
	t_u8 transmitted_bssid : 1;
	/** Multiple BSSID */
	t_u8 multi_bss : 1;
	/** Same SSID */
	t_u8 same_ssid : 1;
	/** OCT Recommended */
	t_u8 oct_recommended : 1;
#else
	/** OCT Recommended */
	t_u8 oct_recommended : 1;
	/** Same SSID */
	t_u8 same_ssid : 1;
	/** Multiple BSSID */
	t_u8 multi_bss : 1;
	/** Transmitted BSSID */
	t_u8 transmitted_bssid : 1;
	/** Member Of ESS With 2.4/5 GHz Co-Located AP */
	t_u8 colocated_ess : 1;
	/** Unsolicited Probe Responses Active */
	t_u8 unsolicited_probe : 1;
	/** Co-Located AP */
	t_u8 colocated : 1;
	/** Reserved */
	t_u8 rsrvd : 1;
#endif /* BIG_ENDIAN_SUPPORT */
} MLAN_PACK_END IEEEtypes_RnrBssParams_t, *pIEEEtypes_RnrBssParams_t;

/** structure for RNR Neighbor AP Information */
typedef MLAN_PACK_START struct _IEEEtypes_RnrNeighborApInfo_t {
	/* TBTT Information Header */
	t_u8 tbtt_info_hdr;
	/* TBTT Information Length */
	t_u8 tbtt_info_len;
	/** Operating Class */
	t_u8 oper_class;
	/** Channel Number */
	t_u8 chan_number;
} MLAN_PACK_END IEEEtypes_RnrNeighborApInfo_t, *pIEEEtypes_RnrNeighborApInfo_t;

/** structure for RNR colocated ap */
typedef MLAN_PACK_START struct _RnrColocatedAp_t {
	/** BSSID */
	t_u8 bssid[MLAN_MAC_ADDR_LENGTH];
	/** SSID */
	mlan_802_11_ssid ssid;
	/** Short-SSID */
	t_u32 short_ssid;
	/** Operating Class */
	t_u8 oper_class;
	/** Channel Number */
	t_u8 chan_number;
	/** BSS parameters */
	IEEEtypes_RnrBssParams_t bss_params;
} MLAN_PACK_END RnrColocatedAp_t, *pRnrColocatedAp_t;

/** Enhance ext scan type defination */
typedef enum _MLAN_EXT_SCAN_TYPE {
	EXT_SCAN_DEFAULT,
	EXT_SCAN_ENHANCE,
	EXT_SCAN_CANCEL,
} MLAN_EXT_SCAN_TYPE;

/**
 *  Input structure to configure an immediate scan cmd to firmware
 *
 *  Specifies a number of parameters to be used in general for the scan
 *    as well as a channel list (wlan_user_scan_chan) for each scan period
 *    desired.
 */
typedef MLAN_PACK_START struct {
	/**
	 *  Flag set to keep the previous scan table intact
	 *
	 *  If set, the scan results will accumulate, replacing any previous
	 *   matched entries for a BSS with the new scan data
	 */
	t_u8 keep_previous_scan;
	/**
	 *  BSS mode to be sent in the firmware command
	 *
	 *  Field can be used to restrict the types of networks returned in the
	 *    scan.  Valid settings are:
	 *
	 *   - MLAN_SCAN_MODE_BSS  (infrastructure)
	 *   - MLAN_SCAN_MODE_IBSS (adhoc)
	 *   - MLAN_SCAN_MODE_ANY  (unrestricted, adhoc and infrastructure)
	 */
	t_u8 bss_mode;
	/**
	 *  Configure the number of probe requests for active chan scans
	 */
	t_u8 num_probes;
	/**
	 *  @brief ssid filter flag
	 */
	t_u8 ssid_filter;
	/**
	 *  @brief BSSID filter sent in the firmware command to limit the
	 * results
	 */
	t_u8 specific_bssid[MLAN_MAC_ADDR_LENGTH];
	/**
	 *  SSID filter list used in the to limit the scan results
	 */
	wlan_user_scan_ssid ssid_list[MRVDRV_MAX_SSID_LIST_LENGTH];
	/**
	 *  Variable number (fixed maximum) of channels to scan up
	 */
	wlan_user_scan_chan chan_list[WLAN_USER_SCAN_CHAN_MAX];
	/** scan channel gap */
	t_u16 scan_chan_gap;
	/** scan type: 0 legacy, 1: enhance scan*/
	t_u8 ext_scan_type;
	/** flag to filer only probe response */
	t_u8 proberesp_only;
	t_u8 random_mac[MLAN_MAC_ADDR_LENGTH];
	/** Number of BSSIDs to be filtered */
	t_u8 bssid_num;
	/** BSSID filter list used in the to limit the scan results */
	mlan_802_11_mac_addr bssid_list[MAX_BSSID_FILTER_LIST];
	/** use scan setting from scan_cfg only  */
	t_u8 scan_cfg_only;
	/**num of 6g scan params for OOB scan */
	t_u8 num_6g_scan_params;
	/** 6g scan params for OOB scan */
	wlan_6g_scan_params scan_param_list[WLAN_MAX_6G_SCAN_PARAMS_LIST];
} MLAN_PACK_END wlan_user_scan_cfg;

/** Default scan interval in millisecond*/
#define DEFAULT_BGSCAN_INTERVAL 30000

/** action get all, except pps/uapsd config */
#define BG_SCAN_ACT_GET 0x0000
/** action set all, except pps/uapsd config */
#define BG_SCAN_ACT_SET 0x0001
/** action get pps/uapsd config */
#define BG_SCAN_ACT_GET_PPS_UAPSD 0x0100
/** action set pps/uapsd config */
#define BG_SCAN_ACT_SET_PPS_UAPSD 0x0101
/** action set all */
#define BG_SCAN_ACT_SET_ALL 0xff01
/** ssid match */
#define BG_SCAN_SSID_MATCH 0x0001
/** ssid match and RSSI exceeded */
#define BG_SCAN_SSID_RSSI_MATCH 0x0004
/**wait for all channel scan to complete to report scan result*/
#define BG_SCAN_WAIT_ALL_CHAN_DONE 0x80000000

#define CHAN_MAX_6G 59 // reference from cfg80211_channels_6ghz table

/** max bgscan chan number */
#define WLAN_BG_SCAN_CHAN_MAX 38

/** max bgscan chan number, include UNII_4 channel */
#define WLAN_BG_SCAN_CHAN_MAX_UNII_4 41

/** max bgscan chan number, 3 + 38 + 59 */
#define WLAN_BG_SCAN_CHAN_MAX_6E 100

/** Enumeration definition */
/** EES MODE */
typedef enum {
	/** EES MODE: LOW */
	EES_MODE_LOW = 0,
	/** EES MODE: MID */
	EES_MODE_MID,
	/** EES MODE: HIGH */
	EES_MODE_HIGH,
	/** EES MODE: OFF */
	EES_MODE_OFF,
	/** EES MODE: LOOP */
	EES_MODE_LOOP = 15,
} ees_modes;

/** EES Maximum SSID */
#define EES_MAX_SSIDS 2

/** ees_ssid_config */
typedef MLAN_PACK_START struct {
	/** SSID */
	t_u8 ssid[MLAN_MAX_SSID_LENGTH + 1];
	/** Maximum length of SSID */
	t_u8 max_len;
	/** PairCipher */
	t_u8 pair_cipher;
	/** GroupCipher */
	t_u8 group_cipher;
} MLAN_PACK_END ees_ssid_config;

/**
 *  Input structure to configure bs scan cmd to firmware
 */
typedef MLAN_PACK_START struct {
	/** action */
	t_u16 action;
	/** enable/disable */
	t_u8 enable;
	/**  BSS type:
	 *   MLAN_SCAN_MODE_BSS  (infrastructure)
	 *   MLAN_SCAN_MODE_IBSS (adhoc)
	 *   MLAN_SCAN_MODE_ANY  (unrestricted, adhoc and infrastructure)
	 */
	t_u8 bss_type;
	/** number of channel scanned during each scan */
	t_u8 chan_per_scan;
	/** interval between consecutive scan */
	t_u32 scan_interval;
	/** bit 0: ssid match bit 1: ssid match and SNR exceeded
	 *  bit 2: ssid match and RSSI exceeded
	 *  bit 31: wait for all channel scan to complete to report scan result
	 */
	t_u32 report_condition;
	/*  Configure the number of probe requests for active chan scans */
	t_u8 num_probes;
	/** RSSI threshold */
	t_u8 rssi_threshold;
	/** SNR threshold */
	t_u8 snr_threshold;
	/** repeat count */
	t_u16 repeat_count;
	/** start later flag */
	t_u16 start_later;
	/** SSID filter list used in the to limit the scan results */
	wlan_user_scan_ssid ssid_list[MRVDRV_MAX_SSID_LIST_LENGTH];
	/** Variable number (fixed maximum) of channels to scan up */
	wlan_user_scan_chan chan_list[WLAN_USER_SCAN_CHAN_MAX];
	/** scan channel gap */
	t_u16 scan_chan_gap;
	/** Enable EES configuration */
	t_u8 config_ees;
	/** EES scan mode */
	t_u16 ees_mode;
	/** EES report condition */
	t_u16 report_cond;
	/** EES High Period scan interval */
	t_u16 high_period;
	/** EES High Period scan count */
	t_u16 high_period_count;
	/** EES Medium Period scan interval */
	t_u16 mid_period;
	/** EES Medium Period scan count */
	t_u16 mid_period_count;
	/** EES Low Period scan interval */
	t_u16 low_period;
	/** EES Low Period scan count */
	t_u16 low_period_count;
	/** Number of networks in the list */
	t_u8 network_count;
	/** Maximum number of connection count */
	t_u8 max_conn_count;
	/** Black List Exp */
	t_u8 black_list_exp;
	/** Array of ees config struct */
	ees_ssid_config ees_ssid_cfg[EES_MAX_SSIDS];
	t_u8 random_mac[MLAN_MAC_ADDR_LENGTH];
	/** 11ai indication */
	t_u8 dot11ai;
} MLAN_PACK_END wlan_bgscan_cfg;
#endif /* STA_SUPPORT */

/** The open AP in OWE transmition Mode */
#define OWE_TRANS_MODE_OPEN 1
/** The security AP in OWE trsnsition Mode */
#define OWE_TRANS_MODE_OWE 2

#define VENDOR_OUI_LEN 4
#define MAX_VENDOR_OUI_NUM 10

#ifdef PRAGMA_PACK
#pragma pack(pop)
#endif

/** BSSDescriptor_t
 *    Structure used to store information for beacon/probe response
 */
typedef struct _BSSDescriptor_t {
	/** MAC address */
	mlan_802_11_mac_addr mac_address;

	/** SSID */
	mlan_802_11_ssid ssid;

	/** Transition MAC address */
	mlan_802_11_mac_addr trans_mac_address;

	/** Transition SSID */
	mlan_802_11_ssid trans_ssid;

	/** OWE Transition mode */
	t_u8 owe_transition_mode;

	/** WEP encryption requirement */
	t_u32 privacy;

	/** Receive signal strength in dBm */
	t_s32 rssi;
	/** channel load */
	t_u8 chan_load;
	/** Channel */
	t_u32 channel;

	/** Freq */
	t_u32 freq;

	/** Beacon period */
	t_u16 beacon_period;

	/** ATIM window */
	t_u32 atim_window;

	/** ERP flags */
	t_u8 erp_flags;

	/** Type of network in use */
	WLAN_802_11_NETWORK_TYPE network_type_use;

	/** Network infrastructure mode */
	t_u32 bss_mode;

	/** Network supported rates */
	WLAN_802_11_RATES supported_rates;

	/** Supported data rates */
	t_u8 data_rates[WLAN_SUPPORTED_RATES];

	/** Current channel bandwidth
	 *  0 : 20MHZ
	 *  1 : 40MHZ
	 *  2 : 80MHZ
	 *  3 : 160MHZ
	 */
	t_u8 curr_bandwidth;

	/** Network band.
	 * BAND_B(0x01): 'b' band
	 * BAND_G(0x02): 'g' band
	 * BAND_A(0X04): 'a' band
	 */
	t_u16 bss_band;

	/** TSF timestamp from the current firmware TSF */
	t_u64 network_tsf;

	/** TSF value included in the beacon/probe response */
	t_u8 time_stamp[8];

	/** PHY parameter set */
	IEEEtypes_PhyParamSet_t phy_param_set;

	/** SS parameter set */
	IEEEtypes_SsParamSet_t ss_param_set;

	/** Capability information */
	IEEEtypes_CapInfo_t cap_info;

	/** WMM IE */
	IEEEtypes_WmmParameter_t wmm_ie;

	/** 802.11h BSS information */
	wlan_11h_bss_info_t wlan_11h_bss_info;

	/** Indicate disabling 11n when associate with AP */
	t_u8 disable_11n;
	/** 802.11n BSS information */
	/** HT Capabilities IE */
	IEEEtypes_HTCap_t *pht_cap;
	/** HT Capabilities Offset */
	t_u16 ht_cap_offset;
	/** HT Information IE */
	IEEEtypes_HTInfo_t *pht_info;
	/** HT Information Offset */
	t_u16 ht_info_offset;
	/** Flag to indicate this is multi_bssid_ap */
	t_u8 multi_bssid_ap;
	/** the mac address of multi-bssid AP */
	mlan_802_11_mac_addr multi_bssid_ap_addr;
	/** Multi BSSID Configuration IE */
	IEEEtypes_MBSSID_Config_t *pmbssid_config;
	/** Multi BSSID Configuration IE offset */
	t_u16 mbssid_config_offset;
	/** 20/40 BSS Coexistence IE */
	IEEEtypes_2040BSSCo_t *pbss_co_2040;
	/** 20/40 BSS Coexistence Offset */
	t_u16 bss_co_2040_offset;
	/** Extended Capabilities IE */
	IEEEtypes_ExtCap_t *pext_cap;
	/** Extended Capabilities Offset */
	t_u16 ext_cap_offset;
	/** Overlapping BSS Scan Parameters IE */
	IEEEtypes_OverlapBSSScanParam_t *poverlap_bss_scan_param;
	/** Overlapping BSS Scan Parameters Offset */
	t_u16 overlap_bss_offset;

	/** VHT Capabilities IE */
	IEEEtypes_VHTCap_t *pvht_cap;
	/** VHT Capabilities IE offset */
	t_u16 vht_cap_offset;
	/** VHT Operations IE */
	IEEEtypes_VHTOprat_t *pvht_oprat;
	/** VHT Operations IE offset */
	t_u16 vht_oprat_offset;
	/** VHT Transmit Power Envelope IE */
	IEEEtypes_VHTtxpower_t *pvht_txpower;
	/** VHT Transmit Power Envelope IE offset */
	t_u16 vht_txpower_offset;
	/** Extended Power Constraint IE */
	IEEEtypes_ExtPwerCons_t *pext_pwer;
	/** Extended Power Constraint IE offset */
	t_u16 ext_pwer_offset;
	/** Extended BSS Load IE  */
	IEEEtypes_ExtBSSload_t *pext_bssload;
	/** Extended BSS Load IE offset */
	t_u16 ext_bssload_offset;
	/** Quiet Channel IE */
	IEEEtypes_QuietChan_t *pquiet_chan;
	/** Quiet Channel IE offset */
	t_u16 quiet_chan_offset;
	/** Operating Mode Notification IE */
	IEEEtypes_OperModeNtf_t *poper_mode;
	/** Operating Mode Notification IE offset */
	t_u16 oper_mode_offset;
	/** HE Capability IE */
	IEEEtypes_HECap_t *phe_cap;
	/** HE Capability IE offset */
	t_u16 he_cap_offset;
	/** HE operation IE */
	IEEEtypes_Extension_t *phe_oprat;
	/** HE operation IE offset */
	t_u16 he_oprat_offset;
	/** HE 6G Capability IE */
	IEEEtypes_HE6GCap_t *phe_6g_cap;
	/** HE 6G Capability IE offset */
	t_u16 he_6g_cap_offset;
	/** RNR IE */
	IEEEtypes_Generic_t *prnr_ie;
	/** RNR IE offset in the beacon buffer */
	t_u16 rnr_offset;
#ifdef STA_SUPPORT
	/** Country information set */
	IEEEtypes_CountryInfoFullSet_t country_info;
#endif /* STA_SUPPORT */

	/** WPA IE */
	IEEEtypes_VendorSpecific_t *pwpa_ie;
	/** WPA IE offset in the beacon buffer */
	t_u16 wpa_offset;
	/** RSN IE */
	IEEEtypes_Generic_t *prsn_ie;
	/** RSN IE offset in the beacon buffer */
	t_u16 rsn_offset;
	/** RSNX IE */
	IEEEtypes_Generic_t *prsnx_ie;
	/** RSNX IE offset in the beacon buffer */
	t_u16 rsnx_offset;
#ifdef STA_SUPPORT
	/** WAPI IE */
	IEEEtypes_Generic_t *pwapi_ie;
	/** WAPI IE offset in the beacon buffer */
	t_u16 wapi_offset;
#endif
	/* Hotspot 2.0 OSEN AKM  IE*/
	IEEEtypes_Generic_t *posen_ie;
	/** osen IE offset in the beacon buffer */
	t_u16 osen_offset;
	/* Mobility domain IE */
	IEEEtypes_MobilityDomain_t *pmd_ie;
	/** Mobility domain IE offset in the beacon buffer */
	t_u16 md_offset;
	/** MU EDCA Parameter IE */
	IEEEtypes_MUEDCAParamSet_t *pmuedca_ie;
	/** MU EDCA Parameter IE offset */
	t_u16 muedca_offset;
	/** Pointer to the returned scan response */
	t_u8 *pbeacon_buf;
	/** Length of the stored scan response */
	t_u32 beacon_buf_size;
	/** Max allocated size for updated scan response */
	t_u32 beacon_buf_size_max;
	/** scan age in secs */
	t_u32 age_in_secs;
	/** vendor oui list */
	t_u8 vendor_oui[VENDOR_OUI_LEN * MAX_VENDOR_OUI_NUM];
	/** vendor OUI count */
	t_u8 vendor_oui_count;
} BSSDescriptor_t, *pBSSDescriptor_t;

#endif /* !_MLAN_IEEE_H_ */
