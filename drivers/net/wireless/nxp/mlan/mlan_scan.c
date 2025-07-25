/** @file mlan_scan.c
 *
 *  @brief Functions implementing wlan scan IOCTL and firmware command APIs
 *
 *  IOCTL handlers as well as command preparation and response routines
 *  for sending scan commands to the firmware.
 *
 *
 *  Copyright 2008-2025 NXP
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
    10/28/2008: initial version
******************************************************/

#include "mlan.h"
#include "mlan_join.h"
#include "mlan_util.h"
#include "mlan_fw.h"
#include "mlan_main.h"
#include "mlan_11n.h"
#include "mlan_11ac.h"
#include "mlan_11ax.h"
#include "mlan_11h.h"
/********************************************************
			Local Constants
********************************************************/
/** minimum scan time for passive to active scan */
#define MIN_PASSIVE_TO_ACTIVE_SCAN_TIME 150

#define MRVDRV_MAX_CHANNELS_PER_SCAN 38
/** The maximum number of channels the firmware can scan per command */
#define MRVDRV_MAX_CHANNELS_PER_SPECIFIC_SCAN 4

/**
 * Number of channels to scan per firmware scan command issuance.
 *
 * Number restricted to prevent hitting the limit on the amount of scan data
 * returned in a single firmware scan command.
 */
#define MRVDRV_CHANNELS_PER_SCAN_CMD 4

/** Memory needed to store a max sized Channel List TLV for a firmware scan */
#define CHAN_TLV_MAX_SIZE                                                      \
	(sizeof(MrvlIEtypesHeader_t) +                                         \
	 (MRVDRV_MAX_CHANNELS_PER_SPECIFIC_SCAN * sizeof(ChanScanParamSet_t)))

/** Memory needed to store supported rate */
#define RATE_TLV_MAX_SIZE                                                      \
	(sizeof(MrvlIEtypes_RatesParamSet_t) + HOSTCMD_SUPPORTED_RATES)

/** Memory needed to store a max number/size WildCard
 *  SSID TLV for a firmware scan */
#define WILDCARD_SSID_TLV_MAX_SIZE                                             \
	(MRVDRV_MAX_SSID_LIST_LENGTH *                                         \
	 (sizeof(MrvlIEtypes_WildCardSsIdParamSet_t) +                         \
	  MRVDRV_MAX_SSID_LENGTH))

/** Memory needed to store a max number/size BSSID TLV for a firmware scan */
#define BSSID_LIST_TLV_MAX_SIZE                                                \
	(sizeof(MrvlIEtypesHeader_t) +                                         \
	 (MRVDRV_MAX_BSSID_LIST * MLAN_MAC_ADDR_LENGTH))

/** WPS TLV MAX size is MAX IE size plus 2 bytes for
 *  t_u16 MRVL TLV extension */
#define WPS_TLV_MAX_SIZE (sizeof(IEEEtypes_VendorSpecific_t) + 2)
/** Maximum memory needed for a wlan_scan_cmd_config
 *  with all TLVs at max */
#define MAX_SCAN_CFG_ALLOC                                                     \
	(sizeof(wlan_scan_cmd_config) + sizeof(MrvlIEtypes_NumProbes_t) +      \
	 sizeof(MrvlIETypes_HTCap_t) + CHAN_TLV_MAX_SIZE + RATE_TLV_MAX_SIZE + \
	 WILDCARD_SSID_TLV_MAX_SIZE + BSSID_LIST_TLV_MAX_SIZE +                \
	 WPS_TLV_MAX_SIZE)

/********************************************************
			Local Variables
********************************************************/

/**
 * Interally used to send a configured scan cmd between
 * driver routines
 */
typedef union {
	/** Scan configuration (variable length) */
	wlan_scan_cmd_config config;
	/** Max allocated block */
	t_u8 config_alloc_buf[MAX_SCAN_CFG_ALLOC];
} wlan_scan_cmd_config_tlv;

/********************************************************
			Global Variables
********************************************************/

/********************************************************
			Local Functions
********************************************************/
/** Cipher suite definition */
enum cipher_suite {
	CIPHER_SUITE_WEP40,
	CIPHER_SUITE_TKIP,
	CIPHER_SUITE_CCMP,
	CIPHER_SUITE_WEP104,
	CIPHER_SUITE_GCMP,
	CIPHER_SUITE_GCMP_256,
	CIPHER_SUITE_CCMP_256,
	CIPHER_SUITE_MAX
};

static t_u8 wpa_ouis[CIPHER_SUITE_MAX][4] = {
	{0x00, 0x50, 0xf2, 0x01}, /* WEP40 */
	{0x00, 0x50, 0xf2, 0x02}, /* TKIP */
	{0x00, 0x50, 0xf2, 0x04}, /* AES */
	{0x00, 0x50, 0xf2, 0x05}, /* WEP104 */
};

static t_u8 rsn_oui[CIPHER_SUITE_MAX][4] = {
	{0x00, 0x0f, 0xac, 0x01}, /* WEP40 */
	{0x00, 0x0f, 0xac, 0x02}, /* TKIP */
	{0x00, 0x0f, 0xac, 0x04}, /* AES */
	{0x00, 0x0f, 0xac, 0x05}, /* WEP104 */
	{0x00, 0x0f, 0xac, 0x08}, /* GCMP */
	{0x00, 0x0f, 0xac, 0x09}, /* GCMP-256 */
	{0x00, 0x0f, 0xac, 0x0a}, /* CCMP-256 */
};

/**
 *  @brief Convert radio type scan parameter to a band config used in join cmd
 *
 *  @param radio_type Scan parameter indicating the radio used for a channel
 *                    in a scan command.
 *
 *  @return          Band type conversion of scanBand used in join/assoc cmds
 *
 */
t_u16 radio_type_to_band(t_u8 radio_type)
{
	t_u16 ret_band;

	switch (radio_type) {
	case BAND_5GHZ:
		ret_band = BAND_A;
		break;
	case BAND_6GHZ:
		ret_band = BAND_6G;
		break;
	case BAND_2GHZ:
	default:
		ret_band = BAND_G;
		break;
	}

	return ret_band;
}

/**
 *  @brief This function will update the channel statistics from scan result
 *
 *  @param pmpriv           A pointer to mlan_private structure
 *  @param pchan_stats      A pointer to chan_statistics_t
 *
 *  @return                 MTRUE/MFALSE
 */
static t_u8 wlan_set_chan_statistics(mlan_private *pmpriv,
				     chan_statistics_t *pchan_stats)
{
	t_u8 i;
	t_u8 ret = MFALSE;
	mlan_adapter *pmadapter = pmpriv->adapter;
	chan_statistics_t *pstats = MNULL;
	ENTER();
	for (i = 0; i < pmadapter->idx_chan_stats; i++) {
		pstats = (chan_statistics_t *)&pmadapter->pchan_stats[i];
		if (pstats->chan_num == pchan_stats->chan_num &&
		    (pstats->bandcfg.chanBand ==
		     pchan_stats->bandcfg.chanBand)) {
			memcpy_ext(pmadapter, (chan_statistics_t *)pstats,
				   pchan_stats, sizeof(chan_statistics_t),
				   sizeof(chan_statistics_t));
			ret = MTRUE;
			break;
		}
	}
	LEAVE();
	return ret;
}

/**
 *  @brief This function will update the channel statistics from scan result
 *
 *  @param pmpriv           A pointer to mlan_private structure
 *  @param pchanstats_tlv   A pointer to MrvlIEtypes_ChannelStats_t tlv
 *
 *  @return                NA
 */
static void
wlan_update_chan_statistics(mlan_private *pmpriv,
			    MrvlIEtypes_ChannelStats_t *pchanstats_tlv)
{
	mlan_adapter *pmadapter = pmpriv->adapter;
	t_u8 i;
	chan_statistics_t *pchan_stats =
		(chan_statistics_t *)((t_u8 *)pchanstats_tlv +
				      sizeof(MrvlIEtypesHeader_t));
	t_u8 num_chan = wlan_le16_to_cpu(pchanstats_tlv->header.len) /
			sizeof(chan_statistics_t);

	ENTER();

	for (i = 0; i < num_chan; i++) {
		pchan_stats->total_networks =
			wlan_le16_to_cpu(pchan_stats->total_networks);
		pchan_stats->cca_scan_duration =
			wlan_le16_to_cpu(pchan_stats->cca_scan_duration);
		pchan_stats->cca_busy_duration =
			wlan_le16_to_cpu(pchan_stats->cca_busy_duration);
		PRINTM(MCMND,
		       "chan=%d, noise=%d, total_network=%d scan_duration=%d, busy_duration=%d\n",
		       pchan_stats->chan_num, pchan_stats->noise,
		       pchan_stats->total_networks,
		       pchan_stats->cca_scan_duration,
		       pchan_stats->cca_busy_duration);
		if (!wlan_set_chan_statistics(pmpriv, pchan_stats)) {
			if (pmadapter->idx_chan_stats >=
			    pmadapter->num_in_chan_stats) {
				PRINTM(MERROR,
				       "Over flow: idx_chan_stats=%d, num_in_chan_stats=%d\n",
				       pmadapter->idx_chan_stats,
				       pmadapter->num_in_chan_stats);
				LEAVE();
				return;
			}
			memcpy_ext(
				pmadapter,
				(chan_statistics_t *)&pmadapter
					->pchan_stats[pmadapter->idx_chan_stats],
				pchan_stats, sizeof(chan_statistics_t),
				sizeof(chan_statistics_t));
			pmadapter->idx_chan_stats++;
		}
		pchan_stats++;
	}
	LEAVE();
	return;
}

/**
 *  @brief This function will parse a given IE for a given OUI
 *
 *  Parse a given WPA/RSN IE to find if it has a given oui in PTK,
 *  if no OUI found for PTK it returns 0.
 *
 *  @param pbss_desc       A pointer to current BSS descriptor
 *  @return                0 on failure to find OUI, 1 on success.
 */
static t_u8 search_oui_in_ie(mlan_adapter *pmadapter, IEBody *ie_body,
			     t_u8 *oui)
{
	t_u8 count;

	count = ie_body->PtkCnt[0];

	ENTER();
	/* There could be multiple OUIs for PTK hence
	 * 1) Take the length.
	 * 2) Check all the OUIs for AES.
	 * 3) If one of them is AES then pass success.
	 */
	while (count) {
		if (!memcmp(pmadapter, ie_body->PtkBody, oui,
			    sizeof(ie_body->PtkBody))) {
			LEAVE();
			return MLAN_OUI_PRESENT;
		}

		--count;
		if (count) {
			ie_body = (IEBody *)((t_u8 *)ie_body +
					     sizeof(ie_body->PtkBody));
		}
	}

	PRINTM(MINFO, "The OUI %x:%x:%x:%x is not found in PTK\n", oui[0],
	       oui[1], oui[2], oui[3]);
	LEAVE();
	return MLAN_OUI_NOT_PRESENT;
}

/**
 *  @brief This function will pass the correct ie and oui to search_oui_in_ie
 *
 *  Check the pbss_desc for appropriate IE and then check if RSN IE has AES
 *  OUI in it. If RSN IE does not have AES in PTK then return 0;
 *
 *  @param pbss_desc       A pointer to current BSS descriptor
 *  @return                0 on failure to find AES OUI, 1 on success.
 */
static t_u8 is_rsn_oui_present(mlan_adapter *pmadapter,
			       BSSDescriptor_t *pbss_desc, t_u32 cipher_suite)
{
	t_u8 *oui = MNULL;
	IEBody *ie_body = MNULL;
	t_u8 ret = MLAN_OUI_NOT_PRESENT;

	ENTER();
	if (pbss_desc->prsn_ie &&
	    (pbss_desc->prsn_ie->ieee_hdr.element_id == RSN_IE) &&
	    (pbss_desc->prsn_ie->ieee_hdr.len > RSN_GTK_OUI_OFFSET)) {
		ie_body = (IEBody *)(((t_u8 *)pbss_desc->prsn_ie->data) +
				     RSN_GTK_OUI_OFFSET);
		oui = &rsn_oui[cipher_suite][0];
		ret = search_oui_in_ie(pmadapter, ie_body, oui);
		if (ret) {
			LEAVE();
			return ret;
		}
	}
	LEAVE();
	return ret;
}

/**
 *  @brief This function will pass the correct ie and oui to search_oui_in_ie
 *
 *  Check the wpa_ie for appropriate IE and then check if RSN IE has AES
 *  OUI in it. If RSN IE does not have AES in PTK then return 0;
 *
 *  @param pmadapter       A pointer to mlan adapter.
 *  @return                0 on failure to find AES OUI, 1 on success.
 */
static t_u8 is_rsn_oui_present_in_wpa_ie(mlan_private *pmpriv,
					 t_u32 cipher_suite)
{
	mlan_adapter *pmadapter = pmpriv->adapter;
	t_u8 *oui = MNULL;
	IEBody *ie_body = MNULL;
	IEEEtypes_Generic_t *prsn_ie = MNULL;
	t_u8 ret = MLAN_OUI_NOT_PRESENT;

	ENTER();
	prsn_ie = (IEEEtypes_Generic_t *)pmpriv->wpa_ie;

	if (prsn_ie && (prsn_ie->ieee_hdr.element_id == RSN_IE) &&
	    (prsn_ie->ieee_hdr.len > RSN_GTK_OUI_OFFSET)) {
		ie_body = (IEBody *)(prsn_ie->data + RSN_GTK_OUI_OFFSET);
		oui = &rsn_oui[cipher_suite][0];
		ret = search_oui_in_ie(pmadapter, ie_body, oui);
		if (ret) {
			LEAVE();
			return ret;
		}
	}

	LEAVE();
	return ret;
}

/**
 *  @brief This function will pass the correct ie and oui to search_oui_in_ie
 *
 *  Check the pbss_desc for appropriate IE and then check if WPA IE has AES
 *  OUI in it. If WPA IE does not have AES in PTK then return 0;
 *
 *  @param pbss_desc       A pointer to current BSS descriptor
 *  @return                0 on failure to find AES OUI, 1 on success.
 */
static t_u8 is_wpa_oui_present(mlan_adapter *pmadapter,
			       BSSDescriptor_t *pbss_desc, t_u32 cipher_suite)
{
	t_u8 *oui = MNULL;
	IEBody *ie_body = MNULL;
	t_u8 ret = MLAN_OUI_NOT_PRESENT;

	ENTER();
	if (((pbss_desc->pwpa_ie) &&
	     ((*(pbss_desc->pwpa_ie)).vend_hdr.element_id == WPA_IE))) {
		ie_body = (IEBody *)pbss_desc->pwpa_ie->data;
		oui = &wpa_ouis[cipher_suite][0];
		ret = search_oui_in_ie(pmadapter, ie_body, oui);
		if (ret) {
			LEAVE();
			return ret;
		}
	}
	LEAVE();
	return ret;
}

/**
 *  @brief compare config band and a band from the scan result,
 *  which is defined by functiion radio_type_to_band(t_u8 radio_type) above
 *
 *  @param cfg_band:  band configured
 *         scan_band: band from scan result
 *
 *  @return  matched: non-zero. unmatched: 0
 *
 */
static t_u16 wlan_is_band_compatible(t_u16 cfg_band, t_u16 scan_band)
{
	t_u16 band;
	switch (scan_band) {
	case BAND_A:
		band = BAND_A | BAND_AN | BAND_AAC;
		break;
	case BAND_6G:
		band = BAND_6G;
		break;
	case BAND_G:
	default:
		band = BAND_B | BAND_G | BAND_GN | BAND_GAC;
	}
	return cfg_band & band;
}

/**
 *  @brief This function finds the best SSID in the Scan List
 *
 *  Search the scan table for the best SSID that also matches the current
 *   adapter network preference (infrastructure or adhoc)
 *
 *  @param pmpriv       A pointer to mlan_private structure
 *  @return             index in BSSID list
 */
static t_s32 wlan_find_best_network_in_list(mlan_private *pmpriv)
{
	mlan_adapter *pmadapter = pmpriv->adapter;
	t_u32 mode = pmpriv->bss_mode;
	t_s32 best_net = -1;
	t_s32 best_rssi = 0;
	t_u32 i;

	ENTER();

	PRINTM(MINFO, "Num of BSSIDs = %d\n", pmadapter->num_in_scan_table);

	for (i = 0; i < pmadapter->num_in_scan_table; i++) {
		switch (mode) {
		case MLAN_BSS_MODE_INFRA:
			if (wlan_is_network_compatible(pmpriv, i, mode) >= 0) {
				if (SCAN_RSSI(pmadapter->pscan_table[i].rssi) >
				    best_rssi) {
					best_rssi = SCAN_RSSI(
						pmadapter->pscan_table[i].rssi);
					best_net = i;
				}
			}
			break;
		case MLAN_BSS_MODE_AUTO:
		default:
			if (SCAN_RSSI(pmadapter->pscan_table[i].rssi) >
			    best_rssi) {
				best_rssi = SCAN_RSSI(
					pmadapter->pscan_table[i].rssi);
				best_net = i;
			}
			break;
		}
	}

	LEAVE();
	return best_net;
}

/**
 * @brief This function add a new colocated ap to coloc_ap_list
 *
 *  @param pmadapter      A pointer to mlan_adapter
 *  @param pcoloc_ap_new  A pointer to the new colocated ap
 *
 *  @return               MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status wlan_add_rnr_coloc_ap(mlan_adapter *pmadapter,
					 wlan_6e_coloc_ap_t *pcoloc_ap_new)
{
	t_u8 add = MTRUE;
	wlan_6e_coloc_ap_t *pcoloc_ap = MNULL;

	ENTER();

	pcoloc_ap =
		(wlan_6e_coloc_ap_t *)util_peek_list(pmadapter->pmoal_handle,
						     &pmadapter->coloc_ap_list,
						     MNULL, MNULL);

	while (pcoloc_ap &&
	       pcoloc_ap != (wlan_6e_coloc_ap_t *)&pmadapter->coloc_ap_list) {
		/* Search the colocated ap list for the same bssid */
		if (!memcmp(pmadapter, pcoloc_ap_new->ap_info.bssid,
			    pcoloc_ap->ap_info.bssid, MLAN_MAC_ADDR_LENGTH)) {
			/*
			 * If the shor SSID or SSID matches as well, it is a
			 * duplicate of this entry. Replace the
			 * old information in the list
			 */
			if ((pcoloc_ap_new->ap_info.short_ssid ==
			     pcoloc_ap->ap_info.short_ssid) ||
			    (pcoloc_ap_new->ap_info.ssid.ssid_len &&
			     (pcoloc_ap_new->ap_info.ssid.ssid_len ==
			      pcoloc_ap->ap_info.ssid.ssid_len) &&
			     (!memcmp(pmadapter,
				      pcoloc_ap_new->ap_info.ssid.ssid,
				      pcoloc_ap->ap_info.ssid.ssid,
				      pcoloc_ap_new->ap_info.ssid.ssid_len)))) {
				add = MFALSE;
				PRINTM(MINFO, "Duplicate of colocated AP!\n");
				break;
			}
		}
		pcoloc_ap = pcoloc_ap->pnext;
	}

	if (add) {
		/* add coloc_ap node to coloc_ap list */
		util_enqueue_list_tail(pmadapter->pmoal_handle,
				       &pmadapter->coloc_ap_list,
				       (pmlan_linked_list)pcoloc_ap_new, MNULL,
				       MNULL);
		LEAVE();
		return MLAN_STATUS_SUCCESS;
	}

	LEAVE();
	return MLAN_STATUS_FAILURE;
}

/**
 *  @brief free all colocated ap from coloc_ap_list
 *
 *  @param pmadapter  A pointer to mlan_adapter
 *
 *  @return      N/A
 */
t_void wlan_free_rnr_coloc_ap(mlan_adapter *pmadapter)
{
	wlan_6e_coloc_ap_t *pcoloc_ap = MNULL;

	ENTER();

	while ((pcoloc_ap = (wlan_6e_coloc_ap_t *)util_peek_list(
			pmadapter->pmoal_handle, &pmadapter->coloc_ap_list,
			MNULL, MNULL))) {
		util_unlink_list(pmadapter->pmoal_handle,
				 &pmadapter->coloc_ap_list,
				 (pmlan_linked_list)pcoloc_ap, MNULL, MNULL);
		pmadapter->callbacks.moal_mfree(pmadapter->pmoal_handle,
						(t_u8 *)pcoloc_ap);
	}

	LEAVE();
}

/**
 * @brief This function find a colocated ap from coloc_ap_list
 *
 *  @param pmadapter      A pointer to mlan_adapter
 *  @param chan_number		Channel to check
 *
 *  @return  MTRUE: colocated ap is found, MFALSE: otherwise
 */
static t_bool wlan_find_rnr_coloc_ap(mlan_adapter *pmadapter, t_u8 chan_number)
{
	t_u8 found = MFALSE;
	wlan_6e_coloc_ap_t *pcoloc_ap = MNULL;

	ENTER();

	pcoloc_ap =
		(wlan_6e_coloc_ap_t *)util_peek_list(pmadapter->pmoal_handle,
						     &pmadapter->coloc_ap_list,
						     MNULL, MNULL);

	while (pcoloc_ap &&
	       pcoloc_ap != (wlan_6e_coloc_ap_t *)&pmadapter->coloc_ap_list) {
		if (pcoloc_ap->ap_info.chan_number == chan_number) {
			found = MTRUE;
			PRINTM(MINFO, "Colocated AP (chan:%d) found!\n",
			       chan_number);
			break;
		}

		pcoloc_ap = pcoloc_ap->pnext;
	}

	LEAVE();
	return found;
}

/**
 *  @brief check if Colocated AP SSID matches scan request
 *
 *  @param pmadapter  A pointer to mlan_adapter
 *  @param pscan_cfg  MNULL or pointer to scan configuration parameters
 *  @param pcoloc_ap  A pointer to the new colocated ap
 *
 *  @return MTRUE: SSID matches, MFALSE: otherwise
 *
 */
static t_bool
wlan_scan_rnr_coloc_ap_ssid_match(mlan_adapter *pmadapter,
				  const wlan_user_scan_cfg *pscan_cfg,
				  wlan_6e_coloc_ap_t *pcoloc_ap)
{
	mlan_callbacks *pcb = &pmadapter->callbacks;
	t_u32 short_ssid = 0;
	t_u32 ssid_len;
	t_u32 ssid_idx;

	if (!pscan_cfg)
		return MFALSE;

	for (ssid_idx = 0; ((ssid_idx < NELEMENTS(pscan_cfg->ssid_list)) &&
			    (*pscan_cfg->ssid_list[ssid_idx].ssid ||
			     pscan_cfg->ssid_list[ssid_idx].max_len));
	     ssid_idx++) {
		ssid_len = wlan_strlen(
			(const char *)pscan_cfg->ssid_list[ssid_idx].ssid);

		/* Wildcard ssid in the scan request */
		if (!ssid_len) {
			if (pcoloc_ap->ap_info.bss_params.multi_bss &&
			    !pcoloc_ap->ap_info.bss_params.transmitted_bssid)
				continue;

			return MTRUE;
		}

		short_ssid = 0;
		/* Calculate 32bit short SSID over request SSID */
		pcb->moal_calc_short_ssid(
			// Typecasting is done to read the value
			// coverity[misra_c_2012_rule_11_8_violation:SUPPRESS]
			(t_u8 *)pscan_cfg->ssid_list[ssid_idx].ssid, ssid_len,
			&short_ssid);

		PRINTM(MINFO,
		       "Short-SSID (0x%0x) calculated over request SSID \"%s\"\n",
		       short_ssid, pscan_cfg->ssid_list[ssid_idx].ssid);

		if ((pcoloc_ap->ap_info.short_ssid == short_ssid) ||
		    (pcoloc_ap->ap_info.ssid.ssid_len &&
		     (pcoloc_ap->ap_info.ssid.ssid_len == ssid_len) &&
		     (!memcmp(pmadapter, pcoloc_ap->ap_info.ssid.ssid,
			      pscan_cfg->ssid_list[ssid_idx].ssid,
			      pcoloc_ap->ap_info.ssid.ssid_len)))) {
			PRINTM(MINFO,
			       "Colocated AP SSID matches scan request!\n");
			return MTRUE;
		}
	}

	return MFALSE;
}

/**
 *  @brief check if need to scan PSC channels
 *
 *  @param pmadapter  A pointer to mlan_adapter
 *  @param pscan_cfg  MNULL or pointer to scan configuration parameters
 *
 *  @return MTRUE: need to scan PSC channels, MFALSE: otherwise
 *
 */
static t_bool wlan_need_scan_psc(mlan_adapter *pmadapter,
				 const wlan_user_scan_cfg *pscan_cfg)
{
	t_bool need_scan_psc = MTRUE;
	wlan_6e_coloc_ap_t *pcoloc_ap = MNULL;

	/* PSC channels should not be scanned in case
	 * a) all APs in the same ESS are colocatedd
	 * b) direct scan with 1 SSID and at least one
	 * of the reported colocated APs with same SSID
	 */
	pcoloc_ap =
		(wlan_6e_coloc_ap_t *)util_peek_list(pmadapter->pmoal_handle,
						     &pmadapter->coloc_ap_list,
						     MNULL, MNULL);

	while (pcoloc_ap &&
	       pcoloc_ap != (wlan_6e_coloc_ap_t *)&pmadapter->coloc_ap_list) {
		if (pcoloc_ap->ap_info.bss_params.colocated_ess &&
		    wlan_scan_rnr_coloc_ap_ssid_match(pmadapter, pscan_cfg,
						      pcoloc_ap)) {
			need_scan_psc = MFALSE;
			break;
		}

		pcoloc_ap = pcoloc_ap->pnext;
	}

	return need_scan_psc;
}

/**
 *  @brief check if the channel is a 6GHz PSC
 *
 *  The Preferred Scanning Channels (PSC) are defined in
 *  IEEE 802.11ax-2021, 26.17.2.3.3
 *
 *  @param chan Channel to check
 *
 *  @return MTRUE: channel is 6GHz PSC, MFALSE: otherwise
 *
 */
static t_bool wlan_chan_is_6g_psc(t_u8 chan)
{
	return chan % 16 == 5;
}

/**
 *  @brief check if the channel is a 6GHz channel and shall be
 *  added in to scan channel list
 *
 *  @param pmadapter     A pointer to mlan_adapter
 *  @param chan          Channel to check
 *  @param need_scan_psc indicate if scan PSC channel or not
 *
 *  @return MTRUE: add this 6GHz channel, MFALSE: otherwise
 *
 */
static t_bool wlan_scan_add_6g_chan(mlan_adapter *pmadapter, t_u8 chan,
				    t_bool need_scan_psc, t_u8 *rnr_flag)
{
	/* Scan reported colocated APs channels or/and PSC channels
	 */
	if ((*rnr_flag = wlan_find_rnr_coloc_ap(pmadapter, chan)) ||
	    (need_scan_psc && wlan_chan_is_6g_psc(chan)))
		return MTRUE;

	return MFALSE;
}

/**
 *  @brief append 6g scan params tlv
 *
 *  @param pmpriv           A pointer to mlan_private structure
 *  @param ptlv_pos         tlv buf
 *  @param pscan_cfg_out    a pointer to wlan_scan_cmd_config
 *  @param channel          channel
 *
 *  @return MTRUE: channel is 6GHz PSC, MFALSE: otherwise
 *
 */
static t_u16 wlan_fill_6g_scan_params(mlan_private *pmpriv, t_u8 *ptlv_pos,
				      wlan_scan_cmd_config *pscan_cfg_out,
				      t_u8 channel)
{
	MrvlIEtypes_6g_scan_params_t *p6g_scan_params;
	t_u16 tlv_len = 0;
	t_u8 i;

	for (i = 0; i < pscan_cfg_out->num_6g_scan_params; i++) {
		if (pscan_cfg_out->scan_param_list[i].channel == channel) {
			p6g_scan_params =
				(MrvlIEtypes_6g_scan_params_t *)(ptlv_pos +
								 tlv_len);
			p6g_scan_params->header.type =
				wlan_cpu_to_le16(TLV_TYPE_6G_SCAN_PARAMS);
			p6g_scan_params->header.len = wlan_cpu_to_le16(
				sizeof(MrvlIEtypes_6g_scan_params_t) -
				sizeof(MrvlIEtypesHeader_t));
			p6g_scan_params->flags = wlan_cpu_to_le16(
				pscan_cfg_out->scan_param_list[i].flags);
			p6g_scan_params->short_ssid = wlan_cpu_to_le16(
				pscan_cfg_out->scan_param_list[i].short_ssid);
			memcpy_ext(pmpriv->adapter, p6g_scan_params->bssid,
				   pscan_cfg_out->scan_param_list[i].bssid,
				   MLAN_MAC_ADDR_LENGTH, MLAN_MAC_ADDR_LENGTH);
			tlv_len += sizeof(MrvlIEtypes_6g_scan_params_t);
			PRINTM(MCMND,
			       "6G Scan Params: flags=0x%0x, short_ssid=0x%0x, bssid=" MACSTR
			       "\n",
			       p6g_scan_params->flags,
			       p6g_scan_params->short_ssid,
			       MAC2STR(p6g_scan_params->bssid));
		}
	}
	return tlv_len;
}

/**
 *  @brief Create a 6G scan prameters list for the driver to scan based on 6E
 * RNR info
 *
 *  Use the colocated ap list kept after 2.4G/5G scan to construct a list
 *    of 6G scan prameter list for 6G scan.
 *
 *  @param pmadapter      A pointer to mlan_adapter
 *  @param pscan_cfg_out  A pointer to wlan_scan_cmd_config
 *
 *  @return                 num of 6G scan prameters
 */
static t_u8 wlan_scan_create_6g_scan_params(mlan_adapter *pmadapter,
					    wlan_scan_cmd_config *pscan_cfg_out)
{
	wlan_6e_coloc_ap_t *pcoloc_ap = MNULL;
	t_u8 num_6g_scan_params = 0;

	ENTER();

	memset(pmadapter, (t_u8 *)pscan_cfg_out->scan_param_list, 0,
	       sizeof(wlan_6g_scan_params) * WLAN_MAX_6G_SCAN_PARAMS_LIST);

	if (pmadapter->scan_6g) {
		pcoloc_ap = (wlan_6e_coloc_ap_t *)util_peek_list(
			pmadapter->pmoal_handle, &pmadapter->coloc_ap_list,
			MNULL, MNULL);

		while (pcoloc_ap &&
		       pcoloc_ap !=
			       (wlan_6e_coloc_ap_t *)&pmadapter->coloc_ap_list) {
			if (num_6g_scan_params < WLAN_MAX_6G_SCAN_PARAMS_LIST) {
				pscan_cfg_out
					->scan_param_list[num_6g_scan_params]
					.channel =
					pcoloc_ap->ap_info.chan_number;
				pscan_cfg_out
					->scan_param_list[num_6g_scan_params]
					.short_ssid =
					pcoloc_ap->ap_info.short_ssid;
				memcpy_ext(pmadapter,
					   pscan_cfg_out
						   ->scan_param_list
							   [num_6g_scan_params]
						   .bssid,
					   pcoloc_ap->ap_info.bssid,
					   MLAN_MAC_ADDR_LENGTH,
					   MLAN_MAC_ADDR_LENGTH);
				if (pcoloc_ap->ap_info.short_ssid)
					pscan_cfg_out
						->scan_param_list
							[num_6g_scan_params]
						.flags |= SHORT_SSID_VALID;
				if (pcoloc_ap->ap_info.bss_params
					    .unsolicited_probe)
					pscan_cfg_out
						->scan_param_list
							[num_6g_scan_params]
						.flags |= UNSOLICITED_PROBE;
				num_6g_scan_params++;
			}
			pcoloc_ap = pcoloc_ap->pnext;
		}
	}

	LEAVE();
	return num_6g_scan_params;
}

/**
 *  @brief Create a channel list for the driver to scan based on region info
 *
 *  Use the driver region/band information to construct a comprehensive list
 *    of channels to scan.  This routine is used for any scan that is not
 *    provided a specific channel list to scan.
 *
 *  @param pmpriv           A pointer to mlan_private structure
 *  @param puser_scan_in    MNULL or pointer to scan configuration parameters
 *  @param pscan_chan_list  Output parameter: Resulting channel list to scan
 *  @param filtered_scan    Flag indicating whether or not a BSSID or SSID
 * filter is being sent in the command to firmware.  Used to increase the number
 * of channels sent in a scan command and to disable the firmware channel scan
 *                          filter.
 *
 *  @return                 num of channel
 */
static t_u8 wlan_scan_create_channel_list(
	mlan_private *pmpriv, const wlan_user_scan_cfg *puser_scan_in,
	ChanScanParamSet_t *pscan_chan_list, t_u8 filtered_scan)
{
	mlan_adapter *pmadapter = pmpriv->adapter;
	region_chan_t *pscan_region;
	chan_freq_power_t *cfp;
	t_u32 region_idx;
	t_u32 chan_idx = 0;
	t_u32 next_chan;
	t_u8 scan_type;
	t_u8 radio_type;
	t_u16 band;
	t_u16 scan_dur = 0;
	t_bool need_scan_psc = MTRUE;
	t_u8 rnr_flag = 0;

	ENTER();

	for (region_idx = 0; region_idx < NELEMENTS(pmadapter->region_channel);
	     region_idx++) {
		if (wlan_11d_is_enabled(pmpriv) &&
		    pmpriv->media_connected != MTRUE) {
			/* Scan all the supported chan for the first scan */
			if (!pmadapter->universal_channel[region_idx].valid)
				continue;
			pscan_region =
				&pmadapter->universal_channel[region_idx];
		} else {
			if (!pmadapter->region_channel[region_idx].valid)
				continue;
			pscan_region = &pmadapter->region_channel[region_idx];
		}

		if (pmadapter->wifi_6g_scan_split) {
			if (!pmadapter->scan_6g) {
				if (pscan_region->band == BAND_6G)
					continue;
			} else {
				if (pscan_region->band != BAND_6G)
					continue;
			}
		}

		if (puser_scan_in && !puser_scan_in->chan_list[0].chan_number &&
		    puser_scan_in->chan_list[0].radio_type & BAND_SPECIFIED) {
			radio_type = puser_scan_in->chan_list[0].radio_type &
				     ~BAND_SPECIFIED;
			if (!radio_type && (pscan_region->band != BAND_B) &&
			    (pscan_region->band != BAND_G))
				continue;
			if (radio_type && (pscan_region->band != BAND_A))
				continue;
		}
		PRINTM(MCMD_D,
		       "create_channel_list: region=%d band=%d num_cfp=%d\n",
		       pscan_region->region, pscan_region->band,
		       pscan_region->num_cfp);
		band = pmpriv->config_bands;
		if (!wlan_is_band_compatible(band, pscan_region->band))
			continue;

		if (pscan_region->band == BAND_6G) {
			if (pmadapter->wifi_6g_scan_coloc_ap) {
				need_scan_psc = wlan_need_scan_psc(
					pmadapter, puser_scan_in);
				PRINTM(MCMD_D, "Scan %s\n",
				       (!need_scan_psc) ?
					       "6 GHz co-located AP channels" :
					       "6 GHz co-located AP channels and PSC channels");
			} else
				PRINTM(MCMD_D, "Scan all 6 GHz channels\n");
		}

		for (next_chan = 0; next_chan < pscan_region->num_cfp;
		     next_chan++) {
			/* Set the default scan type to the user specified type,
			 * will later be changed to passive on a per channel
			 * basis if restricted by regulatory requirements (11d
			 * or 11h)
			 */
			scan_type = pmadapter->scan_type;
			cfp = pscan_region->pcfp + next_chan;
			if (cfp->dynamic.flags & NXP_CHANNEL_DISABLED)
				continue;

			if ((pscan_region->band == BAND_6G) &&
			    (pmadapter->wifi_6g_scan_coloc_ap)) {
				if (!wlan_scan_add_6g_chan(
					    pmadapter, (t_u8)cfp->channel,
					    need_scan_psc, &rnr_flag))
					continue;
			}
			pscan_chan_list[chan_idx].chan_scan_mode.rnr_flag =
				rnr_flag;

			if (wlan_is_chan_passive(pmpriv, pscan_region->band,
						 (t_u8)cfp->channel)) {
				/* do not send probe requests on this channel */
				scan_type = MLAN_SCAN_TYPE_PASSIVE;
			}
			switch (pscan_region->band) {
			case BAND_A:
				pscan_chan_list[chan_idx].bandcfg.chanBand =
					BAND_5GHZ;
				/* Passive scan on DFS channels */
				if (wlan_11h_radar_detect_required(
					    pmpriv, (t_u8)cfp->channel) &&
				    scan_type == MLAN_SCAN_TYPE_PASSIVE)
					scan_type =
						MLAN_SCAN_TYPE_PASSIVE_TO_ACTIVE;
				break;
			case BAND_6G:
				if (pscan_chan_list[chan_idx]
					    .chan_scan_mode.rnr_flag)
					scan_type = MLAN_SCAN_TYPE_ACTIVE;
				else
					scan_type = MLAN_SCAN_TYPE_PASSIVE;
				pscan_chan_list[chan_idx].bandcfg.chanBand =
					BAND_6GHZ;
				break;
			case BAND_B:
			case BAND_G:
				if (wlan_bg_scan_type_is_passive(
					    pmpriv, (t_u8)cfp->channel)) {
					scan_type = MLAN_SCAN_TYPE_PASSIVE;
				}
				pscan_chan_list[chan_idx].bandcfg.chanBand =
					BAND_2GHZ;
				break;
			default:
				pscan_chan_list[chan_idx].bandcfg.chanBand =
					BAND_2GHZ;
				break;
			}

			if (puser_scan_in &&
			    puser_scan_in->chan_list[0].scan_time) {
				scan_dur = (t_u16)puser_scan_in->chan_list[0]
						   .scan_time;
			} else if (scan_type == MLAN_SCAN_TYPE_PASSIVE ||
				   scan_type ==
					   MLAN_SCAN_TYPE_PASSIVE_TO_ACTIVE) {
				scan_dur = pmadapter->passive_scan_time;
			} else if (filtered_scan) {
				scan_dur = pmadapter->specific_scan_time;
			} else {
				scan_dur = pmadapter->active_scan_time;
			}
			if (scan_type == MLAN_SCAN_TYPE_PASSIVE_TO_ACTIVE &&
			    pmadapter->passive_to_active_scan ==
				    MLAN_PASS_TO_ACT_SCAN_EN) {
				scan_dur = MAX(scan_dur,
					       MIN_PASSIVE_TO_ACTIVE_SCAN_TIME);
				pscan_chan_list[chan_idx]
					.chan_scan_mode.passive_to_active_scan =
					MTRUE;
			}
			if (pscan_region->band == BAND_6G)
				scan_dur = pmadapter->wifi_6g_scan_time;

			pscan_chan_list[chan_idx].max_scan_time =
				wlan_cpu_to_le16(scan_dur);

			if (scan_type == MLAN_SCAN_TYPE_PASSIVE ||
			    scan_type == MLAN_SCAN_TYPE_PASSIVE_TO_ACTIVE) {
				pscan_chan_list[chan_idx]
					.chan_scan_mode.passive_scan = MTRUE;
				pscan_chan_list[chan_idx]
					.chan_scan_mode.hidden_ssid_report =
					MTRUE;
			} else {
				pscan_chan_list[chan_idx]
					.chan_scan_mode.passive_scan = MFALSE;
			}

			pscan_chan_list[chan_idx].chan_number =
				(t_u8)cfp->channel;
			PRINTM(MCMD_D,
			       "chan=%d, mode=%d, passive_to_active=%d\n",
			       pscan_chan_list[chan_idx].chan_number,
			       pscan_chan_list[chan_idx]
				       .chan_scan_mode.passive_scan,
			       pscan_chan_list[chan_idx]
				       .chan_scan_mode.passive_to_active_scan);
			chan_idx++;
		}
	}

	LEAVE();
	return chan_idx;
}

/**
 *  @brief Add WPS IE to probe request frame
 *
 *  @param pmpriv             A pointer to mlan_private structure
 *  @param pptlv_out          A pointer to TLV to fill in
 *
 *  @return                   N/A
 */
static void wlan_add_wps_probe_request_ie(mlan_private *pmpriv,
					  t_u8 **pptlv_out)
{
	MrvlIEtypesHeader_t *tlv;

	ENTER();

	if (pmpriv->wps.wps_ie.vend_hdr.len) {
		tlv = (MrvlIEtypesHeader_t *)*pptlv_out;
		tlv->type = wlan_cpu_to_le16(VENDOR_SPECIFIC_221);
		tlv->len = wlan_cpu_to_le16(pmpriv->wps.wps_ie.vend_hdr.len);
		*pptlv_out += sizeof(MrvlIEtypesHeader_t);
		memcpy_ext(pmpriv->adapter, *pptlv_out,
			   pmpriv->wps.wps_ie.vend_hdr.oui,
			   pmpriv->wps.wps_ie.vend_hdr.len,
			   pmpriv->wps.wps_ie.vend_hdr.len);
		*pptlv_out += (pmpriv->wps.wps_ie.vend_hdr.len +
			       sizeof(MrvlIEtypesHeader_t));
	}
	LEAVE();
}

/**
 *  @brief Construct and send multiple scan config commands to the firmware
 *
 *  Previous routines have created a wlan_scan_cmd_config with any requested
 *   TLVs.  This function splits the channel TLV into max_chan_per_scan lists
 *   and sends the portion of the channel TLV along with the other TLVs
 *   to the wlan_cmd routines for execution in the firmware.
 *
 *  @param pmpriv             A pointer to mlan_private structure
 *  @param pioctl_buf         A pointer to MLAN IOCTL Request buffer
 *  @param max_chan_per_scan  Maximum number channels to be included in each
 *                            scan command sent to firmware
 *  @param filtered_scan      Flag indicating whether or not a BSSID or SSID
 *                            filter is being used for the firmware command
 *                            scan command sent to firmware
 *  @param pscan_cfg_out      Scan configuration used for this scan.
 *  @param pchan_tlv_out      Pointer in the pscan_cfg_out where the channel TLV
 *                            should start.  This is past any other TLVs that
 *                            must be sent down in each firmware command.
 *  @param pscan_chan_list    List of channels to scan in max_chan_per_scan
 * segments
 *
 *  @return                   MLAN_STATUS_SUCCESS or error return otherwise
 */
static mlan_status
wlan_scan_channel_list(mlan_private *pmpriv, t_void *pioctl_buf,
		       t_u32 max_chan_per_scan, t_u8 filtered_scan,
		       wlan_scan_cmd_config *pscan_cfg_out,
		       MrvlIEtypes_ChanListParamSet_t *pchan_tlv_out,
		       ChanScanParamSet_t *pscan_chan_list)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_adapter *pmadapter = pmpriv->adapter;
	ChanScanParamSet_t *ptmp_chan_list;
	ChanScanParamSet_t *pstart_chan;
	pmlan_ioctl_req pioctl_req = (mlan_ioctl_req *)pioctl_buf;
	t_u8 *pchan_tlv_out_temp = MNULL;
	t_u8 *ptlv_temp = MNULL;
	t_bool foundJPch14 = MFALSE;
	t_u16 tlv_buf_len = 0;
	t_u32 tlv_idx;
	t_u32 total_scan_time;
	t_u32 done_early;
	t_u32 cmd_no;
	t_u32 first_chan = 1;
	t_u8 *ptlv_pos;
	t_s32 length = 0;
	t_s64 tmp_len = 0;
	MrvlIETypes_HTCap_t *pht_cap;

	MrvlIETypes_VHTCap_t *pvht_cap;
	MrvlIEtypes_Extension_t *phe_cap;
	t_u16 len = 0;
	MrvlIEtypes_He_6g_cap_t *phe_6g_cap;
	t_u8 scan_rnr_chan = MFALSE;
	t_u8 skip_rnr_chan = MFALSE;
	t_u8 chan_rnr_flag = MFALSE;
	t_u8 radio_type = 0;
	t_u8 channel = 0;
	t_s32 offset = 0;

	mlan_callbacks *pcb = (mlan_callbacks *)&pmadapter->callbacks;

	ENTER();

	if (!pscan_cfg_out || !pchan_tlv_out || !pscan_chan_list) {
		PRINTM(MINFO, "Scan: Null detect: %p, %p, %p\n", pscan_cfg_out,
		       pchan_tlv_out, pscan_chan_list);
		if (pioctl_req)
			pioctl_req->status_code = MLAN_ERROR_CMD_SCAN_FAIL;
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}
	if (!pscan_chan_list->chan_number) {
		PRINTM(MERROR, "Scan: No channel configured\n");
		if (pioctl_req)
			pioctl_req->status_code = MLAN_ERROR_CMD_SCAN_FAIL;
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}

	/* check expiry before preparing scan list - may affect blacklist */
	wlan_11h_get_csa_closed_channel(pmpriv);
	if (pscan_cfg_out->num_6g_scan_params) {
		scan_rnr_chan = MFALSE;
		skip_rnr_chan = MTRUE;
	}

process_start:
	pchan_tlv_out->header.type = wlan_cpu_to_le16(TLV_TYPE_CHANLIST);

	/* Set the temp channel struct pointer to the start of the desired list
	 */
	ptmp_chan_list = pscan_chan_list;

	/*
	 * Loop through the desired channel list, sending a new firmware scan
	 * commands for each max_chan_per_scan channels (or for 1,6,11
	 * individually if configured accordingly)
	 */
	while (ptmp_chan_list->chan_number) {
		tlv_idx = 0;
		total_scan_time = 0;
		pchan_tlv_out->header.len = 0;
		pstart_chan = ptmp_chan_list;
		done_early = MFALSE;

		if (scan_rnr_chan && !ptmp_chan_list->chan_scan_mode.rnr_flag) {
			ptmp_chan_list++;
			continue;
		}

		/*
		 * Construct the Channel TLV for the scan command.  Continue to
		 * insert channel TLVs until:
		 *   - the tlv_idx hits the maximum configured per scan command
		 *   - the next channel to insert is 0 (end of desired
		 *     channel list)
		 *   - done_early is set (controlling individual
		 *     scanning of 1,6,11)
		 */
		while (tlv_idx < max_chan_per_scan &&
		       ptmp_chan_list->chan_number && !done_early) {
			if (wlan_is_chan_blacklisted(
				    pmpriv,
				    radio_type_to_band(
					    ptmp_chan_list->bandcfg.chanBand),
				    ptmp_chan_list->chan_number) ||
			    wlan_is_chan_disabled(
				    pmpriv,
				    radio_type_to_band(
					    ptmp_chan_list->bandcfg.chanBand),
				    ptmp_chan_list->chan_number)) {
				PRINTM(MCMND, "Block scan chan = %d\n",
				       ptmp_chan_list->chan_number);
				ptmp_chan_list++;
				continue;
			}

			/** skip rnr channel */
			if (skip_rnr_chan &&
			    ptmp_chan_list->chan_scan_mode.rnr_flag) {
				ptmp_chan_list++;
				continue;
			}

			if (first_chan) {
				ptmp_chan_list->chan_scan_mode.first_chan =
					MTRUE;
				first_chan = 0;
			}
			radio_type = ptmp_chan_list->bandcfg.chanBand;
			channel = ptmp_chan_list->chan_number;
			PRINTM(MCMD_D,
			       "Scan: Chan(%3d), bandcfg(%x), Mode(%d,%d,%d), Dur(%d)\n",
			       ptmp_chan_list->chan_number,
			       ptmp_chan_list->bandcfg,
			       ptmp_chan_list->chan_scan_mode.passive_scan,
			       ptmp_chan_list->chan_scan_mode.disable_chan_filt,
			       ptmp_chan_list->chan_scan_mode
				       .passive_to_active_scan,
			       wlan_le16_to_cpu(ptmp_chan_list->max_scan_time));

			if (foundJPch14 == MTRUE) {
				foundJPch14 = MFALSE;
				/* Restore the TLV buffer */
				pchan_tlv_out =
					(MrvlIEtypes_ChanListParamSet_t *)
						pchan_tlv_out_temp;
				pchan_tlv_out->header.type =
					wlan_cpu_to_le16(TLV_TYPE_CHANLIST);
				pchan_tlv_out->header.len = 0;
				if (ptlv_temp) {
					memcpy_ext(pmadapter,
						   pscan_cfg_out->tlv_buf,
						   ptlv_temp, tlv_buf_len,
						   tlv_buf_len);
					pcb->moal_mfree(pmadapter->pmoal_handle,
							ptlv_temp);
					ptlv_temp = MNULL;
				}
			}

			/* Special Case: For Japan, Scan on CH14 for 11G rates
			   is not allowed
			    Hence Rates TLV needs to be updated to support only
			   11B rates */
			if ((pmadapter->region_code == COUNTRY_CODE_JP_40 ||
			     pmadapter->region_code == COUNTRY_CODE_JP_FF) &&
			    (ptmp_chan_list->chan_number == 14) &&
			    (pmadapter->ext_scan_type != EXT_SCAN_ENHANCE)) {
				t_u8 *ptlv_pos = pscan_cfg_out->tlv_buf;
				t_u16 old_ratetlv_len, new_ratetlv_len;
				MrvlIEtypesHeader_t *header;
				MrvlIEtypes_RatesParamSet_t *prates_tlv;

				/* Preserve the current TLV buffer */
				ret = pcb->moal_malloc(
					pmadapter->pmoal_handle,
					MAX_SCAN_CFG_ALLOC - CHAN_TLV_MAX_SIZE,
					MLAN_MEM_DEF | MOAL_MEM_FLAG_ATOMIC,
					(t_u8 **)&ptlv_temp);
				if (ret != MLAN_STATUS_SUCCESS || !ptlv_temp) {
					PRINTM(MERROR,
					       "Memory allocation for pscan_cfg_out failed!\n");
					if (pioctl_req)
						pioctl_req->status_code =
							MLAN_ERROR_NO_MEM;
					LEAVE();
					return MLAN_STATUS_FAILURE;
				}
				pchan_tlv_out_temp = (t_u8 *)pchan_tlv_out;
				offset = pchan_tlv_out_temp -
					 pscan_cfg_out->tlv_buf;
				tlv_buf_len = (t_u32)offset;
				memcpy_ext(pmadapter, ptlv_temp, ptlv_pos,
					   tlv_buf_len,
					   MAX_SCAN_CFG_ALLOC -
						   CHAN_TLV_MAX_SIZE);

				/* Search for Rates TLV */
				while ((!foundJPch14) &&
				       (ptlv_pos < pchan_tlv_out_temp)) {
					header =
						(MrvlIEtypesHeader_t *)ptlv_pos;
					if (header->type ==
					    wlan_cpu_to_le16(TLV_TYPE_RATES))
						foundJPch14 = MTRUE;
					else
						ptlv_pos +=
							(sizeof(MrvlIEtypesHeader_t) +
							 wlan_le16_to_cpu(
								 header->len));
				}

				if (foundJPch14) {
					/* Update the TLV buffer with *new*
					 * Rates TLV and rearrange remaining TLV
					 * buffer*/
					prates_tlv =
						(MrvlIEtypes_RatesParamSet_t *)
							ptlv_pos;
					old_ratetlv_len =
						sizeof(MrvlIEtypesHeader_t) +
						wlan_le16_to_cpu(
							prates_tlv->header.len);

					prates_tlv->header.len = wlan_copy_rates(
						prates_tlv->rates, 0,
						SupportedRates_B,
						sizeof(SupportedRates_B));
					new_ratetlv_len =
						sizeof(MrvlIEtypesHeader_t) +
						prates_tlv->header.len;
					prates_tlv->header.len =
						wlan_cpu_to_le16(
							prates_tlv->header.len);

					memmove(pmadapter,
						ptlv_pos + new_ratetlv_len,
						ptlv_pos + old_ratetlv_len,
						(t_u32)(pchan_tlv_out_temp -
							(ptlv_pos +
							 old_ratetlv_len)));
					pchan_tlv_out =
						(MrvlIEtypes_ChanListParamSet_t
							 *)(pchan_tlv_out_temp -
							    (old_ratetlv_len -
							     new_ratetlv_len));
					pchan_tlv_out->header.type =
						wlan_cpu_to_le16(
							TLV_TYPE_CHANLIST);
					pchan_tlv_out->header.len = 0;
				}
			}

			/* Copy the current channel TLV to the command being
			 * prepared */
			memcpy_ext(pmadapter,
				   pchan_tlv_out->chan_scan_param + tlv_idx,
				   ptmp_chan_list, sizeof(ChanScanParamSet_t),
				   sizeof(ChanScanParamSet_t));

			/* Increment the TLV header length by the size appended
			 */
			pchan_tlv_out->header.len += sizeof(ChanScanParamSet_t);

			/*
			 * The tlv buffer length is set to the number of
			 * bytes of the between the channel tlv pointer
			 * and the start of the tlv buffer.  This
			 * compensates for any TLVs that were appended
			 * before the channel list.
			 */
			length = (t_u8 *)pchan_tlv_out - pscan_cfg_out->tlv_buf;
			pscan_cfg_out->tlv_buf_len = (t_u32)(length);

			/* Add the size of the channel tlv header and the data
			 * length */
			pscan_cfg_out->tlv_buf_len +=
				(sizeof(pchan_tlv_out->header) +
				 pchan_tlv_out->header.len);

			/* Increment the index to the channel tlv we are
			 * constructing */
			tlv_idx++;

			/* Count the total scan time per command */
			total_scan_time +=
				wlan_le16_to_cpu(ptmp_chan_list->max_scan_time);

			done_early = MFALSE;

			/*
			 * Stop the loop if the *current* channel is in the
			 * 1,6,11 set and we are not filtering on a BSSID or
			 * SSID.
			 */
			if (!filtered_scan &&
			    (ptmp_chan_list->chan_number == 1 ||
			     ptmp_chan_list->chan_number == 6 ||
			     ptmp_chan_list->chan_number == 11)) {
				done_early = MTRUE;
			}

			/*
			 * Stop the loop if the *current* channel is 14
			 * and region code is Japan (0x40 or 0xFF)
			 */
			if ((pmadapter->region_code == COUNTRY_CODE_JP_40 ||
			     pmadapter->region_code == COUNTRY_CODE_JP_FF) &&
			    (ptmp_chan_list->chan_number == 14)) {
				done_early = MTRUE;
			}

			/** send rnr scan per channel */
			if (ptmp_chan_list->chan_scan_mode.rnr_flag) {
				done_early = MTRUE;
				chan_rnr_flag = MTRUE;
				/** clear rnr_flag*/
				ptmp_chan_list->chan_scan_mode.rnr_flag = 0;
			} else
				chan_rnr_flag = MFALSE;

			/* Increment the tmp pointer to the next channel to be
			 * scanned */
			ptmp_chan_list++;

			/*
			 * Stop the loop if the *next* channel is 6G channel
			 */
			if (ptmp_chan_list->bandcfg.chanBand == BAND_6GHZ &&
			    radio_type != BAND_6GHZ)
				done_early = MTRUE;

			/*
			 * Stop the loop if the *next* channel is in the 1,6,11
			 * set. This will cause it to be the only channel
			 * scanned on the next interation
			 */
			if (!filtered_scan &&
			    (ptmp_chan_list->chan_number == 1 ||
			     ptmp_chan_list->chan_number == 6 ||
			     ptmp_chan_list->chan_number == 11)) {
				done_early = MTRUE;
			}

			/*
			 * Stop the loop if the *next* channel is 14
			 * and region code is Japan (0x40 or 0xFF)
			 */
			if ((pmadapter->region_code == COUNTRY_CODE_JP_40 ||
			     pmadapter->region_code == COUNTRY_CODE_JP_FF) &&
			    (ptmp_chan_list->chan_number == 14)) {
				done_early = MTRUE;
			}
			if (pmadapter->ext_scan && pmadapter->ext_scan_enh &&
			    pmadapter->ext_scan_type == EXT_SCAN_ENHANCE &&
			    (!chan_rnr_flag))
				done_early = MFALSE;

			/*
			 * Stop the loop if the *next* channel is of different
			 * Band
			 */
			// coverity[overflow_sink:SUPPRESS]
			if (ptmp_chan_list->bandcfg.chanBand !=
			    ((ChanScanParamSet_t *)(ptmp_chan_list - 1))
				    ->bandcfg.chanBand) {
				done_early = MTRUE;
			}
		}

		/* The total scan time should be less than scan command timeout
		 * value */
		if (!total_scan_time ||
		    total_scan_time > MRVDRV_MAX_TOTAL_SCAN_TIME) {
			PRINTM(MMSG,
			       "Total scan time %d ms is invalid, limit (%d ms), scan skipped\n",
			       total_scan_time, MRVDRV_MAX_TOTAL_SCAN_TIME);
			if (pioctl_req)
				pioctl_req->status_code =
					MLAN_ERROR_CMD_SCAN_FAIL;
			ret = MLAN_STATUS_FAILURE;
			break;
		}
		ptlv_pos = (t_u8 *)pchan_tlv_out + pchan_tlv_out->header.len +
			   sizeof(MrvlIEtypesHeader_t);

		pchan_tlv_out->header.len =
			wlan_cpu_to_le16(pchan_tlv_out->header.len);

		if (ISSUPP_11NENABLED(pmpriv->adapter->fw_cap_info) &&
		    (radio_type != BAND_6GHZ) &&
		    (pmpriv->config_bands & BAND_GN ||
		     pmpriv->config_bands & BAND_AN)) {
			pht_cap = (MrvlIETypes_HTCap_t *)ptlv_pos;
			memset(pmadapter, pht_cap, 0,
			       sizeof(MrvlIETypes_HTCap_t));
			pht_cap->header.type = wlan_cpu_to_le16(HT_CAPABILITY);
			pht_cap->header.len = sizeof(HTCap_t);
			wlan_fill_ht_cap_tlv(pmpriv, pht_cap,
					     pmpriv->config_bands, MTRUE);
			HEXDUMP("SCAN: HT_CAPABILITIES IE", (t_u8 *)pht_cap,
				sizeof(MrvlIETypes_HTCap_t));
			ptlv_pos += sizeof(MrvlIETypes_HTCap_t);
			pht_cap->header.len =
				wlan_cpu_to_le16(pht_cap->header.len);
		}

		if (ISSUPP_11ACENABLED(pmpriv->adapter->fw_cap_info) &&
		    (radio_type != BAND_6GHZ) &&
		    (pmpriv->config_bands & BAND_AAC)) {
			pvht_cap = (MrvlIETypes_VHTCap_t *)ptlv_pos;
			memset(pmadapter, pvht_cap, 0,
			       sizeof(MrvlIETypes_VHTCap_t));
			pvht_cap->header.type =
				wlan_cpu_to_le16(VHT_CAPABILITY);
			pvht_cap->header.len = sizeof(VHT_capa_t);
			wlan_fill_vht_cap_tlv(pmpriv, pvht_cap,
					      pmpriv->config_bands, MFALSE,
					      MFALSE);
			HEXDUMP("SCAN: VHT_CAPABILITIES IE", (t_u8 *)pvht_cap,
				sizeof(MrvlIETypes_VHTCap_t));
			ptlv_pos += sizeof(MrvlIETypes_VHTCap_t);
			pvht_cap->header.len =
				wlan_cpu_to_le16(pvht_cap->header.len);
		}

		if (IS_FW_SUPPORT_11AX(pmadapter) &&
		    ((pmpriv->config_bands & BAND_GAX) ||
		     (pmpriv->config_bands & BAND_AAX))) {
			t_u16 select_band =
				(radio_type == BAND_5GHZ ? BAND_AAX : BAND_GAX);
			phe_cap = (MrvlIEtypes_Extension_t *)ptlv_pos;
			len = wlan_fill_he_cap_tlv(pmpriv, select_band, phe_cap,
						   MFALSE);
			HEXDUMP("SCAN: HE_CAPABILITIES IE", (t_u8 *)phe_cap,
				len);
			ptlv_pos += len;
		}
		if (IS_FW_SUPPORT_6G(pmadapter) && (radio_type == BAND_6GHZ)) {
			phe_6g_cap = (MrvlIEtypes_He_6g_cap_t *)ptlv_pos;
			memset(pmadapter, phe_6g_cap, 0,
			       sizeof(MrvlIEtypes_He_6g_cap_t));
			wlan_fill_he_6g_cap_tlv(pmpriv, phe_6g_cap);
			HEXDUMP("SCAN: HE_6G_CAPABILITIES IE",
				(t_u8 *)phe_6g_cap,
				sizeof(MrvlIEtypes_He_6g_cap_t));
			ptlv_pos += sizeof(MrvlIEtypes_He_6g_cap_t);
			if (scan_rnr_chan && chan_rnr_flag) {
				len = wlan_fill_6g_scan_params(pmpriv, ptlv_pos,
							       pscan_cfg_out,
							       channel);
				ptlv_pos += len;
			}
		}
		tmp_len = (t_u8 *)ptlv_pos - pscan_cfg_out->tlv_buf;
		pscan_cfg_out->tlv_buf_len = (t_u32)tmp_len;

		pmadapter->pscan_channels = pstart_chan;

		/* Send the scan command to the firmware with the specified cfg
		 */
		if (pmadapter->ext_scan)
			cmd_no = HostCmd_CMD_802_11_SCAN_EXT;
		else
			cmd_no = HostCmd_CMD_802_11_SCAN;
		ret = wlan_prepare_cmd(pmpriv, cmd_no, HostCmd_ACT_GEN_SET, 0,
				       MNULL, pscan_cfg_out);
		if (ret)
			break;
	}

	/** scan_rnr_chan */
	if (skip_rnr_chan) {
		skip_rnr_chan = MFALSE;
		scan_rnr_chan = MTRUE;
		goto process_start;
	}

	LEAVE();

	if (ptlv_temp)
		pcb->moal_mfree(pmadapter->pmoal_handle, ptlv_temp);

	if (ret)
		return MLAN_STATUS_FAILURE;

	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief Construct a wlan_scan_cmd_config structure to use in scan commands
 *
 *  Application layer or other functions can invoke wlan_scan_networks
 *    with a scan configuration supplied in a wlan_ioctl_user_scan_cfg struct.
 *    This structure is used as the basis of one or many wlan_scan_cmd_config
 *    commands that are sent to the command processing module and sent to
 *    firmware.
 *
 *  Create a wlan_scan_cmd_config based on the following user supplied
 *    parameters (if present):
 *             - SSID filter
 *             - BSSID filter
 *             - Number of Probes to be sent
 *             - Channel list
 *
 *  If the SSID or BSSID filter is not present, disable/clear the filter.
 *  If the number of probes is not set, use the adapter default setting
 *  Qualify the channel
 *
 *  @param pmpriv              A pointer to mlan_private structure
 *  @param puser_scan_in       MNULL or pointer to scan config parameters
 *  @param pscan_cfg_out       Output parameter: Resulting scan configuration
 *  @param ppchan_list_out     Output parameter: Pointer to the start of the
 *                             channel TLV portion of the output scan config
 *  @param pscan_chan_list     Output parameter: Pointer to the resulting
 *                             channel list to scan
 *  @param pmax_chan_per_scan  Output parameter: Number of channels to scan for
 *                             each issuance of the firmware scan command
 *  @param pfiltered_scan      Output parameter: Flag indicating whether or not
 *                             a BSSID or SSID filter is being sent in the
 *                             command to firmware. Used to increase the number
 *                             of channels sent in a scan command and to
 *                             disable the firmware channel scan filter.
 *  @param pscan_current_only  Output parameter: Flag indicating whether or not
 *                             we are only scanning our current active channel
 *
 *  @return                 MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status wlan_scan_setup_scan_config(
	mlan_private *pmpriv, wlan_user_scan_cfg *puser_scan_in,
	wlan_scan_cmd_config *pscan_cfg_out,
	MrvlIEtypes_ChanListParamSet_t **ppchan_list_out,
	ChanScanParamSet_t *pscan_chan_list, t_u8 *pmax_chan_per_scan,
	t_u8 *pfiltered_scan, t_u8 *pscan_current_only)
{
	mlan_adapter *pmadapter = pmpriv->adapter;
	mlan_status ret = MLAN_STATUS_SUCCESS;
	MrvlIEtypes_NumProbes_t *pnum_probes_tlv;
	MrvlIEtypes_WildCardSsIdParamSet_t *pwildcard_ssid_tlv;
	MrvlIEtypes_RatesParamSet_t *prates_tlv;
	MrvlIEtypes_Bssid_List_t *pbssid_tlv;

	const t_u8 zero_mac[MLAN_MAC_ADDR_LENGTH] = {0, 0, 0, 0, 0, 0};
	t_u8 *ptlv_pos;
	t_u32 num_probes;
	t_u32 ssid_len;
	t_u32 chan_idx = 0;
	t_u32 chan_list_idx = 0;
	t_u32 scan_type;
	t_u16 scan_dur;
	t_u8 channel;
	t_u8 radio_type;
	t_u32 ssid_idx;
	t_u8 ssid_filter;
	WLAN_802_11_RATES rates;
	t_u32 rates_size;
	MrvlIEtypes_ScanChanGap_t *pscan_gap_tlv;
	MrvlIEtypes_BssMode_t *pbss_mode;
	t_u8 num_of_channel = 0;

	ENTER();

	/* The tlv_buf_len is calculated for each scan command.  The TLVs added
	 *   in this routine will be preserved since the routine that sends
	 *   the command will append channelTLVs at *ppchan_list_out.  The
	 *   difference between the *ppchan_list_out and the tlv_buf start will
	 * be used to calculate the size of anything we add in this routine.
	 */
	pscan_cfg_out->tlv_buf_len = 0;

	/* Running tlv pointer.  Assigned to ppchan_list_out at end of function
	 *  so later routines know where channels can be added to the command
	 * buf
	 */
	ptlv_pos = pscan_cfg_out->tlv_buf;

	/* Initialize the scan as un-filtered; the flag is later set to
	 *   TRUE below if a SSID or BSSID filter is sent in the command
	 */
	*pfiltered_scan = MFALSE;

	/* Initialize the scan as not being only on the current channel.  If
	 *   the channel list is customized, only contains one channel, and
	 *   is the active channel, this is set true and data flow is not
	 * halted.
	 */
	*pscan_current_only = MFALSE;

	if (puser_scan_in) {
		ssid_filter = MFALSE;

		/* Set the bss type scan filter, use Adapter setting if unset */
		pscan_cfg_out->bss_mode =
			(puser_scan_in->bss_mode ?
				 (t_u8)puser_scan_in->bss_mode :
				 (t_u8)pmadapter->scan_mode);

		/* Set the number of probes to send, use Adapter setting if
		 * unset */
		num_probes =
			(puser_scan_in->num_probes ? puser_scan_in->num_probes :
						     pmadapter->scan_probes);
		/*
		 * Set the BSSID filter to the incoming configuration,
		 *  if non-zero.  If not set, it will remain disabled
		 * (all zeros).
		 */
		memcpy_ext(pmadapter, pscan_cfg_out->specific_bssid,
			   puser_scan_in->specific_bssid,
			   sizeof(pscan_cfg_out->specific_bssid),
			   sizeof(pscan_cfg_out->specific_bssid));

		if (pmadapter->ext_scan) {
			if (puser_scan_in->bssid_num) {
				pbssid_tlv =
					(MrvlIEtypes_Bssid_List_t *)ptlv_pos;
				pbssid_tlv->header.type = TLV_TYPE_BSSID;
				pbssid_tlv->header.len = wlan_cpu_to_le16(
					MLAN_MAC_ADDR_LENGTH *
					puser_scan_in->bssid_num);
				memcpy_ext(pmadapter, pbssid_tlv->bssid,
					   puser_scan_in->bssid_list,
					   MLAN_MAC_ADDR_LENGTH *
						   puser_scan_in->bssid_num,
					   MLAN_MAC_ADDR_LENGTH *
						   puser_scan_in->bssid_num);
				ptlv_pos += sizeof(MrvlIEtypesHeader_t) +
					    MLAN_MAC_ADDR_LENGTH *
						    puser_scan_in->bssid_num;
				DBG_HEXDUMP(
					MCMD_D, "scan bssid filter", pbssid_tlv,
					sizeof(MrvlIEtypesHeader_t) +
						MLAN_MAC_ADDR_LENGTH *
							puser_scan_in
								->bssid_num);
			} else if (memcmp(pmadapter,
					  pscan_cfg_out->specific_bssid,
					  &zero_mac, sizeof(zero_mac))) {
				pbssid_tlv =
					(MrvlIEtypes_Bssid_List_t *)ptlv_pos;
				pbssid_tlv->header.type = TLV_TYPE_BSSID;
				pbssid_tlv->header.len =
					wlan_cpu_to_le16(MLAN_MAC_ADDR_LENGTH);
				memcpy_ext(pmadapter, pbssid_tlv->bssid,
					   puser_scan_in->specific_bssid,
					   MLAN_MAC_ADDR_LENGTH,
					   MLAN_MAC_ADDR_LENGTH);
				ptlv_pos += sizeof(MrvlIEtypes_Bssid_List_t);
			}
		}

		for (ssid_idx = 0;
		     ((ssid_idx < NELEMENTS(puser_scan_in->ssid_list)) &&
		      (*puser_scan_in->ssid_list[ssid_idx].ssid ||
		       puser_scan_in->ssid_list[ssid_idx].max_len));
		     ssid_idx++) {
			ssid_len = wlan_strlen(
				(char *)puser_scan_in->ssid_list[ssid_idx].ssid);

			pwildcard_ssid_tlv =
				(MrvlIEtypes_WildCardSsIdParamSet_t *)ptlv_pos;
			pwildcard_ssid_tlv->header.type =
				wlan_cpu_to_le16(TLV_TYPE_WILDCARDSSID);
			pwildcard_ssid_tlv->header.len =
				(t_u16)(ssid_len +
					sizeof(pwildcard_ssid_tlv
						       ->max_ssid_length));
			pwildcard_ssid_tlv->max_ssid_length =
				puser_scan_in->ssid_list[ssid_idx].max_len;

			memcpy_ext(pmadapter, pwildcard_ssid_tlv->ssid,
				   puser_scan_in->ssid_list[ssid_idx].ssid,
				   ssid_len, MLAN_MAX_SSID_LENGTH);

			ptlv_pos += (sizeof(pwildcard_ssid_tlv->header) +
				     pwildcard_ssid_tlv->header.len);

			pwildcard_ssid_tlv->header.len = wlan_cpu_to_le16(
				pwildcard_ssid_tlv->header.len);

			PRINTM(MINFO, "Scan: ssid_list[%d]: %s, %d\n", ssid_idx,
			       pwildcard_ssid_tlv->ssid,
			       pwildcard_ssid_tlv->max_ssid_length);

			if (ssid_len) {
				ssid_filter = MTRUE;
				if (!puser_scan_in->ssid_list[ssid_idx].max_len) {
					PRINTM(MCMND, "user scan: %s\n",
					       pwildcard_ssid_tlv->ssid);
					puser_scan_in->ssid_filter = MTRUE;
				}
			}
		}

		/*
		 *  The default number of channels sent in the command is low to
		 *  ensure the response buffer from the firmware does not
		 *  truncate scan results.  That is not an issue with an SSID or
		 *  BSSID filter applied to the scan results in the firmware.
		 */
		if ((ssid_idx && ssid_filter) ||
		    memcmp(pmadapter, pscan_cfg_out->specific_bssid, &zero_mac,
			   sizeof(zero_mac))) {
			*pfiltered_scan = MTRUE;
		}

	} else {
		pscan_cfg_out->bss_mode = (t_u8)pmadapter->scan_mode;
		num_probes = pmadapter->scan_probes;
	}

	/*
	 *  If a specific BSSID or SSID is used, the number of channels in
	 *  the scan command will be increased to the absolute maximum.
	 */
	if (*pfiltered_scan)
		*pmax_chan_per_scan = MRVDRV_MAX_CHANNELS_PER_SPECIFIC_SCAN;
	else
		*pmax_chan_per_scan = MRVDRV_CHANNELS_PER_SCAN_CMD;

	if (puser_scan_in) {
		if (puser_scan_in->scan_chan_gap) {
			*pmax_chan_per_scan =
				MRVDRV_MAX_CHANNELS_PER_SPECIFIC_SCAN;
			PRINTM(MCMND, "Scan: channel gap = 0x%x\n",
			       puser_scan_in->scan_chan_gap);
			pscan_gap_tlv = (MrvlIEtypes_ScanChanGap_t *)ptlv_pos;
			pscan_gap_tlv->header.type =
				wlan_cpu_to_le16(TLV_TYPE_SCAN_CHANNEL_GAP);
			pscan_gap_tlv->header.len = sizeof(pscan_gap_tlv->gap);
			pscan_gap_tlv->gap = wlan_cpu_to_le16(
				(t_u16)puser_scan_in->scan_chan_gap);
			ptlv_pos += sizeof(pscan_gap_tlv->header) +
				    pscan_gap_tlv->header.len;
			pscan_gap_tlv->header.len =
				wlan_cpu_to_le16(pscan_gap_tlv->header.len);
		}
	} else if (pmadapter->scan_chan_gap) {
		*pmax_chan_per_scan = MRVDRV_MAX_CHANNELS_PER_SPECIFIC_SCAN;
		PRINTM(MCMND, "Scan: channel gap = 0x%x\n",
		       pmadapter->scan_chan_gap);
		pscan_gap_tlv = (MrvlIEtypes_ScanChanGap_t *)ptlv_pos;
		pscan_gap_tlv->header.type =
			wlan_cpu_to_le16(TLV_TYPE_SCAN_CHANNEL_GAP);
		pscan_gap_tlv->header.len = sizeof(pscan_gap_tlv->gap);
		pscan_gap_tlv->gap =
			wlan_cpu_to_le16((t_u16)pmadapter->scan_chan_gap);
		ptlv_pos += sizeof(pscan_gap_tlv->header) +
			    pscan_gap_tlv->header.len;
	}
	if (pmadapter->ext_scan) {
		pbss_mode = (MrvlIEtypes_BssMode_t *)ptlv_pos;
		pbss_mode->header.type = wlan_cpu_to_le16(TLV_TYPE_BSS_MODE);
		pbss_mode->header.len = sizeof(pbss_mode->bss_mode);
		pbss_mode->bss_mode = pscan_cfg_out->bss_mode;
		ptlv_pos += sizeof(pbss_mode->header) + pbss_mode->header.len;
		pbss_mode->header.len = wlan_cpu_to_le16(pbss_mode->header.len);
		if (pmadapter->ext_scan_enh) {
			if (puser_scan_in) {
				if (puser_scan_in->ext_scan_type ==
				    EXT_SCAN_ENHANCE)
					pmadapter->ext_scan_type =
						EXT_SCAN_ENHANCE;
				else
					pmadapter->ext_scan_type =
						EXT_SCAN_DEFAULT;
			} else if (pmadapter->ext_scan == EXT_SCAN_TYPE_ENH)
				pmadapter->ext_scan_type = EXT_SCAN_ENHANCE;
			else
				pmadapter->ext_scan_type = EXT_SCAN_DEFAULT;
			if (pmadapter->ext_scan_type == EXT_SCAN_ENHANCE)
				*pmax_chan_per_scan =
					MRVDRV_MAX_CHANNELS_PER_SCAN;
		}
	}
	if ((puser_scan_in &&
	     puser_scan_in->chan_list[0].radio_type == BAND_6GHZ) ||
	    (pmadapter->scan_6g))
		num_probes = 1;
	/* If the input config or adapter has the number of Probes set, add tlv
	 */
	if (num_probes) {
		PRINTM(MINFO, "Scan: num_probes = %d\n", num_probes);

		pnum_probes_tlv = (MrvlIEtypes_NumProbes_t *)ptlv_pos;
		pnum_probes_tlv->header.type =
			wlan_cpu_to_le16(TLV_TYPE_NUMPROBES);
		pnum_probes_tlv->header.len =
			sizeof(pnum_probes_tlv->num_probes);
		pnum_probes_tlv->num_probes =
			wlan_cpu_to_le16((t_u16)num_probes);

		ptlv_pos += sizeof(pnum_probes_tlv->header) +
			    pnum_probes_tlv->header.len;

		pnum_probes_tlv->header.len =
			wlan_cpu_to_le16(pnum_probes_tlv->header.len);
	}

	/* Append rates tlv */
	memset(pmadapter, rates, 0, sizeof(rates));

	rates_size = wlan_get_supported_rates(pmpriv, pmpriv->bss_mode,
					      pmpriv->config_bands, rates);

	prates_tlv = (MrvlIEtypes_RatesParamSet_t *)ptlv_pos;
	prates_tlv->header.type = wlan_cpu_to_le16(TLV_TYPE_RATES);
	prates_tlv->header.len = wlan_cpu_to_le16((t_u16)rates_size);
	memcpy_ext(pmadapter, prates_tlv->rates, rates, rates_size, rates_size);
	ptlv_pos += sizeof(prates_tlv->header) + rates_size;

	PRINTM(MINFO, "SCAN_CMD: Rates size = %d\n", rates_size);

	if (wlan_is_ext_capa_support(pmpriv))
		wlan_add_ext_capa_info_ie(pmpriv, MNULL, &ptlv_pos);
	if (pmpriv->adapter->ecsa_enable) {
		t_u8 bandwidth = BW_20MHZ;
		t_u8 oper_class = 1;
		t_u32 usr_dot_11n_dev_cap;
		if (pmpriv->media_connected) {
			if (pmpriv->config_bands & BAND_A)
				usr_dot_11n_dev_cap =
					pmpriv->usr_dot_11n_dev_cap_a;
			else
				usr_dot_11n_dev_cap =
					pmpriv->usr_dot_11n_dev_cap_bg;
			if (usr_dot_11n_dev_cap & MBIT(17)) {
				bandwidth = BW_40MHZ;
				if (ISSUPP_11ACENABLED(
					    pmadapter->fw_cap_info) &&
				    (pmpriv->config_bands & BAND_AAC))
					bandwidth = BW_80MHZ;
			}
			wlan_get_curr_oper_class(
				pmpriv,
				pmpriv->curr_bss_params.bss_descriptor.channel,
				bandwidth, &oper_class);
		}
		wlan_add_supported_oper_class_ie(pmpriv, &ptlv_pos, oper_class);
	}
	wlan_add_wps_probe_request_ie(pmpriv, &ptlv_pos);

	if (puser_scan_in && puser_scan_in->proberesp_only) {
		MrvlIEtypes_OnlyProberesp_t *proberesp_only =
			(MrvlIEtypes_OnlyProberesp_t *)ptlv_pos;
		memset(pmadapter, proberesp_only, 0,
		       sizeof(MrvlIEtypes_OnlyProberesp_t));
		proberesp_only->header.type =
			wlan_cpu_to_le16(TLV_TYPE_ONLYPROBERESP);
		proberesp_only->header.len = wlan_cpu_to_le16(sizeof(t_u8));
		proberesp_only->proberesp_only = puser_scan_in->proberesp_only;
		ptlv_pos += sizeof(MrvlIEtypes_OnlyProberesp_t);
	}

	if (puser_scan_in && memcmp(pmadapter, puser_scan_in->random_mac,
				    zero_mac, MLAN_MAC_ADDR_LENGTH)) {
		MrvlIEtypes_MacAddr_t *randomMacParam =
			(MrvlIEtypes_MacAddr_t *)ptlv_pos;
		memset(pmadapter, randomMacParam, 0,
		       sizeof(MrvlIEtypes_MacAddr_t));
		randomMacParam->header.type =
			wlan_cpu_to_le16(TLV_TYPE_RANDOM_MAC);
		randomMacParam->header.len =
			wlan_cpu_to_le16(MLAN_MAC_ADDR_LENGTH);
		memcpy_ext(pmadapter, randomMacParam->mac,
			   puser_scan_in->random_mac, MLAN_MAC_ADDR_LENGTH,
			   MLAN_MAC_ADDR_LENGTH);
		ptlv_pos += sizeof(MrvlIEtypes_MacAddr_t);
	}
	/*
	 * Set the output for the channel TLV to the address in the tlv buffer
	 *   past any TLVs that were added in this function (SSID, num_probes).
	 *   Channel TLVs will be added past this for each scan command,
	 *   preserving the TLVs that were previously added.
	 */
	*ppchan_list_out = (MrvlIEtypes_ChanListParamSet_t *)ptlv_pos;

	if (puser_scan_in && puser_scan_in->chan_list[0].chan_number) {
		PRINTM(MINFO, "Scan: Using supplied channel list\n");

		for (chan_idx = 0;
		     chan_idx < WLAN_USER_SCAN_CHAN_MAX &&
		     puser_scan_in->chan_list[chan_idx].chan_number;
		     chan_idx++) {
			radio_type =
				puser_scan_in->chan_list[chan_idx].radio_type;
			/*Ignore 5G/2G channels if radio_type do not match
			 * band*/
			if (!wlan_is_band_compatible(
				    pmpriv->config_bands,
				    radio_type_to_band(radio_type)))
				continue;
			(pscan_chan_list + chan_list_idx)->bandcfg.chanBand =
				radio_type;

			channel =
				puser_scan_in->chan_list[chan_idx].chan_number;
			(pscan_chan_list + chan_list_idx)->chan_number =
				channel;

			scan_type =
				puser_scan_in->chan_list[chan_idx].scan_type;
			if (scan_type == MLAN_SCAN_TYPE_UNCHANGED)
				scan_type = pmadapter->scan_type;

			if (radio_type == BAND_5GHZ) {
				if (pmadapter->fw_bands & BAND_A)
					PRINTM(MINFO,
					       "UserScan request for A Band channel %d!!\n",
					       channel);
				else {
					PRINTM(MERROR,
					       "Scan in A band is not allowed!!\n");
					ret = MLAN_STATUS_FAILURE;
					LEAVE();
					return ret;
				}
			}
			if (!puser_scan_in->scan_cfg_only) {
				if (wlan_is_chan_passive(
					    pmpriv,
					    radio_type_to_band(radio_type),
					    channel)) {
					/* do not send probe requests on this
					 * channel */
					scan_type = MLAN_SCAN_TYPE_PASSIVE;
				}
			}
			/* Prevent active scanning on a radar controlled channel
			 */
			if (radio_type == BAND_5GHZ &&
			    scan_type == MLAN_SCAN_TYPE_PASSIVE) {
				if (pmadapter->active_scan_triggered == MFALSE)
					if (wlan_11h_radar_detect_required(
						    pmpriv, channel)) {
						scan_type =
							MLAN_SCAN_TYPE_PASSIVE_TO_ACTIVE;
					}
			}
			if (radio_type == BAND_2GHZ &&
			    !puser_scan_in->scan_cfg_only &&
			    scan_type != MLAN_SCAN_TYPE_PASSIVE) {
				if (pmadapter->active_scan_triggered == MFALSE)
					if (wlan_bg_scan_type_is_passive(
						    pmpriv, channel)) {
						scan_type =
							MLAN_SCAN_TYPE_PASSIVE;
					}
			}
			if (scan_type == MLAN_SCAN_TYPE_PASSIVE ||
			    scan_type == MLAN_SCAN_TYPE_PASSIVE_TO_ACTIVE) {
				(pscan_chan_list + chan_list_idx)
					->chan_scan_mode.passive_scan = MTRUE;
				(pscan_chan_list + chan_list_idx)
					->chan_scan_mode.hidden_ssid_report =
					MTRUE;
			} else {
				(pscan_chan_list + chan_list_idx)
					->chan_scan_mode.passive_scan = MFALSE;
			}

			if (puser_scan_in->chan_list[chan_idx].scan_time) {
				scan_dur = (t_u16)puser_scan_in
						   ->chan_list[chan_idx]
						   .scan_time;
			} else {
				if (scan_type == MLAN_SCAN_TYPE_PASSIVE ||
				    scan_type ==
					    MLAN_SCAN_TYPE_PASSIVE_TO_ACTIVE) {
					scan_dur = pmadapter->passive_scan_time;
				} else if (*pfiltered_scan) {
					scan_dur =
						pmadapter->specific_scan_time;
				} else {
					scan_dur = pmadapter->active_scan_time;
				}
			}

			if (pmadapter->coex_scan &&
			    pmadapter->coex_min_scan_time &&
			    (pmadapter->coex_min_scan_time > scan_dur))
				scan_dur = pmadapter->coex_min_scan_time;
			if (scan_type == MLAN_SCAN_TYPE_PASSIVE_TO_ACTIVE &&
			    pmadapter->passive_to_active_scan ==
				    MLAN_PASS_TO_ACT_SCAN_EN) {
				(pscan_chan_list + chan_list_idx)
					->chan_scan_mode.passive_to_active_scan =
					MTRUE;
				scan_dur = MAX(MIN_PASSIVE_TO_ACTIVE_SCAN_TIME,
					       scan_dur);
			}
			if (puser_scan_in->chan_list[chan_idx].radio_type ==
			    BAND_6GHZ) {
				if (puser_scan_in->chan_list[chan_idx].rnr_flag) {
					(pscan_chan_list + chan_list_idx)
						->chan_scan_mode.rnr_flag =
						MTRUE;
					(pscan_chan_list + chan_list_idx)
						->chan_scan_mode.passive_scan =
						MFALSE;
					scan_dur = pmadapter->wifi_6g_scan_time;
				} else {
					if (puser_scan_in->ssid_filter)
						(pscan_chan_list +
						 chan_list_idx)
							->chan_scan_mode
							.passive_scan = MFALSE;
					else
						(pscan_chan_list +
						 chan_list_idx)
							->chan_scan_mode
							.passive_scan = MTRUE;
					if (puser_scan_in->chan_list[chan_idx]
						    .scan_time == 0)
						scan_dur =
							pmadapter
								->wifi_6g_scan_time;
				}
			}
			PRINTM(MINFO,
			       "chan=%d, mode=%d, passive_to_active=%d\n",
			       (pscan_chan_list + chan_list_idx)->chan_number,
			       (pscan_chan_list + chan_list_idx)
				       ->chan_scan_mode.passive_scan,
			       (pscan_chan_list + chan_list_idx)
				       ->chan_scan_mode.passive_to_active_scan);

			(pscan_chan_list + chan_list_idx)->min_scan_time =
				wlan_cpu_to_le16(scan_dur);
			(pscan_chan_list + chan_list_idx)->max_scan_time =
				wlan_cpu_to_le16(scan_dur);
			chan_list_idx++;
		}

		/* Check if we are only scanning the current channel */
		if ((chan_idx == 1) &&
		    (puser_scan_in->chan_list[0].chan_number ==
		     pmpriv->curr_bss_params.bss_descriptor.channel)) {
			*pscan_current_only = MTRUE;
			PRINTM(MINFO, "Scan: Scanning current channel only\n");
		}
		if (puser_scan_in) {
			pscan_cfg_out->num_6g_scan_params =
				puser_scan_in->num_6g_scan_params;
			memcpy_ext(pmadapter, pscan_cfg_out->scan_param_list,
				   puser_scan_in->scan_param_list,
				   sizeof(wlan_6g_scan_params) *
					   WLAN_MAX_6G_SCAN_PARAMS_LIST,
				   sizeof(wlan_6g_scan_params) *
					   WLAN_MAX_6G_SCAN_PARAMS_LIST);
		}
	} else {
		num_of_channel =
			wlan_scan_create_channel_list(pmpriv, puser_scan_in,
						      pscan_chan_list,
						      *pfiltered_scan);
		PRINTM(MCMND, "Scan: Creating full region channel list %d\n",
		       num_of_channel);

		pscan_cfg_out->num_6g_scan_params =
			wlan_scan_create_6g_scan_params(pmadapter,
							pscan_cfg_out);
	}

	LEAVE();
	return ret;
}

/**
 *  @brief Inspect the scan response buffer for pointers to expected TLVs
 *
 *  TLVs can be included at the end of the scan response BSS information.
 *    Parse the data in the buffer for pointers to TLVs that can potentially
 *    be passed back in the response
 *
 *  @param pmadapter        Pointer to the mlan_adapter structure
 *  @param ptlv             Pointer to the start of the TLV buffer to parse
 *  @param tlv_buf_size     Size of the TLV buffer
 *  @param req_tlv_type     Request TLV's type
 *  @param pptlv            Output parameter: Pointer to the request TLV if
 * found
 *
 *  @return                 N/A
 */
static t_void wlan_ret_802_11_scan_get_tlv_ptrs(pmlan_adapter pmadapter,
						MrvlIEtypes_Data_t *ptlv,
						t_u32 tlv_buf_size,
						t_u32 req_tlv_type,
						MrvlIEtypes_Data_t **pptlv)
{
	MrvlIEtypes_Data_t *pcurrent_tlv;
	t_u32 tlv_buf_left;
	t_u32 tlv_type;
	t_u32 tlv_len;

	ENTER();

	pcurrent_tlv = ptlv;
	tlv_buf_left = tlv_buf_size;
	*pptlv = MNULL;

	PRINTM(MINFO, "SCAN_RESP: tlv_buf_size = %d\n", tlv_buf_size);

	while (tlv_buf_left >= sizeof(MrvlIEtypesHeader_t)) {
		tlv_type = wlan_le16_to_cpu(pcurrent_tlv->header.type);
		tlv_len = wlan_le16_to_cpu(pcurrent_tlv->header.len);

		if (sizeof(ptlv->header) + tlv_len > tlv_buf_left) {
			PRINTM(MERROR, "SCAN_RESP: TLV buffer corrupt\n");
			break;
		}

		if (req_tlv_type == tlv_type) {
			switch (tlv_type) {
			case TLV_TYPE_TSFTIMESTAMP:
				PRINTM(MINFO,
				       "SCAN_RESP: TSF Timestamp TLV, len = %d\n",
				       tlv_len);
				*pptlv = (MrvlIEtypes_Data_t *)pcurrent_tlv;
				break;
			case TLV_TYPE_CHANNELBANDLIST:
				PRINTM(MINFO,
				       "SCAN_RESP: CHANNEL BAND LIST TLV, len = %d\n",
				       tlv_len);
				*pptlv = (MrvlIEtypes_Data_t *)pcurrent_tlv;
				break;
			case TLV_TYPE_CHANNEL_STATS:
				PRINTM(MINFO,
				       "SCAN_RESP: CHANNEL STATS TLV, len = %d\n",
				       tlv_len);
				*pptlv = (MrvlIEtypes_Data_t *)pcurrent_tlv;
				break;
			default:
				PRINTM(MERROR,
				       "SCAN_RESP: Unhandled TLV = %d\n",
				       tlv_type);
				/* Give up, this seems corrupted */
				LEAVE();
				return;
			}
		}

		if (*pptlv) {
			/* HEXDUMP("SCAN_RESP: TLV Buf", (t_u8 *)*pptlv+4,
			 * tlv_len); */
			break;
		}

		tlv_buf_left -= (sizeof(ptlv->header) + tlv_len);
		pcurrent_tlv =
			(MrvlIEtypes_Data_t *)(pcurrent_tlv->data + tlv_len);

	} /* while */

	LEAVE();
}

/**
 *  @brief This function parse the bss parameters from RNR IE
 *
 *  @param bss_params     The value of BSS parameters
 *  @param pcoloc_ap      A pointer to information of new colocated ap
 *
 *  @return               1 for colocated ap or 0 for non-colocated ap
 */
static t_u8 wlan_parse_rnr_bss_params(t_u8 bss_params,
				      RnrColocatedAp_t *pcoloc_ap_info)
{
	ENTER();

	pcoloc_ap_info->bss_params.oct_recommended =
		GET_RNR_TBTT_INFO_BSSPARAMS_BIT(bss_params, 0);
	pcoloc_ap_info->bss_params.same_ssid =
		GET_RNR_TBTT_INFO_BSSPARAMS_BIT(bss_params, 1);
	pcoloc_ap_info->bss_params.multi_bss =
		GET_RNR_TBTT_INFO_BSSPARAMS_BIT(bss_params, 2);
	pcoloc_ap_info->bss_params.transmitted_bssid =
		GET_RNR_TBTT_INFO_BSSPARAMS_BIT(bss_params, 3);
	pcoloc_ap_info->bss_params.colocated_ess =
		GET_RNR_TBTT_INFO_BSSPARAMS_BIT(bss_params, 4);
	pcoloc_ap_info->bss_params.unsolicited_probe =
		GET_RNR_TBTT_INFO_BSSPARAMS_BIT(bss_params, 5);
	pcoloc_ap_info->bss_params.colocated =
		GET_RNR_TBTT_INFO_BSSPARAMS_BIT(bss_params, 6);

	LEAVE();
	return pcoloc_ap_info->bss_params.colocated;
}

/**
 *  @brief This function parse the TBTT Information from RNR IE
 *
 *  @param pmadapter        A pointer to mlan_adapter structure
 *  @param ptbtt_info       A pointer to current tbtt information
 *  @param tbtt_info_len    A length of tbtt information
 *  @param short_ssid       A shord ssid of reporting AP
 *  @param pssid            A pointer to ssid of the reporting AP
 *  @param pcoloc_ap_info   A pointer to information of new colocated ap
 *
 *  @return                 MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status wlan_parse_rnr_tbtt_info(mlan_adapter *pmadapter,
					    t_u8 *ptbtt_info,
					    t_u8 tbtt_info_len,
					    t_u32 short_ssid,
					    mlan_802_11_ssid *pssid,
					    RnrColocatedAp_t *pcoloc_ap_info)
{
	const t_u32 short_ssid_size = sizeof(pcoloc_ap_info->short_ssid);

	ENTER();

	if (!ptbtt_info) {
		PRINTM(MERROR, "TBTT Information for RNR element is NULL.\n");
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}

	/* skip the TBTT offset, pointer to BSSID */
	ptbtt_info += 1;

	memcpy_ext(pmadapter, pcoloc_ap_info->bssid, ptbtt_info,
		   MLAN_MAC_ADDR_LENGTH, MLAN_MAC_ADDR_LENGTH);

	ptbtt_info += MLAN_MAC_ADDR_LENGTH;

	/* pointer to short SSID, or next to bss parameters */
	if (tbtt_info_len >=
	    IEEE80211_RNR_TBTT_INFO_OFFSET_BSSID_SSSID_BSSPARAMS) {
		memcpy_ext(pmadapter, &pcoloc_ap_info->short_ssid, ptbtt_info,
			   short_ssid_size, short_ssid_size);
		// coloc_ap_entry->short_ssid_valid = true;
		ptbtt_info += short_ssid_size;
	}

	/* skip non-colocated APs */
	if (!wlan_parse_rnr_bss_params(*ptbtt_info, pcoloc_ap_info)) {
		PRINTM(MERROR, "non-colocated ap will be skipped!\n");
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}

	if (tbtt_info_len == IEEE80211_RNR_TBTT_INFO_OFFSET_BSSID_BSSPARAMS) {
		/*
		 * no information about the short SSID. dropped
		 */
		if (!pcoloc_ap_info->bss_params.same_ssid) {
			PRINTM(MERROR,
			       "no information for short SSID, dropped!\n");
			LEAVE();
			return MLAN_STATUS_FAILURE;
		}

		pcoloc_ap_info->short_ssid = short_ssid;
		// coloc_ap_entry->short_ssid_valid = true;

		if (pssid && pssid->ssid_len) {
			pcoloc_ap_info->ssid.ssid_len = pssid->ssid_len;
			memcpy_ext(pmadapter, pcoloc_ap_info->ssid.ssid,
				   pssid->ssid, pssid->ssid_len,
				   MLAN_MAX_SSID_LENGTH);
		}
	}

	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function add colocated ap by parsing the RNR IE from pbss_entry
 *
 *  @param pmadapter   A pointer to mlan_adapter structure
 *  @param pbss_entry  A pointer to BSSDescriptor_t which has RNR IE
 *
 *  @return            N/A
 */
static t_void wlan_parse_rnr_colocated_ap(mlan_adapter *pmadapter,
					  BSSDescriptor_t *pbss_entry)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_callbacks *pcb = &pmadapter->callbacks;
	mlan_ds_scan *pscan = MNULL;
	wlan_user_scan_cfg *puser_scan_in = MNULL;
	const t_u32 nap_info_size = sizeof(IEEEtypes_RnrNeighborApInfo_t);
	wlan_6e_coloc_ap_t *pcoloc_ap = MNULL;
	IEEEtypes_Generic_t *prnr_ie = MNULL;
	IEEEtypes_RnrNeighborApInfo_t *pnap_info = MNULL;
	t_u32 short_ssid = 0;
	t_u8 *pcurrent_ptr = MNULL;
	t_u8 *pend = MNULL;
	t_u8 tbtt_info_cnt = 0;
	t_u8 tbtt_info_len = 0;
	t_u8 i = 0;

	ENTER();

	if (pmadapter->scan_6g) {
		PRINTM(MINFO,
		       "RNR IE should not be transmitted in 6G band channel\n");
		LEAVE();
		return;
	}

	/* parase rnr only for scan request is not from cfg80211 */
	if (pmadapter->pscan_ioctl_req) {
		pscan = (mlan_ds_scan *)pmadapter->pscan_ioctl_req->pbuf;
		if (pscan) {
			if (pscan->sub_command == MLAN_OID_SCAN_USER_CONFIG)
				puser_scan_in =
					(wlan_user_scan_cfg *)pscan->param
						.user_scan.scan_cfg_buf;
		}
	}

	if (puser_scan_in && puser_scan_in->scan_cfg_only) {
		PRINTM(MINFO, "Parse RNR element not for cfg command\n");
		LEAVE();
		return;
	}

	if (!pbss_entry) {
		PRINTM(MERROR, "No bss_new_entry for parsing RNR element\n");
		LEAVE();
		return;
	}

	prnr_ie = pbss_entry->prnr_ie;
	if (!prnr_ie || prnr_ie->ieee_hdr.element_id != RNR) {
		PRINTM(MERROR, "RNR element is not in bss_new_entry\n");
		LEAVE();
		return;
	}

	pcurrent_ptr = prnr_ie->data;
	pend = pcurrent_ptr + prnr_ie->ieee_hdr.len;

	/* Calculate 32bit short SSID over reporting SSID */
	ret = pcb->moal_calc_short_ssid(pbss_entry->ssid.ssid,
					pbss_entry->ssid.ssid_len, &short_ssid);
	if ((ret == MLAN_STATUS_SUCCESS) && short_ssid) {
		PRINTM(MINFO,
		       "Short-SSID (0x%0x) calculated over 6E reporting AP's SSID \"%s\"\n",
		       short_ssid, pbss_entry->ssid.ssid);
	} else {
		PRINTM(MINFO,
		       "Short SSID calculated over 6E reporting AP's SSID failed\n");
		LEAVE();
		return;
	}

	/* RNR IE may contain more than one Neighbor AP Information */
	while (pcurrent_ptr + nap_info_size <= pend) {
		pnap_info = (IEEEtypes_RnrNeighborApInfo_t *)pcurrent_ptr;
		tbtt_info_cnt =
			GET_RNR_TBTT_INFO_HDR_COUNT(pnap_info->tbtt_info_hdr) +
			1;
		tbtt_info_len = pnap_info->tbtt_info_len;

		pcurrent_ptr += nap_info_size;

		if (pend - pcurrent_ptr < tbtt_info_cnt * tbtt_info_len)
			break;

		/*
		 * TBTT Information must include
		 * offset + BSSID + BSS parameters +
		 * (short SSID or same_ssid bit to be set).
		 * ignore other options, and move to the
		 * next AP info
		 */
		if (/*band != NL80211_BAND_6GHZ ||*/
		    (tbtt_info_len !=
			     IEEE80211_RNR_TBTT_INFO_OFFSET_BSSID_BSSPARAMS &&
		     tbtt_info_len <
			     IEEE80211_RNR_TBTT_INFO_OFFSET_BSSID_SSSID_BSSPARAMS)) {
			pcurrent_ptr += tbtt_info_cnt * tbtt_info_len;
			continue;
		}

		for (i = 0; i < tbtt_info_cnt; i++) {
			ret = pcb->moal_malloc(pmadapter->pmoal_handle,
					       sizeof(wlan_6e_coloc_ap_t),
					       MLAN_MEM_DEF |
						       MLAN_MEM_FLAG_ATOMIC,
					       (t_u8 **)&pcoloc_ap);

			if (ret != MLAN_STATUS_SUCCESS || !pcoloc_ap) {
				PRINTM(MERROR,
				       "Memory allocation for coloc_ap failed!\n");
				LEAVE();
				return;
			}

			memset(pmadapter, (t_u8 *)pcoloc_ap, 0,
			       sizeof(wlan_6e_coloc_ap_t));

			ret = wlan_parse_rnr_tbtt_info(pmadapter, pcurrent_ptr,
						       tbtt_info_len,
						       short_ssid,
						       &pbss_entry->ssid,
						       &pcoloc_ap->ap_info);

			if (ret == MLAN_STATUS_SUCCESS) {
				pcoloc_ap->ap_info.oper_class =
					pnap_info->oper_class;
				pcoloc_ap->ap_info.chan_number =
					pnap_info->chan_number;
				/* add new coloc_ap node to coloc_ap_list */
				/*pcoloc_ap->ap_info is allocated inside the
				 * function itself*/
				// coverity[cert_exp33_c_violation:SUPPRESS]
				if (wlan_add_rnr_coloc_ap(pmadapter, pcoloc_ap))
					pcb->moal_mfree(pmadapter->pmoal_handle,
							(t_u8 *)pcoloc_ap);
			} else
				pcb->moal_mfree(pmadapter->pmoal_handle,
						(t_u8 *)pcoloc_ap);

			pcurrent_ptr += tbtt_info_len;
		}
	}

	LEAVE();
}

/**
 *  @brief Interpret a BSS scan response returned from the firmware
 *
 *  Parse the various fixed fields and IEs passed back for a BSS probe
 *   response or beacon from the scan command.  Record information as needed
 *   in the scan table BSSDescriptor_t for that entry.
 *
 *  @param pmadapter    A pointer to mlan_adapter structure
 *  @param pbss_entry   Output parameter: Pointer to the BSS Entry
 *  @param pbeacon_info Pointer to the Beacon information
 *  @param bytes_left   Number of bytes left to parse
 *  @param ext_scan     extended scan
 *
 *  @return             MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status wlan_interpret_bss_desc_with_ie(pmlan_adapter pmadapter,
						   BSSDescriptor_t *pbss_entry,
						   t_u8 **pbeacon_info,
						   t_u32 *bytes_left,
						   t_u8 ext_scan)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	IEEEtypes_ElementId_e element_id;
	IEEEtypes_FhParamSet_t *pfh_param_set;
	IEEEtypes_DsParamSet_t *pds_param_set;
	IEEEtypes_CfParamSet_t *pcf_param_set;
	IEEEtypes_CapInfo_t *pcap_info;
	WLAN_802_11_FIXED_IEs fixed_ie;
	t_u8 *pcurrent_ptr;
	t_s64 offset = 0;
	t_u8 *prate;
	t_u8 element_len;
	t_u16 total_ie_len;
	t_u8 bytes_to_copy;
	t_u8 rate_size;
	t_u16 beacon_size;
	t_u8 found_data_rate_ie;
	t_u32 bytes_left_for_current_beacon;
	IEEEtypes_ERPInfo_t *perp_info;

	IEEEtypes_VendorSpecific_t *pvendor_ie;
	const t_u8 wpa_oui[4] = {0x00, 0x50, 0xf2, 0x01};
	const t_u8 wmm_oui[4] = {0x00, 0x50, 0xf2, 0x02};
	const t_u8 owe_oui[4] = {0x50, 0x6f, 0x9a, 0x1c};
	const t_u8 osen_oui[] = {0x50, 0x6f, 0x9a, 0x12};

	IEEEtypes_CountryInfoSet_t *pcountry_info;
	IEEEtypes_Extension_t *pext_tlv;

	ENTER();

	found_data_rate_ie = MFALSE;
	rate_size = 0;
	beacon_size = 0;

	if (*bytes_left >= sizeof(beacon_size)) {
		/* Extract & convert beacon size from the command buffer */
		memcpy_ext(pmadapter, &beacon_size, *pbeacon_info,
			   sizeof(beacon_size), sizeof(beacon_size));
		beacon_size = wlan_le16_to_cpu(beacon_size);
		*bytes_left -= sizeof(beacon_size);
		*pbeacon_info += sizeof(beacon_size);
	}

	if (!beacon_size || beacon_size > *bytes_left) {
		*pbeacon_info += *bytes_left;
		*bytes_left = 0;

		LEAVE();
		return MLAN_STATUS_FAILURE;
	}

	/* Initialize the current working beacon pointer for this BSS iteration
	 */
	pcurrent_ptr = *pbeacon_info;

	/* Advance the return beacon pointer past the current beacon */
	*pbeacon_info += beacon_size;
	*bytes_left -= beacon_size;

	bytes_left_for_current_beacon = beacon_size;

	if (bytes_left_for_current_beacon <
	    (MLAN_MAC_ADDR_LENGTH + sizeof(t_u8) +
	     sizeof(WLAN_802_11_FIXED_IEs))) {
		PRINTM(MERROR, "InterpretIE: Not enough bytes left\n");
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}

	memcpy_ext(pmadapter, pbss_entry->mac_address, pcurrent_ptr,
		   MLAN_MAC_ADDR_LENGTH, MLAN_MAC_ADDR_LENGTH);
	PRINTM(MINFO, "InterpretIE: AP MAC Addr-" MACSTR "\n",
	       MAC2STR(pbss_entry->mac_address));

	pcurrent_ptr += MLAN_MAC_ADDR_LENGTH;
	bytes_left_for_current_beacon -= MLAN_MAC_ADDR_LENGTH;

	/*
	 * Next 4 fields are RSSI (for legacy scan only), time stamp,
	 *   beacon interval, and capability information
	 */
	if (!ext_scan) {
		/* RSSI is 1 byte long */
		pbss_entry->rssi = (t_s32)(*pcurrent_ptr);
		PRINTM(MINFO, "InterpretIE: RSSI=%02X\n", *pcurrent_ptr);
		pcurrent_ptr += 1;
		bytes_left_for_current_beacon -= 1;
	}

	/*
	 *  The RSSI is not part of the beacon/probe response.  After we have
	 *    advanced pcurrent_ptr past the RSSI field, save the remaining
	 *    data for use at the application layer
	 */
	pbss_entry->pbeacon_buf = pcurrent_ptr;
	pbss_entry->beacon_buf_size = bytes_left_for_current_beacon;

	/* Time stamp is 8 bytes long */
	memcpy_ext(pmadapter, fixed_ie.time_stamp, pcurrent_ptr, 8,
		   sizeof(fixed_ie.time_stamp));
	memcpy_ext(pmadapter, pbss_entry->time_stamp, pcurrent_ptr, 8,
		   sizeof(pbss_entry->time_stamp));
	pcurrent_ptr += 8;
	bytes_left_for_current_beacon -= 8;

	/* Beacon interval is 2 bytes long */
	memcpy_ext(pmadapter, &fixed_ie.beacon_interval, pcurrent_ptr, 2,
		   sizeof(fixed_ie.beacon_interval));
	pbss_entry->beacon_period = wlan_le16_to_cpu(fixed_ie.beacon_interval);
	pcurrent_ptr += 2;
	bytes_left_for_current_beacon -= 2;

	/* Capability information is 2 bytes long */
	memcpy_ext(pmadapter, &fixed_ie.capabilities, pcurrent_ptr, 2,
		   sizeof(fixed_ie.capabilities));
	PRINTM(MINFO, "InterpretIE: fixed_ie.capabilities=0x%X\n",
	       fixed_ie.capabilities);
	fixed_ie.capabilities = wlan_le16_to_cpu(fixed_ie.capabilities);
	pcap_info = (IEEEtypes_CapInfo_t *)&fixed_ie.capabilities;
	memcpy_ext(pmadapter, &pbss_entry->cap_info, pcap_info,
		   sizeof(IEEEtypes_CapInfo_t), sizeof(IEEEtypes_CapInfo_t));
	pcurrent_ptr += 2;
	bytes_left_for_current_beacon -= 2;

	/* Rest of the current buffer are IE's */
	PRINTM(MINFO, "InterpretIE: IELength for this AP = %d\n",
	       bytes_left_for_current_beacon);

	HEXDUMP("InterpretIE: IE info", (t_u8 *)pcurrent_ptr,
		bytes_left_for_current_beacon);

	if (pcap_info->privacy) {
		PRINTM(MINFO, "InterpretIE: AP WEP enabled\n");
		pbss_entry->privacy = Wlan802_11PrivFilter8021xWEP;
	} else {
		pbss_entry->privacy = Wlan802_11PrivFilterAcceptAll;
	}

	pbss_entry->bss_mode = MLAN_BSS_MODE_INFRA;

	if (pcap_info->spectrum_mgmt == 1) {
		PRINTM(MINFO, "InterpretIE: 11h- Spectrum Management "
			      "capability bit found\n");
		pbss_entry->wlan_11h_bss_info.sensed_11h = 1;
	}

	/* Process variable IE */
	while (bytes_left_for_current_beacon >= 2) {
		element_id = (IEEEtypes_ElementId_e)(*((t_u8 *)pcurrent_ptr));
		element_len = *((t_u8 *)pcurrent_ptr + 1);
		total_ie_len = element_len + sizeof(IEEEtypes_Header_t);

		if (bytes_left_for_current_beacon < total_ie_len) {
			PRINTM(MERROR, "InterpretIE: Error in processing IE, "
				       "bytes left < IE length\n");
			bytes_left_for_current_beacon = 0;
			ret = MLAN_STATUS_FAILURE;
			continue;
		}

		switch (element_id) {
		case SSID:
			if (element_len > MRVDRV_MAX_SSID_LENGTH) {
				bytes_left_for_current_beacon = 0;
				ret = MLAN_STATUS_FAILURE;
				continue;
			}
			if (!pbss_entry->ssid.ssid_len) {
				pbss_entry->ssid.ssid_len = element_len;
				memcpy_ext(pmadapter, pbss_entry->ssid.ssid,
					   (pcurrent_ptr + 2), element_len,
					   sizeof(pbss_entry->ssid.ssid));
			}
			PRINTM(MINFO, "InterpretIE: ssid: %-32s\n",
			       pbss_entry->ssid.ssid);
			break;

		case SUPPORTED_RATES:
			if (element_len > WLAN_SUPPORTED_RATES) {
				bytes_left_for_current_beacon = 0;
				ret = MLAN_STATUS_FAILURE;
				continue;
			}
			memcpy_ext(pmadapter, pbss_entry->data_rates,
				   pcurrent_ptr + 2, element_len,
				   sizeof(pbss_entry->data_rates));
			memcpy_ext(pmadapter, pbss_entry->supported_rates,
				   pcurrent_ptr + 2, element_len,
				   sizeof(pbss_entry->supported_rates));
			HEXDUMP("InterpretIE: SupportedRates:",
				pbss_entry->supported_rates, element_len);
			rate_size = element_len;
			found_data_rate_ie = MTRUE;
			break;

		case FH_PARAM_SET:
			pfh_param_set = (IEEEtypes_FhParamSet_t *)pcurrent_ptr;
			pbss_entry->network_type_use = Wlan802_11FH;
			memcpy_ext(pmadapter,
				   &pbss_entry->phy_param_set.fh_param_set,
				   pfh_param_set, total_ie_len,
				   sizeof(IEEEtypes_FhParamSet_t));
			pbss_entry->phy_param_set.fh_param_set.len = MIN(
				element_len, (sizeof(IEEEtypes_FhParamSet_t) -
					      sizeof(IEEEtypes_Header_t)));
			pbss_entry->phy_param_set.fh_param_set.dwell_time =
				wlan_le16_to_cpu(
					pbss_entry->phy_param_set.fh_param_set
						.dwell_time);
			break;

		case DS_PARAM_SET:
			pds_param_set = (IEEEtypes_DsParamSet_t *)pcurrent_ptr;

			pbss_entry->network_type_use = Wlan802_11DS;
			pbss_entry->channel = pds_param_set->current_chan;

			memcpy_ext(pmadapter,
				   &pbss_entry->phy_param_set.ds_param_set,
				   pds_param_set, total_ie_len,
				   sizeof(IEEEtypes_DsParamSet_t));
			pbss_entry->phy_param_set.ds_param_set.len = MIN(
				element_len, (sizeof(IEEEtypes_DsParamSet_t) -
					      sizeof(IEEEtypes_Header_t)));
			break;

		case CF_PARAM_SET:
			pcf_param_set = (IEEEtypes_CfParamSet_t *)pcurrent_ptr;
			memcpy_ext(pmadapter,
				   &pbss_entry->ss_param_set.cf_param_set,
				   pcf_param_set, total_ie_len,
				   sizeof(IEEEtypes_CfParamSet_t));
			pbss_entry->ss_param_set.cf_param_set.len = MIN(
				element_len, (sizeof(IEEEtypes_CfParamSet_t) -
					      sizeof(IEEEtypes_Header_t)));
			break;

		/* Handle Country Info IE */
		case COUNTRY_INFO:
			pcountry_info =
				(IEEEtypes_CountryInfoSet_t *)pcurrent_ptr;

			if (pcountry_info->len <
				    sizeof(pcountry_info->country_code) ||
			    (pcountry_info->len + 2) >
				    sizeof(IEEEtypes_CountryInfoFullSet_t)) {
				PRINTM(MERROR,
				       "InterpretIE: 11D- Err "
				       "country_info len =%d min=%d max=%d\n",
				       pcountry_info->len,
				       sizeof(pcountry_info->country_code),
				       sizeof(IEEEtypes_CountryInfoFullSet_t));
				LEAVE();
				return MLAN_STATUS_FAILURE;
			}

			memcpy_ext(pmadapter, &pbss_entry->country_info,
				   pcountry_info, pcountry_info->len + 2,
				   sizeof(pbss_entry->country_info));
			HEXDUMP("InterpretIE: 11D- country_info:",
				(t_u8 *)pcountry_info,
				(t_u32)(pcountry_info->len + 2));
			break;

		case ERP_INFO:
			perp_info = (IEEEtypes_ERPInfo_t *)pcurrent_ptr;
			pbss_entry->erp_flags = perp_info->erp_flags;
			break;

		case POWER_CONSTRAINT:
		case POWER_CAPABILITY:
		case TPC_REPORT:
		case CHANNEL_SWITCH_ANN:
		case QUIET:
		case SUPPORTED_CHANNELS:
		case TPC_REQUEST:
			wlan_11h_process_bss_elem(
				pmadapter, &pbss_entry->wlan_11h_bss_info,
				pcurrent_ptr);
			break;
		case EXTENDED_SUPPORTED_RATES:
			/*
			 * Only process extended supported rate
			 * if data rate is already found.
			 * Data rate IE should come before
			 * extended supported rate IE
			 */
			if (found_data_rate_ie) {
				if ((element_len + rate_size) >
				    WLAN_SUPPORTED_RATES) {
					bytes_to_copy = (WLAN_SUPPORTED_RATES -
							 rate_size);
				} else {
					bytes_to_copy = element_len;
				}

				prate = (t_u8 *)pbss_entry->data_rates;
				prate += rate_size;
				memcpy_ext(pmadapter, prate, pcurrent_ptr + 2,
					   bytes_to_copy, bytes_to_copy);

				prate = (t_u8 *)pbss_entry->supported_rates;
				prate += rate_size;
				memcpy_ext(pmadapter, prate, pcurrent_ptr + 2,
					   bytes_to_copy, bytes_to_copy);
			}
			HEXDUMP("InterpretIE: ExtSupportedRates:",
				pbss_entry->supported_rates,
				element_len + rate_size);
			break;

		case VENDOR_SPECIFIC_221:
			pvendor_ie = (IEEEtypes_VendorSpecific_t *)pcurrent_ptr;

			if (!memcmp(pmadapter, pvendor_ie->vend_hdr.oui,
				    wpa_oui, sizeof(wpa_oui))) {
				pbss_entry->pwpa_ie =
					(IEEEtypes_VendorSpecific_t *)
						pcurrent_ptr;
				offset = pcurrent_ptr - pbss_entry->pbeacon_buf;
				pbss_entry->wpa_offset = (t_u16)(offset);
				HEXDUMP("InterpretIE: Resp WPA_IE",
					(t_u8 *)pbss_entry->pwpa_ie,
					((*(pbss_entry->pwpa_ie)).vend_hdr.len +
					 sizeof(IEEEtypes_Header_t)));
			} else if (!memcmp(pmadapter, pvendor_ie->vend_hdr.oui,
					   wmm_oui, sizeof(wmm_oui))) {
				if (total_ie_len ==
					    sizeof(IEEEtypes_WmmParameter_t) ||
				    total_ie_len ==
					    sizeof(IEEEtypes_WmmInfo_t)) {
					/*
					 * Only accept and copy the WMM IE if
					 * it matches the size expected for the
					 * WMM Info IE or the WMM Parameter IE.
					 */
					memcpy_ext(pmadapter,
						   (t_u8 *)&pbss_entry->wmm_ie,
						   pcurrent_ptr, total_ie_len,
						   sizeof(pbss_entry->wmm_ie));
					HEXDUMP("InterpretIE: Resp WMM_IE",
						(t_u8 *)&pbss_entry->wmm_ie,
						total_ie_len);
				}
			} else if (IS_FW_SUPPORT_EMBEDDED_OWE(pmadapter) &&
				   !memcmp(pmadapter, pvendor_ie->vend_hdr.oui,
					   owe_oui, sizeof(owe_oui))) {
				/* Current Format of OWE IE is
				 * element_id:element_len:oui:MAC Address:SSID
				 * length:SSID */
				t_u8 trans_ssid_len = *(
					pcurrent_ptr +
					sizeof(IEEEtypes_Header_t) +
					sizeof(owe_oui) + MLAN_MAC_ADDR_LENGTH);

				if (!trans_ssid_len ||
				    trans_ssid_len > MRVDRV_MAX_SSID_LENGTH) {
					bytes_left_for_current_beacon = 0;
					continue;
				}
				if (!pcap_info->privacy)
					pbss_entry->owe_transition_mode =
						OWE_TRANS_MODE_OPEN;
				else
					pbss_entry->owe_transition_mode =
						OWE_TRANS_MODE_OWE;

				memcpy_ext(
					pmadapter,
					pbss_entry->trans_mac_address,
					(pcurrent_ptr +
					 sizeof(IEEEtypes_Header_t) +
					 sizeof(owe_oui)),
					MLAN_MAC_ADDR_LENGTH,
					sizeof(pbss_entry->trans_mac_address));
				pbss_entry->trans_ssid.ssid_len =
					trans_ssid_len;
				memcpy_ext(
					pmadapter, pbss_entry->trans_ssid.ssid,
					(pcurrent_ptr +
					 sizeof(IEEEtypes_Header_t) +
					 sizeof(owe_oui) +
					 MLAN_MAC_ADDR_LENGTH + sizeof(t_u8)),
					trans_ssid_len,
					sizeof(pbss_entry->trans_ssid.ssid));

				PRINTM(MCMND,
				       "InterpretIE: OWE Transition AP privacy=%d MAC Addr-" MACSTR
				       " ssid %s\n",
				       pbss_entry->owe_transition_mode,
				       MAC2STR(pbss_entry->trans_mac_address),
				       pbss_entry->trans_ssid.ssid);
			} else if (!memcmp(pmadapter, pvendor_ie->vend_hdr.oui,
					   osen_oui, sizeof(osen_oui))) {
				pbss_entry->posen_ie =
					(IEEEtypes_Generic_t *)pcurrent_ptr;
				offset = pcurrent_ptr - pbss_entry->pbeacon_buf;
				pbss_entry->osen_offset = (t_u16)(offset);
				HEXDUMP("InterpretIE: Resp OSEN_IE",
					(t_u8 *)pbss_entry->posen_ie,
					(*(pbss_entry->posen_ie)).ieee_hdr.len +
						sizeof(IEEEtypes_Header_t));
			} else {
				if (pbss_entry->vendor_oui_count <
				    MAX_VENDOR_OUI_NUM) {
					/* add oui in list,
					 * pbss_entry->vendor_oui_count does not
					 * exceed MAX_VENDOR_OUI_NUM hence
					 * pbss_entry->vendor_oui buffer size
					 * does not exceed (MAX_VENDOR_OUI_NUM *
					 * VENDOR_OUI_LEN)
					 */
					// coverity[overrun-buffer-arg:
					// SUPPRESS]
					//  coverity[cert_arr30_c_violation:
					//  SUPPRESS]
					//  coverity[cert_str31_c_violation:SUPPRESS]
					memcpy_ext(
						pmadapter,
						(t_u8 *)&pbss_entry->vendor_oui
							[pbss_entry->vendor_oui_count *
							 VENDOR_OUI_LEN],
						pvendor_ie->vend_hdr.oui,
						VENDOR_OUI_LEN, VENDOR_OUI_LEN);
					pbss_entry->vendor_oui_count++;
				}
			}
			break;
		case RSN_IE:
			pbss_entry->prsn_ie =
				(IEEEtypes_Generic_t *)pcurrent_ptr;
			offset = pcurrent_ptr - pbss_entry->pbeacon_buf;
			pbss_entry->rsn_offset = (t_u16)(offset);
			HEXDUMP("InterpretIE: Resp RSN_IE",
				(t_u8 *)pbss_entry->prsn_ie,
				(*(pbss_entry->prsn_ie)).ieee_hdr.len +
					sizeof(IEEEtypes_Header_t));
			break;
		case RSNX_IE:
			pbss_entry->prsnx_ie =
				(IEEEtypes_Generic_t *)pcurrent_ptr;
			offset = pcurrent_ptr - pbss_entry->pbeacon_buf;
			pbss_entry->rsnx_offset = (t_u16)(offset);
			HEXDUMP("InterpretIE: Resp RSNX_IE",
				(t_u8 *)pbss_entry->prsnx_ie,
				(*(pbss_entry->prsnx_ie)).ieee_hdr.len +
					sizeof(IEEEtypes_Header_t));
			break;
		case WAPI_IE:
			pbss_entry->pwapi_ie =
				(IEEEtypes_Generic_t *)pcurrent_ptr;
			offset = pcurrent_ptr - pbss_entry->pbeacon_buf;
			pbss_entry->wapi_offset = (t_u16)(offset);
			HEXDUMP("InterpretIE: Resp WAPI_IE",
				(t_u8 *)pbss_entry->pwapi_ie,
				(*(pbss_entry->pwapi_ie)).ieee_hdr.len +
					sizeof(IEEEtypes_Header_t));
			break;
		case MULTI_BSSID:
			if (IS_FW_SUPPORT_MULTIBSSID(pmadapter)) {
				pbss_entry->multi_bssid_ap = MULTI_BSSID_AP;
				HEXDUMP("InterpretIE: Multi BSSID IE",
					(t_u8 *)pcurrent_ptr, total_ie_len);
			}
			break;
		case HT_CAPABILITY:
			pbss_entry->pht_cap = (IEEEtypes_HTCap_t *)pcurrent_ptr;
			offset = pcurrent_ptr - pbss_entry->pbeacon_buf;
			pbss_entry->ht_cap_offset = (t_u16)(offset);
			HEXDUMP("InterpretIE: Resp HTCAP_IE",
				(t_u8 *)pbss_entry->pht_cap,
				(*(pbss_entry->pht_cap)).ieee_hdr.len +
					sizeof(IEEEtypes_Header_t));
			break;
		case HT_OPERATION:
			pbss_entry->pht_info =
				(IEEEtypes_HTInfo_t *)pcurrent_ptr;
			offset = pcurrent_ptr - pbss_entry->pbeacon_buf;
			pbss_entry->ht_info_offset = (t_u16)(offset);
			HEXDUMP("InterpretIE: Resp HTINFO_IE",
				(t_u8 *)pbss_entry->pht_info,
				(*(pbss_entry->pht_info)).ieee_hdr.len +
					sizeof(IEEEtypes_Header_t));
			break;
		case BSSCO_2040:
			pbss_entry->pbss_co_2040 =
				(IEEEtypes_2040BSSCo_t *)pcurrent_ptr;
			offset = pcurrent_ptr - pbss_entry->pbeacon_buf;
			pbss_entry->bss_co_2040_offset = (t_u16)(offset);
			HEXDUMP("InterpretIE: Resp 2040BSSCOEXISTANCE_IE",
				(t_u8 *)pbss_entry->pbss_co_2040,
				(*(pbss_entry->pbss_co_2040)).ieee_hdr.len +
					sizeof(IEEEtypes_Header_t));
			break;
		case EXT_CAPABILITY:
			pbss_entry->pext_cap =
				(IEEEtypes_ExtCap_t *)pcurrent_ptr;
			offset = pcurrent_ptr - pbss_entry->pbeacon_buf;
			pbss_entry->ext_cap_offset = (t_u16)(offset);
			HEXDUMP("InterpretIE: Resp EXTCAP_IE",
				(t_u8 *)pbss_entry->pext_cap,
				(*(pbss_entry->pext_cap)).ieee_hdr.len +
					sizeof(IEEEtypes_Header_t));
			break;
		case OVERLAPBSSSCANPARAM:
			pbss_entry->poverlap_bss_scan_param =
				(IEEEtypes_OverlapBSSScanParam_t *)pcurrent_ptr;
			offset = pcurrent_ptr - pbss_entry->pbeacon_buf;
			pbss_entry->overlap_bss_offset = (t_u16)(offset);
			HEXDUMP("InterpretIE: Resp OBSS_IE",
				(t_u8 *)pbss_entry->poverlap_bss_scan_param,
				(*(pbss_entry->poverlap_bss_scan_param))
						.ieee_hdr.len +
					sizeof(IEEEtypes_Header_t));
			break;
		case VHT_CAPABILITY:
			pbss_entry->pvht_cap =
				(IEEEtypes_VHTCap_t *)pcurrent_ptr;
			offset = pcurrent_ptr - pbss_entry->pbeacon_buf;
			pbss_entry->vht_cap_offset = (t_u16)(offset);
			HEXDUMP("InterpretIE: Resp VHTCAP_IE",
				(t_u8 *)pbss_entry->pvht_cap,
				(*(pbss_entry->pvht_cap)).ieee_hdr.len +
					sizeof(IEEEtypes_Header_t));
			break;
		case VHT_OPERATION:
			pbss_entry->pvht_oprat =
				(IEEEtypes_VHTOprat_t *)pcurrent_ptr;
			offset = pcurrent_ptr - pbss_entry->pbeacon_buf;
			pbss_entry->vht_oprat_offset = (t_u16)(offset);
			HEXDUMP("InterpretIE: Resp VHTOPER_IE",
				(t_u8 *)pbss_entry->pvht_oprat,
				(*(pbss_entry->pvht_oprat)).ieee_hdr.len +
					sizeof(IEEEtypes_Header_t));
			break;
		case EXT_BSS_LOAD:
			pbss_entry->pext_bssload =
				(IEEEtypes_ExtBSSload_t *)pcurrent_ptr;
			offset = pcurrent_ptr - pbss_entry->pbeacon_buf;
			pbss_entry->ext_bssload_offset = (t_u16)(offset);
			HEXDUMP("InterpretIE: Resp EXTBSSLOAD_IE",
				(t_u8 *)pbss_entry->pext_bssload,
				(*(pbss_entry->pext_bssload)).ieee_hdr.len +
					sizeof(IEEEtypes_Header_t));
			break;
		case VHT_TX_POWER_ENV:
			pbss_entry->pvht_txpower =
				(IEEEtypes_VHTtxpower_t *)pcurrent_ptr;
			offset = pcurrent_ptr - pbss_entry->pbeacon_buf;
			pbss_entry->vht_txpower_offset = (t_u16)(offset);
			HEXDUMP("InterpretIE: Resp TXPOW_IE",
				(t_u8 *)pbss_entry->pvht_txpower,
				(*(pbss_entry->pvht_txpower)).ieee_hdr.len +
					sizeof(IEEEtypes_Header_t));
			break;
		case EXT_POWER_CONSTR:
			pbss_entry->pext_pwer =
				(IEEEtypes_ExtPwerCons_t *)pcurrent_ptr;
			offset = pcurrent_ptr - pbss_entry->pbeacon_buf;
			pbss_entry->ext_pwer_offset = (t_u16)(offset);
			HEXDUMP("InterpretIE: Resp EXTPOW_IE",
				(t_u8 *)pbss_entry->pext_pwer,
				(*(pbss_entry->pext_pwer)).ieee_hdr.len +
					sizeof(IEEEtypes_Header_t));
			break;
		case QUIET_CHAN:
			pbss_entry->pquiet_chan =
				(IEEEtypes_QuietChan_t *)pcurrent_ptr;
			offset = pcurrent_ptr - pbss_entry->pbeacon_buf;
			pbss_entry->quiet_chan_offset = (t_u16)(offset);
			HEXDUMP("InterpretIE: Resp QUIETCHAN_IE",
				(t_u8 *)pbss_entry->pquiet_chan,
				(*(pbss_entry->pquiet_chan)).ieee_hdr.len +
					sizeof(IEEEtypes_Header_t));
			break;
		case BW_CHANNEL_SWITCH:
			/* RANDYTODO */
			break;
		case AID_INFO:
			break;
		case OPER_MODE_NTF:
			pbss_entry->poper_mode =
				(IEEEtypes_OperModeNtf_t *)pcurrent_ptr;
			offset = pcurrent_ptr - pbss_entry->pbeacon_buf;
			pbss_entry->oper_mode_offset = (t_u16)(offset);
			HEXDUMP("InterpretIE: Resp OPERMODENTF_IE",
				(t_u8 *)pbss_entry->poper_mode,
				(*(pbss_entry->poper_mode)).ieee_hdr.len +
					sizeof(IEEEtypes_Header_t));
			break;
		case EXTENSION:
			pext_tlv = (IEEEtypes_Extension_t *)pcurrent_ptr;
			switch (pext_tlv->ext_id) {
			case HE_CAPABILITY:
				pbss_entry->phe_cap =
					(IEEEtypes_HECap_t *)pcurrent_ptr;
				offset = pcurrent_ptr - pbss_entry->pbeacon_buf;
				pbss_entry->he_cap_offset = (t_u16)(offset);
				break;
			case HE_OPERATION:
				pbss_entry->phe_oprat = pext_tlv;
				offset = pcurrent_ptr - pbss_entry->pbeacon_buf;
				pbss_entry->he_oprat_offset = (t_u16)(offset);
				break;
			case MU_EDCA_PARAM_SET:
				PRINTM(MCMND, "MU-EDCA IE received\n");
				pbss_entry->pmuedca_ie =
					(IEEEtypes_MUEDCAParamSet_t *)
						pcurrent_ptr;
				offset = pcurrent_ptr - pbss_entry->pbeacon_buf;
				pbss_entry->muedca_offset = (t_u16)(offset);
				break;
			case HE_6G_CAPABILITY:
				pbss_entry->phe_6g_cap =
					(IEEEtypes_HE6GCap_t *)pcurrent_ptr;
				offset = pcurrent_ptr - pbss_entry->pbeacon_buf;
				pbss_entry->he_6g_cap_offset = (t_u16)(offset);
				break;
			case MBSSID_CONFIG:
				pbss_entry->pmbssid_config =
					(IEEEtypes_MBSSID_Config_t *)
						pcurrent_ptr;
				offset = pcurrent_ptr - pbss_entry->pbeacon_buf;
				pbss_entry->mbssid_config_offset =
					(t_u16)(offset);
				break;
			default:
				break;
			}
			break;
		case MOBILITY_DOMAIN:
			PRINTM(MCMND, "Mobility Domain IE received in Scan\n");
			pbss_entry->pmd_ie =
				(IEEEtypes_MobilityDomain_t *)pcurrent_ptr;
			offset = pcurrent_ptr - pbss_entry->pbeacon_buf;
			pbss_entry->md_offset = (t_u16)(offset);
			HEXDUMP("InterpretIE: Resp Mobility Domain IE",
				(t_u8 *)pbss_entry->pmd_ie,
				(*(pbss_entry->pmd_ie)).ieee_hdr.len +
					sizeof(IEEEtypes_Header_t));
			break;
		case RNR:
			PRINTM(MINFO, "RNR IE received in Scan\n");
			pbss_entry->prnr_ie =
				(IEEEtypes_Generic_t *)pcurrent_ptr;
			offset = pcurrent_ptr - pbss_entry->pbeacon_buf;
			pbss_entry->rnr_offset = (t_u16)(offset);
			HEXDUMP("InterpretIE: RNR IE",
				(t_u8 *)pbss_entry->prnr_ie,
				(*(pbss_entry->prnr_ie)).ieee_hdr.len +
					sizeof(IEEEtypes_Header_t));
			break;
		default:
			break;
		}

		pcurrent_ptr += element_len + 2;

		/* Need to account for IE ID and IE Len */
		bytes_left_for_current_beacon -= (element_len + 2);

	} /* while (bytes_left_for_current_beacon > 2) */

	LEAVE();
	return ret;
}

/**
 *  @brief Adjust ie's position in BSSDescriptor_t
 *
 *  @param pmpriv       A pointer to mlan_private structure
 *  @param pbss_entry   A pointer to BSSDescriptor_t structure
 *
 *  @return           N/A
 */
static t_void wlan_adjust_ie_in_bss_entry(mlan_private *pmpriv,
					  BSSDescriptor_t *pbss_entry)
{
	ENTER();
	if (pbss_entry->pbeacon_buf) {
		if (pbss_entry->pwpa_ie) {
			pbss_entry->pwpa_ie =
				(IEEEtypes_VendorSpecific_t
					 *)(pbss_entry->pbeacon_buf +
					    pbss_entry->wpa_offset);
		}
		if (pbss_entry->prsn_ie) {
			pbss_entry->prsn_ie =
				(IEEEtypes_Generic_t *)(pbss_entry->pbeacon_buf +
							pbss_entry->rsn_offset);
		}
		if (pbss_entry->pwapi_ie) {
			pbss_entry->pwapi_ie =
				(IEEEtypes_Generic_t *)(pbss_entry->pbeacon_buf +
							pbss_entry->wapi_offset);
		}

		if (pbss_entry->posen_ie) {
			pbss_entry->posen_ie =
				(IEEEtypes_Generic_t *)(pbss_entry->pbeacon_buf +
							pbss_entry->osen_offset);
		}
		if (pbss_entry->pmd_ie) {
			pbss_entry->pmd_ie =
				(IEEEtypes_MobilityDomain_t
					 *)(pbss_entry->pbeacon_buf +
					    pbss_entry->md_offset);
		}
		if (pbss_entry->pht_cap) {
			pbss_entry->pht_cap =
				(IEEEtypes_HTCap_t *)(pbss_entry->pbeacon_buf +
						      pbss_entry->ht_cap_offset);
		}
		if (pbss_entry->pht_info) {
			pbss_entry->pht_info =
				(IEEEtypes_HTInfo_t
					 *)(pbss_entry->pbeacon_buf +
					    pbss_entry->ht_info_offset);
		}
		if (pbss_entry->pbss_co_2040) {
			pbss_entry->pbss_co_2040 =
				(IEEEtypes_2040BSSCo_t
					 *)(pbss_entry->pbeacon_buf +
					    pbss_entry->bss_co_2040_offset);
		}
		if (pbss_entry->pext_cap) {
			pbss_entry->pext_cap =
				(IEEEtypes_ExtCap_t
					 *)(pbss_entry->pbeacon_buf +
					    pbss_entry->ext_cap_offset);
		}
		if (pbss_entry->poverlap_bss_scan_param) {
			pbss_entry->poverlap_bss_scan_param =
				(IEEEtypes_OverlapBSSScanParam_t
					 *)(pbss_entry->pbeacon_buf +
					    pbss_entry->overlap_bss_offset);
		}
		if (pbss_entry->pvht_cap) {
			pbss_entry->pvht_cap =
				(IEEEtypes_VHTCap_t
					 *)(pbss_entry->pbeacon_buf +
					    pbss_entry->vht_cap_offset);
		}
		if (pbss_entry->pvht_oprat) {
			pbss_entry->pvht_oprat =
				(IEEEtypes_VHTOprat_t
					 *)(pbss_entry->pbeacon_buf +
					    pbss_entry->vht_oprat_offset);
		}
		if (pbss_entry->pvht_txpower) {
			pbss_entry->pvht_txpower =
				(IEEEtypes_VHTtxpower_t
					 *)(pbss_entry->pbeacon_buf +
					    pbss_entry->vht_txpower_offset);
		}
		if (pbss_entry->pext_pwer) {
			pbss_entry->pext_pwer =
				(IEEEtypes_ExtPwerCons_t
					 *)(pbss_entry->pbeacon_buf +
					    pbss_entry->ext_pwer_offset);
		}
		if (pbss_entry->pext_bssload) {
			pbss_entry->pext_bssload =
				(IEEEtypes_ExtBSSload_t
					 *)(pbss_entry->pbeacon_buf +
					    pbss_entry->ext_bssload_offset);
		}
		if (pbss_entry->pquiet_chan) {
			pbss_entry->pquiet_chan =
				(IEEEtypes_QuietChan_t
					 *)(pbss_entry->pbeacon_buf +
					    pbss_entry->quiet_chan_offset);
		}
		if (pbss_entry->poper_mode) {
			pbss_entry->poper_mode =
				(IEEEtypes_OperModeNtf_t
					 *)(pbss_entry->pbeacon_buf +
					    pbss_entry->oper_mode_offset);
		}
		if (pbss_entry->phe_cap) {
			pbss_entry->phe_cap =
				(IEEEtypes_HECap_t *)(pbss_entry->pbeacon_buf +
						      pbss_entry->he_cap_offset);
		}

		if (pbss_entry->phe_oprat) {
			pbss_entry->phe_oprat =
				(IEEEtypes_Extension_t
					 *)(pbss_entry->pbeacon_buf +
					    pbss_entry->he_oprat_offset);
		}

		if (pbss_entry->pmuedca_ie) {
			pbss_entry->pmuedca_ie =
				(IEEEtypes_MUEDCAParamSet_t
					 *)(pbss_entry->pbeacon_buf +
					    pbss_entry->muedca_offset);
		}
		if (pbss_entry->phe_6g_cap) {
			pbss_entry->phe_6g_cap =
				(IEEEtypes_HE6GCap_t
					 *)(pbss_entry->pbeacon_buf +
					    pbss_entry->he_6g_cap_offset);
		}

		if (pbss_entry->prnr_ie) {
			pbss_entry->prnr_ie =
				(IEEEtypes_Generic_t *)(pbss_entry->pbeacon_buf +
							pbss_entry->rnr_offset);
		}
		if (pbss_entry->prsnx_ie) {
			pbss_entry->prsnx_ie =
				(IEEEtypes_Generic_t *)(pbss_entry->pbeacon_buf +
							pbss_entry->rsnx_offset);
		}
		if (pbss_entry->pmbssid_config) {
			pbss_entry->pmbssid_config =
				(IEEEtypes_MBSSID_Config_t
					 *)(pbss_entry->pbeacon_buf +
					    pbss_entry->mbssid_config_offset);
		}
	} else {
		pbss_entry->pwpa_ie = MNULL;
		pbss_entry->wpa_offset = 0;
		pbss_entry->prsn_ie = MNULL;
		pbss_entry->rsn_offset = 0;
		pbss_entry->pwapi_ie = MNULL;
		pbss_entry->wapi_offset = 0;

		pbss_entry->posen_ie = MNULL;
		pbss_entry->osen_offset = 0;
		pbss_entry->pmd_ie = MNULL;
		pbss_entry->md_offset = 0;
		pbss_entry->pht_cap = MNULL;
		pbss_entry->ht_cap_offset = 0;
		pbss_entry->pht_info = MNULL;
		pbss_entry->ht_info_offset = 0;
		pbss_entry->pbss_co_2040 = MNULL;
		pbss_entry->bss_co_2040_offset = 0;
		pbss_entry->pext_cap = MNULL;
		pbss_entry->ext_cap_offset = 0;
		pbss_entry->poverlap_bss_scan_param = MNULL;
		pbss_entry->overlap_bss_offset = 0;
		pbss_entry->pmuedca_ie = MNULL;
		pbss_entry->muedca_offset = 0;
		pbss_entry->pmbssid_config = MNULL;
		pbss_entry->mbssid_config_offset = 0;
	}
	LEAVE();
	return;
}

/**
 *  @brief Store a beacon or probe response for a BSS returned in the scan
 *
 *  Store a new scan response or an update for a previous scan response.  New
 *    entries need to verify that they do not exceed the total amount of
 *    memory allocated for the table.

 *  Replacement entries need to take into consideration the amount of space
 *    currently allocated for the beacon/probe response and adjust the entry
 *    as needed.
 *
 *  A small amount of extra pad (SCAN_BEACON_ENTRY_PAD) is generally reserved
 *    for an entry in case it is a beacon since a probe response for the
 *    network will by larger per the standard.  This helps to reduce the
 *    amount of memory copying to fit a new probe response into an entry
 *    already occupied by a network's previously stored beacon.
 *
 *  @param pmpriv       A pointer to mlan_private structure
 *  @param beacon_idx   Index in the scan table to store this entry; may be
 *                      replacing an older duplicate entry for this BSS
 *  @param num_of_ent   Number of entries currently in the table
 *  @param pnew_beacon  Pointer to the new beacon/probe response to save
 *
 *  @return           N/A
 */
static t_void wlan_ret_802_11_scan_store_beacon(mlan_private *pmpriv,
						t_u32 beacon_idx,
						t_u32 num_of_ent,
						BSSDescriptor_t *pnew_beacon)
{
	mlan_adapter *pmadapter = pmpriv->adapter;
	t_u8 *pbcn_store;
	t_u32 new_bcn_size;
	t_u32 old_bcn_size;
	t_u32 bcn_space;
	t_u32 adj_idx;
	mlan_status ret = MLAN_STATUS_SUCCESS;
	t_u8 *tmp_buf;
	t_u32 bcn_size = 0;
	t_u32 bcn_offset = 0;

	ENTER();

	if (pmadapter->pscan_table[beacon_idx].pbeacon_buf) {
		new_bcn_size = pnew_beacon->beacon_buf_size;
		old_bcn_size =
			pmadapter->pscan_table[beacon_idx].beacon_buf_size;
		bcn_space =
			pmadapter->pscan_table[beacon_idx].beacon_buf_size_max;
		pbcn_store = pmadapter->pscan_table[beacon_idx].pbeacon_buf;

		/* Set the max to be the same as current entry unless changed
		 * below */
		pnew_beacon->beacon_buf_size_max = bcn_space;

		if (new_bcn_size == old_bcn_size) {
			/*
			 * Beacon is the same size as the previous entry.
			 *   Replace the previous contents with the scan result
			 */
			memcpy_ext(pmadapter, pbcn_store,
				   pnew_beacon->pbeacon_buf,
				   pnew_beacon->beacon_buf_size,
				   pnew_beacon->beacon_buf_size);

		} else if (new_bcn_size <= bcn_space) {
			/*
			 * New beacon size will fit in the amount of space
			 *   we have previously allocated for it
			 */

			/* Copy the new beacon buffer entry over the old one */
			memcpy_ext(pmadapter, pbcn_store,
				   pnew_beacon->pbeacon_buf, new_bcn_size,
				   new_bcn_size);

			/*
			 *  If the old beacon size was less than the
			 *  maximum we had allotted for the entry, and
			 *  the new entry is even smaller, reset the
			 *  max size to the old beacon entry and compress
			 *  the storage space (leaving a new pad space of
			 *  (old_bcn_size - new_bcn_size).
			 */
			if (old_bcn_size < bcn_space &&
			    new_bcn_size <= old_bcn_size) {
				/*
				 * Old Beacon size is smaller than the
				 * allotted storage size. Shrink the
				 * allotted storage space.
				 */
				PRINTM(MINFO,
				       "AppControl: Smaller Duplicate Beacon (%d), "
				       "old = %d, new = %d, space = %d, left = %d\n",
				       beacon_idx, old_bcn_size, new_bcn_size,
				       bcn_space,
				       (pmadapter->bcn_buf_size -
					(pmadapter->pbcn_buf_end -
					 pmadapter->bcn_buf)));

				/*
				 * memmove (since the memory overlaps) the data
				 * after the beacon we just stored to the end
				 * of the current beacon.  This cleans up any
				 * unused space the old larger beacon was using
				 * in the buffer
				 */
				memmove(pmadapter,
					(void *)((t_ptr)pbcn_store +
						 (t_ptr)old_bcn_size),
					(void *)((t_ptr)pbcn_store +
						 (t_ptr)bcn_space),
					(t_u32)((t_ptr)pmadapter->pbcn_buf_end -
						((t_ptr)pbcn_store +
						 (t_ptr)bcn_space)));

				/*
				 * Decrement the end pointer by the difference
				 * between the old larger size and the new
				 * smaller size since we are using less space
				 * due to the new beacon being smaller
				 */
				pmadapter->pbcn_buf_end -=
					(bcn_space - old_bcn_size);

				/*
				 * Set the maximum storage size to the old
				 * beacon size
				 */
				pnew_beacon->beacon_buf_size_max = old_bcn_size;

				/* Adjust beacon buffer pointers that are past
				 * the current */
				for (adj_idx = 0; adj_idx < num_of_ent;
				     adj_idx++) {
					if (pmadapter->pscan_table[adj_idx]
						    .pbeacon_buf > pbcn_store) {
						pmadapter->pscan_table[adj_idx]
							.pbeacon_buf -=
							(bcn_space -
							 old_bcn_size);
						wlan_adjust_ie_in_bss_entry(
							pmpriv,
							&pmadapter->pscan_table
								 [adj_idx]);
					}
				}
			}
		} else if (pmadapter->pbcn_buf_end +
				   (new_bcn_size - bcn_space) <
			   (pmadapter->bcn_buf + pmadapter->bcn_buf_size)) {
			/*
			 * Beacon is larger than space previously allocated
			 * (bcn_space) and there is enough space left in the
			 * beaconBuffer to store the additional data
			 */
			PRINTM(MINFO,
			       "AppControl: Larger Duplicate Beacon (%d), "
			       "old = %d, new = %d, space = %d, left = %d\n",
			       beacon_idx, old_bcn_size, new_bcn_size,
			       bcn_space,
			       (pmadapter->bcn_buf_size -
				(pmadapter->pbcn_buf_end - pmadapter->bcn_buf)));

			/*
			 * memmove (since the memory overlaps) the data
			 *  after the beacon we just stored to the end of
			 *  the current beacon.  This moves the data for
			 *  the beacons after this further in memory to
			 *  make space for the new larger beacon we are
			 *  about to copy in.
			 */
			memmove(pmadapter,
				(void *)((t_ptr)pbcn_store +
					 (t_ptr)new_bcn_size),
				(void *)((t_ptr)pbcn_store + (t_ptr)bcn_space),
				(t_u32)((t_ptr)pmadapter->pbcn_buf_end -
					((t_ptr)pbcn_store + (t_ptr)bcn_space)));

			/* Copy the new beacon buffer entry over the old one */
			memcpy_ext(pmadapter, pbcn_store,
				   pnew_beacon->pbeacon_buf, new_bcn_size,
				   new_bcn_size);

			/*
			 * Move the beacon end pointer by the amount of new
			 * beacon data we are adding
			 */
			pmadapter->pbcn_buf_end += (new_bcn_size - bcn_space);

			/*
			 * This entry is bigger than the allotted max space
			 *  previously reserved.  Increase the max space to
			 *  be equal to the new beacon size
			 */
			pnew_beacon->beacon_buf_size_max = new_bcn_size;

			/* Adjust beacon buffer pointers that are past the
			 * current */
			for (adj_idx = 0; adj_idx < num_of_ent; adj_idx++) {
				if (pmadapter->pscan_table[adj_idx].pbeacon_buf >
				    pbcn_store) {
					pmadapter->pscan_table[adj_idx]
						.pbeacon_buf +=
						(new_bcn_size - bcn_space);
					wlan_adjust_ie_in_bss_entry(
						pmpriv,
						&pmadapter->pscan_table[adj_idx]);
				}
			}
		} else {
			/*
			 * Beacon is larger than the previously allocated
			 * space, but there is not enough free space to
			 * store the additional data
			 */
			PRINTM(MERROR,
			       "AppControl: Failed: Larger Duplicate Beacon (%d),"
			       " old = %d, new = %d, space = %d, left = %d\n",
			       beacon_idx, old_bcn_size, new_bcn_size,
			       bcn_space,
			       (pmadapter->bcn_buf_size -
				(pmadapter->pbcn_buf_end - pmadapter->bcn_buf)));

			/* Storage failure, keep old beacon intact */
			pnew_beacon->beacon_buf_size = old_bcn_size;
			if (pnew_beacon->pwpa_ie)
				pnew_beacon->wpa_offset =
					pmadapter->pscan_table[beacon_idx]
						.wpa_offset;
			if (pnew_beacon->prsn_ie)
				pnew_beacon->rsn_offset =
					pmadapter->pscan_table[beacon_idx]
						.rsn_offset;
			if (pnew_beacon->pwapi_ie)
				pnew_beacon->wapi_offset =
					pmadapter->pscan_table[beacon_idx]
						.wapi_offset;

			if (pnew_beacon->posen_ie)
				pnew_beacon->osen_offset =
					pmadapter->pscan_table[beacon_idx]
						.osen_offset;
			if (pnew_beacon->pmd_ie)
				pnew_beacon->md_offset =
					pmadapter->pscan_table[beacon_idx]
						.md_offset;
			if (pnew_beacon->pht_cap)
				pnew_beacon->ht_cap_offset =
					pmadapter->pscan_table[beacon_idx]
						.ht_cap_offset;
			if (pnew_beacon->pht_info)
				pnew_beacon->ht_info_offset =
					pmadapter->pscan_table[beacon_idx]
						.ht_info_offset;
			if (pnew_beacon->pbss_co_2040)
				pnew_beacon->bss_co_2040_offset =
					pmadapter->pscan_table[beacon_idx]
						.bss_co_2040_offset;
			if (pnew_beacon->pext_cap)
				pnew_beacon->ext_cap_offset =
					pmadapter->pscan_table[beacon_idx]
						.ext_cap_offset;
			if (pnew_beacon->poverlap_bss_scan_param)
				pnew_beacon->overlap_bss_offset =
					pmadapter->pscan_table[beacon_idx]
						.overlap_bss_offset;
			if (pnew_beacon->pvht_cap)
				pnew_beacon->vht_cap_offset =
					pmadapter->pscan_table[beacon_idx]
						.vht_cap_offset;
			if (pnew_beacon->pvht_oprat)
				pnew_beacon->vht_oprat_offset =
					pmadapter->pscan_table[beacon_idx]
						.vht_oprat_offset;
			if (pnew_beacon->pvht_txpower)
				pnew_beacon->vht_txpower_offset =
					pmadapter->pscan_table[beacon_idx]
						.vht_txpower_offset;
			if (pnew_beacon->pext_pwer)
				pnew_beacon->ext_pwer_offset =
					pmadapter->pscan_table[beacon_idx]
						.ext_pwer_offset;
			if (pnew_beacon->pext_bssload)
				pnew_beacon->ext_bssload_offset =
					pmadapter->pscan_table[beacon_idx]
						.ext_bssload_offset;
			if (pnew_beacon->pquiet_chan)
				pnew_beacon->quiet_chan_offset =
					pmadapter->pscan_table[beacon_idx]
						.quiet_chan_offset;
			if (pnew_beacon->poper_mode)
				pnew_beacon->oper_mode_offset =
					pmadapter->pscan_table[beacon_idx]
						.oper_mode_offset;
			if (pnew_beacon->phe_cap)
				pnew_beacon->he_cap_offset =
					pmadapter->pscan_table[beacon_idx]
						.he_cap_offset;
			if (pnew_beacon->phe_oprat)
				pnew_beacon->he_oprat_offset =
					pmadapter->pscan_table[beacon_idx]
						.he_oprat_offset;
			if (pnew_beacon->pmuedca_ie)
				pnew_beacon->muedca_offset =
					pmadapter->pscan_table[beacon_idx]
						.muedca_offset;
			if (pnew_beacon->phe_6g_cap)
				pnew_beacon->he_6g_cap_offset =
					pmadapter->pscan_table[beacon_idx]
						.he_6g_cap_offset;
			if (pnew_beacon->prnr_ie)
				pnew_beacon->rnr_offset =
					pmadapter->pscan_table[beacon_idx]
						.rnr_offset;
			if (pnew_beacon->prsnx_ie)
				pnew_beacon->rsnx_offset =
					pmadapter->pscan_table[beacon_idx]
						.rsnx_offset;
			if (pnew_beacon->pmbssid_config)
				pnew_beacon->mbssid_config_offset =
					pmadapter->pscan_table[beacon_idx]
						.mbssid_config_offset;
		}
		/* Point the new entry to its permanent storage space */
		pnew_beacon->pbeacon_buf = pbcn_store;
		wlan_adjust_ie_in_bss_entry(pmpriv, pnew_beacon);
	} else {
		if ((pmadapter->pbcn_buf_end + pnew_beacon->beacon_buf_size +
			     SCAN_BEACON_ENTRY_PAD >
		     (pmadapter->bcn_buf + pmadapter->bcn_buf_size)) &&
		    (pmadapter->bcn_buf_size < MAX_SCAN_BEACON_BUFFER)) {
			/* no space for this entry, realloc bcn buffer */
			if (pmadapter->callbacks.moal_vmalloc &&
			    pmadapter->callbacks.moal_vfree)
				ret = pmadapter->callbacks.moal_vmalloc(
					pmadapter->pmoal_handle,
					pmadapter->bcn_buf_size +
						DEFAULT_SCAN_BEACON_BUFFER,
					(t_u8 **)&tmp_buf);
			else
				ret = pmadapter->callbacks.moal_malloc(
					pmadapter->pmoal_handle,
					pmadapter->bcn_buf_size +
						DEFAULT_SCAN_BEACON_BUFFER,
					MLAN_MEM_DEF, (t_u8 **)&tmp_buf);

			if ((ret == MLAN_STATUS_SUCCESS) && (tmp_buf)) {
				PRINTM(MCMND,
				       "Realloc Beacon buffer, old size=%d, new_size=%d\n",
				       pmadapter->bcn_buf_size,
				       pmadapter->bcn_buf_size +
					       DEFAULT_SCAN_BEACON_BUFFER);
				bcn_size = pmadapter->pbcn_buf_end -
					   pmadapter->bcn_buf;
				memcpy_ext(pmadapter, tmp_buf,
					   pmadapter->bcn_buf, bcn_size,
					   bcn_size);
				/* Adjust beacon buffer pointers that are past
				 * the current */
				for (adj_idx = 0; adj_idx < num_of_ent;
				     adj_idx++) {
					bcn_offset =
						pmadapter->pscan_table[adj_idx]
							.pbeacon_buf -
						pmadapter->bcn_buf;
					pmadapter->pscan_table[adj_idx]
						.pbeacon_buf =
						tmp_buf + bcn_offset;
					wlan_adjust_ie_in_bss_entry(
						pmpriv,
						&pmadapter->pscan_table[adj_idx]);
				}
				pmadapter->pbcn_buf_end = tmp_buf + bcn_size;
				if (pmadapter->callbacks.moal_vmalloc &&
				    pmadapter->callbacks.moal_vfree)
					pmadapter->callbacks.moal_vfree(
						pmadapter->pmoal_handle,
						(t_u8 *)pmadapter->bcn_buf);
				else
					pmadapter->callbacks.moal_mfree(
						pmadapter->pmoal_handle,
						(t_u8 *)pmadapter->bcn_buf);
				pmadapter->bcn_buf = tmp_buf;
				pmadapter->bcn_buf_size +=
					DEFAULT_SCAN_BEACON_BUFFER;
			}
		}
		/*
		 * No existing beacon data exists for this entry, check to see
		 *   if we can fit it in the remaining space
		 */
		if (pmadapter->pbcn_buf_end + pnew_beacon->beacon_buf_size +
			    SCAN_BEACON_ENTRY_PAD <
		    (pmadapter->bcn_buf + pmadapter->bcn_buf_size)) {
			/*
			 * Copy the beacon buffer data from the local entry
			 * to the adapter dev struct buffer space used to
			 * store the raw beacon data for each entry in the
			 * scan table
			 */
			memcpy_ext(pmadapter, pmadapter->pbcn_buf_end,
				   pnew_beacon->pbeacon_buf,
				   pnew_beacon->beacon_buf_size,
				   pnew_beacon->beacon_buf_size);

			/*
			 * Update the beacon ptr to point to the table
			 * save area
			 */
			pnew_beacon->pbeacon_buf = pmadapter->pbcn_buf_end;
			pnew_beacon->beacon_buf_size_max =
				(pnew_beacon->beacon_buf_size +
				 SCAN_BEACON_ENTRY_PAD);
			wlan_adjust_ie_in_bss_entry(pmpriv, pnew_beacon);

			/* Increment the end pointer by the size reserved */
			pmadapter->pbcn_buf_end +=
				pnew_beacon->beacon_buf_size_max;

			PRINTM(MINFO,
			       "AppControl: Beacon[%02d] sz=%03d,"
			       " used = %04d, left = %04d\n",
			       beacon_idx, pnew_beacon->beacon_buf_size,
			       (pmadapter->pbcn_buf_end - pmadapter->bcn_buf),
			       (pmadapter->bcn_buf_size -
				(pmadapter->pbcn_buf_end - pmadapter->bcn_buf)));
		} else {
			/*
			 * No space for new beacon
			 */
			PRINTM(MCMND,
			       "AppControl: No space beacon (%d): " MACSTR
			       "; sz=%03d, left=%03d\n",
			       beacon_idx, MAC2STR(pnew_beacon->mac_address),
			       pnew_beacon->beacon_buf_size,
			       (pmadapter->bcn_buf_size -
				(pmadapter->pbcn_buf_end - pmadapter->bcn_buf)));

			/*
			 * Storage failure; clear storage records
			 * for this bcn
			 */
			pnew_beacon->pbeacon_buf = MNULL;
			pnew_beacon->beacon_buf_size = 0;
			pnew_beacon->beacon_buf_size_max = 0;
			wlan_adjust_ie_in_bss_entry(pmpriv, pnew_beacon);
		}
	}

	LEAVE();
}

/**
 *  @brief update beacon buffer of the current bss descriptor
 *
 *  @param pmpriv       A pointer to mlan_private structure
 *
 *  @return             MLAN_STATUS_SUCCESS, otherwise failure
 */
static mlan_status wlan_update_curr_bcn(mlan_private *pmpriv)
{
	BSSDescriptor_t *pcurr_bss = &pmpriv->curr_bss_params.bss_descriptor;
	mlan_status ret = MLAN_STATUS_SUCCESS;

	ENTER();

	if (pmpriv->pcurr_bcn_buf && pmpriv->curr_bcn_size) {
		pcurr_bss->pbeacon_buf = pmpriv->pcurr_bcn_buf;
		pcurr_bss->beacon_buf_size = pmpriv->curr_bcn_size;
		pcurr_bss->beacon_buf_size_max = pmpriv->curr_bcn_size;

		/* adjust the pointers in the current bss descriptor */
		if (pcurr_bss->pwpa_ie) {
			pcurr_bss->pwpa_ie =
				(IEEEtypes_VendorSpecific_t
					 *)(pcurr_bss->pbeacon_buf +
					    pcurr_bss->wpa_offset);
		}
		if (pcurr_bss->prsn_ie) {
			pcurr_bss->prsn_ie =
				(IEEEtypes_Generic_t *)(pcurr_bss->pbeacon_buf +
							pcurr_bss->rsn_offset);
		}
		if (pcurr_bss->pmd_ie) {
			pcurr_bss->pmd_ie = (IEEEtypes_MobilityDomain_t
						     *)(pcurr_bss->pbeacon_buf +
							pcurr_bss->md_offset);
		}
		if (pcurr_bss->pht_cap) {
			pcurr_bss->pht_cap =
				(IEEEtypes_HTCap_t *)(pcurr_bss->pbeacon_buf +
						      pcurr_bss->ht_cap_offset);
		}

		if (pcurr_bss->pht_info) {
			pcurr_bss->pht_info =
				(IEEEtypes_HTInfo_t
					 *)(pcurr_bss->pbeacon_buf +
					    pcurr_bss->ht_info_offset);
		}

		if (pcurr_bss->pbss_co_2040) {
			pcurr_bss->pbss_co_2040 =
				(IEEEtypes_2040BSSCo_t
					 *)(pcurr_bss->pbeacon_buf +
					    pcurr_bss->bss_co_2040_offset);
		}

		if (pcurr_bss->pext_cap) {
			pcurr_bss->pext_cap =
				(IEEEtypes_ExtCap_t
					 *)(pcurr_bss->pbeacon_buf +
					    pcurr_bss->ext_cap_offset);
		}

		if (pcurr_bss->poverlap_bss_scan_param) {
			pcurr_bss->poverlap_bss_scan_param =
				(IEEEtypes_OverlapBSSScanParam_t
					 *)(pcurr_bss->pbeacon_buf +
					    pcurr_bss->overlap_bss_offset);
		}

		if (pcurr_bss->pvht_cap) {
			pcurr_bss->pvht_cap =
				(IEEEtypes_VHTCap_t
					 *)(pcurr_bss->pbeacon_buf +
					    pcurr_bss->vht_cap_offset);
		}

		if (pcurr_bss->pvht_oprat) {
			pcurr_bss->pvht_oprat =
				(IEEEtypes_VHTOprat_t
					 *)(pcurr_bss->pbeacon_buf +
					    pcurr_bss->vht_oprat_offset);
		}

		if (pcurr_bss->pvht_txpower) {
			pcurr_bss->pvht_txpower =
				(IEEEtypes_VHTtxpower_t
					 *)(pcurr_bss->pbeacon_buf +
					    pcurr_bss->vht_txpower_offset);
		}

		if (pcurr_bss->pext_pwer) {
			pcurr_bss->pext_pwer =
				(IEEEtypes_ExtPwerCons_t
					 *)(pcurr_bss->pbeacon_buf +
					    pcurr_bss->ext_pwer_offset);
		}

		if (pcurr_bss->pext_bssload) {
			pcurr_bss->pext_bssload =
				(IEEEtypes_ExtBSSload_t
					 *)(pcurr_bss->pbeacon_buf +
					    pcurr_bss->ext_bssload_offset);
		}

		if (pcurr_bss->pquiet_chan) {
			pcurr_bss->pquiet_chan =
				(IEEEtypes_QuietChan_t
					 *)(pcurr_bss->pbeacon_buf +
					    pcurr_bss->quiet_chan_offset);
		}

		if (pcurr_bss->poper_mode) {
			pcurr_bss->poper_mode =
				(IEEEtypes_OperModeNtf_t
					 *)(pcurr_bss->pbeacon_buf +
					    pcurr_bss->oper_mode_offset);
		}
		if (pcurr_bss->phe_cap) {
			pcurr_bss->phe_cap =
				(IEEEtypes_HECap_t *)(pcurr_bss->pbeacon_buf +
						      pcurr_bss->he_cap_offset);
		}

		if (pcurr_bss->phe_oprat) {
			pcurr_bss->phe_oprat =
				(IEEEtypes_Extension_t
					 *)(pcurr_bss->pbeacon_buf +
					    pcurr_bss->he_oprat_offset);
		}

		if (pcurr_bss->pmuedca_ie) {
			pcurr_bss->pmuedca_ie =
				(IEEEtypes_MUEDCAParamSet_t
					 *)(pcurr_bss->pbeacon_buf +
					    pcurr_bss->muedca_offset);
		}
		if (pcurr_bss->phe_6g_cap) {
			pcurr_bss->phe_6g_cap =
				(IEEEtypes_HE6GCap_t
					 *)(pcurr_bss->pbeacon_buf +
					    pcurr_bss->he_6g_cap_offset);
		}

		if (pcurr_bss->prnr_ie) {
			pcurr_bss->prnr_ie =
				(IEEEtypes_Generic_t *)(pcurr_bss->pbeacon_buf +
							pcurr_bss->rnr_offset);
		}
		if (pcurr_bss->prsnx_ie) {
			pcurr_bss->prsnx_ie =
				(IEEEtypes_Generic_t *)(pcurr_bss->pbeacon_buf +
							pcurr_bss->rsnx_offset);
		}
		if (pcurr_bss->pmbssid_config) {
			pcurr_bss->pmbssid_config =
				(IEEEtypes_MBSSID_Config_t
					 *)(pcurr_bss->pbeacon_buf +
					    pcurr_bss->mbssid_config_offset);
		}
		PRINTM(MINFO, "current beacon restored %d\n",
		       pmpriv->curr_bcn_size);
	} else {
		PRINTM(MERROR, "curr_bcn_buf not saved\n");
		ret = MLAN_STATUS_FAILURE;
	}

	LEAVE();
	return ret;
}

/**
 *  @brief get the chan load from chan stats.
 *
 *  @param pmadapter    A pointer to mlan_adapter structure
 *  @param channel      channel  *
 *
 *  @return             channel load
 */
static t_u16 wlan_get_chan_load(mlan_adapter *pmadapter, t_u8 channel)
{
	t_u16 chan_load = 0;
	int i;
	for (i = 0; i < (int)pmadapter->num_in_chan_stats; i++) {
		if ((pmadapter->pchan_stats[i].chan_num == channel) &&
		    pmadapter->pchan_stats[i].cca_scan_duration) {
			chan_load =
				(pmadapter->pchan_stats[i].cca_busy_duration *
				 100) /
				pmadapter->pchan_stats[i].cca_scan_duration;
			break;
		}
	}
	return chan_load;
}

/**
 *  @brief get the chan min/max rssi
 *
 *  @param pmadapter    A pointer to mlan_adapter structure
 *  @param channel      channel  *
 *  @param min_flag     flag to get min rssi
 *  @return             rssi
 */
static t_u8 wlan_get_chan_rssi(mlan_adapter *pmadapter, t_u8 channel,
			       t_u8 min_flag)
{
	t_u8 rssi = 0;
	int i;
	for (i = 0; i < (int)pmadapter->num_in_scan_table; i++) {
		if (pmadapter->pscan_table[i].channel == channel) {
			if (rssi == 0)
				rssi = (t_u8)pmadapter->pscan_table[i].rssi;
			else {
				if (min_flag)
					rssi = (t_u8)MIN(
						rssi,
						pmadapter->pscan_table[i].rssi);
				else
					rssi = (t_u8)MAX(
						rssi,
						pmadapter->pscan_table[i].rssi);
			}
		}
	}
	return rssi;
}

/**
 *  @brief update the min/max rssi for channel statistics.
 *
 *  @param pmadapter    A pointer to mlan_adapter structure
 *	@return             N/A
 */
static t_void wlan_update_chan_rssi(mlan_adapter *pmadapter)
{
	int i;
	t_s8 min_rssi = 0;
	t_s8 max_rssi = 0;
	t_s8 rss = 0;
	for (i = 0; i < (int)pmadapter->num_in_chan_stats; i++) {
		if (pmadapter->pchan_stats[i].chan_num &&
		    pmadapter->pchan_stats[i].cca_scan_duration) {
			min_rssi = -wlan_get_chan_rssi(
				pmadapter, pmadapter->pchan_stats[i].chan_num,
				MFALSE);
			max_rssi = -wlan_get_chan_rssi(
				pmadapter, pmadapter->pchan_stats[i].chan_num,
				MTRUE);
			rss = min_rssi - pmadapter->pchan_stats[i].noise;
			// rss should always > 0, FW need fix the wrong
			// rssi/noise in scantable
			if (rss > 0)
				pmadapter->pchan_stats[i].min_rss = rss;
			else
				pmadapter->pchan_stats[i].min_rss = 0;

			rss = max_rssi - pmadapter->pchan_stats[i].noise;
			if (rss > 0)
				pmadapter->pchan_stats[i].max_rss = rss;
			else
				pmadapter->pchan_stats[i].max_rss = 0;
			PRINTM(MCMND,
			       "chan=%d, min_rssi=%d, max_rssi=%d noise=%d min_rss=%d, max_rss=%d\n",
			       pmadapter->pchan_stats[i].chan_num, min_rssi,
			       max_rssi, pmadapter->pchan_stats[i].noise,
			       pmadapter->pchan_stats[i].min_rss,
			       pmadapter->pchan_stats[i].max_rss);
		}
	}
	return;
}

/**
 *  @brief Post process the scan table after a new scan command has completed
 *
 *  Inspect each entry of the scan table and try to find an entry that
 *    matches our current associated/joined network from the scan.  If
 *    one is found, update the stored copy of the BSSDescriptor for our
 *    current network.
 *
 *  Debug dump the current scan table contents if compiled accordingly.
 *
 *  @param pmpriv       A pointer to mlan_private structure
 *
 *  @return             N/A
 */
static t_void wlan_scan_process_results(mlan_private *pmpriv)
{
	mlan_adapter *pmadapter = pmpriv->adapter;
	t_s32 j;
	t_u32 i;
	mlan_status ret = MLAN_STATUS_SUCCESS;
	BSSDescriptor_t *bss_new_entry = MNULL;
	pmlan_callbacks pcb = &pmadapter->callbacks;

	ENTER();

	if (pmpriv->media_connected == MTRUE) {
		j = wlan_find_bssid_in_list(
			pmpriv,
			pmpriv->curr_bss_params.bss_descriptor.mac_address,
			pmpriv->bss_mode);

		if (j >= 0) {
			memcpy_ext(pmadapter, &pmadapter->pscan_table[j].ssid,
				   &pmpriv->curr_bss_params.bss_descriptor.ssid,
				   sizeof(mlan_802_11_ssid),
				   sizeof(mlan_802_11_ssid));
			pmadapter->callbacks.moal_spin_lock(
				pmadapter->pmoal_handle,
				pmpriv->curr_bcn_buf_lock);
			pmpriv->curr_bss_params.bss_descriptor.pwpa_ie = MNULL;
			pmpriv->curr_bss_params.bss_descriptor.wpa_offset = 0;
			pmpriv->curr_bss_params.bss_descriptor.prsn_ie = MNULL;
			pmpriv->curr_bss_params.bss_descriptor.rsn_offset = 0;
			pmpriv->curr_bss_params.bss_descriptor.pwapi_ie = MNULL;
			pmpriv->curr_bss_params.bss_descriptor.wapi_offset = 0;

			pmpriv->curr_bss_params.bss_descriptor.posen_ie = MNULL;
			pmpriv->curr_bss_params.bss_descriptor.osen_offset = 0;
			pmpriv->curr_bss_params.bss_descriptor.pmd_ie = MNULL;
			pmpriv->curr_bss_params.bss_descriptor.md_offset = 0;
			pmpriv->curr_bss_params.bss_descriptor.pht_cap = MNULL;
			pmpriv->curr_bss_params.bss_descriptor.ht_cap_offset =
				0;
			pmpriv->curr_bss_params.bss_descriptor.pht_info = MNULL;
			pmpriv->curr_bss_params.bss_descriptor.ht_info_offset =
				0;
			pmpriv->curr_bss_params.bss_descriptor.pbss_co_2040 =
				MNULL;
			pmpriv->curr_bss_params.bss_descriptor
				.bss_co_2040_offset = 0;
			pmpriv->curr_bss_params.bss_descriptor.pext_cap = MNULL;
			pmpriv->curr_bss_params.bss_descriptor.ext_cap_offset =
				0;
			pmpriv->curr_bss_params.bss_descriptor
				.poverlap_bss_scan_param = MNULL;
			pmpriv->curr_bss_params.bss_descriptor
				.overlap_bss_offset = 0;
			pmpriv->curr_bss_params.bss_descriptor.pvht_cap = MNULL;
			pmpriv->curr_bss_params.bss_descriptor.vht_cap_offset =
				0;
			pmpriv->curr_bss_params.bss_descriptor.pvht_oprat =
				MNULL;
			pmpriv->curr_bss_params.bss_descriptor.vht_oprat_offset =
				0;
			pmpriv->curr_bss_params.bss_descriptor.pvht_txpower =
				MNULL;
			pmpriv->curr_bss_params.bss_descriptor
				.vht_txpower_offset = 0;
			pmpriv->curr_bss_params.bss_descriptor.pext_pwer =
				MNULL;
			pmpriv->curr_bss_params.bss_descriptor.ext_pwer_offset =
				0;
			pmpriv->curr_bss_params.bss_descriptor.pext_bssload =
				MNULL;
			pmpriv->curr_bss_params.bss_descriptor
				.ext_bssload_offset = 0;
			pmpriv->curr_bss_params.bss_descriptor.pquiet_chan =
				MNULL;
			pmpriv->curr_bss_params.bss_descriptor
				.quiet_chan_offset = 0;
			pmpriv->curr_bss_params.bss_descriptor.poper_mode =
				MNULL;
			pmpriv->curr_bss_params.bss_descriptor.oper_mode_offset =
				0;
			pmpriv->curr_bss_params.bss_descriptor.phe_cap = MNULL;
			pmpriv->curr_bss_params.bss_descriptor.he_cap_offset =
				0;
			pmpriv->curr_bss_params.bss_descriptor.phe_oprat =
				MNULL;
			pmpriv->curr_bss_params.bss_descriptor.he_oprat_offset =
				0;
			pmpriv->curr_bss_params.bss_descriptor.pmuedca_ie =
				MNULL;
			pmpriv->curr_bss_params.bss_descriptor.muedca_offset =
				0;
			pmpriv->curr_bss_params.bss_descriptor.phe_6g_cap =
				MNULL;
			pmpriv->curr_bss_params.bss_descriptor.he_6g_cap_offset =
				0;
			pmpriv->curr_bss_params.bss_descriptor.prnr_ie = MNULL;
			pmpriv->curr_bss_params.bss_descriptor.rnr_offset = 0;
			pmpriv->curr_bss_params.bss_descriptor.prsnx_ie = MNULL;
			pmpriv->curr_bss_params.bss_descriptor.rsnx_offset = 0;
			pmpriv->curr_bss_params.bss_descriptor.pmbssid_config =
				MNULL;
			pmpriv->curr_bss_params.bss_descriptor
				.mbssid_config_offset = 0;
			pmpriv->curr_bss_params.bss_descriptor.pbeacon_buf =
				MNULL;
			pmpriv->curr_bss_params.bss_descriptor.beacon_buf_size =
				0;
			pmpriv->curr_bss_params.bss_descriptor
				.beacon_buf_size_max = 0;

			PRINTM(MINFO,
			       "Found current ssid/bssid in list @ index #%d\n",
			       j);
			/* Make a copy of current BSSID descriptor */
			memcpy_ext(
				pmadapter,
				&pmpriv->curr_bss_params.bss_descriptor,
				&pmadapter->pscan_table[j],
				sizeof(pmpriv->curr_bss_params.bss_descriptor),
				sizeof(pmpriv->curr_bss_params.bss_descriptor));

			pmadapter->callbacks.moal_spin_unlock(
				pmadapter->pmoal_handle,
				pmpriv->curr_bcn_buf_lock);
			wlan_save_curr_bcn(pmpriv);
		} else {
			// Apend to the end of scan table
			if (pmpriv->pcurr_bcn_buf && pmpriv->curr_bcn_size) {
				ret = pcb->moal_malloc(pmadapter->pmoal_handle,
						       sizeof(BSSDescriptor_t),
						       MLAN_MEM_DEF,
						       (t_u8 **)&bss_new_entry);
				if (ret == MLAN_STATUS_SUCCESS &&
				    bss_new_entry) {
					memcpy_ext(
						pmadapter, bss_new_entry,
						&pmpriv->curr_bss_params
							 .bss_descriptor,
						sizeof(pmpriv->curr_bss_params
							       .bss_descriptor),
						sizeof(BSSDescriptor_t));
					if (pmadapter->num_in_scan_table <
					    MRVDRV_MAX_BSSID_LIST)
						pmadapter->num_in_scan_table++;
					pmadapter
						->pscan_table
							[pmadapter->num_in_scan_table -
							 1]
						.pbeacon_buf = MNULL;
					wlan_ret_802_11_scan_store_beacon(
						pmpriv,
						pmadapter->num_in_scan_table -
							1,
						pmadapter->num_in_scan_table,
						bss_new_entry);
					if (bss_new_entry->pbeacon_buf ==
						    MNULL &&
					    pmadapter->num_in_scan_table)
						pmadapter->num_in_scan_table--;
					else
						memcpy_ext(
							pmadapter,
							&pmadapter->pscan_table
								 [pmadapter->num_in_scan_table -
								  1],
							bss_new_entry,
							sizeof(BSSDescriptor_t),
							sizeof(BSSDescriptor_t));
					pcb->moal_mfree(pmadapter->pmoal_handle,
							(t_u8 *)bss_new_entry);
				}
			}
		}
	}

	for (i = 0; i < pmadapter->num_in_scan_table; i++) {
		PRINTM(MINFO,
		       "Scan:(%02d) " MACSTR ", "
		       "RSSI[%03d], SSID[%s]\n",
		       i, MAC2STR(pmadapter->pscan_table[i].mac_address),
		       (t_s32)pmadapter->pscan_table[i].rssi,
		       pmadapter->pscan_table[i].ssid.ssid);
		pmadapter->pscan_table[i].chan_load = wlan_get_chan_load(
			pmadapter, pmadapter->pscan_table[i].channel);
	}
	wlan_update_chan_rssi(pmadapter);

	/*
	 * Prepares domain info from scan table and downloads the
	 *   domain info command to the FW.
	 */
	if (pmpriv->bss_role == MLAN_BSS_ROLE_STA)
		wlan_11d_prepare_dnld_domain_info_cmd(pmpriv);
	PRINTM(MMSG, "wlan: SCAN COMPLETED: scanned AP count=%d\n",
	       pmadapter->num_in_scan_table);
	LEAVE();
}

/**
 *  @brief Delete a specific indexed entry from the scan table.
 *
 *  Delete the scan table entry indexed by table_idx.  Compact the remaining
 *    entries and adjust any buffering of beacon/probe response data
 *    if needed.
 *
 *  @param pmpriv       A pointer to mlan_private structure
 *  @param table_idx    Scan table entry index to delete from the table
 *
 *  @return             N/A
 *
 *  @pre                table_idx must be an index to a valid entry
 */
static t_void wlan_scan_delete_table_entry(mlan_private *pmpriv,
					   t_s32 table_idx)
{
	mlan_adapter *pmadapter = pmpriv->adapter;
	t_u32 del_idx;
	t_u32 beacon_buf_adj;
	t_u8 *pbeacon_buf;

	ENTER();

	/*
	 * Shift the saved beacon buffer data for the scan table back over the
	 *   entry being removed.  Update the end of buffer pointer.  Save the
	 *   deleted buffer allocation size for pointer adjustments for entries
	 *   compacted after the deleted index.
	 */
	beacon_buf_adj = pmadapter->pscan_table[table_idx].beacon_buf_size_max;

	PRINTM(MINFO,
	       "Scan: Delete Entry %d, beacon buffer removal = %d bytes\n",
	       table_idx, beacon_buf_adj);

	/* Check if the table entry had storage allocated for its beacon */
	if (beacon_buf_adj) {
		pbeacon_buf = pmadapter->pscan_table[table_idx].pbeacon_buf;

		/*
		 * Remove the entry's buffer space, decrement the table
		 * end pointer by the amount we are removing
		 */
		pmadapter->pbcn_buf_end -= beacon_buf_adj;

		PRINTM(MINFO,
		       "Scan: Delete Entry %d, compact data: %p <- %p (sz = %d)\n",
		       table_idx, pbeacon_buf, pbeacon_buf + beacon_buf_adj,
		       pmadapter->pbcn_buf_end - pbeacon_buf);

		/*
		 * Compact data storage.  Copy all data after the deleted
		 * entry's end address (pbeacon_buf + beacon_buf_adj) back to
		 * the original start address (pbeacon_buf).
		 *
		 * Scan table entries affected by the move will have their entry
		 *   pointer adjusted below.
		 *
		 * Use memmove since the dest/src memory regions overlap.
		 */
		memmove(pmadapter, pbeacon_buf,
			(void *)((t_ptr)pbeacon_buf + (t_ptr)beacon_buf_adj),
			(t_u32)((t_ptr)pmadapter->pbcn_buf_end -
				(t_ptr)pbeacon_buf));
	}

	PRINTM(MINFO, "Scan: Delete Entry %d, num_in_scan_table = %d\n",
	       table_idx, pmadapter->num_in_scan_table);

	/*
	 * Shift all of the entries after the table_idx back by one, compacting
	 * the table and removing the requested entry
	 */
	for (del_idx = table_idx; (del_idx + 1) < pmadapter->num_in_scan_table;
	     del_idx++) {
		/* Copy the next entry over this one */
		memcpy_ext(pmadapter, pmadapter->pscan_table + del_idx,
			   pmadapter->pscan_table + del_idx + 1,
			   sizeof(BSSDescriptor_t), sizeof(BSSDescriptor_t));

		/*
		 * Adjust this entry's pointer to its beacon buffer based on the
		 *   removed/compacted entry from the deleted index.  Don't
		 * decrement if the buffer pointer is MNULL (no data stored for
		 * this entry).
		 */
		if (pmadapter->pscan_table[del_idx].pbeacon_buf) {
			pmadapter->pscan_table[del_idx].pbeacon_buf -=
				beacon_buf_adj;
			if (pmadapter->pscan_table[del_idx].pwpa_ie) {
				pmadapter->pscan_table[del_idx].pwpa_ie =
					(IEEEtypes_VendorSpecific_t
						 *)(pmadapter
							    ->pscan_table[del_idx]
							    .pbeacon_buf +
						    pmadapter
							    ->pscan_table[del_idx]
							    .wpa_offset);
			}
			if (pmadapter->pscan_table[del_idx].prsn_ie) {
				pmadapter->pscan_table[del_idx].prsn_ie =
					(IEEEtypes_Generic_t
						 *)(pmadapter
							    ->pscan_table[del_idx]
							    .pbeacon_buf +
						    pmadapter
							    ->pscan_table[del_idx]
							    .rsn_offset);
			}
			if (pmadapter->pscan_table[del_idx].pwapi_ie) {
				pmadapter->pscan_table[del_idx].pwapi_ie =
					(IEEEtypes_Generic_t
						 *)(pmadapter
							    ->pscan_table[del_idx]
							    .pbeacon_buf +
						    pmadapter
							    ->pscan_table[del_idx]
							    .wapi_offset);
			}

			if (pmadapter->pscan_table[del_idx].posen_ie) {
				pmadapter->pscan_table[del_idx].posen_ie =
					(IEEEtypes_Generic_t
						 *)(pmadapter
							    ->pscan_table[del_idx]
							    .pbeacon_buf +
						    pmadapter
							    ->pscan_table[del_idx]
							    .osen_offset);
			}
			if (pmadapter->pscan_table[del_idx].pmd_ie) {
				pmadapter->pscan_table[del_idx].pmd_ie =
					(IEEEtypes_MobilityDomain_t
						 *)(pmadapter
							    ->pscan_table[del_idx]
							    .pbeacon_buf +
						    pmadapter
							    ->pscan_table[del_idx]
							    .md_offset);
			}
			if (pmadapter->pscan_table[del_idx].pht_cap) {
				pmadapter->pscan_table[del_idx].pht_cap =
					(IEEEtypes_HTCap_t
						 *)(pmadapter
							    ->pscan_table[del_idx]
							    .pbeacon_buf +
						    pmadapter
							    ->pscan_table[del_idx]
							    .ht_cap_offset);
			}

			if (pmadapter->pscan_table[del_idx].pht_info) {
				pmadapter->pscan_table[del_idx].pht_info =
					(IEEEtypes_HTInfo_t
						 *)(pmadapter
							    ->pscan_table[del_idx]
							    .pbeacon_buf +
						    pmadapter
							    ->pscan_table[del_idx]
							    .ht_info_offset);
			}
			if (pmadapter->pscan_table[del_idx].pbss_co_2040) {
				pmadapter->pscan_table[del_idx].pbss_co_2040 =
					(IEEEtypes_2040BSSCo_t
						 *)(pmadapter
							    ->pscan_table[del_idx]
							    .pbeacon_buf +
						    pmadapter
							    ->pscan_table[del_idx]
							    .bss_co_2040_offset);
			}
			if (pmadapter->pscan_table[del_idx].pext_cap) {
				pmadapter->pscan_table[del_idx].pext_cap =
					(IEEEtypes_ExtCap_t
						 *)(pmadapter
							    ->pscan_table[del_idx]
							    .pbeacon_buf +
						    pmadapter
							    ->pscan_table[del_idx]
							    .ext_cap_offset);
			}
			if (pmadapter->pscan_table[del_idx]
				    .poverlap_bss_scan_param) {
				pmadapter->pscan_table[del_idx]
					.poverlap_bss_scan_param =
					(IEEEtypes_OverlapBSSScanParam_t
						 *)(pmadapter
							    ->pscan_table[del_idx]
							    .pbeacon_buf +
						    pmadapter
							    ->pscan_table[del_idx]
							    .overlap_bss_offset);
			}

			if (pmadapter->pscan_table[del_idx].pvht_cap) {
				pmadapter->pscan_table[del_idx].pvht_cap =
					(IEEEtypes_VHTCap_t
						 *)(pmadapter
							    ->pscan_table[del_idx]
							    .pbeacon_buf +
						    pmadapter
							    ->pscan_table[del_idx]
							    .vht_cap_offset);
			}

			if (pmadapter->pscan_table[del_idx].pvht_oprat) {
				pmadapter->pscan_table[del_idx].pvht_oprat =
					(IEEEtypes_VHTOprat_t
						 *)(pmadapter
							    ->pscan_table[del_idx]
							    .pbeacon_buf +
						    pmadapter
							    ->pscan_table[del_idx]
							    .vht_oprat_offset);
			}
			if (pmadapter->pscan_table[del_idx].pvht_txpower) {
				pmadapter->pscan_table[del_idx].pvht_txpower =
					(IEEEtypes_VHTtxpower_t
						 *)(pmadapter
							    ->pscan_table[del_idx]
							    .pbeacon_buf +
						    pmadapter
							    ->pscan_table[del_idx]
							    .vht_txpower_offset);
			}
			if (pmadapter->pscan_table[del_idx].pext_pwer) {
				pmadapter->pscan_table[del_idx].pext_pwer =
					(IEEEtypes_ExtPwerCons_t
						 *)(pmadapter
							    ->pscan_table[del_idx]
							    .pbeacon_buf +
						    pmadapter
							    ->pscan_table[del_idx]
							    .ext_pwer_offset);
			}
			if (pmadapter->pscan_table[del_idx].pext_bssload) {
				pmadapter->pscan_table[del_idx].pext_bssload =
					(IEEEtypes_ExtBSSload_t
						 *)(pmadapter
							    ->pscan_table[del_idx]
							    .pbeacon_buf +
						    pmadapter
							    ->pscan_table[del_idx]
							    .ext_bssload_offset);
			}
			if (pmadapter->pscan_table[del_idx].pquiet_chan) {
				pmadapter->pscan_table[del_idx].pquiet_chan =
					(IEEEtypes_QuietChan_t
						 *)(pmadapter
							    ->pscan_table[del_idx]
							    .pbeacon_buf +
						    pmadapter
							    ->pscan_table[del_idx]
							    .quiet_chan_offset);
			}
			if (pmadapter->pscan_table[del_idx].poper_mode) {
				pmadapter->pscan_table[del_idx].poper_mode =
					(IEEEtypes_OperModeNtf_t
						 *)(pmadapter
							    ->pscan_table[del_idx]
							    .pbeacon_buf +
						    pmadapter
							    ->pscan_table[del_idx]
							    .oper_mode_offset);
			}
			if (pmadapter->pscan_table[del_idx].phe_cap) {
				pmadapter->pscan_table[del_idx].phe_cap =
					(IEEEtypes_HECap_t
						 *)(pmadapter
							    ->pscan_table[del_idx]
							    .pbeacon_buf +
						    pmadapter
							    ->pscan_table[del_idx]
							    .he_cap_offset);
			}
			if (pmadapter->pscan_table[del_idx].phe_oprat) {
				pmadapter->pscan_table[del_idx].phe_oprat =
					(IEEEtypes_Extension_t
						 *)(pmadapter
							    ->pscan_table[del_idx]
							    .pbeacon_buf +
						    pmadapter
							    ->pscan_table[del_idx]
							    .he_oprat_offset);
			}
			if (pmadapter->pscan_table[del_idx].pmuedca_ie) {
				pmadapter->pscan_table[del_idx].pmuedca_ie =
					(IEEEtypes_MUEDCAParamSet_t
						 *)(pmadapter
							    ->pscan_table[del_idx]
							    .pbeacon_buf +
						    pmadapter
							    ->pscan_table[del_idx]
							    .muedca_offset);
			}
			if (pmadapter->pscan_table[del_idx].phe_6g_cap) {
				pmadapter->pscan_table[del_idx].phe_6g_cap =
					(IEEEtypes_HE6GCap_t
						 *)(pmadapter
							    ->pscan_table[del_idx]
							    .pbeacon_buf +
						    pmadapter
							    ->pscan_table[del_idx]
							    .he_6g_cap_offset);
			}
			if (pmadapter->pscan_table[del_idx].prnr_ie) {
				pmadapter->pscan_table[del_idx].prnr_ie =
					(IEEEtypes_Generic_t
						 *)(pmadapter
							    ->pscan_table[del_idx]
							    .pbeacon_buf +
						    pmadapter
							    ->pscan_table[del_idx]
							    .rnr_offset);
			}
			if (pmadapter->pscan_table[del_idx].pmbssid_config) {
				pmadapter->pscan_table[del_idx].pmbssid_config =
					(IEEEtypes_MBSSID_Config_t
						 *)(pmadapter
							    ->pscan_table[del_idx]
							    .pbeacon_buf +
						    pmadapter
							    ->pscan_table[del_idx]
							    .mbssid_config_offset);
			}
		}
	}

	/* The last entry is invalid now that it has been deleted or moved back
	 */
	memset(pmadapter,
	       pmadapter->pscan_table + pmadapter->num_in_scan_table - 1, 0x00,
	       sizeof(BSSDescriptor_t));

	pmadapter->num_in_scan_table--;

	LEAVE();
}

/**
 *  @brief Delete all entry's age out
 *
 *  @param pmpriv       A pointer to mlan_private structure
 *
 *  @return             N/A
 */
static void wlan_scan_delete_ageout_entry(mlan_private *pmpriv)
{
	BSSDescriptor_t *pbss_entry;
	mlan_adapter *pmadapter = pmpriv->adapter;
	t_s32 table_idx = pmadapter->num_in_scan_table - 1;
	t_u32 i = 0;
	t_u32 age_in_secs = 0;
	t_u32 age_ts_usec = 0;

	ENTER();
#define SCAN_RESULT_AGEOUT 10
	pmadapter->callbacks.moal_get_system_time(pmadapter->pmoal_handle,
						  &age_in_secs, &age_ts_usec);

	for (i = 0; i < pmadapter->num_in_scan_table; i++) {
		pbss_entry = &pmadapter->pscan_table[table_idx];
		if (age_in_secs >
		    (pbss_entry->age_in_secs + SCAN_RESULT_AGEOUT)) {
			PRINTM(MCMND,
			       "SCAN: ageout AP MAC Addr-" MACSTR
			       " ssid: %-32s\n",
			       MAC2STR(pbss_entry->mac_address),
			       pbss_entry->ssid.ssid);
			wlan_scan_delete_table_entry(pmpriv, table_idx);
		}
		table_idx--;
	}
	LEAVE();
	return;
}

/**
 *  @brief Delete all occurrences of a given SSID from the scan table
 *
 *  Iterate through the scan table and delete all entries that match a given
 *    SSID.  Compact the remaining scan table entries.
 *
 *  @param pmpriv       A pointer to mlan_private structure
 *  @param pdel_ssid    Pointer to an SSID to be used in deleting all
 *                        matching SSIDs from the scan table
 *
 *  @return             MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status
wlan_scan_delete_ssid_table_entry(mlan_private *pmpriv,
				  mlan_802_11_ssid *pdel_ssid)
{
	mlan_status ret = MLAN_STATUS_FAILURE;
	t_s32 table_idx;

	ENTER();

	PRINTM(MINFO, "Scan: Delete Ssid Entry: %-32s\n", pdel_ssid->ssid);

	/*
	 * If the requested SSID is found in the table, delete it.  Then keep
	 * searching the table for multiple entries for the SSID until no
	 * more are found
	 */
	while ((table_idx = wlan_find_ssid_in_list(pmpriv, pdel_ssid, MNULL,
						   MLAN_BSS_MODE_AUTO)) >= 0) {
		PRINTM(MINFO, "Scan: Delete SSID Entry: Found Idx = %d\n",
		       table_idx);
		ret = MLAN_STATUS_SUCCESS;
		wlan_scan_delete_table_entry(pmpriv, table_idx);
	}

	LEAVE();
	return ret;
}

/********************************************************
			Global Functions
********************************************************/

/**
 *  @brief Check if a scanned network compatible with the driver settings
 *
 *   WEP     WPA     WPA2    ad-hoc  encrypt                      Network
 * enabled enabled  enabled   AES     mode   Privacy  WPA  WPA2  Compatible
 *    0       0        0       0      NONE      0      0    0   yes No security
 *    0       1        0       0       x        1x     1    x   yes WPA (disable
 * HT if no AES) 0       0        1       0       x        1x     x    1   yes
 * WPA2 (disable HT if no AES) 0       0        0       1      NONE      1 0 0
 * yes Ad-hoc AES 1       0        0       0      NONE      1      0    0   yes
 * Static WEP (disable HT) 0       0        0       0     !=NONE     1      0 0
 * yes Dynamic WEP
 *
 *  @param pmpriv  A pointer to mlan_private
 *  @param index   Index in scan table to check against current driver settings
 *  @param mode    Network mode: Infrastructure or IBSS
 *
 *  @return        Index in ScanTable, or negative value if error
 */
t_s32 wlan_is_network_compatible(mlan_private *pmpriv, t_u32 index, t_u32 mode)
{
	mlan_adapter *pmadapter = pmpriv->adapter;
	BSSDescriptor_t *pbss_desc;

	ENTER();

	pbss_desc = &pmadapter->pscan_table[index];
	/* Don't check for compatibility if roaming */
	if ((pmpriv->media_connected == MTRUE) &&
	    (pmpriv->bss_mode == MLAN_BSS_MODE_INFRA) &&
	    (pbss_desc->bss_mode == MLAN_BSS_MODE_INFRA)) {
		LEAVE();
		return index;
	}

	pbss_desc->disable_11n = MFALSE;

	/* if the HE CAP IE exists, HT CAP IE should exist too */
	/* 2.4G AX AP, don't have VHT CAP */
	if (pbss_desc->phe_cap && !pbss_desc->pht_cap) {
		PRINTM(MINFO,
		       "Disable 11n if VHT CAP/HT CAP IE is not found from the 11AX AP\n");
		pbss_desc->disable_11n = MTRUE;
	}

	/* if the VHT CAP IE exists, the HT CAP IE should exist too */
	if (pbss_desc->pvht_cap && !pbss_desc->pht_cap) {
		PRINTM(MINFO,
		       "Disable 11n if HT CAP IE is not found from the 11AC AP\n");
		pbss_desc->disable_11n = MTRUE;
	}

	if (pbss_desc->wlan_11h_bss_info.chan_switch_ann.element_id ==
	    CHANNEL_SWITCH_ANN) {
		PRINTM(MINFO,
		       "Don't connect to AP with CHANNEL_SWITCH_ANN IE.\n");
		LEAVE();
		return -1;
	}

	if (pmpriv->wps.session_enable == MTRUE) {
		PRINTM(MINFO, "Return success directly in WPS period\n");
		LEAVE();
		return index;
	}
	if ((pbss_desc->owe_transition_mode == OWE_TRANS_MODE_OPEN) &&
	    (pmpriv->sec_info.authentication_mode != MLAN_AUTH_MODE_OWE)) {
		PRINTM(MINFO,
		       "Return success directly in OWE Transition mode\n");
		LEAVE();
		return index;
	}

	if (pmpriv->sec_info.osen_enabled && pbss_desc->posen_ie &&
	    ((*(pbss_desc->posen_ie)).ieee_hdr.element_id ==
	     VENDOR_SPECIFIC_221)) {
		/* Hotspot 2.0 OSEN AKM */
		PRINTM(MMSG,
		       "Return success directly in Hotspot OSEN: index=%d "
		       "encryption_mode=%#x\n",
		       index, pmpriv->sec_info.encryption_mode);
		LEAVE();
		return index;
	}

	if ((pbss_desc->bss_mode == mode) &&
	    (pmpriv->sec_info.ewpa_enabled == MTRUE ||
	     pmpriv->sec_info.authentication_mode == MLAN_AUTH_MODE_OWE ||
	     pbss_desc->owe_transition_mode == OWE_TRANS_MODE_OWE)) {
		if (((pbss_desc->pwpa_ie) &&
		     ((*(pbss_desc->pwpa_ie)).vend_hdr.element_id == WPA_IE)) ||
		    ((pbss_desc->prsn_ie) &&
		     ((*(pbss_desc->prsn_ie)).ieee_hdr.element_id == RSN_IE))) {
			if (((pmpriv->adapter->config_bands & BAND_GN ||
			      pmpriv->adapter->config_bands & BAND_AN) &&
			     pbss_desc->pht_cap) &&
			    (pmpriv->bss_mode == MLAN_BSS_MODE_INFRA) &&
			    !is_wpa_oui_present(pmpriv->adapter, pbss_desc,
						CIPHER_SUITE_CCMP) &&
			    !is_rsn_oui_present(pmpriv->adapter, pbss_desc,
						CIPHER_SUITE_CCMP) &&
			    !is_rsn_oui_present(pmpriv->adapter, pbss_desc,
						CIPHER_SUITE_GCMP) &&
			    !is_rsn_oui_present(pmpriv->adapter, pbss_desc,
						CIPHER_SUITE_GCMP_256) &&
			    !is_rsn_oui_present(pmpriv->adapter, pbss_desc,
						CIPHER_SUITE_CCMP_256)) {
				if (is_wpa_oui_present(pmpriv->adapter,
						       pbss_desc,
						       CIPHER_SUITE_TKIP) ||
				    is_rsn_oui_present(pmpriv->adapter,
						       pbss_desc,
						       CIPHER_SUITE_TKIP)) {
					PRINTM(MINFO,
					       "Disable 11n if AES is not supported by AP\n");
					pbss_desc->disable_11n = MTRUE;
				} else {
					LEAVE();
					return -1;
				}
			}
			LEAVE();
			return index;
		} else {
			PRINTM(MINFO,
			       "ewpa_enabled: Ignore none WPA/WPA2 AP\n");
			LEAVE();
			return -1;
		}
	}

	if (pmpriv->sec_info.wapi_enabled &&
	    (pbss_desc->pwapi_ie &&
	     ((*(pbss_desc->pwapi_ie)).ieee_hdr.element_id == WAPI_IE))) {
		PRINTM(MINFO, "Return success for WAPI AP\n");
		LEAVE();
		return index;
	}

	if (pbss_desc->bss_mode == mode) {
		if (pmpriv->sec_info.wep_status == Wlan802_11WEPDisabled &&
		    !pmpriv->sec_info.wpa_enabled &&
		    !pmpriv->sec_info.wpa2_enabled &&
		    ((!pbss_desc->pwpa_ie) ||
		     ((*(pbss_desc->pwpa_ie)).vend_hdr.element_id != WPA_IE)) &&
		    ((!pbss_desc->prsn_ie) ||
		     ((*(pbss_desc->prsn_ie)).ieee_hdr.element_id != RSN_IE)) &&
		    pmpriv->sec_info.encryption_mode ==
			    MLAN_ENCRYPTION_MODE_NONE &&
		    !pbss_desc->privacy) {
			/* No security */
			LEAVE();
			return index;
		} else if (pmpriv->sec_info.wep_status ==
				   Wlan802_11WEPEnabled &&
			   !pmpriv->sec_info.wpa_enabled &&
			   !pmpriv->sec_info.wpa2_enabled &&
			   pbss_desc->privacy) {
			/* Static WEP enabled */
			PRINTM(MINFO, "Disable 11n in WEP mode\n");
			pbss_desc->disable_11n = MTRUE;
			/* Reject the following cases: */
			/*
			 * case 1: RSN IE w/o WEP OUI and WPA IE w/o WEP OUI
			 * case 2: RSN IE w/o WEP OUI and No WPA IE
			 * case 3: WPA IE w/o WEP OUI and No RSN IE
			 */
			if (((pbss_desc->prsn_ie) &&
			     ((*(pbss_desc->prsn_ie)).ieee_hdr.element_id ==
			      RSN_IE)) ||
			    ((pbss_desc->pwpa_ie) &&
			     ((*(pbss_desc->pwpa_ie)).vend_hdr.element_id ==
			      WPA_IE))) {
				if (!is_rsn_oui_present(pmpriv->adapter,
							pbss_desc,
							CIPHER_SUITE_WEP40) &&
				    !is_rsn_oui_present(pmpriv->adapter,
							pbss_desc,
							CIPHER_SUITE_WEP104) &&
				    !is_wpa_oui_present(pmpriv->adapter,
							pbss_desc,
							CIPHER_SUITE_WEP40) &&
				    !is_wpa_oui_present(pmpriv->adapter,
							pbss_desc,
							CIPHER_SUITE_WEP104))
					index = -1;
			}

			LEAVE();
			return index;
		} else if (pmpriv->sec_info.wep_status ==
				   Wlan802_11WEPDisabled &&
			   pmpriv->sec_info.wpa_enabled &&
			   !pmpriv->sec_info.wpa2_enabled &&
			   ((pbss_desc->pwpa_ie) &&
			    ((*(pbss_desc->pwpa_ie)).vend_hdr.element_id ==
			     WPA_IE))
			   /*
			    * Privacy bit may NOT be set in some APs like
			    * LinkSys WRT54G && pbss_desc->privacy
			    */
		) {
			PRINTM(MINFO,
			       "wlan_is_network_compatible() WPA: index=%d wpa_ie=%#x "
			       "rsn_ie=%#x WEP=%s WPA=%s WPA2=%s EncMode=%#x "
			       "privacy=%#x\n",
			       index,
			       (pbss_desc->pwpa_ie) ?
				       (*(pbss_desc->pwpa_ie))
					       .vend_hdr.element_id :
				       0,
			       (pbss_desc->prsn_ie) ?
				       (*(pbss_desc->prsn_ie))
					       .ieee_hdr.element_id :
				       0,
			       (pmpriv->sec_info.wep_status ==
				Wlan802_11WEPEnabled) ?
				       "e" :
				       "d",
			       (pmpriv->sec_info.wpa_enabled) ? "e" : "d",
			       (pmpriv->sec_info.wpa2_enabled) ? "e" : "d",
			       pmpriv->sec_info.encryption_mode,
			       pbss_desc->privacy);
			if (((pmpriv->adapter->config_bands & BAND_GN ||
			      pmpriv->adapter->config_bands & BAND_AN) &&
			     pbss_desc->pht_cap) &&
			    (pmpriv->bss_mode == MLAN_BSS_MODE_INFRA) &&
			    !is_wpa_oui_present(pmpriv->adapter, pbss_desc,
						CIPHER_SUITE_CCMP)) {
				if (is_wpa_oui_present(pmpriv->adapter,
						       pbss_desc,
						       CIPHER_SUITE_TKIP)) {
					PRINTM(MINFO,
					       "Disable 11n if AES is not supported by AP\n");
					pbss_desc->disable_11n = MTRUE;
				} else {
					LEAVE();
					return -1;
				}
			}
			LEAVE();
			return index;
		} else if (pmpriv->sec_info.wep_status ==
				   Wlan802_11WEPDisabled &&
			   !pmpriv->sec_info.wpa_enabled &&
			   pmpriv->sec_info.wpa2_enabled &&
			   ((pbss_desc->prsn_ie) &&
			    ((*(pbss_desc->prsn_ie)).ieee_hdr.element_id ==
			     RSN_IE))
			   /*
			    * Privacy bit may NOT be set in some APs like
			    * LinkSys WRT54G && pbss_desc->privacy
			    */
		) {
			/* WPA2 enabled */
			PRINTM(MINFO,
			       "wlan_is_network_compatible() WPA2: index=%d wpa_ie=%#x "
			       "rsn_ie=%#x WEP=%s WPA=%s WPA2=%s EncMode=%#x "
			       "privacy=%#x\n",
			       index,
			       (pbss_desc->pwpa_ie) ?
				       (*(pbss_desc->pwpa_ie))
					       .vend_hdr.element_id :
				       0,
			       (pbss_desc->prsn_ie) ?
				       (*(pbss_desc->prsn_ie))
					       .ieee_hdr.element_id :
				       0,
			       (pmpriv->sec_info.wep_status ==
				Wlan802_11WEPEnabled) ?
				       "e" :
				       "d",
			       (pmpriv->sec_info.wpa_enabled) ? "e" : "d",
			       (pmpriv->sec_info.wpa2_enabled) ? "e" : "d",
			       pmpriv->sec_info.encryption_mode,
			       pbss_desc->privacy);
			if (((pmpriv->adapter->config_bands & BAND_GN ||
			      pmpriv->adapter->config_bands & BAND_AN) &&
			     pbss_desc->pht_cap) &&
			    (pmpriv->bss_mode == MLAN_BSS_MODE_INFRA) &&
			    !is_rsn_oui_present(pmpriv->adapter, pbss_desc,
						CIPHER_SUITE_CCMP) &&
			    !is_rsn_oui_present(pmpriv->adapter, pbss_desc,
						CIPHER_SUITE_GCMP) &&
			    !is_rsn_oui_present(pmpriv->adapter, pbss_desc,
						CIPHER_SUITE_GCMP_256) &&
			    !is_rsn_oui_present(pmpriv->adapter, pbss_desc,
						CIPHER_SUITE_CCMP_256)) {
				if (is_rsn_oui_present(pmpriv->adapter,
						       pbss_desc,
						       CIPHER_SUITE_TKIP)) {
					PRINTM(MINFO,
					       "Disable 11n if AES is not supported by AP\n");
					pbss_desc->disable_11n = MTRUE;
				} else if (is_rsn_oui_present_in_wpa_ie(
						   pmpriv, CIPHER_SUITE_CCMP) ||
					   is_rsn_oui_present_in_wpa_ie(
						   pmpriv, CIPHER_SUITE_GCMP) ||
					   is_rsn_oui_present_in_wpa_ie(
						   pmpriv,
						   CIPHER_SUITE_GCMP_256) ||
					   is_rsn_oui_present_in_wpa_ie(
						   pmpriv,
						   CIPHER_SUITE_CCMP_256)) {
					LEAVE();
					return index;
				} else {
					LEAVE();
					return -1;
				}
			}
			LEAVE();
			return index;
		} else if (pmpriv->sec_info.wep_status ==
				   Wlan802_11WEPDisabled &&
			   !pmpriv->sec_info.wpa_enabled &&
			   !pmpriv->sec_info.wpa2_enabled &&
			   ((!pbss_desc->pwpa_ie) ||
			    ((*(pbss_desc->pwpa_ie)).vend_hdr.element_id !=
			     WPA_IE)) &&
			   ((!pbss_desc->prsn_ie) ||
			    ((*(pbss_desc->prsn_ie)).ieee_hdr.element_id !=
			     RSN_IE)) &&
			   pmpriv->sec_info.encryption_mode !=
				   MLAN_ENCRYPTION_MODE_NONE &&
			   pbss_desc->privacy) {
			/* Dynamic WEP enabled */
			pbss_desc->disable_11n = MTRUE;
			PRINTM(MINFO,
			       "wlan_is_network_compatible() dynamic WEP: index=%d "
			       "wpa_ie=%#x rsn_ie=%#x EncMode=%#x privacy=%#x\n",
			       index,
			       (pbss_desc->pwpa_ie) ?
				       (*(pbss_desc->pwpa_ie))
					       .vend_hdr.element_id :
				       0,
			       (pbss_desc->prsn_ie) ?
				       (*(pbss_desc->prsn_ie))
					       .ieee_hdr.element_id :
				       0,
			       pmpriv->sec_info.encryption_mode,
			       pbss_desc->privacy);
			LEAVE();
			return index;
		}
		/* Security doesn't match */
		PRINTM(MINFO,
		       "wlan_is_network_compatible() FAILED: index=%d wpa_ie=%#x "
		       "rsn_ie=%#x WEP=%s WPA=%s WPA2=%s EncMode=%#x privacy=%#x\n",
		       index,
		       (pbss_desc->pwpa_ie) ?
			       (*(pbss_desc->pwpa_ie)).vend_hdr.element_id :
			       0,
		       (pbss_desc->prsn_ie) ?
			       (*(pbss_desc->prsn_ie)).ieee_hdr.element_id :
			       0,
		       (pmpriv->sec_info.wep_status == Wlan802_11WEPEnabled) ?
			       "e" :
			       "d",
		       (pmpriv->sec_info.wpa_enabled) ? "e" : "d",
		       (pmpriv->sec_info.wpa2_enabled) ? "e" : "d",
		       pmpriv->sec_info.encryption_mode, pbss_desc->privacy);
		LEAVE();
		return -1;
	}
	/* Mode doesn't match */
	LEAVE();
	return -1;
}

/**
 *  @brief Internal function used to flush the scan list
 *
 *  @param pmadapter    A pointer to mlan_adapter structure
 *
 *  @return             MLAN_STATUS_SUCCESS
 */
mlan_status wlan_flush_scan_table(pmlan_adapter pmadapter)
{
	t_u8 i = 0;
	ENTER();

	PRINTM(MINFO, "Flushing scan table\n");

	memset(pmadapter, pmadapter->pscan_table, 0,
	       (sizeof(BSSDescriptor_t) * MRVDRV_MAX_BSSID_LIST));
	pmadapter->num_in_scan_table = 0;

	memset(pmadapter, pmadapter->bcn_buf, 0, pmadapter->bcn_buf_size);
	pmadapter->pbcn_buf_end = pmadapter->bcn_buf;

	for (i = 0; i < pmadapter->num_in_chan_stats; i++)
		pmadapter->pchan_stats[i].cca_scan_duration = 0;
	pmadapter->idx_chan_stats = 0;

	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief Internal function used to start a scan based on an input config
 *
 *  Use the input user scan configuration information when provided in
 *    order to send the appropriate scan commands to firmware to populate or
 *    update the internal driver scan table
 *
 *  @param pmpriv          A pointer to mlan_private structure
 *  @param pioctl_buf      A pointer to MLAN IOCTL Request buffer
 *  @param puser_scan_in   Pointer to the input configuration for the requested
 *                         scan.
 *
 *  @return              MLAN_STATUS_SUCCESS or < 0 if error
 */
mlan_status wlan_scan_networks(mlan_private *pmpriv, t_void *pioctl_buf,
			       wlan_user_scan_cfg *puser_scan_in)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_adapter *pmadapter = pmpriv->adapter;
	mlan_callbacks *pcb = (mlan_callbacks *)&pmadapter->callbacks;
	cmd_ctrl_node *pcmd_node = MNULL;
	pmlan_ioctl_req pioctl_req = (mlan_ioctl_req *)pioctl_buf;

	wlan_scan_cmd_config_tlv *pscan_cfg_out = MNULL;
	MrvlIEtypes_ChanListParamSet_t *pchan_list_out;
	t_u32 buf_size;
	ChanScanParamSet_t *pscan_chan_list;

	t_u8 keep_previous_scan;
	t_u8 filtered_scan;
	t_u8 scan_current_chan_only;
	t_u8 max_chan_per_scan;

	ENTER();

	ret = pcb->moal_malloc(pmadapter->pmoal_handle,
			       sizeof(wlan_scan_cmd_config_tlv),
			       MLAN_MEM_DEF | MLAN_MEM_FLAG_ATOMIC,
			       (t_u8 **)&pscan_cfg_out);
	if (ret != MLAN_STATUS_SUCCESS || !pscan_cfg_out) {
		PRINTM(MERROR, "Memory allocation for pscan_cfg_out failed!\n");
		if (pioctl_req)
			pioctl_req->status_code = MLAN_ERROR_NO_MEM;
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}

	buf_size = sizeof(ChanScanParamSet_t) * WLAN_USER_SCAN_CHAN_MAX;
	ret = pcb->moal_malloc(pmadapter->pmoal_handle, buf_size,
			       MLAN_MEM_DEF | MLAN_MEM_FLAG_ATOMIC,
			       (t_u8 **)&pscan_chan_list);
	if (ret != MLAN_STATUS_SUCCESS || !pscan_chan_list) {
		PRINTM(MERROR, "Failed to allocate scan_chan_list\n");
		if (pscan_cfg_out)
			pcb->moal_mfree(pmadapter->pmoal_handle,
					(t_u8 *)pscan_cfg_out);
		if (pioctl_req)
			pioctl_req->status_code = MLAN_ERROR_NO_MEM;
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}

	keep_previous_scan = MFALSE;

	ret = wlan_scan_setup_scan_config(pmpriv, puser_scan_in,
					  &pscan_cfg_out->config,
					  &pchan_list_out, pscan_chan_list,
					  &max_chan_per_scan, &filtered_scan,
					  &scan_current_chan_only);
	if (ret != MLAN_STATUS_SUCCESS) {
		PRINTM(MERROR, "Failed to setup scan config\n");
		if (pscan_cfg_out)
			pcb->moal_mfree(pmadapter->pmoal_handle,
					(t_u8 *)pscan_cfg_out);
		if (pscan_chan_list)
			pcb->moal_mfree(pmadapter->pmoal_handle,
					(t_u8 *)pscan_chan_list);
		if (pioctl_req)
			pioctl_req->status_code = MLAN_ERROR_INVALID_PARAMETER;
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}

	if (puser_scan_in)
		keep_previous_scan = puser_scan_in->keep_previous_scan;

	if (pmadapter->scan_6g)
		keep_previous_scan = MTRUE;

	if (keep_previous_scan == MFALSE) {
		wlan_flush_scan_table(pmadapter);
		pmadapter->pbcn_buf_end = pmadapter->bcn_buf;
		wlan_free_rnr_coloc_ap(pmadapter);
	} else {
		wlan_scan_delete_ageout_entry(pmpriv);
	}

	ret = wlan_scan_channel_list(pmpriv, pioctl_buf, max_chan_per_scan,
				     filtered_scan, &pscan_cfg_out->config,
				     pchan_list_out, pscan_chan_list);

	/* Get scan command from scan_pending_q and put to cmd_pending_q */
	if (ret == MLAN_STATUS_SUCCESS) {
		wlan_request_cmd_lock(pmadapter);
		if (util_peek_list(pmadapter->pmoal_handle,
				   &pmadapter->scan_pending_q, MNULL, MNULL)) {
			pcmd_node = (cmd_ctrl_node *)util_dequeue_list(
				pmadapter->pmoal_handle,
				&pmadapter->scan_pending_q, MNULL, MNULL);
			pmadapter->pscan_ioctl_req = pioctl_req;
			pmadapter->scan_processing = MTRUE;
			pmadapter->scan_state = SCAN_STATE_SCAN_START;
			wlan_insert_cmd_to_pending_q(pmadapter, pcmd_node,
						     MTRUE);
		}
		wlan_release_cmd_lock(pmadapter);
	}
	if (pscan_cfg_out)
		pcb->moal_mfree(pmadapter->pmoal_handle, (t_u8 *)pscan_cfg_out);

	if (pscan_chan_list)
		pcb->moal_mfree(pmadapter->pmoal_handle,
				(t_u8 *)pscan_chan_list);

	LEAVE();
	return ret;
}

/**
 *  @brief Prepare a scan command to be sent to the firmware
 *
 *  Use the wlan_scan_cmd_config sent to the command processing module in
 *   the wlan_prepare_cmd to configure a HostCmd_DS_802_11_SCAN command
 *   struct to send to firmware.
 *
 *  The fixed fields specifying the BSS type and BSSID filters as well as a
 *   variable number/length of TLVs are sent in the command to firmware.
 *
 *  @param pmpriv     A pointer to mlan_private structure
 *  @param pcmd       A pointer to HostCmd_DS_COMMAND structure to be sent to
 *                    firmware with the HostCmd_DS_801_11_SCAN structure
 *  @param pdata_buf  Void pointer cast of a wlan_scan_cmd_config struct used
 *                    to set the fields/TLVs for the command sent to firmware
 *
 *  @return           MLAN_STATUS_SUCCESS
 */
mlan_status wlan_cmd_802_11_scan(mlan_private *pmpriv, HostCmd_DS_COMMAND *pcmd,
				 t_void *pdata_buf)
{
	HostCmd_DS_802_11_SCAN *pscan_cmd = &pcmd->params.scan;
	wlan_scan_cmd_config *pscan_cfg;

	ENTER();

	pscan_cfg = (wlan_scan_cmd_config *)pdata_buf;

	/* Set fixed field variables in scan command */
	pscan_cmd->bss_mode = pscan_cfg->bss_mode;
	memcpy_ext(pmpriv->adapter, pscan_cmd->bssid, pscan_cfg->specific_bssid,
		   sizeof(pscan_cmd->bssid), sizeof(pscan_cmd->bssid));
	memcpy_ext(pmpriv->adapter, pscan_cmd->tlv_buffer, pscan_cfg->tlv_buf,
		   pscan_cfg->tlv_buf_len, pscan_cfg->tlv_buf_len);

	pcmd->command = wlan_cpu_to_le16(HostCmd_CMD_802_11_SCAN);

	/* Size is equal to the sizeof(fixed portions) + the TLV len + header */
	pcmd->size = (t_u16)wlan_cpu_to_le16(
		(t_u16)(sizeof(pscan_cmd->bss_mode) + sizeof(pscan_cmd->bssid) +
			pscan_cfg->tlv_buf_len + S_DS_GEN));

	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief  Check if any hidden SSID found in passive scan channels
 *          and do specific SSID active scan for those channels
 *
 *  @param pmpriv       A pointer to mlan_private structure
 *  @param pioctl_buf   A pointer to mlan_ioctl_req structure
 *
 *  @return             MTRUE/MFALSE
 */

static t_bool wlan_active_scan_req_for_passive_chan(mlan_private *pmpriv,
						    mlan_ioctl_req *pioctl_buf)
{
	t_bool ret = MFALSE;
	mlan_adapter *pmadapter = pmpriv->adapter;
	t_bool scan_reqd = MFALSE;
	t_bool chan_listed = MFALSE;
	t_u8 id = 0;
	t_u32 bss_idx, i;
	t_u8 null_ssid[MLAN_MAX_SSID_LENGTH] = {0};
	mlan_callbacks *pcb = (mlan_callbacks *)&pmpriv->adapter->callbacks;
	wlan_user_scan_cfg *user_scan_cfg = MNULL;
	mlan_ds_scan *pscan = (mlan_ds_scan *)pioctl_buf->pbuf;
	mlan_scan_req *pscan_req = MNULL;
	wlan_user_scan_cfg *puser_scan_in = MNULL;
	t_u16 band;

	ENTER();

	if (pscan->sub_command == MLAN_OID_SCAN_USER_CONFIG) {
		puser_scan_in = (wlan_user_scan_cfg *)
					pscan->param.user_scan.scan_cfg_buf;
		if (!puser_scan_in->ssid_filter)
			goto done;
	}

	if (pmadapter->active_scan_triggered) {
		pmadapter->active_scan_triggered = MFALSE;
		goto done;
	}

	if ((pcb->moal_malloc(pmadapter->pmoal_handle,
			      sizeof(wlan_user_scan_cfg), MLAN_MEM_DEF,
			      (t_u8 **)&user_scan_cfg) !=
	     MLAN_STATUS_SUCCESS) ||
	    !user_scan_cfg) {
		PRINTM(MERROR, "Memory allocation for user_scan_cfg failed\n");
		goto done;
	}
	for (bss_idx = 0; bss_idx < pmadapter->num_in_scan_table; bss_idx++) {
		scan_reqd = MFALSE;
		if (!memcmp(pmadapter,
			    pmadapter->pscan_table[bss_idx].ssid.ssid,
			    null_ssid,
			    pmadapter->pscan_table[bss_idx].ssid.ssid_len)) {
			if (puser_scan_in &&
			    puser_scan_in->chan_list[0].chan_number) {
				for (i = 0;
				     i < WLAN_USER_SCAN_CHAN_MAX &&
				     puser_scan_in->chan_list[i].chan_number;
				     i++) {
					if (puser_scan_in->chan_list[i]
						    .chan_number ==
					    pmadapter->pscan_table[bss_idx]
						    .channel) {
						if (puser_scan_in->chan_list[i]
							    .scan_type ==
						    MLAN_SCAN_TYPE_PASSIVE)
							scan_reqd = MTRUE;
						break;
					}
				}
			} else if (pmadapter->scan_type ==
				   MLAN_SCAN_TYPE_PASSIVE) {
				scan_reqd = MTRUE;
			} else {
				if ((pmadapter->pscan_table[bss_idx].bss_band &
				     BAND_A) &&
				    wlan_11h_radar_detect_required(
					    pmpriv,
					    pmadapter->pscan_table[bss_idx]
						    .channel))
					scan_reqd = MTRUE;
				if (pmadapter->pscan_table[bss_idx].bss_band &
					    (BAND_B | BAND_G) &&
				    wlan_bg_scan_type_is_passive(
					    pmpriv,
					    pmadapter->pscan_table[bss_idx]
						    .channel))
					scan_reqd = MTRUE;
			}

			if (scan_reqd) {
				chan_listed = MFALSE;
				for (i = 0; i < id; i++) {
					band = radio_type_to_band(
						user_scan_cfg->chan_list[i]
							.radio_type);

					if ((user_scan_cfg->chan_list[i]
						     .chan_number ==
					     pmadapter->pscan_table[bss_idx]
						     .channel) &&
					    (band &
					     pmadapter->pscan_table[bss_idx]
						     .bss_band)) {
						chan_listed = MTRUE;
						break;
					}
				}
				if (chan_listed == MTRUE)
					continue;
				user_scan_cfg->chan_list[id].chan_number =
					pmadapter->pscan_table[bss_idx].channel;
				if (pmadapter->pscan_table[bss_idx].bss_band &
				    (BAND_B | BAND_G))
					user_scan_cfg->chan_list[id].radio_type =
						BAND_2GHZ;
				if (pmadapter->pscan_table[bss_idx].bss_band &
				    BAND_A)
					user_scan_cfg->chan_list[id].radio_type =
						BAND_5GHZ;
				user_scan_cfg->chan_list[id].scan_type =
					MLAN_SCAN_TYPE_ACTIVE;
				id++;

				if (id >= WLAN_USER_SCAN_CHAN_MAX)
					break;
			}
		}
	}
	if (id) {
		pmadapter->active_scan_triggered = MTRUE;
		if (pscan->sub_command == MLAN_OID_SCAN_USER_CONFIG) {
			memcpy_ext(pmpriv->adapter, user_scan_cfg->ssid_list,
				   puser_scan_in->ssid_list,
				   sizeof(user_scan_cfg->ssid_list),
				   sizeof(user_scan_cfg->ssid_list));
		} else {
			pscan_req = &pscan->param.scan_req;
			memcpy_ext(pmpriv->adapter,
				   user_scan_cfg->ssid_list[0].ssid,
				   pscan_req->scan_ssid.ssid,
				   pscan_req->scan_ssid.ssid_len,
				   MLAN_MAX_SSID_LENGTH);
		}
		user_scan_cfg->keep_previous_scan = MTRUE;
		if (pmadapter->ext_scan_type == EXT_SCAN_ENHANCE)
			user_scan_cfg->ext_scan_type = EXT_SCAN_ENHANCE;
		PRINTM(MCMND, "active scan request for passive channel %d\n",
		       id);
		if (MLAN_STATUS_SUCCESS !=
		    wlan_scan_networks(pmpriv, pioctl_buf, user_scan_cfg)) {
			goto done;
		}
		ret = MTRUE;
	}
done:
	if (user_scan_cfg)
		pcb->moal_mfree(pmadapter->pmoal_handle, (t_u8 *)user_scan_cfg);

	LEAVE();
	return ret;
}

/**
 *  @brief  Internal function used to start a scan for 6g network
 *
 *  Check the colocated ap list when parsed in 2.4G/5G scan result to
 *    send the appropriate scan commands to firmware to populate or
 *    update the internal driver scan table for 6G
 *
 *  @param pmpriv       A pointer to mlan_private structure
 *  @param pioctl_req   A pointer to mlan_ioctl_req structure
 *
 *  @return             MTRUE/MFALSE
 */
static t_bool wlan_scan_6g_network(mlan_private *pmpriv,
				   mlan_ioctl_req *pioctl_req)
{
	mlan_status status = MLAN_STATUS_SUCCESS;
	mlan_adapter *pmadapter = pmpriv->adapter;
	mlan_ds_scan *pscan = MNULL;
	wlan_user_scan_cfg *puser_scan_in = MNULL;
	wlan_6e_coloc_ap_t *pcoloc_ap = MNULL;
	t_u16 num = 0;

	ENTER();

	if (pioctl_req)
		pscan = (mlan_ds_scan *)pioctl_req->pbuf;

	if (!pioctl_req || !pscan) {
		PRINTM(MERROR, "No scan request could be identified!\n");
		LEAVE();
		return MFALSE;
	}

	if (pscan->sub_command == MLAN_OID_SCAN_USER_CONFIG) {
		puser_scan_in = (wlan_user_scan_cfg *)
					pscan->param.user_scan.scan_cfg_buf;
		if (puser_scan_in->scan_cfg_only ||
		    puser_scan_in->chan_list[0].chan_number)
			return MFALSE;
	}

	PRINTM(MMSG, "START 6G SCAN\n");
	pmadapter->scan_6g = MTRUE;

	/* Dump colocated ap before 6G scanning */
	pcoloc_ap =
		(wlan_6e_coloc_ap_t *)util_peek_list(pmadapter->pmoal_handle,
						     &pmadapter->coloc_ap_list,
						     MNULL, MNULL);
	while (pcoloc_ap &&
	       pcoloc_ap != (wlan_6e_coloc_ap_t *)&pmadapter->coloc_ap_list) {
		PRINTM(MCMND,
		       "Colocated AP(#%02d): " MACSTR ", "
		       "SSID[%s], Short-SSID[0x%0x], Operating Class[%d], Channel Number[%d], BSS parameters[0x%0x]\n",
		       num++, MAC2STR(pcoloc_ap->ap_info.bssid),
		       pcoloc_ap->ap_info.ssid.ssid,
		       pcoloc_ap->ap_info.short_ssid,
		       pcoloc_ap->ap_info.oper_class,
		       pcoloc_ap->ap_info.chan_number,
		       pcoloc_ap->ap_info.bss_params);
		pcoloc_ap = pcoloc_ap->pnext;
	}

	switch (pscan->sub_command) {
	case MLAN_OID_SCAN_NORMAL:
		status = wlan_scan_networks(pmpriv, pioctl_req, MNULL);
		break;
	case MLAN_OID_SCAN_SPECIFIC_SSID:
		status = wlan_scan_specific_ssid(
			pmpriv, pioctl_req, &pscan->param.scan_req.scan_ssid);
		break;
	case MLAN_OID_SCAN_USER_CONFIG:
		status = wlan_scan_networks(pmpriv, pioctl_req,
					    (wlan_user_scan_cfg *)pscan->param
						    .user_scan.scan_cfg_buf);
		break;
	}

	if (status != MLAN_STATUS_SUCCESS)
		return MFALSE;

	LEAVE();
	return MTRUE;
}

/**
 *  @brief This function handles the command response of scan
 *
 *   The response buffer for the scan command has the following
 *      memory layout:
 *
 *     .-------------------------------------------------------------.
 *     |  Header (4 * sizeof(t_u16)):  Standard command response hdr |
 *     .-------------------------------------------------------------.
 *     |  BufSize (t_u16) : sizeof the BSS Description data          |
 *     .-------------------------------------------------------------.
 *     |  NumOfSet (t_u8) : Number of BSS Descs returned             |
 *     .-------------------------------------------------------------.
 *     |  BSSDescription data (variable, size given in BufSize)      |
 *     .-------------------------------------------------------------.
 *     |  TLV data (variable, size calculated using Header->Size,    |
 *     |            BufSize and sizeof the fixed fields above)       |
 *     .-------------------------------------------------------------.
 *
 *  @param pmpriv       A pointer to mlan_private structure
 *  @param resp         A pointer to HostCmd_DS_COMMAND
 *  @param pioctl_buf   A pointer to mlan_ioctl_req structure
 *
 *  @return             MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status wlan_ret_802_11_scan(mlan_private *pmpriv, HostCmd_DS_COMMAND *resp,
				 t_void *pioctl_buf)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_adapter *pmadapter = pmpriv->adapter;
	mlan_callbacks *pcb = MNULL;
	cmd_ctrl_node *pcmd_node = MNULL;
	HostCmd_DS_802_11_SCAN_RSP *pscan_rsp = MNULL;
	BSSDescriptor_t *bss_new_entry = MNULL;
	MrvlIEtypes_Data_t *ptlv;
	MrvlIEtypes_TsfTimestamp_t *ptsf_tlv = MNULL;
	MrvlIEtypes_ChannelStats_t *pchanstats_tlv = MNULL;
	t_u8 *pbss_info;
	t_u32 scan_resp_size;
	t_u32 bytes_left;
	t_u32 num_in_table;
	t_u32 bss_idx;
	t_u32 idx;
	t_u32 tlv_buf_size = 0;
	t_u64 tsf_val;
	chan_freq_power_t *cfp;
	MrvlIEtypes_ChanBandListParamSet_t *pchan_band_tlv = MNULL;
	ChanBandParamSet_t *pchan_band;
	t_u16 band;
	t_u8 is_bgscan_resp;
	t_u32 age_ts_usec;
	t_u8 null_ssid[MLAN_MAX_SSID_LENGTH] = {0};
	t_u32 status_code = 0;
	pmlan_ioctl_req pscan_ioctl_req = MNULL;

	ENTER();
	pcb = (pmlan_callbacks)&pmadapter->callbacks;

	is_bgscan_resp = (resp->command == HostCmd_CMD_802_11_BG_SCAN_QUERY);
	if (is_bgscan_resp)
		pscan_rsp = &resp->params.bg_scan_query_resp.scan_resp;
	else
		pscan_rsp = &resp->params.scan_resp;

	if (pscan_rsp->number_of_sets > MRVDRV_MAX_BSSID_LIST) {
		PRINTM(MERROR,
		       "SCAN_RESP: Invalid number of AP returned (%d)!!\n",
		       pscan_rsp->number_of_sets);
		status_code = MLAN_ERROR_CMD_SCAN_FAIL;
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	bytes_left = wlan_le16_to_cpu(pscan_rsp->bss_descript_size);
	PRINTM(MINFO, "SCAN_RESP: bss_descript_size %d\n", bytes_left);

	scan_resp_size = resp->size;

	PRINTM(MINFO, "SCAN_RESP: returned %d APs before parsing\n",
	       pscan_rsp->number_of_sets);

	/* Update the age_in_second */
	pmadapter->callbacks.moal_get_system_time(
		pmadapter->pmoal_handle, &pmadapter->age_in_secs, &age_ts_usec);

	num_in_table = pmadapter->num_in_scan_table;
	pbss_info = pscan_rsp->bss_desc_and_tlv_buffer;

	/*
	 * The size of the TLV buffer is equal to the entire command response
	 *   size (scan_resp_size) minus the fixed fields (sizeof()'s), the
	 *   BSS Descriptions (bss_descript_size as bytesLef) and the command
	 *   response header (S_DS_GEN)
	 */
	if (scan_resp_size >
	    (bytes_left + sizeof(pscan_rsp->bss_descript_size) +
	     sizeof(pscan_rsp->number_of_sets) + S_DS_GEN)) {
		tlv_buf_size =
			(scan_resp_size -
			 (bytes_left + sizeof(pscan_rsp->bss_descript_size) +
			  sizeof(pscan_rsp->number_of_sets) + S_DS_GEN));
	} else {
		PRINTM(MERROR,
		       "scan_resp_size: Incorrect size of TLV buff size.\n");
	}

	if (is_bgscan_resp &&
	    (tlv_buf_size >
	     sizeof(resp->params.bg_scan_query_resp.report_condition)))
		tlv_buf_size -= sizeof(
			resp->params.bg_scan_query_resp.report_condition);

	ptlv = (MrvlIEtypes_Data_t *)(pscan_rsp->bss_desc_and_tlv_buffer +
				      bytes_left);

	/*
	 * Search the TLV buffer space in the scan response
	 * for any valid TLVs
	 */
	wlan_ret_802_11_scan_get_tlv_ptrs(pmadapter, ptlv, tlv_buf_size,
					  TLV_TYPE_TSFTIMESTAMP,
					  (MrvlIEtypes_Data_t **)&ptsf_tlv);

	/*
	 * Search the TLV buffer space in the scan response
	 * for any valid TLVs
	 */
	wlan_ret_802_11_scan_get_tlv_ptrs(
		pmadapter, ptlv, tlv_buf_size, TLV_TYPE_CHANNELBANDLIST,
		(MrvlIEtypes_Data_t **)&pchan_band_tlv);
	wlan_ret_802_11_scan_get_tlv_ptrs(
		pmadapter, ptlv, tlv_buf_size, TLV_TYPE_CHANNEL_STATS,
		(MrvlIEtypes_Data_t **)&pchanstats_tlv);

	if (pchanstats_tlv)
		wlan_update_chan_statistics(pmpriv, pchanstats_tlv);

	/*
	 * Process each scan response returned (pscan_rsp->number_of_sets).
	 * Save the information in the bss_new_entry and then insert into
	 * the driver scan table either as an update to an existing entry
	 * or as an addition at the end of the table
	 */
	ret = pcb->moal_malloc(pmadapter->pmoal_handle, sizeof(BSSDescriptor_t),
			       MLAN_MEM_DEF, (t_u8 **)&bss_new_entry);

	if (ret != MLAN_STATUS_SUCCESS || !bss_new_entry) {
		PRINTM(MERROR, "Memory allocation for bss_new_entry failed!\n");
		status_code = MLAN_ERROR_NO_MEM;
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	for (idx = 0; idx < pscan_rsp->number_of_sets && bytes_left; idx++) {
		/* Zero out the bss_new_entry we are about to store info in */
		memset(pmadapter, bss_new_entry, 0x00, sizeof(BSSDescriptor_t));

		/* Process the data fields and IEs returned for this BSS */
		if (wlan_interpret_bss_desc_with_ie(
			    pmadapter, bss_new_entry, &pbss_info, &bytes_left,
			    MFALSE) == MLAN_STATUS_SUCCESS) {
			PRINTM(MINFO, "SCAN_RESP: BSSID = " MACSTR "\n",
			       MAC2STR(bss_new_entry->mac_address));

			band = BAND_G;
			if (pchan_band_tlv) {
				pchan_band =
					&pchan_band_tlv->chan_band_param[idx];
				band = radio_type_to_band(
					pchan_band->bandcfg.chanBand);
				if (!bss_new_entry->channel)
					bss_new_entry->channel =
						pchan_band->chan_number;
			}
			/*
			 * Save the band designation for this entry
			 * for use in join
			 */
			bss_new_entry->bss_band = band;
			bss_new_entry->age_in_secs = pmadapter->age_in_secs;
			cfp = wlan_find_cfp_by_band_and_channel(
				pmadapter, bss_new_entry->bss_band,
				(t_u16)bss_new_entry->channel);
			if (cfp)
				bss_new_entry->freq = cfp->freq;
			else
				bss_new_entry->freq = 0;

			/* Skip entry if on blacklisted channel */
			if (cfp && cfp->dynamic.blacklist) {
				PRINTM(MINFO,
				       "SCAN_RESP: dropping entry on blacklist channel.\n");
				continue;
			}

			/*
			 * Search the scan table for the same bssid
			 */
			for (bss_idx = 0; bss_idx < num_in_table; bss_idx++) {
				if (!memcmp(pmadapter,
					    bss_new_entry->mac_address,
					    pmadapter->pscan_table[bss_idx]
						    .mac_address,
					    sizeof(bss_new_entry->mac_address))) {
					/*
					 * If the SSID matches as well, it is a
					 * duplicate of this entry.  Keep the
					 * bss_idx set to this entry so we
					 * replace the old contents in the table
					 */
					if ((bss_new_entry->ssid.ssid_len ==
					     pmadapter->pscan_table[bss_idx]
						     .ssid.ssid_len) &&
					    (!memcmp(
						    pmadapter,
						    bss_new_entry->ssid.ssid,
						    pmadapter
							    ->pscan_table[bss_idx]
							    .ssid.ssid,
						    bss_new_entry->ssid
							    .ssid_len))) {
						PRINTM(MINFO,
						       "SCAN_RESP: Duplicate of index: %d\n",
						       bss_idx);

						break;
					}
					/*
					 * If the SSID is NULL for same BSSID
					 * keep the bss_idx set to this entry
					 * so we replace the old contents in
					 * the table
					 */
					if (!memcmp(pmadapter,
						    pmadapter
							    ->pscan_table[bss_idx]
							    .ssid.ssid,
						    null_ssid,
						    pmadapter
							    ->pscan_table[bss_idx]
							    .ssid.ssid_len)) {
						PRINTM(MINFO,
						       "SCAN_RESP: Duplicate of index: %d\n",
						       bss_idx);
						break;
					}
				}
			}
			/*
			 * If the bss_idx is equal to the number of entries
			 * in the table, the new entry was not a duplicate;
			 * append it to the scan table
			 */
			if (bss_idx == num_in_table) {
				/*
				 * Range check the bss_idx, keep it limited
				 * to the last entry
				 */
				if (bss_idx == MRVDRV_MAX_BSSID_LIST)
					bss_idx--;
				else if (num_in_table < UINT32_MAX)
					num_in_table++;
				else
					break;
			} else {
				if ((bss_new_entry->channel !=
				     pmadapter->pscan_table[bss_idx].channel) &&
				    (bss_new_entry->rssi >
				     pmadapter->pscan_table[bss_idx].rssi)) {
					PRINTM(MCMND,
					       "skip update the duplicate entry with low rssi\n");
					continue;
				}
			}
			/*
			 * Save the beacon/probe response returned for later
			 * application retrieval. Duplicate beacon/probe
			 * responses are updated if possible
			 */
			wlan_ret_802_11_scan_store_beacon(
				pmpriv, bss_idx, num_in_table, bss_new_entry);
			if (bss_new_entry->pbeacon_buf == MNULL) {
				PRINTM(MCMND,
				       "No space for beacon, drop this entry\n");
				num_in_table--;
				continue;
			}
			/*
			 * If the TSF TLV was appended to the scan results, save
			 * this entry's TSF value in the networkTSF field.  The
			 * networkTSF is the firmware's TSF value at the time
			 * the beacon or probe response was received.
			 */
			if (ptsf_tlv) {
				memcpy_ext(
					pmpriv->adapter, &tsf_val,
					&ptsf_tlv->tsf_data[idx * TSF_DATA_SIZE],
					sizeof(tsf_val), sizeof(tsf_val));
				tsf_val = wlan_le64_to_cpu(tsf_val);
				memcpy_ext(pmpriv->adapter,
					   &bss_new_entry->network_tsf,
					   &tsf_val,
					   sizeof(bss_new_entry->network_tsf),
					   sizeof(bss_new_entry->network_tsf));
			}

			/* Copy the locally created bss_new_entry to the scan
			 * table */
			memcpy_ext(pmadapter, &pmadapter->pscan_table[bss_idx],
				   bss_new_entry,
				   sizeof(pmadapter->pscan_table[bss_idx]),
				   sizeof(pmadapter->pscan_table[bss_idx]));

		} else {
			/* Error parsing/interpreting the scan response, skipped
			 */
			PRINTM(MERROR,
			       "SCAN_RESP: wlan_interpret_bss_desc_with_ie returned error\n");
		}
	}

	PRINTM(MINFO, "SCAN_RESP: Scanned %2d APs, %d valid, %d total\n",
	       pscan_rsp->number_of_sets,
	       num_in_table - pmadapter->num_in_scan_table, num_in_table);

	/* Update the total number of BSSIDs in the scan table */
	pmadapter->num_in_scan_table = num_in_table;
	if (is_bgscan_resp)
		goto done;
	wlan_request_cmd_lock(pmadapter);
	if (!util_peek_list(pmadapter->pmoal_handle, &pmadapter->scan_pending_q,
			    MNULL, MNULL)) {
		wlan_release_cmd_lock(pmadapter);
		if (pmadapter->pscan_ioctl_req) {
			if (((mlan_ds_scan *)pmadapter->pscan_ioctl_req->pbuf)
					    ->sub_command ==
				    MLAN_OID_SCAN_SPECIFIC_SSID ||
			    ((mlan_ds_scan *)pmadapter->pscan_ioctl_req->pbuf)
					    ->sub_command ==
				    MLAN_OID_SCAN_USER_CONFIG) {
				if (wlan_active_scan_req_for_passive_chan(
					    pmpriv,
					    pmadapter->pscan_ioctl_req)) {
					goto done;
				}
			}
		}
		if ((pmadapter->wifi_6g_scan_split) && (!pmadapter->scan_6g)) {
			if (wlan_scan_6g_network(pmpriv,
						 pmadapter->pscan_ioctl_req)) {
				goto done;
			}
		}
		/*
		 * Process the resulting scan table:
		 *   - Remove any bad ssids
		 *   - Update our current BSS information from scan data
		 */
		wlan_scan_process_results(pmpriv);

		wlan_request_cmd_lock(pmadapter);
		pmadapter->scan_processing = MFALSE;
		pscan_ioctl_req = pmadapter->pscan_ioctl_req;
		pmadapter->pscan_ioctl_req = MNULL;
		/* Need to indicate IOCTL complete */
		if (pscan_ioctl_req) {
			pscan_ioctl_req->status_code = MLAN_ERROR_NO_ERROR;
			/* Indicate ioctl complete */
			pcb->moal_ioctl_complete(
				pmadapter->pmoal_handle,
				(pmlan_ioctl_req)pscan_ioctl_req,
				MLAN_STATUS_SUCCESS);
		}
		wlan_release_cmd_lock(pmadapter);
		pmadapter->bgscan_reported = MFALSE;
		wlan_recv_event(pmpriv, MLAN_EVENT_ID_DRV_SCAN_REPORT, MNULL);
	} else {
		/* If firmware not ready, do not issue any more scan commands */
		if (pmadapter->hw_status != WlanHardwareStatusReady) {
			wlan_release_cmd_lock(pmadapter);
			status_code = MLAN_ERROR_FW_NOT_READY;
			ret = MLAN_STATUS_FAILURE;
			goto done;
		} else {
			/* Get scan command from scan_pending_q and put to
			 * cmd_pending_q */
			pcmd_node = (cmd_ctrl_node *)util_dequeue_list(
				pmadapter->pmoal_handle,
				&pmadapter->scan_pending_q, MNULL, MNULL);
			wlan_insert_cmd_to_pending_q(pmadapter, pcmd_node,
						     MTRUE);
			wlan_release_cmd_lock(pmadapter);
		}
	}

done:
	if (bss_new_entry)
		pcb->moal_mfree(pmadapter->pmoal_handle, (t_u8 *)bss_new_entry);
	if (ret) {
		/* Flush all pending scan commands */
		wlan_flush_scan_queue(pmadapter);
		wlan_request_cmd_lock(pmadapter);
		pmadapter->scan_processing = MFALSE;
		pscan_ioctl_req = pmadapter->pscan_ioctl_req;
		pmadapter->pscan_ioctl_req = MNULL;
		if (pscan_ioctl_req) {
			pscan_ioctl_req->status_code = status_code;
			/* Indicate ioctl complete */
			pcb->moal_ioctl_complete(pmadapter->pmoal_handle,
						 pscan_ioctl_req,
						 MLAN_STATUS_FAILURE);
		}
		wlan_release_cmd_lock(pmadapter);
	}
	LEAVE();
	return ret;
}

/**
 *  @brief Get ext_scan state from ext_scan_type
 *
 *
 *  @param pcmd       A pointer to HostCmd_DS_COMMAND structure to be sent to
 *                    firmware with the HostCmd_DS_802_11_SCAN_EXT structure
 *
 *  @return
 * SCAN_STATE_EXT_SCAN_ENH/SCAN_STATE_EXT_SCAN_CANCEL/SCAN_STATE_EXT_SCAN_ENH
 */
t_u8 wlan_get_ext_scan_state(HostCmd_DS_COMMAND *pcmd)
{
	HostCmd_DS_802_11_SCAN_EXT *pext_scan_cmd = &pcmd->params.ext_scan;
	if (pext_scan_cmd->ext_scan_type == EXT_SCAN_ENHANCE)
		return SCAN_STATE_EXT_SCAN_ENH;
	if (pext_scan_cmd->ext_scan_type == EXT_SCAN_CANCEL)
		return SCAN_STATE_EXT_SCAN_CANCEL;
	return SCAN_STATE_EXT_SCAN;
}

/**
 *  @brief Prepare an extended scan command to be sent to the firmware
 *
 *  Use the wlan_scan_cmd_config sent to the command processing module in
 *   the wlan_prepare_cmd to configure a HostCmd_DS_802_11_SCAN_EXT command
 *   struct to send to firmware.
 *
 *  @param pmpriv     A pointer to mlan_private structure
 *  @param pcmd       A pointer to HostCmd_DS_COMMAND structure to be sent to
 *                    firmware with the HostCmd_DS_802_11_SCAN_EXT structure
 *  @param pdata_buf  Void pointer cast of a wlan_scan_cmd_config struct used
 *                    to set the fields/TLVs for the command sent to firmware
 *
 *  @return           MLAN_STATUS_SUCCESS
 */
mlan_status wlan_cmd_802_11_scan_ext(mlan_private *pmpriv,
				     HostCmd_DS_COMMAND *pcmd,
				     t_void *pdata_buf)
{
	HostCmd_DS_802_11_SCAN_EXT *pext_scan_cmd = &pcmd->params.ext_scan;
	wlan_scan_cmd_config *pscan_cfg = MNULL;

	ENTER();

	pcmd->command = wlan_cpu_to_le16(HostCmd_CMD_802_11_SCAN_EXT);

	if (pmpriv->adapter->ext_scan_enh == MTRUE) {
		if (pdata_buf) {
			if (pmpriv->adapter->ext_scan_type == EXT_SCAN_ENHANCE)
				pext_scan_cmd->ext_scan_type = EXT_SCAN_ENHANCE;
			else
				pext_scan_cmd->ext_scan_type = EXT_SCAN_DEFAULT;
		} else {
			pcmd->size = wlan_cpu_to_le16((
				t_u16)(sizeof(pext_scan_cmd->ext_scan_type) +
				       (t_u16)(sizeof(pext_scan_cmd->reserved)) +
				       S_DS_GEN));
			pext_scan_cmd->ext_scan_type = EXT_SCAN_CANCEL;
			LEAVE();
			return MLAN_STATUS_SUCCESS;
		}
	}
	if (!pdata_buf) {
		PRINTM(MERROR, "wlan_cmd_802_11_scan_ext: pdata_buf is null\n");
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}
	pscan_cfg = (wlan_scan_cmd_config *)pdata_buf;

	memcpy_ext(pmpriv->adapter, pext_scan_cmd->tlv_buffer,
		   pscan_cfg->tlv_buf, pscan_cfg->tlv_buf_len,
		   pscan_cfg->tlv_buf_len);

	/* Size is equal to the sizeof(fixed portions) + the TLV len + header */
	pcmd->size = wlan_cpu_to_le16(
		(t_u16)(sizeof(pext_scan_cmd->ext_scan_type) +
			(t_u16)sizeof(pext_scan_cmd->reserved) +
			pscan_cfg->tlv_buf_len + S_DS_GEN));

	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function handles the command response of extended scan
 *
 *  @param pmpriv       A pointer to mlan_private structure
 *  @param resp         A pointer to HostCmd_DS_COMMAND
 *  @param pioctl_buf   A pointer to mlan_ioctl_req structure
 *
 *  @return             MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status wlan_ret_802_11_scan_ext(mlan_private *pmpriv,
				     HostCmd_DS_COMMAND *resp,
				     t_void *pioctl_buf)
{
	HostCmd_DS_802_11_SCAN_EXT *pext_scan_cmd = &(resp->params.ext_scan);
	MrvlIEtypesHeader_t *tlv = MNULL;
	MrvlIEtypes_ChannelStats_t *tlv_chanstats = MNULL;
	t_u16 tlv_buf_left = 0;
	t_u16 tlv_type = 0;
	t_u16 tlv_len = 0;
	t_u32 ext_scan_type;
	mlan_callbacks *pcb = (mlan_callbacks *)&pmpriv->adapter->callbacks;
	pmlan_ioctl_req pioctl_req = (pmlan_ioctl_req)pioctl_buf;
	mlan_adapter *pmadapter = pmpriv->adapter;
	ENTER();

	PRINTM(MINFO, "EXT scan returns successfully\n");
	pmadapter->scan_state |= wlan_get_ext_scan_state(resp);
	ext_scan_type = pext_scan_cmd->ext_scan_type;
	if (ext_scan_type == EXT_SCAN_CANCEL) {
		PRINTM(MCMND, "Cancel scan command completed!\n");
		wlan_request_cmd_lock(pmadapter);
		pmadapter->scan_processing = MFALSE;
		pmadapter->scan_state |= SCAN_STATE_SCAN_COMPLETE;
		pmadapter->ext_scan_type = EXT_SCAN_DEFAULT;
		wlan_release_cmd_lock(pmadapter);
		/* Need to indicate IOCTL complete */
		if (pioctl_req != MNULL) {
			pioctl_req->status_code = MLAN_STATUS_SUCCESS;
			/* Indicate ioctl complete */
			pcb->moal_ioctl_complete(pmadapter->pmoal_handle,
						 (pmlan_ioctl_req)pioctl_req,
						 MLAN_STATUS_SUCCESS);
		}
		LEAVE();
		return MLAN_STATUS_SUCCESS;
	} else if (ext_scan_type == EXT_SCAN_ENHANCE) {
		/* Setup the timer after scan command response */
		pcb->moal_start_timer(pmpriv->adapter->pmoal_handle,
				      pmpriv->adapter->pmlan_cmd_timer, MFALSE,
				      MRVDRV_TIMER_10S * 2);
		pmpriv->adapter->cmd_timer_is_set = MTRUE;
		LEAVE();
		return MLAN_STATUS_SUCCESS;
	}
	tlv = (MrvlIEtypesHeader_t *)pext_scan_cmd->tlv_buffer;
	tlv_buf_left = resp->size -
		       (sizeof(HostCmd_DS_802_11_SCAN_EXT) - 1 + S_DS_GEN);
	while (tlv_buf_left >= sizeof(MrvlIEtypesHeader_t)) {
		tlv_type = wlan_le16_to_cpu(tlv->type);
		tlv_len = wlan_le16_to_cpu(tlv->len);
		if (tlv_buf_left < (tlv_len + sizeof(MrvlIEtypesHeader_t))) {
			PRINTM(MERROR, "Error processing scan gap TLV\n");
			break;
		}
		switch (tlv_type) {
		case TLV_TYPE_CHANNEL_STATS:
			tlv_chanstats = (MrvlIEtypes_ChannelStats_t *)tlv;
			wlan_update_chan_statistics(pmpriv, tlv_chanstats);
			break;
		default:
			break;
		}
		tlv_buf_left -= tlv_len + sizeof(MrvlIEtypesHeader_t);
		tlv = (MrvlIEtypesHeader_t *)((t_u8 *)tlv + tlv_len +
					      sizeof(MrvlIEtypesHeader_t));
	}
	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function add a new entry to scan_table
 *
 *  @param pmpriv           A pointer to mlan_private structure
 *  @param bss_new_entry    A pointer to the bss_new_entry
 *  @param num_in_tbl		number of scan entry in scan_table
 *
 *  @return             N/A
 */
static t_void wlan_add_new_entry_to_scan_table(mlan_private *pmpriv,
					       BSSDescriptor_t *bss_new_entry,
					       t_u32 *num_in_tbl)
{
	mlan_adapter *pmadapter = pmpriv->adapter;
	t_u32 bss_idx;
	t_u32 num_in_table = *num_in_tbl;
	t_u8 null_ssid[MLAN_MAX_SSID_LENGTH] = {0};

	/*
	 * Search the scan table for the same bssid
	 */
	for (bss_idx = 0; bss_idx < num_in_table; bss_idx++) {
		if (!memcmp(pmadapter, bss_new_entry->mac_address,
			    pmadapter->pscan_table[bss_idx].mac_address,
			    sizeof(bss_new_entry->mac_address))) {
			/*
			 * If the SSID matches as well, it is a
			 * duplicate of this entry.  Keep the
			 * bss_idx set to this entry so we
			 * replace the old contents in the table
			 */
			if ((bss_new_entry->ssid.ssid_len ==
			     pmadapter->pscan_table[bss_idx].ssid.ssid_len) &&
			    (!memcmp(pmadapter, bss_new_entry->ssid.ssid,
				     pmadapter->pscan_table[bss_idx].ssid.ssid,
				     bss_new_entry->ssid.ssid_len))) {
				PRINTM(MINFO,
				       "EXT_SCAN: Duplicate of index: %d\n",
				       bss_idx);
				break;
			}
			/*
			 * If the SSID is NULL for same BSSID
			 * keep the bss_idx set to this entry
			 * so we replace the old contents in
			 * the table
			 */
			if (!memcmp(pmadapter,
				    pmadapter->pscan_table[bss_idx].ssid.ssid,
				    null_ssid,
				    pmadapter->pscan_table[bss_idx]
					    .ssid.ssid_len)) {
				PRINTM(MINFO,
				       "EXT_SCAN: Duplicate of index: %d\n",
				       bss_idx);
				break;
			}
		}
	}
	/* If the bss_idx is equal to the number of entries
	 * in the table, the new entry was not a duplicate;
	 * append it to the scan table
	 */
	if (bss_idx == num_in_table) {
		/* Range check the bss_idx, keep it limited to the last entry */
		if (bss_idx == MRVDRV_MAX_BSSID_LIST)
			bss_idx--;
		else
			num_in_table++;
	} else {
		if ((bss_new_entry->channel !=
		     pmadapter->pscan_table[bss_idx].channel) &&
		    (bss_new_entry->rssi >
		     pmadapter->pscan_table[bss_idx].rssi)) {
			PRINTM(MCMND,
			       "skip update the duplicate entry with low rssi\n");
			return;
		}
	}
	/*
	 * Save the beacon/probe response returned for later
	 * application retrieval. Duplicate beacon/probe
	 * responses are updated if possible
	 */
	wlan_ret_802_11_scan_store_beacon(pmpriv, bss_idx, num_in_table,
					  bss_new_entry);
	if (bss_new_entry->pbeacon_buf == MNULL) {
		PRINTM(MCMND, "No space for beacon, drop this entry\n");
		num_in_table--;
		goto done;
	} else {
		/* Copy the locally created bss_new_entry to the scan table */
		memcpy_ext(pmadapter, &pmadapter->pscan_table[bss_idx],
			   bss_new_entry,
			   sizeof(pmadapter->pscan_table[bss_idx]),
			   sizeof(pmadapter->pscan_table[bss_idx]));
	}
done:
	*num_in_tbl = num_in_table;
	return;
}

/** 8 bytes timestamp, 2 bytest interval, 2 bytes capability */
#define BEACON_FIX_SIZE 12

/* Element iteration helpers */
#define for_each_element(_elem, _data, _datalen)                               \
	for (_elem = (IEEEtypes_Element_t *)(_data);                           \
	     (const t_u8 *)(_data) + (_datalen) - (const t_u8 *)_elem >=       \
		     (int)sizeof(*_elem) &&                                    \
	     (const t_u8 *)(_data) + (_datalen) - (const t_u8 *)_elem >=       \
		     (int)sizeof(*_elem) + _elem->ieee_hdr.len;                \
	     _elem = (IEEEtypes_Element_t *)(_elem->data +                     \
					     _elem->ieee_hdr.len))

#define for_each_element_id(element, _id, data, datalen)                       \
	for_each_element (element, data, datalen)                              \
		if (element->ieee_hdr.element_id == (_id))

/**
 *  @brief This function updates the NonTx BSS Descriptor entry before adding
 *         it to the scan table
 *  @param pmadapter	A pointer to mlan adapter
 *  @param pbss_entry	A pointer to the parent BSS descriptor entry
 *  @param pnew_entry   A pointer to the nonTx BSS descriptor entry
 *  @param pbeacon_info A pointer to the beacon buffer of NonTx BSS descriptor
 *  @param ie_len		NonTx beacon buffer length
 */
static mlan_status wlan_update_nonTx_bss_desc(mlan_adapter *pmadapter,
					      BSSDescriptor_t *pbss_entry,
					      BSSDescriptor_t *pnew_entry,
					      t_u8 **pbeacon_info, t_u32 ie_len)
{
	mlan_callbacks *pcb = (pmlan_callbacks)&pmadapter->callbacks;
	IEEEtypes_ElementId_e element_id;
	t_u8 *pcurrent_ptr = MNULL;
	t_u8 *pbuf = MNULL;
	t_u8 *prate = MNULL;
	t_u8 element_len = 0;
	t_u8 bytes_to_copy = 0;
	t_u8 rate_size = 0;
	t_u8 found_data_rate_ie = 0;
	t_u16 total_ie_len = 0;
	t_u32 beacon_buf_size = BEACON_FIX_SIZE + ie_len;
	t_u32 bytes_left = 0;
	t_s64 offset = 0;
	mlan_status ret = MLAN_STATUS_FAILURE;
	IEEEtypes_VendorSpecific_t *pvendor_ie;
	const t_u8 wpa_oui[4] = {0x00, 0x50, 0xf2, 0x01};
	const t_u8 wmm_oui[4] = {0x00, 0x50, 0xf2, 0x02};
	const t_u8 owe_oui[4] = {0x50, 0x6f, 0x9a, 0x1c};
	const t_u8 osen_oui[] = {0x50, 0x6f, 0x9a, 0x12};
	IEEEtypes_CountryInfoSet_t *pcountry_info;
	IEEEtypes_Extension_t *pext_tlv;

	ENTER();
	found_data_rate_ie = MFALSE;
	rate_size = 0;

	/* Allocate the beacon buffer for new entry */
	// coverity[overflow_sink:SUPPRESS]
	ret = pcb->moal_malloc(pmadapter->pmoal_handle, beacon_buf_size,
			       MLAN_MEM_DEF, (t_u8 **)&pbuf);
	if (ret != MLAN_STATUS_SUCCESS || !pbuf) {
		ret = MLAN_STATUS_FAILURE;
		PRINTM(MERROR, "Memory allocation for beacon buf failed!\n");
		goto done;
	}

	pnew_entry->beacon_buf_size = beacon_buf_size;
	pnew_entry->pbeacon_buf = pbuf;

	/** Copy fixed IE to beacon buffer */
	memcpy_ext(pmadapter, pbuf, pbss_entry->pbeacon_buf, BEACON_FIX_SIZE,
		   BEACON_FIX_SIZE);

	/** Initialize the current working beacon pointer */
	pcurrent_ptr = *pbeacon_info;

	/** Copy variable IEs to beacon buffer */
	memcpy_ext(pmadapter, pbuf + BEACON_FIX_SIZE, pcurrent_ptr, ie_len,
		   ie_len);

	pcurrent_ptr = pbuf + BEACON_FIX_SIZE;
	bytes_left = ie_len;

	/* Adjust the IE pointers and offsets */
	while (bytes_left >= 2) {
		element_id = (IEEEtypes_ElementId_e)(*((t_u8 *)pcurrent_ptr));
		element_len = *((t_u8 *)pcurrent_ptr + 1);
		total_ie_len = element_len + sizeof(IEEEtypes_Header_t);

		switch (element_id) {
		case SSID:
			// coverity[bad_memset:SUPPRESS]
			memset(pmadapter, (t_u8 *)&pnew_entry->ssid.ssid, 0,
			       sizeof(mlan_802_11_ssid));
			pnew_entry->ssid.ssid_len = element_len;
			memcpy_ext(pmadapter, pnew_entry->ssid.ssid,
				   (pcurrent_ptr + 2), element_len,
				   element_len);
			PRINTM(MMSG, "SSID: %-32s\n", pnew_entry->ssid.ssid);
			break;

		case SUPPORTED_RATES:
			memcpy_ext(pmadapter, pnew_entry->data_rates,
				   pcurrent_ptr + 2, element_len,
				   sizeof(pnew_entry->data_rates));
			memcpy_ext(pmadapter, pnew_entry->supported_rates,
				   pcurrent_ptr + 2, element_len,
				   sizeof(pnew_entry->supported_rates));
			DBG_HEXDUMP(MINFO, "SupportedRates:",
				    pnew_entry->supported_rates, element_len);
			rate_size = element_len;
			found_data_rate_ie = MTRUE;
			break;
		/* Handle Country Info IE */
		case COUNTRY_INFO:
			pcountry_info =
				(IEEEtypes_CountryInfoSet_t *)pcurrent_ptr;
			memcpy_ext(pmadapter, &pnew_entry->country_info,
				   pcountry_info, pcountry_info->len + 2,
				   sizeof(pnew_entry->country_info));
			DBG_HEXDUMP(MINFO,
				    "CountryInfo:", pnew_entry->country_info,
				    element_len + 2);
			break;
		case POWER_CONSTRAINT:
		case POWER_CAPABILITY:
		case TPC_REPORT:
		case CHANNEL_SWITCH_ANN:
		case QUIET:
		case SUPPORTED_CHANNELS:
		case TPC_REQUEST:
			wlan_11h_process_bss_elem(
				pmadapter, &pnew_entry->wlan_11h_bss_info,
				pcurrent_ptr);
			break;
		case EXTENDED_SUPPORTED_RATES:
			/* Only process extended supported rate
			 * if data rate is already found.
			 * Data rate IE should come before
			 * extended supported rate IE
			 */
			if (found_data_rate_ie) {
				if ((element_len + rate_size) >
				    WLAN_SUPPORTED_RATES) {
					bytes_to_copy = (WLAN_SUPPORTED_RATES -
							 rate_size);
				} else {
					bytes_to_copy = element_len;
				}

				prate = (t_u8 *)pnew_entry->data_rates;
				prate += rate_size;
				memcpy_ext(pmadapter, prate, pcurrent_ptr + 2,
					   bytes_to_copy, bytes_to_copy);

				prate = (t_u8 *)pnew_entry->supported_rates;
				prate += rate_size;
				memcpy_ext(pmadapter, prate, pcurrent_ptr + 2,
					   bytes_to_copy, bytes_to_copy);
			}
			DBG_HEXDUMP(MINFO, "Ext SupportedRates:",
				    pnew_entry->supported_rates,
				    element_len + rate_size);
			break;

		case VENDOR_SPECIFIC_221:
			pvendor_ie = (IEEEtypes_VendorSpecific_t *)pcurrent_ptr;

			if (!memcmp(pmadapter, pvendor_ie->vend_hdr.oui,
				    wpa_oui, sizeof(wpa_oui))) {
				pnew_entry->pwpa_ie =
					(IEEEtypes_VendorSpecific_t *)
						pcurrent_ptr;
				offset = pcurrent_ptr - pnew_entry->pbeacon_buf;
				pnew_entry->wpa_offset = (t_u16)(offset);

				DBG_HEXDUMP(
					MINFO,
					"WPA_IE:", (t_u8 *)pnew_entry->pwpa_ie,
					((*(pnew_entry->pwpa_ie)).vend_hdr.len +
					 sizeof(IEEEtypes_Header_t)));
			} else if (!memcmp(pmadapter, pvendor_ie->vend_hdr.oui,
					   wmm_oui, sizeof(wmm_oui))) {
				if (total_ie_len ==
					    sizeof(IEEEtypes_WmmParameter_t) ||
				    total_ie_len ==
					    sizeof(IEEEtypes_WmmInfo_t)) {
					/* Only accept and copy the WMM IE if
					 * it matches the size expected for the
					 * WMM Info IE or the WMM Parameter IE
					 */
					memcpy_ext(pmadapter,
						   (t_u8 *)&pnew_entry->wmm_ie,
						   pcurrent_ptr, total_ie_len,
						   sizeof(pnew_entry->wmm_ie));
					DBG_HEXDUMP(MINFO, "WMM_IE:",
						    (t_u8 *)&pnew_entry->wmm_ie,
						    total_ie_len);
				}
			} else if (IS_FW_SUPPORT_EMBEDDED_OWE(pmadapter) &&
				   !memcmp(pmadapter, pvendor_ie->vend_hdr.oui,
					   owe_oui, sizeof(owe_oui))) {
				/* Current Format of OWE IE is
				 * element_id:element_len:oui:MAC Address:SSID
				 * length:SSID */
				t_u8 trans_ssid_len = *(
					pcurrent_ptr +
					sizeof(IEEEtypes_Header_t) +
					sizeof(owe_oui) + MLAN_MAC_ADDR_LENGTH);

				if (!trans_ssid_len ||
				    trans_ssid_len > MRVDRV_MAX_SSID_LENGTH) {
					bytes_left = 0;
					continue;
				}

				if (!pnew_entry->cap_info.privacy)
					pnew_entry->owe_transition_mode =
						OWE_TRANS_MODE_OPEN;
				else
					pnew_entry->owe_transition_mode =
						OWE_TRANS_MODE_OWE;

				memcpy_ext(
					pmadapter,
					pnew_entry->trans_mac_address,
					(pcurrent_ptr +
					 sizeof(IEEEtypes_Header_t) +
					 sizeof(owe_oui)),
					MLAN_MAC_ADDR_LENGTH,
					sizeof(pnew_entry->trans_mac_address));

				pnew_entry->trans_ssid.ssid_len =
					trans_ssid_len;

				memcpy_ext(
					pmadapter, pnew_entry->trans_ssid.ssid,
					(pcurrent_ptr +
					 sizeof(IEEEtypes_Header_t) +
					 sizeof(owe_oui) +
					 MLAN_MAC_ADDR_LENGTH + sizeof(t_u8)),
					trans_ssid_len,
					sizeof(pnew_entry->trans_ssid.ssid));

				PRINTM(MINFO,
				       "OWE Transition AP privacy=%d MAC Addr-" MACSTR
				       " ssid %s\n",
				       pnew_entry->owe_transition_mode,
				       MAC2STR(pnew_entry->trans_mac_address),
				       pnew_entry->trans_ssid.ssid);
			} else if (!memcmp(pmadapter, pvendor_ie->vend_hdr.oui,
					   osen_oui, sizeof(osen_oui))) {
				pnew_entry->posen_ie =
					(IEEEtypes_Generic_t *)pcurrent_ptr;
				pnew_entry->osen_offset = (t_u16)((
					t_s16)(pcurrent_ptr -
					       pnew_entry->pbeacon_buf));

				DBG_HEXDUMP(
					MINFO, "OSEN_IE:",
					(t_u8 *)pnew_entry->posen_ie,
					(*(pnew_entry->posen_ie)).ieee_hdr.len +
						sizeof(IEEEtypes_Header_t));
			}
			break;
		case RSN_IE:
			pnew_entry->prsn_ie =
				(IEEEtypes_Generic_t *)pcurrent_ptr;
			pnew_entry->rsn_offset = (t_u16)((
				t_s16)(pcurrent_ptr - pnew_entry->pbeacon_buf));
			DBG_HEXDUMP(MINFO,
				    "RSN_IE:", (t_u8 *)pnew_entry->prsn_ie,
				    (*(pnew_entry->prsn_ie)).ieee_hdr.len +
					    sizeof(IEEEtypes_Header_t));
			break;
		case RSNX_IE:
			pnew_entry->prsnx_ie =
				(IEEEtypes_Generic_t *)pcurrent_ptr;
			pnew_entry->rsnx_offset = (t_u16)((
				t_s16)(pcurrent_ptr - pnew_entry->pbeacon_buf));
			DBG_HEXDUMP(MINFO,
				    "RSNX_IE:", (t_u8 *)pnew_entry->prsnx_ie,
				    (*(pnew_entry->prsnx_ie)).ieee_hdr.len +
					    sizeof(IEEEtypes_Header_t));
			break;
		case WAPI_IE:
			pnew_entry->pwapi_ie =
				(IEEEtypes_Generic_t *)pcurrent_ptr;
			pnew_entry->wapi_offset = (t_u16)((
				t_s16)(pcurrent_ptr - pnew_entry->pbeacon_buf));
			DBG_HEXDUMP(MINFO,
				    "WAPI_IE:", (t_u8 *)pnew_entry->pwapi_ie,
				    (*(pnew_entry->pwapi_ie)).ieee_hdr.len +
					    sizeof(IEEEtypes_Header_t));
			break;
		case HT_CAPABILITY:
			pnew_entry->pht_cap = (IEEEtypes_HTCap_t *)pcurrent_ptr;
			pnew_entry->ht_cap_offset = (t_u16)((
				t_s16)(pcurrent_ptr - pnew_entry->pbeacon_buf));
			DBG_HEXDUMP(MINFO,
				    "HTCAP_IE:", (t_u8 *)pnew_entry->pht_cap,
				    (*(pnew_entry->pht_cap)).ieee_hdr.len +
					    sizeof(IEEEtypes_Header_t));
			break;

		case HT_OPERATION:
			pnew_entry->pht_info =
				(IEEEtypes_HTInfo_t *)pcurrent_ptr;
			pnew_entry->ht_info_offset = (t_u16)((
				t_s16)(pcurrent_ptr - pnew_entry->pbeacon_buf));
			DBG_HEXDUMP(MINFO,
				    "HTOPER_IE:", (t_u8 *)pnew_entry->pht_info,
				    (*(pnew_entry->pht_info)).ieee_hdr.len +
					    sizeof(IEEEtypes_Header_t));
			break;

		case BSSCO_2040:
			pnew_entry->pbss_co_2040 =
				(IEEEtypes_2040BSSCo_t *)pcurrent_ptr;
			pnew_entry->bss_co_2040_offset = (t_u16)((
				t_s16)(pcurrent_ptr - pnew_entry->pbeacon_buf));
			DBG_HEXDUMP(MINFO, "2040BSSCOEX_IE:",
				    (t_u8 *)pnew_entry->pbss_co_2040,
				    (*(pnew_entry->pbss_co_2040)).ieee_hdr.len +
					    sizeof(IEEEtypes_Header_t));
			break;

		case EXT_CAPABILITY:
			pnew_entry->pext_cap =
				(IEEEtypes_ExtCap_t *)pcurrent_ptr;
			pnew_entry->ext_cap_offset = (t_u16)((
				t_s16)(pcurrent_ptr - pnew_entry->pbeacon_buf));
			DBG_HEXDUMP(MINFO,
				    "EXTCAP_IE:", (t_u8 *)pnew_entry->pext_cap,
				    (*(pnew_entry->pext_cap)).ieee_hdr.len +
					    sizeof(IEEEtypes_Header_t));
			break;

		case OVERLAPBSSSCANPARAM:
			pnew_entry->poverlap_bss_scan_param =
				(IEEEtypes_OverlapBSSScanParam_t *)pcurrent_ptr;
			pnew_entry->overlap_bss_offset = (t_u16)((
				t_s16)(pcurrent_ptr - pnew_entry->pbeacon_buf));
			DBG_HEXDUMP(MINFO, "OBSS_IE",
				    (t_u8 *)pnew_entry->poverlap_bss_scan_param,
				    (*(pnew_entry->poverlap_bss_scan_param))
						    .ieee_hdr.len +
					    sizeof(IEEEtypes_Header_t));
			break;
		case VHT_CAPABILITY:
			pnew_entry->pvht_cap =
				(IEEEtypes_VHTCap_t *)pcurrent_ptr;
			pnew_entry->vht_cap_offset = (t_u16)((
				t_s16)(pcurrent_ptr - pnew_entry->pbeacon_buf));
			DBG_HEXDUMP(MINFO,
				    "VHTCAP_IE:", (t_u8 *)pnew_entry->pvht_cap,
				    (*(pnew_entry->pvht_cap)).ieee_hdr.len +
					    sizeof(IEEEtypes_Header_t));
			break;

		case VHT_OPERATION:
			pnew_entry->pvht_oprat =
				(IEEEtypes_VHTOprat_t *)pcurrent_ptr;
			pnew_entry->vht_oprat_offset = (t_u16)((
				t_s16)(pcurrent_ptr - pnew_entry->pbeacon_buf));
			DBG_HEXDUMP(MINFO, "VHTOPER_IE:",
				    (t_u8 *)pnew_entry->pvht_oprat,
				    (*(pnew_entry->pvht_oprat)).ieee_hdr.len +
					    sizeof(IEEEtypes_Header_t));
			break;

		case EXT_BSS_LOAD:
			pnew_entry->pext_bssload =
				(IEEEtypes_ExtBSSload_t *)pcurrent_ptr;
			pnew_entry->ext_bssload_offset = (t_u16)((
				t_s16)(pcurrent_ptr - pnew_entry->pbeacon_buf));
			DBG_HEXDUMP(MINFO, "EXTBSSLOAD_IE",
				    (t_u8 *)pnew_entry->pext_bssload,
				    (*(pnew_entry->pext_bssload)).ieee_hdr.len +
					    sizeof(IEEEtypes_Header_t));
			break;

		case VHT_TX_POWER_ENV:
			pnew_entry->pvht_txpower =
				(IEEEtypes_VHTtxpower_t *)pcurrent_ptr;
			pnew_entry->vht_txpower_offset = (t_u16)((
				t_s16)(pcurrent_ptr - pnew_entry->pbeacon_buf));
			DBG_HEXDUMP(MINFO, "TXPOW_IE:",
				    (t_u8 *)pnew_entry->pvht_txpower,
				    (*(pnew_entry->pvht_txpower)).ieee_hdr.len +
					    sizeof(IEEEtypes_Header_t));
			break;

		case EXT_POWER_CONSTR:
			pnew_entry->pext_pwer =
				(IEEEtypes_ExtPwerCons_t *)pcurrent_ptr;
			pnew_entry->ext_pwer_offset = (t_u16)((
				t_s16)(pcurrent_ptr - pnew_entry->pbeacon_buf));
			DBG_HEXDUMP(MINFO, "EXTPOW_IE",
				    (t_u8 *)pnew_entry->pext_pwer,
				    (*(pnew_entry->pext_pwer)).ieee_hdr.len +
					    sizeof(IEEEtypes_Header_t));
			break;

		case QUIET_CHAN:
			pnew_entry->pquiet_chan =
				(IEEEtypes_QuietChan_t *)pcurrent_ptr;
			offset = pcurrent_ptr - pnew_entry->pbeacon_buf;
			pnew_entry->quiet_chan_offset = (t_u16)(offset);
			DBG_HEXDUMP(MINFO, "QUIETCHAN_IE",
				    (t_u8 *)pnew_entry->pquiet_chan,
				    (*(pnew_entry->pquiet_chan)).ieee_hdr.len +
					    sizeof(IEEEtypes_Header_t));
			break;

		case OPER_MODE_NTF:
			pnew_entry->poper_mode =
				(IEEEtypes_OperModeNtf_t *)pcurrent_ptr;
			offset = pcurrent_ptr - pnew_entry->pbeacon_buf;
			pnew_entry->oper_mode_offset = (t_u16)(offset);
			DBG_HEXDUMP(MINFO,
				    "OMN_IE:", (t_u8 *)pnew_entry->poper_mode,
				    (*(pnew_entry->poper_mode)).ieee_hdr.len +
					    sizeof(IEEEtypes_Header_t));
			break;
		case EXTENSION:
			pext_tlv = (IEEEtypes_Extension_t *)pcurrent_ptr;
			switch (pext_tlv->ext_id) {
			case HE_CAPABILITY:
				pnew_entry->phe_cap =
					(IEEEtypes_HECap_t *)pcurrent_ptr;
				offset = pcurrent_ptr - pnew_entry->pbeacon_buf;
				pnew_entry->he_cap_offset = (t_u16)(offset);
				DBG_HEXDUMP(
					MINFO, "HECAP_IE:",
					(t_u8 *)pnew_entry->phe_cap,
					(*(pnew_entry->phe_cap)).ieee_hdr.len +
						sizeof(IEEEtypes_Header_t));
				break;
			case HE_OPERATION:
				pnew_entry->phe_oprat = pext_tlv;
				offset = pcurrent_ptr - pnew_entry->pbeacon_buf;
				pnew_entry->he_oprat_offset = (t_u16)(offset);
				DBG_HEXDUMP(
					MINFO, "HEOPER_IE:",
					(t_u8 *)pnew_entry->phe_oprat,
					(*(pnew_entry->phe_oprat)).ieee_hdr.len +
						sizeof(IEEEtypes_Header_t));
				break;
			case MU_EDCA_PARAM_SET:
				pnew_entry->pmuedca_ie =
					(IEEEtypes_MUEDCAParamSet_t *)
						pcurrent_ptr;
				offset = pcurrent_ptr - pnew_entry->pbeacon_buf;
				pnew_entry->muedca_offset = (t_u16)(offset);
				DBG_HEXDUMP(MINFO, "MUEDCA_IE:",
					    (t_u8 *)pnew_entry->pmuedca_ie,
					    (*(pnew_entry->pmuedca_ie))
							    .ieee_hdr.len +
						    sizeof(IEEEtypes_Header_t));
				break;
			case MBSSID_CONFIG:
				pnew_entry->pmbssid_config =
					(IEEEtypes_MBSSID_Config_t *)
						pcurrent_ptr;
				offset = pcurrent_ptr - pnew_entry->pbeacon_buf;
				pnew_entry->mbssid_config_offset =
					(t_u16)(offset);
				DBG_HEXDUMP(MINFO, "MBSSID_CONFIG_IE:",
					    (t_u8 *)pnew_entry->pmbssid_config,
					    (*(pnew_entry->pmbssid_config))
							    .ieee_hdr.len +
						    sizeof(IEEEtypes_Header_t));
				break;
			case HE_6G_CAPABILITY:
				pnew_entry->phe_6g_cap =
					(IEEEtypes_HE6GCap_t *)pcurrent_ptr;
				offset = pcurrent_ptr - pnew_entry->pbeacon_buf;
				pnew_entry->he_6g_cap_offset = (t_u16)(offset);
				DBG_HEXDUMP(MINFO, "6GCAP_IE:",
					    (t_u8 *)pnew_entry->phe_6g_cap,
					    (*(pnew_entry->phe_6g_cap))
							    .ieee_hdr.len +
						    sizeof(IEEEtypes_Header_t));
				break;
			default:
				break;
			}
			break;
		case MOBILITY_DOMAIN:
			pnew_entry->pmd_ie =
				(IEEEtypes_MobilityDomain_t *)pcurrent_ptr;
			offset = pcurrent_ptr - pnew_entry->pbeacon_buf;
			pnew_entry->md_offset = (t_u16)(offset);
			DBG_HEXDUMP(MINFO, "Mobility Domain IE",
				    (t_u8 *)pnew_entry->pmd_ie,
				    (*(pnew_entry->pmd_ie)).ieee_hdr.len +
					    sizeof(IEEEtypes_Header_t));
			break;
		case RNR:
			pnew_entry->prnr_ie =
				(IEEEtypes_Generic_t *)pcurrent_ptr;
			offset = pcurrent_ptr - pnew_entry->pbeacon_buf;
			pnew_entry->rnr_offset = (t_u16)(offset);
			DBG_HEXDUMP(MINFO, "RNR IE",
				    (t_u8 *)pnew_entry->prnr_ie,
				    (*(pnew_entry->prnr_ie)).ieee_hdr.len +
					    sizeof(IEEEtypes_Header_t));
			break;
		default:
			break;
		}

		pcurrent_ptr += element_len + 2;
		bytes_left -= (element_len + 2);
	}
done:
	LEAVE();
	return ret;
}

/**
 *  @brief This function returns a pointer to IE with matching element ID
 *
 *  @param pmadpater		A pointer to mlan_adapter structure
 *  @param eid   			A pointer to element ID to search for
 *	@param subie			NonTx BSSID buffer from which to search
 *IE
 *	@param subie_len		NonTx BSSID buffer len
 *	@param match			Pointer to IE to be matched
 *	@param match_len		1 for EXT_ID, 0 for Non_Ext ID
 *	@param match_offset		IE offset to be matched
 *  @return elem            Returns pointer to the matched element
 */
static IEEEtypes_Element_t *wlan_find_elem_match(mlan_adapter *pmadapter,
						 t_u8 eid, t_u8 *subie,
						 t_u32 subie_len, t_u8 *match,
						 t_u32 match_len,
						 t_u32 match_offset)
{
	IEEEtypes_Element_t *elem;

	for_each_element_id (elem, eid, subie, subie_len) {
		if (elem->ieee_hdr.len >= match_offset + match_len &&
		    !memcmp(pmadapter, elem->data + match_offset, match,
			    match_len))
			return elem;
	}
	return MNULL;
}

/**
 *  @brief This function generate the bssid from bssid_idx
 *
 *  @param pmadpater        A pointer to mlan_adapter structure
 *  @param pbss_entry       A pointer to the bss_entry which has multi-bssid IE
 *  @param pnew_entry       A pinter to new entry
 *  @param bssid_index      bssid_index from BSSID_IDX IE
 *
 *  @return                N/A
 */
static void wlan_gen_multi_bssid_by_bssid_index(pmlan_adapter pmadapter,
						BSSDescriptor_t *pbss_entry,
						BSSDescriptor_t *pnew_entry,
						t_u8 bssid_index,
						t_u8 max_bssid_indicator)
{
	t_u8 mask = 0xff;
	t_u8 new_bssid[6];
	t_u8 bssid_a;
	t_u8 src_bssid[6];

	memcpy_ext(pmadapter, (t_u8 *)src_bssid, pbss_entry->mac_address,
		   sizeof(mlan_802_11_mac_addr), sizeof(src_bssid));
	memcpy_ext(pmadapter, (t_u8 *)new_bssid,
		   (t_u8 *)&pbss_entry->mac_address,
		   sizeof(mlan_802_11_mac_addr), sizeof(new_bssid));

	mask = (mask >> (8 - max_bssid_indicator));
	bssid_a = src_bssid[5] & (~mask);
	src_bssid[5] = (src_bssid[5] + bssid_index) & mask;
	new_bssid[5] = bssid_a | src_bssid[5];

	memcpy_ext(pmadapter, (t_u8 *)&pnew_entry->mac_address, new_bssid,
		   sizeof(new_bssid), sizeof(mlan_802_11_mac_addr));
	memcpy_ext(pmadapter, (t_u8 *)&pnew_entry->multi_bssid_ap_addr,
		   (t_u8 *)&pbss_entry->mac_address,
		   sizeof(mlan_802_11_mac_addr), sizeof(mlan_802_11_mac_addr));
}

/**
 *  @brief This function checks if the given IE is inherited
 *
 *  @param elem				The element to check
 *  @param non_inherit_ie   A pointer
 *
 *  @return                 TRUE -  if element is inherited from TxBSSID
 *                          FALSE - IF element in NOT inherited
 */
static t_bool wlan_is_element_inherited(IEEEtypes_Element_t *elem,
					IEEEtypes_Element_t *non_inherit_ie)
{
	t_u8 id_len, ext_id_len, i, loop_len, id;
	const t_u8 *list;

	ENTER();
	if (elem->ieee_hdr.element_id == MULTI_BSSID)
		return MFALSE;

	if (!non_inherit_ie || non_inherit_ie->ieee_hdr.len < 2)
		return MTRUE;

	/*
	 * non inheritance element format is:
	 * ext ID (56) | IDs list len | list | extension IDs list len | list
	 * Both lists are optional. Both lengths are mandatory */
	id_len = non_inherit_ie->data[1];
	if (non_inherit_ie->ieee_hdr.len < 3 + id_len)
		return MTRUE;

	ext_id_len = non_inherit_ie->data[2 + id_len];
	if (non_inherit_ie->ieee_hdr.len < 3 + id_len + ext_id_len)
		return MTRUE;

	if (elem->ieee_hdr.element_id == EXTENSION) {
		if (!ext_id_len)
			return MTRUE;
		loop_len = ext_id_len;
		list = &non_inherit_ie->data[3 + id_len];
		id = elem->data[0];
	} else {
		if (!id_len)
			return MTRUE;
		loop_len = id_len;
		list = &non_inherit_ie->data[2];
		id = elem->ieee_hdr.element_id;
	}

	for (i = 0; i < loop_len; i++) {
		if (list[i] == id)
			return MFALSE;
	}

	LEAVE();
	return MTRUE;
}

/**
 *  @brief This function copies an IE fragment by fragment from the parent
 *  		(Tx BSSID) to non-Tx BSSID
 *
 *  @param pmadapter	A pointer to mlan adapter
 *  @param elem			A pointer to the IE to be copied from parent to
 * NonTx BSSID
 *  @param ie     		A pointer to the IE buffer in parent Tx BSSID
 *  @param ie_len     	IE buffer length
 *  @param pos			Pointer to location where IE is copied
 *  @param buf        	Original buffer pointer of the new generated IE
 *  @param buf_len     	Buffer length of generated IE
 *  @return             Copied IE length
 */
static t_u32 wlan_copy_ie_with_fragments(pmlan_adapter pmadapter,
					 IEEEtypes_Element_t *elem, t_u8 *ie,
					 t_u32 ie_len, t_u8 **pos, t_u8 *buf,
					 t_u32 buf_len)
{
	ENTER();
	/* Error checking */
	if (elem->ieee_hdr.len + 2 > buf + buf_len - *pos) {
		PRINTM(MERROR, "Copy length in error!\n");
		return 0;
	}

	/* Copy the IE */
	memcpy_ext(pmadapter, *pos, elem, elem->ieee_hdr.len + 2,
		   elem->ieee_hdr.len + 2);
	DBG_HEXDUMP(MINFO, "Copying the IE", *pos, elem->ieee_hdr.len + 2);
	*pos += elem->ieee_hdr.len + 2;

	/* Finish the copy if element is not fragmented */
	if (elem->ieee_hdr.len != 255) {
		// coverity[overflow_sink:SUPPRESS]
		return *pos - buf;
	}

	/* Adjust the IE offsets */
	ie_len = ie + ie_len - elem->data - elem->ieee_hdr.len;
	ie = (t_u8 *)elem->data + elem->ieee_hdr.len;

	/* Check for remaining fragments and copy if any */
	/* this kernel API considers ie as const only */
	// coverity[misra_c_2012_rule_11_8_violation:SUPPRESS]
	for_each_element (elem, ie, ie_len) {
		if (elem->ieee_hdr.element_id != FRAGMENT)
			break;

		/* Error checking */
		if (elem->ieee_hdr.len + 2 > buf + buf_len - *pos) {
			PRINTM(MERROR, "Copy length in error!!\n");
			return 0;
		}

		/* Copy the IE */
		memcpy_ext(pmadapter, *pos, elem, elem->ieee_hdr.len + 2,
			   elem->ieee_hdr.len + 2);
		*pos += elem->ieee_hdr.len + 2;

		/* Break if element is not fragmented */
		if (elem->ieee_hdr.len != 255)
			break;
	}

	LEAVE();
	// coverity[overflow_sink:SUPPRESS]
	return *pos - buf;
}

/**
 *  @brief This function generates a new NonTx BSSID entry from
 *  	   the parent TxBSS entry
 *
 *  @param pmadapter		A pointer to mlan private
 *  @param ie     			A pointer to the IE buffer in parent Tx
 * BSSID
 *  @param ie_len     		IE buffer length
 *  @param merged_ie		Pointer to merged NonTx BSSID profile
 *  @param merged_ie_len    Length of the NonTx BSSID profile
 *  @param new_ie     		Pointer to new ie buffer
 *  @param new_ie_len     	Length of new ie buffer
 *  @return             	Copied IE length
 */
static t_u32 wlan_gen_new_ie(mlan_private *pmpriv, t_u8 *ie, t_u32 ie_len,
			     t_u8 *merged_ie, t_u32 merged_ie_len, t_u8 *new_ie,
			     t_u32 new_ie_len)
{
	mlan_adapter *pmadapter = pmpriv->adapter;
	IEEEtypes_ElementId_e element_id;
	IEEEtypes_Extension_t *pext_tlv = MNULL;
	IEEEtypes_Element_t *non_inherit_elem = MNULL;
	IEEEtypes_Element_t *parent = MNULL, *sub = MNULL;
	t_u8 element_len = 0, id = 0, ext_id = 0;
	t_u8 *pcurrent_ptr = merged_ie, *pos = new_ie;
	t_u32 left_len = merged_ie_len, match_len = 0;
	t_s32 ret_val = 0;

	ENTER();
	/* Fetch the Non-Inherit Element */
	while (left_len >= 2) {
		element_id = (IEEEtypes_ElementId_e)(*((t_u8 *)pcurrent_ptr));
		element_len = *((t_u8 *)pcurrent_ptr + 1);

		if (element_id == EXTENSION) {
			pext_tlv = (IEEEtypes_Extension_t *)pcurrent_ptr;

			if (pext_tlv->ext_id == NON_INHERITANCE) {
				non_inherit_elem =
					(IEEEtypes_Element_t *)pcurrent_ptr;
				break;
			}
		}
		pcurrent_ptr += element_len + 2;
		left_len -= (element_len + 2);
	}

	if (!non_inherit_elem)
		PRINTM(MINFO, "Non-Inherit Elem NOT present\n");

	/* Copy the elements from the parent MBSSID Beacon IE to the generated
	 * IE
	 * If they are included in the NonTx profile or in the Non inheritance
	 * element, then copy all occurances of these elements, the first time
	 * we encounter them
	 */
	for_each_element (parent, ie, ie_len) {
		if (parent->ieee_hdr.element_id == FRAGMENT)
			continue;

		if (parent->ieee_hdr.element_id == EXTENSION) {
			if (parent->ieee_hdr.len < 1)
				continue;

			id = EXTENSION;
			ext_id = parent->data[0];
			match_len = 1;
		} else {
			id = parent->ieee_hdr.element_id;
			match_len = 0;
		}

		/* Check for the first occurance in NonTx profile */
		sub = wlan_find_elem_match(pmadapter, id, merged_ie,
					   merged_ie_len, &ext_id, match_len,
					   0);

		/* Copy from Tx profile if not present in NonTx profile and
		 * inherited
		 */
		if (!sub &&
		    wlan_is_element_inherited(parent, non_inherit_elem)) {
			if ((ret_val = wlan_copy_ie_with_fragments(
				     pmadapter, parent, ie, ie_len, &pos,
				     new_ie, new_ie_len)) <= 0)
				return 0;

			continue;
		}

		/* Already copied if an earlier element had the same type */
		if (wlan_find_elem_match(pmadapter, id, ie, (t_u8 *)parent - ie,
					 &ext_id, match_len, 0))
			continue;

		/* Not inheriting, copy all similar elements from NonTx profile
		 */
		while (sub) {
			if ((ret_val = wlan_copy_ie_with_fragments(
				     pmadapter, sub, merged_ie, merged_ie_len,
				     &pos, new_ie, new_ie_len)) <= 0)
				return 0;
			// coverity[overflow_sink:SUPPRESS]
			sub = wlan_find_elem_match(
				pmadapter, id, sub->data + sub->ieee_hdr.len,
				merged_ie_len + merged_ie -
					(sub->data + sub->ieee_hdr.len),
				&ext_id, match_len, 0);
		}
	}

	/* The above loop skips the elements that are included in NonTx profile
	 * but NOT in the parent Tx profile; So do a pass over NonTx profile
	 * and append the missed IEs. Skip the NonTx BSSID cpas and Non-
	 * Inheritance element */
	for_each_element (sub, merged_ie, merged_ie_len) {
		if (sub->ieee_hdr.element_id == NONTX_BSSID_CAP)
			continue;

		if (sub->ieee_hdr.element_id == FRAGMENT)
			continue;

		if (sub->ieee_hdr.element_id == EXTENSION) {
			if (sub->ieee_hdr.len < 1)
				continue;

			id = EXTENSION;
			ext_id = sub->data[0];
			match_len = 1;

			if (ext_id == NON_INHERITANCE)
				continue;
		} else {
			id = sub->ieee_hdr.element_id;
			match_len = 0;
		}

		/* Processed if one was included in the parent */
		if (wlan_find_elem_match(pmadapter, id, ie, ie_len, &ext_id,
					 match_len, 0))
			continue;

		if ((ret_val = wlan_copy_ie_with_fragments(
			     pmadapter, sub, merged_ie, merged_ie_len, &pos,
			     new_ie, new_ie_len)) <= 0)
			return 0;
	}

	LEAVE();
	return pos - new_ie;
}

/**
 *  @brief This function generates the nonTx BSSID profile
 *
 *  @param pmadapter       		A pointer to mlan_private structure
 *  @param pbss_entry       	A pointer to BSSDescriptor_t which has
 * multi-bssid IE
 *  @param pmerged_profile  	A pointer to merged NonTX BSSID Profile
 *  @param profile_len      	Length of the merged NonTX Profile
 *  @param num_in_table			A pointer to buffer to save num of entry
 * in scan table.
 *  @param max_bssid_indicator 	max bssid indicator
 *
 *  @return                 N/A
 */
static void wlan_gen_non_trans_bssid_profile(mlan_private *pmpriv,
					     BSSDescriptor_t *pbss_entry,
					     t_u8 *pmerged_profile,
					     t_u32 profile_len,
					     t_u32 *num_in_table,
					     t_u8 max_bssid_indicator)
{
	mlan_adapter *pmadapter = pmpriv->adapter;
	t_u8 *pcurrent_ptr = pmerged_profile;
	IEEEtypes_ElementId_e element_id =
		(IEEEtypes_ElementId_e)(*((t_u8 *)pcurrent_ptr));
	IEEEtypes_MultiBSSIDIndex_t *pbssid_index = MNULL;
	IEEEtypes_NotxBssCap_t *pcap = MNULL;
	mlan_callbacks *pcb = (pmlan_callbacks)&pmadapter->callbacks;
	BSSDescriptor_t *bss_new_entry = MNULL;
	t_u8 *pbeacon_buf = MNULL, *pnew_beacon_buf = MNULL;
	t_u8 element_len = *((t_u8 *)pcurrent_ptr + 1);
	t_u32 left_len = profile_len;
	t_u32 ie_len = pbss_entry->beacon_buf_size - BEACON_FIX_SIZE;
	t_u32 copied_len = 0;
	t_u8 ret = MFALSE;

	ENTER();
	/* Check for NonTx BSSID Capability */
	if (element_id != NONTX_BSSID_CAP || element_len != 2) {
		PRINTM(MERROR,
		       "The first element within the NonTx BSSID profile is not the "
		       "NonTx BSSID Capability element\n");
		LEAVE();
		return;
	}

	/* Check for valid NonTx BSSID */
	while (left_len >= 2) {
		element_id = (IEEEtypes_ElementId_e)(*((t_u8 *)pcurrent_ptr));
		element_len = *((t_u8 *)pcurrent_ptr + 1);

		if ((t_u8)(element_len + sizeof(IEEEtypes_Header_t)) >
		    left_len) {
			PRINTM(MERROR, "Invalid IE length = %d left len %d\n",
			       element_len, left_len);
			goto done;
		}

		/* Find the NonTx Capability */
		if (element_id == NONTX_BSSID_CAP) {
			pcap = (IEEEtypes_NotxBssCap_t *)pcurrent_ptr;
			PRINTM(MINFO, "NonTx Cap =%x\n", pcap->cap);
		}

		/* Find the MBSSID Index */
		if (element_id == MBSSID_INDEX) {
			pbssid_index =
				(IEEEtypes_MultiBSSIDIndex_t *)pcurrent_ptr;
			if (pbssid_index->bssid_index == 0 ||
			    pbssid_index->bssid_index > 46) {
				PRINTM(MERROR,
				       "No valid Multiple BSSID-Index element\n");
				goto done;
			}
			PRINTM(MCMND, "MBSSID: mbssid_index=%d\n",
			       pbssid_index->bssid_index);
			ret = MTRUE;
			break;
		}

		left_len -= (element_len + 2);
		pcurrent_ptr += element_len + 2;
	}

	if (ret == MTRUE) {
		ret = pcb->moal_malloc(pmadapter->pmoal_handle,
				       sizeof(BSSDescriptor_t), MLAN_MEM_DEF,
				       (t_u8 **)&bss_new_entry);

		if (ret != MLAN_STATUS_SUCCESS || !bss_new_entry) {
			PRINTM(MERROR,
			       "Memory allocation for bss_new_entry failed!\n");
			goto done;
		}

		/* Populate the fixed fields of new NonTx BSS entry */
		memcpy_ext(pmadapter, bss_new_entry, pbss_entry,
			   sizeof(BSSDescriptor_t), sizeof(BSSDescriptor_t));

		wlan_gen_multi_bssid_by_bssid_index(pmadapter, pbss_entry,
						    bss_new_entry,
						    pbssid_index->bssid_index,
						    max_bssid_indicator);

		memcpy_ext(pmadapter, &bss_new_entry->cap_info, &pcap->cap,
			   sizeof(IEEEtypes_CapInfo_t),
			   sizeof(IEEEtypes_CapInfo_t));

		bss_new_entry->multi_bssid_ap = MULTI_BSSID_SUB_AP;

		/* Allocate the beacon buffer for new entry */
		// coverity[overflow_sink:SUPPRESS]
		ret = pcb->moal_malloc(pmadapter->pmoal_handle, ie_len,
				       MLAN_MEM_DEF, (t_u8 **)&pbeacon_buf);
		if (ret != MLAN_STATUS_SUCCESS || !pbeacon_buf) {
			PRINTM(MERROR,
			       "Memory allocation for beacon buf failed!\n");
			goto done;
		}

		/** Generate the NonTx BSSID Beacon buffer */
		// coverity[overflow_sink:SUPPRESS]
		copied_len = wlan_gen_new_ie(
			pmpriv, pbss_entry->pbeacon_buf + BEACON_FIX_SIZE,
			ie_len, pmerged_profile, profile_len, pbeacon_buf,
			ie_len);

		if (!copied_len) {
			PRINTM(MERROR, "Failed to generate NonTx BSSID IE!\n");
			goto done;
		} else {
			PRINTM(MMSG, "NonTx Beacon Buffer IE Len = %d\n",
			       copied_len);
		}
		DBG_HEXDUMP(MCMD_D, "NonTx BSSID", pbeacon_buf, copied_len);

		/** Update NonTx BSS descriptor entries */
		// coverity[overflow_sink:SUPPRESS]
		if (MLAN_STATUS_SUCCESS !=
		    wlan_update_nonTx_bss_desc(pmadapter, pbss_entry,
					       bss_new_entry, &pbeacon_buf,
					       copied_len)) {
			PRINTM(MERROR,
			       "Fail to update NonTx BSSID beacon buf\n");
			goto done;
		}
		pnew_beacon_buf = bss_new_entry->pbeacon_buf;
		DBG_HEXDUMP(MCMD_D, "NonTx Beacon buf",
			    bss_new_entry->pbeacon_buf,
			    BEACON_FIX_SIZE + copied_len);

		/** Add the NonTx BSS entry to the scan table */
		wlan_add_new_entry_to_scan_table(pmpriv, bss_new_entry,
						 num_in_table);
	}
done:
	pcb->moal_mfree(pmadapter->pmoal_handle, (t_u8 *)(pnew_beacon_buf));
	pcb->moal_mfree(pmadapter->pmoal_handle, (t_u8 *)bss_new_entry);
	pcb->moal_mfree(pmadapter->pmoal_handle, (t_u8 *)pbeacon_buf);
	LEAVE();
	return;
}

/**
 *  @brief This function finds the next MBSSID element containing the split
 *         NonTx BSSID profile
 *
 *  @param pbss_entry   	A pointer to BSSDescriptor_t that has MBSSID IE
 *  @param ie_len      		Maximum IE len in beacon/probe response
 *  @param pmbssid     		A pointer to MBSSID IE
 *  @param pnontx_bssid     A pointer to NonTx BSSID subelement
 *  @return                 A pointer to next MBSSID element
 */
static IEEEtypes_MultiBSSID_t *
wlan_get_next_mbssid_profile(BSSDescriptor_t *pbss_entry, t_u32 ie_len,
			     IEEEtypes_MultiBSSID_t *pmbssid,
			     IEEEtypes_NonTransBSSIDProfile_t *pnontx_bssid)
{
	IEEEtypes_MultiBSSID_t *pnext_mbssid = MNULL;
	IEEEtypes_NonTransBSSIDProfile_t *pnext_nontx_bssid = MNULL;
	t_u8 *mbssid_end = MNULL;
	t_u8 *pcurrent_ptr = MNULL;
	IEEEtypes_ElementId_e element_id;
	t_u32 bytes_left = 0;
	t_u16 total_ie_len = 0;
	t_u8 element_len = 0;

	ENTER();
	if (!pmbssid || !pnontx_bssid) {
		PRINTM(MERROR, "No MBSSID or NonTx BSSID element present\n");
		return MNULL;
	}

	mbssid_end = pmbssid->sub_elem_data + (pmbssid->ieee_hdr.len - 1);
	pcurrent_ptr = pmbssid->sub_elem_data + (pmbssid->ieee_hdr.len - 1);
	bytes_left = ie_len -
		     (mbssid_end - (pbss_entry->pbeacon_buf + BEACON_FIX_SIZE));

	/* If it is not the last NonTx subelement in current MBSSID IE,
	 * return MNULL
	 */
	if ((pnontx_bssid->profile_data + pnontx_bssid->ieee_hdr.len) <
	    (mbssid_end - 1)) {
		PRINTM(MMSG, "Not the last Subelement\n");
		return MNULL;
	}

	/* Search for the next MBSSID */
	while (bytes_left >= 2) {
		element_id = (IEEEtypes_ElementId_e)(*((t_u8 *)pcurrent_ptr));
		element_len = *((t_u8 *)pcurrent_ptr + 1);
		total_ie_len = element_len + sizeof(IEEEtypes_Header_t);
		PRINTM(MINFO, "bytes_left=%d total_ie_len=%d\n", bytes_left,
		       total_ie_len);

		if (bytes_left < total_ie_len) {
			PRINTM(MERROR, "InterpretIE: Error in processing IE, "
				       "bytes left < IE length\n");
			bytes_left = 0;
			continue;
		}
		if (element_id == MULTI_BSSID) {
			pnext_mbssid = (IEEEtypes_MultiBSSID_t *)pcurrent_ptr;
			break;
		}
		pcurrent_ptr += total_ie_len;
		bytes_left -= total_ie_len;
	}

	/* There isn't a next MBSSID IE - profile is complete.
	 */
	if (!pnext_mbssid) {
		PRINTM(MMSG, "Profile Complete\n");
		return MNULL;
	}

	/* Length error */
	if (pnext_mbssid->ieee_hdr.len < 4) {
		PRINTM(MERROR, "Next MBSSID Length error\n");
		return MNULL;
	}

	/* Next nonTx BSSID */
	pnext_nontx_bssid =
		(IEEEtypes_NonTransBSSIDProfile_t *)pnext_mbssid->sub_elem_data;

	/* Next nonTx BSSID length error */
	if ((pnext_mbssid->sub_elem_data + pnext_mbssid->ieee_hdr.len - 1) <
	    (pnext_nontx_bssid->profile_data +
	     pnext_nontx_bssid->ieee_hdr.len)) {
		PRINTM(MERROR, "Next nonTxBSSID Length error\n");
		return MNULL;
	}

	if ((pnext_nontx_bssid->ieee_hdr.element_id != 0) ||
	    (pnext_nontx_bssid->ieee_hdr.len < 2)) {
		PRINTM(MERROR, "Next nonTxBSSID Length error\n");
		return MNULL;
	}

	/* Check if next nonTx BSSID is start of a new profile
	 * OR a split profile */
	LEAVE();
	return pnext_nontx_bssid->profile_data[0] == NONTX_BSSID_CAP ?
		       MNULL :
		       pnext_mbssid;
}

/**
 *  @brief This function merges the NonTx BSSID profile entries split
 *         across multiple MBSSID elements
 *
 *  @param pmpriv			A pointer to mlan adapter
 *  @param pbss_entry   	A pointer to BSSDescriptor_t that has MBSSID IE
 *  @param pmbssid     		A pointer to MBSSID IE
 *  @param pnontx_bssid     A pointer to NonTx BSSID subelement
 *  @param merged_ie        A pointer to merged NonTx BSSID element
 *  @param max_copy_len     Maximum IE len in beacon/probe response
 *  @return                 Length of merged NonTX BSSID element
 */
static t_u32
wlan_merge_nontx_bssid_profile(pmlan_adapter pmadapter,
			       BSSDescriptor_t *pbss_entry,
			       IEEEtypes_MultiBSSID_t *pmbssid,
			       IEEEtypes_NonTransBSSIDProfile_t *pnontx_bssid,
			       t_u8 *merged_ie, t_u32 max_copy_len)
{
	IEEEtypes_MultiBSSID_t *pnext_mbssid = pmbssid;
	IEEEtypes_NonTransBSSIDProfile_t *pnext_nontx_bssid = pnontx_bssid;
	t_u32 copied_len = 0;
	t_u32 ie_len = pbss_entry->beacon_buf_size - BEACON_FIX_SIZE;

	ENTER();
	if (!pmbssid || !pnontx_bssid) {
		PRINTM(MERROR, "No MBSSID or NonTx BSSID element present\n");
		return 0;
	}

	/* Length error */
	if (pnontx_bssid->ieee_hdr.len > max_copy_len) {
		PRINTM(MERROR,
		       "Invalid NonTxBSSID profile length:%d max_copy_len:%d\n",
		       pnontx_bssid->ieee_hdr.len, max_copy_len);
		return 0;
	}

	copied_len = pnontx_bssid->ieee_hdr.len;
	/* Copy the 1st part of NonTxBssid profile */
	PRINTM(MINFO, "NonTxBSSID 1st part length: %d\n",
	       pnontx_bssid->ieee_hdr.len);
	memcpy_ext(pmadapter, merged_ie, pnontx_bssid->profile_data,
		   pnontx_bssid->ieee_hdr.len, pnontx_bssid->ieee_hdr.len);

	/* Check for split nonTxBssid in next MBSSID elem */
	// coverity[overflow_sink:SUPPRESS]
	while ((pnext_mbssid = wlan_get_next_mbssid_profile(
			pbss_entry, ie_len, pnext_mbssid, pnext_nontx_bssid)) !=
	       MNULL) {
		// coverity[overflow_sink:SUPPRESS]
		pnext_nontx_bssid = (IEEEtypes_NonTransBSSIDProfile_t *)
					    pnext_mbssid->sub_elem_data;

		if (copied_len + pnext_nontx_bssid->ieee_hdr.len >
		    max_copy_len) {
			PRINTM(MINFO, "Total length:%d\n",
			       copied_len + pnext_nontx_bssid->ieee_hdr.len);
			break;
		}
		memcpy_ext(pmadapter, merged_ie + copied_len,
			   pnext_nontx_bssid->profile_data,
			   pnext_nontx_bssid->ieee_hdr.len,
			   pnext_nontx_bssid->ieee_hdr.len);
		copied_len += pnext_nontx_bssid->ieee_hdr.len;
		PRINTM(MINFO, "NonTxBSSID next part length: %d\n",
		       pnext_nontx_bssid->ieee_hdr.len);
	}

	LEAVE();
	// coverity[overflow_sink:SUPPRESS]
	return copied_len;
}

/**
 *  @brief This function parses the multi_bssid IE from pbss_entry
 *
 *  @param pmpriv        A pointer to mlan_private structure
 *  @param pbss_entry    A pointer to BSSDescriptor_t that has Multi-BSSID IE
 *  @param pmulti_bssid  A pointer to Multi-BSSID IE
 *  @param num_in_table  A pointer to number entry in the scan table
 *
 *  @return              void
 */
static t_void wlan_parse_multi_bssid_ie(mlan_private *pmpriv,
					BSSDescriptor_t *pbss_entry,
					IEEEtypes_MultiBSSID_t *pmulti_bssid,
					t_u32 *num_in_table)
{
	mlan_adapter *pmadapter = pmpriv->adapter;
	t_u32 bytes_left = 0, max_copy_len = 0, profile_len = 0;
	t_u8 *pcurrent_ptr = MNULL;
	IEEEtypes_NonTransBSSIDProfile_t *pbssid_profile = MNULL;
	mlan_callbacks *pcb = (pmlan_callbacks)&pmadapter->callbacks;
	t_u8 *pmerged_profile = MNULL;
	int length = 0;
	t_u8 ret = 0;

	ENTER();
	if (!pmulti_bssid)
		return;

	if (pmulti_bssid->ieee_hdr.len < 4) {
		PRINTM(MINFO, "MBSSID IE length error!\n");
		return;
	}

	max_copy_len = pbss_entry->beacon_buf_size - BEACON_FIX_SIZE;
	bytes_left = pmulti_bssid->ieee_hdr.len - 1;
	pcurrent_ptr = pmulti_bssid->sub_elem_data;

	/* Allocate memory for the merged profile */
	// coverity[overflow_sink:SUPPRESS]
	// coverity[overwrite_var:SUPPRESS]
	ret = pcb->moal_malloc(pmadapter->pmoal_handle, max_copy_len,
			       MLAN_MEM_DEF, &pmerged_profile);
	if (ret != MLAN_STATUS_SUCCESS || !pmerged_profile) {
		PRINTM(MERROR,
		       "Memory allocation for pmerged_profile failed!\n");
		goto done;
	}

	while (bytes_left >= 2) {
		/* NonTx BSSID Profile */
		pbssid_profile =
			(IEEEtypes_NonTransBSSIDProfile_t *)pcurrent_ptr;

		if (pbssid_profile->ieee_hdr.element_id !=
		    NONTRANS_BSSID_PROFILE_SUBELEM_ID) {
			PRINTM(MERROR, "Invalid NonTx BSSID IE\n");
			break;
		}
		length = pbssid_profile->ieee_hdr.len + 2;
		if (bytes_left < (t_u32)length) {
			PRINTM(MERROR, "Invalid NonTx BSSID IE length\n");
			break;
		}

		/* Check for NonTx BSSID Capability */
		if (pbssid_profile->profile_data[0] != NONTX_BSSID_CAP) {
			PRINTM(MERROR,
			       "The first element within the NonTx BSSID profile is not the "
			       "NonTx BSSID Capability element\n");
			pcurrent_ptr += pbssid_profile->ieee_hdr.len + 2;
			bytes_left -= pbssid_profile->ieee_hdr.len + 2;
			continue;
		}

		/* Merge the split nonTxBSSID profiles */
		profile_len = wlan_merge_nontx_bssid_profile(
			pmpriv->adapter, pbss_entry, pmulti_bssid,
			pbssid_profile, pmerged_profile, max_copy_len);
		PRINTM(MCMND, "Length of Merged profile: %d\n", profile_len);
		if (!profile_len)
			break;

		DBG_HEXDUMP(MCMD_D, "Merged NonTx Profile", pmerged_profile,
			    profile_len);

		/* Generate the NonTx BSSID entry and add to the scan table */
		// coverity[overflow_sink:SUPPRESS]
		wlan_gen_non_trans_bssid_profile(
			pmpriv, pbss_entry, pmerged_profile, profile_len,
			num_in_table, pmulti_bssid->max_bssid_indicator);

		pcurrent_ptr += pbssid_profile->ieee_hdr.len + 2;
		bytes_left -= pbssid_profile->ieee_hdr.len + 2;

		// coverity[bad_memset:SUPPRESS]
		memset(pmadapter, (t_u8 *)pmerged_profile, 0x00, max_copy_len);
	}
done:
	pcb->moal_mfree(pmadapter->pmoal_handle, (t_u8 *)pmerged_profile);
	LEAVE();
	return;
}

/**
 *  @brief This function search all the mbssid IE in the beacon buffer
 *
 *  @param pmpriv           A pointer to mlan_private structure
 *  @param pbss_entry       A pointer to BSSDescriptor_t which has multi-bssid
 * IE
 *  @param num_in_table     A pointer to buffer to save num of entry in scan
 * table.
 *
 *  @return                 N/A
 */
static void wlan_parse_multi_bssid_ap(mlan_private *pmpriv,
				      BSSDescriptor_t *pbss_entry,
				      t_u32 *num_in_table)
{
	IEEEtypes_ElementId_e element_id;
	t_u8 element_len;
	t_u16 total_ie_len;
	t_u32 bytes_left = pbss_entry->beacon_buf_size - BEACON_FIX_SIZE;
	t_u8 *pcurrent_ptr = pbss_entry->pbeacon_buf + BEACON_FIX_SIZE;
	IEEEtypes_Ssid_t *pssid = (IEEEtypes_Ssid_t *)pcurrent_ptr;

	if (pssid->element_id != SSID) {
		PRINTM(MERROR,
		       "Invalid beacon ie, ssid should be in the first element\n");
		return;
	}
	/* Process variable IE */
	while (bytes_left >= 2) {
		element_id = (IEEEtypes_ElementId_e)(*((t_u8 *)pcurrent_ptr));
		element_len = *((t_u8 *)pcurrent_ptr + 1);
		total_ie_len = element_len + sizeof(IEEEtypes_Header_t);

		if (bytes_left < total_ie_len) {
			PRINTM(MERROR, "InterpretIE: Error in processing IE, "
				       "bytes left < IE length\n");
			bytes_left = 0;
			continue;
		}
		if (element_id == MULTI_BSSID) {
			PRINTM(MINFO, "Found MBSSID IE!!\n");
			wlan_parse_multi_bssid_ie(
				pmpriv, pbss_entry,
				(IEEEtypes_MultiBSSID_t *)pcurrent_ptr,
				num_in_table);
		}
		pcurrent_ptr += total_ie_len;
		bytes_left -= total_ie_len;
	}
	return;
}

/**
 *  @brief This function parse and store the extended scan results
 *
 *  @param pmpriv           A pointer to mlan_private structure
 *  @param number_of_sets   Number of BSS
 *  @param pscan_resp       A pointer to scan response buffer
 *  @param scan_resp_size   Size of scan response buffer
 *
 *  @return             MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status wlan_parse_ext_scan_result(mlan_private *pmpriv,
					      t_u8 number_of_sets,
					      t_u8 *pscan_resp,
					      t_u16 scan_resp_size)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_adapter *pmadapter = pmpriv->adapter;
	mlan_callbacks *pcb = MNULL;
	BSSDescriptor_t *bss_new_entry = MNULL;
	t_u8 *pbss_info;
	t_u32 bytes_left;
	t_u32 bytes_left_for_tlv;
	t_u32 num_in_table;
	t_u32 idx;
	t_u64 tsf_val;
	chan_freq_power_t *cfp;
	t_u16 tlv_type, tlv_len;
	MrvlIEtypes_Data_t *ptlv = MNULL;
	MrvlIEtypes_Bss_Scan_Rsp_t *pscan_rsp_tlv = MNULL;
	MrvlIEtypes_Bss_Scan_Info_t *pscan_info_tlv = MNULL;
	t_u16 band;
	t_u32 age_ts_usec;

	ENTER();
	pcb = (pmlan_callbacks)&pmadapter->callbacks;

	if (number_of_sets > MRVDRV_MAX_BSSID_LIST) {
		PRINTM(MERROR,
		       "EXT_SCAN: Invalid number of AP returned (%d)!!\n",
		       number_of_sets);
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	bytes_left = scan_resp_size;
	PRINTM(MINFO, "EXT_SCAN: bss_descript_size %d\n", scan_resp_size);
	PRINTM(MINFO, "EXT_SCAN: returned %d APs before parsing\n",
	       number_of_sets);
	/* Update the age_in_second */
	pmadapter->callbacks.moal_get_system_time(
		pmadapter->pmoal_handle, &pmadapter->age_in_secs, &age_ts_usec);

	num_in_table = pmadapter->num_in_scan_table;
	ptlv = (MrvlIEtypes_Data_t *)pscan_resp;

	/*
	 *  Process each scan response returned number_of_sets. Save
	 *    the information in the bss_new_entry and then insert into the
	 *    driver scan table either as an update to an existing entry
	 *    or as an addition at the end of the table
	 */
	ret = pcb->moal_malloc(pmadapter->pmoal_handle, sizeof(BSSDescriptor_t),
			       MLAN_MEM_DEF, (t_u8 **)&bss_new_entry);

	if (ret != MLAN_STATUS_SUCCESS || !bss_new_entry) {
		PRINTM(MERROR, "Memory allocation for bss_new_entry failed!\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	for (idx = 0;
	     idx < number_of_sets && bytes_left > sizeof(MrvlIEtypesHeader_t);
	     idx++) {
		tlv_type = wlan_le16_to_cpu(ptlv->header.type);
		tlv_len = wlan_le16_to_cpu(ptlv->header.len);
		if (bytes_left < sizeof(MrvlIEtypesHeader_t) + tlv_len) {
			PRINTM(MERROR,
			       "EXT_SCAN: Error bytes left < TLV length\n");
			break;
		}
		pscan_rsp_tlv = MNULL;
		pscan_info_tlv = MNULL;
		bytes_left_for_tlv = bytes_left;
		/*
		 * BSS response TLV with beacon or probe response buffer
		 * at the initial position of each descriptor
		 */
		if (tlv_type == TLV_TYPE_BSS_SCAN_RSP) {
			pbss_info = (t_u8 *)ptlv;
			pscan_rsp_tlv = (MrvlIEtypes_Bss_Scan_Rsp_t *)ptlv;
			ptlv = (MrvlIEtypes_Data_t *)(ptlv->data + tlv_len);
			bytes_left_for_tlv -=
				(tlv_len + sizeof(MrvlIEtypesHeader_t));
		} else
			break;

		/* Process variable TLV */
		while (bytes_left_for_tlv >= sizeof(MrvlIEtypesHeader_t) &&
		       wlan_le16_to_cpu(ptlv->header.type) !=
			       TLV_TYPE_BSS_SCAN_RSP) {
			tlv_type = wlan_le16_to_cpu(ptlv->header.type);
			tlv_len = wlan_le16_to_cpu(ptlv->header.len);
			if (bytes_left_for_tlv <
			    sizeof(MrvlIEtypesHeader_t) + tlv_len) {
				PRINTM(MERROR,
				       "EXT_SCAN: Error in processing TLV, "
				       "bytes left < TLV length\n");
				pscan_rsp_tlv = MNULL;
				bytes_left_for_tlv = 0;
				continue;
			}
			switch (tlv_type) {
			case TLV_TYPE_BSS_SCAN_INFO:
				pscan_info_tlv =
					(MrvlIEtypes_Bss_Scan_Info_t *)ptlv;
				if (tlv_len !=
				    sizeof(MrvlIEtypes_Bss_Scan_Info_t) -
					    sizeof(MrvlIEtypesHeader_t)) {
					bytes_left_for_tlv = 0;
					continue;
				}
				break;
			default:
				break;
			}
			ptlv = (MrvlIEtypes_Data_t *)(ptlv->data + tlv_len);
			bytes_left -= (tlv_len + sizeof(MrvlIEtypesHeader_t));
			bytes_left_for_tlv -=
				(tlv_len + sizeof(MrvlIEtypesHeader_t));
		}
		/* No BSS response TLV */
		if (pscan_rsp_tlv == MNULL)
			break;

		/*
		 * Advance pointer to the beacon buffer length and
		 * update the bytes count so that the function
		 * wlan_interpret_bss_desc_with_ie() can handle the
		 * scan buffer withut any change
		 */
		pbss_info += sizeof(t_u16);
		bytes_left -= sizeof(t_u16);

		/* Zero out the bss_new_entry we are about to store info in */
		memset(pmadapter, bss_new_entry, 0x00, sizeof(BSSDescriptor_t));

		/* Process the data fields and IEs returned for this BSS */
		if (wlan_interpret_bss_desc_with_ie(
			    pmadapter, bss_new_entry, &pbss_info, &bytes_left,
			    MTRUE) == MLAN_STATUS_SUCCESS) {
			PRINTM(MINFO, "EXT_SCAN: BSSID = " MACSTR "\n",
			       MAC2STR(bss_new_entry->mac_address));

			band = BAND_G;
			/*
			 * If the BSS info TLV was appended to the scan results,
			 * save this entry's TSF value in the networkTSF field.
			 * The networkTSF is the firmware's TSF value at the
			 * time the beacon or probe response was received.
			 */
			if (pscan_info_tlv) {
				/* RSSI is 2 byte long */
				bss_new_entry->rssi = -(t_s32)(wlan_le16_to_cpu(
					pscan_info_tlv->rssi));
				PRINTM(MINFO, "EXT_SCAN: RSSI=%d\n",
				       bss_new_entry->rssi);
				memcpy_ext(pmpriv->adapter, &tsf_val,
					   &pscan_info_tlv->tsf,
					   sizeof(tsf_val), sizeof(tsf_val));
				tsf_val = wlan_le64_to_cpu(tsf_val);
				memcpy_ext(pmpriv->adapter,
					   &bss_new_entry->network_tsf,
					   &tsf_val,
					   sizeof(bss_new_entry->network_tsf),
					   sizeof(bss_new_entry->network_tsf));
				band = radio_type_to_band(
					pscan_info_tlv->bandcfg.chanBand);
				if (!bss_new_entry->channel)
					bss_new_entry->channel =
						pscan_info_tlv->channel;
			}
			/* Save the band designation for this entry for use in
			 * join */
			bss_new_entry->bss_band = band;
			bss_new_entry->age_in_secs = pmadapter->age_in_secs;

			cfp = wlan_find_cfp_by_band_and_channel(
				pmadapter, bss_new_entry->bss_band,
				(t_u16)bss_new_entry->channel);
			if (cfp)
				bss_new_entry->freq = cfp->freq;
			else
				bss_new_entry->freq = 0;

			/* Skip entry if on blacklisted channel */
			if (cfp && cfp->dynamic.blacklist) {
				PRINTM(MINFO,
				       "EXT_SCAN: dropping entry on blacklist channel.\n");
				continue;
			}
			if (IS_FW_SUPPORT_MULTIBSSID(pmadapter)) {
				if (bss_new_entry->multi_bssid_ap ==
				    MULTI_BSSID_AP)
					wlan_parse_multi_bssid_ap(
						pmpriv, bss_new_entry,
						&num_in_table);
			}
			if (pmadapter->wifi_6g_scan_coloc_ap &&
			    IS_FW_SUPPORT_6G(pmadapter)) {
				if (bss_new_entry->prnr_ie)
					wlan_parse_rnr_colocated_ap(
						pmadapter, bss_new_entry);
			}
			wlan_add_new_entry_to_scan_table(pmpriv, bss_new_entry,
							 &num_in_table);

		} else {
			/* Error parsing/interpreting the scan response, skipped
			 */
			PRINTM(MERROR,
			       "EXT_SCAN: wlan_interpret_bss_desc_with_ie returned error\n");
		}
	}

	PRINTM(MCMND, "EXT_SCAN: Scanned %2d APs, %d valid, %d total\n",
	       number_of_sets, num_in_table - pmadapter->num_in_scan_table,
	       num_in_table);

	/* Update the total number of BSSIDs in the scan table */
	pmadapter->num_in_scan_table = num_in_table;
	/* Update the age_in_second */
	pmadapter->callbacks.moal_get_system_time(
		pmadapter->pmoal_handle, &pmadapter->age_in_secs, &age_ts_usec);

done:
	if (bss_new_entry)
		pcb->moal_mfree(pmadapter->pmoal_handle, (t_u8 *)bss_new_entry);

	LEAVE();
	return ret;
}

/**
 *  @brief This function handles the event extended scan report
 *
 *  @param pmpriv       A pointer to mlan_private structure
 *  @param pmbuf        A pointer to mlan_buffer
 *
 *  @return             MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status wlan_handle_event_ext_scan_report(mlan_private *pmpriv,
					      mlan_buffer *pmbuf)
{
	mlan_adapter *pmadapter = pmpriv->adapter;
	mlan_callbacks *pcb = &pmadapter->callbacks;
	mlan_ioctl_req *pioctl_req = MNULL;
	cmd_ctrl_node *pcmd_node = MNULL;
	mlan_status ret = MLAN_STATUS_SUCCESS;

	mlan_event_scan_result *pevent_scan =
		(pmlan_event_scan_result)(pmbuf->pbuf + pmbuf->data_offset);
	t_u8 *ptlv = (pmbuf->pbuf + pmbuf->data_offset +
		      sizeof(mlan_event_scan_result));
	t_u16 tlv_buf_left = wlan_le16_to_cpu(pevent_scan->buf_size);

	DBG_HEXDUMP(MCMD_D, "EVENT EXT_SCAN", pmbuf->pbuf + pmbuf->data_offset,
		    pmbuf->data_len);

	if (!pevent_scan->more_event)
		pmadapter->scan_state |= SCAN_STATE_EXT_SCAN_RESULT |
					 SCAN_STATE_LAST_EXT_SCAN_RESULT;
	else
		pmadapter->scan_state |= SCAN_STATE_EXT_SCAN_RESULT;

	wlan_parse_ext_scan_result(pmpriv, pevent_scan->num_of_set, ptlv,
				   tlv_buf_left);
	if (!pevent_scan->more_event &&
	    (pmadapter->ext_scan_type != EXT_SCAN_ENHANCE)) {
		wlan_request_cmd_lock(pmadapter);
		if (!util_peek_list(pmadapter->pmoal_handle,
				    &pmadapter->scan_pending_q, MNULL, MNULL)) {
			wlan_release_cmd_lock(pmadapter);
			if (pmadapter->pscan_ioctl_req) {
				if (((mlan_ds_scan *)
					     pmadapter->pscan_ioctl_req->pbuf)
						    ->sub_command ==
					    MLAN_OID_SCAN_SPECIFIC_SSID ||
				    ((mlan_ds_scan *)
					     pmadapter->pscan_ioctl_req->pbuf)
						    ->sub_command ==
					    MLAN_OID_SCAN_USER_CONFIG) {
					if (wlan_active_scan_req_for_passive_chan(
						    pmpriv,
						    pmadapter->pscan_ioctl_req)) {
						LEAVE();
						return ret;
					}
				}
			}
			if ((pmadapter->wifi_6g_scan_split) &&
			    (!pmadapter->scan_6g)) {
				if (wlan_scan_6g_network(
					    pmpriv,
					    pmadapter->pscan_ioctl_req)) {
					LEAVE();
					return ret;
				}
			}
			/*
			 * Process the resulting scan table:
			 *   - Remove any bad ssids
			 *   - Update our current BSS information from scan data
			 */
			wlan_scan_process_results(pmpriv);
			wlan_request_cmd_lock(pmadapter);
			pmadapter->scan_processing = MFALSE;
			pmadapter->scan_state |= SCAN_STATE_SCAN_COMPLETE;
			pioctl_req = pmadapter->pscan_ioctl_req;
			pmadapter->pscan_ioctl_req = MNULL;
			/* Need to indicate IOCTL complete */
			if (pioctl_req != MNULL) {
				pioctl_req->status_code = MLAN_ERROR_NO_ERROR;
				/* Indicate ioctl complete */
				pcb->moal_ioctl_complete(
					pmadapter->pmoal_handle,
					(pmlan_ioctl_req)pioctl_req,
					MLAN_STATUS_SUCCESS);
			}
			wlan_release_cmd_lock(pmadapter);

			pmadapter->bgscan_reported = MFALSE;
			wlan_recv_event(pmpriv, MLAN_EVENT_ID_DRV_SCAN_REPORT,
					MNULL);
		} else {
			/* If firmware not ready, do not issue any more scan
			 * commands */
			if (pmadapter->hw_status != WlanHardwareStatusReady) {
				wlan_release_cmd_lock(pmadapter);
				/* Flush all pending scan commands */
				wlan_flush_scan_queue(pmadapter);
				wlan_request_cmd_lock(pmadapter);
				pmadapter->scan_processing = MFALSE;
				pmadapter->scan_state |=
					SCAN_STATE_SCAN_COMPLETE;

				pioctl_req = pmadapter->pscan_ioctl_req;
				pmadapter->pscan_ioctl_req = MNULL;
				/* Indicate IOCTL complete */
				if (pioctl_req != MNULL) {
					pioctl_req->status_code =
						MLAN_ERROR_FW_NOT_READY;

					/* Indicate ioctl complete */
					pcb->moal_ioctl_complete(
						pmadapter->pmoal_handle,
						(pmlan_ioctl_req)pioctl_req,
						MLAN_STATUS_FAILURE);
				}
				wlan_release_cmd_lock(pmadapter);
			} else {
				/* Get scan command from scan_pending_q and put
				 * to cmd_pending_q */
				pcmd_node = (cmd_ctrl_node *)util_dequeue_list(
					pmadapter->pmoal_handle,
					&pmadapter->scan_pending_q, MNULL,
					MNULL);
				wlan_insert_cmd_to_pending_q(pmadapter,
							     pcmd_node, MTRUE);
				wlan_release_cmd_lock(pmadapter);
			}
		}
	}
	LEAVE();
	return ret;
}

/**
 *  @brief This function handles the event extended scan status
 *
 *  @param pmpriv       A pointer to mlan_private structure
 *  @param pmbuf        A pointer to mlan_buffer
 *
 *  @return             MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status wlan_handle_event_ext_scan_status(mlan_private *pmpriv,
					      mlan_buffer *pmbuf)
{
	mlan_adapter *pmadapter = pmpriv->adapter;
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_event_scan_status *scan_event;
	mlan_ioctl_req *pioctl_req;
	mlan_callbacks *pcb = (mlan_callbacks *)&pmadapter->callbacks;
	t_u16 tlv_buf_left, tlv_len, tlv_type;
	MrvlIEtypesHeader_t *tlv;
	MrvlIEtypes_ChannelStats_t *tlv_chan_stats;
	t_u8 status = 0;
	cmd_ctrl_node *pcmd_node = MNULL;

	ENTER();

	if (pmbuf->data_len < sizeof(mlan_event_scan_status)) {
		PRINTM(MERROR, "Wrong ext scan status event data length\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	pmadapter->scan_state |= SCAN_STATE_EXT_SCAN_STATUS;

	scan_event =
		(pmlan_event_scan_status)(pmbuf->pbuf + pmbuf->data_offset);
	DBG_HEXDUMP(MCMD_D, "EVENT: Ext_Scan_Status", scan_event,
		    pmbuf->data_len);
	status = scan_event->scan_status;
	PRINTM(MEVENT, "ext_scan_status: status %d (scan %s), buf_len %d\n",
	       status, status ? "cancelled" : "success", scan_event->buf_len);

	tlv = (MrvlIEtypesHeader_t *)scan_event->event_buf;
	tlv_buf_left = pmbuf->data_len - sizeof(mlan_event_scan_status);
	while (tlv_buf_left >= sizeof(MrvlIEtypesHeader_t)) {
		tlv_type = wlan_le16_to_cpu(tlv->type);
		tlv_len = wlan_le16_to_cpu(tlv->len);
		if (tlv_buf_left < (tlv_len + sizeof(MrvlIEtypesHeader_t))) {
			PRINTM(MERROR,
			       "Error process scan gap tlv: length %d type 0x%x\n",
			       tlv_len, tlv_type);
			ret = MLAN_STATUS_FAILURE;
			goto done;
		}
		switch (tlv_type) {
		case TLV_TYPE_CHANNEL_STATS:
			tlv_chan_stats = (MrvlIEtypes_ChannelStats_t *)tlv;
			wlan_update_chan_statistics(pmpriv, tlv_chan_stats);
			break;
		default:
			break;
		}
		tlv_buf_left -= tlv_len + sizeof(MrvlIEtypesHeader_t);
		tlv = (MrvlIEtypesHeader_t *)((t_u8 *)tlv + tlv_len +
					      sizeof(MrvlIEtypesHeader_t));
	}

done:
	wlan_request_cmd_lock(pmadapter);
	if (util_peek_list(pmadapter->pmoal_handle, &pmadapter->scan_pending_q,
			   MNULL, MNULL)) {
		/* If firmware not ready, do not issue any more scan
		 * commands */
		if (pmadapter->hw_status != WlanHardwareStatusReady) {
			wlan_release_cmd_lock(pmadapter);
			/* Flush all pending scan commands */
			wlan_flush_scan_queue(pmadapter);
			wlan_request_cmd_lock(pmadapter);
			pmadapter->scan_processing = MFALSE;
			pmadapter->scan_state |= SCAN_STATE_SCAN_COMPLETE;
			pioctl_req = pmadapter->pscan_ioctl_req;
			pmadapter->pscan_ioctl_req = MNULL;
			/* Indicate IOCTL complete */
			if (pioctl_req != MNULL) {
				pioctl_req->status_code =
					MLAN_ERROR_FW_NOT_READY;

				/* Indicate ioctl complete */
				pcb->moal_ioctl_complete(
					pmadapter->pmoal_handle,
					(pmlan_ioctl_req)pioctl_req,
					MLAN_STATUS_FAILURE);
			}
			wlan_release_cmd_lock(pmadapter);
		} else {
			/* Get scan command from scan_pending_q and put
			 * to cmd_pending_q */
			pcmd_node = (cmd_ctrl_node *)util_dequeue_list(
				pmadapter->pmoal_handle,
				&pmadapter->scan_pending_q, MNULL, MNULL);
			wlan_insert_cmd_to_pending_q(pmadapter, pcmd_node,
						     MTRUE);
			wlan_release_cmd_lock(pmadapter);
		}
		LEAVE();
		return ret;
	}
	wlan_release_cmd_lock(pmadapter);

	/* Now we got response from FW, cancel the command timer */
	if (!pmadapter->curr_cmd && pmadapter->cmd_timer_is_set) {
		/* Cancel command timeout timer */
		pcb->moal_stop_timer(pmadapter->pmoal_handle,
				     pmadapter->pmlan_cmd_timer);
		/* Cancel command timeout timer */
		pmadapter->cmd_timer_is_set = MFALSE;
	}
	if (pmadapter->pscan_ioctl_req) {
		if (((mlan_ds_scan *)pmadapter->pscan_ioctl_req->pbuf)
				    ->sub_command ==
			    MLAN_OID_SCAN_SPECIFIC_SSID ||
		    ((mlan_ds_scan *)pmadapter->pscan_ioctl_req->pbuf)
				    ->sub_command ==
			    MLAN_OID_SCAN_USER_CONFIG) {
			if (wlan_active_scan_req_for_passive_chan(
				    pmpriv, pmadapter->pscan_ioctl_req)) {
				LEAVE();
				return ret;
			}
		}
	}
	if ((pmadapter->wifi_6g_scan_split) && (!pmadapter->scan_6g)) {
		if (wlan_scan_6g_network(pmpriv, pmadapter->pscan_ioctl_req)) {
			LEAVE();
			return ret;
		}
	}
	/*
	 * Process the resulting scan table:
	 *   - Remove any bad ssids
	 *   - Update our current BSS information from scan data
	 */
	wlan_scan_process_results(pmpriv);
	/** Complete scan ioctl */
	wlan_request_cmd_lock(pmadapter);
	pmadapter->scan_processing = MFALSE;
	pmadapter->scan_state |= SCAN_STATE_SCAN_COMPLETE;
	pioctl_req = pmadapter->pscan_ioctl_req;
	pmadapter->pscan_ioctl_req = MNULL;
	/* Need to indicate IOCTL complete */
	if (pioctl_req != MNULL) {
		pioctl_req->status_code = MLAN_ERROR_NO_ERROR;
		/* Indicate ioctl complete */
		pcb->moal_ioctl_complete(pmadapter->pmoal_handle, pioctl_req,
					 MLAN_STATUS_SUCCESS);
	}
	wlan_release_cmd_lock(pmadapter);
	wlan_move_cmd_to_cmd_pending_q(pmadapter);
	pmadapter->bgscan_reported = MFALSE;
	if (!status)
		wlan_recv_event(pmpriv, MLAN_EVENT_ID_DRV_SCAN_REPORT, MNULL);
	LEAVE();
	return ret;
}

/**
 *  @brief This function prepares command of bg_scan_query.
 *
 *  @param pmpriv     A pointer to mlan_private structure
 *  @param pcmd       A pointer to HostCmd_DS_COMMAND structure
 *  @param pdata_buf  Void pointer cast of a wlan_scan_cmd_config struct used
 *                    to set the fields/TLVs for the command sent to firmware
 *
 *  @return           MLAN_STATUS_SUCCESS
 */
mlan_status wlan_cmd_802_11_bg_scan_query(mlan_private *pmpriv,
					  HostCmd_DS_COMMAND *pcmd,
					  t_void *pdata_buf)
{
	HostCmd_DS_802_11_BG_SCAN_QUERY *bg_query = &pcmd->params.bg_scan_query;

	ENTER();

	pcmd->command = wlan_cpu_to_le16(HostCmd_CMD_802_11_BG_SCAN_QUERY);
	pcmd->size = wlan_cpu_to_le16(sizeof(HostCmd_DS_802_11_BG_SCAN_QUERY) +
				      S_DS_GEN);

	bg_query->flush = MTRUE;

	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief Create a channel list for the driver to scan based on region info
 *
 *  Use the driver region/band information to construct a comprehensive list
 *    of channels to scan.  This routine is used for any scan that is not
 *    provided a specific channel list to scan.
 *
 *  @param pmpriv           A pointer to mlan_private structure
 *  @param pbg_scan_in      pointer to scan configuration parameters
 *  @param tlv_chan_list    A pointer to structure
 * MrvlIEtypes_ChanListParamSet_t
 *
 *  @return                 channel number
 */
static t_u8 wlan_bgscan_create_channel_list(
	mlan_private *pmpriv, const wlan_bgscan_cfg *pbg_scan_in,
	MrvlIEtypes_ChanListParamSet_t *tlv_chan_list, t_u8 max_bgscan_chan)
{
	mlan_adapter *pmadapter = pmpriv->adapter;
	region_chan_t *pscan_region;
	chan_freq_power_t *cfp;
	t_u32 region_idx;
	t_u32 chan_idx = 0;
	t_u32 next_chan;
	t_u8 scan_type;
	t_u8 radio_type;
	t_u16 band;

	ENTER();

	for (region_idx = 0; region_idx < NELEMENTS(pmadapter->region_channel);
	     region_idx++) {
		if (wlan_11d_is_enabled(pmpriv) &&
		    pmpriv->media_connected != MTRUE) {
			/* Scan all the supported chan for the first scan */
			if (!pmadapter->universal_channel[region_idx].valid)
				continue;
			pscan_region =
				&pmadapter->universal_channel[region_idx];
		} else {
			if (!pmadapter->region_channel[region_idx].valid)
				continue;
			pscan_region = &pmadapter->region_channel[region_idx];
		}

		if (pbg_scan_in && !pbg_scan_in->chan_list[0].chan_number &&
		    pbg_scan_in->chan_list[0].radio_type & BAND_SPECIFIED) {
			radio_type = pbg_scan_in->chan_list[0].radio_type &
				     ~BAND_SPECIFIED;
			if (!radio_type && (pscan_region->band != BAND_B) &&
			    (pscan_region->band != BAND_G))
				continue;
			if (radio_type && (pscan_region->band != BAND_A))
				continue;
		}
		band = pmpriv->config_bands;
		if (!wlan_is_band_compatible(band, pscan_region->band))
			continue;
		for (next_chan = 0; next_chan < pscan_region->num_cfp;
		     next_chan++, chan_idx++) {
			if (chan_idx >= max_bgscan_chan)
				break;
			/*
			 * Set the default scan type to ACTIVE SCAN type, will
			 * later be changed to passive on a per channel basis
			 * if restricted by regulatory requirements (11d or 11h)
			 */
			scan_type = MLAN_SCAN_TYPE_ACTIVE;
			cfp = pscan_region->pcfp + next_chan;

			switch (pscan_region->band) {
			case BAND_A:
				tlv_chan_list->chan_scan_param[chan_idx]
					.bandcfg.chanBand = BAND_5GHZ;
				/* Passive scan on DFS channels */
				if (wlan_11h_radar_detect_required(
					    pmpriv, (t_u8)cfp->channel))
					scan_type = MLAN_SCAN_TYPE_PASSIVE;
				break;
			case BAND_6G:
				tlv_chan_list->chan_scan_param[chan_idx]
					.bandcfg.chanBand = BAND_6GHZ;
				break;
			case BAND_B:
			case BAND_G:
				if (wlan_bg_scan_type_is_passive(
					    pmpriv, (t_u8)cfp->channel))
					scan_type = MLAN_SCAN_TYPE_PASSIVE;
				tlv_chan_list->chan_scan_param[chan_idx]
					.bandcfg.chanBand = BAND_2GHZ;
				break;
			default:
				tlv_chan_list->chan_scan_param[chan_idx]
					.bandcfg.chanBand = BAND_2GHZ;
				break;
			}

			if (pbg_scan_in &&
			    pbg_scan_in->chan_list[0].scan_time) {
				tlv_chan_list->chan_scan_param[chan_idx]
					.max_scan_time = wlan_cpu_to_le16(
					(t_u16)pbg_scan_in->chan_list[0]
						.scan_time);
				tlv_chan_list->chan_scan_param[chan_idx]
					.min_scan_time = wlan_cpu_to_le16(
					(t_u16)pbg_scan_in->chan_list[0]
						.scan_time);
			} else if (scan_type == MLAN_SCAN_TYPE_PASSIVE) {
				tlv_chan_list->chan_scan_param[chan_idx]
					.max_scan_time = wlan_cpu_to_le16(
					pmadapter->passive_scan_time);
				tlv_chan_list->chan_scan_param[chan_idx]
					.min_scan_time = wlan_cpu_to_le16(
					pmadapter->passive_scan_time);
			} else {
				tlv_chan_list->chan_scan_param[chan_idx]
					.max_scan_time = wlan_cpu_to_le16(
					pmadapter->specific_scan_time);
				tlv_chan_list->chan_scan_param[chan_idx]
					.min_scan_time = wlan_cpu_to_le16(
					pmadapter->specific_scan_time);
			}

			if (scan_type == MLAN_SCAN_TYPE_PASSIVE) {
				tlv_chan_list->chan_scan_param[chan_idx]
					.chan_scan_mode.passive_scan = MTRUE;
			} else {
				tlv_chan_list->chan_scan_param[chan_idx]
					.chan_scan_mode.passive_scan = MFALSE;
			}

			tlv_chan_list->chan_scan_param[chan_idx].chan_number =
				(t_u8)cfp->channel;
		}
	}

	LEAVE();
	return chan_idx;
}

/**
 *  @brief This function prepares command of bg_scan_config
 *
 *  @param pmpriv     A pointer to mlan_private structure
 *  @param pcmd       A pointer to HostCmd_DS_COMMAND structure
 *  @param pdata_buf  Void pointer cast of a wlan_scan_cmd_config struct used
 *                    to set the fields/TLVs for the command sent to firmware
 *
 *  @return           MLAN_STATUS_SUCCESS
 */
mlan_status wlan_cmd_bgscan_config(mlan_private *pmpriv,
				   HostCmd_DS_COMMAND *pcmd, t_void *pdata_buf)
{
	mlan_adapter *pmadapter = pmpriv->adapter;
	HostCmd_DS_802_11_BG_SCAN_CONFIG *bg_scan =
		&pcmd->params.bg_scan_config;
	wlan_bgscan_cfg *bg_scan_in = (wlan_bgscan_cfg *)pdata_buf;
	t_u16 cmd_size = 0;
	MrvlIEtypes_NumProbes_t *pnum_probes_tlv = MNULL;
	MrvlIEtypes_BeaconLowRssiThreshold_t *rssi_tlv = MNULL;
	MrvlIEtypes_BeaconLowSnrThreshold_t *snr_tlv = MNULL;
	MrvlIEtypes_WildCardSsIdParamSet_t *pwildcard_ssid_tlv = MNULL;
	MrvlIEtypes_ChanListParamSet_t *tlv_chan_list = MNULL;
	MrvlIEtypes_StartLater_t *tlv_start_later = MNULL;
	MrvlIEtypes_RepeatCount_t *tlv_repeat = MNULL;
	MrvlIEtypes_EESParamSet_t *tlv_ees_cfg = MNULL;
	MrvlIEtype_EESNetworkCfg_t *tlv_ees_net_cfg = MNULL;
	MrvlIEtypes_Cipher_t *tlv_ees_cipher = MNULL;
	MrvlIEtypes_SsIdParamSet_t *tlv_ssid = MNULL;
	MrvlIETypes_HTCap_t *pht_cap = MNULL;
	MrvlIETypes_VHTCap_t *pvht_cap = MNULL;
	MrvlIEtypes_Extension_t *phe_cap = MNULL;
	MrvlIEtypes_ScanChanGap_t *pscan_gap_tlv;
	t_u16 len = 0;

	t_u8 index;
	t_u8 *tlv = MNULL;
	t_u16 num_probes = 0;
	t_u32 ssid_idx;
	t_u32 ssid_len = 0;
	t_u32 chan_idx;
	t_u32 chan_num;
	t_u8 radio_type;
	t_u16 scan_dur;
	t_u8 scan_type;
	t_u16 band;
	t_u8 max_bgscan_chan = 0;
	t_u16 scan_chan_gap = 0;
	const t_u8 zero_mac[6] = {0, 0, 0, 0, 0, 0};

	ENTER();

	max_bgscan_chan = WLAN_BG_SCAN_CHAN_MAX;

#if defined(SD9177)
	if (IS_CARD9177(pmadapter->card_type))
		max_bgscan_chan = WLAN_BG_SCAN_CHAN_MAX_UNII_4;
#endif

	if (IS_FW_SUPPORT_6G(pmadapter))
		max_bgscan_chan = WLAN_BG_SCAN_CHAN_MAX_6E;
	PRINTM(MCMND, "max_bgscan_chan=%d\n", max_bgscan_chan);

	pcmd->command = wlan_cpu_to_le16(HostCmd_CMD_802_11_BG_SCAN_CONFIG);
	bg_scan->action = wlan_cpu_to_le16(bg_scan_in->action);
	bg_scan->enable = bg_scan_in->enable;
	bg_scan->bss_type = bg_scan_in->bss_type;
	bg_scan->dot11ai = bg_scan_in->dot11ai;
	cmd_size = sizeof(HostCmd_DS_802_11_BG_SCAN_CONFIG) + S_DS_GEN;
	if (bg_scan_in->scan_interval)
		bg_scan->scan_interval =
			wlan_cpu_to_le32(bg_scan_in->scan_interval);
	else
		bg_scan->scan_interval =
			wlan_cpu_to_le32(DEFAULT_BGSCAN_INTERVAL);
	bg_scan->report_condition =
		wlan_cpu_to_le32(bg_scan_in->report_condition);

	if ((bg_scan_in->action == BG_SCAN_ACT_GET) ||
	    (bg_scan_in->action == BG_SCAN_ACT_GET_PPS_UAPSD) ||
	    (!bg_scan->enable))
		goto done;

	tlv = (t_u8 *)&pcmd->params + sizeof(HostCmd_DS_802_11_BG_SCAN_CONFIG);
	num_probes = (bg_scan_in->num_probes ? bg_scan_in->num_probes :
					       pmadapter->scan_probes);
	if (num_probes) {
		pnum_probes_tlv = (MrvlIEtypes_NumProbes_t *)tlv;
		pnum_probes_tlv->header.type =
			wlan_cpu_to_le16(TLV_TYPE_NUMPROBES);
		pnum_probes_tlv->header.len =
			wlan_cpu_to_le16(sizeof(pnum_probes_tlv->num_probes));
		pnum_probes_tlv->num_probes =
			wlan_cpu_to_le16((t_u16)num_probes);
		tlv += sizeof(MrvlIEtypes_NumProbes_t);
		cmd_size += sizeof(MrvlIEtypes_NumProbes_t);
	}
	if (bg_scan_in->rssi_threshold) {
		rssi_tlv = (MrvlIEtypes_BeaconLowRssiThreshold_t *)tlv;
		rssi_tlv->header.type = wlan_cpu_to_le16(TLV_TYPE_RSSI_LOW);
		rssi_tlv->header.len = wlan_cpu_to_le16(
			sizeof(MrvlIEtypes_BeaconLowRssiThreshold_t) -
			sizeof(MrvlIEtypesHeader_t));
		rssi_tlv->value = bg_scan_in->rssi_threshold;
		rssi_tlv->frequency = 0;
		tlv += sizeof(MrvlIEtypes_BeaconLowRssiThreshold_t);
		cmd_size += sizeof(MrvlIEtypes_BeaconLowRssiThreshold_t);
	}
	if (bg_scan_in->snr_threshold) {
		snr_tlv = (MrvlIEtypes_BeaconLowSnrThreshold_t *)tlv;
		snr_tlv->header.type = wlan_cpu_to_le16(TLV_TYPE_SNR_LOW);
		snr_tlv->header.len = wlan_cpu_to_le16(
			sizeof(MrvlIEtypes_BeaconLowSnrThreshold_t) -
			sizeof(MrvlIEtypesHeader_t));
		snr_tlv->value = bg_scan_in->snr_threshold;
		snr_tlv->frequency = 0;
		tlv += sizeof(MrvlIEtypes_BeaconLowRssiThreshold_t);
		cmd_size += sizeof(MrvlIEtypes_BeaconLowRssiThreshold_t);
	}
	if (bg_scan_in->repeat_count) {
		tlv_repeat = (MrvlIEtypes_RepeatCount_t *)tlv;
		tlv_repeat->header.type =
			wlan_cpu_to_le16(TLV_TYPE_REPEAT_COUNT);
		tlv_repeat->header.len =
			wlan_cpu_to_le16(sizeof(MrvlIEtypes_RepeatCount_t) -
					 sizeof(MrvlIEtypesHeader_t));
		tlv_repeat->repeat_count =
			wlan_cpu_to_le16(bg_scan_in->repeat_count);
		tlv += sizeof(MrvlIEtypes_RepeatCount_t);
		cmd_size += sizeof(MrvlIEtypes_RepeatCount_t);
	}
	scan_chan_gap = (bg_scan_in->scan_chan_gap ? bg_scan_in->scan_chan_gap :
						     pmadapter->scan_chan_gap);
	if (scan_chan_gap) {
		pscan_gap_tlv = (MrvlIEtypes_ScanChanGap_t *)tlv;
		PRINTM(MCMND, "bgScan: channel gap = 0x%x\n", scan_chan_gap);
		pscan_gap_tlv->header.type =
			wlan_cpu_to_le16(TLV_TYPE_SCAN_CHANNEL_GAP);
		pscan_gap_tlv->header.len = sizeof(pscan_gap_tlv->gap);
		pscan_gap_tlv->gap = wlan_cpu_to_le16((t_u16)scan_chan_gap);
		/** indicate FW, gap is optional */
		pscan_gap_tlv->gap |= GAP_FLAG_OPTIONAL;
		tlv += sizeof(pscan_gap_tlv->header) +
		       pscan_gap_tlv->header.len;
		cmd_size += sizeof(MrvlIEtypes_ScanChanGap_t);
	}
	for (ssid_idx = 0; ((ssid_idx < NELEMENTS(bg_scan_in->ssid_list)) &&
			    (*bg_scan_in->ssid_list[ssid_idx].ssid ||
			     bg_scan_in->ssid_list[ssid_idx].max_len));
	     ssid_idx++) {
		ssid_len = wlan_strlen(
			(char *)bg_scan_in->ssid_list[ssid_idx].ssid);
		pwildcard_ssid_tlv = (MrvlIEtypes_WildCardSsIdParamSet_t *)tlv;
		pwildcard_ssid_tlv->header.type =
			wlan_cpu_to_le16(TLV_TYPE_WILDCARDSSID);
		pwildcard_ssid_tlv->header.len =
			(t_u16)(ssid_len +
				sizeof(pwildcard_ssid_tlv->max_ssid_length));
		pwildcard_ssid_tlv->max_ssid_length =
			bg_scan_in->ssid_list[ssid_idx].max_len;
		memcpy_ext(pmadapter, pwildcard_ssid_tlv->ssid,
			   bg_scan_in->ssid_list[ssid_idx].ssid, ssid_len,
			   MLAN_MAX_SSID_LENGTH);
		tlv += sizeof(pwildcard_ssid_tlv->header) +
		       pwildcard_ssid_tlv->header.len;
		cmd_size += sizeof(pwildcard_ssid_tlv->header) +
			    pwildcard_ssid_tlv->header.len;
		pwildcard_ssid_tlv->header.len =
			wlan_cpu_to_le16(pwildcard_ssid_tlv->header.len);
		PRINTM(MCMD_D, "bgScan: ssid_list[%d]: %s, %d\n", ssid_idx,
		       pwildcard_ssid_tlv->ssid,
		       pwildcard_ssid_tlv->max_ssid_length);
	}
	if (bg_scan_in->chan_list[0].chan_number) {
		tlv_chan_list = (MrvlIEtypes_ChanListParamSet_t *)tlv;
		PRINTM(MINFO, "Scan: Using supplied channel list\n");
		chan_num = 0;
		for (chan_idx = 0; chan_idx < max_bgscan_chan &&
				   bg_scan_in->chan_list[chan_idx].chan_number;
		     chan_idx++) {
			radio_type = bg_scan_in->chan_list[chan_idx].radio_type;
			band = pmpriv->config_bands;
			if (!wlan_is_band_compatible(
				    band, radio_type_to_band(radio_type)))
				continue;
			scan_type = bg_scan_in->chan_list[chan_idx].scan_type;
			/* Prevent active scanning on a radar controlled channel
			 */
			if (radio_type == BAND_5GHZ) {
				if (wlan_11h_radar_detect_required(
					    pmpriv,
					    bg_scan_in->chan_list[chan_idx]
						    .chan_number)) {
					scan_type = MLAN_SCAN_TYPE_PASSIVE;
				}
			}
			if (radio_type == BAND_2GHZ) {
				if (wlan_bg_scan_type_is_passive(
					    pmpriv,
					    bg_scan_in->chan_list[chan_idx]
						    .chan_number)) {
					scan_type = MLAN_SCAN_TYPE_PASSIVE;
				}
			}
			tlv_chan_list->chan_scan_param[chan_num].chan_number =
				bg_scan_in->chan_list[chan_idx].chan_number;
			tlv_chan_list->chan_scan_param[chan_num]
				.bandcfg.chanBand =
				bg_scan_in->chan_list[chan_idx].radio_type;

			if (scan_type == MLAN_SCAN_TYPE_PASSIVE) {
				tlv_chan_list->chan_scan_param[chan_num]
					.chan_scan_mode.passive_scan = MTRUE;
			} else {
				tlv_chan_list->chan_scan_param[chan_num]
					.chan_scan_mode.passive_scan = MFALSE;
			}
			if (bg_scan_in->chan_list[chan_idx].scan_time) {
				scan_dur =
					(t_u16)bg_scan_in->chan_list[chan_idx]
						.scan_time;
			} else {
				if (scan_type == MLAN_SCAN_TYPE_PASSIVE) {
					scan_dur = pmadapter->passive_scan_time;
				} else {
					scan_dur =
						pmadapter->specific_scan_time;
				}
			}
			PRINTM(MCMD_D,
			       "BGScan: Chan(%3d), bandcfg(%x), Mode(%d), Dur(%d)\n",
			       tlv_chan_list->chan_scan_param[chan_num]
				       .chan_number,
			       tlv_chan_list->chan_scan_param[chan_num].bandcfg,
			       tlv_chan_list->chan_scan_param[chan_num]
				       .chan_scan_mode.passive_scan,
			       scan_dur);

			if (tlv_chan_list->chan_scan_param[chan_num]
				    .chan_scan_mode.passive_scan)
				tlv_chan_list->chan_scan_param[chan_num]
					.chan_scan_mode.passive_to_active_scan =
					MTRUE;

			tlv_chan_list->chan_scan_param[chan_num].min_scan_time =
				wlan_cpu_to_le16(scan_dur);
			tlv_chan_list->chan_scan_param[chan_num].max_scan_time =
				wlan_cpu_to_le16(scan_dur);
			chan_num++;
		}
		tlv_chan_list->header.type =
			wlan_cpu_to_le16(TLV_TYPE_CHANLIST);
		tlv_chan_list->header.len =
			wlan_cpu_to_le16(sizeof(ChanScanParamSet_t) * chan_num);
		tlv += sizeof(MrvlIEtypesHeader_t) +
		       sizeof(ChanScanParamSet_t) * chan_num;
		cmd_size += sizeof(MrvlIEtypesHeader_t) +
			    sizeof(ChanScanParamSet_t) * chan_num;
	} else {
		tlv_chan_list = (MrvlIEtypes_ChanListParamSet_t *)tlv;
		chan_num = wlan_bgscan_create_channel_list(
			pmpriv, bg_scan_in, tlv_chan_list, max_bgscan_chan);
		tlv_chan_list->header.type =
			wlan_cpu_to_le16(TLV_TYPE_CHANLIST);
		tlv_chan_list->header.len =
			wlan_cpu_to_le16(sizeof(ChanScanParamSet_t) * chan_num);
		tlv += sizeof(MrvlIEtypesHeader_t) +
		       sizeof(ChanScanParamSet_t) * chan_num;
		cmd_size += sizeof(MrvlIEtypesHeader_t) +
			    sizeof(ChanScanParamSet_t) * chan_num;
	}
	if (bg_scan_in->chan_per_scan) {
		bg_scan->chan_per_scan =
			MIN(max_bgscan_chan, bg_scan_in->chan_per_scan);
	} else {
		if (bg_scan_in->report_condition & BG_SCAN_WAIT_ALL_CHAN_DONE)
			bg_scan->chan_per_scan = chan_num;
		else
			bg_scan->chan_per_scan =
				MRVDRV_MAX_CHANNELS_PER_SPECIFIC_SCAN;
	}
	if (ISSUPP_11NENABLED(pmpriv->adapter->fw_cap_info) &&
	    (pmpriv->config_bands & BAND_GN ||
	     pmpriv->config_bands & BAND_AN)) {
		pht_cap = (MrvlIETypes_HTCap_t *)tlv;
		memset(pmadapter, pht_cap, 0, sizeof(MrvlIETypes_HTCap_t));
		pht_cap->header.type = wlan_cpu_to_le16(HT_CAPABILITY);
		pht_cap->header.len = sizeof(HTCap_t);
		wlan_fill_ht_cap_tlv(pmpriv, pht_cap, pmpriv->config_bands,
				     MTRUE);
		DBG_HEXDUMP(MCMD_D, "BGSCAN: HT_CAPABILITIES IE",
			    (t_u8 *)pht_cap, sizeof(MrvlIETypes_HTCap_t));
		tlv += sizeof(MrvlIETypes_HTCap_t);
		cmd_size += sizeof(MrvlIETypes_HTCap_t);
		pht_cap->header.len = wlan_cpu_to_le16(pht_cap->header.len);
	}
	if (ISSUPP_11ACENABLED(pmpriv->adapter->fw_cap_info) &&
	    (pmpriv->config_bands & BAND_AAC)) {
		pvht_cap = (MrvlIETypes_VHTCap_t *)tlv;
		memset(pmadapter, pvht_cap, 0, sizeof(MrvlIETypes_VHTCap_t));
		pvht_cap->header.type = wlan_cpu_to_le16(VHT_CAPABILITY);
		pvht_cap->header.len = sizeof(VHT_capa_t);
		wlan_fill_vht_cap_tlv(pmpriv, pvht_cap, pmpriv->config_bands,
				      MFALSE, MFALSE);
		DBG_HEXDUMP(MCMD_D, "BGSCAN: VHT_CAPABILITIES IE",
			    (t_u8 *)pvht_cap, sizeof(MrvlIETypes_VHTCap_t));
		tlv += sizeof(MrvlIETypes_VHTCap_t);
		cmd_size += sizeof(MrvlIETypes_VHTCap_t);
		pvht_cap->header.len = wlan_cpu_to_le16(pvht_cap->header.len);
	}

	if (IS_FW_SUPPORT_11AX(pmadapter) &&
	    ((pmpriv->config_bands & BAND_GAX) ||
	     (pmpriv->config_bands & BAND_AAX))) {
		phe_cap = (MrvlIEtypes_Extension_t *)tlv;
		len = wlan_fill_he_cap_tlv(pmpriv, pmpriv->config_bands,
					   phe_cap, MFALSE);
		DBG_HEXDUMP(MCMD_D, "BGSCAN: HE_CAPABILITIES IE",
			    (t_u8 *)phe_cap, len);
		tlv += len;
		cmd_size += len;
	}
	if (wlan_is_ext_capa_support(pmpriv)) {
		wlan_add_ext_capa_info_ie(pmpriv, MNULL, &tlv);
		cmd_size += sizeof(MrvlIETypes_ExtCap_t);
	}
	if (pmpriv->adapter->ecsa_enable) {
		t_u8 bandwidth = BW_20MHZ;
		t_u8 oper_class = 1;
		t_u32 usr_dot_11n_dev_cap;
		if (pmpriv->media_connected) {
			if (pmpriv->config_bands & BAND_A)
				usr_dot_11n_dev_cap =
					pmpriv->usr_dot_11n_dev_cap_a;
			else
				usr_dot_11n_dev_cap =
					pmpriv->usr_dot_11n_dev_cap_bg;
			if (usr_dot_11n_dev_cap & MBIT(17)) {
				bandwidth = BW_40MHZ;
				if (ISSUPP_11ACENABLED(
					    pmadapter->fw_cap_info) &&
				    (pmpriv->config_bands & BAND_AAC))
					bandwidth = BW_80MHZ;
			}
			wlan_get_curr_oper_class(
				pmpriv,
				pmpriv->curr_bss_params.bss_descriptor.channel,
				bandwidth, &oper_class);
		}
		len = wlan_add_supported_oper_class_ie(pmpriv, &tlv,
						       oper_class);
		cmd_size += len;
	}

	tlv_start_later = (MrvlIEtypes_StartLater_t *)tlv;
	tlv_start_later->header.type =
		wlan_cpu_to_le16(TLV_TYPE_STARTBGSCANLATER);
	tlv_start_later->header.len = wlan_cpu_to_le16(
		sizeof(MrvlIEtypes_StartLater_t) - sizeof(MrvlIEtypesHeader_t));
	tlv_start_later->value = wlan_cpu_to_le16(bg_scan_in->start_later);
	tlv += sizeof(MrvlIEtypes_StartLater_t);
	cmd_size += sizeof(MrvlIEtypes_StartLater_t);

	if (bg_scan_in->config_ees) {
		/* Fill EES configuration */
		tlv_ees_cfg = (MrvlIEtypes_EESParamSet_t *)tlv;
		tlv_ees_cfg->header.type = wlan_cpu_to_le16(TLV_TYPE_EES_CFG);
		tlv_ees_cfg->header.len =
			wlan_cpu_to_le16(sizeof(MrvlIEtypes_EESParamSet_t) -
					 sizeof(MrvlIEtypesHeader_t));
		tlv_ees_cfg->ees_mode = wlan_cpu_to_le16(bg_scan_in->ees_mode);
		tlv_ees_cfg->report_cond =
			wlan_cpu_to_le16(bg_scan_in->report_cond);
		tlv_ees_cfg->high_period =
			wlan_cpu_to_le16(bg_scan_in->high_period);
		tlv_ees_cfg->high_period_count =
			wlan_cpu_to_le16(bg_scan_in->high_period_count);
		tlv_ees_cfg->mid_period =
			wlan_cpu_to_le16(bg_scan_in->mid_period);
		tlv_ees_cfg->mid_period_count =
			wlan_cpu_to_le16(bg_scan_in->mid_period_count);
		tlv_ees_cfg->low_period =
			wlan_cpu_to_le16(bg_scan_in->low_period);
		tlv_ees_cfg->low_period_count =
			wlan_cpu_to_le16(bg_scan_in->low_period_count);
		tlv += sizeof(MrvlIEtypes_EESParamSet_t);
		cmd_size += sizeof(MrvlIEtypes_EESParamSet_t);

		if (bg_scan_in->network_count) {
			/* Fill EES network configuration */
			tlv_ees_net_cfg = (MrvlIEtype_EESNetworkCfg_t *)tlv;
			tlv_ees_net_cfg->header.type =
				wlan_cpu_to_le16(TLV_TYPE_EES_NET_CFG);
			tlv_ees_net_cfg->header.len = wlan_cpu_to_le16(
				sizeof(MrvlIEtype_EESNetworkCfg_t) -
				sizeof(MrvlIEtypesHeader_t));
			tlv_ees_net_cfg->network_count =
				bg_scan_in->network_count;
			tlv_ees_net_cfg->max_conn_count =
				bg_scan_in->max_conn_count;
			tlv_ees_net_cfg->black_list_exp =
				bg_scan_in->black_list_exp;
			tlv += sizeof(MrvlIEtype_EESNetworkCfg_t);
			cmd_size += sizeof(MrvlIEtype_EESNetworkCfg_t);
			for (index = 0; index < bg_scan_in->network_count;
			     index++) {
				if (wlan_strlen((char *)bg_scan_in
							->ees_ssid_cfg[index]
							.ssid)) {
					/* Fill SSID settings */
					tlv_ssid =
						(MrvlIEtypes_SsIdParamSet_t *)
							tlv;
					tlv_ssid->header.type =
						wlan_cpu_to_le16(TLV_TYPE_SSID);
					tlv_ssid->header.len = wlan_cpu_to_le16(
						(t_u16)bg_scan_in
							->ees_ssid_cfg[index]
							.max_len);
					memcpy_ext(
						pmadapter, tlv_ssid->ssid,
						bg_scan_in->ees_ssid_cfg[index]
							.ssid,
						bg_scan_in->ees_ssid_cfg[index]
							.max_len,
						MLAN_MAX_SSID_LENGTH);
					tlv += sizeof(MrvlIEtypesHeader_t) +
					       tlv_ssid->header.len;
					cmd_size +=
						sizeof(MrvlIEtypesHeader_t) +
						tlv_ssid->header.len;
				} else {
					/* Fill Wildcard SSID settings */
					pwildcard_ssid_tlv =
						(MrvlIEtypes_WildCardSsIdParamSet_t
							 *)tlv;
					pwildcard_ssid_tlv->header.type =
						wlan_cpu_to_le16(
							TLV_TYPE_WILDCARDSSID);
					pwildcard_ssid_tlv->header
						.len = wlan_cpu_to_le16(
						sizeof(MrvlIEtypes_WildCardSsIdParamSet_t) -
						sizeof(MrvlIEtypesHeader_t));
					pwildcard_ssid_tlv->max_ssid_length =
						MLAN_MAX_SSID_LENGTH;
					tlv += sizeof(MrvlIEtypesHeader_t) +
					       sizeof(pwildcard_ssid_tlv
							      ->max_ssid_length);
					cmd_size +=
						sizeof(MrvlIEtypesHeader_t) +
						sizeof(pwildcard_ssid_tlv
							       ->max_ssid_length);
				}
				/* Fill Cipher settings */
				tlv_ees_cipher = (MrvlIEtypes_Cipher_t *)tlv;
				tlv_ees_cipher->header.type =
					wlan_cpu_to_le16(TLV_TYPE_CIPHER);
				tlv_ees_cipher->header.len = wlan_cpu_to_le16(
					sizeof(MrvlIEtypes_Cipher_t) -
					sizeof(MrvlIEtypesHeader_t));
				tlv_ees_cipher->pair_cipher =
					bg_scan_in->ees_ssid_cfg[index]
						.pair_cipher;
				tlv_ees_cipher->group_cipher =
					bg_scan_in->ees_ssid_cfg[index]
						.group_cipher;
				tlv += sizeof(MrvlIEtypes_Cipher_t);
				cmd_size += sizeof(MrvlIEtypes_Cipher_t);
			}
		}
	}

	if (memcmp(pmadapter, bg_scan_in->random_mac, zero_mac,
		   MLAN_MAC_ADDR_LENGTH)) {
		MrvlIEtypes_MacAddr_t *randomMacParam =
			(MrvlIEtypes_MacAddr_t *)tlv;
		memset(pmadapter, randomMacParam, 0,
		       sizeof(MrvlIEtypes_MacAddr_t));
		randomMacParam->header.type =
			wlan_cpu_to_le16(TLV_TYPE_RANDOM_MAC);
		randomMacParam->header.len =
			wlan_cpu_to_le16(MLAN_MAC_ADDR_LENGTH);
		memcpy_ext(pmadapter, randomMacParam->mac,
			   bg_scan_in->random_mac, MLAN_MAC_ADDR_LENGTH,
			   MLAN_MAC_ADDR_LENGTH);
		tlv += sizeof(MrvlIEtypes_MacAddr_t);
		cmd_size += sizeof(MrvlIEtypes_MacAddr_t);
	}
done:
	pcmd->size = wlan_cpu_to_le16(cmd_size);
	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function handles the command response of extended scan
 *
 *  @param pmpriv       A pointer to mlan_private structure
 *  @param resp         A pointer to HostCmd_DS_COMMAND
 *  @param pioctl_buf   A pointer to mlan_ioctl_req structure
 *
 *  @return             MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status wlan_ret_bgscan_config(mlan_private *pmpriv,
				   HostCmd_DS_COMMAND *resp,
				   mlan_ioctl_req *pioctl_buf)
{
	mlan_ds_scan *pscan = MNULL;
	HostCmd_DS_802_11_BG_SCAN_CONFIG *bg_scan =
		&resp->params.bg_scan_config;
	wlan_bgscan_cfg *bg_scan_out = MNULL;

	ENTER();
	if (pioctl_buf) {
		pscan = (mlan_ds_scan *)pioctl_buf->pbuf;
		bg_scan_out =
			(wlan_bgscan_cfg *)pscan->param.user_scan.scan_cfg_buf;
		bg_scan_out->action = wlan_le16_to_cpu(bg_scan->action);
		if ((bg_scan_out->action == BG_SCAN_ACT_GET) ||
		    (bg_scan_out->action == BG_SCAN_ACT_GET_PPS_UAPSD)) {
			bg_scan_out->enable = bg_scan->enable;
			bg_scan_out->bss_type = bg_scan->bss_type;
			bg_scan_out->chan_per_scan = bg_scan->chan_per_scan;
			bg_scan_out->scan_interval =
				wlan_le32_to_cpu(bg_scan->scan_interval);
			bg_scan_out->report_condition =
				wlan_le32_to_cpu(bg_scan->report_condition);
			pioctl_buf->data_read_written =
				sizeof(mlan_ds_scan) + MLAN_SUB_COMMAND_SIZE;
		}
	}
	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function handles the command response of bgscan_query
 *  @param pmpriv       A pointer to mlan_private structure
 *  @param resp         A pointer to HostCmd_DS_COMMAND
 *  @param pioctl_buf   A pointer to mlan_ioctl_req structure
 *
 *  @return             MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status wlan_ret_802_11_bgscan_query(mlan_private *pmpriv,
					 HostCmd_DS_COMMAND *resp,
					 mlan_ioctl_req *pioctl_buf)
{
	mlan_ds_scan *pscan = MNULL;
	mlan_adapter *pmadapter = pmpriv->adapter;
	t_u8 i;
	ENTER();
	for (i = 0; i < pmadapter->num_in_chan_stats; i++)
		pmadapter->pchan_stats[i].cca_scan_duration = 0;
	pmadapter->idx_chan_stats = 0;

	wlan_ret_802_11_scan(pmpriv, resp, MNULL);
	if (pioctl_buf) {
		pscan = (mlan_ds_scan *)pioctl_buf->pbuf;
		pscan->param.scan_resp.pscan_table =
			(t_u8 *)pmadapter->pscan_table;
		pscan->param.scan_resp.num_in_scan_table =
			pmadapter->num_in_scan_table;
		pscan->param.scan_resp.age_in_secs = pmadapter->age_in_secs;
		pscan->param.scan_resp.pchan_stats =
			(t_u8 *)pmadapter->pchan_stats;
		pscan->param.scan_resp.num_in_chan_stats =
			pmadapter->num_in_chan_stats;

		pioctl_buf->data_read_written =
			sizeof(mlan_scan_resp) + MLAN_SUB_COMMAND_SIZE;
	}
	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function finds ssid in ssid list.
 *
 *  @param pmpriv       A pointer to mlan_private structure
 *  @param ssid         SSID to find in the list
 *  @param bssid        BSSID to qualify the SSID selection (if provided)
 *  @param mode         Network mode: Infrastructure or IBSS
 *
 *  @return             index in BSSID list or < 0 if error
 */
t_s32 wlan_find_ssid_in_list(mlan_private *pmpriv, mlan_802_11_ssid *ssid,
			     t_u8 *bssid, t_u32 mode)
{
	mlan_adapter *pmadapter = pmpriv->adapter;
	t_s32 net = -1;
	t_s32 j;
	t_u8 best_rssi = 0;
	t_u32 i;

	ENTER();
	PRINTM(MINFO, "Num of entries in scan table = %d\n",
	       pmadapter->num_in_scan_table);

	/*
	 * Loop through the table until the maximum is reached or until a match
	 *   is found based on the bssid field comparison
	 */
	for (i = 0;
	     i < pmadapter->num_in_scan_table && (!bssid || (bssid && net < 0));
	     i++) {
		if (!wlan_ssid_cmp(pmadapter, &pmadapter->pscan_table[i].ssid,
				   ssid) &&
		    (!bssid ||
		     !memcmp(pmadapter, pmadapter->pscan_table[i].mac_address,
			     bssid, MLAN_MAC_ADDR_LENGTH))) {
			if ((mode == MLAN_BSS_MODE_INFRA) &&
			    !wlan_is_band_compatible(
				    pmpriv->config_bands,
				    pmadapter->pscan_table[i].bss_band))
				continue;

			switch (mode) {
			case MLAN_BSS_MODE_INFRA:
				j = wlan_is_network_compatible(pmpriv, i, mode);

				if (j >= 0) {
					if (SCAN_RSSI(pmadapter->pscan_table[i]
							      .rssi) >
					    best_rssi) {
						best_rssi = SCAN_RSSI(
							pmadapter
								->pscan_table[i]
								.rssi);
						net = i;
					}
				} else {
					if (net == -1)
						net = j;
				}
				break;
			case MLAN_BSS_MODE_AUTO:
			default:
				/*
				 * Do not check compatibility if the mode
				 * requested is Auto/Unknown.  Allows generic
				 * find to work without verifying against the
				 * Adapter security settings
				 */
				if (SCAN_RSSI(pmadapter->pscan_table[i].rssi) >
				    best_rssi) {
					best_rssi = SCAN_RSSI(
						pmadapter->pscan_table[i].rssi);
					net = i;
				}
				break;
			}
		}
	}

	LEAVE();
	return net;
}

/**
 *  @brief This function finds a specific compatible BSSID in the scan list
 *
 *  @param pmpriv       A pointer to mlan_private structure
 *  @param bssid        BSSID to find in the scan list
 *  @param mode         Network mode: Infrastructure or IBSS
 *
 *  @return             index in BSSID list or < 0 if error
 */
t_s32 wlan_find_bssid_in_list(mlan_private *pmpriv, t_u8 *bssid, t_u32 mode)
{
	mlan_adapter *pmadapter = pmpriv->adapter;
	t_s32 net = -1;
	t_u32 i;

	ENTER();

	if (!bssid) {
		LEAVE();
		return -1;
	}

	PRINTM(MINFO, "FindBSSID: Num of BSSIDs = %d\n",
	       pmadapter->num_in_scan_table);

	/*
	 * Look through the scan table for a compatible match. The ret return
	 *   variable will be equal to the index in the scan table (greater
	 *   than zero) if the network is compatible.  The loop will continue
	 *   past a matched bssid that is not compatible in case there is an
	 *   AP with multiple SSIDs assigned to the same BSSID
	 */
	for (i = 0; net < 0 && i < pmadapter->num_in_scan_table; i++) {
		if (!memcmp(pmadapter, pmadapter->pscan_table[i].mac_address,
			    bssid, MLAN_MAC_ADDR_LENGTH)) {
			if ((mode == MLAN_BSS_MODE_INFRA) &&
			    !wlan_is_band_compatible(
				    pmpriv->config_bands,
				    pmadapter->pscan_table[i].bss_band))
				continue;
			switch (mode) {
			case MLAN_BSS_MODE_INFRA:
				net = wlan_is_network_compatible(pmpriv, i,
								 mode);
				break;
			default:
				net = i;
				break;
			}
		}
	}

	LEAVE();
	return net;
}

/**
 *  @brief Compare two SSIDs
 *
 *  @param pmadapter A pointer to mlan_adapter structure
 *  @param ssid1     A pointer to ssid to compare
 *  @param ssid2     A pointer to ssid to compare
 *
 *  @return         0--ssid is same, otherwise is different
 */
t_s32 wlan_ssid_cmp(pmlan_adapter pmadapter, mlan_802_11_ssid *ssid1,
		    mlan_802_11_ssid *ssid2)
{
	ENTER();

	if (!ssid1 || !ssid2) {
		LEAVE();
		return -1;
	}

	if (ssid1->ssid_len != ssid2->ssid_len) {
		LEAVE();
		return -1;
	}

	LEAVE();
	return memcmp(pmadapter, ssid1->ssid, ssid2->ssid, ssid1->ssid_len);
}

/**
 *  @brief Find the AP with specific ssid in the scan list
 *
 *  @param pmpriv               A pointer to mlan_private structure
 *  @param preq_ssid_bssid      A pointer to AP's ssid returned
 *
 *  @return                     MLAN_STATUS_SUCCESS--success, otherwise--fail
 */
mlan_status wlan_find_best_network(mlan_private *pmpriv,
				   mlan_ssid_bssid *preq_ssid_bssid)
{
	mlan_adapter *pmadapter = pmpriv->adapter;
	mlan_status ret = MLAN_STATUS_SUCCESS;
	BSSDescriptor_t *preq_bss;
	t_s32 i;

	ENTER();

	memset(pmadapter, preq_ssid_bssid, 0, sizeof(mlan_ssid_bssid));

	if (!pmadapter->pscan_table) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	i = wlan_find_best_network_in_list(pmpriv);

	if (i >= 0) {
		preq_bss = &pmadapter->pscan_table[i];
		memcpy_ext(pmadapter, &preq_ssid_bssid->ssid, &preq_bss->ssid,
			   sizeof(mlan_802_11_ssid),
			   sizeof(preq_ssid_bssid->ssid));
		memcpy_ext(pmadapter, (t_u8 *)&preq_ssid_bssid->bssid,
			   (t_u8 *)&preq_bss->mac_address, MLAN_MAC_ADDR_LENGTH,
			   MLAN_MAC_ADDR_LENGTH);

		/* Make sure we are in the right mode */
		if (pmpriv->bss_mode == MLAN_BSS_MODE_AUTO)
			pmpriv->bss_mode = preq_bss->bss_mode;
		preq_ssid_bssid->channel = (t_u16)preq_bss->channel;
		if (preq_bss->pmd_ie &&
		    wlan_ft_akm_is_used(pmpriv, (t_u8 *)preq_bss->prsn_ie)) {
			preq_ssid_bssid->ft_md = preq_bss->pmd_ie->mdid;
			preq_ssid_bssid->ft_cap = preq_bss->pmd_ie->ft_cap;
		}
		preq_ssid_bssid->bss_band = preq_bss->bss_band;
		preq_ssid_bssid->idx = i + 1;
	}

	if (!preq_ssid_bssid->ssid.ssid_len) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	PRINTM(MINFO,
	       "Best network found = [%s], "
	       "[" MACSTR "]\n",
	       preq_ssid_bssid->ssid.ssid, MAC2STR(preq_ssid_bssid->bssid));

done:
	LEAVE();
	return ret;
}

/**
 *  @brief Send a scan command for all available channels filtered on a spec
 *
 *  @param pmpriv       A pointer to mlan_private structure
 *  @param pioctl_buf   A pointer to MLAN IOCTL Request buffer
 *  @param preq_ssid    A pointer to AP's ssid returned
 *
 *  @return             MLAN_STATUS_SUCCESS--success, otherwise--fail
 */
mlan_status wlan_scan_specific_ssid(mlan_private *pmpriv, t_void *pioctl_buf,
				    mlan_802_11_ssid *preq_ssid)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_callbacks *pcb = (mlan_callbacks *)&pmpriv->adapter->callbacks;
	wlan_user_scan_cfg *pscan_cfg;
	pmlan_ioctl_req pioctl_req = (mlan_ioctl_req *)pioctl_buf;

	ENTER();

	if (!preq_ssid) {
		if (pioctl_req)
			pioctl_req->status_code = MLAN_ERROR_CMD_SCAN_FAIL;
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	if (!pmpriv->adapter->scan_6g)
		wlan_scan_delete_ssid_table_entry(pmpriv, preq_ssid);

	ret = pcb->moal_malloc(pmpriv->adapter->pmoal_handle,
			       sizeof(wlan_user_scan_cfg),
			       MLAN_MEM_DEF | MLAN_MEM_FLAG_ATOMIC,
			       (t_u8 **)&pscan_cfg);

	if (ret != MLAN_STATUS_SUCCESS || !pscan_cfg) {
		PRINTM(MERROR, "Memory allocation for pscan_cfg failed!\n");
		if (pioctl_req)
			pioctl_req->status_code = MLAN_ERROR_NO_MEM;
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	memcpy_ext(pmpriv->adapter, pscan_cfg->ssid_list[0].ssid,
		   preq_ssid->ssid, preq_ssid->ssid_len, MLAN_MAX_SSID_LENGTH);
	pscan_cfg->keep_previous_scan = MFALSE;

	ret = wlan_scan_networks(pmpriv, pioctl_buf, pscan_cfg);

	if (pscan_cfg)
		pcb->moal_mfree(pmpriv->adapter->pmoal_handle,
				(t_u8 *)pscan_cfg);

done:
	LEAVE();
	return ret;
}

/**
 *  @brief Save a beacon buffer of the current bss descriptor
 *  Save the current beacon buffer to restore in the following cases that
 *  makes the bcn_buf not to contain the current ssid's beacon buffer.
 *    - the current ssid was not found somehow in the last scan.
 *    - the current ssid was the last entry of the scan table and overloaded.
 *
 *  @param pmpriv       A pointer to mlan_private structure
 *
 *  @return             N/A
 */
t_void wlan_save_curr_bcn(mlan_private *pmpriv)
{
	mlan_adapter *pmadapter = pmpriv->adapter;
	mlan_callbacks *pcb = (pmlan_callbacks)&pmadapter->callbacks;
	BSSDescriptor_t *pcurr_bss = &pmpriv->curr_bss_params.bss_descriptor;
	mlan_status ret = MLAN_STATUS_SUCCESS;

	ENTER();
	pcb->moal_spin_lock(pmadapter->pmoal_handle, pmpriv->curr_bcn_buf_lock);
	/* save the beacon buffer if it is not saved or updated */
	if ((pmpriv->pcurr_bcn_buf == MNULL) ||
	    (pmpriv->curr_bcn_size != pcurr_bss->beacon_buf_size) ||
	    (memcmp(pmpriv->adapter, pmpriv->pcurr_bcn_buf,
		    pcurr_bss->pbeacon_buf, pcurr_bss->beacon_buf_size))) {
		if (pmpriv->pcurr_bcn_buf) {
			pcb->moal_mfree(pmadapter->pmoal_handle,
					pmpriv->pcurr_bcn_buf);
			pmpriv->pcurr_bcn_buf = MNULL;
		}
		pmpriv->curr_bcn_size = pcurr_bss->beacon_buf_size;

		if (pmpriv->curr_bcn_size) {
			ret = pcb->moal_malloc(pmadapter->pmoal_handle,
					       pcurr_bss->beacon_buf_size,
					       MLAN_MEM_DEF |
						       MLAN_MEM_FLAG_ATOMIC,
					       &pmpriv->pcurr_bcn_buf);

			if ((ret == MLAN_STATUS_SUCCESS) &&
			    pmpriv->pcurr_bcn_buf) {
				memcpy_ext(pmpriv->adapter,
					   pmpriv->pcurr_bcn_buf,
					   pcurr_bss->pbeacon_buf,
					   pcurr_bss->beacon_buf_size,
					   pcurr_bss->beacon_buf_size);
				PRINTM(MINFO, "current beacon saved %d\n",
				       pmpriv->curr_bcn_size);
			} else {
				PRINTM(MERROR,
				       "Fail to allocate curr_bcn_buf\n");
			}
		}
	}
	wlan_update_curr_bcn(pmpriv);
	pcb->moal_spin_unlock(pmadapter->pmoal_handle,
			      pmpriv->curr_bcn_buf_lock);

	LEAVE();
}

/**
 *  @brief Free a beacon buffer of the current bss descriptor
 *
 *  @param pmpriv       A pointer to mlan_private structure
 *
 *  @return             N/A
 */
t_void wlan_free_curr_bcn(mlan_private *pmpriv)
{
	mlan_adapter *pmadapter = pmpriv->adapter;
	mlan_callbacks *pcb = (pmlan_callbacks)&pmadapter->callbacks;

	ENTER();
	if (pmpriv->pcurr_bcn_buf) {
		pcb->moal_mfree(pmadapter->pmoal_handle, pmpriv->pcurr_bcn_buf);
		pmpriv->pcurr_bcn_buf = MNULL;
	}
	LEAVE();
}
