/** @file mlan_11h.c
 *
 *  @brief This file contains functions for 802.11H.
 *
 *
 *  Copyright 2008-2021, 2024-2025 NXP
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

/*************************************************************
Change Log:
    03/26/2009: initial version
************************************************************/

#include "mlan.h"
#include "mlan_join.h"
#include "mlan_util.h"
#include "mlan_fw.h"
#include "mlan_main.h"
#include "mlan_ioctl.h"
#include "mlan_11h.h"
#include "mlan_11n.h"
#ifdef UAP_SUPPORT
#include "mlan_uap.h"
#endif

/********************************************************
			Local Variables
********************************************************/

/** Default IBSS DFS recovery interval (in TBTTs); used for adhoc start */
#define WLAN_11H_DEFAULT_DFS_RECOVERY_INTERVAL 100

/** Default 11h power constraint used to offset the maximum transmit power */
#define WLAN_11H_TPC_POWERCONSTRAINT 0

/** 11h TPC Power capability minimum setting, sent in TPC_INFO command to fw */
#define WLAN_11H_TPC_POWERCAPABILITY_MIN 5

/** 11h TPC Power capability maximum setting, sent in TPC_INFO command to fw */
#define WLAN_11H_TPC_POWERCAPABILITY_MAX 20

/** Regulatory requirement for the duration of a channel availability check */
#define WLAN_11H_CHANNEL_AVAIL_CHECK_DURATION 60000 /* in ms */

/** Starting Frequency for 11A band */
#define START_FREQ_11A_BAND 5000 /* in MHz */

/** DFS Channel Move Time */
#define DFS_CHAN_MOVE_TIME 10 /* in sec */

/** Regulatory requirement for the duration of a non-occupancy period */
#define WLAN_11H_NON_OCCUPANCY_PERIOD 1800 /* in sec (30mins) */

/** Maximum allowable age (seconds) on DFS report data */
#define MAX_DFS_REPORT_USABLE_AGE_SEC (120) /* 2 minutes */

/** Minimum delay for CHAN_SW IE to broadcast by FW */
#define MIN_RDH_CHAN_SW_IE_PERIOD_MSEC (400) /* 4 beacons @ 100ms */

/** Maximum delay for CHAN_SW IE to broadcast by FW */
#define MAX_RDH_CHAN_SW_IE_PERIOD_MSEC (3000) /* 5 beacons @ 600ms */

/** Maximum retries on selecting new random channel */
#define MAX_RANDOM_CHANNEL_RETRIES (20)

/** Maximum retries on selecting new random non-dfs channel */
#define MAX_SWITCH_CHANNEL_RETRIES (30)

/** Value for undetermined priv_curr_idx on first entry to new RDH stage */
#define RDH_STAGE_FIRST_ENTRY_PRIV_IDX (0xff)

/** Region codes 0x10, 0x20:  channels 1 thru 11 supported */
static const IEEEtypes_SupportChan_Subband_t wlan_11h_2_4G_region_FCC = {1, 11};

/** Region codes 0x30, 0x32, 0x41, 0x50:  channels 1 thru 13 supported */
static const IEEEtypes_SupportChan_Subband_t wlan_11h_2_4G_region_EU = {1, 13};

/** Region code 0x40:  only channel 14 supported */
static const IEEEtypes_SupportChan_Subband_t wlan_11h_2_4G_region_JPN40 = {14,
									   1};

/** JPN sub-band config : Start Channel = 8, NumChans = 3 */
static const IEEEtypes_SupportChan_Subband_t wlan_11h_JPN_bottom_band = {8, 3};

/** U-NII sub-band config : Start Channel = 36, NumChans = 4 */
static const IEEEtypes_SupportChan_Subband_t wlan_11h_unii_lower_band = {36, 4};

/** U-NII sub-band config : Start Channel = 52, NumChans = 4 */
static const IEEEtypes_SupportChan_Subband_t wlan_11h_unii_middle_band = {52,
									  4};

/** U-NII sub-band config : Start Channel = 100, NumChans = 11 */
static const IEEEtypes_SupportChan_Subband_t wlan_11h_unii_mid_upper_band = {
	100, 11};

/** U-NII sub-band config : Start Channel = 100, NumChans = 5 */
static const IEEEtypes_SupportChan_Subband_t wlan_11h_unii_mid_upper_band_0 = {
	100, 5};

/** U-NII sub-band config : Start Channel = 132, NumChans = 3 */
static const IEEEtypes_SupportChan_Subband_t wlan_11h_unii_mid_upper_band_1 = {
	132, 3};

/** U-NII sub-band config : Start Channel = 149, NumChans = 5 */
static const IEEEtypes_SupportChan_Subband_t wlan_11h_unii_upper_band = {149,
									 5};
/** U-NII sub-band config : Start Channel = 169, NumChans = 3 */
static const IEEEtypes_SupportChan_Subband_t wlan_11h_unii_4_band = {169, 3};

/** Internally passed structure used to send a CMD_802_11_TPC_INFO command */
typedef struct {
	t_u8 chan; /**< Channel to which the power constraint applies */
	t_u8 power_constraint; /**< Local power constraint to send to firmware
				*/
} wlan_11h_tpc_info_param_t;

/********************************************************
			Global Variables
********************************************************/

/********************************************************
			Local Functions
********************************************************/

/**
 *  @brief Utility function to get a random number based on the underlying OS
 *
 *  @param pmadapter Pointer to mlan_adapter
 *  @return random integer
 */
static t_u32 wlan_11h_get_random_num(pmlan_adapter pmadapter)
{
	t_u32 sec, usec;

	ENTER();
	pmadapter->callbacks.moal_get_system_time(pmadapter->pmoal_handle, &sec,
						  &usec);
	sec = (sec & 0xFFFF) + (sec >> 16);
	usec = (usec & 0xFFFF) + (usec >> 16);

	LEAVE();
	return (usec << 16) | sec;
}

/**
 *  @brief find all bonded channel.
 *
 *  @param pri_chan   primary channel
 *  @param bw         channel bandwidth
 *  @param chan_list  buffer to return channel list.
 *
 *  @return           number of channel
 */
static t_u8 woal_get_bonded_channels(t_u8 pri_chan, t_u8 bw, t_u8 *chan_list)
{
	t_u8 ht40_plus[] = {52, 60, 100, 108, 116, 124, 132, 140};
	t_u8 ht40_minus[] = {56, 64, 104, 112, 120, 128, 136, 144};
	t_u8 vht80_dfs[4][4] = {{52, 56, 60, 64},
				{100, 104, 108, 112},
				{116, 120, 124, 128},
				{132, 136, 140, 144}};
	t_u8 find = MFALSE;
	int j;
	int i;
	t_u8 sec_chan = 0;
	t_u8 n_chan = 1;
	ENTER();

	if (bw == CHAN_BW_20MHZ) {
		chan_list[0] = pri_chan;
	} else if (bw == CHAN_BW_40MHZ) {
		chan_list[0] = pri_chan;
		for (i = 0; i < sizeof(ht40_minus); i++) {
			if (pri_chan == (t_u8)ht40_plus[i]) {
				sec_chan = pri_chan + 4;
				n_chan = 2;
				break;
			}
		}
		for (i = 0; i < sizeof(ht40_minus); i++) {
			if (pri_chan == (t_u8)ht40_minus[i]) {
				sec_chan = pri_chan - 4;
				n_chan = 2;
				break;
			}
		}
		chan_list[1] = sec_chan;
	} else if (bw == CHAN_BW_80MHZ) {
		for (i = 0; i < 4; i++) {
			for (j = 0; j < 4; j++) {
				if (pri_chan == (t_u8)vht80_dfs[i][j]) {
					find = MTRUE;
					break;
				}
			}
			if (find)
				break;
		}
		if (find) {
			n_chan = 4;
			for (j = 0; j < n_chan; j++) {
				chan_list[j] = (t_u8)vht80_dfs[i][j];
			}
		}
	}
	LEAVE();
	return n_chan;
}

/**
 *  @brief Set channel's dfs state
 *
 *  @param priv         Private driver information structure
 *  @param chan         primary channel
 *  @param bw           channel bandwidth
 *  @param dfs_state    dfs state
 *
 *  @return  N/A
 */
static t_void wlan_11h_set_chan_dfs_state(mlan_private *priv, t_u8 chan,
					  t_u8 bw, dfs_state_t dfs_state)
{
	t_u8 n_chan;
	t_u8 chan_list[4] = {0};
	t_u8 i;
	n_chan = woal_get_bonded_channels(chan, bw, chan_list);
	for (i = 0; i < n_chan; i++)
		wlan_set_chan_dfs_state(priv, BAND_A, chan_list[i], dfs_state);
}

/**
 *  @brief reset dfs_checking_chan's dfs state
 *
 *  @param priv         Private driver information structure
 *  @param dfs_state    dfs state
 *
 *  @return  N/A
 */
t_void wlan_11h_reset_dfs_checking_chan_dfs_state(mlan_private *priv,
						  dfs_state_t dfs_state)
{
	wlan_dfs_device_state_t *pstate_dfs = &priv->adapter->state_dfs;
	dfs_state_t state;
	ENTER();
	if (pstate_dfs->dfs_check_channel) {
		state = wlan_get_chan_dfs_state(priv, BAND_A,
						pstate_dfs->dfs_check_channel);
		if (state == DFS_AVAILABLE)
			wlan_11h_set_chan_dfs_state(
				priv, pstate_dfs->dfs_check_channel,
				pstate_dfs->dfs_check_bandwidth, dfs_state);
	}
	LEAVE();
}

/**
 *  @brief Setup the Supported Channel IE sent in association requests
 *
 *  The Supported Channels IE is required to be sent when the spectrum
 *    management capability (11h) is enabled.  The element contains a
 *    starting channel and number of channels tuple for each sub-band
 *    the STA supports.  This information is based on the operating region.
 *
 *  @param priv      Private driver information structure
 *  @param band      Band in use
 *  @param psup_chan Output parameter: Pointer to the Supported Chan element
 *                   setup by this function.
 *
 *  @return
 *    - Length of the returned element in psup_chan output parameter
 *    - 0 if returned element is not setup
 */
static t_u16
wlan_11h_set_supp_channels_ie(mlan_private *priv, t_u16 band,
			      IEEEtypes_SupportedChannels_t *psup_chan)
{
	t_u16 num_subbands = 0;
	t_u16 ret_len = 0;
	t_u8 cfp_bg, cfp_a;

	ENTER();
	memset(priv->adapter, psup_chan, 0x00,
	       sizeof(IEEEtypes_SupportedChannels_t));

	cfp_bg = cfp_a = priv->adapter->region_code;
	if (!priv->adapter->region_code) {
		/* Invalid region code, use CFP code */
		cfp_bg = priv->adapter->cfp_code_bg;
		cfp_a = priv->adapter->cfp_code_a;
	}

	if ((band & BAND_B) || (band & BAND_G)) {
		/*
		 * Channels are contiguous in 2.4GHz, usually only one subband.
		 */
		switch (cfp_bg) {
		case 0x10: /* USA FCC   */
		case 0x20: /* Canada IC */
		default:
			psup_chan->subband[num_subbands++] =
				wlan_11h_2_4G_region_FCC;
			break;
		case 0x30: /* Europe ETSI */
		case 0x41: /* Japan  */
		case 0x50: /* China  */
			psup_chan->subband[num_subbands++] =
				wlan_11h_2_4G_region_EU;
			break;
		case 0x40: /* Japan  */
			psup_chan->subband[num_subbands++] =
				wlan_11h_2_4G_region_JPN40;
			break;
		case 0xff: /* Japan special */
			psup_chan->subband[num_subbands++] =
				wlan_11h_2_4G_region_EU;
			psup_chan->subband[num_subbands++] =
				wlan_11h_2_4G_region_JPN40;
			break;
		}
	} else if (band & BAND_A) {
		/*
		 * Set the supported channel elements based on the region code,
		 * incrementing num_subbands for each sub-band we append to the
		 * element.
		 */
		switch (cfp_a) {
		case 0x10: /* USA FCC   */
			psup_chan->subband[num_subbands++] =
				wlan_11h_unii_lower_band;
			psup_chan->subband[num_subbands++] =
				wlan_11h_unii_middle_band;
			psup_chan->subband[num_subbands++] =
				wlan_11h_unii_mid_upper_band;
			psup_chan->subband[num_subbands++] =
				wlan_11h_unii_upper_band;
			psup_chan->subband[num_subbands++] =
				wlan_11h_unii_4_band;
			break;
		case 0x20: /* Canada IC */
		case 0x30: /* Europe ETSI */
		default:
			psup_chan->subband[num_subbands++] =
				wlan_11h_unii_lower_band;
			psup_chan->subband[num_subbands++] =
				wlan_11h_unii_middle_band;
			psup_chan->subband[num_subbands++] =
				wlan_11h_unii_mid_upper_band;
			psup_chan->subband[num_subbands++] =
				wlan_11h_unii_upper_band;
			break;
		case 0x50: /* China */
			psup_chan->subband[num_subbands++] =
				wlan_11h_unii_lower_band;
			psup_chan->subband[num_subbands++] =
				wlan_11h_unii_middle_band;
			psup_chan->subband[num_subbands++] =
				wlan_11h_unii_upper_band;
			break;
		case 0x40: /* Japan */
		case 0x41: /* Japan */
		case 0xff: /* Japan special */
			psup_chan->subband[num_subbands++] =
				wlan_11h_JPN_bottom_band;
			psup_chan->subband[num_subbands++] =
				wlan_11h_unii_lower_band;
			psup_chan->subband[num_subbands++] =
				wlan_11h_unii_middle_band;
			psup_chan->subband[num_subbands++] =
				wlan_11h_unii_mid_upper_band;
			break;
		case 0x1: /* Low band (5150-5250 MHz) channels */
			psup_chan->subband[num_subbands++] =
				wlan_11h_unii_lower_band;
			break;
		case 0x2: /* Lower middle band (5250-5350 MHz) channels */
			psup_chan->subband[num_subbands++] =
				wlan_11h_unii_middle_band;
			break;
		case 0x3: /* Upper middle band (5470-5725 MHz) channels */
			psup_chan->subband[num_subbands++] =
				wlan_11h_unii_mid_upper_band;
			break;
		case 0x4: /* High band (5725-5850 MHz) channels */
			psup_chan->subband[num_subbands++] =
				wlan_11h_unii_upper_band;
			break;
		case 0x5: /* Low band (5150-5250 MHz) and High band (5725-5850
			     MHz) channels */
			psup_chan->subband[num_subbands++] =
				wlan_11h_unii_lower_band;
			psup_chan->subband[num_subbands++] =
				wlan_11h_unii_upper_band;
			break;
		case 0x6: /* Low band (5150-5250 MHz) and Lower middle band
			     (5250-5350 MHz) and High band (5725-5850 MHz)
			     channels */
			psup_chan->subband[num_subbands++] =
				wlan_11h_unii_lower_band;
			psup_chan->subband[num_subbands++] =
				wlan_11h_unii_middle_band;
			psup_chan->subband[num_subbands++] =
				wlan_11h_unii_upper_band;
			break;
		case 0x7:
			/* 36-48 */
			psup_chan->subband[num_subbands++] =
				wlan_11h_unii_lower_band;
			/* 52-64 */
			psup_chan->subband[num_subbands++] =
				wlan_11h_unii_middle_band;
			/* 100-116 */
			psup_chan->subband[num_subbands++] =
				wlan_11h_unii_mid_upper_band_0;
			/* 132-140 */
			psup_chan->subband[num_subbands++] =
				wlan_11h_unii_mid_upper_band_1;
			/* 149-165 */
			psup_chan->subband[num_subbands++] =
				wlan_11h_unii_upper_band;
			break;
		}
	}

	/*
	 * If we have setup any supported subbands in the element, return a
	 *    valid IE along with its size, else return 0.
	 */
	if (num_subbands) {
		psup_chan->element_id = SUPPORTED_CHANNELS;
		psup_chan->len =
			num_subbands * sizeof(IEEEtypes_SupportChan_Subband_t);

		ret_len = (t_u16)(psup_chan->len + sizeof(psup_chan->len) +
				  sizeof(psup_chan->element_id));

		HEXDUMP("11h: SupChan", (t_u8 *)psup_chan, ret_len);
	}

	LEAVE();
	return ret_len;
}

/**
 *  @brief Prepare CMD_802_11_TPC_ADAPT_REQ firmware command
 *
 *  @param priv      Private driver information structure
 *  @param pcmd_ptr  Output parameter: Pointer to the command being prepared
 *                   for the firmware
 *  @param pinfo_buf HostCmd_DS_802_11_TPC_ADAPT_REQ passed as void data block
 *
 *  @return          MLAN_STATUS_SUCCESS
 */
static mlan_status wlan_11h_cmd_tpc_request(mlan_private *priv,
					    HostCmd_DS_COMMAND *pcmd_ptr,
					    t_void *pinfo_buf)
{
	ENTER();

	memcpy_ext(priv->adapter, &pcmd_ptr->params.tpc_req, pinfo_buf,
		   sizeof(HostCmd_DS_802_11_TPC_ADAPT_REQ),
		   sizeof(HostCmd_DS_802_11_TPC_ADAPT_REQ));

	pcmd_ptr->params.tpc_req.req.timeout =
		wlan_cpu_to_le16(pcmd_ptr->params.tpc_req.req.timeout);

	/* Converted to little endian in wlan_11h_cmd_process */
	pcmd_ptr->size = sizeof(HostCmd_DS_802_11_TPC_ADAPT_REQ) + S_DS_GEN;

	HEXDUMP("11h: 11_TPC_ADAPT_REQ:", (t_u8 *)pcmd_ptr,
		(t_u32)pcmd_ptr->size);

	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief Prepare CMD_802_11_TPC_INFO firmware command
 *
 *  @param priv      Private driver information structure
 *  @param pcmd_ptr  Output parameter: Pointer to the command being prepared
 *                   for the firmware
 *  @param pinfo_buf wlan_11h_tpc_info_param_t passed as void data block
 *
 *  @return          MLAN_STATUS_SUCCESS
 */
static mlan_status wlan_11h_cmd_tpc_info(mlan_private *priv,
					 HostCmd_DS_COMMAND *pcmd_ptr,
					 t_void *pinfo_buf)
{
	HostCmd_DS_802_11_TPC_INFO *ptpc_info = &pcmd_ptr->params.tpc_info;
	MrvlIEtypes_LocalPowerConstraint_t *pconstraint =
		&ptpc_info->local_constraint;
	MrvlIEtypes_PowerCapability_t *pcap = &ptpc_info->power_cap;

	wlan_11h_device_state_t *pstate = &priv->adapter->state_11h;
	const wlan_11h_tpc_info_param_t *ptpc_info_param =
		(wlan_11h_tpc_info_param_t *)pinfo_buf;

	ENTER();

	pcap->min_power = pstate->min_tx_power_capability;
	pcap->max_power = pstate->max_tx_power_capability;
	pcap->header.len = wlan_cpu_to_le16(2);
	pcap->header.type = wlan_cpu_to_le16(TLV_TYPE_POWER_CAPABILITY);

	pconstraint->chan = ptpc_info_param->chan;
	pconstraint->constraint = ptpc_info_param->power_constraint;
	pconstraint->header.type = wlan_cpu_to_le16(TLV_TYPE_POWER_CONSTRAINT);
	pconstraint->header.len = wlan_cpu_to_le16(2);

	/* Converted to little endian in wlan_11h_cmd_process */
	pcmd_ptr->size = sizeof(HostCmd_DS_802_11_TPC_INFO) + S_DS_GEN;

	HEXDUMP("11h: TPC INFO", (t_u8 *)pcmd_ptr, (t_u32)pcmd_ptr->size);

	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief  Prepare CMD_802_11_CHAN_SW_ANN firmware command
 *
 *  @param priv      Private driver information structure
 *  @param pcmd_ptr  Output parameter: Pointer to the command being
 *                   prepared to for firmware
 *  @param pinfo_buf HostCmd_DS_802_11_CHAN_SW_ANN passed as void data block
 *
 *  @return          MLAN_STATUS_SUCCESS
 */
static mlan_status wlan_11h_cmd_chan_sw_ann(mlan_private *priv,
					    HostCmd_DS_COMMAND *pcmd_ptr,
					    t_void *pinfo_buf)
{
	const HostCmd_DS_802_11_CHAN_SW_ANN *pch_sw_ann =
		(HostCmd_DS_802_11_CHAN_SW_ANN *)pinfo_buf;

	ENTER();

	/* Converted to little endian in wlan_11h_cmd_process */
	pcmd_ptr->size = sizeof(HostCmd_DS_802_11_CHAN_SW_ANN) + S_DS_GEN;

	memcpy_ext(priv->adapter, &pcmd_ptr->params.chan_sw_ann, pch_sw_ann,
		   sizeof(HostCmd_DS_802_11_CHAN_SW_ANN),
		   sizeof(HostCmd_DS_802_11_CHAN_SW_ANN));

	PRINTM(MINFO, "11h: ChSwAnn: %#x-%u, Seq=%u, Ret=%u\n",
	       pcmd_ptr->command, pcmd_ptr->size, pcmd_ptr->seq_num,
	       pcmd_ptr->result);
	PRINTM(MINFO, "11h: ChSwAnn: Ch=%d, Cnt=%d, Mode=%d\n",
	       pch_sw_ann->new_chan, pch_sw_ann->switch_count,
	       pch_sw_ann->switch_mode);

	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief  Prepare CMD_CHAN_REPORT_REQUEST firmware command
 *
 *  @param priv      Private driver information structure
 *  @param pcmd_ptr  Output parameter: Pointer to the command being
 *                   prepared to for firmware
 *  @param pinfo_buf HostCmd_DS_CHAN_RPT_REQ passed as void data block
 *
 *  @return          MLAN_STATUS_SUCCESS or MLAN_STATUS_PENDING
 */
static mlan_status wlan_11h_cmd_chan_rpt_req(mlan_private *priv,
					     HostCmd_DS_COMMAND *pcmd_ptr,
					     t_void *pinfo_buf)
{
	HostCmd_DS_CHAN_RPT_REQ *pchan_rpt_req =
		(HostCmd_DS_CHAN_RPT_REQ *)pinfo_buf;
	wlan_dfs_device_state_t *pstate_dfs = &priv->adapter->state_dfs;
	MrvlIEtypes_ChanRpt11hBasic_t *ptlv_basic;
	t_bool is_cancel_req = MFALSE;
	MrvlIEtypes_ZeroDfsOperation_t *ptlv_zero_dfs;
	t_u8 dfs53cfg = priv->adapter->dfs53cfg;
	MrvlIEtypes_DfsW53Cfg_t *ptlv_dfs53cfg;

	ENTER();

	/*
	 * pchan_rpt_req->millisec_dwell_time would be zero if the chan_rpt_req
	 * is to cancel current ongoing report
	 */
	if (pchan_rpt_req->millisec_dwell_time == 0)
		is_cancel_req = MTRUE;

	if (pstate_dfs->dfs_check_pending && !is_cancel_req &&
	    priv->bss_type != MLAN_BSS_TYPE_DFS) {
		PRINTM(MERROR,
		       "11h: ChanRptReq - previous CMD_CHAN_REPORT_REQUEST has"
		       " not returned its result yet (as EVENT_CHANNEL_READY)."
		       "  This command will be dropped.\n");
		LEAVE();
		return MLAN_STATUS_PENDING;
	}

	/* Converted to little endian in wlan_11h_cmd_process */
	pcmd_ptr->size = sizeof(HostCmd_DS_CHAN_RPT_REQ) + S_DS_GEN;

	memcpy_ext(priv->adapter, &pcmd_ptr->params.chan_rpt_req, pchan_rpt_req,
		   sizeof(HostCmd_DS_CHAN_RPT_REQ),
		   sizeof(HostCmd_DS_CHAN_RPT_REQ));
	pcmd_ptr->params.chan_rpt_req.chan_desc.startFreq =
		wlan_cpu_to_le16(pchan_rpt_req->chan_desc.startFreq);
	pcmd_ptr->params.chan_rpt_req.millisec_dwell_time =
		wlan_cpu_to_le32(pchan_rpt_req->millisec_dwell_time);

	/* if DFS channel, add BASIC report TLV, and set radar bit */
	if (!is_cancel_req && wlan_11h_radar_detect_required(
				      priv, pchan_rpt_req->chan_desc.chanNum)) {
		ptlv_basic =
			(MrvlIEtypes_ChanRpt11hBasic_t *)(((t_u8 *)(pcmd_ptr)) +
							  pcmd_ptr->size);
		ptlv_basic->Header.type =
			wlan_cpu_to_le16(TLV_TYPE_CHANRPT_11H_BASIC);
		ptlv_basic->Header.len =
			wlan_cpu_to_le16(sizeof(MeasRptBasicMap_t));
		memset(priv->adapter, &ptlv_basic->map, 0,
		       sizeof(MeasRptBasicMap_t));
		ptlv_basic->map.radar = 1;
		pcmd_ptr->size += sizeof(MrvlIEtypes_ChanRpt11hBasic_t);
	}

	if ((priv->adapter->region_code == COUNTRY_CODE_JP_40 ||
	     priv->adapter->region_code == COUNTRY_CODE_JP_FF) &&
	    (dfs53cfg != DFS_W53_DEFAULT_FW)) {
		ptlv_dfs53cfg =
			(MrvlIEtypes_DfsW53Cfg_t *)(((t_u8 *)(pcmd_ptr)) +
						    pcmd_ptr->size);
		ptlv_dfs53cfg->Header.type =
			wlan_cpu_to_le16(TLV_TYPE_DFS_W53_CFG);
		ptlv_dfs53cfg->Header.len = wlan_cpu_to_le16(sizeof(t_u8));
		ptlv_dfs53cfg->dfs53cfg = dfs53cfg;
		pcmd_ptr->size += sizeof(MrvlIEtypes_DfsW53Cfg_t);
	}

	if (priv->bss_type == MLAN_BSS_TYPE_DFS) {
		memcpy_ext(priv->adapter, &priv->chan_rep_req, pchan_rpt_req,
			   sizeof(mlan_ds_11h_chan_rep_req),
			   sizeof(priv->chan_rep_req));
		ptlv_zero_dfs =
			(MrvlIEtypes_ZeroDfsOperation_t *)(((t_u8 *)(pcmd_ptr)) +
							   pcmd_ptr->size);
		ptlv_zero_dfs->Header.type =
			wlan_cpu_to_le16(TLV_TYPE_ZERO_DFS_OPERATION);
		ptlv_zero_dfs->Header.len = wlan_cpu_to_le16(sizeof(t_u8));
		if (!is_cancel_req) {
			ptlv_zero_dfs->zero_dfs_enbl = MTRUE;
			PRINTM(MCMND, "DFS: START: chan=%d bw=%d\n",
			       pchan_rpt_req->chan_desc.chanNum,
			       pchan_rpt_req->chan_desc.bandcfg.chanWidth);
		} else {
			ptlv_zero_dfs->zero_dfs_enbl = MFALSE;
			PRINTM(MCMND, "DFS: STOP\n");
		}
		pcmd_ptr->size += sizeof(MrvlIEtypes_ZeroDfsOperation_t);
		LEAVE();
		return MLAN_STATUS_SUCCESS;
	}

	/* update dfs sturcture.
	 * dfs_check_pending is set when we receive CMD_RESP == SUCCESS */
	pstate_dfs->dfs_check_pending = MFALSE;
	pstate_dfs->dfs_radar_found = MFALSE;
	pstate_dfs->dfs_check_priv = MNULL;
	if (!is_cancel_req) {
		pstate_dfs->dfs_check_channel =
			pchan_rpt_req->chan_desc.chanNum;
		pstate_dfs->dfs_check_bandwidth =
			pchan_rpt_req->chan_desc.bandcfg.chanWidth;
	}

	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief Set the local power capability and constraint TLV
 *
 *  @param ppbuffer                The buffer to add these two TLVs
 *  @param channel                 Channel to which the power constraint applies
 *  @param power_constraint        Power constraint to be applied on the channel
 *  @param min_tx_power_capability Min. Tx Power in Power Capability IE
 *  @param max_tx_power_capability Max. Tx Power in Power Capability IE
 *
 *  @return                        The len increased
 */
static t_u32 wlan_11h_set_local_power_constraint_tlv(
	t_u8 **ppbuffer, t_u8 channel, t_u8 power_constraint,
	t_u8 min_tx_power_capability, t_u8 max_tx_power_capability)
{
	MrvlIEtypes_PowerCapability_t *pcap;
	MrvlIEtypes_LocalPowerConstraint_t *pconstraint;
	t_u8 *start_ptr = MNULL;
	t_u32 ret_len = 0;

	ENTER();

	/* Null Checks */
	if ((ppbuffer == MNULL) || (((t_u8 *)(*ppbuffer)) == MNULL)) {
		LEAVE();
		return 0;
	}

	start_ptr = (t_u8 *)(*ppbuffer);

	PRINTM(MINFO,
	       "11h: Set local power constraint = %d channel=%d min_tx_pwr=%d max_tx_pwr=%d\n",
	       power_constraint, channel, min_tx_power_capability,
	       max_tx_power_capability);

	pcap = (MrvlIEtypes_PowerCapability_t *)*ppbuffer;
	pcap->header.type = wlan_cpu_to_le16(TLV_TYPE_POWER_CAPABILITY);
	pcap->header.len = wlan_cpu_to_le16(2);
	pcap->min_power = min_tx_power_capability;
	pcap->max_power = max_tx_power_capability;
	*ppbuffer += sizeof(MrvlIEtypesHeader_t) + 2;

	pconstraint = (MrvlIEtypes_LocalPowerConstraint_t *)*ppbuffer;
	pconstraint->header.type = wlan_cpu_to_le16(TLV_TYPE_POWER_CONSTRAINT);
	pconstraint->header.len = wlan_cpu_to_le16(2);
	pconstraint->chan = channel;
	pconstraint->constraint = power_constraint;
	*ppbuffer += sizeof(MrvlIEtypesHeader_t) + 2;
	ret_len = *ppbuffer - start_ptr;

	LEAVE();
	return ret_len;
}

/**
 *  @brief  Utility function to process a join to an infrastructure BSS
 *
 *  @param priv          Private driver information structure
 *  @param ppbuffer      Output parameter: Pointer to the TLV output buffer,
 *                       modified on return to point after the appended 11h TLVs
 *  @param band          Band on which we are joining the BSS
 *  @param channel       Channel on which we are joining the BSS
 *  @param p11h_bss_info Pointer to the 11h BSS information for this network
 *                       that was parsed out of the scan response.
 *
 *  @return              Integer number of bytes appended to the TLV output
 *                       buffer (ppbuffer)
 */
static t_u32 wlan_11h_process_infra_join(mlan_private *priv, t_u8 **ppbuffer,
					 t_u16 band, t_u32 channel,
					 wlan_11h_bss_info_t *p11h_bss_info)
{
	MrvlIEtypesHeader_t ie_header;
	IEEEtypes_SupportedChannels_t sup_chan_ie;
	t_u32 ret_len = 0;
	t_u16 sup_chan_len = 0;

	ENTER();

	/* Null Checks */
	if ((ppbuffer == MNULL) || (((t_u8 *)(*ppbuffer)) == MNULL)) {
		LEAVE();
		return 0;
	}

	ret_len += wlan_11h_set_local_power_constraint_tlv(
		ppbuffer, (t_u8)channel,
		(t_u8)p11h_bss_info->power_constraint.local_constraint,
		(t_u8)priv->adapter->state_11h.min_tx_power_capability,
		(t_u8)priv->adapter->state_11h.max_tx_power_capability);

	/* Setup the Supported Channels IE */
	sup_chan_len = wlan_11h_set_supp_channels_ie(priv, band, &sup_chan_ie);

	/*
	 * If we returned a valid Supported Channels IE, wrap and append it
	 */
	if (sup_chan_len) {
		/* Wrap the supported channels IE with a passthrough TLV type */
		ie_header.type = wlan_cpu_to_le16(TLV_TYPE_PASSTHROUGH);
		ie_header.len = wlan_cpu_to_le16(sup_chan_len);
		memcpy_ext(priv->adapter, *ppbuffer, &ie_header,
			   sizeof(ie_header), sizeof(ie_header));

		/*
		 * Increment the return size and the return buffer
		 * pointer param
		 */
		*ppbuffer += sizeof(ie_header);
		ret_len += sizeof(ie_header);

		/*
		 * Copy the supported channels IE to the output buf,
		 * advance pointer
		 */
		memcpy_ext(priv->adapter, *ppbuffer, &sup_chan_ie, sup_chan_len,
			   sup_chan_len);
		*ppbuffer += sup_chan_len;
		ret_len += sup_chan_len;
	}

	LEAVE();
	return ret_len;
}

#if defined(UAP_SUPPORT)
/**
 *  @brief Return whether the driver has enabled 11h for the interface
 *
 *  Association/Join commands are dynamic in that they enable 11h in the
 *    driver/firmware when they are detected in the existing BSS.
 *
 *  @param priv  Private driver information structure
 *
 *  @return
 *    - MTRUE if 11h is enabled
 *    - MFALSE otherwise
 */
static t_bool wlan_11h_is_enabled(mlan_private *priv)
{
	ENTER();
	LEAVE();
	return priv->intf_state_11h.is_11h_enabled;
}
#endif

/**
 *  @brief Return whether the device has activated slave radar detection.
 *
 *  @param priv  Private driver information structure
 *
 *  @return
 *    - MTRUE if slave radar detection is enabled in firmware
 *    - MFALSE otherwise
 */
static t_bool wlan_11h_is_slave_radar_det_active(mlan_private *priv)
{
	ENTER();
	LEAVE();
	return priv->adapter->state_11h.is_slave_radar_det_active;
}
/**
 *  @brief Return whether the slave interface is active, and on DFS channel.
 *  priv is assumed to already be a dfs slave interface, doesn't check this.
 *
 *  @param priv  Private driver information structure
 *
 *  @return
 *    - MTRUE if priv is slave, and meets both conditions
 *    - MFALSE otherwise
 */
static t_bool wlan_11h_is_slave_active_on_dfs_chan(mlan_private *priv)
{
	t_bool ret = MFALSE;

	ENTER();
	if ((priv->media_connected == MTRUE) &&
	    (priv->curr_bss_params.band & BAND_A) &&
	    wlan_11h_radar_detect_required(
		    priv, priv->curr_bss_params.bss_descriptor.channel))
		ret = MTRUE;

	LEAVE();
	return ret;
}

/**
 *  @brief Check if the current input channel is on radar channel
 *
 *
 *  @param priv    Private driver information structure
 *  @param channel Channel to determine radar detection requirements
 *
 *  @return
 *    - MTRUE if radar detection is required
 *    - MFALSE otherwise
 */
static t_bool wlan_11h_is_radar_channel(mlan_private *priv, t_u8 channel)
{
	t_bool required = MFALSE;

	ENTER();

	/*
	 * No checks for 11h or measurement code being enabled is placed here
	 * since regulatory requirements exist whether we support them or not.
	 */

	required = wlan_get_cfp_radar_detect(priv, channel);

	LEAVE();
	return required;
}

/**
 *  @brief Return whether the master interface is active, and on DFS channel.
 *  priv is assumed to already be a dfs master interface, doesn't check this.
 *
 *  @param priv  Private driver information structure
 *
 *  @return
 *    - MTRUE if priv is master, and meets both conditions
 *    - MFALSE otherwise
 */
static t_bool wlan_11h_is_master_active_on_dfs_chan(mlan_private *priv)
{
	t_bool ret = MFALSE;

	ENTER();
	if (GET_BSS_ROLE(priv) == MLAN_BSS_ROLE_STA) {
		/* Ad-hoc creator */
	} else if (GET_BSS_ROLE(priv) == MLAN_BSS_ROLE_UAP) {
		/* UAP */
#ifdef UAP_SUPPORT
		if ((priv->uap_bss_started == MTRUE) &&
		    (priv->uap_state_chan_cb.bandcfg.chanBand == BAND_5GHZ) &&
		    wlan_11h_radar_detect_required(
			    priv, priv->uap_state_chan_cb.channel))
			ret = MTRUE;
#endif
	}
	LEAVE();
	return ret;
}

/**
 *  @brief Determine if priv is DFS Master interface
 *
 *  @param priv Pointer to mlan_private
 *
 *  @return MTRUE or MFALSE
 */
static t_bool wlan_11h_is_dfs_master(mlan_private *priv)
{
	t_bool ret = MFALSE;

	ENTER();
	/* UAP: all are master */
	if (GET_BSS_ROLE(priv) == MLAN_BSS_ROLE_UAP)
		ret = MTRUE;

	/* all other cases = slave interface */
	LEAVE();
	return ret;
}

/* Need this as function to pass to wlan_count_priv_cond() */
/**
 *  @brief Determine if priv is DFS Slave interface
 *
 *  @param priv Pointer to mlan_private
 *
 *  @return MTRUE or MFALSE
 */

static t_bool wlan_11h_is_dfs_slave(mlan_private *priv)
{
	t_bool ret = MFALSE;
	ENTER();
	ret = !wlan_11h_is_dfs_master(priv);
	LEAVE();
	return ret;
}

/**
 *  @brief This function checks if interface is active.
 *
 *  @param pmpriv       A pointer to mlan_private structure
 *
 *  @return             MTRUE or MFALSE
 */
t_bool wlan_is_intf_active(mlan_private *pmpriv)
{
	t_bool ret = MFALSE;
	ENTER();

#ifdef UAP_SUPPORT
	if (GET_BSS_ROLE(pmpriv) == MLAN_BSS_ROLE_UAP)
		/*
		 * NOTE: UAP's media_connected == true only after first STA
		 * associated. Need different variable to tell if UAP
		 * has been started.
		 */
		ret = pmpriv->uap_bss_started;
	else
#endif
		if (GET_BSS_ROLE(pmpriv) == MLAN_BSS_ROLE_STA)
		ret = pmpriv->media_connected;

	LEAVE();
	return ret;
}

/**
 *  @brief This function gets current radar detect flags
 *
 *  @param pmadapter    A pointer to mlan_adapter structure
 *
 *  @return             11H MIB setting for radar detect
 */
static t_u32 wlan_11h_get_current_radar_detect_flags(mlan_adapter *pmadapter)
{
	t_u32 radar_det_flags = 0;

	ENTER();
	if (pmadapter->state_11h.is_master_radar_det_active)
		radar_det_flags |= MASTER_RADAR_DET_MASK;
	if (pmadapter->state_11h.is_slave_radar_det_active)
		radar_det_flags |= SLAVE_RADAR_DET_MASK;

	PRINTM(MINFO, "%s: radar_det_state_curr=0x%x\n", __func__,
	       radar_det_flags);

	LEAVE();
	return radar_det_flags;
}

/**
 *  @brief This function checks if radar detect flags have/should be changed.
 *
 *  @param pmadapter    A pointer to mlan_adapter structure
 *  @param pnew_state   Output param with new state, if return MTRUE.
 *
 *  @return             MTRUE (need update) or MFALSE (no change in flags)
 */
static t_bool wlan_11h_check_radar_det_state(mlan_adapter *pmadapter,
					     t_u32 *pnew_state)
{
	t_u32 radar_det_state_new = 0;
	t_bool ret;

	ENTER();
	PRINTM(MINFO,
	       "%s: master_radar_det_pending=%d, "
	       " slave_radar_det_pending=%d\n",
	       __func__, pmadapter->state_11h.master_radar_det_enable_pending,
	       pmadapter->state_11h.slave_radar_det_enable_pending);

	/* new state comes from evaluating interface states & pending starts */
	if (pmadapter->state_11h.master_radar_det_enable_pending ||
	    (wlan_count_priv_cond(pmadapter,
				  wlan_11h_is_master_active_on_dfs_chan,
				  wlan_11h_is_dfs_master) > 0))
		radar_det_state_new |= MASTER_RADAR_DET_MASK;
	if (pmadapter->state_11h.slave_radar_det_enable_pending ||
	    (wlan_count_priv_cond(pmadapter,
				  wlan_11h_is_slave_active_on_dfs_chan,
				  wlan_11h_is_dfs_slave) > 0))
		radar_det_state_new |= SLAVE_RADAR_DET_MASK;

	PRINTM(MINFO, "%s: radar_det_state_new=0x%x\n", __func__,
	       radar_det_state_new);

	/* now compare flags with current state */
	ret = (wlan_11h_get_current_radar_detect_flags(pmadapter) !=
	       radar_det_state_new) ?
		      MTRUE :
		      MFALSE;
	if (ret)
		*pnew_state = radar_det_state_new;

	LEAVE();
	return ret;
}

/**
 *  @brief generate the channel center frequency index
 *
 *  @param channel_num       channel number
 *
 *  @return                 frenquency index
 */
static t_u8 wlan_11h_get_channel_freq_idx(t_u8 channel_num)
{
	t_u8 index;
	t_u8 center_freq[] = {42, 58, 106, 122, 138, 155};
	t_u8 chan_idx, ret = 0;

	chan_idx = channel_num - 100;

	for (index = 0; index < sizeof(center_freq); index++) {
		if ((chan_idx >= (center_freq[index] - 6)) &&
		    (chan_idx <= (center_freq[index] + 6))) {
			ret = center_freq[index];
			break;
		}
	}

	return ret;
}

/**
 *  @brief Prepare ioctl for add/remove CHAN_SW IE - RADAR_DETECTED event
 * handling
 *
 *  @param pmadapter        Pointer to mlan_adapter
 *  @param pioctl_req       Pointer to completed mlan_ioctl_req (allocated
 * inside)
 *  @param ppcust_chansw_ie Poniter to customer ie
 *  @param is_adding_ie     CHAN_SW IE is to be added (MTRUE), or removed
 * (MFALSE)
 *
 *  @return                 MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status
wlan_11h_prepare_custom_ie_chansw(mlan_adapter *pmadapter,
				  mlan_ioctl_req **ppioctl_req,
				  t_bool is_adding_ie)
{
	mlan_ioctl_req *pioctl_req = MNULL;
	mlan_ds_misc_cfg *pds_misc_cfg = MNULL;
	custom_ie *pcust_chansw_ie = MNULL;
	IEEEtypes_ChanSwitchAnn_t *pchansw_ie = MNULL;
	mlan_status ret;
	IEEEtypes_Header_t *pChanSwWrap_ie = MNULL;
	IEEEtypes_WideBWChanSwitch_t *pbwchansw_ie = MNULL;
	IEEEtypes_VhtTpcEnvelope_t *pvhttpcEnv_ie = MNULL;
	t_u8 index;
	mlan_private *pmpriv = MNULL;

	ENTER();

	if (pmadapter == MNULL || ppioctl_req == MNULL) {
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}

	/* allocate buffer for mlan_ioctl_req and mlan_ds_misc_cfg */
	/* FYI - will be freed as part of cmd_response handler */
	ret = pmadapter->callbacks.moal_malloc(
		pmadapter->pmoal_handle,
		sizeof(mlan_ioctl_req) + sizeof(mlan_ds_misc_cfg), MLAN_MEM_DEF,
		(t_u8 **)&pioctl_req);
	if ((ret != MLAN_STATUS_SUCCESS) || !pioctl_req) {
		PRINTM(MERROR, "%s(): Could not allocate ioctl req\n",
		       __func__);
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}
	pds_misc_cfg = (mlan_ds_misc_cfg *)((t_u8 *)pioctl_req +
					    sizeof(mlan_ioctl_req));

	/* prepare mlan_ioctl_req */
	pioctl_req->req_id = MLAN_IOCTL_MISC_CFG;
	pioctl_req->action = MLAN_ACT_SET;
	pioctl_req->pbuf = (t_u8 *)pds_misc_cfg;
	pioctl_req->buf_len = sizeof(mlan_ds_misc_cfg);

	/* prepare mlan_ds_misc_cfg */
	pds_misc_cfg->sub_command = MLAN_OID_MISC_CUSTOM_IE;
	pds_misc_cfg->param.cust_ie.type = TLV_TYPE_MGMT_IE;
	pds_misc_cfg->param.cust_ie.len = (sizeof(custom_ie) - MAX_IE_SIZE);

	/* configure custom_ie api settings */
	pcust_chansw_ie =
		(custom_ie *)&pds_misc_cfg->param.cust_ie.ie_data_list[0];
	pcust_chansw_ie->ie_index = 0xffff; /* Auto index */
	pcust_chansw_ie->ie_length = sizeof(IEEEtypes_ChanSwitchAnn_t);
	pcust_chansw_ie->mgmt_subtype_mask =
		(is_adding_ie) ? MBIT(8) | MBIT(5) /* add IE for BEACON |
						      PROBE_RSP */
				 :
				 0; /* remove IE */

	/* prepare CHAN_SW IE inside ioctl */
	pchansw_ie = (IEEEtypes_ChanSwitchAnn_t *)pcust_chansw_ie->ie_buffer;
	pchansw_ie->element_id = CHANNEL_SWITCH_ANN;
	pchansw_ie->len =
		sizeof(IEEEtypes_ChanSwitchAnn_t) - sizeof(IEEEtypes_Header_t);
	pchansw_ie->chan_switch_mode = 1; /* STA should not transmit */
	pchansw_ie->new_channel_num = pmadapter->state_rdh.new_channel;

	pchansw_ie->chan_switch_count = pmadapter->dfs_cs_count;
	PRINTM(MCMD_D, "New Channel = %d Channel switch count = %d\n",
	       pmadapter->state_rdh.new_channel, pchansw_ie->chan_switch_count);

	for (index = 0; index < pmadapter->state_rdh.priv_list_count; index++) {
		pmpriv = pmadapter->state_rdh.priv_list[index];
		/*find the first AP interface*/
		if (GET_BSS_ROLE(pmpriv) == MLAN_BSS_ROLE_UAP) {
			if (pmpriv->is_11ac_enabled) {
				pChanSwWrap_ie =
					(IEEEtypes_Header_t
						 *)((t_u8 *)pchansw_ie +
						    sizeof(IEEEtypes_ChanSwitchAnn_t));
				pChanSwWrap_ie->element_id = EXT_POWER_CONSTR;
				/*will have multiple sub IEs*/
				pChanSwWrap_ie->len = 0;

				/* prepare the Wide Bandwidth Channel Switch IE
				 * Channel Switch IE */
				pbwchansw_ie =
					(IEEEtypes_WideBWChanSwitch_t
						 *)((t_u8 *)pChanSwWrap_ie +
						    sizeof(IEEEtypes_Header_t));
				pbwchansw_ie->ieee_hdr.element_id =
					BW_CHANNEL_SWITCH;
				pbwchansw_ie->ieee_hdr.len =
					sizeof(IEEEtypes_WideBWChanSwitch_t) -
					sizeof(IEEEtypes_Header_t);
				/*fix 80MHZ now*/
				pbwchansw_ie->new_channel_width =
					VHT_OPER_CHWD_80MHZ;
				pbwchansw_ie->new_channel_center_freq0 =
					wlan_11h_get_channel_freq_idx(
						pmadapter->state_rdh
							.new_channel);
				pbwchansw_ie->new_channel_center_freq1 =
					wlan_11h_get_channel_freq_idx(
						pmadapter->state_rdh
							.new_channel);
				pChanSwWrap_ie->len +=
					sizeof(IEEEtypes_WideBWChanSwitch_t);

				/*prepare the VHT Transmit Power Envelope IE*/
				pvhttpcEnv_ie =
					(IEEEtypes_VhtTpcEnvelope_t
						 *)((t_u8 *)pChanSwWrap_ie +
						    sizeof(IEEEtypes_Header_t) +
						    sizeof(IEEEtypes_WideBWChanSwitch_t));
				pvhttpcEnv_ie->ieee_hdr.element_id =
					VHT_TX_POWER_ENV;
				pvhttpcEnv_ie->ieee_hdr.len =
					sizeof(IEEEtypes_VhtTpcEnvelope_t) -
					sizeof(IEEEtypes_Header_t);
				/* Local Max TX Power Count= 3,
				 * Local TX Power Unit Inter=EIP(0) */
				pvhttpcEnv_ie->tpc_info = 3;
				pvhttpcEnv_ie->local_max_tp_20mhz = 0xff;
				pvhttpcEnv_ie->local_max_tp_40mhz = 0xff;
				pvhttpcEnv_ie->local_max_tp_80mhz = 0xff;
				pvhttpcEnv_ie->local_max_tp_160mhz_80_80mhz =
					0xff;
				pChanSwWrap_ie->len +=
					sizeof(IEEEtypes_VhtTpcEnvelope_t);

				pcust_chansw_ie->ie_length +=
					sizeof(IEEEtypes_WideBWChanSwitch_t) +
					sizeof(IEEEtypes_VhtTpcEnvelope_t) +
					sizeof(IEEEtypes_Header_t);

				PRINTM(MINFO,
				       "Append Wide Bandwidth Channel Switch IE\n");
				break;
			}
		}
	}

	pds_misc_cfg->param.cust_ie.len += pcust_chansw_ie->ie_length;
	DBG_HEXDUMP(MCMD_D, "11h: custom_ie containing CHAN_SW IE",
		    (t_u8 *)pcust_chansw_ie, pds_misc_cfg->param.cust_ie.len);

	/* assign output pointer before returning */
	*ppioctl_req = pioctl_req;
	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

#ifdef UAP_SUPPORT
/**
 *  @brief Check if start channel 165 is allowed to operate in
 *  previous uAP channel's band config
 *
 *  @param priv          a pointer to mlan_private structure
 *  @param start_chn     Random Start channel choosen after radar detection
 *  @param uap_band_cfg  Private driver uAP band configuration information
 * structure
 *
 *  @return MFALSE if the channel is not allowed in given band
 */
static t_bool wlan_11h_is_band_valid(mlan_private *priv, t_u8 start_chn,
				     Band_Config_t uap_band_cfg)
{
	/* if band width is not 20MHZ (either 40 or 80MHz)
	 * return MFALSE, 165 is not allowed in bands other than 20MHZ
	 */
	if (start_chn == 165) {
		if (priv->adapter->region_code == COUNTRY_CODE_US)
			return MTRUE;
		if (uap_band_cfg.chanWidth != CHAN_BW_20MHZ)
			return MFALSE;
	}
	return MTRUE;
}

/**
 *  @brief Retrieve a randomly selected starting channel if needed for 11h
 *
 *  If 11h is enabled and 5GHz band is selected in band_config
 *    return a random channel in A band, else one from BG band.
 *
 *  @param priv          Private driver information structure
 *  @param uap_band_cfg  Private driver information structure
 *
 *  @return      Starting channel
 */
static t_u8 wlan_11h_get_uap_start_channel(mlan_private *priv,
					   Band_Config_t uap_band_cfg)
{
	t_u8 start_chn;
	mlan_adapter *adapter = priv->adapter;
	t_u32 region;
	t_u32 rand_entry;
	region_chan_t *chn_tbl;
	t_u8 rand_tries = 0;

	/*TODO:  right now mostly a copy of wlan_11h_get_adhoc_start_channel.
	 *       Improve to be more specfic to UAP, e.g.
	 *       1. take into account COUNTRY_CODE -> region_code
	 *       2. check domain_info for value channels
	 */
	ENTER();

	/*
	 * Set start_chn to the Default.
	 * Used if 11h is disabled or the band
	 * does not require 11h support.
	 */
	start_chn = DEFAULT_AD_HOC_CHANNEL;

	/*
	 * Check that we are looking for a channel in the A Band
	 */
	if (uap_band_cfg.chanBand == BAND_5GHZ) {
		/*
		 * Set default to the A Band default.
		 * Used if random selection fails
		 * or if 11h is not enabled
		 */
		start_chn = DEFAULT_AD_HOC_CHANNEL_A;

		/*
		 * Check that 11h is enabled in the driver
		 */
		if (wlan_11h_is_enabled(priv)) {
			/*
			 * Search the region_channel tables for a channel table
			 * that is marked for the A Band.
			 */
			for (region = 0; (region < MAX_REGION_CHANNEL_NUM);
			     region++) {
				chn_tbl = &adapter->region_channel[region];

				/* Check if table is valid and marked for A Band
				 */
				if (chn_tbl->valid &&
				    chn_tbl->region == adapter->region_code &&
				    chn_tbl->band & BAND_A) {
					/*
					 * Set the start channel.  Get a random
					 * number and use it to pick an entry
					 * in the table between 0 and the number
					 * of channels in the table (NumCFP).
					 */
					rand_entry = wlan_11h_get_random_num(
							     adapter) %
						     chn_tbl->num_cfp;
					start_chn =
						(t_u8)chn_tbl->pcfp[rand_entry]
							.channel;
					/* Loop until a non-dfs channel is found
					 * with compatible band bounded by
					 * chn_tbl->num_cfp entries in the
					 * channel table
					 */
					while (((chn_tbl->pcfp[rand_entry]
							 .dynamic.flags &
						 NXP_CHANNEL_DISABLED) ||
						(wlan_11h_is_channel_under_nop(
							 adapter, start_chn) ||
						 ((adapter->state_rdh.stage ==
						   RDH_GET_INFO_CHANNEL) &&
						  wlan_11h_radar_detect_required(
							  priv, start_chn)) ||
						 !(wlan_11h_is_band_valid(
							 priv, start_chn,
							 uap_band_cfg)))) &&
					       (++rand_tries <
						chn_tbl->num_cfp)) {
						rand_entry++;
						rand_entry = rand_entry %
							     chn_tbl->num_cfp;
						start_chn =
							(t_u8)chn_tbl
								->pcfp[rand_entry]
								.channel;
						PRINTM(MINFO,
						       "start chan=%d rand_entry=%d\n",
						       start_chn, rand_entry);
					}

					if (rand_tries == chn_tbl->num_cfp) {
						PRINTM(MERROR,
						       "Failed to get UAP start channel\n");
						start_chn = 0;
					}
				}
			}
		}
	}

	PRINTM(MCMD_D, "11h: UAP Get Start Channel %d\n", start_chn);
	LEAVE();
	return start_chn;
}
#endif /* UAP_SUPPORT */

#ifdef DEBUG_LEVEL1
static const char *DFS_TS_REPR_STRINGS[] = {"", "NOP_start", "CAC_completed"};
#endif

/**
 *  @brief Search for a dfs timestamp in the list with desired channel.
 *
 *  Assumes there will only be one timestamp per channel in the list.
 *
 *  @param pmadapter  Pointer to mlan_adapter
 *  @param channel    Channel number
 *
 *  @return           Pointer to timestamp if found, or MNULL
 */
static wlan_dfs_timestamp_t *
wlan_11h_find_dfs_timestamp(mlan_adapter *pmadapter, t_u8 channel)
{
	wlan_dfs_timestamp_t *pts = MNULL, *pts_found = MNULL;

	ENTER();
	pts = (wlan_dfs_timestamp_t *)util_peek_list(
		pmadapter->pmoal_handle, &pmadapter->state_dfs.dfs_ts_head,
		MNULL, MNULL);

	while (pts && pts != (wlan_dfs_timestamp_t *)&pmadapter->state_dfs
				      .dfs_ts_head) {
		PRINTM(MINFO,
		       "dfs_timestamp(@ %p) - chan=%d, repr=%d(%s),"
		       " time(sec.usec)=%lu.%06lu\n",
		       pts, pts->channel, pts->represents,
		       DFS_TS_REPR_STRINGS[pts->represents], pts->ts_sec,
		       pts->ts_usec);

		if (pts->channel == channel) {
			pts_found = pts;
			break;
		}
		pts = pts->pnext;
	}

	LEAVE();
	return pts_found;
}

/**
 *  @brief Removes dfs timestamp from list.
 *
 *  @param pmadapter  Pointer to mlan_adapter
 *  @param pdfs_ts    Pointer to dfs_timestamp to remove
 */
static t_void wlan_11h_remove_dfs_timestamp(mlan_adapter *pmadapter,
					    wlan_dfs_timestamp_t *pdfs_ts)
{
	ENTER();
	/* dequeue and delete timestamp */
	util_unlink_list(pmadapter->pmoal_handle,
			 &pmadapter->state_dfs.dfs_ts_head,
			 (pmlan_linked_list)pdfs_ts, MNULL, MNULL);
	pmadapter->callbacks.moal_mfree(pmadapter->pmoal_handle,
					(t_u8 *)pdfs_ts);
	LEAVE();
}

/**
 *  @brief Add a dfs timestamp to the list
 *
 *  Assumes there will only be one timestamp per channel in the list,
 *  and that timestamp modes (represents) are mutually exclusive.
 *
 *  @param pmadapter  Pointer to mlan_adapter
 *  @param repr       Timestamp 'represents' value (see _dfs_timestamp_repr_e)
 *  @param channel    Channel number
 *
 *  @return           Pointer to timestamp if found, or MNULL
 */
static mlan_status wlan_11h_add_dfs_timestamp(mlan_adapter *pmadapter,
					      t_u8 repr, t_u8 channel)
{
	wlan_dfs_timestamp_t *pdfs_ts = MNULL;
	mlan_status ret = MLAN_STATUS_SUCCESS;

	ENTER();
	pdfs_ts = wlan_11h_find_dfs_timestamp(pmadapter, channel);

	if (!pdfs_ts) {
		/* need to allocate new timestamp */
		ret = pmadapter->callbacks.moal_malloc(
			pmadapter->pmoal_handle, sizeof(wlan_dfs_timestamp_t),
			MLAN_MEM_DEF, (t_u8 **)&pdfs_ts);
		if ((ret != MLAN_STATUS_SUCCESS) || !pdfs_ts) {
			PRINTM(MERROR, "%s(): Could not allocate dfs_ts\n",
			       __func__);
			LEAVE();
			return MLAN_STATUS_FAILURE;
		}

		util_enqueue_list_tail(pmadapter->pmoal_handle,
				       &pmadapter->state_dfs.dfs_ts_head,
				       (pmlan_linked_list)pdfs_ts, MNULL,
				       MNULL);
		pdfs_ts->channel = channel;
	}
	/* (else, use existing timestamp for channel; see assumptions above) */

	/* update params */
	pmadapter->callbacks.moal_get_system_time(
		pmadapter->pmoal_handle, &pdfs_ts->ts_sec, &pdfs_ts->ts_usec);
	pdfs_ts->represents = repr;

	PRINTM(MCMD_D,
	       "11h: add/update dfs_timestamp - chan=%d, repr=%d(%s),"
	       " time(sec.usec)=%lu.%06lu\n",
	       pdfs_ts->channel, pdfs_ts->represents,
	       DFS_TS_REPR_STRINGS[pdfs_ts->represents], pdfs_ts->ts_sec,
	       pdfs_ts->ts_usec);

	LEAVE();
	return ret;
}

/**
 *  @brief Add all bonded channel's dfs timestamp to the list
 *
 *  @param pmadapter  Pointer to mlan_adapter
 *  @param repr       Timestamp 'represents' value (see _dfs_timestamp_repr_e)
 *  @param channel    Channel number
 *  @param bandwidth  Channel bandwidth
 *
 *  @return           Pointer to timestamp if found, or MNULL
 */
static void wlan_11h_add_all_dfs_timestamp(mlan_adapter *pmadapter, t_u8 repr,
					   t_u8 channel, t_u8 bandwidth)
{
	t_u8 n_chan;
	t_u8 chan_list[4] = {0};
	t_u8 i;
	n_chan = woal_get_bonded_channels(channel, bandwidth, chan_list);
	for (i = 0; i < n_chan; i++)
		wlan_11h_add_dfs_timestamp(pmadapter, repr, chan_list[i]);
}

/********************************************************
			Global functions
********************************************************/

/**
 *  @brief Return whether the device has activated master radar detection.
 *
 *  @param priv  Private driver information structure
 *
 *  @return
 *    - MTRUE if master radar detection is enabled in firmware
 *    - MFALSE otherwise
 */
t_bool wlan_11h_is_master_radar_det_active(mlan_private *priv)
{
	ENTER();
	LEAVE();
	return priv->adapter->state_11h.is_master_radar_det_active;
}

/**
 *  @brief Configure master radar detection.
 *  Call wlan_11h_check_update_radar_det_state() afterwards
 *    to push this to firmware.
 *
 *  @param priv  Private driver information structure
 *  @param enable Whether to enable or disable master radar detection
 *
 *  @return  MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 *
 *  @sa wlan_11h_check_update_radar_det_state
 */
mlan_status wlan_11h_config_master_radar_det(mlan_private *priv, t_bool enable)
{
	mlan_status ret = MLAN_STATUS_FAILURE;

	/* Force disable master radar detection on in-AP interfaces */
	if (priv->adapter->dfs_repeater)
		enable = MFALSE;

	ENTER();
	if (wlan_11h_is_dfs_master(priv) &&
	    priv->adapter->init_para.dfs_master_radar_det_en) {
		priv->adapter->state_11h.master_radar_det_enable_pending =
			enable;
		ret = MLAN_STATUS_SUCCESS;
	}

	LEAVE();
	return ret;
}

/**
 *  @brief Configure slave radar detection.
 *  Call wlan_11h_check_update_radar_det_state() afterwards
 *    to push this to firmware.
 *
 *  @param priv  Private driver information structure
 *  @param enable Whether to enable or disable slave radar detection
 *
 *  @return  MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 *
 *  @sa wlan_11h_check_update_radar_det_state
 */
mlan_status wlan_11h_config_slave_radar_det(mlan_private *priv, t_bool enable)
{
	mlan_status ret = MLAN_STATUS_FAILURE;

	/* Force disable radar detection on STA interfaces */
	if (priv->adapter->dfs_repeater)
		enable = MFALSE;

	ENTER();
	if (wlan_11h_is_dfs_slave(priv) &&
	    priv->adapter->init_para.dfs_slave_radar_det_en) {
		priv->adapter->state_11h.slave_radar_det_enable_pending =
			enable;
		ret = MLAN_STATUS_SUCCESS;
	}
	LEAVE();
	return ret;
}

#ifdef UAP_SUPPORT
/**
 *  @brief Return whether the slave interface is on DFS channel.
 *  priv is assumed to already be a dfs slave interface, doesn't check this.
 *
 *  @param priv  Private driver information structure
 *
 *  @return
 *    - MTRUE if priv is slave, and meets both conditions
 *    - MFALSE otherwise
 */
static t_bool wlan_11h_is_slave_on_dfs_chan(mlan_private *priv)
{
	t_bool ret = MFALSE;

	ENTER();
	if ((priv->media_connected == MTRUE) &&
	    (priv->curr_bss_params.band & BAND_A) &&
	    wlan_11h_is_radar_channel(
		    priv, priv->curr_bss_params.bss_descriptor.channel))
		ret = MTRUE;

	LEAVE();
	return ret;
}

/**
 *  @brief check if dfs_master and dfs_slave are in same channel
 *
 *  @param pmadapter Pointer to mlan_adapter structure
 *
 *  @return        MTRUE-dfs_master and dfs_slave interface on same DFS channel
 *
 */
static t_u8 wlan_11h_check_dfs_channel(mlan_adapter *pmadapter)
{
	mlan_private *priv_master = MNULL;
	mlan_private *priv_slave = MNULL;
	mlan_private *priv_list[MLAN_MAX_BSS_NUM] = {MNULL};

	if (wlan_get_privs_by_two_cond(
		    pmadapter, wlan_11h_is_master_active_on_dfs_chan,
		    wlan_11h_is_dfs_master, MTRUE, priv_list)) {
		priv_master = priv_list[0];
		PRINTM(MINFO, "%s: found dfs_master priv=%p\n", __func__,
		       priv_master);
	}
	if (wlan_get_privs_by_two_cond(pmadapter, wlan_11h_is_slave_on_dfs_chan,
				       wlan_11h_is_dfs_slave, MTRUE,
				       priv_list)) {
		priv_slave = priv_list[0];
		PRINTM(MINFO, "%s: found dfs_slave priv=%p\n", __func__,
		       priv_slave);
	}
	if (!priv_slave || !priv_master)
		return MFALSE;
	if (priv_master->uap_state_chan_cb.channel !=
	    priv_slave->curr_bss_params.bss_descriptor.channel)
		return MFALSE;
	return MTRUE;
}

/**
 *   @brief disable 11h and DFS function
 *
 *  @param priv         Private driver information structure
 *  @param pioctl_buf   A pointer to MLAN IOCTL Request buffer
 *
 *  @return      MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status wlan_11h_disable_dfs(mlan_private *priv, t_void *pioctl_buf)
{
	t_u32 enable = 0;
	mlan_status ret = MLAN_STATUS_SUCCESS;

	ENTER();

	/*
	 * Send cmd to FW to enable/disable 11h function in firmware
	 */
	ret = wlan_prepare_cmd(priv, HostCmd_CMD_802_11_SNMP_MIB,
			       HostCmd_ACT_GEN_SET, Dot11H_i,
			       (t_void *)pioctl_buf, &enable);
	if (ret)
		ret = MLAN_STATUS_FAILURE;
	else
		/* Set boolean flag in driver 11h state */
		priv->intf_state_11h.is_11h_active = MFALSE;

	PRINTM(MINFO, "11h: DFS %s\n", "Deactivate");

	LEAVE();
	return ret;
}

/**
 *  @brief check if we need enable dfs_master
 *
 *  @param priv  Pointer to mlan_private structure
 *				 priv should be UAP priv
 *
 *  @return      N/A
 *
 */
void wlan_11h_update_dfs_master_state_by_uap(mlan_private *pmpriv)
{
	mlan_adapter *pmadapter = pmpriv->adapter;
	mlan_status ret = MLAN_STATUS_SUCCESS;
	if (pmadapter->dfs_mode && wlan_11h_check_dfs_channel(pmadapter)) {
		PRINTM(MCMND,
		       "11h: disable DFS master when AP+STA on same DFS channel\n");
		ret = wlan_11h_disable_dfs(pmpriv, MNULL);
		return;
	}
	if (!wlan_11h_is_active(pmpriv)) {
		/* active 11h extention in Fw */
		PRINTM(MCMND,
		       "11h: Enable DFS master after AP up or chan_switch\n");
		ret = wlan_11h_activate(pmpriv, MNULL, MTRUE);
		ret = wlan_11h_config_master_radar_det(pmpriv, MTRUE);
		ret = wlan_11h_check_update_radar_det_state(pmpriv);
	}
	if (pmpriv->uap_host_based && !pmpriv->adapter->init_para.dfs_offload)
		pmpriv->intf_state_11h.is_11h_host = MTRUE;
	wlan_11h_set_dfs_check_chan(pmpriv, pmpriv->uap_channel,
				    pmpriv->uap_bandwidth);
	return;
}

/**
 *  @brief check if dfs_master and dfs_slave are in same channel
 *
 *  @param pmadapter Pointer to mlan_adapter structure
 *
 *  @return        MTRUE-dfs_master and dfs_slave interface on same DFS channel
 *
 */
void wlan_11h_update_dfs_master_state_by_sta(mlan_private *pmpriv)
{
	mlan_private *priv_master = MNULL;
	mlan_private *priv_slave = MNULL;
	mlan_private *priv_list[MLAN_MAX_BSS_NUM] = {MNULL};
	mlan_adapter *pmadapter = pmpriv->adapter;
	mlan_status ret = MLAN_STATUS_SUCCESS;

	if (wlan_get_privs_by_two_cond(
		    pmadapter, wlan_11h_is_master_active_on_dfs_chan,
		    wlan_11h_is_dfs_master, MTRUE, priv_list)) {
		priv_master = priv_list[0];
		PRINTM(MINFO, "%s: found dfs_master priv=%p\n", __func__,
		       priv_master);
	}
	if (wlan_get_privs_by_two_cond(pmadapter, wlan_11h_is_slave_on_dfs_chan,
				       wlan_11h_is_dfs_slave, MTRUE,
				       priv_list)) {
		priv_slave = priv_list[0];
		PRINTM(MINFO, "%s: found dfs_slave priv=%p\n", __func__,
		       priv_slave);
	}
	if (!priv_slave || !priv_master)
		return;
	if (priv_master->uap_state_chan_cb.channel ==
	    priv_slave->curr_bss_params.bss_descriptor.channel) {
		PRINTM(MCMND,
		       "11h: disable DFS master when AP+STA on same DFS channel\n");
		ret = wlan_11h_disable_dfs(priv_master, MNULL);
	}
	return;
}

/**
 *  @brief update the dfs master state on station disconnect
 *
 *  @param priv  Pointer to mlan_private structure
 *				 priv should be UAP priv
 *
 *  @return      N/A
 *
 */
void wlan_11h_update_dfs_master_state_on_disconect(mlan_private *priv)
{
	mlan_private *priv_master = MNULL;
	mlan_private *priv_list[MLAN_MAX_BSS_NUM] = {MNULL};
	mlan_adapter *pmadapter = priv->adapter;
	mlan_status ret = MLAN_STATUS_SUCCESS;
	if (wlan_get_privs_by_two_cond(
		    pmadapter, wlan_11h_is_master_active_on_dfs_chan,
		    wlan_11h_is_dfs_master, MTRUE, priv_list)) {
		priv_master = priv_list[0];
		PRINTM(MINFO, "%s: found dfs_master priv=%p\n", __func__,
		       priv_master);
	}
	if (!priv_master) {
		wlan_11h_check_update_radar_det_state(priv);
		return;
	}
	if (!wlan_11h_is_active(priv_master)) {
		PRINTM(MCMND, "11h: Enable DFS master after STA disconnect\n");
		/* active 11h extention in Fw */
		ret = wlan_11h_activate(priv_master, MNULL, MTRUE);
		ret = wlan_11h_config_master_radar_det(priv_master, MTRUE);
		ret = wlan_11h_check_update_radar_det_state(priv_master);
	}
	if (priv_master->uap_host_based && !pmadapter->init_para.dfs_offload)
		priv_master->intf_state_11h.is_11h_host = MTRUE;
	wlan_11h_set_dfs_check_chan(priv_master, priv_master->uap_channel,
				    priv_master->uap_bandwidth);
	return;
}
#endif

/**
 *  @brief Checks all interfaces and determines if radar_detect flag states
 *         have/should be changed.  If so, sends SNMP_MIB 11H command to FW.
 *         Call this function on any interface enable/disable/channel change.
 *
 *  @param pmpriv  Pointer to mlan_private structure
 *
 *  @return        MLAN_STATUS_SUCCESS (update or not)
 *              or MLAN_STATUS_FAILURE (cmd failure)
 *
 *  @sa    wlan_11h_check_radar_det_state
 */
mlan_status wlan_11h_check_update_radar_det_state(mlan_private *pmpriv)
{
	t_u32 new_radar_det_state = 0;
	t_u32 mib_11h = 0;
	mlan_status ret = MLAN_STATUS_SUCCESS;

	ENTER();

	if (wlan_11h_check_radar_det_state(pmpriv->adapter,
					   &new_radar_det_state)) {
		PRINTM(MCMD_D, "%s: radar_det_state being updated.\n",
		       __func__);

		mib_11h |= new_radar_det_state;
		/* keep priv's existing 11h state */
		if (pmpriv->intf_state_11h.is_11h_active)
			mib_11h |= ENABLE_11H_MASK;

		/* Send cmd to FW to enable/disable 11h function in firmware */
		ret = wlan_prepare_cmd(pmpriv, HostCmd_CMD_802_11_SNMP_MIB,
				       HostCmd_ACT_GEN_SET, Dot11H_i, MNULL,
				       &mib_11h);
		if (ret)
			ret = MLAN_STATUS_FAILURE;
	}

	/* updated state sent OR no change, thus no longer pending */
	pmpriv->adapter->state_11h.master_radar_det_enable_pending = MFALSE;
	pmpriv->adapter->state_11h.slave_radar_det_enable_pending = MFALSE;

	LEAVE();
	return ret;
}

/**
 *  @brief Query 11h firmware enabled state.
 *
 *  Return whether the firmware currently has 11h extensions enabled
 *
 *  @param priv  Private driver information structure
 *
 *  @return
 *    - MTRUE if 11h has been activated in the firmware
 *    - MFALSE otherwise
 *
 *  @sa wlan_11h_activate
 */
t_bool wlan_11h_is_active(mlan_private *priv)
{
	ENTER();
	LEAVE();
	return priv->intf_state_11h.is_11h_active;
}

/**
 *  @brief Enable the transmit interface and record the state.
 *
 *  @param priv  Private driver information structure
 *
 *  @return      N/A
 */
t_void wlan_11h_tx_enable(mlan_private *priv)
{
	ENTER();
	if (priv->intf_state_11h.tx_disabled) {
		if (priv->media_connected == MTRUE) {
			wlan_recv_event(priv, MLAN_EVENT_ID_FW_START_TX, MNULL);
			priv->intf_state_11h.tx_disabled = MFALSE;
		}
	}
	LEAVE();
}

/**
 *  @brief Disable the transmit interface and record the state.
 *
 *  @param priv  Private driver information structure
 *
 *  @return      N/A
 */
t_void wlan_11h_tx_disable(mlan_private *priv)
{
	ENTER();
	if (!priv->intf_state_11h.tx_disabled) {
		if (priv->media_connected == MTRUE) {
			priv->intf_state_11h.tx_disabled = MTRUE;
			wlan_recv_event(priv, MLAN_EVENT_ID_FW_STOP_TX, MNULL);
		}
	}
	LEAVE();
}

/**
 *  @brief Enable or Disable the 11h extensions in the firmware
 *
 *  @param priv         Private driver information structure
 *  @param pioctl_buf   A pointer to MLAN IOCTL Request buffer
 *  @param flag         Enable 11h if MTRUE, disable otherwise
 *
 *  @return      MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status wlan_11h_activate(mlan_private *priv, t_void *pioctl_buf,
			      t_bool flag)
{
	t_u32 enable = flag & ENABLE_11H_MASK;
	mlan_status ret = MLAN_STATUS_SUCCESS;

	ENTER();
	/* add bits for master/slave radar detect into enable. */
	enable |= wlan_11h_get_current_radar_detect_flags(priv->adapter);

	/* Whenever repeater mode is on make sure
	 * we do not enable master or slave radar det mode.
	 * HW will not detect radar in dfs_repeater mode.
	 */
	if (priv->adapter->dfs_repeater) {
		enable &= ~(MASTER_RADAR_DET_MASK | SLAVE_RADAR_DET_MASK);
	}

	/*
	 * Send cmd to FW to enable/disable 11h function in firmware
	 */
	ret = wlan_prepare_cmd(priv, HostCmd_CMD_802_11_SNMP_MIB,
			       HostCmd_ACT_GEN_SET, Dot11H_i,
			       (t_void *)pioctl_buf, &enable);
	if (ret)
		ret = MLAN_STATUS_FAILURE;
	else
		/* Set boolean flag in driver 11h state */
		priv->intf_state_11h.is_11h_active = flag;

	PRINTM(MINFO, "11h: %s\n", flag ? "Activate" : "Deactivate");

	LEAVE();
	return ret;
}

/**
 *  @brief Initialize the 11h parameters and enable 11h when starting an IBSS
 *
 *  @param adapter mlan_adapter structure
 *
 *  @return      N/A
 */
t_void wlan_11h_init(mlan_adapter *adapter)
{
	wlan_11h_device_state_t *pstate_11h = &adapter->state_11h;
	IEEEtypes_Quiet_t *pquiet = &adapter->state_11h.quiet_ie;
	wlan_dfs_device_state_t *pstate_dfs = &adapter->state_dfs;
	wlan_radar_det_hndlg_state_t *pstate_rdh = &adapter->state_rdh;
	wlan_dfs_testing_settings_t *pdfs_test = &adapter->dfs_test_params;

	ENTER();

	/* Initialize 11H struct */
	pstate_11h->usr_def_power_constraint = WLAN_11H_TPC_POWERCONSTRAINT;
	pstate_11h->min_tx_power_capability = WLAN_11H_TPC_POWERCAPABILITY_MIN;
	pstate_11h->max_tx_power_capability = WLAN_11H_TPC_POWERCAPABILITY_MAX;

	pstate_11h->recvd_chanswann_event = MFALSE;
	pstate_11h->master_radar_det_enable_pending = MFALSE;
	pstate_11h->slave_radar_det_enable_pending = MFALSE;
	pstate_11h->is_master_radar_det_active = MFALSE;
	pstate_11h->is_slave_radar_det_active = MFALSE;

	/*Initialize quiet_ie*/
	memset(adapter, pquiet, 0, sizeof(IEEEtypes_Quiet_t));
	pquiet->element_id = QUIET;
	pquiet->len =
		(sizeof(pquiet->quiet_count) + sizeof(pquiet->quiet_period) +
		 sizeof(pquiet->quiet_duration) + sizeof(pquiet->quiet_offset));

	/* Initialize DFS struct */
	pstate_dfs->dfs_check_pending = MFALSE;
	pstate_dfs->dfs_radar_found = MFALSE;
	pstate_dfs->dfs_check_channel = 0;
	pstate_dfs->dfs_report_time_sec = 0;
	util_init_list((pmlan_linked_list)&pstate_dfs->dfs_ts_head);

	/* Initialize RDH struct */
	pstate_rdh->stage = RDH_OFF;
	pstate_rdh->priv_list_count = 0;
	pstate_rdh->priv_curr_idx = 0;
	pstate_rdh->curr_channel = 0;
	pstate_rdh->new_channel = 0;
	memset(adapter, &(pstate_rdh->uap_band_cfg), 0,
	       sizeof(pstate_rdh->uap_band_cfg));
	pstate_rdh->max_bcn_dtim_ms = 0;
	memset(adapter, pstate_rdh->priv_list, 0,
	       sizeof(pstate_rdh->priv_list));

	/* Initialize dfs channel switch count */
#define DFS_CS_COUNT 5
	adapter->dfs_cs_count = DFS_CS_COUNT;

	/* Initialize DFS testing struct */
	pdfs_test->user_cac_period_msec = 0;
	pdfs_test->user_nop_period_sec = 0;
	pdfs_test->no_channel_change_on_radar = MFALSE;
	pdfs_test->fixed_new_channel_on_radar = 0;
	pdfs_test->cac_restart = 0;
	pdfs_test->millisec_dwell_time = 0;
	adapter->dfs53cfg = adapter->init_para.dfs53cfg;

	LEAVE();
}

/**
 *  @brief Cleanup for the 11h parameters that allocated memory, etc.
 *
 *  @param adapter mlan_adapter structure
 *
 *  @return      N/A
 */
t_void wlan_11h_cleanup(mlan_adapter *adapter)
{
	wlan_dfs_device_state_t *pstate_dfs = &adapter->state_dfs;
	wlan_dfs_timestamp_t *pdfs_ts;

	ENTER();

	/* cleanup dfs_timestamp list */
	pdfs_ts = (wlan_dfs_timestamp_t *)util_peek_list(
		adapter->pmoal_handle, &pstate_dfs->dfs_ts_head, MNULL, MNULL);
	while (pdfs_ts) {
		util_unlink_list(adapter->pmoal_handle,
				 &pstate_dfs->dfs_ts_head,
				 (pmlan_linked_list)pdfs_ts, MNULL, MNULL);
		adapter->callbacks.moal_mfree(adapter->pmoal_handle,
					      (t_u8 *)pdfs_ts);

		pdfs_ts = (wlan_dfs_timestamp_t *)util_peek_list(
			adapter->pmoal_handle, &pstate_dfs->dfs_ts_head, MNULL,
			MNULL);
	}
	LEAVE();
}

/**
 *  @brief Initialize the 11h parameters and enable 11h when starting an IBSS
 *
 *  @param pmpriv Pointer to mlan_private structure
 *
 *  @return      N/A
 */
t_void wlan_11h_priv_init(mlan_private *pmpriv)
{
	wlan_11h_interface_state_t *pistate_11h = &pmpriv->intf_state_11h;

	ENTER();

	pistate_11h->is_11h_enabled = MTRUE;
	pistate_11h->is_11h_active = MFALSE;
	pistate_11h->adhoc_auto_sel_chan = MTRUE;
	pistate_11h->tx_disabled = MFALSE;
	pistate_11h->dfs_slave_csa_chan = 0;
	pistate_11h->dfs_slave_csa_expire_at_sec = 0;

	LEAVE();
}

/**
 *  @brief Retrieve channel closed for operation by Channel Switch Announcement
 *
 *  After receiving CSA, we must not transmit in any form on the original
 *    channel for a certain duration.  This checks the time, and returns
 *    the channel if valid.
 *
 *  @param priv  Private driver information structure
 *
 *  @return      Closed channel, else 0
 */
t_u8 wlan_11h_get_csa_closed_channel(mlan_private *priv)
{
	t_u32 sec, usec;

	ENTER();

	if (!priv->intf_state_11h.dfs_slave_csa_chan) {
		LEAVE();
		return 0;
	}

	/* have csa channel, check if expired or not */
	priv->adapter->callbacks.moal_get_system_time(
		priv->adapter->pmoal_handle, &sec, &usec);
	if (sec > priv->intf_state_11h.dfs_slave_csa_expire_at_sec) {
		/* expired:  remove channel from blacklist table, and clear vars
		 */
		wlan_set_chan_blacklist(priv, BAND_A,
					priv->intf_state_11h.dfs_slave_csa_chan,
					MFALSE);
		priv->intf_state_11h.dfs_slave_csa_chan = 0;
		priv->intf_state_11h.dfs_slave_csa_expire_at_sec = 0;
	}

	LEAVE();
	return priv->intf_state_11h.dfs_slave_csa_chan;
}

/**
 *  @brief Check if the current region's regulations require the input channel
 *         to be scanned for radar.
 *
 *  Based on statically defined requirements for sub-bands per regulatory
 *    agency requirements.
 *
 *  Used in adhoc start to determine if channel availability check is required
 *
 *  @param priv    Private driver information structure
 *  @param channel Channel to determine radar detection requirements
 *
 *  @return
 *    - MTRUE if radar detection is required
 *    - MFALSE otherwise
 */
/**  @sa wlan_11h_issue_radar_detect
 */
t_bool wlan_11h_radar_detect_required(mlan_private *priv, t_u8 channel)
{
	t_bool required = MFALSE;

	ENTER();

	/*
	 * No checks for 11h or measurement code being enabled is placed here
	 * since regulatory requirements exist whether we support them or not.
	 */

	required = wlan_get_cfp_radar_detect(priv, channel);

	if (!priv->adapter->region_code) {
		PRINTM(MINFO,
		       "11h: Radar detection in CFP code BG:%#x "
		       ", A:%#x "
		       "is %srequired for channel %d\n",
		       priv->adapter->cfp_code_bg, priv->adapter->cfp_code_a,
		       (required ? "" : "not "), channel);
	} else
		PRINTM(MINFO,
		       "11h: Radar detection in region %#02x "
		       "is %srequired for channel %d\n",
		       priv->adapter->region_code, (required ? "" : "not "),
		       channel);

	if (required == MTRUE && priv->media_connected == MTRUE &&
	    priv->curr_bss_params.bss_descriptor.channel == channel) {
		required = MFALSE;

		PRINTM(MINFO, "11h: Radar detection not required. "
			      "Already operating on the channel\n");
	}

	LEAVE();
	return required;
}

t_s32 wlan_11h_cancel_radar_detect(mlan_private *priv)
{
	t_s32 ret;
	HostCmd_DS_CHAN_RPT_REQ chan_rpt_req;
	memset(priv->adapter, &chan_rpt_req, 0x00, sizeof(chan_rpt_req));
	ret = wlan_prepare_cmd(priv, HostCmd_CMD_CHAN_REPORT_REQUEST,
			       HostCmd_ACT_GEN_SET, 0, (t_void *)MNULL,
			       (t_void *)&chan_rpt_req);
	return ret;
}

/**
 *  @brief Perform a radar measurement if required on given channel
 *
 *  Check to see if the provided channel requires a channel availability
 *    check (60 second radar detection measurement).  If required, perform
 *    measurement, stalling calling thread until the measurement completes
 *    and then report result.
 *
 *  Used when starting an adhoc or AP network.
 *
 *  @param priv         Private driver information structure
 *  @param pioctl_req   Pointer to IOCTL request buffer
 *  @param channel      Channel on which to perform radar measurement
 *  @param bandcfg      Channel Band config structure
 *
 *  @return
 *    - MTRUE  if radar measurement request was successfully issued
 *    - MFALSE if radar detection is not required
 *    - < 0 for error during radar detection (if performed)
 *
 *  @sa wlan_11h_radar_detect_required
 */
t_s32 wlan_11h_issue_radar_detect(mlan_private *priv,
				  pmlan_ioctl_req pioctl_req, t_u8 channel,
				  Band_Config_t bandcfg)
{
	t_s32 ret;
	HostCmd_DS_CHAN_RPT_REQ chan_rpt_req;
	mlan_adapter *pmadapter = priv->adapter;
	mlan_ds_11h_cfg *ds_11hcfg = MNULL;

	ENTER();

	ret = wlan_11h_radar_detect_required(priv, channel);
	if (ret) {
		/* Prepare and issue CMD_CHAN_RPT_REQ. */
		memset(priv->adapter, &chan_rpt_req, 0x00,
		       sizeof(chan_rpt_req));

		chan_rpt_req.chan_desc.startFreq = START_FREQ_11A_BAND;

		if (pmadapter->chanrpt_param_bandcfg) {
			chan_rpt_req.chan_desc.bandcfg = bandcfg;
		} else {
			*((t_u8 *)&chan_rpt_req.chan_desc.bandcfg) =
				(t_u8)bandcfg.chanWidth;
		}

		chan_rpt_req.chan_desc.chanNum = channel;
		chan_rpt_req.millisec_dwell_time =
			WLAN_11H_CHANNEL_AVAIL_CHECK_DURATION;

		/* ETSI new requirement for ch 120, 124 and 128 */
		if (wlan_is_etsi_country(pmadapter, pmadapter->country_code)) {
			if (channel == 120 || channel == 124 ||
			    channel == 128) {
				chan_rpt_req.millisec_dwell_time =
					WLAN_11H_CHANNEL_AVAIL_CHECK_DURATION *
					10;
			}
			if (channel == 116 &&
			    ((bandcfg.chanWidth == CHAN_BW_40MHZ) ||
			     (bandcfg.chanWidth == CHAN_BW_80MHZ))) {
				chan_rpt_req.millisec_dwell_time =
					WLAN_11H_CHANNEL_AVAIL_CHECK_DURATION *
					10;
			}
		}

		/* Save dwell time information to be used later in moal */
		if (pioctl_req) {
			ds_11hcfg = (mlan_ds_11h_cfg *)pioctl_req->pbuf;
			if (!ds_11hcfg->param.chan_rpt_req.host_based) {
				ds_11hcfg->param.chan_rpt_req
					.millisec_dwell_time =
					chan_rpt_req.millisec_dwell_time;
			}
		}

		if (priv->adapter->dfs_test_params.user_cac_period_msec) {
			PRINTM(MCMD_D,
			       "dfs_testing - user CAC period=%d (msec)\n",
			       priv->adapter->dfs_test_params
				       .user_cac_period_msec);
			chan_rpt_req.millisec_dwell_time =
				priv->adapter->dfs_test_params
					.user_cac_period_msec;
		}
		if (priv->adapter->dfs_test_params.cac_restart) {
			priv->adapter->dfs_test_params.chan =
				chan_rpt_req.chan_desc.chanNum;
			if (chan_rpt_req.millisec_dwell_time)
				priv->adapter->dfs_test_params
					.millisec_dwell_time =
					chan_rpt_req.millisec_dwell_time;
			else
				chan_rpt_req.millisec_dwell_time =
					priv->adapter->dfs_test_params
						.millisec_dwell_time;
			memcpy_ext(priv->adapter,
				   &priv->adapter->dfs_test_params.bandcfg,
				   &bandcfg, sizeof(bandcfg), sizeof(bandcfg));
		}
		PRINTM(MMSG,
		       "11h: issuing DFS Radar check for channel=%d."
		       "  Please wait for response...\n",
		       channel);

		ret = wlan_prepare_cmd(priv, HostCmd_CMD_CHAN_REPORT_REQUEST,
				       HostCmd_ACT_GEN_SET, 0,
				       (t_void *)pioctl_req,
				       (t_void *)&chan_rpt_req);
	}

	LEAVE();
	return ret;
}

/**
 *  @brief Checks if a radar measurement was performed on channel,
 *         and if so, whether radar was detected on it.
 *
 *  Used when starting an adhoc network.
 *
 *  @param priv         Private driver information structure
 *  @param chan         Channel to check upon
 *
 *  @return
 *    - MLAN_STATUS_SUCCESS if no radar on channel
 *    - MLAN_STATUS_FAILURE if radar was found on channel
 *    - (TBD??) MLAN_STATUS_PENDING if radar report NEEDS TO BE REISSUED
 *
 *  @sa wlan_11h_issue_radar_detect
 *  @sa wlan_11h_process_start
 */
mlan_status wlan_11h_check_chan_report(mlan_private *priv, t_u8 chan)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	wlan_dfs_device_state_t *pstate_dfs = &priv->adapter->state_dfs;
	t_u32 sec, usec;

	ENTER();

	/* check report we hold is valid or not */
	priv->adapter->callbacks.moal_get_system_time(
		priv->adapter->pmoal_handle, &sec, &usec);

	PRINTM(MINFO, "11h: %s()\n", __func__);
	PRINTM(MINFO, "- sec_now=%d, sec_report=%d.\n", sec,
	       pstate_dfs->dfs_report_time_sec);
	PRINTM(MINFO, "- rpt_channel=%d, rpt_radar=%d.\n",
	       pstate_dfs->dfs_check_channel, pstate_dfs->dfs_radar_found);

	if ((!pstate_dfs->dfs_check_pending) &&
	    (chan == pstate_dfs->dfs_check_channel) &&
	    ((sec - pstate_dfs->dfs_report_time_sec) <
	     MAX_DFS_REPORT_USABLE_AGE_SEC)) {
		/* valid and not out-dated, check if radar */
		if (pstate_dfs->dfs_radar_found) {
			PRINTM(MMSG, "Radar was detected on channel %d.\n",
			       chan);
			ret = MLAN_STATUS_FAILURE;
		}
	} else {
		/* When Cache is not valid. This is required during extending
		 * cache validity during bss_stop
		 */
		pstate_dfs->dfs_check_channel = 0;

		/*TODO:  reissue report request if not pending.
		 *       BUT HOW to make the code wait for it???
		 * For now, just fail since we don't have the info. */

		ret = MLAN_STATUS_PENDING;
	}

	LEAVE();
	return ret;
}

/**
 *  @brief Process an TLV buffer for a pending BSS Join command for
 *         both adhoc and infra networks
 *
 *  The TLV command processing for a BSS join for either adhoc or
 *    infrastructure network is performed with this function.  The
 *    capability bits are inspected for the IBSS flag and the appropriate
 *    local routines are called to setup the necessary TLVs.
 *
 *  Activate 11h functionality in the firmware if the spectrum management
 *    capability bit is found in the network information for the BSS we are
 *    joining.
 *
 *  @param priv          Private driver information structure
 *  @param ppbuffer      Output parameter: Pointer to the TLV output buffer,
 *                       modified on return to point after the appended 11h TLVs
 *  @param pcap_info     Pointer to the capability info for the BSS to join
 *  @param band          Band on which we are joining the BSS
 *  @param channel       Channel on which we are joining the BSS
 *  @param p11h_bss_info Pointer to the 11h BSS information for this
 *                       network that was parsed out of the scan response.
 *
 *  @return              Integer number of bytes appended to the TLV output
 *                       buffer (ppbuffer)
 */
t_s32 wlan_11h_process_join(mlan_private *priv, t_u8 **ppbuffer,
			    IEEEtypes_CapInfo_t *pcap_info, t_u16 band,
			    t_u32 channel, wlan_11h_bss_info_t *p11h_bss_info)
{
	t_s32 ret = 0;

	ENTER();

	if (priv->media_connected == MTRUE) {
		if (wlan_11h_is_active(priv) == p11h_bss_info->sensed_11h) {
			/*
			 * Assume DFS parameters are the same for roaming as
			 * long as the current & next APs have the same spectrum
			 * mgmt capability bit setting
			 */
			PRINTM(MINFO,
			       "Assume DFS parameters are the same for roaming\n");
		} else {
			/* No support for roaming between DFS/non-DFS yet */
			PRINTM(MINFO,
			       "No support for roaming between DFS/non-DFS yet\n");
		}

		LEAVE();
		return ret;
	}

	if (p11h_bss_info->sensed_11h) {
		if (!wlan_fw_11d_is_enabled(priv)) {
			/* No use having 11h enabled without 11d enabled */
			if (wlan_11d_enable(priv, MNULL, ENABLE_11D)) {
				PRINTM(MERROR, "Fail to enable 11D\n");
				LEAVE();
				return ret;
			}
#ifdef STA_SUPPORT
			wlan_11d_parse_dnld_countryinfo(
				priv, priv->pattempted_bss_desc);
#endif
		}
		/*
		 * Activate 11h functions in firmware,
		 * turns on capability bit
		 */
		wlan_11h_activate(priv, MNULL, MTRUE);
		pcap_info->spectrum_mgmt = MTRUE;

		/* If using a DFS channel, enable radar detection. */
		if ((band & BAND_A) &&
		    wlan_11h_radar_detect_required(priv, channel)) {
			if (!wlan_11h_is_slave_radar_det_active(priv))
				wlan_11h_config_slave_radar_det(priv, MTRUE);
		}
		wlan_11h_check_update_radar_det_state(priv);

		{
			PRINTM(MINFO, "11h: Infra join: Sensed\n");
			ret = wlan_11h_process_infra_join(
				priv, ppbuffer, band, channel, p11h_bss_info);
		}
	} else {
	}

	LEAVE();
	return ret;
}

/**
 *
 *  @brief  Prepare the HostCmd_DS_Command structure for an 11h command.
 *
 *  Use the Command field to determine if the command being set up is for
 *     11h and call one of the local command handlers accordingly for:
 *
 *        - HostCmd_CMD_802_11_TPC_ADAPT_REQ
 *        - HostCmd_CMD_802_11_TPC_INFO
 *        - HostCmd_CMD_802_11_CHAN_SW_ANN
 */
/**       - HostCmd_CMD_CHAN_REPORT_REQUEST
 */
/**
 *  @param priv      Private driver information structure
 *  @param pcmd_ptr  Output parameter: Pointer to the command being prepared
 *                   for the firmware
 *  @param pinfo_buf Void buffer pass through with data necessary for a
 *                   specific command type
 */
/**  @return          MLAN_STATUS_SUCCESS, MLAN_STATUS_FAILURE
 *                    or MLAN_STATUS_PENDING
 */
/**  @sa wlan_11h_cmd_tpc_request
 *  @sa wlan_11h_cmd_tpc_info
 *  @sa wlan_11h_cmd_chan_sw_ann
 */
/** @sa wlan_11h_cmd_chan_report_req
 */
mlan_status wlan_11h_cmd_process(mlan_private *priv,
				 HostCmd_DS_COMMAND *pcmd_ptr,
				 t_void *pinfo_buf)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;

	ENTER();
	switch (pcmd_ptr->command) {
	case HostCmd_CMD_802_11_TPC_ADAPT_REQ:
		ret = wlan_11h_cmd_tpc_request(priv, pcmd_ptr, pinfo_buf);
		break;
	case HostCmd_CMD_802_11_TPC_INFO:
		ret = wlan_11h_cmd_tpc_info(priv, pcmd_ptr, pinfo_buf);
		break;
	case HostCmd_CMD_802_11_CHAN_SW_ANN:
		ret = wlan_11h_cmd_chan_sw_ann(priv, pcmd_ptr, pinfo_buf);
		break;
	case HostCmd_CMD_CHAN_REPORT_REQUEST:
		ret = wlan_11h_cmd_chan_rpt_req(priv, pcmd_ptr, pinfo_buf);
		break;
	default:
		ret = MLAN_STATUS_FAILURE;
	}

	pcmd_ptr->command = wlan_cpu_to_le16(pcmd_ptr->command);
	pcmd_ptr->size = wlan_cpu_to_le16(pcmd_ptr->size);

	LEAVE();
	return ret;
}

/**
 *  @brief Handle the command response from the firmware if from an 11h command
 *
 *  Use the Command field to determine if the command response being
 *    is for 11h.  Call the local command response handler accordingly for:
 *
 *        - HostCmd_CMD_802_11_TPC_ADAPT_REQ
 *        - HostCmd_CMD_802_11_TPC_INFO
 *        - HostCmd_CMD_802_11_CHAN_SW_ANN
 */
/**       - HostCmd_CMD_CHAN_REPORT_REQUEST
 */
/**
 *  @param priv  Private driver information structure
 *  @param resp  HostCmd_DS_COMMAND struct returned from the firmware
 *               command
 *
 *  @return      MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status wlan_11h_cmdresp_process(mlan_private *priv,
				     const HostCmd_DS_COMMAND *resp)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;

	ENTER();
	switch (resp->command) {
	case HostCmd_CMD_802_11_TPC_ADAPT_REQ:
		HEXDUMP("11h: TPC REQUEST Rsp:", (t_u8 *)resp,
			(t_u32)resp->size);
		memcpy_ext(priv->adapter, priv->adapter->curr_cmd->pdata_buf,
			   &resp->params.tpc_req,
			   sizeof(HostCmd_DS_802_11_TPC_ADAPT_REQ),
			   sizeof(HostCmd_DS_802_11_TPC_ADAPT_REQ));
		break;

	case HostCmd_CMD_802_11_TPC_INFO:
		HEXDUMP("11h: TPC INFO Rsp Data:", (t_u8 *)resp,
			(t_u32)resp->size);
		break;

	case HostCmd_CMD_802_11_CHAN_SW_ANN:
		PRINTM(MINFO, "11h: Ret ChSwAnn: Sz=%u, Seq=%u, Ret=%u\n",
		       resp->size, resp->seq_num, resp->result);
		break;

	case HostCmd_CMD_CHAN_REPORT_REQUEST:
		if (priv->bss_type == MLAN_BSS_TYPE_DFS)
			break;
		priv->adapter->state_dfs.dfs_check_priv = priv;
		priv->adapter->state_dfs.dfs_check_pending = MTRUE;

		if (resp->params.chan_rpt_req.millisec_dwell_time == 0) {
			/* from wlan_11h_ioctl_dfs_chan_report */
			priv->adapter->state_dfs.dfs_check_pending = MFALSE;
			priv->adapter->state_dfs.dfs_check_priv = MNULL;
			priv->adapter->state_dfs.dfs_check_channel = 0;
			PRINTM(MINFO, "11h: Cancelling Chan Report \n");
		} else {
			PRINTM(MERROR,
			       "11h: Ret ChanRptReq.  Set dfs_check_pending and wait"
			       " for EVENT_CHANNEL_REPORT.\n");
		}

		break;

	default:
		ret = MLAN_STATUS_FAILURE;
	}

	LEAVE();
	return ret;
}

/**
 *  @brief Process an element from a scan response, copy relevant info for 11h
 *
 *  @param pmadapter Pointer to mlan_adapter
 *  @param p11h_bss_info Output parameter: Pointer to the 11h BSS information
 *                       for the network that is being processed
 *  @param pelement      Pointer to the current IE we are inspecting for 11h
 *                       relevance
 *
 *  @return              MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status wlan_11h_process_bss_elem(mlan_adapter *pmadapter,
				      wlan_11h_bss_info_t *p11h_bss_info,
				      const t_u8 *pelement)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	t_u8 element_len = *((const t_u8 *)pelement + 1);

	ENTER();
	switch (*pelement) {
	case POWER_CONSTRAINT:
		PRINTM(MINFO, "11h: Power Constraint IE Found\n");
		p11h_bss_info->sensed_11h = MTRUE;
		memcpy_ext(pmadapter, &p11h_bss_info->power_constraint,
			   pelement, element_len + sizeof(IEEEtypes_Header_t),
			   sizeof(IEEEtypes_PowerConstraint_t));
		p11h_bss_info->power_constraint.len =
			MIN(element_len, (sizeof(IEEEtypes_PowerConstraint_t) -
					  sizeof(IEEEtypes_Header_t)));
		break;

	case POWER_CAPABILITY:
		PRINTM(MINFO, "11h: Power Capability IE Found\n");
		p11h_bss_info->sensed_11h = MTRUE;
		memcpy_ext(pmadapter, &p11h_bss_info->power_capability,
			   pelement, element_len + sizeof(IEEEtypes_Header_t),
			   sizeof(IEEEtypes_PowerCapability_t));
		p11h_bss_info->power_capability.len =
			MIN(element_len, (sizeof(IEEEtypes_PowerCapability_t) -
					  sizeof(IEEEtypes_Header_t)));
		break;

	case TPC_REPORT:
		PRINTM(MINFO, "11h: Tpc Report IE Found\n");
		p11h_bss_info->sensed_11h = MTRUE;
		memcpy_ext(pmadapter, &p11h_bss_info->tpc_report, pelement,
			   element_len + sizeof(IEEEtypes_Header_t),
			   sizeof(IEEEtypes_TPCReport_t));
		p11h_bss_info->tpc_report.len =
			MIN(element_len, (sizeof(IEEEtypes_TPCReport_t) -
					  sizeof(IEEEtypes_Header_t)));
		break;

	case CHANNEL_SWITCH_ANN:
		PRINTM(MINFO, "11h: Channel Switch Ann IE Found\n");
		p11h_bss_info->sensed_11h = MTRUE;
		memcpy_ext(pmadapter, &p11h_bss_info->chan_switch_ann, pelement,
			   element_len + sizeof(IEEEtypes_Header_t),
			   sizeof(IEEEtypes_ChanSwitchAnn_t));
		p11h_bss_info->chan_switch_ann.len =
			MIN(element_len, (sizeof(IEEEtypes_ChanSwitchAnn_t) -
					  sizeof(IEEEtypes_Header_t)));
		break;

	case QUIET:
		PRINTM(MINFO, "11h: Quiet IE Found\n");
		p11h_bss_info->sensed_11h = MTRUE;
		memcpy_ext(pmadapter, &p11h_bss_info->quiet, pelement,
			   element_len + sizeof(IEEEtypes_Header_t),
			   sizeof(IEEEtypes_Quiet_t));
		p11h_bss_info->quiet.len =
			MIN(element_len, (sizeof(IEEEtypes_Quiet_t) -
					  sizeof(IEEEtypes_Header_t)));
		break;

	case SUPPORTED_CHANNELS:
	case TPC_REQUEST:
		/*
		 * These elements are not in beacons/probe responses.
		 * Included here to cover set of enumerated 11h elements.
		 */
		break;

	default:
		ret = MLAN_STATUS_FAILURE;
	}

	LEAVE();
	return ret;
}

/**
 *  @brief Driver handling for CHANNEL_SWITCH_ANN event
 *
 *  @param priv Pointer to mlan_private
 *
 *  @return MLAN_STATUS_SUCCESS, MLAN_STATUS_FAILURE or MLAN_STATUS_PENDING
 */
mlan_status wlan_11h_handle_event_chanswann(mlan_private *priv)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
#ifdef STA_SUPPORT
	mlan_deauth_param deauth_param;
#endif
	t_u32 sec, usec;
#ifdef UAP_SUPPORT
	mlan_adapter *pmadapter = priv->adapter;
	int i;
	t_u8 radar_detected = MFALSE;
	mlan_private *pmpriv = MNULL;
#endif

	ENTER();
#ifdef UAP_SUPPORT
	/** No need handle AP if mc_policy is disabled, FW will move the AP to
	 * client's new channel */
	if (pmadapter->mc_policy &&
	    priv->adapter->state_11h.is_master_radar_det_active) {
		for (i = 0; i < MIN(pmadapter->priv_num, MLAN_MAX_BSS_NUM);
		     i++) {
			if (pmadapter->priv[i] &&
			    (pmadapter->priv[i]->bss_role ==
			     MLAN_BSS_ROLE_UAP) &&
			    pmadapter->priv[i]->uap_bss_started &&
			    (priv->curr_bss_params.bss_descriptor.channel ==
			     pmadapter->priv[i]->uap_channel)) {
				PRINTM(MCMND,
				       "Receive channel switch Ann event on uap_channel=%d\n",
				       pmadapter->priv[i]->uap_channel);
				radar_detected = MTRUE;
				pmpriv = pmadapter->priv[i];
				break;
			}
		}
		if (radar_detected) {
			if (!pmpriv->intf_state_11h.is_11h_host) {
				if (pmadapter->state_rdh.stage == RDH_OFF) {
					pmadapter->state_rdh.stage =
						RDH_CHK_INTFS;
					wlan_11h_radar_detected_handling(
						pmadapter, pmpriv);
					if (pmpriv->uap_host_based)
						wlan_recv_event(
							pmpriv,
							MLAN_EVENT_ID_FW_RADAR_DETECTED,
							MNULL);
				} else {
					PRINTM(MEVENT,
					       "Ignore Event Radar Detected - handling already in progress.\n");
				}
			} else {
				if (pmpriv->adapter->dfs_test_params
					    .no_channel_change_on_radar ||
				    pmpriv->adapter->dfs_test_params
					    .fixed_new_channel_on_radar) {
					if (pmadapter->state_rdh.stage ==
						    RDH_OFF ||
					    pmadapter->state_rdh.stage ==
						    RDH_SET_CUSTOM_IE) {
						pmadapter->state_rdh.stage =
							RDH_CHK_INTFS;
						wlan_11h_radar_detected_handling(
							pmadapter, pmpriv);
					} else
						PRINTM(MEVENT,
						       "Ignore Event Radar Detected - handling already in progress.\n");
				} else {
					pmpriv->intf_state_11h.tx_disabled =
						MTRUE;
					wlan_recv_event(
						pmpriv,
						MLAN_EVENT_ID_FW_RADAR_DETECTED,
						MNULL);
				}
			}
		}
	}
#endif /* UAP_SUPPORT */
	if (priv->adapter->ecsa_enable) {
		t_u8 stop_tx = *(t_u8 *)priv->adapter->event_body;
		if (stop_tx)
			priv->adapter->state_rdh.tx_block = MTRUE;
		LEAVE();
		return ret;
	}
	priv->adapter->state_11h.recvd_chanswann_event = MTRUE;

	/* unlikely:  clean up previous csa if still on-going */
	if (priv->intf_state_11h.dfs_slave_csa_chan) {
		wlan_set_chan_blacklist(priv, BAND_A,
					priv->intf_state_11h.dfs_slave_csa_chan,
					MFALSE);
	}

	/* record channel and time of occurence */
	priv->intf_state_11h.dfs_slave_csa_chan =
		priv->curr_bss_params.bss_descriptor.channel;
	priv->adapter->callbacks.moal_get_system_time(
		priv->adapter->pmoal_handle, &sec, &usec);
	priv->intf_state_11h.dfs_slave_csa_expire_at_sec =
		sec + DFS_CHAN_MOVE_TIME;

#ifdef STA_SUPPORT
	/* do directed deauth.  recvd_chanswann_event flag will cause different
	 * reason code */
	PRINTM(MINFO, "11h: handle_event_chanswann() - sending deauth\n");
	memcpy_ext(priv->adapter, deauth_param.mac_addr,
		   &priv->curr_bss_params.bss_descriptor.mac_address,
		   MLAN_MAC_ADDR_LENGTH, MLAN_MAC_ADDR_LENGTH);
	deauth_param.reason_code = DEF_DEAUTH_REASON_CODE;
	ret = wlan_disconnect(priv, MNULL, &deauth_param);

	/* clear region table so next scan will be all passive */
	PRINTM(MINFO, "11h: handle_event_chanswann() - clear region table\n");
	wlan_11d_clear_parsedtable(priv);

	/* add channel to blacklist table */
	PRINTM(MINFO,
	       "11h: handle_event_chanswann() - scan blacklist csa channel\n");
	wlan_set_chan_blacklist(priv, BAND_A,
				priv->intf_state_11h.dfs_slave_csa_chan, MTRUE);
#endif /* STA_SUPPORT */

	priv->adapter->state_11h.recvd_chanswann_event = MFALSE;
	LEAVE();
	return ret;
}

/**
 *  @brief 802.11h DFS Testing configuration
 *
 *  @param pmadapter    Pointer to mlan_adapter
 *  @param pioctl_req   Pointer to mlan_ioctl_req
 *
 *  @return MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status wlan_11h_ioctl_dfs_testing(pmlan_adapter pmadapter,
				       pmlan_ioctl_req pioctl_req)
{
	mlan_ds_11h_cfg *ds_11hcfg = MNULL;
	mlan_ds_11h_dfs_testing *dfs_test = MNULL;
	wlan_dfs_testing_settings_t *pdfs_test_params = MNULL;

	ENTER();

	ds_11hcfg = (mlan_ds_11h_cfg *)pioctl_req->pbuf;
	dfs_test = &ds_11hcfg->param.dfs_testing;
	pdfs_test_params = &pmadapter->dfs_test_params;

	if (pioctl_req->action == MLAN_ACT_GET) {
		dfs_test->usr_cac_period_msec =
			pdfs_test_params->user_cac_period_msec;
		dfs_test->usr_nop_period_sec =
			pdfs_test_params->user_nop_period_sec;
		dfs_test->usr_no_chan_change =
			pdfs_test_params->no_channel_change_on_radar;
		dfs_test->usr_fixed_new_chan =
			pdfs_test_params->fixed_new_channel_on_radar;
		dfs_test->usr_cac_restart = pdfs_test_params->cac_restart;
	} else {
		pdfs_test_params->user_cac_period_msec =
			dfs_test->usr_cac_period_msec;
		pdfs_test_params->user_nop_period_sec =
			dfs_test->usr_nop_period_sec;
		pdfs_test_params->no_channel_change_on_radar =
			dfs_test->usr_no_chan_change;
		pdfs_test_params->fixed_new_channel_on_radar =
			dfs_test->usr_fixed_new_chan;
		pdfs_test_params->cac_restart = dfs_test->usr_cac_restart;
	}

	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief 802.11h IOCTL to get nop channel list
 *
 *  @param pmadapter    Pointer to mlan_adapter
 *  @param pioctl_req   Pointer to mlan_ioctl_req
 *
 *  @return MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status wlan_11h_ioctl_nop_channel_list(pmlan_adapter pmadapter,
					    pmlan_ioctl_req pioctl_req)
{
	mlan_ds_11h_cfg *ds_11hcfg = MNULL;
	int i, j;
	chan_freq_power_t *pcfp = MNULL;
	t_u8 num_chan = 0;
	ENTER();

	ds_11hcfg = (mlan_ds_11h_cfg *)pioctl_req->pbuf;
	/*get the cfp table first */
	for (i = 0; i < MAX_REGION_CHANNEL_NUM; i++) {
		if (pmadapter->region_channel[i].band == BAND_A) {
			pcfp = pmadapter->region_channel[i].pcfp;
			break;
		}
	}
	if (!pcfp) {
		/* This means operation in BAND-A is not support, we can
		 * just return false here, it's harmless
		 */
		goto done;
	}
	/*get the radar detection requirements according to chan num */
	for (j = 0; j < pmadapter->region_channel[i].num_cfp; j++) {
		if (pcfp[j].passive_scan_or_radar_detect) {
			if (wlan_11h_is_channel_under_nop(pmadapter,
							  pcfp[j].channel)) {
				ds_11hcfg->param.nop_chan_list
					.chan_list[num_chan] = pcfp[j].channel;
				num_chan++;
			}
		}
	}
done:
	ds_11hcfg->param.nop_chan_list.num_chan = num_chan;
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief 802.11h IOCTL to handle channel NOP status check/clear
 *  @brief If given channel is under NOP, return a new non-dfs
 *  @brief channel
 *
 *  @param pmadapter    Pointer to mlan_adapter
 *  @param pioctl_req   Pointer to mlan_ioctl_req
 *
 *  @return MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status wlan_11h_ioctl_channel_nop_info(pmlan_adapter pmadapter,
					    pmlan_ioctl_req pioctl_req)
{
	pmlan_private pmpriv = MNULL;
	mlan_ds_11h_cfg *ds_11hcfg = MNULL;
	t_s32 ret = MLAN_STATUS_FAILURE;
	mlan_ds_11h_chan_nop_info *ch_nop_info = MNULL;

	ENTER();

	if (pioctl_req) {
		pmpriv = pmadapter->priv[pioctl_req->bss_index];
		ds_11hcfg = (mlan_ds_11h_cfg *)pioctl_req->pbuf;
		ch_nop_info = &ds_11hcfg->param.ch_nop_info;

		if (pioctl_req->action == MLAN_ACT_GET) {
			ch_nop_info->chan_under_nop =
				wlan_11h_is_channel_under_nop(
					pmadapter, ch_nop_info->curr_chan);
			if (ch_nop_info->chan_under_nop &&
			    ch_nop_info->check_new_chan) {
				wlan_11h_switch_non_dfs_chan(
					pmpriv, &ch_nop_info->new_chan.channel);
				if (ch_nop_info->chan_width == CHAN_BW_80MHZ ||
				    ch_nop_info->chan_width == CHAN_BW_40MHZ)
					wlan_11h_update_bandcfg(
						pmpriv,
						&ch_nop_info->new_chan.bandcfg,
						ch_nop_info->new_chan.channel);
				if (ch_nop_info->chan_width == CHAN_BW_80MHZ)
					ch_nop_info->new_chan.center_chan =
						wlan_get_center_freq_idx(
							pmpriv,
							ch_nop_info->new_chan
								.bandcfg
								.chanBand,
							ch_nop_info->new_chan
								.channel,
							ch_nop_info->chan_width);
			}
		} else if (pioctl_req->action == MLAN_ACT_CLEAR) {
			wlan_11h_cleanup(pmadapter);
			wlan_reset_all_chan_dfs_state(pmpriv, BAND_A,
						      DFS_USABLE);
		}
		ret = MLAN_STATUS_SUCCESS;
	}

	LEAVE();
	return ret;
}

/**
 *  @brief 802.11h DFS Channel Switch Count Configuration
 *
 *  @param pmadapter    Pointer to mlan_adapter
 *  @param pioctl_req   Pointer to mlan_ioctl_req
 *
 *  @return MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status wlan_11h_ioctl_chan_switch_count(pmlan_adapter pmadapter,
					     pmlan_ioctl_req pioctl_req)
{
	mlan_ds_11h_cfg *ds_11hcfg = MNULL;
	t_s32 ret = MLAN_STATUS_FAILURE;

	ENTER();

	if (pioctl_req) {
		ds_11hcfg = (mlan_ds_11h_cfg *)pioctl_req->pbuf;

		if (pioctl_req->action == MLAN_ACT_GET) {
			ds_11hcfg->param.cs_count = pmadapter->dfs_cs_count;
		} else {
			pmadapter->dfs_cs_count = ds_11hcfg->param.cs_count;
		}
		ret = MLAN_STATUS_SUCCESS;
	}

	LEAVE();
	return ret;
}

/**
 *  @brief Get/Set 802.11h  channel dfs state
 *
 *  @param pmadapter    Pointer to mlan_adapter
 *  @param pioctl_req   Pointer to mlan_ioctl_req
 *
 *  @return MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status wlan_11h_ioctl_chan_dfs_state(pmlan_adapter pmadapter,
					  pmlan_ioctl_req pioctl_req)
{
	mlan_ds_11h_cfg *ds_11hcfg = MNULL;
	t_s32 ret = MLAN_STATUS_FAILURE;
	pmlan_private priv = MNULL;

	ENTER();

	if (pioctl_req) {
		ds_11hcfg = (mlan_ds_11h_cfg *)pioctl_req->pbuf;
		priv = pmadapter->priv[pioctl_req->bss_index];

		if (pioctl_req->action == MLAN_ACT_GET) {
			if (MFALSE ==
			    wlan_11h_is_channel_under_nop(
				    pmadapter,
				    ds_11hcfg->param.ch_dfs_state.channel))
				PRINTM(MINFO, "Channel is not in NOP\n");
			ds_11hcfg->param.ch_dfs_state.dfs_required =
				wlan_11h_radar_detect_required(
					priv,
					ds_11hcfg->param.ch_dfs_state.channel);
			if (ds_11hcfg->param.ch_dfs_state.dfs_required)
				ds_11hcfg->param.ch_dfs_state
					.dfs_state = wlan_get_chan_dfs_state(
					priv, BAND_A,
					ds_11hcfg->param.ch_dfs_state.channel);
		} else {
			if (ds_11hcfg->param.ch_dfs_state.dfs_state ==
			    DFS_UNAVAILABLE) {
				wlan_11h_add_dfs_timestamp(
					pmadapter, DFS_TS_REPR_NOP_START,
					ds_11hcfg->param.ch_dfs_state.channel);
			} else if (ds_11hcfg->param.ch_dfs_state.dfs_state ==
				   DFS_AVAILABLE) {
				if (MFALSE ==
				    wlan_11h_is_channel_under_nop(
					    pmadapter,
					    ds_11hcfg->param.ch_dfs_state
						    .channel))
					PRINTM(MINFO,
					       "Channel is not in NOP\n");
			}
			wlan_set_chan_dfs_state(
				priv, BAND_A,
				ds_11hcfg->param.ch_dfs_state.channel,
				ds_11hcfg->param.ch_dfs_state.dfs_state);
		}
		ret = MLAN_STATUS_SUCCESS;
	}

	LEAVE();
	return ret;
}

/**
 *  @brief 802.11h DFS chan report
 *
 *  @param priv         Pointer to mlan_private
 *  @param pioctl_req   Pointer to mlan_ioctl_req
 *
 *  @return MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status wlan_11h_ioctl_dfs_chan_report(mlan_private *priv,
					   pmlan_ioctl_req pioctl_req)
{
	mlan_ds_11h_cfg *ds_11hcfg = MNULL;
	HostCmd_DS_CHAN_RPT_REQ *chan_rpt_req = MNULL;
	t_s32 ret;

	ENTER();

	ds_11hcfg = (mlan_ds_11h_cfg *)pioctl_req->pbuf;

	chan_rpt_req =
		(HostCmd_DS_CHAN_RPT_REQ *)&ds_11hcfg->param.chan_rpt_req;

	ret = wlan_prepare_cmd(priv, HostCmd_CMD_CHAN_REPORT_REQUEST,
			       HostCmd_ACT_GEN_SET, 0, (t_void *)pioctl_req,
			       (t_void *)chan_rpt_req);

	if (ret == MLAN_STATUS_SUCCESS)
		ret = MLAN_STATUS_PENDING;

	LEAVE();
	return ret;
}

/**
 *  @brief Check if channel is under NOP (Non-Occupancy Period)
 *  If so, the channel should not be used until the period expires.
 *
 *  @param pmadapter  Pointer to mlan_adapter
 *  @param channel    Channel number
 *
 *  @return MTRUE or MFALSE
 */
t_bool wlan_11h_is_channel_under_nop(mlan_adapter *pmadapter, t_u8 channel)
{
	wlan_dfs_timestamp_t *pdfs_ts = MNULL;
	t_u32 now_sec, now_usec;
	t_bool ret = MFALSE;
	mlan_private *priv;
	ENTER();
	pdfs_ts = wlan_11h_find_dfs_timestamp(pmadapter, channel);

	if (pdfs_ts && (pdfs_ts->channel == channel) &&
	    (pdfs_ts->represents == DFS_TS_REPR_NOP_START)) {
		/* found NOP_start timestamp entry on channel */
		pmadapter->callbacks.moal_get_system_time(
			pmadapter->pmoal_handle, &now_sec, &now_usec);
		if (pmadapter->dfs_test_params.user_nop_period_sec) {
			PRINTM(MCMD_D,
			       "dfs_testing - user NOP period=%d (sec)\n",
			       pmadapter->dfs_test_params.user_nop_period_sec);
			if ((now_sec - pdfs_ts->ts_sec) <=
			    pmadapter->dfs_test_params.user_nop_period_sec) {
				ret = MTRUE;
			}
		} else {
			if ((now_sec - pdfs_ts->ts_sec) <=
			    WLAN_11H_NON_OCCUPANCY_PERIOD)
				ret = MTRUE;
		}

		/* if entry is expired, remove it */
		if (!ret) {
			wlan_11h_remove_dfs_timestamp(pmadapter, pdfs_ts);
			priv = wlan_get_priv(pmadapter, MLAN_BSS_ROLE_ANY);
			if (priv)
				wlan_set_chan_dfs_state(priv, BAND_A, channel,
							DFS_USABLE);
		} else
			PRINTM(MMSG,
			       "11h: channel %d is under NOP - can't use.\n",
			       channel);
	}

	LEAVE();
	return ret;
}

/**
 *  @brief Driver handling for CHANNEL_REPORT_RDY event
 *  This event will have the channel report data appended.
 *
 *  @param priv     Pointer to mlan_private
 *  @param pevent   Pointer to mlan_event
 *  @param radar_chan 	Pointer to radar channel
 *  @param bandwidth    Pointer to band width
 *
 *  @return MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status wlan_11h_handle_event_chanrpt_ready(mlan_private *priv,
						mlan_event *pevent,
						t_u8 *radar_chan,
						t_u8 *bandwidth)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	HostCmd_DS_CHAN_RPT_RSP *pchan_rpt_rsp;
	MrvlIEtypes_Data_t *ptlv;
	MeasRptBasicMap_t *pmeas_rpt_basic;
	t_u8 *pbuffer;
	t_s32 evt_len;
	t_u16 tlv_len;
	t_u32 sec, usec;
	wlan_dfs_device_state_t *pstate_dfs = &priv->adapter->state_dfs;
	t_u8 dfs_radar_found = MFALSE;
	t_u8 dfs_check_channel = pstate_dfs->dfs_check_channel;
	t_u8 dfs_check_bandwidth = pstate_dfs->dfs_check_bandwidth;
	MrvlIEtypes_channel_band_t *tlv;

	ENTER();
	pchan_rpt_rsp = (HostCmd_DS_CHAN_RPT_RSP *)&pevent->event_buf;
	DBG_HEXDUMP(MCMD_D, "11h: Event ChanRptReady (HostCmd_DS_CHAN_RPT_RSP)",
		    (t_u8 *)pchan_rpt_rsp, pevent->event_len);

	if (priv->bss_type == MLAN_BSS_TYPE_DFS) {
		dfs_check_channel = priv->chan_rep_req.chanNum;
		dfs_check_bandwidth = priv->chan_rep_req.bandcfg.chanWidth;
	}

	if (wlan_le32_to_cpu(pchan_rpt_rsp->cmd_result) ==
	    MLAN_CMD_RESULT_SUCCESS) {
		pbuffer = (t_u8 *)&pchan_rpt_rsp->tlv_buffer;
		evt_len = pevent->event_len;
		evt_len -= sizeof(HostCmd_DS_CHAN_RPT_RSP) -
			   sizeof(pchan_rpt_rsp->tlv_buffer);

		while (evt_len >= (t_s32)sizeof(MrvlIEtypesHeader_t)) {
			ptlv = (MrvlIEtypes_Data_t *)pbuffer;
			tlv_len = wlan_le16_to_cpu(ptlv->header.len);

			switch (wlan_le16_to_cpu(ptlv->header.type)) {
			case TLV_TYPE_CHANRPT_11H_BASIC:
				pmeas_rpt_basic =
					(MeasRptBasicMap_t *)&ptlv->data;
				if (pmeas_rpt_basic->radar)
					dfs_radar_found = MTRUE;
				break;
			case TLV_TYPE_CHANNELBANDLIST:
				tlv = (MrvlIEtypes_channel_band_t *)ptlv;
				dfs_check_channel = tlv->channel;
				dfs_check_bandwidth = tlv->bandcfg.chanWidth;
				break;
			default:
				break;
			}

			pbuffer += (tlv_len + sizeof(ptlv->header));
			evt_len -= (tlv_len + sizeof(ptlv->header));
			evt_len = (evt_len > 0) ? evt_len : 0;
		}
	} else {
		ret = MLAN_STATUS_FAILURE;
	}
	if (dfs_radar_found) {
		PRINTM(MMSG, "RADAR Detected on channel %d bw=%d !\n",
		       dfs_check_channel, dfs_check_bandwidth);
		/* add channel to NOP list */
		wlan_11h_add_all_dfs_timestamp(priv->adapter,
					       DFS_TS_REPR_NOP_START,
					       dfs_check_channel,
					       dfs_check_bandwidth);
	}
	*radar_chan = dfs_check_channel;
	*bandwidth = dfs_check_bandwidth;
	if (dfs_radar_found)
		wlan_11h_set_chan_dfs_state(priv, dfs_check_channel,
					    dfs_check_bandwidth,
					    DFS_UNAVAILABLE);
	else
		wlan_11h_set_chan_dfs_state(priv, dfs_check_channel,
					    dfs_check_bandwidth, DFS_AVAILABLE);
	pstate_dfs->dfs_radar_found = dfs_radar_found;
	/* Update DFS structure. */
	priv->adapter->callbacks.moal_get_system_time(
		priv->adapter->pmoal_handle, &sec, &usec);
	pstate_dfs->dfs_report_time_sec = sec;
	pstate_dfs->dfs_check_pending = MFALSE;

	LEAVE();
	return ret;
}

/**
 *  @brief Print debug info in RADAR_DETECTED event
 *  This event may have the dfs record data appended.
 *
 *  @param priv   		Pointer to mlan_private
 *  @param pevent 		Pointer to mlan_event
 *  @param radar_chan 	Pointer to radar channel
 *  @param bandwidth    Pointer to band width
 *
 *  @return MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status wlan_11h_print_event_radar_detected(mlan_private *priv,
						mlan_event *pevent,
						t_u8 *radar_chan,
						t_u8 *bandwidth)
{
	wlan_dfs_device_state_t *pstate_dfs = &priv->adapter->state_dfs;
	t_u8 dfs_check_bandwidth = pstate_dfs->dfs_check_bandwidth;
	MrvlIEtypes_channel_band_t *tlv;
	ENTER();
	*radar_chan = pstate_dfs->dfs_check_channel;
	if (pevent->event_len >= sizeof(MrvlIEtypes_channel_band_t)) {
		tlv = (MrvlIEtypes_channel_band_t *)&pevent->event_buf;
		*radar_chan = tlv->channel;
		dfs_check_bandwidth = tlv->bandcfg.chanWidth;
	} else {
		if (priv->bss_type == MLAN_BSS_TYPE_DFS) {
			*radar_chan = priv->chan_rep_req.chanNum;
			dfs_check_bandwidth =
				priv->chan_rep_req.bandcfg.chanWidth;
		}
	}
	*bandwidth = dfs_check_bandwidth;
	wlan_11h_add_all_dfs_timestamp(priv->adapter, DFS_TS_REPR_NOP_START,
				       *radar_chan, dfs_check_bandwidth);
	wlan_11h_set_chan_dfs_state(priv, *radar_chan, dfs_check_bandwidth,
				    DFS_UNAVAILABLE);
	PRINTM(MEVENT, "DFS: Radar detected on %d bw=%d\n", *radar_chan,
	       dfs_check_bandwidth);
	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief Check if RADAR_DETECTED handling is blocking data tx
 *
 *  @param pmadapter    Pointer to mlan_adapter
 *
 *  @return MTRUE or MFALSE
 */
t_bool wlan_11h_radar_detected_tx_blocked(mlan_adapter *pmadapter)
{
	if (pmadapter->state_rdh.tx_block)
		return MTRUE;
	switch (pmadapter->state_rdh.stage) {
	case RDH_OFF:
	case RDH_CHK_INTFS:
	case RDH_STOP_TRAFFIC:
		return MFALSE;
	}
	return MTRUE;
}

/**
 *  @brief Callback for RADAR_DETECTED event driver handling
 *
 *  @param priv    Void pointer to mlan_private
 *
 *  @return MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status wlan_11h_radar_detected_callback(t_void *priv)
{
	mlan_status ret;
	ENTER();
	ret = wlan_11h_radar_detected_handling(
		((mlan_private *)(priv))->adapter, (mlan_private *)priv);
	LEAVE();
	return ret;
}

#ifdef UAP_SUPPORT
/**
 *  @brief Function for handling sta disconnect event in dfs_repeater mode
 *
 *  @param pmadapter	pointer to mlan_adapter
 *
 *  @return NONE
 */
void wlan_dfs_rep_disconnect(mlan_adapter *pmadapter)
{
	mlan_private *priv_list[MLAN_MAX_BSS_NUM];
	mlan_private *pmpriv = MNULL;
	t_u8 pcount, i;

	memset(pmadapter, priv_list, 0x00, sizeof(priv_list));
	pcount = wlan_get_privs_by_cond(pmadapter, wlan_is_intf_active,
					priv_list);

	/* Stop all the active BSSes */
	for (i = 0; i < pcount; i++) {
		pmpriv = priv_list[i];

		if (GET_BSS_ROLE(pmpriv) != MLAN_BSS_ROLE_UAP)
			continue;

		if (wlan_11h_radar_detect_required(pmpriv,
						   pmadapter->dfsr_channel)) {
			mlan_status ret = MLAN_STATUS_SUCCESS;
			ret = wlan_prepare_cmd(pmpriv,
					       HostCmd_CMD_APCMD_BSS_STOP,
					       HostCmd_ACT_GEN_SET, 0, MNULL,
					       MNULL);
			if (ret) {
				PRINTM(MMSG, "Error sending message to FW\n");
			}
		}
	}
}

/**
 *  @brief Function for handling sta BW change event in dfs_repeater mode
 *
 *  @param pmadapter	pointer to mlan_adapter
 *
 *  @return NONE
 */
void wlan_dfs_rep_bw_change(mlan_adapter *pmadapter)
{
	mlan_private *priv_list[MLAN_MAX_BSS_NUM];
	mlan_private *pmpriv = MNULL;
	t_u8 pcount, i;
	mlan_status ret = MLAN_STATUS_SUCCESS;

	memset(pmadapter, priv_list, 0x00, sizeof(priv_list));
	pcount = wlan_get_privs_by_cond(pmadapter, wlan_is_intf_active,
					priv_list);
	if (pcount == 1) {
		pmpriv = priv_list[0];
		if (GET_BSS_ROLE(pmpriv) == MLAN_BSS_ROLE_STA) {
			PRINTM(MMSG,
			       "dfs-repeater: BW change detected\n"
			       "no active priv's, skip event handling.\n");
			return;
		}
	}

	/* Stop all the active BSSes */
	for (i = 0; i < pcount; i++) {
		pmpriv = priv_list[i];

		if (GET_BSS_ROLE(pmpriv) == MLAN_BSS_ROLE_UAP) {
			/* Check if uAPs running on non-dfs channel. If they do
			 * then there is no need to restart the uAPs
			 */
			if (!wlan_11h_radar_detect_required(
				    pmpriv, pmadapter->dfsr_channel))
				return;

			ret = wlan_prepare_cmd(pmpriv,
					       HostCmd_CMD_APCMD_BSS_STOP,
					       HostCmd_ACT_GEN_SET, 0, MNULL,
					       MNULL);
			if (ret) {
				PRINTM(MERROR, "Error sending message to FW\n");
			}
		}
	}

	/* Start all old active BSSes */
	for (i = 0; i < pcount; i++) {
		pmpriv = priv_list[i];

		if (GET_BSS_ROLE(pmpriv) == MLAN_BSS_ROLE_UAP) {
			ret = wlan_prepare_cmd(pmpriv,
					       HostCmd_CMD_APCMD_BSS_START,
					       HostCmd_ACT_GEN_SET, 0, MNULL,
					       MNULL);
			if (ret) {
				PRINTM(MERROR, "Error sending message to FW\n");
			}
		}
	}
}
#endif

/**
 *  @brief Update band config for the new channel
 *
 *  @param uap_band_cfg  uap's old channel's band configuration
 *  @param new_channel   new channel that the device is switching to
 *
 *  @return MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE or MLAN_STATUS_PENDING
 */
void wlan_11h_update_bandcfg(mlan_private *pmpriv, Band_Config_t *uap_band_cfg,
			     t_u8 new_channel)
{
	t_u8 chan_offset;
	ENTER();

	/* Update the channel offset for 20MHz, 40MHz and 80MHz
	 * Clear the channel bandwidth for 20MHz
	 * since channel switch could be happening from 40/80MHz to 20MHz
	 */
	chan_offset = wlan_get_second_channel_offset(pmpriv, new_channel);
	uap_band_cfg->chan2Offset = chan_offset;

	if (!chan_offset) { /* 40MHz/80MHz */
		PRINTM(MCMD_D, "20MHz channel, clear channel bandwidth\n");
		uap_band_cfg->chanWidth = CHAN_BW_20MHZ;
	}
	LEAVE();
}

#ifdef UAP_SUPPORT
/**
 * @brief Get priv current index -- this is used to enter correct rdh_state
 * during radar handling
 *
 * @param pmpriv           Pointer to mlan_private
 * @param pstate_rdh       Pointer to radar detected state handler
 *
 * @return                 MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status
wlan_11h_get_priv_curr_idx(mlan_private *pmpriv,
			   wlan_radar_det_hndlg_state_t *pstate_rdh)
{
	t_bool found = MFALSE;
	ENTER();

	PRINTM(MINFO, "%s:pmpriv =%p\n", __func__, pmpriv);
	while ((++pstate_rdh->priv_curr_idx) < pstate_rdh->priv_list_count) {
		if (pmpriv ==
		    pstate_rdh->priv_list[pstate_rdh->priv_curr_idx]) {
			PRINTM(MINFO, "found matching priv: priv_idx=%d\n",
			       pstate_rdh->priv_curr_idx);
			found = MTRUE;
			break;
		}
	}
	return (found == MTRUE) ? MLAN_STATUS_SUCCESS : MLAN_STATUS_FAILURE;
}
#endif

/**
 *  @brief Driver handling for remove customeie
 *
 *  @param pmadapter    Pointer to mlan_adapter
 *  @param pmpriv       Pointer to mlan_private
 *
 *  @return MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE or MLAN_STATUS_PENDING
 */
mlan_status wlan_11h_remove_custom_ie(mlan_adapter *pmadapter,
				      mlan_private *pmpriv)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	wlan_radar_det_hndlg_state_t *pstate_rdh = &pmadapter->state_rdh;
	mlan_ioctl_req *pioctl_req = MNULL;

	ENTER();
	if (pstate_rdh->stage == RDH_SET_CUSTOM_IE) {
		pstate_rdh->priv_curr_idx = RDH_STAGE_FIRST_ENTRY_PRIV_IDX;
		PRINTM(MMSG, "Removing CHAN_SW IE from interfaces.\n");
		while ((++pstate_rdh->priv_curr_idx) <
		       pstate_rdh->priv_list_count) {
			pmpriv =
				pstate_rdh->priv_list[pstate_rdh->priv_curr_idx];
			if (!wlan_11h_is_dfs_master(pmpriv))
				continue;
			ret = wlan_11h_prepare_custom_ie_chansw(
				pmadapter, &pioctl_req, MFALSE);
			if ((ret != MLAN_STATUS_SUCCESS) || !pioctl_req) {
				PRINTM(MERROR,
				       "%s(): Error in preparing CHAN_SW IE.\n",
				       __func__);
				goto done;
			}

			pioctl_req->bss_index = pmpriv->bss_index;
			ret = wlan_misc_ioctl_custom_ie_list(
				pmadapter, pioctl_req, MFALSE);
			if (ret != MLAN_STATUS_SUCCESS &&
			    ret != MLAN_STATUS_PENDING) {
				PRINTM(MERROR,
				       "%s(): Could not remove IE for priv=%p [priv_bss_idx=%d]!\n",
				       __func__, pmpriv, pmpriv->bss_index);
				/* TODO: hiow to handle this error case??
				 * ignore & continue? */
			}
			/* free ioctl buffer memory before we leave */
			pmadapter->callbacks.moal_mfree(pmadapter->pmoal_handle,
							(t_u8 *)pioctl_req);
		}
	}
done:
	LEAVE();
	return ret;
}

/**
 *  @brief Driver handling for RADAR_DETECTED event
 *
 *  @param pmadapter    Pointer to mlan_adapter
 *  @param pmpriv       Pointer to mlan_private
 *
 *  @return MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE or MLAN_STATUS_PENDING
 */
mlan_status wlan_11h_radar_detected_handling(mlan_adapter *pmadapter,
					     mlan_private *pmpriv)
{
#ifdef DEBUG_LEVEL1
	const char *rdh_stage_str[] = {"RDH_OFF",
				       "RDH_CHK_INTFS",
				       "RDH_STOP_TRAFFIC",
				       "RDH_GET_INFO_CHANNEL",
				       "RDH_GET_INFO_BEACON_DTIM",
				       "RDH_SET_CUSTOM_IE",
				       "RDH_REM_CUSTOM_IE",
				       "RDH_STOP_INTFS",
				       "RDH_SET_NEW_CHANNEL",
				       "RDH_RESTART_INTFS",
				       "RDH_RESTART_TRAFFIC"};
#endif

	mlan_status ret = MLAN_STATUS_SUCCESS;
	t_u32 i;
	wlan_radar_det_hndlg_state_t *pstate_rdh = &pmadapter->state_rdh;

	ENTER();

	if (!pmpriv) {
		PRINTM(MERROR, "Invalid radar priv -- Exit radar handling\n");
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}

	switch (pstate_rdh->stage) {
	case RDH_CHK_INTFS:
		PRINTM(MCMD_D, "%s(): stage(%d)=%s\n", __func__,
		       pstate_rdh->stage, rdh_stage_str[pstate_rdh->stage]);

		/* get active interfaces */
		memset(pmadapter, pstate_rdh->priv_list, 0x00,
		       sizeof(pstate_rdh->priv_list));
		pstate_rdh->priv_list_count = wlan_get_privs_by_cond(
			pmadapter, wlan_is_intf_active, pstate_rdh->priv_list);
		PRINTM(MCMD_D, "%s():  priv_list_count = %d\n", __func__,
		       pstate_rdh->priv_list_count);
		for (i = 0; i < pstate_rdh->priv_list_count; i++)
			PRINTM(MINFO, "%s():  priv_list[%d] = %p\n", __func__,
			       i, pstate_rdh->priv_list[i]);

		if (pstate_rdh->priv_list_count == 0) {
			/* no interfaces active... nothing to do */
			PRINTM(MMSG, "11h: Radar Detected - no active priv's,"
				     " skip event handling.\n");
			pstate_rdh->stage = RDH_OFF;
			PRINTM(MCMD_D, "%s(): finished - stage(%d)=%s\n",
			       __func__, pstate_rdh->stage,
			       rdh_stage_str[pstate_rdh->stage]);
			break; /* EXIT CASE */
		}

		/* else: start handling */
		pstate_rdh->curr_channel = 0;
		pstate_rdh->new_channel = 0;
		memset(pmadapter, &(pstate_rdh->uap_band_cfg), 0,
		       sizeof(pstate_rdh->uap_band_cfg));
		pstate_rdh->max_bcn_dtim_ms = 0;
		pstate_rdh->priv_curr_idx = RDH_STAGE_FIRST_ENTRY_PRIV_IDX;
		pstate_rdh->stage = RDH_STOP_TRAFFIC;
		/* fall through */

	case RDH_STOP_TRAFFIC:
		PRINTM(MCMD_D, "%s(): stage(%d)=%s\n", __func__,
		       pstate_rdh->stage, rdh_stage_str[pstate_rdh->stage]);

		PRINTM(MMSG,
		       "11h: Radar Detected - stopping host tx traffic.\n");
		for (i = 0; i < pstate_rdh->priv_list_count; i++)
			wlan_11h_tx_disable(pstate_rdh->priv_list[i]);

		pstate_rdh->priv_curr_idx = RDH_STAGE_FIRST_ENTRY_PRIV_IDX;
		pstate_rdh->stage = RDH_GET_INFO_CHANNEL;
		/* fall through */

	case RDH_GET_INFO_CHANNEL:
		PRINTM(MCMD_D, "%s(): stage(%d)=%s, priv_idx=%d\n", __func__,
		       pstate_rdh->stage, rdh_stage_str[pstate_rdh->stage],
		       pstate_rdh->priv_curr_idx);

		/* here, prefer STA info over UAP info - one less CMD to send */
		if (pstate_rdh->priv_curr_idx ==
		    RDH_STAGE_FIRST_ENTRY_PRIV_IDX) {
#ifdef UAP_SUPPORT
			if (GET_BSS_ROLE(pmpriv) == MLAN_BSS_ROLE_UAP) {
				ret = wlan_11h_get_priv_curr_idx(pmpriv,
								 pstate_rdh);
				if (ret != MLAN_STATUS_SUCCESS) {
					PRINTM(MERROR,
					       "Unable to locate pmpriv in current active priv_list\n");
					break; /* EXIT CASE */
				}

				/* send cmd to get first UAP's info */
				pmpriv->uap_state_chan_cb.pioctl_req_curr =
					MNULL;
				pmpriv->uap_state_chan_cb.get_chan_callback =
					wlan_11h_radar_detected_callback;
				ret = wlan_uap_get_channel(pmpriv);
				break; /* EXIT CASE */
			} else
#endif
			{
				/* Assume all STAs on same channel, find first
				 * STA */
				MASSERT(pstate_rdh->priv_list_count > 0);
				for (i = 0; i < pstate_rdh->priv_list_count;
				     i++) {
					pmpriv = pstate_rdh->priv_list[i];
					if (GET_BSS_ROLE(pmpriv) ==
					    MLAN_BSS_ROLE_STA)
						break;
				}
				/* STA info kept in driver, just copy */
				pstate_rdh->curr_channel =
					pmpriv->curr_bss_params.bss_descriptor
						.channel;
			}
		}
#ifdef UAP_SUPPORT
		else if (pstate_rdh->priv_curr_idx <
			 pstate_rdh->priv_list_count) {
			/* repeat entry: UAP return with info */
			pstate_rdh->curr_channel =
				pmpriv->uap_state_chan_cb.channel;
			pstate_rdh->uap_band_cfg =
				pmpriv->uap_state_chan_cb.bandcfg;
			PRINTM(MCMD_D,
			       "%s(): uap_band_cfg=0x%02x curr_chan=%d, curr_idx=%d bss_role=%d\n",
			       __func__, pstate_rdh->uap_band_cfg,
			       pstate_rdh->curr_channel,
			       pstate_rdh->priv_curr_idx, GET_BSS_ROLE(pmpriv));
		}
#endif

		/* add channel to NOP list */
		wlan_11h_add_dfs_timestamp(pmadapter, DFS_TS_REPR_NOP_START,
					   pstate_rdh->curr_channel);

		/* choose new channel (!= curr channel) and move on */
#ifdef UAP_SUPPORT
		if (GET_BSS_ROLE(pmpriv) == MLAN_BSS_ROLE_UAP)
			pstate_rdh->new_channel =
				wlan_11h_get_uap_start_channel(
					pmpriv,
					pmpriv->uap_state_chan_cb.bandcfg);
		else
#endif

			if (!pstate_rdh->new_channel ||
			    (pstate_rdh->new_channel ==
			     pstate_rdh->curr_channel)) { /* report error */
			PRINTM(MERROR,
			       "%s():  ERROR - Failed to choose new_chan"
			       " (!= curr_chan) !!\n",
			       __func__);
#ifdef UAP_SUPPORT
			if (GET_BSS_ROLE(pmpriv) == MLAN_BSS_ROLE_UAP) {
				ret = wlan_prepare_cmd(
					pmpriv, HostCmd_CMD_APCMD_BSS_STOP,
					HostCmd_ACT_GEN_SET, 0, MNULL, MNULL);
				PRINTM(MERROR,
				       "STOP UAP and exit radar handling...\n");
				pstate_rdh->stage = RDH_OFF;
				break; /* leads to exit case */
			}
#endif
		}
		if (!pmadapter->dfs_test_params.no_channel_change_on_radar &&
		    pmadapter->dfs_test_params.fixed_new_channel_on_radar) {
			PRINTM(MCMD_D, "dfs_testing - user fixed new_chan=%d\n",
			       pmadapter->dfs_test_params
				       .fixed_new_channel_on_radar);
			pstate_rdh->new_channel =
				pmadapter->dfs_test_params
					.fixed_new_channel_on_radar;
		}
		/* applies to DFS with ECSA support */
		if (pmadapter->dfs_test_params.no_channel_change_on_radar) {
			pstate_rdh->new_channel = pstate_rdh->curr_channel;
		}
		PRINTM(MCMD_D, "%s():  curr_chan=%d, new_chan=%d\n", __func__,
		       pstate_rdh->curr_channel, pstate_rdh->new_channel);

		pstate_rdh->priv_curr_idx = RDH_STAGE_FIRST_ENTRY_PRIV_IDX;
		pstate_rdh->stage = RDH_GET_INFO_BEACON_DTIM;
		/* fall through */

	case RDH_GET_INFO_BEACON_DTIM:
		PRINTM(MCMD_D, "%s(): stage(%d)=%s, priv_idx=%d\n", __func__,
		       pstate_rdh->stage, rdh_stage_str[pstate_rdh->stage],
		       pstate_rdh->priv_curr_idx);

#ifdef UAP_SUPPORT
		/* check all intfs in this stage to find longest period */
		/* UAP intf callback returning with info */
		if (pstate_rdh->priv_curr_idx < pstate_rdh->priv_list_count) {
			t_u16 bcn_dtim_msec;
			pmpriv =
				pstate_rdh->priv_list[pstate_rdh->priv_curr_idx];
			PRINTM(MCMD_D, "%s():  uap.bcn_pd=%d, uap.dtim_pd=%d\n",
			       __func__,
			       pmpriv->uap_state_chan_cb.beacon_period,
			       pmpriv->uap_state_chan_cb.dtim_period);
			bcn_dtim_msec =
				(pmpriv->uap_state_chan_cb.beacon_period *
				 pmpriv->uap_state_chan_cb.dtim_period);
			if (bcn_dtim_msec > pstate_rdh->max_bcn_dtim_ms)
				pstate_rdh->max_bcn_dtim_ms = bcn_dtim_msec;
		}
#endif

		/* check next intf */
		while ((++pstate_rdh->priv_curr_idx) <
			       pstate_rdh->priv_list_count &&
		       (pstate_rdh->priv_curr_idx < MLAN_MAX_BSS_NUM)) {
			pmpriv =
				pstate_rdh->priv_list[pstate_rdh->priv_curr_idx];

#ifdef UAP_SUPPORT
			if (GET_BSS_ROLE(pmpriv) == MLAN_BSS_ROLE_UAP) {
				pmpriv->uap_state_chan_cb.pioctl_req_curr =
					MNULL;
				pmpriv->uap_state_chan_cb.get_chan_callback =
					wlan_11h_radar_detected_callback;
				ret = wlan_uap_get_beacon_dtim(pmpriv);
				break; /* leads to exit case */
			} else
#endif
			{ /* get STA info from driver and compare here */
				t_u16 bcn_pd_msec = 100;
				t_u16 dtim_pd_msec = 1;
				t_u16 bcn_dtim_msec;

				/* adhoc creator */
				{
					bcn_pd_msec = pmpriv->curr_bss_params
							      .bss_descriptor
							      .beacon_period;
					/* if (priv->bss_mode !=
					 * MLAN_BSS_MODE_IBSS) */
					/* TODO: mlan_scan.c needs to parse TLV
					 * 0x05 (TIM) for dtim_period */
				}
				PRINTM(MCMD_D,
				       "%s():  sta.bcn_pd=%d, sta.dtim_pd=%d\n",
				       __func__, bcn_pd_msec, dtim_pd_msec);
				bcn_dtim_msec = (bcn_pd_msec * dtim_pd_msec);
				if (bcn_dtim_msec > pstate_rdh->max_bcn_dtim_ms)
					pstate_rdh->max_bcn_dtim_ms =
						bcn_dtim_msec;
			}
		}

		if (pstate_rdh->priv_curr_idx < pstate_rdh->priv_list_count)
			break; /* EXIT CASE (for UAP) */
		/* else */
		pstate_rdh->priv_curr_idx = RDH_STAGE_FIRST_ENTRY_PRIV_IDX;
		pstate_rdh->stage = RDH_SET_CUSTOM_IE;
		/* fall through */

	case RDH_SET_CUSTOM_IE:
		PRINTM(MCMD_D, "%s(): stage(%d)=%s, priv_idx=%d\n", __func__,
		       pstate_rdh->stage, rdh_stage_str[pstate_rdh->stage],
		       pstate_rdh->priv_curr_idx);

		/* add CHAN_SW IE - Need apply on each interface */
		if (pstate_rdh->priv_curr_idx ==
		    RDH_STAGE_FIRST_ENTRY_PRIV_IDX) {
			mlan_ioctl_req *pioctl_req = MNULL;
			PRINTM(MMSG,
			       "11h: Radar Detected - adding CHAN_SW IE to interfaces.\n");
			while ((++pstate_rdh->priv_curr_idx) <
			       pstate_rdh->priv_list_count) {
				pmpriv = pstate_rdh->priv_list
						 [pstate_rdh->priv_curr_idx];
				if (!wlan_11h_is_dfs_master(pmpriv))
					continue;
				ret = wlan_11h_prepare_custom_ie_chansw(
					pmadapter, &pioctl_req, MTRUE);
				if ((ret != MLAN_STATUS_SUCCESS) ||
				    !pioctl_req) {
					PRINTM(MERROR,
					       "%s(): Error in preparing CHAN_SW IE.\n",
					       __func__);
					break; /* EXIT CASE */
				}

				pioctl_req->bss_index = pmpriv->bss_index;
				ret = wlan_misc_ioctl_custom_ie_list(
					pmadapter, pioctl_req, MFALSE);
				if (ret != MLAN_STATUS_SUCCESS &&
				    ret != MLAN_STATUS_PENDING) {
					PRINTM(MERROR,
					       "%s(): Could not set IE for priv=%p [priv_bss_idx=%d]!\n",
					       __func__, pmpriv,
					       pmpriv->bss_index);
					/* TODO: how to handle this error case??
					 * ignore & continue? */
				}
				/* free ioctl buffer memory before we leave */
				pmadapter->callbacks.moal_mfree(
					pmadapter->pmoal_handle,
					(t_u8 *)pioctl_req);
			}
			break; /* EXIT CASE */
		}
		/* else */
		pstate_rdh->priv_curr_idx = RDH_STAGE_FIRST_ENTRY_PRIV_IDX;
		pstate_rdh->stage = RDH_REM_CUSTOM_IE;
		/* fall through */

	case RDH_REM_CUSTOM_IE:
		PRINTM(MCMD_D, "%s(): stage(%d)=%s, priv_idx=%d\n", __func__,
		       pstate_rdh->stage, rdh_stage_str[pstate_rdh->stage],
		       pstate_rdh->priv_curr_idx);

		/* remove CHAN_SW IE - Need apply on each interface */
		if (pstate_rdh->priv_curr_idx ==
		    RDH_STAGE_FIRST_ENTRY_PRIV_IDX) {
			mlan_ioctl_req *pioctl_req = MNULL;
			/*
			 * first entry to this stage, do delay
			 * DFS requires a minimum of 5 chances for clients to
			 * hear this IE. Use delay:  5 beacons <=
			 * (BCN_DTIM_MSEC*5) <= 3 seconds).
			 */
			t_u16 delay_ms =
				MAX(MIN_RDH_CHAN_SW_IE_PERIOD_MSEC,
				    MIN((4 * pstate_rdh->max_bcn_dtim_ms),
					MAX_RDH_CHAN_SW_IE_PERIOD_MSEC));
			PRINTM(MMSG,
			       "11h: Radar Detected - delay %d ms for FW to"
			       " broadcast CHAN_SW IE.\n",
			       delay_ms);
			wlan_mdelay(pmadapter, delay_ms);
			PRINTM(MMSG,
			       "11h: Radar Detected - delay over, removing"
			       " CHAN_SW IE from interfaces.\n");
			while ((++pstate_rdh->priv_curr_idx) <
			       pstate_rdh->priv_list_count) {
				pmpriv = pstate_rdh->priv_list
						 [pstate_rdh->priv_curr_idx];
				if (!wlan_11h_is_dfs_master(pmpriv))
					continue;
				ret = wlan_11h_prepare_custom_ie_chansw(
					pmadapter, &pioctl_req, MFALSE);
				if ((ret != MLAN_STATUS_SUCCESS) ||
				    !pioctl_req) {
					PRINTM(MERROR,
					       "%s(): Error in preparing CHAN_SW IE.\n",
					       __func__);
					break; /* EXIT CASE */
				}

				pioctl_req->bss_index = pmpriv->bss_index;
				ret = wlan_misc_ioctl_custom_ie_list(
					pmadapter, pioctl_req, MFALSE);
				if (ret != MLAN_STATUS_SUCCESS &&
				    ret != MLAN_STATUS_PENDING) {
					PRINTM(MERROR,
					       "%s(): Could not remove IE for priv=%p [priv_bss_idx=%d]!\n",
					       __func__, pmpriv,
					       pmpriv->bss_index);
					/* TODO: hiow to handle this error
					 * case??  ignore & continue? */
				}
				/* free ioctl buffer memory before we leave */
				pmadapter->callbacks.moal_mfree(
					pmadapter->pmoal_handle,
					(t_u8 *)pioctl_req);
			}
			break; /* EXIT CASE */
		}
		/* else */
		pstate_rdh->priv_curr_idx = RDH_STAGE_FIRST_ENTRY_PRIV_IDX;
		pstate_rdh->stage = RDH_STOP_INTFS;
		/* fall through */

	case RDH_STOP_INTFS:
		PRINTM(MCMD_D, "%s(): stage(%d)=%s, priv_idx=%d\n", __func__,
		       pstate_rdh->stage, rdh_stage_str[pstate_rdh->stage],
		       pstate_rdh->priv_curr_idx);

		/* issues one cmd (DEAUTH/ADHOC_STOP/BSS_STOP) to each intf */
		while ((++pstate_rdh->priv_curr_idx) <
		       pstate_rdh->priv_list_count) {
			pmpriv =
				pstate_rdh->priv_list[pstate_rdh->priv_curr_idx];
#ifdef UAP_SUPPORT
			if (GET_BSS_ROLE(pmpriv) == MLAN_BSS_ROLE_UAP) {
				ret = wlan_prepare_cmd(
					pmpriv, HostCmd_CMD_APCMD_BSS_STOP,
					HostCmd_ACT_GEN_SET, 0, MNULL, MNULL);
				break; /* leads to exit case */
			}
#endif
#ifdef STA_SUPPORT
			if (GET_BSS_ROLE(pmpriv) == MLAN_BSS_ROLE_STA) {
				if (pmpriv->media_connected == MTRUE) {
					wlan_disconnect(pmpriv, MNULL, MNULL);
					break; /* leads to exit case */
				}
			}
#endif
		}

		if (pstate_rdh->priv_curr_idx < pstate_rdh->priv_list_count ||
		    ret == MLAN_STATUS_FAILURE)
			break; /* EXIT CASE */
		/* else */
		pstate_rdh->priv_curr_idx = RDH_STAGE_FIRST_ENTRY_PRIV_IDX;
		pstate_rdh->stage = RDH_SET_NEW_CHANNEL;

		if (pmadapter->dfs_test_params.no_channel_change_on_radar) {
			PRINTM(MCMD_D,
			       "dfs_testing - no channel change on radar."
			       "  Overwrite new_chan = curr_chan.\n");
			pstate_rdh->new_channel = pstate_rdh->curr_channel;
			pstate_rdh->priv_curr_idx =
				RDH_STAGE_FIRST_ENTRY_PRIV_IDX;
			pstate_rdh->stage = RDH_RESTART_INTFS;
			goto rdh_restart_intfs; /* skip next stage */
		}
		/* fall through */

	case RDH_SET_NEW_CHANNEL:
		PRINTM(MCMD_D, "%s(): stage(%d)=%s, priv_idx=%d\n", __func__,
		       pstate_rdh->stage, rdh_stage_str[pstate_rdh->stage],
		       pstate_rdh->priv_curr_idx);

		/* only set new channel for UAP intfs */
		while ((++pstate_rdh->priv_curr_idx) <
		       pstate_rdh->priv_list_count) {
			pmpriv =
				pstate_rdh->priv_list[pstate_rdh->priv_curr_idx];
#ifdef UAP_SUPPORT
			if (GET_BSS_ROLE(pmpriv) == MLAN_BSS_ROLE_UAP) {
				pmpriv->uap_state_chan_cb.pioctl_req_curr =
					MNULL;
				pmpriv->uap_state_chan_cb.get_chan_callback =
					wlan_11h_radar_detected_callback;

				/* DFS only in 5GHz */
				wlan_11h_update_bandcfg(
					pmpriv, &pstate_rdh->uap_band_cfg,
					pstate_rdh->new_channel);
				PRINTM(MCMD_D,
				       "RDH_SET_NEW_CHANNEL: uAP band config = 0x%x channel=%d\n",
				       pstate_rdh->uap_band_cfg,
				       pstate_rdh->new_channel);

				ret = wlan_uap_set_channel(
					pmpriv, pstate_rdh->uap_band_cfg,
					pstate_rdh->new_channel);
				break; /* leads to exit case */
			}
#endif
		}

		if (pstate_rdh->priv_curr_idx < pstate_rdh->priv_list_count ||
		    ret == MLAN_STATUS_FAILURE)
			break; /* EXIT CASE (for UAP) */
		/* else */
		pstate_rdh->priv_curr_idx = RDH_STAGE_FIRST_ENTRY_PRIV_IDX;
		pstate_rdh->stage = RDH_RESTART_INTFS;
		/* fall through */

	case RDH_RESTART_INTFS:
	rdh_restart_intfs:
		PRINTM(MCMD_D, "%s(): stage(%d)=%s, priv_idx=%d\n", __func__,
		       pstate_rdh->stage, rdh_stage_str[pstate_rdh->stage],
		       pstate_rdh->priv_curr_idx);

		/* can only restart master intfs */
		while ((++pstate_rdh->priv_curr_idx) <
		       pstate_rdh->priv_list_count) {
			pmpriv =
				pstate_rdh->priv_list[pstate_rdh->priv_curr_idx];
#ifdef UAP_SUPPORT
			if (GET_BSS_ROLE(pmpriv) == MLAN_BSS_ROLE_UAP) {
				if (wlan_11h_radar_detect_required(
					    pmpriv, pstate_rdh->new_channel)) {
					/* Radar detection is required for this
					   channel, make sure 11h is activated
					   in the firmware */
					ret = wlan_11h_activate(pmpriv, MNULL,
								MTRUE);
					ret = wlan_11h_config_master_radar_det(
						pmpriv, MTRUE);
					ret = wlan_11h_check_update_radar_det_state(
						pmpriv);
				}
				ret = wlan_prepare_cmd(
					pmpriv, HostCmd_CMD_APCMD_BSS_START,
					HostCmd_ACT_GEN_SET, 0, MNULL, MNULL);
				break; /* leads to exit case */
			}
#endif
#ifdef STA_SUPPORT
#endif
		}

		if (pstate_rdh->priv_curr_idx < pstate_rdh->priv_list_count ||
		    ret == MLAN_STATUS_FAILURE)
			break; /* EXIT CASE (for UAP) */
		/* else */
		pstate_rdh->priv_curr_idx = RDH_STAGE_FIRST_ENTRY_PRIV_IDX;
		pstate_rdh->stage = RDH_RESTART_TRAFFIC;
		/* fall through */

	case RDH_RESTART_TRAFFIC:
		PRINTM(MCMD_D, "%s(): stage(%d)=%s\n", __func__,
		       pstate_rdh->stage, rdh_stage_str[pstate_rdh->stage]);
		/* remove custome ie */
		if (pmadapter->ecsa_enable) {
			mlan_ioctl_req *pioctl_req = MNULL;
			pstate_rdh->priv_curr_idx =
				RDH_STAGE_FIRST_ENTRY_PRIV_IDX;
			while ((++pstate_rdh->priv_curr_idx) <
			       pstate_rdh->priv_list_count) {
				pmpriv = pstate_rdh->priv_list
						 [pstate_rdh->priv_curr_idx];
				if (!wlan_11h_is_dfs_master(pmpriv))
					continue;
				ret = wlan_11h_prepare_custom_ie_chansw(
					pmadapter, &pioctl_req, MFALSE);
				if ((ret != MLAN_STATUS_SUCCESS) ||
				    !pioctl_req) {
					PRINTM(MERROR,
					       "%s(): Error in preparing CHAN_SW IE.\n",
					       __func__);
					break; /* EXIT CASE */
				}

				pioctl_req->bss_index = pmpriv->bss_index;

				ret = wlan_misc_ioctl_custom_ie_list(
					pmadapter, pioctl_req, MFALSE);
				if (ret != MLAN_STATUS_SUCCESS &&
				    ret != MLAN_STATUS_PENDING) {
					PRINTM(MERROR,
					       "%s(): Could not remove IE for priv=%p [priv_bss_idx=%d]!\n",
					       __func__, pmpriv,
					       pmpriv->bss_index);
					/* TODO: hiow to handle this error
					 * case??  ignore & continue? */
				}
				/* free ioctl buffer memory before we leave */
				pmadapter->callbacks.moal_mfree(
					pmadapter->pmoal_handle,
					(t_u8 *)pioctl_req);
			}
		}
		/* continue traffic for reactivated interfaces */
		PRINTM(MMSG,
		       "11h: Radar Detected - restarting host tx traffic.\n");
		for (i = 0; i < pstate_rdh->priv_list_count; i++)
			wlan_11h_tx_enable(pstate_rdh->priv_list[i]);

		pstate_rdh->stage = RDH_OFF; /* DONE! */
		PRINTM(MCMD_D, "%s(): finished - stage(%d)=%s\n", __func__,
		       pstate_rdh->stage, rdh_stage_str[pstate_rdh->stage]);

		break;

	default:
		pstate_rdh->stage = RDH_OFF; /* cancel RDH to unblock Tx packets
					      */
		break;
	}

	LEAVE();
	return ret;
}

/**
 *  @brief DFS Event Preprocessing.
 *  Operates directly on pmadapter variables.
 *
 *  1. EVENT_RADAR_DETECTED comes from firmware without specific
 *     bss_num/bss_type.  Find it an appropriate interface and
 *     update event_cause field in event_buf.
 *
 *  @param pmadapter    Pointer to mlan_adapter
 *
 *  @return    MLAN_STATUS_SUCCESS (update successful)
 *          or MLAN_STATUS_FAILURE (no change)
 */
mlan_status wlan_11h_dfs_event_preprocessing(mlan_adapter *pmadapter)
{
	mlan_status ret = MLAN_STATUS_FAILURE;
	mlan_private *pmpriv = MNULL;
	mlan_private *priv_list[MLAN_MAX_BSS_NUM] = {MNULL};

	ENTER();
	switch (pmadapter->event_cause & EVENT_ID_MASK) {
	case EVENT_RADAR_DETECTED:
		/* find active intf:  prefer dfs_master over dfs_slave */
		if (wlan_get_privs_by_two_cond(
			    pmadapter, wlan_11h_is_master_active_on_dfs_chan,
			    wlan_11h_is_dfs_master, MTRUE, priv_list)) {
			pmpriv = priv_list[0];
			PRINTM(MINFO, "%s: found dfs_master priv=%p\n",
			       __func__, pmpriv);
		} else if (wlan_get_privs_by_two_cond(
				   pmadapter,
				   wlan_11h_is_slave_active_on_dfs_chan,
				   wlan_11h_is_dfs_slave, MTRUE, priv_list)) {
			pmpriv = priv_list[0];
			PRINTM(MINFO, "%s: found dfs_slave priv=%p\n", __func__,
			       pmpriv);
		} else if (pmadapter->state_dfs.dfs_check_pending ||
			   pmadapter->state_dfs.dfs_check_channel) {
			pmpriv = (mlan_private *)(pmadapter->state_dfs
							  .dfs_check_priv);
			PRINTM(MINFO, "%s: found dfs priv=%p\n", __func__,
			       pmpriv);
		}

		/* update event_cause if we found an appropriate priv */
		if (pmpriv) {
			pmlan_buffer pmevbuf = pmadapter->pmlan_buffer_event;
			t_u32 new_event_cause =
				pmadapter->event_cause & EVENT_ID_MASK;
			new_event_cause |=
				((GET_BSS_NUM(pmpriv) & 0xff) << 16) |
				((pmpriv->bss_type & 0xff) << 24);
			PRINTM(MINFO, "%s: priv - bss_num=%d, bss_type=%d\n",
			       __func__, GET_BSS_NUM(pmpriv), pmpriv->bss_type);
			memcpy_ext(pmadapter,
				   pmevbuf->pbuf + pmevbuf->data_offset,
				   &new_event_cause, sizeof(new_event_cause),
				   sizeof(new_event_cause));
			ret = MLAN_STATUS_SUCCESS;
		} else {
			PRINTM(MERROR,
			       "Failed to find dfs master/slave priv\n");
			ret = MLAN_STATUS_FAILURE;
		}
		break;
	}

	LEAVE();
	return ret;
}

/**
 *  @brief try to switch to a non-dfs channel
 *
 *  @param priv    Void pointer to mlan_private
 *
 *  @param chan    pointer to channel
 *
 *  @return MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE or MLAN_STATUS_PENDING
 */
mlan_status wlan_11h_switch_non_dfs_chan(mlan_private *priv, t_u8 *chan)
{
	mlan_status ret = MLAN_STATUS_FAILURE;
	t_u32 i;
	t_u32 rand_entry;
	t_u8 def_chan;
	t_u8 rand_tries = 0;
	region_chan_t *chn_tbl = MNULL;
	pmlan_adapter pmadapter = priv->adapter;

	ENTER();

	if (!pmadapter->dfs_test_params.no_channel_change_on_radar &&
	    pmadapter->dfs_test_params.fixed_new_channel_on_radar) {
		PRINTM(MCMD_D, "dfs_testing - user fixed new_chan=%d\n",
		       pmadapter->dfs_test_params.fixed_new_channel_on_radar);
		*chan = pmadapter->dfs_test_params.fixed_new_channel_on_radar;

		LEAVE();
		return MLAN_STATUS_SUCCESS;
	}

	/*get the channel table first*/
	for (i = 0; i < MAX_REGION_CHANNEL_NUM; i++) {
		if (pmadapter->region_channel[i].band == BAND_A &&
		    pmadapter->region_channel[i].valid) {
			chn_tbl = &pmadapter->region_channel[i];
			break;
		}
	}

	if (!chn_tbl || !chn_tbl->pcfp)
		goto done;

	do {
		rand_entry =
			wlan_11h_get_random_num(pmadapter) % chn_tbl->num_cfp;
		def_chan = (t_u8)chn_tbl->pcfp[rand_entry].channel;
		rand_tries++;
	} while ((wlan_11h_is_channel_under_nop(pmadapter, def_chan) ||
		  chn_tbl->pcfp[rand_entry].passive_scan_or_radar_detect ==
			  MTRUE) &&
		 (rand_tries < MAX_SWITCH_CHANNEL_RETRIES));

	/* meet max retries, use the lowest non-dfs channel */
	if (rand_tries == MAX_SWITCH_CHANNEL_RETRIES) {
		for (i = 0; i < chn_tbl->num_cfp; i++) {
			if (chn_tbl->pcfp[i].passive_scan_or_radar_detect ==
				    MFALSE &&
			    !wlan_11h_is_channel_under_nop(
				    pmadapter,
				    (t_u8)chn_tbl->pcfp[i].channel)) {
				def_chan = (t_u8)chn_tbl->pcfp[i].channel;
				break;
			}
		}
		if (i == chn_tbl->num_cfp)
			goto done;
	}

	*chan = def_chan;
	ret = MLAN_STATUS_SUCCESS;
done:
	LEAVE();
	return ret;
}

/**
 *  @brief set dfs check channel
 *
 *  @param priv    Void pointer to mlan_private
 *
 *  @param chan    pointer to channel
 *  @param bandwidth    band width
 *
 *  @return  N/A
 */
void wlan_11h_set_dfs_check_chan(mlan_private *priv, t_u8 chan, t_u8 bandwidth)
{
	wlan_dfs_device_state_t *pstate_dfs = &priv->adapter->state_dfs;
	ENTER();
	pstate_dfs->dfs_check_channel = chan;
	pstate_dfs->dfs_check_bandwidth = bandwidth;
	PRINTM(MCMND, "Set dfs_check_channel=%d\n", chan);
	LEAVE();
}

/**
 *  @brief 802.11h DFS W53 configuration
 *
 *  @param pmadapter    Pointer to mlan_adapter
 *  @param pioctl_req   Pointer to mlan_ioctl_req
 *
 *  @return MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status wlan_11h_ioctl_dfs_w53_cfg(pmlan_adapter pmadapter,
				       pmlan_ioctl_req pioctl_req)
{
	mlan_ds_11h_cfg *ds_11hcfg = MNULL;
	mlan_ds_11h_dfs_w53_cfg *dfs_w53_cfg = MNULL;

	ENTER();

	ds_11hcfg = (mlan_ds_11h_cfg *)pioctl_req->pbuf;
	dfs_w53_cfg = &ds_11hcfg->param.dfs_w53_cfg;

	if (pioctl_req->action == MLAN_ACT_GET) {
		dfs_w53_cfg->dfs53cfg = pmadapter->dfs53cfg;
	} else {
		pmadapter->dfs53cfg = dfs_w53_cfg->dfs53cfg;
	}

	LEAVE();

	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief 802.11h DFS mode configuration
 *
 *  @param pmadapter    Pointer to mlan_adapter
 *  @param pioctl_req   Pointer to mlan_ioctl_req
 *
 *  @return MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status wlan_11h_ioctl_dfs_mode(pmlan_adapter pmadapter,
				    pmlan_ioctl_req pioctl_req)
{
	mlan_ds_11h_cfg *ds_11hcfg = MNULL;

	ENTER();

	ds_11hcfg = (mlan_ds_11h_cfg *)pioctl_req->pbuf;

	if (pioctl_req->action == MLAN_ACT_GET) {
		ds_11hcfg->param.dfs_mode = pmadapter->dfs_mode;
	} else {
		pmadapter->dfs_mode = ds_11hcfg->param.dfs_mode;
	}
	LEAVE();
	return MLAN_STATUS_SUCCESS;
}
