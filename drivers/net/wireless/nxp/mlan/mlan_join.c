/** @file mlan_join.c
 *
 *  @brief Functions implementing wlan infrastructure and adhoc join routines
 *
 *  IOCTL handlers as well as command preparation and response routines
 *  for sending adhoc start, adhoc join, and association commands
 *  to the firmware.
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
    10/30/2008: initial version
******************************************************/

#include "mlan.h"
#include "mlan_join.h"
#include "mlan_util.h"
#include "mlan_fw.h"
#include "mlan_main.h"
#include "mlan_wmm.h"
#include "mlan_11n.h"
#include "mlan_11ac.h"
#include "mlan_11ax.h"
#include "mlan_11h.h"
/********************************************************
			Local Constants
********************************************************/

/********************************************************
			Local Variables
********************************************************/

/********************************************************
			Global Variables
********************************************************/

/********************************************************
			Local Functions
********************************************************/
/**
 *  @brief Append a generic IE as a pass through TLV to a TLV buffer.
 *
 *  This function is called from the network join command prep. routine.
 *    If the IE buffer has been setup by the application, this routine appends
 *    the buffer as a pass through TLV type to the request.
 *
 *  @param priv     A pointer to mlan_private structure
 *  @param ppbuffer pointer to command buffer pointer
 *
 *  @return         bytes added to the buffer
 */
static int wlan_cmd_append_generic_ie(mlan_private *priv, t_u8 **ppbuffer)
{
	int ret_len = 0;
	MrvlIEtypesHeader_t ie_header;

	ENTER();

	/* Null Checks */
	if (ppbuffer == MNULL) {
		LEAVE();
		return 0;
	}
	if (*ppbuffer == MNULL) {
		LEAVE();
		return 0;
	}

	/*
	 * If there is a generic ie buffer setup, append it to the return
	 *   parameter buffer pointer.
	 */
	if (priv->gen_ie_buf_len) {
		PRINTM(MINFO, "append generic IE %d to %p\n",
		       priv->gen_ie_buf_len, *ppbuffer);

		/* Wrap the generic IE buffer with a pass through TLV type */
		ie_header.type = wlan_cpu_to_le16(TLV_TYPE_PASSTHROUGH);
		ie_header.len = wlan_cpu_to_le16(priv->gen_ie_buf_len);
		memcpy_ext(priv->adapter, *ppbuffer, &ie_header,
			   sizeof(ie_header), sizeof(ie_header));

		/* Increment the return size and the return buffer pointer param
		 */
		*ppbuffer += sizeof(ie_header);
		ret_len += sizeof(ie_header);

		/* Copy the generic IE buffer to the output buffer, advance
		 * pointer */
		memcpy_ext(priv->adapter, *ppbuffer, priv->gen_ie_buf,
			   priv->gen_ie_buf_len, priv->gen_ie_buf_len);

		/* Increment the return size and the return buffer pointer param
		 */
		*ppbuffer += priv->gen_ie_buf_len;
		ret_len += priv->gen_ie_buf_len;

		/* Reset the generic IE buffer */
		priv->gen_ie_buf_len = 0;
	}

	/* return the length appended to the buffer */
	LEAVE();
	return ret_len;
}

/**
 *  @brief Append  IE as a pass through TLV to a TLV buffer.
 *
 *  This routine appends IE as a pass through TLV type to the request.
 *
 *  @param priv     A pointer to mlan_private structure
 *  @param ie       A pointer to IE buffer
 *  @param ppbuffer pointer to command buffer pointer
 *
 *  @return         bytes added to the buffer
 */
static int wlan_cmd_append_pass_through_ie(mlan_private *priv,
					   IEEEtypes_Generic_t *ie,
					   t_u8 **ppbuffer)
{
	int ret_len = 0;
	MrvlIEtypesHeader_t ie_header;

	ENTER();

	/* Null Checks */
	if (ppbuffer == MNULL) {
		LEAVE();
		return 0;
	}
	if (*ppbuffer == MNULL) {
		LEAVE();
		return 0;
	}
	if (ie->ieee_hdr.len) {
		PRINTM(MINFO, "append generic IE %d to %p\n", ie->ieee_hdr.len,
		       *ppbuffer);

		/* Wrap the generic IE buffer with a pass through TLV type */
		ie_header.type = wlan_cpu_to_le16(TLV_TYPE_PASSTHROUGH);
		ie_header.len = wlan_cpu_to_le16(ie->ieee_hdr.len +
						 sizeof(IEEEtypes_Header_t));
		memcpy_ext(priv->adapter, *ppbuffer, &ie_header,
			   sizeof(ie_header), sizeof(ie_header));

		/* Increment the return size and the return buffer pointer param
		 */
		*ppbuffer += sizeof(ie_header);
		ret_len += sizeof(ie_header);

		/* Copy the generic IE buffer to the output buffer, advance
		 * pointer */
		memcpy_ext(priv->adapter, *ppbuffer, ie,
			   ie->ieee_hdr.len + sizeof(IEEEtypes_Header_t),
			   ie->ieee_hdr.len + sizeof(IEEEtypes_Header_t));

		/* Increment the return size and the return buffer pointer param
		 */
		*ppbuffer += ie->ieee_hdr.len + sizeof(IEEEtypes_Header_t);
		ret_len += ie->ieee_hdr.len + sizeof(IEEEtypes_Header_t);
	}
	/* return the length appended to the buffer */
	LEAVE();
	return ret_len;
}

/**
 *  @brief Append TSF tracking info from the scan table for the target AP
 *
 *  This function is called from the network join command prep. routine.
 *     The TSF table TSF sent to the firmware contains two TSF values:
 *        - the TSF of the target AP from its previous beacon/probe response
 *        - the TSF timestamp of our local MAC at the time we observed the
 *          beacon/probe response.
 *
 *     The firmware uses the timestamp values to set an initial TSF value
 *        in the MAC for the new association after a reassociation attempt.
 *
 *    @param pmpriv     A pointer to mlan_private structure
 *    @param ppbuffer   A pointer to command buffer pointer
 *    @param pbss_desc  A pointer to the BSS Descriptor from the scan table of
 *                      the AP we are trying to join
 *
 *    @return         bytes added to the buffer
 */
static int wlan_cmd_append_tsf_tlv(mlan_private *pmriv, t_u8 **ppbuffer,
				   BSSDescriptor_t *pbss_desc)
{
	MrvlIEtypes_TsfTimestamp_t tsf_tlv;
	t_u64 tsf_val;

	ENTER();

	/* Null Checks */
	if (ppbuffer == MNULL) {
		LEAVE();
		return 0;
	}
	if (*ppbuffer == MNULL) {
		LEAVE();
		return 0;
	}

	memset(pmriv->adapter, &tsf_tlv, 0x00,
	       sizeof(MrvlIEtypes_TsfTimestamp_t));

	tsf_tlv.header.type = wlan_cpu_to_le16(TLV_TYPE_TSFTIMESTAMP);
	tsf_tlv.header.len = wlan_cpu_to_le16(2 * sizeof(tsf_val));

	memcpy_ext(pmriv->adapter, *ppbuffer, &tsf_tlv, sizeof(tsf_tlv.header),
		   sizeof(tsf_tlv.header));
	*ppbuffer += sizeof(tsf_tlv.header);

	/* TSF timestamp from the firmware TSF when the bcn/prb rsp was received
	 */
	tsf_val = wlan_cpu_to_le64(pbss_desc->network_tsf);
	memcpy_ext(pmriv->adapter, *ppbuffer, &tsf_val, sizeof(tsf_val),
		   sizeof(tsf_val));
	*ppbuffer += sizeof(tsf_val);

	memcpy_ext(pmriv->adapter, &tsf_val, pbss_desc->time_stamp,
		   sizeof(pbss_desc->time_stamp), sizeof(tsf_val));

	PRINTM(MINFO, "ASSOC: TSF offset calc: %016llx - %016llx\n", tsf_val,
	       pbss_desc->network_tsf);

	memcpy_ext(pmriv->adapter, *ppbuffer, &tsf_val, sizeof(tsf_val),
		   sizeof(tsf_val));
	*ppbuffer += sizeof(tsf_val);

	LEAVE();
	return sizeof(tsf_tlv.header) + (2 * sizeof(tsf_val));
}

/**
 *  @brief This function finds out the common rates between rate1 and rate2.
 *
 *  It will fill common rates in rate1 as output if found.
 *
 *  NOTE: Setting the MSB of the basic rates needs to be taken
 *   care of, either before or after calling this function
 *
 *  @param pmpriv      A pointer to mlan_private structure
 *  @param rate1       the buffer which keeps input and output
 *  @param rate1_size  the size of rate1 buffer
 *  @param rate2       the buffer which keeps rate2
 *  @param rate2_size  the size of rate2 buffer.
 *
 *  @return            MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status wlan_get_common_rates(mlan_private *pmpriv, t_u8 *rate1,
					 t_u32 rate1_size, t_u8 *rate2,
					 t_u32 rate2_size)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_callbacks *pcb = (mlan_callbacks *)&pmpriv->adapter->callbacks;
	t_u8 *ptr = rate1;
	t_u8 *tmp = MNULL;
	t_u32 i, j;

	ENTER();

	if (rate1_size > WLAN_SUPPORTED_RATES) {
		PRINTM(MERROR, "Invalid rate buffer size\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	ret = pcb->moal_malloc(pmpriv->adapter->pmoal_handle, rate1_size,
			       MLAN_MEM_DEF | MLAN_MEM_FLAG_ATOMIC, &tmp);
	if (ret != MLAN_STATUS_SUCCESS || !tmp) {
		PRINTM(MERROR, "Failed to allocate buffer\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	memcpy_ext(pmpriv->adapter, tmp, rate1, rate1_size, rate1_size);
	memset(pmpriv->adapter, rate1, 0, rate1_size);

	for (i = 0; rate2[i] && i < rate2_size; i++) {
		/* tmp is already malloc'd above */
		// coverity[cert_exp33_c_violation:SUPPRESS]
		for (j = 0; tmp[j] && j < rate1_size; j++) {
			/* Check common rate, excluding the bit
			 * for basic rate */
			if ((rate2[i] & 0x7F) == (tmp[j] & 0x7F)) {
				/* i & j are restricted by rate2_size and
				 * rate1_size resp */
				// coverity[integer_overflow:SUPPRESS]
				*rate1++ = tmp[j];
				break;
			}
		}
	}

	HEXDUMP("rate1 (AP) Rates", tmp, rate1_size);
	HEXDUMP("rate2 (Card) Rates", rate2, rate2_size);
	HEXDUMP("Common Rates", ptr, rate1 - ptr);
	PRINTM(MINFO, "Tx DataRate is set to 0x%X\n", pmpriv->data_rate);

	if (!pmpriv->is_data_rate_auto) {
		// coverity[integer_overflow:SUPPRESS]
		while (rate1_size && *ptr) {
			/* loop exits when rate1_size becomes 0 */
			// coverity[integer_overflow:SUPPRESS]
			if ((*ptr & 0x7f) == pmpriv->data_rate) {
				ret = MLAN_STATUS_SUCCESS;
				goto done;
			}
			ptr++;
			rate1_size--;
		}
		PRINTM(MMSG,
		       "Previously set fixed data rate %#x is not "
		       "compatible with the network\n",
		       pmpriv->data_rate);

		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	ret = MLAN_STATUS_SUCCESS;
done:
	if (tmp)
		pcb->moal_mfree(pmpriv->adapter->pmoal_handle, tmp);

	LEAVE();
	return ret;
}

/**
 *  @brief Create the intersection of the rates supported by a target BSS and
 *         our pmadapter settings for use in an assoc/join command.
 *
 *  @param pmpriv           A pointer to mlan_private structure
 *  @param pbss_desc        BSS Descriptor whose rates are used in the setup
 *  @param pout_rates       Output: Octet array of rates common between the BSS
 *                          and the pmadapter supported rates settings
 *  @param pout_rates_size  Output: Number of rates/octets set in pout_rates
 *
 *  @return                 MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status wlan_setup_rates_from_bssdesc(mlan_private *pmpriv,
						 BSSDescriptor_t *pbss_desc,
						 t_u8 *pout_rates,
						 t_u32 *pout_rates_size)
{
	t_u8 card_rates[WLAN_SUPPORTED_RATES] = {0};
	t_u32 card_rates_size = 0;
	ENTER();
	/* Copy AP supported rates */
	memcpy_ext(pmpriv->adapter, pout_rates, pbss_desc->supported_rates,
		   WLAN_SUPPORTED_RATES, WLAN_SUPPORTED_RATES);

	if ((pmpriv->adapter->region_code == COUNTRY_CODE_JP_40 ||
	     pmpriv->adapter->region_code == COUNTRY_CODE_JP_FF) &&
	    (pbss_desc->phy_param_set.ds_param_set.current_chan == 14)) {
		/* Special Case: For Japan, 11G rates on CH14 are not allowed*/
		card_rates_size = wlan_get_supported_rates(
			pmpriv, pmpriv->bss_mode, BAND_B, card_rates);
	} else {
		/* Get the STA supported rates */
		card_rates_size =
			wlan_get_supported_rates(pmpriv, pmpriv->bss_mode,
						 pmpriv->config_bands,
						 card_rates);
	}
	/* Get the common rates between AP and STA supported rates */
	if (wlan_get_common_rates(pmpriv, pout_rates, WLAN_SUPPORTED_RATES,
				  card_rates, card_rates_size)) {
		*pout_rates_size = 0;
		PRINTM(MERROR, "wlan_get_common_rates failed\n");
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}

	*pout_rates_size =
		MIN(wlan_strlen((char *)pout_rates), WLAN_SUPPORTED_RATES);

	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief Update the scan entry TSF timestamps to reflect a new association
 *
 *  @param pmpriv        A pointer to mlan_private structure
 *  @param pnew_bss_desc A pointer to the newly associated AP's scan table entry
 *
 *  @return              N/A
 */
static t_void wlan_update_tsf_timestamps(mlan_private *pmpriv,
					 BSSDescriptor_t *pnew_bss_desc)
{
	mlan_adapter *pmadapter = pmpriv->adapter;
	t_u32 table_idx;
	t_u64 new_tsf_base;
	t_s64 tsf_delta;

	ENTER();

	memcpy_ext(pmpriv->adapter, &new_tsf_base, pnew_bss_desc->time_stamp,
		   sizeof(pnew_bss_desc->time_stamp), sizeof(new_tsf_base));

	tsf_delta = new_tsf_base - pnew_bss_desc->network_tsf;

	PRINTM(MINFO, "TSF: Update TSF timestamps, 0x%016llx -> 0x%016llx\n",
	       pnew_bss_desc->network_tsf, new_tsf_base);

	for (table_idx = 0; table_idx < pmadapter->num_in_scan_table;
	     table_idx++) {
		pmadapter->pscan_table[table_idx].network_tsf += tsf_delta;
	}

	LEAVE();
}

/**
 *  @brief Append a wapi IE
 *
 *  This function is called from the network join command prep. routine.
 *    If the IE buffer has been setup by the application, this routine appends
 *    the buffer as a wapi TLV type to the request.
 *
 *  @param priv     A pointer to mlan_private structure
 *  @param ppbuffer pointer to command buffer pointer
 *
 *  @return         bytes added to the buffer
 */
static int wlan_cmd_append_wapi_ie(mlan_private *priv, t_u8 **ppbuffer)
{
	int retlen = 0;
	MrvlIEtypesHeader_t ie_header;

	ENTER();

	/* Null Checks */
	if (ppbuffer == MNULL) {
		LEAVE();
		return 0;
	}
	if (*ppbuffer == MNULL) {
		LEAVE();
		return 0;
	}

	/*
	 * If there is a wapi ie buffer setup, append it to the return
	 *   parameter buffer pointer.
	 */
	if (priv->wapi_ie_len) {
		PRINTM(MCMND, "append wapi ie %d to %p\n", priv->wapi_ie_len,
		       *ppbuffer);

		/* Wrap the generic IE buffer with a pass through TLV type */
		ie_header.type = wlan_cpu_to_le16(TLV_TYPE_WAPI_IE);
		ie_header.len = wlan_cpu_to_le16(priv->wapi_ie_len);
		memcpy_ext(priv->adapter, *ppbuffer, &ie_header,
			   sizeof(ie_header), sizeof(ie_header));

		/* Increment the return size and the return buffer pointer param
		 */
		*ppbuffer += sizeof(ie_header);
		retlen += sizeof(ie_header);

		/* Copy the wapi IE buffer to the output buffer, advance pointer
		 */
		memcpy_ext(priv->adapter, *ppbuffer, priv->wapi_ie,
			   priv->wapi_ie_len, priv->wapi_ie_len);

		/* Increment the return size and the return buffer pointer param
		 */
		*ppbuffer += priv->wapi_ie_len;
		retlen += priv->wapi_ie_len;
	}
	/* return the length appended to the buffer */
	LEAVE();
	return retlen;
}

/**
 *  @brief Append a osen IE
 *
 *  This function is called from the network join command prep. routine.
 *    If the IE buffer has been setup by the application, this routine appends
 *    the buffer as a osen TLV type to the request.
 *
 *  @param priv     A pointer to mlan_private structure
 *  @param ppbuffer pointer to command buffer pointer
 *
 *  @return         bytes added to the buffer
 */
static int wlan_cmd_append_osen_ie(mlan_private *priv, t_u8 **ppbuffer)
{
	int retlen = 0;
	MrvlIEtypesHeader_t ie_header;

	ENTER();

	/* Null Checks */
	if (ppbuffer == MNULL) {
		LEAVE();
		return 0;
	}
	if (*ppbuffer == MNULL) {
		LEAVE();
		return 0;
	}

	/*
	 * If there is a osen ie buffer setup, append it to the return
	 *   parameter buffer pointer.
	 */
	if (priv->osen_ie_len) {
		PRINTM(MCMND, "append osen ie %d to %p\n", priv->osen_ie_len,
		       *ppbuffer);

		/* Wrap the generic IE buffer with a pass through TLV type */
		ie_header.type = wlan_cpu_to_le16(TLV_TYPE_VENDOR_SPECIFIC_IE);
		ie_header.len = wlan_cpu_to_le16(priv->osen_ie[1]);
		memcpy_ext(priv->adapter, *ppbuffer, &ie_header,
			   sizeof(ie_header), sizeof(ie_header));

		/* Increment the return size and the return buffer pointer param
		 */
		*ppbuffer += sizeof(ie_header);
		retlen += sizeof(ie_header);

		/* Copy the osen IE buffer to the output buffer, advance pointer
		 */
		memcpy_ext(priv->adapter, *ppbuffer, &priv->osen_ie[2],
			   priv->osen_ie[1], priv->osen_ie[1]);

		/* Increment the return size and the return buffer pointer param
		 */
		*ppbuffer += priv->osen_ie[1];
		retlen += priv->osen_ie[1];
	}
	/* return the length appended to the buffer */
	LEAVE();
	return retlen;
}

/**
 *  @brief This function get the rsn_cap from RSN ie buffer.
 *
 *  @param data         A pointer to rsn_ie data after IE header
 *  @param len          Length of ie rsn_ie data after IE header
 *  @param return       rsn_cap
 */
static t_u16 wlan_get_rsn_cap(t_u8 *data, t_u8 len)
{
	t_u16 rsn_cap = 0;
	t_u16 *ptr;
	t_u16 *end_ptr;
	t_u16 pairwise_cipher_count = 0;
	t_u16 akm_suite_count = 0;

	if (len < 20) {
		/* Version(2B)+GRP(4B)+PairwiseCnt(2B)+PairwiseList(4B)+
			akmCnt(2B)+akmList(4B)+rsnCap(2B) = 20B */
		PRINTM(MERROR,
		       "RSNE: IE len should not less than 20 Bytes, len=%d\n",
		       len);
		goto done;
	}
	/* rsn_cap = data + 2 bytes version + 4 bytes
	 * group_cipher_suite + 2 bytes pairwise_cipher_count +
	 * pairwise_cipher_count * PAIRWISE_CIPHER_SUITE_LEN + 2 bytes
	 * akm_suite_count + akm_suite_count * AKM_SUITE_LEN
	 */
	end_ptr = (t_u16 *)(data + len);
	ptr = (t_u16 *)(data + sizeof(t_u16) + 4 * sizeof(t_u8));
	pairwise_cipher_count = wlan_le16_to_cpu(*ptr);
	ptr = (t_u16 *)(data + sizeof(t_u16) + 4 * sizeof(t_u8) +
			sizeof(t_u16) +
			pairwise_cipher_count * PAIRWISE_CIPHER_SUITE_LEN);
	if ((pairwise_cipher_count == 0) || (ptr >= end_ptr)) {
		PRINTM(MERROR, "RSNE: PAIRWISE_CIPHER not correct\n");
		goto done;
	}
	akm_suite_count = wlan_le16_to_cpu(*ptr);
	ptr = (t_u16 *)(data + sizeof(t_u16) + 4 * sizeof(t_u8) +
			sizeof(t_u16) +
			pairwise_cipher_count * PAIRWISE_CIPHER_SUITE_LEN +
			sizeof(t_u16) + akm_suite_count * AKM_SUITE_LEN);
	if ((akm_suite_count == 0) || (ptr > end_ptr)) {
		PRINTM(MERROR, "RSNE: AKM Suite or RSNCAP not correct\n");
		goto done;
	}
	rsn_cap = wlan_le16_to_cpu(*ptr);

done:
	PRINTM(MCMND, "rsn_cap=0x%x\n", rsn_cap);
	return rsn_cap;
}

/**
 *  @brief This function check if we should enable 11w
 *
 *  @param pmpriv       	A pointer to mlan_private structure
 *
 *  @param BSSDescriptor_t      A pointer to BSSDescriptor_t data structure
 *  @param return       	MTRUE/MFALSE
 */
static t_u8 wlan_use_mfp(mlan_private *pmpriv, BSSDescriptor_t *pbss_desc)
{
	t_u16 ap_rsn_cap = 0;
	t_u16 sta_rsn_cap = 0;
	t_u8 ap_mfpc, ap_mfpr;
	t_u8 sta_mfpc, sta_mfpr;

	if (pmpriv->wpa_ie[0] != RSN_IE)
		return 0;
	sta_rsn_cap =
		wlan_get_rsn_cap(pmpriv->wpa_ie + 2, *(pmpriv->wpa_ie + 1));
	if (!pbss_desc->prsn_ie)
		return 0;
	ap_rsn_cap = wlan_get_rsn_cap(pbss_desc->prsn_ie->data,
				      pbss_desc->prsn_ie->ieee_hdr.len);
	ap_mfpc = ((ap_rsn_cap & (0x1 << MFPC_BIT)) == (0x1 << MFPC_BIT));
	ap_mfpr = ((ap_rsn_cap & (0x1 << MFPR_BIT)) == (0x1 << MFPR_BIT));
	sta_mfpc = ((sta_rsn_cap & (0x1 << MFPC_BIT)) == (0x1 << MFPC_BIT));
	sta_mfpr = ((sta_rsn_cap & (0x1 << MFPR_BIT)) == (0x1 << MFPR_BIT));
	if (!ap_mfpc && !ap_mfpr)
		return MFALSE;
	if (!sta_mfpc && !sta_mfpr)
		return MFALSE;
	return MTRUE;
}

/********************************************************
				Global Functions
********************************************************/

/**
 *  @brief This function updates RSN IE in the association request.
 *
 *  @param pmpriv       A pointer to mlan_private structure
 *
 *  @param ptlv_rsn_ie       A pointer to rsn_ie TLV
 */
static int wlan_update_rsn_ie(mlan_private *pmpriv,
			      MrvlIEtypes_RsnParamSet_t *ptlv_rsn_ie,
			      t_u16 *rsn_ie_len, t_u8 *akm_type)
{
	t_u16 *prsn_cap;
	t_u8 *ptr;
	t_u8 *pairwise_cipher_count_ptr;
	t_u8 *group_mgmt_cipher_suite_ptr = MNULL;
	t_u8 *pmkid_list_ptr = MNULL;
	t_u8 *end_ptr;
	t_u16 pmf_mask = 0x00;
	t_u16 pairwise_cipher_count = 0;
	t_u16 akm_suite_count = 0;
	t_u16 pmkid_count = 0;
	t_u8 i;

#define PREFERENCE_TKIP 1
	/* Cipher Perference Order:
	   (5) CIPHER_SYITE_TYPE_GCMP_256 = 9
	   (4) CIPHER_SYITE_TYPE_GCMP_128 = 8
	   (3) CIPHER_SYITE_TYPE_CCMP_256 = 10
	   (2) CIPHER_SYITE_TYPE_CCMP_128 = 4
	   (1) CIPHER_SYITE_TYPE_TKIP     = 2
	   (0) Skip
	*/
	t_u8 preference_selected;
	t_u8 cipher_selected_id;
#if 0 // defined(ENABLE_GCMP_SUPPORT)
      //  embedded supplicant doesn't support GCMP yet
	t_u8 cipher_preference[11] = {0, 0, 1, 0, 2, 0, 0, 0, 4, 5, 3};
#else
	t_u8 cipher_preference[5] = {0, 0, 1, 0, 2};
#endif
	t_u8 oui[4] = {0x00, 0x0f, 0xac, 0x00};

	/* AKM Perference Order:
	   (6) AKM_SUITE_TYPE_FT_SAE     = 9   //Not supported in esupp
	   (5) AKM_SUITE_TYPE_SAE        = 8
	   (4) AKM_SUITE_TYPE_OWE        = 18
	   (3) AKM_SUITE_TYPE_FT_PSK     = 4   //Not supported in esupp
	   (2) AKM_SUITE_TYPE_PSK_SHA256 = 6
	   (1) AKM_SUITE_TYPE_PSK        = 2
	   (0) Skip
	*/
	t_u8 akm_type_selected;
	t_u8 akm_type_id = 0;
	t_u8 akm_preference[19] = {0, 0, 1, 0, 0, 0, 2, 0, 5, 0,
				   0, 0, 0, 0, 0, 0, 0, 0, 4};
	mlan_adapter *pmadapter = pmpriv->adapter;

	int ap_mfpc = 0, ap_mfpr = 0, ret = MLAN_STATUS_SUCCESS;

	if (*rsn_ie_len < 20) {
		/* Version(2B)+GRP(4B)+PairwiseCnt(2B)+PairwiseList(4B)+
			akmCnt(2B)+akmList(4B)+rsnCap(2B) = 20B */
		PRINTM(MERROR,
		       "RSNE: IE len should not less than 20 Bytes, len=%d\n",
		       *rsn_ie_len);
		return MLAN_STATUS_FAILURE;
	}
	pmf_mask = (((pmpriv->pmfcfg.mfpc << MFPC_BIT) |
		     (pmpriv->pmfcfg.mfpr << MFPR_BIT)) |
		    (~PMF_MASK));
	/* prsn_cap = prsn_ie->rsn_ie + 2 bytes version + 4 bytes
	 * group_cipher_suite + 2 bytes pairwise_cipher_count +
	 * pairwise_cipher_count * PAIRWISE_CIPHER_SUITE_LEN + 2 bytes
	 * akm_suite_count + akm_suite_count * AKM_SUITE_LEN
	 */
	end_ptr = ptlv_rsn_ie->rsn_ie + *rsn_ie_len;

	ptr = ptlv_rsn_ie->rsn_ie + sizeof(t_u16) + 4 * sizeof(t_u8);

	pairwise_cipher_count_ptr = ptr;
	pairwise_cipher_count = wlan_le16_to_cpu(*(t_u16 *)ptr);
	ptr += sizeof(t_u16);

	if ((pairwise_cipher_count == 0) ||
	    (ptr + PAIRWISE_CIPHER_SUITE_LEN * pairwise_cipher_count) >=
		    end_ptr) {
		PRINTM(MERROR, "RSNE: PAIRWISE_CIPHER not correct\n");
		return MLAN_STATUS_FAILURE;
	}

	preference_selected = 0;
	cipher_selected_id = 0;
	for (i = 0; i < pairwise_cipher_count; i++) {
		if ((ptr[3] < sizeof(cipher_preference)) &&
		    (cipher_preference[ptr[3]] > preference_selected)) {
			preference_selected = cipher_preference[ptr[3]];
			cipher_selected_id = ptr[3];
		}
		ptr += PAIRWISE_CIPHER_SUITE_LEN;
	}

	if (preference_selected == 0) {
		PRINTM(MERROR, "RSNE: PAIRWISE_CIPHER not supported\n");
		return MLAN_STATUS_FAILURE;
	}
	if ((preference_selected == PREFERENCE_TKIP) &&
	    ((*akm_type == AssocAgentAuth_Wpa3Sae) ||
	     (*akm_type == AssocAgentAuth_Owe))) {
		PRINTM(MERROR,
		       "RSNE: PAIRWISE_CIPHER TKIP not allowed for AKM %s\n",
		       (*akm_type == AssocAgentAuth_Wpa3Sae) ? "SAE" : "ÖWE");
		return MLAN_STATUS_FAILURE;
	}
	if ((preference_selected == PREFERENCE_TKIP) &&
	    (*akm_type == AssocAgentAuth_Auto)) {
		*akm_type = AssocAgentAuth_Open;
	}
	/* Process AKM
	 * Preference order for AssocAgentAuth_Auto:
	 *  FT Authentication using SAE 00-0F-AC:9  (not supported in embedded
	 * supplicant) SAE Authentication 00-0F-AC:8 OWE Authentication
	 * 00-0F-AC:18 FT Authentication using PSK 00-0F-AC:4  (not supported in
	 * embedded supplicant) PSK using SHA-256 00-0F-AC:6 PSK 00-0F-AC:2
	 */
	ptr = ptlv_rsn_ie->rsn_ie + sizeof(t_u16) + 4 * sizeof(t_u8) +
	      sizeof(t_u16) + pairwise_cipher_count * PAIRWISE_CIPHER_SUITE_LEN;
	akm_suite_count = wlan_le16_to_cpu(*(t_u16 *)ptr);
	ptr += sizeof(t_u16); // move pointer to AKM suite

	if ((akm_suite_count == 0) || (ptr + AKM_SUITE_LEN * akm_suite_count +
				       sizeof(t_u16)) > end_ptr) { // sizeof(t_u16)
								   // is for
								   // rsncap
		PRINTM(MERROR, "RSNE: AKM Suite or RSNCAP not correct\n");
		return MLAN_STATUS_FAILURE;
	}

	akm_type_selected = 0;
	if (*akm_type == AssocAgentAuth_Auto) {
		// find the best one
		for (i = 0; i < akm_suite_count; i++) {
			if ((ptr[3] < sizeof(akm_preference)) &&
			    (akm_preference[ptr[3]] > akm_type_selected)) {
				akm_type_selected = akm_preference[ptr[3]];
				akm_type_id = ptr[3];
			}
			ptr += AKM_SUITE_LEN;
		}
		if (akm_type_selected) {
			if (akm_type_id == 6)
				*akm_type = AssocAgentAuth_Open;
			else if (akm_type_id == 2)
				*akm_type = AssocAgentAuth_Open;
			else if (akm_type_id == 18)
				*akm_type = AssocAgentAuth_Owe;
			else if (akm_type_id == 8)
				*akm_type = AssocAgentAuth_Wpa3Sae;
		}
	} else {
		// find the matched AKM
		for (i = 0; i < akm_suite_count; i++) {
			if (ptr[3] < sizeof(akm_preference)) {
				if ((*akm_type == AssocAgentAuth_Open) &&
				    (ptr[3] == 6)) {
					break;
				} else if ((*akm_type == AssocAgentAuth_Open) &&
					   (ptr[3] == 2)) {
					break;
				} else if ((*akm_type ==
					    AssocAgentAuth_Wpa3Sae) &&
					   (ptr[3] == 8)) {
					break;
				} else if ((*akm_type == AssocAgentAuth_Owe) &&
					   (ptr[3] == 18)) {
					break;
				}
			}
			ptr += AKM_SUITE_LEN;
		}
		if (i == akm_suite_count) {
			akm_type_selected = 0; // not found
		} else {
			akm_type_selected = akm_preference[ptr[3]];
			akm_type_id = ptr[3];
		}
	}

	if (akm_type_selected == 0) {
		PRINTM(MERROR, "RSNE: AKM Suite not found for authtype %d\n",
		       *akm_type);
		return MLAN_STATUS_FAILURE;
	}
	/* Process RSNCAP */
	ptr = ptlv_rsn_ie->rsn_ie + sizeof(t_u16) + 4 * sizeof(t_u8) +
	      sizeof(t_u16) +
	      pairwise_cipher_count * PAIRWISE_CIPHER_SUITE_LEN +
	      sizeof(t_u16) + akm_suite_count * AKM_SUITE_LEN;
	prsn_cap = (t_u16 *)ptr;

	ap_mfpc = ((*prsn_cap & (0x1 << MFPC_BIT)) == (0x1 << MFPC_BIT));
	ap_mfpr = ((*prsn_cap & (0x1 << MFPR_BIT)) == (0x1 << MFPR_BIT));

	/* Check for negative case
	 * If WPA3SAE AP has PMF=0, block the association */
	if ((*akm_type == AssocAgentAuth_Wpa3Sae) && (!ap_mfpc && !ap_mfpr)) {
		PRINTM(MERROR,
		       "RSNE: WPA3-SAE AP with incorrect PMF setting, can't associate to AP\n");
		return MLAN_STATUS_FAILURE;
	}

	if ((!ap_mfpc && !ap_mfpr && pmpriv->pmfcfg.mfpr) ||
	    ((!ap_mfpc) && ap_mfpr) ||
	    (ap_mfpc && ap_mfpr && (!pmpriv->pmfcfg.mfpc))) {
		PRINTM(MERROR,
		       "RSNE: Mismatch in PMF config of STA and AP, can't associate to AP\n");
		return MLAN_STATUS_FAILURE;
	}
	*prsn_cap |= PMF_MASK;
	*prsn_cap &= pmf_mask;

	// PMKID
	ptr += sizeof(t_u16);
	if (end_ptr >= (ptr + sizeof(t_u16))) {
		pmkid_count = wlan_le16_to_cpu(*(t_u16 *)ptr);
		ptr += sizeof(t_u16);

		if (pmkid_count &&
		    (end_ptr >= (ptr + pmkid_count * PMKID_LEN))) {
			pmkid_list_ptr = ptr;
			ptr += pmkid_count * PMKID_LEN;
		}
	}
	// Group Mgmt Cipher Suite
	if ((end_ptr >= (ptr + GROUP_MGMT_CIPHER_SUITE_LEN)) &&
	    (pmf_mask & PMF_MASK)) {
		group_mgmt_cipher_suite_ptr = ptr;
	}
	/* Compose new RSNE */
	// pairwiase
	ptr = pairwise_cipher_count_ptr;
	*(t_u16 *)ptr = wlan_cpu_to_le16(1);
	ptr += sizeof(t_u16);
	oui[3] = cipher_selected_id;
	*(t_u32 *)ptr = *(t_u32 *)oui;
	ptr += PAIRWISE_CIPHER_SUITE_LEN;
	// akm
	*(t_u16 *)ptr = wlan_cpu_to_le16(1);
	ptr += sizeof(t_u16);
	oui[3] = akm_type_id;
	*(t_u32 *)ptr = *(t_u32 *)oui;
	ptr += AKM_SUITE_LEN;
	// RSNCAP
	*(t_u16 *)ptr = wlan_cpu_to_le16(*prsn_cap);
	ptr += sizeof(t_u16);
	// PMKID list
	if (pmkid_list_ptr || group_mgmt_cipher_suite_ptr) {
		// Add PMKID
		*(t_u16 *)ptr = wlan_cpu_to_le16(pmkid_count);
		ptr += sizeof(t_u16);
		if (pmkid_count) {
			memcpy_ext(pmadapter, ptr, (t_u8 *)pmkid_list_ptr,
				   (pmkid_count * PMKID_LEN), (end_ptr - ptr));
			ptr += pmkid_count * PMKID_LEN;
		}
		if (group_mgmt_cipher_suite_ptr) {
			// Add Group Mgmt Cipher Suite
			memcpy_ext(pmadapter, ptr,
				   (t_u8 *)group_mgmt_cipher_suite_ptr,
				   GROUP_MGMT_CIPHER_SUITE_LEN,
				   (end_ptr - ptr));
			ptr += GROUP_MGMT_CIPHER_SUITE_LEN;
		}
	}
	*rsn_ie_len = ptr - ptlv_rsn_ie->rsn_ie;
	return ret;
}

/**
 *  @brief This function is to find FT AKM in RSN.
 *
 *  @param pmpriv       A pointer to mlan_private structure
 *
 *  @param rsn_ie       A pointer to rsn_ie
 *
 */
t_u8 wlan_ft_akm_is_used(mlan_private *pmpriv, t_u8 *rsn_ie)
{
	t_u8 *temp;
	t_u16 count;
	t_u16 pairwise_cipher_count = 0;
	t_u16 akm_suite_count = 0;
	t_u8 found = 0;
	t_u8 rsn_ft_1x_oui[4] = {0x00, 0x0f, 0xac, 0x03};
	t_u8 rsn_ft_psk_oui[4] = {0x00, 0x0f, 0xac, 0x04};
	t_u8 rsn_ft_sae_oui[4] = {0x00, 0x0f, 0xac, 0x09};
	mlan_adapter *pmadapter = pmpriv->adapter;

	ENTER();

	if (!rsn_ie)
		goto done;

	if (rsn_ie[0] != RSN_IE)
		goto done;

	/*  2 bytes header + 2 bytes version + 4 bytes group_cipher_suite +
	 *  2 bytes pairwise_cipher_count + pairwise_cipher_count *
	 * PAIRWISE_CIPHER_SUITE_LEN (4) + 2 bytes akm_suite_count +
	 * akm_suite_count * AKM_SUITE_LEN (4)
	 */
	count = *(t_u16 *)(rsn_ie + 2 + 2 + 4 * sizeof(t_u8));
	pairwise_cipher_count = wlan_le16_to_cpu(count);
	count = *(t_u16 *)(rsn_ie + 2 + 2 + 4 * sizeof(t_u8) + sizeof(t_u16) +
			   pairwise_cipher_count * 4);
	akm_suite_count = wlan_le16_to_cpu(count);
	temp = (t_u8 *)(rsn_ie + 2 + sizeof(t_u16) + 4 * sizeof(t_u8) +
			sizeof(t_u16) + pairwise_cipher_count * 4 +
			sizeof(t_u16));

	while (akm_suite_count) {
		if (!memcmp(pmadapter, temp, rsn_ft_1x_oui,
			    sizeof(rsn_ft_1x_oui)) ||
		    !memcmp(pmadapter, temp, rsn_ft_psk_oui,
			    sizeof(rsn_ft_psk_oui)) ||
		    !memcmp(pmadapter, temp, rsn_ft_sae_oui,
			    sizeof(rsn_ft_sae_oui))) {
			found = 1;
			break;
		}
		temp += 4;
		akm_suite_count--;
	}

done:
	LEAVE();
	return found;
}

/**
 *  @brief This function is to find specific IE.
 *
 *  @param ie           A pointer to ie buffer
 *  @param ie_len    Length of ie buffer
 *  @param ie_type  Type of ie that wants to be found in ie buffer
 *
 *  @return     MFALSE if not found; MTURE if found
 */
static t_u8 wlan_find_ie(t_u8 *ie, t_u8 ie_len, t_u8 ie_type)
{
	IEEEtypes_Header_t *pheader = MNULL;
	t_u8 *pos = MNULL;
	t_s8 ret_len;
	t_u8 ret = MFALSE;
	t_u8 len = 0;

	ENTER();

	pos = (t_u8 *)ie;
	ret_len = ie_len;
	while (ret_len >= 2) {
		pheader = (IEEEtypes_Header_t *)pos;
		len = pheader->len + sizeof(IEEEtypes_Header_t);
		if ((t_s8)len > ret_len) {
			PRINTM(MMSG, "invalid IE length = %d left len %d\n",
			       pheader->len, ret_len);
			break;
		}
		if (pheader->element_id == ie_type) {
			ret = MTRUE;
			break;
		}
		ret_len -= pheader->len + sizeof(IEEEtypes_Header_t);
		pos += pheader->len + sizeof(IEEEtypes_Header_t);
	}

	LEAVE();
	return ret;
}

/**
 *  @brief This function prepares command of association.
 *
 *  @param pmpriv       A pointer to mlan_private structure
 *  @param cmd          A pointer to HostCmd_DS_COMMAND structure
 *  @param pdata_buf    A pointer cast of BSSDescriptor_t from the
 *                        scan table to assoc
 *
 *  @return             MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status wlan_cmd_802_11_associate(mlan_private *pmpriv,
				      HostCmd_DS_COMMAND *cmd,
				      t_void *pdata_buf)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_adapter *pmadapter = pmpriv->adapter;
	HostCmd_DS_802_11_ASSOCIATE *passo = &cmd->params.associate;
	BSSDescriptor_t *pbss_desc;
	MrvlIEtypes_SsIdParamSet_t *pssid_tlv;
	MrvlIEtypes_PhyParamSet_t *pphy_tlv;
	MrvlIEtypes_SsParamSet_t *pss_tlv;
	MrvlIEtypes_RatesParamSet_t *prates_tlv;
	MrvlIEtypes_AuthType_t *pauth_tlv = MNULL;
	MrvlIEtypes_RsnParamSet_t *prsn_ie_tlv = MNULL;
	MrvlIEtypes_SAE_PWE_Mode_t *prsnx_ie_tlv = MNULL;
	MrvlIEtypes_SecurityCfg_t *psecurity_cfg_ie = MNULL;
	MrvlIEtypes_ChanListParamSet_t *pchan_tlv;
	WLAN_802_11_RATES rates;
	t_u32 rates_size;
	t_u16 tmp_cap;
	t_u8 *pos;
	IEEEtypes_CapInfo_t *pcap_info;
	t_u8 ft_akm = 0;
	t_u8 oper_class;
	t_u8 oper_class_flag = MFALSE;
	t_u8 akm_type = 0;
	MrvlIEtypes_HostMlme_t *host_mlme_tlv = MNULL;
	MrvlIEtypes_PrevBssid_t *prev_bssid_tlv = MNULL;
	t_u8 zero_mac[MLAN_MAC_ADDR_LENGTH] = {0};
	MrvlIEtypes_MultiAp_t *multi_ap_tlv = MNULL;
	MrvlIETypes_VHTCap_t *pvhtcap = MNULL;
	MrvlIEtypes_He_cap_t *phecap = MNULL;
	MrvlIEtypesHeader_t *papInfo_tlv = MNULL;
	MrvlIEtypesHeader_t *pvendorOUI_tlv;
	t_u8 ie_len = 0;
	t_s16 size = 0;

	ENTER();

	pbss_desc = (BSSDescriptor_t *)pdata_buf;
	pos = (t_u8 *)&cmd->params;

	cmd->command = wlan_cpu_to_le16(HostCmd_CMD_802_11_ASSOCIATE);

	/* Save so we know which BSS Desc to use in the response handler */
	pmpriv->pattempted_bss_desc = pbss_desc;
	memcpy_ext(pmpriv->adapter, &pmpriv->curr_bss_params.attemp_bssid,
		   pbss_desc->mac_address, MLAN_MAC_ADDR_LENGTH,
		   MLAN_MAC_ADDR_LENGTH);
	pmpriv->delay_link_lost = MFALSE;
	/* back up previous AP's assoc_resp and assoc_req buffer*/
	if (pmpriv->media_connected) {
		memcpy_ext(pmpriv->adapter, &pmpriv->prev_bssid,
			   pmpriv->curr_bss_params.bss_descriptor.mac_address,
			   MLAN_MAC_ADDR_LENGTH, MLAN_MAC_ADDR_LENGTH);
		memcpy_ext(pmadapter, pmpriv->prior_assoc_rsp,
			   pmpriv->assoc_rsp_buf, pmpriv->assoc_rsp_size,
			   ASSOC_RSP_BUF_SIZE);
		memcpy_ext(pmadapter, pmpriv->prior_assoc_req,
			   pmpriv->assoc_req_buf, pmpriv->assoc_req_size,
			   ASSOC_RSP_BUF_SIZE);
		pmpriv->prior_assoc_rsp_size = pmpriv->assoc_rsp_size;
		pmpriv->prior_assoc_req_size = pmpriv->assoc_req_size;
	} else {
		pmpriv->prior_assoc_rsp_size = 0;
		pmpriv->prior_assoc_req_size = 0;
	}
	/* clear assoc_rsp_size */
	pmpriv->assoc_rsp_size = 0;
	pmpriv->assoc_req_size = 0;

	memcpy_ext(pmadapter, passo->peer_sta_addr, pbss_desc->mac_address,
		   sizeof(pbss_desc->mac_address),
		   sizeof(passo->peer_sta_addr));
	pos += sizeof(passo->peer_sta_addr);

	/* Set the listen interval */
	passo->listen_interval = wlan_cpu_to_le16(pmpriv->listen_interval);
	/* Set the beacon period */
	passo->beacon_period = wlan_cpu_to_le16(pbss_desc->beacon_period);

	pos += sizeof(passo->cap_info);
	pos += sizeof(passo->listen_interval);
	pos += sizeof(passo->beacon_period);
	pos += sizeof(passo->dtim_period);

	pssid_tlv = (MrvlIEtypes_SsIdParamSet_t *)pos;
	pssid_tlv->header.type = wlan_cpu_to_le16(TLV_TYPE_SSID);
	pssid_tlv->header.len = (t_u16)pbss_desc->ssid.ssid_len;
	memcpy_ext(pmadapter, pssid_tlv->ssid, pbss_desc->ssid.ssid,
		   pssid_tlv->header.len, pssid_tlv->header.len);
	pos += sizeof(pssid_tlv->header) + pssid_tlv->header.len;
	pssid_tlv->header.len = wlan_cpu_to_le16(pssid_tlv->header.len);

	pphy_tlv = (MrvlIEtypes_PhyParamSet_t *)pos;
	pphy_tlv->header.type = wlan_cpu_to_le16(TLV_TYPE_PHY_DS);
	pphy_tlv->header.len = sizeof(pphy_tlv->fh_ds.ds_param_set);
	memcpy_ext(pmadapter, &pphy_tlv->fh_ds.ds_param_set,
		   &pbss_desc->phy_param_set.ds_param_set.current_chan,
		   sizeof(pphy_tlv->fh_ds.ds_param_set),
		   sizeof(pphy_tlv->fh_ds.ds_param_set));
	pos += sizeof(pphy_tlv->header) + pphy_tlv->header.len;
	pphy_tlv->header.len = wlan_cpu_to_le16(pphy_tlv->header.len);

	pss_tlv = (MrvlIEtypes_SsParamSet_t *)pos;
	pss_tlv->header.type = wlan_cpu_to_le16(TLV_TYPE_CF);
	pss_tlv->header.len = sizeof(pss_tlv->cf_ibss.cf_param_set);
	pos += sizeof(pss_tlv->header) + pss_tlv->header.len;
	pss_tlv->header.len = wlan_cpu_to_le16(pss_tlv->header.len);

	/* Get the common rates supported between the driver and the BSS Desc */
	if (wlan_setup_rates_from_bssdesc(pmpriv, pbss_desc, rates,
					  &rates_size)) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	/* Save the data rates into Current BSS state structure */
	pmpriv->curr_bss_params.num_of_rates = rates_size;
	memcpy_ext(pmadapter, &pmpriv->curr_bss_params.data_rates, rates,
		   rates_size, WLAN_SUPPORTED_RATES);

	/* Setup the Rates TLV in the association command */
	prates_tlv = (MrvlIEtypes_RatesParamSet_t *)pos;
	prates_tlv->header.type = wlan_cpu_to_le16(TLV_TYPE_RATES);
	prates_tlv->header.len = wlan_cpu_to_le16((t_u16)rates_size);
	memcpy_ext(pmadapter, prates_tlv->rates, rates, rates_size, rates_size);
	pos += sizeof(prates_tlv->header) + rates_size;
	PRINTM(MINFO, "ASSOC_CMD: Rates size = %d\n", rates_size);

	/* Add the Authentication type to be used for Auth frames if needed */
	if ((pmpriv->sec_info.authentication_mode != MLAN_AUTH_MODE_AUTO) ||
	    (pbss_desc->owe_transition_mode == OWE_TRANS_MODE_OWE)) {
		pauth_tlv = (MrvlIEtypes_AuthType_t *)pos;
		pauth_tlv->header.type = wlan_cpu_to_le16(TLV_TYPE_AUTH_TYPE);
		pauth_tlv->header.len = sizeof(pauth_tlv->auth_type);
		if ((pmpriv->sec_info.wep_status == Wlan802_11WEPEnabled) ||
		    (pmpriv->sec_info.authentication_mode ==
		     MLAN_AUTH_MODE_NETWORKEAP))
			pauth_tlv->auth_type = wlan_cpu_to_le16(
				(t_u16)pmpriv->sec_info.authentication_mode);
		else if (pmpriv->sec_info.authentication_mode ==
			 MLAN_AUTH_MODE_FT)
			pauth_tlv->auth_type =
				wlan_cpu_to_le16(AssocAgentAuth_FastBss_Skip);
		else if (pmpriv->sec_info.authentication_mode ==
			 MLAN_AUTH_MODE_FILS)
			pauth_tlv->auth_type =
				wlan_cpu_to_le16(AssocAgentAuth_FILS);
		else if (pmpriv->sec_info.authentication_mode ==
			 MLAN_AUTH_MODE_SAE)
			pauth_tlv->auth_type =
				wlan_cpu_to_le16(AssocAgentAuth_Wpa3Sae);
		else if (!pmpriv->curr_bss_params.host_mlme &&
			 ((pbss_desc->owe_transition_mode ==
			   OWE_TRANS_MODE_OWE) ||
			  pmpriv->sec_info.authentication_mode ==
				  MLAN_AUTH_MODE_OWE))
			pauth_tlv->auth_type =
				wlan_cpu_to_le16(AssocAgentAuth_Owe);
		else
			pauth_tlv->auth_type =
				wlan_cpu_to_le16(AssocAgentAuth_Open);
		pos += sizeof(pauth_tlv->header) + pauth_tlv->header.len;
		pauth_tlv->header.len = wlan_cpu_to_le16(pauth_tlv->header.len);
	}

	if ((pauth_tlv != MNULL) &&
	    (pauth_tlv->auth_type ==
	     wlan_cpu_to_le16(AssocAgentAuth_Wpa3Sae))) {
		if (pbss_desc->prsnx_ie && pbss_desc->prsnx_ie->ieee_hdr.len &&
		    (pbss_desc->prsnx_ie->data[0] & (0x1 << SAE_H2E_BIT))) {
			MrvlIEtypes_SAE_PWE_Mode_t *psae_pwe_mode_tlv;

			/* Setup the sae pwe derivation mode TLV in the
			 * association command */
			psae_pwe_mode_tlv = (MrvlIEtypes_SAE_PWE_Mode_t *)pos;
			psae_pwe_mode_tlv->header.type = wlan_cpu_to_le16(
				TLV_TYPE_WPA3_SAE_PWE_DERIVATION_MODE);
			psae_pwe_mode_tlv->header.len = wlan_cpu_to_le16(
				sizeof(psae_pwe_mode_tlv->pwe));
			psae_pwe_mode_tlv->pwe[0] =
				pbss_desc->prsnx_ie->data[0];
			pos += sizeof(psae_pwe_mode_tlv->header) +
			       sizeof(psae_pwe_mode_tlv->pwe);
		}
	}

	if (IS_SUPPORT_MULTI_BANDS(pmadapter) &&
	    (pbss_desc->bss_band & pmpriv->config_bands) &&
	    !(ISSUPP_11NENABLED(pmadapter->fw_cap_info) &&
	      (!pbss_desc->disable_11n) &&
	      (pmpriv->config_bands & BAND_GN ||
	       pmpriv->config_bands & BAND_AN) &&
	      (pbss_desc->pht_cap))) {
		/* Append a channel TLV for the channel the attempted AP was
		 * found on */
		pchan_tlv = (MrvlIEtypes_ChanListParamSet_t *)pos;
		pchan_tlv->header.type = wlan_cpu_to_le16(TLV_TYPE_CHANLIST);
		pchan_tlv->header.len =
			wlan_cpu_to_le16(sizeof(ChanScanParamSet_t));

		memset(pmadapter, pchan_tlv->chan_scan_param, 0x00,
		       sizeof(ChanScanParamSet_t));
		pchan_tlv->chan_scan_param[0].chan_number =
			(pbss_desc->phy_param_set.ds_param_set.current_chan);
		PRINTM(MINFO, "Assoc: TLV Chan = %d\n",
		       pchan_tlv->chan_scan_param[0].chan_number);

		pchan_tlv->chan_scan_param[0].bandcfg.chanBand =
			wlan_band_to_radio_type(pbss_desc->bss_band);
		if (pbss_desc->bss_band == BAND_6G)
			pchan_tlv->chan_scan_param[0].bandcfg.chanWidth =
				wlan_get_6g_ap_bandconfig(
					pbss_desc,
					&pchan_tlv->chan_scan_param[0].bandcfg);
		PRINTM(MINFO, "Assoc: TLV Bandcfg = %x\n",
		       pchan_tlv->chan_scan_param[0].bandcfg);
		pos += sizeof(pchan_tlv->header) + sizeof(ChanScanParamSet_t);
	}
	if (!pmpriv->wps.session_enable) {
		if ((pmpriv->sec_info.wpa_enabled ||
		     pmpriv->sec_info.wpa2_enabled)) {
			prsn_ie_tlv = (MrvlIEtypes_RsnParamSet_t *)pos;
			/* WPA_IE or RSN_IE */
			prsn_ie_tlv->header.type = (t_u16)pmpriv->wpa_ie[0];
			prsn_ie_tlv->header.type =
				prsn_ie_tlv->header.type & 0x00FF;
			prsn_ie_tlv->header.type =
				wlan_cpu_to_le16(prsn_ie_tlv->header.type);
			prsn_ie_tlv->header.len = (t_u16)pmpriv->wpa_ie[1];
			prsn_ie_tlv->header.len =
				prsn_ie_tlv->header.len & 0x00FF;
			if (prsn_ie_tlv->header.len <=
			    (sizeof(pmpriv->wpa_ie) - 2))
				memcpy_ext(pmadapter, prsn_ie_tlv->rsn_ie,
					   &pmpriv->wpa_ie[2],
					   prsn_ie_tlv->header.len,
					   prsn_ie_tlv->header.len);
			else {
				ret = MLAN_STATUS_FAILURE;
				goto done;
			}
			HEXDUMP("ASSOC_CMD: RSN IE", (t_u8 *)prsn_ie_tlv,
				sizeof(prsn_ie_tlv->header) +
					prsn_ie_tlv->header.len);
			pos += sizeof(prsn_ie_tlv->header) +
			       prsn_ie_tlv->header.len;
			prsn_ie_tlv->header.len =
				wlan_cpu_to_le16(prsn_ie_tlv->header.len);
			/** parse rsn ie to find whether ft akm is used*/
			ft_akm = wlan_ft_akm_is_used(pmpriv, pmpriv->wpa_ie);
			/* Append PMF Configuration coming from cfg80211 layer
			 */
			psecurity_cfg_ie = (MrvlIEtypes_SecurityCfg_t *)pos;
			psecurity_cfg_ie->header.type =
				wlan_cpu_to_le16(TLV_TYPE_SECURITY_CFG);

			pmpriv->curr_bss_params.use_mfp =
				wlan_use_mfp(pmpriv, pbss_desc);
			PRINTM(MCMND, "use_mfp=%d\n",
			       pmpriv->curr_bss_params.use_mfp);

			if (!pmpriv->curr_bss_params.use_mfp)
				psecurity_cfg_ie->use_mfp = MFALSE;
			else
				psecurity_cfg_ie->use_mfp = MTRUE;
			psecurity_cfg_ie->header.len = sizeof(t_u8);
			pos += sizeof(psecurity_cfg_ie->header) +
			       psecurity_cfg_ie->header.len;
			psecurity_cfg_ie->header.len =
				wlan_cpu_to_le16(psecurity_cfg_ie->header.len);
		} else if (pmpriv->sec_info.ewpa_enabled ||
			   (pbss_desc->owe_transition_mode ==
			    OWE_TRANS_MODE_OWE) ||
			   (pmpriv->sec_info.authentication_mode ==
			    MLAN_AUTH_MODE_OWE)) {
			prsn_ie_tlv = (MrvlIEtypes_RsnParamSet_t *)pos;
			if (pbss_desc->pwpa_ie) {
				prsn_ie_tlv->header.type =
					(t_u16)(*(pbss_desc->pwpa_ie))
						.vend_hdr.element_id;
				prsn_ie_tlv->header.type =
					prsn_ie_tlv->header.type & 0x00FF;
				prsn_ie_tlv->header.type = wlan_cpu_to_le16(
					prsn_ie_tlv->header.type);
				prsn_ie_tlv->header.len =
					(t_u16)(*(pbss_desc->pwpa_ie))
						.vend_hdr.len;
				prsn_ie_tlv->header.len =
					prsn_ie_tlv->header.len & 0x00FF;
				if (prsn_ie_tlv->header.len <=
				    (sizeof(pmpriv->wpa_ie))) {
					memcpy_ext(pmadapter,
						   prsn_ie_tlv->rsn_ie,
						   &((*(pbss_desc->pwpa_ie))
							     .vend_hdr.oui[0]),
						   prsn_ie_tlv->header.len,
						   prsn_ie_tlv->header.len);
				} else {
					ret = MLAN_STATUS_FAILURE;
					goto done;
				}

				HEXDUMP("ASSOC_CMD: RSN IE",
					(t_u8 *)prsn_ie_tlv,
					sizeof(prsn_ie_tlv->header) +
						prsn_ie_tlv->header.len);
				pos += sizeof(prsn_ie_tlv->header) +
				       prsn_ie_tlv->header.len;
				prsn_ie_tlv->header.len = wlan_cpu_to_le16(
					prsn_ie_tlv->header.len);
			}
			if (pbss_desc->prsn_ie) {
				prsn_ie_tlv = (MrvlIEtypes_RsnParamSet_t *)pos;
				prsn_ie_tlv->header.type =
					(t_u16)(*(pbss_desc->prsn_ie))
						.ieee_hdr.element_id;
				prsn_ie_tlv->header.type =
					prsn_ie_tlv->header.type & 0x00FF;
				prsn_ie_tlv->header.type = wlan_cpu_to_le16(
					prsn_ie_tlv->header.type);
				prsn_ie_tlv->header.len =
					(t_u16)(*(pbss_desc->prsn_ie))
						.ieee_hdr.len;
				prsn_ie_tlv->header.len =
					prsn_ie_tlv->header.len & 0x00FF;
				if (prsn_ie_tlv->header.len <=
				    (sizeof(pmpriv->wpa_ie))) {
					memcpy_ext(pmadapter,
						   prsn_ie_tlv->rsn_ie,
						   &((*(pbss_desc->prsn_ie))
							     .data[0]),
						   prsn_ie_tlv->header.len,
						   prsn_ie_tlv->header.len);
					akm_type =
						pauth_tlv ?
							wlan_le16_to_cpu(
								pauth_tlv
									->auth_type) :
							AssocAgentAuth_Auto;
					ret = wlan_update_rsn_ie(
						pmpriv, prsn_ie_tlv,
						&prsn_ie_tlv->header.len,
						&akm_type);
					if (ret != MLAN_STATUS_SUCCESS) {
						goto done;
					}
				} else {
					ret = MLAN_STATUS_FAILURE;
					goto done;
				}

				HEXDUMP("ASSOC_CMD: RSN IE",
					(t_u8 *)prsn_ie_tlv,
					sizeof(prsn_ie_tlv->header) +
						prsn_ie_tlv->header.len);
				pos += sizeof(prsn_ie_tlv->header) +
				       prsn_ie_tlv->header.len;
				prsn_ie_tlv->header.len = wlan_cpu_to_le16(
					prsn_ie_tlv->header.len);

				if ((pauth_tlv == MNULL) &&
				    (pmpriv->sec_info.authentication_mode ==
				     MLAN_AUTH_MODE_AUTO)) {
					pauth_tlv =
						(MrvlIEtypes_AuthType_t *)pos;
					pauth_tlv->header.type =
						wlan_cpu_to_le16(
							TLV_TYPE_AUTH_TYPE);
					pauth_tlv->header.len =
						sizeof(pauth_tlv->auth_type);
					pauth_tlv->auth_type =
						wlan_cpu_to_le16(akm_type);

					pos += sizeof(pauth_tlv->header) +
					       pauth_tlv->header.len;
					pauth_tlv->header.len =
						wlan_cpu_to_le16(
							pauth_tlv->header.len);
				}
			}
			if ((pbss_desc->prsnx_ie) &&
			    (akm_type == AssocAgentAuth_Wpa3Sae)) {
				prsnx_ie_tlv =
					(MrvlIEtypes_SAE_PWE_Mode_t *)pos;
				prsnx_ie_tlv->header.type =
					(t_u16)(*(pbss_desc->prsnx_ie))
						.ieee_hdr.element_id;
				prsnx_ie_tlv->header.type =
					prsnx_ie_tlv->header.type & 0x00FF;
				prsnx_ie_tlv->header.type = wlan_cpu_to_le16(
					prsnx_ie_tlv->header.type);
				prsnx_ie_tlv->header.len =
					(t_u16)(*(pbss_desc->prsnx_ie))
						.ieee_hdr.len;
				prsnx_ie_tlv->header.len =
					prsnx_ie_tlv->header.len & 0x00FF;

				memcpy_ext(pmadapter, prsnx_ie_tlv->pwe,
					   &((*(pbss_desc->prsnx_ie)).data[0]),
					   prsnx_ie_tlv->header.len,
					   prsnx_ie_tlv->header.len);

				HEXDUMP("ASSOC_CMD: RSNX IE",
					(t_u8 *)prsnx_ie_tlv,
					sizeof(prsnx_ie_tlv->header) +
						prsnx_ie_tlv->header.len);

				pos += sizeof(prsnx_ie_tlv->header) +
				       prsnx_ie_tlv->header.len;
				prsnx_ie_tlv->header.len = wlan_cpu_to_le16(
					prsnx_ie_tlv->header.len);
			}
		}
	}

	if (ISSUPP_11NENABLED(pmadapter->fw_cap_info) &&
	    (!pbss_desc->disable_11n) &&
	    wlan_11n_bandconfig_allowed(pmpriv, pbss_desc->bss_band))
		wlan_cmd_append_11n_tlv(pmpriv, pbss_desc, &pos);
	else if ((pmpriv->hotspot_cfg & HOTSPOT_ENABLED) &&
		 !(pmpriv->hotspot_cfg & HOTSPOT_BY_SUPPLICANT))
		wlan_add_ext_capa_info_ie(pmpriv, pbss_desc, &pos);
	if (pmpriv->adapter->ecsa_enable) {
		oper_class_flag =
			wlan_find_ie(pmpriv->gen_ie_buf, pmpriv->gen_ie_buf_len,
				     REGULATORY_CLASS);
		if (!oper_class_flag) {
			if (MLAN_STATUS_SUCCESS ==
			    wlan_get_curr_oper_class(
				    pmpriv,
				    pbss_desc->phy_param_set.ds_param_set
					    .current_chan,
				    pbss_desc->curr_bandwidth, &oper_class))
				wlan_add_supported_oper_class_ie(pmpriv, &pos,
								 oper_class);
		}
	}
	if (ISSUPP_11ACENABLED(pmadapter->fw_cap_info) &&
	    (!pbss_desc->disable_11n) &&
	    wlan_11ac_bandconfig_allowed(pmpriv, pbss_desc->bss_band))
		wlan_cmd_append_11ac_tlv(pmpriv, pbss_desc, &pos);

	if ((IS_FW_SUPPORT_11AX(pmadapter)) &&
	    wlan_11ax_bandconfig_allowed(pmpriv, pbss_desc))
		wlan_cmd_append_11ax_tlv(pmpriv, pbss_desc, &pos);

	if ((!pbss_desc->disable_11n) &&
	    (ISSUPP_11NENABLED(pmadapter->fw_cap_info) ||
	     ISSUPP_11ACENABLED(pmadapter->fw_cap_info) ||
	     IS_FW_SUPPORT_11AX(pmadapter))) {
		PRINTM(MCMND, "STBC NOT supported, Will be disabled\n");
	}
	if ((IS_FW_SUPPORT_6G(pmadapter)) &&
	    wlan_116e_bandconfig_allowed(pmpriv, pbss_desc))
		wlan_cmd_append_116e_tlv(pmpriv, pbss_desc, &pos);

	wlan_wmm_process_association_req(pmpriv, &pos, &pbss_desc->wmm_ie);
	if (pmpriv->sec_info.wapi_enabled && pmpriv->wapi_ie_len)
		wlan_cmd_append_wapi_ie(pmpriv, &pos);

	if (pmpriv->sec_info.osen_enabled && pmpriv->osen_ie_len)
		wlan_cmd_append_osen_ie(pmpriv, &pos);

	wlan_cmd_append_generic_ie(pmpriv, &pos);

	if (ft_akm && pbss_desc->pmd_ie)
		wlan_cmd_append_pass_through_ie(
			pmpriv, (IEEEtypes_Generic_t *)pbss_desc->pmd_ie, &pos);
	wlan_cmd_append_tsf_tlv(pmpriv, &pos, pbss_desc);

	if (pmpriv->curr_bss_params.host_mlme) {
		host_mlme_tlv = (MrvlIEtypes_HostMlme_t *)pos;
		host_mlme_tlv->header.type =
			wlan_cpu_to_le16(TLV_TYPE_HOST_MLME);
		host_mlme_tlv->header.len = sizeof(host_mlme_tlv->host_mlme);
		host_mlme_tlv->host_mlme = MTRUE;
		pos += sizeof(host_mlme_tlv->header) +
		       host_mlme_tlv->header.len;
		host_mlme_tlv->header.len =
			wlan_cpu_to_le16(host_mlme_tlv->header.len);
	}
	if (memcmp(pmadapter, &pmpriv->curr_bss_params.prev_bssid, zero_mac,
		   MLAN_MAC_ADDR_LENGTH)) {
		prev_bssid_tlv = (MrvlIEtypes_PrevBssid_t *)pos;
		prev_bssid_tlv->header.type =
			wlan_cpu_to_le16(TLV_TYPE_PREV_BSSID);
		prev_bssid_tlv->header.len =
			wlan_cpu_to_le16(MLAN_MAC_ADDR_LENGTH);
		memcpy_ext(pmadapter, prev_bssid_tlv->prev_bssid,
			   &pmpriv->curr_bss_params.prev_bssid,
			   MLAN_MAC_ADDR_LENGTH, MLAN_MAC_ADDR_LENGTH);
		PRINTM(MCMND, "ASSOCIATE: PREV_BSSID = " MACSTR "\n",
		       MAC2STR(pmpriv->curr_bss_params.prev_bssid));
		pos += sizeof(prev_bssid_tlv->header) + MLAN_MAC_ADDR_LENGTH;
	}

	if (pmpriv->multi_ap_flag) {
		multi_ap_tlv = (MrvlIEtypes_MultiAp_t *)pos;
		multi_ap_tlv->header.type = wlan_cpu_to_le16(TLV_TYPE_MULTI_AP);
		multi_ap_tlv->header.len = sizeof(multi_ap_tlv->flag);
		multi_ap_tlv->flag = pmpriv->multi_ap_flag;
		PRINTM(MINFO, " TLV multi_ap_flag : 0x%x\n",
		       multi_ap_tlv->flag);
		pos += sizeof(multi_ap_tlv->header) + multi_ap_tlv->header.len;
		multi_ap_tlv->header.len =
			wlan_cpu_to_le16(sizeof(multi_ap_tlv->flag));
	}

	if (wlan_11d_create_dnld_countryinfo(pmpriv, pbss_desc->bss_band)) {
		PRINTM(MERROR, "Dnld_countryinfo_11d failed\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	if (wlan_11d_parse_dnld_countryinfo(pmpriv,
					    pmpriv->pattempted_bss_desc)) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	/* pass AP's required capabilities */
	if (pbss_desc->vendor_oui_count) {
		papInfo_tlv = (MrvlIEtypesHeader_t *)pos;
		papInfo_tlv->type = wlan_cpu_to_le16(TLV_TYPE_AP_INFO);
		papInfo_tlv->len = 0;
		pos += sizeof(MrvlIEtypesHeader_t);

		/* append AP vht cap tlv */
		if (pbss_desc->pvht_cap) {
			ie_len = sizeof(MrvlIEtypesHeader_t) +
				 sizeof(VHT_capa_t);
			papInfo_tlv->len += ie_len;
			pvhtcap = (MrvlIETypes_VHTCap_t *)pos;
			pvhtcap->header.type = wlan_cpu_to_le16(VHT_CAPABILITY);
			pvhtcap->header.len = sizeof(VHT_capa_t);
			memcpy_ext(pmadapter,
				   (t_u8 *)pvhtcap +
					   sizeof(MrvlIEtypesHeader_t),
				   (t_u8 *)pbss_desc->pvht_cap +
					   sizeof(IEEEtypes_Header_t),
				   sizeof(VHT_capa_t), sizeof(VHT_capa_t));
			pos += ie_len;
			pvhtcap->header.len =
				wlan_cpu_to_le16(pvhtcap->header.len);
		}

		/* append AP he cap tlv */
		if (pbss_desc->phe_cap) {
			ie_len = sizeof(MrvlIEtypesHeader_t) +
				 pbss_desc->phe_cap->ieee_hdr.len;
			papInfo_tlv->len += ie_len;
			phecap = (MrvlIEtypes_He_cap_t *)pos;
			phecap->type = wlan_cpu_to_le16(EXTENSION);
			phecap->len = pbss_desc->phe_cap->ieee_hdr.len;
			memcpy_ext(pmadapter,
				   (t_u8 *)phecap + sizeof(MrvlIEtypesHeader_t),
				   (t_u8 *)pbss_desc->phe_cap +
					   sizeof(IEEEtypes_Header_t),
				   pbss_desc->phe_cap->ieee_hdr.len,
				   pbss_desc->phe_cap->ieee_hdr.len);
			pos += ie_len;
			phecap->len = wlan_cpu_to_le16(phecap->len);
		}

		/* append AP vendor oui list tlv */
		if (pbss_desc->vendor_oui_count) {
			ie_len = sizeof(MrvlIEtypesHeader_t) +
				 pbss_desc->vendor_oui_count * VENDOR_OUI_LEN;
			papInfo_tlv->len += ie_len;
			pvendorOUI_tlv = (MrvlIEtypesHeader_t *)pos;
			pvendorOUI_tlv->type =
				wlan_cpu_to_le16(VENDOR_IE_OUIS_TLV_ID);
			pvendorOUI_tlv->len =
				pbss_desc->vendor_oui_count * VENDOR_OUI_LEN;
			pos += sizeof(MrvlIEtypesHeader_t);
			memcpy_ext(pmadapter, pos,
				   (t_u8 *)&pbss_desc->vendor_oui,
				   pvendorOUI_tlv->len, pvendorOUI_tlv->len);
			pos += pvendorOUI_tlv->len;
			pvendorOUI_tlv->len =
				wlan_cpu_to_le16(pvendorOUI_tlv->len);
		}
		papInfo_tlv->len = wlan_cpu_to_le16(papInfo_tlv->len);
	}

	/*
	 * Call 11h join API after capability bits are set so adhoc/infra 11h
	 * behavior can be properly triggered.  pos modified if data is appended
	 */
	wlan_11h_process_join(
		pmpriv, &pos, &passo->cap_info, (t_u8)pbss_desc->bss_band,
		pbss_desc->phy_param_set.ds_param_set.current_chan,
		&pbss_desc->wlan_11h_bss_info);
	size = pos - (t_u8 *)passo;
	cmd->size = wlan_cpu_to_le16((t_u16)size + S_DS_GEN);

	/* Set the Capability info at last */
	memcpy_ext(pmadapter, &tmp_cap, &pbss_desc->cap_info,
		   sizeof(passo->cap_info), sizeof(tmp_cap));

	/* retain spectrum_mgmt capability */
	pcap_info = &passo->cap_info;
	if (pcap_info->spectrum_mgmt)
		SPECTRUM_MGMT_ENABLED(tmp_cap);

	if (pmpriv->config_bands == BAND_B)
		SHORT_SLOT_TIME_DISABLED(tmp_cap);

	if (pmpriv->adapter->pcard_info->support_11mc)
		RADIO_MEASUREMENT_ENABLED(tmp_cap);

	tmp_cap &= CAPINFO_MASK;
	PRINTM(MINFO, "ASSOC_CMD: tmp_cap=%4X CAPINFO_MASK=%4lX\n", tmp_cap,
	       CAPINFO_MASK);
	tmp_cap = wlan_cpu_to_le16(tmp_cap);
	memcpy_ext(pmadapter, &passo->cap_info, &tmp_cap, sizeof(tmp_cap),
		   sizeof(passo->cap_info));

done:
	LEAVE();
	return ret;
}

/**
 *  @brief Association firmware command response handler
 *
 *   The response buffer for the association command has the following
 *      memory layout.
 *
 *   For cases where an association response was not received (indicated
 *      by the CapInfo and AId field):
 *
 *     .------------------------------------------------------------.
 *     |  Header(4 * sizeof(t_u16)):  Standard command response hdr |
 *     .------------------------------------------------------------.
 *     |  cap_info/Error Return(t_u16):                             |
 *     |           0xFFFF(-1): Internal error for association       |
 *     |           0xFFFE(-2): Authentication unhandled message     |
 *     |           0xFFFD(-3): Authentication refused               |
 *     |           0xFFFC(-4): Timeout waiting for AP response      |
 *     |           0xFFFB(-5): Internal error for authentication    |
 *     .------------------------------------------------------------.
 *     |  status_code(t_u16):                                       |
 *     |        If cap_info is -1:                                  |
 *     |           An internal firmware failure prevented the       |
 *     |           command from being processed. The status code    |
 *     |           is 6 if associate response parameter invlaid,    |
 *     |           1 otherwise.                                     |
 *     |                                                            |
 *     |        If cap_info is -2:                                  |
 *     |           An authentication frame was received but was     |
 *     |           not handled by the firmware. IEEE Status code    |
 *     |           for the failure is returned.                     |
 *     |                                                            |
 *     |        If cap_info is -3:                                  |
 *     |           An authentication frame was received and the     |
 *     |           status_code is the IEEE Status reported in the   |
 *     |           response.                                        |
 *     |                                                            |
 *     |        If cap_info is -4:                                  |
 *     |           (1) Association response timeout                 |
 *     |           (2) Authentication response timeout              |
 *     |                                                            |
 *     |        If cap_info is -5:                                  |
 *     |           An internal firmware failure prevented the       |
 *     |           command from being processed. The status code    |
 *     |           is 6 if authentication parameter invlaid,        |
 *     |           1 otherwise.                                     |
 *     .------------------------------------------------------------.
 *     |  a_id(t_u16): 0xFFFF                                       |
 *     .------------------------------------------------------------.
 *
 *
 *   For cases where an association response was received, the IEEE
 *     standard association response frame is returned:
 *
 *     .------------------------------------------------------------.
 *     |  Header(4 * sizeof(t_u16)):  Standard command response hdr |
 *     .------------------------------------------------------------.
 *     |  cap_info(t_u16): IEEE Capability                          |
 *     .------------------------------------------------------------.
 *     |  status_code(t_u16): IEEE Status Code                      |
 *     .------------------------------------------------------------.
 *     |  a_id(t_u16): IEEE Association ID                          |
 *     .------------------------------------------------------------.
 *     |  IEEE IEs(variable): Any received IEs comprising the       |
 *     |                      remaining portion of a received       |
 *     |                      association response frame.           |
 *     .------------------------------------------------------------.
 *
 *  For simplistic handling, the status_code field can be used to determine
 *    an association success (0) or failure (non-zero).
 *
 *  @param pmpriv       A pointer to mlan_private structure
 *  @param resp         A pointer to HostCmd_DS_COMMAND
 *  @param pioctl_buf   A pointer to mlan_ioctl_req structure
 *
 *  @return             MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status wlan_ret_802_11_associate(mlan_private *pmpriv,
				      HostCmd_DS_COMMAND *resp,
				      t_void *pioctl_buf)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_ioctl_req *pioctl_req = (mlan_ioctl_req *)pioctl_buf;
	IEEEtypes_AssocRsp_t *passoc_rsp;
	BSSDescriptor_t *pbss_desc;
	t_u8 enable_data = MTRUE;
	t_u8 event_buf[100];
	mlan_event *pevent = (mlan_event *)event_buf;
	t_u8 cur_mac[MLAN_MAC_ADDR_LENGTH];
	t_u8 media_connected = pmpriv->media_connected;
	mlan_adapter *pmadapter = pmpriv->adapter;
	assoc_logger_data *assoc_succ;
	mlan_ds_bss *bss;
	IEEEtypes_MgmtHdr_t *hdr;
	t_u16 sub_type = 0;
	t_u16 assoc_rsp_size = 0;

	ENTER();

	hdr = (IEEEtypes_MgmtHdr_t *)&resp->params;
	sub_type = IEEE80211_GET_FC_MGMT_FRAME_SUBTYPE(hdr->FrmCtl);
	assoc_rsp_size = resp->size - S_DS_GEN;
	if ((assoc_rsp_size >=
	     (sizeof(IEEEtypes_MgmtHdr_t) + sizeof(IEEEtypes_AssocRsp_t))) &&
	    !memcmp(pmpriv->adapter, hdr->BssId,
		    pmpriv->curr_bss_params.attemp_bssid,
		    MLAN_MAC_ADDR_LENGTH) &&
	    ((sub_type == SUBTYPE_ASSOC_RESP) ||
	     (sub_type == SUBTYPE_REASSOC_RESP))) {
		passoc_rsp =
			(IEEEtypes_AssocRsp_t *)((t_u8 *)(&resp->params) +
						 sizeof(IEEEtypes_MgmtHdr_t));
		pmpriv->curr_bss_params.host_mlme = MTRUE;
	} else
		passoc_rsp = (IEEEtypes_AssocRsp_t *)&resp->params;
	passoc_rsp->status_code = wlan_le16_to_cpu(passoc_rsp->status_code);
	pmpriv->delay_link_lost = MFALSE;
	if (pmpriv->media_connected == MTRUE)
		memcpy_ext(pmpriv->adapter, cur_mac,
			   pmpriv->curr_bss_params.bss_descriptor.mac_address,
			   MLAN_MAC_ADDR_LENGTH, MLAN_MAC_ADDR_LENGTH);

	HEXDUMP("ASSOC_RESP:", (t_u8 *)&resp->params, (resp->size - S_DS_GEN));

	pmpriv->assoc_rsp_size =
		MIN(resp->size - S_DS_GEN, sizeof(pmpriv->assoc_rsp_buf));

	memcpy_ext(pmpriv->adapter, pmpriv->assoc_rsp_buf,
		   &resp->params.assoc_rsp_buf, pmpriv->assoc_rsp_size,
		   pmpriv->assoc_rsp_size);

	if (pioctl_req != MNULL) {
		bss = (mlan_ds_bss *)pioctl_req->pbuf;
		bss->param.ssid_bssid.assoc_rsp.assoc_resp_len =
			pmpriv->assoc_rsp_size;
		memcpy_ext(pmpriv->adapter,
			   bss->param.ssid_bssid.assoc_rsp.assoc_resp_buf,
			   pmpriv->assoc_rsp_buf, pmpriv->assoc_rsp_size,
			   ASSOC_RSP_BUF_SIZE);
	}
	if (passoc_rsp->status_code) {
		if (pmpriv->curr_bss_params.host_mlme &&
		    (pmpriv->assoc_rsp_size < (sizeof(IEEEtypes_MgmtHdr_t) +
					       sizeof(IEEEtypes_AssocRsp_t))))
			wlan_recv_event(pmpriv, MLAN_EVENT_ID_DRV_ASSOC_FAILURE,
					MNULL);
		if (pmpriv->media_connected == MTRUE) {
			if (pmpriv->port_ctrl_mode == MTRUE)
				pmpriv->port_open = pmpriv->prior_port_status;
			if (!memcmp(pmpriv->adapter, cur_mac,
				    pmpriv->pattempted_bss_desc->mac_address,
				    MLAN_MAC_ADDR_LENGTH) ||
			    !pmpriv->prior_assoc_rsp_size) {
				wlan_reset_connect_state(pmpriv, MTRUE);
			} else {
				// fallback to previous AP
				memcpy_ext(
					pmpriv->adapter,
					&pmpriv->curr_bss_params.attemp_bssid,
					pmpriv->curr_bss_params.bss_descriptor
						.mac_address,
					MLAN_MAC_ADDR_LENGTH,
					MLAN_MAC_ADDR_LENGTH);
				wlan_recv_event(
					pmpriv,
					MLAN_EVENT_ID_DRV_ASSOC_FAILURE_REPORT,
					MNULL);
				// update Assoc resp/req buf, if Assoc Rsp
				// failure.
				pmpriv->assoc_rsp_size =
					pmpriv->prior_assoc_rsp_size;
				memcpy_ext(pmadapter, pmpriv->assoc_rsp_buf,
					   pmpriv->prior_assoc_rsp,
					   pmpriv->assoc_rsp_size,
					   ASSOC_RSP_BUF_SIZE);
				pmpriv->assoc_req_size =
					pmpriv->prior_assoc_req_size;
				memcpy_ext(pmadapter, pmpriv->assoc_req_buf,
					   pmpriv->prior_assoc_req,
					   pmpriv->assoc_req_size,
					   ASSOC_RSP_BUF_SIZE);

				// coverity[no_effect:SUPPRESS]
				memset(pmpriv->adapter,
				       pmpriv->curr_bss_params.prev_bssid, 0,
				       MLAN_MAC_ADDR_LENGTH);
			}
		} else
			wlan_reset_connect_state(pmpriv, MTRUE);
		pmpriv->adapter->dbg.num_cmd_assoc_failure++;
		pmpriv->adapter->dbg.num_cons_assoc_failure++;
		PRINTM(MERROR,
		       "ASSOC_RESP: Association Failed, "
		       "status code = %d, error = 0x%x, a_id = 0x%x\n",
		       passoc_rsp->status_code,
		       wlan_le16_to_cpu(*(t_u16 *)&passoc_rsp->capability),
		       wlan_le16_to_cpu(passoc_rsp->a_id));
		memset(pmadapter, event_buf, 0, sizeof(event_buf));
		pevent->bss_index = pmpriv->bss_index;
		pevent->event_id = MLAN_EVENT_ID_DRV_ASSOC_FAILURE_LOGGER;
		pevent->event_len = sizeof(passoc_rsp->status_code);
		memcpy_ext(pmpriv->adapter, (t_u8 *)pevent->event_buf,
			   &passoc_rsp->status_code, pevent->event_len,
			   pevent->event_len);
		wlan_recv_event(pmpriv, MLAN_EVENT_ID_DRV_ASSOC_FAILURE_LOGGER,
				pevent);

		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	/* Send a Media Connected event, according to the Spec */
	pmpriv->media_connected = MTRUE;
	pmpriv->multi_ap_flag = 0;
	pmpriv->adapter->pps_uapsd_mode = MFALSE;
	pmpriv->adapter->tx_lock_flag = MFALSE;
	pmpriv->adapter->delay_null_pkt = MFALSE;

	/* Set the attempted BSSID Index to current */
	pbss_desc = pmpriv->pattempted_bss_desc;

	PRINTM(MCMND, "ASSOC_RESP: %-32s (a_id = 0x%x)\n", pbss_desc->ssid.ssid,
	       wlan_le16_to_cpu(passoc_rsp->a_id));
	/* Restore default extended capabilities */
	memcpy_ext(pmpriv->adapter, &pmpriv->ext_cap, &pmpriv->def_ext_cap,
		   sizeof(pmpriv->ext_cap), sizeof(pmpriv->ext_cap));
	/* Make a copy of current BSSID descriptor */
	memcpy_ext(pmpriv->adapter, &pmpriv->curr_bss_params.bss_descriptor,
		   pbss_desc, sizeof(BSSDescriptor_t), sizeof(BSSDescriptor_t));

	/* Update curr_bss_params */
	pmpriv->curr_bss_params.bss_descriptor.channel =
		pbss_desc->phy_param_set.ds_param_set.current_chan;

	pmpriv->curr_bss_params.band = pbss_desc->bss_band;

	/* Store current channel for further reference.
	 * This would save one extra call to get current
	 * channel when disconnect/bw_ch event is raised.
	 */
	pmpriv->adapter->dfsr_channel =
		pmpriv->curr_bss_params.bss_descriptor.channel;

	/*`
	 * Adjust the timestamps in the scan table to be relative to the newly
	 * associated AP's TSF
	 */
	wlan_update_tsf_timestamps(pmpriv, pbss_desc);

	if (pbss_desc->wmm_ie.vend_hdr.element_id == WMM_IE)
		pmpriv->curr_bss_params.wmm_enabled = MTRUE;
	else
		pmpriv->curr_bss_params.wmm_enabled = MFALSE;

	if (pmpriv->wmm_required && pmpriv->curr_bss_params.wmm_enabled)
		pmpriv->wmm_enabled = MTRUE;
	else
		pmpriv->wmm_enabled = MFALSE;

	pmpriv->curr_bss_params.wmm_uapsd_enabled = MFALSE;

	if (pmpriv->wmm_enabled == MTRUE)
		pmpriv->curr_bss_params.wmm_uapsd_enabled =
			pbss_desc->wmm_ie.qos_info.qos_uapsd;

	PRINTM(MINFO, "ASSOC_RESP: curr_pkt_filter is 0x%x\n",
	       pmpriv->curr_pkt_filter);
	if (pmpriv->sec_info.wpa_enabled || pmpriv->sec_info.wpa2_enabled)
		pmpriv->wpa_is_gtk_set = MFALSE;
	if (pmpriv->wmm_enabled)
		/* Don't re-enable carrier until we get the WMM_GET_STATUS event
		 */
		enable_data = MFALSE;
	else
		/* Since WMM is not enabled, setup the queues with the defaults
		 */
		wlan_wmm_setup_queues(pmpriv);

	if (enable_data)
		PRINTM(MINFO, "Post association, re-enabling data flow\n");

	/* Reset SNR/NF/RSSI values */
	pmpriv->data_rssi_last = 0;
	pmpriv->data_nf_last = 0;
	pmpriv->data_rssi_avg = 0;
	pmpriv->data_nf_avg = 0;
	pmpriv->bcn_rssi_last = 0;
	pmpriv->bcn_nf_last = 0;
	pmpriv->bcn_rssi_avg = 0;
	pmpriv->bcn_nf_avg = 0;
	pmpriv->rxpd_rate = 0;
	pmpriv->rxpd_rate_info = 0;
	/* Reset mib statistics*/
	pmpriv->amsdu_rx_cnt = 0;
	pmpriv->amsdu_tx_cnt = 0;
	pmpriv->msdu_in_rx_amsdu_cnt = 0;
	pmpriv->msdu_in_tx_amsdu_cnt = 0;
	if (pbss_desc->pvht_cap && pbss_desc->pht_cap) {
		if (GET_VHTCAP_MAXMPDULEN(
			    pbss_desc->pvht_cap->vht_cap.vht_cap_info) == 2)
			pmpriv->max_amsdu = MLAN_TX_DATA_BUF_SIZE_12K;
		else if (GET_VHTCAP_MAXMPDULEN(
				 pbss_desc->pvht_cap->vht_cap.vht_cap_info) ==
			 1)
			pmpriv->max_amsdu = MLAN_TX_DATA_BUF_SIZE_8K;
		else
			pmpriv->max_amsdu = MLAN_TX_DATA_BUF_SIZE_4K;
	} else if (pbss_desc->pht_cap) {
		if (GETHT_MAXAMSDU(pbss_desc->pht_cap->ht_cap.ht_cap_info))
			pmpriv->max_amsdu = MLAN_TX_DATA_BUF_SIZE_8K;
		else
			pmpriv->max_amsdu = MLAN_TX_DATA_BUF_SIZE_4K;
	} else if (pbss_desc->phe_6g_cap) {
		if (GET_6G_BAND_CAP_MAXMPDULEN(pbss_desc->phe_6g_cap->capa) ==
		    2)
			pmpriv->max_amsdu = MLAN_TX_DATA_BUF_SIZE_12K;
		else if (GET_6G_BAND_CAP_MAXMPDULEN(
				 pbss_desc->phe_6g_cap->capa) == 1)
			pmpriv->max_amsdu = MLAN_TX_DATA_BUF_SIZE_8K;
		else
			pmpriv->max_amsdu = MLAN_TX_DATA_BUF_SIZE_4K;
	}

	wlan_save_curr_bcn(pmpriv);

	pmpriv->adapter->dbg.num_cmd_assoc_success++;
	pmpriv->adapter->dbg.num_cons_assoc_failure = 0;
	PRINTM(MINFO, "ASSOC_RESP: Associated\n");
	pevent->bss_index = pmpriv->bss_index;
	pevent->event_id = MLAN_EVENT_ID_DRV_CONNECTED;
	pevent->event_len = MLAN_MAC_ADDR_LENGTH;
	memcpy_ext(pmpriv->adapter, (t_u8 *)pevent->event_buf,
		   (t_u8 *)pmpriv->curr_bss_params.bss_descriptor.mac_address,
		   MLAN_MAC_ADDR_LENGTH, pevent->event_len);

	/* Add the ra_list here for infra mode as there will be only 1 ra always
	 */
	if (media_connected) {
		/** replace ralist's mac address with new mac address */
		if (0 ==
		    wlan_ralist_update(
			    pmpriv, cur_mac,
			    pmpriv->curr_bss_params.bss_descriptor.mac_address))
			wlan_ralist_add(pmpriv,
					pmpriv->curr_bss_params.bss_descriptor
						.mac_address);
		wlan_11n_cleanup_reorder_tbl(pmpriv);
		wlan_11n_deleteall_txbastream_tbl(pmpriv);

	} else
		wlan_ralist_add(
			pmpriv,
			pmpriv->curr_bss_params.bss_descriptor.mac_address);

	wlan_recv_event(pmpriv, MLAN_EVENT_ID_DRV_CONNECTED, pevent);
#ifdef UAP_SUPPORT
	if (pmpriv->adapter->dfs_mode)
		wlan_11h_update_dfs_master_state_by_sta(pmpriv);
#endif

	wlan_coex_ampdu_rxwinsize(pmpriv->adapter);

	if (!pmpriv->sec_info.wpa_enabled && !pmpriv->sec_info.wpa2_enabled &&
	    !pmpriv->sec_info.ewpa_enabled && !pmpriv->sec_info.wapi_enabled &&
	    !pmpriv->wps.session_enable && !pmpriv->sec_info.osen_enabled) {
		/* We are in Open/WEP mode, open port immediately */
		if (pmpriv->port_ctrl_mode == MTRUE) {
			pmpriv->port_open = MTRUE;
			PRINTM(MINFO, "ASSOC_RESP: port_status = OPEN\n");
		}
	}
	if (pmpriv->sec_info.wpa_enabled || pmpriv->sec_info.wpa2_enabled ||
	    pmpriv->sec_info.ewpa_enabled || pmpriv->sec_info.wapi_enabled ||
	    pmpriv->wps.session_enable || pmpriv->sec_info.osen_enabled)
		pmpriv->adapter->scan_block = MTRUE;

	pevent = (mlan_event *)event_buf;
	memset(pmadapter, event_buf, 0, sizeof(event_buf));
	pevent->bss_index = pmpriv->bss_index;
	pevent->event_id = MLAN_EVENT_ID_DRV_ASSOC_SUCC_LOGGER;
	pevent->event_len = sizeof(assoc_logger_data);
	assoc_succ = (assoc_logger_data *)pevent->event_buf;
	memcpy_ext(pmpriv->adapter, (t_u8 *)assoc_succ->bssid,
		   pbss_desc->mac_address, MLAN_MAC_ADDR_LENGTH,
		   MLAN_MAC_ADDR_LENGTH);
	memcpy_ext(pmpriv->adapter, (t_u8 *)assoc_succ->oui,
		   pbss_desc->mac_address, MLAN_MAC_ADDR_LENGTH / 2,
		   MLAN_MAC_ADDR_LENGTH / 2);
	assoc_succ->ssid_len = pbss_desc->ssid.ssid_len;
	memcpy_ext(pmpriv->adapter, (t_u8 *)assoc_succ->ssid,
		   pbss_desc->ssid.ssid, pbss_desc->ssid.ssid_len,
		   MLAN_MAX_SSID_LENGTH);
	assoc_succ->rssi = pbss_desc->rssi;
	assoc_succ->channel = pbss_desc->channel;
	wlan_recv_event(pmpriv, MLAN_EVENT_ID_DRV_ASSOC_SUCC_LOGGER, pevent);

done:
	/* Need to indicate IOCTL complete */
	if (pioctl_req != MNULL) {
		if (ret != MLAN_STATUS_SUCCESS) {
			if (passoc_rsp->status_code)
				pioctl_req->status_code =
					(wlan_le16_to_cpu(*(t_u16 *)&passoc_rsp
								   ->capability)
					 << 16) +
					passoc_rsp->status_code;
			else
				pioctl_req->status_code =
					MLAN_ERROR_CMD_ASSOC_FAIL;
		} else {
			pioctl_req->status_code = MLAN_ERROR_NO_ERROR;
		}
	}
	LEAVE();
	return ret;
}

/**
 *  @brief Associated to a specific BSS discovered in a scan
 *
 *  @param pmpriv       A pointer to mlan_private structure
 *  @param pioctl_buf   A pointer to MLAN IOCTL Request buffer
 *  @param pbss_desc     A pointer to the BSS descriptor to associate with.
 *
 *  @return             MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status wlan_associate(mlan_private *pmpriv, t_void *pioctl_buf,
			   BSSDescriptor_t *pbss_desc)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	t_u8 current_bssid[MLAN_MAC_ADDR_LENGTH];
	pmlan_ioctl_req pioctl_req = (mlan_ioctl_req *)pioctl_buf;

	ENTER();

	/* Return error if the pmadapter or table entry
	 *  is not marked as infra */
	if ((pmpriv->bss_mode != MLAN_BSS_MODE_INFRA) ||
	    (pbss_desc->bss_mode != MLAN_BSS_MODE_INFRA)) {
		if (pioctl_req)
			pioctl_req->status_code = MLAN_ERROR_IOCTL_INVALID;
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}

	memcpy_ext(pmpriv->adapter, &current_bssid,
		   &pmpriv->curr_bss_params.bss_descriptor.mac_address,
		   sizeof(current_bssid), sizeof(current_bssid));

	ret = wlan_prepare_cmd(pmpriv, HostCmd_CMD_802_11_ASSOCIATE,
			       HostCmd_ACT_GEN_SET, 0, pioctl_buf, pbss_desc);

	LEAVE();
	return ret;
}

/**
 *  @brief Send Deauthentication Request or Stop the AdHoc network depending on
 * mode
 *
 *  @param pmpriv    A pointer to mlan_private structure
 *  @param pioctl_req A pointer to mlan_ioctl_req structure
 *  @param deauth_param A pointer to mlan_deauth_param structure
 *
 *  @return          MLAN_STATUS_SUCCESS--success, MLAN_STATUS_FAILURE--fail,
 * MLAN_STATUS_PENDING--pending
 */
mlan_status wlan_disconnect(mlan_private *pmpriv, mlan_ioctl_req *pioctl_req,
			    mlan_deauth_param *deauth_param)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_deauth_param local_param = {.mac_addr = {0, 0, 0, 0, 0, 0},
					 .reason_code = DEF_DEAUTH_REASON_CODE};
	t_u8 zero_mac[] = {0, 0, 0, 0, 0, 0};

	ENTER();

	if (deauth_param)
		memcpy_ext(pmpriv->adapter, &local_param, deauth_param,
			   sizeof(*deauth_param), sizeof(local_param));
	if (pmpriv->media_connected == MTRUE) {
		if (pmpriv->bss_mode == MLAN_BSS_MODE_INFRA) {
			if (!deauth_param ||
			    !memcmp(pmpriv->adapter, deauth_param->mac_addr,
				    zero_mac, sizeof(zero_mac)))
				memcpy_ext(pmpriv->adapter,
					   local_param.mac_addr,
					   (t_u8 *)&pmpriv->curr_bss_params
						   .bss_descriptor.mac_address,
					   MLAN_MAC_ADDR_LENGTH,
					   MLAN_MAC_ADDR_LENGTH);
#ifdef WIFI_DIRECT_SUPPORT
			if (pmpriv->bss_type == MLAN_BSS_TYPE_WIFIDIRECT)
				ret = wlan_prepare_cmd(
					pmpriv, HostCmd_CMD_802_11_DISASSOCIATE,
					HostCmd_ACT_GEN_SET, 0,
					(t_void *)pioctl_req, &local_param);
			else
#endif
				ret = wlan_prepare_cmd(
					pmpriv,
					HostCmd_CMD_802_11_DEAUTHENTICATE,
					HostCmd_ACT_GEN_SET, 0,
					(t_void *)pioctl_req, &local_param);

			if (ret == MLAN_STATUS_SUCCESS && pioctl_req)
				ret = MLAN_STATUS_PENDING;
		}
	}

	LEAVE();
	return ret;
}

/**
 *  @brief Convert band to radio type used in channel TLV
 *
 *  @param band     Band enumeration to convert to a channel TLV radio type
 *
 *  @return         Radio type designator for use in a channel TLV
 */
t_u8 wlan_band_to_radio_type(t_u16 band)
{
	t_u8 ret_radio_type;

	ENTER();

	switch (band) {
	case BAND_A:
	case BAND_AN:
	case BAND_A | BAND_AN:
	case BAND_A | BAND_AN | BAND_AAC:
		ret_radio_type = BAND_5GHZ;
		break;
	case BAND_6G:
		ret_radio_type = BAND_6GHZ;
		break;
	case BAND_B:
	case BAND_G:
	case BAND_B | BAND_G:
	default:
		ret_radio_type = BAND_2GHZ;
		break;
	}

	LEAVE();
	return ret_radio_type;
}
