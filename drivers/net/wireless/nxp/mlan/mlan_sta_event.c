/** @file mlan_sta_event.c
 *
 *  @brief This file contains MLAN event handling.
 *
 *
 *  Copyright 2008-2022, 2024-2025 NXP
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

/********************************************************
Change log:
    10/13/2008: initial version
********************************************************/

#include "mlan.h"
#include "mlan_join.h"
#include "mlan_util.h"
#include "mlan_fw.h"
#include "mlan_main.h"
#include "mlan_wmm.h"
#include "mlan_11n.h"
#include "mlan_11h.h"
#ifdef PCIE
#include "mlan_pcie.h"
#endif /* PCIE */

/********************************************************
			Global Variables
********************************************************/

/********************************************************
			Local Functions
********************************************************/

/**
 *  @brief This function handles link lost, deauth and
 *          disassoc events.
 *
 *  @param pmpriv   A pointer to mlan_private structure
 *  @return         N/A
 */
static t_void wlan_handle_disconnect_event(pmlan_private pmpriv)
{
	ENTER();

	if (pmpriv->media_connected == MTRUE)
		wlan_reset_connect_state(pmpriv, MTRUE);

	LEAVE();
}

/**
 *  @brief This function will parse the TDLS event for further wlan action
 *
 *  @param priv     A pointer to mlan_private
 *  @param pevent   A pointer to event buf
 *
 *  @return         N/A
 */
static void wlan_parse_tdls_event(pmlan_private priv, pmlan_buffer pevent)
{
	Event_tdls_generic *tdls_event =
		(Event_tdls_generic *)(pevent->pbuf + pevent->data_offset +
				       sizeof(mlan_event_id));
	sta_node *sta_ptr = MNULL;
	pmlan_adapter pmadapter = priv->adapter;
	t_u8 i = 0;
	IEEEtypes_HTCap_t *pht_cap = MNULL;
	t_u16 ie_len = 0;
	mlan_ds_misc_tdls_oper tdls_oper;
	t_u8 event_buf[100];
	mlan_event *ptdls_event = (mlan_event *)event_buf;
	tdls_tear_down_event *tdls_evt =
		(tdls_tear_down_event *)ptdls_event->event_buf;
	ENTER();

	/* reason code is not mandatory, hence less by sizeof(t_u16) */
	if (pevent->data_len < (sizeof(Event_tdls_generic) - sizeof(t_u16) -
				sizeof(mlan_event_id))) {
		PRINTM(MERROR, "Invalid length %d for TDLS event\n",
		       pevent->data_len);
		LEAVE();
		return;
	}
	sta_ptr = wlan_get_station_entry(priv, tdls_event->peer_mac_addr);
	PRINTM(MEVENT, "TDLS_EVENT: %d " MACSTR "\n",
	       wlan_le16_to_cpu(tdls_event->event_type),
	       MAC2STR(tdls_event->peer_mac_addr));
	switch (wlan_le16_to_cpu(tdls_event->event_type)) {
	case TDLS_EVENT_TYPE_SETUP_REQ:
		if (sta_ptr == MNULL) {
			sta_ptr = wlan_add_station_entry(
				priv, tdls_event->peer_mac_addr);
			if (sta_ptr) {
				sta_ptr->status = TDLS_SETUP_INPROGRESS;
				wlan_hold_tdls_packets(
					priv, tdls_event->peer_mac_addr);
			}
		}
		break;

	case TDLS_EVENT_TYPE_LINK_ESTABLISHED:
		if (sta_ptr) {
			sta_ptr->status = TDLS_SETUP_COMPLETE;
			/* parse the TLV for station's capability */
			ie_len = wlan_le16_to_cpu(
				tdls_event->u.ie_data.ie_length);
			if (ie_len) {
				pht_cap = (IEEEtypes_HTCap_t *)
					wlan_get_specific_ie(
						priv,
						tdls_event->u.ie_data.ie_ptr,
						ie_len, HT_CAPABILITY, 0);
				if (pht_cap) {
					sta_ptr->is_11n_enabled = MTRUE;
					if (GETHT_MAXAMSDU(
						    pht_cap->ht_cap.ht_cap_info))
						sta_ptr->max_amsdu =
							MLAN_TX_DATA_BUF_SIZE_8K;
					else
						sta_ptr->max_amsdu =
							MLAN_TX_DATA_BUF_SIZE_4K;
				}
			}
			for (i = 0; i < MAX_NUM_TID; i++) {
				if (sta_ptr->is_11n_enabled ||
				    sta_ptr->is_11ax_enabled)
					sta_ptr->ampdu_sta[i] =
						priv->aggr_prio_tbl[i]
							.ampdu_user;
				else
					sta_ptr->ampdu_sta[i] =
						BA_STREAM_NOT_ALLOWED;
			}
			memset(priv->adapter, sta_ptr->rx_seq, 0xff,
			       sizeof(sta_ptr->rx_seq));
			wlan_restore_tdls_packets(priv,
						  tdls_event->peer_mac_addr,
						  TDLS_SETUP_COMPLETE);
			pmadapter->tdls_status = TDLS_IN_BASE_CHANNEL;
		}
		break;

	case TDLS_EVENT_TYPE_SETUP_FAILURE:
		wlan_restore_tdls_packets(priv, tdls_event->peer_mac_addr,
					  TDLS_SETUP_FAILURE);
		if (sta_ptr)
			wlan_delete_station_entry(priv,
						  tdls_event->peer_mac_addr);
		if (MTRUE == wlan_is_station_list_empty(priv))
			pmadapter->tdls_status = TDLS_NOT_SETUP;
		else
			pmadapter->tdls_status = TDLS_IN_BASE_CHANNEL;
		break;
	case TDLS_EVENT_TYPE_LINK_TORN_DOWN:
		if (sta_ptr) {
			if (sta_ptr->external_tdls) {
				mlan_status ret = MLAN_STATUS_SUCCESS;
				PRINTM(MMSG,
				       "Receive TDLS TEAR DOWN event, Disable TDLS LINK\n");
				pmadapter->tdls_status = TDLS_TEAR_DOWN;
				memset(pmadapter, &tdls_oper, 0,
				       sizeof(tdls_oper));
				tdls_oper.tdls_action = WLAN_TDLS_DISABLE_LINK;
				memcpy_ext(priv->adapter, tdls_oper.peer_mac,
					   tdls_event->peer_mac_addr,
					   MLAN_MAC_ADDR_LENGTH,
					   MLAN_MAC_ADDR_LENGTH);
				/* Send command to firmware to delete tdls
				 * link*/
				ret = wlan_prepare_cmd(
					priv, HostCmd_CMD_TDLS_OPERATION,
					HostCmd_ACT_GEN_SET, 0, (t_void *)MNULL,
					&tdls_oper);
				if (ret)
					PRINTM(MERROR,
					       "11D: failed to send cmd to FW\n");
				ptdls_event->bss_index = priv->bss_index;
				ptdls_event->event_id =
					MLAN_EVENT_ID_DRV_TDLS_TEARDOWN_REQ;
				ptdls_event->event_len =
					sizeof(tdls_tear_down_event);
				memcpy_ext(priv->adapter,
					   (t_u8 *)tdls_evt->peer_mac_addr,
					   tdls_event->peer_mac_addr,
					   MLAN_MAC_ADDR_LENGTH,
					   MLAN_MAC_ADDR_LENGTH);
				tdls_evt->reason_code = wlan_le16_to_cpu(
					tdls_event->u.reason_code);
				wlan_recv_event(
					priv,
					MLAN_EVENT_ID_DRV_TDLS_TEARDOWN_REQ,
					ptdls_event);
				/* Signal MOAL to trigger mlan_main_process */
				wlan_recv_event(
					priv, MLAN_EVENT_ID_DRV_DEFER_HANDLING,
					MNULL);
				LEAVE();
				return;
			}
			wlan_restore_tdls_packets(priv,
						  tdls_event->peer_mac_addr,
						  TDLS_TEAR_DOWN);
			if (sta_ptr->is_11n_enabled ||
			    sta_ptr->is_11ax_enabled) {
				wlan_cleanup_reorder_tbl(
					priv, tdls_event->peer_mac_addr);
				wlan_11n_cleanup_txbastream_tbl(
					priv, tdls_event->peer_mac_addr);
			}
			wlan_delete_station_entry(priv,
						  tdls_event->peer_mac_addr);
			if (MTRUE == wlan_is_station_list_empty(priv))
				pmadapter->tdls_status = TDLS_NOT_SETUP;
			else
				pmadapter->tdls_status = TDLS_IN_BASE_CHANNEL;
		}
		break;
	case TDLS_EVENT_TYPE_CHAN_SWITCH_RESULT:
		PRINTM(MEVENT,
		       "TDLS_CHAN_SWITCH_RESULT: status=0x%x, reason=0x%x current_channel=%d\n",
		       tdls_event->u.switch_result.status,
		       tdls_event->u.switch_result.reason,
		       (int)tdls_event->u.switch_result.current_channel);
		if (tdls_event->u.switch_result.status == MLAN_STATUS_SUCCESS) {
			if (tdls_event->u.switch_result.current_channel ==
			    TDLS_BASE_CHANNEL) {
				/* enable traffic to AP */
				if (pmadapter->tdls_status !=
				    TDLS_IN_BASE_CHANNEL) {
					wlan_update_non_tdls_ralist(
						priv, tdls_event->peer_mac_addr,
						MFALSE);
					pmadapter->tdls_status =
						TDLS_IN_BASE_CHANNEL;
				}
			} else if (tdls_event->u.switch_result.current_channel ==
				   TDLS_OFF_CHANNEL) {
				/* pause traffic to AP */
				if (pmadapter->tdls_status !=
				    TDLS_IN_OFF_CHANNEL) {
					wlan_update_non_tdls_ralist(
						priv, tdls_event->peer_mac_addr,
						MTRUE);
					pmadapter->tdls_status =
						TDLS_IN_OFF_CHANNEL;
				}
			}
		} else {
			if (tdls_event->u.switch_result.current_channel ==
			    TDLS_BASE_CHANNEL)
				pmadapter->tdls_status = TDLS_IN_BASE_CHANNEL;
			else if (tdls_event->u.switch_result.current_channel ==
				 TDLS_OFF_CHANNEL)
				pmadapter->tdls_status = TDLS_IN_OFF_CHANNEL;
		}
		break;
	case TDLS_EVENT_TYPE_START_CHAN_SWITCH:
		PRINTM(MEVENT, "TDLS start channel switch....\n");
		pmadapter->tdls_status = TDLS_SWITCHING_CHANNEL;
		break;
	case TDLS_EVENT_TYPE_CHAN_SWITCH_STOPPED:
		PRINTM(MEVENT, "TDLS channel switch stopped, reason=%d\n",
		       tdls_event->u.cs_stop_reason);
		break;
	case TDLS_EVENT_TYPE_DEBUG:
	case TDLS_EVENT_TYPE_PACKET:
		break;
	default:
		PRINTM(MERROR, "unknown event type %d\n",
		       wlan_le16_to_cpu(tdls_event->event_type));
		break;
	}
	LEAVE();
}

/**
 *  @brief This function send the tdls teardown request event.
 *
 *  @param priv    A pointer to mlan_private
 *
 *  @return        N/A
 */
static void wlan_send_tdls_tear_down_request(pmlan_private priv)
{
	t_u8 event_buf[100];
	mlan_event *ptdls_event = (mlan_event *)event_buf;
	tdls_tear_down_event *tdls_evt =
		(tdls_tear_down_event *)ptdls_event->event_buf;
	sta_node *sta_ptr = MNULL;

	ENTER();

	sta_ptr = (sta_node *)util_peek_list(
		priv->adapter->pmoal_handle, &priv->sta_list,
		priv->adapter->callbacks.moal_spin_lock,
		priv->adapter->callbacks.moal_spin_unlock);
	if (!sta_ptr) {
		LEAVE();
		return;
	}
	while (sta_ptr != (sta_node *)&priv->sta_list) {
		if (sta_ptr->external_tdls) {
			ptdls_event->bss_index = priv->bss_index;
			ptdls_event->event_id =
				MLAN_EVENT_ID_DRV_TDLS_TEARDOWN_REQ;
			ptdls_event->event_len = sizeof(tdls_tear_down_event);
			memcpy_ext(priv->adapter,
				   (t_u8 *)tdls_evt->peer_mac_addr,
				   sta_ptr->mac_addr, MLAN_MAC_ADDR_LENGTH,
				   MLAN_MAC_ADDR_LENGTH);
			tdls_evt->reason_code =
				MLAN_REASON_TDLS_TEARDOWN_UNSPECIFIED;
			wlan_recv_event(priv,
					MLAN_EVENT_ID_DRV_TDLS_TEARDOWN_REQ,
					ptdls_event);
		}
		sta_ptr = sta_ptr->pnext;
	}
	LEAVE();
	return;
}

/**
 *  @brief This function will handle the generic NAN event for further wlan
 * action based on the Event subtypes
 *
 *  @param pmpriv     A pointer to mlan_private
 *  @param evt_buf    A pointer to mlan_event
 *  @param pmbuf    A pointer to mlan buffer
 *
 *  @return         N/A
 */
static void wlan_process_nan_event(pmlan_private pmpriv, pmlan_buffer pmbuf)
{
	t_u8 *evt_buf = MNULL;
	mlan_event *pevent;
	mlan_status ret = MLAN_STATUS_SUCCESS;
	event_nan_generic *nan_event =
		(event_nan_generic *)(pmbuf->pbuf + pmbuf->data_offset +
				      sizeof(mlan_event_id));
	pmlan_adapter pmadapter = pmpriv->adapter;
	pmlan_callbacks pcb = &pmadapter->callbacks;

	ENTER();

	ret = pcb->moal_malloc(pmadapter->pmoal_handle, MAX_EVENT_SIZE,
			       MLAN_MEM_DEF, &evt_buf);
	if (ret != MLAN_STATUS_SUCCESS || !evt_buf) {
		LEAVE();
		return;
	}

	pevent = (pmlan_event)evt_buf;

	pevent->bss_index = pmpriv->bss_index;
	if (wlan_le16_to_cpu(nan_event->event_sub_type) ==
		    NAN_EVT_SUBTYPE_SD_EVENT ||
	    wlan_le16_to_cpu(nan_event->event_sub_type) ==
		    NAN_EVT_SUBTYPE_SDF_TX_DONE) {
		pevent->event_id = MLAN_EVENT_ID_DRV_PASSTHRU;
		pevent->event_len = pmbuf->data_len;
		memcpy_ext(pmadapter, (t_u8 *)pevent->event_buf,
			   pmbuf->pbuf + pmbuf->data_offset, pevent->event_len,
			   pevent->event_len);
		wlan_recv_event(pmpriv, pevent->event_id, pevent);
		pcb->moal_mfree(pmadapter->pmoal_handle, evt_buf);
	} else {
		t_u8 test_mac[MLAN_MAC_ADDR_LENGTH] = {0x00, 0x11, 0x22,
						       0x33, 0x44, 0x55};
		pevent->event_id = MLAN_EVENT_ID_DRV_CONNECTED;
		pevent->event_len = MLAN_MAC_ADDR_LENGTH;
		memcpy_ext(pmpriv->adapter, (t_u8 *)pevent->event_buf, test_mac,
			   MLAN_MAC_ADDR_LENGTH, pevent->event_len);
		wlan_ralist_add(pmpriv, test_mac);
		memcpy_ext(pmpriv->adapter,
			   pmpriv->curr_bss_params.bss_descriptor.mac_address,
			   test_mac, MLAN_MAC_ADDR_LENGTH,
			   MLAN_MAC_ADDR_LENGTH);
		wlan_recv_event(pmpriv, MLAN_EVENT_ID_DRV_CONNECTED, pevent);
		if (pmpriv->port_ctrl_mode == MTRUE)
			pmpriv->port_open = MTRUE;
		pmpriv->media_connected = MTRUE;
		PRINTM_NETINTF(MEVENT, pmpriv);
		PRINTM(MEVENT, "nan interface - opened\n");
		pcb->moal_mfree(pmadapter->pmoal_handle, evt_buf);
	}

	LEAVE();
	return;
}

/********************************************************
			Global Functions
********************************************************/
/**
 *  @brief This function handles disconnect event, reports disconnect
 *          to upper layer, cleans tx/rx packets,
 *          resets link state etc.
 *
 *  @param priv             A pointer to mlan_private structure
 *  @param drv_disconnect   Flag indicating the driver should disconnect
 *                          and flush pending packets.
 *
 *  @return                 N/A
 */
t_void wlan_reset_connect_state(pmlan_private priv, t_u8 drv_disconnect)
{
	mlan_adapter *pmadapter = priv->adapter;
	mlan_status ret = MLAN_STATUS_SUCCESS;
	state_11d_t enable;
	t_u8 event_buf[100];
	mlan_event *pevent = (mlan_event *)event_buf;

	ENTER();

	PRINTM(MINFO, "Handles disconnect event.\n");

#ifdef UAP_SUPPORT
	/* If DFS repeater mode is enabled and station interface disconnects
	 * then make sure that all uAPs are stopped.
	 */
	if (pmadapter->dfs_repeater)
		wlan_dfs_rep_disconnect(pmadapter);
#endif

	if (drv_disconnect) {
		priv->media_connected = MFALSE;
		pmadapter->state_rdh.tx_block = MFALSE;
#ifdef UAP_SUPPORT
		if (pmadapter->dfs_mode)
			wlan_11h_update_dfs_master_state_on_disconect(priv);
		else
#endif
			wlan_11h_check_update_radar_det_state(priv);
	}

	if (priv->port_ctrl_mode == MTRUE) {
		/* Close the port on Disconnect */
		PRINTM(MINFO, "DISC: port_status = CLOSED\n");
		priv->port_open = MFALSE;
	}
	memset(pmadapter, &priv->gtk_rekey, 0,
	       sizeof(mlan_ds_misc_gtk_rekey_data));
	priv->tx_pause = MFALSE;
	pmadapter->scan_block = MFALSE;

	/* Reset SNR/NF/RSSI values */
	priv->data_rssi_last = 0;
	priv->data_nf_last = 0;
	priv->data_rssi_avg = 0;
	priv->data_nf_avg = 0;
	priv->bcn_rssi_last = 0;
	priv->bcn_nf_last = 0;
	priv->bcn_rssi_avg = 0;
	priv->bcn_nf_avg = 0;
	priv->rxpd_rate = 0;
	priv->rxpd_rate_info = 0;
	priv->max_amsdu = 0;
	priv->amsdu_disable = MFALSE;
	priv->multi_ap_flag = 0;
	wlan_coex_ampdu_rxwinsize(pmadapter);

	priv->sec_info.ewpa_enabled = MFALSE;
	priv->sec_info.wpa_enabled = MFALSE;
	priv->sec_info.wpa2_enabled = MFALSE;
	priv->wpa_ie_len = 0;

	priv->sec_info.wapi_enabled = MFALSE;
	priv->wapi_ie_len = 0;
	priv->sec_info.wapi_key_on = MFALSE;

	priv->wps.session_enable = MFALSE;
	memset(priv->adapter, (t_u8 *)&priv->wps.wps_ie, 0x00,
	       sizeof(priv->wps.wps_ie));
	priv->sec_info.osen_enabled = MFALSE;
	priv->osen_ie_len = 0;

	priv->sec_info.encryption_mode = MLAN_ENCRYPTION_MODE_NONE;

	/*Enable auto data rate */
	priv->is_data_rate_auto = MTRUE;
	priv->data_rate = 0;

	if (drv_disconnect) {
		/* Free Tx and Rx packets, report disconnect to upper layer */
		wlan_clean_txrx(priv);

		/* Need to erase the current SSID and BSSID info */
		memset(pmadapter, &priv->curr_bss_params, 0x00,
		       sizeof(priv->curr_bss_params));
	}
	wlan_send_tdls_tear_down_request(priv);
	wlan_delete_station_list(priv);
	pmadapter->tdls_status = TDLS_NOT_SETUP;
	priv->wmm_qosinfo = priv->saved_wmm_qosinfo;
	pmadapter->sleep_period.period = pmadapter->saved_sleep_period.period;
	pmadapter->tx_lock_flag = MFALSE;
	pmadapter->pps_uapsd_mode = MFALSE;
	pmadapter->delay_null_pkt = MFALSE;

	if ((wlan_fw_11d_is_enabled(priv)) &&
	    (priv->state_11d.user_enable_11d == DISABLE_11D)) {
		priv->state_11d.enable_11d = DISABLE_11D;
		enable = DISABLE_11D;

		/* Send cmd to FW to enable/disable 11D function */
		ret = wlan_prepare_cmd(priv, HostCmd_CMD_802_11_SNMP_MIB,
				       HostCmd_ACT_GEN_SET, Dot11D_i, MNULL,
				       &enable);
		if (ret)
			PRINTM(MERROR, "11D: Failed to enable 11D\n");
	}
	if (pmadapter->num_cmd_timeout && pmadapter->curr_cmd &&
	    (pmadapter->cmd_timer_is_set == MFALSE)) {
		LEAVE();
		return;
	}

	pevent->bss_index = priv->bss_index;
	pevent->event_id = MLAN_EVENT_ID_FW_DISCONNECTED;
	pevent->event_len = sizeof(priv->disconnect_reason_code);
	memcpy_ext(priv->adapter, pevent->event_buf,
		   &priv->disconnect_reason_code, pevent->event_len,
		   pevent->event_len);
	wlan_recv_event(priv, MLAN_EVENT_ID_FW_DISCONNECTED, pevent);
	priv->disconnect_reason_code = 0;
	priv->delay_link_lost = MFALSE;

	LEAVE();
}

/**
 *  @brief This function sends the OBSS scan parameters to the application
 *
 *  @param pmpriv     A pointer to mlan_private structure
 *
 *  @return           N/A
 */
t_void wlan_2040_coex_event(pmlan_private pmpriv)
{
	t_u8 event_buf[100];
	mlan_event *pevent = (mlan_event *)event_buf;
	t_u8 ele_len;

	ENTER();

	if (pmpriv->curr_bss_params.bss_descriptor.poverlap_bss_scan_param &&
	    pmpriv->curr_bss_params.bss_descriptor.poverlap_bss_scan_param
			    ->ieee_hdr.element_id == OVERLAPBSSSCANPARAM) {
		ele_len = pmpriv->curr_bss_params.bss_descriptor
				  .poverlap_bss_scan_param->ieee_hdr.len;
		pevent->bss_index = pmpriv->bss_index;
		pevent->event_id = MLAN_EVENT_ID_DRV_OBSS_SCAN_PARAM;
		pevent->event_len = ele_len;
		/* Copy OBSS scan parameters */
		memcpy_ext(pmpriv->adapter, (t_u8 *)pevent->event_buf,
			   (t_u8 *)&pmpriv->curr_bss_params.bss_descriptor
				   .poverlap_bss_scan_param->obss_scan_param,
			   ele_len, pevent->event_len);
		wlan_recv_event(pmpriv, MLAN_EVENT_ID_DRV_OBSS_SCAN_PARAM,
				pevent);
	}

	LEAVE();
}

/**
 *  @brief This function will process tx pause event
 *
 *  @param priv    A pointer to mlan_private
 *  @param pevent  A pointer to event buf
 *
 *  @return        N/A
 */
static void wlan_process_sta_tx_pause_event(pmlan_private priv,
					    pmlan_buffer pevent)
{
	t_u16 tlv_type, tlv_len;
	int tlv_buf_left = pevent->data_len - sizeof(t_u32);
	MrvlIEtypesHeader_t *tlv =
		(MrvlIEtypesHeader_t *)(pevent->pbuf + pevent->data_offset +
					sizeof(t_u32));
	MrvlIEtypes_tx_pause_t *tx_pause_tlv;
	sta_node *sta_ptr = MNULL;
	tdlsStatus_e status;
	t_u8 *bssid = MNULL;
	ENTER();
	if (priv->media_connected)
		bssid = priv->curr_bss_params.bss_descriptor.mac_address;
	while (tlv_buf_left >= (int)sizeof(MrvlIEtypesHeader_t)) {
		tlv_type = wlan_le16_to_cpu(tlv->type);
		tlv_len = wlan_le16_to_cpu(tlv->len);
		if ((sizeof(MrvlIEtypesHeader_t) + tlv_len) >
		    (unsigned int)tlv_buf_left) {
			PRINTM(MERROR, "wrong tlv: tlvLen=%d, tlvBufLeft=%d\n",
			       tlv_len, tlv_buf_left);
			break;
		}
		if (tlv_type == TLV_TYPE_TX_PAUSE) {
			tx_pause_tlv = (MrvlIEtypes_tx_pause_t *)tlv;
			PRINTM(MCMND, "TxPause: " MACSTR " pause=%d, pkts=%d\n",
			       MAC2STR(tx_pause_tlv->peermac),
			       tx_pause_tlv->tx_pause, tx_pause_tlv->pkt_cnt);
			status = wlan_get_tdls_link_status(
				priv, tx_pause_tlv->peermac);
			if (status != TDLS_NOT_SETUP) {
				if (MTRUE == wlan_is_tdls_link_setup(status)) {
					sta_ptr = wlan_get_station_entry(
						priv, tx_pause_tlv->peermac);
					if (sta_ptr) {
						if (sta_ptr->tx_pause !=
						    tx_pause_tlv->tx_pause) {
							sta_ptr->tx_pause =
								tx_pause_tlv
									->tx_pause;
							wlan_update_ralist_tx_pause(
								priv,
								tx_pause_tlv
									->peermac,
								tx_pause_tlv
									->tx_pause);
						}
					}
				}
			} else {
				if (tx_pause_tlv->tx_pause)
					priv->tx_pause = MTRUE;
				else
					priv->tx_pause = MFALSE;
			}
		}
		tlv_buf_left -= (sizeof(MrvlIEtypesHeader_t) + tlv_len);
		tlv = (MrvlIEtypesHeader_t *)((t_u8 *)tlv + tlv_len +
					      sizeof(MrvlIEtypesHeader_t));
	}

	LEAVE();
	return;
}

/**
 *  @brief This function will print diconnect reason code according
 *  to IEEE 802.11 spec
 *
 *  @param reason_code    reason code for the deauth/disaccoc
 *                        received from firmware
 *  @return        N/A
 */
static void wlan_print_disconnect_reason(t_u16 reason_code)
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
 *  @brief This function handles events generated by firmware
 *
 *  @param priv A pointer to mlan_private structure
 *
 *  @return     MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status wlan_ops_sta_process_event(t_void *priv)
{
	pmlan_private pmpriv = (pmlan_private)priv;
	pmlan_adapter pmadapter = pmpriv->adapter;
	mlan_status ret = MLAN_STATUS_SUCCESS;
	t_u32 eventcause = pmadapter->event_cause;
	t_u8 *event_buf = MNULL;
	t_u8 *evt_buf = MNULL;
	pmlan_buffer pmbuf = pmadapter->pmlan_buffer_event;
	t_u16 reason_code;
	pmlan_callbacks pcb = &pmadapter->callbacks;
	mlan_event *pevent = MNULL;
	t_u8 addr[MLAN_MAC_ADDR_LENGTH];
	Event_WLS_FTM_t *event_ftm = MNULL;
	chan_band_info *pchan_band_info = MNULL;
	t_u8 radar_chan;
	t_u8 bandwidth;
	chan_band_reginfo_t *psta_info = MNULL;
	chan_band_reginfo_t *psta_reg_info = MNULL;
	t_u16 enable = 0;
	Event_Link_Lost *link_lost_evt = MNULL;
	remain_on_channel_info *roc_info = MNULL;

	ENTER();

	if (!pmbuf) {
		PRINTM(MERROR, "ERR:event buffer is null\n");
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}

	/* Event length check */
	if ((pmbuf->data_len - sizeof(eventcause)) > MAX_EVENT_SIZE) {
		pmbuf->status_code = MLAN_ERROR_PKT_SIZE_INVALID;
		PRINTM(MERROR, "ERR:event buffer len invalid=%d\n",
		       pmbuf->data_len);
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}

	/* Allocate memory for event buffer */
	ret = pcb->moal_malloc(pmadapter->pmoal_handle,
			       MAX_EVENT_SIZE + sizeof(mlan_event),
			       MLAN_MEM_DEF, &event_buf);
	if ((ret != MLAN_STATUS_SUCCESS) || !event_buf) {
		PRINTM(MERROR, "Could not allocate buffer for event buf\n");
		if (pmbuf)
			pmbuf->status_code = MLAN_ERROR_NO_MEM;
		goto done;
	}
	pevent = (pmlan_event)event_buf;
	memset(pmadapter, event_buf, 0, MAX_EVENT_SIZE);

	if (eventcause != EVENT_PS_SLEEP && eventcause != EVENT_PS_AWAKE &&
	    pmbuf->data_len > sizeof(eventcause))
		DBG_HEXDUMP(MEVT_D, "EVENT", pmbuf->pbuf + pmbuf->data_offset,
			    pmbuf->data_len);

	switch (eventcause) {
	case EVENT_DUMMY_HOST_WAKEUP_SIGNAL:
		PRINTM(MERROR,
		       "Invalid EVENT: DUMMY_HOST_WAKEUP_SIGNAL, ignoring it\n");
		break;
	case EVENT_LINK_SENSED:
		PRINTM(MEVENT, "EVENT: LINK_SENSED\n");
		break;

	case EVENT_DEAUTHENTICATED:
		if (pmpriv->wps.session_enable) {
			PRINTM(MMSG,
			       "wlan: Receive deauth event in wps session\n");
			break;
		}
		reason_code = wlan_le16_to_cpu(*(t_u16 *)(pmbuf->pbuf +
							  pmbuf->data_offset +
							  sizeof(eventcause)));
		PRINTM(MMSG, "wlan: EVENT: Deauthenticated (reason 0x%x)\n",
		       reason_code);
		wlan_print_disconnect_reason(reason_code);
		pmpriv->disconnect_reason_code = reason_code;
		pmadapter->dbg.num_event_deauth++;
		wlan_handle_disconnect_event(pmpriv);

		break;

	case EVENT_DISASSOCIATED:
		if (pmpriv->wps.session_enable) {
			PRINTM(MMSG,
			       "wlan: Receive disassociate event in wps session\n");
			break;
		}
		reason_code = wlan_le16_to_cpu(*(t_u16 *)(pmbuf->pbuf +
							  pmbuf->data_offset +
							  sizeof(eventcause)));
		PRINTM(MMSG, "wlan: EVENT: Disassociated (reason 0x%x)\n",
		       reason_code);
		wlan_print_disconnect_reason(reason_code);
		pmpriv->disconnect_reason_code = reason_code;
		pmadapter->dbg.num_event_disassoc++;
		wlan_handle_disconnect_event(pmpriv);
		break;

	case EVENT_LINK_LOST:
		if (pmpriv->curr_bss_params.host_mlme &&
		    !pmpriv->assoc_rsp_size) {
			pmpriv->prior_assoc_rsp_size = 0;
			pmpriv->prior_assoc_req_size = 0;
			pmpriv->delay_link_lost = MTRUE;
			PRINTM(MMSG,
			       "wlan: skip link lost event before associate complete\n");
			break;
		}
		if (pmbuf && (pmbuf->data_len >=
			      sizeof(eventcause) + sizeof(Event_Link_Lost))) {
			link_lost_evt = (Event_Link_Lost *)(pmbuf->pbuf +
							    pmbuf->data_offset +
							    sizeof(eventcause));
			PRINTM(MMSG,
			       "wlan: EVENT: Link lost (reason 0x%x) bssid: " MACSTR
			       "\n",
			       link_lost_evt->reason_code,
			       MAC2STR(link_lost_evt->bssid));
			pmpriv->disconnect_reason_code =
				link_lost_evt->reason_code;
			if (memcmp(pmpriv->adapter, link_lost_evt->bssid,
				   &pmpriv->curr_bss_params.attemp_bssid,
				   MLAN_MAC_ADDR_LENGTH)) {
				pmpriv->prior_assoc_rsp_size = 0;
				pmpriv->prior_assoc_req_size = 0;
				pmpriv->delay_link_lost = MTRUE;
				PRINTM(MMSG, "wlan: skip link lost event\n");
				PRINTM(MMSG, "pattempted_bssid: " MACSTR "\n",
				       MAC2STR((
					       t_u8 *)(&pmpriv->curr_bss_params
								.attemp_bssid)));
				break;
			}
		} else {
			if (memcmp(pmpriv->adapter,
				   pmpriv->curr_bss_params.bss_descriptor
					   .mac_address,
				   &pmpriv->curr_bss_params.attemp_bssid,
				   MLAN_MAC_ADDR_LENGTH)) {
				PRINTM(MMSG, "wlan: skip link lost event\n");
				PRINTM(MMSG,
				       "pattempted_bssid: " MACSTR
				       " curr_bssid:" MACSTR "\n",
				       MAC2STR((t_u8 *)(&pmpriv->curr_bss_params
								 .attemp_bssid)),
				       MAC2STR(pmpriv->curr_bss_params
						       .bss_descriptor
						       .mac_address));
				break;
			}
			reason_code = wlan_le16_to_cpu(
				*(t_u16 *)(pmbuf->pbuf + pmbuf->data_offset +
					   sizeof(eventcause)));
			PRINTM(MMSG, "wlan: EVENT: Link lost (reason 0x%x)\n",
			       reason_code);
			pmpriv->disconnect_reason_code = reason_code;
		}
		pmadapter->dbg.num_event_link_lost++;
		wlan_handle_disconnect_event(pmpriv);
		break;

	case EVENT_PS_SLEEP:
		PRINTM(MINFO, "EVENT: SLEEP\n");
		if (pmadapter->second_mac)
			PRINTM(MEVENT, "__");
		else
			PRINTM(MEVENT, "_");

		/* Handle unexpected PS SLEEP event */
		if (pmadapter->ps_state == PS_STATE_SLEEP_CFM)
			break;
		pmadapter->ps_state = PS_STATE_PRE_SLEEP;

		wlan_check_ps_cond(pmadapter);
		break;

	case EVENT_PS_AWAKE:
		PRINTM(MINFO, "EVENT: AWAKE\n");
		if (pmadapter->second_mac)
			PRINTM(MEVENT, "||");
		else
			PRINTM(MEVENT, "|");
		if (!pmadapter->pps_uapsd_mode && pmpriv->media_connected &&
		    (pmpriv->port_open || !pmpriv->port_ctrl_mode) &&
		    pmadapter->sleep_period.period) {
			pmadapter->pps_uapsd_mode = MTRUE;
			PRINTM(MEVENT, "PPS/UAPSD mode activated\n");
		}
		/* Handle unexpected PS AWAKE event */
		if (pmadapter->ps_state == PS_STATE_SLEEP_CFM)
			break;
		pmadapter->tx_lock_flag = MFALSE;
		if (pmadapter->pps_uapsd_mode && pmadapter->gen_null_pkt) {
			if (MTRUE ==
			    wlan_check_last_packet_indication(pmpriv)) {
				if (!pmadapter->data_sent
#if defined(USB)
				    && wlan_is_port_ready(pmadapter,
							  pmpriv->port_index)
#endif
				) {
					if (wlan_send_null_packet(
						    pmpriv,
						    MRVDRV_TxPD_POWER_MGMT_NULL_PACKET |
							    MRVDRV_TxPD_POWER_MGMT_LAST_PACKET) ==
					    MLAN_STATUS_SUCCESS) {
						ret = MLAN_STATUS_SUCCESS;
						goto done;
					}
				}
			}
		}
		pmadapter->ps_state = PS_STATE_AWAKE;
		pmadapter->pm_wakeup_card_req = MFALSE;
		pmadapter->pm_wakeup_fw_try = MFALSE;
		break;

	case EVENT_HS_ACT_REQ:
		PRINTM(MEVENT, "EVENT: HS_ACT_REQ\n");
		ret = wlan_prepare_cmd(priv, HostCmd_CMD_802_11_HS_CFG_ENH, 0,
				       0, MNULL, MNULL);
		break;
	case EVENT_MIC_ERR_UNICAST:
		PRINTM(MEVENT, "EVENT: UNICAST MIC ERROR\n");
		wlan_recv_event(pmpriv, MLAN_EVENT_ID_FW_MIC_ERR_UNI, MNULL);
		break;

	case EVENT_MIC_ERR_MULTICAST:
		PRINTM(MEVENT, "EVENT: MULTICAST MIC ERROR\n");
		wlan_recv_event(pmpriv, MLAN_EVENT_ID_FW_MIC_ERR_MUL, MNULL);
		break;
	case EVENT_MIB_CHANGED:
	case EVENT_INIT_DONE:
		break;

	case EVENT_ASSOC_REQ_IE:
		pmpriv->assoc_req_size = pmbuf->data_len - sizeof(eventcause);
		evt_buf =
			(pmbuf->pbuf + pmbuf->data_offset + sizeof(eventcause));
		memcpy_ext(pmpriv->adapter, pmpriv->assoc_req_buf, evt_buf,
			   pmbuf->data_len - sizeof(eventcause),
			   ASSOC_RSP_BUF_SIZE);
		break;

	case EVENT_FW_DEBUG_INFO:
		pevent->bss_index = pmpriv->bss_index;
		pevent->event_id = MLAN_EVENT_ID_FW_DEBUG_INFO;
		pevent->event_len = pmbuf->data_len - sizeof(eventcause);
		memcpy_ext(pmadapter, (t_u8 *)pevent->event_buf,
			   pmbuf->pbuf + pmbuf->data_offset +
				   sizeof(eventcause),
			   pevent->event_len, pevent->event_len);
		PRINTM(MEVENT, "EVENT: FW Debug Info %s\n",
		       (t_u8 *)pevent->event_buf);
		wlan_recv_event(pmpriv, pevent->event_id, pevent);
		break;

	case EVENT_CHAN_SWITCH_TO_6G_BLOCK:
		reason_code = wlan_le16_to_cpu(*(t_u16 *)(pmbuf->pbuf +
							  pmbuf->data_offset +
							  sizeof(eventcause)));
		print_chan_switch_block_event(reason_code);
		break;

	case EVENT_BG_SCAN_REPORT:
		PRINTM(MEVENT, "EVENT: BGS_REPORT\n");
		pmadapter->bgscan_reported = MTRUE;
		wlan_recv_event(pmpriv, MLAN_EVENT_ID_FW_BG_SCAN, MNULL);
		break;
	case EVENT_BG_SCAN_STOPPED:
		PRINTM(MEVENT, "EVENT: BGS_STOPPED\n");
		wlan_recv_event(pmpriv, MLAN_EVENT_ID_FW_BG_SCAN_STOPPED,
				MNULL);
		break;

	case EVENT_PORT_RELEASE:
		PRINTM(MEVENT, "EVENT: PORT RELEASE\n");
		pmpriv->tx_pause = MFALSE;
		/* Open the port for e-supp mode */
		if (pmpriv->port_ctrl_mode == MTRUE) {
			PRINTM(MINFO, "PORT_REL: port_status = OPEN\n");
			pmpriv->port_open = MTRUE;
		}
		pmadapter->scan_block = MFALSE;
		wlan_recv_event(pmpriv, MLAN_EVENT_ID_FW_PORT_RELEASE, MNULL);
		/* Send OBSS scan param to the application */
		wlan_2040_coex_event(pmpriv);
		break;

	case EVENT_STOP_TX:
		PRINTM(MEVENT, "EVENT: Stop Tx (%#x)\n", eventcause);
		wlan_11h_tx_disable(pmpriv); /* this fn will send event up to
						MOAL */
		break;
	case EVENT_START_TX:
		PRINTM(MEVENT, "EVENT: Start Tx (%#x)\n", eventcause);
		wlan_11h_tx_enable(pmpriv); /* this fn will send event up to
					       MOAL */
		break;
	case EVENT_CHANNEL_SWITCH:
		PRINTM(MEVENT, "EVENT: Channel Switch (%#x)\n", eventcause);
		if (pmadapter->ecsa_enable) {
			MrvlIEtypes_channel_band_t *pchan_info =
				(MrvlIEtypes_channel_band_t
					 *)(pmadapter->event_body);
			t_u8 channel = pchan_info->channel;
			chan_freq_power_t *cfp = MNULL;
			DBG_HEXDUMP(MCMD_D, "chan band config",
				    (t_u8 *)pchan_info,
				    sizeof(MrvlIEtypes_channel_band_t));
			PRINTM(MEVENT, "Switch to channel %d success!\n",
			       channel);
#define MAX_CHANNEL_BAND_B 14
			if (channel <= MAX_CHANNEL_BAND_B)
				cfp = wlan_find_cfp_by_band_and_channel(
					pmadapter, BAND_B, channel);
			else
				cfp = wlan_find_cfp_by_band_and_channel(
					pmadapter, BAND_A, channel);
			pmpriv->curr_bss_params.bss_descriptor.channel =
				channel;
			if (cfp)
				pmpriv->curr_bss_params.bss_descriptor.freq =
					cfp->freq;
			else
				pmpriv->curr_bss_params.bss_descriptor.freq = 0;
#ifdef UAP_SUPPORT
			if (pmpriv->adapter->dfs_mode)
				wlan_11h_update_dfs_master_state_by_sta(pmpriv);
#endif
			if (pmpriv->adapter->state_rdh.stage ==
			    RDH_SET_CUSTOM_IE) {
				pmadapter->state_rdh.stage =
					RDH_RESTART_TRAFFIC;
				wlan_11h_radar_detected_handling(pmadapter,
								 pmpriv);
			}
			pmadapter->state_rdh.tx_block = MFALSE;
			/* Setup event buffer */
			pevent->bss_index = pmpriv->bss_index;
			pevent->event_id =
				MLAN_EVENT_ID_FW_CHAN_SWITCH_COMPLETE;
			pevent->event_len = sizeof(chan_band_info);
			pchan_band_info = (chan_band_info *)pevent->event_buf;
			/* Copy event data */
			memcpy_ext(pmadapter, (t_u8 *)&pchan_band_info->bandcfg,
				   (t_u8 *)&pchan_info->bandcfg,
				   sizeof(pchan_info->bandcfg),
				   sizeof(pchan_band_info->bandcfg));
			pchan_band_info->channel = pchan_info->channel;
			if (pchan_band_info->bandcfg.chanWidth == CHAN_BW_80MHZ)
				pchan_band_info
					->center_chan = wlan_get_center_freq_idx(
					priv, pchan_band_info->bandcfg.chanBand,
					pchan_info->channel, CHANNEL_BW_80MHZ);
			wlan_recv_event(pmpriv,
					MLAN_EVENT_ID_FW_CHAN_SWITCH_COMPLETE,
					pevent);
		}
		break;
	case EVENT_CHANNEL_SWITCH_ANN:
		PRINTM_NETINTF(MEVENT, pmpriv);
		PRINTM(MEVENT, "EVENT: Channel Switch Announcement\n");
		/* Here, pass up event first, as handling will send deauth */
		wlan_recv_event(pmpriv, MLAN_EVENT_ID_FW_CHANNEL_SWITCH_ANN,
				MNULL);
		wlan_11h_handle_event_chanswann(pmpriv);
		break;
	case EVENT_RADAR_DETECTED:
		PRINTM_NETINTF(MEVENT, pmpriv);
		PRINTM(MEVENT, "EVENT: Radar Detected\n");

		/* Send as passthru first, this event can cause other events */
		pevent->bss_index = pmpriv->bss_index;
		pevent->event_id = MLAN_EVENT_ID_DRV_PASSTHRU;
		pevent->event_len = MIN(pmbuf->data_len, MAX_EVENT_SIZE);
		memcpy_ext(pmadapter, (t_u8 *)pevent->event_buf,
			   pmbuf->pbuf + pmbuf->data_offset, pevent->event_len,
			   pevent->event_len);
		wlan_recv_event(pmpriv, pevent->event_id, pevent);

		if (pmadapter->state_rdh.stage == RDH_OFF) {
			pmadapter->state_rdh.stage = RDH_CHK_INTFS;
			wlan_11h_radar_detected_handling(pmadapter, pmpriv);
		} else {
			PRINTM(MEVENT, "Ignore Event Radar Detected - handling"
				       " already in progress.\n");
		}

		break;

	case EVENT_CHANNEL_REPORT_RDY:
		PRINTM_NETINTF(MEVENT, pmpriv);
		PRINTM(MEVENT, "EVENT: Channel Report Ready\n");
		pevent->bss_index = pmpriv->bss_index;
		pevent->event_id = MLAN_EVENT_ID_FW_CHANNEL_REPORT_RDY;
		pevent->event_len = pmbuf->data_len - sizeof(eventcause);
		/* Copy event data */
		memcpy_ext(pmadapter, (t_u8 *)pevent->event_buf,
			   pmbuf->pbuf + pmbuf->data_offset +
				   sizeof(eventcause),
			   pevent->event_len, pevent->event_len);
		/* Handle / pass event data */
		ret = wlan_11h_handle_event_chanrpt_ready(
			pmpriv, pevent, &radar_chan, &bandwidth);
		/* Also send this event as passthru */
		pevent->event_id = MLAN_EVENT_ID_DRV_PASSTHRU;
		pevent->event_len = pmbuf->data_len;
		// Ensure event_len does not exceed buffer size
		pevent->event_len = MIN(pmbuf->data_len, MAX_EVENT_SIZE);
		memcpy_ext(pmadapter, (t_u8 *)pevent->event_buf,
			   pmbuf->pbuf + pmbuf->data_offset, pevent->event_len,
			   pevent->event_len);
		wlan_recv_event(pmpriv, pevent->event_id, pevent);
		/* Send up this Event to unblock MOAL waitqueue */
		wlan_recv_event(pmpriv, MLAN_EVENT_ID_DRV_MEAS_REPORT, MNULL);
		break;
	case EVENT_EXT_SCAN_REPORT:
		PRINTM(MEVENT, "EVENT: EXT_SCAN Report (%d)\n",
		       pmbuf->data_len);
		if (pmadapter->pscan_ioctl_req && pmadapter->ext_scan)
			ret = wlan_handle_event_ext_scan_report(priv, pmbuf);
		break;
	case EVENT_EXT_SCAN_STATUS_REPORT:
		PRINTM(MEVENT, "EVENT: EXT_SCAN status report (%d)\n",
		       pmbuf->data_len);
		pmadapter->ext_scan_timeout = MFALSE;
		ret = wlan_handle_event_ext_scan_status(priv, pmbuf);
		break;
	case EVENT_MEAS_REPORT_RDY:
		PRINTM(MEVENT, "EVENT: Measurement Report Ready (%#x)\n",
		       eventcause);
		ret = wlan_prepare_cmd(priv, HostCmd_CMD_MEASUREMENT_REPORT,
				       HostCmd_ACT_GEN_SET, 0, MNULL, MNULL);
		break;
	case EVENT_WMM_STATUS_CHANGE:
		if (pmbuf &&
		    pmbuf->data_len >
			    sizeof(eventcause) + sizeof(MrvlIEtypesHeader_t)) {
			PRINTM(MEVENT, "EVENT: WMM status changed: %d\n",
			       pmbuf->data_len);

			evt_buf = (pmbuf->pbuf + pmbuf->data_offset +
				   sizeof(eventcause));

			wlan_ret_wmm_get_status(pmpriv, evt_buf,
						pmbuf->data_len -
							sizeof(eventcause));
		} else {
			PRINTM(MEVENT, "EVENT: WMM status changed\n");
			ret = wlan_cmd_wmm_status_change(pmpriv);
		}
		break;

	case EVENT_RSSI_LOW:
		PRINTM(MEVENT, "EVENT: Beacon RSSI_LOW\n");
		pevent->bss_index = pmpriv->bss_index;
		pevent->event_id = MLAN_EVENT_ID_FW_BCN_RSSI_LOW;
		pevent->event_len = sizeof(t_u16);
		/** Fw send bcnRssi low value in event reason field*/
		memcpy_ext(pmadapter, (t_u8 *)pevent->event_buf,
			   (t_u8 *)&pmadapter->event_body, pevent->event_len,
			   pevent->event_len);
		wlan_recv_event(pmpriv, pevent->event_id, pevent);
		break;
	case EVENT_SNR_LOW:
		PRINTM(MEVENT, "EVENT: Beacon SNR_LOW\n");
		wlan_recv_event(pmpriv, MLAN_EVENT_ID_FW_BCN_SNR_LOW, MNULL);
		break;
	case EVENT_MAX_FAIL:
		PRINTM(MEVENT, "EVENT: MAX_FAIL\n");
		wlan_recv_event(pmpriv, MLAN_EVENT_ID_FW_MAX_FAIL, MNULL);
		break;
	case EVENT_RSSI_HIGH:
		PRINTM(MEVENT, "EVENT: Beacon RSSI_HIGH\n");
		pevent->bss_index = pmpriv->bss_index;
		pevent->event_id = MLAN_EVENT_ID_FW_BCN_RSSI_HIGH;
		pevent->event_len = sizeof(t_u16);
		/** Fw send bcnRssi high value in event reason field*/
		memcpy_ext(pmadapter, (t_u8 *)pevent->event_buf,
			   (t_u8 *)&pmadapter->event_body, pevent->event_len,
			   pevent->event_len);
		wlan_recv_event(pmpriv, pevent->event_id, pevent);
		break;
	case EVENT_SNR_HIGH:
		PRINTM(MEVENT, "EVENT: Beacon SNR_HIGH\n");
		wlan_recv_event(pmpriv, MLAN_EVENT_ID_FW_BCN_SNR_HIGH, MNULL);
		break;
	case EVENT_DATA_RSSI_LOW:
		PRINTM(MEVENT, "EVENT: Data RSSI_LOW\n");
		wlan_recv_event(pmpriv, MLAN_EVENT_ID_FW_DATA_RSSI_LOW, MNULL);
		break;
	case EVENT_DATA_SNR_LOW:
		PRINTM(MEVENT, "EVENT: Data SNR_LOW\n");
		wlan_recv_event(pmpriv, MLAN_EVENT_ID_FW_DATA_SNR_LOW, MNULL);
		break;
	case EVENT_DATA_RSSI_HIGH:
		PRINTM(MEVENT, "EVENT: Data RSSI_HIGH\n");
		wlan_recv_event(pmpriv, MLAN_EVENT_ID_FW_DATA_RSSI_HIGH, MNULL);
		break;
	case EVENT_DATA_SNR_HIGH:
		PRINTM(MEVENT, "EVENT: Data SNR_HIGH\n");
		wlan_recv_event(pmpriv, MLAN_EVENT_ID_FW_DATA_SNR_HIGH, MNULL);
		break;
	case EVENT_LINK_QUALITY:
		PRINTM(MEVENT, "EVENT: Link Quality\n");
		wlan_recv_event(pmpriv, MLAN_EVENT_ID_FW_LINK_QUALITY, MNULL);
		break;
	case EVENT_PRE_BEACON_LOST:
		PRINTM(MEVENT, "EVENT: Pre-Beacon Lost\n");
		wlan_recv_event(pmpriv, MLAN_EVENT_ID_FW_PRE_BCN_LOST, MNULL);
		break;
	case EVENT_ADDBA:
		PRINTM(MEVENT, "EVENT: ADDBA Request\n");
		if (pmpriv->media_connected == MTRUE &&
		    !pmpriv->adapter->remain_on_channel)
			wlan_11n_add_bastream(pmpriv, pmadapter->event_body);
		else
			PRINTM(MERROR,
			       "Ignore ADDBA Request event in disconnected state\n");
		break;
	case EVENT_DELBA:
		PRINTM(MEVENT, "EVENT: DELBA Request\n");
		if (pmpriv->media_connected == MTRUE &&
		    !pmpriv->adapter->remain_on_channel)
			wlan_11n_delete_bastream(pmpriv, pmadapter->event_body);
		else
			PRINTM(MERROR,
			       "Ignore DELBA Request event in disconnected state\n");
		break;
	case EVENT_BA_STREAM_TIMEOUT:
		PRINTM(MEVENT, "EVENT:  BA Stream timeout\n");
		if (pmpriv->media_connected == MTRUE &&
		    !pmpriv->adapter->remain_on_channel)
			wlan_11n_ba_stream_timeout(
				pmpriv, (HostCmd_DS_11N_BATIMEOUT *)
						pmadapter->event_body);
		else
			PRINTM(MERROR,
			       "Ignore BA Stream timeout event in disconnected state\n");
		break;
	case EVENT_RXBA_SYNC:
		PRINTM(MEVENT, "EVENT:  RXBA_SYNC\n");
		wlan_11n_rxba_sync_event(pmpriv, pmadapter->event_body,
					 pmbuf->data_len - sizeof(eventcause));
		break;
	case EVENT_AMSDU_AGGR_CTRL:
		PRINTM(MEVENT, "EVENT:  AMSDU_AGGR_CTRL %d\n",
		       *(t_u16 *)pmadapter->event_body);
		pmadapter->tx_buf_size =
			MIN(pmadapter->curr_tx_buf_size,
			    wlan_le16_to_cpu(*(t_u16 *)pmadapter->event_body));
		if (pmbuf->data_len == sizeof(eventcause) + sizeof(t_u32)) {
			enable = wlan_le16_to_cpu(
				*(t_u16 *)(pmadapter->event_body +
					   sizeof(t_u16)));
			if (enable)
				pmpriv->amsdu_disable = MFALSE;
			else
				pmpriv->amsdu_disable = MTRUE;
			PRINTM(MEVENT, "amsdu_disable=%d\n",
			       pmpriv->amsdu_disable);
		}
		PRINTM(MEVENT, "tx_buf_size %d\n", pmadapter->tx_buf_size);
		break;

	case EVENT_WEP_ICV_ERR:
		PRINTM(MEVENT, "EVENT: WEP ICV error\n");
		pevent->bss_index = pmpriv->bss_index;
		pevent->event_id = MLAN_EVENT_ID_FW_WEP_ICV_ERR;
		pevent->event_len = sizeof(Event_WEP_ICV_ERR);
		memcpy_ext(pmadapter, (t_u8 *)pevent->event_buf,
			   pmadapter->event_body, pevent->event_len,
			   pevent->event_len);
		wlan_recv_event(pmpriv, MLAN_EVENT_ID_FW_WEP_ICV_ERR, pevent);
		break;

	case EVENT_BW_CHANGE:
		PRINTM(MEVENT, "EVENT: BW Change\n");
		pevent->bss_index = pmpriv->bss_index;
		pevent->event_id = MLAN_EVENT_ID_FW_BW_CHANGED;
		pevent->event_len = sizeof(t_u8);
		/* Copy event body from the event buffer */
		memcpy_ext(pmadapter, (t_u8 *)pevent->event_buf,
			   pmadapter->event_body, pevent->event_len,
			   pevent->event_len);
#ifdef UAP_SUPPORT
		if (pmadapter->dfs_repeater)
			wlan_dfs_rep_bw_change(pmadapter);
#endif
		wlan_recv_event(pmpriv, MLAN_EVENT_ID_FW_BW_CHANGED, pevent);
		break;

#ifdef WIFI_DIRECT_SUPPORT
	case EVENT_WIFIDIRECT_GENERIC_EVENT:
		PRINTM(MEVENT, "EVENT: WIFIDIRECT event %d\n", eventcause);
		pevent->bss_index = pmpriv->bss_index;
		pevent->event_id = MLAN_EVENT_ID_DRV_PASSTHRU;
		// Ensure event_len does not exceed buffer size
		pevent->event_len = MIN(pmbuf->data_len, MAX_EVENT_SIZE);
		memcpy_ext(pmadapter, (t_u8 *)pevent->event_buf,
			   pmbuf->pbuf + pmbuf->data_offset, pevent->event_len,
			   MAX_EVENT_SIZE);
		wlan_recv_event(pmpriv, pevent->event_id, pevent);
		break;
	case EVENT_WIFIDIRECT_SERVICE_DISCOVERY:
		PRINTM(MEVENT, "EVENT: WIFIDIRECT service discovery event %d\n",
		       eventcause);
		pevent->bss_index = pmpriv->bss_index;
		pevent->event_id = MLAN_EVENT_ID_DRV_PASSTHRU;
		// Ensure event_len does not exceed buffer size
		pevent->event_len = MIN(pmbuf->data_len, MAX_EVENT_SIZE);
		memcpy_ext(pmadapter, (t_u8 *)pevent->event_buf,
			   pmbuf->pbuf + pmbuf->data_offset, pevent->event_len,
			   pevent->event_len);
		wlan_recv_event(pmpriv, pevent->event_id, pevent);
		break;
#endif /* WIFI_DIRECT_SUPPORT */
	case EVENT_REMAIN_ON_CHANNEL_EXPIRED:
		PRINTM_NETINTF(MEVENT, pmpriv);
		PRINTM(MEVENT, "EVENT: REMAIN_ON_CHANNEL_EXPIRED reason=%d\n",
		       *(t_u16 *)pmadapter->event_body);
		pmpriv->adapter->remain_on_channel = MFALSE;
		wlan_recv_event(pmpriv, MLAN_EVENT_ID_DRV_FLUSH_RX_WORK, MNULL);
		wlan_recv_event(pmpriv, MLAN_EVENT_ID_FW_REMAIN_ON_CHAN_EXPIRED,
				MNULL);

		pevent->event_id = MLAN_EVENT_ID_FW_REMAIN_ON_CHAN_EXPIRED;
		pevent->bss_index = pmpriv->bss_index;
		pevent->event_len = sizeof(remain_on_channel_info);
		roc_info = (remain_on_channel_info *)pevent->event_buf;

		roc_info->delay_link_lost = pmpriv->delay_link_lost;
		if (pmpriv->delay_link_lost && pmpriv->media_connected) {
			pmpriv->delay_link_lost = MFALSE;
			wlan_handle_disconnect_event(pmpriv);
		}
		wlan_recv_event(pmpriv, MLAN_EVENT_ID_FW_REMAIN_ON_CHAN_EXPIRED,
				pevent);

		break;
	case EVENT_TDLS_GENERIC_EVENT:
		PRINTM(MEVENT, "EVENT: TDLS event %d\n", eventcause);
		wlan_parse_tdls_event(pmpriv, pmbuf);
		pevent->bss_index = pmpriv->bss_index;
		pevent->event_id = MLAN_EVENT_ID_DRV_PASSTHRU;
		// Ensure event_len does not exceed buffer size
		pevent->event_len = MIN(pmbuf->data_len, MAX_EVENT_SIZE);
		memcpy_ext(pmadapter, (t_u8 *)pevent->event_buf,
			   pmbuf->pbuf + pmbuf->data_offset, pevent->event_len,
			   pevent->event_len);
		wlan_recv_event(pmpriv, pevent->event_id, pevent);
		break;

	case EVENT_TX_DATA_PAUSE:
		PRINTM(MEVENT, "EVENT: TX_DATA_PAUSE\n");
		wlan_process_sta_tx_pause_event(priv, pmbuf);
		break;

	case EVENT_SAD_REPORT: {
#ifdef DEBUG_LEVEL1
		t_u8 *pevt_dat =
			pmbuf->pbuf + pmbuf->data_offset + sizeof(t_u32);
#endif
		PRINTM(MEVENT,
		       "EVENT: Antenna Diversity %d  (%d, %d, %d, %d)\n",
		       eventcause, pevt_dat[0] + 1, pevt_dat[1] + 1,
		       pevt_dat[2], pevt_dat[3]);
	} break;
	case EVENT_MULTI_CHAN_INFO:
		PRINTM(MEVENT, "EVENT: MULTI_CHAN_INFO\n");
		wlan_handle_event_multi_chan_info(pmpriv, pmbuf);
		break;

	case EVENT_FW_DUMP_INFO:
		PRINTM(MINFO, "EVENT: Dump FW info\n");
		pevent->bss_index = pmpriv->bss_index;
		pevent->event_id = MLAN_EVENT_ID_FW_DUMP_INFO;
		// Ensure event_len does not exceed buffer size
		pevent->event_len = MIN(pmbuf->data_len, MAX_EVENT_SIZE);
		memcpy_ext(pmadapter, (t_u8 *)pevent->event_buf,
			   pmbuf->pbuf + pmbuf->data_offset, pevent->event_len,
			   pevent->event_len);
		wlan_recv_event(pmpriv, pevent->event_id, pevent);
		break;
	case EVENT_TX_STATUS_REPORT:
		PRINTM(MINFO, "EVENT: TX_STATUS\n");
		pevent->bss_index = pmpriv->bss_index;
		pevent->event_id = MLAN_EVENT_ID_FW_TX_STATUS;
		// Ensure event_len does not exceed buffer size
		pevent->event_len = MIN(pmbuf->data_len, MAX_EVENT_SIZE);
		memcpy_ext(pmadapter, (t_u8 *)pevent->event_buf,
			   pmbuf->pbuf + pmbuf->data_offset, pevent->event_len,
			   pevent->event_len);

		wlan_recv_event(pmpriv, pevent->event_id, pevent);
		break;
	case EVENT_BT_COEX_WLAN_PARA_CHANGE:
		PRINTM(MEVENT, "EVENT: BT coex wlan param update\n");
		wlan_bt_coex_wlan_param_update_event(pmpriv, pmbuf);
		break;
	case EVENT_NAN_GENERIC:
		PRINTM(MEVENT, "EVENT: NAN_GENERIC_EVENT\n");
		wlan_process_nan_event(pmpriv, pmbuf);
		break;

#if defined(PCIE)
	case EVENT_SSU_DUMP_DMA:
		PRINTM(MEVENT, "EVENT: EVENT_SSU_DUMP_DMA\n");
		if (!pmadapter->ssu_buf || !pmadapter->ssu_buf->pbuf)
			break;
		pcb->moal_unmap_memory(pmadapter->pmoal_handle,
				       pmadapter->ssu_buf->pbuf +
					       pmadapter->ssu_buf->data_offset,
				       pmadapter->ssu_buf->buf_pa,
				       MLAN_SSU_BUF_SIZE, PCI_DMA_FROMDEVICE);
		/* If ADMA is supported, SSU header could not be received with
		 * SSU data. Instead, SSU header is received through this event.
		 * So, copy the header into the buffer before passing the buffer
		 * to upper layer for file writting
		 */
		memcpy_ext(pmadapter,
			   (t_u8 *)pmadapter->ssu_buf->pbuf +
				   pmadapter->ssu_buf->data_offset,
			   pmbuf->pbuf + pmbuf->data_offset +
				   sizeof(eventcause),
			   (pmbuf->data_len - sizeof(eventcause)),
			   (pmbuf->data_len - sizeof(eventcause)));

		DBG_HEXDUMP(MEVT_D, "SSU data",
			    (t_u8 *)pmadapter->ssu_buf->pbuf +
				    pmadapter->ssu_buf->data_offset,
			    512);
		pevent->bss_index = pmpriv->bss_index;
		pevent->event_id = MLAN_EVENT_ID_SSU_DUMP_FILE;
		pevent->event_len = MLAN_SSU_BUF_SIZE;
		*(t_ptr *)pevent->event_buf = (t_ptr)pmadapter->ssu_buf->pbuf +
					      pmadapter->ssu_buf->data_offset;
		wlan_recv_event(pmpriv, pevent->event_id, pevent);
		wlan_free_ssu_pcie_buf(pmadapter);
		break;
#endif
	case EVENT_CSI:
		PRINTM(MEVENT, "EVENT: EVENT_CSI on STA\n");
		wlan_process_csi_event(pmpriv);
		break;
	case EVENT_MEF_HOST_WAKEUP:
		PRINTM(MEVENT, "EVENT: EVENT_MEF_HOST_WAKEUP len=%d\n",
		       pmbuf->data_len);
		break;
	case EVENT_MANAGEMENT_FRAME_WAKEUP:
		PRINTM(MEVENT, "EVENT: EVENT_MANAGEMENT_FRAME_WAKEUP HOST\n");
		break;
	case EVENT_ROAM_OFFLOAD:
		memcpy_ext(pmadapter, addr,
			   pmpriv->curr_bss_params.bss_descriptor.mac_address,
			   MLAN_MAC_ADDR_LENGTH, MLAN_MAC_ADDR_LENGTH);
		memcpy_ext(pmadapter,
			   pmpriv->curr_bss_params.bss_descriptor.mac_address,
			   (t_u8 *)(pmadapter->event_body + 2),
			   MLAN_MAC_ADDR_LENGTH, MLAN_MAC_ADDR_LENGTH);
		/** replace ralist's mac address with new mac address */
		if (0 ==
		    wlan_ralist_update(
			    pmpriv, addr,
			    pmpriv->curr_bss_params.bss_descriptor.mac_address))
			wlan_ralist_add(pmpriv,
					pmpriv->curr_bss_params.bss_descriptor
						.mac_address);
		wlan_11n_cleanup_reorder_tbl(pmpriv);
		wlan_11n_deleteall_txbastream_tbl(pmpriv);
		/*Update the BSS for inform kernel, otherwise kernel will give
		 * warning for not find BSS*/
		memcpy_ext(pmadapter, (t_u8 *)&pmadapter->pscan_table[0],
			   (t_u8 *)&pmpriv->curr_bss_params.bss_descriptor,
			   sizeof(BSSDescriptor_t), sizeof(BSSDescriptor_t));
		if (!pmadapter->num_in_scan_table)
			pmadapter->num_in_scan_table = 1;
		PRINTM(MEVENT, "EVENT: ROAM OFFLOAD IN FW SUCCESS\n");
		pevent->bss_index = pmpriv->bss_index;
		pevent->event_id = MLAN_EVENT_ID_FW_ROAM_OFFLOAD_RESULT;
		/** Drop event id length and 2 bytes reverved length*/
		if ((pmbuf->data_len - sizeof(eventcause)) > 2) {
			pevent->event_len =
				pmbuf->data_len - sizeof(eventcause) - 2;
			memcpy_ext(pmadapter, (t_u8 *)pevent->event_buf,
				   pmadapter->event_body + 2, pevent->event_len,
				   pevent->event_len);
			wlan_recv_event(pmpriv, pevent->event_id, pevent);
		} else {
			PRINTM(MERROR,
			       "EVENT: ERR:: ROAM OFFLOAD IN FW has invalid length\n");
		}
		break;
	case EVENT_CLOUD_KEEP_ALIVE_RETRY_FAIL:
		break;
	case EVENT_WLS_FTM_COMPLETE:
		PRINTM(MEVENT, "EVENT: FTM_GENERIC_EVENT\n");
		pevent->bss_index = pmpriv->bss_index;
		event_ftm =
			(Event_WLS_FTM_t *)(pmbuf->pbuf + pmbuf->data_offset);
		if (event_ftm->sub_event_id == WLS_SUB_EVENT_RTT_RESULTS)
			wlan_fill_hal_rtt_results(pmpriv, event_ftm,
						  pmbuf->data_len, pevent);
		else {
			pevent->event_id = MLAN_EVENT_ID_DRV_PASSTHRU;
			// Ensure event_len does not exceed buffer size
			pevent->event_len =
				MIN(pmbuf->data_len, MAX_EVENT_SIZE);
			memcpy_ext(pmadapter, (t_u8 *)pevent->event_buf,
				   pmbuf->pbuf + pmbuf->data_offset,
				   pevent->event_len, pevent->event_len);
		}
		wlan_recv_event(pmpriv, pevent->event_id, pevent);
		break;
	case EVENT_VDLL_IND:
		wlan_process_vdll_event(pmpriv, pmbuf);
		break;
	case EVENT_FW_HANG_REPORT:
		if (pmbuf->data_len < (sizeof(eventcause) + sizeof(t_u16))) {
			PRINTM(MEVENT,
			       "EVENT: EVENT_FW_HANG_REPORT skip for len too short: %d\n",
			       pmbuf->data_len);
			break;
		}
		PRINTM(MEVENT, "EVENT: EVENT_FW_HANG_REPORT reasoncode=%d\n",
		       wlan_le16_to_cpu(*(t_u16 *)(pmbuf->pbuf +
						   pmbuf->data_offset +
						   sizeof(eventcause))));
		pmadapter->fw_hang_report = MTRUE;
		wlan_recv_event(pmpriv, MLAN_EVENT_ID_DRV_DBG_DUMP, MNULL);
		break;
	case EVENT_IMD3_CAL_START:
		PRINTM(MEVENT, "EVENT: EVENT_IMD3_CAL_START\n");
		break;
	case EVENT_IMD3_CAL_END:
		PRINTM(MEVENT, "EVENT: EVENT_IMD3_CAL_END\n");
		break;

	case EVENT_DPD_CAL:
		wlan_process_dpd_cal_event(pmpriv, pmbuf);
		break;

	case EVENT_CHAN_LOAD: {
		t_u8 *ptr = MNULL;
		HostCmd_DS_GET_CH_LOAD *cfg_cmd = MNULL;
		ptr = (t_u8 *)(pmbuf->pbuf + pmbuf->data_offset);
		ptr += 4; /* data start */
		cfg_cmd = (HostCmd_DS_GET_CH_LOAD *)ptr;
		pmpriv->ch_load_param = wlan_le16_to_cpu(cfg_cmd->ch_load);
		pmpriv->noise = wlan_le16_to_cpu(cfg_cmd->noise);
		pmpriv->rx_quality = wlan_le16_to_cpu(cfg_cmd->rx_quality);
		break;
	}
	case EVENT_CHANNEL_SWITCH_REGINFO:
		PRINTM(MEVENT, "EVENT: Channel Switch Reginfo (%#x)\n",
		       eventcause);
		psta_info = (chan_band_reginfo_t *)(pmadapter->event_body);
		DBG_HEXDUMP(MCMD_D, "chan band reginfo", (t_u8 *)psta_info,
			    sizeof(chan_band_reginfo_t));
		/* Setup event buffer */
		pevent->bss_index = pmpriv->bss_index;
		pevent->event_id = MLAN_EVENT_ID_FW_CHAN_SWITCH_REGINFO;
		pevent->event_len = sizeof(chan_band_reginfo_t);
		psta_reg_info = (chan_band_reginfo_t *)pevent->event_buf;
		/* Copy event data */
		if (psta_reg_info->bandcfg.chanBand == BAND_6GHZ) {
			memcpy_ext(pmadapter, (t_u8 *)&psta_reg_info->bandcfg,
				   (t_u8 *)&psta_info->bandcfg,
				   sizeof(psta_info->bandcfg),
				   sizeof(psta_reg_info->bandcfg));
			psta_reg_info->channel = psta_info->channel;
			psta_reg_info->regInfo = psta_info->regInfo;
			wlan_recv_event(pmpriv,
					MLAN_EVENT_ID_FW_CHAN_SWITCH_REGINFO,
					pevent);
		} else
			PRINTM(MEVENT,
			       "Ignoring the Channel Switch Reg Info Event\n");
		break;
	default:
		PRINTM(MEVENT, "EVENT: unknown event id: %#x\n", eventcause);
		wlan_recv_event(pmpriv, MLAN_EVENT_ID_FW_UNKNOWN, MNULL);
		break;
	}
done:
	if (event_buf)
		pcb->moal_mfree(pmadapter->pmoal_handle, event_buf);
	LEAVE();
	return ret;
}
