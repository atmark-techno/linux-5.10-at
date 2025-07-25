/** @file mlan_wmm.c
 *
 *  @brief This file contains functions for WMM.
 *
 *
 *  Copyright 2008-2021, 2025 NXP
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
    10/24/2008: initial version
********************************************************/

#include "mlan.h"
#ifdef STA_SUPPORT
#include "mlan_join.h"
#endif
#include "mlan_util.h"
#include "mlan_fw.h"
#include "mlan_main.h"
#include "mlan_wmm.h"
#include "mlan_11n.h"
#include "mlan_11ax.h"
#ifdef SDIO
#include "mlan_sdio.h"
#endif /* SDIO */
#ifdef PCIE
#include "mlan_pcie.h"
#endif /* PCIE */

/********************************************************
			Local Variables
********************************************************/

/** WMM information IE */
static const t_u8 wmm_info_ie[] = {WMM_IE, 0x07, 0x00, 0x50, 0xf2,
				   0x02,   0x00, 0x01, 0x00};

/** Type enumeration of WMM AC_QUEUES */
typedef MLAN_PACK_START enum _wmm_ac_e {
	AC_BE,
	AC_BK,
	AC_VI,
	AC_VO
} MLAN_PACK_END wmm_ac_e;

/**
 * AC Priorities go from AC_BK to AC_VO.  The ACI enumeration for AC_BK (1)
 *   is higher than the enumeration for AC_BE (0); hence the needed
 *   mapping conversion for wmm AC to priority Queue Index
 */
static const t_u8 wmm_aci_to_qidx_map[] = {WMM_AC_BE, WMM_AC_BK, WMM_AC_VI,
					   WMM_AC_VO};

/** AC to ACI mapping as per IEEE802.11 spec */
static const t_u8 wmm_ac_to_aci_map[] = {AC_BK, AC_BE, AC_VI, AC_VO};

/**
 * This table will be used to store the tid values based on ACs.
 * It is initialized to default values per TID.
 */
static t_u8 tos_to_tid[] = {
	/* TID        DSCP_P2   DSCP_P1  DSCP_P0   WMM_AC   */
	0x01, /*    0         1        0       AC_BK   */
	0x02, /*    0         0        0       AC_BK   */
	0x00, /*    0         0        1       AC_BE   */
	0x03, /*    0         1        1       AC_BE   */
	0x04, /*    1         0        0       AC_VI   */
	0x05, /*    1         0        1       AC_VI   */
	0x06, /*    1         1        0       AC_VO   */
	0x07 /*    1         1        1       AC_VO   */
};

/**
 * This table inverses the tos_to_tid operation to get a priority
 * which is in sequential order, and can be compared.
 * Use this to compare the priority of two different TIDs.
 */
t_u8 tos_to_tid_inv[] = {0x02, /* from tos_to_tid[2] = 0 */
			 0x00, /* from tos_to_tid[0] = 1 */
			 0x01, /* from tos_to_tid[1] = 2 */
			 0x03, 0x04, 0x05, 0x06, 0x07};

/**
 * This table will provide the tid value for given ac. This table does not
 * change and will be used to copy back the default values to tos_to_tid in
 * case of disconnect.
 */
t_u8 ac_to_tid[4][2] = {{1, 2}, {0, 3}, {4, 5}, {6, 7}};

/* Map of TOS UP values to WMM AC */
static const mlan_wmm_ac_e tos_to_ac[] = {WMM_AC_BE, WMM_AC_BK, WMM_AC_BK,
					  WMM_AC_BE, WMM_AC_VI, WMM_AC_VI,
					  WMM_AC_VO, WMM_AC_VO};

raListTbl *wlan_wmm_get_ralist_node(pmlan_private priv, t_u8 tid,
				    t_u8 *ra_addr);

/********************************************************
			Local Functions
********************************************************/
#ifdef DEBUG_LEVEL2
/**
 *  @brief Debug print function to display the priority parameters for a WMM AC
 *
 *  @param pac_param	Pointer to the AC parameters to display
 *
 *  @return		N/A
 */
static void
wlan_wmm_ac_debug_print(const IEEEtypes_WmmAcParameters_t *pac_param)
{
	const char *ac_str[] = {"BK", "BE", "VI", "VO"};

	ENTER();

	PRINTM(MINFO,
	       "WMM AC_%s: ACI=%d, ACM=%d, Aifsn=%d, "
	       "EcwMin=%d, EcwMax=%d, TxopLimit=%d\n",
	       ac_str[wmm_aci_to_qidx_map[pac_param->aci_aifsn.aci]],
	       pac_param->aci_aifsn.aci, pac_param->aci_aifsn.acm,
	       pac_param->aci_aifsn.aifsn, pac_param->ecw.ecw_min,
	       pac_param->ecw.ecw_max,
	       wlan_le16_to_cpu(pac_param->tx_op_limit));

	LEAVE();
}
/** Print the WMM AC for debug purpose */
#define PRINTM_AC(pac_param) wlan_wmm_ac_debug_print(pac_param)
#else
/** Print the WMM AC for debug purpose */
#define PRINTM_AC(pac_param)
#endif

/**
 *  @brief Allocate route address
 *
 *  @param pmadapter       Pointer to the mlan_adapter structure
 *  @param ra              Pointer to the route address
 *
 *  @return         ra_list
 */
static raListTbl *wlan_wmm_allocate_ralist_node(pmlan_adapter pmadapter,
						t_u8 *ra)
{
	raListTbl *ra_list = MNULL;

	ENTER();

	if (pmadapter->callbacks.moal_malloc(pmadapter->pmoal_handle,
					     sizeof(raListTbl), MLAN_MEM_DEF,
					     (t_u8 **)&ra_list)) {
		PRINTM(MERROR, "Fail to allocate ra_list\n");
		goto done;
	}
	util_init_list((pmlan_linked_list)ra_list);
	util_init_list_head((t_void *)pmadapter->pmoal_handle,
			    &ra_list->buf_head, MFALSE,
			    pmadapter->callbacks.moal_init_lock);

	memcpy_ext(pmadapter, ra_list->ra, ra, MLAN_MAC_ADDR_LENGTH,
		   MLAN_MAC_ADDR_LENGTH);

	ra_list->del_ba_count = 0;
	ra_list->total_pkts = 0;
	ra_list->tx_pause = 0;
	PRINTM(MINFO, "RAList: Allocating buffers for TID %p\n", ra_list);
done:
	LEAVE();
	return ra_list;
}

/**
 *  @brief Add packet to TDLS pending TX queue
 *
 *  @param priv		  A pointer to mlan_private
 *  @param pmbuf      Pointer to the mlan_buffer data struct
 *
 *  @return           N/A
 */
static t_void wlan_add_buf_tdls_txqueue(pmlan_private priv, pmlan_buffer pmbuf)
{
	mlan_adapter *pmadapter = priv->adapter;
	ENTER();
	util_enqueue_list_tail(pmadapter->pmoal_handle, &priv->tdls_pending_txq,
			       (pmlan_linked_list)pmbuf,
			       pmadapter->callbacks.moal_spin_lock,
			       pmadapter->callbacks.moal_spin_unlock);
	LEAVE();
}

/**
 *  @brief Clean up the tdls pending TX queue
 *
 *  @param priv		A pointer to mlan_private
 *
 *  @return      N/A
 */
static t_void wlan_cleanup_tdls_txq(pmlan_private priv)
{
	pmlan_buffer pmbuf;
	mlan_adapter *pmadapter = priv->adapter;
	ENTER();

	pmadapter->callbacks.moal_spin_lock(pmadapter->pmoal_handle,
					    priv->tdls_pending_txq.plock);
	while ((pmbuf = (pmlan_buffer)util_peek_list(pmadapter->pmoal_handle,
						     &priv->tdls_pending_txq,
						     MNULL, MNULL))) {
		util_unlink_list(pmadapter->pmoal_handle,
				 &priv->tdls_pending_txq,
				 (pmlan_linked_list)pmbuf, MNULL, MNULL);
		wlan_write_data_complete(pmadapter, pmbuf, MLAN_STATUS_FAILURE);
	}
	pmadapter->callbacks.moal_spin_unlock(pmadapter->pmoal_handle,
					      priv->tdls_pending_txq.plock);
	LEAVE();
}

/**
 * @brief Map ACs to TID
 *
 * @param priv             Pointer to the mlan_private driver data struct
 * @param queue_priority   Queue_priority structure
 *
 * @return 	   N/A
 */
static void wlan_wmm_queue_priorities_tid(pmlan_private priv,
					  t_u8 queue_priority[])
{
	int i;

	ENTER();

	for (i = 0; i < 4; ++i) {
		tos_to_tid[7 - (i * 2)] = ac_to_tid[queue_priority[i]][1];
		tos_to_tid[6 - (i * 2)] = ac_to_tid[queue_priority[i]][0];
	}

	for (i = 0; i < MAX_NUM_TID; i++)
		tos_to_tid_inv[tos_to_tid[i]] = (t_u8)i;

	/* in case priorities have changed, force highest priority so
	 * next packet will check from top to re-establish the highest
	 */
	util_scalar_write(priv->adapter->pmoal_handle,
			  &priv->wmm.highest_queued_prio, HIGH_PRIO_TID,
			  priv->adapter->callbacks.moal_spin_lock,
			  priv->adapter->callbacks.moal_spin_unlock);

	LEAVE();
}

/**
 *  @brief Evaluate whether or not an AC is to be downgraded
 *
 *  @param priv     Pointer to the mlan_private driver data struct
 *  @param eval_ac  AC to evaluate for downgrading
 *
 *  @return WMM AC  The eval_ac traffic is to be sent on.
 */
static mlan_wmm_ac_e wlan_wmm_eval_downgrade_ac(pmlan_private priv,
						mlan_wmm_ac_e eval_ac)
{
	int down_ac;
	mlan_wmm_ac_e ret_ac;
	WmmAcStatus_t *pac_status;

	ENTER();

	pac_status = &priv->wmm.ac_status[eval_ac];

	if (pac_status->disabled == MFALSE) {
		LEAVE();
		/* Okay to use this AC, its enabled */
		return eval_ac;
	}

	/* Setup a default return value of the lowest priority */
	ret_ac = WMM_AC_BK;

	/*
	 * Find the highest AC that is enabled and does not require admission
	 * control.  The spec disallows downgrading to an AC, which is enabled
	 * due to a completed admission control.  Unadmitted traffic is not
	 * to be sent on an AC with admitted traffic.
	 */
	for (down_ac = WMM_AC_BK; down_ac < eval_ac; down_ac++) {
		pac_status = &priv->wmm.ac_status[down_ac];

		if ((pac_status->disabled == MFALSE) &&
		    (pac_status->flow_required == MFALSE))
			/* AC is enabled and does not require admission control
			 */
			ret_ac = (mlan_wmm_ac_e)down_ac;
	}

	LEAVE();
	return ret_ac;
}

/**
 *  @brief Convert the IP TOS field to an WMM AC Queue assignment
 *
 *  @param pmadapter A pointer to mlan_adapter structure
 *  @param tos       IP TOS field
 *
 *  @return     WMM AC Queue mapping of the IP TOS field
 */
mlan_wmm_ac_e wlan_wmm_convert_tos_to_ac(pmlan_adapter pmadapter, t_u32 tos)
{
	ENTER();

	if (tos >= NELEMENTS(tos_to_ac)) {
		LEAVE();
		return WMM_AC_BE;
	}

	LEAVE();
	return tos_to_ac[tos];
}

/**
 *  @brief  Evaluate a given TID and downgrade it to a lower TID if the
 *          WMM Parameter IE received from the AP indicates that the AP
 *          is disabled (due to call admission control (ACM bit). Mapping
 *          of TID to AC is taken care internally
 *
 *  @param priv		Pointer to the mlan_private data struct
 *  @param tid      tid to evaluate for downgrading
 *
 *  @return       Same tid as input if downgrading not required or
 *                the tid the traffic for the given tid should be downgraded to
 */
static INLINE t_u8 wlan_wmm_downgrade_tid(pmlan_private priv, t_u32 tid)
{
	mlan_wmm_ac_e ac_down;
	pmlan_adapter pmadapter = priv->adapter;

	ENTER();

	ac_down = priv->wmm.ac_down_graded_vals[wlan_wmm_convert_tos_to_ac(
		pmadapter, tid)];
	LEAVE();
	/*
	 * Send the index to tid array, picking from the array will be
	 * taken care by dequeuing function
	 */
	if (tid == 1 || tid == 2)
		return ac_to_tid[ac_down][(tid + 1) % 2];
	else if (tid >= MAX_NUM_TID)
		return ac_to_tid[ac_down][0];
	else
		return ac_to_tid[ac_down][tid % 2];
}

/**
 *  @brief Delete packets in RA node
 *
 *  @param priv         Pointer to the mlan_private driver data struct
 *  @param ra_list      Pointer to raListTbl
 *
 *  @return             N/A
 */
static INLINE void wlan_wmm_del_pkts_in_ralist_node(pmlan_private priv,
						    raListTbl *ra_list)
{
	pmlan_buffer pmbuf;
	pmlan_adapter pmadapter = priv->adapter;

	ENTER();
	while ((pmbuf = (pmlan_buffer)util_peek_list(pmadapter->pmoal_handle,
						     &ra_list->buf_head, MNULL,
						     MNULL))) {
		util_unlink_list(pmadapter->pmoal_handle, &ra_list->buf_head,
				 (pmlan_linked_list)pmbuf, MNULL, MNULL);
		wlan_write_data_complete(pmadapter, pmbuf, MLAN_STATUS_FAILURE);
	}
	util_free_list_head((t_void *)pmadapter->pmoal_handle,
			    &ra_list->buf_head,
			    pmadapter->callbacks.moal_free_lock);

	LEAVE();
}

/**
 *  @brief Delete packets in RA list
 *
 *  @param priv			Pointer to the mlan_private driver data struct
 *  @param ra_list_head	ra list header
 *
 *  @return		N/A
 */
static INLINE void wlan_wmm_del_pkts_in_ralist(pmlan_private priv,
					       mlan_list_head *ra_list_head)
{
	raListTbl *ra_list;

	ENTER();

	ra_list = (raListTbl *)util_peek_list(priv->adapter->pmoal_handle,
					      ra_list_head, MNULL, MNULL);

	while (ra_list && ra_list != (raListTbl *)ra_list_head) {
		wlan_wmm_del_pkts_in_ralist_node(priv, ra_list);

		ra_list = ra_list->pnext;
	}

	LEAVE();
}

/**
 *  @brief Clean up the wmm queue
 *
 *  @param priv  Pointer to the mlan_private driver data struct
 *
 *  @return      N/A
 */
static void wlan_wmm_cleanup_queues(pmlan_private priv)
{
	int i;

	ENTER();

	for (i = 0; i < MAX_NUM_TID; i++) {
		wlan_wmm_del_pkts_in_ralist(priv,
					    &priv->wmm.tid_tbl_ptr[i].ra_list);
		priv->wmm.pkts_queued[i] = 0;
		priv->wmm.pkts_paused[i] = 0;
	}
	util_scalar_write(priv->adapter->pmoal_handle,
			  &priv->wmm.tx_pkts_queued, 0, MNULL, MNULL);
	util_scalar_write(priv->adapter->pmoal_handle,
			  &priv->wmm.highest_queued_prio, HIGH_PRIO_TID, MNULL,
			  MNULL);

	LEAVE();
}

/**
 *  @brief Delete all wmm_sta_table
 *
 *  @param priv     Pointer to the mlan_private driver data struct
 *
 *  @return         N/A
 */
static void wlan_wmm_delete_all_sta_entries(pmlan_private priv)
{
	pmlan_adapter pmadapter = priv->adapter;
	t_void *const pmoal_handle = pmadapter->pmoal_handle;
	mlan_linked_list *sta_entry;

	while ((sta_entry =
			util_peek_list_nl(pmoal_handle, &priv->wmm.all_stas))) {
		struct wmm_sta_table *sta = util_container_of(
			sta_entry, struct wmm_sta_table, all_stas_entry);

		util_unlink_list_nl(pmoal_handle, &sta->all_stas_entry);
		util_unlink_list_safe_nl(pmoal_handle,
					 &sta->pending_stas_entry);
		util_unlink_list_safe_nl(pmoal_handle, &sta->active_sta_entry);

		pmadapter->callbacks.moal_mfree(pmoal_handle, (t_u8 *)sta);
	}

	util_list_head_reset(&priv->wmm.all_stas);

	priv->wmm.selected_ra_list = MNULL;
	pmadapter->ra_list_tracing.ra_list = MNULL;
}

/**
 *  @brief Delete all route address from RA list
 *
 *  @param priv     Pointer to the mlan_private driver data struct
 *
 *  @return         N/A
 */
static void wlan_wmm_delete_all_ralist(pmlan_private priv)
{
	raListTbl *ra_list;
	int i;
	pmlan_adapter pmadapter = priv->adapter;
	t_void *const pmoal_handle = pmadapter->pmoal_handle;

	ENTER();

	for (i = 0; i < MAX_NUM_TID; ++i) {
		PRINTM(MINFO, "RAList: Freeing buffers for TID %d\n", i);
		while ((ra_list = (raListTbl *)util_peek_list(
				pmoal_handle, &priv->wmm.tid_tbl_ptr[i].ra_list,
				MNULL, MNULL))) {
			util_unlink_list(pmoal_handle,
					 &priv->wmm.tid_tbl_ptr[i].ra_list,
					 (pmlan_linked_list)ra_list, MNULL,
					 MNULL);

			if (util_is_node_in_list(&ra_list->pending_txq_entry))
				util_unlink_list_nl(
					pmoal_handle,
					&ra_list->pending_txq_entry);

			pmadapter->callbacks.moal_mfree(pmoal_handle,
							(t_u8 *)ra_list);
		}

		util_list_head_reset(&priv->wmm.tid_tbl_ptr[i].ra_list);
		priv->wmm.tid_tbl_ptr[i].ra_list_curr = MNULL;
	}

	LEAVE();
}

/**
 *   @brief Get queue RA pointer
 *
 *   @param priv     Pointer to the mlan_private driver data struct
 *   @param tid      TID
 *   @param ra_addr  Pointer to the route address
 *
 *   @return         ra_list
 */
static raListTbl *wlan_wmm_get_queue_raptr(pmlan_private priv, t_u8 tid,
					   t_u8 *ra_addr)
{
	raListTbl *ra_list;
#ifdef UAP_SUPPORT
	t_u8 bcast_addr[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
#endif

	ENTER();
	ra_list = wlan_wmm_get_ralist_node(priv, tid, ra_addr);
	if (ra_list) {
		LEAVE();
		return ra_list;
	}
#ifdef UAP_SUPPORT
	if ((GET_BSS_ROLE(priv) == MLAN_BSS_ROLE_UAP) &&
	    (0 !=
	     memcmp(priv->adapter, ra_addr, bcast_addr, sizeof(bcast_addr)))) {
		if (MNULL == wlan_get_station_entry(priv, ra_addr)) {
			PRINTM_NETINTF(MERROR, priv);
			PRINTM(MERROR,
			       "Drop packets to unknow station " MACSTR "\n",
			       MAC2STR(ra_addr));
			LEAVE();
			return MNULL;
		}
	}
#endif
	wlan_ralist_add(priv, ra_addr);

	ra_list = wlan_wmm_get_ralist_node(priv, tid, ra_addr);
	LEAVE();
	return ra_list;
}

#ifdef STA_SUPPORT
/**
 *  @brief Sends wmmac host event
 *
 *  @param priv      Pointer to the mlan_private driver data struct
 *  @param type_str  Type of host event
 *  @param src_addr  Pointer to the source Address
 *  @param tid       TID
 *  @param up        User priority
 *  @param status    Status code or Reason code
 *
 *  @return     N/A
 */
static void wlan_send_wmmac_host_event(pmlan_private priv, char *type_str,
				       t_u8 *src_addr, t_u8 tid, t_u8 up,
				       t_u8 status)
{
	t_u8 event_buf[100];
	mlan_event *pevent;
	t_u8 *pout_buf;

	ENTER();

	/* Format one of the following two output strings:
	**    - TSPEC:ADDTS_RSP:[<status code>]:TID=X:UP=Y
	**    - TSPEC:DELTS_RX:[<reason code>]:TID=X:UP=Y
	*/
	pevent = (mlan_event *)event_buf;
	pout_buf = pevent->event_buf;

	memcpy_ext(priv->adapter, pout_buf, (t_u8 *)"TSPEC:", 6, 6);
	pout_buf += 6;

	memcpy_ext(priv->adapter, pout_buf, (t_u8 *)type_str,
		   wlan_strlen(type_str), wlan_strlen(type_str));
	pout_buf += wlan_strlen(type_str);

	*pout_buf++ = ':';
	*pout_buf++ = '[';

	if (status >= 100) {
		*pout_buf++ = (status / 100) + '0';
		status = (status % 100);
	}

	if (status >= 10) {
		*pout_buf++ = (status / 10) + '0';
		status = (status % 10);
	}

	*pout_buf++ = status + '0';

	memcpy_ext(priv->adapter, pout_buf, (t_u8 *)"]:TID", 5, 5);
	pout_buf += 5;
	*pout_buf++ = tid + '0';

	memcpy_ext(priv->adapter, pout_buf, (t_u8 *)":UP", 3, 3);
	pout_buf += 3;
	*pout_buf++ = up + '0';

	*pout_buf = '\0';

	pevent->bss_index = priv->bss_index;
	pevent->event_id = MLAN_EVENT_ID_DRV_REPORT_STRING;
	pevent->event_len = wlan_strlen((const char *)(pevent->event_buf));

	wlan_recv_event(priv, MLAN_EVENT_ID_DRV_REPORT_STRING, pevent);
	LEAVE();
}
#endif /* STA_SUPPORT */

/**
 *  @brief This function gets the highest priority list pointer
 *
 *  @param pmadapter      A pointer to mlan_adapter
 *  @param priv           A pointer to mlan_private
 *  @param tid            A pointer to return tid
 *
 *  @return             raListTbl
 */
static raListTbl *wlan_wmm_get_highest_priolist_ptr(pmlan_adapter pmadapter,
						    pmlan_private *priv,
						    int *tid)
{
	pmlan_private priv_tmp;
	raListTbl *ptr, *head;
	mlan_bssprio_node *bssprio_node, *bssprio_head;
	tid_tbl_t *tid_ptr;
	int i, j;
	int next_prio = 0;
	t_u8 next_tid = 0;
	ENTER();

	PRINTM(MDAT_D, "POP\n");
	for (j = pmadapter->priv_num - 1; j >= 0; --j) {
		if (!(util_peek_list(pmadapter->pmoal_handle,
				     &pmadapter->bssprio_tbl[j].bssprio_head,
				     MNULL, MNULL)))
			continue;

		if (pmadapter->bssprio_tbl[j].bssprio_cur ==
		    (mlan_bssprio_node *)&pmadapter->bssprio_tbl[j]
			    .bssprio_head) {
			pmadapter->bssprio_tbl[j].bssprio_cur =
				pmadapter->bssprio_tbl[j].bssprio_cur->pnext;
		}

		bssprio_head = bssprio_node =
			pmadapter->bssprio_tbl[j].bssprio_cur;

		do {
			priv_tmp = bssprio_node->priv;
			if ((priv_tmp->port_ctrl_mode == MTRUE) &&
			    (priv_tmp->port_open == MFALSE)) {
				PRINTM(MINFO,
				       "get_highest_prio_ptr(): "
				       "PORT_CLOSED Ignore pkts from BSS%d\n",
				       priv_tmp->bss_index);
				/* Ignore data pkts from a BSS if port is closed
				 */
				goto next_intf;
			}
			if (priv_tmp->tx_pause == MTRUE) {
				PRINTM(MINFO,
				       "get_highest_prio_ptr(): "
				       "TX PASUE Ignore pkts from BSS%d\n",
				       priv_tmp->bss_index);
				/* Ignore data pkts from a BSS if tx pause */
				goto next_intf;
			}
#if defined(USB)
			if (!wlan_is_port_ready(pmadapter,
						priv_tmp->port_index)) {
				PRINTM(MINFO,
				       "get_highest_prio_ptr(): "
				       "usb port is busy,Ignore pkts from BSS%d\n",
				       priv_tmp->bss_index);
				/* Ignore data pkts from a BSS if usb port is
				 * busy */
				goto next_intf;
			}
#endif

			pmadapter->callbacks.moal_spin_lock(
				pmadapter->pmoal_handle,
				priv_tmp->wmm.ra_list_spinlock);

			for (i = util_scalar_read(
				     pmadapter->pmoal_handle,
				     &priv_tmp->wmm.highest_queued_prio, MNULL,
				     MNULL);
			     i >= LOW_PRIO_TID; --i) {
				tid_ptr = &(priv_tmp)
						   ->wmm
						   .tid_tbl_ptr[tos_to_tid[i]];
				if (!util_peek_list(pmadapter->pmoal_handle,
						    &tid_ptr->ra_list, MNULL,
						    MNULL))
					continue;

				/*
				 * Always choose the next ra we transmitted
				 * last time, this way we pick the ra's in
				 * round robin fashion.
				 */
				head = ptr = tid_ptr->ra_list_curr->pnext;
				if (ptr == (raListTbl *)&tid_ptr->ra_list)
					head = ptr = ptr->pnext;

				do {
					if (!ptr->tx_pause &&
					    util_peek_list(
						    pmadapter->pmoal_handle,
						    &ptr->buf_head, MNULL,
						    MNULL)) {
						/* Because WMM only support
						 * BK/BE/VI/VO, we have 8 tid
						 * We should balance the traffic
						 * of the same AC */
						if (i % 2)
							next_prio = i - 1;
						else
							next_prio = i + 1;
						next_prio = (next_prio < 1) ?
								    0 :
								    next_prio;
						next_tid = MIN(
							tos_to_tid[next_prio],
							MAX_NUM_TID);
						if (priv_tmp->wmm.pkts_queued
							    [next_tid] &&
						    (priv_tmp->wmm.pkts_queued
							     [next_tid] >
						     priv_tmp->wmm.pkts_paused
							     [next_tid]))
							util_scalar_write(
								pmadapter->pmoal_handle,
								&priv_tmp->wmm
									 .highest_queued_prio,
								next_prio,
								MNULL, MNULL);
						else
							/* if
							 * highest_queued_prio >
							 * i, set it to i */
							util_scalar_conditional_write(
								pmadapter->pmoal_handle,
								&priv_tmp->wmm
									 .highest_queued_prio,
								MLAN_SCALAR_COND_GREATER_THAN,
								i, i, MNULL,
								MNULL);
						*priv = priv_tmp;
						*tid = tos_to_tid[i];
						/* hold priv->ra_list_spinlock
						 * to maintain ptr */
						PRINTM(MDAT_D,
						       "get highest prio ptr %p, tid %d\n",
						       ptr, *tid);
						LEAVE();
						return ptr;
					}

					ptr = ptr->pnext;
					if (ptr ==
					    (raListTbl *)&tid_ptr->ra_list)
						ptr = ptr->pnext;
				} while (ptr != head);
			}

			/* If priv still has packets queued, reset to
			 * HIGH_PRIO_TID */
			if (util_scalar_read(pmadapter->pmoal_handle,
					     &priv_tmp->wmm.tx_pkts_queued,
					     MNULL, MNULL))
				util_scalar_write(
					pmadapter->pmoal_handle,
					&priv_tmp->wmm.highest_queued_prio,
					HIGH_PRIO_TID, MNULL, MNULL);
			else
				/* No packet at any TID for this priv.  Mark as
				 * such to skip checking TIDs for this priv
				 * (until pkt is added). */
				util_scalar_write(
					pmadapter->pmoal_handle,
					&priv_tmp->wmm.highest_queued_prio,
					NO_PKT_PRIO_TID, MNULL, MNULL);

			pmadapter->callbacks.moal_spin_unlock(
				pmadapter->pmoal_handle,
				priv_tmp->wmm.ra_list_spinlock);

		next_intf:
			bssprio_node = bssprio_node->pnext;
			if (bssprio_node ==
			    (mlan_bssprio_node *)&pmadapter->bssprio_tbl[j]
				    .bssprio_head)
				bssprio_node = bssprio_node->pnext;
			pmadapter->bssprio_tbl[j].bssprio_cur = bssprio_node;
		} while (bssprio_node != bssprio_head);
	}

	LEAVE();
	return MNULL;
}

/**
 *  @brief Calculates byte budget based on time budget and currect PHY rate
 *
 *  @param pmadapter        Pointer to the mlan_adapter structure
 *  @param time_budget_us   Time budget in usec
 *  @param phy_rate_kbps    TX PHY rate in kbit/sec
 *
 *  @return                 byte budget
 */
static t_u32 wlan_wmm_get_byte_budget(pmlan_adapter pmadapter,
				      t_u32 time_budget_us, t_u32 phy_rate_kbps)
{
	const t_u32 min_budget = MV_ETH_FRAME_LEN;
	t_u64 byte_budget = pmadapter->callbacks.moal_do_div(
		(t_u64)phy_rate_kbps * time_budget_us, 8 * 1000u);

	if (byte_budget > INT_MAX)
		return INT_MAX;

	if (byte_budget < min_budget)
		return min_budget;

	return byte_budget;
}

/**
 *  @brief Allocate sta_table address
 *
 *  @param pmadapter       Pointer to the mlan_adapter structure
 *  @param ra              Pointer to the route address
 *
 *  @return         sta_table
 */
static struct wmm_sta_table *
wlan_wmm_allocate_sta_table(pmlan_adapter pmadapter, t_u8 *ra)
{
	struct wmm_sta_table *sta_table = MNULL;
	int i;
	const t_u32 default_rate = 200 * 1000;
	const t_u32 default_queue_packets = 1024;

	ENTER();

	if (!pmadapter->mclient_tx_supported)
		goto done;

	if (pmadapter->callbacks.moal_malloc(pmadapter->pmoal_handle,
					     sizeof(*sta_table), MLAN_MEM_DEF,
					     (t_u8 **)&sta_table)) {
		PRINTM(MERROR, "Fail to allocate sta_table\n");
		goto done;
	}

	util_init_list(&sta_table->all_stas_entry);
	util_init_list(&sta_table->pending_stas_entry);
	util_init_list(&sta_table->active_sta_entry);
	memcpy_ext(pmadapter, sta_table->ra, ra, MLAN_MAC_ADDR_LENGTH,
		   MLAN_MAC_ADDR_LENGTH);

	sta_table->budget.time_budget_init_us = pmadapter->init_para.tx_budget;
	sta_table->budget.byte_budget_init = wlan_wmm_get_byte_budget(
		pmadapter, sta_table->budget.time_budget_init_us, default_rate);
	sta_table->budget.queue_packets = default_queue_packets;
	sta_table->budget.phy_rate_kbps = default_rate;

	sta_table->budget.mpdu_with_amsdu_pps_cap =
		pmadapter->tx_mpdu_with_amsdu_pps;
	sta_table->budget.mpdu_no_amsdu_pps_cap =
		pmadapter->tx_mpdu_no_amsdu_pps;

	sta_table->budget.mpdu_with_amsdu_budget_init =
		pmadapter->callbacks.moal_do_div(
			(t_u64)sta_table->budget.mpdu_with_amsdu_pps_cap *
				sta_table->budget.time_budget_init_us,
			1000000);
	sta_table->budget.mpdu_no_amsdu_budget_init =
		pmadapter->callbacks.moal_do_div(
			(t_u64)sta_table->budget.mpdu_no_amsdu_pps_cap *
				sta_table->budget.time_budget_init_us,
			1000000);

	for (i = 0; i < NELEMENTS(sta_table->budget.bytes); ++i) {
		sta_table->budget.bytes[i] = sta_table->budget.byte_budget_init;
		sta_table->budget.mpdus[i] =
			sta_table->budget.mpdu_with_amsdu_budget_init;
	}

done:
	LEAVE();

	return sta_table;
}

/**
 *  @brief Generates pseudorandom number
 *
 *  @return         random number
 */
static t_u32 wmm_get_random_num(void)
{
	static t_u32 state = 2463534242u;

	state ^= state << 13;
	state ^= state >> 17;
	state ^= state << 5;

	return state;
}

/**
 *  @brief reset remaning AIFS to default value
 *
 *  @param txq_cont   Pointer to mlan_wmm_contention structure
 *
 *  @return           N/A
 */
static void wlan_wmm_txq_contention_reset_aifs(mlan_wmm_contention *txq_cont)
{
	txq_cont->remaining_aifs = txq_cont->param.aifsn * 9 + 16;
}

/**
 *  @brief reset remaning backoff to default value
 *
 *  @param txq_cont   Pointer to mlan_wmm_contention structure
 *
 *  @return           N/A
 */
static void wlan_wmm_txq_contention_reset_backoff(mlan_wmm_contention *txq_cont)
{
	const t_u32 mask = (1u << txq_cont->ecw) - 1;
	txq_cont->remaining_backoff = (wmm_get_random_num() & mask) * 9;
}

/**
 *  @brief get remaning time till contention end in usec
 *
 *  @param txq_cont   Pointer to mlan_wmm_contention structure
 *
 *  @return           remaning time
 */
static t_u32 wlan_wmm_txq_get_remaining_time(mlan_wmm_contention *txq_cont)
{
	return txq_cont->remaining_aifs + txq_cont->remaining_backoff;
}

/**
 *  @brief decrement contention timer by specified duration
 *
 *  @param txq_cont   Pointer to mlan_wmm_contention structure
 *  @param duration   duration in usec
 *
 *  @return           MTRUE if timer reach 0
 */
static t_bool wlan_wmm_txq_count_donw(mlan_wmm_contention *txq_cont,
				      t_u32 duration)
{
	if (txq_cont->remaining_aifs > 0) {
		t_u32 del = MIN(txq_cont->remaining_aifs, duration);

		txq_cont->remaining_aifs -= del;
		duration -= del;
	}

	if (txq_cont->remaining_aifs == 0)
		txq_cont->remaining_backoff -= duration;

	// coverity[integer_overflow:SUPPRESS]
	return txq_cont->remaining_backoff == 0 &&
	       txq_cont->remaining_aifs == 0;
}

/**
 *  @brief This function is called when txq_cont win contention
 *
 *  @param txq_cont   Pointer to mlan_wmm_contention structure
 *
 *  @return           N/A
 */
static void wlan_wmm_txq_win(mlan_wmm_contention *txq_cont)
{
	txq_cont->ecw = txq_cont->param.ecwmin;
	wlan_wmm_txq_contention_reset_aifs(txq_cont);
	wlan_wmm_txq_contention_reset_backoff(txq_cont);
}

/**
 *  @brief This function is called when txq_cont lost contention
 *
 *  @param txq_cont   Pointer to mlan_wmm_contention structure
 *
 *  @return           N/A
 */
static void wlan_wmm_txq_lost(mlan_wmm_contention *txq_cont)
{
	wlan_wmm_txq_contention_reset_aifs(txq_cont);
	if (txq_cont->move_cw_on_lost) {
		txq_cont->ecw = MIN(txq_cont->ecw + 1, txq_cont->param.ecwmax);
		wlan_wmm_txq_contention_reset_backoff(txq_cont);
	}
}

/**
 *  @brief This function is called when collision detected
 *
 *  @param txq_cont   Pointer to mlan_wmm_contention structure
 *
 *  @return           N/A
 */
static void wlan_wmm_txq_collision(mlan_wmm_contention *txq_cont)
{
	txq_cont->ecw = MIN(txq_cont->ecw + 1, txq_cont->param.ecwmax);
	wlan_wmm_txq_contention_reset_aifs(txq_cont);
	wlan_wmm_txq_contention_reset_backoff(txq_cont);
}

/**
 *  @brief Setup contention parameters based on WMM structure
 *
 *  @param txq_cont   Pointer to mlan_wmm_contention structure
 *  @param wmm_param   Pointer to IEEEtypes_WmmAcParameters_t structure
 *  @param move_cw_on_lost   tells if CW should be doubled in case of lose
 *                             in contention
 *
 *  @return           N/A
 */
static void
wlan_wmm_txq_contention_setup(mlan_wmm_contention *txq_cont,
			      t_bool move_cw_on_lost,
			      const IEEEtypes_WmmAcParameters_t *wmm_param)
{
	txq_cont->param.aifsn = wmm_param->aci_aifsn.aifsn;
	txq_cont->param.ecwmin = wmm_param->ecw.ecw_min;
	txq_cont->param.ecwmax = wmm_param->ecw.ecw_max;
	txq_cont->ecw = txq_cont->param.ecwmin;
	txq_cont->move_cw_on_lost = move_cw_on_lost;
	wlan_wmm_txq_contention_reset_aifs(txq_cont);
	wlan_wmm_txq_contention_reset_backoff(txq_cont);
}

/**
 *  @brief Get WMM contention winner
 *
 *  @param pmadapter   Pointer to pmlan_adapter structure
 *  @param mlan        Pointer to mlan_private structure
 *
 *  @return           TX queue index, -1 if no pending data
 */
static int wlan_wmm_contention_get_winner(pmlan_adapter pmadapter,
					  mlan_private *mlan)
{
	int queue;
	t_u32 active_queues = 0;
	t_u32 remaining_time = INT_MAX;
	t_bool has_winner = MFALSE;
	int winner_queue = -1;

	for (queue = 0; queue < NELEMENTS(mlan->wmm.pending_txq); ++queue) {
		pmlan_linked_list entry = util_peek_list_nl(
			pmadapter->pmoal_handle, &mlan->wmm.pending_txq[queue]);

		if (entry) {
			t_u32 q_remaining_time =
				wlan_wmm_txq_get_remaining_time(
					&mlan->wmm.txq_contention[queue]);
			remaining_time = MIN(remaining_time, q_remaining_time);
			active_queues |= MBIT(queue);
		}
	}

	for (queue = 0; queue < NELEMENTS(mlan->wmm.pending_txq); ++queue) {
		if (active_queues & MBIT(queue)) {
			mlan_wmm_contention *txq_cont =
				&mlan->wmm.txq_contention[queue];
			t_bool winner = wlan_wmm_txq_count_donw(txq_cont,
								remaining_time);

			if (winner && !has_winner) {
				has_winner = MTRUE;
				winner_queue = queue;
				wlan_wmm_txq_win(txq_cont);
			} else if (winner && has_winner) {
				wlan_wmm_txq_collision(txq_cont);
			} else {
				wlan_wmm_txq_lost(txq_cont);
			}
		}
	}

	return winner_queue;
}

/**
 *  @brief Initi contention parameters
 *
 *  @param mlan        Pointer to mlan_private structure
 *  @param ac_params   Pointer to IEEEtypes_WmmAcParameters_t array
 *
 *  @return           N/A
 */
void wlan_wmm_contention_init(
	mlan_private *mlan,
	const IEEEtypes_WmmAcParameters_t ac_params[MAX_AC_QUEUES])
{
	int queue;

	if (ac_params == MNULL)
		return;

	for (queue = 0; queue < NELEMENTS(mlan->wmm.queue_priority); ++queue) {
		const mlan_wmm_ac_e ac = mlan->wmm.queue_priority[queue];
		const wmm_ac_e aci = wmm_ac_to_aci_map[ac];
		/* adjust BK`s contention logic to mimic legacy behaviour */
		const t_bool move_cw_on_lost = aci == AC_BK;

		wlan_wmm_txq_contention_setup(&mlan->wmm.txq_contention[queue],
					      move_cw_on_lost, &ac_params[aci]);
	}
}

/**
 *  @brief get AC queue index for transmission
 *
 *  @param pmadapter   Pointer to pmlan_adapter structure
 *  @param mlan        Pointer to mlan_private structure
 *
 *  @return           AC queue index to TX
 */
static int wlan_wmm_get_next_ac(pmlan_adapter pmadapter, mlan_private *mlan)
{
	return wlan_wmm_contention_get_winner(pmadapter, mlan);
}

/**
 *  @brief checks if BSS is ready for TX
 *
 *  @param pmadapter   Pointer to pmlan_adapter structure
 *  @param mlan        Pointer to mlan_private structure
 *
 *  @return           MTRUE when BSS is reasy for TX
 */
static t_bool wlan_wmm_is_bss_ready_for_tx(pmlan_adapter pmadapter,
					   mlan_private *mlan)
{
	if ((mlan->port_ctrl_mode == MTRUE) && (mlan->port_open == MFALSE)) {
		PRINTM(MINFO, "PORT_CLOSED Ignore pkts from BSS%d\n",
		       mlan->bss_index);
		return MFALSE;
	}

	if (mlan->tx_pause) {
		PRINTM(MINFO, "TX PASUE Ignore pkts from BSS%d\n",
		       mlan->bss_index);
		return MFALSE;
	}
#if defined(USB)
	if (!wlan_is_port_ready(pmadapter, mlan->port_index)) {
		PRINTM(MINFO, "usb port is busy,Ignore pkts from BSS%d\n",
		       mlan->bss_index);
		return MFALSE;
	}
#endif

	return MTRUE;
}

static mlan_private *wlan_wmm_get_next_mlan(pmlan_adapter pmadapter,
					    mlan_bssprio_tbl *bssprio_tbl)
{
	mlan_bssprio_node *bss = bssprio_tbl->bssprio_cur;
	const mlan_callbacks *const cb = &pmadapter->callbacks;
	t_void *pmoal = pmadapter->pmoal_handle;
	mlan_bssprio_node *const stop_point = bss;

	do {
		mlan_private *mlan;
		t_u32 queue;

		bssprio_tbl->bssprio_cur = bss;
		if (bss == (mlan_bssprio_node *)&bssprio_tbl->bssprio_head)
			goto next_bss;

		mlan = bss->priv;

		if (!wlan_wmm_is_bss_ready_for_tx(pmadapter, mlan)) {
			goto next_bss;
		}

		cb->moal_spin_lock(pmoal, mlan->wmm.ra_list_spinlock);

		for (queue = 0; queue < NELEMENTS(mlan->wmm.pending_txq);
		     ++queue) {
			if (util_peek_list_nl(pmoal,
					      &mlan->wmm.pending_txq[queue])) {
				bssprio_tbl->bssprio_cur =
					bssprio_tbl->bssprio_cur->pnext;
				return mlan;
			}
		}

		cb->moal_spin_unlock(pmoal, mlan->wmm.ra_list_spinlock);

	next_bss:
		bss = bss->pnext;
	} while (bss != stop_point);

	return MNULL;
}

/**
 *  @brief gets next BSS to TX
 *
 *  @param pmadapter   Pointer to pmlan_adapter structure
 *
 *  @return           mlan_private
 */
static mlan_private *wlan_wmm_get_next_bss(pmlan_adapter pmadapter)
{
	int i;
	t_void *pmoal = pmadapter->pmoal_handle;
	mlan_private *mlan = MNULL;
	ENTER();

	for (i = pmadapter->priv_num - 1; i >= 0 && mlan == MNULL; --i) {
		mlan_bssprio_tbl *bssprio_tbl = &pmadapter->bssprio_tbl[i];

		if (!util_peek_list_nl(pmoal, &bssprio_tbl->bssprio_head))
			continue;

		mlan = wlan_wmm_get_next_mlan(pmadapter, bssprio_tbl);
	}

	LEAVE();

	return mlan;
}

/**
 *  @brief helper function to refill budget
 *
 *  @param current_value   current budget value
 *  @param init_value      Addition to bugget
 *
 *  @return            new budget value
 */
static t_s32 wlan_wmm_refill_budget(t_s32 current_value, t_u32 init_value)
{
	if (current_value > 0)
		return init_value;

	// coverity[integer_overflow:SUPPRESS]
	return current_value + init_value;
}

/**
 *  @brief checks if ra_list has budget to be scheduled for TX
 *
 *  @param pmadapter   Pointer to pmlan_adapter structure
 *  @param mlan        Pointer to mlan_private structure
 *  @param ra_list     Pointer to ra_list structure
 *
 *  @return           MTRUE when ra_list can be scheduled
 */
static t_bool wlan_wmm_process_ra_list_quoats(pmlan_adapter pmadapter,
					      mlan_private *mlan,
					      raListTbl *ra_list)
{
	struct wmm_sta_table *sta = ra_list->sta;
	const t_u8 tid = ra_list->tid;
	t_bool ready = MTRUE;

	if (sta->budget.bytes[tid] <= 0 || sta->budget.mpdus[tid] <= 0) {
		const t_u32 mpdu_budget_init =
			ra_list->amsdu_in_ampdu ?
				sta->budget.mpdu_with_amsdu_budget_init :
				sta->budget.mpdu_no_amsdu_budget_init;
		util_unlink_list_nl(pmadapter->pmoal_handle,
				    &ra_list->pending_txq_entry);

		util_enqueue_list_tail_nl(
			pmadapter->pmoal_handle,
			&mlan->wmm.pending_txq[ra_list->queue],
			&ra_list->pending_txq_entry);

		sta->budget.bytes[tid] = wlan_wmm_refill_budget(
			sta->budget.bytes[tid], sta->budget.byte_budget_init);
		sta->budget.mpdus[tid] = wlan_wmm_refill_budget(
			sta->budget.mpdus[tid], mpdu_budget_init);

		mlan->wmm.selected_ra_list = MNULL;

		ready = MFALSE;
	} else {
		mlan->wmm.selected_ra_list = ra_list;
	}

	return ready;
}

/**
 *  @brief gets next ra_list to TX
 *
 *  @param pmadapter   Pointer to pmlan_adapter structure
 *  @param mlan        Pointer to mlan_private structure
 *
 *  @return           raListTbl
 */
static raListTbl *wlan_wmm_get_next_ra_list(pmlan_adapter pmadapter,
					    mlan_private *mlan)
{
	int ac;
	t_void *pmoal = pmadapter->pmoal_handle;

	while ((ac = wlan_wmm_get_next_ac(pmadapter, mlan)) >= 0) {
		pmlan_linked_list entry;

		while ((entry = util_peek_list_nl(
				pmoal, &mlan->wmm.pending_txq[ac])) != MNULL) {
			raListTbl *ra_list;
			t_bool has_data;

			ra_list = util_container_of(entry, raListTbl,
						    pending_txq_entry);
			has_data = ra_list->total_pkts &&
				   util_peek_list_nl(pmoal, &ra_list->buf_head);

			/* ra_list can be empry since we re-queue it once we hit
			 * budget limit */
			if (!has_data) {
				struct wmm_sta_table *sta = ra_list->sta;
				const t_u32 mpdu_budget_init =
					ra_list->amsdu_in_ampdu ?
						sta->budget
							.mpdu_with_amsdu_budget_init :
						sta->budget
							.mpdu_no_amsdu_budget_init;

				sta->budget.bytes[ra_list->tid] =
					sta->budget.byte_budget_init;
				sta->budget.mpdus[ra_list->tid] =
					mpdu_budget_init;

				util_unlink_list_nl(
					pmoal, &ra_list->pending_txq_entry);
				continue;
			}

			if (wlan_wmm_process_ra_list_quoats(pmadapter, mlan,
							    ra_list)) {
				return ra_list;
			}
		}
	}

	return MNULL;
}

/**
 *  @brief Track byte budget consumed by ra_list
 *
 *  @param ra_list   Pointer to raListTbl structure
 *  @param pmbuf     Pointer to mlan_buffer structure
 *
 *  @return          N/A
 */
void wlan_wmm_consume_byte_budget(raListTbl *ra_list, mlan_buffer *pmbuf)
{
	struct wmm_sta_table *sta = ra_list->sta;

	if (sta != MNULL) {
		t_u32 data_len = pmbuf->data_len;

		if (ra_list->ba_status != BA_STREAM_SETUP_COMPLETE) {
			/*
			 * Assumption that non-agg consumes 2 times more
			 * airtime. It is not always true but we don't have
			 * airtime feedback from FW anyway.
			 */
			data_len *= 2;
		}

		sta->budget.bytes[ra_list->tid] -= data_len;
	}
}

/**
 *  @brief Track MPDU budget consumed by ra_list
 *
 *  @param ra_list   Pointer to raListTbl structure
 *  @param pmbuf     Pointer to mlan_buffer structure
 *
 *  @return          N/A
 */
void wlan_wmm_consume_mpdu_budget(raListTbl *ra_list)
{
	struct wmm_sta_table *sta = ra_list->sta;

	if (sta != MNULL)
		sta->budget.mpdus[ra_list->tid]--;
}

/**
 *  @brief Debug function to track ra_list switching during scheduling
 *
 *  @param mlan      Pointer to mlan_private structure
 *  @param ra_list   Pointer to raListTbl structure
 *
 *  @return          N/A
 */
static void wlan_wmm_track_ra_list_switch(mlan_private *mlan,
					  raListTbl *ra_list)
{
	const int use_runtime_log = 1;
	mlan_adapter *adapter = mlan->adapter;

	if (ra_list != adapter->ra_list_tracing.ra_list) {
		raListTbl *old_list = adapter->ra_list_tracing.ra_list;
		t_u32 in_tx_ring = 0;
#ifdef PCIE
		if (IS_PCIE(mlan->adapter->card_type))
			in_tx_ring = mlan->adapter->pcard_pcie->txbd_pending;
#endif
		if (use_runtime_log && old_list != MNULL) {
			struct wmm_sta_table *old_sta = old_list->sta;

			PRINTM(MSCH_D,
			       "mclient: switch[ %u/%u : q size: %u/%u bb: r %d m %d ri %u mi %u %u p %u] %pM tid %u -> %pM tid %u",
			       adapter->ra_list_tracing.pushed_pkg, in_tx_ring,
			       old_list->total_pkts,
			       old_sta->budget.queue_packets,
			       old_sta->budget.bytes[old_list->tid],
			       old_sta->budget.mpdus[old_list->tid],
			       old_sta->budget.byte_budget_init,
			       old_sta->budget.mpdu_no_amsdu_budget_init,
			       old_sta->budget.mpdu_with_amsdu_budget_init,
			       old_sta->budget.phy_rate_kbps / 1000,
			       old_list->ra, old_list->tid, ra_list->ra,
			       ra_list->tid);
		}

		adapter->ra_list_tracing.ra_list = ra_list;
		adapter->ra_list_tracing.pushed_pkg = 0;
	}

	adapter->ra_list_tracing.pushed_pkg++;
}

/**
 *  @brief gets next raListTbl for TX based on currect scheduling configuration
 *
 *  @param pmadapter      A pointer to mlan_adapter
 *  @param priv           A pointer to mlan_private
 *  @param tid            A pointer to return tid
 *
 *  @return             raListTbl
 */
static raListTbl *wlan_wmm_get_next_priolist_ptr(pmlan_adapter pmadapter,
						 pmlan_private *priv, int *tid)
{
	mlan_private *mlan = pmadapter->selected_mlan_bss;
	mlan_callbacks *cbs = &pmadapter->callbacks;
	void *const pmoal_handle = pmadapter->pmoal_handle;
	raListTbl *ra_list = MNULL;

	if (!pmadapter->priv_num)
		return MNULL;

	if (!pmadapter->mclient_tx_supported)
		return wlan_wmm_get_highest_priolist_ptr(pmadapter, priv, tid);

	if (mlan) {
		cbs->moal_spin_lock(pmoal_handle, mlan->wmm.ra_list_spinlock);
	} else {
		mlan = wlan_wmm_get_next_bss(pmadapter);
	}

	for (; mlan != MNULL; mlan = wlan_wmm_get_next_bss(pmadapter)) {
		ra_list = mlan->wmm.selected_ra_list;

		if (ra_list == MNULL || ra_list->total_pkts == 0) {
			ra_list = wlan_wmm_get_next_ra_list(pmadapter, mlan);
		} else if (!wlan_wmm_process_ra_list_quoats(pmadapter, mlan,
							    ra_list)) {
			pmadapter->selected_mlan_bss = MNULL;
			cbs->moal_spin_unlock(pmoal_handle,
					      mlan->wmm.ra_list_spinlock);
			continue;
		}

		if (ra_list != MNULL) {
			wlan_wmm_track_ra_list_switch(mlan, ra_list);

			*priv = mlan;
			*tid = ra_list->tid;

			pmadapter->selected_mlan_bss = mlan;

			return ra_list;
		}

		cbs->moal_spin_unlock(pmoal_handle, mlan->wmm.ra_list_spinlock);
	}

	pmadapter->selected_mlan_bss = MNULL;
	// coverity[cert_arr30_c_violation:SUPPRESS]
	return wlan_wmm_get_highest_priolist_ptr(pmadapter, priv, tid);
}

/**
 *  @brief This function gets the number of packets in the Tx queue
 *
 *  @param priv           A pointer to mlan_private
 *  @param ptr            A pointer to RA list table
 *  @param max_buf_size   Maximum buffer size
 *
 *  @return             Packet count
 */
static int wlan_num_pkts_in_txq(mlan_private *priv, raListTbl *ptr,
				int max_buf_size)
{
	int count = 0, total_size = 0;
	pmlan_buffer pmbuf;

	ENTER();

	for (pmbuf = (pmlan_buffer)ptr->buf_head.pnext;
	     pmbuf != (pmlan_buffer)(&ptr->buf_head); pmbuf = pmbuf->pnext) {
		total_size += pmbuf->data_len;
		if (total_size < max_buf_size)
			++count;
		else
			break;
	}

	LEAVE();
	return count;
}

/**
 *  @brief This function sends a single packet
 *
 *  @param priv         A pointer to mlan_private
 *  @param ptr          A pointer to RA list table
 *  @param ptrindex     ptr's TID index
 *
 *  @return             N/A
 */
static INLINE void wlan_send_single_packet(pmlan_private priv, raListTbl *ptr,
					   int ptrindex)
{
	pmlan_buffer pmbuf;
	pmlan_buffer pmbuf_next;
	mlan_tx_param tx_param;
	pmlan_adapter pmadapter = priv->adapter;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	if (ptrindex < 0 || ptrindex >= MAX_NUM_TID) {
		LEAVE();
		return;
	}
	pmbuf = (pmlan_buffer)util_dequeue_list(pmadapter->pmoal_handle,
						&ptr->buf_head, MNULL, MNULL);
	if (pmbuf) {
		PRINTM(MINFO, "Dequeuing the packet %p %p\n", ptr, pmbuf);
		wlan_wmm_consume_mpdu_budget(ptr);
		wlan_wmm_consume_byte_budget(ptr, pmbuf);
		priv->wmm.pkts_queued[ptrindex]--;
		util_scalar_decrement(pmadapter->pmoal_handle,
				      &priv->wmm.tx_pkts_queued, MNULL, MNULL);
		ptr->total_pkts--;
		pmbuf_next = (pmlan_buffer)util_peek_list(
			pmadapter->pmoal_handle, &ptr->buf_head, MNULL, MNULL);
		pmadapter->callbacks.moal_spin_unlock(
			pmadapter->pmoal_handle, priv->wmm.ra_list_spinlock);

		tx_param.next_pkt_len =
			((pmbuf_next) ? pmbuf_next->data_len +
						Tx_PD_SIZEOF(pmadapter) :
					0);
		status = wlan_process_tx(priv, pmbuf, &tx_param);

		if (status == MLAN_STATUS_RESOURCE) {
			/** Queue the packet back at the head */
			PRINTM(MDAT_D, "Queuing pkt back to raList %p %p\n",
			       ptr, pmbuf);
			pmadapter->callbacks.moal_spin_lock(
				pmadapter->pmoal_handle,
				priv->wmm.ra_list_spinlock);

			if (!wlan_is_ralist_valid(priv, ptr, ptrindex)) {
				pmadapter->callbacks.moal_spin_unlock(
					pmadapter->pmoal_handle,
					priv->wmm.ra_list_spinlock);
				wlan_write_data_complete(pmadapter, pmbuf,
							 MLAN_STATUS_FAILURE);
				LEAVE();
				return;
			}
			priv->wmm.pkts_queued[ptrindex]++;
			util_scalar_increment(pmadapter->pmoal_handle,
					      &priv->wmm.tx_pkts_queued, MNULL,
					      MNULL);
			util_enqueue_list_head(pmadapter->pmoal_handle,
					       &ptr->buf_head,
					       (pmlan_linked_list)pmbuf, MNULL,
					       MNULL);

			ptr->total_pkts++;
			pmbuf->flags |= MLAN_BUF_FLAG_REQUEUED_PKT;
			pmadapter->callbacks.moal_spin_unlock(
				pmadapter->pmoal_handle,
				priv->wmm.ra_list_spinlock);
		} else {
			pmadapter->callbacks.moal_spin_lock(
				pmadapter->pmoal_handle,
				priv->wmm.ra_list_spinlock);
			if (wlan_is_ralist_valid(priv, ptr, ptrindex)) {
				priv->wmm.packets_out[ptrindex]++;
				priv->wmm.tid_tbl_ptr[ptrindex].ra_list_curr =
					ptr;
			}
			wlan_advance_bss_on_pkt_push(
				pmadapter,
				&pmadapter->bssprio_tbl[priv->bss_priority]);
			pmadapter->callbacks.moal_spin_unlock(
				pmadapter->pmoal_handle,
				priv->wmm.ra_list_spinlock);
		}
	} else {
		pmadapter->callbacks.moal_spin_unlock(
			pmadapter->pmoal_handle, priv->wmm.ra_list_spinlock);
		PRINTM(MINFO, "Nothing to send\n");
	}

	LEAVE();
}

/**
 *  @brief This function checks if this mlan_buffer is already processed.
 *
 *  @param priv     A pointer to mlan_private
 *  @param ptr      A pointer to RA list table
 *
 *  @return         MTRUE or MFALSE
 */
static INLINE int wlan_is_ptr_processed(mlan_private *priv, raListTbl *ptr)
{
	pmlan_buffer pmbuf;

	pmbuf = (pmlan_buffer)util_peek_list(priv->adapter->pmoal_handle,
					     &ptr->buf_head, MNULL, MNULL);
	if (pmbuf && (pmbuf->flags & MLAN_BUF_FLAG_REQUEUED_PKT))
		return MTRUE;

	return MFALSE;
}

/**
 *  @brief This function sends a single packet that has been processed
 *
 *  @param priv         A pointer to mlan_private
 *  @param ptr          A pointer to RA list table
 *  @param ptrindex     ptr's TID index
 *
 *  @return             N/A
 */
static INLINE void wlan_send_processed_packet(pmlan_private priv,
					      raListTbl *ptr, int ptrindex)
{
	pmlan_buffer pmbuf_next = MNULL;
	mlan_tx_param tx_param;
	pmlan_buffer pmbuf;
	pmlan_adapter pmadapter = priv->adapter;
	mlan_status ret = MLAN_STATUS_FAILURE;

	ptrindex = (ptrindex < 1) ? 0 : ptrindex;
	pmbuf = (pmlan_buffer)util_dequeue_list(pmadapter->pmoal_handle,
						&ptr->buf_head, MNULL, MNULL);
	if (pmbuf) {
		pmbuf_next = (pmlan_buffer)util_peek_list(
			pmadapter->pmoal_handle, &ptr->buf_head, MNULL, MNULL);
		pmadapter->callbacks.moal_spin_unlock(
			pmadapter->pmoal_handle, priv->wmm.ra_list_spinlock);
		tx_param.next_pkt_len =
			((pmbuf_next) ? pmbuf_next->data_len +
						Tx_PD_SIZEOF(pmadapter) :
					0);

		ret = pmadapter->ops.host_to_card(priv, MLAN_TYPE_DATA, pmbuf,
						  &tx_param);
		switch (ret) {
#ifdef USB
		case MLAN_STATUS_PRESOURCE:
			PRINTM(MINFO, "MLAN_STATUS_PRESOURCE is returned\n");
			break;
#endif
		case MLAN_STATUS_RESOURCE:
			PRINTM(MINFO, "MLAN_STATUS_RESOURCE is returned\n");
			pmadapter->callbacks.moal_spin_lock(
				pmadapter->pmoal_handle,
				priv->wmm.ra_list_spinlock);

			if (!wlan_is_ralist_valid(priv, ptr, ptrindex)) {
				pmadapter->callbacks.moal_spin_unlock(
					pmadapter->pmoal_handle,
					priv->wmm.ra_list_spinlock);
				wlan_write_data_complete(pmadapter, pmbuf,
							 MLAN_STATUS_FAILURE);
				LEAVE();
				return;
			}
			util_enqueue_list_head(pmadapter->pmoal_handle,
					       &ptr->buf_head,
					       (pmlan_linked_list)pmbuf, MNULL,
					       MNULL);

			pmbuf->flags |= MLAN_BUF_FLAG_REQUEUED_PKT;
			pmadapter->callbacks.moal_spin_unlock(
				pmadapter->pmoal_handle,
				priv->wmm.ra_list_spinlock);
			break;
		case MLAN_STATUS_FAILURE:
			PRINTM(MERROR, "Error: Failed to write data\n");
			pmadapter->dbg.num_tx_host_to_card_failure++;
			pmbuf->status_code = MLAN_ERROR_DATA_TX_FAIL;
			wlan_write_data_complete(pmadapter, pmbuf, ret);
			break;
		case MLAN_STATUS_PENDING:
			break;
		case MLAN_STATUS_SUCCESS:
			DBG_HEXDUMP(
				MDAT_D, "Tx", pmbuf->pbuf + pmbuf->data_offset,
				MIN(pmbuf->data_len + Tx_PD_SIZEOF(pmadapter),
				    MAX_DATA_DUMP_LEN));
			wlan_write_data_complete(pmadapter, pmbuf, ret);
			break;
		default:
			break;
		}
		if (ret != MLAN_STATUS_RESOURCE) {
			pmadapter->callbacks.moal_spin_lock(
				pmadapter->pmoal_handle,
				priv->wmm.ra_list_spinlock);
			if (wlan_is_ralist_valid(priv, ptr, ptrindex)) {
				priv->wmm.packets_out[ptrindex]++;
				priv->wmm.tid_tbl_ptr[ptrindex].ra_list_curr =
					ptr;
				ptr->total_pkts--;
			}

			wlan_advance_bss_on_pkt_push(
				pmadapter,
				&pmadapter->bssprio_tbl[priv->bss_priority]);
			priv->wmm.pkts_queued[ptrindex]--;
			util_scalar_decrement(pmadapter->pmoal_handle,
					      &priv->wmm.tx_pkts_queued, MNULL,
					      MNULL);
			pmadapter->callbacks.moal_spin_unlock(
				pmadapter->pmoal_handle,
				priv->wmm.ra_list_spinlock);
		}
	} else {
		pmadapter->callbacks.moal_spin_unlock(
			pmadapter->pmoal_handle, priv->wmm.ra_list_spinlock);
	}
}

/**
 *  @brief This function dequeues a packet
 *
 *  @param pmadapter  A pointer to mlan_adapter
 *
 *  @return         MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static int wlan_dequeue_tx_packet(pmlan_adapter pmadapter)
{
	raListTbl *ptr;
	pmlan_private priv = MNULL;
	int ptrindex = 0;
	t_u8 ra[MLAN_MAC_ADDR_LENGTH];
	int tid_del = 0;
	int tid = 0;
	mlan_buffer *pmbuf = MNULL;

	ENTER();

	if (!pmadapter->priv_num) {
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}

	ptr = wlan_wmm_get_next_priolist_ptr(pmadapter, &priv, &ptrindex);

	if (!ptr || ptrindex < 0) {
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}

	/*  Note:- Spinlock is locked in wlan_wmm_get_highest_priolist_ptr
	 *  when it returns a pointer (for the priv it returns),
	 *  and is unlocked in wlan_send_processed_packet,
	 *  wlan_send_single_packet or wlan_11n_aggregate_pkt.
	 *  The spinlock would be required for some parts of both of function.
	 *  But, the the bulk of these function will execute w/o spinlock.
	 *  Unlocking the spinlock inside these function will help us avoid
	 *  taking the spinlock again, check to see if the ptr is still
	 *  valid and then proceed. This is done purely to increase
	 *  execution time. */

	/* Note:- Also, anybody adding code which does not get into
	 * wlan_send_processed_packet, wlan_send_single_packet, or
	 * wlan_11n_aggregate_pkt should make sure ra_list_spinlock
	 * is freed. Otherwise there would be a lock up. */

	tid = wlan_get_tid(priv->adapter, ptr);
	if (tid >= MAX_NUM_TID)
		tid = wlan_wmm_downgrade_tid(priv, tid);

	if (wlan_is_ptr_processed(priv, ptr)) {
		wlan_send_processed_packet(priv, ptr, ptrindex);
		LEAVE();
		return MLAN_STATUS_SUCCESS;
	}
	if (ptr->del_ba_count >= DEL_BA_THRESHOLD)
		wlan_update_del_ba_count(priv, ptr);
	if (pmadapter->tp_state_on) {
		pmbuf = (pmlan_buffer)util_peek_list(
			pmadapter->pmoal_handle, &ptr->buf_head, MNULL, MNULL);
		if (pmbuf) {
			pmadapter->callbacks.moal_tp_accounting(
				pmadapter->pmoal_handle, pmbuf, 3);
			if (pmadapter->tp_state_drop_point == 3) {
				pmbuf = (pmlan_buffer)util_dequeue_list(
					pmadapter->pmoal_handle, &ptr->buf_head,
					MNULL, MNULL);
				PRINTM(MERROR, "Dequeuing the packet %p %p\n",
				       ptr, pmbuf);
				priv->wmm.pkts_queued[ptrindex]--;
				util_scalar_decrement(pmadapter->pmoal_handle,
						      &priv->wmm.tx_pkts_queued,
						      MNULL, MNULL);
				ptr->total_pkts--;
				pmadapter->callbacks.moal_spin_unlock(
					pmadapter->pmoal_handle,
					priv->wmm.ra_list_spinlock);
				wlan_write_data_complete(pmadapter, pmbuf,
							 MLAN_STATUS_SUCCESS);
				LEAVE();
				return MLAN_STATUS_SUCCESS;
			}
		}
	}
	if (!ptr->is_wmm_enabled || priv->adapter->remain_on_channel ||
	    (ptr->ba_status || ptr->del_ba_count >= DEL_BA_THRESHOLD)
#ifdef STA_SUPPORT
	    || priv->wps.session_enable
#endif /* STA_SUPPORT */
	) {
		if (ptr->is_wmm_enabled && ptr->ba_status &&
		    ptr->amsdu_in_ampdu &&
		    wlan_is_amsdu_allowed(priv, ptr, tid) &&
		    (wlan_num_pkts_in_txq(priv, ptr, pmadapter->tx_buf_size) >=
		     MIN_NUM_AMSDU)) {
			wlan_11n_aggregate_pkt(priv, ptr, priv->intf_hr_len,
					       ptrindex);
		} else
			wlan_send_single_packet(priv, ptr, ptrindex);
	} else {
		if (wlan_is_ampdu_allowed(priv, ptr, tid) &&
		    (ptr->packet_count > ptr->ba_packet_threshold)) {
			if (wlan_is_bastream_avail(priv)) {
				PRINTM(MINFO,
				       "BA setup threshold %d reached. tid=%d\n",
				       ptr->packet_count, tid);
				if (!wlan_11n_get_txbastream_tbl(
					    priv, tid, ptr->ra, MFALSE)) {
					wlan_11n_create_txbastream_tbl(
						priv, ptr->ra, tid,
						BA_STREAM_SETUP_SENT_ADDBA);
					wlan_send_addba(priv, tid, ptr->ra);
				}
			} else if (wlan_find_stream_to_delete(priv, ptr, tid,
							      &tid_del, ra)) {
				PRINTM(MDAT_D, "tid_del=%d tid=%d\n", tid_del,
				       tid);
				if (!wlan_11n_get_txbastream_tbl(
					    priv, tid, ptr->ra, MFALSE)) {
					wlan_11n_create_txbastream_tbl(
						priv, ptr->ra, tid,
						BA_STREAM_SETUP_INPROGRESS);
					wlan_11n_set_txbastream_status(
						priv, tid_del, ra,
						BA_STREAM_SENT_DELBA, MFALSE);
					wlan_send_delba(priv, MNULL, tid_del,
							ra, 1);
				}
			}
		}
		if (wlan_is_amsdu_allowed(priv, ptr, tid) &&
		    (wlan_num_pkts_in_txq(priv, ptr, pmadapter->tx_buf_size) >=
		     MIN_NUM_AMSDU)) {
			wlan_11n_aggregate_pkt(priv, ptr, priv->intf_hr_len,
					       ptrindex);
		} else {
			wlan_send_single_packet(priv, ptr, ptrindex);
		}
	}

	if (ptr->sta && ptr->sta->ps_sleep) {
		wlan_update_ralist_tx_pause(priv, ptr->ra, 1);
	}

	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief remove ra_list from pendiing TX queues
 *
 *  @param priv        A pointer to mlan_private
 *  @param pmadapter   A pointer to pmlan_adapter
 *  @param ra_list     A pointer to raListTbl
 *
 *  @return           N/A
 */
static void wlan_ralist_remove_from_pending_txq(pmlan_private priv,
						pmlan_adapter pmadapter,
						raListTbl *ra_list)
{
	mlan_adapter *adapter = priv->adapter;

	if (util_is_node_in_list(&ra_list->pending_txq_entry))
		util_unlink_list_nl(pmadapter->pmoal_handle,
				    &ra_list->pending_txq_entry);

	if (ra_list == priv->wmm.selected_ra_list)
		priv->wmm.selected_ra_list = MNULL;

	if (ra_list == adapter->ra_list_tracing.ra_list)
		adapter->ra_list_tracing.ra_list = MNULL;

	if (ra_list->sta)
		util_unlink_list_safe_nl(pmadapter->pmoal_handle,
					 &ra_list->sta->pending_stas_entry);
}

/**
 *  @brief Add ra_list back to pendiing TX queue
 *
 *  @param priv        A pointer to mlan_private
 *  @param pmadapter   A pointer to pmlan_adapter
 *  @param ra_list     A pointer to raListTbl
 *  @param tid         tid
 *
 *  @return           N/A
 */
static void wlan_ralist_add_to_pending_txq(pmlan_private priv,
					   pmlan_adapter pmadapter,
					   raListTbl *ra_list, t_u8 tid)
{
	t_u8 queue = wlan_wmm_select_queue(priv, tid);

	util_enqueue_list_head(pmadapter->pmoal_handle,
			       &priv->wmm.pending_txq[queue],
			       &ra_list->pending_txq_entry, MNULL, MNULL);
}

/**
 *  @brief update tx_pause flag in ra_list
 *
 *  @param priv		  A pointer to mlan_private
 *  @param mac        peer mac address
 *  @param tx_pause   tx_pause flag (0/1)
 *
 *
 *  @return           packets queued for this mac
 */
t_u16 wlan_update_ralist_tx_pause(pmlan_private priv, t_u8 *mac, t_u8 tx_pause)
{
	int i;
	pmlan_adapter pmadapter = priv->adapter;
	t_u32 pkt_cnt = 0;
	t_u32 tx_pkts_queued = 0;
	ENTER();

	pmadapter->callbacks.moal_spin_lock(pmadapter->pmoal_handle,
					    priv->wmm.ra_list_spinlock);
	for (i = 0; i < MAX_NUM_TID; ++i) {
		raListTbl *ra_list = wlan_wmm_get_ralist_node(priv, i, mac);
		if (ra_list == MNULL || ra_list->tx_pause == tx_pause)
			continue;

		pkt_cnt += ra_list->total_pkts;
		ra_list->tx_pause = tx_pause;
		if (tx_pause) {
			priv->wmm.pkts_paused[i] += ra_list->total_pkts;
			wlan_ralist_remove_from_pending_txq(priv, pmadapter,
							    ra_list);
		} else {
			priv->wmm.pkts_paused[i] -= ra_list->total_pkts;
			wlan_ralist_add_to_pending_txq(priv, pmadapter, ra_list,
						       i);
		}
	}

	if (pkt_cnt) {
		tx_pkts_queued = util_scalar_read(pmadapter->pmoal_handle,
						  &priv->wmm.tx_pkts_queued,
						  MNULL, MNULL);
		if (tx_pause)
			tx_pkts_queued -= pkt_cnt;
		else
			tx_pkts_queued += pkt_cnt;
		util_scalar_write(priv->adapter->pmoal_handle,
				  &priv->wmm.tx_pkts_queued, tx_pkts_queued,
				  MNULL, MNULL);
		util_scalar_write(priv->adapter->pmoal_handle,
				  &priv->wmm.highest_queued_prio, HIGH_PRIO_TID,
				  MNULL, MNULL);
	}
	pmadapter->callbacks.moal_spin_unlock(pmadapter->pmoal_handle,
					      priv->wmm.ra_list_spinlock);

	LEAVE();
	return pkt_cnt;
}

#ifdef STA_SUPPORT
/**
 *  @brief update tx_pause flag in none tdls ra_list
 *
 *  @param priv		  A pointer to mlan_private
 *  @param mac        peer mac address
 *  @param tx_pause   tx_pause flag (0/1)
 *
 *  @return           N/A
 */
t_void wlan_update_non_tdls_ralist(mlan_private *priv, t_u8 *mac, t_u8 tx_pause)
{
	raListTbl *ra_list;
	int i;
	pmlan_adapter pmadapter = priv->adapter;
	t_u32 pkt_cnt = 0;
	t_u32 tx_pkts_queued = 0;
	ENTER();

	pmadapter->callbacks.moal_spin_lock(pmadapter->pmoal_handle,
					    priv->wmm.ra_list_spinlock);
	for (i = 0; i < MAX_NUM_TID; ++i) {
		ra_list = (raListTbl *)util_peek_list(
			priv->adapter->pmoal_handle,
			&priv->wmm.tid_tbl_ptr[i].ra_list, MNULL, MNULL);
		while (ra_list &&
		       (ra_list !=
			(raListTbl *)&priv->wmm.tid_tbl_ptr[i].ra_list)) {
			if (memcmp(priv->adapter, ra_list->ra, mac,
				   MLAN_MAC_ADDR_LENGTH) &&
			    ra_list->tx_pause != tx_pause) {
				pkt_cnt += ra_list->total_pkts;
				ra_list->tx_pause = tx_pause;
				if (tx_pause)
					priv->wmm.pkts_paused[i] +=
						ra_list->total_pkts;
				else
					priv->wmm.pkts_paused[i] -=
						ra_list->total_pkts;
			}
			ra_list = ra_list->pnext;
		}
	}
	if (pkt_cnt) {
		tx_pkts_queued = util_scalar_read(pmadapter->pmoal_handle,
						  &priv->wmm.tx_pkts_queued,
						  MNULL, MNULL);
		if (tx_pause)
			tx_pkts_queued -= pkt_cnt;
		else
			tx_pkts_queued += pkt_cnt;
		util_scalar_write(priv->adapter->pmoal_handle,
				  &priv->wmm.tx_pkts_queued, tx_pkts_queued,
				  MNULL, MNULL);
		util_scalar_write(priv->adapter->pmoal_handle,
				  &priv->wmm.highest_queued_prio, HIGH_PRIO_TID,
				  MNULL, MNULL);
	}
	pmadapter->callbacks.moal_spin_unlock(pmadapter->pmoal_handle,
					      priv->wmm.ra_list_spinlock);
	LEAVE();
	return;
}

/**
 *  @brief find tdls buffer from ralist
 *
 *  @param priv		  A pointer to mlan_private
 *  @param ralist     A pointer to ralistTbl
 *  @param mac        TDLS peer mac address
 *
 *  @return           pmlan_buffer or MNULL
 */
static pmlan_buffer wlan_find_tdls_packets(mlan_private *priv,
					   raListTbl *ra_list, t_u8 *mac)
{
	pmlan_buffer pmbuf = MNULL;
	mlan_adapter *pmadapter = priv->adapter;
	t_u8 ra[MLAN_MAC_ADDR_LENGTH];
	ENTER();
	pmbuf = (pmlan_buffer)util_peek_list(priv->adapter->pmoal_handle,
					     &ra_list->buf_head, MNULL, MNULL);
	if (!pmbuf) {
		LEAVE();
		return MNULL;
	}
	while (pmbuf != (pmlan_buffer)&ra_list->buf_head) {
		memcpy_ext(pmadapter, ra, pmbuf->pbuf + pmbuf->data_offset,
			   MLAN_MAC_ADDR_LENGTH, MLAN_MAC_ADDR_LENGTH);
		if (!memcmp(priv->adapter, ra, mac, MLAN_MAC_ADDR_LENGTH)) {
			LEAVE();
			return pmbuf;
		}
		pmbuf = pmbuf->pnext;
	}
	LEAVE();
	return MNULL;
}

/**
 *  @brief find tdls buffer from tdls pending queue
 *
 *  @param priv		  A pointer to mlan_private
 *  @param mac        TDLS peer mac address
 *
 *  @return           pmlan_buffer or MNULL
 */
static pmlan_buffer wlan_find_packets_tdls_txq(mlan_private *priv, t_u8 *mac)
{
	pmlan_buffer pmbuf = MNULL;
	mlan_adapter *pmadapter = priv->adapter;
	t_u8 ra[MLAN_MAC_ADDR_LENGTH];
	ENTER();
	pmbuf = (pmlan_buffer)util_peek_list(priv->adapter->pmoal_handle,
					     &priv->tdls_pending_txq, MNULL,
					     MNULL);
	if (!pmbuf) {
		LEAVE();
		return MNULL;
	}
	while (pmbuf != (pmlan_buffer)&priv->tdls_pending_txq) {
		memcpy_ext(pmadapter, ra, pmbuf->pbuf + pmbuf->data_offset,
			   MLAN_MAC_ADDR_LENGTH, MLAN_MAC_ADDR_LENGTH);
		if (!memcmp(priv->adapter, ra, mac, MLAN_MAC_ADDR_LENGTH)) {
			LEAVE();
			return pmbuf;
		}
		pmbuf = pmbuf->pnext;
	}
	LEAVE();
	return MNULL;
}

/**
 *  @brief Remove TDLS ralist and move packets to AP's ralist
 *
 *  @param priv		  A pointer to mlan_private
 *  @param mac        TDLS peer mac address
 *
 *  @return           N/A
 */
static t_void wlan_wmm_delete_tdls_ralist(pmlan_private priv, t_u8 *mac)
{
	raListTbl *ra_list;
	raListTbl *ra_list_ap = MNULL;
	int i;
	pmlan_adapter pmadapter = priv->adapter;
	pmlan_buffer pmbuf;
	ENTER();

	for (i = 0; i < MAX_NUM_TID; ++i) {
		ra_list = wlan_wmm_get_ralist_node(priv, i, mac);
		if (ra_list) {
			PRINTM(MDATA, "delete TDLS ralist %p\n", ra_list);
			ra_list_ap = (raListTbl *)util_peek_list(
				pmadapter->pmoal_handle,
				&priv->wmm.tid_tbl_ptr[i].ra_list, MNULL,
				MNULL);
			if (!ra_list_ap) {
				LEAVE();
				return;
			}
			while ((pmbuf = (pmlan_buffer)util_peek_list(
					pmadapter->pmoal_handle,
					&ra_list->buf_head, MNULL, MNULL))) {
				util_unlink_list(pmadapter->pmoal_handle,
						 &ra_list->buf_head,
						 (pmlan_linked_list)pmbuf,
						 MNULL, MNULL);
				util_enqueue_list_tail(pmadapter->pmoal_handle,
						       &ra_list_ap->buf_head,
						       (pmlan_linked_list)pmbuf,
						       MNULL, MNULL);
				ra_list_ap->total_pkts++;
				ra_list_ap->packet_count++;
			}
			util_free_list_head(
				(t_void *)pmadapter->pmoal_handle,
				&ra_list->buf_head,
				pmadapter->callbacks.moal_free_lock);

			util_unlink_list(pmadapter->pmoal_handle,
					 &priv->wmm.tid_tbl_ptr[i].ra_list,
					 (pmlan_linked_list)ra_list, MNULL,
					 MNULL);
			pmadapter->callbacks.moal_mfree(pmadapter->pmoal_handle,
							(t_u8 *)ra_list);
			if (priv->wmm.tid_tbl_ptr[i].ra_list_curr == ra_list)
				priv->wmm.tid_tbl_ptr[i].ra_list_curr =
					ra_list_ap;
		}
	}

	LEAVE();
}
#endif /* STA_SUPPORT */
/********************************************************
			Global Functions
********************************************************/

/**
 *  @brief Get the threshold value for BA setup using system time.
 *
 *  @param pmadapter       Pointer to the mlan_adapter structure
 *
 *  @return         threshold value.
 */
t_u8 wlan_get_random_ba_threshold(pmlan_adapter pmadapter)
{
	t_u32 sec, usec;
	t_u8 ba_threshold = 0;

	ENTER();

	/* setup ba_packet_threshold here random number between
	   [BA_SETUP_PACKET_OFFSET,
	   BA_SETUP_PACKET_OFFSET+BA_SETUP_MAX_PACKET_THRESHOLD-1] */

#define BA_SETUP_MAX_PACKET_THRESHOLD 16

	pmadapter->callbacks.moal_get_system_time(pmadapter->pmoal_handle, &sec,
						  &usec);
	sec = (sec & 0xFFFF) + (sec >> 16);
	usec = (usec & 0xFFFF) + (usec >> 16);

	ba_threshold =
		(t_u8)((((sec << 16) + usec) % BA_SETUP_MAX_PACKET_THRESHOLD) +
		       pmadapter->min_ba_threshold);
	PRINTM(MINFO, "pmadapter->min_ba_threshold = %d\n",
	       pmadapter->min_ba_threshold);
	PRINTM(MINFO, "setup BA after %d packets\n", ba_threshold);

	LEAVE();
	return ba_threshold;
}

/**
 *  @brief  This function cleans Tx/Rx queues
 *
 *  @param priv		A pointer to mlan_private
 *
 *  @return		N/A
 */
t_void wlan_clean_txrx(pmlan_private priv)
{
	mlan_adapter *pmadapter = priv->adapter;
	t_u8 i = 0;

	ENTER();
	wlan_cleanup_bypass_txq(priv);
	if (GET_BSS_ROLE(priv) == MLAN_BSS_ROLE_STA) {
		wlan_cleanup_tdls_txq(priv);
	}
	wlan_11n_cleanup_reorder_tbl(priv);
	wlan_11n_deleteall_txbastream_tbl(priv);
#if defined(USB)
	if (IS_USB(pmadapter->card_type))
		wlan_reset_usb_tx_aggr(priv->adapter);
#endif
#ifdef PCIE
	if (IS_PCIE(pmadapter->card_type))
		wlan_clean_pcie_ring_buf(priv->adapter);
#endif
	pmadapter->callbacks.moal_spin_lock(pmadapter->pmoal_handle,
					    priv->wmm.ra_list_spinlock);
	wlan_wmm_cleanup_queues(priv);
	wlan_wmm_delete_all_sta_entries(priv);
	wlan_wmm_delete_all_ralist(priv);
	memcpy_ext(pmadapter, tos_to_tid, ac_to_tid, sizeof(tos_to_tid),
		   sizeof(tos_to_tid));
	for (i = 0; i < MAX_NUM_TID; i++)
		tos_to_tid_inv[tos_to_tid[i]] = (t_u8)i;
#ifdef UAP_SUPPORT
	priv->num_drop_pkts = 0;
#endif
#ifdef SDIO
	if (IS_SD(pmadapter->card_type)) {
		memset(pmadapter, pmadapter->pcard_sd->mpa_tx_count, 0,
		       sizeof(pmadapter->pcard_sd->mpa_tx_count));
		pmadapter->pcard_sd->mpa_sent_no_ports = 0;
		pmadapter->pcard_sd->mpa_sent_last_pkt = 0;
		memset(pmadapter, pmadapter->pcard_sd->mpa_rx_count, 0,
		       sizeof(pmadapter->pcard_sd->mpa_rx_count));
	}
#endif
	pmadapter->callbacks.moal_spin_unlock(pmadapter->pmoal_handle,
					      priv->wmm.ra_list_spinlock);

	LEAVE();
}

/**
 *  @brief Set the WMM queue priorities to their default values
 *
 *  @param priv     Pointer to the mlan_private driver data struct
 *
 *  @return         N/A
 */
void wlan_wmm_default_queue_priorities(pmlan_private priv)
{
	ENTER();

	/* Default queue priorities: VO->VI->BE->BK */
	priv->wmm.queue_priority[0] = WMM_AC_VO;
	priv->wmm.queue_priority[1] = WMM_AC_VI;
	priv->wmm.queue_priority[2] = WMM_AC_BE;
	priv->wmm.queue_priority[3] = WMM_AC_BK;

	LEAVE();
}

/**
 *  @brief Initialize WMM priority queues
 *
 *  @param priv     Pointer to the mlan_private driver data struct
 *  @param pwmm_ie  Pointer to the IEEEtypes_WmmParameter_t data struct
 *
 *  @return         N/A
 */
void wlan_wmm_setup_queue_priorities(pmlan_private priv,
				     IEEEtypes_WmmParameter_t *pwmm_ie)
{
	t_u16 cw_min, avg_back_off, tmp[4];
	t_u32 i, j, num_ac;
	t_u8 ac_idx;

	ENTER();

	if (!pwmm_ie || priv->wmm_enabled == MFALSE) {
		/* WMM is not enabled, just set the defaults and return */
		wlan_wmm_default_queue_priorities(priv);
		LEAVE();
		return;
	}
	memset(priv->adapter, tmp, 0, sizeof(tmp));

	HEXDUMP("WMM: setup_queue_priorities: param IE", (t_u8 *)pwmm_ie,
		sizeof(IEEEtypes_WmmParameter_t));

	PRINTM(MINFO,
	       "WMM Parameter IE: version=%d, "
	       "qos_info Parameter Set Count=%d, Reserved=%#x\n",
	       pwmm_ie->vend_hdr.version, pwmm_ie->qos_info.para_set_count,
	       pwmm_ie->reserved);

	for (num_ac = 0; num_ac < NELEMENTS(pwmm_ie->ac_params); num_ac++) {
		cw_min = (1 << pwmm_ie->ac_params[num_ac].ecw.ecw_min) - 1;
		avg_back_off = (cw_min >> 1) +
			       pwmm_ie->ac_params[num_ac].aci_aifsn.aifsn;

		ac_idx = wmm_aci_to_qidx_map[pwmm_ie->ac_params[num_ac]
						     .aci_aifsn.aci];
		priv->wmm.queue_priority[ac_idx] = ac_idx;
		tmp[ac_idx] = avg_back_off;

		PRINTM(MCMND, "WMM: CWmax=%d CWmin=%d Avg Back-off=%d\n",
		       (1 << pwmm_ie->ac_params[num_ac].ecw.ecw_max) - 1,
		       cw_min, avg_back_off);
		PRINTM_AC(&pwmm_ie->ac_params[num_ac]);
	}

	HEXDUMP("WMM: avg_back_off", (t_u8 *)tmp, sizeof(tmp));
	HEXDUMP("WMM: queue_priority", priv->wmm.queue_priority,
		sizeof(priv->wmm.queue_priority));

	/* Bubble sort */
	for (i = 0; i < num_ac; i++) {
		for (j = 1; j < num_ac - i; j++) {
			if (tmp[j - 1] > tmp[j]) {
				SWAP_U16(tmp[j - 1], tmp[j]);
				SWAP_U8(priv->wmm.queue_priority[j - 1],
					priv->wmm.queue_priority[j]);
			} else if (tmp[j - 1] == tmp[j]) {
				if (priv->wmm.queue_priority[j - 1] <
				    priv->wmm.queue_priority[j]) {
					SWAP_U8(priv->wmm.queue_priority[j - 1],
						priv->wmm.queue_priority[j]);
				}
			}
		}
	}

	wlan_wmm_queue_priorities_tid(priv, priv->wmm.queue_priority);

	HEXDUMP("WMM: avg_back_off, sort", (t_u8 *)tmp, sizeof(tmp));
	DBG_HEXDUMP(MCMD_D, "WMM: queue_priority, sort",
		    priv->wmm.queue_priority, sizeof(priv->wmm.queue_priority));
	LEAVE();
}

/**
 *  @brief Downgrade WMM priority queue
 *
 *  @param priv     Pointer to the mlan_private driver data struct
 *
 *  @return         N/A
 */
void wlan_wmm_setup_ac_downgrade(pmlan_private priv)
{
	int ac_val;

	ENTER();

	PRINTM(MINFO, "WMM: AC Priorities: BK(0), BE(1), VI(2), VO(3)\n");

	if (priv->wmm_enabled == MFALSE) {
		/* WMM is not enabled, default priorities */
		for (ac_val = WMM_AC_BK; ac_val <= WMM_AC_VO; ac_val++) {
			priv->wmm.ac_down_graded_vals[ac_val] =
				(mlan_wmm_ac_e)ac_val;
		}
	} else {
		for (ac_val = WMM_AC_BK; ac_val <= WMM_AC_VO; ac_val++) {
			priv->wmm.ac_down_graded_vals[ac_val] =
				wlan_wmm_eval_downgrade_ac(
					priv, (mlan_wmm_ac_e)ac_val);
			PRINTM(MINFO, "WMM: AC PRIO %d maps to %d\n", ac_val,
			       priv->wmm.ac_down_graded_vals[ac_val]);
		}
	}

	LEAVE();
}

/**
 *  @brief This function checks whether a station has WMM enabled or not
 *
 *  @param priv     A pointer to mlan_private
 *  @param mac      station mac address
 *  @return         MTRUE or MFALSE
 */
static t_u8 is_station_wmm_enabled(mlan_private *priv, t_u8 *mac)
{
	sta_node *sta_ptr = MNULL;
	sta_ptr = wlan_get_station_entry(priv, mac);
	if (sta_ptr) {
		if (sta_ptr->is_11n_enabled || sta_ptr->is_11ax_enabled)
			return MTRUE;
	}
	return MFALSE;
}

/**
 *  @brief This function checks whether wmm is supported
 *
 *  @param priv     A pointer to mlan_private
 *  @param ra       Address of the receiver STA
 *
 *  @return         MTRUE or MFALSE
 */
static int wlan_is_wmm_enabled(mlan_private *priv, t_u8 *ra)
{
	int ret = MFALSE;
	ENTER();
#ifdef UAP_SUPPORT
	if (GET_BSS_ROLE(priv) == MLAN_BSS_ROLE_UAP) {
		if ((!(ra[0] & 0x01)) &&
		    (priv->is_11n_enabled || priv->is_11ax_enabled))
			ret = is_station_wmm_enabled(priv, ra);
	}
#endif /* UAP_SUPPORT */
	LEAVE();
	return ret;
}

/**
 *  @brief  Allocate and add a RA list for all TIDs with the given RA
 *
 *  @param priv  Pointer to the mlan_private driver data struct
 *  @param ra	 Address of the receiver STA (AP in case of infra)
 *
 *  @return      N/A
 */
void wlan_ralist_add(mlan_private *priv, t_u8 *ra)
{
	int i;
	raListTbl *ra_list;
	pmlan_adapter pmadapter = priv->adapter;
	tdlsStatus_e status;
	struct wmm_sta_table *sta = MNULL;

	ENTER();

	sta = wlan_wmm_allocate_sta_table(pmadapter, ra);
	if (sta) {
		util_enqueue_list_tail_nl(pmadapter->pmoal_handle,
					  &priv->wmm.all_stas,
					  &sta->all_stas_entry);
	}

	for (i = 0; i < MAX_NUM_TID; ++i) {
		ra_list = wlan_wmm_allocate_ralist_node(pmadapter, ra);
		PRINTM(MINFO, "Creating RA List %p for tid %d\n", ra_list, i);
		if (!ra_list)
			break;
		ra_list->max_amsdu = 0;
		ra_list->ba_status = BA_STREAM_NOT_SETUP;
		ra_list->amsdu_in_ampdu = MFALSE;
		ra_list->tid = i;
		ra_list->queue = wlan_wmm_select_queue(priv, i);
		ra_list->sta = sta;
		if (sta)
			sta->ra_lists[i] = ra_list;
		util_init_list(&ra_list->pending_txq_entry);

		if (queuing_ra_based(priv)) {
			ra_list->is_wmm_enabled = wlan_is_wmm_enabled(priv, ra);
			if (ra_list->is_wmm_enabled)
				ra_list->max_amsdu =
					get_station_max_amsdu_size(priv, ra);
			ra_list->tx_pause = wlan_is_tx_pause(priv, ra);
		} else {
			ra_list->is_tdls_link = MFALSE;
			ra_list->tx_pause = MFALSE;
			status = wlan_get_tdls_link_status(priv, ra);
			if (MTRUE == wlan_is_tdls_link_setup(status)) {
				ra_list->is_wmm_enabled =
					is_station_wmm_enabled(priv, ra);
				if (ra_list->is_wmm_enabled)
					ra_list->max_amsdu =
						get_station_max_amsdu_size(priv,
									   ra);
				ra_list->is_tdls_link = MTRUE;
			} else {
				ra_list->is_wmm_enabled = IS_11N_ENABLED(priv);
				ra_list->is_wmm_enabled |=
					IS_116E_ENABLED(priv);
				if (ra_list->is_wmm_enabled)
					ra_list->max_amsdu = priv->max_amsdu;
			}
		}

		PRINTM_NETINTF(MDATA, priv);
		PRINTM(MDATA, "ralist %p: is_wmm_enabled=%d max_amsdu=%d\n",
		       ra_list, ra_list->is_wmm_enabled, ra_list->max_amsdu);

		if (ra_list->is_wmm_enabled) {
			ra_list->packet_count = 0;
			ra_list->ba_packet_threshold =
				wlan_get_random_ba_threshold(pmadapter);
		}

		util_enqueue_list_tail(pmadapter->pmoal_handle,
				       &priv->wmm.tid_tbl_ptr[i].ra_list,
				       (pmlan_linked_list)ra_list, MNULL,
				       MNULL);

		if (!priv->wmm.tid_tbl_ptr[i].ra_list_curr)
			priv->wmm.tid_tbl_ptr[i].ra_list_curr = ra_list;
	}

	LEAVE();
	// coverity[leaked_storage:SUPPRESS]
}

/**
 *  @brief Initialize the WMM parameter.
 *
 *  @param pmadapter  Pointer to the mlan_adapter data structure
 *
 *  @return         N/A
 */
t_void wlan_init_wmm_param(pmlan_adapter pmadapter)
{
	/* Reuse the same structure of WmmAcParameters_t for configuration
	 * purpose here. the definition of acm bit is changed to ucm (user
	 * configuration mode) FW will take the setting of
	 * aifsn,ecw_max,ecw_min, tx_op_limit only when ucm is set to 1.
	 * othewise the default setting/behavoir in firmware will be used.
	 */
	pmadapter->ac_params[AC_BE].aci_aifsn.acm = 0;
	pmadapter->ac_params[AC_BE].aci_aifsn.aci = AC_BE;
	pmadapter->ac_params[AC_BE].aci_aifsn.aifsn = 3;
	pmadapter->ac_params[AC_BE].ecw.ecw_max = 10;
	pmadapter->ac_params[AC_BE].ecw.ecw_min = 4;
	pmadapter->ac_params[AC_BE].tx_op_limit = 0;

	pmadapter->ac_params[AC_BK].aci_aifsn.acm = 0;
	pmadapter->ac_params[AC_BK].aci_aifsn.aci = AC_BK;
	pmadapter->ac_params[AC_BK].aci_aifsn.aifsn = 7;
	pmadapter->ac_params[AC_BK].ecw.ecw_max = 10;
	pmadapter->ac_params[AC_BK].ecw.ecw_min = 4;
	pmadapter->ac_params[AC_BK].tx_op_limit = 0;

	pmadapter->ac_params[AC_VI].aci_aifsn.acm = 0;
	pmadapter->ac_params[AC_VI].aci_aifsn.aci = AC_VI;
	pmadapter->ac_params[AC_VI].aci_aifsn.aifsn = 2;
	pmadapter->ac_params[AC_VI].ecw.ecw_max = 4;
	pmadapter->ac_params[AC_VI].ecw.ecw_min = 3;
	pmadapter->ac_params[AC_VI].tx_op_limit = 188;

	pmadapter->ac_params[AC_VO].aci_aifsn.acm = 0;
	pmadapter->ac_params[AC_VO].aci_aifsn.aci = AC_VO;
	pmadapter->ac_params[AC_VO].aci_aifsn.aifsn = 2;
	pmadapter->ac_params[AC_VO].ecw.ecw_max = 3;
	pmadapter->ac_params[AC_VO].ecw.ecw_min = 2;
	pmadapter->ac_params[AC_VO].tx_op_limit = 102;
}

/**
 *  @brief Initialize the WMM state information and the WMM data path queues.
 *
 *  @param pmadapter  Pointer to the mlan_adapter data structure
 *
 *  @return         N/A
 */
t_void wlan_wmm_init(pmlan_adapter pmadapter)
{
	static const IEEEtypes_WmmAcParameters_t default_ac_params[] = {
		[AC_BE] = {.aci_aifsn = {.aifsn = 3, .aci = AC_BE},
			   .ecw = {.ecw_min = 4, .ecw_max = 10}},
		[AC_BK] = {.aci_aifsn = {.aifsn = 7, .aci = AC_BK},
			   .ecw = {.ecw_min = 4, .ecw_max = 10}},
		[AC_VI] = {.aci_aifsn = {.aifsn = 2, .aci = AC_VI},
			   .ecw = {.ecw_min = 3, .ecw_max = 4}},
		[AC_VO] = {.aci_aifsn = {.aifsn = 2, .aci = AC_VO},
			   .ecw = {.ecw_min = 2, .ecw_max = 3}}};
	int i, j;
	pmlan_private priv;

	ENTER();

	for (j = 0; j < pmadapter->priv_num; ++j) {
		priv = pmadapter->priv[j];
		if (priv) {
			for (i = 0; i < MAX_NUM_TID; ++i) {
				priv->aggr_prio_tbl[i].amsdu =
					tos_to_tid_inv[i];
				priv->aggr_prio_tbl[i].ampdu_ap =
					priv->aggr_prio_tbl[i].ampdu_user =
						tos_to_tid_inv[i];
				priv->wmm.pkts_queued[i] = 0;
				priv->wmm.pkts_paused[i] = 0;
				priv->wmm.tid_tbl_ptr[i].ra_list_curr = MNULL;
				priv->wmm.selected_ra_list = MNULL;
			}
			priv->wmm.drv_pkt_delay_max = WMM_DRV_DELAY_MAX;

			priv->aggr_prio_tbl[6].amsdu = BA_STREAM_NOT_ALLOWED;
			priv->aggr_prio_tbl[7].amsdu = BA_STREAM_NOT_ALLOWED;
			priv->aggr_prio_tbl[6].ampdu_ap =
				priv->aggr_prio_tbl[6].ampdu_user =
					BA_STREAM_NOT_ALLOWED;

			priv->aggr_prio_tbl[7].ampdu_ap =
				priv->aggr_prio_tbl[7].ampdu_user =
					BA_STREAM_NOT_ALLOWED;
			priv->add_ba_param.timeout =
				MLAN_DEFAULT_BLOCK_ACK_TIMEOUT;
			if (!pmadapter->tx_ba_timeout_support)
				priv->add_ba_param.timeout = 0;
#ifdef STA_SUPPORT
			if (priv->bss_type == MLAN_BSS_TYPE_STA) {
				priv->add_ba_param.tx_win_size =
					MLAN_STA_AMPDU_DEF_TXWINSIZE;
				priv->add_ba_param.rx_win_size =
					MLAN_STA_AMPDU_DEF_RXWINSIZE;
			}
#endif
#ifdef WIFI_DIRECT_SUPPORT
			if (priv->bss_type == MLAN_BSS_TYPE_WIFIDIRECT) {
				priv->add_ba_param.tx_win_size =
					MLAN_WFD_AMPDU_DEF_TXRXWINSIZE;
				priv->add_ba_param.rx_win_size =
					MLAN_WFD_AMPDU_DEF_TXRXWINSIZE;
			}
#endif
			if (priv->bss_type == MLAN_BSS_TYPE_NAN) {
				priv->add_ba_param.tx_win_size =
					MLAN_NAN_AMPDU_DEF_TXRXWINSIZE;
				priv->add_ba_param.rx_win_size =
					MLAN_NAN_AMPDU_DEF_TXRXWINSIZE;
			}
#ifdef UAP_SUPPORT
			if (priv->bss_type == MLAN_BSS_TYPE_UAP) {
				priv->add_ba_param.tx_win_size =
					MLAN_UAP_AMPDU_DEF_TXWINSIZE;
				priv->add_ba_param.rx_win_size =
					MLAN_UAP_AMPDU_DEF_RXWINSIZE;
			}
#endif
			priv->user_rxwinsize = priv->add_ba_param.rx_win_size;
			priv->add_ba_param.tx_amsdu = MTRUE;
			priv->add_ba_param.rx_amsdu = MTRUE;
			memset(priv->adapter, priv->rx_seq, 0xff,
			       sizeof(priv->rx_seq));
			wlan_wmm_default_queue_priorities(priv);

			wlan_wmm_contention_init(priv, default_ac_params);
		}
	}

	LEAVE();
}

/**
 *  @brief Setup the queue priorities and downgrade any queues as required
 *         by the WMM info.  Setups default values if WMM is not active
 *         for this association.
 *
 *  @param priv     Pointer to the mlan_private driver data struct
 *
 *  @return         N/A
 */
void wlan_wmm_setup_queues(pmlan_private priv)
{
	ENTER();
	wlan_wmm_setup_queue_priorities(priv, MNULL);
	wlan_wmm_setup_ac_downgrade(priv);
	LEAVE();
}

#ifdef STA_SUPPORT
/**
 *  @brief  Send a command to firmware to retrieve the current WMM status
 *
 *  @param priv     Pointer to the mlan_private driver data struct
 *
 *  @return         MLAN_STATUS_SUCCESS; MLAN_STATUS_FAILURE
 */
mlan_status wlan_cmd_wmm_status_change(pmlan_private priv)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;

	ENTER();

	ret = wlan_prepare_cmd(priv, HostCmd_CMD_WMM_GET_STATUS, 0, 0, MNULL,
			       MNULL);
	LEAVE();
	return ret;
}
#endif

/**
 *  @brief Check if wmm TX queue is empty
 *
 *  @param pmadapter  Pointer to the mlan_adapter driver data struct
 *
 *  @return         MFALSE if not empty; MTRUE if empty
 */
int wlan_wmm_lists_empty(pmlan_adapter pmadapter)
{
	int j;
	pmlan_private priv;

	ENTER();

	for (j = 0; j < pmadapter->priv_num; ++j) {
		priv = pmadapter->priv[j];
		if (priv) {
			if ((priv->port_ctrl_mode == MTRUE) &&
			    (priv->port_open == MFALSE)) {
				PRINTM(MINFO,
				       "wmm_lists_empty: PORT_CLOSED Ignore pkts from BSS%d\n",
				       j);
				continue;
			}
			if (priv->tx_pause)
				continue;
#if defined(USB)
			if (!wlan_is_port_ready(pmadapter, priv->port_index))
				continue;
#endif

			if (util_scalar_read(
				    pmadapter->pmoal_handle,
				    &priv->wmm.tx_pkts_queued,
				    pmadapter->callbacks.moal_spin_lock,
				    pmadapter->callbacks.moal_spin_unlock)) {
				LEAVE();
				return MFALSE;
			}
		}
	}

	LEAVE();
	return MTRUE;
}

/**
 *   @brief Get ralist node
 *
 *   @param priv     Pointer to the mlan_private driver data struct
 *   @param tid      TID
 *   @param ra_addr  Pointer to the route address
 *
 *   @return         ra_list or MNULL
 */
raListTbl *wlan_wmm_get_ralist_node(pmlan_private priv, t_u8 tid, t_u8 *ra_addr)
{
	raListTbl *ra_list;
	ENTER();
	ra_list =
		(raListTbl *)util_peek_list(priv->adapter->pmoal_handle,
					    &priv->wmm.tid_tbl_ptr[tid].ra_list,
					    MNULL, MNULL);
	while (ra_list &&
	       (ra_list != (raListTbl *)&priv->wmm.tid_tbl_ptr[tid].ra_list)) {
		if (!memcmp(priv->adapter, ra_list->ra, ra_addr,
			    MLAN_MAC_ADDR_LENGTH)) {
			LEAVE();
			return ra_list;
		}
		ra_list = ra_list->pnext;
	}
	LEAVE();
	return MNULL;
}

/**
 *   @brief Get wmm_sta_table node
 *
 *   @param priv     Pointer to the mlan_private driver data struct
 *   @param ra_addr  Pointer to the route address
 *
 *   @return         wmm_sta_table or MNULL
 */
static struct wmm_sta_table *wlan_wmm_get_sta(pmlan_private priv,
					      const t_u8 *ra_addr)
{
	mlan_linked_list *sta_entry;
	mlan_adapter *adapter = priv->adapter;

	if (!adapter->mclient_tx_supported)
		return MNULL;

	ENTER();

	for (sta_entry = util_peek_list_nl(adapter->pmoal_handle,
					   &priv->wmm.all_stas);
	     sta_entry && sta_entry != (void *)&priv->wmm.all_stas;
	     sta_entry = sta_entry->pnext) {
		struct wmm_sta_table *sta = util_container_of(
			sta_entry, struct wmm_sta_table, all_stas_entry);

		if (memcmp(adapter, sta->ra, ra_addr, MLAN_MAC_ADDR_LENGTH) ==
		    0) {
			LEAVE();
			return sta;
		}
	}

	LEAVE();
	return MNULL;
}

/**
 *   @brief Check if RA list is valid or not
 *
 *   @param priv     Pointer to the mlan_private driver data struct
 *   @param ra_list  Pointer to raListTbl
 *   @param ptrindex TID pointer index
 *
 *   @return         MTRUE- valid. MFALSE- invalid.
 */
int wlan_is_ralist_valid(mlan_private *priv, raListTbl *ra_list, int ptrindex)
{
	raListTbl *rlist;

	ENTER();

	if (ptrindex < 0 || ptrindex >= MAX_NUM_TID) {
		LEAVE();
		return MFALSE;
	}

	rlist = (raListTbl *)util_peek_list(
		priv->adapter->pmoal_handle,
		&priv->wmm.tid_tbl_ptr[ptrindex].ra_list, MNULL, MNULL);

	while (rlist &&
	       (rlist !=
		(raListTbl *)&priv->wmm.tid_tbl_ptr[ptrindex].ra_list)) {
		if (rlist == ra_list) {
			LEAVE();
			return MTRUE;
		}

		rlist = rlist->pnext;
	}
	LEAVE();
	return MFALSE;
}

/**
 *  @brief  Update an existing raList with a new RA and 11n capability
 *
 *  @param priv     Pointer to the mlan_private driver data struct
 *  @param old_ra   Old receiver address
 *  @param new_ra   New receiver address
 *
 *  @return         integer count of updated nodes
 */
int wlan_ralist_update(mlan_private *priv, t_u8 *old_ra, t_u8 *new_ra)
{
	t_u8 tid;
	int update_count;
	raListTbl *ra_list;

	ENTER();

	update_count = 0;

	for (tid = 0; tid < MAX_NUM_TID; ++tid) {
		ra_list = wlan_wmm_get_ralist_node(priv, tid, old_ra);

		if (ra_list) {
			update_count++;

			if (queuing_ra_based(priv)) {
				ra_list->is_wmm_enabled =
					wlan_is_wmm_enabled(priv, new_ra);
				if (ra_list->is_wmm_enabled)
					ra_list->max_amsdu =
						get_station_max_amsdu_size(
							priv, new_ra);
			} else {
				ra_list->is_wmm_enabled = IS_11N_ENABLED(priv);
				ra_list->is_wmm_enabled |=
					IS_116E_ENABLED(priv);
				if (ra_list->is_wmm_enabled)
					ra_list->max_amsdu = priv->max_amsdu;
			}

			ra_list->tx_pause = MFALSE;
			ra_list->packet_count = 0;
			ra_list->ba_packet_threshold =
				wlan_get_random_ba_threshold(priv->adapter);
			ra_list->amsdu_in_ampdu = MFALSE;
			ra_list->ba_status = BA_STREAM_NOT_SETUP;
			PRINTM(MINFO,
			       "ralist_update: %p, %d, " MACSTR "-->" MACSTR
			       "\n",
			       ra_list, ra_list->is_wmm_enabled,
			       MAC2STR(ra_list->ra), MAC2STR(new_ra));

			memcpy_ext(priv->adapter, ra_list->ra, new_ra,
				   MLAN_MAC_ADDR_LENGTH, MLAN_MAC_ADDR_LENGTH);
		}
	}

	LEAVE();
	return update_count;
}

/**
 *  @brief  Update TX PHY rate for STAs in sta_list
 *
 *  @param pmpriv     Pointer to the mlan_private driver data struct
 *  @param sta_list   NULL terminated list of STAs
 *
 *  @return MLAN_STATUS_SUCCESS
 */
static mlan_status wlan_misc_get_sta_rate(pmlan_private pmpriv,
					  t_u8 *sta_list[])
{
	HostCmd_CMD_802_11_STA_TX_RATE txRateReq = {0};
	mlan_status ret = MLAN_STATUS_SUCCESS;
	int i;

	for (i = 0; i < NELEMENTS(txRateReq.entry) && sta_list[i]; ++i) {
		memcpy_ext(pmpriv->adapter, txRateReq.entry[i].sta_mac,
			   sta_list[i], MLAN_MAC_ADDR_LENGTH,
			   MLAN_MAC_ADDR_LENGTH);
	}

	if (i > 0) {
		txRateReq.num_entries = i;
		ret = wlan_prepare_cmd(pmpriv, HostCmd_CMD_PEER_TX_RATE_QUERY,
				       HostCmd_ACT_GEN_GET, 0, MNULL,
				       &txRateReq);
	}

	return ret;
}

/**
 *  @brief Trigger STAs TX rate update
 *
 *  @param pmadapter  Pointer to the pmlan_adapter driver data struct
 *  @param priv       Pointer to the mlan_private driver data struct
 *
 *  @return N/A
 */
static void wlan_wmm_update_sta_txrate_info(pmlan_adapter pmadapter,
					    pmlan_private priv)
{
	t_u8 *sta_list[MAX_STA_IN_TX_RATE_REQ + 1] = {0};
	t_void *pmoal = pmadapter->pmoal_handle;
	mlan_callbacks *cbs = &pmadapter->callbacks;
	int idx = 0;
	const int idx_limit = MAX_STA_IN_TX_RATE_REQ;
	pmlan_linked_list list_entry;
	t_u64 time_now;
	static const t_u64 update_interval = 1000ull * 1000ull * 1000ull;

	cbs->moal_get_host_time_ns(&time_now);

	if (util_is_time_before(time_now, priv->wmm.next_rate_update)) {
		return;
	}

	if (priv->wmm.is_rate_update_pending) {
		return;
	}

	priv->wmm.next_rate_update = time_now + update_interval;

	while (idx < idx_limit &&
	       (list_entry = util_peek_list_nl(
			pmoal, &priv->wmm.pending_stas)) != MNULL) {
		struct wmm_sta_table *sta = util_container_of(
			list_entry, struct wmm_sta_table, pending_stas_entry);
		const t_bool is_bmcast = (sta->ra[0] & 0x01);

		if (!is_bmcast) {
			sta_list[idx++] = sta->ra;
		}

		util_unlink_list_nl(pmoal, list_entry);
	}

	if (idx > 0) {
		priv->wmm.is_rate_update_pending = MTRUE;
		wlan_misc_get_sta_rate(priv, sta_list);
	}
}

/**
 *  @brief Updates STAs activity
 *
 *  @param pmadapter  Pointer to the pmlan_adapter driver data struct
 *  @param priv       Pointer to the mlan_private driver data struct
 *  @param ra_list    Pointer to the raListTbl struct
 *  @param sta_table  Pointer to the wmm_sta_table struct
 *
 *  @return N/A
 */
static t_void wlan_wmm_record_sta_tx(pmlan_adapter pmadapter,
				     pmlan_private priv, raListTbl *ra_list,
				     struct wmm_sta_table *sta_table)
{
	t_bool is_bmcast;
	const t_u32 tx_active_threshold = 4;

	if (!sta_table)
		return;

	is_bmcast = (sta_table->ra[0] & 0x01);
	if (!is_bmcast && ra_list->total_pkts > tx_active_threshold &&
	    !util_is_node_in_list(&sta_table->active_sta_entry)) {
		util_enqueue_list_tail_nl(pmadapter->pmoal_handle,
					  &priv->wmm.active_stas.list,
					  &sta_table->active_sta_entry);
		priv->wmm.active_stas.n_stas++;
	}
}

/**
 *  @brief Updates queue_packets budget for all active STAs
 *
 *  @param pmadapter  Pointer to the pmlan_adapter driver data struct
 *  @param priv       Pointer to the mlan_private driver data struct
 *
 *  @return N/A
 */
static t_void wlan_wmm_update_queue_packets_budget(pmlan_adapter pmadapter,
						   pmlan_private priv)
{
	t_void *pmoal = pmadapter->pmoal_handle;
	pmlan_linked_list list_entry;
	const t_u32 queue_packets_limit = pmadapter->init_para.max_tx_pending;
	t_u64 total_capacity = 0;
	t_u64 time_now;
	mlan_callbacks *cbs = &pmadapter->callbacks;
	static const t_u64 update_interval = 200ull * 1000ull * 1000ull;
	const t_u32 min_sta_share = 128;
	const t_u32 max_pending_tx_time_us = 200u * 1000u;

	if (!pmadapter->mclient_tx_supported)
		return;

	cbs->moal_get_host_time_ns(&time_now);

	if (util_is_time_before(time_now, priv->wmm.active_stas.next_update))
		return;

	priv->wmm.active_stas.n_stas = 0;
	priv->wmm.active_stas.next_update = time_now + update_interval;

	for (list_entry = util_peek_list_nl(pmoal, &priv->wmm.active_stas.list);
	     util_is_list_node(&priv->wmm.active_stas.list, list_entry);
	     list_entry = list_entry->pnext) {
		struct wmm_sta_table *sta = util_container_of(
			list_entry, struct wmm_sta_table, active_sta_entry);

		total_capacity += sta->budget.byte_budget_init;
	}

	if (total_capacity == 0)
		return;

	while ((list_entry = util_peek_list_nl(
			pmoal, &priv->wmm.active_stas.list)) != MNULL) {
		struct wmm_sta_table *sta = util_container_of(
			list_entry, struct wmm_sta_table, active_sta_entry);
		const t_u64 sta_capacity = sta->budget.byte_budget_init;
		const t_u32 max_pkts_by_airtime =
			wlan_wmm_get_byte_budget(pmadapter,
						 max_pending_tx_time_us,
						 sta->budget.phy_rate_kbps) /
			MV_ETH_FRAME_LEN;
		t_u32 sta_share = pmadapter->callbacks.moal_do_div(
			(t_u64)queue_packets_limit * sta_capacity,
			total_capacity);

		sta_share = MAX(sta_share, min_sta_share);
		sta_share = MIN(sta_share, queue_packets_limit * 7 / 8);
		sta_share = MIN(sta_share, max_pkts_by_airtime);

		sta->budget.queue_packets = sta_share;
		util_unlink_list_nl(pmoal, list_entry);
	}
}

/**
 *  @brief Add packet to WMM queue
 *
 *  @param pmadapter  Pointer to the mlan_adapter driver data struct
 *  @param pmbuf      Pointer to the mlan_buffer data struct
 *
 *  @return         N/A
 */
t_void wlan_wmm_add_buf_txqueue(pmlan_adapter pmadapter, pmlan_buffer pmbuf)
{
	pmlan_private priv = pmadapter->priv[pmbuf->bss_index];
	t_u32 tid;
	raListTbl *ra_list;
	struct wmm_sta_table *sta_table;
	t_u8 ra[MLAN_MAC_ADDR_LENGTH], tid_down;
	tdlsStatus_e status;
#ifdef UAP_SUPPORT
	psta_node sta_ptr = MNULL;
#endif

	ENTER();

	pmbuf->buf_type = MLAN_BUF_TYPE_DATA;
	if (!priv->media_connected) {
		PRINTM_NETINTF(MWARN, priv);
		PRINTM(MWARN, "Drop packet %p in disconnect state\n", pmbuf);
		wlan_write_data_complete(pmadapter, pmbuf, MLAN_STATUS_FAILURE);
		LEAVE();
		return;
	}
	tid = pmbuf->priority;
	pmadapter->callbacks.moal_spin_lock(pmadapter->pmoal_handle,
					    priv->wmm.ra_list_spinlock);
	tid_down = wlan_wmm_downgrade_tid(priv, tid);

	/* In case of infra as we have already created the list during
	   association we just don't have to call get_queue_raptr, we will have
	   only 1 raptr for a tid in case of infra */
	if (!queuing_ra_based(priv)) {
		memcpy_ext(pmadapter, ra, pmbuf->pbuf + pmbuf->data_offset,
			   MLAN_MAC_ADDR_LENGTH, MLAN_MAC_ADDR_LENGTH);
		status = wlan_get_tdls_link_status(priv, ra);
		if (MTRUE == wlan_is_tdls_link_setup(status)) {
			ra_list = wlan_wmm_get_queue_raptr(priv, tid_down, ra);
			pmbuf->flags |= MLAN_BUF_FLAG_TDLS;
		} else if (status == TDLS_SETUP_INPROGRESS) {
			wlan_add_buf_tdls_txqueue(priv, pmbuf);
			pmadapter->callbacks.moal_spin_unlock(
				pmadapter->pmoal_handle,
				priv->wmm.ra_list_spinlock);
			LEAVE();
			return;
		} else
			ra_list = (raListTbl *)util_peek_list(
				pmadapter->pmoal_handle,
				&priv->wmm.tid_tbl_ptr[tid_down].ra_list, MNULL,
				MNULL);
	} else {
		if (pmbuf->flags & MLAN_BUF_FLAG_EASYMESH)
			memcpy_ext(pmadapter, ra, pmbuf->mac,
				   MLAN_MAC_ADDR_LENGTH, MLAN_MAC_ADDR_LENGTH);
		else
			memcpy_ext(pmadapter, ra,
				   pmbuf->pbuf + pmbuf->data_offset,
				   MLAN_MAC_ADDR_LENGTH, MLAN_MAC_ADDR_LENGTH);
		/** put multicast/broadcast packet in the same ralist */
		if (ra[0] & 0x01)
			memset(pmadapter, ra, 0xff, sizeof(ra));
#ifdef UAP_SUPPORT
		else if (priv->bss_type == MLAN_BSS_TYPE_UAP) {
			sta_ptr = wlan_get_station_entry(priv, ra);
			if (sta_ptr) {
				sta_ptr->stats.tx_bytes += pmbuf->data_len;
				sta_ptr->stats.tx_packets++;
				if (!sta_ptr->is_wmm_enabled &&
				    !priv->is_11ac_enabled) {
					tid_down = wlan_wmm_downgrade_tid(priv,
									  0xff);
				}
			}
		}
#endif
		ra_list = wlan_wmm_get_queue_raptr(priv, tid_down, ra);
	}

	if (!ra_list) {
		PRINTM_NETINTF(MWARN, priv);
		PRINTM(MWARN,
		       "Drop packet %p, ra_list=%p, media_connected=%d\n",
		       pmbuf, ra_list, priv->media_connected);
		pmadapter->callbacks.moal_spin_unlock(
			pmadapter->pmoal_handle, priv->wmm.ra_list_spinlock);
		wlan_write_data_complete(pmadapter, pmbuf, MLAN_STATUS_FAILURE);
		LEAVE();
		return;
	}

	sta_table = wlan_wmm_get_sta(priv, ra_list->ra);
	wlan_wmm_record_sta_tx(pmadapter, priv, ra_list, sta_table);
	wlan_wmm_update_queue_packets_budget(pmadapter, priv);

	if (sta_table &&
	    ra_list->total_pkts > sta_table->budget.queue_packets &&
	    !(pmbuf->flags & MLAN_BUF_FLAG_TCP_PKT)) {
		pmadapter->callbacks.moal_spin_unlock(
			pmadapter->pmoal_handle, priv->wmm.ra_list_spinlock);
		wlan_write_data_complete(pmadapter, pmbuf,
					 MLAN_STATUS_RESOURCE);
		LEAVE();

		return;
	}

	PRINTM_NETINTF(MDATA, priv);
	PRINTM(MDATA,
	       "Adding pkt %p (priority=%d, tid_down=%d) to ra_list %p\n",
	       pmbuf, pmbuf->priority, tid_down, ra_list);
	util_enqueue_list_tail(pmadapter->pmoal_handle, &ra_list->buf_head,
			       (pmlan_linked_list)pmbuf, MNULL, MNULL);

	ra_list->total_pkts++;
	ra_list->packet_count++;

	priv->wmm.pkts_queued[tid_down]++;
	if (ra_list->tx_pause) {
		priv->wmm.pkts_paused[tid_down]++;
	} else {
		util_scalar_increment(pmadapter->pmoal_handle,
				      &priv->wmm.tx_pkts_queued, MNULL, MNULL);
		/* if highest_queued_prio < prio(tid_down), set it to
		 * prio(tid_down) */
		util_scalar_conditional_write(
			pmadapter->pmoal_handle, &priv->wmm.highest_queued_prio,
			MLAN_SCALAR_COND_LESS_THAN, tos_to_tid_inv[tid_down],
			tos_to_tid_inv[tid_down], MNULL, MNULL);
	}

	if (sta_table) {
		if (!ra_list->tx_pause &&
		    !util_is_node_in_list(&sta_table->pending_stas_entry)) {
			util_enqueue_list_tail_nl(
				pmadapter->pmoal_handle,
				&priv->wmm.pending_stas,
				&sta_table->pending_stas_entry);
		}

		wlan_wmm_update_sta_txrate_info(pmadapter, priv);

		/* enqueue node-tid to pending AC */
		if (!ra_list->tx_pause &&
		    !util_is_node_in_list(&ra_list->pending_txq_entry)) {
			t_u8 queue = wlan_wmm_select_queue(priv, tid_down);

			util_enqueue_list_tail_nl(pmadapter->pmoal_handle,
						  &priv->wmm.pending_txq[queue],
						  &ra_list->pending_txq_entry);
		}
	}
	/* Record the current time the packet was queued; used to determine
	 *   the amount of time the packet was queued in the driver before it
	 *   was sent to the firmware.  The delay is then sent along with the
	 *   packet to the firmware for aggregate delay calculation for stats
	 *   and MSDU lifetime expiry.
	 */
	pmadapter->callbacks.moal_get_system_time(
		pmadapter->pmoal_handle, &pmbuf->in_ts_sec, &pmbuf->in_ts_usec);
	pmadapter->callbacks.moal_spin_unlock(pmadapter->pmoal_handle,
					      priv->wmm.ra_list_spinlock);

	LEAVE();
}

#ifdef STA_SUPPORT
/**
 *  @brief Process the GET_WMM_STATUS command response from firmware
 *
 *  The GET_WMM_STATUS response may contain multiple TLVs for:
 *      - AC Queue status TLVs
 *      - Current WMM Parameter IE TLV
 *      - Admission Control action frame TLVs
 *
 *  This function parses the TLVs and then calls further functions
 *   to process any changes in the queue prioritize or state.
 *
 *  @param priv      Pointer to the mlan_private driver data struct
 *  @param ptlv      Pointer to the tlv block returned in the response.
 *  @param resp_len  Length of TLV block
 *
 *  @return MLAN_STATUS_SUCCESS
 */
mlan_status wlan_ret_wmm_get_status(pmlan_private priv, t_u8 *ptlv,
				    int resp_len)
{
	t_u8 *pcurrent = ptlv;
	t_u32 tlv_len;
	t_u8 send_wmm_event;
	MrvlIEtypes_Data_t *ptlv_hdr;
	MrvlIEtypes_WmmQueueStatus_t *ptlv_wmm_q_status;
	IEEEtypes_WmmParameter_t *pwmm_param_ie = MNULL;
	WmmAcStatus_t *pac_status;

	MrvlIETypes_ActionFrame_t *ptlv_action;
	IEEEtypes_Action_WMM_AddTsRsp_t *padd_ts_rsp;
	IEEEtypes_Action_WMM_DelTs_t *pdel_ts;

	ENTER();

	send_wmm_event = MFALSE;
	if (resp_len < (int)sizeof(ptlv_hdr->header)) {
		PRINTM(MINFO,
		       "WMM: WMM_GET_STATUS err: cmdresp low length received: %d\n",
		       resp_len);
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}
	PRINTM(MINFO, "WMM: WMM_GET_STATUS cmdresp received: %d\n", resp_len);
	HEXDUMP("CMD_RESP: WMM_GET_STATUS", pcurrent, resp_len);

	while (resp_len >= (int)sizeof(ptlv_hdr->header)) {
		ptlv_hdr = (MrvlIEtypes_Data_t *)pcurrent;
		tlv_len = wlan_le16_to_cpu(ptlv_hdr->header.len);
		if ((tlv_len + sizeof(ptlv_hdr->header)) > resp_len) {
			PRINTM(MERROR,
			       "WMM get status: Error in processing  TLV buffer\n");
			resp_len = 0;
			continue;
		}

		switch (wlan_le16_to_cpu(ptlv_hdr->header.type)) {
		case TLV_TYPE_WMMQSTATUS:
			ptlv_wmm_q_status =
				(MrvlIEtypes_WmmQueueStatus_t *)ptlv_hdr;
			PRINTM(MEVENT, "WMM_STATUS: QSTATUS TLV: %u\n",
			       ptlv_wmm_q_status->queue_index);

			PRINTM(MINFO,
			       "CMD_RESP: WMM_GET_STATUS: QSTATUS TLV: %u, %d, %d\n",
			       ptlv_wmm_q_status->queue_index,
			       ptlv_wmm_q_status->flow_required,
			       ptlv_wmm_q_status->disabled);

			/* Pick the minimum among these to avoid array out of
			 * bounds */
			ptlv_wmm_q_status->queue_index = MIN(
				ptlv_wmm_q_status->queue_index, MAX_AC_QUEUES);
			if (ptlv_wmm_q_status->queue_index < MAX_AC_QUEUES) {
				pac_status =
					&priv->wmm.ac_status
						 [ptlv_wmm_q_status->queue_index];
				pac_status->disabled =
					ptlv_wmm_q_status->disabled;
				pac_status->flow_required =
					ptlv_wmm_q_status->flow_required;
				pac_status->flow_created =
					ptlv_wmm_q_status->flow_created;
			}
			break;

		case TLV_TYPE_VENDOR_SPECIFIC_IE: /* WMM_IE */
			/*
			 * Point the regular IEEE IE 2 bytes into the NXP IE
			 *   and setup the IEEE IE type and length byte fields
			 */

			PRINTM(MEVENT, "WMM STATUS: WMM IE\n");

			HEXDUMP("WMM: WMM TLV:", (t_u8 *)ptlv_hdr, tlv_len + 4);

			pwmm_param_ie =
				(IEEEtypes_WmmParameter_t *)(pcurrent + 2);
			pwmm_param_ie->vend_hdr.len = (t_u8)tlv_len;
			pwmm_param_ie->vend_hdr.element_id = WMM_IE;

			PRINTM(MINFO,
			       "CMD_RESP: WMM_GET_STATUS: WMM Parameter Set: %d\n",
			       pwmm_param_ie->qos_info.para_set_count);

			memcpy_ext(priv->adapter,
				   (t_u8 *)&priv->curr_bss_params.bss_descriptor
					   .wmm_ie,
				   pwmm_param_ie,
				   (pwmm_param_ie->vend_hdr.len + 2),
				   sizeof(IEEEtypes_WmmParameter_t));
			send_wmm_event = MTRUE;
			break;

		case TLV_TYPE_IEEE_ACTION_FRAME:
			PRINTM(MEVENT, "WMM_STATUS: IEEE Action Frame\n");
			ptlv_action = (MrvlIETypes_ActionFrame_t *)pcurrent;

			ptlv_action->actionFrame.wmmAc.tspecAct.category =
				wlan_le32_to_cpu(ptlv_action->actionFrame.wmmAc
							 .tspecAct.category);
			if (ptlv_action->actionFrame.wmmAc.tspecAct.category ==
			    IEEE_MGMT_ACTION_CATEGORY_WMM_TSPEC) {
				ptlv_action->actionFrame.wmmAc.tspecAct.action =
					wlan_le32_to_cpu(
						ptlv_action->actionFrame.wmmAc
							.tspecAct.action);
				switch (ptlv_action->actionFrame.wmmAc.tspecAct
						.action) {
				case TSPEC_ACTION_CODE_ADDTS_RSP:
					padd_ts_rsp = &ptlv_action->actionFrame
							       .wmmAc.addTsRsp;
					wlan_send_wmmac_host_event(
						priv, "ADDTS_RSP",
						ptlv_action->srcAddr,
						padd_ts_rsp->tspecIE.TspecBody
							.TSInfo.TID,
						padd_ts_rsp->tspecIE.TspecBody
							.TSInfo.UserPri,
						padd_ts_rsp->statusCode);
					break;

				case TSPEC_ACTION_CODE_DELTS:
					pdel_ts = &ptlv_action->actionFrame
							   .wmmAc.delTs;
					wlan_send_wmmac_host_event(
						priv, "DELTS_RX",
						ptlv_action->srcAddr,
						pdel_ts->tspecIE.TspecBody
							.TSInfo.TID,
						pdel_ts->tspecIE.TspecBody
							.TSInfo.UserPri,
						pdel_ts->reasonCode);
					break;

				case TSPEC_ACTION_CODE_ADDTS_REQ:
				default:
					break;
				}
			}
			break;

		default:
			break;
		}

		pcurrent += (tlv_len + sizeof(ptlv_hdr->header));
		resp_len -= (tlv_len + sizeof(ptlv_hdr->header));
	}

	wlan_wmm_setup_queue_priorities(priv, pwmm_param_ie);
	wlan_wmm_setup_ac_downgrade(priv);
	if (pwmm_param_ie != MNULL)
		wlan_wmm_contention_init(priv, pwmm_param_ie->ac_params);

	if (send_wmm_event) {
		wlan_recv_event(priv, MLAN_EVENT_ID_FW_WMM_CONFIG_CHANGE,
				MNULL);
	}

	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief Call back from the command module to allow insertion of a WMM TLV
 *
 *  If the BSS we are associating to supports WMM, add the required WMM
 *    Information IE to the association request command buffer in the form
 *    of a NXP extended IEEE IE.
 *
 *  @param priv         Pointer to the mlan_private driver data struct
 *  @param ppassoc_buf  Output parameter: Pointer to the TLV output buffer,
 *                      modified on return to point after the appended WMM TLV
 *  @param pwmm_ie      Pointer to the WMM IE for the BSS we are joining
 *
 *  @return Length of data appended to the association tlv buffer
 */
t_u32 wlan_wmm_process_association_req(pmlan_private priv, t_u8 **ppassoc_buf,
				       IEEEtypes_WmmParameter_t *pwmm_ie)
{
	MrvlIEtypes_WmmParamSet_t *pwmm_tlv;
	t_u32 ret_len = 0;

	ENTER();

	/* Null checks */
	if (!ppassoc_buf) {
		LEAVE();
		return 0;
	}
	if (!(*ppassoc_buf)) {
		LEAVE();
		return 0;
	}

	if (!pwmm_ie) {
		LEAVE();
		return 0;
	}

	PRINTM(MINFO, "WMM: process assoc req: bss->wmmIe=0x%x\n",
	       pwmm_ie->vend_hdr.element_id);

	if (priv->wmm_required && pwmm_ie->vend_hdr.element_id == WMM_IE) {
		pwmm_tlv = (MrvlIEtypes_WmmParamSet_t *)*ppassoc_buf;
		pwmm_tlv->header.type = (t_u16)wmm_info_ie[0];
		pwmm_tlv->header.type = wlan_cpu_to_le16(pwmm_tlv->header.type);
		pwmm_tlv->header.len = (t_u16)wmm_info_ie[1];
		memcpy_ext(priv->adapter, pwmm_tlv->wmm_ie, &wmm_info_ie[2],
			   pwmm_tlv->header.len, pwmm_tlv->header.len);
		if (pwmm_ie->qos_info.qos_uapsd)
			memcpy_ext(priv->adapter,
				   (t_u8 *)(pwmm_tlv->wmm_ie +
					    pwmm_tlv->header.len -
					    sizeof(priv->wmm_qosinfo)),
				   &priv->wmm_qosinfo,
				   sizeof(priv->wmm_qosinfo),
				   sizeof(priv->wmm_qosinfo));

		ret_len = sizeof(pwmm_tlv->header) + pwmm_tlv->header.len;
		pwmm_tlv->header.len = wlan_cpu_to_le16(pwmm_tlv->header.len);

		HEXDUMP("ASSOC_CMD: WMM IE", (t_u8 *)pwmm_tlv, ret_len);
		*ppassoc_buf += ret_len;
	}

	LEAVE();
	return ret_len;
}
#endif /* STA_SUPPORT */

/**
 *   @brief Compute the time delay in the driver queues for a given packet.
 *
 *   When the packet is received at the OS/Driver interface, the current
 *     time is set in the packet structure.  The difference between the present
 *     time and that received time is computed in this function and limited
 *     based on pre-compiled limits in the driver.
 *
 *   @param priv   Ptr to the mlan_private driver data struct
 *   @param pmbuf  Ptr to the mlan_buffer which has been previously timestamped
 *
 *   @return  Time delay of the packet in 2ms units after having limit applied
 */
t_u8 wlan_wmm_compute_driver_packet_delay(pmlan_private priv,
					  const pmlan_buffer pmbuf)
{
	t_u8 ret_val = 0;
	t_u32 out_ts_sec, out_ts_usec;
	t_s32 queue_delay, delay;
	t_s32 temp_delay = 0;
	ENTER();

	priv->adapter->callbacks.moal_get_system_time(
		priv->adapter->pmoal_handle, &out_ts_sec, &out_ts_usec);
	if (priv->adapter->tp_state_on) {
		pmbuf->out_ts_sec = out_ts_sec;
		pmbuf->out_ts_usec = out_ts_usec;
		if (pmbuf->in_ts_sec)
			priv->adapter->callbacks.moal_tp_accounting(
				priv->adapter->pmoal_handle, pmbuf, 11);
	}
	if (!wlan_secure_sub(&out_ts_sec, pmbuf->in_ts_sec, &temp_delay,
			     TYPE_SINT32))
		PRINTM(MERROR, "%s:TS(sec) not valid \n", __func__);

	queue_delay = temp_delay * 1000;

	if (!wlan_secure_sub(&out_ts_usec, pmbuf->in_ts_usec, &temp_delay,
			     TYPE_SINT32))
		PRINTM(MERROR, "%s:TS(usec) not valid \n", __func__);

	queue_delay += temp_delay / 1000;
	/*
	 * Queue delay is passed as a uint8 in units of 2ms (ms shifted
	 *  by 1). Min value (other than 0) is therefore 2ms, max is 510ms.
	 *
	 * Pass max value if queue_delay is beyond the uint8 range
	 */
	delay = MIN(queue_delay, (t_s32)priv->wmm.drv_pkt_delay_max) >> 1;
	ret_val = (t_u8)delay;

	PRINTM(MINFO, "WMM: Pkt Delay: %d ms, %d ms sent to FW\n", queue_delay,
	       ret_val);

	LEAVE();
	return ret_val;
}

/**
 *  @brief Transmit the highest priority packet awaiting in the WMM Queues
 *
 *  @param pmadapter Pointer to the mlan_adapter driver data struct
 *
 *  @return        N/A
 */
void wlan_wmm_process_tx(pmlan_adapter pmadapter)
{
	ENTER();

	if (!pmadapter->priv_num) {
		LEAVE();
		return;
	}

	do {
		if (wlan_dequeue_tx_packet(pmadapter))
			break;
#ifdef SDIO
		if (IS_SD(pmadapter->card_type) &&
		    (pmadapter->ireg & UP_LD_CMD_PORT_HOST_INT_STATUS)) {
			wlan_send_mp_aggr_buf(pmadapter);
			break;
		}
#endif

#ifdef PCIE
		if (IS_PCIE(pmadapter->card_type) &&
		    (pmadapter->ireg &
		     pmadapter->pcard_pcie->reg->host_intr_event_rdy))
			break;
#endif
#ifdef USB
		if (IS_USB(pmadapter->card_type) && pmadapter->event_received)
			break;
#endif
		/* Check if busy */
	} while (!pmadapter->data_sent && !pmadapter->tx_lock_flag &&
		 !wlan_wmm_lists_empty(pmadapter));

	LEAVE();
	return;
}

/**
 *  @brief select wmm queue
 *
 *  @param pmpriv       A pointer to mlan_private structure
 *  @param tid          TID 0-7
 *
 *  @return             wmm_queue priority (0-3)
 */
t_u8 wlan_wmm_select_queue(mlan_private *pmpriv, t_u8 tid)
{
	pmlan_adapter pmadapter = pmpriv->adapter;
	t_u8 i;
	mlan_wmm_ac_e ac_down =
		pmpriv->wmm.ac_down_graded_vals[wlan_wmm_convert_tos_to_ac(
			pmadapter, tid)];

	ENTER();

	for (i = 0; i < 4; i++) {
		if (pmpriv->wmm.queue_priority[i] == ac_down) {
			LEAVE();
			return i;
		}
	}
	LEAVE();
	return 0;
}

/**
 *  @brief Delete tx packets in RA list
 *
 *  @param priv			Pointer to the mlan_private driver data struct
 *  @param ra_list_head	ra list header
 *  @param tid          tid
 *
 *  @return		N/A
 */
static INLINE t_u8 wlan_del_tx_pkts_in_ralist(pmlan_private priv,
					      mlan_list_head *ra_list_head,
					      int tid)
{
	raListTbl *ra_list = MNULL;
	pmlan_adapter pmadapter = priv->adapter;
	pmlan_buffer pmbuf = MNULL;
	t_u8 ret = MFALSE;
	ENTER();

	if (tid < 0 || tid >= MAX_NUM_TID) {
		LEAVE();
		return ret;
	}
	ra_list = (raListTbl *)util_peek_list(priv->adapter->pmoal_handle,
					      ra_list_head, MNULL, MNULL);
	while (ra_list && ra_list != (raListTbl *)ra_list_head) {
		if (ra_list->total_pkts &&
		    (ra_list->tx_pause ||
		     (ra_list->total_pkts > RX_LOW_THRESHOLD))) {
			pmbuf = (pmlan_buffer)util_dequeue_list(
				pmadapter->pmoal_handle, &ra_list->buf_head,
				MNULL, MNULL);
			if (pmbuf) {
				PRINTM(MDATA,
				       "Drop pkts: tid=%d tx_pause=%d pkts=%d " MACSTR
				       "\n",
				       tid, ra_list->tx_pause,
				       ra_list->total_pkts,
				       MAC2STR(ra_list->ra));
				wlan_write_data_complete(pmadapter, pmbuf,
							 MLAN_STATUS_FAILURE);
				priv->wmm.pkts_queued[tid]--;
				priv->num_drop_pkts++;
				ra_list->total_pkts--;
				if (ra_list->tx_pause)
					priv->wmm.pkts_paused[tid]--;
				else
					util_scalar_decrement(
						pmadapter->pmoal_handle,
						&priv->wmm.tx_pkts_queued,
						MNULL, MNULL);
				ret = MTRUE;
				break;
			}
		}
		ra_list = ra_list->pnext;
	}

	LEAVE();
	return ret;
}

/**
 *  @brief Drop tx pkts
 *
 *  @param priv			Pointer to the mlan_private driver data struct
 *
 *  @return		N/A
 */
t_void wlan_drop_tx_pkts(pmlan_private priv)
{
	int j;
	static int i;
	pmlan_adapter pmadapter = priv->adapter;
	pmadapter->callbacks.moal_spin_lock(pmadapter->pmoal_handle,
					    priv->wmm.ra_list_spinlock);
	for (j = 0; j < MAX_NUM_TID; j++, i++) {
		if (i < 0)
			break;
		if (i == MAX_NUM_TID)
			i = 0;
		if (wlan_del_tx_pkts_in_ralist(
			    priv, &priv->wmm.tid_tbl_ptr[i].ra_list, i)) {
			i++;
			break;
		}
	}
	pmadapter->callbacks.moal_spin_unlock(pmadapter->pmoal_handle,
					      priv->wmm.ra_list_spinlock);
	return;
}

/**
 *  @brief Remove peer ralist
 *
 *  @param priv		  A pointer to mlan_private
 *  @param mac        peer mac address
 *
 *  @return           N/A
 */
t_void wlan_wmm_delete_peer_ralist(pmlan_private priv, t_u8 *mac)
{
	raListTbl *ra_list;
	struct wmm_sta_table *sta;
	int i;
	pmlan_adapter pmadapter = priv->adapter;
	mlan_callbacks *cbs = &pmadapter->callbacks;
	void *const pmoal_handle = pmadapter->pmoal_handle;
	t_u32 pkt_cnt = 0;
	t_u32 tx_pkts_queued = 0;

	ENTER();
	cbs->moal_spin_lock(pmoal_handle, priv->wmm.ra_list_spinlock);

	for (i = 0; i < MAX_NUM_TID; ++i) {
		ra_list = wlan_wmm_get_ralist_node(priv, i, mac);
		if (ra_list) {
			PRINTM(MINFO, "delete sta ralist %p\n", ra_list);
			priv->wmm.pkts_queued[i] -= ra_list->total_pkts;
			if (ra_list->tx_pause)
				priv->wmm.pkts_paused[i] -= ra_list->total_pkts;
			else
				pkt_cnt += ra_list->total_pkts;
			wlan_wmm_del_pkts_in_ralist_node(priv, ra_list);

			if (ra_list == priv->wmm.selected_ra_list)
				priv->wmm.selected_ra_list = MNULL;

			if (ra_list == pmadapter->ra_list_tracing.ra_list)
				pmadapter->ra_list_tracing.ra_list = MNULL;

			if (util_is_node_in_list(&ra_list->pending_txq_entry))
				util_unlink_list_nl(
					pmoal_handle,
					&ra_list->pending_txq_entry);

			util_unlink_list(pmoal_handle,
					 &priv->wmm.tid_tbl_ptr[i].ra_list,
					 (pmlan_linked_list)ra_list, MNULL,
					 MNULL);
			cbs->moal_mfree(pmoal_handle, (t_u8 *)ra_list);
			if (priv->wmm.tid_tbl_ptr[i].ra_list_curr == ra_list)
				priv->wmm.tid_tbl_ptr[i].ra_list_curr =
					(raListTbl *)&priv->wmm.tid_tbl_ptr[i]
						.ra_list;
		}
	}
	if (pkt_cnt) {
		tx_pkts_queued = util_scalar_read(
			pmoal_handle, &priv->wmm.tx_pkts_queued, MNULL, MNULL);
		tx_pkts_queued -= pkt_cnt;
		util_scalar_write(priv->adapter->pmoal_handle,
				  &priv->wmm.tx_pkts_queued, tx_pkts_queued,
				  MNULL, MNULL);
		util_scalar_write(priv->adapter->pmoal_handle,
				  &priv->wmm.highest_queued_prio, HIGH_PRIO_TID,
				  MNULL, MNULL);
	}

	sta = wlan_wmm_get_sta(priv, mac);
	if (sta) {
		util_unlink_list_nl(pmoal_handle, &sta->all_stas_entry);
		util_unlink_list_safe_nl(pmoal_handle,
					 &sta->pending_stas_entry);
		util_unlink_list_safe_nl(pmoal_handle, &sta->active_sta_entry);

		cbs->moal_mfree(pmoal_handle, (t_u8 *)sta);
	}

	cbs->moal_spin_unlock(pmoal_handle, priv->wmm.ra_list_spinlock);
	LEAVE();
}

#ifdef STA_SUPPORT
/**
 *  @brief Hold TDLS packets to tdls pending queue
 *
 *  @param priv		A pointer to mlan_private
 *  @param mac      station mac address
 *
 *  @return      N/A
 */
t_void wlan_hold_tdls_packets(pmlan_private priv, t_u8 *mac)
{
	pmlan_buffer pmbuf;
	mlan_adapter *pmadapter = priv->adapter;
	raListTbl *ra_list = MNULL;
	t_u8 i;

	ENTER();
	pmadapter->callbacks.moal_spin_lock(pmadapter->pmoal_handle,
					    priv->wmm.ra_list_spinlock);
	PRINTM(MDATA, "wlan_hold_tdls_packets: " MACSTR "\n", MAC2STR(mac));
	for (i = 0; i < MAX_NUM_TID; ++i) {
		ra_list = (raListTbl *)util_peek_list(
			pmadapter->pmoal_handle,
			&priv->wmm.tid_tbl_ptr[i].ra_list, MNULL, MNULL);
		if (ra_list) {
			while ((pmbuf = wlan_find_tdls_packets(priv, ra_list,
							       mac))) {
				util_unlink_list(pmadapter->pmoal_handle,
						 &ra_list->buf_head,
						 (pmlan_linked_list)pmbuf,
						 MNULL, MNULL);
				ra_list->total_pkts--;
				priv->wmm.pkts_queued[i]--;
				util_scalar_decrement(pmadapter->pmoal_handle,
						      &priv->wmm.tx_pkts_queued,
						      MNULL, MNULL);
				ra_list->packet_count--;
				wlan_add_buf_tdls_txqueue(priv, pmbuf);
				PRINTM(MDATA, "hold tdls packet=%p\n", pmbuf);
			}
		}
	}
	pmadapter->callbacks.moal_spin_unlock(pmadapter->pmoal_handle,
					      priv->wmm.ra_list_spinlock);
	LEAVE();
}

/**
 *  @brief move TDLS packets back to ralist
 *
 *  @param priv		  A pointer to mlan_private
 *  @param mac        TDLS peer mac address
 *  @param status     tdlsStatus
 *
 *  @return           pmlan_buffer or MNULL
 */
t_void wlan_restore_tdls_packets(pmlan_private priv, t_u8 *mac,
				 tdlsStatus_e status)
{
	pmlan_buffer pmbuf;
	mlan_adapter *pmadapter = priv->adapter;
	raListTbl *ra_list = MNULL;
	t_u32 tid;
	t_u32 tid_down;

	ENTER();
	PRINTM(MDATA, "wlan_restore_tdls_packets: " MACSTR " status=%d\n",
	       MAC2STR(mac), status);

	pmadapter->callbacks.moal_spin_lock(pmadapter->pmoal_handle,
					    priv->wmm.ra_list_spinlock);

	while ((pmbuf = wlan_find_packets_tdls_txq(priv, mac))) {
		util_unlink_list(pmadapter->pmoal_handle,
				 &priv->tdls_pending_txq,
				 (pmlan_linked_list)pmbuf, MNULL, MNULL);
		tid = pmbuf->priority;
		tid_down = wlan_wmm_downgrade_tid(priv, tid);
		if (status == TDLS_SETUP_COMPLETE) {
			ra_list = wlan_wmm_get_queue_raptr(priv, tid_down, mac);
			pmbuf->flags |= MLAN_BUF_FLAG_TDLS;
		} else {
			ra_list = (raListTbl *)util_peek_list(
				pmadapter->pmoal_handle,
				&priv->wmm.tid_tbl_ptr[tid_down].ra_list, MNULL,
				MNULL);
			pmbuf->flags &= ~MLAN_BUF_FLAG_TDLS;
		}
		if (!ra_list) {
			PRINTM_NETINTF(MWARN, priv);
			PRINTM(MWARN,
			       "Drop packet %p, ra_list=%p media_connected=%d\n",
			       pmbuf, ra_list, priv->media_connected);
			wlan_write_data_complete(pmadapter, pmbuf,
						 MLAN_STATUS_FAILURE);
			continue;
		}
		PRINTM_NETINTF(MDATA, priv);
		PRINTM(MDATA,
		       "ADD TDLS pkt %p (priority=%d) back to ra_list %p\n",
		       pmbuf, pmbuf->priority, ra_list);
		util_enqueue_list_tail(pmadapter->pmoal_handle,
				       &ra_list->buf_head,
				       (pmlan_linked_list)pmbuf, MNULL, MNULL);
		ra_list->total_pkts++;
		ra_list->packet_count++;
		priv->wmm.pkts_queued[tid_down]++;
		util_scalar_increment(pmadapter->pmoal_handle,
				      &priv->wmm.tx_pkts_queued, MNULL, MNULL);
		util_scalar_conditional_write(
			pmadapter->pmoal_handle, &priv->wmm.highest_queued_prio,
			MLAN_SCALAR_COND_LESS_THAN, tos_to_tid_inv[tid_down],
			tos_to_tid_inv[tid_down], MNULL, MNULL);
	}
	if (status != TDLS_SETUP_COMPLETE)
		wlan_wmm_delete_tdls_ralist(priv, mac);
	pmadapter->callbacks.moal_spin_unlock(pmadapter->pmoal_handle,
					      priv->wmm.ra_list_spinlock);
	LEAVE();
}

/**
 *  @brief This function prepares the command of HOST ADDTS
 *
 *  @param pmpriv       A pointer to mlan_private structure
 *  @param cmd          A pointer to HostCmd_DS_COMMAND structure
 *  @param pdata_buf    A pointer to data buffer
 *  @return             MLAN_STATUS_SUCCESS
 */
mlan_status wlan_cmd_wmm_host_addts_req(pmlan_private pmpriv,
					HostCmd_DS_COMMAND *cmd,
					t_void *pdata_buf)
{
	mlan_ds_tx_addts_cfg *paddts = (mlan_ds_tx_addts_cfg *)pdata_buf;
	HostCmd_DS_WMM_HOST_ADDTS_REQ *pcmd_addts = &cmd->params.host_add_ts;

	ENTER();

	cmd->command = wlan_cpu_to_le16(HostCmd_CMD_WMM_HOST_ADDTS_REQ);
	cmd->size = wlan_cpu_to_le16(sizeof(*pcmd_addts) + S_DS_GEN);
	cmd->result = 0;

	pcmd_addts->tsid = paddts->tsid;
	;
	pcmd_addts->user_prio = paddts->user_prio;
	pcmd_addts->admitted_time = wlan_cpu_to_le16(paddts->admitted_time);
	memcpy_ext(pmpriv->adapter, pcmd_addts->peer_addr, paddts->peer,
		   sizeof(pcmd_addts->peer_addr), MLAN_MAC_ADDR_LENGTH);

	LEAVE();

	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function prepares the command of HOST DELTS
 *
 *  @param pmpriv       A pointer to mlan_private structure
 *  @param cmd          A pointer to HostCmd_DS_COMMAND structure
 *  @param pdata_buf    A pointer to data buffer
 *  @return             MLAN_STATUS_SUCCESS
 */
mlan_status wlan_cmd_wmm_host_delts_req(pmlan_private pmpriv,
					HostCmd_DS_COMMAND *cmd,
					t_void *pdata_buf)
{
	mlan_ds_tx_delts_cfg *pdelts = (mlan_ds_tx_delts_cfg *)pdata_buf;
	HostCmd_DS_WMM_HOST_DELTS_REQ *pcmd_delts = &cmd->params.host_del_ts;

	ENTER();

	cmd->command = wlan_cpu_to_le16(HostCmd_CMD_WMM_HOST_DELTS_REQ);
	cmd->size = wlan_cpu_to_le16(sizeof(*pcmd_delts) + S_DS_GEN);
	cmd->result = 0;

	pcmd_delts->tsid = pdelts->tsid;
	memcpy_ext(pmpriv->adapter, pcmd_delts->peer_addr, pdelts->peer,
		   sizeof(pcmd_delts->peer_addr), sizeof(MLAN_MAC_ADDR_LENGTH));

	LEAVE();

	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function prepares the command of ADDTS
 *
 *  @param pmpriv       A pointer to mlan_private structure
 *  @param cmd          A pointer to HostCmd_DS_COMMAND structure
 *  @param pdata_buf    A pointer to data buffer
 *  @return             MLAN_STATUS_SUCCESS
 */
mlan_status wlan_cmd_wmm_addts_req(pmlan_private pmpriv,
				   HostCmd_DS_COMMAND *cmd, t_void *pdata_buf)
{
	mlan_ds_wmm_addts *paddts = (mlan_ds_wmm_addts *)pdata_buf;
	HostCmd_DS_WMM_ADDTS_REQ *pcmd_addts = &cmd->params.add_ts;

	ENTER();

	cmd->command = wlan_cpu_to_le16(HostCmd_CMD_WMM_ADDTS_REQ);
	cmd->size = wlan_cpu_to_le16(sizeof(pcmd_addts->dialog_token) +
				     sizeof(pcmd_addts->timeout_ms) +
				     sizeof(pcmd_addts->command_result) +
				     sizeof(pcmd_addts->ieee_status_code) +
				     paddts->ie_data_len + S_DS_GEN);
	cmd->result = 0;

	pcmd_addts->timeout_ms = wlan_cpu_to_le32(paddts->timeout);
	pcmd_addts->dialog_token = paddts->dialog_tok;
	memcpy_ext(pmpriv->adapter, pcmd_addts->tspec_data, paddts->ie_data,
		   paddts->ie_data_len, WMM_TSPEC_SIZE);

	LEAVE();

	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function handles the command response of ADDTS
 *
 *  @param pmpriv       A pointer to mlan_private structure
 *  @param resp         A pointer to HostCmd_DS_COMMAND
 *  @param pioctl_buf   A pointer to mlan_ioctl_req structure
 *
 *  @return             MLAN_STATUS_SUCCESS
 */
mlan_status wlan_ret_wmm_addts_req(pmlan_private pmpriv,
				   const HostCmd_DS_COMMAND *resp,
				   mlan_ioctl_req *pioctl_buf)
{
	mlan_ds_wmm_cfg *pwmm = MNULL;
	mlan_ds_wmm_addts *paddts = MNULL;
	const HostCmd_DS_WMM_ADDTS_REQ *presp_addts = &resp->params.add_ts;

	ENTER();

	if (pioctl_buf) {
		pwmm = (mlan_ds_wmm_cfg *)pioctl_buf->pbuf;
		paddts = (mlan_ds_wmm_addts *)&pwmm->param.addts;
		paddts->result = wlan_le32_to_cpu(presp_addts->command_result);
		paddts->dialog_tok = presp_addts->dialog_token;
		paddts->status_code = (t_u32)presp_addts->ieee_status_code;

		if (paddts->result == MLAN_CMD_RESULT_SUCCESS) {
			/* The tspecData field is potentially variable in size
			 * due to extra IEs that may have been in the ADDTS
			 * response action frame. Calculate the data length from
			 * the firmware command response.
			 */
			paddts->ie_data_len =
				(t_u8)(resp->size -
				       sizeof(presp_addts->command_result) -
				       sizeof(presp_addts->timeout_ms) -
				       sizeof(presp_addts->dialog_token) -
				       sizeof(presp_addts->ieee_status_code) -
				       S_DS_GEN);

			/* Copy the TSPEC data include any extra IEs after the
			 * TSPEC */
			// coverity[cert_arr30_c_violation: SUPPRESS]
			memcpy_ext(pmpriv->adapter, paddts->ie_data,
				   presp_addts->tspec_data, paddts->ie_data_len,
				   sizeof(paddts->ie_data));
		} else {
			paddts->ie_data_len = 0;
		}
		PRINTM(MINFO, "TSPEC: ADDTS ret = %d,%d sz=%d\n",
		       paddts->result, paddts->status_code,
		       paddts->ie_data_len);

		HEXDUMP("TSPEC: ADDTS data", paddts->ie_data,
			paddts->ie_data_len);
	}

	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function prepares the command of DELTS
 *
 *  @param pmpriv       A pointer to mlan_private structure
 *  @param cmd          A pointer to HostCmd_DS_COMMAND structure
 *  @param pdata_buf    A pointer to data buffer
 *  @return             MLAN_STATUS_SUCCESS
 */
mlan_status wlan_cmd_wmm_delts_req(pmlan_private pmpriv,
				   HostCmd_DS_COMMAND *cmd, t_void *pdata_buf)
{
	mlan_ds_wmm_delts *pdelts = (mlan_ds_wmm_delts *)pdata_buf;
	HostCmd_DS_WMM_DELTS_REQ *pcmd_delts = &cmd->params.del_ts;

	ENTER();

	cmd->command = wlan_cpu_to_le16(HostCmd_CMD_WMM_DELTS_REQ);
	cmd->size = wlan_cpu_to_le16(sizeof(pcmd_delts->dialog_token) +
				     sizeof(pcmd_delts->command_result) +
				     sizeof(pcmd_delts->ieee_reason_code) +
				     pdelts->ie_data_len + S_DS_GEN);
	cmd->result = 0;
	pcmd_delts->ieee_reason_code = (t_u8)pdelts->status_code;
	memcpy_ext(pmpriv->adapter, pcmd_delts->tspec_data, pdelts->ie_data,
		   pdelts->ie_data_len, WMM_TSPEC_SIZE);

	LEAVE();

	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function handles the command response of DELTS
 *
 *  @param pmpriv       A pointer to mlan_private structure
 *  @param resp         A pointer to HostCmd_DS_COMMAND
 *  @param pioctl_buf   A pointer to mlan_ioctl_req structure
 *
 *  @return             MLAN_STATUS_SUCCESS
 */
mlan_status wlan_ret_wmm_delts_req(pmlan_private pmpriv,
				   const HostCmd_DS_COMMAND *resp,
				   mlan_ioctl_req *pioctl_buf)
{
	mlan_ds_wmm_cfg *pwmm;
	const IEEEtypes_WMM_TSPEC_t *ptspec_ie;
	const HostCmd_DS_WMM_DELTS_REQ *presp_delts = &resp->params.del_ts;

	ENTER();

	if (pioctl_buf) {
		pwmm = (mlan_ds_wmm_cfg *)pioctl_buf->pbuf;
		pwmm->param.delts.result =
			wlan_le32_to_cpu(presp_delts->command_result);

		PRINTM(MINFO, "TSPEC: DELTS result = %d\n",
		       presp_delts->command_result);

		if (pwmm->param.delts.result == 0) {
			ptspec_ie = (const IEEEtypes_WMM_TSPEC_t *)
					    presp_delts->tspec_data;
			wlan_send_wmmac_host_event(
				pmpriv, "DELTS_TX", MNULL,
				ptspec_ie->TspecBody.TSInfo.TID,
				ptspec_ie->TspecBody.TSInfo.UserPri,
				presp_delts->ieee_reason_code);
		}
	}

	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function prepares the command of WMM_QUEUE_STATS
 *
 *  @param pmpriv       A pointer to mlan_private structure
 *  @param cmd          A pointer to HostCmd_DS_COMMAND structure
 *  @param pdata_buf    A pointer to data buffer
 *  @return             MLAN_STATUS_SUCCESS
 */
mlan_status wlan_cmd_wmm_queue_stats(pmlan_private pmpriv,
				     HostCmd_DS_COMMAND *cmd, t_void *pdata_buf)
{
	mlan_ds_wmm_queue_stats *pqstats = (mlan_ds_wmm_queue_stats *)pdata_buf;
	HostCmd_DS_WMM_QUEUE_STATS *pcmd_qstats = &cmd->params.queue_stats;
	t_u8 id;

	ENTER();

	cmd->command = wlan_cpu_to_le16(HostCmd_CMD_WMM_QUEUE_STATS);
	cmd->size =
		wlan_cpu_to_le16(sizeof(HostCmd_DS_WMM_QUEUE_STATS) + S_DS_GEN);
	cmd->result = 0;

	pcmd_qstats->action = pqstats->action;
	pcmd_qstats->select_is_userpri = 1;
	pcmd_qstats->select_bin = pqstats->user_priority;
	pcmd_qstats->pkt_count = wlan_cpu_to_le16(pqstats->pkt_count);
	pcmd_qstats->pkt_loss = wlan_cpu_to_le16(pqstats->pkt_loss);
	pcmd_qstats->avg_queue_delay =
		wlan_cpu_to_le32(pqstats->avg_queue_delay);
	pcmd_qstats->avg_tx_delay = wlan_cpu_to_le32(pqstats->avg_tx_delay);
	pcmd_qstats->used_time = wlan_cpu_to_le16(pqstats->used_time);
	pcmd_qstats->policed_time = wlan_cpu_to_le16(pqstats->policed_time);
	for (id = 0; id < MLAN_WMM_STATS_PKTS_HIST_BINS; id++) {
		pcmd_qstats->delay_histogram[id] =
			wlan_cpu_to_le16(pqstats->delay_histogram[id]);
	}

	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function handles the command response of WMM_QUEUE_STATS
 *
 *  @param pmpriv       A pointer to mlan_private structure
 *  @param resp         A pointer to HostCmd_DS_COMMAND
 *  @param pioctl_buf   A pointer to mlan_ioctl_req structure
 *
 *  @return             MLAN_STATUS_SUCCESS
 */
mlan_status wlan_ret_wmm_queue_stats(pmlan_private pmpriv,
				     const HostCmd_DS_COMMAND *resp,
				     mlan_ioctl_req *pioctl_buf)
{
	mlan_ds_wmm_cfg *pwmm = MNULL;
	mlan_ds_wmm_queue_stats *pqstats = MNULL;
	const HostCmd_DS_WMM_QUEUE_STATS *presp_qstats =
		&resp->params.queue_stats;
	t_u8 id;

	ENTER();

	if (pioctl_buf) {
		pwmm = (mlan_ds_wmm_cfg *)pioctl_buf->pbuf;
		pqstats = (mlan_ds_wmm_queue_stats *)&pwmm->param.q_stats;

		pqstats->action = presp_qstats->action;
		pqstats->user_priority = presp_qstats->select_bin;
		pqstats->pkt_count = wlan_le16_to_cpu(presp_qstats->pkt_count);
		pqstats->pkt_loss = wlan_le16_to_cpu(presp_qstats->pkt_loss);
		pqstats->avg_queue_delay =
			wlan_le32_to_cpu(presp_qstats->avg_queue_delay);
		pqstats->avg_tx_delay =
			wlan_le32_to_cpu(presp_qstats->avg_tx_delay);
		pqstats->used_time = wlan_le16_to_cpu(presp_qstats->used_time);
		pqstats->policed_time =
			wlan_le16_to_cpu(presp_qstats->policed_time);
		for (id = 0; id < MLAN_WMM_STATS_PKTS_HIST_BINS; id++) {
			pqstats->delay_histogram[id] = wlan_le16_to_cpu(
				presp_qstats->delay_histogram[id]);
		}
	}

	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function prepares the command of WMM_TS_STATUS
 *
 *  @param pmpriv       A pointer to mlan_private structure
 *  @param cmd          A pointer to HostCmd_DS_COMMAND structure
 *  @param pdata_buf    A pointer to data buffer
 *  @return             MLAN_STATUS_SUCCESS
 */
mlan_status wlan_cmd_wmm_ts_status(pmlan_private pmpriv,
				   HostCmd_DS_COMMAND *cmd, t_void *pdata_buf)
{
	mlan_ds_wmm_ts_status *pts_status = (mlan_ds_wmm_ts_status *)pdata_buf;
	HostCmd_DS_WMM_TS_STATUS *pcmd_ts_status = &cmd->params.ts_status;

	ENTER();

	cmd->command = wlan_cpu_to_le16(HostCmd_CMD_WMM_TS_STATUS);
	cmd->size =
		wlan_cpu_to_le16(sizeof(HostCmd_DS_WMM_TS_STATUS) + S_DS_GEN);
	cmd->result = 0;

	memcpy_ext(pmpriv->adapter, (t_void *)pcmd_ts_status,
		   (t_void *)pts_status, sizeof(HostCmd_DS_WMM_TS_STATUS),
		   sizeof(HostCmd_DS_WMM_TS_STATUS));

	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function handles the command response of WMM_TS_STATUS
 *
 *  @param pmpriv       A pointer to mlan_private structure
 *  @param resp         A pointer to HostCmd_DS_COMMAND
 *  @param pioctl_buf   A pointer to mlan_ioctl_req structure
 *
 *  @return             MLAN_STATUS_SUCCESS
 */
mlan_status wlan_ret_wmm_ts_status(pmlan_private pmpriv,
				   HostCmd_DS_COMMAND *resp,
				   mlan_ioctl_req *pioctl_buf)
{
	mlan_ds_wmm_cfg *pwmm = MNULL;
	HostCmd_DS_WMM_TS_STATUS *presp_ts_status = &resp->params.ts_status;

	ENTER();

	if (pioctl_buf) {
		pwmm = (mlan_ds_wmm_cfg *)pioctl_buf->pbuf;
		presp_ts_status->medium_time =
			wlan_le16_to_cpu(presp_ts_status->medium_time);
		memcpy_ext(pmpriv->adapter, (t_void *)&pwmm->param.ts_status,
			   (t_void *)presp_ts_status,
			   sizeof(mlan_ds_wmm_ts_status),
			   sizeof(mlan_ds_wmm_ts_status));
	}

	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief Set/Get WMM status
 *
 *  @param pmadapter   A pointer to mlan_adapter structure
 *  @param pioctl_req A pointer to ioctl request buffer
 *
 *  @return     MLAN_STATUS_SUCCESS --success
 */
static mlan_status wlan_wmm_ioctl_enable(pmlan_adapter pmadapter,
					 pmlan_ioctl_req pioctl_req)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_private *pmpriv = pmadapter->priv[pioctl_req->bss_index];
	mlan_ds_wmm_cfg *wmm = MNULL;
	ENTER();
	wmm = (mlan_ds_wmm_cfg *)pioctl_req->pbuf;
	if (pioctl_req->action == MLAN_ACT_GET)
		wmm->param.wmm_enable = (t_u32)pmpriv->wmm_required;
	else
		pmpriv->wmm_required = (t_u8)wmm->param.wmm_enable;
	pioctl_req->data_read_written = sizeof(t_u32) + MLAN_SUB_COMMAND_SIZE;
	LEAVE();
	return ret;
}

/**
 *  @brief Set/Get WMM QoS configuration
 *
 *  @param pmadapter   A pointer to mlan_adapter structure
 *  @param pioctl_req A pointer to ioctl request buffer
 *
 *  @return     MLAN_STATUS_SUCCESS --success
 */
static mlan_status wlan_wmm_ioctl_qos(pmlan_adapter pmadapter,
				      pmlan_ioctl_req pioctl_req)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_private *pmpriv = pmadapter->priv[pioctl_req->bss_index];
	mlan_ds_wmm_cfg *wmm = MNULL;

	ENTER();

	wmm = (mlan_ds_wmm_cfg *)pioctl_req->pbuf;

	if (pioctl_req->action == MLAN_ACT_GET)
		wmm->param.qos_cfg = pmpriv->wmm_qosinfo;
	else {
		pmpriv->wmm_qosinfo = wmm->param.qos_cfg;
		pmpriv->saved_wmm_qosinfo = wmm->param.qos_cfg;
	}

	pioctl_req->data_read_written = sizeof(t_u8) + MLAN_SUB_COMMAND_SIZE;

	LEAVE();
	return ret;
}

/**
 *  @brief Request for add a TSPEC
 *
 *  @param pmadapter    A pointer to mlan_adapter structure
 *  @param pioctl_req   A pointer to ioctl request buffer
 *
 *  @return             MLAN_STATUS_PENDING --success, otherwise fail
 */
static mlan_status wlan_wmm_ioctl_addts_req(pmlan_adapter pmadapter,
					    pmlan_ioctl_req pioctl_req)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	pmlan_private pmpriv = pmadapter->priv[pioctl_req->bss_index];
	mlan_ds_wmm_cfg *cfg = MNULL;

	ENTER();
	cfg = (mlan_ds_wmm_cfg *)pioctl_req->pbuf;

	/* Send request to firmware */
	ret = wlan_prepare_cmd(pmpriv, HostCmd_CMD_WMM_ADDTS_REQ, 0, 0,
			       (t_void *)pioctl_req,
			       (t_void *)&cfg->param.addts);

	if (ret == MLAN_STATUS_SUCCESS)
		ret = MLAN_STATUS_PENDING;

	LEAVE();
	return ret;
}

/**
 *  @brief Request for delete a TSPEC
 *
 *  @param pmadapter    A pointer to mlan_adapter structure
 *  @param pioctl_req   A pointer to ioctl request buffer
 *
 *  @return             MLAN_STATUS_PENDING --success, otherwise fail
 */
static mlan_status wlan_wmm_ioctl_delts_req(pmlan_adapter pmadapter,
					    pmlan_ioctl_req pioctl_req)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	pmlan_private pmpriv = pmadapter->priv[pioctl_req->bss_index];
	mlan_ds_wmm_cfg *cfg = MNULL;

	ENTER();
	cfg = (mlan_ds_wmm_cfg *)pioctl_req->pbuf;

	/* Send request to firmware */
	ret = wlan_prepare_cmd(pmpriv, HostCmd_CMD_WMM_DELTS_REQ, 0, 0,
			       (t_void *)pioctl_req,
			       (t_void *)&cfg->param.delts);

	if (ret == MLAN_STATUS_SUCCESS)
		ret = MLAN_STATUS_PENDING;

	LEAVE();
	return ret;
}

/**
 *  @brief Request for add a TSPEC
 *
 *  @param pmadapter    A pointer to mlan_adapter structure
 *  @param pioctl_req   A pointer to ioctl request buffer
 *
 *  @return             MLAN_STATUS_PENDING --success, otherwise fail
 */
static mlan_status wlan_wmm_ioctl_host_addts_req(pmlan_adapter pmadapter,
						 pmlan_ioctl_req pioctl_req)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	pmlan_private pmpriv = pmadapter->priv[pioctl_req->bss_index];
	mlan_ds_wmm_cfg *cfg = MNULL;

	ENTER();

	cfg = (mlan_ds_wmm_cfg *)pioctl_req->pbuf;

	/* Send request to firmware */
	ret = wlan_prepare_cmd(pmpriv, HostCmd_CMD_WMM_HOST_ADDTS_REQ, 0, 0,
			       (t_void *)pioctl_req,
			       (t_void *)&cfg->param.host_addts);

	if (ret == MLAN_STATUS_SUCCESS)
		ret = MLAN_STATUS_PENDING;

	LEAVE();
	return ret;
}

/**
 *  @brief Request for delete a TSPEC
 *
 *  @param pmadapter    A pointer to mlan_adapter structure
 *  @param pioctl_req   A pointer to ioctl request buffer
 *
 *  @return             MLAN_STATUS_PENDING --success, otherwise fail
 */
static mlan_status wlan_wmm_ioctl_host_delts_req(pmlan_adapter pmadapter,
						 pmlan_ioctl_req pioctl_req)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	pmlan_private pmpriv = pmadapter->priv[pioctl_req->bss_index];
	mlan_ds_wmm_cfg *cfg = MNULL;

	ENTER();
	cfg = (mlan_ds_wmm_cfg *)pioctl_req->pbuf;

	/* Send request to firmware */
	ret = wlan_prepare_cmd(pmpriv, HostCmd_CMD_WMM_HOST_DELTS_REQ, 0, 0,
			       (t_void *)pioctl_req,
			       (t_void *)&cfg->param.host_delts);

	if (ret == MLAN_STATUS_SUCCESS)
		ret = MLAN_STATUS_PENDING;

	LEAVE();
	return ret;
}

/**
 *  @brief To get and start/stop queue stats on a WMM AC
 *
 *  @param pmadapter    A pointer to mlan_adapter structure
 *  @param pioctl_req   A pointer to ioctl request buffer
 *
 *  @return             MLAN_STATUS_PENDING --success, otherwise fail
 */
static mlan_status wlan_wmm_ioctl_queue_stats(pmlan_adapter pmadapter,
					      pmlan_ioctl_req pioctl_req)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	pmlan_private pmpriv = pmadapter->priv[pioctl_req->bss_index];
	mlan_ds_wmm_cfg *cfg = MNULL;

	ENTER();
	cfg = (mlan_ds_wmm_cfg *)pioctl_req->pbuf;

	/* Send request to firmware */
	ret = wlan_prepare_cmd(pmpriv, HostCmd_CMD_WMM_QUEUE_STATS, 0, 0,
			       (t_void *)pioctl_req,
			       (t_void *)&cfg->param.q_stats);

	if (ret == MLAN_STATUS_SUCCESS)
		ret = MLAN_STATUS_PENDING;

	LEAVE();
	return ret;
}

/**
 *  @brief Get the status of the WMM AC queues
 *
 *  @param pmadapter    A pointer to mlan_adapter structure
 *  @param pioctl_req   A pointer to ioctl request buffer
 *
 *  @return             MLAN_STATUS_SUCCESS --success
 */
static mlan_status wlan_wmm_ioctl_queue_status(pmlan_adapter pmadapter,
					       pmlan_ioctl_req pioctl_req)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	pmlan_private pmpriv = pmadapter->priv[pioctl_req->bss_index];
	mlan_ds_wmm_cfg *cfg = MNULL;
	mlan_ds_wmm_queue_status *pqstatus = MNULL;
	WmmAcStatus_t *pac_status = MNULL;
	mlan_wmm_ac_e ac_idx;

	ENTER();

	cfg = (mlan_ds_wmm_cfg *)pioctl_req->pbuf;
	pqstatus = (mlan_ds_wmm_queue_status *)&cfg->param.q_status;

	for (ac_idx = WMM_AC_BK; ac_idx <= WMM_AC_VO; ac_idx++) {
		pac_status = &pmpriv->wmm.ac_status[ac_idx];

		/* Firmware status */
		pqstatus->ac_status[ac_idx].flow_required =
			pac_status->flow_required;
		pqstatus->ac_status[ac_idx].flow_created =
			pac_status->flow_created;
		pqstatus->ac_status[ac_idx].disabled = pac_status->disabled;

		/* ACM bit reflected in firmware status (redundant) */
		pqstatus->ac_status[ac_idx].wmm_acm = pac_status->flow_required;
	}

	LEAVE();
	return ret;
}

/**
 *  @brief Get the status of the WMM Traffic Streams
 *
 *  @param pmadapter    A pointer to mlan_adapter structure
 *  @param pioctl_req   A pointer to ioctl request buffer
 *
 *  @return             MLAN_STATUS_PENDING --success, otherwise fail
 */
static mlan_status wlan_wmm_ioctl_ts_status(pmlan_adapter pmadapter,
					    pmlan_ioctl_req pioctl_req)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	pmlan_private pmpriv = pmadapter->priv[pioctl_req->bss_index];
	mlan_ds_wmm_cfg *cfg = MNULL;

	ENTER();

	cfg = (mlan_ds_wmm_cfg *)pioctl_req->pbuf;

	/* Send request to firmware */
	ret = wlan_prepare_cmd(pmpriv, HostCmd_CMD_WMM_TS_STATUS, 0, 0,
			       (t_void *)pioctl_req,
			       (t_void *)&cfg->param.ts_status);

	if (ret == MLAN_STATUS_SUCCESS)
		ret = MLAN_STATUS_PENDING;

	LEAVE();
	return ret;
}
#endif /* STA_SUPPORT */

/**
 *  @brief This function prepares the command of WMM_PARAM_CONFIG
 *
 *  @param pmpriv       A pointer to mlan_private structure
 *  @param cmd          A pointer to HostCmd_DS_COMMAND structure
 *  @param cmd_action   cmd action.
 *  @param pdata_buf    A pointer to data buffer
 *  @return             MLAN_STATUS_SUCCESS
 */
mlan_status wlan_cmd_wmm_param_config(pmlan_private pmpriv,
				      HostCmd_DS_COMMAND *cmd, t_u16 cmd_action,
				      t_void *pdata_buf)
{
	wmm_ac_parameters_t *ac_params = (wmm_ac_parameters_t *)pdata_buf;
	HostCmd_DS_WMM_PARAM_CONFIG *pcmd_cfg = &cmd->params.param_config;
	t_u8 i = 0;

	ENTER();

	cmd->command = wlan_cpu_to_le16(HostCmd_CMD_WMM_PARAM_CONFIG);
	cmd->size = wlan_cpu_to_le16(sizeof(HostCmd_DS_WMM_PARAM_CONFIG) +
				     S_DS_GEN);
	cmd->result = 0;

	pcmd_cfg->action = cmd_action;
	if (cmd_action == HostCmd_ACT_GEN_SET) {
		memcpy_ext(pmpriv->adapter, pcmd_cfg->ac_params, ac_params,
			   sizeof(wmm_ac_parameters_t) * MAX_AC_QUEUES,
			   sizeof(wmm_ac_parameters_t) * MAX_AC_QUEUES);
		for (i = 0; i < MAX_AC_QUEUES; i++) {
			pcmd_cfg->ac_params[i].tx_op_limit = wlan_cpu_to_le16(
				pcmd_cfg->ac_params[i].tx_op_limit);
		}
	}
	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function handles the command response of WMM_PARAM_CONFIG
 *
 *  @param pmpriv       A pointer to mlan_private structure
 *  @param resp         A pointer to HostCmd_DS_COMMAND
 *  @param pioctl_buf   A pointer to mlan_ioctl_req structure
 *
 *  @return             MLAN_STATUS_SUCCESS
 */
mlan_status wlan_ret_wmm_param_config(pmlan_private pmpriv,
				      const HostCmd_DS_COMMAND *resp,
				      mlan_ioctl_req *pioctl_buf)
{
	mlan_ds_wmm_cfg *pwmm = MNULL;
	const HostCmd_DS_WMM_PARAM_CONFIG *pcfg =
		(const HostCmd_DS_WMM_PARAM_CONFIG *)&resp->params.param_config;
	t_u8 i;

	ENTER();

	if (pioctl_buf) {
		pwmm = (mlan_ds_wmm_cfg *)pioctl_buf->pbuf;
		memcpy_ext(pmpriv->adapter, pwmm->param.ac_params,
			   pcfg->ac_params,
			   sizeof(wmm_ac_parameters_t) * MAX_AC_QUEUES,
			   sizeof(wmm_ac_parameters_t) * MAX_AC_QUEUES);
		for (i = 0; i < MAX_AC_QUEUES; i++) {
			pwmm->param.ac_params[i].tx_op_limit = wlan_le16_to_cpu(
				pwmm->param.ac_params[i].tx_op_limit);
		}
	}

	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function prepares the command of WMM_QUEUE_CONFIG
 *
 *  @param pmpriv       A pointer to mlan_private structure
 *  @param cmd          A pointer to HostCmd_DS_COMMAND structure
 *  @param pdata_buf    A pointer to data buffer
 *  @return             MLAN_STATUS_SUCCESS
 */
mlan_status wlan_cmd_wmm_queue_config(pmlan_private pmpriv,
				      HostCmd_DS_COMMAND *cmd,
				      t_void *pdata_buf)
{
	mlan_ds_wmm_queue_config *pqcfg = (mlan_ds_wmm_queue_config *)pdata_buf;
	HostCmd_DS_WMM_QUEUE_CONFIG *pcmd_qcfg = &cmd->params.queue_config;

	ENTER();

	cmd->command = wlan_cpu_to_le16(HostCmd_CMD_WMM_QUEUE_CONFIG);
	cmd->size = wlan_cpu_to_le16(
		sizeof(pcmd_qcfg->action) + sizeof(pcmd_qcfg->access_category) +
		sizeof(pcmd_qcfg->msdu_lifetime_expiry) + S_DS_GEN);
	cmd->result = 0;

	pcmd_qcfg->action = pqcfg->action;
	pcmd_qcfg->access_category = pqcfg->access_category;
	pcmd_qcfg->msdu_lifetime_expiry =
		wlan_cpu_to_le16(pqcfg->msdu_lifetime_expiry);

	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function handles the command response of WMM_QUEUE_CONFIG
 *
 *  @param pmpriv       A pointer to mlan_private structure
 *  @param resp         A pointer to HostCmd_DS_COMMAND
 *  @param pioctl_buf   A pointer to mlan_ioctl_req structure
 *
 *  @return             MLAN_STATUS_SUCCESS
 */
mlan_status wlan_ret_wmm_queue_config(pmlan_private pmpriv,
				      const HostCmd_DS_COMMAND *resp,
				      mlan_ioctl_req *pioctl_buf)
{
	mlan_ds_wmm_cfg *pwmm = MNULL;
	const HostCmd_DS_WMM_QUEUE_CONFIG *presp_qcfg =
		&resp->params.queue_config;

	ENTER();

	if (pioctl_buf) {
		pwmm = (mlan_ds_wmm_cfg *)pioctl_buf->pbuf;
		pwmm->param.q_cfg.action = wlan_le32_to_cpu(presp_qcfg->action);
		pwmm->param.q_cfg.access_category =
			wlan_le32_to_cpu(presp_qcfg->access_category);
		pwmm->param.q_cfg.msdu_lifetime_expiry =
			wlan_le16_to_cpu(presp_qcfg->msdu_lifetime_expiry);
	}

	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief Set/Get a specified AC Queue's parameters
 *
 *  @param pmadapter    A pointer to mlan_adapter structure
 *  @param pioctl_req   A pointer to ioctl request buffer
 *
 *  @return             MLAN_STATUS_PENDING --success, otherwise fail
 */
static mlan_status wlan_wmm_ioctl_queue_config(pmlan_adapter pmadapter,
					       pmlan_ioctl_req pioctl_req)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	pmlan_private pmpriv = pmadapter->priv[pioctl_req->bss_index];
	mlan_ds_wmm_cfg *cfg = MNULL;

	ENTER();
	cfg = (mlan_ds_wmm_cfg *)pioctl_req->pbuf;

	/* Send request to firmware */
	ret = wlan_prepare_cmd(pmpriv, HostCmd_CMD_WMM_QUEUE_CONFIG, 0, 0,
			       (t_void *)pioctl_req,
			       (t_void *)&cfg->param.q_cfg);

	if (ret == MLAN_STATUS_SUCCESS)
		ret = MLAN_STATUS_PENDING;

	LEAVE();
	return ret;
}

/**
 *  @brief WMM configuration handler
 *
 *  @param pmadapter   A pointer to mlan_adapter structure
 *  @param pioctl_req A pointer to ioctl request buffer
 *
 *  @return     MLAN_STATUS_SUCCESS --success, otherwise fail
 */
mlan_status wlan_wmm_cfg_ioctl(pmlan_adapter pmadapter,
			       pmlan_ioctl_req pioctl_req)
{
	mlan_status status = MLAN_STATUS_SUCCESS;
	mlan_ds_wmm_cfg *wmm = MNULL;

	ENTER();

	if (pioctl_req->buf_len < sizeof(mlan_ds_wmm_cfg)) {
		PRINTM(MWARN, "MLAN bss IOCTL length is too short.\n");
		pioctl_req->data_read_written = 0;
		pioctl_req->buf_len_needed = sizeof(mlan_ds_wmm_cfg);
		pioctl_req->status_code = MLAN_ERROR_INVALID_PARAMETER;
		LEAVE();
		return MLAN_STATUS_RESOURCE;
	}
	wmm = (mlan_ds_wmm_cfg *)pioctl_req->pbuf;
	switch (wmm->sub_command) {
#ifdef STA_SUPPORT
	case MLAN_OID_WMM_CFG_ENABLE:
		status = wlan_wmm_ioctl_enable(pmadapter, pioctl_req);
		break;
	case MLAN_OID_WMM_CFG_QOS:
		status = wlan_wmm_ioctl_qos(pmadapter, pioctl_req);
		break;
	case MLAN_OID_WMM_CFG_ADDTS:
		status = wlan_wmm_ioctl_addts_req(pmadapter, pioctl_req);
		break;
	case MLAN_OID_WMM_CFG_DELTS:
		status = wlan_wmm_ioctl_delts_req(pmadapter, pioctl_req);
		break;
	case MLAN_OID_WMM_CFG_HOST_ADDTS:
		status = wlan_wmm_ioctl_host_addts_req(pmadapter, pioctl_req);
		break;
	case MLAN_OID_WMM_CFG_HOST_DELTS:
		status = wlan_wmm_ioctl_host_delts_req(pmadapter, pioctl_req);
		break;
	case MLAN_OID_WMM_CFG_QUEUE_STATS:
		status = wlan_wmm_ioctl_queue_stats(pmadapter, pioctl_req);
		break;
	case MLAN_OID_WMM_CFG_QUEUE_STATUS:
		status = wlan_wmm_ioctl_queue_status(pmadapter, pioctl_req);
		break;
	case MLAN_OID_WMM_CFG_TS_STATUS:
		status = wlan_wmm_ioctl_ts_status(pmadapter, pioctl_req);
		break;
#endif
	case MLAN_OID_WMM_CFG_QUEUE_CONFIG:
		status = wlan_wmm_ioctl_queue_config(pmadapter, pioctl_req);
		break;
	default:
		pioctl_req->status_code = MLAN_ERROR_IOCTL_INVALID;
		status = MLAN_STATUS_FAILURE;
		break;
	}
	LEAVE();
	return status;
}

/**
 *  @brief Get ralist info
 *
 *  @param priv         A pointer to mlan_private structure
 *  @param buf          A pointer to ralist_info structure
 *  @return             number of ralist entry
 *
 */
int wlan_get_ralist_info(mlan_private *priv, ralist_info *buf)
{
	ralist_info *plist = buf;
	mlan_list_head *ra_list_head = MNULL;
	raListTbl *ra_list;
	int i;
	int count = 0;
	for (i = 0; i < MAX_NUM_TID; i++) {
		ra_list_head = &priv->wmm.tid_tbl_ptr[i].ra_list;
		ra_list =
			(raListTbl *)util_peek_list(priv->adapter->pmoal_handle,
						    ra_list_head, MNULL, MNULL);
		while (ra_list && ra_list != (raListTbl *)ra_list_head) {
			if (ra_list->total_pkts) {
				plist->total_pkts = ra_list->total_pkts;
				plist->tid = i;
				plist->tx_pause = ra_list->tx_pause;
				memcpy_ext(priv->adapter, plist->ra,
					   ra_list->ra, MLAN_MAC_ADDR_LENGTH,
					   MLAN_MAC_ADDR_LENGTH);

				if (((t_u8 *)plist - (t_u8 *)buf) >=
				    (MLAN_MAX_RALIST_NUM * sizeof(ralist_info)))
					break;
				plist++;
				count++;
				if (count >= MLAN_MAX_RALIST_NUM)
					break;
			}
			ra_list = ra_list->pnext;
		}
	}
	LEAVE();
	return count;
}

/**
 *  @brief dump ralist info
 *
 *  @param priv         A pointer to mlan_private structure
 *
 *  @return             N/A
 *
 */
void wlan_dump_ralist(mlan_private *priv)
{
	mlan_list_head *ra_list_head = MNULL;
	raListTbl *ra_list;
	mlan_adapter *pmadapter = priv->adapter;
	int i;
	t_u32 tx_pkts_queued;

	tx_pkts_queued =
		util_scalar_read(pmadapter->pmoal_handle,
				 &priv->wmm.tx_pkts_queued, MNULL, MNULL);
	PRINTM(MERROR, "bss_index = %d, tx_pkts_queued = %d tx_pause\n",
	       priv->bss_index, tx_pkts_queued, priv->tx_pause);
	if (!tx_pkts_queued)
		return;
	for (i = 0; i < MAX_NUM_TID; i++) {
		ra_list_head = &priv->wmm.tid_tbl_ptr[i].ra_list;
		ra_list =
			(raListTbl *)util_peek_list(priv->adapter->pmoal_handle,
						    ra_list_head, MNULL, MNULL);
		while (ra_list && ra_list != (raListTbl *)ra_list_head) {
			if (ra_list->total_pkts) {
				PRINTM(MERROR,
				       "ralist ra: %02x:%02x:%02x:%02x:%02x:%02x tid=%d pkts=%d pause=%d\n",
				       ra_list->ra[0], ra_list->ra[1],
				       ra_list->ra[2], ra_list->ra[3],
				       ra_list->ra[4], ra_list->ra[5], i,
				       ra_list->total_pkts, ra_list->tx_pause);
			}
			ra_list = ra_list->pnext;
		}
	}
	return;
}

/**
 *  @brief get tid down
 *
 *  @param priv         A pointer to mlan_private structure
 * 	@param tid 			tid
 *
 *  @return             tid_down
 *
 */
int wlan_get_wmm_tid_down(mlan_private *priv, int tid)
{
	return wlan_wmm_downgrade_tid(priv, tid);
}

#define ieee_mbit_rate(float_value) ((t_u32)(float_value * 1000u))

/**
 *  @brief The function gets PHY rate in kbit/s for HE PPDU.
 *
 *  @param bw     PPDU bandwith
 *  @param gi     GI type
 *  @param nss    number of spatial streams
 *  @param mcs    MCS
 *
 *  @return            PHY rate in kbit per second
 */
static t_u32 wlam_wmm_get_he_rate(t_u32 bw, t_u32 gi, t_u32 nss, t_u32 mcs)
{
	const t_u32 gi_1x_0p8 = 0;
	const t_u32 gi_2x_0p8 = 1;
	const t_u32 gi_2x_1p6 = 2;

	const t_u32 bw_20 = 0;
	const t_u32 bw_40 = 1;
	const t_u32 bw_160 = 3;
	t_u32 rate;

	static const t_u32 he_80_3p2_gi_mcS_to_rate[] = {
		/* intentional float value multiplication with 1000 */
		// coverity[misra_c_2012_rule_10_8_violation:SUPPRESS]
		ieee_mbit_rate(30.6),
		// coverity[misra_c_2012_rule_10_8_violation:SUPPRESS]
		ieee_mbit_rate(61.3),
		// coverity[misra_c_2012_rule_10_8_violation:SUPPRESS]
		ieee_mbit_rate(91.9),
		// coverity[misra_c_2012_rule_10_8_violation:SUPPRESS]
		ieee_mbit_rate(122.5),
		// coverity[misra_c_2012_rule_10_8_violation:SUPPRESS]
		ieee_mbit_rate(183.8), ieee_mbit_rate(245),
		// coverity[misra_c_2012_rule_10_8_violation:SUPPRESS]
		ieee_mbit_rate(275.6),
		// coverity[misra_c_2012_rule_10_8_violation:SUPPRESS]
		ieee_mbit_rate(306.3),
		// coverity[misra_c_2012_rule_10_8_violation:SUPPRESS]
		ieee_mbit_rate(367.5),
		// coverity[misra_c_2012_rule_10_8_violation:SUPPRESS]
		ieee_mbit_rate(408.3),
		// coverity[misra_c_2012_rule_10_8_violation:SUPPRESS]
		ieee_mbit_rate(459.4),
		// coverity[misra_c_2012_rule_10_8_violation:SUPPRESS]
		ieee_mbit_rate(510.4)};

	if (mcs >= NELEMENTS(he_80_3p2_gi_mcS_to_rate))
		return 0;

	rate = he_80_3p2_gi_mcS_to_rate[mcs];

	// 11ax scaling rules
	// 1.6_gi_rate = 0.8_gi_rate / 1.059
	// 3.2_gi_rate = 1.6_gi_rate / 1.112
	// 160mhz_rate = 80mhz_rate * 2.0
	// 40mhz_rate = 80mhz_rate / 2.09
	// 20mhz_rate = 40mhz_rate / 2

	if (gi == gi_1x_0p8 || gi == gi_2x_0p8) {
		rate = (rate * 1112u) / 1000u;
		rate = (rate * 1059u) / 1000u;
	} else if (gi == gi_2x_1p6) {
		rate = (rate * 1112u) / 1000u;
	}

	if (bw == bw_20) {
		rate = (rate * 1000u) / 2090u;
		rate /= 2;
	} else if (bw == bw_40) {
		rate = (rate * 1000u) / 2090u;
	} else if (bw == bw_160) {
		rate *= 2;
	}

	rate = rate * nss;

	// coverity[integer_overflow:SUPPRESS]
	return rate;
}

/**
 *  @brief The function gets PHY rate in kbit/s for VHT PPDU.
 *
 *  @param bw     PPDU bandwith
 *  @param sgi    short GI
 *  @param nss    number of spatial streams
 *  @param mcs    MCS
 *
 *  @return            PHY rate in kbit per second
 */
static t_u32 wlam_wmm_get_vht_rate(t_u32 bw, t_u32 sgi, t_u32 nss, t_u32 mcs)
{
	const t_u32 bw_20 = 0;
	const t_u32 bw_40 = 1;
	const t_u32 bw_160 = 3;
	t_u32 rate;

	static const t_u32 vht_80_lgi_mcs_to_rate[] = {
		/* intentional float value multiplication with 1000 */
		// coverity[misra_c_2012_rule_10_8_violation:SUPPRESS]
		ieee_mbit_rate(29.3),
		// coverity[misra_c_2012_rule_10_8_violation:SUPPRESS]
		ieee_mbit_rate(58.5),
		// coverity[misra_c_2012_rule_10_8_violation:SUPPRESS]
		ieee_mbit_rate(87.8),
		// coverity[misra_c_2012_rule_10_8_violation:SUPPRESS]
		ieee_mbit_rate(117.0),
		// coverity[misra_c_2012_rule_10_8_violation:SUPPRESS]
		ieee_mbit_rate(175.5),
		// coverity[misra_c_2012_rule_10_8_violation:SUPPRESS]
		ieee_mbit_rate(234.0),
		// coverity[misra_c_2012_rule_10_8_violation:SUPPRESS]
		ieee_mbit_rate(263.3),
		// coverity[misra_c_2012_rule_10_8_violation:SUPPRESS]
		ieee_mbit_rate(292.5),
		// coverity[misra_c_2012_rule_10_8_violation:SUPPRESS]
		ieee_mbit_rate(351.0),
		// coverity[misra_c_2012_rule_10_8_violation:SUPPRESS]
		ieee_mbit_rate(390.0)};

	if (mcs >= NELEMENTS(vht_80_lgi_mcs_to_rate))
		return 0;

	rate = vht_80_lgi_mcs_to_rate[mcs];

	// 11ac scaling rules:
	// sgi_rate = lgi_rate * 1.111
	// 160mhz_rate = 80_mhz_rate * 2.0
	// 40mhz_rate = 80_mhz_rate / 2.166
	// 20mhz_rate = 40mhz_rate / 2.077
	// XSS_rate = 1ss_rate * NSS
	if (sgi)
		rate = (rate * 1111) / 1000;

	if (bw == bw_20) {
		rate = (rate * 1000u) / 2166u;
		rate = (rate * 1000u) / 2077u;
	} else if (bw == bw_40) {
		rate = (rate * 1000u) / 2166u;
	} else if (bw == bw_160) {
		rate *= 2;
	}

	rate = rate * nss;

	// coverity[integer_overflow:SUPPRESS]
	return rate;
}

/**
 *  @brief The function gets PHY rate in kbit/s for HT PPDU.
 *
 *  @param bw     PPDU bandwith
 *  @param sgi    short GI
 *  @param mcs    MCS
 *
 *  @return            PHY rate in kbit per second
 */
static t_u32 wlam_wmm_get_ht_rate(t_u32 bw, t_u32 sgi, t_u32 mcs)
{
	const t_u32 bw_40 = 1;
	t_u32 rate;

	static const t_u32 ht_20_lgi_mcs_to_rate[] = {
		/* intentional float value multiplication with 1000 */
		// coverity[misra_c_2012_rule_10_8_violation:SUPPRESS]
		ieee_mbit_rate(6.5),
		ieee_mbit_rate(13),
		// coverity[misra_c_2012_rule_10_8_violation:SUPPRESS]
		ieee_mbit_rate(19.5),
		ieee_mbit_rate(26),
		ieee_mbit_rate(39),
		ieee_mbit_rate(52),
		// coverity[misra_c_2012_rule_10_8_violation:SUPPRESS]
		ieee_mbit_rate(58.5),
		ieee_mbit_rate(65),
		ieee_mbit_rate(13),
		ieee_mbit_rate(26),
		ieee_mbit_rate(39),
		ieee_mbit_rate(52),
		ieee_mbit_rate(78),
		ieee_mbit_rate(104),
		ieee_mbit_rate(117),
		ieee_mbit_rate(130),
	};

	if (mcs >= NELEMENTS(ht_20_lgi_mcs_to_rate))
		return 0;

	rate = ht_20_lgi_mcs_to_rate[mcs];

	// 11n scaling rules:
	// sgi_rate = lgi_rate * 1.111
	// 40mhz_rate = 20_mhz_rate * 2.077

	if (sgi)
		rate = (rate * 1111) / 1000;

	if (bw == bw_40) {
		rate = (rate * 2077u) / 1000;
	}

	// coverity[integer_overflow:SUPPRESS]
	return rate;
}

/**
 *  @brief The function gets PHY rate in kbit/s for legacy PPDU.
 *
 *  @param rate_idx    rate index
 *
 *  @return            PHY rate in kbit per second
 */
static t_u32 wlam_wmm_get_legacy_rate(t_u32 rate_idx)
{
	static const t_u32 legacy_rate_idx_to_rate[] = {
		/* intentional float value multiplication with 1000 */
		ieee_mbit_rate(1),
		ieee_mbit_rate(2),
		// coverity[misra_c_2012_rule_10_8_violation:SUPPRESS]
		ieee_mbit_rate(5.5),
		ieee_mbit_rate(11),
		ieee_mbit_rate(22),
		ieee_mbit_rate(6),
		ieee_mbit_rate(9),
		ieee_mbit_rate(12),
		ieee_mbit_rate(18),
		ieee_mbit_rate(24),
		ieee_mbit_rate(36),
		ieee_mbit_rate(48),
		ieee_mbit_rate(54),
		ieee_mbit_rate(72),
	};

	if (rate_idx >= NELEMENTS(legacy_rate_idx_to_rate))
		return 0;

	return legacy_rate_idx_to_rate[rate_idx];
}

/**
 *  @brief The function sets byte tx_budget based on currect TX rate and
 *          allocated airtime.
 *
 *  @param priv    Pointer to the pmlan_private driver data struct
 *  @param sta     Pointer to the wmm_sta_table data struct
 *  @param rate    STA`s TX rate
 *
 *  @return N/A
 */
static void wlan_wmm_adjust_sta_tx_budget(pmlan_private priv,
					  struct wmm_sta_table *sta,
					  HostCmd_TX_RATE_QUERY *rate)
{
	mlan_adapter *pmadapter = priv->adapter;
	const t_u8 ppdu_type_legacy = 0;
	const t_u8 ppdu_type_ht = 1;
	const t_u8 ppdu_type_vht = 2;
	const t_u8 ppdu_type_he = 3;
	t_u32 phy_rate = 0;
	t_u8 nss = (rate->tx_rate >> 4) + 1;
	t_u8 mcs = (rate->tx_rate) & 0x0f;
	t_u8 ppdu_format = rate->tx_rate_info & 0x3;
	t_u8 ppdu_bw = (rate->tx_rate_info >> 2) & 0x3;
	t_u8 ru_size = (rate->ext_tx_rate_info >> 1) & 0x7;
	t_u8 dcm = (rate->ext_tx_rate_info) & 0x1;
	t_u8 gi = (rate->tx_rate_info >> 4) & 0x1;

	gi |= (rate->tx_rate_info >> 6) & 0x02;

	if (ppdu_format == ppdu_type_he)
		phy_rate = wlam_wmm_get_he_rate(ppdu_bw, gi, nss, mcs);
	else if (ppdu_format == ppdu_type_vht)
		phy_rate = wlam_wmm_get_vht_rate(ppdu_bw, gi, nss, mcs);
	else if (ppdu_format == ppdu_type_ht)
		phy_rate = wlam_wmm_get_ht_rate(ppdu_bw, gi, rate->tx_rate);
	else if (ppdu_format == ppdu_type_legacy)
		phy_rate = wlam_wmm_get_legacy_rate(rate->tx_rate);

	if (phy_rate > 0) {
		const t_u32 old_phy_rate = sta->budget.phy_rate_kbps;
		sta->budget.byte_budget_init = wlan_wmm_get_byte_budget(
			pmadapter, sta->budget.time_budget_init_us, phy_rate);
		sta->budget.phy_rate_kbps = phy_rate;

		if (old_phy_rate / phy_rate >= 2 ||
		    phy_rate / old_phy_rate >= 2) {
			PRINTM(MWARN,
			       "mclient: %pM rate jump %u -> %u, phy type %u\n",
			       sta->ra, old_phy_rate, phy_rate, ppdu_format);
		}
	}

	if (phy_rate == 0) {
		PRINTM(MERROR,
		       "mclient_error: invalid rate %u, tx_rate %u, nss %u, mcs %u, ppdu_format %u, ppdu_bw %u, gi %u, dcm %u, ru_size %u, budget %d sta %pM\n",
		       phy_rate, rate->tx_rate, nss, mcs, ppdu_format, ppdu_bw,
		       gi, dcm, ru_size, sta->budget.byte_budget_init, sta->ra);
	}
}

/**
 *  @brief The function is called when TX rate command response received
 *
 *  @param priv    Pointer to the pmlan_private driver data struct
 *  @param mac     STA`s MAC
 *  @param rate    STA`s TX rate
 *
 *  @return N/A
 */
void wlan_wmm_update_sta_tx_rate(pmlan_private priv, t_u8 *mac,
				 HostCmd_TX_RATE_QUERY *rate)
{
	struct wmm_sta_table *sta = wlan_wmm_get_sta(priv, mac);

	if (sta) {
		priv->wmm.is_rate_update_pending = MFALSE;
		wlan_wmm_adjust_sta_tx_budget(priv, sta, rate);
	}
}

/**
 *  @brief The function is called when PS state change event is received
 *
 *  @param priv    Pointer to the pmlan_private driver data struct
 *  @param mac     STA`s MAC
 *  @param sleep   NTRUE is STA is in IEEE PS mode
 *
 *  @return N/A
 */
void wlan_update_sta_ps_state(pmlan_private priv, t_u8 *mac, t_u8 sleep)
{
	mlan_adapter *pmadapter = priv->adapter;
	struct wmm_sta_table *sta;
	t_bool resume_sta = MFALSE;
	mlan_callbacks *cbs = &pmadapter->callbacks;

	cbs->moal_spin_lock(pmadapter->pmoal_handle,
			    priv->wmm.ra_list_spinlock);

	sta = wlan_wmm_get_sta(priv, mac);
	if (sta) {
		if (!sleep)
			resume_sta = MTRUE;

		if (sta->ps_sleep == sleep) {
			PRINTM(MWARN,
			       "mclient-err: %pM already in same state %u\n",
			       sta->ra, sta->ps_sleep);
		}

		sta->ps_sleep = sleep;
	}

	cbs->moal_spin_unlock(pmadapter->pmoal_handle,
			      priv->wmm.ra_list_spinlock);
	if (resume_sta)
		wlan_update_ralist_tx_pause(priv, mac, 0);
}
