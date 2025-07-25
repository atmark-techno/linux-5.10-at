/** @file mlan_pcie.c
 *
 *  @brief This file contains PCI-E specific code
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
    02/01/2012: initial version
********************************************************/

#include "mlan.h"
#ifdef STA_SUPPORT
#include "mlan_join.h"
#endif
#include "mlan_util.h"
#include "mlan_fw.h"
#include "mlan_main.h"
#include "mlan_init.h"
#include "mlan_wmm.h"
#include "mlan_11n.h"
#include "mlan_pcie.h"

/********************************************************
			Local Variables
********************************************************/
#ifdef PCIE8897
static const struct _mlan_pcie_card_reg mlan_reg_pcie8897 = {
	.reg_txbd_rdptr = PCIE8897_RD_DATA_PTR_Q0_Q1,
	.reg_txbd_wrptr = PCIE8897_WR_DATA_PTR_Q0_Q1,
	.reg_rxbd_rdptr = PCIE8897_RD_DATA_PTR_Q0_Q1,
	.reg_rxbd_wrptr = PCIE8897_WR_DATA_PTR_Q0_Q1,
	.reg_evtbd_rdptr = REG_EVTBD_RDPTR,
	.reg_evtbd_wrptr = REG_EVTBD_WRPTR,
	.reg_host_int_mask = PCIE_HOST_INT_MASK,
	.reg_host_int_status_mask = PCIE_HOST_INT_STATUS_MASK,
	.reg_host_int_status = PCIE_HOST_INT_STATUS,
	.reg_cpu_int_event = PCIE_CPU_INT_EVENT,
	.reg_ip_rev = PCIE_IP_REV_REG,
	.reg_drv_ready = REG_DRV_READY,
	.reg_cpu_int_status = PCIE_CPU_INT_STATUS,
	.reg_scratch_0 = PCIE_SCRATCH_0_REG,
	.reg_scratch_1 = PCIE_SCRATCH_1_REG,
	.reg_scratch_2 = PCIE_SCRATCH_2_REG,
	.reg_scratch_3 = PCIE_SCRATCH_3_REG,
	.host_intr_mask = HOST_INTR_MASK,
	.host_intr_dnld_done = HOST_INTR_DNLD_DONE,
	.host_intr_upld_rdy = HOST_INTR_UPLD_RDY,
	.host_intr_cmd_done = HOST_INTR_CMD_DONE,
	.host_intr_event_rdy = HOST_INTR_EVENT_RDY,
	.txrx_rw_ptr_mask = 0x000003FF,
	.txrx_rw_ptr_wrap_mask = 0x000007FF,
	.txrx_rw_ptr_rollover_ind = MBIT(10),
	.use_adma = MFALSE,
	.msi_int_wr_clr = MTRUE,
};

static const struct _mlan_card_info mlan_card_info_pcie8897 = {
	.max_tx_buf_size = MLAN_TX_DATA_BUF_SIZE_4K,
	.v16_fw_api = 0,
	.supp_ps_handshake = 0,
	.default_11n_tx_bf_cap = DEFAULT_11N_TX_BF_CAP_2X2,
	.support_11mc = 0,
};
#endif

#ifdef PCIE8997
static const struct _mlan_pcie_card_reg mlan_reg_pcie8997 = {
	.reg_txbd_rdptr = PCIE8997_RD_DATA_PTR_Q0_Q1,
	.reg_txbd_wrptr = PCIE8997_WR_DATA_PTR_Q0_Q1,
	.reg_rxbd_rdptr = PCIE8997_RD_DATA_PTR_Q0_Q1,
	.reg_rxbd_wrptr = PCIE8997_WR_DATA_PTR_Q0_Q1,
	.reg_evtbd_rdptr = REG_EVTBD_RDPTR,
	.reg_evtbd_wrptr = REG_EVTBD_WRPTR,
	.reg_host_int_mask = PCIE_HOST_INT_MASK,
	.reg_host_int_status_mask = PCIE_HOST_INT_STATUS_MASK,
	.reg_host_int_status = PCIE_HOST_INT_STATUS,
	.reg_cpu_int_event = PCIE_CPU_INT_EVENT,
	.reg_ip_rev = PCIE_IP_REV_REG,
	.reg_drv_ready = REG_DRV_READY,
	.reg_cpu_int_status = PCIE_CPU_INT_STATUS,
	.reg_scratch_0 = PCIE_SCRATCH_0_REG,
	.reg_scratch_1 = PCIE_SCRATCH_1_REG,
	.reg_scratch_2 = PCIE_SCRATCH_2_REG,
	.reg_scratch_3 = PCIE_SCRATCH_3_REG,
	.host_intr_mask = HOST_INTR_MASK,
	.host_intr_dnld_done = HOST_INTR_DNLD_DONE,
	.host_intr_upld_rdy = HOST_INTR_UPLD_RDY,
	.host_intr_cmd_done = HOST_INTR_CMD_DONE,
	.host_intr_event_rdy = HOST_INTR_EVENT_RDY,
	.host_intr_cmd_dnld = HOST_INTR_CMD_DNLD,
	.txrx_rw_ptr_mask = 0x00000FFF,
	.txrx_rw_ptr_wrap_mask = 0x00001FFF,
	.txrx_rw_ptr_rollover_ind = MBIT(12),
	.use_adma = MFALSE,
	.msi_int_wr_clr = MTRUE,
};

static const struct _mlan_card_info mlan_card_info_pcie8997 = {
	.max_tx_buf_size = MLAN_TX_DATA_BUF_SIZE_4K,
	.v16_fw_api = 1,
	.supp_ps_handshake = 0,
	.default_11n_tx_bf_cap = DEFAULT_11N_TX_BF_CAP_2X2,
	.support_11mc = 1,
};
#endif

#ifdef PCIEAW693
static const struct _mlan_pcie_card_reg mlan_reg_pcieaw693 = {
	.reg_txbd_rdptr = PCIE9098_TXBD_RDPTR,
	.reg_txbd_wrptr = PCIE9098_TXBD_WRPTR,
	.reg_rxbd_rdptr = PCIE9098_RXBD_RDPTR,
	.reg_rxbd_wrptr = PCIE9098_RXBD_WRPTR,
	.reg_evtbd_rdptr = PCIE9098_EVTBD_RDPTR,
	.reg_evtbd_wrptr = PCIE9098_EVTBD_WRPTR,
	.reg_host_int_mask = PCIE9097_B0_HOST_INT_MASK,
	.reg_host_int_status_mask = PCIE9097_B0_HOST_INT_STATUS_MASK,
	.reg_host_int_status = PCIE9097_B0_HOST_INT_STATUS,
	.reg_host_int_clr_sel = PCIE9097_B0_HOST_INT_CLR_SEL,
	.reg_cpu_int_event = PCIE9098_CPU_INT_EVENT,
	.reg_ip_rev = PCIE9098_DEV_ID_REG,
	.reg_drv_ready = PCIE9098_DRV_READY,
	.reg_cpu_int_status = PCIE9098_CPU_INT_STATUS,
	.reg_rev_id = PCIE9098_REV_ID_REG,
	.reg_scratch_0 = PCIE9098_SCRATCH_0_REG,
	.reg_scratch_1 = PCIE9098_SCRATCH_1_REG,
	.reg_scratch_2 = PCIE9098_SCRATCH_2_REG,
	.reg_scratch_3 = PCIE9098_SCRATCH_3_REG,
	.reg_scratch_6 = PCIE9098_SCRATCH_6_REG,
	.reg_scratch_7 = PCIE9098_SCRATCH_7_REG,
	.host_intr_mask = PCIE9098_HOST_INTR_MASK,
	.host_intr_dnld_done = PCIE9098_HOST_INTR_DNLD_DONE,
	.host_intr_upld_rdy = PCIE9098_HOST_INTR_UPLD_RDY,
	.host_intr_cmd_done = PCIE9098_HOST_INTR_CMD_DONE,
	.host_intr_event_rdy = PCIE9098_HOST_INTR_EVENT_RDY,
	.host_intr_cmd_dnld = PCIE9098_HOST_INTR_CMD_DNLD,
	.adma_align_size = ADMA_ALIGN_SIZE_1,
	.adma_min_pkt_size = ADMA_MIN_PKT_SIZE_32,
	.use_adma = MTRUE,
	.msi_int_wr_clr = MTRUE,
};
#endif

#if defined(PCIE9098) || defined(PCIE9097) || defined(PCIEIW624)
static const struct _mlan_pcie_card_reg mlan_reg_pcie9097_b0 = {
	.reg_txbd_rdptr = PCIE9098_TXBD_RDPTR,
	.reg_txbd_wrptr = PCIE9098_TXBD_WRPTR,
	.reg_rxbd_rdptr = PCIE9098_RXBD_RDPTR,
	.reg_rxbd_wrptr = PCIE9098_RXBD_WRPTR,
	.reg_evtbd_rdptr = PCIE9098_EVTBD_RDPTR,
	.reg_evtbd_wrptr = PCIE9098_EVTBD_WRPTR,
	.reg_host_int_mask = PCIE9097_B0_HOST_INT_MASK,
	.reg_host_int_status_mask = PCIE9097_B0_HOST_INT_STATUS_MASK,
	.reg_host_int_status = PCIE9097_B0_HOST_INT_STATUS,
	.reg_host_int_clr_sel = PCIE9097_B0_HOST_INT_CLR_SEL,
	.reg_cpu_int_event = PCIE9098_CPU_INT_EVENT,
	.reg_ip_rev = PCIE9098_DEV_ID_REG,
	.reg_drv_ready = PCIE9098_DRV_READY,
	.reg_cpu_int_status = PCIE9098_CPU_INT_STATUS,
	.reg_rev_id = PCIE9098_REV_ID_REG,
	.reg_scratch_0 = PCIE9098_SCRATCH_0_REG,
	.reg_scratch_1 = PCIE9098_SCRATCH_1_REG,
	.reg_scratch_2 = PCIE9098_SCRATCH_2_REG,
	.reg_scratch_3 = PCIE9098_SCRATCH_3_REG,
	.reg_scratch_6 = PCIE9098_SCRATCH_6_REG,
	.reg_scratch_7 = PCIE9098_SCRATCH_7_REG,
	.host_intr_mask = PCIE9098_HOST_INTR_MASK,
	.host_intr_dnld_done = PCIE9098_HOST_INTR_DNLD_DONE,
	.host_intr_upld_rdy = PCIE9098_HOST_INTR_UPLD_RDY,
	.host_intr_cmd_done = PCIE9098_HOST_INTR_CMD_DONE,
	.host_intr_event_rdy = PCIE9098_HOST_INTR_EVENT_RDY,
	.host_intr_cmd_dnld = PCIE9098_HOST_INTR_CMD_DNLD,
	.adma_align_size = ADMA_ALIGN_SIZE,
	.adma_min_pkt_size = ADMA_MIN_PKT_SIZE,
	.use_adma = MTRUE,
	.msi_int_wr_clr = MTRUE,
};
#endif

#if defined(PCIE9098) || defined(PCIE9097) || defined(PCIEIW624)
static const struct _mlan_pcie_card_reg mlan_reg_pcie9098 = {
	.reg_txbd_rdptr = PCIE9098_TXBD_RDPTR,
	.reg_txbd_wrptr = PCIE9098_TXBD_WRPTR,
	.reg_rxbd_rdptr = PCIE9098_RXBD_RDPTR,
	.reg_rxbd_wrptr = PCIE9098_RXBD_WRPTR,
	.reg_evtbd_rdptr = PCIE9098_EVTBD_RDPTR,
	.reg_evtbd_wrptr = PCIE9098_EVTBD_WRPTR,
	.reg_host_int_mask = PCIE9098_HOST_INT_MASK,
	.reg_host_int_status_mask = PCIE9098_HOST_INT_STATUS_MASK,
	.reg_host_int_status = PCIE9098_HOST_INT_STATUS,
	.reg_host_int_clr_sel = PCIE9098_HOST_INT_CLR_SEL,
	.reg_cpu_int_event = PCIE9098_CPU_INT_EVENT,
	.reg_ip_rev = PCIE9098_DEV_ID_REG,
	.reg_drv_ready = PCIE9098_DRV_READY,
	.reg_cpu_int_status = PCIE9098_CPU_INT_STATUS,
	.reg_rev_id = PCIE9098_REV_ID_REG,
	.reg_scratch_0 = PCIE9098_SCRATCH_0_REG,
	.reg_scratch_1 = PCIE9098_SCRATCH_1_REG,
	.reg_scratch_2 = PCIE9098_SCRATCH_2_REG,
	.reg_scratch_3 = PCIE9098_SCRATCH_3_REG,
	.reg_scratch_6 = PCIE9098_SCRATCH_6_REG,
	.reg_scratch_7 = PCIE9098_SCRATCH_7_REG,
	.host_intr_mask = PCIE9098_HOST_INTR_MASK,
	.host_intr_dnld_done = PCIE9098_HOST_INTR_DNLD_DONE,
	.host_intr_upld_rdy = PCIE9098_HOST_INTR_UPLD_RDY,
	.host_intr_cmd_done = PCIE9098_HOST_INTR_CMD_DONE,
	.host_intr_event_rdy = PCIE9098_HOST_INTR_EVENT_RDY,
	.host_intr_cmd_dnld = PCIE9098_HOST_INTR_CMD_DNLD,
	.adma_align_size = ADMA_ALIGN_SIZE,
	.adma_min_pkt_size = ADMA_MIN_PKT_SIZE,
	.use_adma = MTRUE,
	.msi_int_wr_clr = MTRUE,
};
#endif

#if defined(PCIE9098) || defined(PCIE9097) || defined(PCIEAW693) ||            \
	defined(PCIEIW624)
static const struct _mlan_card_info mlan_card_info_pcie9098 = {
	.max_tx_buf_size = MLAN_TX_DATA_BUF_SIZE_4K,
	.v16_fw_api = 1,
	.v17_fw_api = 1,
	.supp_ps_handshake = 0,
	.default_11n_tx_bf_cap = DEFAULT_11N_TX_BF_CAP_2X2,
	.support_11mc = 1,
};
#endif
/********************************************************
			Global Variables
********************************************************/

/********************************************************
			Local Functions
********************************************************/

static mlan_status wlan_pcie_delete_evtbd_ring(pmlan_adapter pmadapter);
static mlan_status wlan_pcie_delete_rxbd_ring(pmlan_adapter pmadapter);

#if defined(PCIE9098) || defined(PCIE9097) || defined(PCIEAW693) ||            \
	defined(PCIEIW624)
/**
 *  @brief This function program dma_cfg/dma_cfg2/dma_cfg3 registers
 *
 *  @param pmadapter  A pointer to mlan_adapter structure
 *  @param q_addr     adma queue address
 *  @param direction  adma direction (ADMA_DEVICE_TO_HOST/ADMA_HOST_TO_DEVICE)
 *  @param dma_mode   dma mode (DMA_MODE_DUAL_DESC/DMA_MODE_DIRECT)
 *  @param size       desc num/dma_size
 *  @param init       init flag
 *
 *  @return 	      MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status wlan_init_dma_cfg_registers(mlan_adapter *pmadapter,
					       t_u32 q_addr, t_u8 direction,
					       t_u8 dma_mode, t_u16 size,
					       t_u8 init)
{
	t_u32 dma_cfg, dma_cfg2 = 0;
	t_u32 dma_cfg3 = 0;
	pmlan_callbacks pcb = &pmadapter->callbacks;
	mlan_status ret = MLAN_STATUS_SUCCESS;

	ENTER();
	if (init) {
		/* Read the dma_cfg2 register */
		if (pcb->moal_read_reg(pmadapter->pmoal_handle,
				       q_addr + ADMA_DMA_CFG2, &dma_cfg2)) {
			PRINTM(MERROR, "Fail to read DMA CFG2 register\n");
			ret = MLAN_STATUS_FAILURE;
			goto done;
		}
		dma_cfg2 &= ~(ADMA_SRC_DMA_DONE_INT_BYPASS_EN |
			      ADMA_DST_DMA_DONE_INT_BYPASS_EN |
			      ADMA_MSI_LEGACY_SRC_DMA_DONE_INT_BYPASS_EN |
			      ADMA_MSI_LEGACY_DST_DMA_DONE_INT_BYPASS_EN |
			      ADMA_MSI_LEGACY_ENABLE | ADMA_MSIX_ENABLE |
			      ADMA_MSIX_INT_SRC_DST_SEL);

		if (dma_mode == DMA_MODE_DUAL_DESC) {
			if (direction == ADMA_HOST_TO_DEVICE) {
				dma_cfg2 |= ADMA_SRC_DMA_DONE_INT_BYPASS_EN;
				dma_cfg2 |=
					ADMA_MSI_LEGACY_SRC_DMA_DONE_INT_BYPASS_EN;
			} else {
				/* Read the dma_cfg3 register */
				if (pcb->moal_read_reg(pmadapter->pmoal_handle,
						       q_addr + ADMA_DMA_CFG3,
						       &dma_cfg3)) {
					PRINTM(MERROR,
					       "Fail to read DMA CFG3 register\n");
					ret = MLAN_STATUS_FAILURE;
					goto done;
				}
				dma_cfg3 |= ADMA_Q_PTR_CLR;
				if (pcb->moal_write_reg(pmadapter->pmoal_handle,
							q_addr + ADMA_DMA_CFG3,
							(t_u32)dma_cfg3)) {
					PRINTM(MERROR,
					       "Failed to write ADMA_DMA_CFG3.\n");
					ret = MLAN_STATUS_FAILURE;
				}
				dma_cfg2 |= ADMA_DST_DMA_DONE_INT_BYPASS_EN;
				dma_cfg2 |=
					ADMA_MSI_LEGACY_DST_DMA_DONE_INT_BYPASS_EN;
			}
		} else {
			if (direction == ADMA_HOST_TO_DEVICE)
				dma_cfg2 |= ADMA_SRC_ADDR_IS_HOST;
			else
				dma_cfg2 |= ADMA_DST_ADDR_IS_HOST;
		}
		dma_cfg2 |= ADMA_MSI_LEGACY_ENABLE;
		PRINTM(MCMND, "dma_cfg2=0x%x\n", dma_cfg2);

		/* enable INT_BYPASS_EN in the dma_cfg2 register */
		if (pcb->moal_write_reg(pmadapter->pmoal_handle,
					q_addr + ADMA_DMA_CFG2,
					(t_u32)dma_cfg2)) {
			PRINTM(MERROR, "Failed to write DMA CFG2.\n");
			ret = MLAN_STATUS_FAILURE;
			goto done;
		}
	}
	/* Read the TX ring read pointer set by firmware */
	if (pcb->moal_read_reg(pmadapter->pmoal_handle, q_addr + ADMA_DMA_CFG,
			       &dma_cfg)) {
		PRINTM(MERROR, "Fail to read DMA CFG register\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	if (direction == ADMA_HOST_TO_DEVICE) {
		if (dma_mode == DMA_MODE_DUAL_DESC) {
			dma_cfg &= ~(DESC_MODE_MASK | DMA_MODE_MASK |
				     SRC_NUM_DESC_MASK | DMA_SIZE_MASK);
			dma_cfg |= (t_u32)size << SRC_NUM_DESC_BIT;
			dma_cfg |= DESC_MODE_RING << 2;
		} else {
			dma_cfg &= ~(DESC_MODE_MASK | DMA_MODE_MASK |
				     SRC_NUM_DESC_MASK | DST_NUM_DESC_MASK |
				     DMA_SIZE_MASK);
			dma_cfg |= (t_u32)size << DMA_SIZE_BIT;
		}
	} else {
		if (dma_mode == DMA_MODE_DUAL_DESC) {
			dma_cfg &= ~(DESC_MODE_MASK | DMA_MODE_MASK |
				     DST_NUM_DESC_MASK | DMA_SIZE_MASK);
			dma_cfg |= (t_u32)size << DST_NUM_DESC_BIT;
			dma_cfg |= DESC_MODE_RING << 2;
		} else {
			dma_cfg &= ~(DESC_MODE_MASK | DMA_MODE_MASK |
				     SRC_NUM_DESC_MASK | DST_NUM_DESC_MASK);
		}
	}
	dma_cfg |= (t_u32)dma_mode;
	PRINTM(MINFO, "dma_cfg=0x%x\n", dma_cfg);
	if (pcb->moal_write_reg(pmadapter->pmoal_handle, q_addr + ADMA_DMA_CFG,
				dma_cfg)) {
		PRINTM(MERROR, "Fail to set DMA CFG register\n");
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}
done:
	LEAVE();
	return ret;
}

/**
 *  @brief This function init the adma setting
 *
 *  @param pmadapter  A pointer to mlan_adapter structure
 *  @param adma_type  TX/RX data, event, cmd/cmdresp
 *  @param pbase      physical address
 *  @param size       desc num/dma_size
 *  @param init       init flag
 *
 *  @return 	      MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status wlan_init_adma(mlan_adapter *pmadapter, t_u8 type,
				  t_u64 pbase, t_u16 size, t_u8 init)
{
	t_u32 int_mapping;
	mlan_status ret = MLAN_STATUS_SUCCESS;
	t_u32 q_addr = 0;
	t_u8 direction = 0;
	t_u8 dma_mode = 0;
	pmlan_callbacks pcb = &pmadapter->callbacks;
	ENTER();
	if (init)
		PRINTM(MCMND, "Init ADMA: type=%d, size=%d init=%d\n", type,
		       size, init);
	switch (type) {
	case ADMA_TX_DATA:
		q_addr = ADMA_CHAN0_Q0;
		direction = ADMA_HOST_TO_DEVICE;
		dma_mode = DMA_MODE_DUAL_DESC;
		break;
	case ADMA_RX_DATA:
		q_addr = ADMA_CHAN1_Q0;
		direction = ADMA_DEVICE_TO_HOST;
		dma_mode = DMA_MODE_DUAL_DESC;
		break;
	case ADMA_EVENT:
		q_addr = ADMA_CHAN1_Q1;
		direction = ADMA_DEVICE_TO_HOST;
		dma_mode = DMA_MODE_DUAL_DESC;
		break;
	case ADMA_CMD:
		q_addr = ADMA_CHAN2_Q0;
		direction = ADMA_HOST_TO_DEVICE;
		dma_mode = DMA_MODE_DIRECT;
		break;
	case ADMA_CMDRESP:
		q_addr = ADMA_CHAN2_Q1;
		direction = ADMA_DEVICE_TO_HOST;
		dma_mode = DMA_MODE_DIRECT;
		break;
	default:
		PRINTM(MERROR, "unknow adma type\n");
		ret = MLAN_STATUS_FAILURE;
		break;
	}
	if (ret)
		goto done;
	if (init) {
		if (dma_mode == DMA_MODE_DUAL_DESC) {
			if (direction == ADMA_HOST_TO_DEVICE)
				int_mapping = DEST_INT_TO_DEVICE;
			else
				int_mapping = DEST_INT_TO_HOST;
		} else {
			int_mapping = 0;
		}
		/* set INT_MAPPING register */
		if (pcb->moal_write_reg(pmadapter->pmoal_handle,
					q_addr + ADMA_INT_MAPPING,
					(t_u32)int_mapping)) {
			PRINTM(MERROR,
			       "Failed to write ADMA_INT_MAPPING register.\n");
			ret = MLAN_STATUS_FAILURE;
			goto done;
		}
	}

	if (direction == ADMA_HOST_TO_DEVICE) {
		/* Write the lower 32bits of the physical address to
		 * ADMA_SRC_LOW */
		if (pcb->moal_write_reg(pmadapter->pmoal_handle,
					q_addr + ADMA_SRC_LOW, (t_u32)pbase)) {
			PRINTM(MERROR, "Failed to write ADMA_SRC_LOW.\n");
			ret = MLAN_STATUS_FAILURE;
			goto done;
		}
		/* Write the upper 32bits of the physical address to
		 * ADMA_SRC_HIGH */
		if (pcb->moal_write_reg(pmadapter->pmoal_handle,
					q_addr + ADMA_SRC_HIGH,
					(t_u32)((t_u64)pbase >> 32))) {
			PRINTM(MERROR, "Failed to write ADMA_SRC_HIGH.\n");
			ret = MLAN_STATUS_FAILURE;
			goto done;
		}
		if (init) {
			/** Enable DMA done interrupt */
			if (pcb->moal_write_reg(
				    pmadapter->pmoal_handle,
				    q_addr + ADMA_SRC_INT_STATUS_MASK,
				    DEF_ADMA_INT_MASK)) {
				PRINTM(MERROR,
				       "Failed to write ADMA_SRC_INT_STATUS_MASK.\n");
				ret = MLAN_STATUS_FAILURE;
				goto done;
			}
			if (pcb->moal_write_reg(pmadapter->pmoal_handle,
						q_addr + ADMA_SRC_INT_MASK,
						DEF_ADMA_INT_MASK)) {
				PRINTM(MERROR,
				       "Failed to write ADMA_SRC_INT_MASK.\n");
				ret = MLAN_STATUS_FAILURE;
				goto done;
			}
		}
	} else {
		/* Write the lower 32bits of the physical address to
		 * ADMA_DST_LOW */
		if (pcb->moal_write_reg(pmadapter->pmoal_handle,
					q_addr + ADMA_DST_LOW, (t_u32)pbase)) {
			PRINTM(MERROR, "Failed to write ADMA_DST_LOW.\n");
			ret = MLAN_STATUS_FAILURE;
			goto done;
		}
		/* Write the upper 32bits of the physical address to
		 * ADMA_DST_HIGH */
		if (pcb->moal_write_reg(pmadapter->pmoal_handle,
					q_addr + ADMA_DST_HIGH,
					(t_u32)((t_u64)pbase >> 32))) {
			PRINTM(MERROR, "Failed to write ADMA_DST_HIGH.\n");
			ret = MLAN_STATUS_FAILURE;
			goto done;
		}
		if (init && (dma_mode == DMA_MODE_DUAL_DESC)) {
			/** Enable DMA done interrupt */
			if (pcb->moal_write_reg(
				    pmadapter->pmoal_handle,
				    q_addr + ADMA_DST_INT_STATUS_MASK,
				    DEF_ADMA_INT_MASK)) {
				PRINTM(MERROR,
				       "Failed to write ADMA_SRC_INT_STATUS_MASK.\n");
				ret = MLAN_STATUS_FAILURE;
				goto done;
			}
			if (pcb->moal_write_reg(pmadapter->pmoal_handle,
						q_addr + ADMA_DST_INT_MASK,
						DEF_ADMA_INT_MASK)) {
				PRINTM(MERROR,
				       "Failed to write ADMA_SRC_INT_MASK.\n");
				ret = MLAN_STATUS_FAILURE;
				goto done;
			}
		}
	}
	if (!IS_PCIEIW624(pmadapter->card_type) &&
	    !(IS_PCIEAW693(pmadapter->card_type) &&
	      (pmadapter->card_rev > CHIP_AW693_REV_A0))) {
		if (wlan_init_dma_cfg_registers(pmadapter, q_addr, direction,
						dma_mode, size, init)) {
			PRINTM(MERROR,
			       "Failed to configure dma_cfg registers\n");
			ret = MLAN_STATUS_FAILURE;
			goto done;
		}
	}

	if (type == ADMA_CMD && !init) {
		/* Write 1 to src_wr_ptr to trigger direct dma */
		if (pcb->moal_write_reg(pmadapter->pmoal_handle,
					q_addr + ADMA_SRC_RW_PTR, 1)) {
			PRINTM(MERROR, "Failed to write ADMA_SRC_HIGH.\n");
			ret = MLAN_STATUS_FAILURE;
			goto done;
		}
	}
done:
	LEAVE();
	return ret;
}

/**
 *  @brief This function init the adma ring size from user input
 *
 *  @param pmadapter  A pointer to mlan_adapter structure
 *
 *  @return 	      N/A
 */
static void wlan_pcie_init_adma_ring_size(mlan_adapter *pmadapter)
{
	t_u16 num_desc = 0;
	t_u16 ring_size = 0;

	ring_size = pmadapter->init_para.ring_size;
	if (!ring_size)
		return;
	if (ring_size < MAX_TXRX_BD)
		ring_size = MAX_TXRX_BD;
	else if (ring_size > ADMA_MAX_TXRX_BD)
		ring_size = ADMA_MAX_TXRX_BD;
	if (ring_size != pmadapter->pcard_pcie->txrx_bd_size) {
		ring_size = ring_size >> 1;
		while (ring_size > 0) {
			ring_size = ring_size >> 1;
			num_desc++;
		}
		pmadapter->pcard_pcie->txrx_bd_size = 1 << num_desc;
		pmadapter->pcard_pcie->txrx_num_desc = num_desc;
	}
	PRINTM(MMSG, "ring_size =%d num_desc=%d\n",
	       pmadapter->pcard_pcie->txrx_bd_size,
	       pmadapter->pcard_pcie->txrx_num_desc);
	return;
}

#endif

#if defined(PCIE9098) || defined(PCIE9097) || defined(PCIEAW693) ||            \
	defined(PCIEIW624)
#if defined(PCIEAW693)
/**
 *  @brief This function get max msdu count
 *
 *  @param pmadapter A pointer to mlan_adapter structure
 *
 *  @return          max msdu count
 */
static t_u32 wlan_pcie_get_max_msdu_cnt(mlan_adapter *pmadapter)
{
	t_u32 ring_size = 0;
	const t_u32 num_tx_buffers = pmadapter->pcard_pcie->txrx_bd_size;
	const t_u32 num_pending = pmadapter->pcard_pcie->txbd_pending;

	ENTER();
#ifdef PCIEAW693
	if (!IS_PCIEAW693(pmadapter->card_type))
		return 0;
#endif

	if (num_tx_buffers > num_pending)
		ring_size = num_tx_buffers - num_pending;

	LEAVE();
	return ring_size;
}
#endif

#if defined(PCIEAW693)
/**
 *  @brief This function return the index of desc with EOP flag.
 *
 *  @param pmadapter     A pointer to mlan_adapter structure
 *  @param num_tx_buffs  Ring size
 *  @param index         index in Tx ring
 *  @param wrindx        Current write index in TX ring
 *
 *  @return              index in Tx ring with EOP flag
 */
static t_u32 wlan_get_adma_desc_with_eop(mlan_pcie_card *pcard,
					 t_u32 num_tx_buffs, t_u32 index,
					 t_u32 wrindx)
{
	for (; index != wrindx; index = (index + 1) & (num_tx_buffs - 1)) {
		adma_dual_desc_buf *desc = pcard->txbd_ring[index];
		if (desc->flags & ADMA_BD_FLAG_EOP)
			return index;
	}

	return wrindx;
}

/**
 *  @brief This function enables tx_done interrupt in pending descriptor if
 * needed.
 *
 *  @param pmadapter     A pointer to mlan_adapter structure
 *  @param num_tx_buffs  Ring size
 *  @param wrindx        Current write index in TX ring
 *
 *  @return              N/A
 */
static void wlan_get_adma_buf_flag_sliding(mlan_adapter *pmadapter,
					   t_u32 num_tx_buffs, t_u32 wrindx)
{
	const t_u32 txbd_pending_threshold = num_tx_buffs / 4;
	const t_u32 invalid_index = 0xffffffff;
	mlan_pcie_card *pcard = pmadapter->pcard_pcie;
	t_u32 last_write_index_with_irq = invalid_index;
	const t_u32 mid =
		(wrindx - pcard->txbd_pending / 2) & (num_tx_buffs - 1);
	/* the calling function validates bounds for num_tx_buffs and wrindx */
	// coverity[integer_overflow:SUPPRESS]
	const t_u32 mid_idx =
		wlan_get_adma_desc_with_eop(pcard, num_tx_buffs, mid, wrindx);

	if (mid_idx != wrindx && pcard->txbd_pending > txbd_pending_threshold &&
	    pcard->last_write_index_with_irq != mid_idx) {
		adma_dual_desc_buf *desc = pcard->txbd_ring[mid_idx];

		desc->flags |= wlan_cpu_to_le16(ADMA_BD_FLAG_INT_EN);
		last_write_index_with_irq = mid_idx;
	}

	if (last_write_index_with_irq != invalid_index) {
		if (pcard->last_write_index_with_irq != invalid_index &&
		    pcard->last_write_index_with_irq != mid_idx) {
			adma_dual_desc_buf *desc =
				pcard->txbd_ring
					[pcard->last_write_index_with_irq];

			desc->flags &= ~wlan_cpu_to_le16(ADMA_BD_FLAG_INT_EN);
		}

		pcard->last_write_index_with_irq = last_write_index_with_irq;
	}
}
#endif

/**
 *  @brief This function enables tx_done interrupt in pending descriptor if
 * needed.
 *
 *  @param pmadapter     A pointer to mlan_adapter structure
 *  @param num_tx_buffs  Ring size
 *  @param wrindx        Current write index in TX ring
 *
 *  @return              N/A
 */
static t_u16 wlan_get_adma_buf_flag(mlan_adapter *pmadapter, t_u32 num_tx_buffs,
				    t_u32 wrindx)
{
	mlan_pcie_card *pcard = pmadapter->pcard_pcie;
	t_u16 flags = 0;
	/* enable DMA transfer done interrupt when ring is full so we don't need
	 * to wait when it became empty */
	const t_u32 txbd_pending_threshold = num_tx_buffs - 1;
	const t_bool need_int_en =
		pcard->txbd_pending >= txbd_pending_threshold;

	if (need_int_en) {
		const t_u32 distance = num_tx_buffs / 2;
		t_u32 past_wrindx = (wrindx - distance) & (num_tx_buffs - 1);
		const t_u32 distance_to_last =
			(past_wrindx - pcard->last_write_index_with_irq) &
			(num_tx_buffs - 1);

		if (pcard->last_write_index_with_irq == 0xffffffff ||
		    distance_to_last >= distance) {
			flags = ADMA_BD_FLAG_INT_EN;
			pcard->last_write_index_with_irq = past_wrindx;
		}
	}

	if (pcard->last_write_index_with_irq == wrindx) {
		/* we are going to overwrite descriptor where we set interrupt,
		 * need to invalidate */
		pcard->last_write_index_with_irq = 0xffffffff;
	}
	return flags;
}

/**
 *  @brief This function set the host interrupt select mask
 *
 *  @param pmadapter  A pointer to mlan_adapter structure
 *
 *
 *  @return 	      MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status wlan_pcie_set_host_int_select_reg(mlan_adapter *pmadapter,
						     t_u32 int_sel_mask)
{
	pmlan_callbacks pcb = &pmadapter->callbacks;
	ENTER();
	/* Simply write the mask to the register */
	if (pcb->moal_write_reg(pmadapter->pmoal_handle, PCIE9098_HOST_INT_SEL,
				int_sel_mask)) {
		PRINTM(MWARN, "Set host interrupt select register failed\n");
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}
	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function set the host interrupt select mask
 *
 *  @param pmadapter  A pointer to mlan_adapter structure
 *  @param enable     0-disable 1-enable
 *
 *  @return 	      MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status wlan_pcie_set_host_int_select_mask(mlan_adapter *pmadapter,
						      t_u8 enable)
{
	pmlan_callbacks pcb = &pmadapter->callbacks;
	t_u32 int_sel_mask = 0;
	t_u32 int_clr_mask = 0;
	ENTER();

	if (enable) {
		int_sel_mask = PCIE9098_HOST_INTR_SEL_MASK;
		int_clr_mask = pmadapter->pcard_pcie->reg->host_intr_mask;
	}

	/* Simply write the mask to the register */
	if (pcb->moal_write_reg(pmadapter->pmoal_handle, PCIE9098_HOST_INT_SEL,
				int_sel_mask)) {
		PRINTM(MWARN, "Set host interrupt select register failed\n");
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}
	if (pmadapter->pcard_pcie->pcie_int_mode == PCIE_INT_MODE_MSI) {
		if (!pmadapter->pcard_pcie->reg->msi_int_wr_clr) {
			/** enable read to clear interrupt */
			if (pcb->moal_write_reg(pmadapter->pmoal_handle,
						pmadapter->pcard_pcie->reg
							->reg_host_int_clr_sel,
						int_clr_mask)) {
				PRINTM(MWARN,
				       "enable read to clear interrupt failed\n");
				LEAVE();
				return MLAN_STATUS_FAILURE;
			}
		}
	}
	LEAVE();
	return MLAN_STATUS_SUCCESS;
}
#endif

/**
 *  @brief This function downloads boot command to the card.
 *
 *  @param pmadapter    A pointer to mlan_adapter structure
 *  @param pmbuf        A pointer to mlan_buffer
 *  @param mapping      mapping flag
 *
 *  @return 	        MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status wlan_pcie_send_boot_cmd(mlan_adapter *pmadapter,
					   mlan_buffer *pmbuf, t_u8 mapping)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	pmlan_callbacks pcb;

	ENTER();

	if (!pmadapter || !pmbuf) {
		PRINTM(MERROR, "NULL Pointer\n");
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}

	pcb = &pmadapter->callbacks;

	if (mapping) {
		if (MLAN_STATUS_FAILURE ==
		    pcb->moal_map_memory(pmadapter->pmoal_handle,
					 pmbuf->pbuf + pmbuf->data_offset,
					 &pmbuf->buf_pa, WLAN_UPLD_SIZE,
					 PCI_DMA_TODEVICE)) {
			PRINTM(MERROR, "BootCmd: failed to moal_map_memory\n");
			LEAVE();
			return MLAN_STATUS_FAILURE;
		}
	}

	if (!(pmbuf->pbuf && pmbuf->data_len && pmbuf->buf_pa)) {
		PRINTM(MERROR, "%s: Invalid buffer <%p, %#x:%x, len=%d>\n",
		       __func__, pmbuf->pbuf,
		       (t_u32)((t_u64)pmbuf->buf_pa >> 32),
		       (t_u32)pmbuf->buf_pa, pmbuf->data_len);
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	/* Write the lower 32bits of the physical address to scratch
	 * register 0 */
	if (pcb->moal_write_reg(pmadapter->pmoal_handle,
				pmadapter->pcard_pcie->reg->reg_scratch_0,
				(t_u32)pmbuf->buf_pa)) {
		PRINTM(MERROR,
		       "Failed to write download command to boot code\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	/* Write the upper 32bits of the physical address to scratch
	 * register 1 */
	if (pcb->moal_write_reg(pmadapter->pmoal_handle,
				pmadapter->pcard_pcie->reg->reg_scratch_1,
				(t_u32)((t_u64)pmbuf->buf_pa >> 32))) {
		PRINTM(MERROR,
		       "Failed to write download command to boot code\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	/* Write the command length to scratch register 2 */
	if (pcb->moal_write_reg(pmadapter->pmoal_handle,
				pmadapter->pcard_pcie->reg->reg_scratch_2,
				pmbuf->data_len)) {
		PRINTM(MERROR,
		       "Failed to write command length to scratch register 2\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	/* Ring the door bell */
	if (pcb->moal_write_reg(pmadapter->pmoal_handle,
				pmadapter->pcard_pcie->reg->reg_cpu_int_event,
				CPU_INTR_DOOR_BELL)) {
		PRINTM(MERROR, "Failed to assert door-bell interrupt\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	LEAVE();
	return ret;

done:
	if (MLAN_STATUS_FAILURE ==
	    pcb->moal_unmap_memory(
		    pmadapter->pmoal_handle, pmbuf->pbuf + pmbuf->data_offset,
		    pmbuf->buf_pa, WLAN_UPLD_SIZE, PCI_DMA_TODEVICE))
		PRINTM(MERROR, "BootCmd: failed to moal_unmap_memory\n");
	LEAVE();
	return ret;
}

#if defined(PCIE8997) || defined(PCIE9098) || defined(PCIE9097) ||             \
	defined(PCIEAW693) || defined(PCIEIW624)
/**
 *  @brief This function handles command response completion
 *
 *  @param pmadapter    A pointer to mlan_adapter structure
 *  @param pmbuf        A pointer to mlan_buffer
 *
 *  @return 	        MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status wlan_pcie_send_vdll_complete(mlan_adapter *pmadapter)
{
	mlan_buffer *pcmdbuf;
	pmlan_callbacks pcb = &pmadapter->callbacks;
	ENTER();
	/*unmap the cmd pmbuf, so the cpu can not access the memory in the
	 * command node*/
	pcmdbuf = pmadapter->pcard_pcie->vdll_cmd_buf;
	if (pcmdbuf) {
		pcb->moal_unmap_memory(pmadapter->pmoal_handle,
				       pcmdbuf->pbuf + pcmdbuf->data_offset,
				       pcmdbuf->buf_pa, pcmdbuf->data_len,
				       PCI_DMA_TODEVICE);
		pmadapter->pcard_pcie->vdll_cmd_buf = MNULL;
	}
	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function downloads VDLL image to the card.
 *
 *  @param pmadapter A pointer to mlan_adapter structure
 *  @param pmbuf     A pointer to mlan_buffer
 *
 *  @return 	     MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status wlan_pcie_send_vdll(mlan_adapter *pmadapter,
				       mlan_buffer *pmbuf)
{
	mlan_status ret = MLAN_STATUS_PENDING;
	pmlan_callbacks pcb = &pmadapter->callbacks;
	t_u16 *tmp;
	t_u8 *payload;

	ENTER();
	pmadapter->cmd_sent = MTRUE;
	payload = pmbuf->pbuf + pmbuf->data_offset;

	tmp = (t_u16 *)&payload[0];
	*tmp = wlan_cpu_to_le16((t_u16)pmbuf->data_len);
	tmp = (t_u16 *)&payload[2];
	*tmp = wlan_cpu_to_le16(MLAN_TYPE_VDLL);

	if (MLAN_STATUS_FAILURE ==
	    pcb->moal_map_memory(
		    pmadapter->pmoal_handle, pmbuf->pbuf + pmbuf->data_offset,
		    &pmbuf->buf_pa, pmbuf->data_len, PCI_DMA_TODEVICE)) {
		PRINTM(MERROR,
		       "PCIE - Download VDLL block: moal_map_memory failed\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	pmadapter->pcard_pcie->vdll_cmd_buf = pmbuf;

#if defined(PCIE9098) || defined(PCIE9097) || defined(PCIEAW693) ||            \
	defined(PCIEIW624)
	/* issue the DMA */
	if (pmadapter->pcard_pcie->reg->use_adma) {
		if (IS_PCIEIW624(pmadapter->card_type) ||
		    (IS_PCIEAW693(pmadapter->card_type) &&
		     (pmadapter->card_rev > CHIP_AW693_REV_A0))) {
			if (wlan_pcie_send_boot_cmd(pmadapter, pmbuf, MFALSE)) {
				PRINTM(MERROR,
				       "Failed to send vdll block to device\n");
				ret = MLAN_STATUS_FAILURE;
				goto done;
			}
		} else {
			/* send the VDLL block down to the firmware */
			wlan_init_adma(pmadapter, ADMA_CMD, pmbuf->buf_pa,
				       pmbuf->data_len, MFALSE);
		}
	}
#endif

#ifdef PCIE8997
	if (!pmadapter->pcard_pcie->reg->use_adma) {
		if (wlan_pcie_send_boot_cmd(pmadapter, pmbuf, MFALSE)) {
			PRINTM(MERROR, "Failed to send vdll block to device\n");
			ret = MLAN_STATUS_FAILURE;
			goto done;
		}
	}
#endif

	PRINTM(MINFO, "PCIE - Download VDLL Block: successful.\n");
done:
	if (ret == MLAN_STATUS_FAILURE)
		pmadapter->cmd_sent = MFALSE;

	LEAVE();
	return ret;
}
#endif

/**
 *  @brief This function disables the host interrupt
 *
 *  @param pmadapter  A pointer to mlan_adapter structure
 *
 *  @return 	      MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status wlan_pcie_disable_host_int_mask(mlan_adapter *pmadapter)
{
	pmlan_callbacks pcb = &pmadapter->callbacks;

	ENTER();
	if (pcb->moal_write_reg(pmadapter->pmoal_handle,
				pmadapter->pcard_pcie->reg->reg_host_int_mask,
				0x00000000)) {
		PRINTM(MWARN, "Disable host interrupt failed\n");
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}
	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function enables the host interrupt
 *
 *  @param pmadapter  A pointer to mlan_adapter structure
 *
 *  @return 	      MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status wlan_pcie_enable_host_int_mask(mlan_adapter *pmadapter)
{
	pmlan_callbacks pcb = &pmadapter->callbacks;
	ENTER();
	/* Simply write the mask to the register */
	if (pcb->moal_write_reg(pmadapter->pmoal_handle,
				pmadapter->pcard_pcie->reg->reg_host_int_mask,
				pmadapter->pcard_pcie->reg->host_intr_mask)) {
		PRINTM(MWARN, "Enable host interrupt failed\n");
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}
	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function enables the host interrupts.
 *
 *  @param pmadapter A pointer to mlan_adapter structure
 *  @param enable   0-disable 1-enable
 *
 *  @return        MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status
wlan_pcie_enable_host_int_status_mask(mlan_adapter *pmadapter, t_u8 enable)
{
	pmlan_callbacks pcb = &pmadapter->callbacks;
	t_u32 host_int_status_mask = 0;
	ENTER();
	if (enable)
		host_int_status_mask =
			pmadapter->pcard_pcie->reg->host_intr_mask;
	/* Enable host int status mask */
	if (pcb->moal_write_reg(
		    pmadapter->pmoal_handle,
		    pmadapter->pcard_pcie->reg->reg_host_int_status_mask,
		    host_int_status_mask)) {
		PRINTM(MWARN, "Enable host interrupt status mask failed\n");
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}
	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function disables the host interrupts.
 *
 *  @param pmadapter A pointer to mlan_adapter structure
 *
 *  @return        MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status wlan_disable_pcie_host_int(mlan_adapter *pmadapter)
{
	mlan_status ret;

	ENTER();
	ret = wlan_pcie_enable_host_int_status_mask(pmadapter, MFALSE);
	if (ret) {
		LEAVE();
		return ret;
	}
#if defined(PCIE9098) || defined(PCIE9097) || defined(PCIEAW693) ||            \
	defined(PCIEIW624)
	if ((pmadapter->card_type == CARD_TYPE_PCIE9098) ||
	    (pmadapter->card_type == CARD_TYPE_PCIEIW624) ||
	    (pmadapter->card_type == CARD_TYPE_PCIEAW693) ||
	    (pmadapter->card_type == CARD_TYPE_PCIE9097)) {
		ret = wlan_pcie_set_host_int_select_mask(pmadapter, MFALSE);
		if (ret) {
			LEAVE();
			return ret;
		}
	}
#endif
	ret = wlan_pcie_disable_host_int_mask(pmadapter);
	LEAVE();
	return ret;
}

/**
 *  @brief This function checks the interrupt status and
 *  handle it accordingly.
 *
 *  @param pmadapter    A pointer to mlan_adapter structure
 *
 *  @return             MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status wlan_clear_pending_int_status(mlan_adapter *pmadapter)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	t_u32 pcie_ireg = 0;
	pmlan_callbacks pcb = &pmadapter->callbacks;

	ENTER();

	if (pcb->moal_read_reg(pmadapter->pmoal_handle,
			       pmadapter->pcard_pcie->reg->reg_host_int_status,
			       &pcie_ireg)) {
		PRINTM(MERROR, "Read host int status register failed\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	if ((pcie_ireg != 0xFFFFFFFF) && (pcie_ireg)) {
		PRINTM(MMSG, "pcie_ireg=0x%x\n", pcie_ireg);
		if (pmadapter->pcard_pcie->reg->msi_int_wr_clr) {
			if (pcb->moal_write_reg(pmadapter->pmoal_handle,
						pmadapter->pcard_pcie->reg
							->reg_host_int_status,
						~pcie_ireg)) {
				PRINTM(MERROR,
				       "Write host int status  register failed\n");
				ret = MLAN_STATUS_FAILURE;
				goto done;
			}
		}
	}
done:
	LEAVE();
	return ret;
}

/**
 *  @brief This function enables the host interrupts.
 *
 *  @param pmadapter A pointer to mlan_adapter structure
 *
 *  @return        MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status wlan_enable_pcie_host_int(mlan_adapter *pmadapter)
{
	mlan_status ret;

	ENTER();
	wlan_clear_pending_int_status(pmadapter);
	ret = wlan_pcie_enable_host_int_status_mask(pmadapter, MTRUE);
	if (ret) {
		LEAVE();
		return ret;
	}
#if defined(PCIE9098) || defined(PCIE9097) || defined(PCIEAW693) ||            \
	defined(PCIEIW624)
	if ((pmadapter->card_type == CARD_TYPE_PCIE9098) ||
	    (pmadapter->card_type == CARD_TYPE_PCIEIW624) ||
	    (pmadapter->card_type == CARD_TYPE_PCIEAW693) ||
	    (pmadapter->card_type == CARD_TYPE_PCIE9097)) {
		ret = wlan_pcie_set_host_int_select_mask(pmadapter, MTRUE);
		if (ret) {
			LEAVE();
			return ret;
		}
	}
#endif
	ret = wlan_pcie_enable_host_int_mask(pmadapter);
	LEAVE();
	return ret;
}

/**
 *  @brief This function creates buffer descriptor ring for TX
 *
 *  @param pmadapter  A pointer to mlan_adapter structure
 *
 *  @return 	      MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status wlan_pcie_create_txbd_ring(mlan_adapter *pmadapter)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	pmlan_callbacks pcb = &pmadapter->callbacks;
	t_u32 i;
#if defined(PCIE8997) || defined(PCIE8897)
	pmlan_pcie_data_buf ptx_bd_buf;
#endif
#if defined(PCIE9098) || defined(PCIE9097) || defined(PCIEAW693) ||            \
	defined(PCIEIW624)
	padma_dual_desc_buf padma_bd_buf;
#endif

	ENTER();
	/*
	 * driver maintaines the write pointer and firmware maintaines the read
	 * pointer.
	 */
	pmadapter->pcard_pcie->txbd_wrptr = 0;
	pmadapter->pcard_pcie->txbd_pending = 0;
	pmadapter->pcard_pcie->txbd_rdptr = 0;

	/* allocate shared memory for the BD ring and divide the same in to
	   several descriptors */
#if defined(PCIE8997) || defined(PCIE8897)
	if (!pmadapter->pcard_pcie->reg->use_adma)
		pmadapter->pcard_pcie->txbd_ring_size =
			sizeof(mlan_pcie_data_buf) *
			pmadapter->pcard_pcie->txrx_bd_size;
#endif

#if defined(PCIE9098) || defined(PCIE9097) || defined(PCIEAW693) ||            \
	defined(PCIEIW624)
	if (pmadapter->pcard_pcie->reg->use_adma)
		pmadapter->pcard_pcie->txbd_ring_size =
			sizeof(adma_dual_desc_buf) *
			pmadapter->pcard_pcie->txrx_bd_size;
#endif
	PRINTM(MINFO, "TX ring: allocating %d bytes\n",
	       pmadapter->pcard_pcie->txbd_ring_size);

	ret = pcb->moal_malloc_consistent(
		pmadapter->pmoal_handle, pmadapter->pcard_pcie->txbd_ring_size,
		&pmadapter->pcard_pcie->txbd_ring_vbase,
		&pmadapter->pcard_pcie->txbd_ring_pbase);

	if (ret != MLAN_STATUS_SUCCESS) {
		PRINTM(MERROR, "%s: No free moal_malloc_consistent\n",
		       __FUNCTION__);
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}

	PRINTM(MINFO,
	       "TX ring: - base: %p, pbase: %#x:%x,"
	       "len: %x\n",
	       pmadapter->pcard_pcie->txbd_ring_vbase,
	       (t_u32)((t_u64)pmadapter->pcard_pcie->txbd_ring_pbase >> 32),
	       (t_u32)pmadapter->pcard_pcie->txbd_ring_pbase,
	       pmadapter->pcard_pcie->txbd_ring_size);

	for (i = 0; i < pmadapter->pcard_pcie->txrx_bd_size; i++) {
		pmadapter->pcard_pcie->tx_buf_list[i] = MNULL;
#if defined(PCIE9098) || defined(PCIE9097) || defined(PCIEAW693) ||            \
	defined(PCIEIW624)
		if (pmadapter->pcard_pcie->reg->use_adma) {
			padma_bd_buf =
				(adma_dual_desc_buf
					 *)(pmadapter->pcard_pcie
						    ->txbd_ring_vbase +
					    (sizeof(adma_dual_desc_buf) * i));
			pmadapter->pcard_pcie->txbd_ring[i] =
				(t_void *)padma_bd_buf;
			padma_bd_buf->paddr = 0;
			padma_bd_buf->len = 0;
			padma_bd_buf->flags = wlan_cpu_to_le16(
				ADMA_BD_FLAG_INT_EN | ADMA_BD_FLAG_SRC_HOST |
				ADMA_BD_FLAG_SOP | ADMA_BD_FLAG_EOP);
			padma_bd_buf->pkt_size = 0;
			padma_bd_buf->reserved = 0;
		}
#endif

#if defined(PCIE8997) || defined(PCIE8897)
		if (!pmadapter->pcard_pcie->reg->use_adma) {
			ptx_bd_buf =
				(mlan_pcie_data_buf
					 *)(pmadapter->pcard_pcie
						    ->txbd_ring_vbase +
					    (sizeof(mlan_pcie_data_buf) * i));
			pmadapter->pcard_pcie->txbd_ring[i] =
				(t_void *)ptx_bd_buf;
			ptx_bd_buf->paddr = 0;
			ptx_bd_buf->len = 0;
			ptx_bd_buf->flags = 0;
			ptx_bd_buf->frag_len = 0;
			ptx_bd_buf->offset = 0;
		}
#endif
	}
	LEAVE();
	return ret;
}

/**
 *  @brief This function frees TX buffer descriptor ring
 *
 *  @param pmadapter  A pointer to mlan_adapter structure
 *
 *  @return 	      MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status wlan_pcie_delete_txbd_ring(mlan_adapter *pmadapter)
{
	t_u32 i;
	pmlan_callbacks pcb = &pmadapter->callbacks;
	mlan_buffer *pmbuf = MNULL;
#if defined(PCIE8997) || defined(PCIE8897)
	mlan_pcie_data_buf *ptx_bd_buf;
#endif
#if defined(PCIE9098) || defined(PCIE9097) || defined(PCIEAW693) ||            \
	defined(PCIEIW624)
	adma_dual_desc_buf *padma_bd_buf;
#endif

	ENTER();

	for (i = 0; i < pmadapter->pcard_pcie->txrx_bd_size; i++) {
		if (pmadapter->pcard_pcie->tx_buf_list[i]) {
			pmbuf = pmadapter->pcard_pcie->tx_buf_list[i];
			pcb->moal_unmap_memory(pmadapter->pmoal_handle,
					       pmbuf->pbuf + pmbuf->data_offset,
					       pmbuf->buf_pa,
					       MLAN_RX_DATA_BUF_SIZE,
					       PCI_DMA_TODEVICE);
			wlan_write_data_complete(pmadapter, pmbuf,
						 MLAN_STATUS_FAILURE);
		}
		pmadapter->pcard_pcie->tx_buf_list[i] = MNULL;
#if defined(PCIE8997) || defined(PCIE8897)
		if (!pmadapter->pcard_pcie->reg->use_adma) {
			ptx_bd_buf =
				(mlan_pcie_data_buf *)
					pmadapter->pcard_pcie->txbd_ring[i];

			if (ptx_bd_buf) {
				ptx_bd_buf->paddr = 0;
				ptx_bd_buf->len = 0;
				ptx_bd_buf->flags = 0;
				ptx_bd_buf->frag_len = 0;
				ptx_bd_buf->offset = 0;
			}
		}
#endif

#if defined(PCIE9098) || defined(PCIE9097) || defined(PCIEAW693) ||            \
	defined(PCIEIW624)
		if (pmadapter->pcard_pcie->reg->use_adma) {
			padma_bd_buf =
				(adma_dual_desc_buf *)
					pmadapter->pcard_pcie->txbd_ring[i];

			if (padma_bd_buf) {
				padma_bd_buf->paddr = 0;
				padma_bd_buf->len = 0;
				padma_bd_buf->flags = 0;
				padma_bd_buf->pkt_size = 0;
				padma_bd_buf->reserved = 0;
			}
		}
#endif
		pmadapter->pcard_pcie->txbd_ring[i] = MNULL;
	}

	if (pmadapter->pcard_pcie->txbd_ring_vbase) {
		pmadapter->callbacks.moal_mfree_consistent(
			pmadapter->pmoal_handle,
			pmadapter->pcard_pcie->txbd_ring_size,
			pmadapter->pcard_pcie->txbd_ring_vbase,
			pmadapter->pcard_pcie->txbd_ring_pbase);
	}
	pmadapter->pcard_pcie->txbd_pending = 0;
	pmadapter->pcard_pcie->txbd_ring_size = 0;
	pmadapter->pcard_pcie->txbd_wrptr = 0;
	pmadapter->pcard_pcie->txbd_rdptr = 0;
	pmadapter->pcard_pcie->txbd_ring_vbase = MNULL;
	pmadapter->pcard_pcie->txbd_ring_pbase = 0;

	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function creates buffer descriptor ring for RX
 *
 *  @param pmadapter  A pointer to mlan_adapter structure
 *
 *  @return 	      MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status wlan_pcie_create_rxbd_ring(mlan_adapter *pmadapter)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	pmlan_callbacks pcb = &pmadapter->callbacks;
	mlan_buffer *pmbuf = MNULL;
	t_u32 i;
#if defined(PCIE8997) || defined(PCIE8897)
	mlan_pcie_data_buf *prxbd_buf;
#endif
#if defined(PCIE9098) || defined(PCIE9097) || defined(PCIEAW693) ||            \
	defined(PCIEIW624)
	adma_dual_desc_buf *padma_bd_buf;
#endif

	ENTER();

	pmadapter->pcard_pcie->rxbd_rdptr = 0;
#if defined(PCIE8997) || defined(PCIE8897)
	/*
	 * driver maintaines the write pointer and firmware maintaines the read
	 * pointer. The read pointer starts at 0 (zero) while the write pointer
	 * starts at zero with rollover bit set
	 */
	if (!pmadapter->pcard_pcie->reg->use_adma) {
		pmadapter->pcard_pcie->rxbd_wrptr =
			pmadapter->pcard_pcie->reg->txrx_rw_ptr_rollover_ind;
		/* allocate shared memory for the BD ring and divide the same in
		to several descriptors */
		pmadapter->pcard_pcie->rxbd_ring_size =
			sizeof(mlan_pcie_data_buf) *
			pmadapter->pcard_pcie->txrx_bd_size;
	}
#endif

#if defined(PCIE9098) || defined(PCIE9097) || defined(PCIEAW693) ||            \
	defined(PCIEIW624)
	/*
	 * driver maintaines the write pointer and firmware maintaines the read
	 * pointer. The read pointer starts at 0 (zero) while the write pointer
	 * starts at pmadapter->pcard_pcie->txrx_bd_size;
	 */
	if (pmadapter->pcard_pcie->reg->use_adma) {
		pmadapter->pcard_pcie->rxbd_wrptr =
			pmadapter->pcard_pcie->txrx_bd_size;
		pmadapter->pcard_pcie->rxbd_ring_size =
			sizeof(adma_dual_desc_buf) *
			pmadapter->pcard_pcie->txrx_bd_size;
	}
#endif

	PRINTM(MINFO, "RX ring: allocating %d bytes\n",
	       pmadapter->pcard_pcie->rxbd_ring_size);

	ret = pcb->moal_malloc_consistent(
		pmadapter->pmoal_handle, pmadapter->pcard_pcie->rxbd_ring_size,
		&pmadapter->pcard_pcie->rxbd_ring_vbase,
		&pmadapter->pcard_pcie->rxbd_ring_pbase);

	if (ret != MLAN_STATUS_SUCCESS) {
		PRINTM(MERROR, "%s: No free moal_malloc_consistent\n",
		       __FUNCTION__);
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}

	PRINTM(MINFO,
	       "RX ring: - base: %p, pbase: %#x:%x,"
	       "len: %#x\n",
	       pmadapter->pcard_pcie->rxbd_ring_vbase,
	       (t_u32)((t_u64)pmadapter->pcard_pcie->rxbd_ring_pbase >> 32),
	       (t_u32)pmadapter->pcard_pcie->rxbd_ring_pbase,
	       pmadapter->pcard_pcie->rxbd_ring_size);

	for (i = 0; i < pmadapter->pcard_pcie->txrx_bd_size; i++) {
		/* Allocate buffer here so that firmware can DMA data on it */
		pmbuf = wlan_alloc_mlan_buffer(pmadapter, MLAN_RX_DATA_BUF_SIZE,
					       MLAN_RX_HEADER_LEN,
					       MOAL_ALLOC_MLAN_BUFFER);
		if (!pmbuf) {
			PRINTM(MERROR,
			       "RX ring create : Unable to allocate mlan_buffer\n");
			wlan_pcie_delete_rxbd_ring(pmadapter);
			LEAVE();
			return MLAN_STATUS_FAILURE;
		}

		pmadapter->pcard_pcie->rx_buf_list[i] = pmbuf;

		if (MLAN_STATUS_FAILURE ==
		    pcb->moal_map_memory(pmadapter->pmoal_handle,
					 pmbuf->pbuf + pmbuf->data_offset,
					 &pmbuf->buf_pa, MLAN_RX_DATA_BUF_SIZE,
					 PCI_DMA_FROMDEVICE)) {
			PRINTM(MERROR,
			       "Rx ring create : moal_map_memory failed\n");
			wlan_pcie_delete_rxbd_ring(pmadapter);
			LEAVE();
			return MLAN_STATUS_FAILURE;
		}

		PRINTM(MINFO,
		       "RX ring: add new mlan_buffer base: %p, "
		       "buf_base: %p, buf_pbase: %#x:%x, "
		       "buf_len: %#x\n",
		       pmbuf, pmbuf->pbuf, (t_u32)((t_u64)pmbuf->buf_pa >> 32),
		       (t_u32)pmbuf->buf_pa, pmbuf->data_len);

#if defined(PCIE8997) || defined(PCIE8897)
		if (!pmadapter->pcard_pcie->reg->use_adma) {
			prxbd_buf =
				(mlan_pcie_data_buf
					 *)(pmadapter->pcard_pcie
						    ->rxbd_ring_vbase +
					    (sizeof(mlan_pcie_data_buf) * i));
			pmadapter->pcard_pcie->rxbd_ring[i] =
				(t_void *)prxbd_buf;
			prxbd_buf->paddr = wlan_cpu_to_le64(pmbuf->buf_pa);
			prxbd_buf->len =
				wlan_cpu_to_le16((t_u16)pmbuf->data_len);
			prxbd_buf->flags = wlan_cpu_to_le16(MLAN_BD_FLAG_SOP |
							    MLAN_BD_FLAG_EOP);
			prxbd_buf->offset = 0;
			prxbd_buf->frag_len =
				wlan_cpu_to_le16((t_u16)pmbuf->data_len);
		}
#endif

#if defined(PCIE9098) || defined(PCIE9097) || defined(PCIEAW693) ||            \
	defined(PCIEIW624)
		if (pmadapter->pcard_pcie->reg->use_adma) {
			padma_bd_buf =
				(adma_dual_desc_buf
					 *)(pmadapter->pcard_pcie
						    ->rxbd_ring_vbase +
					    (sizeof(adma_dual_desc_buf) * i));
			pmadapter->pcard_pcie->rxbd_ring[i] =
				(t_void *)padma_bd_buf;
			padma_bd_buf->paddr = wlan_cpu_to_le64(pmbuf->buf_pa);
			padma_bd_buf->len = wlan_cpu_to_le16(pmbuf->data_len);
			padma_bd_buf->flags = wlan_cpu_to_le16(
				ADMA_BD_FLAG_INT_EN | ADMA_BD_FLAG_DST_HOST);
			padma_bd_buf->pkt_size = 0;
			padma_bd_buf->reserved = 0;
		}
#endif
	}

	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function frees RX buffer descriptor ring
 *
 *  @param pmadapter  A pointer to mlan_adapter structure
 *
 *  @return 	      MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status wlan_pcie_delete_rxbd_ring(mlan_adapter *pmadapter)
{
	t_u32 i;
	pmlan_callbacks pcb = &pmadapter->callbacks;
	mlan_buffer *pmbuf = MNULL;
#if defined(PCIE8997) || defined(PCIE8897)
	mlan_pcie_data_buf *prxbd_buf;
#endif
#if defined(PCIE9098) || defined(PCIE9097) || defined(PCIEAW693) ||            \
	defined(PCIEIW624)
	adma_dual_desc_buf *padma_bd_buf;
#endif

	ENTER();
	for (i = 0; i < pmadapter->pcard_pcie->txrx_bd_size; i++) {
		if (pmadapter->pcard_pcie->rx_buf_list[i]) {
			pmbuf = pmadapter->pcard_pcie->rx_buf_list[i];
			pcb->moal_unmap_memory(pmadapter->pmoal_handle,
					       pmbuf->pbuf + pmbuf->data_offset,
					       pmbuf->buf_pa,
					       MLAN_RX_DATA_BUF_SIZE,
					       PCI_DMA_FROMDEVICE);
			wlan_free_mlan_buffer(
				pmadapter,
				pmadapter->pcard_pcie->rx_buf_list[i]);
		}
		pmadapter->pcard_pcie->rx_buf_list[i] = MNULL;

#if defined(PCIE8997) || defined(PCIE8897)
		if (!pmadapter->pcard_pcie->reg->use_adma) {
			prxbd_buf = (mlan_pcie_data_buf *)
					    pmadapter->pcard_pcie->rxbd_ring[i];

			if (prxbd_buf) {
				prxbd_buf->paddr = 0;
				prxbd_buf->offset = 0;
				prxbd_buf->frag_len = 0;
			}
		}
#endif

#if defined(PCIE9098) || defined(PCIE9097) || defined(PCIEAW693) ||            \
	defined(PCIEIW624)
		if (pmadapter->pcard_pcie->reg->use_adma) {
			padma_bd_buf =
				(adma_dual_desc_buf *)
					pmadapter->pcard_pcie->rxbd_ring[i];

			if (padma_bd_buf) {
				padma_bd_buf->paddr = 0;
				padma_bd_buf->flags = 0;
				padma_bd_buf->pkt_size = 0;
				padma_bd_buf->reserved = 0;
				padma_bd_buf->len = 0;
			}
		}
#endif
		pmadapter->pcard_pcie->rxbd_ring[i] = MNULL;
	}

	if (pmadapter->pcard_pcie->rxbd_ring_vbase)
		pmadapter->callbacks.moal_mfree_consistent(
			pmadapter->pmoal_handle,
			pmadapter->pcard_pcie->rxbd_ring_size,
			pmadapter->pcard_pcie->rxbd_ring_vbase,
			pmadapter->pcard_pcie->rxbd_ring_pbase);
	pmadapter->pcard_pcie->rxbd_ring_size = 0;
	pmadapter->pcard_pcie->rxbd_rdptr = 0;
	pmadapter->pcard_pcie->rxbd_wrptr = 0;
	pmadapter->pcard_pcie->rxbd_ring_vbase = MNULL;
	pmadapter->pcard_pcie->rxbd_ring_pbase = 0;

	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function creates buffer descriptor ring for Events
 *
 *  @param pmadapter  A pointer to mlan_adapter structure
 *
 *  @return 	      MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status wlan_pcie_create_evtbd_ring(mlan_adapter *pmadapter)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	pmlan_callbacks pcb = &pmadapter->callbacks;
	mlan_buffer *pmbuf = MNULL;
	t_u32 i;
#if defined(PCIE8997) || defined(PCIE8897)
	pmlan_pcie_evt_buf pevtbd_buf;
#endif

#if defined(PCIE9098) || defined(PCIE9097) || defined(PCIEAW693) ||            \
	defined(PCIEIW624)
	adma_dual_desc_buf *padma_bd_buf;
#endif

	ENTER();
	/*
	 * driver maintaines the write pointer and firmware maintaines the read
	 * pointer. The read pointer starts at 0 (zero) while the write pointer
	 * starts at zero with rollover bit set
	 */
	pmadapter->pcard_pcie->evtbd_rdptr = 0;
#if defined(PCIE8997) || defined(PCIE8897)
	if (!pmadapter->pcard_pcie->reg->use_adma) {
		pmadapter->pcard_pcie->evtbd_wrptr = EVT_RW_PTR_ROLLOVER_IND;
		pmadapter->pcard_pcie->evtbd_ring_size =
			sizeof(mlan_pcie_evt_buf) * MLAN_MAX_EVT_BD;
	}
#endif

#if defined(PCIE9098) || defined(PCIE9097) || defined(PCIEAW693) ||            \
	defined(PCIEIW624)
	if (pmadapter->pcard_pcie->reg->use_adma) {
		pmadapter->pcard_pcie->evtbd_wrptr = MLAN_MAX_EVT_BD;
		pmadapter->pcard_pcie->evtbd_ring_size =
			sizeof(adma_dual_desc_buf) * MLAN_MAX_EVT_BD;
	}
#endif
	PRINTM(MINFO, "Evt ring: allocating %d bytes\n",
	       pmadapter->pcard_pcie->evtbd_ring_size);

	ret = pcb->moal_malloc_consistent(
		pmadapter->pmoal_handle, pmadapter->pcard_pcie->evtbd_ring_size,
		&pmadapter->pcard_pcie->evtbd_ring_vbase,
		&pmadapter->pcard_pcie->evtbd_ring_pbase);

	if (ret != MLAN_STATUS_SUCCESS) {
		PRINTM(MERROR, "%s: No free moal_malloc_consistent\n",
		       __FUNCTION__);
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}
	PRINTM(MINFO,
	       "Evt ring: - base: %p, pbase: %#x:%x,"
	       "len: %#x\n",
	       pmadapter->pcard_pcie->evtbd_ring_vbase,
	       (t_u32)((t_u64)pmadapter->pcard_pcie->evtbd_ring_pbase >> 32),
	       (t_u32)pmadapter->pcard_pcie->evtbd_ring_pbase,
	       pmadapter->pcard_pcie->evtbd_ring_size);

	for (i = 0; i < MLAN_MAX_EVT_BD; i++) {
		/* Allocate buffer here so that firmware can DMA data on it */
		pmbuf = wlan_alloc_mlan_buffer(pmadapter, MAX_EVENT_SIZE,
					       MLAN_RX_HEADER_LEN,
					       MOAL_ALLOC_MLAN_BUFFER);
		if (!pmbuf) {
			PRINTM(MERROR,
			       "Event ring create : Unable to allocate mlan_buffer\n");
			wlan_pcie_delete_evtbd_ring(pmadapter);
			LEAVE();
			return MLAN_STATUS_FAILURE;
		}

		pmadapter->pcard_pcie->evt_buf_list[i] = pmbuf;

		if (MLAN_STATUS_FAILURE ==
		    pcb->moal_map_memory(pmadapter->pmoal_handle,
					 pmbuf->pbuf + pmbuf->data_offset,
					 &pmbuf->buf_pa, MAX_EVENT_SIZE,
					 PCI_DMA_FROMDEVICE)) {
			PRINTM(MERROR,
			       "Event ring create : moal_map_memory failed\n");
			wlan_pcie_delete_evtbd_ring(pmadapter);
			LEAVE();
			return MLAN_STATUS_FAILURE;
		}

#if defined(PCIE8997) || defined(PCIE8897)
		if (!pmadapter->pcard_pcie->reg->use_adma) {
			pevtbd_buf =
				(mlan_pcie_evt_buf
					 *)(pmadapter->pcard_pcie
						    ->evtbd_ring_vbase +
					    (sizeof(mlan_pcie_evt_buf) * i));
			pmadapter->pcard_pcie->evtbd_ring[i] =
				(t_void *)pevtbd_buf;
			pevtbd_buf->paddr = wlan_cpu_to_le64(pmbuf->buf_pa);
			pevtbd_buf->len =
				wlan_cpu_to_le16((t_u16)pmbuf->data_len);
			pevtbd_buf->flags = 0;
		}
#endif

#if defined(PCIE9098) || defined(PCIE9097) || defined(PCIEAW693) ||            \
	defined(PCIEIW624)
		if (pmadapter->pcard_pcie->reg->use_adma) {
			padma_bd_buf =
				(adma_dual_desc_buf
					 *)(pmadapter->pcard_pcie
						    ->evtbd_ring_vbase +
					    (sizeof(adma_dual_desc_buf) * i));
			pmadapter->pcard_pcie->evtbd_ring[i] =
				(t_void *)padma_bd_buf;
			padma_bd_buf->paddr = wlan_cpu_to_le64(pmbuf->buf_pa);
			padma_bd_buf->len = wlan_cpu_to_le16(pmbuf->data_len);
			padma_bd_buf->flags = wlan_cpu_to_le16(
				ADMA_BD_FLAG_INT_EN | ADMA_BD_FLAG_DST_HOST);
			padma_bd_buf->pkt_size = 0;
			padma_bd_buf->reserved = 0;
		}
#endif
	}

	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function frees event buffer descriptor ring
 *
 *  @param pmadapter  A pointer to mlan_adapter structure
 *
 *  @return 	      MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status wlan_pcie_delete_evtbd_ring(mlan_adapter *pmadapter)
{
	t_u32 i;
	pmlan_callbacks pcb = &pmadapter->callbacks;
	mlan_buffer *pmbuf = MNULL;
#if defined(PCIE8997) || defined(PCIE8897)
	mlan_pcie_evt_buf *pevtbd_buf;
#endif
#if defined(PCIE9098) || defined(PCIE9097) || defined(PCIEAW693) ||            \
	defined(PCIEIW624)
	adma_dual_desc_buf *padma_bd_buf;
#endif

	ENTER();
	for (i = 0; i < MLAN_MAX_EVT_BD; i++) {
		if (pmadapter->pcard_pcie->evt_buf_list[i]) {
			pmbuf = pmadapter->pcard_pcie->evt_buf_list[i];
			pcb->moal_unmap_memory(pmadapter->pmoal_handle,
					       pmbuf->pbuf + pmbuf->data_offset,
					       pmbuf->buf_pa, MAX_EVENT_SIZE,
					       PCI_DMA_FROMDEVICE);
			wlan_free_mlan_buffer(pmadapter, pmbuf);
		}

		pmadapter->pcard_pcie->evt_buf_list[i] = MNULL;

#if defined(PCIE8997) || defined(PCIE8897)
		if (!pmadapter->pcard_pcie->reg->use_adma) {
			pevtbd_buf =
				(mlan_pcie_evt_buf *)
					pmadapter->pcard_pcie->evtbd_ring[i];

			if (pevtbd_buf) {
				pevtbd_buf->paddr = 0;
				pevtbd_buf->len = 0;
				pevtbd_buf->flags = 0;
			}
		}
#endif

#if defined(PCIE9098) || defined(PCIE9097) || defined(PCIEAW693) ||            \
	defined(PCIEIW624)
		if (pmadapter->pcard_pcie->reg->use_adma) {
			padma_bd_buf =
				(adma_dual_desc_buf *)
					pmadapter->pcard_pcie->evtbd_ring[i];

			if (padma_bd_buf) {
				padma_bd_buf->paddr = 0;
				padma_bd_buf->len = 0;
				padma_bd_buf->flags = 0;
				padma_bd_buf->pkt_size = 0;
				padma_bd_buf->reserved = 0;
			}
		}
#endif
		pmadapter->pcard_pcie->evtbd_ring[i] = MNULL;
	}

	if (pmadapter->pcard_pcie->evtbd_ring_vbase)
		pmadapter->callbacks.moal_mfree_consistent(
			pmadapter->pmoal_handle,
			pmadapter->pcard_pcie->evtbd_ring_size,
			pmadapter->pcard_pcie->evtbd_ring_vbase,
			pmadapter->pcard_pcie->evtbd_ring_pbase);

	pmadapter->pcard_pcie->evtbd_rdptr = 0;
	pmadapter->pcard_pcie->evtbd_wrptr = 0;
	pmadapter->pcard_pcie->evtbd_ring_size = 0;
	pmadapter->pcard_pcie->evtbd_ring_vbase = MNULL;
	pmadapter->pcard_pcie->evtbd_ring_pbase = 0;

	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function allocates buffer for CMD and CMDRSP
 *
 *  @param pmadapter  A pointer to mlan_adapter structure
 *
 *  @return 	      MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status wlan_pcie_alloc_cmdrsp_buf(mlan_adapter *pmadapter)
{
	mlan_buffer *pmbuf = MNULL;
	pmlan_callbacks pcb = &pmadapter->callbacks;
	mlan_status ret = MLAN_STATUS_SUCCESS;
	/** Virtual base address of command response */
	t_u8 *cmdrsp_vbase = MNULL;
	/** Physical base address of command response */
	t_u64 cmdrsp_pbase = 0;

	ENTER();

	/* Allocate memory for receiving command response data */
	pmbuf = wlan_alloc_mlan_buffer(pmadapter, 0, 0, MOAL_MALLOC_BUFFER);
	if (!pmbuf) {
		PRINTM(MERROR,
		       "Command resp buffer create : Unable to allocate mlan_buffer\n");
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}
	ret = pcb->moal_malloc_consistent(pmadapter->pmoal_handle,
					  MRVDRV_SIZE_OF_CMD_BUFFER,
					  &cmdrsp_vbase, &cmdrsp_pbase);

	if (ret != MLAN_STATUS_SUCCESS) {
		PRINTM(MERROR, "%s: No free moal_malloc_consistent\n",
		       __FUNCTION__);
		/* free pmbuf */
		wlan_free_mlan_buffer(pmadapter, pmbuf);
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}

	pmbuf->buf_pa = cmdrsp_pbase;
	pmbuf->pbuf = cmdrsp_vbase;
	pmbuf->data_offset = 0;
	pmbuf->data_len = MRVDRV_SIZE_OF_CMD_BUFFER;
	pmbuf->total_pcie_buf_len = MRVDRV_SIZE_OF_CMD_BUFFER;
	pmadapter->pcard_pcie->cmdrsp_buf = pmbuf;

	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function frees CMD and CMDRSP buffer
 *
 *  @param pmadapter  A pointer to mlan_adapter structure
 *
 *  @return 	      MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status wlan_pcie_delete_cmdrsp_buf(mlan_adapter *pmadapter)
{
	mlan_buffer *pmbuf = MNULL;
	pmlan_callbacks pcb;
	t_u8 *cmdrsp_vbase;
	t_u64 cmdrsp_pbase;
	ENTER();

	if (!pmadapter) {
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}

	if (pmadapter->pcard_pcie->cmdrsp_buf) {
		pmbuf = pmadapter->pcard_pcie->cmdrsp_buf;
		cmdrsp_vbase = pmbuf->pbuf;
		cmdrsp_pbase = pmbuf->buf_pa;
		if (cmdrsp_vbase)
			pmadapter->callbacks.moal_mfree_consistent(
				pmadapter->pmoal_handle,
				pmbuf->total_pcie_buf_len, cmdrsp_vbase,
				cmdrsp_pbase);
		wlan_free_mlan_buffer(pmadapter, pmbuf);
		pmadapter->pcard_pcie->cmdrsp_buf = MNULL;
	}

	if (pmadapter->pcard_pcie->cmd_buf) {
		pmbuf = pmadapter->pcard_pcie->cmd_buf;
		pcb = &pmadapter->callbacks;
		pcb->moal_unmap_memory(pmadapter->pmoal_handle,
				       pmbuf->pbuf + pmbuf->data_offset,
				       pmbuf->buf_pa, MRVDRV_SIZE_OF_CMD_BUFFER,
				       PCI_DMA_TODEVICE);
	}

	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

#if defined(PCIE8997) || defined(PCIE8897)
#define PCIE_TXBD_EMPTY(wrptr, rdptr, mask, rollover_ind)                      \
	(((wrptr & mask) == (rdptr & mask)) &&                                 \
	 ((wrptr & rollover_ind) == (rdptr & rollover_ind)))

/**
 *  @brief This function flushes the TX buffer descriptor ring
 *
 *  @param pmadapter  A pointer to mlan_adapter structure
 *
 *  @return 	      MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status wlan_pcie_flush_txbd_ring(mlan_adapter *pmadapter)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	pmlan_callbacks pcb = &pmadapter->callbacks;
	t_u32 txrx_rw_ptr_mask = pmadapter->pcard_pcie->reg->txrx_rw_ptr_mask;
	t_u32 txrx_rw_ptr_rollover_ind =
		pmadapter->pcard_pcie->reg->txrx_rw_ptr_rollover_ind;

	ENTER();

	if (!PCIE_TXBD_EMPTY(pmadapter->pcard_pcie->txbd_wrptr,
			     pmadapter->pcard_pcie->txbd_rdptr,
			     txrx_rw_ptr_mask, txrx_rw_ptr_rollover_ind)) {
		pmadapter->pcard_pcie->txbd_flush = MTRUE;
		/* write pointer already set at last send */
		/* send dnld-rdy intr again, wait for completion */
		if (pcb->moal_write_reg(
			    pmadapter->pmoal_handle,
			    pmadapter->pcard_pcie->reg->reg_cpu_int_event,
			    CPU_INTR_DNLD_RDY)) {
			PRINTM(MERROR,
			       "SEND DATA (FLUSH): failed to assert dnld-rdy interrupt.\n");
			ret = MLAN_STATUS_FAILURE;
		}
	}

	LEAVE();
	return ret;
}
#endif

/**
 *  @brief This function check the tx pending buffer
 *
 *  @param pmadapter  A pointer to mlan_adapter structure
 *  @param rdptr      tx rdptr
 *
 *  @return           MTRUE/MFALSE;
 */
static t_u8 wlan_check_tx_pending_buffer(mlan_adapter *pmadapter, t_u32 rdptr)
{
#if defined(PCIE8997) || defined(PCIE8897)
	t_u32 txrx_rw_ptr_mask = pmadapter->pcard_pcie->reg->txrx_rw_ptr_mask;
	t_u32 txrx_rw_ptr_rollover_ind =
		pmadapter->pcard_pcie->reg->txrx_rw_ptr_rollover_ind;
	if (!pmadapter->pcard_pcie->reg->use_adma) {
		if (((pmadapter->pcard_pcie->txbd_rdptr & txrx_rw_ptr_mask) !=
		     (rdptr & txrx_rw_ptr_mask)) ||
		    ((pmadapter->pcard_pcie->txbd_rdptr &
		      txrx_rw_ptr_rollover_ind) !=
		     (rdptr & txrx_rw_ptr_rollover_ind)))
			return MTRUE;
		else
			return MFALSE;
	}
#endif
#if defined(PCIE9098) || defined(PCIE9097) || defined(PCIEAW693) ||            \
	defined(PCIEIW624)
	if (pmadapter->pcard_pcie->reg->use_adma) {
		if ((pmadapter->pcard_pcie->txbd_rdptr &
		     ADMA_RW_PTR_WRAP_MASK) != (rdptr & ADMA_RW_PTR_WRAP_MASK))
			return MTRUE;
		else
			return MFALSE;
	}
#endif
	return MFALSE;
}

/**
 *  @brief This function unmaps and frees downloaded data buffer
 *
 *  @param pmadapter  A pointer to mlan_adapter structure
 *
 *  @return           MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status wlan_pcie_send_data_complete(mlan_adapter *pmadapter)
{
	const t_u32 num_tx_buffs = pmadapter->pcard_pcie->txrx_bd_size;
	mlan_status ret = MLAN_STATUS_SUCCESS;
	pmlan_callbacks pcb = &pmadapter->callbacks;
	mlan_buffer *pmbuf;
	t_u32 wrdoneidx;
	t_u32 rdptr = 0;
	t_u32 unmap_count = 0;
#if defined(PCIE8997) || defined(PCIE8897)
	t_u32 txrx_rw_ptr_mask = pmadapter->pcard_pcie->reg->txrx_rw_ptr_mask;
	t_u32 txrx_rw_ptr_rollover_ind =
		pmadapter->pcard_pcie->reg->txrx_rw_ptr_rollover_ind;
	mlan_pcie_data_buf *ptx_bd_buf;
#endif
#if defined(PCIE9098) || defined(PCIE9097) || defined(PCIEAW693) ||            \
	defined(PCIEIW624)
	adma_dual_desc_buf *padma_bd_buf;
	t_u32 wrptr;
#endif

	ENTER();

	if (!wlan_is_tx_pending(pmadapter)) {
		LEAVE();
		return ret;
	}
	/* Read the TX ring read pointer set by firmware */
	if (pcb->moal_read_reg(pmadapter->pmoal_handle,
			       pmadapter->pcard_pcie->reg->reg_txbd_rdptr,
			       &rdptr)) {
		PRINTM(MERROR,
		       "func%d: SEND DATA COMP: failed to read REG_TXBD_RDPTR\n",
		       pmadapter->pcard_pcie->func_num);
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	PRINTM(MINFO, "SEND DATA COMP:  rdptr_prev=0x%x, rdptr=0x%x\n",
	       pmadapter->pcard_pcie->txbd_rdptr, rdptr);

#if defined(PCIE8997) || defined(PCIE8897)
	if (!pmadapter->pcard_pcie->reg->use_adma)
		rdptr = rdptr >> TXBD_RW_PTR_START;
#endif

#if defined(PCIE9098) || defined(PCIE9097) || defined(PCIEAW693) ||            \
	defined(PCIEIW624)
	if (pmadapter->pcard_pcie->reg->use_adma) {
		wrptr = rdptr & 0xffff;
		rdptr = rdptr >> ADMA_RPTR_START;
		if (wrptr != pmadapter->pcard_pcie->txbd_wrptr)
			PRINTM(MINFO, "wlan: Unexpected wrptr 0x%x 0x%x\n",
			       wrptr, pmadapter->pcard_pcie->txbd_wrptr);
	}
#endif

	/* free from previous txbd_rdptr to current txbd_rdptr */
	while (wlan_check_tx_pending_buffer(pmadapter, rdptr)) {
		wrdoneidx =
			pmadapter->pcard_pcie->txbd_rdptr & (num_tx_buffs - 1);
		pmbuf = pmadapter->pcard_pcie->tx_buf_list[wrdoneidx];
		if (pmbuf) {
			PRINTM(MDAT_D,
			       "SEND DATA COMP: Detach pmbuf %p at tx_ring[%d], pmadapter->txbd_rdptr=0x%x\n",
			       pmbuf, wrdoneidx,
			       pmadapter->pcard_pcie->txbd_rdptr);
			ret = pcb->moal_unmap_memory(
				pmadapter->pmoal_handle,
				pmbuf->pbuf + pmbuf->data_offset, pmbuf->buf_pa,
				pmbuf->data_len, PCI_DMA_TODEVICE);
			if (ret == MLAN_STATUS_FAILURE) {
				PRINTM(MERROR, "%s: moal_unmap_memory failed\n",
				       __FUNCTION__);
				break;
			}
			unmap_count++;
#if defined(PCIE8997) || defined(PCIE8897)
			if (pmadapter->pcard_pcie->txbd_flush)
				wlan_write_data_complete(pmadapter, pmbuf,
							 MLAN_STATUS_FAILURE);
			else
#endif
				wlan_write_data_complete(pmadapter, pmbuf,
							 MLAN_STATUS_SUCCESS);
		}

		pmadapter->pcard_pcie->tx_buf_list[wrdoneidx] = MNULL;
#if defined(PCIE8997) || defined(PCIE8897)
		if (!pmadapter->pcard_pcie->reg->use_adma) {
			ptx_bd_buf = (mlan_pcie_data_buf *)pmadapter->pcard_pcie
					     ->txbd_ring[wrdoneidx];
			ptx_bd_buf->paddr = 0;
			ptx_bd_buf->len = 0;
			ptx_bd_buf->flags = 0;
			ptx_bd_buf->frag_len = 0;
			ptx_bd_buf->offset = 0;
			pcb->moal_spin_lock(pmadapter->pmoal_handle,
					    pmadapter->pmlan_pcie_lock);
			pmadapter->pcard_pcie->txbd_rdptr++;
			if ((pmadapter->pcard_pcie->txbd_rdptr &
			     txrx_rw_ptr_mask) == num_tx_buffs)
				pmadapter->pcard_pcie->txbd_rdptr =
					((pmadapter->pcard_pcie->txbd_rdptr &
					  txrx_rw_ptr_rollover_ind) ^
					 txrx_rw_ptr_rollover_ind);
			pcb->moal_spin_unlock(pmadapter->pmoal_handle,
					      pmadapter->pmlan_pcie_lock);
		}
#endif
#if defined(PCIE9098) || defined(PCIE9097) || defined(PCIEAW693) ||            \
	defined(PCIEIW624)
		if (pmadapter->pcard_pcie->reg->use_adma) {
			padma_bd_buf =
				(adma_dual_desc_buf *)pmadapter->pcard_pcie
					->txbd_ring[wrdoneidx];
			padma_bd_buf->paddr = 0;
			padma_bd_buf->len = 0;
			padma_bd_buf->flags = 0;
			padma_bd_buf->pkt_size = 0;
			padma_bd_buf->reserved = 0;
			pcb->moal_spin_lock(pmadapter->pmoal_handle,
					    pmadapter->pmlan_pcie_lock);
			pmadapter->pcard_pcie->txbd_rdptr++;
			pmadapter->pcard_pcie->txbd_rdptr &=
				ADMA_RW_PTR_WRAP_MASK;
			pcb->moal_spin_unlock(pmadapter->pmoal_handle,
					      pmadapter->pmlan_pcie_lock);
		}
#endif
	}

#if defined(PCIE8997) || defined(PCIE8897)
	if (pmadapter->pcard_pcie->txbd_flush) {
		if (PCIE_TXBD_EMPTY(pmadapter->pcard_pcie->txbd_wrptr,
				    pmadapter->pcard_pcie->txbd_rdptr,
				    txrx_rw_ptr_mask, txrx_rw_ptr_rollover_ind))
			pmadapter->pcard_pcie->txbd_flush = MFALSE;
		else
			wlan_pcie_flush_txbd_ring(pmadapter);
	}
#endif
done:
	pcb->moal_spin_lock(pmadapter->pmoal_handle,
			    pmadapter->pmlan_pcie_lock);
	pmadapter->pcard_pcie->txbd_pending -= unmap_count;
	if (unmap_count) {
#ifdef PCIEAW693
		if (IS_PCIEAW693(pmadapter->card_type) &&
		    (wlan_pcie_get_max_msdu_cnt(pmadapter) < 2))
			pmadapter->data_sent = MTRUE;
		else
#endif
			pmadapter->data_sent = MFALSE;
	}
	pcb->moal_spin_unlock(pmadapter->pmoal_handle,
			      pmadapter->pmlan_pcie_lock);

	LEAVE();
	return ret;
}

/**
 *  @brief This function process pcie tx_complete
 *
 *
 *  @param pmadapter    A pointer to mlan_adapter structure
 *
 *  @return             N/A
 */
static void wlan_pcie_process_tx_complete(mlan_adapter *pmadapter)
{
	pmlan_callbacks pcb = &pmadapter->callbacks;
	ENTER();
	pcb->moal_spin_lock(pmadapter->pmoal_handle, pmadapter->pmlan_tx_lock);
	if (pmadapter->pcie_tx_processing) {
		pmadapter->more_tx_task_flag = MTRUE;
		pcb->moal_spin_unlock(pmadapter->pmoal_handle,
				      pmadapter->pmlan_tx_lock);
		goto exit_tx_proc;
	} else {
		pmadapter->pcie_tx_processing = MTRUE;
		pcb->moal_spin_unlock(pmadapter->pmoal_handle,
				      pmadapter->pmlan_tx_lock);
	}
tx_process_start:
	wlan_pcie_send_data_complete(pmadapter);

	pcb->moal_spin_lock(pmadapter->pmoal_handle, pmadapter->pmlan_tx_lock);
	if (pmadapter->more_tx_task_flag) {
		pmadapter->more_tx_task_flag = MFALSE;
		pcb->moal_spin_unlock(pmadapter->pmoal_handle,
				      pmadapter->pmlan_tx_lock);
		goto tx_process_start;
	}
	pmadapter->pcie_tx_processing = MFALSE;
	pcb->moal_spin_unlock(pmadapter->pmoal_handle,
			      pmadapter->pmlan_tx_lock);
exit_tx_proc:
	LEAVE();
	return;
}

#if defined(PCIE8997) || defined(PCIE8897)
#define PCIE_TXBD_NOT_FULL(wrptr, rdptr, mask, rollover_ind)                   \
	(((wrptr & mask) != (rdptr & mask)) ||                                 \
	 ((wrptr & rollover_ind) == (rdptr & rollover_ind)))
#endif

#if defined(PCIE9098) || defined(PCIE9097) || defined(PCIEAW693) ||            \
	defined(PCIEIW624)
#define ADMA_TXBD_IS_FULL(wrptr, rdptr, mask, rollover_ind)                    \
	(((wrptr & mask) == (rdptr & mask)) &&                                 \
	 ((wrptr & rollover_ind) != (rdptr & rollover_ind)))
#endif

static t_u8 wlan_check_txbd_not_full(mlan_adapter *pmadapter)
{
	t_u32 txrx_rw_ptr_mask;
	t_u32 txrx_rw_ptr_rollover_ind;
#if defined(PCIE8997) || defined(PCIE8897)
	if (!pmadapter->pcard_pcie->reg->use_adma) {
		txrx_rw_ptr_mask = pmadapter->pcard_pcie->reg->txrx_rw_ptr_mask;
		txrx_rw_ptr_rollover_ind =
			pmadapter->pcard_pcie->reg->txrx_rw_ptr_rollover_ind;
		if (PCIE_TXBD_NOT_FULL(pmadapter->pcard_pcie->txbd_wrptr,
				       pmadapter->pcard_pcie->txbd_rdptr,
				       txrx_rw_ptr_mask,
				       txrx_rw_ptr_rollover_ind))
			return MTRUE;
		else
			return MFALSE;
	}
#endif
#if defined(PCIE9098) || defined(PCIE9097) || defined(PCIEAW693) ||            \
	defined(PCIEIW624)
	if (pmadapter->pcard_pcie->reg->use_adma) {
		txrx_rw_ptr_mask = pmadapter->pcard_pcie->txrx_bd_size - 1;
		txrx_rw_ptr_rollover_ind = pmadapter->pcard_pcie->txrx_bd_size;
		if (!ADMA_TXBD_IS_FULL(pmadapter->pcard_pcie->txbd_wrptr,
				       pmadapter->pcard_pcie->txbd_rdptr,
				       txrx_rw_ptr_mask,
				       txrx_rw_ptr_rollover_ind))
			return MTRUE;
		else
			return MFALSE;
	}
#endif
	return MFALSE;
}

#if defined(PCIE9098) || defined(PCIE9097) || defined(PCIEAW693) ||            \
	defined(PCIEIW624)
#if defined(PCIEAW693)
/**
 *  @brief This function downloads data to the card.
 *
 *  @param pmadapter A pointer to mlan_adapter structure
 *
 *  @param pmbuf     A pointer to mlan_buffer (pmbuf->data_len should include
 * PCIE header)
 *  @param flag      A pointer to mlan_tx_param
 *
 *  @return          MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status wlan_pcie_send_adma_data(mlan_adapter *pmadapter,
					    mlan_buffer *pmbuf, t_u16 flags,
					    t_u16 pkt_size)
{
	t_u32 reg_txbd_wrptr = pmadapter->pcard_pcie->reg->reg_txbd_wrptr;
	adma_dual_desc_buf *padma_bd_buf = MNULL;
	const t_u32 num_tx_buffs = pmadapter->pcard_pcie->txrx_bd_size;
	mlan_status ret = MLAN_STATUS_PENDING;
	pmlan_callbacks pcb = &pmadapter->callbacks;
	t_u32 rxbd_val = 0;
	t_u32 wrindx;
	t_u32 wr_ptr_start = 0;

	ENTER();

	if (!pmbuf) {
		PRINTM(MERROR, "%s() has no buffer", __FUNCTION__);
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	if (!(pmbuf->pbuf && pmbuf->data_len)) {
		PRINTM(MERROR, "Invalid parameter <%p, %#x>\n", pmbuf->pbuf,
		       pmbuf->data_len);
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	if (num_tx_buffs == 0) {
		PRINTM(MERROR, "Invalid number of tx buffers %d \n",
		       num_tx_buffs);
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	PRINTM(MDATA,
	       "wlan_pcie_send_adma_data: len=%d flags=0x%x, pkt_size=%d\n",
	       pmbuf->data_len, flags, pkt_size);
	PRINTM(MINFO, "SEND DATA: <Rd: %#x, Wr: %#x>\n",
	       pmadapter->pcard_pcie->txbd_rdptr,
	       pmadapter->pcard_pcie->txbd_wrptr);

	if (wlan_check_txbd_not_full(pmadapter)) {
		pmadapter->data_sent = MTRUE;

		/* Map pmbuf, and attach to tx ring */
		if (MLAN_STATUS_FAILURE ==
		    pcb->moal_map_memory(pmadapter->pmoal_handle,
					 pmbuf->pbuf + pmbuf->data_offset,
					 &pmbuf->buf_pa, pmbuf->data_len,
					 PCI_DMA_TODEVICE)) {
			PRINTM(MERROR,
			       "SEND DATA: failed to moal_map_memory\n");
			ret = MLAN_STATUS_FAILURE;
			goto done;
		}
		wrindx = pmadapter->pcard_pcie->txbd_wrptr & (num_tx_buffs - 1);
		PRINTM(MDAT_D,
		       "SEND DATA: Attach pmbuf %p at tx_ring[%d], txbd_wrptr=0x%x\n",
		       pmbuf, wrindx, pmadapter->pcard_pcie->txbd_wrptr);
		pcb->moal_spin_lock(pmadapter->pmoal_handle,
				    pmadapter->pmlan_pcie_lock);
		pmadapter->pcard_pcie->tx_buf_list[wrindx] = pmbuf;
		pmadapter->pcard_pcie->txbd_pending++;
		pcb->moal_spin_unlock(pmadapter->pmoal_handle,
				      pmadapter->pmlan_pcie_lock);

		wr_ptr_start = ADMA_WPTR_START;
		padma_bd_buf = (adma_dual_desc_buf *)
				       pmadapter->pcard_pcie->txbd_ring[wrindx];
		if (!padma_bd_buf) {
			PRINTM(MERROR, "SEND DATA: padma_bd_buf is null\n");
			ret = MLAN_STATUS_FAILURE;
			goto done_unmap;
		}
		padma_bd_buf->paddr = wlan_cpu_to_le64(pmbuf->buf_pa);
		padma_bd_buf->len =
			ALIGN_SZ(pmbuf->data_len,
				 pmadapter->pcard_pcie->reg->adma_align_size);
		if (padma_bd_buf->len <
		    pmadapter->pcard_pcie->reg->adma_min_pkt_size)
			padma_bd_buf->len =
				pmadapter->pcard_pcie->reg->adma_min_pkt_size;

		padma_bd_buf->flags = ADMA_BD_FLAG_SRC_HOST;
		if (flags & ADMA_BD_FLAG_SOP) {
			padma_bd_buf->flags |= ADMA_BD_FLAG_SOP;
			padma_bd_buf->pkt_size = pkt_size;
		} else {
			padma_bd_buf->pkt_size = padma_bd_buf->len;
		}
		if (flags & ADMA_BD_FLAG_EOP) {
			wlan_get_adma_buf_flag_sliding(pmadapter, num_tx_buffs,
						       wrindx);
			padma_bd_buf->flags |= ADMA_BD_FLAG_EOP;
		}
		padma_bd_buf->flags = wlan_cpu_to_le16(padma_bd_buf->flags);
		padma_bd_buf->len = wlan_cpu_to_le16(padma_bd_buf->len);
		padma_bd_buf->pkt_size =
			wlan_cpu_to_le16(padma_bd_buf->pkt_size);
		pmadapter->pcard_pcie->last_tx_pkt_size[wrindx] =
			pmbuf->data_len;
		pmadapter->pcard_pcie->txbd_wrptr++;
		pmadapter->pcard_pcie->txbd_wrptr &= ADMA_RW_PTR_WRAP_MASK;
		DBG_HEXDUMP(MDAT_D, "adma_bd_buf", padma_bd_buf,
			    sizeof(adma_dual_desc_buf));
		if (flags & ADMA_BD_FLAG_EOP) {
			PRINTM(MINFO, "REG_TXBD_WRPT(0x%x) = 0x%x\n",
			       reg_txbd_wrptr,
			       ((pmadapter->pcard_pcie->txbd_wrptr
				 << wr_ptr_start) |
				rxbd_val));
			/* Write the TX ring write pointer in to REG_TXBD_WRPTR
			 */
			if (pcb->moal_write_reg(
				    pmadapter->pmoal_handle, reg_txbd_wrptr,
				    (pmadapter->pcard_pcie->txbd_wrptr
				     << wr_ptr_start) |
					    rxbd_val)) {
				PRINTM(MERROR,
				       "SEND DATA: failed to write REG_TXBD_WRPTR\n");
				ret = MLAN_STATUS_FAILURE;
				goto done_unmap;
			}
		}
		PRINTM(MINFO, "SEND DATA: Updated <Rd: %#x, Wr: %#x>\n",
		       pmadapter->pcard_pcie->txbd_rdptr,
		       pmadapter->pcard_pcie->txbd_wrptr);

		if (wlan_check_txbd_not_full(pmadapter)) {
#ifdef PCIEAW693
			if (IS_PCIEAW693(pmadapter->card_type) &&
			    (wlan_pcie_get_max_msdu_cnt(pmadapter) < 2)) {
				pmadapter->data_sent = MTRUE;
				wlan_pcie_process_tx_complete(pmadapter);
			} else
#endif
				pmadapter->data_sent = MFALSE;
		} else
			wlan_pcie_process_tx_complete(pmadapter);
		if (pmadapter->data_sent)
			pmadapter->data_sent_cnt++;

		PRINTM(MINFO, "Sent packet to firmware successfully\n");
	} else {
		pmadapter->data_sent = MTRUE;
		PRINTM(MERROR,
		       "TX Ring full, can't send anymore packets to firmware\n");
		PRINTM(MERROR,
		       "SEND ADMA DATA (FULL!): <Rd: %#x, Wr: %#x> txbd_pending=%d\n",
		       pmadapter->pcard_pcie->txbd_rdptr,
		       pmadapter->pcard_pcie->txbd_wrptr,
		       pmadapter->pcard_pcie->txbd_pending);
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	LEAVE();
	return ret;

done_unmap:
	if (MLAN_STATUS_FAILURE ==
	    pcb->moal_unmap_memory(
		    pmadapter->pmoal_handle, pmbuf->pbuf + pmbuf->data_offset,
		    pmbuf->buf_pa, pmbuf->data_len, PCI_DMA_TODEVICE)) {
		PRINTM(MERROR, "SEND DATA: failed to moal_unmap_memory\n");
		ret = MLAN_STATUS_FAILURE;
	}
	pcb->moal_spin_lock(pmadapter->pmoal_handle,
			    pmadapter->pmlan_pcie_lock);
	pmadapter->pcard_pcie->txbd_pending--;
	pmadapter->pcard_pcie->tx_buf_list[wrindx] = MNULL;
	pcb->moal_spin_unlock(pmadapter->pmoal_handle,
			      pmadapter->pmlan_pcie_lock);

	if (padma_bd_buf) {
		padma_bd_buf->paddr = 0;
		padma_bd_buf->len = 0;
		padma_bd_buf->flags = 0;
		padma_bd_buf->pkt_size = 0;
		padma_bd_buf->reserved = 0;
	}
done:
	LEAVE();
	return ret;
}
#endif

#endif

/**
 *  @brief This function downloads data to the card.
 *
 *  @param pmadapter A pointer to mlan_adapter structure
 *  @param type      packet type
 *  @param pmbuf     A pointer to mlan_buffer (pmbuf->data_len should include
 * PCIE header)
 *  @param tx_param  A pointer to mlan_tx_param
 *
 *  @return          MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status wlan_pcie_send_data(mlan_adapter *pmadapter, t_u8 type,
				       mlan_buffer *pmbuf,
				       mlan_tx_param *tx_param)
{
	t_u32 reg_txbd_wrptr = pmadapter->pcard_pcie->reg->reg_txbd_wrptr;
#if defined(PCIE8997) || defined(PCIE8897)
	t_u32 txrx_rw_ptr_mask = pmadapter->pcard_pcie->reg->txrx_rw_ptr_mask;
	t_u32 txrx_rw_ptr_rollover_ind =
		pmadapter->pcard_pcie->reg->txrx_rw_ptr_rollover_ind;
	mlan_pcie_data_buf *ptx_bd_buf = MNULL;
#endif
#if defined(PCIE9098) || defined(PCIE9097) || defined(PCIEAW693) ||            \
	defined(PCIEIW624)
	adma_dual_desc_buf *padma_bd_buf = MNULL;
#endif
	const t_u32 num_tx_buffs = pmadapter->pcard_pcie->txrx_bd_size;
	mlan_status ret = MLAN_STATUS_PENDING;
	mlan_status status;
	pmlan_callbacks pcb = &pmadapter->callbacks;
	t_u32 rxbd_val = 0;
	t_u32 wrindx;
	t_u16 *tmp;
	t_u8 *payload;
	t_u32 wr_ptr_start = 0;

	ENTER();

	if (pmadapter->pcard_pcie->reg->use_adma) {
		if (wlan_is_tx_pending(pmadapter))
			wlan_pcie_process_tx_complete(pmadapter);
	}

	if (!pmbuf) {
		PRINTM(MERROR, "%s() has no buffer", __FUNCTION__);
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	if (!(pmbuf->pbuf && pmbuf->data_len)) {
		PRINTM(MERROR, "Invalid parameter <%p, %#x>\n", pmbuf->pbuf,
		       pmbuf->data_len);
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	if (num_tx_buffs == 0) {
		PRINTM(MERROR, "Invalid number of tx buffers %d \n",
		       num_tx_buffs);
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	PRINTM(MINFO, "SEND DATA: <Rd: %#x, Wr: %#x>\n",
	       pmadapter->pcard_pcie->txbd_rdptr,
	       pmadapter->pcard_pcie->txbd_wrptr);

	if (wlan_check_txbd_not_full(pmadapter)) {
		pmadapter->data_sent = MTRUE;

		payload = pmbuf->pbuf + pmbuf->data_offset;
		tmp = (t_u16 *)&payload[0];
		*tmp = wlan_cpu_to_le16((t_u16)pmbuf->data_len);
		tmp = (t_u16 *)&payload[2];
		*tmp = wlan_cpu_to_le16(type);

		/* Map pmbuf, and attach to tx ring */
		if (MLAN_STATUS_FAILURE ==
		    pcb->moal_map_memory(pmadapter->pmoal_handle,
					 pmbuf->pbuf + pmbuf->data_offset,
					 &pmbuf->buf_pa, pmbuf->data_len,
					 PCI_DMA_TODEVICE)) {
			PRINTM(MERROR,
			       "SEND DATA: failed to moal_map_memory\n");
			ret = MLAN_STATUS_FAILURE;
			goto done;
		}
		wrindx = pmadapter->pcard_pcie->txbd_wrptr & (num_tx_buffs - 1);
		PRINTM(MDAT_D,
		       "SEND DATA: Attach pmbuf %p at tx_ring[%d], txbd_wrptr=0x%x\n",
		       pmbuf, wrindx, pmadapter->pcard_pcie->txbd_wrptr);

		pcb->moal_spin_lock(pmadapter->pmoal_handle,
				    pmadapter->pmlan_pcie_lock);

		pmadapter->pcard_pcie->tx_buf_list[wrindx] = pmbuf;
#if defined(PCIE8997) || defined(PCIE8897)
		if (!pmadapter->pcard_pcie->reg->use_adma) {
			wr_ptr_start = TXBD_RW_PTR_START;
			ptx_bd_buf = (mlan_pcie_data_buf *)pmadapter->pcard_pcie
					     ->txbd_ring[wrindx];
			ptx_bd_buf->paddr = wlan_cpu_to_le64(pmbuf->buf_pa);
			ptx_bd_buf->len =
				wlan_cpu_to_le16((t_u16)pmbuf->data_len);
			ptx_bd_buf->flags = wlan_cpu_to_le16(MLAN_BD_FLAG_SOP |
							     MLAN_BD_FLAG_EOP);
			ptx_bd_buf->frag_len =
				wlan_cpu_to_le16((t_u16)pmbuf->data_len);
			ptx_bd_buf->offset = 0;
			pmadapter->pcard_pcie->last_tx_pkt_size[wrindx] =
				pmbuf->data_len;

			pmadapter->pcard_pcie->txbd_wrptr++;
			if ((pmadapter->pcard_pcie->txbd_wrptr &
			     txrx_rw_ptr_mask) == num_tx_buffs)
				pmadapter->pcard_pcie->txbd_wrptr =
					((pmadapter->pcard_pcie->txbd_wrptr &
					  txrx_rw_ptr_rollover_ind) ^
					 txrx_rw_ptr_rollover_ind);
			rxbd_val = pmadapter->pcard_pcie->rxbd_wrptr &
				   pmadapter->pcard_pcie->reg
					   ->txrx_rw_ptr_wrap_mask;
		}
#endif

#if defined(PCIE9098) || defined(PCIE9097) || defined(PCIEAW693) ||            \
	defined(PCIEIW624)
		if (pmadapter->pcard_pcie->reg->use_adma) {
			wr_ptr_start = ADMA_WPTR_START;
			padma_bd_buf = (adma_dual_desc_buf *)pmadapter
					       ->pcard_pcie->txbd_ring[wrindx];
			padma_bd_buf->paddr = wlan_cpu_to_le64(pmbuf->buf_pa);
			padma_bd_buf->len = pmbuf->data_len;

#if defined(PCIEAW693)
			if (IS_PCIEAW693(pmadapter->card_type)) {
				padma_bd_buf->flags = 0;
				wlan_get_adma_buf_flag_sliding(
					pmadapter, num_tx_buffs, wrindx);
			} else
#endif
			{
				padma_bd_buf->flags = wlan_get_adma_buf_flag(
					pmadapter, num_tx_buffs, wrindx);
			}
			padma_bd_buf->flags = wlan_cpu_to_le16(
				padma_bd_buf->flags | ADMA_BD_FLAG_SOP |
				ADMA_BD_FLAG_EOP | ADMA_BD_FLAG_SRC_HOST);
			padma_bd_buf->len = wlan_cpu_to_le16(padma_bd_buf->len);
			padma_bd_buf->pkt_size = padma_bd_buf->len;
			pmadapter->pcard_pcie->last_tx_pkt_size[wrindx] =
				pmbuf->data_len;
			pmadapter->pcard_pcie->txbd_wrptr++;
			pmadapter->pcard_pcie->txbd_wrptr &=
				ADMA_RW_PTR_WRAP_MASK;
		}
#endif
		pmadapter->pcard_pcie->txbd_pending++;
		PRINTM(MINFO, "REG_TXBD_WRPT(0x%x) = 0x%x\n", reg_txbd_wrptr,
		       ((pmadapter->pcard_pcie->txbd_wrptr << wr_ptr_start) |
			rxbd_val));
		/* Write the TX ring write pointer in to REG_TXBD_WRPTR */
		status = pcb->moal_write_reg(
			pmadapter->pmoal_handle, reg_txbd_wrptr,
			(pmadapter->pcard_pcie->txbd_wrptr << wr_ptr_start) |
				rxbd_val);

		pcb->moal_spin_unlock(pmadapter->pmoal_handle,
				      pmadapter->pmlan_pcie_lock);

		if (status) {
			PRINTM(MERROR,
			       "SEND DATA: failed to write REG_TXBD_WRPTR\n");
			ret = MLAN_STATUS_FAILURE;
			goto done_unmap;
		}

		PRINTM(MINFO, "SEND DATA: Updated <Rd: %#x, Wr: %#x>\n",
		       pmadapter->pcard_pcie->txbd_rdptr,
		       pmadapter->pcard_pcie->txbd_wrptr);

		if (wlan_check_txbd_not_full(pmadapter)) {
#ifdef PCIEAW693
			if (IS_PCIEAW693(pmadapter->card_type) &&
			    (wlan_pcie_get_max_msdu_cnt(pmadapter) < 2)) {
				pmadapter->data_sent = MTRUE;
				wlan_pcie_process_tx_complete(pmadapter);
			} else
#endif
				pmadapter->data_sent = MFALSE;
		} else
			wlan_pcie_process_tx_complete(pmadapter);
		if (pmadapter->data_sent)
			pmadapter->data_sent_cnt++;

		PRINTM(MINFO, "Sent packet to firmware successfully\n");
	} else {
		pmadapter->data_sent = MTRUE;
		PRINTM(MERROR,
		       "TX Ring full, can't send anymore packets to firmware\n");
		PRINTM(MERROR,
		       "SEND DATA (FULL!): <Rd: %#x, Wr: %#x> txbd_pending=%d\n",
		       pmadapter->pcard_pcie->txbd_rdptr,
		       pmadapter->pcard_pcie->txbd_wrptr,
		       pmadapter->pcard_pcie->txbd_pending);
#if defined(PCIE8997) || defined(PCIE8897)
		if (!pmadapter->pcard_pcie->reg->use_adma) {
			/* Send the TX ready interrupt */
			if (pcb->moal_write_reg(pmadapter->pmoal_handle,
						pmadapter->pcard_pcie->reg
							->reg_cpu_int_event,
						CPU_INTR_DNLD_RDY))
				PRINTM(MERROR,
				       "SEND DATA (FULL): failed to assert dnld-rdy interrupt\n");
		}
#endif
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	LEAVE();
	return ret;

done_unmap:
	if (MLAN_STATUS_FAILURE ==
	    pcb->moal_unmap_memory(
		    pmadapter->pmoal_handle, pmbuf->pbuf + pmbuf->data_offset,
		    pmbuf->buf_pa, pmbuf->data_len, PCI_DMA_TODEVICE)) {
		PRINTM(MERROR, "SEND DATA: failed to moal_unmap_memory\n");
		ret = MLAN_STATUS_FAILURE;
	}
	pcb->moal_spin_lock(pmadapter->pmoal_handle,
			    pmadapter->pmlan_pcie_lock);
	pmadapter->pcard_pcie->txbd_pending--;
	pmadapter->pcard_pcie->tx_buf_list[wrindx] = MNULL;
	pcb->moal_spin_unlock(pmadapter->pmoal_handle,
			      pmadapter->pmlan_pcie_lock);

#if defined(PCIE8997) || defined(PCIE8897)
	if (!pmadapter->pcard_pcie->reg->use_adma && ptx_bd_buf) {
		ptx_bd_buf->paddr = 0;
		ptx_bd_buf->len = 0;
		ptx_bd_buf->flags = 0;
		ptx_bd_buf->frag_len = 0;
		ptx_bd_buf->offset = 0;
	}
#endif
#if defined(PCIE9098) || defined(PCIE9097) || defined(PCIEAW693) ||            \
	defined(PCIEIW624)
	if (pmadapter->pcard_pcie->reg->use_adma && padma_bd_buf) {
		padma_bd_buf->paddr = 0;
		padma_bd_buf->len = 0;
		padma_bd_buf->flags = 0;
		padma_bd_buf->pkt_size = 0;
		padma_bd_buf->reserved = 0;
	}
#endif
done:
	LEAVE();
	return ret;
}

/**
 *  @brief This function check the rx pending buffer
 *
 *  @param pmadapter  A pointer to mlan_adapter structure
 *  @param rdptr      rx rdptr
 *
 *  @return           MTRUE/MFALSE;
 */
static t_u8 wlan_check_rx_pending_buffer(mlan_adapter *pmadapter, t_u32 rdptr)
{
#if defined(PCIE8997) || defined(PCIE8897)
	t_u32 txrx_rw_ptr_mask = pmadapter->pcard_pcie->reg->txrx_rw_ptr_mask;
	t_u32 txrx_rw_ptr_rollover_ind =
		pmadapter->pcard_pcie->reg->txrx_rw_ptr_rollover_ind;
	if (!pmadapter->pcard_pcie->reg->use_adma) {
		if (((rdptr & txrx_rw_ptr_mask) !=
		     (pmadapter->pcard_pcie->rxbd_rdptr & txrx_rw_ptr_mask)) ||
		    ((rdptr & txrx_rw_ptr_rollover_ind) !=
		     (pmadapter->pcard_pcie->rxbd_rdptr &
		      txrx_rw_ptr_rollover_ind)))
			return MTRUE;
		else
			return MFALSE;
	}
#endif

#if defined(PCIE9098) || defined(PCIE9097) || defined(PCIEAW693) ||            \
	defined(PCIEIW624)
	if (pmadapter->pcard_pcie->reg->use_adma) {
		if ((pmadapter->pcard_pcie->rxbd_rdptr &
		     ADMA_RW_PTR_WRAP_MASK) != (rdptr & ADMA_RW_PTR_WRAP_MASK))
			return MTRUE;
		else
			return MFALSE;
	}
#endif
	return MFALSE;
}

/**
 *  @brief This function check if the rx pending buffer is full
 *
 *  @param pmadapter  A pointer to mlan_adapter structure
 *  @param rdptr           rx rdptr
 *  @param rxbd_rdptr      rxbd_rdptr
 *
 *  @return           MTRUE/MFALSE;
 */
static t_u8 wlan_is_rx_pending_full(mlan_adapter *pmadapter, t_u32 rdptr)
{
#if defined(PCIE8997) || defined(PCIE8897)
	t_u32 txrx_rw_ptr_mask = pmadapter->pcard_pcie->reg->txrx_rw_ptr_mask;
	t_u32 txrx_rw_ptr_rollover_ind =
		pmadapter->pcard_pcie->reg->txrx_rw_ptr_rollover_ind;
	if (!pmadapter->pcard_pcie->reg->use_adma) {
		PRINTM(MDATA,
		       "local wrptr: 0x%x(0x%x) -> reg rdptr: 0x%x(0x%x)\n",
		       (pmadapter->pcard_pcie->rxbd_wrptr & txrx_rw_ptr_mask),
		       (pmadapter->pcard_pcie->rxbd_wrptr &
			txrx_rw_ptr_rollover_ind),
		       (rdptr & txrx_rw_ptr_mask),
		       (rdptr & txrx_rw_ptr_rollover_ind));
		if (((rdptr & txrx_rw_ptr_mask) ==
		     (pmadapter->pcard_pcie->rxbd_wrptr & txrx_rw_ptr_mask)) &&
		    ((rdptr & txrx_rw_ptr_rollover_ind) ==
		     (pmadapter->pcard_pcie->rxbd_wrptr &
		      txrx_rw_ptr_rollover_ind)))
			return MTRUE;
		else
			return MFALSE;
	}
#endif

#if defined(PCIE9098) || defined(PCIE9097) || defined(PCIEAW693) ||            \
	defined(PCIEIW624)
	if (pmadapter->pcard_pcie->reg->use_adma) {
		PRINTM(MDATA, "local wrptr: 0x%x -> reg rdptr: 0x%x\n",
		       (pmadapter->pcard_pcie->rxbd_wrptr &
			ADMA_RW_PTR_WRAP_MASK),
		       (rdptr & ADMA_RW_PTR_WRAP_MASK));
		if ((rdptr & ADMA_RW_PTR_WRAP_MASK) ==
		    (pmadapter->pcard_pcie->rxbd_wrptr & ADMA_RW_PTR_WRAP_MASK))
			return MTRUE;
		else
			return MFALSE;
	}
#endif
	return MFALSE;
}

/**
 *  @brief This function updates allocated pmbuf to Rx BD
 *
 *  @param pmadapter  A pointer to mlan_adapter structure
 *  @param rd_index   index of rx Ring
 *  @param pmbuf      pmbuf to attach to rx Ring
 *
 */
static void wlan_pcie_rx_ring_attach_buf(mlan_adapter *pmadapter,
					 t_u32 rd_index, mlan_buffer *pmbuf)
{
#if defined(PCIE8997) || defined(PCIE8897)
	mlan_pcie_data_buf *prxbd_buf;
#endif
#if defined(PCIE9098) || defined(PCIE9097) || defined(PCIEAW693) ||            \
	defined(PCIEIW624)
	adma_dual_desc_buf *padma_bd_buf;
#endif

#if defined(PCIE8997) || defined(PCIE8897)
	if (!pmadapter->pcard_pcie->reg->use_adma) {
		prxbd_buf = (mlan_pcie_data_buf *)
				    pmadapter->pcard_pcie->rxbd_ring[rd_index];
		if (pmbuf) {
			prxbd_buf->paddr = wlan_cpu_to_le64(pmbuf->buf_pa);
			prxbd_buf->len =
				wlan_cpu_to_le16((t_u16)pmbuf->data_len);
			prxbd_buf->flags = wlan_cpu_to_le16(MLAN_BD_FLAG_SOP |
							    MLAN_BD_FLAG_EOP);
			prxbd_buf->offset = 0;
			prxbd_buf->frag_len =
				wlan_cpu_to_le16((t_u16)pmbuf->data_len);
		}
	}
#endif
#if defined(PCIE9098) || defined(PCIE9097) || defined(PCIEAW693) ||            \
	defined(PCIEIW624)
	if (pmadapter->pcard_pcie->reg->use_adma) {
		padma_bd_buf =
			(adma_dual_desc_buf *)
				pmadapter->pcard_pcie->rxbd_ring[rd_index];
		if (pmbuf) {
			padma_bd_buf->paddr = wlan_cpu_to_le64(pmbuf->buf_pa);
			padma_bd_buf->len = wlan_cpu_to_le16(pmbuf->data_len);
			padma_bd_buf->flags = wlan_cpu_to_le16(
				ADMA_BD_FLAG_INT_EN | ADMA_BD_FLAG_DST_HOST);
			padma_bd_buf->pkt_size = 0;
			padma_bd_buf->reserved = 0;
		}
	}
#endif
}

/**
 *  @brief This function updates Read, Write ptr based on update_action.
 *
 *  @param pmadapter          A pointer to mlan_adapter structure
 *  @param rd_index           rx rd_index
 *  @param update_action      RX_RD_UPDATE, RX_WR_UPDATE
 *
 *  @return                   MLAN_STATUS_SUCCESS/MLAN_STATUS_FAILURE;
 */
mlan_status wlan_pcie_rx_ring_move_rdwrptr(mlan_adapter *pmadapter,
					   t_u32 rd_index, t_u8 update_action)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	pmlan_callbacks pcb = &pmadapter->callbacks;
	t_u32 txbd_val = 0;
#if defined(PCIE8997) || defined(PCIE8897)
	t_u32 txrx_rw_ptr_mask = pmadapter->pcard_pcie->reg->txrx_rw_ptr_mask;
	t_u32 txrx_rw_ptr_rollover_ind =
		pmadapter->pcard_pcie->reg->txrx_rw_ptr_rollover_ind;
#endif
	PRINTM(MINFO, "wlan_pcie_rx_ring_move_rdwrptr %d", update_action);
#if defined(PCIE8997) || defined(PCIE8897)
	if (!pmadapter->pcard_pcie->reg->use_adma) {
		if (update_action & RX_RD_UPDATE) {
			/* update rxbd's rdptrs */
			if ((++pmadapter->pcard_pcie->rxbd_rdptr &
			     txrx_rw_ptr_mask) ==
			    pmadapter->pcard_pcie->txrx_bd_size) {
				pmadapter->pcard_pcie->rxbd_rdptr =
					((pmadapter->pcard_pcie->rxbd_rdptr &
					  txrx_rw_ptr_rollover_ind) ^
					 txrx_rw_ptr_rollover_ind);
			}
		}
		pcb->moal_spin_lock(pmadapter->pmoal_handle,
				    pmadapter->pmlan_pcie_lock);
		if (update_action & RX_WR_UPDATE) {
			/* update rxbd's wrptrs */
			if ((++pmadapter->pcard_pcie->rxbd_wrptr &
			     txrx_rw_ptr_mask) ==
			    pmadapter->pcard_pcie->txrx_bd_size) {
				pmadapter->pcard_pcie->rxbd_wrptr =
					((pmadapter->pcard_pcie->rxbd_wrptr &
					  txrx_rw_ptr_rollover_ind) ^
					 txrx_rw_ptr_rollover_ind);
			}
		}
		txbd_val = pmadapter->pcard_pcie->txbd_wrptr &
			   pmadapter->pcard_pcie->reg->txrx_rw_ptr_wrap_mask;
		txbd_val = txbd_val << TXBD_RW_PTR_START;
	}
#endif
#if defined(PCIE9098) || defined(PCIE9097) || defined(PCIEAW693) ||            \
	defined(PCIEIW624)
	if (pmadapter->pcard_pcie->reg->use_adma) {
		if (update_action & RX_RD_UPDATE) {
			pmadapter->pcard_pcie->rxbd_rdptr++;
			pmadapter->pcard_pcie->rxbd_rdptr &=
				ADMA_RW_PTR_WRAP_MASK;
		}
		if (update_action & RX_WR_UPDATE) {
			pmadapter->pcard_pcie->rxbd_wrptr++;
			pmadapter->pcard_pcie->rxbd_wrptr &=
				ADMA_RW_PTR_WRAP_MASK;
		}
	}
#endif
	PRINTM(MINFO, "RECV DATA: Updated <Wr: %#x, Rd: %#x>\n",
	       pmadapter->pcard_pcie->rxbd_wrptr,
	       pmadapter->pcard_pcie->rxbd_rdptr);
	/* Write the RX ring write pointer in to REG_RXBD_WRPTR */
	ret = pcb->moal_write_reg(pmadapter->pmoal_handle,
				  pmadapter->pcard_pcie->reg->reg_rxbd_wrptr,
				  pmadapter->pcard_pcie->rxbd_wrptr | txbd_val);
#if defined(PCIE8997) || defined(PCIE8897)
	if (!pmadapter->pcard_pcie->reg->use_adma) {
		pcb->moal_spin_unlock(pmadapter->pmoal_handle,
				      pmadapter->pmlan_pcie_lock);
	}
#endif
	if (ret) {
		PRINTM(MERROR, "RECV DATA: failed to write REG_RXBD_WRPTR\n");
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}

	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function gets called from Rx Interrupt, update pmbuf and
 * Read/write pointer.
 *
 *  @param pmadapter          A pointer to mlan_adapter structure
 *  @param rd_index           rx rd_index
 *  @param pmbuf              pointer to pmbuf
 *
 *  @return                   MLAN_STATUS_SUCCESS/MLAN_STATUS_FAILURE;
 */
static mlan_status wlan_pcie_reattach_handle(mlan_adapter *pmadapter,
					     t_u32 rd_index,
					     mlan_buffer **pmbuf)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	t_u8 update_rx_action = (RX_RD_UPDATE | RX_WR_UPDATE);
	mlan_private *priv = wlan_get_priv(pmadapter, MLAN_BSS_ROLE_ANY);

	/* Failing Rd-index(s), get re-filled under Main WQ */
	/* Update the Current RD as Last Index for Re-fill work */
	pmadapter->callbacks.moal_spin_lock(
		pmadapter->pmoal_handle,
		pmadapter->rx_refill_start_index.plock);
	if (util_scalar_read(pmadapter->pmoal_handle,
			     &pmadapter->rx_refill_start_index, MNULL,
			     MNULL) != MLAN_INVALID_TXRX_INDEX_VAL) {
		update_rx_action = RX_RD_UPDATE;
		pmadapter->rx_refill_last_index = rd_index;
	}
	pmadapter->callbacks.moal_spin_unlock(
		pmadapter->pmoal_handle,
		pmadapter->rx_refill_start_index.plock);

	/* Default action is always RD+WR */
	if (update_rx_action == (RX_RD_UPDATE | RX_WR_UPDATE)) {
		ret = wlan_pcie_reattach_pmbuf(pmadapter, rd_index, pmbuf);
		/* update failure RD index for Re-fill work (main WQ) */
		if (ret != MLAN_STATUS_SUCCESS) {
			pmadapter->callbacks.moal_spin_lock(
				pmadapter->pmoal_handle,
				pmadapter->rx_refill_start_index.plock);
			update_rx_action = RX_RD_UPDATE;
			util_scalar_write(pmadapter->pmoal_handle,
					  &pmadapter->rx_refill_start_index,
					  rd_index, MNULL, MNULL);
			pmadapter->rx_refill_last_index = rd_index;
			pmadapter->callbacks.moal_spin_unlock(
				pmadapter->pmoal_handle,
				pmadapter->rx_refill_start_index.plock);
			wlan_recv_event(priv, MLAN_EVENT_ID_DRV_DEFER_HANDLING,
					MNULL);
		}
	}
	if ((MLAN_STATUS_SUCCESS !=
	     wlan_pcie_rx_ring_move_rdwrptr(pmadapter, rd_index,
					    update_rx_action))) {
		ret = MLAN_STATUS_FAILURE;
	}

	return ret;
}

/**
 *  @brief This function allocates pmbuf for rx Ring..
 *
 *  @param pmadapter          A pointer to mlan_adapter structure
 *  @param rd_index           rx rd_index
 *  @param pmbuf              pointer to pmbuf to allocate DMA memory for Rx
 * Ring.
 *
 *  @return                   MLAN_STATUS_SUCCESS/MLAN_STATUS_FAILURE;
 */
mlan_status wlan_pcie_reattach_pmbuf(mlan_adapter *pmadapter, t_u32 rd_index,
				     mlan_buffer **pmbuf)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	pmlan_callbacks pcb = &pmadapter->callbacks;

	/* Create new buffer and attach it to Rx Ring */
	*pmbuf = wlan_alloc_mlan_buffer(pmadapter, MLAN_RX_DATA_BUF_SIZE,
					MLAN_RX_HEADER_LEN,
					MOAL_ALLOC_MLAN_BUFFER);
	if (!(*pmbuf)) {
		PRINTM(MERROR, "RECV DATA: Unable to allocate mlan_buffer %d\n",
		       rd_index);
		ret = MLAN_STATUS_FAILURE;
		pmadapter->pcard_pcie->rx_buf_list[rd_index] = MNULL;
		return ret;
	}
	if (MLAN_STATUS_FAILURE ==
	    pcb->moal_map_memory(pmadapter->pmoal_handle,
				 (*pmbuf)->pbuf + (*pmbuf)->data_offset,
				 &(*pmbuf)->buf_pa, MLAN_RX_DATA_BUF_SIZE,
				 PCI_DMA_FROMDEVICE)) {
		wlan_free_mlan_buffer(pmadapter, *pmbuf);
		ret = MLAN_STATUS_FAILURE;
		return ret;
	}
	PRINTM(MDAT_D, "RECV DATA: Attach new pmbuf %p at rx_ring[%d]\n",
	       *pmbuf, rd_index);
	pmadapter->pcard_pcie->rx_buf_list[rd_index] = *pmbuf;
	wlan_pcie_rx_ring_attach_buf(pmadapter, rd_index, *pmbuf);
	return ret;
}

/**
 *  @brief This function handles received buffer ring and
 *  dispatches packets to upper
 *
 *  @param pmadapter  A pointer to mlan_adapter structure
 *
 *  @return 	      MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status wlan_pcie_process_recv_data(mlan_adapter *pmadapter)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	pmlan_callbacks pcb = &pmadapter->callbacks;
	t_u32 rdptr = 0;
	t_u32 rd_index;
	t_s32 size = 0;
	mlan_buffer *pmbuf = MNULL;
	t_u16 rx_len = 0, rx_type;
	const t_u32 num_rx_buffs = pmadapter->pcard_pcie->txrx_bd_size;
	t_u32 reg_rxbd_rdptr = pmadapter->pcard_pcie->reg->reg_rxbd_rdptr;

	t_u32 in_ts_sec = 0;
	t_u32 in_ts_usec = 0;

	ENTER();

	/* Read the RX ring Read pointer set by firmware */
	if (pcb->moal_read_reg(pmadapter->pmoal_handle, reg_rxbd_rdptr,
			       &rdptr)) {
		PRINTM(MERROR,
		       "func%d: RECV DATA: failed to read REG_RXBD_RDPTR\n",
		       pmadapter->pcard_pcie->func_num);
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
#if defined(PCIE9098) || defined(PCIE9097) || defined(PCIEAW693) ||            \
	defined(PCIEIW624)
	if (pmadapter->pcard_pcie->reg->use_adma)
		rdptr = rdptr >> ADMA_RPTR_START;
#endif

	if (pmadapter->tp_state_on &&
	    wlan_is_rx_pending_full(pmadapter, rdptr)) {
		PRINTM(MDATA, "RX FULL!\n");
		pmadapter->callbacks.moal_tp_accounting_rx_param(
			pmadapter->pmoal_handle, 6, 0);
	}
	while (wlan_check_rx_pending_buffer(pmadapter, rdptr)) {
		if (pmadapter->rx_pkts_queued > HIGH_RX_PENDING) {
			PRINTM(MEVENT, "RX Pause\n");
			pmadapter->delay_rx_data_flag = MTRUE;
			goto done;
		}

		/* detach pmbuf (with data) from Rx Ring */
		rd_index =
			pmadapter->pcard_pcie->rxbd_rdptr & (num_rx_buffs - 1);
		size = pmadapter->pcard_pcie->txrx_bd_size - 1;
		if (rd_index > (t_u32)(size)) {
			PRINTM(MERROR, "RECV DATA: Invalid Rx buffer index.\n");
			ret = MLAN_STATUS_FAILURE;
			goto done;
		}
		pmbuf = pmadapter->pcard_pcie->rx_buf_list[rd_index];
		/* pmbuf should be always valid here.
		 * if not valid, it can be processed through Re-attach again.
		 */
		if (!pmbuf) {
			PRINTM(MERROR, "RECV DATA: invalid pmbuf");
			goto reattach;
		}
		if (MLAN_STATUS_FAILURE ==
		    pcb->moal_unmap_memory(pmadapter->pmoal_handle,
					   pmbuf->pbuf + pmbuf->data_offset,
					   pmbuf->buf_pa, MLAN_RX_DATA_BUF_SIZE,
					   PCI_DMA_FROMDEVICE)) {
			PRINTM(MERROR,
			       "RECV DATA: moal_unmap_memory failed.\n");
			ret = MLAN_STATUS_FAILURE;
			goto done;
		}
		PRINTM(MDAT_D,
		       "RECV DATA: Detach pmbuf %p at rx_ring[%d], pmadapter->rxbd_rdptr=0x%x\n",
		       pmbuf, rd_index, pmadapter->pcard_pcie->rxbd_rdptr);

		/* Get data length from interface header -
		   first 2 bytes are len, second 2 bytes are type */
		rx_len = *((t_u16 *)(pmbuf->pbuf + pmbuf->data_offset));
		rx_len = wlan_le16_to_cpu(rx_len);
		rx_type = *((t_u16 *)(pmbuf->pbuf + pmbuf->data_offset + 2));
		rx_type = wlan_le16_to_cpu(rx_type);

		PRINTM(MINFO,
		       "RECV DATA: <Wr: %#x, Rd: %#x>, Len=%d rx_type=%d\n",
		       pmadapter->pcard_pcie->rxbd_wrptr, rdptr, rx_len,
		       rx_type);

		if (rx_len && rx_len <= MLAN_RX_DATA_BUF_SIZE) {
			pmadapter->pcard_pcie->rx_buf_list[rd_index] = MNULL;
			/* send buffer to host (which will free it) */
			pmbuf->data_len = rx_len - PCIE_INTF_HEADER_LEN;
			pmbuf->data_offset += PCIE_INTF_HEADER_LEN;
			// rx_trace 5
			if (pmadapter->tp_state_on) {
				pmadapter->callbacks.moal_tp_accounting(
					pmadapter->pmoal_handle, pmbuf,
					5 /*RX_DROP_P1*/);
				pcb->moal_get_system_time(
					pmadapter->pmoal_handle, &in_ts_sec,
					&in_ts_usec);
				pmbuf->in_ts_sec = in_ts_sec;
				pmbuf->in_ts_usec = in_ts_usec;
			}
			if (pmadapter->tp_state_drop_point ==
			    5 /*RX_DROP_P1*/) {
				pmadapter->ops.data_complete(pmadapter, pmbuf,
							     ret);
			} else {
				PRINTM(MINFO,
				       "RECV DATA: Received packet from FW successfully\n");
				pmadapter->callbacks.moal_spin_lock(
					pmadapter->pmoal_handle,
					pmadapter->rx_data_queue.plock);
				util_enqueue_list_tail(
					pmadapter->pmoal_handle,
					&pmadapter->rx_data_queue,
					(pmlan_linked_list)pmbuf, MNULL, MNULL);
				pmadapter->rx_pkts_queued++;
				if (pmadapter->tp_state_on)
					pmadapter->callbacks
						.moal_tp_accounting_rx_param(
							pmadapter->pmoal_handle,
							1,
							pmadapter
								->rx_pkts_queued);
				pmadapter->callbacks.moal_spin_unlock(
					pmadapter->pmoal_handle,
					pmadapter->rx_data_queue.plock);

				pmadapter->data_received = MTRUE;
			}
		} else {
			/* Queue the mlan_buffer again.
			 * should not hit this, unless a real bigger pkt than
			 * supported size. Invalid length could be due to
			 * corrupted buffer. better to free and re-attach a new
			 * pmbuf.
			 */
			PRINTM(MERROR, "PCIE: Drop invalid packet, length=%d",
			       rx_len);
			wlan_free_mlan_buffer(pmadapter, pmbuf);
			pmbuf = MNULL;
		}
	reattach:
		if ((rx_len <= MLAN_RX_DATA_BUF_SIZE) || (!pmbuf)) {
			if (MLAN_STATUS_FAILURE ==
			    wlan_pcie_reattach_handle(pmadapter, rd_index,
						      &pmbuf)) {
				PRINTM(MERROR, "RECV DATA: reattach failed\n");
			}
		}

		if ((pmadapter->ps_state == PS_STATE_SLEEP) ||
		    (pmadapter->ps_state == PS_STATE_SLEEP_CFM))
			goto done;

		/* Read the RX ring read pointer set by firmware */
		if (pcb->moal_read_reg(pmadapter->pmoal_handle, reg_rxbd_rdptr,
				       &rdptr)) {
			PRINTM(MERROR,
			       "func%d: RECV DATA: failed to read REG_RXBD_RDPTR\n",
			       pmadapter->pcard_pcie->func_num);
			ret = MLAN_STATUS_FAILURE;
			goto done;
		}
#if defined(PCIE9098) || defined(PCIE9097) || defined(PCIEAW693) ||            \
	defined(PCIEIW624)
		if (pmadapter->pcard_pcie->reg->use_adma)
			rdptr = rdptr >> ADMA_RPTR_START;
#endif
	}

done:
	LEAVE();
	return ret;
}

/**
 *  @brief This function downloads command to the card.
 *
 *  @param pmadapter A pointer to mlan_adapter structure
 *  @param pmbuf     A pointer to mlan_buffer (pmbuf->data_len should include
 * PCIE header)
 *
 *  @return 	     MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status wlan_pcie_send_cmd(mlan_adapter *pmadapter,
				      mlan_buffer *pmbuf)
{
	mlan_status ret = MLAN_STATUS_PENDING;
	pmlan_callbacks pcb;
	t_u8 *payload = MNULL;

	ENTER();
	if (!(pmadapter && pmbuf)) {
		PRINTM(MERROR, "%s() has no buffer", __FUNCTION__);
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	if (!(pmbuf->pbuf && pmbuf->data_len)) {
		PRINTM(MERROR, "Invalid parameter <%p, %#x>\n", pmbuf->pbuf,
		       pmbuf->data_len);
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	/* Make sure a command response buffer is available */
	if (!pmadapter->pcard_pcie->cmdrsp_buf) {
		PRINTM(MERROR,
		       "No response buffer available, send command failed\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	pmadapter->cmd_sent = MTRUE;
	payload = pmbuf->pbuf + pmbuf->data_offset;
	*(t_u16 *)&payload[0] = wlan_cpu_to_le16((t_u16)pmbuf->data_len);
	*(t_u16 *)&payload[2] = wlan_cpu_to_le16(MLAN_TYPE_CMD);

	pcb = &pmadapter->callbacks;
	if (MLAN_STATUS_FAILURE ==
	    pcb->moal_map_memory(
		    pmadapter->pmoal_handle, pmbuf->pbuf + pmbuf->data_offset,
		    &pmbuf->buf_pa, MLAN_RX_CMD_BUF_SIZE, PCI_DMA_TODEVICE)) {
		PRINTM(MERROR, "Command buffer : moal_map_memory failed\n");
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}
	pmadapter->pcard_pcie->cmd_buf = pmbuf;

#if defined(PCIE8997) || defined(PCIE8897)
	if (!pmadapter->pcard_pcie->reg->use_adma) {
		/* To send a command, the driver will:
		    1. Write the 64bit physical address of the data buffer to
			SCRATCH1 + SCRATCH0
		    2. Ring the door bell (i.e. set the door bell interrupt)

		    In response to door bell interrupt, the firmware will
		   perform the DMA of the command packet (first header to obtain
		   the total length and then rest of the command).
		*/

		if (pmadapter->pcard_pcie->cmdrsp_buf) {
			/* Write the lower 32bits of the cmdrsp buffer physical
			   address */
			if (pcb->moal_write_reg(pmadapter->pmoal_handle,
						REG_CMDRSP_ADDR_LO,
						(t_u32)pmadapter->pcard_pcie
							->cmdrsp_buf->buf_pa)) {
				PRINTM(MERROR,
				       "Failed to write download command to boot code.\n");
				ret = MLAN_STATUS_FAILURE;
				goto done;
			}
			/* Write the upper 32bits of the cmdrsp buffer physical
			    address */
			if (pcb->moal_write_reg(
				    pmadapter->pmoal_handle, REG_CMDRSP_ADDR_HI,
				    (t_u32)((t_u64)pmadapter->pcard_pcie
						    ->cmdrsp_buf->buf_pa >>
					    32))) {
				PRINTM(MERROR,
				       "Failed to write download command to boot code.\n");
				ret = MLAN_STATUS_FAILURE;
				goto done;
			}
		}
		if (wlan_pcie_send_boot_cmd(pmadapter, pmbuf, MFALSE)) {
			PRINTM(MERROR, "Failed to send hostcmd to device\n");
			ret = MLAN_STATUS_FAILURE;
			goto done;
		}
	}
#endif
#if defined(PCIE9098) || defined(PCIE9097) || defined(PCIEAW693) ||            \
	defined(PCIEIW624)
	if (pmadapter->pcard_pcie->reg->use_adma) {
		/* To send a command, the driver will:
		   1. driver prepare the cmdrep buffer for adma
		   2. driver programs dma_mode field to direct programming mode
		   and programs dma_size field to define DMA data transfer size.
		   3. driver programs src_base_addr register to define source
		   location of DMA data
		   4. driver sets src_wptr to 1 to initiate DMA operation
		*/
		wlan_init_adma(pmadapter, ADMA_CMDRESP,
			       pmadapter->pcard_pcie->cmdrsp_buf->buf_pa,
			       MRVDRV_SIZE_OF_CMD_BUFFER, MFALSE);
		if (IS_PCIEIW624(pmadapter->card_type) ||
		    (IS_PCIEAW693(pmadapter->card_type) &&
		     (pmadapter->card_rev > CHIP_AW693_REV_A0))) {
			if (wlan_pcie_send_boot_cmd(pmadapter, pmbuf, MFALSE)) {
				PRINTM(MERROR,
				       "Failed to send vdll block to device\n");
				ret = MLAN_STATUS_FAILURE;
				goto done;
			}
		} else {
			wlan_init_adma(pmadapter, ADMA_CMD,
				       pmadapter->pcard_pcie->cmd_buf->buf_pa,
				       pmadapter->pcard_pcie->cmd_buf->data_len,
				       MFALSE);
		}
	}
#endif
done:
	if ((ret == MLAN_STATUS_FAILURE) && pmadapter)
		pmadapter->cmd_sent = MFALSE;

	LEAVE();
	return ret;
}

#if defined(PCIE8997) || defined(PCIE8897)
#define MLAN_SLEEP_COOKIE_DEF 0xBEEFBEEF
#define MAX_DELAY_LOOP_COUNT 100

static void mlan_delay_for_sleep_cookie(mlan_adapter *pmadapter,
					t_u32 max_delay_loop_cnt)
{
	t_u8 *buffer;
	t_u32 sleep_cookie = 0;
	t_u32 count = 0;
	pmlan_buffer pmbuf = pmadapter->pcard_pcie->cmdrsp_buf;

	for (count = 0; count < max_delay_loop_cnt; count++) {
		buffer = pmbuf->pbuf;
		sleep_cookie = *(t_u32 *)buffer;

		if (sleep_cookie == MLAN_SLEEP_COOKIE_DEF) {
			PRINTM(MINFO, "sleep cookie FOUND at count = %d!!\n",
			       count);
			break;
		}
		wlan_udelay(pmadapter, 20);
	}

	if (count >= max_delay_loop_cnt)
		PRINTM(MINFO, "sleep cookie not found!!\n");
}
#endif

/**
 *  @brief This function handles command complete interrupt
 *
 *  @param pmadapter  A pointer to mlan_adapter structure
 *
 *  @return 	      MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status wlan_pcie_process_cmd_resp(mlan_adapter *pmadapter)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	pmlan_callbacks pcb = &pmadapter->callbacks;
	pmlan_buffer pmbuf = pmadapter->pcard_pcie->cmdrsp_buf;
	pmlan_buffer cmd_buf = MNULL;
	t_u16 resp_len = 0;
	t_u8 cmd_resp_received = MFALSE;

	ENTER();

	PRINTM(MINFO, "Rx CMD Response\n");

	if (pmbuf == MNULL) {
		PRINTM(MMSG, "Rx CMD response pmbuf is null\n");
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}

	/* Get data length from interface header -
	   first 2 bytes are len, second 2 bytes are type */
	resp_len = *((t_u16 *)(pmbuf->pbuf + pmbuf->data_offset));

	pmadapter->upld_len = wlan_le16_to_cpu(resp_len);
	pmadapter->upld_len -= PCIE_INTF_HEADER_LEN;
	cmd_buf = pmadapter->pcard_pcie->cmd_buf;
	if (cmd_buf) {
		pcb->moal_unmap_memory(pmadapter->pmoal_handle,
				       cmd_buf->pbuf + cmd_buf->data_offset,
				       cmd_buf->buf_pa, WLAN_UPLD_SIZE,
				       PCI_DMA_TODEVICE);
		pmadapter->pcard_pcie->cmd_buf = MNULL;
	}
	if (!pmadapter->curr_cmd) {
		if (pmadapter->ps_state == PS_STATE_SLEEP_CFM) {
			wlan_process_sleep_confirm_resp(
				pmadapter,
				pmbuf->pbuf + pmbuf->data_offset +
					PCIE_INTF_HEADER_LEN,
				pmadapter->upld_len);
			/* We are sending sleep confirm done interrupt in the
			 * middle of sleep handshake. There is a corner case
			 * when Tx done interrupt is received from firmware
			 * during sleep handshake due to which host and firmware
			 * power states go out of sync causing Tx data timeout
			 * problem. Hence sleep confirm done interrupt is sent
			 * at the end of sleep handshake to fix the problem
			 *
			 * Host could be reading the interrupt during polling
			 * (while loop) or to address a FW interrupt. In either
			 * case, after clearing the interrupt driver needs to
			 * send a sleep confirm event at the end of processing
			 * command response right here. This marks the end of
			 * the sleep handshake with firmware.
			 */
			wlan_pcie_enable_host_int_mask(pmadapter);
			if (pcb->moal_write_reg(pmadapter->pmoal_handle,
						pmadapter->pcard_pcie->reg
							->reg_cpu_int_event,
						CPU_INTR_SLEEP_CFM_DONE)) {
				PRINTM(MERROR, "Write register failed\n");
				LEAVE();
				return MLAN_STATUS_FAILURE;
			}
#if defined(PCIE8997) || defined(PCIE8897)
			mlan_delay_for_sleep_cookie(pmadapter,
						    MAX_DELAY_LOOP_COUNT);
#endif
		}
		memcpy_ext(pmadapter, pmadapter->upld_buf,
			   pmbuf->pbuf + pmbuf->data_offset +
				   PCIE_INTF_HEADER_LEN,
			   pmadapter->upld_len, MRVDRV_SIZE_OF_CMD_BUFFER);

	} else {
		cmd_resp_received = MTRUE;
		pmbuf->data_len = pmadapter->upld_len;
		pmbuf->data_offset += PCIE_INTF_HEADER_LEN;
		pmadapter->curr_cmd->respbuf = pmbuf;

		/* Take the pointer and set it to CMD node and will
		   return in the response complete callback */
		pmadapter->pcard_pcie->cmdrsp_buf = MNULL;
#if defined(PCIE8997) || defined(PCIE8897)
		if (!pmadapter->pcard_pcie->reg->use_adma) {
			/* Clear the cmd-rsp buffer address in scratch
			registers. This will prevent firmware from writing to
			the same response buffer again. */
			if (pcb->moal_write_reg(pmadapter->pmoal_handle,
						REG_CMDRSP_ADDR_LO, 0)) {
				PRINTM(MERROR,
				       "Rx CMD: failed to clear cmd_rsp address.\n");
				ret = MLAN_STATUS_FAILURE;
				goto done;
			}
			/* Write the upper 32bits of the cmdrsp buffer physical
			    address */
			if (pcb->moal_write_reg(pmadapter->pmoal_handle,
						REG_CMDRSP_ADDR_HI, 0)) {
				PRINTM(MERROR,
				       "Rx CMD: failed to clear cmd_rsp address.\n");
				ret = MLAN_STATUS_FAILURE;
				goto done;
			}
		}
#endif
#if defined(PCIE9098) || defined(PCIE9097) || defined(PCIEAW693) ||            \
	defined(PCIEIW624)
		if (pmadapter->pcard_pcie->reg->use_adma) {
			/* Clear the cmd-rsp buffer address in adma registers.
			   This will prevent firmware from writing to the same
			   response buffer again. */
			if (wlan_init_adma(pmadapter, ADMA_CMDRESP, 0, 0,
					   MFALSE)) {
				PRINTM(MERROR,
				       "Rx CMD: failed to clear cmd_rsp address.\n");
				ret = MLAN_STATUS_FAILURE;
				goto done;
			}
		}
#endif
	}

done:
	if (cmd_resp_received) {
		wlan_request_cmd_lock(pmadapter);
		pmadapter->cmd_resp_received = cmd_resp_received;
		wlan_release_cmd_lock(pmadapter);
	}
	LEAVE();
	return ret;
}

/**
 *  @brief This function handles command response completion
 *
 *  @param pmadapter    A pointer to mlan_adapter structure
 *  @param pmbuf        A pointer to mlan_buffer
 *
 *  @return 	        MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status wlan_pcie_cmdrsp_complete(mlan_adapter *pmadapter,
					     mlan_buffer *pmbuf,
					     mlan_status status)
{
	ENTER();

	/*return the cmd response pmbuf*/
	if (pmbuf) {
		pmbuf->data_len = MRVDRV_SIZE_OF_CMD_BUFFER;
		pmbuf->data_offset -= PCIE_INTF_HEADER_LEN;
		pmadapter->pcard_pcie->cmdrsp_buf = pmbuf;
	}

	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function check pending evt buffer
 *
 *  @param pmadapter  A pointer to mlan_adapter structure
 *  @param rdptr      evt rdptr
 *
 *  @return           MTRUE/MFALSE;
 */
static t_u8 wlan_check_evt_buffer(mlan_adapter *pmadapter, t_u32 rdptr)
{
#if defined(PCIE8997) || defined(PCIE8897)
	if (!pmadapter->pcard_pcie->reg->use_adma) {
		if (((rdptr & EVT_RW_PTR_MASK) !=
		     (pmadapter->pcard_pcie->evtbd_rdptr & EVT_RW_PTR_MASK)) ||
		    ((rdptr & EVT_RW_PTR_ROLLOVER_IND) !=
		     (pmadapter->pcard_pcie->evtbd_rdptr &
		      EVT_RW_PTR_ROLLOVER_IND)))
			return MTRUE;
		else
			return MFALSE;
	}
#endif
#if defined(PCIE9098) || defined(PCIE9097) || defined(PCIEAW693) ||            \
	defined(PCIEIW624)
	if (pmadapter->pcard_pcie->reg->use_adma) {
		if ((pmadapter->pcard_pcie->evtbd_rdptr &
		     ADMA_RW_PTR_WRAP_MASK) != (rdptr & ADMA_RW_PTR_WRAP_MASK))
			return MTRUE;
		else
			return MFALSE;
	}
#endif
	return MFALSE;
}

/**
 *  @brief This function handles FW event ready interrupt
 *
 *  @param pmadapter  A pointer to mlan_adapter structure
 *
 *  @return 	      MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status wlan_pcie_process_event_ready(mlan_adapter *pmadapter)
{
	t_u32 rd_index =
		pmadapter->pcard_pcie->evtbd_rdptr & (MLAN_MAX_EVT_BD - 1);
	t_u32 rdptr = 0;
	t_u32 event;
	pmlan_callbacks pcb = &pmadapter->callbacks;
#if defined(PCIE8997) || defined(PCIE8897)
	mlan_pcie_evt_buf *pevtbd_buf;
#endif
#if defined(PCIE9098) || defined(PCIE9097) || defined(PCIEAW693) ||            \
	defined(PCIEIW624)
	adma_dual_desc_buf *padma_bd_buf;
#endif
	ENTER();

	if (rd_index >= MLAN_MAX_EVT_BD) {
		PRINTM(MINFO, "Invalid rd_index...\n");
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}

	/* Read the event ring read pointer set by firmware */
	if (pcb->moal_read_reg(pmadapter->pmoal_handle,
			       pmadapter->pcard_pcie->reg->reg_evtbd_rdptr,
			       &rdptr)) {
		PRINTM(MERROR,
		       "func%d: EvtRdy: failed to read REG_EVTBD_RDPTR\n",
		       pmadapter->pcard_pcie->func_num);
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}
#if defined(PCIE9098) || defined(PCIE9097) || defined(PCIEAW693) ||            \
	defined(PCIEIW624)
	if (pmadapter->pcard_pcie->reg->use_adma)
		rdptr = rdptr >> ADMA_RPTR_START;
#endif
	PRINTM(MINFO, "EvtRdy: Initial <Wr: 0x%x, Rd: 0x%x>\n",
	       pmadapter->pcard_pcie->evtbd_wrptr, rdptr);
	if (wlan_check_evt_buffer(pmadapter, rdptr)) {
		mlan_buffer *pmbuf_evt;
		t_u16 evt_len;

		PRINTM(MINFO, "EvtRdy: Read Index: %d\n", rd_index);
		pmbuf_evt = pmadapter->pcard_pcie->evt_buf_list[rd_index];

		/*unmap the pmbuf for CPU Access*/
		pcb->moal_unmap_memory(pmadapter->pmoal_handle,
				       pmbuf_evt->pbuf + pmbuf_evt->data_offset,
				       pmbuf_evt->buf_pa, MAX_EVENT_SIZE,
				       PCI_DMA_FROMDEVICE);

		/* Take the pointer and set it to event pointer in adapter
		   and will return back after event handling callback */
#if defined(PCIE8997) || defined(PCIE8897)
		if (!pmadapter->pcard_pcie->reg->use_adma) {
			pevtbd_buf = (mlan_pcie_evt_buf *)pmadapter->pcard_pcie
					     ->evtbd_ring[rd_index];
			pevtbd_buf->paddr = 0;
			pevtbd_buf->len = 0;
			pevtbd_buf->flags = 0;
		}
#endif

#if defined(PCIE9098) || defined(PCIE9097) || defined(PCIEAW693) ||            \
	defined(PCIEIW624)
		if (pmadapter->pcard_pcie->reg->use_adma) {
			padma_bd_buf =
				(adma_dual_desc_buf *)pmadapter->pcard_pcie
					->evtbd_ring[rd_index];
			padma_bd_buf->paddr = 0;
			padma_bd_buf->len = 0;
			padma_bd_buf->flags = 0;
			padma_bd_buf->pkt_size = 0;
			padma_bd_buf->reserved = 0;
		}
#endif
		pmadapter->pcard_pcie->evt_buf_list[rd_index] = MNULL;

		if (pmbuf_evt->data_offset >
		    UINT32_MAX - PCIE_INTF_HEADER_LEN) {
			PRINTM(MERROR,
			       "ERR: data_offset value is not correct\n");
			LEAVE();
			return MLAN_STATUS_FAILURE;
		}

		event = *((t_u32 *)&pmbuf_evt->pbuf[pmbuf_evt->data_offset +
						    PCIE_INTF_HEADER_LEN]);
		/* The first 4bytes will be the event transfer header
		   len is 2 bytes followed by type which is 2 bytes */
		evt_len = *((t_u16 *)&pmbuf_evt->pbuf[pmbuf_evt->data_offset]);
		evt_len = wlan_le16_to_cpu(evt_len);

		if ((evt_len > 0) && (evt_len > MLAN_EVENT_HEADER_LEN) &&
		    (evt_len - MLAN_EVENT_HEADER_LEN < MAX_EVENT_SIZE))
			memcpy_ext(pmadapter, pmadapter->event_body,
				   pmbuf_evt->pbuf + pmbuf_evt->data_offset +
					   MLAN_EVENT_HEADER_LEN,
				   evt_len - MLAN_EVENT_HEADER_LEN,
				   sizeof(pmadapter->event_body));

		pmbuf_evt->data_offset += PCIE_INTF_HEADER_LEN;
		pmbuf_evt->data_len = evt_len - PCIE_INTF_HEADER_LEN;
		PRINTM(MINFO, "Event length: %d\n", pmbuf_evt->data_len);

		pcb->moal_spin_lock(pmadapter->pmoal_handle,
				    pmadapter->pmlan_event_lock);
		pmadapter->pmlan_buffer_event = pmbuf_evt;
		pmadapter->event_cause = wlan_le32_to_cpu(event);
		pmadapter->event_received = MTRUE;
		pcb->moal_spin_unlock(pmadapter->pmoal_handle,
				      pmadapter->pmlan_event_lock);
		pmadapter->pcard_pcie->evtbd_rdptr++;
#if defined(PCIE8997) || defined(PCIE8897)
		if (!pmadapter->pcard_pcie->reg->use_adma) {
			if ((pmadapter->pcard_pcie->evtbd_rdptr &
			     EVT_RW_PTR_MASK) == MLAN_MAX_EVT_BD) {
				pmadapter->pcard_pcie->evtbd_rdptr =
					((pmadapter->pcard_pcie->evtbd_rdptr &
					  EVT_RW_PTR_ROLLOVER_IND) ^
					 EVT_RW_PTR_ROLLOVER_IND);
			}
		}
#endif

#if defined(PCIE9098) || defined(PCIE9097) || defined(PCIEAW693) ||            \
	defined(PCIEIW624)
		if (pmadapter->pcard_pcie->reg->use_adma)
			pmadapter->pcard_pcie->evtbd_rdptr &=
				ADMA_RW_PTR_WRAP_MASK;
#endif

		/* Do not update the event write pointer here, wait till the
		   buffer is released. This is just to make things simpler,
		   we need to find a better method of managing these buffers.
		*/
	} else {
		PRINTM(MINTR, "------>EVENT DONE\n");
		if (pcb->moal_write_reg(
			    pmadapter->pmoal_handle,
			    pmadapter->pcard_pcie->reg->reg_cpu_int_event,
			    CPU_INTR_EVENT_DONE)) {
			PRINTM(MERROR,
			       "Failed to asset event done interrupt\n");
			return MLAN_STATUS_FAILURE;
		}
	}
	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function process pcie received event
 *
 *
 *  @param pmadapter    A pointer to mlan_adapter structure
 *
 *  @return             N/A
 */
static void wlan_pcie_process_event(mlan_adapter *pmadapter)
{
	pmlan_callbacks pcb = &pmadapter->callbacks;
	ENTER();
	pcb->moal_spin_lock(pmadapter->pmoal_handle,
			    pmadapter->pmlan_event_lock);
	if (pmadapter->pcie_event_processing || pmadapter->event_received ||
	    pmadapter->event_cause) {
		pmadapter->more_event_flag = MTRUE;
		pcb->moal_spin_unlock(pmadapter->pmoal_handle,
				      pmadapter->pmlan_event_lock);
		goto exit_event_proc;
	} else {
		pmadapter->pcie_event_processing = MTRUE;
		pmadapter->more_event_flag = MFALSE;
		pcb->moal_spin_unlock(pmadapter->pmoal_handle,
				      pmadapter->pmlan_event_lock);
	}
event_process_start:
	wlan_pcie_process_event_ready(pmadapter);

	pcb->moal_spin_lock(pmadapter->pmoal_handle,
			    pmadapter->pmlan_event_lock);
	if (pmadapter->more_event_flag && !pmadapter->event_cause) {
		pmadapter->more_event_flag = MFALSE;
		pcb->moal_spin_unlock(pmadapter->pmoal_handle,
				      pmadapter->pmlan_event_lock);
		goto event_process_start;
	}
	pmadapter->pcie_event_processing = MFALSE;
	pcb->moal_spin_unlock(pmadapter->pmoal_handle,
			      pmadapter->pmlan_event_lock);
exit_event_proc:
	LEAVE();
	return;
}

/**
 *  @brief This function handles event completion
 *
 *  @param pmadapter    A pointer to mlan_adapter structure
 *  @param pmbuf        A pointer to mlan_buffer
 *
 *  @return 	        MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status wlan_pcie_event_complete(mlan_adapter *pmadapter,
					    mlan_buffer *pmbuf,
					    mlan_status status)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	pmlan_callbacks pcb = &pmadapter->callbacks;
	t_u32 wrptr =
		pmadapter->pcard_pcie->evtbd_wrptr & (MLAN_MAX_EVT_BD - 1);
	t_u32 rdptr = 0;
#if defined(PCIE8997) || defined(PCIE8897)
	mlan_pcie_evt_buf *pevtbd_buf;
#endif
#if defined(PCIE9098) || defined(PCIE9097) || defined(PCIEAW693) ||            \
	defined(PCIEIW624)
	adma_dual_desc_buf *padma_bd_buf;
#endif

	ENTER();

	if (!pmbuf) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	if (wrptr >= MLAN_MAX_EVT_BD) {
		PRINTM(MERROR, "EvtCom: Invalid wrptr 0x%x\n", wrptr);
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	/* Read the event ring read pointer set by firmware */
	if (pcb->moal_read_reg(pmadapter->pmoal_handle,
			       pmadapter->pcard_pcie->reg->reg_evtbd_rdptr,
			       &rdptr)) {
		PRINTM(MERROR,
		       "func%d: EvtCom: failed to read REG_EVTBD_RDPTR\n",
		       pmadapter->pcard_pcie->func_num);
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
#if defined(PCIE9098) || defined(PCIE9097) || defined(PCIEAW693) ||            \
	defined(PCIEIW624)
	if (pmadapter->pcard_pcie->reg->use_adma)
		rdptr = rdptr >> ADMA_RPTR_START;
#endif

	if (!pmadapter->pcard_pcie->evt_buf_list[wrptr]) {
		pmbuf->data_len = MAX_EVENT_SIZE;
		pmbuf->data_offset -= PCIE_INTF_HEADER_LEN;

		if (MLAN_STATUS_FAILURE ==
		    pcb->moal_map_memory(pmadapter->pmoal_handle,
					 pmbuf->pbuf + pmbuf->data_offset,
					 &pmbuf->buf_pa, MAX_EVENT_SIZE,
					 PCI_DMA_FROMDEVICE)) {
			PRINTM(MERROR, "EvtCom: failed to moal_map_memory\n");
			ret = MLAN_STATUS_FAILURE;
			goto done;
		}

		pmadapter->pcard_pcie->evt_buf_list[wrptr] = pmbuf;
#if defined(PCIE8997) || defined(PCIE8897)
		if (!pmadapter->pcard_pcie->reg->use_adma) {
			pevtbd_buf = (mlan_pcie_evt_buf *)pmadapter->pcard_pcie
					     ->evtbd_ring[wrptr];
			pevtbd_buf->paddr = wlan_cpu_to_le64(pmbuf->buf_pa);
			pevtbd_buf->len =
				wlan_cpu_to_le16((t_u16)pmbuf->data_len);
			pevtbd_buf->flags = 0;
		}
#endif

#if defined(PCIE9098) || defined(PCIE9097) || defined(PCIEAW693) ||            \
	defined(PCIEIW624)
		if (pmadapter->pcard_pcie->reg->use_adma) {
			padma_bd_buf = (adma_dual_desc_buf *)pmadapter
					       ->pcard_pcie->evtbd_ring[wrptr];
			padma_bd_buf->paddr = wlan_cpu_to_le64(pmbuf->buf_pa);
			padma_bd_buf->len = wlan_cpu_to_le16(pmbuf->data_len);
			padma_bd_buf->flags = 0;
			padma_bd_buf->flags = wlan_cpu_to_le16(
				ADMA_BD_FLAG_INT_EN | ADMA_BD_FLAG_DST_HOST);
			padma_bd_buf->pkt_size = 0;
			padma_bd_buf->reserved = 0;
		}
#endif
		pmbuf = MNULL;
	} else {
		PRINTM(MINFO,
		       "EvtCom: ERROR: Buffer is still valid at "
		       "index %d, <%p, %p>\n",
		       wrptr, pmadapter->pcard_pcie->evt_buf_list[wrptr],
		       pmbuf);
	}

	pmadapter->pcard_pcie->evtbd_wrptr++;

#if defined(PCIE8997) || defined(PCIE8897)
	if (!pmadapter->pcard_pcie->reg->use_adma) {
		if ((pmadapter->pcard_pcie->evtbd_wrptr & EVT_RW_PTR_MASK) ==
		    MLAN_MAX_EVT_BD) {
			pmadapter->pcard_pcie->evtbd_wrptr =
				((pmadapter->pcard_pcie->evtbd_wrptr &
				  EVT_RW_PTR_ROLLOVER_IND) ^
				 EVT_RW_PTR_ROLLOVER_IND);
		}
	}
#endif
#if defined(PCIE9098) || defined(PCIE9097) || defined(PCIEAW693) ||            \
	defined(PCIEIW624)
	if (pmadapter->pcard_pcie->reg->use_adma)
		pmadapter->pcard_pcie->evtbd_wrptr &= ADMA_RW_PTR_WRAP_MASK;
#endif
	PRINTM(MINFO, "EvtCom: Updated <Wr: 0x%x, Rd: 0x%x>\n",
	       pmadapter->pcard_pcie->evtbd_wrptr, rdptr);

	/* Write the event ring write pointer in to REG_EVTBD_WRPTR */
	if (pcb->moal_write_reg(pmadapter->pmoal_handle,
				pmadapter->pcard_pcie->reg->reg_evtbd_wrptr,
				pmadapter->pcard_pcie->evtbd_wrptr)) {
		PRINTM(MERROR, "EvtCom: failed to write REG_EVTBD_WRPTR\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

done:
	/* Free the buffer for failure case */
	if (ret && pmbuf)
		wlan_free_mlan_buffer(pmadapter, pmbuf);

	pcb->moal_spin_lock(pmadapter->pmoal_handle,
			    pmadapter->pmlan_event_lock);

	pmadapter->event_cause = 0;
	pmadapter->pmlan_buffer_event = MNULL;

	pcb->moal_spin_unlock(pmadapter->pmoal_handle,
			      pmadapter->pmlan_event_lock);

	PRINTM(MINFO, "EvtCom: Check Events Again\n");
	wlan_pcie_process_event(pmadapter);

	LEAVE();
	return ret;
}

/**
 *  @brief  This function init rx port in firmware
 *
 *  @param pmadapter	A pointer to mlan_adapter
 *
 *  @return		MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status wlan_pcie_init_fw(pmlan_adapter pmadapter)
{
	pmlan_callbacks pcb = &pmadapter->callbacks;
	t_u32 txbd_val = 0;
	mlan_status ret = MLAN_STATUS_SUCCESS;
#if defined(PCIE8997) || defined(PCIE8897)
	if (!pmadapter->pcard_pcie->reg->use_adma) {
		txbd_val = pmadapter->pcard_pcie->txbd_wrptr &
			   pmadapter->pcard_pcie->reg->txrx_rw_ptr_wrap_mask;
		txbd_val = txbd_val << TXBD_RW_PTR_START;
	}
#endif
	/* Write the RX ring write pointer in to REG_RXBD_WRPTR */
	if (pcb->moal_write_reg(pmadapter->pmoal_handle,
				pmadapter->pcard_pcie->reg->reg_rxbd_wrptr,
				pmadapter->pcard_pcie->rxbd_wrptr | txbd_val)) {
		PRINTM(MERROR, "Init FW: failed to write REG_RXBD_WRPTR\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	/* Write the event ring write pointer in to REG_EVTBD_WRPTR */
	if (pcb->moal_write_reg(pmadapter->pmoal_handle,
				pmadapter->pcard_pcie->reg->reg_evtbd_wrptr,
				pmadapter->pcard_pcie->evtbd_wrptr)) {
		PRINTM(MERROR, "Init FW: failed to write REG_EVTBD_WRPTR\n");
		ret = MLAN_STATUS_FAILURE;
	}
done:
	return ret;
}

/**
 *  @brief  This function downloads FW blocks to device
 *
 *  @param pmadapter	A pointer to mlan_adapter
 *  @param fw			A pointer to firmware image
 *
 *  @return             MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status wlan_pcie_prog_fw_w_helper(mlan_adapter *pmadapter,
					      mlan_fw_image *fw)
{
	mlan_status ret = MLAN_STATUS_FAILURE;
	const t_u8 *firmware = fw->pfw_buf;
	t_u32 firmware_len = fw->fw_len;
	t_u32 offset = 0;
	mlan_buffer *pmbuf = MNULL;
	t_u32 txlen, tries, len;
	t_u32 block_retry_cnt = 0;
	pmlan_callbacks pcb = MNULL;
#if defined(PCIE9098)
	t_u32 rev_id_reg = 0;
	t_u32 revision_id = 0;
#endif
	t_u8 check_fw_status = MFALSE;
	t_u32 fw_dnld_status = 0;
	t_u32 fw_dnld_offset = 0;
	t_u8 mic_retry = 0;

	ENTER();
	if (!pmadapter) {
		PRINTM(MERROR, "adapter structure is not valid\n");
		goto done;
	}

	if (!firmware || !firmware_len) {
		PRINTM(MERROR,
		       "No firmware image found! Terminating download\n");
		goto done;
	}

	PRINTM(MINFO, "Downloading FW image (%d bytes)\n", firmware_len);

	if (wlan_disable_pcie_host_int(pmadapter)) {
		PRINTM(MERROR, "prog_fw: Disabling interrupts failed\n");
		goto done;
	}

	pmbuf = wlan_alloc_mlan_buffer(pmadapter, WLAN_UPLD_SIZE, 0,
				       MOAL_ALLOC_MLAN_BUFFER);
	if (!pmbuf) {
		PRINTM(MERROR, "prog_fw: Unable to allocate mlan_buffer\n");
		goto done;
	}

	pcb = &pmadapter->callbacks;

#if defined(PCIE9098)
	if (IS_PCIE9098(pmadapter->card_type)) {
		rev_id_reg = pmadapter->pcard_pcie->reg->reg_rev_id;
		ret = pcb->moal_read_reg(pmadapter->pmoal_handle, rev_id_reg,
					 &revision_id);
		if (ret != MLAN_STATUS_SUCCESS) {
			PRINTM(MERROR,
			       "Failed to read PCIe revision id register\n");
			goto done;
		}
		/* Skyhawk A0, need to check both CRC and MIC error */
		if (revision_id >= CHIP_9098_REV_A0)
			check_fw_status = MTRUE;
	}
#endif
	/* For PCIE9097, need check both CRC and MIC error */
#if defined(PCIE9097)
	if (IS_PCIE9097(pmadapter->card_type))
		check_fw_status = MTRUE;
#endif
#if defined(PCIEIW624)
	if (IS_PCIEIW624(pmadapter->card_type))
		check_fw_status = MTRUE;
#endif

	/* Perform firmware data transfer */
	do {
		t_u32 ireg_intr = 0;
		t_u32 read_retry_cnt = 0;

		/* More data? */
		if (offset >= firmware_len)
			break;

		for (tries = 0; tries < MAX_POLL_TRIES; tries++) {
			ret = pcb->moal_read_reg(
				pmadapter->pmoal_handle,
				pmadapter->pcard_pcie->reg->reg_scratch_2,
				&len);
			if (ret) {
				PRINTM(MWARN,
				       "Failed reading length from boot code\n");
				goto done;
			}
			if (len || offset)
				break;
			wlan_udelay(pmadapter, 10);
		}

		if (!len) {
			break;
		} else if (len > WLAN_UPLD_SIZE) {
			PRINTM(MERROR,
			       "FW download failure @ %d, invalid length %d\n",
			       offset, len);
			goto done;
		}

		txlen = len;

		if (len & MBIT(0)) {
			if (check_fw_status) {
				/* Get offset from fw dnld offset Register */
				ret = pcb->moal_read_reg(
					pmadapter->pmoal_handle,
					pmadapter->pcard_pcie->reg
						->reg_scratch_6,
					&fw_dnld_offset);
				if (ret != MLAN_STATUS_SUCCESS) {
					PRINTM(MERROR,
					       "FW download failure @ %d, reading fw dnld offset failed\n",
					       offset);
					goto done;
				}
				/* Get CRC MIC error from fw dnld status
				 * Register */
				ret = pcb->moal_read_reg(
					pmadapter->pmoal_handle,
					pmadapter->pcard_pcie->reg
						->reg_scratch_7,
					&fw_dnld_status);
				if (ret != MLAN_STATUS_SUCCESS) {
					PRINTM(MERROR,
					       "FW download failure @ %d, reading fw dnld status failed\n",
					       offset);
					goto done;
				}
				PRINTM(MERROR,
				       "FW download error: status=0x%x offset = 0x%x fw offset = 0x%x\n",
				       fw_dnld_status, offset, fw_dnld_offset);
			}
			block_retry_cnt++;
			if (block_retry_cnt > MAX_WRITE_IOMEM_RETRY) {
				PRINTM(MERROR,
				       "FW download failure @ %d, over max "
				       "retry count\n",
				       offset);
				goto done;
			}
			PRINTM(MERROR,
			       "FW CRC error indicated by the "
			       "helper: len = 0x%04X, txlen = %d\n",
			       len, txlen);
			len &= ~MBIT(0);
			/* Setting this to 0 to resend from same offset */
			txlen = 0;
			if (fw_dnld_status & (MBIT(6) | MBIT(7))) {
				offset = 0;
				mic_retry++;
				if (mic_retry > MAX_FW_RETRY) {
					PRINTM(MERROR,
					       "FW download failure @ %d, over max "
					       "mic retry count\n",
					       offset);
					goto done;
				}
			}
		} else {
			block_retry_cnt = 0;
			/* Set blocksize to transfer - checking for last block
			 */
			if (firmware_len - offset < txlen)
				txlen = firmware_len - offset;

			PRINTM(MINFO, ".");

			/* Copy payload to buffer */
			memmove(pmadapter, pmbuf->pbuf + pmbuf->data_offset,
				&firmware[offset], txlen);
			pmbuf->data_len = txlen;
		}

		/* Send the boot command to device */
		if (wlan_pcie_send_boot_cmd(pmadapter, pmbuf, MTRUE)) {
			PRINTM(MERROR,
			       "Failed to send firmware download command\n");
			goto done;
		}
		/* Wait for the command done interrupt */
		do {
			if (read_retry_cnt > MAX_READ_REG_RETRY) {
				PRINTM(MERROR,
				       "prog_fw: Failed to get command done interrupt "
				       "retry count = %d\n",
				       read_retry_cnt);
				goto done;
			}
			if (pcb->moal_read_reg(pmadapter->pmoal_handle,
					       pmadapter->pcard_pcie->reg
						       ->reg_cpu_int_status,
					       &ireg_intr)) {
				PRINTM(MERROR,
				       "prog_fw: Failed to read "
				       "interrupt status during fw dnld\n");
				/* buffer was mapped in send_boot_cmd, unmap
				 * first */
				pcb->moal_unmap_memory(
					pmadapter->pmoal_handle,
					pmbuf->pbuf + pmbuf->data_offset,
					pmbuf->buf_pa, WLAN_UPLD_SIZE,
					PCI_DMA_TODEVICE);
				goto done;
			}
			read_retry_cnt++;
			pcb->moal_usleep_range(pmadapter->pmoal_handle, 10, 20);
		} while ((ireg_intr & CPU_INTR_DOOR_BELL) ==
			 CPU_INTR_DOOR_BELL);
		/* got interrupt - can unmap buffer now */
		if (MLAN_STATUS_FAILURE ==
		    pcb->moal_unmap_memory(pmadapter->pmoal_handle,
					   pmbuf->pbuf + pmbuf->data_offset,
					   pmbuf->buf_pa, WLAN_UPLD_SIZE,
					   PCI_DMA_TODEVICE)) {
			PRINTM(MERROR,
			       "prog_fw: failed to moal_unmap_memory\n");
			goto done;
		}
		offset += txlen;
	} while (MTRUE);

	PRINTM(MMSG, "FW download over, size %d bytes\n", offset);

	ret = MLAN_STATUS_SUCCESS;
done:
	if (pmbuf)
		wlan_free_mlan_buffer(pmadapter, pmbuf);

	LEAVE();
	return ret;
}

/********************************************************
			Global Functions
********************************************************/
/**
 *	@brief This function get pcie device from card type
 *
 *	@param pmadapter  A pointer to mlan_adapter structure
 *	@return 		  MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status wlan_get_pcie_device(pmlan_adapter pmadapter)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	t_u16 card_type = pmadapter->card_type;

	ENTER();

	ret = pmadapter->callbacks.moal_malloc(pmadapter->pmoal_handle,
					       sizeof(mlan_pcie_card),
					       MLAN_MEM_DEF,
					       (t_u8 **)&pmadapter->pcard_pcie);
	if (ret != MLAN_STATUS_SUCCESS || !pmadapter->pcard_pcie) {
		PRINTM(MERROR, "Failed to allocate pcard_pcie\n");
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}

	switch (card_type) {
#ifdef PCIE8897
	case CARD_TYPE_PCIE8897:
		pmadapter->pcard_pcie->reg = &mlan_reg_pcie8897;
		pmadapter->pcard_info = &mlan_card_info_pcie8897;
		pmadapter->pcard_pcie->txrx_bd_size = MAX_TXRX_BD;
		break;
#endif
#ifdef PCIE8997
	case CARD_TYPE_PCIE8997:
		pmadapter->pcard_pcie->reg = &mlan_reg_pcie8997;
		pmadapter->pcard_info = &mlan_card_info_pcie8997;
		pmadapter->pcard_pcie->txrx_bd_size = MAX_TXRX_BD;
		break;
#endif
#if defined(PCIE9098) || defined(PCIE9097) || defined(PCIEIW624)
	case CARD_TYPE_PCIE9097:
	case CARD_TYPE_PCIE9098:
	case CARD_TYPE_PCIEIW624:
		pmadapter->pcard_pcie->reg = &mlan_reg_pcie9098;
		pmadapter->pcard_info = &mlan_card_info_pcie9098;
		pmadapter->pcard_pcie->txrx_bd_size = ADMA_DEF_TXRX_BD;
		pmadapter->pcard_pcie->txrx_num_desc = TXRX_DEF_NUM_DESC;
		if ((card_type == CARD_TYPE_PCIE9097 &&
		     pmadapter->card_rev == CHIP_9097_REV_B0) ||
		    (card_type == CARD_TYPE_PCIEIW624))
			pmadapter->pcard_pcie->reg = &mlan_reg_pcie9097_b0;

		break;
#endif
#ifdef PCIEAW693
	case CARD_TYPE_PCIEAW693:
		pmadapter->pcard_pcie->reg = &mlan_reg_pcieaw693;
		pmadapter->pcard_info = &mlan_card_info_pcie9098;
		pmadapter->pcard_pcie->txrx_bd_size = ADMA_DEF_TXRX_BD;
		pmadapter->pcard_pcie->txrx_num_desc = TXRX_DEF_NUM_DESC;
		break;
#endif
	default:
		PRINTM(MERROR, "can't get right pcie card type \n");
		ret = MLAN_STATUS_FAILURE;
		break;
	}

	LEAVE();
	return ret;
}

/**
 *  @brief PCIE wakeup handler
 *
 *  @param pmadapter  A pointer to mlan_adapter structure
 *
 *  @return 	      MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status wlan_pcie_wakeup(mlan_adapter *pmadapter)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	t_u32 data = 0;
	ENTER();
	/* Enable interrupts or any chip access will wakeup device */
	ret = pmadapter->callbacks.moal_read_reg(
		pmadapter->pmoal_handle, pmadapter->pcard_pcie->reg->reg_ip_rev,
		&data);

	if (ret == MLAN_STATUS_SUCCESS) {
		PRINTM(MINFO,
		       "PCIE wakeup: Read PCI register to wakeup device ...\n");
	} else {
		PRINTM(MINFO, "PCIE wakeup: Failed to wakeup device ...\n");
		ret = MLAN_STATUS_FAILURE;
	}
	LEAVE();
	return ret;
}

/**
 *  @brief This function gets interrupt status.
 *
 */
/**
 *  @param msg_id  A message id
 *  @param pmadapter  A pointer to mlan_adapter structure
 *  @return         MLAN_STATUS_FAILURE -- if the intererupt is not for us
 */
static mlan_status wlan_pcie_interrupt(t_u16 msg_id, pmlan_adapter pmadapter)
{
	t_u32 pcie_ireg = 0;
	pmlan_callbacks pcb = &pmadapter->callbacks;
	t_void *pmoal_handle = pmadapter->pmoal_handle;
	t_void *pint_lock = pmadapter->pint_lock;
	mlan_status ret = MLAN_STATUS_SUCCESS;

	ENTER();

	if (pmadapter->pcard_pcie->pcie_int_mode == PCIE_INT_MODE_MSI) {
		pcb->moal_spin_lock(pmoal_handle, pint_lock);
		pmadapter->ireg = 1;
		pcb->moal_spin_unlock(pmoal_handle, pint_lock);
	} else {
		wlan_pcie_disable_host_int_mask(pmadapter);
		if (pcb->moal_read_reg(
			    pmoal_handle,
			    pmadapter->pcard_pcie->reg->reg_host_int_status,
			    &pcie_ireg)) {
			PRINTM(MERROR, "Read func%d register failed\n",
			       pmadapter->pcard_pcie->func_num);
			LEAVE();
			return MLAN_STATUS_FAILURE;
		}

		if ((pcie_ireg != 0xFFFFFFFF) && (pcie_ireg)) {
			PRINTM(MINTR, "func%d: pcie_ireg=0x%x\n",
			       pmadapter->pcard_pcie->func_num, pcie_ireg);
			if (!pmadapter->pps_uapsd_mode &&
			    (pmadapter->ps_state == PS_STATE_SLEEP)) {
				/* Potentially for PCIe we could get other
				 * interrupts like shared. */
				pmadapter->pm_wakeup_fw_try = MFALSE;
				pmadapter->ps_state = PS_STATE_AWAKE;
				pmadapter->pm_wakeup_card_req = MFALSE;
			}
			pcb->moal_spin_lock(pmoal_handle, pint_lock);
			pmadapter->ireg |= pcie_ireg;
			pcb->moal_spin_unlock(pmoal_handle, pint_lock);

			/* Clear the pending interrupts */
			if (pcb->moal_write_reg(pmoal_handle,
						pmadapter->pcard_pcie->reg
							->reg_host_int_status,
						~pcie_ireg)) {
				PRINTM(MWARN, "Write register failed\n");
				LEAVE();
				return ret;
			}
		} else {
			wlan_pcie_enable_host_int_mask(pmadapter);
			PRINTM(MINFO, "This is not our interrupt\n");
			ret = MLAN_STATUS_FAILURE;
		}
	}

	LEAVE();
	return ret;
}

/**
 *  @brief This function checks the interrupt status and
 *  handle it accordingly.
 *
 *  @param pmadapter    A pointer to mlan_adapter structure
 *
 *  @return             MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status wlan_process_pcie_int_status(mlan_adapter *pmadapter)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	t_u32 pcie_ireg = 0;
	pmlan_callbacks pcb = &pmadapter->callbacks;

	ENTER();

	pcb->moal_spin_lock(pmadapter->pmoal_handle, pmadapter->pint_lock);
	if (pmadapter->pcard_pcie->pcie_int_mode != PCIE_INT_MODE_MSI)
		pcie_ireg = pmadapter->ireg;
	pmadapter->ireg = 0;
	pcb->moal_spin_unlock(pmadapter->pmoal_handle, pmadapter->pint_lock);
	if (pmadapter->pcard_pcie->pcie_int_mode == PCIE_INT_MODE_MSI) {
		if (pcb->moal_read_reg(
			    pmadapter->pmoal_handle,
			    pmadapter->pcard_pcie->reg->reg_host_int_status,
			    &pcie_ireg)) {
			PRINTM(MERROR, "Read func%d register failed\n",
			       pmadapter->pcard_pcie->func_num);
			ret = MLAN_STATUS_FAILURE;
			goto done;
		}

		if ((pcie_ireg != 0xFFFFFFFF) && (pcie_ireg)) {
			PRINTM(MINTR, "func%d: pcie_ireg=0x%x\n",
			       pmadapter->pcard_pcie->func_num, pcie_ireg);
			if (pmadapter->pcard_pcie->reg->msi_int_wr_clr) {
				if (pcb->moal_write_reg(
					    pmadapter->pmoal_handle,
					    pmadapter->pcard_pcie->reg
						    ->reg_host_int_status,
					    ~pcie_ireg)) {
					PRINTM(MERROR,
					       "Write register failed\n");
					ret = MLAN_STATUS_FAILURE;
					goto done;
				}
			}
			if (!pmadapter->pps_uapsd_mode &&
			    (pmadapter->ps_state == PS_STATE_SLEEP)) {
				/* Potentially for PCIe we could get other
				 * interrupts like shared. */
				pmadapter->pm_wakeup_fw_try = MFALSE;
				pmadapter->ps_state = PS_STATE_AWAKE;
				pmadapter->pm_wakeup_card_req = MFALSE;
			}
		}
	}
	while (pcie_ireg & pmadapter->pcard_pcie->reg->host_intr_mask) {
		if (pcie_ireg &
		    pmadapter->pcard_pcie->reg->host_intr_dnld_done) {
			pcie_ireg &=
				~pmadapter->pcard_pcie->reg->host_intr_dnld_done;
			PRINTM(MINFO, "<--- DATA sent Interrupt --->\n");
			pmadapter->callbacks.moal_tp_accounting_rx_param(
				pmadapter->pmoal_handle, 3, 0);
			wlan_recv_event(
				wlan_get_priv(pmadapter, MLAN_BSS_ROLE_ANY),
				MLAN_EVENT_ID_DRV_DEFER_TX_COMPLTE, MNULL);
		}
		if (pcie_ireg &
		    pmadapter->pcard_pcie->reg->host_intr_upld_rdy) {
			pcie_ireg &=
				~pmadapter->pcard_pcie->reg->host_intr_upld_rdy;
			PRINTM(MINFO, "Rx DATA\n");
			pcb->moal_spin_lock(pmadapter->pmoal_handle,
					    pmadapter->pmlan_rx_lock);
			pmadapter->pcard_pcie->rx_pending = MTRUE;
			pcb->moal_spin_unlock(pmadapter->pmoal_handle,
					      pmadapter->pmlan_rx_lock);
			pmadapter->callbacks.moal_tp_accounting_rx_param(
				pmadapter->pmoal_handle, 0, 0);
			wlan_recv_event(wlan_get_priv(pmadapter,
						      MLAN_BSS_ROLE_ANY),
					MLAN_EVENT_ID_DRV_DEFER_RX_DATA, MNULL);
			if (wlan_is_tx_pending(pmadapter))
				wlan_recv_event(
					wlan_get_priv(pmadapter,
						      MLAN_BSS_ROLE_ANY),
					MLAN_EVENT_ID_DRV_DEFER_TX_COMPLTE,
					MNULL);
		}
		if (pcie_ireg &
		    pmadapter->pcard_pcie->reg->host_intr_event_rdy) {
			pcie_ireg &=
				~pmadapter->pcard_pcie->reg->host_intr_event_rdy;
			PRINTM(MINFO, "Rx EVENT\n");
			pmadapter->pcie_event_int = MTRUE;
			wlan_recv_event(
				wlan_get_priv(pmadapter, MLAN_BSS_ROLE_ANY),
				MLAN_EVENT_ID_DRV_DEFER_HANDLING, MNULL);
		}
		if (pcie_ireg &
		    pmadapter->pcard_pcie->reg->host_intr_cmd_done) {
			pcie_ireg &=
				~pmadapter->pcard_pcie->reg->host_intr_cmd_done;
			if (pmadapter->cmd_sent &&
			    !pmadapter->vdll_ctrl.vdll_len) {
				PRINTM(MINFO, "<--- CMD sent Interrupt --->\n");
				pmadapter->cmd_sent = MFALSE;
			}
			wlan_recv_event(wlan_get_priv(pmadapter,
						      MLAN_BSS_ROLE_ANY),
					MLAN_EVENT_ID_DRV_DEFER_CMDRESP, MNULL);
		}
#if defined(PCIE8997) || defined(PCIE9098) || defined(PCIE9097) ||             \
	defined(PCIEAW693) || defined(PCIEIW624)
		if (pmadapter->pcard_pcie->reg->host_intr_cmd_dnld &&
		    (pcie_ireg &
		     pmadapter->pcard_pcie->reg->host_intr_cmd_dnld)) {
			pcie_ireg &=
				~pmadapter->pcard_pcie->reg->host_intr_cmd_dnld;
			pmadapter->pcie_cmd_dnld_int = MTRUE;
			PRINTM(MINFO, "<--- CMD DNLD DONE Interrupt --->\n");
			wlan_recv_event(
				wlan_get_priv(pmadapter, MLAN_BSS_ROLE_ANY),
				MLAN_EVENT_ID_DRV_DEFER_HANDLING, MNULL);
		}
#endif
		if (pmadapter->pcard_pcie->pcie_int_mode == PCIE_INT_MODE_MSI) {
			pcb->moal_spin_lock(pmadapter->pmoal_handle,
					    pmadapter->pint_lock);
			pmadapter->ireg = 0;
			pcb->moal_spin_unlock(pmadapter->pmoal_handle,
					      pmadapter->pint_lock);
		}
		if (pcb->moal_read_reg(
			    pmadapter->pmoal_handle,
			    pmadapter->pcard_pcie->reg->reg_host_int_status,
			    &pcie_ireg)) {
			PRINTM(MERROR, "Read func%d register failed\n",
			       pmadapter->pcard_pcie->func_num);
			ret = MLAN_STATUS_FAILURE;
			goto done;
		}

		if ((pcie_ireg != 0xFFFFFFFF) && (pcie_ireg)) {
			PRINTM(MINTR, "func%d: Poll pcie_ireg=0x%x\n",
			       pmadapter->pcard_pcie->func_num, pcie_ireg);
			if ((pmadapter->pcard_pcie->pcie_int_mode ==
			     PCIE_INT_MODE_LEGACY) ||
			    pmadapter->pcard_pcie->reg->msi_int_wr_clr) {
				if (pcb->moal_write_reg(
					    pmadapter->pmoal_handle,
					    pmadapter->pcard_pcie->reg
						    ->reg_host_int_status,
					    ~pcie_ireg)) {
					PRINTM(MWARN,
					       "Write register failed\n");
					return MLAN_STATUS_FAILURE;
				}
			}
			if (pmadapter->pcard_pcie->pcie_int_mode !=
			    PCIE_INT_MODE_MSI) {
				pcb->moal_spin_lock(pmadapter->pmoal_handle,
						    pmadapter->pint_lock);
				pcie_ireg |= pmadapter->ireg;
				pmadapter->ireg = 0;
				pcb->moal_spin_unlock(pmadapter->pmoal_handle,
						      pmadapter->pint_lock);
			}
			/* Don't update the pmadapter->pcie_ireg,
			 * serving the status right now */
		}
	}
	PRINTM(MINFO, "cmd_sent=%d data_sent=%d\n", pmadapter->cmd_sent,
	       pmadapter->data_sent);
	if (pmadapter->pcard_pcie->pcie_int_mode != PCIE_INT_MODE_MSI) {
		if (pmadapter->ps_state != PS_STATE_SLEEP ||
		    pmadapter->pcard_info->supp_ps_handshake)
			wlan_pcie_enable_host_int_mask(pmadapter);
	}
done:
	LEAVE();
	return ret;
}

/**
 *  @brief This function sets DRV_READY register
 *
 *  @param pmadapter  A pointer to mlan_adapter structure
 *  @param val        Value
 *
 *  @return           MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 *
 */
mlan_status wlan_set_drv_ready_reg(mlan_adapter *pmadapter, t_u32 val)
{
	pmlan_callbacks pcb = &pmadapter->callbacks;

	ENTER();

	if (pcb->moal_write_reg(pmadapter->pmoal_handle,
				pmadapter->pcard_pcie->reg->reg_drv_ready,
				val)) {
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}

	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function checks if the interface is ready to download
 *  or not while other download interface is present
 *
 *  @param pmadapter  A pointer to mlan_adapter structure
 *  @param val        Winner status (0: winner)
 *
 *  @return           MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 *
 */
static mlan_status wlan_pcie_check_winner_status(mlan_adapter *pmadapter,
						 t_u32 *val)
{
	t_u32 winner = 0;
	pmlan_callbacks pcb = &pmadapter->callbacks;

	ENTER();

	if (MLAN_STATUS_SUCCESS !=
	    pcb->moal_read_reg(pmadapter->pmoal_handle,
			       pmadapter->pcard_pcie->reg->reg_scratch_3,
			       &winner)) {
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}
	*val = winner;

	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function checks if the firmware is ready to accept
 *  command or not.
 *
 *  @param pmadapter  A pointer to mlan_adapter structure
 *  @param pollnum    Maximum polling number
 *
 *  @return           MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status wlan_pcie_check_fw_status(mlan_adapter *pmadapter,
					     t_u32 pollnum)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	pmlan_callbacks pcb = &pmadapter->callbacks;
	t_u32 firmware_stat = 0;
	t_u32 tries;

	ENTER();

	/* Wait for firmware initialization event */
	for (tries = 0; tries < pollnum; tries++) {
		if (pcb->moal_read_reg(pmadapter->pmoal_handle,
				       pmadapter->pcard_pcie->reg->reg_scratch_3,
				       &firmware_stat))
			ret = MLAN_STATUS_FAILURE;
		else
			ret = MLAN_STATUS_SUCCESS;
		if (ret)
			continue;
		if (firmware_stat == PCIE_FIRMWARE_READY) {
			ret = MLAN_STATUS_SUCCESS;
			break;
		} else {
			wlan_mdelay(pmadapter, 10);
			ret = MLAN_STATUS_FAILURE;
		}
	}

	LEAVE();
	return ret;
}

/**
 *  @brief This function init the pcie interface
 *
 *  @param pmadapter  A pointer to mlan_adapter structure
 *
 *  @return           MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status wlan_pcie_init(mlan_adapter *pmadapter)
{
	ENTER();

	PRINTM(MINFO, "Setting driver ready signature\n");
	if (wlan_set_drv_ready_reg(pmadapter, PCIE_FIRMWARE_READY)) {
		PRINTM(MERROR, "Failed to write driver ready signature\n");
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}
	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief  This function downloads firmware to card
 *
 *  @param pmadapter	A pointer to mlan_adapter
 *  @param pmfw			A pointer to firmware image
 *
 *  @return		MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status wlan_pcie_dnld_fw(pmlan_adapter pmadapter,
				     pmlan_fw_image pmfw)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	t_u32 poll_num = 1;
	t_u32 winner = 0;

	ENTER();

	ret = wlan_pcie_init(pmadapter);
	if (ret == MLAN_STATUS_FAILURE) {
		PRINTM(MERROR, "WLAN PCIE init failed\n", ret);
		LEAVE();
		return ret;
	}
	/* Check if firmware is already running */
	ret = wlan_pcie_check_fw_status(pmadapter, poll_num);
	if (ret == MLAN_STATUS_SUCCESS) {
		PRINTM(MMSG, "WLAN FW already running! Skip FW download\n");
		goto done;
	}
	poll_num = MAX_FIRMWARE_POLL_TRIES;

	/* Check if other interface is downloading */
	ret = wlan_pcie_check_winner_status(pmadapter, &winner);
	if (ret == MLAN_STATUS_FAILURE) {
		PRINTM(MFATAL, "WLAN read winner status failed!\n");
		goto done;
	}
	if (winner) {
		PRINTM(MMSG,
		       "WLAN is not the winner (0x%x). Skip FW download\n",
		       winner);
		poll_num = MAX_MULTI_INTERFACE_POLL_TRIES;
		goto poll_fw;
	}

	/* Download the firmware image via helper */
	ret = wlan_pcie_prog_fw_w_helper(pmadapter, pmfw);
	if (ret != MLAN_STATUS_SUCCESS) {
		PRINTM(MERROR, "prog_fw failed ret=%#x\n", ret);
		LEAVE();
		return ret;
	}
poll_fw:
	/* Check if the firmware is downloaded successfully or not */
	ret = wlan_pcie_check_fw_status(pmadapter, poll_num);
	if (ret != MLAN_STATUS_SUCCESS) {
		PRINTM(MFATAL, "FW failed to be active in time!\n");
		ret = MLAN_STATUS_FAILURE;
		LEAVE();
		return ret;
	}
done:

	/* re-enable host interrupt for mlan after fw dnld is successful */
	wlan_enable_pcie_host_int(pmadapter);

	LEAVE();
	return ret;
}

/**
 *  @brief This function downloads data from driver to card.
 *
 *  Both commands and data packets are transferred to the card
 *  by this function. This function adds the PCIE specific header
 *  to the front of the buffer before transferring. The header
 *  contains the length of the packet and the type. The firmware
 *  handles the packets based upon this set type.
 *
 *  @param pmpriv    A pointer to mlan_private structure
 *  @param type	     data or command
 *  @param pmbuf     A pointer to mlan_buffer (pmbuf->data_len should include
 * PCIE header)
 *  @param tx_param  A pointer to mlan_tx_param (can be MNULL if type is
 * command)
 *
 *  @return 	     MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status wlan_pcie_host_to_card(pmlan_private pmpriv, t_u8 type,
				   mlan_buffer *pmbuf, mlan_tx_param *tx_param)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	pmlan_adapter pmadapter = pmpriv->adapter;

	ENTER();

	if (!pmbuf) {
		PRINTM(MERROR, "Passed NULL pmbuf to %s\n", __FUNCTION__);
		return MLAN_STATUS_FAILURE;
	}

	if (type == MLAN_TYPE_DATA) {
		ret = wlan_pcie_send_data(pmadapter, type, pmbuf, tx_param);
	} else if (type == MLAN_TYPE_CMD)
		ret = wlan_pcie_send_cmd(pmadapter, pmbuf);
#if defined(PCIE8997) || defined(PCIE9098) || defined(PCIE9097) ||             \
	defined(PCIEAW693) || defined(PCIEIW624)
	else if (type == MLAN_TYPE_VDLL)
		ret = wlan_pcie_send_vdll(pmadapter, pmbuf);
#endif
	LEAVE();
	return ret;
}

/**
 *  @brief This function allocates the PCIE buffer for SSU
 *
 *  @param pmadapter  A pointer to mlan_adapter structure
 *
 *  @return           MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status wlan_alloc_ssu_pcie_buf(pmlan_adapter pmadapter)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	pmlan_callbacks pcb = &pmadapter->callbacks;
	mlan_buffer *pmbuf = MNULL;

	ENTER();

	if (pmadapter->ssu_buf) {
		PRINTM(MCMND, "ssu buffer already allocated\n");
		LEAVE();
		return ret;
	}
	/* Allocate buffer here so that firmware can DMA data on it */
	pmbuf = wlan_alloc_mlan_buffer(pmadapter, MLAN_SSU_BUF_SIZE,
				       MLAN_SSU_HEADER_SIZE,
				       MOAL_ALLOC_MLAN_BUFFER);
	if (!pmbuf) {
		PRINTM(MERROR,
		       "SSU buffer create : Unable to allocate mlan_buffer\n");
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}
	if (MLAN_STATUS_FAILURE ==
	    pcb->moal_map_memory(
		    pmadapter->pmoal_handle, pmbuf->pbuf + pmbuf->data_offset,
		    &pmbuf->buf_pa, MLAN_SSU_BUF_SIZE, PCI_DMA_FROMDEVICE)) {
		PRINTM(MERROR, "%s: moal_map_memory failed\n", __FUNCTION__);
		/* free pmbuf */
		wlan_free_mlan_buffer(pmadapter, pmbuf);
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}

	PRINTM(MCMND,
	       "SSU buffer: add new mlan_buffer base: %p, "
	       "buf_base: %p, data_offset: %x, buf_pbase: %#x:%x, "
	       "buf_len: %#x\n",
	       pmbuf, pmbuf->pbuf, pmbuf->data_offset,
	       (t_u32)((t_u64)pmbuf->buf_pa >> 32), (t_u32)pmbuf->buf_pa,
	       pmbuf->data_len);

	pmadapter->ssu_buf = pmbuf;

	LEAVE();
	return ret;
}

/**
 *  @brief This function frees the allocated ssu buffer.
 *
 *  @param pmadapter  A pointer to mlan_adapter structure
 *
 *  @return           MLAN_STATUS_SUCCESS
 */
mlan_status wlan_free_ssu_pcie_buf(pmlan_adapter pmadapter)
{
	pmlan_callbacks pcb = MNULL;
	mlan_buffer *pmbuf = MNULL;

	ENTER();
	if (!pmadapter) {
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}

	pcb = &pmadapter->callbacks;
	if (pmadapter->ssu_buf) {
		pmbuf = pmadapter->ssu_buf;
		wlan_free_mlan_buffer(pmadapter, pmbuf);
	}
	pmadapter->ssu_buf = MNULL;

	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function allocates the PCIE ring buffers
 *
 *  The following initializations steps are followed -
 *      - Allocate TXBD ring buffers
 *      - Allocate RXBD ring buffers
 *      - Allocate event BD ring buffers
 *      - Allocate command and command response buffer
 *      - Allocate sleep cookie buffer
 *
 *  @param pmadapter  A pointer to mlan_adapter structure
 *
 *  @return           MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status wlan_alloc_pcie_ring_buf(pmlan_adapter pmadapter)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;

	ENTER();
#if defined(PCIE9098) || defined(PCIE9097) || defined(PCIEAW693) ||            \
	defined(PCIEIW624)
	if ((pmadapter->card_type == CARD_TYPE_PCIE9098) ||
	    (pmadapter->card_type == CARD_TYPE_PCIEIW624) ||
	    (pmadapter->card_type == CARD_TYPE_PCIEAW693) ||
	    (pmadapter->card_type == CARD_TYPE_PCIE9097)) {
		wlan_pcie_init_adma_ring_size(pmadapter);
	}
#endif
	pmadapter->pcard_pcie->cmdrsp_buf = MNULL;
	ret = wlan_pcie_create_txbd_ring(pmadapter);
	if (ret)
		goto err_cre_txbd;
	ret = wlan_pcie_create_rxbd_ring(pmadapter);
	if (ret)
		goto err_cre_rxbd;
	ret = wlan_pcie_create_evtbd_ring(pmadapter);
	if (ret)
		goto err_cre_evtbd;
	ret = wlan_pcie_alloc_cmdrsp_buf(pmadapter);
	if (ret)
		goto err_alloc_cmdbuf;
	return ret;
err_alloc_cmdbuf:
	wlan_pcie_delete_evtbd_ring(pmadapter);
err_cre_evtbd:
	wlan_pcie_delete_rxbd_ring(pmadapter);
err_cre_rxbd:
	wlan_pcie_delete_txbd_ring(pmadapter);
err_cre_txbd:

	LEAVE();
	return ret;
}

/**
 *  @brief This function frees the allocated ring buffers.
 *
 *  The following are freed by this function -
 *      - TXBD ring buffers
 *      - RXBD ring buffers
 *      - Event BD ring buffers
 *      - Command and command response buffer
 *      - Sleep cookie buffer
 *
 *  @param pmadapter  A pointer to mlan_adapter structure
 *
 *  @return           MLAN_STATUS_SUCCESS
 */
mlan_status wlan_free_pcie_ring_buf(pmlan_adapter pmadapter)
{
	ENTER();

	wlan_pcie_delete_cmdrsp_buf(pmadapter);
	wlan_pcie_delete_evtbd_ring(pmadapter);
	wlan_pcie_delete_rxbd_ring(pmadapter);
	wlan_pcie_delete_txbd_ring(pmadapter);
	pmadapter->pcard_pcie->cmdrsp_buf = MNULL;
#ifdef RPTR_MEM_COP
	if ((pmadapter->card_type == CARD_TYPE_PCIE9098) ||
	    (pmadapter->card_type == CARD_TYPE_PCIEAW693) ||
	    (pmadapter->card_type ==
	     CARD_TYPE_PCIEIW624)(pmadapter->card_type == CARD_TYPE_PCIE9097))
		wlan_pcie_free_rdptrs(pmadapter);
#endif

	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function cleans up packets in the ring buffers.
 *
 *  The following are cleaned by this function -
 *      - TXBD ring buffers
 *
 *  @param pmadapter  A pointer to mlan_adapter structure
 *
 *  @return           MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status wlan_clean_pcie_ring_buf(pmlan_adapter pmadapter)
{
	ENTER();
#if defined(PCIE8997) || defined(PCIE8897)
	if (!pmadapter->pcard_pcie->reg->use_adma)
		wlan_pcie_flush_txbd_ring(pmadapter);
#endif
	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function prepares command to set PCI-Express
 *  host buffer configuration
 *
 *  @param pmpriv       A pointer to mlan_private structure
 *
 *  @return             MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status wlan_set_pcie_buf_config(mlan_private *pmpriv)
{
	pmlan_adapter pmadapter = MNULL;
#if defined(PCIE8997) || defined(PCIE8897)
	HostCmd_DS_PCIE_HOST_BUF_DETAILS host_spec;
#endif
#if defined(PCIE)
#if defined(PCIE9098) || defined(PCIE9097) || defined(PCIEAW693) ||            \
	defined(PCIEIW624)
	HostCmd_DS_PCIE_ADMA_INIT pcie_adma_cfg;
#endif
#endif
	mlan_status ret = MLAN_STATUS_SUCCESS;

	ENTER();
	if (!pmpriv) {
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}
	pmadapter = pmpriv->adapter;
#if defined(PCIE8997) || defined(PCIE8897)
	if (!pmadapter->pcard_pcie->reg->use_adma) {
		memset(pmadapter, &host_spec, 0,
		       sizeof(HostCmd_DS_PCIE_HOST_BUF_DETAILS));

		/* Send the ring base addresses and count to firmware */
		host_spec.txbd_addr_lo = wlan_cpu_to_le32(
			(t_u32)(pmadapter->pcard_pcie->txbd_ring_pbase));
		host_spec.txbd_addr_hi = wlan_cpu_to_le32((
			t_u32)(((t_u64)pmadapter->pcard_pcie->txbd_ring_pbase) >>
			       32));
		host_spec.txbd_count =
			wlan_cpu_to_le32(pmadapter->pcard_pcie->txrx_bd_size);
		host_spec.rxbd_addr_lo = wlan_cpu_to_le32(
			(t_u32)(pmadapter->pcard_pcie->rxbd_ring_pbase));
		host_spec.rxbd_addr_hi = wlan_cpu_to_le32((
			t_u32)(((t_u64)pmadapter->pcard_pcie->rxbd_ring_pbase) >>
			       32));
		host_spec.rxbd_count =
			wlan_cpu_to_le32(pmadapter->pcard_pcie->txrx_bd_size);
		host_spec.evtbd_addr_lo = wlan_cpu_to_le32(
			(t_u32)(pmadapter->pcard_pcie->evtbd_ring_pbase));
		host_spec.evtbd_addr_hi =
			wlan_cpu_to_le32((t_u32)(((t_u64)pmadapter->pcard_pcie
							  ->evtbd_ring_pbase) >>
						 32));
		host_spec.evtbd_count = wlan_cpu_to_le32(MLAN_MAX_EVT_BD);

		ret = wlan_prepare_cmd(pmpriv,
				       HostCmd_CMD_PCIE_HOST_BUF_DETAILS,
				       HostCmd_ACT_GEN_SET, 0, MNULL,
				       &host_spec);
		if (ret) {
			PRINTM(MERROR,
			       "PCIE_HOST_BUF_CFG: send command failed\n");
			ret = MLAN_STATUS_FAILURE;
		}
	}
#endif
#if defined(PCIE9098) || defined(PCIE9097) || defined(PCIEAW693) ||            \
	defined(PCIEIW624)
	if (pmadapter->pcard_pcie->reg->use_adma) {
		if (IS_PCIEIW624(pmadapter->card_type) ||
		    (IS_PCIEAW693(pmadapter->card_type) &&
		     (pmadapter->card_rev > CHIP_AW693_REV_A0))) {
			pcie_adma_cfg.tx_ring_size =
				pmadapter->pcard_pcie->txrx_bd_size;
			pcie_adma_cfg.rx_ring_size =
				pmadapter->pcard_pcie->txrx_bd_size;
			pcie_adma_cfg.evt_ring_size = MLAN_MAX_EVT_BD;
			pcie_adma_cfg.int_mode =
				pmadapter->pcard_pcie->pcie_int_mode;
			ret = wlan_prepare_cmd(pmpriv,
					       HostCmd_CMD_PCIE_ADMA_INIT,
					       HostCmd_ACT_GEN_SET, 0, MNULL,
					       &pcie_adma_cfg);
			if (ret) {
				PRINTM(MERROR,
				       "PCIE_ADMA_INIT: send command failed\n");
				ret = MLAN_STATUS_FAILURE;
			}
		}
		/** config ADMA for Tx Data */
		wlan_init_adma(pmadapter, ADMA_TX_DATA,
			       pmadapter->pcard_pcie->txbd_ring_pbase,
			       pmadapter->pcard_pcie->txrx_num_desc, MTRUE);
		/** config ADMA for Rx Data */
		wlan_init_adma(pmadapter, ADMA_RX_DATA,
			       pmadapter->pcard_pcie->rxbd_ring_pbase,
			       pmadapter->pcard_pcie->txrx_num_desc, MTRUE);
		/** config ADMA for Rx Event */
		wlan_init_adma(pmadapter, ADMA_EVENT,
			       pmadapter->pcard_pcie->evtbd_ring_pbase,
			       EVT_NUM_DESC, MTRUE);
		/** config ADMA for cmd */
		wlan_init_adma(pmadapter, ADMA_CMD, 0, 0, MTRUE);
		/** config ADMA for cmdresp */
		wlan_init_adma(pmadapter, ADMA_CMDRESP,
			       pmadapter->pcard_pcie->cmdrsp_buf->buf_pa, 0,
			       MTRUE);
	}
#endif
	wlan_pcie_init_fw(pmadapter);
	LEAVE();
	return ret;
}

#if defined(PCIE8997) || defined(PCIE8897)
/**
 *  @brief This function prepares command PCIE host buffer config.
 *
 *  @param pmpriv       A pointer to mlan_private structure
 *  @param cmd          A pointer to HostCmd_DS_COMMAND structure
 *  @param cmd_action   The action: GET or SET
 *  @param pdata_buf    A pointer to data buffer
 *
 *  @return             MLAN_STATUS_SUCCESS
 */
mlan_status wlan_cmd_pcie_host_buf_cfg(pmlan_private pmpriv,
				       HostCmd_DS_COMMAND *cmd,
				       t_u16 cmd_action, t_pvoid pdata_buf)
{
	HostCmd_DS_PCIE_HOST_BUF_DETAILS *ppcie_hoost_spec =
		&cmd->params.pcie_host_spec;

	ENTER();

	cmd->command = wlan_cpu_to_le16(HostCmd_CMD_PCIE_HOST_BUF_DETAILS);
	cmd->size = wlan_cpu_to_le16(
		(sizeof(HostCmd_DS_PCIE_HOST_BUF_DETAILS)) + S_DS_GEN);

	if (cmd_action == HostCmd_ACT_GEN_SET) {
		memcpy_ext(pmpriv->adapter, ppcie_hoost_spec, pdata_buf,
			   sizeof(HostCmd_DS_PCIE_HOST_BUF_DETAILS),
			   sizeof(HostCmd_DS_PCIE_HOST_BUF_DETAILS));
	}

	LEAVE();
	return MLAN_STATUS_SUCCESS;
}
#endif

#if defined(PCIE)
/**
 *  @brief This function prepares command PCIE ADMA init.
 *
 *  @param pmpriv       A pointer to mlan_private structure
 *  @param cmd          A pointer to HostCmd_DS_COMMAND structure
 *  @param cmd_action   The action: GET or SET
 *  @param pdata_buf    A pointer to data buffer
 *
 *  @return             MLAN_STATUS_SUCCESS
 */
mlan_status wlan_cmd_pcie_adma_init(pmlan_private pmpriv,
				    HostCmd_DS_COMMAND *cmd, t_u16 cmd_action,
				    t_pvoid pdata_buf)
{
	HostCmd_DS_PCIE_ADMA_INIT *ppcie_adma_cfg =
		&cmd->params.pcie_adma_config;

	ENTER();

	cmd->command = wlan_cpu_to_le16(HostCmd_CMD_PCIE_ADMA_INIT);
	cmd->size = wlan_cpu_to_le16((sizeof(HostCmd_DS_PCIE_ADMA_INIT)) +
				     S_DS_GEN);

	if (cmd_action == HostCmd_ACT_GEN_SET) {
		memcpy_ext(pmpriv->adapter, ppcie_adma_cfg, pdata_buf,
			   sizeof(HostCmd_DS_PCIE_ADMA_INIT),
			   sizeof(HostCmd_DS_PCIE_ADMA_INIT));
	}
	LEAVE();
	return MLAN_STATUS_SUCCESS;
}
#endif

/**
 *  @brief This function wakes up the card.
 *
 *  @param pmadapter		A pointer to mlan_adapter structure
 *  @param timeout          set timeout flag
 *
 *  @return			MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status wlan_pm_pcie_wakeup_card(pmlan_adapter pmadapter,
					    t_u8 timeout)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	t_u32 age_ts_usec;

	ENTER();
	PRINTM(MEVENT, "func%d: Wakeup device...\n",
	       pmadapter->pcard_pcie->func_num);
	pmadapter->callbacks.moal_get_system_time(pmadapter->pmoal_handle,
						  &pmadapter->pm_wakeup_in_secs,
						  &age_ts_usec);

	if (timeout) {
		pmadapter->callbacks.moal_start_timer(
			pmadapter->pmoal_handle, pmadapter->pwakeup_fw_timer,
			MFALSE, MRVDRV_TIMER_3S);
		pmadapter->wakeup_fw_timer_is_set = MTRUE;
	}

	ret = wlan_pcie_wakeup(pmadapter);

	LEAVE();
	return ret;
}

static mlan_status wlan_pcie_debug_dump(pmlan_adapter pmadapter)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	pmlan_buffer pmbuf = pmadapter->pcard_pcie->cmdrsp_buf;
	ENTER();

	if (pmbuf == MNULL) {
		PRINTM(MMSG, "Rx CMD response pmbuf is null\n");
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}
	PRINTM(MERROR, "Dump Rx CMD Response Buf:\n");
	DBG_HEXDUMP(MERROR, "CmdResp Buf", pmbuf->pbuf + pmbuf->data_offset,
		    64);

	LEAVE();
	return ret;
}

/**
 *  @brief This function handle data complete
 *
 *  @param pmadapter A pointer to mlan_adapter structure
 *  @param pmbuf     A pointer to the mlan_buffer
 *  @return          N/A
 */
static mlan_status wlan_pcie_data_complete(pmlan_adapter pmadapter,
					   mlan_buffer *pmbuf,
					   mlan_status status)
{
	ENTER();

	wlan_free_mlan_buffer(pmadapter, pmbuf);

	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function checks the interrupt status and
 *  handle it accordingly.
 *
 *  @param pmadapter    A pointer to mlan_adapter structure
 *
 *  @return             MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status wlan_pcie_interrupt_ext(t_u16 msg_id,
					   pmlan_adapter pmadapter)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	ENTER();
	ret = wlan_pcie_interrupt(msg_id, pmadapter);
	if (ret == MLAN_STATUS_SUCCESS) {
		/* if we get interrupt, it means device wakes up, no need to
		 * wake up */
		pmadapter->pm_wakeup_timeout = 0;
		wlan_process_pcie_int_status(pmadapter);
	}
	LEAVE();
	return ret;
}

/**
 *  @brief This function process pcie receive data
 *
 *
 *  @param pmadapter    A pointer to mlan_adapter structure
 *
 *  @return             N/A
 */
static void wlan_pcie_process_rx(mlan_adapter *pmadapter)
{
	pmlan_callbacks pcb = &pmadapter->callbacks;
	ENTER();
	pcb->moal_spin_lock(pmadapter->pmoal_handle, pmadapter->pmlan_rx_lock);
	if (pmadapter->pcie_rx_processing) {
		pcb->moal_spin_unlock(pmadapter->pmoal_handle,
				      pmadapter->pmlan_rx_lock);
		goto exit_rx_proc;
	} else {
		pmadapter->pcie_rx_processing = MTRUE;
		pcb->moal_spin_unlock(pmadapter->pmoal_handle,
				      pmadapter->pmlan_rx_lock);
	}

	wlan_pcie_process_recv_data(pmadapter);

	pcb->moal_spin_lock(pmadapter->pmoal_handle, pmadapter->pmlan_rx_lock);
	pmadapter->pcie_rx_processing = MFALSE;
	pmadapter->pcard_pcie->rx_pending = MFALSE;
	pcb->moal_spin_unlock(pmadapter->pmoal_handle,
			      pmadapter->pmlan_rx_lock);
exit_rx_proc:
	LEAVE();
	return;
}

/**
 *  @brief This function process pcie_cmd_dnld interrupt
 *
 *
 *  @param pmadapter    A pointer to mlan_adapter structure
 *
 *  @return             N/A
 */
static void wlan_pcie_process_cmd_dnld(mlan_adapter *pmadapter)
{
	ENTER();
	if (pmadapter->cmd_sent)
		pmadapter->cmd_sent = MFALSE;
#if defined(PCIE8997) || defined(PCIE9098) || defined(PCIE9097) ||             \
	defined(PCIEAW693) || defined(PCIEIW624)
	if (pmadapter->pcard_pcie->vdll_cmd_buf)
		wlan_pcie_send_vdll_complete(pmadapter);
#endif

	LEAVE();
	return;
}

/**
 *  @brief This function checks the interrupt status and
 *  handle it accordingly.
 *
 *  @param pmadapter    A pointer to mlan_adapter structure
 *
 *  @return             MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status wlan_process_pcie_int_status_ext(mlan_adapter *pmadapter,
						    t_u8 type)
{
	ENTER();
	switch (type) {
	case RX_DATA: // Rx Data
		wlan_pcie_process_rx(pmadapter);
		break;
	case RX_EVENT: // Rx event
		wlan_pcie_process_event(pmadapter);
		break;
	case TX_COMPLETE: // Tx data complete
		wlan_pcie_process_tx_complete(pmadapter);
		break;
	case RX_CMD_RESP: // Rx CMD Resp
		wlan_pcie_process_cmd_resp(pmadapter);
		break;
	case RX_CMD_DNLD:
		wlan_pcie_process_cmd_dnld(pmadapter);
		break;
	default:
		break;
	}
	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function enables rx host interrupts.
 *
 *  @param pmadapter A pointer to mlan_adapter structure
 *  @param enable   0-disable 1-enable
 *
 *  @return        MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status wlan_pcie_select_host_int(mlan_adapter *pmadapter)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
#if defined(PCIE9098) || defined(PCIE9097) || defined(PCIEAW693) ||            \
	defined(PCIEIW624)
	t_u32 int_sel_mask =
		PCIE9098_HOST_INTR_SEL_MASK & ~PCIE9098_HOST_INTR_UPLD_RDY;
#endif

#if defined(PCIE9098) || defined(PCIE9097) || defined(PCIEAW693) ||            \
	defined(PCIEIW624)
	if (pmadapter->pcard_pcie->reg->use_adma) {
		ret = wlan_pcie_set_host_int_select_reg(pmadapter,
							int_sel_mask);
		PRINTM(MMSG, "set int_sel_mask=0x%x\n", int_sel_mask);
	}
#endif
	return ret;
}

#if defined(PCIEAW693)
/**
 *  @brief This function send buffer list to FW
 *
 *  @param pmadapter A pointer to mlan_adapter structure
 *  @param type      packet type
 *  @param num_pkt   num of pkt
 *  @param pkt_size  packet size
 *
 *  @return          MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status wlan_pcie_send_data_list(mlan_adapter *pmadapter, t_u8 type,
					    t_u8 num_pkt, t_u16 pkt_size)
{
	pmlan_buffer pmbuf;

	t_u16 *tmp;
	t_u8 *payload;
	t_u8 i;
	ENTER();
#ifdef PCIEAW693
	if (!IS_PCIEAW693(pmadapter->card_type)) {
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}
#endif
	if ((((pmadapter->pcard_pcie->txbd_wrptr >> 1) %
	      TX_DONE_POLL_DISTANCE) == 0) &&
	    wlan_is_tx_pending(pmadapter))
		wlan_pcie_process_tx_complete(pmadapter);

	if (num_pkt == 1) {
		pmbuf = (pmlan_buffer)util_dequeue_list(pmadapter->pmoal_handle,
							&pmadapter->amsdu_txq,
							MNULL, MNULL);
		if (pmbuf) {
			wlan_pcie_send_data(pmadapter, type, pmbuf, MNULL);
		}
		return MLAN_STATUS_SUCCESS;
	}
	for (i = 0; i < num_pkt; i++) {
		pmbuf = (pmlan_buffer)util_dequeue_list(pmadapter->pmoal_handle,
							&pmadapter->amsdu_txq,
							MNULL, MNULL);
		if (!pmbuf)
			break;
		/* first pkt */
		if (i == 0) {
			payload = pmbuf->pbuf + pmbuf->data_offset;
			tmp = (t_u16 *)&payload[0];
			*tmp = wlan_cpu_to_le16((t_u16)pkt_size);
			tmp = (t_u16 *)&payload[2];
			*tmp = wlan_cpu_to_le16(type);
			wlan_pcie_send_adma_data(pmadapter, pmbuf,
						 ADMA_BD_FLAG_SOP, pkt_size);
		} else if (i == (num_pkt - 1)) { /* last pkt */
			wlan_pcie_send_adma_data(pmadapter, pmbuf,
						 ADMA_BD_FLAG_EOP, 0);
		} else {
			wlan_pcie_send_adma_data(pmadapter, pmbuf, 0, 0);
		}
	}
	LEAVE();
	return MLAN_STATUS_SUCCESS;
}
#endif

mlan_adapter_operations mlan_pcie_ops = {
	.dnld_fw = wlan_pcie_dnld_fw,
	.interrupt = wlan_pcie_interrupt_ext,
	.process_int_status = wlan_process_pcie_int_status_ext,
	.host_to_card = wlan_pcie_host_to_card,
	.wakeup_card = wlan_pm_pcie_wakeup_card,
	.reset_card = wlan_pcie_wakeup,
	.event_complete = wlan_pcie_event_complete,
	.data_complete = wlan_pcie_data_complete,
	.cmdrsp_complete = wlan_pcie_cmdrsp_complete,
	.handle_rx_packet = wlan_handle_rx_packet,
	.select_host_int = wlan_pcie_select_host_int,
	.debug_dump = wlan_pcie_debug_dump,
	.intf_header_len = PCIE_INTF_HEADER_LEN,
#ifdef PCIEAW693
	.send_data_list = wlan_pcie_send_data_list,
	.get_max_msdu_cnt = wlan_pcie_get_max_msdu_cnt,
#endif
};
