/** @file moal_pcie.c
 *
 *  @brief This file contains PCIE IF (interface) module
 *  related functions.
 *
 *
 * Copyright 2008-2022, 2024-2025 NXP
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
    02/01/2012: initial version
********************************************************/

#include <linux/firmware.h>

#if defined(STA_CFG80211) || defined(UAP_CFG80211)
#include "moal_cfg80211.h"
#endif

#include "moal_pcie.h"
#ifdef UAP_SUPPORT
#include "moal_uap.h"
#endif
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 70)
#ifdef IMX_SUPPORT
#include <linux/busfreq-imx.h>
#endif
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 1, 0)
#if IS_ENABLED(CONFIG_IPV6)
#include <net/addrconf.h>
#endif
#endif

/********************************************************
			Local Variables
********************************************************/
#define DRV_NAME "NXP mdriver PCIe"

/* PCIE resume handler */
static int woal_pcie_resume(struct pci_dev *pdev);
static void woal_pcie_reg_dbg(moal_handle *phandle);
static void woal_pcie_unregister_dev(moal_handle *handle);
static void woal_pcie_cleanup(pcie_service_card *card);
static mlan_status woal_pcie_init(pcie_service_card *card);
static void woal_pcie_work(struct work_struct *work);

/** WLAN IDs */
static const struct pci_device_id wlan_ids[] = {
#ifdef PCIE8897
	{
		PCIE_VENDOR_ID_MRVL,
		PCIE_DEVICE_ID_88W8897P,
		PCI_ANY_ID,
		PCI_ANY_ID,
		0,
		0,
	},
#endif
#ifdef PCIE8997
	{
		PCIE_VENDOR_ID_MRVL,
		PCIE_DEVICE_ID_88W8997P,
		PCI_ANY_ID,
		PCI_ANY_ID,
		0,
		0,
	},
	{
		PCIE_VENDOR_ID_V2_MRVL,
		PCIE_DEVICE_ID_88W8997P,
		PCI_ANY_ID,
		PCI_ANY_ID,
		0,
		0,
	},
#endif
#ifdef PCIE9097
	{
		PCIE_VENDOR_ID_V2_MRVL,
		PCIE_DEVICE_ID_88W9097,
		PCI_ANY_ID,
		PCI_ANY_ID,
		0,
		0,
	},
#endif
#ifdef PCIE9098
	{
		PCIE_VENDOR_ID_V2_MRVL,
		PCIE_DEVICE_ID_88W9098P_FN0,
		PCI_ANY_ID,
		PCI_ANY_ID,
		0,
		0,
	},
	{
		PCIE_VENDOR_ID_V2_MRVL,
		PCIE_DEVICE_ID_88W9098P_FN1,
		PCI_ANY_ID,
		PCI_ANY_ID,
		0,
		0,
	},
#endif
#ifdef PCIEAW693
	{
		PCIE_VENDOR_ID_NXP,
		PCIE_DEVICE_ID_88WAW693_FN0,
		PCI_ANY_ID,
		PCI_ANY_ID,
		0,
		0,
	},
	{
		PCIE_VENDOR_ID_NXP,
		PCIE_DEVICE_ID_88WAW693_FN1,
		PCI_ANY_ID,
		PCI_ANY_ID,
		0,
		0,
	},
#endif
#ifdef PCIEIW624
	{
		PCIE_VENDOR_ID_NXP,
		PCIE_DEVICE_ID_88WIW624,
		PCI_ANY_ID,
		PCI_ANY_ID,
		0,
		0,
	},
#endif

	{},
};
/* moal interface ops */
static moal_if_ops pcie_ops;

MODULE_DEVICE_TABLE(pci, wlan_ids);


/********************************************************
			Global Variables
********************************************************/

/********************************************************
			Local Functions
********************************************************/

static mlan_status woal_pcie_preinit(struct pci_dev *pdev);
#if defined(PCIE8897) || defined(PCIE8997) || defined(PCIE9098) ||             \
	defined(PCIE9097) || defined(PCIEAW693) || defined(PCIEIW624)
static rdwr_status woal_pcie_rdwr_firmware(moal_handle *phandle, t_u8 doneflag,
					   t_u8 resetflag);
#endif

/**  @brief This function updates the card types
 *
 *  @param handle   A Pointer to the moal_handle structure
 *  @param card     A Pointer to card
 *
 *  @return         N/A
 */
static t_u16 woal_update_card_type(t_void *card)
{
	pcie_service_card *cardp_pcie = (pcie_service_card *)card;
	t_u16 card_type = 0;

	/* Update card type */
#ifdef PCIE8897
	if (cardp_pcie->dev->device == PCIE_DEVICE_ID_88W8897P) {
		card_type = CARD_TYPE_PCIE8897;
		moal_memcpy_ext(NULL, driver_version, CARD_PCIE8897,
				strlen(CARD_PCIE8897), strlen(driver_version));
		/* we are copying card name in middle of full version, we can
		   not copy null termination. This was already tried and
		   reverted as full version got terminated in middlei(See commit
		   57c27201f9a23562337491f3cbb9833ca348076c). thus suppressing
		   the coverity warning  for all card types in this function. */
		// coverity[string_null:SUPPRESS]
		// coverity[cert_str32_c_violation:SUPPRESS]
		moal_memcpy_ext(NULL,
				driver_version + strlen(INTF_CARDTYPE) +
					strlen(KERN_VERSION),
				V15, strlen(V15),
				strlen(driver_version) - strlen(INTF_CARDTYPE) -
					strlen(KERN_VERSION));
	}
#endif
#ifdef PCIE8997
	if (cardp_pcie->dev->device == PCIE_DEVICE_ID_88W8997P) {
		card_type = CARD_TYPE_PCIE8997;
		moal_memcpy_ext(NULL, driver_version, CARD_PCIE8997,
				strlen(CARD_PCIE8997), strlen(driver_version));
		// coverity[string_null:SUPPRESS]
		// coverity[cert_str32_c_violation:SUPPRESS]
		moal_memcpy_ext(NULL,
				driver_version + strlen(INTF_CARDTYPE) +
					strlen(KERN_VERSION),
				V16, strlen(V16),
				strlen(driver_version) - strlen(INTF_CARDTYPE) -
					strlen(KERN_VERSION));
	}
#endif
#ifdef PCIE9097
	if (cardp_pcie->dev->device == PCIE_DEVICE_ID_88W9097) {
		card_type = CARD_TYPE_PCIE9097;
		moal_memcpy_ext(NULL, driver_version, CARD_PCIE9097,
				strlen(CARD_PCIE9097), strlen(driver_version));
		// coverity[string_null:SUPPRESS]
		// coverity[cert_str32_c_violation:SUPPRESS]
		moal_memcpy_ext(NULL,
				driver_version + strlen(INTF_CARDTYPE) +
					strlen(KERN_VERSION),
				V17, strlen(V17),
				strlen(driver_version) - strlen(INTF_CARDTYPE) -
					strlen(KERN_VERSION));
	}
#endif
#ifdef PCIE9098
	if (cardp_pcie->dev->device == PCIE_DEVICE_ID_88W9098P_FN0 ||
	    cardp_pcie->dev->device == PCIE_DEVICE_ID_88W9098P_FN1) {
		card_type = CARD_TYPE_PCIE9098;
		moal_memcpy_ext(NULL, driver_version, CARD_PCIE9098,
				strlen(CARD_PCIE9098), strlen(driver_version));
		// coverity[string_null:SUPPRESS]
		// coverity[cert_str32_c_violation:SUPPRESS]
		moal_memcpy_ext(NULL,
				driver_version + strlen(INTF_CARDTYPE) +
					strlen(KERN_VERSION),
				V17, strlen(V17),
				strlen(driver_version) - strlen(INTF_CARDTYPE) -
					strlen(KERN_VERSION));
	}
#endif
#ifdef PCIEAW693
	if (cardp_pcie->dev->device == PCIE_DEVICE_ID_88WAW693_FN0 ||
	    cardp_pcie->dev->device == PCIE_DEVICE_ID_88WAW693_FN1) {
		card_type = CARD_TYPE_PCIEAW693;
		moal_memcpy_ext(NULL, driver_version, CARD_PCIEAW693,
				strlen(CARD_PCIEAW693), strlen(driver_version));
		// coverity[string_null:SUPPRESS]
		// coverity[cert_str32_c_violation:SUPPRESS]
		moal_memcpy_ext(NULL,
				driver_version + strlen(INTF_CARDTYPE) +
					strlen(KERN_VERSION),
				V18, strlen(V18),
				strlen(driver_version) - strlen(INTF_CARDTYPE) -
					strlen(KERN_VERSION));
	}
#endif
#ifdef PCIEIW624
	if (cardp_pcie->dev->device == PCIE_DEVICE_ID_88WIW624) {
		card_type = CARD_TYPE_PCIEIW624;
		moal_memcpy_ext(NULL, driver_version, CARD_PCIEIW624,
				strlen(CARD_PCIEIW624), strlen(driver_version));
		// coverity[string_null:SUPPRESS]
		// coverity[cert_str32_c_violation:SUPPRESS]
		moal_memcpy_ext(NULL,
				driver_version + strlen(INTF_CARDTYPE) +
					strlen(KERN_VERSION),
				V18, strlen(V18),
				strlen(driver_version) - strlen(INTF_CARDTYPE) -
					strlen(KERN_VERSION));
	}
#endif

	return card_type;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 18, 0)
/**
 * @brief Function to program scratch register to ask device to clear ADMA
 *
 * @param handle    A pointer to moal_handle structure
 *
 * @return        MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */

static mlan_status woal_reset_adma(moal_handle *handle)
{
	int tries = 0;
	int ret = MLAN_STATUS_SUCCESS;
	t_u32 value;
	t_u32 reset_reg = handle->card_info->fw_reset_reg;
	t_u8 reset_adma_val = 0x97;
	/* wake up device before set the reset reg */
	handle->ops.read_reg(handle, handle->card_info->fw_wakeup_reg, &value);
	udelay(100);
	if (handle->ops.write_reg(handle, reset_reg, reset_adma_val) !=
	    MLAN_STATUS_SUCCESS) {
		PRINTM(MERROR, "Failed to write register.\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	for (tries = 0; tries < 100; ++tries) {
		ret = handle->ops.read_reg(handle, reset_reg, &value);
		if (value == 0) {
			break;
		}
		moal_usleep_range(handle, 100, 200);
	}

	if (value == 0) {
		PRINTM(MMSG, "%s:ADMA reset done\n", __func__);
		ret = MLAN_STATUS_SUCCESS;
	} else {
		PRINTM(MERROR, "%s:ADMA reset failed(value:%x)\n", __func__,
		       value);
		ret = MLAN_STATUS_FAILURE;
	}
done:
	return ret;
}

/**
 * @brief Function to process pre/post PCIe function level reset
 *
 * @param handle    A pointer to moal_handle structure
 * @param prepare   True :- its a pre FLR call from the kernel
 *		    False :- its a post FLR call from the kernel
 * @param flr       True: call from FLR
 *
 * Note: This function is mix of woal_switch_drv_mode() and
 * remove_card(). Idea is to cleanup the software only without
 * touching the PCIe specific code. Likewise, during init init
 * everything, including hw, but do not reinitiate PCIe stack
 *
 * @return        MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status woal_do_flr(moal_handle *handle, bool prepare, bool flr_flag)
{
	unsigned int i;
	int index = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;
	moal_private *priv = NULL;
	pcie_service_card *card = NULL;
	int fw_serial_bkp = 0;

	ENTER();

	if (!handle) {
		PRINTM(MINFO, "\n Handle null during prepare=%d\n", prepare);
		LEAVE();
		return status;
	}

	card = (pcie_service_card *)handle->card;

	if (card == NULL) {
		PRINTM(MERROR, "The parameter 'card' is NULL\n");
		LEAVE();
		return (mlan_status)MLAN_STATUS_FAILURE;
	}

	if (!IS_PCIE8997(handle->card_type) &&
	    !IS_PCIE9097(handle->card_type) &&
	    !IS_PCIEIW624(handle->card_type) &&
	    !IS_PCIEAW693(handle->card_type) &&
	    !IS_PCIE9098(handle->card_type)) {
		LEAVE();
		return status;
	}

	if (MOAL_ACQ_SEMAPHORE_BLOCK(&AddRemoveCardSem))
		goto exit_sem_err;

	if (!prepare)
		goto perform_init;
	if (!handle->pmlan_adapter)
		goto exit;

	/* Reset all interfaces */
	priv = woal_get_priv(handle, MLAN_BSS_ROLE_ANY);
	woal_reset_intf(priv, MOAL_IOCTL_WAIT, MTRUE);
	woal_clean_up(handle);
	mlan_ioctl(handle->pmlan_adapter, NULL);

	/* Shutdown firmware */
	handle->init_wait_q_woken = MFALSE;
	status = mlan_shutdown_fw(handle->pmlan_adapter);

	if (status == MLAN_STATUS_PENDING)
		wait_event_interruptible(handle->init_wait_q,
					 handle->init_wait_q_woken);

	if (atomic_read(&handle->rx_pending) ||
	    atomic_read(&handle->tx_pending) ||
	    atomic_read(&handle->ioctl_pending)) {
		PRINTM(MERROR,
		       "ERR: rx_pending=%d,tx_pending=%d,ioctl_pending=%d\n",
		       atomic_read(&handle->rx_pending),
		       atomic_read(&handle->tx_pending),
		       atomic_read(&handle->ioctl_pending));
	}

	unregister_inetaddr_notifier(&handle->woal_notifier);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 1, 0)
#if IS_ENABLED(CONFIG_IPV6)
	unregister_inet6addr_notifier(&handle->woal_inet6_notifier);
#endif
#endif

#ifdef WIFI_DIRECT_SUPPORT
#if defined(STA_CFG80211) && defined(UAP_CFG80211)
#if CFG80211_VERSION_CODE >= WIFI_DIRECT_KERNEL_VERSION
	/* Remove virtual interface */
	woal_remove_virtual_interface(handle);
#endif
#endif
#endif
	/* Remove interface */
	for (i = 0; i < handle->priv_num; i++)
		woal_remove_interface(handle, i);
	handle->priv_num = 0;
#if defined(STA_CFG80211) || defined(UAP_CFG80211)
	/* Unregister wiphy device and free */
	if (handle->wiphy) {
		wiphy_unregister(handle->wiphy);
		woal_cfg80211_free_bands(handle->wiphy);
		wiphy_free(handle->wiphy);
		handle->wiphy = NULL;
	}
#endif
	/* Unregister mlan */
	if (handle->pmlan_adapter) {
		mlan_unregister(handle->pmlan_adapter);
		if (atomic_read(&handle->lock_count) ||
		    atomic_read(&handle->malloc_count) ||
		    atomic_read(&handle->mbufalloc_count)) {
			PRINTM(MERROR,
			       "mlan has memory leak: lock_count=%d,"
			       " malloc_count=%d, mbufalloc_count=%d\n",
			       atomic_read(&handle->lock_count),
			       atomic_read(&handle->malloc_count),
			       atomic_read(&handle->mbufalloc_count));
		}
		if (atomic_read(&handle->malloc_cons_count)) {
			PRINTM(MERROR,
			       "mlan has memory leak: malloc_cons_count=%d\n",
			       atomic_read(&handle->malloc_cons_count));
		}
		handle->pmlan_adapter = NULL;
	}
	handle->fw_dump = MFALSE;

	goto exit;

perform_init:
	handle->priv_num = 0;

	/* Init SW */
	if (woal_init_sw(handle)) {
		PRINTM(MFATAL, "Software Init Failed\n");
		goto err_init_fw;
	}
	if (!handle->pmlan_adapter)
		goto err_init_fw;

#if defined(PCIE9098) || defined(PCIEAW693)
	if ((card->dev->device == PCIE_DEVICE_ID_88W9098P_FN1) ||
	    (card->dev->device == PCIE_DEVICE_ID_88WAW693_FN1))
		mlan_set_int_mode(handle->pmlan_adapter, pcie_int_mode, 1);
	else
#endif
		/* Update pcie_int_mode in mlan adapter */
		mlan_set_int_mode(handle->pmlan_adapter,
				  handle->params.pcie_int_mode, 0);

	/* Init FW and HW */
	/* Load wlan only binary */
	if (flr_flag) {
		fw_serial_bkp = moal_extflg_isset(handle, EXT_FW_SERIAL);
		moal_extflg_clear(handle, EXT_FW_SERIAL);
		woal_update_firmware_name(handle);
	}
	if (woal_init_fw(handle)) {
		PRINTM(MFATAL, "Firmware Init Failed\n");
		woal_pcie_reg_dbg(handle);
		if (fw_serial_bkp)
			moal_extflg_set(handle, EXT_FW_SERIAL);
		goto err_init_fw;
	}

	if (flr_flag && fw_serial_bkp)
		moal_extflg_set(handle, EXT_FW_SERIAL);
	if (IS_PCIE9098(handle->card_type))
		handle->event_fw_dump = MTRUE;
exit:
	MOAL_REL_SEMAPHORE(&AddRemoveCardSem);

exit_sem_err:
	LEAVE();
	return status;

err_init_fw:
	if (handle->is_fw_dump_timer_set) {
		woal_cancel_timer(&handle->fw_dump_timer);
		handle->is_fw_dump_timer_set = MFALSE;
	}
	if ((handle->hardware_status == HardwareStatusFwReady) ||
	    (handle->hardware_status == HardwareStatusReady)) {
		PRINTM(MINFO, "shutdown mlan\n");
		handle->init_wait_q_woken = MFALSE;
		status = mlan_shutdown_fw(handle->pmlan_adapter);
		if (status == MLAN_STATUS_PENDING)
			wait_event_interruptible(handle->init_wait_q,
						 handle->init_wait_q_woken);
	}
#ifdef ANDROID_KERNEL
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 1, 0)
	wakeup_source_trash(&handle->ws);
#else
	wake_lock_destroy(&handle->wake_lock);
#endif
#endif
#ifdef CONFIG_PROC_FS
	woal_proc_exit(handle);
#endif
	/* Unregister device */
	PRINTM(MINFO, "unregister device\n");
	woal_pcie_unregister_dev(handle);
	handle->surprise_removed = MTRUE;
#ifdef REASSOCIATION
	if (handle->reassoc_thread.pid)
		wake_up_interruptible(&handle->reassoc_thread.wait_q);
	/* waiting for main thread quit */
	while (handle->reassoc_thread.pid)
		woal_sched_timeout(2);
#endif /* REASSOCIATION */
	if (moal_extflg_isset(handle, EXT_NAPI))
		netif_napi_del(&handle->napi_rx);
	woal_terminate_workqueue(handle);
	woal_free_moal_handle(handle);

	for (index = 0; index < MAX_MLAN_ADAPTER; index++) {
		if (m_handle[index] == handle)
			break;
	}
	if (index < MAX_MLAN_ADAPTER)
		m_handle[index] = NULL;
	card->handle = NULL;
	MOAL_REL_SEMAPHORE(&AddRemoveCardSem);
	LEAVE();
	return (mlan_status)MLAN_STATUS_FAILURE;
}
#endif

/**
 *  @brief This function handles PCIE driver probe
 *
 *  @param pdev     A pointer to pci_dev structure
 *  @param id       A pointer to pci_device_id structure
 *
 *  @return         error code
 */
static int woal_pcie_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	pcie_service_card *card = NULL;
	moal_handle *handle;
	t_u16 card_type = 0;
	int ret = 0;

	ENTER();

	PRINTM(MINFO, "vendor=0x%4.04X device=0x%4.04X rev=%d\n", pdev->vendor,
	       pdev->device, pdev->revision);

	/* Preinit PCIE device so allocate PCIE memory can be successful */
	if (woal_pcie_preinit(pdev)) {
		PRINTM(MFATAL, "MOAL PCIE preinit failed\n");
		LEAVE();
		return -EFAULT;
	}

	card = kzalloc(sizeof(pcie_service_card), GFP_KERNEL);
	if (!card) {
		PRINTM(MERROR, "%s: failed to alloc memory\n", __func__);
		ret = -ENOMEM;
		goto err;
	}

	card->dev = pdev;

	card_type = woal_update_card_type(card);
	if (!card_type) {
		PRINTM(MERROR, "pcie probe: woal_update_card_type() failed\n");
		ret = MLAN_STATUS_FAILURE;
		goto err;
	}
	if (MLAN_STATUS_SUCCESS != woal_pcie_init(card)) {
		PRINTM(MERROR, "woal_pcie_init failed\n");
		ret = -EFAULT;
		goto err;
	}
	INIT_WORK(&card->reset_work, woal_pcie_work);

	handle = woal_add_card(card, &card->dev->dev, &pcie_ops, card_type);
	if (handle == NULL || IS_ERR(handle)) {
		woal_pcie_cleanup(card);
		PRINTM(MMSG, "%s: failed\n", __func__);
		ret = IS_ERR(handle) ? PTR_ERR(handle) : -EFAULT;
		goto err;
	}

#ifdef IMX_SUPPORT
	woal_regist_oob_wakeup_irq(card->handle);
#endif /* IMX_SUPPORT */

	LEAVE();
	return ret;
err:
	kfree(card);
	if (pci_is_enabled(pdev))
		pci_disable_device(pdev);

	LEAVE();
	return ret;
}

/**
 *  @brief This function handles PCIE driver remove
 *
 *  @param pdev     A pointer to pci_dev structure
 *
 *  @return         error code
 */
static void woal_pcie_remove(struct pci_dev *dev)
{
	pcie_service_card *card;
	moal_handle *handle;

	ENTER();
	card = pci_get_drvdata(dev);
	if (!card) {
		PRINTM(MINFO, "PCIE card removed from slot\n");
		LEAVE();
		return;
	}
	cancel_work_sync(&card->reset_work);
	handle = card->handle;
	if (!handle) {
		PRINTM(MINFO, "PCIE card handle removed\n");
		woal_pcie_cleanup(card);
		kfree(card);
		LEAVE();
		return;
	}
	handle->surprise_removed = MTRUE;

#ifdef IMX_SUPPORT
	woal_unregist_oob_wakeup_irq(card->handle);
#endif /* IMX_SUPPORT */
	woal_remove_card(card);
	woal_pcie_cleanup(card);
	kfree(card);

	LEAVE();
	return;
}

/**
 *  @brief This function handles PCIE driver remove
 *
 *  @param pdev     A pointer to pci_dev structure
 *
 *  @return         error code
 */
static void woal_pcie_shutdown(struct pci_dev *dev)
{
	pcie_service_card *card;
	moal_handle *handle;

	ENTER();
	PRINTM(MCMND, "<--- Enter woal_pcie_shutdown --->\n");

	card = pci_get_drvdata(dev);
	if (!card) {
		PRINTM(MINFO, "PCIE card removed from slot\n");
		LEAVE();
		return;
	}
	handle = card->handle;
	if (!handle) {
		PRINTM(MINFO, "Invalid handle\n");
		LEAVE();
		return;
	}
	if (handle->second_mac)
		goto done;
#if defined(PCIE9098) || defined(PCIE9097) || defined(PCIEAW693) ||            \
	defined(PCIEIW624)
	if (IS_PCIE9098(handle->card_type) || IS_PCIEIW624(handle->card_type) ||
	    IS_PCIEAW693(handle->card_type) || IS_PCIE9097(handle->card_type)) {
		if (RDWR_STATUS_FAILURE !=
		    woal_pcie_rdwr_firmware(handle, 0, 1))
			PRINTM(MMSG, "wlan: start in-bound IR...\n");
	}
#endif
done:
	handle->surprise_removed = MTRUE;
	pci_disable_device(dev);
	PRINTM(MCMND, "<--- Leave woal_pcie_shutdown --->\n");
	LEAVE();
	return;
}

/**
 *  @brief Handle suspend
 *
 *  @param pdev     A pointer to pci_dev structure
 *  @param state    PM state message
 *
 *  @return         error code
 */
static int woal_pcie_suspend(struct pci_dev *pdev, pm_message_t state)
{
	pcie_service_card *cardp;
	moal_handle *handle = NULL;
	moal_handle *ref_handle = NULL;
	int i;
	int ret = MLAN_STATUS_SUCCESS;
	int hs_actived;
	mlan_ds_ps_info pm_info;
	int keep_power = 0;

	ENTER();
	if (pdev) {
		cardp = (pcie_service_card *)pci_get_drvdata(pdev);
		if (!cardp || !cardp->handle) {
			LEAVE();
			return MLAN_STATUS_SUCCESS;
		}
	} else {
		PRINTM(MERROR, "PCIE device is not specified\n");
		LEAVE();
		return -ENOSYS;
	}

	handle = cardp->handle;
	if (handle->second_mac)
		PRINTM(MCMND, "<--- Enter woal_pcie_suspend# --->\n");
	else
		PRINTM(MCMND, "<--- Enter woal_pcie_suspend --->\n");
	if (handle->is_suspended == MTRUE) {
		PRINTM(MWARN, "Device already suspended\n");
		LEAVE();
		return MLAN_STATUS_SUCCESS;
	}
	if (handle->fw_dump) {
		PRINTM(MMSG, "suspend not allowed while FW dump!");
		ret = -EBUSY;
		goto done;
	}
	if (moal_extflg_isset(handle, EXT_PM_KEEP_POWER))
		keep_power = MTRUE;
	else
		keep_power = MFALSE;

	for (i = 0; i < MIN(handle->priv_num, MLAN_MAX_BSS_NUM); i++) {
		if (handle->priv[i])
			woal_cancel_scan(handle->priv[i], MOAL_IOCTL_WAIT);
#ifdef UAP_SUPPORT
		if (handle->priv[i] && !keep_power &&
		    handle->priv[i]->bss_started == MTRUE) {
			if (woal_uap_bss_ctrl(handle->priv[i], MOAL_IOCTL_WAIT,
					      UAP_BSS_STOP)) {
				PRINTM(MERROR, "%s: stop uap failed \n",
				       __func__);
			}
		}
#endif
	}
	handle->suspend_fail = MFALSE;
	if (keep_power) {
		memset(&pm_info, 0, sizeof(pm_info));
#define MAX_RETRY_NUM 8
		for (i = 0; i < MAX_RETRY_NUM; i++) {
			if (MLAN_STATUS_SUCCESS ==
			    woal_get_pm_info(woal_get_priv(handle,
							   MLAN_BSS_ROLE_ANY),
					     &pm_info)) {
				if (pm_info.is_suspend_allowed == MTRUE)
					break;
				else
					PRINTM(MMSG,
					       "Suspend not allowed and retry again\n");
			}
			woal_sched_timeout(100);
		}
		if (pm_info.is_suspend_allowed == MFALSE) {
			PRINTM(MMSG, "Suspend not allowed\n");
			ret = -EBUSY;
			goto done;
		}
	}
	for (i = 0; i < handle->priv_num; i++) {
		if (handle->priv[i])
			netif_device_detach(handle->priv[i]->netdev);
	}

	if (keep_power) {
		woal_sched_timeout(200);
		/* Enable Host Sleep */
		hs_actived = woal_enable_hs(
			woal_get_priv(handle, MLAN_BSS_ROLE_ANY));
		if (hs_actived == MTRUE) {
			/* Indicate device suspended */
			handle->is_suspended = MTRUE;
			woal_sched_timeout(10);
		} else {
			PRINTM(MMSG, "HS not actived, suspend fail!");
			handle->suspend_fail = MTRUE;
			for (i = 0; i < handle->priv_num; i++) {
				if (handle->priv[i])
					netif_device_attach(
						handle->priv[i]->netdev);
			}
			ret = -EBUSY;
			goto done;
		}
	}
	woal_flush_workqueue(handle);
	if (!keep_power) {
		handle->surprise_removed = MTRUE;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 18, 0)
		woal_do_flr(handle, true, false);
#endif
		handle->is_suspended = MTRUE;
	}
#ifdef IMX_SUPPORT
	woal_enable_oob_wakeup_irq(handle);
#endif /* IMX_SUPPORT */
	pci_enable_wake(pdev, pci_choose_state(pdev, state), 1);
	pci_save_state(pdev);
	ref_handle = (moal_handle *)handle->pref_mac;
	if (ref_handle && ref_handle->is_suspended)
		pci_set_power_state(pdev, pci_choose_state(pdev, state));
done:
	PRINTM(MCMND, "<--- Leave woal_pcie_suspend --->\n");
	LEAVE();
	return ret;
}

/**
 *  @brief Handle resume
 *
 *  @param pdev     A pointer to pci_dev structure
 *
 *  @return         error code
 */
static int woal_pcie_resume(struct pci_dev *pdev)
{
	moal_handle *handle;
	pcie_service_card *cardp;
	int keep_power = 0;
	int i;

	ENTER();
	if (pdev) {
		cardp = (pcie_service_card *)pci_get_drvdata(pdev);
		if (!cardp || !cardp->handle) {
			PRINTM(MERROR, "Card or handle is not valid\n");
			LEAVE();
			return MLAN_STATUS_SUCCESS;
		}
	} else {
		PRINTM(MERROR, "PCIE device is not specified\n");
		LEAVE();
		return -ENOSYS;
	}
	handle = cardp->handle;
	if (handle->second_mac)
		PRINTM(MCMND, "<--- Enter woal_pcie_resume# --->\n");
	else
		PRINTM(MCMND, "<--- Enter woal_pcie_resume --->\n");
	if (handle->is_suspended == MFALSE) {
		PRINTM(MWARN, "Device already resumed\n");
		goto done;
	}
	handle->is_suspended = MFALSE;

	if (moal_extflg_isset(handle, EXT_PM_KEEP_POWER))
		keep_power = MTRUE;
	else
		keep_power = MFALSE;

	pci_set_power_state(pdev, PCI_D0);
	pci_restore_state(pdev);
	pci_enable_wake(pdev, PCI_D0, 0);
	if (!keep_power) {
		handle->surprise_removed = MFALSE;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 18, 0)
		woal_do_flr(handle, false, false);
#endif
#if defined(STA_CFG80211) || defined(UAP_CFG80211)
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 2, 0)
		handle->cfg80211_suspend = MFALSE;
#endif
#endif
	} else {
		if (woal_check_driver_status(handle)) {
			PRINTM(MERROR, "Resuem, device is in hang state\n");
			LEAVE();
			return MLAN_STATUS_SUCCESS;
		}
		for (i = 0; i < handle->priv_num; i++) {
			if (handle->priv[i])
				netif_device_attach(handle->priv[i]->netdev);
		}

		woal_cancel_hs(woal_get_priv(handle, MLAN_BSS_ROLE_ANY),
			       MOAL_NO_WAIT);
#ifdef IMX_SUPPORT
		woal_disable_oob_wakeup_irq(handle);
#endif /* IMX_SUPPORT */
	}
done:
	PRINTM(MCMND, "<--- Leave woal_pcie_resume --->\n");
	LEAVE();
	return 0;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 18, 0)

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 13, 0)
/**
 *  @brief Pcie reset prepare handler
 *
 *  @param pdev     A pointer to pci_dev structure
 */
static void woal_pcie_reset_prepare(struct pci_dev *pdev)
{
	pcie_service_card *card;
	moal_handle *handle;
	moal_handle *ref_handle = NULL;

	ENTER();

	card = pci_get_drvdata(pdev);
	if (!card) {
		PRINTM(MINFO, "PCIE card removed from slot\n");
		LEAVE();
		return;
	}

	handle = card->handle;

	if (!handle) {
		PRINTM(MINFO, "Invalid handle\n");
		LEAVE();
		return;
	}

	PRINTM(MMSG, "%s: vendor=0x%4.04X device=0x%4.04X rev=%d Pre-FLR\n",
	       __func__, pdev->vendor, pdev->device, pdev->revision);

	/* Kernel would be performing FLR after this notification.
	 * Cleanup up all software withouth cleaning anything related to
	 * PCIe and HW.
	 * Note. FW might not be healthy.
	 */
	// handle-> mac0 , ref_handle->second mac
	if (handle->pref_mac) {
		if (handle->second_mac) {
			handle = (moal_handle *)handle->pref_mac;
			ref_handle = (moal_handle *)handle->pref_mac;
		} else {
			ref_handle = (moal_handle *)handle->pref_mac;
		}
	}
	handle->surprise_removed = MTRUE;
	handle->fw_reseting = MTRUE;
	// TODO: Can add more chips once the related code has been ported to fw
	// v18
	if (IS_PCIE9097(handle->card_type) || IS_PCIE9098(handle->card_type) ||
	    IS_PCIEAW693(handle->card_type)) {
		woal_reset_adma(handle);
	}

	woal_do_flr(handle, true, true);
	if (ref_handle) {
		ref_handle->surprise_removed = MTRUE;
		ref_handle->fw_reseting = MTRUE;
		woal_do_flr(ref_handle, true, true);
	}

	LEAVE();
}
/**
 *  @brief Pcie reset done handler
 *
 *  @param pdev     A pointer to pci_dev structure
 */
static void woal_pcie_reset_done(struct pci_dev *pdev)
{
	pcie_service_card *card;
	moal_handle *handle;
	moal_handle *ref_handle = NULL;
	ENTER();

	card = pci_get_drvdata(pdev);
	if (!card) {
		PRINTM(MINFO, "PCIE card removed from slot\n");
		LEAVE();
		return;
	}

	handle = card->handle;
	if (!handle) {
		PRINTM(MINFO, "Invalid handle\n");
		LEAVE();
		return;
	}

	PRINTM(MMSG, "%s: vendor=0x%4.04X device=0x%4.04X rev=%d Post-FLR\n",
	       __func__, pdev->vendor, pdev->device, pdev->revision);

	/* Kernel stores and restores PCIe function context before and
	 * after performing FLR, respectively.
	 *
	 * Reconfigure the sw and fw including fw redownload
	 */
	// handle-> mac0 , ref_handle->second mac
	if (handle->pref_mac) {
		if (handle->second_mac) {
			handle = (moal_handle *)handle->pref_mac;
			ref_handle = (moal_handle *)handle->pref_mac;
		} else {
			ref_handle = (moal_handle *)handle->pref_mac;
		}
	}
	handle->surprise_removed = MFALSE;
	if (MLAN_STATUS_SUCCESS == woal_do_flr(handle, false, true))
		handle->fw_reseting = MFALSE;
	else
		handle = NULL;
	if (ref_handle) {
		ref_handle->surprise_removed = MFALSE;
		if (MLAN_STATUS_SUCCESS == woal_do_flr(ref_handle, false, true))
			ref_handle->fw_reseting = MFALSE;
	}
	wifi_status = WIFI_STATUS_OK;
	if (handle)
		woal_send_auto_recovery_complete_event(handle);
	LEAVE();
}
#else
static void woal_pcie_reset_notify(struct pci_dev *pdev, bool prepare)
{
	pcie_service_card *card;
	moal_handle *handle;
	moal_handle *ref_handle = NULL;

	ENTER();

	card = pci_get_drvdata(pdev);
	if (!card) {
		PRINTM(MINFO, "PCIE card removed from slot\n");
		LEAVE();
		return;
	}

	handle = card->handle;
	if (!handle) {
		PRINTM(MINFO, "Invalid handle\n");
		LEAVE();
		return;
	}

	PRINTM(MMSG, "%s: vendor=0x%4.04X device=0x%4.04X rev=%d %s\n",
	       __func__, pdev->vendor, pdev->device, pdev->revision,
	       prepare ? "Pre-FLR" : "Post-FLR");

	// handle-> mac0 , ref_handle->second mac
	if (handle->pref_mac) {
		if (handle->second_mac) {
			handle = (moal_handle *)handle->pref_mac;
			ref_handle = (moal_handle *)handle->pref_mac;
		} else {
			ref_handle = (moal_handle *)handle->pref_mac;
		}
	}

	if (prepare) {
		/* Kernel would be performing FLR after this notification.
		 * Cleanup up all software withouth cleaning anything related to
		 * PCIe and HW.
		 * Note. FW might not be healthy.
		 */
		handle->surprise_removed = MTRUE;
		handle->fw_reseting = MTRUE;
		// TODO: Can add more chips once the related code has been
		// ported to fw v18
		if (IS_PCIE9097(handle->card_type) ||
		    IS_PCIE9098(handle->card_type) ||
		    IS_PCIEAW693(handle->card_type)) {
			woal_reset_adma(handle);
		}
		woal_do_flr(handle, prepare, true);
		if (ref_handle) {
			ref_handle->surprise_removed = MTRUE;
			ref_handle->fw_reseting = MTRUE;
			woal_do_flr(ref_handle, prepare, true);
		}
	} else {
		/* Kernel stores and restores PCIe function context before and
		 * after performing FLR, respectively.
		 *
		 * Reconfigure the sw and fw including fw redownload
		 */
		handle->surprise_removed = MFALSE;
		if (MLAN_STATUS_SUCCESS == woal_do_flr(handle, prepare, true))
			handle->fw_reseting = MFALSE;
		else
			handle = NULL;
		if (ref_handle) {
			ref_handle->surprise_removed = MFALSE;
			if (MLAN_STATUS_SUCCESS ==
			    woal_do_flr(ref_handle, prepare, true))
				ref_handle->fw_reseting = MFALSE;
		}
		wifi_status = WIFI_STATUS_OK;
		if (handle)
			woal_send_auto_recovery_complete_event(handle);
	}
	LEAVE();
}
#endif

static const struct pci_error_handlers woal_pcie_err_handler[] = {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 13, 0)
	{
		.reset_prepare = woal_pcie_reset_prepare,
		.reset_done = woal_pcie_reset_done,
	},
#else
	{
		.reset_notify = woal_pcie_reset_notify,
	},
#endif
};
#endif // KERNEL_VERSION(3.18.0)

/* PCI Device Driver */
static struct pci_driver REFDATA wlan_pcie = {
	.name = "wlan_pcie",
	.id_table = wlan_ids,
	.probe = woal_pcie_probe,
	.remove = woal_pcie_remove,
	.shutdown = woal_pcie_shutdown,
#ifdef CONFIG_PM
	/* Power Management Hooks */
	.suspend = woal_pcie_suspend,
	.resume = woal_pcie_resume,
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 18, 0)
	.err_handler = woal_pcie_err_handler,
#endif
};

/********************************************************
			Global Functions
********************************************************/

/**
 *  @brief This function writes data into card register
 *
 *  @param handle   A Pointer to the moal_handle structure
 *  @param reg      Register offset
 *  @param data     Value
 *
 *  @return    		MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status woal_pcie_write_reg(moal_handle *handle, t_u32 reg,
				       t_u32 data)
{
	pcie_service_card *card = (pcie_service_card *)handle->card;

	iowrite32(data, card->pci_mmap1 + reg);
	PRINTM(MREG, "pcie w %x = %x\n", reg, data);

	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function reads data from card register
 *
 *  @param handle   A Pointer to the moal_handle structure
 *  @param reg      Register offset
 *  @param data     Value
 *
 *  @return    		MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status woal_pcie_read_reg(moal_handle *handle, t_u32 reg,
				      t_u32 *data)
{
	pcie_service_card *card = (pcie_service_card *)handle->card;
	*data = ioread32(card->pci_mmap1 + reg);
	PRINTM(MREG, "pcie r %x = %x\n", reg, *data);

	if (*data == MLAN_STATUS_FAILURE)
		return MLAN_STATUS_FAILURE;

	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function writes multiple bytes into card memory
 *
 *  @param handle   A Pointer to the moal_handle structure
 *  @param pmbuf	Pointer to mlan_buffer structure
 *  @param port		Port
 *  @param timeout 	Time out value
 *
 *  @return    		MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status woal_pcie_write_data_sync(moal_handle *handle,
					     mlan_buffer *pmbuf, t_u32 port,
					     t_u32 timeout)
{
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function reads multiple bytes from card memory
 *
 *  @param handle   A Pointer to the moal_handle structure
 *  @param pmbuf	Pointer to mlan_buffer structure
 *  @param port		Port
 *  @param timeout 	Time out value
 *
 *  @return    		MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status woal_pcie_read_data_sync(moal_handle *handle,
					    mlan_buffer *pmbuf, t_u32 port,
					    t_u32 timeout)
{
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function handles the interrupt.
 *
 *  @param irq	    The irq no. of PCIE device
 *  @param dev_id   A pointer to the pci_dev structure
 *
 *  @return         IRQ_HANDLED
 */
static irqreturn_t woal_pcie_interrupt(int irq, void *dev_id)
{
	struct pci_dev *pdev;
	pcie_service_card *card;
	moal_handle *handle;
	mlan_status ret = MLAN_STATUS_SUCCESS;

	pdev = (struct pci_dev *)dev_id;
	if (!pdev) {
		PRINTM(MFATAL, "%s: pdev is NULL\n", (t_u8 *)pdev);
		goto exit;
	}

	card = (pcie_service_card *)pci_get_drvdata(pdev);
	if (!card || !card->handle) {
		PRINTM(MFATAL, "%s: card=%p handle=%p\n", __func__, card,
		       card ? card->handle : NULL);
		goto exit;
	}
	handle = card->handle;
	/* No need to handle Interrupt during FW reload, we can safely return
	 * success to Kernel */
	if (handle->surprise_removed == MTRUE && handle->fw_reseting) {
		PRINTM(MINFO, "*** SKIP INTR handling during FW reload ***\n");
		ret = MLAN_STATUS_SUCCESS;
		return IRQ_HANDLED;
	}
	PRINTM(MINFO, "*** IN PCIE IRQ ***\n");
	handle->main_state = MOAL_RECV_INT;
	if (handle->second_mac)
		PRINTM(MINTR, "**\n");
	else
		PRINTM(MINTR, "*\n");
	if (handle->is_suspended) {
		PRINTM(MERROR, "Receive interrupt in hs_suspended\n");
	}
	ret = mlan_interrupt(0xffff, handle->pmlan_adapter);

exit:
	if (ret == MLAN_STATUS_SUCCESS)
		return IRQ_HANDLED;
	else
		return IRQ_NONE;
}

/**
 *  @brief This function pre-initializes the PCI-E host
 *  memory space, etc.
 *
 *  @param handle   A pointer to moal_handle structure
 *
 *  @return         MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status woal_pcie_preinit(struct pci_dev *pdev)
{
	int ret;

	if (pdev->multifunction)
		device_disable_async_suspend(&pdev->dev);

	ret = pci_enable_device(pdev);

	if (ret)
		goto err_enable_dev;

	pci_set_master(pdev);

	PRINTM(MINFO, "Try set_consistent_dma_mask(32)\n");
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 18, 0)
	ret = dma_set_mask(&pdev->dev, DMA_BIT_MASK(32));
#else
	ret = pci_set_dma_mask(pdev, DMA_BIT_MASK(32));
#endif
	if (ret) {
		PRINTM(MERROR, "set_dma_mask(32) failed\n");
		goto err_set_dma_mask;
	}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 18, 0)
	ret = dma_set_coherent_mask(&pdev->dev, DMA_BIT_MASK(32));
#else
	ret = pci_set_consistent_dma_mask(pdev, DMA_BIT_MASK(32));
#endif
	if (ret) {
		PRINTM(MERROR, "set_consistent_dma_mask(64) failed\n");
		goto err_set_dma_mask;
	}
	return MLAN_STATUS_SUCCESS;

err_set_dma_mask:
	pci_disable_device(pdev);
err_enable_dev:
	return MLAN_STATUS_FAILURE;
}

/**
 *  @brief This function initializes the PCI-E host
 *  memory space, etc.
 *
 *  @param card   A pointer to pcie_service_card structure
 *
 *  @return         MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status woal_pcie_init(pcie_service_card *card)
{
	struct pci_dev *pdev = NULL;
	int ret;

	pdev = card->dev;
	pci_set_drvdata(pdev, card);
#if 0
	ret = pci_enable_device(pdev);
	if (ret)
		goto err_enable_dev;

	pci_set_master(pdev);

	PRINTM(MINFO, "Try set_consistent_dma_mask(32)\n");
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 18, 0)
	ret = dma_set_mask(&pdev->dev, DMA_BIT_MASK(32));
#else
	ret = pci_set_dma_mask(pdev, DMA_BIT_MASK(32));
#endif
	if (ret) {
		PRINTM(MERROR, "set_dma_mask(32) failed\n");
		goto err_set_dma_mask;
	}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 18, 0)
	ret = dma_set_coherent_mask(&pdev->dev, DMA_BIT_MASK(32));
#else
	ret = pci_set_consistent_dma_mask(pdev, DMA_BIT_MASK(32));
#endif
	if (ret) {
		PRINTM(MERROR, "set_consistent_dma_mask(64) failed\n");
		goto err_set_dma_mask;
	}
#endif

	ret = pci_request_region(pdev, 0, DRV_NAME);
	if (ret) {
		PRINTM(MERROR, "req_reg(0) error\n");
		goto err_req_region0;
	}
	card->pci_mmap = pci_iomap(pdev, 0, 0);
	if (!card->pci_mmap) {
		PRINTM(MERROR, "iomap(0) error\n");
		goto err_iomap0;
	}
	ret = pci_request_region(pdev, 2, DRV_NAME);
	if (ret) {
		PRINTM(MERROR, "req_reg(2) error\n");
		goto err_req_region2;
	}
	card->pci_mmap1 = pci_iomap(pdev, 2, 0);
	if (!card->pci_mmap1) {
		PRINTM(MERROR, "iomap(2) error\n");
		goto err_iomap2;
	}

	PRINTM(MMSG,
	       "PCI memory map Virt0: %p PCI memory map Virt2: "
	       "%p\n",
	       card->pci_mmap, card->pci_mmap1);

	return MLAN_STATUS_SUCCESS;

err_iomap2:
	pci_release_region(pdev, 2);
err_req_region2:
	pci_iounmap(pdev, card->pci_mmap);
err_iomap0:
	pci_release_region(pdev, 0);
err_req_region0:
#if 0
err_set_dma_mask:
#endif

#if 0
err_enable_dev:
#endif
	pci_set_drvdata(pdev, NULL);
	return MLAN_STATUS_FAILURE;
}

/**
 *  @brief This function registers the PCIE device
 *
 *  @param handle   A pointer to moal_handle structure
 *
 *  @return         MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status woal_pcie_register_dev(moal_handle *handle)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	pcie_service_card *card = NULL;
	struct pci_dev *pdev = NULL;
	ENTER();

	if (!handle || !handle->card) {
		PRINTM(MINFO, "%s: handle=%p card=%p\n", __FUNCTION__, handle,
		       handle ? handle->card : NULL);
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}

	card = (pcie_service_card *)handle->card;
	pdev = card->dev;
	/* save adapter pointer in card */
	card->handle = handle;

	switch (pcie_int_mode) {
		/* fall through */
	case PCIE_INT_MODE_MSI:
		pcie_int_mode = PCIE_INT_MODE_MSI;
		ret = pci_enable_msi(pdev);
		if (ret == 0) {
			ret = request_irq(pdev->irq, woal_pcie_interrupt, 0,
					  "mrvl_pcie_msi", pdev);
			if (ret) {
				PRINTM(MFATAL, "request_irq failed: ret=%d\n",
				       ret);
				pci_disable_msi(pdev);
			} else {
				break;
			}
		}
		// follow through

		/* fall through */
	case PCIE_INT_MODE_LEGACY:
		pcie_int_mode = PCIE_INT_MODE_LEGACY;
		ret = request_irq(pdev->irq, woal_pcie_interrupt, IRQF_SHARED,
				  "mrvl_pcie", pdev);
		if (ret) {
			PRINTM(MFATAL, "request_irq failed: ret=%d\n", ret);
			ret = MLAN_STATUS_FAILURE;
			goto done;
		}

		break;

	default:
		PRINTM(MFATAL, "pcie_int_mode %d failed\n", pcie_int_mode);
		ret = MLAN_STATUS_FAILURE;
		goto done;
		break;
	}

#if defined(PCIE9098) || defined(PCIEAW693)
	if ((card->dev->device == PCIE_DEVICE_ID_88W9098P_FN1) ||
	    (card->dev->device == PCIE_DEVICE_ID_88WAW693_FN1))
		mlan_set_int_mode(handle->pmlan_adapter, pcie_int_mode, 1);
	else
#endif
		mlan_set_int_mode(handle->pmlan_adapter, pcie_int_mode, 0);

done:
	LEAVE();
	return ret;
}

/**
 *  @brief This function cleans up the host memory spaces
 *
 *  @param card   A pointer to pcie_service_card structure
 *
 *  @return         N/A
 */
static void woal_pcie_cleanup(pcie_service_card *card)
{
	struct pci_dev *pdev = NULL;
	pdev = card->dev;
	PRINTM(MINFO, "Clearing driver ready signature\n");

	if (pdev) {
		pci_iounmap(pdev, card->pci_mmap);
		pci_iounmap(pdev, card->pci_mmap1);

		if (pci_is_enabled(pdev))
			pci_disable_device(pdev);

		pci_release_region(pdev, 0);
		pci_release_region(pdev, 2);
		pci_set_drvdata(pdev, NULL);
	}
}

/**
 *  @brief This function unregisters the PCIE device
 *
 *  @param handle   A pointer to moal_handle structure
 *
 *  @return         MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static void woal_pcie_unregister_dev(moal_handle *handle)
{
	pcie_service_card *card =
		handle ? (pcie_service_card *)handle->card : NULL;
	struct pci_dev *pdev = NULL;
	ENTER();

	if (card) {
		pdev = card->dev;
		PRINTM(MINFO, "%s(): calling free_irq()\n", __func__);

		switch (pcie_int_mode) {
		case PCIE_INT_MODE_MSI:
			free_irq(card->dev->irq, pdev);
			pci_disable_msi(pdev);
			break;

		case PCIE_INT_MODE_LEGACY:
			free_irq(card->dev->irq, pdev);
			break;

		default:
			PRINTM(MFATAL, "pcie_int_mode %d failed\n",
			       pcie_int_mode);
			break;
		}
		card->handle = NULL;
	}
	LEAVE();
}

/**
 *  @brief This function registers the IF module in bus driver
 *
 *  @return	    MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status woal_pcie_bus_register(void)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	ENTER();

	/* API registers the NXP PCIE driver */
	if (pci_register_driver(&wlan_pcie)) {
		PRINTM(MFATAL, "PCIE Driver Registration Failed \n");
		ret = MLAN_STATUS_FAILURE;
	}

	LEAVE();
	return ret;
}

/**
 *  @brief This function de-registers the IF module in bus driver
 *
 *  @return 	   N/A
 */
void woal_pcie_bus_unregister(void)
{
	ENTER();

	/* PCIE Driver Unregistration */
	pci_unregister_driver(&wlan_pcie);

	LEAVE();
}

#if defined(PCIE9098) || defined(PCIE9097) || defined(PCIEAW693) ||            \
	defined(PCIEIW624)
#define PCIE9098_DUMP_CTRL_REG 0x1C94
#define PCIE9098_DUMP_START_REG 0x1C98
#define PCIE9098_DUMP_END_REG 0x1C9F
#endif
#if defined(PCIE8897) || defined(PCIE8997)
#define DEBUG_DUMP_CTRL_REG 0xCF4
#define DEBUG_DUMP_START_REG 0xCF8
#define DEBUG_DUMP_END_REG 0xCFF
#endif

#if defined(PCIE9098) || defined(PCIE9097) || defined(PCIEAW693) ||            \
	defined(PCIEIW624)
#define PCIE9098_SCRATCH_12_REG 0x1C90
#define PCIE9098_SCRATCH_14_REG 0x1C98
#define PCIE9098_SCRATCH_15_REG 0x1C9C
#define PCIE9098_DUMP_REG_START 0x1C20
#define PCIE9098_DUMP_REG_END 0x1C9C
#endif

#if defined(PCIE8997) || defined(PCIE8897)
#define PCIE_SCRATCH_12_REG 0x0CF0;
#define PCIE_SCRATCH_14_REG 0x0CF8;
#define PCIE_SCRATCH_15_REG 0x0CFC;
#define PCIE_DUMP_START_REG 0xC00
#define PCIE_DUMP_END_REG 0xCFC
#endif
/**
 *  @brief This function save the log of pcie register value
 *
 *  @param phandle   A pointer to moal_handle
 *  @param buffer    A pointer to buffer saving log
 *
 *  @return         The length of this log
 */
static int woal_pcie_dump_reg_info(moal_handle *phandle, t_u8 *buffer)
{
	char *drv_ptr = (char *)buffer;
	t_u32 reg = 0, value = 0;
	t_u8 i;
	char buf[256], *ptr;
	pcie_service_card *card = (pcie_service_card *)phandle->card;
	int config_reg_table[] = {0x00, 0x04, 0x10, 0x18, 0x2c,
				  0x3c, 0x44, 0x80, 0x98, 0x170};
	t_u32 dump_start_reg = 0;
	t_u32 dump_end_reg = 0;
	t_u32 scratch_14_reg = 0;
	t_u32 scratch_15_reg = 0;
#if defined(PCIE9098) || defined(PCIE9097) || defined(PCIEAW693) ||            \
	defined(PCIEIW624)
	/* Tx/Rx/Event AMDA start address */
	t_u32 adma_reg_table[] = {0x10000, 0x10800, 0x10880, 0x11000, 0x11080};
	t_u8 j;
#endif
	ENTER();
	mlan_pm_wakeup_card(phandle->pmlan_adapter, MTRUE);
	drv_ptr += sprintf(drv_ptr,
			   "------------PCIe Registers dump-------------\n");
	drv_ptr += sprintf(drv_ptr, "Config Space Registers:\n");
	for (i = 0; i < ARRAY_SIZE(config_reg_table); i++) {
		pci_read_config_dword(card->dev, config_reg_table[i], &value);
		drv_ptr += sprintf(drv_ptr, "reg:0x%02x value=0x%08x\n",
				   config_reg_table[i], value);
	}

	reg = phandle->card_info->fw_stuck_code_reg;
	if (reg != 0) {
		woal_pcie_read_reg(phandle, reg, &value);
		value = (value & 0xff00) >> 8;
		if (value) {
			PRINTM(MERROR, "FW in debug mode (0x%x)\n", value);
			drv_ptr += sprintf(drv_ptr, "FW in debug mode (0x%x)\n",
					   value);
			if (value == FW_STUCK_CODE_VERSION_MISMATCH) {
				phandle->driver_init = MFALSE;
				PRINTM(MERROR,
				       "WLAN and BT FW version mismatch !! Redownload both FWs with correct and same version\n");
				drv_ptr += sprintf(
					drv_ptr,
					"WLAN and BT FW version mismatch !! Redownload both FWs with correct and same version\n");
			}
		}
	}

	drv_ptr += sprintf(drv_ptr, "FW Scrach Registers:\n");

#if defined(PCIE8897) || defined(PCIE8997)
	if (IS_PCIE8897(phandle->card_type) ||
	    IS_PCIE8997(phandle->card_type)) {
		reg = PCIE_SCRATCH_12_REG;
		dump_start_reg = PCIE_DUMP_START_REG;
		dump_end_reg = PCIE_DUMP_END_REG;
		scratch_14_reg = PCIE_SCRATCH_14_REG;
		scratch_15_reg = PCIE_SCRATCH_15_REG;
	}
#endif

#if defined(PCIE9098) || defined(PCIE9097) || defined(PCIEAW693) ||            \
	defined(PCIEIW624)
	if (IS_PCIE9098(phandle->card_type) ||
	    IS_PCIEIW624(phandle->card_type) ||
	    IS_PCIEAW693(phandle->card_type) ||
	    IS_PCIE9097(phandle->card_type)) {
		reg = PCIE9098_SCRATCH_12_REG;
		dump_start_reg = PCIE9098_DUMP_REG_START;
		dump_end_reg = PCIE9098_DUMP_REG_END;
		scratch_14_reg = PCIE9098_SCRATCH_14_REG;
		scratch_15_reg = PCIE9098_SCRATCH_15_REG;
	}
#endif

	woal_pcie_read_reg(phandle, reg, &value);
	drv_ptr += sprintf(drv_ptr, "reg:0x%x value=0x%x\n", reg, value);
	for (i = 0; i < 2; i++) {
		reg = scratch_14_reg;
		woal_pcie_read_reg(phandle, reg, &value);
		drv_ptr +=
			sprintf(drv_ptr, "reg:0x%x value=0x%x\n", reg, value);

		reg = scratch_15_reg;
		woal_pcie_read_reg(phandle, reg, &value);
		drv_ptr +=
			sprintf(drv_ptr, "reg:0x%x value=0x%x\n", reg, value);

		mdelay(100);
	}
	drv_ptr +=
		sprintf(drv_ptr,
			"Interface registers dump from offset 0x%x to 0x%x\n",
			dump_start_reg, dump_end_reg);
	memset(buf, 0, sizeof(buf));
	ptr = buf;
	i = 1;
	for (reg = dump_start_reg; reg <= dump_end_reg; reg += 4) {
		woal_pcie_read_reg(phandle, reg, &value);
		ptr += sprintf(ptr, "%08x ", value);
		if (!(i % 8)) {
			drv_ptr += sprintf(drv_ptr, "%s\n", buf);
			memset(buf, 0, sizeof(buf));
			ptr = buf;
		}
		i++;
	}
#if defined(PCIE9098) || defined(PCIE9097) || defined(PCIEAW693) ||            \
	defined(PCIEIW624)
	if (IS_PCIE9098(phandle->card_type) ||
	    IS_PCIEIW624(phandle->card_type) ||
	    IS_PCIEAW693(phandle->card_type) ||
	    IS_PCIE9097(phandle->card_type)) {
		drv_ptr += sprintf(
			drv_ptr,
			"PCIE registers from offset 0x1c20 to 0x1c9c:\n");
		memset(buf, 0, sizeof(buf));
		ptr = buf;
		i = 1;
		for (reg = 0x1c20; reg <= 0x1c9c; reg += 4) {
			woal_pcie_read_reg(phandle, reg, &value);
			ptr += sprintf(ptr, "%08x ", value);
			if (!(i % 8)) {
				drv_ptr += sprintf(drv_ptr, "%s\n", buf);
				memset(buf, 0, sizeof(buf));
				ptr = buf;
			}
			i++;
		}
		drv_ptr += sprintf(drv_ptr, "%s\n", buf);
	}
	if (IS_PCIE9098(phandle->card_type) ||
	    IS_PCIEIW624(phandle->card_type) ||
	    IS_PCIEAW693(phandle->card_type) ||
	    IS_PCIE9097(phandle->card_type)) {
		drv_ptr += sprintf(drv_ptr,
				   "ADMA Tx/Rx/Event/Cmd/CmdResp registers:\n");
		for (j = 0; j < ARRAY_SIZE(adma_reg_table); j++) {
			drv_ptr += sprintf(
				drv_ptr,
				"ADMA registers dump from offset 0x%x to 0x%x\n",
				adma_reg_table[j], adma_reg_table[j] + 0x68);
			memset(buf, 0, sizeof(buf));
			ptr = buf;
			i = 1;
			for (reg = adma_reg_table[j];
			     reg <= (adma_reg_table[j] + 0x68); reg += 4) {
				woal_pcie_read_reg(phandle, reg, &value);
				ptr += sprintf(ptr, "%08x ", value);
				if (!(i % 8)) {
					drv_ptr +=
						sprintf(drv_ptr, "%s\n", buf);
					memset(buf, 0, sizeof(buf));
					ptr = buf;
				}
				i++;
			}
			drv_ptr += sprintf(drv_ptr, "%s\n", buf);
		}
	}
#endif
	drv_ptr += sprintf(drv_ptr,
			   "-----------PCIe Registers dump End-----------\n");
	mlan_pm_wakeup_card(phandle->pmlan_adapter, MFALSE);
	LEAVE();
	return drv_ptr - (char *)buffer;
}

/**
 *  @brief This function reads and displays PCIE scratch registers for debugging
 *
 *  @param phandle  A pointer to moal_handle
 *
 *  @return         N/A
 */
static void woal_pcie_reg_dbg(moal_handle *phandle)
{
	t_u32 reg = 0, value = 0;
	t_u8 i;
	char buf[256], *ptr;
	pcie_service_card *card = (pcie_service_card *)phandle->card;
	int config_reg_table[] = {0x00, 0x04, 0x10, 0x18, 0x2c,
				  0x3c, 0x44, 0x80, 0x98, 0x170};
	t_u32 dump_start_reg = 0;
	t_u32 dump_end_reg = 0;
	t_u32 scratch_14_reg = 0;
	t_u32 scratch_15_reg = 0;
#if defined(PCIE9098) || defined(PCIE9097) || defined(PCIEAW693) ||            \
	defined(PCIEIW624)
	/* Tx/Rx/Event AMDA start address */
	t_u32 adma_reg_table[] = {0x10000, 0x10800, 0x10880, 0x11000, 0x11080};
	t_u8 j;
#endif
	mlan_pm_wakeup_card(phandle->pmlan_adapter, MTRUE);
	PRINTM(MMSG, "Config Space Registers:\n");
	for (i = 0; i < ARRAY_SIZE(config_reg_table); i++) {
		pci_read_config_dword(card->dev, config_reg_table[i], &value);
		PRINTM(MERROR, "reg:0x%02x value=0x%08x\n", config_reg_table[i],
		       value);
	}

	reg = phandle->card_info->fw_stuck_code_reg;
	if (reg != 0) {
		woal_pcie_read_reg(phandle, reg, &value);
		value = (value & 0xff00) >> 8;
		if (value) {
			PRINTM(MERROR, "FW in debug mode (0x%x)\n", value);
			if (value == FW_STUCK_CODE_VERSION_MISMATCH) {
				phandle->driver_init = MFALSE;
				PRINTM(MERROR,
				       "WLAN and BT FW version mismatch !! Redownload both FWs with correct and same version\n");
			}
		}
	}

	PRINTM(MMSG, "FW Scrach Registers:\n");
#if defined(PCIE8897) || defined(PCIE8997)
	if (IS_PCIE8897(phandle->card_type) ||
	    IS_PCIE8997(phandle->card_type)) {
		reg = PCIE_SCRATCH_12_REG;
		dump_start_reg = PCIE_DUMP_START_REG;
		dump_end_reg = PCIE_DUMP_END_REG;
		scratch_14_reg = PCIE_SCRATCH_14_REG;
		scratch_15_reg = PCIE_SCRATCH_15_REG;
	}
#endif

#if defined(PCIE9098) || defined(PCIE9097) || defined(PCIEAW693) ||            \
	defined(PCIEIW624)
	if (IS_PCIE9098(phandle->card_type) ||
	    IS_PCIEIW624(phandle->card_type) ||
	    IS_PCIEAW693(phandle->card_type) ||
	    IS_PCIE9097(phandle->card_type)) {
		reg = PCIE9098_SCRATCH_12_REG;
		dump_start_reg = PCIE9098_DUMP_START_REG;
		dump_end_reg = PCIE9098_DUMP_END_REG;
		scratch_14_reg = PCIE9098_SCRATCH_14_REG;
		scratch_15_reg = PCIE9098_SCRATCH_15_REG;
	}
#endif
	woal_pcie_read_reg(phandle, reg, &value);
	PRINTM(MERROR, "reg:0x%x value=0x%x\n", reg, value);
	for (i = 0; i < 2; i++) {
		reg = scratch_14_reg;
		woal_pcie_read_reg(phandle, reg, &value);
		PRINTM(MERROR, "reg:0x%x value=0x%x\n", reg, value);

		reg = scratch_15_reg;
		woal_pcie_read_reg(phandle, reg, &value);
		PRINTM(MERROR, "reg:0x%x value=0x%x\n", reg, value);

		mdelay(100);
	}
	PRINTM(MMSG, "Interface registers dump from offset 0x%x to 0x%x\n",
	       dump_start_reg, dump_end_reg);
	memset(buf, 0, sizeof(buf));
	ptr = buf;
	i = 1;
	for (reg = dump_start_reg; reg <= dump_end_reg; reg += 4) {
		woal_pcie_read_reg(phandle, reg, &value);
		ptr += sprintf(ptr, "%08x ", value);
		if (!(i % 8)) {
			PRINTM(MMSG, "%s\n", buf);
			memset(buf, 0, sizeof(buf));
			ptr = buf;
		}
		i++;
	}
#if defined(PCIE9098) || defined(PCIE9097) || defined(PCIEAW693) ||            \
	defined(PCIEIW624)
	if (IS_PCIE9098(phandle->card_type) ||
	    IS_PCIEIW624(phandle->card_type) ||
	    IS_PCIEAW693(phandle->card_type) ||
	    IS_PCIE9097(phandle->card_type)) {
		PRINTM(MMSG, "PCIE registers from offset 0x1c20 to 0x1c9c:\n");
		memset(buf, 0, sizeof(buf));
		ptr = buf;
		i = 1;
		for (reg = 0x1c20; reg <= 0x1c9c; reg += 4) {
			woal_pcie_read_reg(phandle, reg, &value);
			ptr += sprintf(ptr, "%08x ", value);
			if (!(i % 8)) {
				PRINTM(MMSG, "%s\n", buf);
				memset(buf, 0, sizeof(buf));
				ptr = buf;
			}
			i++;
		}
		PRINTM(MMSG, "%s\n", buf);
	}
	if (IS_PCIE9098(phandle->card_type) ||
	    IS_PCIEIW624(phandle->card_type) ||
	    IS_PCIEAW693(phandle->card_type) ||
	    IS_PCIE9097(phandle->card_type)) {
		PRINTM(MMSG, "ADMA Tx/Rx/Event/Cmd/CmdResp registers:\n");
		for (j = 0; j < ARRAY_SIZE(adma_reg_table); j++) {
			PRINTM(MMSG,
			       "ADMA registers dump from offset 0x%x to 0x%x\n",
			       adma_reg_table[j], adma_reg_table[j] + 0x68);
			memset(buf, 0, sizeof(buf));
			ptr = buf;
			i = 1;
			for (reg = adma_reg_table[j];
			     reg <= (adma_reg_table[j] + 0x68); reg += 4) {
				woal_pcie_read_reg(phandle, reg, &value);
				ptr += sprintf(ptr, "%08x ", value);
				if (!(i % 8)) {
					PRINTM(MMSG, "%s\n", buf);
					memset(buf, 0, sizeof(buf));
					ptr = buf;
				}
				i++;
			}
			PRINTM(MMSG, "%s\n", buf);
		}
	}
#endif
	mlan_pm_wakeup_card(phandle->pmlan_adapter, MFALSE);
}

#define DEBUG_FW_DONE 0xFF
#define MAX_POLL_TRIES 100

typedef enum {
	DUMP_TYPE_ITCM = 0,
	DUMP_TYPE_DTCM = 1,
	DUMP_TYPE_SQRAM = 2,
	DUMP_TYPE_IRAM = 3,
	DUMP_TYPE_APU = 4,
	DUMP_TYPE_CIU = 5,
	DUMP_TYPE_ICU = 6,
	DUMP_TYPE_MAC = 7,
} dumped_mem_type;

#define MAX_NAME_LEN 8

typedef struct {
	t_u8 mem_name[MAX_NAME_LEN];
	t_u8 *mem_Ptr;
	struct file *pfile_mem;
	t_u8 done_flag;
	t_u8 type;
} memory_type_mapping;

#ifdef PCIE8897
#define DEBUG_HOST_READY_8897 0xEE
#define DEBUG_MEMDUMP_FINISH_8897 0xFE
static memory_type_mapping mem_type_mapping_tbl_8897[] = {
	{"ITCM", NULL, NULL, 0xF0, FW_DUMP_TYPE_MEM_ITCM},
	{"DTCM", NULL, NULL, 0xF1, FW_DUMP_TYPE_MEM_DTCM},
	{"SQRAM", NULL, NULL, 0xF2, FW_DUMP_TYPE_MEM_SQRAM},
	{"IRAM", NULL, NULL, 0xF3, FW_DUMP_TYPE_MEM_IRAM},
	{"APU", NULL, NULL, 0xF4, FW_DUMP_TYPE_REG_APU},
	{"CIU", NULL, NULL, 0xF5, FW_DUMP_TYPE_REG_CIU},
	{"ICU", NULL, NULL, 0xF6, FW_DUMP_TYPE_REG_ICU},
	{"MAC", NULL, NULL, 0xF7, FW_DUMP_TYPE_REG_MAC},
};
#endif

#if defined(PCIE8997) || defined(PCIE9098) || defined(PCIE9097) ||             \
	defined(PCIEAW693) || defined(PCIEIW624)
#define DEBUG_HOST_READY_8997 0xCC
#define DEBUG_HOST_EVENT_READY 0xAA
#define DEBUG_HOST_RESET_READY 0x99
static memory_type_mapping mem_type_mapping_tbl_8997 = {"DUMP", NULL, NULL,
							0xDD, 0x00};

#endif

#if defined(PCIE8897) || defined(PCIE8997) || defined(PCIE9098) ||             \
	defined(PCIE9097) || defined(PCIEAW693) || defined(PCIEIW624)
/**
 *  @brief This function reads data by 8 bit from card register
 *
 *  @param handle   A Pointer to the moal_handle structure
 *  @param reg      Register offset
 *  @param data     Value
 *
 *  @return    		MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status woal_read_reg_eight_bit(moal_handle *handle, t_u32 reg,
					   t_u8 *data)
{
	pcie_service_card *card = (pcie_service_card *)handle->card;
	*data = ioread8(card->pci_mmap1 + reg);
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function read/write firmware
 *
 *  @param phandle   A pointer to moal_handle
 *  @param doneflag  done flag
 *  @param resetflag reset flag;
 *
 *  @return         MLAN_STATUS_SUCCESS
 */
static rdwr_status woal_pcie_rdwr_firmware(moal_handle *phandle, t_u8 doneflag,
					   t_u8 resetflag)
{
	int ret = 0;
	int tries = 0;
	t_u8 ctrl_data = 0;
	t_u32 reg_data = 0;
	t_u32 debug_host_ready = 0;
	t_u32 dump_ctrl_reg = 0;

#ifdef PCIE8897
	if (IS_PCIE8897(phandle->card_type)) {
		debug_host_ready = DEBUG_HOST_READY_8897;
		dump_ctrl_reg = DEBUG_DUMP_CTRL_REG;
	}
#endif
#if defined(PCIE8997)
	if (IS_PCIE8997(phandle->card_type)) {
		debug_host_ready = DEBUG_HOST_READY_8997;
		dump_ctrl_reg = DEBUG_DUMP_CTRL_REG;
	}
#endif

#if defined(PCIE9098) || defined(PCIE9097) || defined(PCIEAW693) ||            \
	defined(PCIEIW624)
	if (IS_PCIE9098(phandle->card_type) ||
	    IS_PCIEIW624(phandle->card_type) ||
	    IS_PCIEAW693(phandle->card_type) ||
	    IS_PCIE9097(phandle->card_type)) {
		if (phandle->event_fw_dump)
			debug_host_ready = DEBUG_HOST_EVENT_READY;
		else
			debug_host_ready = DEBUG_HOST_READY_8997;
		if (resetflag)
			debug_host_ready = DEBUG_HOST_RESET_READY;
		dump_ctrl_reg = PCIE9098_DUMP_CTRL_REG;
	}
#endif

	ret = woal_pcie_write_reg(phandle, dump_ctrl_reg, debug_host_ready);
	if (ret) {
		PRINTM(MERROR, "PCIE Write ERR, reg=0x%x debug_reay=0x%x\n",
		       dump_ctrl_reg, debug_host_ready);
		return RDWR_STATUS_FAILURE;
	}
#if defined(PCIE9098) || defined(PCIE9097) || defined(PCIEAW693) ||            \
	defined(PCIEIW624)
	if (IS_PCIE9098(phandle->card_type) ||
	    IS_PCIEIW624(phandle->card_type) ||
	    IS_PCIEAW693(phandle->card_type) ||
	    IS_PCIE9097(phandle->card_type)) {
		if (phandle->event_fw_dump || resetflag)
			return RDWR_STATUS_SUCCESS;
	}
#endif
	ret = woal_pcie_read_reg(phandle, dump_ctrl_reg, &reg_data);
	if (ret) {
		PRINTM(MERROR, "PCIE Read DEBUG_DUMP_CTRL_REG 0x%x fail\n",
		       dump_ctrl_reg);
		return RDWR_STATUS_FAILURE;
	}
	for (tries = 0; tries < MAX_POLL_TRIES; tries++) {
		ret = woal_read_reg_eight_bit(phandle, dump_ctrl_reg,
					      &ctrl_data);
		if (ret) {
			PRINTM(MERROR, "PCIE READ reg 0x%x 8bit ERR\n",
			       dump_ctrl_reg);
			return RDWR_STATUS_FAILURE;
		}
		if (ctrl_data == DEBUG_FW_DONE)
			break;
		if (doneflag && ctrl_data == doneflag)
			return RDWR_STATUS_DONE;
		if (ctrl_data != debug_host_ready) {
			PRINTM(MMSG,
			       "The ctrl reg was changed, ctrl_data=0x%x, host_ready:0x%x try again!\n",
			       ctrl_data, debug_host_ready);
			ret = woal_pcie_write_reg(phandle, dump_ctrl_reg,
						  debug_host_ready);
			if (ret) {
				PRINTM(MERROR, "PCIE Write ERR\n");
				return RDWR_STATUS_FAILURE;
			}
		}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 36)
		usleep_range(99, 100);
#else
		udelay(100);
#endif
	}
	if (ctrl_data == debug_host_ready) {
		PRINTM(MERROR, "Fail to pull ctrl_data=0x%x host_ready=0x%x\n",
		       ctrl_data, debug_host_ready);
		return RDWR_STATUS_FAILURE;
	}
	return RDWR_STATUS_SUCCESS;
}
#endif

#ifdef PCIE8897
/**
 *  @brief This function dump firmware memory to file
 *
 *  @param phandle   A pointer to moal_handle
 *
 *  @return         N/A
 */
static void woal_pcie_dump_fw_info_v1(moal_handle *phandle)
{
	int ret = 0;
	unsigned int reg, reg_start, reg_end;
	t_u8 *dbg_ptr = NULL;
	t_u32 sec, usec;
	t_u8 dump_num = 0;
	t_u8 idx = 0;
	t_u8 doneflag = 0;
	rdwr_status stat;
	t_u8 i = 0;
	t_u8 read_reg = 0;
	t_u32 memory_size = 0;
	t_u32 memdump_finsh = 0;
#ifndef DUMP_TO_PROC
	t_u8 path_name[64], file_name[32], firmware_dump_file[128];
#endif
	t_u8 *end_ptr = NULL;
	memory_type_mapping *mem_type_mapping_tbl = mem_type_mapping_tbl_8897;

	if (!phandle) {
		PRINTM(MERROR, "Could not dump firmwware info\n");
		return;
	}
#ifdef DUMP_TO_PROC
	if (!phandle->fw_dump_buf) {
		ret = moal_vmalloc(phandle, FW_DUMP_INFO_LEN,
				   &(phandle->fw_dump_buf));
		if (ret != MLAN_STATUS_SUCCESS || !phandle->fw_dump_buf) {
			PRINTM(MERROR, "Failed to vmalloc fw dump bufffer\n");
			return;
		}
	} else {
		memset(phandle->fw_dump_buf, 0x00, FW_DUMP_INFO_LEN);
	}
	phandle->fw_dump_len = 0;
#else
	memdump_finsh = DEBUG_MEMDUMP_FINISH_8897;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 1, 0)
	/** Create dump directory*/
	woal_create_dump_dir(phandle, path_name, sizeof(path_name));
#else
	memset(path_name, 0, sizeof(path_name));
	strcpy(path_name, "/data");
#endif
	PRINTM(MMSG, "Directory name is %s\n", path_name);
	woal_dump_drv_info(phandle, path_name);
#endif
	/* start dump fw memory	*/
	moal_get_system_time(phandle, &sec, &usec);
	PRINTM(MMSG, "====PCIE DEBUG MODE OUTPUT START: %u.%06u ====\n", sec,
	       usec);
	/* read the number of the memories which will dump */
	if (RDWR_STATUS_FAILURE ==
	    woal_pcie_rdwr_firmware(phandle, doneflag, 0))
		goto done;
	reg = DEBUG_DUMP_START_REG;
	ret = woal_read_reg_eight_bit(phandle, reg, &dump_num);
	if (ret) {
		PRINTM(MMSG, "PCIE READ MEM NUM ERR\n");
		goto done;
	}

	/* read the length of every memory which will dump */
	for (idx = 0;
	     idx < dump_num && idx < ARRAY_SIZE(mem_type_mapping_tbl_8897);
	     idx++) {
		if (RDWR_STATUS_FAILURE ==
		    woal_pcie_rdwr_firmware(phandle, doneflag, 0))
			goto done;
		memory_size = 0;
		reg = DEBUG_DUMP_START_REG;
		for (i = 0; i < 4; i++) {
			ret = woal_read_reg_eight_bit(phandle, reg, &read_reg);
			if (ret) {
				PRINTM(MMSG, "PCIE READ ERR\n");
				goto done;
			}
			memory_size |= (read_reg << i * 8);
			reg++;
		}
		if (memory_size == 0) {
			PRINTM(MMSG, "Firmware Dump Finished!\n");
			ret = woal_pcie_write_reg(phandle, DEBUG_DUMP_CTRL_REG,
						  memdump_finsh);
			if (ret) {
				PRINTM(MERROR,
				       "PCIE Write MEMDUMP_FINISH ERR\n");
				goto done;
			}
			break;
		} else {
			PRINTM(MMSG, "%s_SIZE=0x%x\n",
			       mem_type_mapping_tbl[idx].mem_name, memory_size);
			ret = moal_vmalloc(
				phandle, memory_size + 1,
				(t_u8 **)&mem_type_mapping_tbl[idx].mem_Ptr);
			if ((ret != MLAN_STATUS_SUCCESS) ||
			    !mem_type_mapping_tbl[idx].mem_Ptr) {
				PRINTM(MERROR,
				       "Error: vmalloc %s buffer failed!!!\n",
				       mem_type_mapping_tbl[idx].mem_name);
				goto done;
			}
			dbg_ptr = mem_type_mapping_tbl[idx].mem_Ptr;
			end_ptr = dbg_ptr + memory_size;
		}
		doneflag = mem_type_mapping_tbl[idx].done_flag;
		moal_get_system_time(phandle, &sec, &usec);
		PRINTM(MMSG, "Start %s output %u.%06u, please wait...\n",
		       mem_type_mapping_tbl[idx].mem_name, sec, usec);
		do {
			stat = woal_pcie_rdwr_firmware(phandle, doneflag, 0);
			if (RDWR_STATUS_FAILURE == stat)
				goto done;

			reg_start = DEBUG_DUMP_START_REG;
			reg_end = DEBUG_DUMP_END_REG;
			for (reg = reg_start; reg <= reg_end; reg++) {
				ret = woal_read_reg_eight_bit(phandle, reg,
							      dbg_ptr);
				if (ret) {
					PRINTM(MMSG, "PCIE READ ERR\n");
					goto done;
				}
				if (dbg_ptr < end_ptr)
					dbg_ptr++;
				else
					PRINTM(MINFO,
					       "pre-allocced buf is not enough\n");
			}
			if (RDWR_STATUS_DONE == stat) {
				PRINTM(MMSG, "%s done: size=0x%lx\n",
				       mem_type_mapping_tbl[idx].mem_name,
				       (dbg_ptr -
					mem_type_mapping_tbl[idx].mem_Ptr));
#ifdef DUMP_TO_PROC
				woal_save_dump_info_to_buf(
					phandle,
					mem_type_mapping_tbl[idx].mem_Ptr,
					memory_size,
					mem_type_mapping_tbl[idx].type);
#else
				memset(file_name, 0, sizeof(file_name));
				sprintf(file_name, "%s%s", "file_pcie_",
					mem_type_mapping_tbl[idx].mem_name);
				if (MLAN_STATUS_SUCCESS !=
				    woal_save_dump_info_to_file(
					    path_name, file_name,
					    mem_type_mapping_tbl[idx].mem_Ptr,
					    memory_size))
					PRINTM(MMSG,
					       "Can't save dump file %s in %s\n",
					       file_name, path_name);
#endif
				moal_vfree(phandle,
					   mem_type_mapping_tbl[idx].mem_Ptr);
				mem_type_mapping_tbl[idx].mem_Ptr = NULL;
				break;
			}
		} while (1);
	}
#ifdef DUMP_TO_PROC
	woal_append_end_block(phandle);
#endif
	moal_get_system_time(phandle, &sec, &usec);
	PRINTM(MMSG, "====PCIE DEBUG MODE OUTPUT END: %u.%06u ====\n", sec,
	       usec);
	/* end dump fw memory */
#ifndef DUMP_TO_PROC
	memset(firmware_dump_file, 0, sizeof(firmware_dump_file));
	sprintf(firmware_dump_file, "%s/%s", path_name, file_name);
	moal_memcpy_ext(phandle, phandle->firmware_dump_file,
			firmware_dump_file, sizeof(firmware_dump_file),
			sizeof(phandle->firmware_dump_file));
#endif
done:
	for (idx = 0;
	     idx < dump_num && idx < ARRAY_SIZE(mem_type_mapping_tbl_8897);
	     idx++) {
		if (mem_type_mapping_tbl[idx].mem_Ptr) {
			moal_vfree(phandle, mem_type_mapping_tbl[idx].mem_Ptr);
			mem_type_mapping_tbl[idx].mem_Ptr = NULL;
		}
	}

	return;
}
#endif

#if defined(PCIE8997) || defined(PCIE9098) || defined(PCIE9097) ||             \
	defined(PCIEAW693) || defined(PCIEIW624)
/**
 *  @brief This function dump firmware memory to file
 *
 *  @param phandle   A pointer to moal_handle
 *
 *  @return         N/A
 */
static void woal_pcie_dump_fw_info_v2(moal_handle *phandle)
{
	int ret = 0;
	unsigned int reg, reg_start, reg_end;
	t_u8 *dbg_ptr = NULL;
	t_u8 *tmp_ptr = NULL;
	t_u32 sec, usec;
	t_u8 dump_num = 0;
	t_u8 doneflag = 0;
	rdwr_status stat;
	t_u32 memory_size = 0;
#ifndef DUMP_TO_PROC
	t_u8 path_name[64], file_name[32], firmware_dump_file[128];
	moal_handle *ref_handle;
#endif
	t_u8 *end_ptr = NULL;
	memory_type_mapping *mem_type_mapping_tbl = &mem_type_mapping_tbl_8997;
	t_u32 dump_start_reg = 0;
	t_u32 dump_end_reg = 0;

	if (!phandle) {
		PRINTM(MERROR, "Could not dump firmwware info\n");
		return;
	}
#if defined(PCIE9098) || defined(PCIE9097) || defined(PCIEAW693) ||            \
	defined(PCIEIW624)
	if (IS_PCIE9098(phandle->card_type) ||
	    IS_PCIEIW624(phandle->card_type) ||
	    IS_PCIEAW693(phandle->card_type) ||
	    IS_PCIE9097(phandle->card_type)) {
		if (phandle->event_fw_dump) {
			if (RDWR_STATUS_FAILURE !=
			    woal_pcie_rdwr_firmware(phandle, doneflag, 0)) {
				PRINTM(MMSG,
				       "====PCIE FW DUMP EVENT MODE START ====\n");
				return;
			}
		}
	}
#endif
#ifndef DUMP_TO_PROC
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 1, 0)
	/** Create dump directory*/
	woal_create_dump_dir(phandle, path_name, sizeof(path_name));
#else
	memset(path_name, 0, sizeof(path_name));
	strcpy(path_name, "/data");
#endif
	PRINTM(MMSG, "Create DUMP directory success:dir_name=%s\n", path_name);
	ref_handle = (moal_handle *)phandle->pref_mac;
	if (ref_handle)
		woal_dump_drv_info(ref_handle, path_name);
	woal_dump_drv_info(phandle, path_name);
#endif

	/* start dump fw memory	*/
	moal_get_system_time(phandle, &sec, &usec);
	PRINTM(MMSG, "====PCIE DEBUG MODE OUTPUT START: %u.%06u ====\n", sec,
	       usec);
	/* read the number of the memories which will dump */
	if (RDWR_STATUS_FAILURE ==
	    woal_pcie_rdwr_firmware(phandle, doneflag, 0))
		goto done;
#if defined(PCIE9098) || defined(PCIE9097) || defined(PCIEAW693) ||            \
	defined(PCIEIW624)
	if (IS_PCIE9098(phandle->card_type) ||
	    IS_PCIEIW624(phandle->card_type) ||
	    IS_PCIEAW693(phandle->card_type) ||
	    IS_PCIE9097(phandle->card_type)) {
		dump_start_reg = PCIE9098_DUMP_START_REG;
		dump_end_reg = PCIE9098_DUMP_END_REG;
	}
#endif
#ifdef PCIE8997
	if (IS_PCIE8997(phandle->card_type)) {
		dump_start_reg = DEBUG_DUMP_START_REG;
		dump_end_reg = DEBUG_DUMP_END_REG;
	}
#endif
	reg = dump_start_reg;
	ret = woal_read_reg_eight_bit(phandle, reg, &dump_num);
	if (ret) {
		PRINTM(MMSG, "PCIE READ MEM NUM ERR\n");
		goto done;
	}

	memory_size = 0x80000;
	ret = moal_vmalloc(phandle, memory_size + 1,
			   (t_u8 **)&mem_type_mapping_tbl->mem_Ptr);
	if ((ret != MLAN_STATUS_SUCCESS) || !mem_type_mapping_tbl->mem_Ptr) {
		PRINTM(MERROR, "Error: vmalloc %s buffer failed!!!\n",
		       mem_type_mapping_tbl->mem_name);
		goto done;
	}
	dbg_ptr = mem_type_mapping_tbl->mem_Ptr;
	end_ptr = dbg_ptr + memory_size;

	doneflag = mem_type_mapping_tbl->done_flag;
	moal_get_system_time(phandle, &sec, &usec);
	PRINTM(MMSG, "Start %s output %u.%06u, please wait...\n",
	       mem_type_mapping_tbl->mem_name, sec, usec);
	do {
		stat = woal_pcie_rdwr_firmware(phandle, doneflag, 0);
		if (RDWR_STATUS_FAILURE == stat)
			goto done;

		reg_start = dump_start_reg;
		reg_end = dump_end_reg;
		for (reg = reg_start; reg <= reg_end; reg++) {
			ret = woal_read_reg_eight_bit(phandle, reg, dbg_ptr);
			if (ret) {
				PRINTM(MMSG, "PCIE READ ERR\n");
				goto done;
			}
			dbg_ptr++;
			if (dbg_ptr >= end_ptr) {
				PRINTM(MINFO,
				       "pre-allocced buf is not enough\n");
				ret = moal_vmalloc(phandle,
						   memory_size + 0x4000 + 1,
						   (t_u8 **)&tmp_ptr);
				if ((ret != MLAN_STATUS_SUCCESS) || !tmp_ptr) {
					PRINTM(MERROR,
					       "Error: vmalloc  buffer failed!!!\n");
					goto done;
				}
				moal_memcpy_ext(phandle, tmp_ptr,
						mem_type_mapping_tbl->mem_Ptr,
						memory_size,
						memory_size + 0x4000);
				moal_vfree(phandle,
					   mem_type_mapping_tbl->mem_Ptr);
				mem_type_mapping_tbl->mem_Ptr = tmp_ptr;
				tmp_ptr = NULL;
				dbg_ptr = mem_type_mapping_tbl->mem_Ptr +
					  memory_size;
				memory_size += 0x4000;
				end_ptr = mem_type_mapping_tbl->mem_Ptr +
					  memory_size;
			}
		}
		if (RDWR_STATUS_DONE == stat) {
#ifdef MLAN_64BIT
			PRINTM(MMSG,
			       "%s done:"
			       "size = 0x%lx\n",
			       mem_type_mapping_tbl->mem_name,
			       dbg_ptr - mem_type_mapping_tbl->mem_Ptr);
#else
			PRINTM(MMSG,
			       "%s done:"
			       "size = 0x%x\n",
			       mem_type_mapping_tbl->mem_name,
			       dbg_ptr - mem_type_mapping_tbl->mem_Ptr);
#endif
#ifdef DUMP_TO_PROC
			if (phandle->fw_dump_buf) {
				moal_vfree(phandle, phandle->fw_dump_buf);
				phandle->fw_dump_buf = NULL;
				phandle->fw_dump_len = 0;
			}
			phandle->fw_dump_buf = mem_type_mapping_tbl->mem_Ptr;
			phandle->fw_dump_len =
				dbg_ptr - mem_type_mapping_tbl->mem_Ptr;
			mem_type_mapping_tbl->mem_Ptr = NULL;
#else
			memset(file_name, 0, sizeof(file_name));
			sprintf(file_name, "%s%s", "file_pcie_",
				mem_type_mapping_tbl->mem_name);
			if (MLAN_STATUS_SUCCESS !=
			    woal_save_dump_info_to_file(
				    path_name, file_name,
				    mem_type_mapping_tbl->mem_Ptr,
				    dbg_ptr - mem_type_mapping_tbl->mem_Ptr))
				PRINTM(MMSG, "Can't save dump file %s in %s\n",
				       file_name, path_name);
			moal_vfree(phandle, mem_type_mapping_tbl->mem_Ptr);
			mem_type_mapping_tbl->mem_Ptr = NULL;
#endif
			break;
		}
	} while (1);
	moal_get_system_time(phandle, &sec, &usec);
	PRINTM(MMSG, "====PCIE DEBUG MODE OUTPUT END: %u.%06u ====\n", sec,
	       usec);
	/* end dump fw memory */
#ifndef DUMP_TO_PROC
	memset(firmware_dump_file, 0, sizeof(firmware_dump_file));
	sprintf(firmware_dump_file, "%s/%s", path_name, file_name);
	moal_memcpy_ext(phandle, phandle->firmware_dump_file,
			firmware_dump_file, sizeof(firmware_dump_file),
			sizeof(phandle->firmware_dump_file));
#endif
done:
	if (mem_type_mapping_tbl->mem_Ptr) {
		moal_vfree(phandle, mem_type_mapping_tbl->mem_Ptr);
		mem_type_mapping_tbl->mem_Ptr = NULL;
	}

	return;
}
#endif

/**
 *  @brief This function check if this is second mac
 *
 *  @param handle   A pointer to moal_handle structure
 *  @return         MTRUE/MFALSE
 *
 */
static t_u8 woal_pcie_is_second_mac(moal_handle *handle)
{
#if defined(PCIE9098) || defined(PCIEAW693)
	pcie_service_card *card = (pcie_service_card *)handle->card;
	if ((card->dev->device == PCIE_DEVICE_ID_88W9098P_FN1) ||
	    (card->dev->device == PCIE_DEVICE_ID_88WAW693_FN1))
		return MTRUE;
#endif
	return MFALSE;
}

static void woal_pcie_dump_fw_info(moal_handle *phandle)
{
	moal_private *priv = NULL;
#ifdef DUMP_TO_PROC
	if (phandle->fw_dump_buf) {
		PRINTM(MMSG, "FW dump already exist, free existing dump\n");
		moal_vfree(phandle, phandle->fw_dump_buf);
		phandle->fw_dump_buf = NULL;
		phandle->fw_dump_len = 0;
	}
#endif
	mlan_pm_wakeup_card(phandle->pmlan_adapter, MTRUE);
	phandle->fw_dump = MTRUE;
#ifdef PCIE8897
	if (IS_PCIE8897(phandle->card_type))
		woal_pcie_dump_fw_info_v1(phandle);
#endif
#if defined(PCIE8997) || defined(PCIE9098) || defined(PCIE9097) ||             \
	defined(PCIEAW693) || defined(PCIEIW624)
	if (IS_PCIE8997(phandle->card_type) ||
	    IS_PCIEAW693(phandle->card_type) ||
	    IS_PCIEIW624(phandle->card_type) ||
	    IS_PCIE9098(phandle->card_type) ||
	    IS_PCIE9097(phandle->card_type)) {
		woal_pcie_dump_fw_info_v2(phandle);
		if (phandle->event_fw_dump) {
			queue_work(phandle->workqueue, &phandle->main_work);
			phandle->is_fw_dump_timer_set = MTRUE;
			woal_mod_timer(&phandle->fw_dump_timer,
				       MOAL_FW_DUMP_TIMER);
			return;
		}
	}
#endif
	phandle->fw_dump = MFALSE;
	if (!phandle->priv_num)
		return;
	priv = woal_get_priv(phandle, MLAN_BSS_ROLE_ANY);
	if (priv == NULL) {
		return;
	}
	woal_send_fw_dump_complete_event(priv);
	mlan_pm_wakeup_card(phandle->pmlan_adapter, MFALSE);
	queue_work(phandle->workqueue, &phandle->main_work);
	woal_process_hang(phandle);
}

/**
 *  @brief This function get fw name
 *
 *  @param handle   A pointer to moal_handle structure
 *  @return         MLAN_STATUS_SUCCESS
 *
 */
static mlan_status woal_pcie_get_fw_name(moal_handle *handle)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
#if defined(PCIE9098) || defined(PCIEAW693)
	pcie_service_card *card = (pcie_service_card *)handle->card;
	moal_handle *ref_handle = NULL;
#endif

#if defined(PCIE8997) || defined(PCIE9098) || defined(PCIE9097) ||             \
	defined(PCIEAW693) || defined(PCIEIW624)
	t_u32 rev_id_reg = handle->card_info->rev_id_reg;
	t_u32 revision_id = 0;
#endif

#if defined(PCIE8997) || defined(PCIE9098) || defined(PCIE9097) ||             \
	defined(PCIEAW693) || defined(PCIEIW624)
	t_u32 host_strap_reg = handle->card_info->host_strap_reg;
	t_u32 magic_reg = handle->card_info->magic_reg;
	t_u32 strap = 0;
	t_u32 magic = 0;
#endif
#if defined(PCIEIW624) || defined(PCIEAW693)
	t_u32 boot_mode_reg = handle->card_info->boot_mode_reg;
	t_u32 boot_mode;
#endif
	t_u32 value = 0;

	ENTER();

	/* Make sure device is awake before reading host interface registers */
	woal_pcie_read_reg(handle, handle->card_info->fw_wakeup_reg, &value);

	if (handle->params.fw_name) {
#ifdef PCIE9097
		if (IS_PCIE9097(handle->card_type)) {
			woal_pcie_read_reg(handle, rev_id_reg, &revision_id);
			revision_id &= 0xff;
			PRINTM(MCMND, "revision_id=0x%x\n", revision_id);
			switch (revision_id) {
			case PCIE9097_A0:
				break;
			case PCIE9097_B0:
			case PCIE9097_B1:
				handle->card_rev = CHIP_9097_REV_B0;
				break;
			default:
				break;
			}
		}
#endif
#ifdef PCIEAW693
		if (IS_PCIEAW693(handle->card_type)) {
			woal_pcie_read_reg(handle, rev_id_reg, &revision_id);
			revision_id &= 0xff;
			PRINTM(MCMND, "revision_id=0x%x\n", revision_id);
			switch (revision_id) {
			case PCIEAW693_A1:
				handle->card_rev = CHIP_AW693_REV_A1;
				break;
			default:
				handle->card_rev = CHIP_AW693_REV_A0;
				break;
			}
		}
#endif
		goto done;
	}

#ifdef PCIE8997
	if (IS_PCIE8997(handle->card_type)) {
		woal_pcie_read_reg(handle, rev_id_reg, &revision_id);
		woal_pcie_read_reg(handle, host_strap_reg, &strap);
		woal_pcie_read_reg(handle, magic_reg, &magic);
		revision_id &= 0xff;
		strap &= 0x7;
		magic &= 0xff;
		PRINTM(MCMND, "magic=0x%x, strap=0x%x, revision_id=0x%x\n",
		       magic, strap, revision_id);
		if ((revision_id == PCIE8997_A1) &&
		    (magic == CHIP_MAGIC_VALUE)) {
			if (strap == CARD_TYPE_PCIE_UART)
				strcpy(handle->card_info->fw_name,
				       PCIEUART8997_DEFAULT_COMBO_FW_NAME);
			else
				strcpy(handle->card_info->fw_name,
				       PCIEUSB8997_DEFAULT_COMBO_FW_NAME);
		}
	}
#endif
#ifdef PCIE9098
	if (IS_PCIE9098(handle->card_type)) {
		if (card->dev->device == PCIE_DEVICE_ID_88W9098P_FN0) {
			woal_pcie_read_reg(handle, rev_id_reg, &revision_id);
			woal_pcie_read_reg(handle, host_strap_reg, &strap);
			woal_pcie_read_reg(handle, magic_reg, &magic);
			revision_id &= 0xff;
			strap &= 0x7;
			magic &= 0xff;
			PRINTM(MCMND,
			       "magic=0x%x, strap=0x%x, revision_id=0x%x\n",
			       magic, strap, revision_id);
			switch (revision_id) {
			case PCIE9098_Z1Z2:
				if (magic == CHIP_MAGIC_VALUE) {
					if (strap == CARD_TYPE_PCIE_UART)
						strcpy(handle->card_info
							       ->fw_name,
						       PCIEUART9098_DEFAULT_COMBO_FW_NAME);
					else if (strap == CARD_TYPE_PCIE_PCIE)
						strcpy(handle->card_info
							       ->fw_name,
						       PCIEPCIE9098_DEFAULT_COMBO_FW_NAME);
					else
						strcpy(handle->card_info
							       ->fw_name,
						       PCIEUSB9098_DEFAULT_COMBO_FW_NAME);
				}
				strcpy(handle->card_info->fw_name_wlan,
				       PCIE9098_DEFAULT_WLAN_FW_NAME);
				break;
			case PCIE9098_A0:
			case PCIE9098_A1:
			case PCIE9098_A2:
				if (magic == CHIP_MAGIC_VALUE) {
					if (strap == CARD_TYPE_PCIE_UART)
						strcpy(handle->card_info
							       ->fw_name,
						       PCIEUART9098_COMBO_V1_FW_NAME);
					else if (strap == CARD_TYPE_PCIE_PCIE)
						strcpy(handle->card_info
							       ->fw_name,
						       PCIEPCIE9098_COMBO_V1_FW_NAME);
					else
						strcpy(handle->card_info
							       ->fw_name,
						       PCIEUSB9098_COMBO_V1_FW_NAME);
				} else {
					strcpy(handle->card_info->fw_name,
					       PCIEUART9098_COMBO_V1_FW_NAME);
				}
				strcpy(handle->card_info->fw_name_wlan,
				       PCIE9098_WLAN_V1_FW_NAME);
				break;
			default:
				break;
			}
		} else {
			ref_handle = (moal_handle *)handle->pref_mac;
			if (ref_handle) {
				strcpy(handle->card_info->fw_name,
				       ref_handle->card_info->fw_name);
				strcpy(handle->card_info->fw_name_wlan,
				       ref_handle->card_info->fw_name_wlan);
			}
		}
	}
#endif
#ifdef PCIE9097
	if (IS_PCIE9097(handle->card_type)) {
		woal_pcie_read_reg(handle, rev_id_reg, &revision_id);
		woal_pcie_read_reg(handle, host_strap_reg, &strap);
		woal_pcie_read_reg(handle, magic_reg, &magic);
		revision_id &= 0xff;
		strap &= 0x7;
		magic &= 0xff;
		PRINTM(MCMND, "magic=0x%x, strap=0x%x, revision_id=0x%x\n",
		       magic, strap, revision_id);
		switch (revision_id) {
		case PCIE9097_A0:
			if (magic == CHIP_MAGIC_VALUE) {
				if (strap == CARD_TYPE_PCIE_UART)
					strcpy(handle->card_info->fw_name,
					       PCIEUART9097_DEFAULT_COMBO_FW_NAME);
				else
					strcpy(handle->card_info->fw_name,
					       PCIEUSB9097_DEFAULT_COMBO_FW_NAME);
			}
			strcpy(handle->card_info->fw_name_wlan,
			       PCIE9097_DEFAULT_WLAN_FW_NAME);
			break;
		case PCIE9097_B0:
		case PCIE9097_B1:
			if (magic == CHIP_MAGIC_VALUE) {
				if (strap == CARD_TYPE_PCIE_UART)
					strcpy(handle->card_info->fw_name,
					       PCIEUART9097_COMBO_V1_FW_NAME);
				else
					strcpy(handle->card_info->fw_name,
					       PCIEUSB9097_COMBO_V1_FW_NAME);
			} else {
				strcpy(handle->card_info->fw_name,
				       PCIEUART9097_COMBO_V1_FW_NAME);
			}
			strcpy(handle->card_info->fw_name_wlan,
			       PCIE9097_WLAN_V1_FW_NAME);
			handle->card_rev = CHIP_9097_REV_B0;
			break;
		default:
			break;
		}
	}
#endif
#ifdef PCIEAW693
	if (IS_PCIEAW693(handle->card_type)) {
		if (card->dev->device == PCIE_DEVICE_ID_88WAW693_FN0) {
			woal_pcie_read_reg(handle, rev_id_reg, &revision_id);
			woal_pcie_read_reg(handle, host_strap_reg, &strap);
			woal_pcie_read_reg(handle, magic_reg, &magic);
			woal_pcie_read_reg(handle, boot_mode_reg, &boot_mode);
			revision_id &= 0xff;
			strap &= 0x7;
			magic &= 0xff;
			boot_mode &= 0x03;
			PRINTM(MCMND,
			       "magic=0x%x, boot_mode=0x%x, strap=0x%x, revision_id=0x%x\n",
			       magic, boot_mode, strap, revision_id);
			if (boot_mode == 0x03)
				PRINTM(MMSG,
				       "wlan: PCIE-AW693 in secure-boot mode\n");

			switch (revision_id) {
			case PCIEAW693_A1:
				handle->card_rev = CHIP_AW693_REV_A1;
				if (magic == CHIP_MAGIC_VALUE) {
					if (strap == CARD_TYPE_PCIE_UART)
						strcpy(handle->card_info
							       ->fw_name,
						       PCIEUARTAW693_COMBO_V1_FW_NAME);
					else
						strcpy(handle->card_info
							       ->fw_name,
						       PCIEAW693_COMBO_V1_FW_NAME);
				} else {
					strcpy(handle->card_info->fw_name,
					       PCIEAW693_COMBO_V1_FW_NAME);
				}
				strcpy(handle->card_info->fw_name_wlan,
				       PCIEAW693_WLAN_V1_FW_NAME);
				if (boot_mode != 0x03) {
					/* remove extension .se */
					if (strstr(handle->card_info->fw_name,
						   ".se")) {
						// strstr's output is sanitized
						// in if condition
						// coverity[cert_pos54_c_violation:SUPPRESS]
						// coverity[cert_err33_c_violation:SUPPRESS]
						memset(strstr(handle->card_info
								      ->fw_name,
							      ".se"),
						       '\0', sizeof(".se"));
					}
					if (strstr(handle->card_info
							   ->fw_name_wlan,
						   ".se")) {
						// strstr's output is sanitized
						// in if condition
						// coverity[cert_pos54_c_violation:SUPPRESS]
						// coverity[cert_err33_c_violation:SUPPRESS]
						memset(strstr(handle->card_info
								      ->fw_name_wlan,
							      ".se"),
						       '\0', sizeof(".se"));
					}
				}
				break;
			case PCIEAW693_A0:
				handle->card_rev = CHIP_AW693_REV_A0;
				if (magic == CHIP_MAGIC_VALUE) {
					if (strap == CARD_TYPE_PCIE_UART)
						strcpy(handle->card_info
							       ->fw_name,
						       PCIEUARTAW693_DEFAULT_COMBO_FW_NAME);
					else
						strcpy(handle->card_info
							       ->fw_name,
						       PCIEAW693_DEFAULT_COMBO_FW_NAME);
				}
				strcpy(handle->card_info->fw_name_wlan,
				       PCIEAW693_DEFAULT_WLAN_FW_NAME);
				break;
			default:
				break;
			}
		} else {
			ref_handle = (moal_handle *)handle->pref_mac;
			if (ref_handle) {
				woal_pcie_read_reg(handle, rev_id_reg,
						   &revision_id);
				revision_id &= 0xff;
				PRINTM(MCMND, "revision_id=0x%x\n",
				       revision_id);
				switch (revision_id) {
				case PCIEAW693_A1:
					handle->card_rev = CHIP_AW693_REV_A1;
					break;
				default:
					handle->card_rev = CHIP_AW693_REV_A0;
					break;
				}
				strcpy(handle->card_info->fw_name,
				       ref_handle->card_info->fw_name);
				strcpy(handle->card_info->fw_name_wlan,
				       ref_handle->card_info->fw_name_wlan);
			}
		}
	}
#endif
#ifdef PCIEIW624
	if (IS_PCIEIW624(handle->card_type)) {
		woal_pcie_read_reg(handle, rev_id_reg, &revision_id);
		woal_pcie_read_reg(handle, host_strap_reg, &strap);
		woal_pcie_read_reg(handle, magic_reg, &magic);
		woal_pcie_read_reg(handle, boot_mode_reg, &boot_mode);
		revision_id &= 0xff;
		strap &= 0x7;
		magic &= 0xff;
		boot_mode &= 0x03;
		PRINTM(MCMND,
		       "magic=0x%x boot_mode=0x%x, strap=0x%x, revision_id=0x%x\n",
		       magic, boot_mode, strap, revision_id);
		if (boot_mode == 0x03)
			PRINTM(MMSG, "wlan: PCIE-IW624 in secure-boot mode\n");
		if (strap == CARD_TYPE_PCIEIW624_UARTUART) {
			if (handle->params.dual_nb)
				strcpy(handle->card_info->fw_name,
				       PCIEUARTUARTIW624_DEFAULT_COMBO_FW_NAME);
			else
				strcpy(handle->card_info->fw_name,
				       PCIEUARTIW624_DEFAULT_COMBO_FW_NAME);
		} else if (strap == CARD_TYPE_PCIEIW624_UARTSPI) {
			if (handle->params.dual_nb)
				strcpy(handle->card_info->fw_name,
				       PCIEUARTSPIIW624_DEFAULT_COMBO_FW_NAME);
			else
				strcpy(handle->card_info->fw_name,
				       PCIEUARTIW624_DEFAULT_COMBO_FW_NAME);
		} else {
			if (handle->params.dual_nb)
				strcpy(handle->card_info->fw_name,
				       PCIEUSBUSBIW624_DEFAULT_COMBO_FW_NAME);
			else
				strcpy(handle->card_info->fw_name,
				       PCIEUSBIW624_DEFAULT_COMBO_FW_NAME);
		}
	}
#endif
done:
	PRINTM(MCMND, "combo fw:%s wlan fw:%s \n", handle->card_info->fw_name,
	       handle->card_info->fw_name_wlan);
	LEAVE();
	return ret;
}

/**
 *  @brief This function trigger in-band reset to FW
 *
 *  @param handle   A pointer to moal_handle structure
 *  @return         0 or failure
 *
 */
static int woal_pcie_reset_fw(moal_handle *handle)
{
	int ret = 0, tries = 0;
	t_u32 value = 1;
	t_u32 reset_reg = handle->card_info->fw_reset_reg;
	t_u8 reset_val = handle->card_info->fw_reset_val;

	ENTER();
	if (IS_PCIE8897(handle->card_type)) {
		PRINTM(MERROR, "PCIE8897 don't support PCIE in-band reset\n");
		LEAVE();
		return -EFAULT;
	}
	woal_pcie_read_reg(handle, handle->card_info->fw_wakeup_reg, &value);
	udelay(100);

	/* Write register to notify FW */
	if (woal_pcie_write_reg(handle, reset_reg, reset_val) !=
	    MLAN_STATUS_SUCCESS) {
		PRINTM(MERROR, "Failed to write reregister.\n");
		ret = -EFAULT;
		goto done;
	}
	/* Poll register around 100 ms */
	for (tries = 0; tries < MAX_POLL_TRIES; ++tries) {
		woal_pcie_read_reg(handle, reset_reg, &value);
		if (value == 0)
			/* FW is ready */
			break;
		udelay(1000);
	}

	if (value) {
		PRINTM(MERROR, "Failed to poll FW reset register %X=0x%x\n",
		       reset_reg, value);
		ret = -EFAULT;
		goto done;
	}

	PRINTM(MMSG, "PCIE Trigger FW In-band Reset success.");
done:
	LEAVE();
	return ret;
}

/**
 *  @brief This function handle the pcie work
 *
 *  @param WORK   A pointer to work_struct
 *  @return         N/A
 *
 */
static void woal_pcie_work(struct work_struct *work)
{
	pcie_service_card *card =
		// Coverity violation raised for kernel's API
		// coverity[cert_arr39_c_violation:SUPPRESS]
		container_of(work, pcie_service_card, reset_work);
	moal_handle *handle = NULL;
	moal_handle *ref_handle = NULL;
	handle = card->handle;
	if (!handle)
		return;

	PRINTM(MMSG, "========START IN-BAND RESET===========\n");

	// handle-> mac0 , ref_handle->second mac
	if (handle->pref_mac) {
		if (handle->second_mac) {
			handle = (moal_handle *)handle->pref_mac;
			ref_handle = (moal_handle *)handle->pref_mac;
		} else {
			ref_handle = (moal_handle *)handle->pref_mac;
		}
		if (ref_handle) {
			ref_handle->surprise_removed = MTRUE;
			woal_clean_up(ref_handle);
			mlan_ioctl(ref_handle->pmlan_adapter, NULL);
		}
	}
	handle->surprise_removed = MTRUE;
	handle->fw_reseting = MTRUE;
	// TODO: Can add more chips once the related code has been ported to fw
	// v18
	if (IS_PCIE9097(handle->card_type) || IS_PCIE9098(handle->card_type) ||
	    IS_PCIEAW693(handle->card_type)) {
		woal_reset_adma(handle);
	}
	woal_do_flr(handle, true, true);
	if (ref_handle) {
		ref_handle->surprise_removed = MTRUE;
		ref_handle->fw_reseting = MTRUE;
		woal_do_flr(ref_handle, true, true);
	}
	if (woal_pcie_reset_fw(handle)) {
		PRINTM(MERROR, "PCIe In-band Reset Fail\n");
		woal_send_auto_recovery_failure_event(handle);
		wifi_status = WIFI_STATUS_FW_RECOVERY_FAIL;
		return;
	}
	handle->surprise_removed = MFALSE;

	woal_free_module_param(handle);
	woal_init_module_param(handle);

	if (MLAN_STATUS_SUCCESS == woal_do_flr(handle, false, true))
		handle->fw_reseting = MFALSE;
	else {
		handle = NULL;
		wifi_status = WIFI_STATUS_FW_RECOVERY_FAIL;
		return;
	}
	if (ref_handle) {
		ref_handle->surprise_removed = MFALSE;

		woal_free_module_param(ref_handle);
		woal_init_module_param(ref_handle);

		if (MLAN_STATUS_SUCCESS == woal_do_flr(ref_handle, false, true))
			ref_handle->fw_reseting = MFALSE;
	}
	card->work_flags = MFALSE;
	wifi_status = WIFI_STATUS_OK;
	woal_send_auto_recovery_complete_event(handle);
	PRINTM(MMSG, "========END IN-BAND RESET===========\n");
	return;
}

/**
 *  @brief This function start reset_work
 *
 *  @param handle   A pointer to moal_handle structure
 *  @return         MTRUE/MFALSE
 *
 */
static void woal_pcie_card_reset(moal_handle *handle)
{
	pcie_service_card *card = (pcie_service_card *)handle->card;
	if (!card->work_flags) {
		card->work_flags = MTRUE;
		schedule_work(&card->reset_work);
	}
}

static moal_if_ops pcie_ops = {
	.register_dev = woal_pcie_register_dev,
	.unregister_dev = woal_pcie_unregister_dev,
	.read_reg = woal_pcie_read_reg,
	.write_reg = woal_pcie_write_reg,
	.read_data_sync = woal_pcie_read_data_sync,
	.write_data_sync = woal_pcie_write_data_sync,
	.get_fw_name = woal_pcie_get_fw_name,
	.dump_fw_info = woal_pcie_dump_fw_info,
	.reg_dbg = woal_pcie_reg_dbg,
	.dump_reg_info = woal_pcie_dump_reg_info,
	.card_reset = woal_pcie_card_reset,
	.is_second_mac = woal_pcie_is_second_mac,
};
