/** @file moal_uap.c
 *
 * @brief This file contains the major functions in UAP
 * driver.
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
    10/21/2008: initial version
********************************************************/

#include "moal_main.h"
#include "moal_uap.h"
#ifdef SDIO
#include "moal_sdio.h"
#endif /* SDIO */
#include "moal_eth_ioctl.h"
#if defined(STA_CFG80211) && defined(UAP_CFG80211)
#include "moal_cfg80211.h"
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0)
#include <linux/compat.h>
#endif

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
 *  @brief uap addba parameter handler
 *
 *  @param dev      A pointer to net_device structure
 *  @param req      A pointer to ifreq structure
 *  @return         0 --success, otherwise fail
 */
static int woal_uap_addba_param(struct net_device *dev, struct ifreq *req)
{
	moal_private *priv = (moal_private *)netdev_priv(dev);
	mlan_ioctl_req *ioctl_req = NULL;
	mlan_ds_11n_cfg *cfg_11n = NULL;
	uap_addba_param param;
	int ret = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();
	memset(&param, 0, sizeof(param));

	/* Sanity check */
	if (req->ifr_data == NULL) {
		PRINTM(MERROR, "uap_addba_param() corrupt data\n");
		ret = -EFAULT;
		goto done;
	}
	if (copy_from_user(&param, req->ifr_data, sizeof(param))) {
		PRINTM(MERROR, "Copy from user failed\n");
		ret = -EFAULT;
		goto done;
	}
	PRINTM(MIOCTL,
	       "addba param: action=%u, timeout=%u, txwinsize=%u, rxwinsize=%u txamsdu=%u rxamsdu=%u\n",
	       param.action, param.timeout, param.txwinsize, param.rxwinsize,
	       param.txamsdu, param.rxamsdu);
	ioctl_req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_11n_cfg));
	if (ioctl_req == NULL) {
		LEAVE();
		return -ENOMEM;
	}
	cfg_11n = (mlan_ds_11n_cfg *)ioctl_req->pbuf;
	cfg_11n->sub_command = MLAN_OID_11N_CFG_ADDBA_PARAM;
	ioctl_req->req_id = MLAN_IOCTL_11N_CFG;

	if (!param.action)
		/* Get addba param from MLAN */
		ioctl_req->action = MLAN_ACT_GET;
	else {
		/* Set addba param in MLAN */
		ioctl_req->action = MLAN_ACT_SET;
		cfg_11n->param.addba_param.timeout = param.timeout;
		cfg_11n->param.addba_param.txwinsize = param.txwinsize;
		cfg_11n->param.addba_param.rxwinsize = param.rxwinsize;
		cfg_11n->param.addba_param.txamsdu = param.txamsdu;
		cfg_11n->param.addba_param.rxamsdu = param.rxamsdu;
	}
	status = woal_request_ioctl(priv, ioctl_req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}
	param.timeout = cfg_11n->param.addba_param.timeout;
	param.txwinsize = cfg_11n->param.addba_param.txwinsize;
	param.rxwinsize = cfg_11n->param.addba_param.rxwinsize;
	param.txamsdu = cfg_11n->param.addba_param.txamsdu;
	param.rxamsdu = cfg_11n->param.addba_param.rxamsdu;

	/* Copy to user */
	if (copy_to_user(req->ifr_data, &param, sizeof(param))) {
		PRINTM(MERROR, "Copy to user failed!\n");
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
 *  @brief uap aggr priority tbl
 *
 *  @param dev      A pointer to net_device structure
 *  @param req      A pointer to ifreq structure
 *  @return         0 --success, otherwise fail
 */
static int woal_uap_aggr_priotbl(struct net_device *dev, struct ifreq *req)
{
	moal_private *priv = (moal_private *)netdev_priv(dev);
	mlan_ioctl_req *ioctl_req = NULL;
	mlan_ds_11n_cfg *cfg_11n = NULL;
	uap_aggr_prio_tbl param;
	int ret = 0;
	int i = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();
	memset(&param, 0, sizeof(param));

	/* Sanity check */
	if (req->ifr_data == NULL) {
		PRINTM(MERROR, "woal_uap_aggr_priotbl() corrupt data\n");
		ret = -EFAULT;
		goto done;
	}
	if (copy_from_user(&param, req->ifr_data, sizeof(param))) {
		PRINTM(MERROR, "Copy from user failed\n");
		ret = -EFAULT;
		goto done;
	}
	DBG_HEXDUMP(MCMD_D, "aggr_prio_tbl", (t_u8 *)&param, sizeof(param));

	ioctl_req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_11n_cfg));
	if (ioctl_req == NULL) {
		LEAVE();
		return -ENOMEM;
	}
	cfg_11n = (mlan_ds_11n_cfg *)ioctl_req->pbuf;
	cfg_11n->sub_command = MLAN_OID_11N_CFG_AGGR_PRIO_TBL;
	ioctl_req->req_id = MLAN_IOCTL_11N_CFG;

	if (!param.action) {
		/* Get aggr_prio_tbl from MLAN */
		ioctl_req->action = MLAN_ACT_GET;
	} else {
		/* Set aggr_prio_tbl in MLAN */
		ioctl_req->action = MLAN_ACT_SET;
		for (i = 0; i < MAX_NUM_TID; i++) {
			cfg_11n->param.aggr_prio_tbl.ampdu[i] = param.ampdu[i];
			cfg_11n->param.aggr_prio_tbl.amsdu[i] = param.amsdu[i];
		}
	}
	status = woal_request_ioctl(priv, ioctl_req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}
	for (i = 0; i < MAX_NUM_TID; i++) {
		param.ampdu[i] = cfg_11n->param.aggr_prio_tbl.ampdu[i];
		param.amsdu[i] = cfg_11n->param.aggr_prio_tbl.amsdu[i];
	}
	/* Copy to user */
	if (copy_to_user(req->ifr_data, &param, sizeof(param))) {
		PRINTM(MERROR, "Copy to user failed!\n");
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
 *  @brief uap addba reject tbl
 *
 *  @param dev      A pointer to net_device structure
 *  @param req      A pointer to ifreq structure
 *  @return         0 --success, otherwise fail
 */
static int woal_uap_addba_reject(struct net_device *dev, struct ifreq *req)
{
	moal_private *priv = (moal_private *)netdev_priv(dev);
	mlan_ioctl_req *ioctl_req = NULL;
	mlan_ds_11n_cfg *cfg_11n = NULL;
	addba_reject_para param;
	int ret = 0;
	int i = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();
	memset(&param, 0, sizeof(param));

	/* Sanity check */
	if (req->ifr_data == NULL) {
		PRINTM(MERROR, "woal_uap_addba_reject() corrupt data\n");
		ret = -EFAULT;
		goto done;
	}
	if (copy_from_user(&param, req->ifr_data, sizeof(param))) {
		PRINTM(MERROR, "Copy from user failed\n");
		ret = -EFAULT;
		goto done;
	}
	DBG_HEXDUMP(MCMD_D, "addba_reject tbl", (t_u8 *)&param, sizeof(param));

	ioctl_req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_11n_cfg));
	if (ioctl_req == NULL) {
		LEAVE();
		return -ENOMEM;
	}
	cfg_11n = (mlan_ds_11n_cfg *)ioctl_req->pbuf;
	cfg_11n->sub_command = MLAN_OID_11N_CFG_ADDBA_REJECT;
	ioctl_req->req_id = MLAN_IOCTL_11N_CFG;

	if (!param.action) {
		/* Get addba_reject tbl from MLAN */
		ioctl_req->action = MLAN_ACT_GET;
	} else {
		/* Set addba_reject tbl in MLAN */
		ioctl_req->action = MLAN_ACT_SET;
		for (i = 0; i < MAX_NUM_TID; i++)
			cfg_11n->param.addba_reject[i] = param.addba_reject[i];
	}
	status = woal_request_ioctl(priv, ioctl_req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}
	for (i = 0; i < MAX_NUM_TID; i++)
		param.addba_reject[i] = cfg_11n->param.addba_reject[i];
	/* Copy to user */
	if (copy_to_user(req->ifr_data, &param, sizeof(param))) {
		PRINTM(MERROR, "Copy to user failed!\n");
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
 *  @brief uap get_fw_info handler
 *
 *  @param dev      A pointer to net_device structure
 *  @param req      A pointer to ifreq structure
 *  @return         0 --success, otherwise fail
 */
static int woal_uap_get_fw_info(struct net_device *dev, struct ifreq *req)
{
	moal_private *priv = (moal_private *)netdev_priv(dev);
	uap_fw_info fw;
	mlan_fw_info fw_info;
	int ret = 0;

	ENTER();
	memset(&fw, 0, sizeof(fw));
	memset(&fw_info, 0, sizeof(fw_info));

	/* Sanity check */
	if (req->ifr_data == NULL) {
		PRINTM(MERROR, "woal_uap_get_fw_info() corrupt data\n");
		ret = -EFAULT;
		goto done;
	}
	if (copy_from_user(&fw, req->ifr_data, sizeof(fw))) {
		PRINTM(MERROR, "Copy from user failed\n");
		ret = -EFAULT;
		goto done;
	}
	if (MLAN_STATUS_SUCCESS !=
	    woal_request_get_fw_info(priv, MOAL_IOCTL_WAIT, &fw_info)) {
		ret = -EFAULT;
		goto done;
	}
	fw.fw_release_number = fw_info.fw_ver;
	fw.hw_dev_mcs_support = fw_info.hw_dev_mcs_support;
	fw.fw_bands = fw_info.fw_bands;
	fw.region_code = fw_info.region_code;
	fw.hw_dot_11n_dev_cap = fw_info.hw_dot_11n_dev_cap;
	/* Copy to user */
	if (copy_to_user(req->ifr_data, &fw, sizeof(fw))) {
		PRINTM(MERROR, "Copy to user failed!\n");
		ret = -EFAULT;
		goto done;
	}
done:
	LEAVE();
	return ret;
}

/**
 *  @brief configure deep sleep
 *
 *  @param dev      A pointer to net_device structure
 *  @param req      A pointer to ifreq structure
 *  @return         0 --success, otherwise fail
 */
static int woal_uap_deep_sleep(struct net_device *dev, struct ifreq *req)
{
	moal_private *priv = (moal_private *)netdev_priv(dev);
	mlan_ioctl_req *ioctl_req = NULL;
	mlan_ds_pm_cfg *pm = NULL;
	deep_sleep_para param;
	int ret = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();
	memset(&param, 0, sizeof(param));

	/* Sanity check */
	if (req->ifr_data == NULL) {
		PRINTM(MERROR, "woal_uap_deep_sleep() corrupt data\n");
		ret = -EFAULT;
		goto done;
	}
	if (copy_from_user(&param, req->ifr_data, sizeof(param))) {
		PRINTM(MERROR, "Copy from user failed\n");
		ret = -EFAULT;
		goto done;
	}
	DBG_HEXDUMP(MCMD_D, "deep_sleep_para", (t_u8 *)&param, sizeof(param));

	ioctl_req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_pm_cfg));
	if (ioctl_req == NULL) {
		LEAVE();
		return -ENOMEM;
	}
	pm = (mlan_ds_pm_cfg *)ioctl_req->pbuf;
	pm->sub_command = MLAN_OID_PM_CFG_DEEP_SLEEP;
	ioctl_req->req_id = MLAN_IOCTL_PM_CFG;

	if (!param.action) {
		/* Get deep_sleep status from MLAN */
		ioctl_req->action = MLAN_ACT_GET;
	} else {
		/* Set deep_sleep in MLAN */
		ioctl_req->action = MLAN_ACT_SET;
		if (param.deep_sleep == MTRUE) {
			pm->param.auto_deep_sleep.auto_ds = DEEP_SLEEP_ON;
			pm->param.auto_deep_sleep.idletime = param.idle_time;
		} else {
			pm->param.auto_deep_sleep.auto_ds = DEEP_SLEEP_OFF;
		}
	}
	status = woal_request_ioctl(priv, ioctl_req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}
	if (pm->param.auto_deep_sleep.auto_ds == DEEP_SLEEP_ON)
		param.deep_sleep = MTRUE;
	else
		param.deep_sleep = MFALSE;
	param.idle_time = pm->param.auto_deep_sleep.idletime;
	/* Copy to user */
	if (copy_to_user(req->ifr_data, &param, sizeof(param))) {
		PRINTM(MERROR, "Copy to user failed!\n");
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
 *  @brief configure band steering
 *
 *  @param dev      A pointer to net_device structure
 *  @param req      A pointer to ifreq structure
 *  @return         0 --success, otherwise fail
 */
static int woal_uap_band_steer(struct net_device *dev, struct ifreq *req)
{
	moal_private *priv = (moal_private *)netdev_priv(dev);
	mlan_ioctl_req *ioctl_req = NULL;
	mlan_ds_misc_cfg *pm = NULL;
	band_steer_para param;
	int ret = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;
	ENTER();
	memset(&param, 0, sizeof(param));

	/* Sanity check */
	if (req->ifr_data == NULL) {
		PRINTM(MERROR, "woal_uap_band_steer() corrupt data\n");
		ret = -EFAULT;
		goto done;
	}
	if (copy_from_user(&param, req->ifr_data, sizeof(param))) {
		PRINTM(MERROR, "Copy from user failed\n");
		ret = -EFAULT;
		goto done;
	}
	DBG_HEXDUMP(MCMD_D, "band_steer_para", (t_u8 *)&param, sizeof(param));

	ioctl_req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (ioctl_req == NULL) {
		LEAVE();
		return -ENOMEM;
	}
	pm = (mlan_ds_misc_cfg *)ioctl_req->pbuf;
	pm->sub_command = MLAN_OID_MISC_BAND_STEERING;
	ioctl_req->req_id = MLAN_IOCTL_MISC_CFG;

	pm->param.band_steer_cfg.action = param.action;
	pm->param.band_steer_cfg.block_2g_prb_req = param.block_2g_prb_req;
	pm->param.band_steer_cfg.state = param.state;
	pm->param.band_steer_cfg.max_btm_req_allowed =
		param.max_btm_req_allowed;

	status = woal_request_ioctl(priv, ioctl_req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	param.action = pm->param.band_steer_cfg.action;
	param.block_2g_prb_req = pm->param.band_steer_cfg.block_2g_prb_req;
	param.state = pm->param.band_steer_cfg.state;
	param.max_btm_req_allowed =
		pm->param.band_steer_cfg.max_btm_req_allowed;

	/* Copy to user */
	if (copy_to_user(req->ifr_data, &param, sizeof(param))) {
		PRINTM(MERROR, "Copy to user failed!\n");
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
 *  @brief configure beacon stuck detect mechanism
 *
 *  @param dev      A pointer to net_device structure
 *  @param req      A pointer to ifreq structure
 *  @return         0 --success, otherwise fail
 */
static int woal_uap_beacon_stuck(struct net_device *dev, struct ifreq *req)
{
	moal_private *priv = (moal_private *)netdev_priv(dev);
	mlan_ioctl_req *ioctl_req = NULL;
	mlan_ds_misc_cfg *pm = NULL;
	beacon_stuck_detect_para param;
	int ret = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	memset(&param, 0, sizeof(param));

	/* Sanity check */
	if (req->ifr_data == NULL) {
		PRINTM(MERROR, "woal_uap_beacon_stuck() corrupt data\n");
		ret = -EFAULT;
		goto done;
	}

	if (copy_from_user(&param, req->ifr_data, sizeof(param))) {
		PRINTM(MERROR, "Copy from user failed\n");
		ret = -EFAULT;
		goto done;
	}

	DBG_HEXDUMP(MCMD_D, "beacon_stuck_detect_para", (t_u8 *)&param,
		    sizeof(param));

	ioctl_req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (ioctl_req == NULL) {
		LEAVE();
		return -ENOMEM;
	}

	pm = (mlan_ds_misc_cfg *)ioctl_req->pbuf;
	pm->sub_command = MLAN_OID_MISC_BEACON_STUCK;
	ioctl_req->req_id = MLAN_IOCTL_MISC_CFG;

	pm->param.beacon_stuck_cfg.action = param.action;
	pm->param.beacon_stuck_cfg.beacon_stuck_detect_count =
		param.beacon_stuck_detect_count;
	pm->param.beacon_stuck_cfg.recovery_confirm_count =
		param.recovery_confirm_count;

	status = woal_request_ioctl(priv, ioctl_req, MOAL_IOCTL_WAIT);

	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	param.action = pm->param.beacon_stuck_cfg.action;
	param.beacon_stuck_detect_count =
		pm->param.beacon_stuck_cfg.beacon_stuck_detect_count;
	param.recovery_confirm_count =
		pm->param.beacon_stuck_cfg.recovery_confirm_count;

	/* Copy to user */
	if (copy_to_user(req->ifr_data, &param, sizeof(param))) {
		PRINTM(MERROR, "Copy to user failed!\n");
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
 *  @brief configure tx_pause settings
 *
 *  @param dev      A pointer to net_device structure
 *  @param req      A pointer to ifreq structure
 *  @return         0 --success, otherwise fail
 */
static int woal_uap_txdatapause(struct net_device *dev, struct ifreq *req)
{
	moal_private *priv = (moal_private *)netdev_priv(dev);
	mlan_ioctl_req *ioctl_req = NULL;
	mlan_ds_misc_cfg *misc = NULL;
	tx_data_pause_para param;
	int ret = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();
	memset(&param, 0, sizeof(param));

	/* Sanity check */
	if (req->ifr_data == NULL) {
		PRINTM(MERROR, "woal_uap_txdatapause corrupt data\n");
		ret = -EFAULT;
		goto done;
	}
	if (copy_from_user(&param, req->ifr_data, sizeof(param))) {
		PRINTM(MERROR, "Copy from user failed\n");
		ret = -EFAULT;
		goto done;
	}
	DBG_HEXDUMP(MCMD_D, "tx_data_pause_para", (t_u8 *)&param,
		    sizeof(param));

	ioctl_req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (ioctl_req == NULL) {
		LEAVE();
		return -ENOMEM;
	}
	misc = (mlan_ds_misc_cfg *)ioctl_req->pbuf;
	misc->sub_command = MLAN_OID_MISC_TX_DATAPAUSE;
	ioctl_req->req_id = MLAN_IOCTL_MISC_CFG;

	if (!param.action) {
		/* Get Tx data pause status from MLAN */
		ioctl_req->action = MLAN_ACT_GET;
	} else {
		/* Set Tx data pause in MLAN */
		ioctl_req->action = MLAN_ACT_SET;
		misc->param.tx_datapause.tx_pause = param.txpause;
		misc->param.tx_datapause.tx_buf_cnt = param.txbufcnt;
	}
	status = woal_request_ioctl(priv, ioctl_req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	param.txpause = misc->param.tx_datapause.tx_pause;
	param.txbufcnt = misc->param.tx_datapause.tx_buf_cnt;

	/* Copy to user */
	if (copy_to_user(req->ifr_data, &param, sizeof(param))) {
		PRINTM(MERROR, "Copy to user failed!\n");
		ret = -EFAULT;
		goto done;
	}
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(ioctl_req);
	LEAVE();
	return ret;
}

#ifdef SDIO
/**
 *  @brief uap sdcmd52rw ioctl handler
 *
 *  @param dev      A pointer to net_device structure
 *  @param req      A pointer to ifreq structure
 *  @return         0 --success, otherwise fail
 */
static int woal_uap_sdcmd52_rw(struct net_device *dev, struct ifreq *req)
{
	moal_private *priv = (moal_private *)netdev_priv(dev);
	sdcmd52_para param;
	t_u8 func, data = 0;
	int ret = 0, reg;

	ENTER();
	memset(&param, 0, sizeof(param));

	/* Sanity check */
	if (req->ifr_data == NULL) {
		PRINTM(MERROR, "woal_uap_sdcmd52_rw() corrupt data\n");
		ret = -EFAULT;
		goto done;
	}
	if (copy_from_user(&param, req->ifr_data, sizeof(param))) {
		PRINTM(MERROR, "Copy from user failed\n");
		ret = -EFAULT;
		goto done;
	}

	func = (t_u8)param.cmd52_params[0];
	reg = (t_u32)param.cmd52_params[1];

	if (!param.action) {
		PRINTM(MINFO, "Cmd52 read, func=%d, reg=0x%08X\n", func, reg);
#ifdef SDIO_MMC
		sdio_claim_host(((sdio_mmc_card *)priv->phandle->card)->func);
		if (func)
			data = sdio_readb(
				((sdio_mmc_card *)priv->phandle->card)->func,
				reg, &ret);
		else
			data = sdio_f0_readb(
				((sdio_mmc_card *)priv->phandle->card)->func,
				reg, &ret);
		sdio_release_host(((sdio_mmc_card *)priv->phandle->card)->func);
		if (ret) {
			PRINTM(MERROR,
			       "sdio_readb: reading register 0x%X failed\n",
			       reg);
			goto done;
		}
#else
		if (sdio_read_ioreg(priv->phandle->card, func, reg, &data) <
		    0) {
			PRINTM(MERROR,
			       "sdio_read_ioreg: reading register 0x%X failed\n",
			       reg);
			ret = MLAN_STATUS_FAILURE;
			goto done;
		}
#endif
		param.cmd52_params[2] = data;
	} else {
		data = (t_u8)param.cmd52_params[2];
		PRINTM(MINFO, "Cmd52 write, func=%d, reg=0x%08X, data=0x%02X\n",
		       func, reg, data);
#ifdef SDIO_MMC
		sdio_claim_host(((sdio_mmc_card *)priv->phandle->card)->func);
		if (func)
			sdio_writeb(
				((sdio_mmc_card *)priv->phandle->card)->func,
				data, reg, &ret);
		else
			sdio_f0_writeb(
				((sdio_mmc_card *)priv->phandle->card)->func,
				data, reg, &ret);
		sdio_release_host(((sdio_mmc_card *)priv->phandle->card)->func);
		if (ret) {
			PRINTM(MERROR,
			       "sdio_writeb: writing register 0x%X failed\n",
			       reg);
			goto done;
		}
#else
		if (sdio_write_ioreg(priv->phandle->card, func, reg, data) <
		    0) {
			PRINTM(MERROR,
			       "sdio_write_ioreg: writing register 0x%X failed\n",
			       reg);
			ret = MLAN_STATUS_FAILURE;
			goto done;
		}
#endif
	}
	/* Copy to user */
	if (copy_to_user(req->ifr_data, &param, sizeof(param))) {
		PRINTM(MERROR, "Copy to user failed!\n");
		ret = -EFAULT;
	}
done:
	LEAVE();
	return ret;
}
#endif

/**
 *  @brief enable/disable 11h
 *
 *  @param enable      MTRUE/MFALSE
 *  @return            0 --success, otherwise fail
 */
int woal_uap_11h_ctrl(moal_private *priv, t_u32 enable)
{
	mlan_ioctl_req *ioctl_req = NULL;
	mlan_ds_snmp_mib *snmp = NULL;
	int ret = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;
	ENTER();
	ioctl_req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_snmp_mib));
	if (ioctl_req == NULL) {
		LEAVE();
		return -ENOMEM;
	}
	snmp = (mlan_ds_snmp_mib *)ioctl_req->pbuf;
	ioctl_req->req_id = MLAN_IOCTL_SNMP_MIB;
	snmp->sub_command = MLAN_OID_SNMP_MIB_DOT11H;
	snmp->param.oid_value = enable;
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
 *  @brief configure snmp mib
 *
 *  @param dev      A pointer to net_device structure
 *  @param req      A pointer to ifreq structure
 *  @return         0 --success, otherwise fail
 */
static int woal_uap_snmp_mib(struct net_device *dev, struct ifreq *req)
{
	moal_private *priv = (moal_private *)netdev_priv(dev);
	mlan_ioctl_req *ioctl_req = NULL;
	mlan_ds_snmp_mib *snmp = NULL;
	snmp_mib_para param;
	t_u8 value[MAX_SNMP_VALUE_SIZE];
	int ret = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();
	memset(&param, 0, sizeof(param));
	memset(value, 0, MAX_SNMP_VALUE_SIZE);

	/* Sanity check */
	if (req->ifr_data == NULL) {
		PRINTM(MERROR, "woal_uap_snmp_mib() corrupt data\n");
		ret = -EFAULT;
		goto done;
	}

	/* Copy from user */
	if (copy_from_user(&param, req->ifr_data, sizeof(param))) {
		PRINTM(MERROR, "Copy from user failed\n");
		ret = -EFAULT;
		goto done;
	}
	DBG_HEXDUMP(MCMD_D, "snmp_mib_para", (t_u8 *)&param, sizeof(param));
	if (param.action) {
		if (copy_from_user(
			    value,
			    (void *)((t_u8 *)(req->ifr_data) + sizeof(param)),
			    MIN(param.oid_val_len, MAX_SNMP_VALUE_SIZE))) {
			PRINTM(MERROR, "Copy from user failed\n");
			ret = -EFAULT;
			goto done;
		}
		DBG_HEXDUMP(MCMD_D, "snmp_mib_para value", value,
			    MIN(param.oid_val_len, sizeof(t_u32)));
	}

	ioctl_req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_snmp_mib));
	if (ioctl_req == NULL) {
		LEAVE();
		return -ENOMEM;
	}
	snmp = (mlan_ds_snmp_mib *)ioctl_req->pbuf;
	ioctl_req->req_id = MLAN_IOCTL_SNMP_MIB;
	switch (param.oid) {
	case OID_80211D_ENABLE:
		snmp->sub_command = MLAN_OID_SNMP_MIB_DOT11D;
		break;
	case OID_80211H_ENABLE:
		snmp->sub_command = MLAN_OID_SNMP_MIB_DOT11H;
		break;
	default:
		ret = -EINVAL;
		PRINTM(MERROR, "%s: Unsupported SNMP_MIB OID (%d).\n", __func__,
		       param.oid);
		goto done;
	}

	if (!param.action) {
		/* Get mib value from MLAN */
		ioctl_req->action = MLAN_ACT_GET;
	} else {
		/* Set mib value to MLAN */
		ioctl_req->action = MLAN_ACT_SET;
		snmp->param.oid_value = *(t_u32 *)value;
	}
	status = woal_request_ioctl(priv, ioctl_req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	/* Copy to user */
	if (copy_to_user(req->ifr_data, &param, sizeof(param))) {
		PRINTM(MERROR, "Copy to user failed!\n");
		ret = -EFAULT;
		goto done;
	}
	if (!param.action) { /* GET */
		if (copy_to_user(
			    (void *)((t_u8 *)(req->ifr_data) + sizeof(param)),
			    &snmp->param.oid_value,
			    MIN(param.oid_val_len, sizeof(t_u32)))) {
			PRINTM(MERROR, "Copy from user failed\n");
			ret = -EFAULT;
			goto done;
		}
	}

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(ioctl_req);
	LEAVE();
	return ret;
}

/**
 *  @brief configure domain info
 *
 *  @param dev      A pointer to net_device structure
 *  @param req      A pointer to ifreq structure
 *  @return         0 --success, otherwise fail
 */
static int woal_uap_domain_info(struct net_device *dev, struct ifreq *req)
{
	moal_private *priv = (moal_private *)netdev_priv(dev);
	mlan_ioctl_req *ioctl_req = NULL;
	mlan_ds_11d_cfg *cfg11d = NULL;
	domain_info_para param;
	t_u8 tlv[MAX_DOMAIN_TLV_LEN + MAX_REG_DOMAIN_TLV_LEN];
	t_u16 tlv_data_len = 0;
	int ret = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();
	memset(&param, 0, sizeof(param));
	memset(tlv, 0, MAX_DOMAIN_TLV_LEN + MAX_REG_DOMAIN_TLV_LEN);

	/* Sanity check */
	if (req->ifr_data == NULL) {
		PRINTM(MERROR, "woal_uap_domain_info() corrupt data\n");
		ret = -EFAULT;
		goto done;
	}

	/* Copy from user */
	if (copy_from_user(&param, req->ifr_data, sizeof(param))) {
		PRINTM(MERROR, "Copy from user failed\n");
		ret = -EFAULT;
		goto done;
	}
	DBG_HEXDUMP(MCMD_D, "domain_info_para", (t_u8 *)&param, sizeof(param));
	if (param.action) {
		/* get tlv header */
		if (copy_from_user(
			    &tlv[0],
			    (void *)((t_u8 *)(req->ifr_data) + sizeof(param)),
			    TLV_HEADER_LEN)) {
			PRINTM(MERROR, "Copy from user failed\n");
			ret = -EFAULT;
			goto done;
		}
		tlv_data_len = ((t_u16 *)(tlv))[1];
		tlv_data_len += MAX_REG_DOMAIN_TLV_LEN;
		if ((TLV_HEADER_LEN + tlv_data_len) > (int)sizeof(tlv)) {
			PRINTM(MERROR, "TLV buffer is overflowed");
			ret = -EINVAL;
			goto done;
		}
		/* get full tlv */
		if (copy_from_user(
			    tlv,
			    (void *)((t_u8 *)(req->ifr_data) + sizeof(param)),
			    TLV_HEADER_LEN + tlv_data_len)) {
			PRINTM(MERROR, "Copy from user failed\n");
			ret = -EFAULT;
			goto done;
		}
		DBG_HEXDUMP(MCMD_D, "domain_info_para tlv", tlv,
			    TLV_HEADER_LEN + tlv_data_len);
	}

	ioctl_req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_11d_cfg));
	if (ioctl_req == NULL) {
		LEAVE();
		return -ENOMEM;
	}
	cfg11d = (mlan_ds_11d_cfg *)ioctl_req->pbuf;
	ioctl_req->req_id = MLAN_IOCTL_11D_CFG;
	cfg11d->sub_command = MLAN_OID_11D_DOMAIN_INFO;

	if (!param.action) {
		/* Get mib value from MLAN */
		ioctl_req->action = MLAN_ACT_GET;
	} else {
		/* Set mib value to MLAN */
		ioctl_req->action = MLAN_ACT_SET;
		moal_memcpy_ext(priv->phandle, cfg11d->param.domain_tlv, tlv,
				TLV_HEADER_LEN + tlv_data_len,
				sizeof(cfg11d->param.domain_tlv));
	}
	status = woal_request_ioctl(priv, ioctl_req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	/* Copy to user */
	if (copy_to_user(req->ifr_data, &param, sizeof(param))) {
		PRINTM(MERROR, "Copy to user failed!\n");
		ret = -EFAULT;
		goto done;
	}
	if (!param.action) { /* GET */
		tlv_data_len = ((t_u16 *)(cfg11d->param.domain_tlv))[1];
		if (copy_to_user(
			    (void *)((t_u8 *)(req->ifr_data) + sizeof(param)),
			    &cfg11d->param.domain_tlv,
			    TLV_HEADER_LEN + tlv_data_len)) {
			PRINTM(MERROR, "Copy from user failed\n");
			ret = -EFAULT;
			goto done;
		}
	}

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(ioctl_req);
	LEAVE();
	return ret;
}

/**
 *  @brief configure dfs testing settings
 *
 *  @param dev      A pointer to net_device structure
 *  @param req      A pointer to ifreq structure
 *  @return         0 --success, otherwise fail
 */
static int woal_uap_dfs_testing(struct net_device *dev, struct ifreq *req)
{
	moal_private *priv = (moal_private *)netdev_priv(dev);
	mlan_ioctl_req *ioctl_req = NULL;
	mlan_ds_11h_cfg *cfg11h = NULL;
	dfs_testing_para param;
	int ret = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();
	memset(&param, 0, sizeof(param));

	/* Sanity check */
	if (req->ifr_data == NULL) {
		PRINTM(MERROR, "woal_uap_dfs_testing() corrupt data\n");
		ret = -EFAULT;
		goto done;
	}

	/* Copy from user */
	if (copy_from_user(&param, req->ifr_data, sizeof(param))) {
		PRINTM(MERROR, "Copy from user failed\n");
		ret = -EFAULT;
		goto done;
	}
	DBG_HEXDUMP(MCMD_D, "dfs_testing_para", (t_u8 *)&param, sizeof(param));

	ioctl_req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_11h_cfg));
	if (ioctl_req == NULL) {
		LEAVE();
		return -ENOMEM;
	}
	cfg11h = (mlan_ds_11h_cfg *)ioctl_req->pbuf;
	ioctl_req->req_id = MLAN_IOCTL_11H_CFG;
	cfg11h->sub_command = MLAN_OID_11H_DFS_TESTING;

	if (!param.action) {
		/* Get mib value from MLAN */
		ioctl_req->action = MLAN_ACT_GET;
	} else {
		/* Set mib value to MLAN */
		ioctl_req->action = MLAN_ACT_SET;
		cfg11h->param.dfs_testing.usr_cac_period_msec =
			param.usr_cac_period * 1000;
		cfg11h->param.dfs_testing.usr_nop_period_sec =
			param.usr_nop_period;
		cfg11h->param.dfs_testing.usr_no_chan_change =
			param.no_chan_change;
		cfg11h->param.dfs_testing.usr_fixed_new_chan =
			param.fixed_new_chan;
		cfg11h->param.dfs_testing.usr_cac_restart = param.cac_restart;
		priv->phandle->cac_restart = param.cac_restart;
		priv->phandle->cac_period_jiffies = param.usr_cac_period * HZ;
		priv->user_cac_period_msec =
			cfg11h->param.dfs_testing.usr_cac_period_msec;
	}
	status = woal_request_ioctl(priv, ioctl_req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	if (!param.action) { /* GET */
		param.usr_cac_period =
			cfg11h->param.dfs_testing.usr_cac_period_msec / 1000;
		param.usr_nop_period =
			cfg11h->param.dfs_testing.usr_nop_period_sec;
		param.no_chan_change =
			cfg11h->param.dfs_testing.usr_no_chan_change;
		param.fixed_new_chan =
			cfg11h->param.dfs_testing.usr_fixed_new_chan;
		param.cac_restart = cfg11h->param.dfs_testing.usr_cac_restart;
	}
	/* Copy to user */
	if (copy_to_user(req->ifr_data, &param, sizeof(param))) {
		PRINTM(MERROR, "Copy to user failed!\n");
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
 *  @brief uap channel NOP status check ioctl handler
 *
 *  @param priv             A pointer to moal_private structure
 *  @param wait_option      Wait option
 *  @param data             BSS control type
 *  @return                 0 --success, otherwise fail
 */
int woal_uap_get_channel_nop_info(moal_private *priv, t_u8 wait_option,
				  mlan_ds_11h_chan_nop_info *ch_info)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_11h_cfg *ds_11hcfg = NULL;

	int ret = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	if (!ch_info) {
		PRINTM(MERROR, "Invalid chan_info\n");
		LEAVE();
		return -EFAULT;
	}
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_11h_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	req->req_id = MLAN_IOCTL_11H_CFG;
	req->action = MLAN_ACT_GET;

	ds_11hcfg = (mlan_ds_11h_cfg *)req->pbuf;
	ds_11hcfg->sub_command = MLAN_OID_11H_CHAN_NOP_INFO;
	moal_memcpy_ext(priv->phandle, &ds_11hcfg->param.ch_nop_info, ch_info,
			sizeof(mlan_ds_11h_chan_nop_info),
			sizeof(ds_11hcfg->param.ch_nop_info));
	status = woal_request_ioctl(priv, req, wait_option);
	if (status == MLAN_STATUS_FAILURE) {
		ret = -EFAULT;
		goto done;
	}
	moal_memcpy_ext(priv->phandle, ch_info, &ds_11hcfg->param.ch_nop_info,
			sizeof(mlan_ds_11h_chan_nop_info),
			sizeof(mlan_ds_11h_chan_nop_info));

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief configure channel switch count
 *
 *  @param dev      A pointer to net_device structure
 *  @param req      A pointer to ifreq structure
 *  @return         0 --success, otherwise fail
 */
static int woal_uap_chan_switch_count_cfg(struct net_device *dev,
					  struct ifreq *req)
{
	moal_private *priv = (moal_private *)netdev_priv(dev);
	mlan_ioctl_req *ioctl_req = NULL;
	mlan_ds_11h_cfg *cfg11h = NULL;
	cscount_cfg_t param;
	int ret = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();
	memset(&param, 0, sizeof(param));

	/* Sanity check */
	if (req->ifr_data == NULL) {
		PRINTM(MERROR, "%s corrupt data\n", __func__);
		ret = -EFAULT;
		goto done;
	}

	/* Copy from user */
	if (copy_from_user(&param, req->ifr_data, sizeof(param))) {
		PRINTM(MERROR, "Copy from user failed\n");
		ret = -EFAULT;
		goto done;
	}
	DBG_HEXDUMP(MCMD_D, "cscount_cfg_t", (t_u8 *)&param, sizeof(param));

	ioctl_req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_11h_cfg));
	if (ioctl_req == NULL) {
		LEAVE();
		return -ENOMEM;
	}
	cfg11h = (mlan_ds_11h_cfg *)ioctl_req->pbuf;
	ioctl_req->req_id = MLAN_IOCTL_11H_CFG;
	cfg11h->sub_command = MLAN_OID_11H_CHAN_SWITCH_COUNT;

	if (!param.action) {
		/* Get mib value from MLAN */
		ioctl_req->action = MLAN_ACT_GET;
	} else {
		/* Set mib value to MLAN */
		ioctl_req->action = MLAN_ACT_SET;
		cfg11h->param.cs_count = param.cs_count;
	}
	status = woal_request_ioctl(priv, ioctl_req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	if (!param.action) { /* GET */
		param.cs_count = cfg11h->param.cs_count;
	}
	/* Copy to user */
	if (copy_to_user(req->ifr_data, &param, sizeof(param))) {
		PRINTM(MERROR, "Copy to user failed!\n");
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
 *  @brief Configure TX beamforming support
 *
 *  @param dev      A pointer to net_device structure
 *  @param req      A pointer to ifreq structure
 *  @return         0 --success, otherwise fail
 */
static int woal_uap_tx_bf_cfg(struct net_device *dev, struct ifreq *req)
{
	int ret = 0;
	moal_private *priv = (moal_private *)netdev_priv(dev);
	mlan_ds_11n_tx_bf_cfg bf_cfg;
	tx_bf_cfg_para_hdr param;
	t_u16 action = 0;

	ENTER();

	memset(&param, 0, sizeof(param));
	memset(&bf_cfg, 0, sizeof(bf_cfg));

	/* Sanity check */
	if (req->ifr_data == NULL) {
		PRINTM(MERROR, "woal_uap_tx_bf_cfg corrupt data\n");
		ret = -EFAULT;
		goto done;
	}
	if (copy_from_user(&param, req->ifr_data, sizeof(param))) {
		PRINTM(MERROR, "Copy from user failed\n");
		ret = -EFAULT;
		goto done;
	}
	if (!param.action)
		/* Get BF configurations */
		action = MLAN_ACT_GET;
	else
		/* Set BF configurations */
		action = MLAN_ACT_SET;
	if (copy_from_user(&bf_cfg,
			   (void *)((t_u8 *)(req->ifr_data) +
				    sizeof(tx_bf_cfg_para_hdr)),
			   sizeof(bf_cfg))) {
		PRINTM(MERROR, "Copy from user failed\n");
		ret = -EFAULT;
		goto done;
	}
	DBG_HEXDUMP(MCMD_D, "bf_cfg", (t_u8 *)&bf_cfg, sizeof(bf_cfg));

	if (MLAN_STATUS_SUCCESS !=
	    woal_set_get_tx_bf_cfg(priv, action, &bf_cfg)) {
		ret = -EFAULT;
		goto done;
	}

	/* Copy to user */
	if (copy_to_user((void *)((t_u8 *)(req->ifr_data) +
				  sizeof(tx_bf_cfg_para_hdr)),
			 &bf_cfg, sizeof(bf_cfg))) {
		PRINTM(MERROR, "Copy to user failed!\n");
		ret = -EFAULT;
		goto done;
	}

done:
	LEAVE();
	return ret;
}

/**
 *  @brief Set/Get 11n configurations
 *
 *  @param dev      A pointer to net_device structure
 *  @param req      A pointer to ifreq structure
 *  @return         0 --success, otherwise fail
 */
static int woal_uap_ht_tx_cfg(struct net_device *dev, struct ifreq *req)
{
	int ret = 0;
	moal_private *priv = (moal_private *)netdev_priv(dev);
	mlan_ds_11n_cfg *cfg_11n = NULL;
	mlan_ds_11n_tx_cfg httx_cfg;
	mlan_ioctl_req *ioctl_req = NULL;
	ht_tx_cfg_para_hdr param;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	memset(&param, 0, sizeof(ht_tx_cfg_para_hdr));
	memset(&httx_cfg, 0, sizeof(mlan_ds_11n_tx_cfg));

	/* Sanity check */
	if (req->ifr_data == NULL) {
		PRINTM(MERROR, "woal_uap_ht_tx_cfg corrupt data\n");
		ret = -EFAULT;
		goto done;
	}
	if (copy_from_user(&param, req->ifr_data, sizeof(ht_tx_cfg_para_hdr))) {
		PRINTM(MERROR, "Copy from user failed\n");
		ret = -EFAULT;
		goto done;
	}
	ioctl_req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_11n_cfg));
	if (ioctl_req == NULL) {
		ret = -ENOMEM;
		goto done;
	}
	cfg_11n = (mlan_ds_11n_cfg *)ioctl_req->pbuf;
	cfg_11n->sub_command = MLAN_OID_11N_CFG_TX;
	ioctl_req->req_id = MLAN_IOCTL_11N_CFG;
	if (copy_from_user(&httx_cfg,
			   (void *)((t_u8 *)(req->ifr_data) +
				    sizeof(ht_tx_cfg_para_hdr)),
			   sizeof(mlan_ds_11n_tx_cfg))) {
		PRINTM(MERROR, "Copy from user failed\n");
		ret = -EFAULT;
		goto done;
	}
	if (!param.action) {
		/* Get 11n tx parameters from MLAN */
		ioctl_req->action = MLAN_ACT_GET;
	} else {
		/* Set HT Tx configurations */
		cfg_11n->param.tx_cfg.httxcap = httx_cfg.httxcap;
		PRINTM(MINFO, "SET: httxcap:0x%x\n", httx_cfg.httxcap);
		/* Update 11n tx parameters in MLAN */
		ioctl_req->action = MLAN_ACT_SET;
	}
	status = woal_request_ioctl(priv, ioctl_req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}
	if (ioctl_req->action == MLAN_ACT_GET) {
		httx_cfg.httxcap = cfg_11n->param.tx_cfg.httxcap;
		PRINTM(MINFO, "GET: httxcap:0x%x\n", httx_cfg.httxcap);
	}
	/* Copy to user */
	if (copy_to_user((void *)((t_u8 *)(req->ifr_data) +
				  sizeof(ht_tx_cfg_para_hdr)),
			 &httx_cfg, sizeof(mlan_ds_11n_tx_cfg))) {
		PRINTM(MERROR, "Copy to user failed!\n");
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
 *  @brief Set/Get Set/Get 11AC configurations
 *
 *  @param dev      A pointer to net_device structure
 *  @param req      A pointer to ifreq structure
 *  @return         0 --success, otherwise fail
 */
static int woal_uap_vht_cfg(struct net_device *dev, struct ifreq *req)
{
	moal_private *priv = (moal_private *)netdev_priv(dev);
	int ret = 0, resbuf_len = 0;
	mlan_ds_11ac_cfg *cfg_11ac = NULL;
	mlan_ioctl_req *ioctl_req = NULL;
	mlan_ds_11ac_vht_cfg *vhtcfg = NULL, vht_cfg;
	t_u8 *respbuf = NULL;
	vht_cfg_para_hdr param;
	mlan_status status = MLAN_STATUS_SUCCESS;
#define CMD_RESPBUF_LEN 2048
	gfp_t flag;

	ENTER();

	memset(&param, 0, sizeof(vht_cfg_para_hdr));

	flag = (in_atomic() || irqs_disabled()) ? GFP_ATOMIC : GFP_KERNEL;
	respbuf = kzalloc(CMD_RESPBUF_LEN, flag);
	if (!respbuf) {
		ret = -ENOMEM;
		goto done;
	}

	/* Sanity check */
	if (req->ifr_data == NULL) {
		PRINTM(MERROR, "woal_uap_ht_tx_cfg corrupt data\n");
		ret = -EFAULT;
		goto done;
	}
	if (copy_from_user(&param, req->ifr_data, sizeof(vht_cfg_para_hdr))) {
		PRINTM(MERROR, "Copy from user failed\n");
		ret = -EFAULT;
		goto done;
	}

	ioctl_req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_11ac_cfg));
	if (ioctl_req == NULL) {
		ret = -ENOMEM;
		goto done;
	}
	cfg_11ac = (mlan_ds_11ac_cfg *)ioctl_req->pbuf;
	cfg_11ac->sub_command = MLAN_OID_11AC_VHT_CFG;
	ioctl_req->req_id = MLAN_IOCTL_11AC_CFG;
	if (copy_from_user(&vht_cfg,
			   (void *)((t_u8 *)(req->ifr_data) +
				    sizeof(vht_cfg_para_hdr)),
			   sizeof(mlan_ds_11ac_vht_cfg))) {
		PRINTM(MERROR, "Copy from user failed\n");
		ret = -EFAULT;
		goto done;
	}
	if (vht_cfg.band == BAND_SELECT_BOTH) {
		cfg_11ac->param.vht_cfg.band = (BAND_SELECT_BG | BAND_SELECT_A);
	} else {
		cfg_11ac->param.vht_cfg.band = vht_cfg.band;
	}
	if (!param.action) {
		/* GET operation */
		if (vht_cfg.band == BAND_SELECT_BOTH) {
			/* if get both bands, get BG first */
			cfg_11ac->param.vht_cfg.band = BAND_SELECT_BG;
		}
		PRINTM(MINFO, "GET: vhtcfg band: 0x%x\n",
		       cfg_11ac->param.vht_cfg.band);
		if (priv->bss_role == MLAN_BSS_ROLE_UAP)
			cfg_11ac->param.vht_cfg.txrx = MLAN_RADIO_RX;
		else
			cfg_11ac->param.vht_cfg.txrx = vht_cfg.txrx;
		PRINTM(MINFO, "GET: vhtcfg txrx: 0x%x\n",
		       cfg_11ac->param.vht_cfg.txrx);
		ioctl_req->action = MLAN_ACT_GET;
	} else {
		/* Band */
		if (vht_cfg.band == BAND_SELECT_BOTH)
			cfg_11ac->param.vht_cfg.band =
				(BAND_SELECT_BG | BAND_SELECT_A);
		else
			cfg_11ac->param.vht_cfg.band = vht_cfg.band;
		PRINTM(MINFO, "SET: vhtcfg band: 0x%x\n",
		       cfg_11ac->param.vht_cfg.band);
		/* Tx/Rx */
		cfg_11ac->param.vht_cfg.txrx = vht_cfg.txrx;
		PRINTM(MINFO, "SET: vhtcfg txrx: 0x%x\n",
		       cfg_11ac->param.vht_cfg.txrx);
		/* BW cfg */
		cfg_11ac->param.vht_cfg.bwcfg = vht_cfg.bwcfg;
		PRINTM(MINFO, "SET: vhtcfg bw cfg:0x%x\n",
		       cfg_11ac->param.vht_cfg.bwcfg);

		cfg_11ac->param.vht_cfg.vht_cap_info = vht_cfg.vht_cap_info;
		PRINTM(MINFO, "SET: vhtcfg vht_cap_info:0x%x\n",
		       cfg_11ac->param.vht_cfg.vht_cap_info);
		cfg_11ac->param.vht_cfg.vht_tx_mcs = vht_cfg.vht_tx_mcs;
		cfg_11ac->param.vht_cfg.vht_rx_mcs = vht_cfg.vht_rx_mcs;
		/* Update 11AC parameters in MLAN */
		ioctl_req->action = MLAN_ACT_SET;
	}
	status = woal_request_ioctl(priv, ioctl_req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	/* number of vhtcfg entries */
	*respbuf = 1;
	vhtcfg = (mlan_ds_11ac_vht_cfg *)(respbuf + 1);
	moal_memcpy_ext(priv->phandle, vhtcfg, &cfg_11ac->param.vht_cfg,
			sizeof(mlan_ds_11ac_vht_cfg),
			sizeof(mlan_ds_11ac_vht_cfg));
	resbuf_len = 1 + sizeof(mlan_ds_11ac_vht_cfg);

	if ((ioctl_req->action == MLAN_ACT_GET) &&
	    (vht_cfg.band == BAND_SELECT_BOTH)) {
		cfg_11ac->param.vht_cfg.band = BAND_SELECT_A;
		status = woal_request_ioctl(priv, ioctl_req, MOAL_IOCTL_WAIT);
		if (status != MLAN_STATUS_SUCCESS) {
			ret = -EFAULT;
			goto done;
		}
		/* number of vhtcfg entries */
		*respbuf = 2;
		vhtcfg++;
		moal_memcpy_ext(priv->phandle, vhtcfg, &cfg_11ac->param.vht_cfg,
				sizeof(mlan_ds_11ac_vht_cfg),
				sizeof(mlan_ds_11ac_vht_cfg));
		resbuf_len += sizeof(mlan_ds_11ac_vht_cfg);
	}
	if (ioctl_req->action == MLAN_ACT_GET) {
		if (copy_to_user(req->ifr_data, respbuf, resbuf_len)) {
			PRINTM(MERROR, "Copy to user failed\n");
			ret = -EFAULT;
		}
	}
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(ioctl_req);
	if (respbuf)
		kfree(respbuf);
	LEAVE();
	return ret;
}

/**
 *  @brief uap hs_cfg ioctl handler
 *
 *  @param dev      A pointer to net_device structure
 *  @param req      A pointer to ifreq structure
 *  @return         0 --success, otherwise fail
 */
static int woal_uap_hs_cfg(struct net_device *dev, struct ifreq *req,
			   BOOLEAN invoke_hostcmd)
{
	moal_private *priv = (moal_private *)netdev_priv(dev);
	mlan_ds_hs_cfg hscfg;
	ds_hs_cfg hs_cfg;
	mlan_bss_info bss_info;
	t_u16 action;
	int ret = 0;

	ENTER();

	memset(&hscfg, 0, sizeof(mlan_ds_hs_cfg));
	memset(&hs_cfg, 0, sizeof(ds_hs_cfg));

	/* Sanity check */
	if (req->ifr_data == NULL) {
		PRINTM(MERROR, "uap_hs_cfg() corrupt data\n");
		ret = -EFAULT;
		goto done;
	}
	if (copy_from_user(&hs_cfg, req->ifr_data, sizeof(ds_hs_cfg))) {
		PRINTM(MERROR, "Copy from user failed\n");
		ret = -EFAULT;
		goto done;
	}

	PRINTM(MIOCTL,
	       "ioctl hscfg: flags=0x%x condition=0x%x gpio=%u gap=0x%x\n",
	       hs_cfg.flags, hs_cfg.conditions, hs_cfg.gpio, hs_cfg.gap);

	/* HS config is blocked if HS is already activated */
	if ((hs_cfg.flags & HS_CFG_FLAG_CONDITION) &&
	    (hs_cfg.conditions != HOST_SLEEP_CFG_CANCEL ||
	     invoke_hostcmd == MFALSE)) {
		memset(&bss_info, 0, sizeof(bss_info));
		if (MLAN_STATUS_SUCCESS !=
		    woal_get_bss_info(priv, MOAL_IOCTL_WAIT, &bss_info)) {
			PRINTM(MERROR, "ERR: failed in getting bss info\n");
			ret = -EFAULT;
			goto done;
		}
		if (bss_info.is_hs_configured) {
			PRINTM(MERROR, "HS already configured\n");
			ret = -EFAULT;
			goto done;
		}
	}

	if (hs_cfg.flags & HS_CFG_FLAG_SET) {
		action = MLAN_ACT_SET;
		if (hs_cfg.flags != HS_CFG_FLAG_ALL) {
			if (MLAN_STATUS_SUCCESS !=
			    woal_set_get_hs_params(priv, MLAN_ACT_GET,
						   MOAL_IOCTL_WAIT, &hscfg)) {
				PRINTM(MERROR,
				       "Unable to get HS Configuration\n");
			}
		}
		if (hs_cfg.flags & HS_CFG_FLAG_CONDITION)
			hscfg.conditions = hs_cfg.conditions;
		if (hs_cfg.flags & HS_CFG_FLAG_GPIO)
			hscfg.gpio = hs_cfg.gpio;
		if (hs_cfg.flags & HS_CFG_FLAG_GAP)
			hscfg.gap = hs_cfg.gap;

		if (invoke_hostcmd == MTRUE) {
			/* Issue IOCTL to set up parameters */
			hscfg.is_invoke_hostcmd = MFALSE;
			if (MLAN_STATUS_SUCCESS !=
			    woal_set_get_hs_params(priv, action,
						   MOAL_IOCTL_WAIT, &hscfg)) {
				ret = -EFAULT;
				goto done;
			}
		}
	} else {
		action = MLAN_ACT_GET;
	}

	/* Issue IOCTL to invoke hostcmd */
	hscfg.is_invoke_hostcmd = invoke_hostcmd;
	if (MLAN_STATUS_SUCCESS !=
	    woal_set_get_hs_params(priv, action, MOAL_IOCTL_WAIT, &hscfg)) {
		ret = -EFAULT;
		goto done;
	}
	if (!(hs_cfg.flags & HS_CFG_FLAG_SET)) {
		hs_cfg.flags = HS_CFG_FLAG_CONDITION | HS_CFG_FLAG_GPIO |
			       HS_CFG_FLAG_GAP;
		hs_cfg.conditions = hscfg.conditions;
		hs_cfg.gpio = hscfg.gpio;
		hs_cfg.gap = hscfg.gap;
		/* Copy to user */
		if (copy_to_user(req->ifr_data, &hs_cfg, sizeof(ds_hs_cfg))) {
			PRINTM(MERROR, "Copy to user failed!\n");
			ret = -EFAULT;
			goto done;
		}
	}

done:
	LEAVE();
	return ret;
}

/**
 *  @brief Set Host Sleep parameters
 *
 *  @param priv         A pointer to moal_private structure
 *  @param wrq          A pointer to iwreq structure
 *
 *  @return             0 --success, otherwise fail
 */
static int woal_uap_hs_set_para(struct net_device *dev, struct ifreq *req)
{
	int ret = 0;

	ENTER();

	if (req->ifr_data != NULL) {
		ret = woal_uap_hs_cfg(dev, req, MFALSE);
		goto done;
	} else {
		PRINTM(MERROR, "Invalid data\n");
		ret = -EINVAL;
		goto done;
	}
done:
	LEAVE();
	return ret;
}

/**
 *  @brief uap mgmt_frame_control ioctl handler
 *
 *  @param dev      A pointer to net_device structure
 *  @param req      A pointer to ifreq structure
 *  @return         0 --success, otherwise fail
 */
static int woal_uap_mgmt_frame_control(struct net_device *dev,
				       struct ifreq *req)
{
	moal_private *priv = (moal_private *)netdev_priv(dev);
	int ret = 0;
	t_u16 action = 0;
	mgmt_frame_ctrl param;
	mlan_uap_bss_param *sys_config = NULL;

	ENTER();

	/* Sanity check */
	if (req->ifr_data == NULL) {
		PRINTM(MERROR, "uap_mgmt_frame_ctrl() corrupt data\n");
		ret = -EFAULT;
		goto done;
	}

	/* Get user data */
	if (copy_from_user(&param, req->ifr_data, sizeof(param))) {
		PRINTM(MERROR, "Copy from user failed\n");
		ret = -EFAULT;
		goto done;
	}
	sys_config = kzalloc(sizeof(mlan_uap_bss_param), GFP_ATOMIC);
	if (!sys_config) {
		PRINTM(MERROR, "Fail to alloc memory for mlan_uap_bss_param\n");
		ret = -EFAULT;
		goto done;
	}

	if (param.action)
		action = MLAN_ACT_SET;
	else
		action = MLAN_ACT_GET;
	if (action == MLAN_ACT_SET) {
		/* Initialize the invalid values so that the correct
		   values below are downloaded to firmware */
		woal_set_sys_config_invalid_data(sys_config);
		sys_config->mgmt_ie_passthru_mask = param.mask;
	}

	if (MLAN_STATUS_SUCCESS != woal_set_get_sys_config(priv, action,
							   MOAL_IOCTL_WAIT,
							   sys_config)) {
		ret = -EFAULT;
		goto done;
	}

	if (action == MLAN_ACT_GET) {
		param.mask = sys_config->mgmt_ie_passthru_mask;
		if (copy_to_user(req->ifr_data, &param, sizeof(param))) {
			PRINTM(MERROR, "Copy to user failed\n");
			ret = -EFAULT;
		}
	}
done:
	kfree(sys_config);
	LEAVE();
	return ret;
}

/**
 * @brief Set/Get tx rate
 *
 *  @param dev      A pointer to net_device structure
 *  @param req      A pointer to ifreq structure
 *
 * @return           0 --success, otherwise fail
 */
static int woal_uap_tx_rate_cfg(struct net_device *dev, struct ifreq *req)
{
	moal_private *priv = (moal_private *)netdev_priv(dev);
	int ret = 0, i = 0;
	mlan_ds_rate *rate = NULL;
	mlan_ioctl_req *mreq = NULL;
	tx_rate_cfg_t tx_rate_config;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	/* Sanity check */
	if (req->ifr_data == NULL) {
		PRINTM(MERROR, "uap_tx_rate_cfg() corrupt data\n");
		ret = -EFAULT;
		goto done;
	}

	memset(&tx_rate_config, 0, sizeof(tx_rate_cfg_t));
	/* Get user data */
	if (copy_from_user(&tx_rate_config, req->ifr_data,
			   sizeof(tx_rate_cfg_t))) {
		PRINTM(MERROR, "Copy from user failed\n");
		ret = -EFAULT;
		goto done;
	}

	mreq = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_rate));
	if (mreq == NULL) {
		ret = -ENOMEM;
		goto done;
	}
	rate = (mlan_ds_rate *)mreq->pbuf;
	rate->param.rate_cfg.rate_type = MLAN_RATE_INDEX;
	rate->sub_command = MLAN_OID_RATE_CFG;
	mreq->req_id = MLAN_IOCTL_RATE;

	rate->auto_null_fixrate_enable = 0xFF;

	if (!(tx_rate_config.action))
		mreq->action = MLAN_ACT_GET;
	else {
		if ((tx_rate_config.user_data_cnt <= 0) ||
		    (tx_rate_config.user_data_cnt > 5)) {
			PRINTM(MERROR, "Invalid user_data_cnt\n");
			ret = -EINVAL;
			goto done;
		}

		mreq->action = MLAN_ACT_SET;
		if (tx_rate_config.rate_format == AUTO_RATE)
			rate->param.rate_cfg.is_rate_auto = 1;
		else {
			if ((tx_rate_config.rate_format < 0) ||
			    (tx_rate_config.rate < 0)) {
				PRINTM(MERROR,
				       "Invalid format or rate selection\n");
				ret = -EINVAL;
				goto done;
			}
			/* rate_format sanity check */
			if ((tx_rate_config.rate_format >
			     MLAN_RATE_FORMAT_HE)) {
				PRINTM(MERROR, "Invalid format selection\n");
				ret = -EINVAL;
				goto done;
			}
			rate->param.rate_cfg.rate_format =
				tx_rate_config.rate_format;

			/* rate sanity check */
			if (tx_rate_config.user_data_cnt >= 2) {
				if (((tx_rate_config.rate_format ==
				      MLAN_RATE_FORMAT_LG) &&
				     (tx_rate_config.rate >
				      MLAN_RATE_INDEX_OFDM7)) ||
				    ((tx_rate_config.rate_format ==
				      MLAN_RATE_FORMAT_HT) &&
				     (tx_rate_config.rate != 32) &&
				     (tx_rate_config.rate > 15)) ||
				    ((tx_rate_config.rate_format ==
				      MLAN_RATE_FORMAT_VHT) &&
				     (tx_rate_config.rate >
				      MLAN_RATE_INDEX_MCS9)) ||
				    ((tx_rate_config.rate_format ==
				      MLAN_RATE_FORMAT_HE) &&
				     (tx_rate_config.rate >
				      MLAN_RATE_INDEX_MCS11))) {
					PRINTM(MERROR,
					       "Invalid rate selection\n");
					ret = -EINVAL;
					goto done;
				}
				rate->param.rate_cfg.rate = tx_rate_config.rate;
			}

			/* nss sanity check */
			if ((tx_rate_config.rate_format == 2) ||
			    (tx_rate_config.rate_format == 3)) {
				if ((tx_rate_config.nss < 1) ||
				    (tx_rate_config.nss > 2)) {
					PRINTM(MERROR,
					       "Invalid nss selection %d\n",
					       tx_rate_config.nss);
					ret = -EINVAL;
					goto done;
				}
				rate->param.rate_cfg.nss = tx_rate_config.nss;
			}
			if (tx_rate_config.user_data_cnt <= 3)
				rate->param.rate_cfg.rate_setting = 0xffff;
			else
				rate->param.rate_cfg.rate_setting =
					tx_rate_config.rate_setting;

			if (tx_rate_config.user_data_cnt == 5) {
				if (tx_rate_config.auto_null_fixrate_enable >
				    1) {
					ret = -EINVAL;
					goto done;
				}

				rate->auto_null_fixrate_enable =
					tx_rate_config.auto_null_fixrate_enable;
			}
		}
	}

	status = woal_request_ioctl(priv, mreq, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}
	if (tx_rate_config.action) {
		priv->rate_index = tx_rate_config.action;
	} else {
		if (rate->param.rate_cfg.is_rate_auto)
			tx_rate_config.rate_format = AUTO_RATE;
		else {
			/* fixed rate */
			tx_rate_config.rate_format =
				rate->param.rate_cfg.rate_format;
			tx_rate_config.rate = rate->param.rate_cfg.rate;
			if (rate->param.rate_cfg.rate_format ==
				    MLAN_RATE_FORMAT_VHT ||
			    rate->param.rate_cfg.rate_format ==
				    MLAN_RATE_FORMAT_HE)
				tx_rate_config.nss = rate->param.rate_cfg.nss;
			tx_rate_config.rate_setting =
				rate->param.rate_cfg.rate_setting;
		}
		for (i = 0; i < MAX_BITMAP_RATES_SIZE; i++) {
			tx_rate_config.bitmap_rates[i] =
				rate->param.rate_cfg.bitmap_rates[i];
		}

		if (copy_to_user(req->ifr_data, &tx_rate_config,
				 sizeof(tx_rate_cfg_t))) {
			PRINTM(MERROR, "Copy to user failed\n");
			ret = -EFAULT;
		}
	}
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(mreq);
	LEAVE();
	return ret;
}

/**
 * @brief Set/Get RF antenna mode
 *
 *  @param dev      A pointer to net_device structure
 *  @param req      A pointer to ifreq structure
 *
 * @return           0 --success, otherwise fail
 */
static int woal_uap_antenna_cfg(struct net_device *dev, struct ifreq *req)
{
	moal_private *priv = (moal_private *)netdev_priv(dev);
	int ret = 0;
	mlan_ds_radio_cfg *radio = NULL;
	mlan_ioctl_req *mreq = NULL;
	ant_cfg_t antenna_config;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	/* Sanity check */
	if (req->ifr_data == NULL) {
		PRINTM(MERROR, "uap_antenna_cfg() corrupt data\n");
		ret = -EFAULT;
		goto done;
	}

	memset(&antenna_config, 0, sizeof(ant_cfg_t));
	/* Get user data */
	if (copy_from_user(&antenna_config, req->ifr_data, sizeof(ant_cfg_t))) {
		PRINTM(MERROR, "Copy from user failed\n");
		ret = -EFAULT;
		goto done;
	}
	mreq = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_radio_cfg));
	if (mreq == NULL) {
		ret = -ENOMEM;
		goto done;
	}
	radio = (mlan_ds_radio_cfg *)mreq->pbuf;
	radio->sub_command = MLAN_OID_ANT_CFG;
	mreq->req_id = MLAN_IOCTL_RADIO_CFG;
	if (!(antenna_config.action))
		mreq->action = MLAN_ACT_GET;
	else {
		mreq->action = MLAN_ACT_SET;
		radio->param.ant_cfg.tx_antenna = antenna_config.tx_mode;
		radio->param.ant_cfg.rx_antenna = antenna_config.rx_mode;
		radio->param.ant_cfg.tx_antenna_6g = antenna_config.tx_mode_6g;
		radio->param.ant_cfg.rx_antenna_6g = antenna_config.rx_mode_6g;
	}

	status = woal_request_ioctl(priv, mreq, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		PRINTM(MERROR, "Failed to set new antenna config\n");
		ret = -EFAULT;
		goto done;
	}

	/* Notify the CFG80211 layer only on SUCCESS from FW */
	if ((status == MLAN_STATUS_SUCCESS) && (mreq->action == MLAN_ACT_SET)) {
#if defined(STA_CFG80211) || defined(UAP_CFG80211)
		if (IS_CARD9098(priv->phandle->card_type) ||
		    IS_CARD9097(priv->phandle->card_type) ||
		    IS_CARDIW624(priv->phandle->card_type) ||
		    IS_CARDAW693(priv->phandle->card_type)) {
			if (IS_STA_CFG80211(
				    priv->phandle->params.cfg80211_wext))
				woal_cfg80211_notify_antcfg(
					priv, priv->phandle->wiphy, radio);
		}
#endif
	}

	if (mreq->action == MLAN_ACT_GET) {
		antenna_config.tx_mode = radio->param.ant_cfg.tx_antenna;
		antenna_config.rx_mode = radio->param.ant_cfg.rx_antenna;
		antenna_config.tx_mode_6g = radio->param.ant_cfg.tx_antenna_6g;
		antenna_config.rx_mode_6g = radio->param.ant_cfg.rx_antenna_6g;
		if (copy_to_user(req->ifr_data, &antenna_config,
				 sizeof(ant_cfg_t))) {
			PRINTM(MERROR, "Copy to user failed\n");
			ret = -EFAULT;
		}
	}
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(mreq);
	LEAVE();
	return ret;
}

/**
 * @brief Set/Get HT stream configurations
 *
 *  @param dev      A pointer to net_device structure
 *  @param req      A pointer to ifreq structure
 *
 * @return           0 --success, otherwise fail
 */
static int woal_uap_htstream_cfg(struct net_device *dev, struct ifreq *req)
{
	moal_private *priv = (moal_private *)netdev_priv(dev);
	int ret = 0;
	mlan_ds_11n_cfg *cfg = NULL;
	mlan_ioctl_req *ioctl_req = NULL;
	htstream_cfg_t htstream_cfg;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	memset(&htstream_cfg, 0, sizeof(htstream_cfg_t));

	/* Sanity check */
	if (req->ifr_data == NULL) {
		PRINTM(MERROR, "woal_uap_htstream_cfg corrupt data\n");
		ret = -EFAULT;
		goto done;
	}
	if (copy_from_user(&htstream_cfg, req->ifr_data,
			   sizeof(htstream_cfg_t))) {
		PRINTM(MERROR, "Copy from user failed\n");
		ret = -EFAULT;
		goto done;
	}
	ioctl_req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_11n_cfg));
	if (ioctl_req == NULL) {
		ret = -ENOMEM;
		goto done;
	}
	cfg = (mlan_ds_11n_cfg *)ioctl_req->pbuf;
	cfg->sub_command = MLAN_OID_11N_CFG_STREAM_CFG;
	ioctl_req->req_id = MLAN_IOCTL_11N_CFG;

	if (!htstream_cfg.action) {
		/* Get operation */
		ioctl_req->action = MLAN_ACT_GET;
	} else {
		/* Update HT stream parameter in MLAN */
		ioctl_req->action = MLAN_ACT_SET;
		/* Set HT Stream configuration */
		cfg->param.stream_cfg = htstream_cfg.stream_cfg;
		PRINTM(MINFO, "SET: htstream_cfg:0x%x\n",
		       cfg->param.stream_cfg);
	}
	status = woal_request_ioctl(priv, ioctl_req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}
	/* Copy to user */
	if (ioctl_req->action == MLAN_ACT_GET) {
		PRINTM(MINFO, "GET: htstream_cfg:0x%x\n",
		       htstream_cfg.stream_cfg);
		htstream_cfg.stream_cfg = cfg->param.stream_cfg;
		if (copy_to_user(req->ifr_data, &htstream_cfg,
				 sizeof(htstream_cfg_t))) {
			PRINTM(MERROR, "Copy to user failed!\n");
			ret = -EFAULT;
			goto done;
		}
	}
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(ioctl_req);
	LEAVE();
	return ret;
}

/**
 * @brief Get DFS_REPEATER mode
 *
 *  @param dev      A pointer to net_device structure
 *  @param req      A pointer to ifreq structure
 *
 * @return           0 --success, otherwise fail
 */
static int woal_uap_dfs_repeater(struct net_device *dev, struct ifreq *req)
{
	moal_private *priv = (moal_private *)netdev_priv(dev);
	int ret = 0;
	dfs_repeater_mode param;
	mlan_ds_misc_cfg *misc = NULL;
	mlan_ioctl_req *mreq = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	/* Sanity check */
	if (req->ifr_data == NULL) {
		PRINTM(MERROR, "uap_antenna_cfg() corrupt data\n");
		ret = -EFAULT;
		goto done;
	}

	memset(&param, 0, sizeof(dfs_repeater_mode));
	/* Get user data */
	if (copy_from_user(&param, req->ifr_data, sizeof(dfs_repeater_mode))) {
		PRINTM(MERROR, "Copy from user failed\n");
		ret = -EFAULT;
		goto done;
	}
	mreq = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (mreq == NULL) {
		ret = -ENOMEM;
		goto done;
	}
	misc = (mlan_ds_misc_cfg *)mreq->pbuf;
	misc->sub_command = MLAN_OID_MISC_DFS_REAPTER_MODE;
	mreq->req_id = MLAN_IOCTL_MISC_CFG;
	mreq->action = MLAN_ACT_GET;

	status = woal_request_ioctl(priv, mreq, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}
	param.mode = misc->param.dfs_repeater.mode;

	if (copy_to_user(req->ifr_data, &param, sizeof(dfs_repeater_mode))) {
		PRINTM(MERROR, "Copy to user failed\n");
		ret = -EFAULT;
	}
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(mreq);
	LEAVE();
	return ret;
}

/**
 *  @brief Issue MLAN_OID_11H_CHAN_REPORT_REQUEST ioctl to cancel dozer
 *
 *  @param priv     Pointer to the moal_private driver data struct
 *  @param action   MLAN_ACT_SET/MLAN_ACT_GET
 *  @param
 *
 *  @return         0 --success, otherwise fail
 */
int woal_11h_chan_dfs_state(moal_private *priv, t_u8 action,
			    mlan_ds_11h_chan_dfs_state *ch_dfs_state)
{
	int ret = 0;
	mlan_ioctl_req *req = NULL;
	mlan_ds_11h_cfg *ds_11hcfg = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_11h_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}
	ds_11hcfg = (mlan_ds_11h_cfg *)req->pbuf;

	ds_11hcfg->sub_command = MLAN_OID_11H_CHAN_DFS_STATE;
	req->req_id = MLAN_IOCTL_11H_CFG;
	req->action = action;
	moal_memcpy_ext(priv->phandle, &ds_11hcfg->param.ch_dfs_state,
			ch_dfs_state, sizeof(mlan_ds_11h_chan_dfs_state),
			sizeof(ds_11hcfg->param.ch_dfs_state));
	/* Send Channel Check command and wait until the report is ready */
	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}
	moal_memcpy_ext(priv->phandle, ch_dfs_state,
			&ds_11hcfg->param.ch_dfs_state,
			sizeof(mlan_ds_11h_chan_dfs_state),
			sizeof(mlan_ds_11h_chan_dfs_state));
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief find all bonded channel.
 *
 *  @param pri_chan   primary channel
 *  @param bw         channel bandwidth
 *  @param ch_dfs_state  a pointer to mlan_ds_11h_chan_dfs_state array
 *
 *  @return           number of channel
 */
static int woal_uap_get_dfs_chan(t_u8 pri_chan, t_u8 bw,
				 mlan_ds_11h_chan_dfs_state *ch_dfs_state)
{
	int ht40_plus[] = {52, 60, 100, 108, 116, 124, 132, 140};
	int ht40_minus[] = {56, 64, 104, 112, 120, 128, 136, 144};
	int vht80_dfs[4][4] = {{52, 56, 60, 64},
			       {100, 104, 108, 112},
			       {116, 120, 124, 128},
			       {132, 136, 140, 144}};
	t_u8 find = false;
	int j;
	int i;
	t_u8 sec_chan = 0;
	mlan_ds_11h_chan_dfs_state *pos = ch_dfs_state;
	t_u8 n_chan = 1;

	if (bw == CHAN_BW_20MHZ) {
		pos->channel = pri_chan;
	} else if (bw == CHAN_BW_40MHZ) {
		pos->channel = pri_chan;
		pos++;
		for (i = 0; i < (sizeof(ht40_minus) / sizeof(int)); i++) {
			if (pri_chan == (t_u8)ht40_plus[i]) {
				sec_chan = pri_chan + 4;
				n_chan = 2;
				break;
			}
		}
		for (i = 0; i < (sizeof(ht40_minus) / sizeof(int)); i++) {
			if (pri_chan == (t_u8)ht40_minus[i]) {
				sec_chan = pri_chan - 4;
				n_chan = 2;
				break;
			}
		}
		pos->channel = sec_chan;
	} else if (bw == CHAN_BW_80MHZ) {
		for (i = 0; i < 4; i++) {
			for (j = 0; j < 4; j++) {
				if (pri_chan == (t_u8)vht80_dfs[i][j]) {
					find = true;
					break;
				}
			}
			if (find)
				break;
		}
		if (find) {
			n_chan = 4;
			for (j = 0; j < n_chan; j++) {
				pos->channel = (t_u8)vht80_dfs[i][j];
				pos++;
			}
		}
	}
	return n_chan;
}

#ifdef UAP_CFG80211
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 14, 0)
/**
 * @brief update channel dfs state in mlan module
 *
 * @param channel         given radar channel
 * @param dfs_state       dfs_state
 *
 * @return                N/A
 */
static void woal_set_channel_dfs_state(t_u8 channel, t_u8 dfs_state)
{
	int index;
	mlan_ds_11h_chan_dfs_state ch_dfs_state;
	moal_private *priv;
	memset(&ch_dfs_state, 0, sizeof(ch_dfs_state));
	ch_dfs_state.channel = channel;
	ch_dfs_state.dfs_state = dfs_state;
	for (index = 0; index < MAX_MLAN_ADAPTER; index++) {
		if (m_handle[index]) {
			priv = woal_get_priv(m_handle[index],
					     MLAN_BSS_ROLE_UAP);
			if (priv) {
				if (woal_11h_chan_dfs_state(priv, MLAN_ACT_SET,
							    &ch_dfs_state))
					PRINTM(MERROR,
					       "Set DFS state for chan:%d failed\n",
					       ch_dfs_state.channel);
			}
		}
	}
}

/**
 * @brief update channel dfs state to all wiphy
 *
 * @param priv     Pointer to the moal_private driver data struct
 * @param channel         given radar channel
 * @param bandwidth       channel's bandwidth
 * @param dfs_state       dfs_state
 *
 * @return                N/A
 */
void woal_update_channels_dfs_state(moal_private *priv, t_u8 channel,
				    t_u8 bandwidth, t_u8 dfs_state)
{
	mlan_ds_11h_chan_dfs_state ch_dfs_state[4];
	int cfg80211_wext = priv->phandle->params.cfg80211_wext;
	t_u8 n_chan;
	int i;
	ENTER();
	memset(ch_dfs_state, 0, sizeof(ch_dfs_state));
	n_chan = woal_uap_get_dfs_chan(channel, bandwidth, &ch_dfs_state[0]);
	if (IS_UAP_CFG80211(cfg80211_wext)) {
		for (i = 0; i < n_chan; i++) {
			woal_update_channel_dfs_state(ch_dfs_state[i].channel,
						      dfs_state);
		}
	}
	for (i = 0; i < n_chan; i++) {
		woal_set_channel_dfs_state(ch_dfs_state[i].channel, dfs_state);
	}
	LEAVE();
	return;
}

/**
 * @brief reset uap channel dfs_state to DFS_USABLE
 *
 * @param priv     Pointer to the moal_private driver data struct
 *
 * @return                N/A
 */
void woal_update_uap_channel_dfs_state(moal_private *priv)
{
	mlan_ds_11h_chan_dfs_state ch_dfs_state;
	t_u8 channel;
	t_u8 bandwidth;
	ENTER();
	if (woal_is_etsi_country(priv->phandle->country_code)) {
		LEAVE();
		return;
	}
	if (priv->bss_role == MLAN_BSS_ROLE_UAP && priv->bss_started &&
	    priv->uap_host_based) {
		channel = priv->chan.chan->hw_value;
		memset(&ch_dfs_state, 0, sizeof(ch_dfs_state));
		ch_dfs_state.channel = channel;
		if (woal_11h_chan_dfs_state(priv, MLAN_ACT_GET,
					    &ch_dfs_state)) {
			PRINTM(MERROR, "%s: woal_11h_chan_dfs_state failed \n",
			       __func__);
			LEAVE();
			return;
		}

		if (ch_dfs_state.dfs_required &&
		    ch_dfs_state.dfs_state == DFS_AVAILABLE) {
			switch (priv->chan.width) {
			case NL80211_CHAN_WIDTH_40:
				bandwidth = CHAN_BW_40MHZ;
				break;
			case NL80211_CHAN_WIDTH_80:
				bandwidth = CHAN_BW_80MHZ;
				break;
			default:
				bandwidth = CHAN_BW_20MHZ;
				break;
			}
			woal_update_channels_dfs_state(priv, channel, bandwidth,
						       DFS_USABLE);
		}
	}
	LEAVE();
}
#endif
#endif

/**
 * @brief skip cac on specific channel
 * @and Wext
 *
 *  @param dev      A pointer to net_device structure
 *  @param req      A pointer to ifreq structure
 *
 * @return           0 --success, otherwise fail
 */
static int woal_uap_skip_cac(struct net_device *dev, struct ifreq *req)
{
	int ret = 0;
	skip_cac_para param;
	moal_private *priv = (moal_private *)netdev_priv(dev);
#ifdef UAP_CFG80211
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 14, 0)
	int cfg80211_wext = priv->phandle->params.cfg80211_wext;
#endif
#endif
	dfs_state_t dfs_state;
	mlan_ds_11h_chan_dfs_state ch_dfs_state[4];
	t_u8 n_chan;
	int i = 0;
	ENTER();

	/* Sanity check */
	if (req->ifr_data == NULL) {
		PRINTM(MERROR, "skip_dfs_cac() corrupt data\n");
		ret = -EFAULT;
		goto done;
	}
	memset(&param, 0, sizeof(skip_cac_para));
	/* Get user data */
	if (copy_from_user(&param, req->ifr_data, sizeof(skip_cac_para))) {
		PRINTM(MERROR, "Copy from user failed\n");
		ret = -EFAULT;
		goto done;
	}
	if (param.skip_cac)
		dfs_state = DFS_AVAILABLE;
	else
		dfs_state = DFS_USABLE;
	memset(&ch_dfs_state, 0, sizeof(ch_dfs_state));
	n_chan = woal_uap_get_dfs_chan(param.channel, param.bw,
				       &ch_dfs_state[0]);
	for (i = 0; i < n_chan; i++) {
		if (woal_11h_chan_dfs_state(priv, MLAN_ACT_GET,
					    &ch_dfs_state[i]))
			PRINTM(MERROR, "Get DFS state for chan:%d failed\n",
			       ch_dfs_state[i].channel);
	}
	for (i = 0; i < n_chan; i++) {
		if (param.skip_cac && ch_dfs_state[i].dfs_state == DFS_USABLE)
			PRINTM(MMSG,
			       "DFS: Requst skip cac on the channel %d which hasn't do CAC before!\n",
			       ch_dfs_state[i].channel);
		ch_dfs_state[i].dfs_state = dfs_state;
		if (woal_11h_chan_dfs_state(priv, MLAN_ACT_SET,
					    &ch_dfs_state[i]))
			PRINTM(MERROR, "Set DFS state for chan:%d failed\n",
			       ch_dfs_state[i].channel);
		else
			PRINTM(MCMND, "DFS: Skip CAC on chan %d %d\n",
			       ch_dfs_state[i].channel, param.skip_cac);
	}
#ifdef UAP_CFG80211
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 14, 0)
	if (IS_UAP_CFG80211(cfg80211_wext)) {
		for (i = 0; i < n_chan; i++) {
			if (param.skip_cac)
				woal_update_channel_dfs_state(
					ch_dfs_state[i].channel, DFS_AVAILABLE);
			else
				woal_update_channel_dfs_state(
					ch_dfs_state[i].channel, DFS_USABLE);
		}
	}
#endif
#endif
done:
	LEAVE();
	return ret;
}

/**
 * @brief Get DFS_REPEATER mode
 *
 *  @param dev      A pointer to net_device structure
 *  @param req      A pointer to ifreq structure
 *
 * @return           0 --success, otherwise fail
 */
static int woal_uap_cac_timer_status(struct net_device *dev, struct ifreq *req)
{
	moal_private *priv = (moal_private *)netdev_priv(dev);
	int ret = 0;
	cac_timer_status param;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	/* Sanity check */
	if (req->ifr_data == NULL) {
		PRINTM(MERROR, "uap_antenna_cfg() corrupt data\n");
		ret = -EFAULT;
		goto done;
	}

	memset(&param, 0, sizeof(cac_timer_status));

	/* Get user data */
	if (copy_from_user(&param, req->ifr_data, sizeof(cac_timer_status))) {
		PRINTM(MERROR, "Copy from user failed\n");
		ret = -EFAULT;
		goto done;
	}

	/* Currently default action is get */
	param.mode = 0;

	if (priv->phandle->cac_period == MTRUE) {
		long cac_left_jiffies;

		cac_left_jiffies =
			MEAS_REPORT_TIME -
			(jiffies - priv->phandle->meas_start_jiffies);

		/* cac_left_jiffies would be negative if timer has already
		 * elapsed. positive if timer is still yet to lapsed
		 */
		if (cac_left_jiffies > 0)
			param.mode = (t_u32)cac_left_jiffies / HZ;
	}

	if (copy_to_user(req->ifr_data, &param, sizeof(cac_timer_status))) {
		PRINTM(MERROR, "Copy to user failed\n");
		ret = -EFAULT;
	}
done:
	if (status != MLAN_STATUS_PENDING)
		LEAVE();
	return ret;
}

/**
 * @brief set/get uap operation control
 *
 *  @param dev      A pointer to net_device structure
 *  @param req      A pointer to ifreq structure
 *
 * @return           0 --success, otherwise fail
 */
static int woal_uap_operation_ctrl(struct net_device *dev, struct ifreq *req)
{
	int ret = 0;
	moal_private *priv = (moal_private *)netdev_priv(dev);
	mlan_uap_oper_ctrl uap_oper;
	uap_oper_para_hdr param;
	mlan_ds_bss *bss = NULL;
	mlan_ioctl_req *ioctl_req = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	memset(&param, 0, sizeof(uap_oper_para_hdr));
	memset(&uap_oper, 0, sizeof(mlan_uap_oper_ctrl));

	/* Sanity check */
	if (req->ifr_data == NULL) {
		PRINTM(MERROR, "woal_uap_operation_ctrl corrupt data\n");
		ret = -EFAULT;
		goto done;
	}
	if (copy_from_user(&param, req->ifr_data, sizeof(uap_oper_para_hdr))) {
		PRINTM(MERROR, "Copy from user failed\n");
		ret = -EFAULT;
		goto done;
	}
	ioctl_req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_bss));
	if (ioctl_req == NULL) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	bss = (mlan_ds_bss *)ioctl_req->pbuf;
	bss->sub_command = MLAN_OID_UAP_OPER_CTRL;
	ioctl_req->req_id = MLAN_IOCTL_BSS;
	if (copy_from_user(&uap_oper,
			   (void *)((t_u8 *)(req->ifr_data) +
				    sizeof(uap_oper_para_hdr)),
			   sizeof(mlan_uap_oper_ctrl))) {
		PRINTM(MERROR, "Copy from user failed\n");
		ret = -EFAULT;
		goto done;
	}
	if (!param.action) {
		/* Get uap operation control parameters from FW */
		ioctl_req->action = MLAN_ACT_GET;
	} else {
		/* Set uap operation control configurations */
		ioctl_req->action = MLAN_ACT_SET;
		bss->param.ap_oper_ctrl.ctrl_value = uap_oper.ctrl_value;
		if (uap_oper.ctrl_value == 2)
			bss->param.ap_oper_ctrl.chan_opt = uap_oper.chan_opt;
		if (uap_oper.chan_opt == 3) {
			bss->param.ap_oper_ctrl.band_cfg = uap_oper.band_cfg;
			bss->param.ap_oper_ctrl.channel = uap_oper.channel;
		}
	}
	status = woal_request_ioctl(priv, ioctl_req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	/* Copy to user */
	if (copy_to_user((void *)((t_u8 *)(req->ifr_data) +
				  sizeof(uap_oper_para_hdr)),
			 &bss->param.ap_oper_ctrl,
			 sizeof(mlan_uap_oper_ctrl))) {
		PRINTM(MERROR, "Copy to user failed!\n");
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
 * @brief Set/Get wacp_mode
 *
 *  @param dev      A pointer to net_device structure
 *  @param req      A pointer to ifreq structure
 *
 * @return           0 --success, otherwise fail
 */
static int woal_uap_wacp_mode(struct net_device *dev, struct ifreq *req)
{
	moal_private *priv = (moal_private *)netdev_priv(dev);
	mlan_ioctl_req *ioctl_req = NULL;
	mlan_ds_misc_cfg *pcfg_misc = NULL;
	wacp_mode_para param;
	int ret = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	/* Sanity check */
	if (req->ifr_data == NULL) {
		PRINTM(MERROR, "woal_uap_wacp_mode() corrupt data\n");
		ret = -EFAULT;
		goto done;
	}
	memset(&param, 0, sizeof(wacp_mode_para));
	/* Get user data */
	if (copy_from_user(&param, req->ifr_data, sizeof(wacp_mode_para))) {
		PRINTM(MERROR, "Copy from user failed\n");
		ret = -EFAULT;
		goto done;
	}

	/* Allocate an IOCTL request buffer */
	ioctl_req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (ioctl_req == NULL) {
		status = MLAN_STATUS_FAILURE;
		goto done;
	}

	/* Fill request buffer */
	pcfg_misc = (mlan_ds_misc_cfg *)ioctl_req->pbuf;
	pcfg_misc->sub_command = MLAN_OID_MISC_WACP_MODE;
	ioctl_req->req_id = MLAN_IOCTL_MISC_CFG;
	ioctl_req->action = param.action;
	pcfg_misc->param.wacp_mode = param.wacp_mode;

	/* Send IOCTL request to MLAN */
	status = woal_request_ioctl(priv, ioctl_req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	param.wacp_mode = pcfg_misc->param.wacp_mode;
	/** Update the moal wacp_mode */
	if (param.action == MLAN_ACT_SET) {
		priv->phandle->params.wacp_mode = param.wacp_mode;
	}

	/* Copy to user */
	if (copy_to_user(req->ifr_data, &param, sizeof(param))) {
		PRINTM(MERROR, "Copy to user failed!\n");
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
 *  @brief uap ioctl handler
 *
 *  @param dev      A pointer to net_device structure
 *  @param req      A pointer to ifreq structure
 *  @return         0 --success, otherwise fail
 */
static int woal_uap_ioctl(struct net_device *dev, struct ifreq *req)
{
	int ret = 0;
	t_u32 subcmd = 0;
	ENTER();
	/* Sanity check */
	if (req->ifr_data == NULL) {
		PRINTM(MERROR, "uap_ioctl() corrupt data\n");
		ret = -EFAULT;
		goto done;
	}
	if (copy_from_user(&subcmd, req->ifr_data, sizeof(subcmd))) {
		PRINTM(MERROR, "Copy from user failed\n");
		ret = -EFAULT;
		goto done;
	}

	PRINTM(MIOCTL, "ioctl subcmd=%u\n", subcmd);
	switch (subcmd) {
	case UAP_ADDBA_PARA:
		ret = woal_uap_addba_param(dev, req);
		break;
	case UAP_AGGR_PRIOTBL:
		ret = woal_uap_aggr_priotbl(dev, req);
		break;
	case UAP_ADDBA_REJECT:
		ret = woal_uap_addba_reject(dev, req);
		break;
	case UAP_FW_INFO:
		ret = woal_uap_get_fw_info(dev, req);
		break;
	case UAP_DEEP_SLEEP:
		ret = woal_uap_deep_sleep(dev, req);
		break;
	case UAP_TX_DATA_PAUSE:
		ret = woal_uap_txdatapause(dev, req);
		break;
#ifdef SDIO
	case UAP_SDCMD52_RW:
		ret = woal_uap_sdcmd52_rw(dev, req);
		break;
#endif
	case UAP_SNMP_MIB:
		ret = woal_uap_snmp_mib(dev, req);
		break;
	case UAP_DFS_TESTING:
		ret = woal_uap_dfs_testing(dev, req);
		break;
	case UAP_CHAN_SWITCH_COUNT_CFG:
		ret = woal_uap_chan_switch_count_cfg(dev, req);
		break;
	case UAP_DOMAIN_INFO:
		ret = woal_uap_domain_info(dev, req);
		break;
	case UAP_TX_BF_CFG:
		ret = woal_uap_tx_bf_cfg(dev, req);
		break;
	case UAP_HT_TX_CFG:
		ret = woal_uap_ht_tx_cfg(dev, req);
		break;
	case UAP_VHT_CFG:
		ret = woal_uap_vht_cfg(dev, req);
		break;
	case UAP_HS_CFG:
		ret = woal_uap_hs_cfg(dev, req, MTRUE);
		break;
	case UAP_HS_SET_PARA:
		ret = woal_uap_hs_set_para(dev, req);
		break;
	case UAP_MGMT_FRAME_CONTROL:
		ret = woal_uap_mgmt_frame_control(dev, req);
		break;
	case UAP_TX_RATE_CFG:
		ret = woal_uap_tx_rate_cfg(dev, req);
		break;
	case UAP_ANTENNA_CFG:
		ret = woal_uap_antenna_cfg(dev, req);
		break;
	case UAP_HT_STREAM_CFG:
		ret = woal_uap_htstream_cfg(dev, req);
		break;
	case UAP_DFS_REPEATER_MODE:
		ret = woal_uap_dfs_repeater(dev, req);
		break;
	case UAP_CAC_TIMER_STATUS:
		ret = woal_uap_cac_timer_status(dev, req);
		break;
	case UAP_SKIP_CAC:
		ret = woal_uap_skip_cac(dev, req);
		break;
	case UAP_OPERATION_CTRL:
		ret = woal_uap_operation_ctrl(dev, req);
		break;
	case UAP_BAND_STEER:
		ret = woal_uap_band_steer(dev, req);
		break;
	case UAP_BEACON_STUCK_DETECT:
		ret = woal_uap_beacon_stuck(dev, req);
		break;
	case UAP_WACP_MODE:
		woal_uap_wacp_mode(dev, req);
		break;
	default:
		break;
	}
done:
	LEAVE();
	return ret;
}

/**
 *  @brief uap station deauth ioctl handler
 *
 *  @param dev      A pointer to net_device structure
 *  @param req      A pointer to ifreq structure
 *  @return         0 --success, otherwise fail
 */
static int woal_uap_sta_deauth_ioctl(struct net_device *dev, struct ifreq *req)
{
	moal_private *priv = (moal_private *)netdev_priv(dev);
	mlan_ioctl_req *ioctl_req = NULL;
	mlan_ds_bss *bss = NULL;
	mlan_deauth_param deauth_param;
	int ret = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	memset(&deauth_param, 0, sizeof(mlan_deauth_param));
	/* Sanity check */
	if (req->ifr_data == NULL) {
		PRINTM(MERROR, "uap_sta_deauth_ioctl() corrupt data\n");
		ret = -EFAULT;
		goto done;
	}
	if (copy_from_user(&deauth_param, req->ifr_data,
			   sizeof(mlan_deauth_param))) {
		PRINTM(MERROR, "Copy from user failed\n");
		ret = -EFAULT;
		goto done;
	}

	PRINTM(MIOCTL, "ioctl deauth station: " MACSTR ", reason=%d\n",
	       MAC2STR(deauth_param.mac_addr), deauth_param.reason_code);

	ioctl_req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_bss));
	if (ioctl_req == NULL) {
		ret = -ENOMEM;
		goto done;
	}
	bss = (mlan_ds_bss *)ioctl_req->pbuf;

	bss->sub_command = MLAN_OID_UAP_DEAUTH_STA;
	ioctl_req->req_id = MLAN_IOCTL_BSS;
	ioctl_req->action = MLAN_ACT_SET;

	moal_memcpy_ext(priv->phandle, &bss->param.deauth_param, &deauth_param,
			sizeof(mlan_deauth_param),
			sizeof(bss->param.deauth_param));
	status = woal_request_ioctl(priv, ioctl_req, MOAL_IOCTL_WAIT);
	if ((status != MLAN_STATUS_SUCCESS) &&
	    (status != MLAN_STATUS_PENDING)) {
		ret = -EFAULT;
		if (copy_to_user(req->ifr_data, &ioctl_req->status_code,
				 sizeof(t_u32)))
			PRINTM(MERROR, "Copy to user failed!\n");
		goto done;
	}

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(ioctl_req);
	LEAVE();
	return ret;
}

/**
 * @brief Set/Get radio
 *
 *  @param dev      A pointer to net_device structure
 *  @param req      A pointer to ifreq structure
 *
 * @return           0 --success, otherwise fail
 */
static int woal_uap_radio_ctl(struct net_device *dev, struct ifreq *req)
{
	moal_private *priv = (moal_private *)netdev_priv(dev);
	int ret = 0;
	mlan_ds_radio_cfg *radio = NULL;
	mlan_ioctl_req *mreq = NULL;
	int data[2] = {0, 0};
	mlan_bss_info bss_info;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	/* Sanity check */
	if (req->ifr_data == NULL) {
		PRINTM(MERROR, "uap_radio_ctl() corrupt data\n");
		ret = -EFAULT;
		goto done;
	}

	/* Get user data */
	if (copy_from_user(&data, req->ifr_data, sizeof(data))) {
		PRINTM(MERROR, "Copy from user failed\n");
		ret = -EFAULT;
		goto done;
	}

	if (data[0]) {
		mreq = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_radio_cfg));
		if (mreq == NULL) {
			ret = -ENOMEM;
			goto done;
		}
		radio = (mlan_ds_radio_cfg *)mreq->pbuf;
		radio->sub_command = MLAN_OID_RADIO_CTRL;
		mreq->req_id = MLAN_IOCTL_RADIO_CFG;
		mreq->action = MLAN_ACT_SET;
		radio->param.radio_on_off = (t_u32)data[1];
		status = woal_request_ioctl(priv, mreq, MOAL_IOCTL_WAIT);
		if (status != MLAN_STATUS_SUCCESS)
			ret = -EFAULT;
		if (status != MLAN_STATUS_PENDING)
			kfree(mreq);
	} else {
		/* Get radio status */
		memset(&bss_info, 0, sizeof(bss_info));
		if (MLAN_STATUS_SUCCESS !=
		    woal_get_bss_info(priv, MOAL_IOCTL_WAIT, &bss_info)) {
			PRINTM(MERROR, "ERR: failed in getting bss info\n");
			ret = -EFAULT;
			goto done;
		}

		data[1] = bss_info.radio_on;
		if (copy_to_user(req->ifr_data, data, sizeof(data))) {
			PRINTM(MERROR, "Copy to user failed\n");
			ret = -EFAULT;
		}
	}
done:
	LEAVE();
	return ret;
}

/**
 *  @brief uap bss control ioctl handler
 *
 *  @param dev      A pointer to net_device structure
 *  @param req      A pointer to ifreq structure
 *  @return         0 --success, otherwise fail
 */
static int woal_uap_bss_ctrl_ioctl(struct net_device *dev, struct ifreq *req)
{
	moal_private *priv = (moal_private *)netdev_priv(dev);
	int ret = 0, data = 0;

	ENTER();

	/* Sanity check */
	if (req->ifr_data == NULL) {
		PRINTM(MERROR, "uap_bss_ctrl() corrupt data\n");
		ret = -EFAULT;
		goto done;
	}
	if (copy_from_user(&data, req->ifr_data, sizeof(data))) {
		PRINTM(MERROR, "Copy from user failed\n");
		ret = -EFAULT;
		goto done;
	}

	ret = woal_uap_bss_ctrl(priv, MOAL_IOCTL_WAIT, data);

done:
	LEAVE();
	return ret;
}

/**
 *  @brief uap report mic error ioctl handler
 *
 *  @param dev      A pointer to net_device structure
 *  @param req      A pointer to ifreq structure
 *  @return         0 --success, otherwise fail
 */
static int woal_uap_report_mic_ioctl(struct net_device *dev, struct ifreq *req)
{
	moal_private *priv = (moal_private *)netdev_priv(dev);
	mlan_ioctl_req *ioctl_req = NULL;
	mlan_ds_sec_cfg *sec = NULL;
	t_u8 mac_addr[MLAN_MAC_ADDR_LENGTH];
	int ret = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	memset(mac_addr, 0, MLAN_MAC_ADDR_LENGTH);
	/* Sanity check */
	if (req->ifr_data == NULL) {
		PRINTM(MERROR, "uap_report_mic_ioctl() corrupt data\n");
		ret = -EFAULT;
		goto done;
	}
	if (copy_from_user(mac_addr, req->ifr_data, MLAN_MAC_ADDR_LENGTH)) {
		PRINTM(MERROR, "Copy from user failed\n");
		ret = -EFAULT;
		goto done;
	}

	PRINTM(MINFO, "ioctl report mic err station: " MACSTR "\n",
	       MAC2STR(mac_addr));

	ioctl_req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_sec_cfg));
	if (ioctl_req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	sec = (mlan_ds_sec_cfg *)ioctl_req->pbuf;
	sec->sub_command = MLAN_OID_SEC_CFG_REPORT_MIC_ERR;
	ioctl_req->req_id = MLAN_IOCTL_SEC_CFG;
	ioctl_req->action = MLAN_ACT_SET;
	moal_memcpy_ext(priv->phandle, sec->param.sta_mac, mac_addr,
			MLAN_MAC_ADDR_LENGTH, sizeof(sec->param.sta_mac));

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
 *  @brief uap set key ioctl handler
 *
 *  @param dev      A pointer to net_device structure
 *  @param req      A pointer to ifreq structure
 *  @return         0 --success, otherwise fail
 */
static int woal_uap_set_key_ioctl(struct net_device *dev, struct ifreq *req)
{
	moal_private *priv = (moal_private *)netdev_priv(dev);
	mlan_ioctl_req *ioctl_req = NULL;
	mlan_ds_sec_cfg *sec = NULL;
	encrypt_key key;
	int ret = 0;
	t_u8 bcast_addr[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	memset(&key, 0, sizeof(encrypt_key));
	/* Sanity check */
	if (req->ifr_data == NULL) {
		PRINTM(MERROR, "uap_set_key_ioctl() corrupt data\n");
		ret = -EFAULT;
		goto done;
	}
	if (copy_from_user(&key, req->ifr_data, sizeof(encrypt_key))) {
		PRINTM(MERROR, "Copy from user failed\n");
		ret = -EFAULT;
		goto done;
	}

	PRINTM(MIOCTL,
	       "ioctl report set key: " MACSTR " key_index=%u, key_len=%u \n",
	       MAC2STR(key.mac_addr), key.key_index, key.key_len);

	if ((key.key_len > MLAN_MAX_KEY_LENGTH) || (key.key_index > 3)) {
		ret = -EINVAL;
		goto done;
	}

	ioctl_req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_sec_cfg));
	if (ioctl_req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	sec = (mlan_ds_sec_cfg *)ioctl_req->pbuf;
	sec->sub_command = MLAN_OID_SEC_CFG_ENCRYPT_KEY,
	ioctl_req->req_id = MLAN_IOCTL_SEC_CFG;
	ioctl_req->action = MLAN_ACT_SET;

	moal_memcpy_ext(priv->phandle, sec->param.encrypt_key.mac_addr,
			key.mac_addr, MLAN_MAC_ADDR_LENGTH,
			sizeof(sec->param.encrypt_key.mac_addr));
	sec->param.encrypt_key.key_index = key.key_index;
	sec->param.encrypt_key.key_len = key.key_len;
	moal_memcpy_ext(priv->phandle, sec->param.encrypt_key.key_material,
			key.key_material, key.key_len,
			sizeof(sec->param.encrypt_key.key_material));
	if (0 == memcmp(sec->param.encrypt_key.mac_addr, bcast_addr, ETH_ALEN))
		sec->param.encrypt_key.key_flags = KEY_FLAG_GROUP_KEY;
	else
		sec->param.encrypt_key.key_flags = KEY_FLAG_SET_TX_KEY;

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
 *  @brief Set/Get uap power mode
 *
 *  @param priv                 A pointer to moal_private structure
 *  @param action               Action set or get
 *  @param ps_mgmt              A pointer to mlan_ds_ps_mgmt structure
 *
 *  @return                     MLAN_STATUS_SUCCESS -- success, otherwise fail
 */
mlan_status woal_set_get_uap_power_mode(moal_private *priv, t_u32 action,
					mlan_ds_ps_mgmt *ps_mgmt)
{
	mlan_ioctl_req *ioctl_req = NULL;
	mlan_ds_pm_cfg *pm_cfg = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();
	if (!ps_mgmt) {
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}

	ioctl_req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_pm_cfg));
	if (ioctl_req == NULL) {
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}
	pm_cfg = (mlan_ds_pm_cfg *)ioctl_req->pbuf;
	pm_cfg->sub_command = MLAN_OID_PM_CFG_PS_MODE;
	ioctl_req->req_id = MLAN_IOCTL_PM_CFG;
	ioctl_req->action = action;
	if (action == MLAN_ACT_SET)
		moal_memcpy_ext(priv->phandle, &pm_cfg->param.ps_mgmt, ps_mgmt,
				sizeof(mlan_ds_ps_mgmt),
				sizeof(pm_cfg->param.ps_mgmt));
	status = woal_request_ioctl(priv, ioctl_req, MOAL_IOCTL_WAIT);
	if (status == MLAN_STATUS_SUCCESS) {
		if (action == MLAN_ACT_GET)
			moal_memcpy_ext(priv->phandle, ps_mgmt,
					&pm_cfg->param.ps_mgmt,
					sizeof(mlan_ds_ps_mgmt),
					sizeof(mlan_ds_ps_mgmt));
	}
	if (status != MLAN_STATUS_PENDING)
		kfree(ioctl_req);
	LEAVE();
	return status;
}

/**
 *  @brief uap power mode ioctl handler
 *
 *  @param dev      A pointer to net_device structure
 *  @param req      A pointer to ifreq structure
 *  @return         0 --success, otherwise fail
 */
static int woal_uap_power_mode_ioctl(struct net_device *dev, struct ifreq *req)
{
	moal_private *priv = (moal_private *)netdev_priv(dev);
	mlan_ioctl_req *ioctl_req = NULL;
	mlan_ds_pm_cfg *pm_cfg = NULL;
	mlan_ds_ps_mgmt ps_mgmt;
	int ret = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	memset(&ps_mgmt, 0, sizeof(mlan_ds_ps_mgmt));

	/* Sanity check */
	if (req->ifr_data == NULL) {
		PRINTM(MERROR, "uap_power_mode_ioctl() corrupt data\n");
		ret = -EFAULT;
		goto done;
	}
	if (copy_from_user(&ps_mgmt, req->ifr_data, sizeof(mlan_ds_ps_mgmt))) {
		PRINTM(MERROR, "Copy from user failed\n");
		ret = -EFAULT;
		goto done;
	}
	if (priv->bss_type != MLAN_BSS_TYPE_UAP) {
		PRINTM(MERROR, "Invlaid BSS_TYPE for UAP power mode command\n");
		ret = -EFAULT;
		goto done;
	}
	PRINTM(MIOCTL,
	       "ioctl power: flag=0x%x ps_mode=%u ctrl_bitmap=%u min_sleep=%u max_sleep=%u "
	       "inact_to=%u min_awake=%u max_awake=%u\n",
	       ps_mgmt.flags, ps_mgmt.ps_mode, ps_mgmt.sleep_param.ctrl_bitmap,
	       ps_mgmt.sleep_param.min_sleep, ps_mgmt.sleep_param.max_sleep,
	       ps_mgmt.inact_param.inactivity_to, ps_mgmt.inact_param.min_awake,
	       ps_mgmt.inact_param.max_awake);

	if (ps_mgmt.flags & ~(PS_FLAG_PS_MODE | PS_FLAG_SLEEP_PARAM |
			      PS_FLAG_INACT_SLEEP_PARAM)) {
		PRINTM(MERROR, "Invalid parameter: flags = 0x%x\n",
		       ps_mgmt.flags);
		ret = -EINVAL;
		goto done;
	}

	if (ps_mgmt.ps_mode > PS_MODE_INACTIVITY) {
		PRINTM(MERROR, "Invalid parameter: ps_mode = %d\n",
		       (int)ps_mgmt.flags);
		ret = -EINVAL;
		goto done;
	}

	ioctl_req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_pm_cfg));
	if (ioctl_req == NULL) {
		ret = -ENOMEM;
		goto done;
	}
	pm_cfg = (mlan_ds_pm_cfg *)ioctl_req->pbuf;
	pm_cfg->sub_command = MLAN_OID_PM_CFG_PS_MODE;
	ioctl_req->req_id = MLAN_IOCTL_PM_CFG;
	if (ps_mgmt.flags) {
		ioctl_req->action = MLAN_ACT_SET;
		moal_memcpy_ext(priv->phandle, &pm_cfg->param.ps_mgmt, &ps_mgmt,
				sizeof(mlan_ds_ps_mgmt),
				sizeof(pm_cfg->param.ps_mgmt));
	} else {
		ioctl_req->action = MLAN_ACT_GET;
	}

	status = woal_request_ioctl(priv, ioctl_req, MOAL_IOCTL_WAIT);
	if ((status != MLAN_STATUS_SUCCESS) &&
	    (status != MLAN_STATUS_PENDING)) {
		ret = -EFAULT;
		if (copy_to_user(req->ifr_data, &ioctl_req->status_code,
				 sizeof(t_u32)))
			PRINTM(MERROR, "Copy to user failed!\n");
		goto done;
	}
	if (!ps_mgmt.flags) {
		/* Copy to user */
		if (copy_to_user(req->ifr_data, &pm_cfg->param.ps_mgmt,
				 sizeof(mlan_ds_ps_mgmt))) {
			PRINTM(MERROR, "Copy to user failed!\n");
			ret = -EFAULT;
			goto done;
		}
	}
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(ioctl_req);
	LEAVE();
	return ret;
}

/**
 *  @brief uap BSS config ioctl handler
 *
 *  @param dev      A pointer to net_device structure
 *  @param req      A pointer to ifreq structure
 *  @return         0 --success, otherwise fail
 */
static int woal_uap_bss_cfg_ioctl(struct net_device *dev, struct ifreq *req)
{
	moal_private *priv = (moal_private *)netdev_priv(dev);
	int ret = 0;
	mlan_ds_bss *bss = NULL;
	mlan_ioctl_req *ioctl_req = NULL;
	int offset = 0;
	t_u32 action = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	/* Sanity check */
	if (req->ifr_data == NULL) {
		PRINTM(MERROR, "uap_bss_cfg_ioctl() corrupt data\n");
		ret = -EFAULT;
		goto done;
	}

	/* Get action */
	if (copy_from_user(&action, req->ifr_data + offset, sizeof(action))) {
		PRINTM(MERROR, "Copy from user failed\n");
		ret = -EFAULT;
		goto done;
	}
	offset += sizeof(action);

	/* Allocate an IOCTL request buffer */
	ioctl_req = (mlan_ioctl_req *)woal_alloc_mlan_ioctl_req(
		sizeof(mlan_ds_bss));
	if (ioctl_req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	bss = (mlan_ds_bss *)ioctl_req->pbuf;
	bss->sub_command = MLAN_OID_UAP_BSS_CONFIG;
	ioctl_req->req_id = MLAN_IOCTL_BSS;
	if (action == 1)
		ioctl_req->action = MLAN_ACT_SET;
	else
		ioctl_req->action = MLAN_ACT_GET;

	if (ioctl_req->action == MLAN_ACT_SET) {
		/* Get the BSS config from user */
		if (copy_from_user(&bss->param.bss_config,
				   req->ifr_data + offset,
				   sizeof(mlan_uap_bss_param))) {
			PRINTM(MERROR, "Copy from user failed\n");
			ret = -EFAULT;
			goto done;
		}
	}

	status = woal_request_ioctl(priv, ioctl_req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	if (ioctl_req->action == MLAN_ACT_GET) {
		offset = sizeof(action);

		/* Copy to user : BSS config */
		if (copy_to_user(req->ifr_data + offset, &bss->param.bss_config,
				 sizeof(mlan_uap_bss_param))) {
			PRINTM(MERROR, "Copy to user failed!\n");
			ret = -EFAULT;
			goto done;
		}
	}
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(ioctl_req);
	LEAVE();
	return ret;
}

/**
 *  @brief uap get station list handler
 *
 *  @param dev      A pointer to net_device structure
 *  @param req      A pointer to ifreq structure
 *  @return         0 --success, otherwise fail
 */
static int woal_uap_get_sta_list_ioctl(struct net_device *dev,
				       struct ifreq *req)
{
	moal_private *priv = (moal_private *)netdev_priv(dev);
	int ret = 0;
	mlan_ds_get_info *info = NULL;
	mlan_ioctl_req *ioctl_req = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	/* Sanity check */
	if (req->ifr_data == NULL) {
		PRINTM(MERROR, "uap_get_sta_list_ioctl() corrupt data\n");
		ret = -EFAULT;
		goto done;
	}

	/* Allocate an IOCTL request buffer */
	ioctl_req = (mlan_ioctl_req *)woal_alloc_mlan_ioctl_req(
		sizeof(mlan_ds_get_info) +
		(MAX_STA_LIST_IE_SIZE * MAX_NUM_CLIENTS));
	if (ioctl_req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	info = (mlan_ds_get_info *)ioctl_req->pbuf;
	info->sub_command = MLAN_OID_UAP_STA_LIST;
	ioctl_req->req_id = MLAN_IOCTL_GET_INFO;
	ioctl_req->action = MLAN_ACT_GET;

	status = woal_request_ioctl(priv, ioctl_req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	if (ioctl_req->action == MLAN_ACT_GET) {
		/* Copy to user : sta_list */
		if (copy_to_user(req->ifr_data, &info->param.sta_list,
				 ioctl_req->data_read_written)) {
			PRINTM(MERROR, "Copy to user failed!\n");
			ret = -EFAULT;
			goto done;
		}
	}
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(ioctl_req);
	LEAVE();
	return ret;
}

/**
 *  @brief uAP set WAPI key ioctl
 *
 *  @param priv      A pointer to moal_private structure
 *  @param msg       A pointer to wapi_msg structure
 *
 *  @return          0 --success, otherwise fail
 */
static int woal_uap_set_wapi_key_ioctl(moal_private *priv, wapi_msg *msg)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_sec_cfg *sec = NULL;
	int ret = 0;
	wapi_key_msg *key_msg = NULL;
	t_u8 bcast_addr[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();
	if (msg->msg_len != sizeof(wapi_key_msg)) {
		ret = -EINVAL;
		goto done;
	}
	key_msg = (wapi_key_msg *)msg->msg;

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_sec_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}
	sec = (mlan_ds_sec_cfg *)req->pbuf;
	sec->sub_command = MLAN_OID_SEC_CFG_ENCRYPT_KEY;
	req->req_id = MLAN_IOCTL_SEC_CFG;
	req->action = MLAN_ACT_SET;

	sec->param.encrypt_key.is_wapi_key = MTRUE;
	sec->param.encrypt_key.key_len = MLAN_MAX_KEY_LENGTH;
	moal_memcpy_ext(priv->phandle, sec->param.encrypt_key.mac_addr,
			key_msg->mac_addr, ETH_ALEN,
			sizeof(sec->param.encrypt_key.mac_addr));
	sec->param.encrypt_key.key_index = key_msg->key_id;
	if (0 == memcmp(key_msg->mac_addr, bcast_addr, ETH_ALEN))
		sec->param.encrypt_key.key_flags = KEY_FLAG_GROUP_KEY;
	else
		sec->param.encrypt_key.key_flags = KEY_FLAG_SET_TX_KEY;
	moal_memcpy_ext(priv->phandle, sec->param.encrypt_key.key_material,
			key_msg->key, sec->param.encrypt_key.key_len,
			sizeof(sec->param.encrypt_key.key_material));

	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS)
		ret = -EFAULT;
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Enable/Disable wapi in firmware
 *
 *  @param priv          A pointer to moal_private structure
 *  @param enable        MTRUE/MFALSE
 *
 *  @return              MLAN_STATUS_SUCCESS/MLAN_STATUS_PENDING -- success,
 * otherwise fail
 */
static mlan_status woal_enable_wapi(moal_private *priv, t_u8 enable)
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
	bss->sub_command = MLAN_OID_UAP_BSS_CONFIG;
	req->req_id = MLAN_IOCTL_BSS;
	req->action = MLAN_ACT_GET;

	/* Send IOCTL request to MLAN */
	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		PRINTM(MERROR,
		       "Get AP setting  failed! status=%d, error_code=0x%x\n",
		       status, req->status_code);
	}

	/* Change AP default setting */
	req->action = MLAN_ACT_SET;
	if (enable == MFALSE) {
		bss->param.bss_config.auth_mode = MLAN_AUTH_MODE_OPEN;
		bss->param.bss_config.protocol = PROTOCOL_NO_SECURITY;
	} else {
		bss->param.bss_config.auth_mode = MLAN_AUTH_MODE_OPEN;
		bss->param.bss_config.protocol = PROTOCOL_WAPI;
	}

	/* Send IOCTL request to MLAN */
	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		PRINTM(MERROR,
		       "Set AP setting failed! status=%d, error_code=0x%x\n",
		       status, req->status_code);
	}
	if (enable) {
		if (woal_uap_bss_ctrl(priv, MOAL_IOCTL_WAIT, UAP_BSS_START))
			PRINTM(MERROR, "%s: uap bss start failed \n", __func__);
	}
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return status;
}

/**
 *  @brief uAP set WAPI flag ioctl
 *
 *  @param priv      A pointer to moal_private structure
 *  @param msg       A pointer to wapi_msg structure
 *
 *  @return          0 --success, otherwise fail
 */
static int woal_uap_set_wapi_flag_ioctl(moal_private *priv, wapi_msg *msg)
{
	t_u8 wapi_psk_ie[] = {0x44, 0x14, 0x01, 0x00, 0x01, 0x00, 0x00, 0x14,
			      0x72, 0x02, 0x01, 0x00, 0x00, 0x14, 0x72, 0x01,
			      0x00, 0x14, 0x72, 0x01, 0x00, 0x00};
	t_u8 wapi_cert_ie[] = {0x44, 0x14, 0x01, 0x00, 0x01, 0x00, 0x00, 0x14,
			       0x72, 0x01, 0x01, 0x00, 0x00, 0x14, 0x72, 0x01,
			       0x00, 0x14, 0x72, 0x01, 0x00, 0x00};
	mlan_ds_misc_cfg *misc = NULL;
	mlan_ioctl_req *req = NULL;
	int ret = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	if (woal_uap_bss_ctrl(priv, MOAL_IOCTL_WAIT, UAP_BSS_STOP)) {
		PRINTM(MERROR, "%s: uap bss stop failed \n", __func__);
		ret = -EFAULT;
		goto done;
	}

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}
	misc = (mlan_ds_misc_cfg *)req->pbuf;
	misc->sub_command = MLAN_OID_MISC_GEN_IE;
	req->req_id = MLAN_IOCTL_MISC_CFG;
	req->action = MLAN_ACT_SET;

	misc->param.gen_ie.type = MLAN_IE_TYPE_GEN_IE;
	misc->param.gen_ie.len = sizeof(wapi_psk_ie);
	if (msg->msg[0] & WAPI_MODE_PSK) {
		moal_memcpy_ext(priv->phandle, misc->param.gen_ie.ie_data,
				wapi_psk_ie, misc->param.gen_ie.len,
				sizeof(misc->param.gen_ie.ie_data));
	} else if (msg->msg[0] & WAPI_MODE_CERT) {
		moal_memcpy_ext(priv->phandle, misc->param.gen_ie.ie_data,
				wapi_cert_ie, misc->param.gen_ie.len,
				sizeof(misc->param.gen_ie.ie_data));
	} else if (msg->msg[0] == 0) {
		/* disable WAPI in driver */
		if (MLAN_STATUS_SUCCESS !=
		    woal_set_wapi_enable(priv, MOAL_IOCTL_WAIT, 0))
			ret = -EFAULT;
		woal_enable_wapi(priv, MFALSE);
		goto done;
	} else {
		ret = -EINVAL;
		goto done;
	}
	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}
	woal_enable_wapi(priv, MTRUE);
done:
	if ((status != MLAN_STATUS_PENDING) && req)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief set wapi ioctl function
 *
 *  @param dev      A pointer to net_device structure
 *  @param req      A pointer to ifreq structure
 *  @return         0 --success, otherwise fail
 */
static int woal_uap_set_wapi(struct net_device *dev, struct ifreq *req)
{
	moal_private *priv = (moal_private *)netdev_priv(dev);
	wapi_msg msg;
	int ret = 0;

	ENTER();

	/* Sanity check */
	if (req->ifr_data == NULL) {
		PRINTM(MERROR, "uap_set_wapi() corrupt data\n");
		ret = -EFAULT;
		goto done;
	}

	memset(&msg, 0, sizeof(msg));

	if (copy_from_user(&msg, req->ifr_data, sizeof(msg))) {
		PRINTM(MERROR, "Copy from user failed\n");
		ret = -EFAULT;
		goto done;
	}

	PRINTM(MIOCTL, "set wapi msg_type = %d, msg_len=%d\n", msg.msg_type,
	       msg.msg_len);
	DBG_HEXDUMP(MCMD_D, "wapi msg", msg.msg,
		    MIN(msg.msg_len, sizeof(msg.msg)));

	switch (msg.msg_type) {
	case P80211_PACKET_WAPIFLAG:
		ret = woal_uap_set_wapi_flag_ioctl(priv, &msg);
		break;
	case P80211_PACKET_SETKEY:
		ret = woal_uap_set_wapi_key_ioctl(priv, &msg);
		break;
	default:
		ret = -EOPNOTSUPP;
		break;
	}
done:
	LEAVE();
	return ret;
}

/********************************************************
		Global Functions
********************************************************/
/**
 *  @brief Initialize the members of mlan_uap_bss_param
 *  which are uploaded from firmware
 *
 *  @param priv     A pointer to moal_private structure
 *  @param sys_cfg  A pointer to mlan_uap_bss_param structure
 *  @param wait_option      Wait option
 *
 *  @return         MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status woal_uap_get_bss_param(moal_private *priv,
					  mlan_uap_bss_param *sys_cfg,
					  t_u8 wait_option)
{
	mlan_ds_bss *info = NULL;
	mlan_ioctl_req *req = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_bss));
	if (req == NULL) {
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}

	info = (mlan_ds_bss *)req->pbuf;
	info->sub_command = MLAN_OID_UAP_BSS_CONFIG;
	req->req_id = MLAN_IOCTL_BSS;
	req->action = MLAN_ACT_GET;

	status = woal_request_ioctl(priv, req, wait_option);
	if (status != MLAN_STATUS_SUCCESS) {
		PRINTM(MERROR, "Get bss info failed!\n");
		status = MLAN_STATUS_FAILURE;
		goto done;
	}
	moal_memcpy_ext(priv->phandle, sys_cfg, &info->param.bss_config,
			sizeof(mlan_uap_bss_param), sizeof(mlan_uap_bss_param));

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return status;
}

/**
 *  @brief  Set uap httxcfg
 *
 *  @param priv     A pointer to moal_private structure
 *  @param band_cfg Band cfg
 *  @param en       Enabled/Disabled
 *
 *  @return         0 --success, otherwise fail
 */
int woal_set_uap_ht_tx_cfg(moal_private *priv, Band_Config_t bandcfg,
			   t_u16 ht_cap, t_u8 en)
{
	int ret = 0;
	mlan_ds_11n_cfg *cfg_11n = NULL;
	mlan_ioctl_req *ioctl_req = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	ioctl_req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_11n_cfg));
	if (ioctl_req == NULL) {
		ret = -ENOMEM;
		goto done;
	}
	cfg_11n = (mlan_ds_11n_cfg *)ioctl_req->pbuf;
	cfg_11n->sub_command = MLAN_OID_11N_CFG_TX;
	ioctl_req->req_id = MLAN_IOCTL_11N_CFG;

	/* Set HT Tx configurations */
	if (bandcfg.chanBand == BAND_2GHZ) {
		if (en)
			cfg_11n->param.tx_cfg.httxcap = ht_cap;
		else
			cfg_11n->param.tx_cfg.httxcap = 0;
		cfg_11n->param.tx_cfg.misc_cfg = BAND_SELECT_BG;
	} else if (bandcfg.chanBand == BAND_5GHZ) {
		if (en)
			cfg_11n->param.tx_cfg.httxcap = ht_cap;
		else
			cfg_11n->param.tx_cfg.httxcap = 0;
		cfg_11n->param.tx_cfg.misc_cfg = BAND_SELECT_A;
	}
	PRINTM(MCMND, "SET: httxcap=0x%x band:0x%x\n",
	       cfg_11n->param.tx_cfg.httxcap, cfg_11n->param.tx_cfg.misc_cfg);
	/* Update 11n tx parameters in MLAN */
	ioctl_req->action = MLAN_ACT_SET;
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
 *  @brief Set 11n status based on the configured security mode
 *
 *  @param priv     A pointer to moal_private structure
 *  @param sys_cfg  A pointer to mlan_uap_bss_param structure
 *  @param action   MLAN_ACT_DISABLE or MLAN_ACT_ENABLE
 *
 *  @return         MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status woal_uap_set_11n_status(moal_private *priv,
				    mlan_uap_bss_param *sys_cfg, t_u8 action)
{
	mlan_status status = MLAN_STATUS_SUCCESS;
	mlan_fw_info fw_info;

	ENTER();
	memset(&fw_info, 0, sizeof(mlan_fw_info));
	if (action == MLAN_ACT_DISABLE) {
		if ((sys_cfg->supported_mcs_set[0] == 0) &&
		    (sys_cfg->supported_mcs_set[4] == 0) &&
		    (sys_cfg->supported_mcs_set[1] == 0)) {
			goto done;
		} else {
			sys_cfg->supported_mcs_set[0] = 0;
			sys_cfg->supported_mcs_set[4] = 0;
			sys_cfg->supported_mcs_set[1] = 0;
		}
	}

	if (action == MLAN_ACT_ENABLE) {
		woal_request_get_fw_info(priv, MOAL_IOCTL_WAIT, &fw_info);
		sys_cfg->supported_mcs_set[0] = 0xFF;
		if (sys_cfg->bandcfg.chan2Offset)
			sys_cfg->supported_mcs_set[4] = 0x01;
		else
			sys_cfg->supported_mcs_set[4] = 0x0;
		if (fw_info.usr_dev_mcs_support == HT_STREAM_MODE_2X2)
			sys_cfg->supported_mcs_set[1] = 0xFF;
		else
			sys_cfg->supported_mcs_set[1] = 0;
	}

done:
	LEAVE();
	return status;
}

#define VHT_CAP_11AC_MASK 0x007fffff

/**
 *  @brief enable/disable 11AC
 *
 *  @param priv     A pointer to moal_private structure
 *  @param action   MLAN_ACT_DISABLE or MLAN_ACT_ENABLE
 *  @param vht20_40 Enable VHT 20 MHz or 40 MHz band
 *  @param vhtcap_ie A pointer to vht capability IE
 *
 *  @return         0--success, otherwise failure
 */
int woal_uap_set_11ac_status(moal_private *priv, t_u8 action, t_u8 band,
			     t_u8 vht20_40, const IEEEtypes_VHTCap_t *vhtcap_ie)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_11ac_cfg *cfg_11ac = NULL;
	mlan_fw_info fw_info;
	int ret = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	memset(&fw_info, 0, sizeof(mlan_fw_info));
	woal_request_get_fw_info(priv, MOAL_IOCTL_WAIT, &fw_info);
	if ((band == BAND_5GHZ && !(fw_info.fw_bands & BAND_AAC)) ||
	    (band == BAND_2GHZ && !(fw_info.fw_bands & BAND_GAC))) {
		PRINTM(MERROR, "fw doesn't support 11ac\n");
		ret = -EFAULT;
		goto done;
	}

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_11ac_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	cfg_11ac = (mlan_ds_11ac_cfg *)req->pbuf;
	cfg_11ac->sub_command = MLAN_OID_11AC_VHT_CFG;
	req->req_id = MLAN_IOCTL_11AC_CFG;
	req->action = MLAN_ACT_SET;

	cfg_11ac->param.vht_cfg.band = BAND_SELECT_A;
	cfg_11ac->param.vht_cfg.txrx = MLAN_RADIO_TXRX;

	/*
	 * p2p GO (negotiation or auto GO) cases, wpa_supplicant will download
	 * invalid vht capability with value 0 in beacon parameters, so for p2p
	 * GO case (vht_cap_info = 0), driver will use hardware 11ac vht
	 * capability value instead of up layer value.
	 */
	if (vhtcap_ie && vhtcap_ie->vht_cap.vht_cap_info != 0) {
		cfg_11ac->param.vht_cfg.vht_cap_info =
			woal_le32_to_cpu(vhtcap_ie->vht_cap.vht_cap_info);
		/** todo mcs configuration */
	} else {
		cfg_11ac->param.vht_cfg.vht_cap_info =
			fw_info.usr_dot_11ac_dev_cap_a;
	}
	if (action == MLAN_ACT_DISABLE) {
		cfg_11ac->param.vht_cfg.bwcfg = MFALSE;
		cfg_11ac->param.vht_cfg.vht_cap_info &= ~VHT_CAP_11AC_MASK;
		cfg_11ac->param.vht_cfg.vht_rx_mcs =
			cfg_11ac->param.vht_cfg.vht_tx_mcs = 0xffff;
		cfg_11ac->param.vht_cfg.skip_usr_11ac_mcs_cfg = MTRUE;
	} else {
		if (vht20_40)
			cfg_11ac->param.vht_cfg.bwcfg = MFALSE;
		else
			cfg_11ac->param.vht_cfg.bwcfg = MTRUE;
		cfg_11ac->param.vht_cfg.vht_cap_info &=
			~DEFALUT_11AC_CAP_BEAMFORMING_RESET_MASK;
		cfg_11ac->param.vht_cfg.vht_tx_mcs =
			fw_info.usr_dot_11ac_mcs_support >> 16;
		cfg_11ac->param.vht_cfg.vht_rx_mcs =
			fw_info.usr_dot_11ac_mcs_support & 0xffff;
		cfg_11ac->param.vht_cfg.skip_usr_11ac_mcs_cfg = MTRUE;
	}
	PRINTM(MCMND,
	       "Uap:11ac=%d vht_cap_info=0x%x, vht_tx_mcs=0x%x, vht_rx_mcs=0x%x\n",
	       action, cfg_11ac->param.vht_cfg.vht_cap_info,
	       cfg_11ac->param.vht_cfg.vht_tx_mcs,
	       cfg_11ac->param.vht_cfg.vht_rx_mcs);

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
 *  @brief Get/Set 11ax cfg
 *
 *  @param priv     A pointer to moal_private structure
 *  @param action   MLAN_ACT_SET or MLAN_ACT_GET
 *  @param he_cfg   a pointer to mlan_ds_11ax_he_cfg
 *  @param wait_option  wait_option
 *
 *  @return         0--success, otherwise failure
 */
int woal_11ax_cfg(moal_private *priv, t_u8 action, mlan_ds_11ax_he_cfg *he_cfg,
		  t_u8 wait_option)
{
	int ret = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;
	mlan_ioctl_req *req = NULL;
	mlan_ds_11ax_cfg *cfg_11ax = NULL;
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_11ax_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}
	cfg_11ax = (mlan_ds_11ax_cfg *)req->pbuf;
	cfg_11ax->sub_command = MLAN_OID_11AX_HE_CFG;
	req->req_id = MLAN_IOCTL_11AX_CFG;
	req->action = action;
	moal_memcpy_ext(priv->phandle, &cfg_11ax->param.he_cfg, he_cfg,
			sizeof(mlan_ds_11ax_he_cfg),
			sizeof(mlan_ds_11ax_he_cfg));
	status = woal_request_ioctl(priv, req, wait_option);
	if (status == MLAN_STATUS_SUCCESS) {
		moal_memcpy_ext(priv->phandle, he_cfg, &cfg_11ax->param.he_cfg,
				sizeof(mlan_ds_11ax_he_cfg),
				sizeof(mlan_ds_11ax_he_cfg));
	}
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief enable/disable 11AX
 *
 *  @param priv     A pointer to moal_private structure
 *  @param action   MLAN_ACT_DISABLE or MLAN_ACT_ENABLE
 *  @param band     band config
 *  @param hecap_ie
 *
 *  @return         0--success, otherwise failure
 */
int woal_uap_set_11ax_status(moal_private *priv, t_u8 action, t_u8 band,
			     const IEEEtypes_HECap_t *hecap_ie)
{
	mlan_fw_info fw_info;
	int ret = 0;
	mlan_ds_11ax_he_cfg he_cfg;
	ENTER();

	memset(&fw_info, 0, sizeof(mlan_fw_info));
	woal_request_get_fw_info(priv, MOAL_IOCTL_WAIT, &fw_info);
	if ((band == BAND_5GHZ && !(fw_info.fw_bands & BAND_AAX)) ||
	    (band == BAND_2GHZ && !(fw_info.fw_bands & BAND_GAX))) {
		PRINTM(MERROR, "fw doesn't support 11ax\n");
		ret = -EFAULT;
		goto done;
	}
	if ((band == BAND_6GHZ) && !(fw_info.fw_bands & BAND_6G)) {
		PRINTM(MERROR, "FW doesn't support 6E 11ax\n");
		ret = -EFAULT;
		goto done;
	}
	memset(&he_cfg, 0, sizeof(he_cfg));
	if (band == BAND_5GHZ)
		he_cfg.band = MBIT(1);
	else if (band == BAND_2GHZ)
		he_cfg.band = MBIT(0);
	else if (band == BAND_6GHZ)
		he_cfg.band = MBIT(2);
	else {
		PRINTM(MERROR, "Invalid band!\n");
		ret = -EFAULT;
		goto done;
	}
	if (woal_11ax_cfg(priv, MLAN_ACT_GET, &he_cfg, MOAL_IOCTL_WAIT)) {
		PRINTM(MERROR, "Fail to get 11ax cfg!\n");
		ret = -EFAULT;
		goto done;
	}
	if (hecap_ie) {
		DBG_HEXDUMP(MCMD_D, "hecap_ie", (const t_u8 *)hecap_ie,
			    hecap_ie->ieee_hdr.len +
				    sizeof(IEEEtypes_Header_t));
		he_cfg.he_cap.id = hecap_ie->ieee_hdr.element_id;
		he_cfg.he_cap.len = hecap_ie->ieee_hdr.len;
		moal_memcpy_ext(priv->phandle, &he_cfg.he_cap.ext_id,
				&hecap_ie->ext_id, he_cfg.he_cap.len,
				he_cfg.he_cap.len);
	}
#define HE_MAC_CAP_TWT_REQ_SUPPORT MBIT(1)
	/* uap mode should be TWT responder only */
	he_cfg.he_cap.he_mac_cap[0] &= ~HE_MAC_CAP_TWT_REQ_SUPPORT;
	if (action == MLAN_ACT_DISABLE) {
		if (he_cfg.he_cap.len &&
		    (he_cfg.he_cap.ext_id == HE_CAPABILITY)) {
			memset(he_cfg.he_cap.he_txrx_mcs_support, 0xff,
			       sizeof(he_cfg.he_cap.he_txrx_mcs_support));
		} else {
			PRINTM(MCMND, "11ax already disabled\n");
			goto done;
		}
	}
	DBG_HEXDUMP(MCMD_D, "HE_CFG ", (t_u8 *)&he_cfg, sizeof(he_cfg));
	ret = woal_11ax_cfg(priv, MLAN_ACT_SET, &he_cfg, MOAL_IOCTL_WAIT);
done:
	LEAVE();
	return ret;
}

/**
 *  @brief Parse AP configuration from ASCII string
 *
 *  @param priv       A pointer to moal_private structure
 *  @param ap_cfg   A pointer to mlan_uap_bss_param structure
 *  @param buf      A pointer to user data
 *
 *  @return         0 --success, otherwise fail
 */
static int woal_uap_ap_cfg_parse_data(moal_private *priv,
				      mlan_uap_bss_param *ap_cfg, char *buf)
{
	int ret = 0, atoi_ret;
	int set_sec = 0, set_key = 0, set_chan = 0;
	int set_preamble = 0, set_scb = 0, set_ssid = 0;
	char *begin = buf, *value = NULL, *opt = NULL;

	ENTER();

	while (begin) {
		value = woal_strsep(&begin, ',', '/');
		opt = woal_strsep(&value, '=', '/');
		if (opt && !strncmp(opt, "END", strlen("END"))) {
			if (!ap_cfg->ssid.ssid_len) {
				PRINTM(MERROR,
				       "Minimum option required is SSID\n");
				ret = -EINVAL;
				goto done;
			}
			PRINTM(MINFO, "Parsing terminated by string END\n");
			break;
		}
		if (!opt || !value || !value[0]) {
			PRINTM(MERROR, "Invalid option\n");
			ret = -EINVAL;
			goto done;
		} else if (!strncmp(opt, "ASCII_CMD", strlen("ASCII_CMD"))) {
			if (strncmp(value, "AP_CFG", strlen("AP_CFG"))) {
				PRINTM(MERROR,
				       "ASCII_CMD: %s not matched with AP_CFG\n",
				       value);
				ret = -EFAULT;
				goto done;
			}
			value = woal_strsep(&begin, ',', '/');
			opt = woal_strsep(&value, '=', '/');
			if (!opt || !value || !value[0]) {
				PRINTM(MERROR,
				       "Minimum option required is SSID\n");
				ret = -EINVAL;
				goto done;
			} else if (!strncmp(opt, "SSID", strlen("SSID"))) {
				if (set_ssid) {
					PRINTM(MWARN,
					       "Skipping SSID, found again!\n");
					continue;
				}
				if (strlen(value) >= MLAN_MAX_SSID_LENGTH) {
					PRINTM(MERROR,
					       "SSID length exceeds max length\n");
					ret = -EFAULT;
					goto done;
				}
				ap_cfg->ssid.ssid_len = strlen(value);
				strncpy((char *)ap_cfg->ssid.ssid, value,
					MIN(MLAN_MAX_SSID_LENGTH - 1,
					    strlen(value)));
				PRINTM(MINFO, "ssid=%s, len=%d\n",
				       ap_cfg->ssid.ssid,
				       (int)ap_cfg->ssid.ssid_len);
				set_ssid = 1;
			} else {
				PRINTM(MERROR,
				       "AP_CFG: Invalid option %s, expect SSID\n",
				       opt);
				ret = -EINVAL;
				goto done;
			}
		} else if (!strncmp(opt, "SEC", strlen("SEC"))) {
			if (set_sec) {
				PRINTM(MWARN, "Skipping SEC, found again!\n");
				continue;
			}
			if (!strnicmp(value, "open", strlen("open"))) {
				ap_cfg->auth_mode = MLAN_AUTH_MODE_OPEN;
				if (set_key)
					ap_cfg->wpa_cfg.length = 0;
				ap_cfg->key_mgmt = KEY_MGMT_NONE;
				ap_cfg->protocol = PROTOCOL_NO_SECURITY;
			} else if (!strnicmp(value, "wpa2-psk",
					     strlen("wpa2-psk"))) {
				ap_cfg->auth_mode = MLAN_AUTH_MODE_OPEN;
				ap_cfg->protocol = PROTOCOL_WPA2;
				ap_cfg->key_mgmt = KEY_MGMT_PSK;
				ap_cfg->wpa_cfg.pairwise_cipher_wpa =
					CIPHER_AES_CCMP;
				ap_cfg->wpa_cfg.pairwise_cipher_wpa2 =
					CIPHER_AES_CCMP;
				ap_cfg->wpa_cfg.group_cipher = CIPHER_AES_CCMP;
			} else if (!strnicmp(value, "wpa-psk",
					     strlen("wpa-psk"))) {
				ap_cfg->auth_mode = MLAN_AUTH_MODE_OPEN;
				ap_cfg->protocol = PROTOCOL_WPA;
				ap_cfg->key_mgmt = KEY_MGMT_PSK;
				ap_cfg->wpa_cfg.pairwise_cipher_wpa =
					CIPHER_TKIP;
				ap_cfg->wpa_cfg.group_cipher = CIPHER_TKIP;
			} else if (!strnicmp(value, "wep128",
					     strlen("wep128"))) {
				ap_cfg->auth_mode = MLAN_AUTH_MODE_OPEN;
				if (set_key)
					ap_cfg->wpa_cfg.length = 0;
				ap_cfg->key_mgmt = KEY_MGMT_NONE;
				ap_cfg->protocol = PROTOCOL_STATIC_WEP;
			} else {
				PRINTM(MERROR,
				       "AP_CFG: Invalid value=%s for %s\n",
				       value, opt);
				ret = -EFAULT;
				goto done;
			}
			set_sec = 1;
		} else if (!strncmp(opt, "KEY", strlen("KEY"))) {
			if (set_key) {
				PRINTM(MWARN, "Skipping KEY, found again!\n");
				continue;
			}
			if (set_sec &&
			    ap_cfg->protocol == PROTOCOL_STATIC_WEP) {
				if (strlen(value) != MAX_WEP_KEY_SIZE) {
					PRINTM(MERROR,
					       "Invalid WEP KEY length\n");
					ret = -EFAULT;
					goto done;
				}
				ap_cfg->wep_cfg.key0.key_index = 0;
				ap_cfg->wep_cfg.key0.is_default = 1;
				ap_cfg->wep_cfg.key0.length = strlen(value);
				moal_memcpy_ext(
					priv->phandle, ap_cfg->wep_cfg.key0.key,
					value, strlen(value),
					sizeof(ap_cfg->wep_cfg.key0.key));
				set_key = 1;
				continue;
			}
			if (set_sec && ap_cfg->protocol != PROTOCOL_WPA2 &&
			    ap_cfg->protocol != PROTOCOL_WPA) {
				PRINTM(MWARN,
				       "Warning! No KEY for open mode\n");
				set_key = 1;
				continue;
			}
			if (strlen(value) < MLAN_MIN_PASSPHRASE_LENGTH ||
			    strlen(value) > MLAN_PMK_HEXSTR_LENGTH) {
				PRINTM(MERROR, "Invalid PSK/PMK length\n");
				ret = -EINVAL;
				goto done;
			}
			ap_cfg->wpa_cfg.length = strlen(value);
			moal_memcpy_ext(priv->phandle,
					ap_cfg->wpa_cfg.passphrase, value,
					strlen(value),
					sizeof(ap_cfg->wpa_cfg.passphrase));
			set_key = 1;
		} else if (!strncmp(opt, "CHANNEL", strlen("CHANNEL"))) {
			if (set_chan) {
				PRINTM(MWARN,
				       "Skipping CHANNEL, found again!\n");
				continue;
			}
			if (woal_atoi(&atoi_ret, value)) {
				ret = -EINVAL;
				goto done;
			}
			if (atoi_ret < 1 || atoi_ret > MLAN_MAX_CHANNEL) {
				PRINTM(MERROR,
				       "AP_CFG: Channel must be between 1 and %d"
				       "(both included)\n",
				       MLAN_MAX_CHANNEL);
				ret = -EINVAL;
				goto done;
			}
			ap_cfg->channel = atoi_ret;
			set_chan = 1;
		} else if (!strncmp(opt, "PREAMBLE", strlen("PREAMBLE"))) {
			if (set_preamble) {
				PRINTM(MWARN,
				       "Skipping PREAMBLE, found again!\n");
				continue;
			}
			if (woal_atoi(&atoi_ret, value)) {
				ret = -EINVAL;
				goto done;
			}
			/* This is a READ only value from FW, so we
			 * can not set this and pass it successfully */
			set_preamble = 1;
		} else if (!strncmp(opt, "MAX_SCB", strlen("MAX_SCB"))) {
			if (set_scb) {
				PRINTM(MWARN,
				       "Skipping MAX_SCB, found again!\n");
				continue;
			}
			if (woal_atoi(&atoi_ret, value)) {
				ret = -EINVAL;
				goto done;
			}
			if (atoi_ret < 1 || atoi_ret > MAX_STA_COUNT) {
				PRINTM(MERROR,
				       "AP_CFG: MAX_SCB must be between 1 to %d "
				       "(both included)\n",
				       MAX_STA_COUNT);
				ret = -EINVAL;
				goto done;
			}
			ap_cfg->max_sta_count = (t_u16)atoi_ret;
			set_scb = 1;
		} else {
			PRINTM(MERROR, "Invalid option %s\n", opt);
			ret = -EINVAL;
			goto done;
		}
	}

done:
	LEAVE();
	return ret;
}

#if defined(UAP_CFG80211)
#if defined(STA_WEXT) || defined(UAP_WEXT)
/**
 *  @brief Set/Get multi AP mode
 *
 *  @param priv             A pointer to moal_private structure
 *  @param wrq              A pointer to structure iwreq
 *
 *  @return                  0 --success, otherwise fail
 */
int woal_uap_set_get_multi_ap_mode(moal_private *priv, struct iwreq *wrq)
{
	int ret = 0;
	int mode = 0;

	ENTER();

	if (wrq->u.data.length) {
		if (wrq->u.data.length > 1) {
			PRINTM(MERROR, "Invalid no of arguments!\n");
			ret = -EINVAL;
			goto done;
		}
		if (copy_from_user(&mode, wrq->u.data.pointer, sizeof(int))) {
			PRINTM(MERROR, "copy from user failed\n");
			ret = -EFAULT;
			goto done;
		}
		if (mode == EASY_MESH_MULTI_AP_BSS_MODE_3)
			/* Supports backhaul and fronthaul BSS */
			priv->multi_ap_flag = EASY_MESH_MULTI_AP_BH_AND_FH_BSS;
		else if (mode == EASY_MESH_MULTI_AP_BSS_MODE_2)
			/* Supports backhaul BSS */
			priv->multi_ap_flag = EASY_MESH_MULTI_AP_BH_BSS;
		else if (mode == EASY_MESH_MULTI_AP_BSS_MODE_1)
			/* Supports fronthaul BSS */
			priv->multi_ap_flag = EASY_MESH_MULTI_AP_FH_BSS;
	} else {
		if (priv->multi_ap_flag == EASY_MESH_MULTI_AP_BH_AND_FH_BSS)
			mode = EASY_MESH_MULTI_AP_BSS_MODE_3;
		else if (priv->multi_ap_flag == EASY_MESH_MULTI_AP_BH_BSS)
			mode = EASY_MESH_MULTI_AP_BSS_MODE_2;
		else if (priv->multi_ap_flag == EASY_MESH_MULTI_AP_FH_BSS)
			mode = EASY_MESH_MULTI_AP_BSS_MODE_1;
		wrq->u.data.length = 1;
		if (copy_to_user(wrq->u.data.pointer, &mode, sizeof(int)))
			ret = -EFAULT;
	}
done:
	LEAVE();
	return ret;
}
#endif
#endif

/**
 *  @brief Set AP configuration
 *
 *  @param priv     A pointer to moal_private structure
 *  @param data     A pointer to user data
 *  @param len      Length of buf
 *
 *  @return         0 --success, otherwise fail
 */
int woal_uap_set_ap_cfg(moal_private *priv, t_u8 *data, int len)
{
	int ret = 0;
	static char buf[MAX_BUF_LEN];
	mlan_uap_bss_param *sys_config = NULL;
	int restart = 0;

	ENTER();

#define MIN_AP_CFG_CMD_LEN 16 /* strlen("ASCII_CMD=AP_CFG") */
	if ((len - 1) <= MIN_AP_CFG_CMD_LEN) {
		PRINTM(MERROR, "Invalid length of command\n");
		ret = -EINVAL;
		goto done;
	}
	sys_config = kzalloc(sizeof(mlan_uap_bss_param), GFP_ATOMIC);
	if (!sys_config) {
		PRINTM(MERROR, "Fail to alloc memory for mlan_uap_bss_param\n");
		ret = -EFAULT;
		goto done;
	}
	memset(buf, 0, MAX_BUF_LEN);
	moal_memcpy_ext(priv->phandle, buf, data, len, sizeof(buf) - 1);

	/* Initialize the uap bss values which are uploaded from firmware */
	woal_uap_get_bss_param(priv, sys_config, MOAL_IOCTL_WAIT);

	/* Setting the default values */
	sys_config->channel = 6;
	sys_config->preamble_type = 0;

	ret = woal_uap_ap_cfg_parse_data(priv, sys_config, buf);
	if (ret)
		goto done;

	/* If BSS already started stop it first and restart
	 * after changing the setting */
	if (priv->bss_started == MTRUE) {
		ret = woal_uap_bss_ctrl(priv, MOAL_IOCTL_WAIT, UAP_BSS_STOP);
		if (ret)
			goto done;
		restart = 1;
	}

	/* If the security mode is configured as WEP or WPA-PSK,
	 * it will disable 11n automatically, and if configured as
	 * open(off) or wpa2-psk, it will automatically enable 11n */
	if ((sys_config->protocol == PROTOCOL_STATIC_WEP) ||
	    (sys_config->protocol == PROTOCOL_WPA)) {
		if (MLAN_STATUS_SUCCESS !=
		    woal_uap_set_11n_status(priv, sys_config,
					    MLAN_ACT_DISABLE)) {
			ret = -EFAULT;
			goto done;
		}
	} else {
		if (MLAN_STATUS_SUCCESS !=
		    woal_uap_set_11n_status(priv, sys_config,
					    MLAN_ACT_ENABLE)) {
			ret = -EFAULT;
			goto done;
		}
	}

	if (MLAN_STATUS_SUCCESS != woal_set_get_sys_config(priv, MLAN_ACT_SET,
							   MOAL_IOCTL_WAIT,
							   sys_config)) {
		ret = -EFAULT;
		goto done;
	}

	/* Start the BSS after successful configuration */
	if (restart)
		ret = woal_uap_bss_ctrl(priv, MOAL_IOCTL_WAIT, UAP_BSS_START);

done:
	kfree(sys_config);
	LEAVE();
	return ret;
}

/**
 *  @brief Set/Get ap scan channel list
 *
 *  @param priv             A pointer to moal_private structure
 *  @param action           MLAN_ACT_SET or MLAN_ACT_GET
 *  @param scan_channels    A pointer to mlan_uap_scan_channels structure
 *
 *  @return                 MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status
woal_set_get_ap_scan_channels(moal_private *priv, t_u16 action,
			      mlan_uap_scan_channels *scan_channels)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_ds_bss *bss = NULL;
	mlan_ioctl_req *req = NULL;

	ENTER();

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_bss));
	if (req == NULL) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	bss = (mlan_ds_bss *)req->pbuf;
	bss->sub_command = MLAN_OID_UAP_SCAN_CHANNELS;
	req->req_id = MLAN_IOCTL_BSS;
	req->action = action;

	moal_memcpy_ext(priv->phandle, &bss->param.ap_scan_channels,
			scan_channels, sizeof(mlan_uap_scan_channels),
			sizeof(bss->param.ap_scan_channels));

	ret = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (ret != MLAN_STATUS_SUCCESS)
		goto done;
	if (action == MLAN_ACT_GET)
		moal_memcpy_ext(priv->phandle, scan_channels,
				&bss->param.ap_scan_channels,
				sizeof(mlan_uap_scan_channels),
				sizeof(mlan_uap_scan_channels));

done:
	if (ret != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Set/Get uap channel
 *
 *  @param priv             A pointer to moal_private structure
 *  @param action           MLAN_ACT_SET or MLAN_ACT_GET
 *  @param wait_option      wait option
 *  @param uap_channel      A pointer to mlan_uap_channel structure
 *
 *  @return                 MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status woal_set_get_ap_channel(moal_private *priv, t_u16 action,
				    t_u8 wait_option,
				    chan_band_info *uap_channel)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_ds_bss *bss = NULL;
	mlan_ioctl_req *req = NULL;

	ENTER();

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_bss));
	if (req == NULL) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	bss = (mlan_ds_bss *)req->pbuf;
	bss->sub_command = MLAN_OID_UAP_CHANNEL;
	req->req_id = MLAN_IOCTL_BSS;
	req->action = action;

	moal_memcpy_ext(priv->phandle, &bss->param.ap_channel, uap_channel,
			sizeof(chan_band_info), sizeof(bss->param.ap_channel));
	ret = woal_request_ioctl(priv, req, wait_option);
	if (ret != MLAN_STATUS_SUCCESS)
		goto done;
	if (action == MLAN_ACT_GET)
		moal_memcpy_ext(priv->phandle, uap_channel,
				&bss->param.ap_channel, sizeof(chan_band_info),
				sizeof(chan_band_info));

done:
	if (ret != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief start ACS scan
 *
 *  @param priv             A pointer to moal_private structure
 *  @param action           MLAN_ACT_SET or MLAN_ACT_GET
 *
 *  @return                 MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status woal_start_acs_scan(moal_private *priv)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_ds_bss *bss = NULL;
	mlan_ioctl_req *req = NULL;

	ENTER();

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_bss));
	if (req == NULL) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	bss = (mlan_ds_bss *)req->pbuf;
	bss->sub_command = MLAN_OID_UAP_ACS_SCAN;
	req->req_id = MLAN_IOCTL_BSS;
	req->action = MLAN_ACT_SET;

	ret = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (ret != MLAN_STATUS_SUCCESS)
		goto done;
	PRINTM(MIOCTL,
	       "ACS scan done: bandcfg:[chanBand=0x%x chanWidth=0x%x chan2Offset=0x%x scanMode=0x%x], channel=%d\n",
	       bss->param.ap_acs_scan.bandcfg.chanBand,
	       bss->param.ap_acs_scan.bandcfg.chanWidth,
	       bss->param.ap_acs_scan.bandcfg.chan2Offset,
	       bss->param.ap_acs_scan.bandcfg.scanMode,
	       bss->param.ap_acs_scan.chan);
done:
	if (ret != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brie check if we need do ACS scan
 *
 *  @param priv             A pointer to moal_private structure
 *
 *  @return                 MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status woal_do_acs_check(moal_private *priv)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_uap_bss_param *sys_config = NULL;
	mlan_uap_scan_channels *scan_channels = NULL;
	chan_band_info uap_channel;
	ENTER();

	sys_config = kzalloc(sizeof(mlan_uap_bss_param), GFP_ATOMIC);
	if (!sys_config) {
		PRINTM(MERROR, "Fail to alloc memory for mlan_uap_bss_param\n");
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}

	if (MLAN_STATUS_SUCCESS != woal_set_get_sys_config(priv, MLAN_ACT_GET,
							   MOAL_IOCTL_WAIT,
							   sys_config)) {
		PRINTM(MERROR, "Fail to get sys config data\n");
		kfree(sys_config);
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}
	if (!(sys_config->bandcfg.scanMode == SCAN_MODE_ACS)) {
		kfree(sys_config);
		LEAVE();
		return ret;
	}
	scan_channels = kzalloc(sizeof(mlan_uap_scan_channels), GFP_ATOMIC);
	if (scan_channels == NULL) {
		PRINTM(MERROR, "Fail to alloc scan channels buffer\n");
		kfree(sys_config);
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}
	scan_channels->remove_nop_channel = MTRUE;
	if (MLAN_STATUS_SUCCESS !=
	    woal_set_get_ap_scan_channels(priv, MLAN_ACT_GET, scan_channels)) {
		PRINTM(MERROR, "Fail to get scan channels\n");
		goto done;
	}

	if (scan_channels->num_remvoed_channel && scan_channels->num_of_chan) {
		scan_channels->remove_nop_channel = 0;
		/** set back new channel list after remove nop channels */
		if (MLAN_STATUS_SUCCESS !=
		    woal_set_get_ap_scan_channels(priv, MLAN_ACT_SET,
						  scan_channels)) {
			PRINTM(MERROR, "Fail to get scan channels\n");
			goto done;
		}
	}
	if (scan_channels->num_of_chan)
		ret = woal_start_acs_scan(priv);
	else
		ret = MLAN_STATUS_FAILURE;
	/** set to default channel 6 when 5G ACS is configured */
	if ((ret != MLAN_STATUS_SUCCESS) &&
	    (sys_config->bandcfg.chanBand == BAND_5GHZ)) {
		memset(&uap_channel, 0, sizeof(uap_channel));
		uap_channel.bandcfg.chanBand = DEFAULT_UAP_BAND;
		uap_channel.channel = DEFAULT_UAP_CHANNEL;
		ret = woal_set_get_ap_channel(priv, MLAN_ACT_SET,
					      MOAL_IOCTL_WAIT, &uap_channel);
	}
done:
	kfree(scan_channels);
	kfree(sys_config);
	LEAVE();
	return ret;
}

/**
 * @brief Set default configurations pre BSS START
 *
 * @param            A pointer to moal_private
 *
 * @return           0 --success, otherwise fail
 */
static mlan_status woal_uap_set_pre_confing(moal_private *priv)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;

#if 0 // DMCS enable/disable by parameter in development phase.
	ENTER();

	if (priv->phandle->params.wacp_mode) {
		/** Enable DMCS, if wacp_mode enabled */
		if (MLAN_STATUS_SUCCESS != woal_set_dmcs(priv, 1, MOAL_IOCTL_WAIT)) {
			PRINTM(MERROR, "Set DMCS failed\n");
			ret = MLAN_STATUS_FAILURE;
			goto done;
		}
	}

done:
#endif
	LEAVE();
	return ret;
}

#define BF_CFG_ACT_SET 1

/**
 * @brief Set default configurations post BSS START
 *
 * @param            A pointer to moal_private
 *
 * @return           0 --success, otherwise fail
 */
static mlan_status woal_uap_set_post_confing(moal_private *priv)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	t_u16 prot_mode;
	mlan_ds_11n_aggr_prio_tbl aggr_prio_tbl;
	mlan_ds_11n_addba_param addba_param;
	mlan_ds_11n_tx_bf_cfg bf_cfg;
	mlan_bf_global_cfg_args *bf_global = NULL;
	int i;
	t_u8 addba_reject[MAX_NUM_TID];
	t_u8 tos_to_tid_inv[] = {0x02, 0x00, 0x01, 0x03,
				 0x04, 0x05, 0x06, 0x07};
	t_u8 set_htc_cap;
	t_u32 tx_bf_cap = 0;

	ENTER();

	if (priv->phandle->params.wacp_mode) {
		/** Enable AMPDU/AMSDU, if wacp_mode enabled */
		for (i = 0; i < MAX_NUM_TID; i++) {
			aggr_prio_tbl.ampdu[i] = tos_to_tid_inv[i];
			aggr_prio_tbl.amsdu[i] = tos_to_tid_inv[i];
		}
		if (MLAN_STATUS_SUCCESS !=
		    woal_ioctl_aggr_prio_tbl(priv, MLAN_ACT_SET,
					     &aggr_prio_tbl)) {
			PRINTM(MERROR, "Set AMPDU/AMSDU failed\n");
			ret = MLAN_STATUS_FAILURE;
			goto done;
		}

		/** Disable addbareject, if wacp_mode enabled */
		for (i = 0; i < MAX_NUM_TID; i++) {
			addba_reject[i] = MFALSE;
		}
		if (MLAN_STATUS_SUCCESS !=
		    woal_ioctl_addba_reject(priv, MLAN_ACT_SET, addba_reject)) {
			PRINTM(MERROR, "Set addbareject failed\n");
			ret = MLAN_STATUS_FAILURE;
			goto done;
		}

		/** Set addbapara, if wacp_mode enabled */
		memset(&addba_param, 0, sizeof(addba_param));
		addba_param.timeout = 65535;
		addba_param.txwinsize = 64;
		addba_param.rxwinsize = 64;
		addba_param.txamsdu = 0;
		addba_param.rxamsdu = 0;
		if (MLAN_STATUS_SUCCESS !=
		    woal_ioctl_addba_param(priv, MLAN_ACT_SET, &addba_param)) {
			PRINTM(MERROR, "Set addba_param failed\n");
			ret = MLAN_STATUS_FAILURE;
			goto done;
		}

#if defined(STA_CFG80211) || defined(UAP_CFG80211)
		/** Enable ed_mac, if wacp_mode enabled */
		if (MLAN_STATUS_SUCCESS !=
		    woal_edmac_cfg(priv, priv->phandle->country_code)) {
			PRINTM(MERROR, "Enable edmac failed\n");
			ret = MLAN_STATUS_FAILURE;
			goto done;
		}
#endif

		/** Set AMPDU protect mode to TX_AMPDU_RTS_CTS, if wacp_mode
		 * enabled */
		prot_mode = TX_AMPDU_RTS_CTS;
		if (MLAN_STATUS_SUCCESS !=
		    woal_ioctl_tx_ampdu_prot_mode(priv, MLAN_ACT_SET,
						  &prot_mode)) {
			PRINTM(MERROR, "Set AMPDU protect mode failed\n");
			ret = MLAN_STATUS_FAILURE;
			goto done;
		}

		/** Enable HTC CAP if wacp_mode enabled */
		set_htc_cap = 1;
		if (MLAN_STATUS_SUCCESS !=
		    woal_ioctl_hostcmd_htc_cap(priv, MLAN_ACT_SET,
					       &set_htc_cap)) {
			PRINTM(MERROR, "Set Hostcmd htc_cap failed\n");
			ret = MLAN_STATUS_FAILURE;
			goto done;
		}

		/** Disable beamforming and sounding if wacp_mode enabled */
		woal_set_get_tx_bf_cap(priv, MLAN_ACT_GET, &tx_bf_cap);
		if (tx_bf_cap) {
			memset(&bf_cfg, 0, sizeof(bf_cfg));
			/* Pointer to corresponding buffer */
			bf_global = &bf_cfg.body.bf_global_cfg;
			bf_cfg.action = BF_CFG_ACT_SET;
			bf_global->bf_enbl = 0; /* Disable beamforming */
			bf_global->sounding_enbl = 0; /* Disable sounding */
			bf_global->fb_type = 3; /* Set FB type to 3 */
			bf_global->snr_threshold = 10; /* Set snr threshold to
							  10 */
			bf_global->sounding_interval = 500; /* Set sounding
							       interval to 500
							       ms */
			bf_global->bf_mode = 5; /* Set beamforming mode to 5 */
			if (MLAN_STATUS_SUCCESS !=
			    woal_set_get_tx_bf_cfg(priv, MLAN_ACT_SET,
						   &bf_cfg)) {
				PRINTM(MERROR, "Set TX beamforming failed\n");
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
 *  @brief uap BSS control ioctl handler
 *
 *  @param priv             A pointer to moal_private structure
 *  @param wait_option      Wait option
 *  @param data             BSS control type
 *  @return                 0 --success, otherwise fail
 */
int woal_uap_bss_ctrl(moal_private *priv, t_u8 wait_option, int data)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_bss *bss = NULL;
	int ret = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	PRINTM(MIOCTL, "ioctl bss ctrl=%d\n", data);

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_bss));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}
	bss = (mlan_ds_bss *)req->pbuf;
	switch (data) {
	case UAP_BSS_START:
		if (priv->bss_started == MTRUE) {
			PRINTM(MWARN, "Warning: BSS already started!\n");
			/* goto done; */
		} else if (!priv->uap_host_based
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 14, 0)
			   || moal_extflg_isset(priv->phandle, EXT_DFS_OFFLOAD)
#endif
		) {
			status = woal_do_acs_check(priv);
			if (status)
				PRINTM(MMSG, "woal_do_acs_check fails\n");
			/* about to start bss: issue channel check */
			status = woal_11h_channel_check_ioctl(priv,
							      MOAL_IOCTL_WAIT);
			if (status) {
				PRINTM(MMSG, "11h channel check fails\n");
				status = MLAN_STATUS_FAILURE;
				ret = -1;
				goto done;
			}
		}
		/* DMCS and other that do not use FW cmd directly configurations
		 */
		if (MLAN_STATUS_SUCCESS != woal_uap_set_pre_confing(priv)) {
			PRINTM(MERROR,
			       "Set WACP Pre default configurations failed\n");
			ret = -EFAULT;
			goto done;
		}

		bss->sub_command = MLAN_OID_BSS_START;
		if (priv->uap_host_based) {
			bss->param.host_based |= UAP_FLAG_HOST_BASED;
#ifdef UAP_CFG80211
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
			if (moal_extflg_isset(priv->phandle, EXT_HOST_MLME))
				bss->param.host_based |= UAP_FLAG_HOST_MLME;
#endif
#endif
		}
		break;
	case UAP_BSS_STOP:
		if (priv->bss_started == MFALSE) {
			PRINTM(MWARN, "Warning: BSS already stopped!\n");
			/* This is a situation where CAC it started and BSS
			 * start is dealyed and before CAC timer expires BSS
			 * stop is triggered.
			 *
			 * Do not skip sending the BSS_STOP command since there
			 * are many routines triggered on BSS_STOP command
			 * response.
			 */
			woal_cancel_cac_block(priv);
		}
		bss->sub_command = MLAN_OID_BSS_STOP;
		break;
	case UAP_BSS_RESET:
		bss->sub_command = MLAN_OID_UAP_BSS_RESET;
		woal_cancel_cac_block(priv);
		break;
	default:
		PRINTM(MMSG, "We don't support this uap_bss_ctrl cmd %d\n",
		       data);
		ret = -EFAULT;
		goto done;
	}
	req->req_id = MLAN_IOCTL_BSS;
	req->action = MLAN_ACT_SET;

	status = woal_request_ioctl(priv, req, wait_option);
	if (status == MLAN_STATUS_FAILURE) {
		ret = -EFAULT;
		goto done;
	}
	if (data == UAP_BSS_STOP || data == UAP_BSS_RESET) {
		priv->bss_started = MFALSE;
		woal_stop_queue(priv->netdev);
		if (netif_carrier_ok(priv->netdev))
			netif_carrier_off(priv->netdev);
		if (data == UAP_BSS_RESET) {
			if (MLAN_STATUS_FAILURE ==
			    woal_request_set_mac_address(priv, wait_option))
				PRINTM(MERROR,
				       "Fail to set mac address after UAP_BSS_RESET\n");
		}
		woal_flush_tx_stat_queue(priv);
		woal_flush_tcp_sess_queue(priv);
	} else if (data == UAP_BSS_START) {
		/* Due to the influence of DMCS, the configurations using FW cmd
		 * must wait until BSS START. */
		if (MLAN_STATUS_SUCCESS != woal_uap_set_post_confing(priv))
			PRINTM(MERROR,
			       "Set WACP Post default configurations failed\n");
	}
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief This function sets multicast addresses to firmware
 *
 *  @param dev     A pointer to net_device structure
 *  @return        N/A
 */
void woal_uap_set_multicast_list(struct net_device *dev)
{
	ENTER();

	LEAVE();
}

/**
 *  @brief ioctl function - entry point
 *
 *  @param dev      A pointer to net_device structure
 *  @param req      A pointer to ifreq structure
 *  @param cmd      Command
 *
 *  @return         0 --success, otherwise fail
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0)
int woal_uap_do_ioctl(struct net_device *dev, struct ifreq *req,
		      void __user *data, int cmd)
#else
int woal_uap_do_ioctl(struct net_device *dev, struct ifreq *req, int cmd)
#endif
{
	int ret = 0;
	ENTER();
#if 0
#ifdef CONFIG_COMPAT
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0)
	if (in_compat_syscall()) /* not implemented yet */
		return -EOPNOTSUPP;
#endif
#endif
#endif

	switch (cmd) {
	case WOAL_ANDROID_DEF_CMD:
		/** android default ioctl ID is SIOCDEVPRIVATE + 1 */
		ret = woal_android_priv_cmd(dev, req);
		break;
	case UAP_IOCTL_CMD:
		ret = woal_uap_ioctl(dev, req);
		break;
	case UAP_POWER_MODE:
		ret = woal_uap_power_mode_ioctl(dev, req);
		break;
	case UAP_BSS_CTRL:
		ret = woal_uap_bss_ctrl_ioctl(dev, req);
		break;
	case UAP_WAPI_MSG:
		ret = woal_uap_set_wapi(dev, req);
		break;
	case UAP_BSS_CONFIG:
		ret = woal_uap_bss_cfg_ioctl(dev, req);
		break;
	case UAP_STA_DEAUTH:
		ret = woal_uap_sta_deauth_ioctl(dev, req);
		break;
	case UAP_RADIO_CTL:
		ret = woal_uap_radio_ctl(dev, req);
		break;
	case UAP_REPORT_MIC_ERR:
		ret = woal_uap_report_mic_ioctl(dev, req);
		break;
	case UAP_SET_KEY:
		ret = woal_uap_set_key_ioctl(dev, req);
		break;
	case UAPHOSTPKTINJECT:
		ret = woal_send_host_packet(dev, req);
		break;
	case UAP_GET_STA_LIST:
		ret = woal_uap_get_sta_list_ioctl(dev, req);
		break;
	case UAP_CUSTOM_IE:
		ret = woal_custom_ie_ioctl(dev, req);
		break;
	case UAP_GET_BSS_TYPE:
		ret = woal_get_bss_type(dev, req);
		break;
	case WOAL_ANDROID_PRIV_CMD:
		ret = woal_android_priv_cmd(dev, req);
		break;
	default:
#ifdef UAP_WEXT
		ret = woal_uap_do_priv_ioctl(dev, req, cmd);
#else
		ret = -EOPNOTSUPP;
#endif
		break;
	}

	LEAVE();
	return ret;
}

#ifdef CONFIG_PROC_FS
/**
 *  @brief Get version
 *
 *  @param priv         A pointer to moal_private structure
 *  @param version      A pointer to version buffer
 *  @param max_len      max length of version buffer
 *
 *  @return             N/A
 */
void woal_uap_get_version(moal_private *priv, char *version, int max_len)
{
	mlan_ds_get_info *info = NULL;
	mlan_ioctl_req *req = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_get_info));
	if (req == NULL) {
		LEAVE();
		return;
	}

	info = (mlan_ds_get_info *)req->pbuf;
	info->sub_command = MLAN_OID_GET_VER_EXT;
	req->req_id = MLAN_IOCTL_GET_INFO;
	req->action = MLAN_ACT_GET;

	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status == MLAN_STATUS_SUCCESS) {
		PRINTM(MINFO, "MOAL UAP VERSION: %s\n",
		       info->param.ver_ext.version_str);
		snprintf(version, max_len, priv->phandle->driver_version,
			 info->param.ver_ext.version_str);
	}

	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return;
}
#endif

/**
 *  @brief Get uap statistics
 *
 *  @param priv                 A pointer to moal_private structure
 *  @param wait_option          Wait option
 *  @param ustats               A pointer to mlan_ds_uap_stats structure
 *
 *  @return                     MLAN_STATUS_SUCCESS/MLAN_STATUS_PENDING --
 * success, otherwise fail
 */
mlan_status woal_uap_get_stats(moal_private *priv, t_u8 wait_option,
			       mlan_ds_uap_stats *ustats)
{
	mlan_ds_get_info *info = NULL;
	mlan_ioctl_req *req = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_get_info));
	if (req == NULL) {
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}

	info = (mlan_ds_get_info *)req->pbuf;
	info->sub_command = MLAN_OID_GET_STATS;
	req->req_id = MLAN_IOCTL_GET_INFO;
	req->action = MLAN_ACT_GET;

	status = woal_request_ioctl(priv, req, wait_option);
	if (status == MLAN_STATUS_SUCCESS) {
		if (ustats)
			moal_memcpy_ext(priv->phandle, ustats,
					&info->param.ustats,
					sizeof(mlan_ds_uap_stats),
					sizeof(mlan_ds_uap_stats));
#ifdef UAP_WEXT
		priv->w_stats.discard.fragment =
			info->param.ustats.fcs_error_count;
		priv->w_stats.discard.retries = info->param.ustats.retry_count;
		priv->w_stats.discard.misc =
			info->param.ustats.ack_failure_count;
#endif
	}

	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return status;
}

/**
 *  @brief Set/Get system configuration parameters
 *
 *  @param priv             A pointer to moal_private structure
 *  @param action           MLAN_ACT_SET or MLAN_ACT_GET
 *  @param ap_wmm_para      A pointer to wmm_parameter_t structure
 *
 *  @return                 MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status woal_set_get_ap_wmm_para(moal_private *priv, t_u16 action,
				     wmm_parameter_t *ap_wmm_para)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_ds_bss *bss = NULL;
	mlan_ioctl_req *req = NULL;

	ENTER();

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_bss));
	if (req == NULL) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	bss = (mlan_ds_bss *)req->pbuf;
	bss->sub_command = MLAN_OID_UAP_CFG_WMM_PARAM;
	req->req_id = MLAN_IOCTL_BSS;
	req->action = action;

	if (action == MLAN_ACT_SET)
		moal_memcpy_ext(priv->phandle, &bss->param.ap_wmm_para,
				ap_wmm_para, sizeof(wmm_parameter_t),
				sizeof(bss->param.ap_wmm_para));

	ret = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (ret != MLAN_STATUS_SUCCESS)
		goto done;
	if (bss->param.ap_wmm_para.reserved != MLAN_STATUS_COMPLETE) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	if (action == MLAN_ACT_GET)
		moal_memcpy_ext(priv->phandle, ap_wmm_para,
				&bss->param.ap_wmm_para,
				sizeof(wmm_parameter_t),
				sizeof(wmm_parameter_t));

done:
	if (ret != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Set/Get system configuration parameters
 *
 *  @param priv             A pointer to moal_private structure
 *  @param action           MLAN_ACT_SET or MLAN_ACT_GET
 *  @param wait_option      Wait option
 *  @param sys_cfg          A pointer to mlan_uap_bss_param structure
 *
 *  @return                 MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status woal_set_get_sys_config(moal_private *priv, t_u16 action,
				    t_u8 wait_option,
				    mlan_uap_bss_param *sys_cfg)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_ds_bss *bss = NULL;
	mlan_ioctl_req *req = NULL;

	ENTER();

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_bss));
	if (req == NULL) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	bss = (mlan_ds_bss *)req->pbuf;
	bss->sub_command = MLAN_OID_UAP_BSS_CONFIG;
	req->req_id = MLAN_IOCTL_BSS;
	req->action = action;

	if (action == MLAN_ACT_SET)
		moal_memcpy_ext(priv->phandle, &bss->param.bss_config, sys_cfg,
				sizeof(mlan_uap_bss_param),
				sizeof(bss->param.bss_config));

	ret = woal_request_ioctl(priv, req, wait_option);
	if (ret != MLAN_STATUS_SUCCESS)
		goto done;

	if (action == MLAN_ACT_GET)
		moal_memcpy_ext(priv->phandle, sys_cfg, &bss->param.bss_config,
				sizeof(mlan_uap_bss_param),
				sizeof(mlan_uap_bss_param));

done:
	if (ret != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Set invalid data for each member of mlan_uap_bss_param
 *  structure
 *
 *  @param config   A pointer to mlan_uap_bss_param structure
 *
 *  @return         N/A
 */
void woal_set_sys_config_invalid_data(mlan_uap_bss_param *config)
{
	ENTER();

	memset(config, 0, sizeof(mlan_uap_bss_param));
	config->bcast_ssid_ctl = 0x7F;
	config->radio_ctl = 0x7F;
	config->dtim_period = 0x7F;
	config->beacon_period = 0x7FFF;
	config->tx_data_rate = 0x7FFF;
	config->mcbc_data_rate = 0x7FFF;
	config->tx_power_level = 0x7F;
	config->tx_antenna = 0x7F;
	config->rx_antenna = 0x7F;
	config->pkt_forward_ctl = 0x7F;
	config->max_sta_count = 0x7FFF;
	config->auth_mode = 0x7F;
	config->sta_ageout_timer = 0x7FFFFFFF;
	config->pairwise_update_timeout = 0x7FFFFFFF;
	config->pwk_retries = 0x7FFFFFFF;
	config->groupwise_update_timeout = 0x7FFFFFFF;
	config->gwk_retries = 0x7FFFFFFF;
	config->mgmt_ie_passthru_mask = 0x7FFFFFFF;
	config->ps_sta_ageout_timer = 0x7FFFFFFF;
	config->rts_threshold = 0x7FFF;
	config->frag_threshold = 0x7FFF;
	config->retry_limit = 0x7FFF;
	config->filter.filter_mode = 0x7FFF;
	config->filter.mac_count = 0x7FFF;
	config->wpa_cfg.rsn_protection = 0x7F;
	config->wpa_cfg.gk_rekey_time = 0x7FFFFFFF;
	config->enable_2040coex = 0x7F;
	config->wmm_para.qos_info = 0x7F;

	LEAVE();
}
