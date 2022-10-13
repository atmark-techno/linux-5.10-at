/******************************************************************************

 Copyright (C) 2022 THALES DIS AIS Deutschland GmbH <CinterionWMSupport@thalesgroup.com>
 Company name change from Gemalto M2M GmbH to THALES DIS AIS Deutschland GmbH
 Copyright (C) 2013 Gemalto M2M GmbH

 All Rights Reserved.

 Gemalto provides this source code under the GPL v2 License.
 The GPL v2 license is available at

 https://opensource.org/licenses/gpl-license.php

******************************************************************************/

//////////////////////////////////////////////////////////////////////////////
//
// Linmuxpwr.c
//
// This file contains the implementation of the linmux power management
// interface.
//
//////////////////////////////////////////////////////////////////////////////

#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/module.h>

#include "linmux.h"
#include "muxdbg.h"
#include "baseport.h"
#include "linmuxtty.h"

//////////////////////////////////////////////////////////////////////////////
// Power management callbacks
//////////////////////////////////////////////////////////////////////////////

int mux_pm_suspend(struct device *dev);
int mux_pm_resume(struct device *dev);
int mux_pm_freeze(struct device *dev);
int mux_pm_restore(struct device *dev);
void mux_pm_shutdown(struct platform_device *pdev);

//////////////////////////////////////////////////////////////////////////////

static const struct dev_pm_ops if_pm_ops = {
  .suspend  = mux_pm_suspend,
  .resume   = mux_pm_resume,
  .freeze   = mux_pm_freeze,
  .restore  = mux_pm_restore,
};

//////////////////////////////////////////////////////////////////////////////

static struct of_device_id linmux_dt_ids[] = {
  { .compatible = "linmux" },
  { }
};

struct platform_driver MuxPlatformDriver = {
  .probe = mux_serial_probe,
  .remove = mux_serial_remove,
  .driver = {
    .name  = LINMUX_PLATFORM_NAME,
    .pm    = &if_pm_ops,
    .owner = THIS_MODULE,
    .of_match_table = of_match_ptr(linmux_dt_ids),
  },
  .shutdown = mux_pm_shutdown,
};

MODULE_ALIAS("platform:linmux");
MODULE_DEVICE_TABLE(of, linmux_dt_ids);

//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
//
// The release callback function of the device. It is called when the device
// is unregistered. Handling of the base port is completely managed via the
// virtual multiplex ports. So we're leaving this function empty. Because
// the device manager throws an exception when the devices callback pointer
// is left empty we have to define it nevertheless.
//
// Parameters:
// dev: The corresponding platform device.
//
// Return:
// None.
//
//////////////////////////////////////////////////////////////////////////////
void mux_dev_release(struct device *dev) {
}

//////////////////////////////////////////////////////////////////////////////
//
// Callback function for system supsend to RAM. We lock access to the base
// port and close it.
//
// Parameters:
// dev: The corresponding platform device.
//
// Return:
// 0 in case of success, a negative error value in case of an error.
//
//////////////////////////////////////////////////////////////////////////////
int mux_pm_suspend(struct device *dev) {
  DBG_ENTER(ZONE_FCT_POWER_MGM, "Dev=%p, PlatformData=%p", dev, dev->platform_data);
  bp_Suspend((PT_BASEPORT)dev->platform_data);
  DBG_LEAVE(ZONE_FCT_POWER_MGM, "Ret=0, Dev=%p", dev);
  return 0;
}

//////////////////////////////////////////////////////////////////////////////
//
// Callback function for system resume from RAM. We unlock access to the base
// port and reopen it.
//
// Parameters:
// dev: The corresponding platform device.
//
// Return:
// 0 in case of success, a negative error value in case of an error.
//
//////////////////////////////////////////////////////////////////////////////
int mux_pm_resume(struct device *dev) {
  DBG_ENTER(ZONE_FCT_POWER_MGM, "Dev=%p, PlatformData=%p", dev, dev->platform_data);
  bp_Resume((PT_BASEPORT)dev->platform_data);
  DBG_LEAVE(ZONE_FCT_POWER_MGM, "Ret=0, Dev=%p", dev);
  return 0;
}

//////////////////////////////////////////////////////////////////////////////
//
// Callback function for system supsend to disc (hybernate). We lock access
// to the base port and close it.
//
// Parameters:
// dev: The corresponding platform device.
//
// Return:
// 0 in case of success, a negative error value in case of an error.
//
//////////////////////////////////////////////////////////////////////////////
int mux_pm_freeze(struct device *dev) {
  DBG_ENTER(ZONE_FCT_POWER_MGM, "Dev=%p, PlatformData=%p", dev, dev->platform_data);
  bp_Suspend((PT_BASEPORT)dev->platform_data);
  DBG_LEAVE(ZONE_FCT_POWER_MGM, "Ret=0, Dev=%p", dev);
  return 0;
}

//////////////////////////////////////////////////////////////////////////////
//
// Callback function for system resume from disc (hybernate). We unlock
// access to the base port and reopen it.
//
// Parameters:
// dev: The corresponding platform device.
//
// Return:
// 0 in case of success, a negative error value in case of an error.
//
//////////////////////////////////////////////////////////////////////////////
int mux_pm_restore(struct device *dev) {
  DBG_ENTER(ZONE_FCT_POWER_MGM, "Dev=%p, PlatformData=%p", dev, dev->platform_data);
  bp_Resume((PT_BASEPORT)dev->platform_data);
  DBG_LEAVE(ZONE_FCT_POWER_MGM, "Ret=0, Dev=%p", dev);
  return 0;
}

//////////////////////////////////////////////////////////////////////////////
//
// Callback function for system shutdown. We emergancy shutdown muxer to
// release used resources.
//
// Parameters:
// pdev: The corresponding platform device.
//
// Return:
// None.
//
//////////////////////////////////////////////////////////////////////////////
void mux_pm_shutdown(struct platform_device *pdev) {
  DBG_ENTER(ZONE_FCT_POWER_MGM, "PlatformDev=%p, PlatformData=%p", pdev, pdev->dev.platform_data);
  bp_Shutdown((PT_BASEPORT)pdev->dev.platform_data);
  DBG_LEAVE(ZONE_FCT_POWER_MGM, "PlatformDev=%p", pdev);
}
