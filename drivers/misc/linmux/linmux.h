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
// Linmux.h
//
// This file contains global definitions and prototypes for the linmux
// driver.
//
//////////////////////////////////////////////////////////////////////////////


#ifndef __LINMUX_H
#define __LINMUX_H


#include "baseport.h"

//////////////////////////////////////////////////////////////////////////////

#define LINMUX_PLATFORM_NAME     "LinMux"

//////////////////////////////////////////////////////////////////////////////

int IncOpenCount(PT_BASEPORT pBasePort);
void DecOpenCount(PT_BASEPORT pBasePort);
int mux_fs_init(void);
void mux_fs_exit(void);

#endif // __LINMUX_H

