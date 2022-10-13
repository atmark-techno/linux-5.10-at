/******************************************************************************

 Copyright (C) 2022 THALES DIS AIS Deutschland GmbH <CinterionWMSupport@thalesgroup.com>
 Company name change from Gemalto M2M GmbH to THALES DIS AIS Deutschland GmbH
 Copyright (C) 2013 Gemalto M2M GmbH

 All Rights Reserved.

 Gemalto provides this source code under the GPL v2 License.
 The GPL v2 license is available at

 https://opensource.org/licenses/gpl-license.php

******************************************************************************/

#ifndef __MUX_MSGQUEUE_H
#define __MUX_MSGQUEUE_H

#include "global.h"
#include "ddmpglob.h"
#include "ddmpdlch.h"


#ifdef  __cplusplus
extern "C" {
#endif


enum {
    MUXMSG_OK           =   0,
    MUXMSG_QUEUE_EMPTY,
    MUXMSG_QUEUE_FULL
};

/*****                           Macros                                  *****/

/*****                          Typedefs                                 *****/


void MuxMsg_Init (
    MUX_INSTANCE_t      *pMux
);

DWORD MuxMsg_Get (
    MUX_INSTANCE_t      *pMux,
    MP_PRIMITIVE        *pMuxMsg
);

DWORD MuxMsg_Put (
    MUX_INSTANCE_t      *pMux,
    MP_PRIMITIVE        *pMuxMsg
);

BOOL MuxMsg_Exist(
    MUX_INSTANCE_t      *pMux
);


#ifdef  __cplusplus
}
#endif


#endif // __MUX_MSGQUEUE_H__
