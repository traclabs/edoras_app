/************************************************************************
**
**      GSC-18128-1, "Core Flight Executive Version 6.7"
**
**      Copyright (c) 2006-2019 United States Government as represented by
**      the Administrator of the National Aeronautics and Space Administration.
**      All Rights Reserved.
**
**      Licensed under the Apache License, Version 2.0 (the "License");
**      you may not use this file except in compliance with the License.
**      You may obtain a copy of the License at
**
**        http://www.apache.org/licenses/LICENSE-2.0
**
**      Unless required by applicable law or agreed to in writing, software
**      distributed under the License is distributed on an "AS IS" BASIS,
**      WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
**      See the License for the specific language governing permissions and
**      limitations under the License.
**
** File: edoras_app_msgids.h
**
** Purpose:
**  Define Edoras App Message IDs
**
** Notes:
**
**
*************************************************************************/
#ifndef _edoras_app_msgids_h_
#define _edoras_app_msgids_h_

#include "cfe_msgids.h"

// CMD_MID_BASE

#define EDORAS_APP_TWIST_1_GROUND_MID     (CFE_PLATFORM_CMD_MID_BASE + 0x27)
#define EDORAS_APP_TWIST_2_GROUND_MID (CFE_PLATFORM_CMD_MID_BASE + 0x28)

#define EDORAS_APP_POSE_1_FLIGHT_MID     (CFE_PLATFORM_CMD_MID_BASE + 0x29)
#define EDORAS_APP_POSE_2_FLIGHT_MID (CFE_PLATFORM_CMD_MID_BASE + 0x2A)

#define EDORAS_APP_SEND_HK_MID (CFE_PLATFORM_CMD_MID_BASE + 0x2B) // Send data back to Earth

// TLM_MID_BASE

//#define EDORAS_APP_HK_TLM_MID      (CFE_PLATFORM_TLM_MID_BASE + 0x26)
#define EDORAS_APP_HR_CONTROL_MID  (CFE_PLATFORM_TLM_MID_BASE + 0x27)

#define EDORAS_APP_POSE_1_GROUND_MID   (CFE_PLATFORM_TLM_MID_BASE + 0x28)
#define EDORAS_APP_POSE_2_GROUND_MID   (CFE_PLATFORM_TLM_MID_BASE + 0x29)

#define EDORAS_APP_TWIST_1_FLIGHT_MID   (CFE_PLATFORM_TLM_MID_BASE + 0x2A)
#define EDORAS_APP_TWIST_2_FLIGHT_MID   (CFE_PLATFORM_TLM_MID_BASE + 0x2B)


#endif /* _edoras_app_msgids_h_ */

/*********************************/
/* End of File Comment           */
/*********************************/
