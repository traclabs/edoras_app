/*******************************************************************************
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
** File: edoras_app.h
**
** Purpose:
**   This file is main hdr file for the ros application.
**
**
*******************************************************************************/

#ifndef _edoras_app_h_
#define _edoras_app_h_

/*
** Required header files.
*/
#include "cfe.h"
#include "cfe_error.h"
#include "cfe_evs.h"
#include "cfe_sb.h"
#include "cfe_es.h"

#include "edoras_app_perfids.h"
#include "edoras_app_msgids.h"
#include "edoras_app_msg.h"

#include <edoras_core/interface.h>

/***********************************************************************/
#define EDORAS_APP_PIPE_DEPTH 32 /* Depth of the Command Pipe for Application */
/************************************************************************
** Type Definitions
*************************************************************************/

/*
** Global Data
*/
typedef struct 
{
const TypeInfo_t * ti; 
void* ts_library;
const TypeSupport_t* ts;
const char* interface_type;
const char* interface_name;
} ParseData_t;

typedef struct
{
    /*
    ** Command interface counters...
    */
    uint8 CmdCounter;
    uint8 ErrCounter;

    uint32 square_counter;
    uint32 hk_counter;
   
    // Run Status variable used in the main processing loop
    uint32 RunStatus;

    // Operational data (not reported in housekeeping)
    CFE_SB_PipeId_t CommandPipe;

    // Keep note of updates
    bool rcvd_cmd_1;
    bool rcvd_cmd_2;

    // Initialization data (not reported in housekeeping)...
    char   PipeName[CFE_MISSION_MAX_API_LEN];
    uint16 PipeDepth;

    CFE_EVS_BinFilter_t EventFilters[EDORAS_APP_EVENT_COUNTS];

} EdorasAppData_t;

/****************************************************************************/
/*
** Local function prototypes.
**
** Note: Except for the entry point (EdorasAppMain), these
**       functions are not called from any other source module.
*/
void  EdorasAppMain(void);

int32 EdorasAppInit(void);

void  EdorasAppProcessCommandPacket(CFE_SB_Buffer_t *SBBufPtr);
void  EdorasAppProcessGroundCmd(CFE_SB_Buffer_t *SBBufPtr, int _robot_id);
void EdorasAppProcessFlightTlm(CFE_SB_Buffer_t *SBBufPtr, int _robot_id);
int32 EdorasAppReportHousekeeping(void);

bool EdorasAppVerifyCmdLength(CFE_MSG_Message_t *MsgPtr, size_t ExpectedLength);


#endif /* _edoras_app_h_ */
