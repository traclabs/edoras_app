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
** File: gateway_app_events.h
**
** Purpose:
**  Define Rover App Events IDs
**
** Notes:
**
*************************************************************************/
#ifndef _gateway_app_events_h_
#define _gateway_app_events_h_

#define GATEWAY_APP_RESERVED_EID          0
#define GATEWAY_APP_STARTUP_INF_EID       1
#define GATEWAY_APP_COMMAND_ERR_EID       2
#define GATEWAY_APP_COMMANDNOP_INF_EID    3
#define GATEWAY_APP_COMMANDTWIST_INF_EID    4
#define GATEWAY_APP_INVALID_MSGID_ERR_EID 5
#define GATEWAY_APP_LEN_ERR_EID           6
#define GATEWAY_APP_PIPE_ERR_EID          7

#define GATEWAY_APP_EVENT_COUNTS 7

#endif /* _gateway_app_events_h_ */

/************************/
/*  End of File Comment */
/************************/
