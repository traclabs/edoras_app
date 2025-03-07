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
** File: edoras_app_msg.h
**
** Purpose:
**  Define Edoras App Messages and info
**
** Notes:
**
**
*******************************************************************************/
#ifndef _edoras_app_msg_h_
#define _edoras_app_msg_h_

/**
 * EdorasApp command codes
 */
#define EDORAS_APP_NOOP_CC        0
#define EDORAS_APP_SET_TWIST_CC   1

/*************************************************************************/

/*
** Type definition (generic "no arguments" command)
*/
typedef struct 
{
   CFE_MSG_CommandHeader_t CmdHeader;
} EdorasAppNoArgsCmd_t;

typedef struct
{
   float linear_x;
   float linear_y;
   float linear_z;
   
   float angular_x;
   float angular_y;
   float angular_z;
} EdorasAppTwist_t;

typedef struct
{
   float x;
   float y;
   float z;
   
   float qx;
   float qy;
   float qz;
   float qw;   
} EdorasAppPose_t;

typedef struct
{
  EdorasAppPose_t pose;
  EdorasAppTwist_t twist;
  
} EdorasAppOdometry_t;

typedef struct
{
   CFE_MSG_CommandHeader_t CmdHeader; // 8
   EdorasAppTwist_t twist; // 6 floats = 6x4 = 24
} EdorasAppTwistCmd_t;

/*
** The following commands all share the "NoArgs" format
**
** They are each given their own type name matching the command name, which
** allows them to change independently in the future without changing the prototype
** of the handler function
*/
typedef EdorasAppNoArgsCmd_t EdorasAppNoopCmd_t;
//typedef EdorasAppTwistCmd_t  EdorasAppTwistStateCmd_t;

/*************************************************************************/

typedef struct
{
    uint8 CommandErrorCounter;
    uint8 CommandCounter;
    EdorasAppOdometry_t state;
} EdorasAppHkTlmPayload_t;

typedef struct
{
    CFE_MSG_TelemetryHeader_t  TlmHeader; /**< \brief Telemetry header */
    EdorasAppHkTlmPayload_t Payload;   /**< \brief Telemetry payload */
} EdorasAppHkTlm_t;


// These 2 messages are for communication with the robot on FSW side
typedef struct
{
    CFE_MSG_TelemetryHeader_t  TlmHeader; /**< \brief Telemetry header */
    EdorasAppTwist_t twist; /**< Twist currently being applied **/

} EdorasAppTlmRobotCommand_t;

typedef struct
{
    CFE_MSG_CommandHeader_t  CmdHeader; /**< \brief Command header */
    EdorasAppOdometry_t odom; /**< Twist the robot is currently using **/

} EdorasAppCmdRobotState_t;


#endif /* _edoras_app_msg_h_ */

/************************/
/*  End of File Comment */
/************************/



















