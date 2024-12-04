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
** File: gateway_app_msg.h
**
** Purpose:
**  Define Gateway App Messages and info
**
** Notes:
**
**
*******************************************************************************/
#ifndef _gateway_app_msg_h_
#define _gateway_app_msg_h_

/**
 * GatewayApp command codes
 */
#define GATEWAY_APP_NOOP_CC        0
#define GATEWAY_APP_SET_TWIST_CC   1

/*************************************************************************/

/*
** Type definition (generic "no arguments" command)
*/
typedef struct 
{
   CFE_MSG_CommandHeader_t CmdHeader;
} GatewayAppNoArgsCmd_t;

typedef struct
{
   float linear_x;
   float linear_y;
   float linear_z;
   
   float angular_x;
   float angular_y;
   float angular_z;
} GatewayAppTwist_t;

typedef struct
{
   float x;
   float y;
   float z;
   
   float qx;
   float qy;
   float qz;
   float qw;   
} GatewayAppPose_t;

typedef struct
{
  GatewayAppPose_t pose;
  GatewayAppTwist_t twist;
  
} GatewayAppOdometry_t;

typedef struct
{
   CFE_MSG_CommandHeader_t CmdHeader;
   GatewayAppTwist_t twist;
} GatewayAppTwistCmd_t;

/*
** The following commands all share the "NoArgs" format
**
** They are each given their own type name matching the command name, which
** allows them to change independently in the future without changing the prototype
** of the handler function
*/
typedef GatewayAppNoArgsCmd_t GatewayAppNoopCmd_t;
//typedef GatewayAppTwistCmd_t  GatewayAppTwistStateCmd_t;

/*************************************************************************/

typedef struct
{
    uint8 CommandErrorCounter;
    uint8 CommandCounter;
    GatewayAppOdometry_t state;
} GatewayAppHkTlmPayload_t;

typedef struct
{
    CFE_MSG_TelemetryHeader_t  TlmHeader; /**< \brief Telemetry header */
    GatewayAppHkTlmPayload_t Payload;   /**< \brief Telemetry payload */
} GatewayAppHkTlm_t;


// These 2 messages are for communication with the robot on FSW side
typedef struct
{
    CFE_MSG_TelemetryHeader_t  TlmHeader; /**< \brief Telemetry header */
    GatewayAppTwist_t twist; /**< Twist currently being applied **/

} GatewayAppTlmRobotCommand_t;

typedef struct
{
    CFE_MSG_CommandHeader_t  CmdHeader; /**< \brief Command header */
    GatewayAppOdometry_t odom; /**< Twist the robot is currently using **/

} GatewayAppCmdRobotState_t;


#endif /* _gateway_app_msg_h_ */

/************************/
/*  End of File Comment */
/************************/



















