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
** File: gateway_app.c
**
** Purpose:
**   This file contains the source code for the ros App.
**
*******************************************************************************/

/*
** Include Files:
*/
#include "gateway_app_events.h"
#include "gateway_app_version.h"
#include "gateway_app.h"
#include "gateway_app_table.h"

#include <string.h>

#include <math.h>

/*
** global data
*/
GatewayAppData_t GatewayAppData;
GatewayAppOdometry_t lastOdomMsg;

void HighRateControLoop(void);

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  * *  * * * * **/
/* GatewayAppMain() -- Application entry point and main process loop         */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  * *  * * * * **/
void GatewayAppMain(void)
{
    int32            status;
    CFE_SB_Buffer_t *SBBufPtr;

    /*
    ** Create the first Performance Log entry
    */
    CFE_ES_PerfLogEntry(GATEWAY_APP_PERF_ID);

    /*
    ** Perform application specific initialization
    ** If the Initialization fails, set the RunStatus to
    ** CFE_ES_RunStatus_APP_ERROR and the App will not enter the RunLoop
    */
    status = GatewayAppInit();

    if (status != CFE_SUCCESS)
    {
        GatewayAppData.RunStatus = CFE_ES_RunStatus_APP_ERROR;
    }

    /*
    ** Runloop
    */
    while (CFE_ES_RunLoop(&GatewayAppData.RunStatus) == true)
    {   printf("Loop is running in Gateway ----------------------- \n");
        /*
        ** Performance Log Exit Stamp
        */
        CFE_ES_PerfLogExit(GATEWAY_APP_PERF_ID);

        /* Pend on receipt of command packet */
        status = CFE_SB_ReceiveBuffer(&SBBufPtr, GatewayAppData.CommandPipe, CFE_SB_PEND_FOREVER);

        if (status == CFE_SUCCESS)
        {
            GatewayAppProcessCommandPacket(SBBufPtr);
        }
        else
        {
            CFE_EVS_SendEvent(GATEWAY_APP_PIPE_ERR_EID, CFE_EVS_EventType_ERROR,
                              "Edoras App: SB Pipe Read Error, App Will Exit");

            GatewayAppData.RunStatus = CFE_ES_RunStatus_APP_ERROR;
        }

        /*
        ** Performance Log Entry Stamp
        */
        CFE_ES_PerfLogEntry(GATEWAY_APP_PERF_ID);
    }
    printf("GATEWAY Loop was ended **************************** \n");
    /*
    ** Performance Log Exit Stamp
    */
    CFE_ES_PerfLogExit(GATEWAY_APP_PERF_ID);

    CFE_ES_ExitApp(GatewayAppData.RunStatus);

} /* End of GatewayAppMain() */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  */
/*                                                                            */
/* GatewayAppInit() --  initialization                                       */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
int32 GatewayAppInit(void)
{
    int32 status;

    GatewayAppData.RunStatus = CFE_ES_RunStatus_APP_RUN;

    /*
    ** Initialize app command execution counters
    */
    GatewayAppData.CmdCounter = 0;
    GatewayAppData.ErrCounter = 0;
    GatewayAppData.square_counter = 0;
    GatewayAppData.hk_counter = 0;

    GatewayAppData.HkTlm.Payload.state.pose.x = 0.0;
    GatewayAppData.HkTlm.Payload.state.pose.y = 0.0;
    GatewayAppData.HkTlm.Payload.state.pose.z = 0.0;
    GatewayAppData.HkTlm.Payload.state.pose.qx = 0.0;
    GatewayAppData.HkTlm.Payload.state.pose.qy = 0.0;
    GatewayAppData.HkTlm.Payload.state.pose.qz = 0.0;
    GatewayAppData.HkTlm.Payload.state.pose.qw = 0.0;

    /*
    ** Initialize app configuration data
    */
    GatewayAppData.PipeDepth = GATEWAY_APP_PIPE_DEPTH;

    strncpy(GatewayAppData.PipeName, "GATEWAY_APP_PIPE", sizeof(GatewayAppData.PipeName));
    GatewayAppData.PipeName[sizeof(GatewayAppData.PipeName) - 1] = 0;

    /*
    ** Initialize event filter table...
    */
    GatewayAppData.EventFilters[0].EventID = GATEWAY_APP_STARTUP_INF_EID;
    GatewayAppData.EventFilters[0].Mask    = 0x0000;
    GatewayAppData.EventFilters[1].EventID = GATEWAY_APP_COMMAND_ERR_EID;
    GatewayAppData.EventFilters[1].Mask    = 0x0000;
    GatewayAppData.EventFilters[2].EventID = GATEWAY_APP_COMMANDNOP_INF_EID;
    GatewayAppData.EventFilters[2].Mask    = 0x0000;
    GatewayAppData.EventFilters[3].EventID = GATEWAY_APP_COMMANDTWIST_INF_EID;
    GatewayAppData.EventFilters[3].Mask    = 0x0000;
    GatewayAppData.EventFilters[4].EventID = GATEWAY_APP_INVALID_MSGID_ERR_EID;
    GatewayAppData.EventFilters[4].Mask    = 0x0000;
    GatewayAppData.EventFilters[5].EventID = GATEWAY_APP_LEN_ERR_EID;
    GatewayAppData.EventFilters[5].Mask    = 0x0000;
    GatewayAppData.EventFilters[6].EventID = GATEWAY_APP_PIPE_ERR_EID;
    GatewayAppData.EventFilters[6].Mask    = 0x0000;

    status = CFE_EVS_Register(GatewayAppData.EventFilters, GATEWAY_APP_EVENT_COUNTS, CFE_EVS_EventFilter_BINARY);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("GatewayApp: Error Registering Events, RC = 0x%08lX\n", (unsigned long)status);
        return (status);
    }

    /*
    ** Initialize housekeeping packet (clear user data area).
    */
    CFE_MSG_Init(&GatewayAppData.HkTlm.TlmHeader.Msg, CFE_SB_ValueToMsgId(GATEWAY_APP_HK_TLM_MID), sizeof(GatewayAppData.HkTlm));
    CFE_MSG_Init(&GatewayAppData.LastTwist.TlmHeader.Msg, CFE_SB_ValueToMsgId(GATEWAY_APP_TLM_TWIST_MID), sizeof(GatewayAppData.LastTwist));

    /*
    ** Create Software Bus message pipe.
    */
    status = CFE_SB_CreatePipe(&GatewayAppData.CommandPipe, GatewayAppData.PipeDepth, GatewayAppData.PipeName);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("Edoras App: Error creating pipe, RC = 0x%08lX\n", (unsigned long)status);
        return (status);
    }

    /*
    ** Subscribe to Housekeeping request commands
    */
    status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(GATEWAY_APP_SEND_HK_MID), GatewayAppData.CommandPipe);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("Edoras App: Error Subscribing to HK request, RC = 0x%08lX\n", (unsigned long)status);
        return (status);
    }

    /*
    ** Subscribe to ground command packets
    */
    status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(GATEWAY_APP_CMD_MID), GatewayAppData.CommandPipe);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("Edoras App: Error Subscribing to Command, RC = 0x%08lX\n", (unsigned long)status);

        return (status);
    }


    /*
    ** Subscribe to flight odom data
    */
    status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(GATEWAY_APP_CMD_ODOM_MID), GatewayAppData.CommandPipe);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("Edoras App: Error Subscribing to Odom data, RC = 0x%08lX\n", (unsigned long)status);

        return (status);
    }

    
    /*
    ** Subscribe to HR wakeup
    */
    status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(GATEWAY_APP_HR_CONTROL_MID), GatewayAppData.CommandPipe);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("Edoras App: Error Subscribing to HR Wakeup Command, RC = 0x%08lX\n", (unsigned long)status);

        return (status);
    }

    CFE_EVS_SendEvent(GATEWAY_APP_STARTUP_INF_EID, CFE_EVS_EventType_INFORMATION, "Edoras App Initialized.%s",
                      GATEWAY_APP_VERSION_STRING);

    return (CFE_SUCCESS);

} /* End of GatewayAppInit() */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*  Name:  GatewayAppProcessCommandPacket                                    */
/*                                                                            */
/*  Purpose:                                                                  */
/*     This routine will process any packet that is received on the ros    */
/*     command pipe.                                                          */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * *  * * * * * * *  * *  * * * * */
void GatewayAppProcessCommandPacket(CFE_SB_Buffer_t *SBBufPtr)
{
    CFE_SB_MsgId_t MsgId = CFE_SB_INVALID_MSG_ID;

    CFE_MSG_GetMsgId(&SBBufPtr->Msg, &MsgId);
    printf("GatewayAppProcessCommandPacket() -- we're processing the cmd from MID: 0x%04x\n", CFE_SB_MsgIdToValue(MsgId));
    switch (CFE_SB_MsgIdToValue(MsgId))
    {
        case GATEWAY_APP_CMD_MID:
            GatewayAppProcessGroundCommand(SBBufPtr);
            break;

        case GATEWAY_APP_SEND_HK_MID:
            GatewayAppReportHousekeeping((CFE_MSG_CommandHeader_t *)SBBufPtr);
            break;

        case GATEWAY_APP_CMD_ODOM_MID:
            GatewayAppProcessFlightOdom(SBBufPtr);
            break;

        case GATEWAY_APP_HR_CONTROL_MID:
            HighRateControLoop();
            break;
            
        default:
            CFE_EVS_SendEvent(GATEWAY_APP_INVALID_MSGID_ERR_EID, CFE_EVS_EventType_ERROR,
                              "Edoras App: invalid command packet,MID = 0x%x", (unsigned int)CFE_SB_MsgIdToValue(MsgId));
            break;
    }

    return;

} /* End GatewayAppProcessCommandPacket */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/* GatewayAppProcessGroundCommand() -- Edoras App ground commands                */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
void GatewayAppProcessGroundCommand(CFE_SB_Buffer_t *SBBufPtr)
{
    CFE_MSG_FcnCode_t CommandCode = 0;

    CFE_MSG_GetFcnCode(&SBBufPtr->Msg, &CommandCode);

    printf("GatewayAppProcessGroundCommand() -- we're getting a ground command...%d\n", CommandCode);

    /*
    ** Process "known" Edoras App ground commands
    */
    switch (CommandCode)
    {
        case GATEWAY_APP_NOOP_CC:
            if (GatewayAppVerifyCmdLength(&SBBufPtr->Msg, sizeof(GatewayAppNoopCmd_t)))
            {
                GatewayAppNoop((GatewayAppNoopCmd_t *)SBBufPtr);
            }

            break;

        case GATEWAY_APP_SET_TWIST_CC:
            if (GatewayAppVerifyCmdLength(&SBBufPtr->Msg, sizeof(GatewayAppTwistCmd_t)))
            {
                GatewayAppCmdTwist((GatewayAppTwistCmd_t *)SBBufPtr);
            }

            break;

        /* default case already found during FC vs length test */
        default:
            CFE_EVS_SendEvent(GATEWAY_APP_COMMAND_ERR_EID, CFE_EVS_EventType_ERROR,
                              "Invalid ground command code: CC = %d", CommandCode);
            break;
    }


    return;

} /* End of GatewayAppProcessFlightOdom() */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/* GatewayAppProcessFlightOdom() -- Edoras App flight odometry                   */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
void GatewayAppProcessFlightOdom(CFE_SB_Buffer_t *SBBufPtr)
{
    CFE_MSG_FcnCode_t CommandCode = 0;

    CFE_MSG_GetFcnCode(&SBBufPtr->Msg, &CommandCode);

    printf("GatewayAppProcessGroundCommand() -- we're getting a flight odometry message ...%d\n", CommandCode);

    // Read
    if (GatewayAppVerifyCmdLength(&SBBufPtr->Msg, sizeof(GatewayAppCmdRobotState_t)))
    {
       GatewayAppCmdRobotState_t* state = (GatewayAppCmdRobotState_t *)SBBufPtr;
       
       // Fill the lastState
       lastOdomMsg = state->odom;                     
    }


    return;

} /* End of GatewayAppProcessFlightCommand() */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*  Name:  GatewayAppReportHousekeeping                                          */
/*                                                                            */
/*  Purpose:                                                                  */
/*         This function is triggered in response to a task telemetry request */
/*         from the housekeeping task. This function will gather the Apps     */
/*         telemetry, packetize it and send it to the housekeeping task via   */
/*         the software bus                                                   */
/* * * * * * * * * * * * * * * * * * * * * * * *  * * * * * * *  * *  * * * * */
int32 GatewayAppReportHousekeeping(const CFE_MSG_CommandHeader_t *Msg)
{
    printf("GatewayAppReportHousekeeping() -- sending odom as part of housekeeping...\n");
    
    /*
    ** Get command execution counters...
    */
    GatewayAppData.HkTlm.Payload.CommandErrorCounter = GatewayAppData.ErrCounter*2;
    GatewayAppData.ErrCounter++;
    GatewayAppData.HkTlm.Payload.CommandCounter      = GatewayAppData.CmdCounter++;

    OS_printf("GatewayAppReportHousekeeping reporting: %d\n", GatewayAppData.HkTlm.Payload.CommandCounter);

 
    CFE_SB_TimeStampMsg(&GatewayAppData.HkTlm.TlmHeader.Msg);
    CFE_SB_TransmitMsg(&GatewayAppData.HkTlm.TlmHeader.Msg, true);

    return CFE_SUCCESS;

} /* End of GatewayAppReportHousekeeping() */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/* GatewayAppNoop -- ROS NOOP commands                                          */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
int32 GatewayAppNoop(const GatewayAppNoopCmd_t *Msg)
{
    CFE_EVS_SendEvent(GATEWAY_APP_COMMANDNOP_INF_EID, CFE_EVS_EventType_INFORMATION, "Edoras App: NOOP command %s",
                      GATEWAY_APP_VERSION);

    return CFE_SUCCESS;
} /* End of GatewayAppNoop */


int32 GatewayAppCmdTwist(const GatewayAppTwistCmd_t *Msg)
{
    GatewayAppData.LastTwist.twist.linear_x = Msg->twist.linear_x;
    GatewayAppData.LastTwist.twist.linear_y = Msg->twist.linear_y; 
    GatewayAppData.LastTwist.twist.linear_z = Msg->twist.linear_z;
    GatewayAppData.LastTwist.twist.angular_x = Msg->twist.angular_x;
    GatewayAppData.LastTwist.twist.angular_y = Msg->twist.angular_y;
    GatewayAppData.LastTwist.twist.angular_z = Msg->twist.angular_z;


    CFE_EVS_SendEvent(GATEWAY_APP_COMMANDTWIST_INF_EID, CFE_EVS_EventType_INFORMATION, "Edoras App: twist command %s",
                      GATEWAY_APP_VERSION);

    return CFE_SUCCESS;
    
}

void HighRateControLoop(void) {
    
    // 1. Publish the twist to State in rosfsw (it is like sending a command to the robot)
    // (we should use another name, telemetry is not supposed to command anything)

    // if (GatewayAppData.square_counter%1000 == 0)    
    {
    CFE_SB_TimeStampMsg(&GatewayAppData.LastTwist.TlmHeader.Msg);
    CFE_SB_TransmitMsg(&GatewayAppData.LastTwist.TlmHeader.Msg, true);    
    }

 
    
    // 2. Update the telemetry information        
    GatewayAppOdometry_t *st = &lastOdomMsg; //GatewayAppGoal.StateTlm;

    GatewayAppData.HkTlm.Payload.state.pose.x = st->pose.x;
    GatewayAppData.HkTlm.Payload.state.pose.y = st->pose.y;
    GatewayAppData.HkTlm.Payload.state.pose.z = st->pose.z;
    GatewayAppData.HkTlm.Payload.state.pose.qx = st->pose.qx;
    GatewayAppData.HkTlm.Payload.state.pose.qy = st->pose.qy;
    GatewayAppData.HkTlm.Payload.state.pose.qz = st->pose.qz;
    GatewayAppData.HkTlm.Payload.state.pose.qw = st->pose.qw;

    GatewayAppData.HkTlm.Payload.state.twist.linear_x = st->twist.linear_x;
    GatewayAppData.HkTlm.Payload.state.twist.linear_y = st->twist.linear_y;
    GatewayAppData.HkTlm.Payload.state.twist.linear_z = st->twist.linear_z;
    GatewayAppData.HkTlm.Payload.state.twist.angular_x = st->twist.angular_x;
    GatewayAppData.HkTlm.Payload.state.twist.angular_y = st->twist.angular_y;
    GatewayAppData.HkTlm.Payload.state.twist.angular_z = st->twist.angular_z;                

    // This data is sent when a Housekeeping request is received, 
    // (usually, at a low rate) so nothing sent here
    //memcpy(&st->joints, &GatewayAppData.HkTlm.Payload.state, sizeof(GatewayAppSSRMS_t) );
    
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/* GatewayAppVerifyCmdLength() -- Verify command packet length                   */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
bool GatewayAppVerifyCmdLength(CFE_MSG_Message_t *MsgPtr, size_t ExpectedLength)
{
    bool              result       = true;
    size_t            ActualLength = 0;
    CFE_SB_MsgId_t    MsgId        = CFE_SB_INVALID_MSG_ID;
    CFE_MSG_FcnCode_t FcnCode      = 0;

    printf("GatewayAppVerifyCmdLength() --\n");

    CFE_MSG_GetSize(MsgPtr, &ActualLength);

    /*
    ** Verify the command packet length.
    */
    if (ExpectedLength != ActualLength)
    {
        CFE_MSG_GetMsgId(MsgPtr, &MsgId);
        CFE_MSG_GetFcnCode(MsgPtr, &FcnCode);

        CFE_EVS_SendEvent(GATEWAY_APP_LEN_ERR_EID, CFE_EVS_EventType_ERROR,
                          "Invalid Msg length: ID = 0x%X,  CC = %u, Len = %u, Expected = %u",
                          (unsigned int)CFE_SB_MsgIdToValue(MsgId), (unsigned int)FcnCode, (unsigned int)ActualLength,
                          (unsigned int)ExpectedLength);

        result = false;

        GatewayAppData.ErrCounter++;
    }

    return (result);

} /* End of GatewayAppVerifyCmdLength() */
