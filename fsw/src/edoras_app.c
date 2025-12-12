/*******************************************************************************
**
** File: edoras_app.c
**
** Purpose:
**   This file contains the source code for the ros App.
**
*******************************************************************************/

#include "edoras_app_events.h"
#include "edoras_app_version.h"
#include "edoras_app.h"
#include "edoras_app_table.h"

#include <string.h>

#include <math.h>

#define MULTIHOST 

#define ROBOT_PORT 8585
#define CFS_PORT 8080

#ifdef MULTIHOST
#define ROBOT_IP "10.5.0.4" // rosfsw
#define CFS_IP "10.5.0.3"  // fsw
#else
#define ROBOT_IP "127.0.0.1" // rosfsw
#define CFS_IP "127.0.0.1"  // fsw
#endif

// Code that creates a socket and sends information from cFS to the robot
#include "robot_comm_udp_gateway_big_arm.h"

// Global data
EdorasAppData_t EdorasAppData;
CommData_t commData;

typedef struct
{
    CFE_MSG_TelemetryHeader_t  TlmHeader;
    float joints[7];
} JointStateData_t;

typedef struct
{
    CFE_MSG_CommandHeader_t  TlmHeader;
    float pos[3];
    float rot[4];
} EndEffectorCommandData_t;

JointStateData_t tlm_joint_state;
EndEffectorCommandData_t cmd_ee_pose;

void HighRateControlLoop(void);


/**
 * @brief Application entry point and main process loop
 */
void EdorasAppMain(void)
{
    int32            status;
    CFE_SB_Buffer_t *SBBufPtr;

    CFE_ES_PerfLogEntry(EDORAS_APP_PERF_ID);
    
    // Initialize
    status = EdorasAppInit();
    if (status != CFE_SUCCESS)
    {   perror("Setting run status to be error !");
        EdorasAppData.RunStatus = CFE_ES_RunStatus_APP_ERROR;
    }

    // Start communication with the executable that controls robot in ROS, using a socket
    CFE_ES_WriteToSysLog("Edoras App: Start comm, cfs port: %d ip: %s robot port: %d ip: %s ******* \n", 
                          CFS_PORT, CFS_IP, ROBOT_PORT, ROBOT_IP);
    if(!setupComm(&commData, CFS_PORT, ROBOT_PORT, CFS_IP, ROBOT_IP))
    {
       perror("Error setting up communication to the robot using sockets");
       EdorasAppData.RunStatus = CFE_ES_RunStatus_APP_ERROR;
    }

    // Run loop
    while (CFE_ES_RunLoop(&EdorasAppData.RunStatus) == true)
    {   
        // Performance Log Exit Stamp
        CFE_ES_PerfLogExit(EDORAS_APP_PERF_ID);

        // Pend on receipt of command packet
        status = CFE_SB_ReceiveBuffer(&SBBufPtr, EdorasAppData.CommandPipe, CFE_SB_PEND_FOREVER);

        if (status == CFE_SUCCESS)
        {
            EdorasAppProcessCommandPacket(SBBufPtr);
        }
        else
        {
            CFE_EVS_SendEvent(EDORAS_APP_PIPE_ERR_EID, CFE_EVS_EventType_ERROR,
                              "Edoras App: SB Pipe Read Error, App Will Exit");
            EdorasAppData.RunStatus = CFE_ES_RunStatus_APP_ERROR;
        }

        // Performance Log Entry Stamp
        CFE_ES_PerfLogEntry(EDORAS_APP_PERF_ID);
    }

    // Performance Log Exit Stamp
    CFE_ES_PerfLogExit(EDORAS_APP_PERF_ID);

    CFE_ES_ExitApp(EdorasAppData.RunStatus);
}

/**
 * @brief Initialize app
 */
int32 EdorasAppInit(void)
{
    int32 status;
    EdorasAppData.RunStatus = CFE_ES_RunStatus_APP_RUN;

    // Initialize app command execution counters
    EdorasAppData.CmdCounter = 0;
    EdorasAppData.ErrCounter = 0;
    EdorasAppData.square_counter = 0;
    EdorasAppData.hk_counter = 0;

    // Initialize app configuration data
    EdorasAppData.PipeDepth = EDORAS_APP_PIPE_DEPTH;

    strncpy(EdorasAppData.PipeName, "EDORAS_APP_PIPE", sizeof(EdorasAppData.PipeName));
    EdorasAppData.PipeName[sizeof(EdorasAppData.PipeName) - 1] = 0;

    // Initialize event filter table...
    EdorasAppData.EventFilters[0].EventID = EDORAS_APP_STARTUP_INF_EID;
    EdorasAppData.EventFilters[0].Mask    = 0x0000;
    EdorasAppData.EventFilters[1].EventID = EDORAS_APP_COMMAND_ERR_EID;
    EdorasAppData.EventFilters[1].Mask    = 0x0000;
    EdorasAppData.EventFilters[2].EventID = EDORAS_APP_COMMANDNOP_INF_EID;
    EdorasAppData.EventFilters[2].Mask    = 0x0000;
    EdorasAppData.EventFilters[3].EventID = EDORAS_APP_COMMANDCMD_INF_EID;
    EdorasAppData.EventFilters[3].Mask    = 0x0000;
    EdorasAppData.EventFilters[4].EventID = EDORAS_APP_INVALID_MSGID_ERR_EID;
    EdorasAppData.EventFilters[4].Mask    = 0x0000;
    EdorasAppData.EventFilters[5].EventID = EDORAS_APP_LEN_ERR_EID;
    EdorasAppData.EventFilters[5].Mask    = 0x0000;
    EdorasAppData.EventFilters[6].EventID = EDORAS_APP_PIPE_ERR_EID;
    EdorasAppData.EventFilters[6].Mask    = 0x0000;

    status = CFE_EVS_Register(EdorasAppData.EventFilters, EDORAS_APP_EVENT_COUNTS, CFE_EVS_EventFilter_BINARY);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("EdorasApp: Error Registering Events, RC = 0x%08lX\n", (unsigned long)status);
        return (status);
    }
    
    // Initialize housekeeping packet (clear user data area).
    CFE_MSG_Init(&EdorasAppData.HkTlm.TlmHeader.Msg, CFE_SB_ValueToMsgId(EDORAS_APP_HK_TLM_MID), sizeof(EdorasAppData.HkTlm));

    CFE_MSG_Init(&tlm_joint_state.TlmHeader.Msg, CFE_SB_ValueToMsgId(EDORAS_APP_TLM_MID), sizeof(tlm_joint_state) );

    // Create Software Bus message pipe.
    status = CFE_SB_CreatePipe(&EdorasAppData.CommandPipe, EdorasAppData.PipeDepth, EdorasAppData.PipeName);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("Edoras App: Error creating pipe, RC = 0x%08lX\n", (unsigned long)status);
        return (status);
    }

    // Subscribe to Housekeeping request commands
    status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(EDORAS_APP_SEND_HK_MID), EdorasAppData.CommandPipe);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("Edoras App: Error Subscribing to HK request, RC = 0x%08lX\n", (unsigned long)status);
        return (status);
    }

    // Subscribe to ground command packets
    status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(EDORAS_APP_CMD_MID), EdorasAppData.CommandPipe);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("Edoras App: Error Subscribing to Command, RC = 0x%08lX\n", (unsigned long)status);

        return (status);
    }
    
    // Subscribe to HR wakeup
    status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(EDORAS_APP_HR_CONTROL_MID), EdorasAppData.CommandPipe);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("Edoras App: Error Subscribing to HR Wakeup Command, RC = 0x%08lX\n", (unsigned long)status);

        return (status);
    }

    CFE_EVS_SendEvent(EDORAS_APP_STARTUP_INF_EID, CFE_EVS_EventType_INFORMATION, "Edoras App Initialized.%s",
                      EDORAS_APP_VERSION_STRING);

    return (CFE_SUCCESS);
} 


/**
 * @function:  EdorasAppProcessCommandPacket
 */
void EdorasAppProcessCommandPacket(CFE_SB_Buffer_t *SBBufPtr)
{
    CFE_SB_MsgId_t MsgId = CFE_SB_INVALID_MSG_ID;

    CFE_MSG_GetMsgId(&SBBufPtr->Msg, &MsgId);
    switch (CFE_SB_MsgIdToValue(MsgId))
    {
        case EDORAS_APP_CMD_MID:
            EdorasAppProcessGroundCommand(SBBufPtr);
            break;

        case EDORAS_APP_SEND_HK_MID:
            EdorasAppReportHousekeeping((CFE_MSG_CommandHeader_t *)SBBufPtr);
            break;

        case EDORAS_APP_HR_CONTROL_MID:
            HighRateControlLoop();
            break;
            
        default:
            CFE_EVS_SendEvent(EDORAS_APP_INVALID_MSGID_ERR_EID, CFE_EVS_EventType_ERROR,
                              "Edoras App: invalid command packet,MID = 0x%x", (unsigned int)CFE_SB_MsgIdToValue(MsgId));
            break;
    }

    return;

}


/**                                   
 * @function EdorasAppProcessGroundCommand
 * @brief Edoras App ground commands               
 */
void EdorasAppProcessGroundCommand(CFE_SB_Buffer_t *SBBufPtr)
{
    CFE_MSG_FcnCode_t CommandCode = 0;
    CFE_MSG_GetFcnCode(&SBBufPtr->Msg, &CommandCode);

    // Process "known" Edoras App ground commands
    switch (CommandCode)
    {
        case EDORAS_APP_NOOP_CC:
            if (EdorasAppVerifyCmdLength(&SBBufPtr->Msg, sizeof(EdorasAppNoopCmd_t)))
            {
                EdorasAppNoop((EdorasAppNoopCmd_t *)SBBufPtr);
            }

            break;

        case EDORAS_APP_CMD_CC:
        {
            memcpy(&cmd_ee_pose, SBBufPtr, sizeof(cmd_ee_pose));
            OS_printf("Cmd ee pose: %f %f %f -- %f %f %f %f \n", 
              cmd_ee_pose.pos[0], cmd_ee_pose.pos[1], cmd_ee_pose.pos[2],
              cmd_ee_pose.rot[0], cmd_ee_pose.rot[1], cmd_ee_pose.rot[2], cmd_ee_pose.rot[3]);         
        
            // Send data to robot using the socket
            sendPoseCmd(&commData, cmd_ee_pose.pos, cmd_ee_pose.rot);
        }
             break;

        /* default case already found during FC vs length test */
        default:
            CFE_EVS_SendEvent(EDORAS_APP_COMMAND_ERR_EID, CFE_EVS_EventType_ERROR,
                              "Invalid ground command code: CC = %d", CommandCode);
            break;
    }


    return;

}

/**
 *  Name:  EdorasAppReportHousekeeping
 */
int32 EdorasAppReportHousekeeping(const CFE_MSG_CommandHeader_t *Msg)
{
    { 
     // Read telemetry, if any
     int32_t sec; uint32_t nanosec;
     if(!receiveJointStateTlm(&commData, tlm_joint_state.joints, &sec, &nanosec))
       return CFE_SUCCESS;

     OS_printf("Size of tlm header: %ld, size of Tlm data size: %ld. data: %f %f %f %f %f %f %f \n", 
            sizeof(CFE_MSG_TelemetryHeader_t), sizeof(tlm_joint_state), tlm_joint_state.joints[0], tlm_joint_state.joints[1], tlm_joint_state.joints[2], 
            tlm_joint_state.joints[3], tlm_joint_state.joints[4], 
            tlm_joint_state.joints[5], tlm_joint_state.joints[6]);
       
     // If data received from robot update telemetry data
     // to send back to ground    
      CFE_SB_TimeStampMsg(&tlm_joint_state.TlmHeader.Msg);
 
     // Send TELEMETRY back to ground (TransmitMsg)
     // update_header: If true, the sequence counter bit in the primary header will increase each time
     // If false, it will remain zero.
     bool update_header = true;
     CFE_SB_TransmitMsg(&tlm_joint_state.TlmHeader.Msg, update_header);      
    }
 
    /*
    ** Get command execution counters...
    */
    EdorasAppData.HkTlm.Payload.CommandErrorCounter = EdorasAppData.ErrCounter*2;
    EdorasAppData.ErrCounter++;
    EdorasAppData.HkTlm.Payload.CommandCounter      = EdorasAppData.CmdCounter++;

    OS_printf("EdorasAppReportHousekeeping reporting: %d\n", EdorasAppData.HkTlm.Payload.CommandCounter);
 
    CFE_SB_TimeStampMsg(&EdorasAppData.HkTlm.TlmHeader.Msg);
    CFE_SB_TransmitMsg(&EdorasAppData.HkTlm.TlmHeader.Msg, true);

    return CFE_SUCCESS;
}

/**
 * @brief noop command - does nothing
 */
int32 EdorasAppNoop(const EdorasAppNoopCmd_t *Msg)
{
    CFE_EVS_SendEvent(EDORAS_APP_COMMANDNOP_INF_EID, CFE_EVS_EventType_INFORMATION, "Edoras App: NOOP command %s",
                      EDORAS_APP_VERSION);

    return CFE_SUCCESS;
}


/**
 * @brief Not used in this example
 */
void HighRateControlLoop(void) {
        
    // 2. Update the telemetry information        
    //EdorasAppOdometry_t *st = &lastOdomMsg; //EdorasAppGoal.StateTlm;

    /*EdorasAppData.HkTlm.Payload.state.pose.x = st->pose.x;
    EdorasAppData.HkTlm.Payload.state.pose.y = st->pose.y;

    EdorasAppData.HkTlm.Payload.state.twist.linear_x = st->twist.linear_x;
    EdorasAppData.HkTlm.Payload.state.twist.linear_y = st->twist.linear_y;*/

    // This data is sent when a Housekeeping request is received, 
    // (usually, at a low rate) so nothing sent here
    //memcpy(&st->joints, &EdorasAppData.HkTlm.Payload.state, sizeof(EdorasAppSSRMS_t) );
    
}

/*                                                                            
 * @function EdorasAppVerifyCmdLength
 * @brief Verify command packet length
 */ 
bool EdorasAppVerifyCmdLength(CFE_MSG_Message_t *MsgPtr, size_t ExpectedLength)
{
    bool              result       = true;
    size_t            ActualLength = 0;
    CFE_SB_MsgId_t    MsgId        = CFE_SB_INVALID_MSG_ID;
    CFE_MSG_FcnCode_t FcnCode      = 0;

    CFE_MSG_GetSize(MsgPtr, &ActualLength);

    // Verify the command packet length.
    if (ExpectedLength != ActualLength)
    {
        CFE_MSG_GetMsgId(MsgPtr, &MsgId);
        CFE_MSG_GetFcnCode(MsgPtr, &FcnCode);

        CFE_EVS_SendEvent(EDORAS_APP_LEN_ERR_EID, CFE_EVS_EventType_ERROR,
                          "Invalid Msg length: ID = 0x%X,  CC = %u, Len = %u, Expected = %u",
                          (unsigned int)CFE_SB_MsgIdToValue(MsgId), (unsigned int)FcnCode, (unsigned int)ActualLength,
                          (unsigned int)ExpectedLength);

        result = false;

        EdorasAppData.ErrCounter++;
    }

    return (result);

}
