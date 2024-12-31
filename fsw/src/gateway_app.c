/*******************************************************************************
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

#include "debug_utils_c_version.h"


#include "serialize_library.h"
#include "robot_comm_udp_test.h"

#define ROBOT_PORT 8585
#define CFS_PORT 8080


/*
** global data
*/
GatewayAppData_t GatewayAppData;
GatewayAppOdometry_t lastOdomMsg;

CommData_t commData;

ParseData_t parse_pose_;
ParseData_t parse_twist_;

typedef struct
{
    CFE_MSG_TelemetryHeader_t  TlmHeader;
    uint8_t data[56];
} PoseData_t;

PoseData_t tlm_pose;

void HighRateControlLoop(void);

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
    // 0: CFE_ES_RunStatus_UNDEFINED, 1: CFE_ES_RunStatus_APP_RUN, 2: CFE_ES_RunStatus_APP_EXIT, 3: CFE_ES_RunStatus_APP_ERROR
    status = GatewayAppInit();
    if (status != CFE_SUCCESS)
    {   printf("Setting run status to be error !!!!!!XXXXXXXXXXXXX!!!!!!!\n");
        GatewayAppData.RunStatus = CFE_ES_RunStatus_APP_ERROR;
    }

    // Start comm
    if(!setupComm(&commData, CFS_PORT, ROBOT_PORT))
    {
       perror("Error setting up communication to the robot using sockets");
    }

    // Run loop
    while (CFE_ES_RunLoop(&GatewayAppData.RunStatus) == true)
    {   printf("Loop is running in Gateway ----------------------- \n");

        // Performance Log Exit Stamp
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
            printf("RUN STATUS APP ERROR!!!!!! ********* !!!!!!!!!!! \n");   
            GatewayAppData.RunStatus = CFE_ES_RunStatus_APP_ERROR;
        }

        /*
        ** Performance Log Entry Stamp
        */
        CFE_ES_PerfLogEntry(GATEWAY_APP_PERF_ID);
    }
    printf("GATEWAY Loop was ended **************************** \n");
    // Performance Log Exit Stamp
    CFE_ES_PerfLogExit(GATEWAY_APP_PERF_ID);

    CFE_ES_ExitApp(GatewayAppData.RunStatus);

} /* End of GatewayAppMain() */

void initializeParseData(const char* _interface_name, const char* _interface_type, ParseData_t *_parse_data )
{
    _parse_data->interface_type = _interface_name;
    _parse_data->interface_name = _interface_type;
    _parse_data->ti = get_type_info(_parse_data->interface_type, _parse_data->interface_name);

    _parse_data->ts_library = get_type_support_library(_parse_data->interface_type, _parse_data->interface_name);
    _parse_data->ts = get_type_support(_parse_data->interface_type, _parse_data->interface_name, _parse_data->ts_library);
}

size_t getSizeWithTlmHeader(ParseData_t *_parse_data)
{
   size_t size_msg = _parse_data->ti->size_of_;
   size_t size_tlm_hdr = sizeof(CFE_MSG_TelemetryHeader_t);
   printf("Size msg: %ld size tlm hdr: %ld \n", size_msg, size_tlm_hdr);
   return size_msg + size_tlm_hdr;
}

/**
 * @function GatewayAppInit
 * @brief Initialize app
 */
int32 GatewayAppInit(void)
{
    // Init ROS message stuff
    initializeParseData("geometry_msgs", "Pose", &parse_pose_);
    initializeParseData("geometry_msgs", "Twist", &parse_twist_);

    int32 status;

    GatewayAppData.RunStatus = CFE_ES_RunStatus_APP_RUN;

    // Initialize app command execution counters
    GatewayAppData.CmdCounter = 0;
    GatewayAppData.ErrCounter = 0;
    GatewayAppData.square_counter = 0;
    GatewayAppData.hk_counter = 0;

    GatewayAppData.HkTlm.Payload.state.pose.x = 0.0;

    // Initialize app configuration data
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
    
    // Initialize housekeeping packet (clear user data area).
    CFE_MSG_Init(&GatewayAppData.HkTlm.TlmHeader.Msg, CFE_SB_ValueToMsgId(GATEWAY_APP_HK_TLM_MID), sizeof(GatewayAppData.HkTlm));

    CFE_MSG_Init(&tlm_pose.TlmHeader.Msg, CFE_SB_ValueToMsgId(GATEWAY_APP_TLM_MID), sizeof(tlm_pose) );

    // Create Software Bus message pipe.
    status = CFE_SB_CreatePipe(&GatewayAppData.CommandPipe, GatewayAppData.PipeDepth, GatewayAppData.PipeName);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("Edoras App: Error creating pipe, RC = 0x%08lX\n", (unsigned long)status);
        return (status);
    }

    // Subscribe to Housekeeping request commands
    status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(GATEWAY_APP_SEND_HK_MID), GatewayAppData.CommandPipe);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("Edoras App: Error Subscribing to HK request, RC = 0x%08lX\n", (unsigned long)status);
        return (status);
    }

    // Subscribe to ground command packets
    status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(GATEWAY_APP_CMD_MID), GatewayAppData.CommandPipe);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("Edoras App: Error Subscribing to Command, RC = 0x%08lX\n", (unsigned long)status);

        return (status);
    }

    // Subscribe to flight odom data
    status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(GATEWAY_APP_CMD_ODOM_MID), GatewayAppData.CommandPipe);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("Edoras App: Error Subscribing to Odom data, RC = 0x%08lX\n", (unsigned long)status);

        return (status);
    }

    
    // Subscribe to HR wakeup
    status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(GATEWAY_APP_HR_CONTROL_MID), GatewayAppData.CommandPipe);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("Edoras App: Error Subscribing to HR Wakeup Command, RC = 0x%08lX\n", (unsigned long)status);

        return (status);
    }

    CFE_EVS_SendEvent(GATEWAY_APP_STARTUP_INF_EID, CFE_EVS_EventType_INFORMATION, "Edoras App Initialized.%s",
                      GATEWAY_APP_VERSION_STRING);
    printf("DEBUG -- Returning cfe success from init... \n");
    return (CFE_SUCCESS);

} 


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
            HighRateControlLoop();
            break;
            
        default:
            CFE_EVS_SendEvent(GATEWAY_APP_INVALID_MSGID_ERR_EID, CFE_EVS_EventType_ERROR,
                              "Edoras App: invalid command packet,MID = 0x%x", (unsigned int)CFE_SB_MsgIdToValue(MsgId));
            break;
    }

    return;

}


/**                                   
 * @function GatewayAppProcessGroundCommand
 * @brief Edoras App ground commands               
 */
void GatewayAppProcessGroundCommand(CFE_SB_Buffer_t *SBBufPtr)
{
    CFE_MSG_FcnCode_t CommandCode = 0;
    CFE_MSG_GetFcnCode(&SBBufPtr->Msg, &CommandCode);

    printf("GatewayAppProcessGroundCommand() -- we're getting a ground command...%d\n", CommandCode);

    // Process "known" Edoras App ground commands
    switch (CommandCode)
    {
        case GATEWAY_APP_NOOP_CC:
            printf("Noop...\n");
            if (GatewayAppVerifyCmdLength(&SBBufPtr->Msg, sizeof(GatewayAppNoopCmd_t)))
            {
                GatewayAppNoop((GatewayAppNoopCmd_t *)SBBufPtr);
            }

            break;

        case GATEWAY_APP_SET_TWIST_CC:
        {
            printf("Receiving a Twist command....\n");
            //if (GatewayAppVerifyCmdLength(&SBBufPtr->Msg, sizeof(GatewayAppTwistCmd_t)))
            //{
              // Let's see if we can deserialize
              
            // You know the first 8 bytes are the header
            size_t offset = 0;
            unsigned char header[8];
            memcpy(&header, SBBufPtr + offset, sizeof(header));
            printf("Got command data!: Deserializing: header :D :D :D: %02x, %02x, %02x, %02x, %02x, %02x, %02x, %02x \n", 
                   header[0], header[1], header[2], header[3], header[4], header[5], header[6], header[7]);
  
            size_t actual_length = 0;
            CFE_MSG_GetSize(&SBBufPtr->Msg, &actual_length);
            printf("***************** Actual length of command: %ld. Minus header: %ld \n", actual_length, actual_length - 8);
        
        // Parse the information  
        offset = 8;
        //uint8_t data_buffer[actual_length - offset];
        //memcpy(&data_buffer, SBBufPtr + offset, actual_length - offset); 
        //printf("Coping %ld bytes to data_buffer \n", actual_length - offset);
        
        size_t buffer_length_test;
        memcpy(&buffer_length_test, (uint8_t*)SBBufPtr + offset, sizeof(size_t));
        printf("Reading raw: buffer length test: %ld \n", buffer_length_test);
        uint8_t* msg_pointer;
        offset = 8;
        size_t buffer_size;
        msg_pointer = from_uint_buffer_to_msg_pointer( (uint8_t*)SBBufPtr, offset, parse_twist_.ts, parse_twist_.ti, &buffer_size);
        
        // Get data
        double vel_lin, vel_ang;
        get_float64(msg_pointer, parse_twist_.ti, "linear.x", &vel_lin);
        get_float64(msg_pointer, parse_twist_.ti, "angular.z", &vel_ang);        
        //debug_parse_buffer(msg_pointer, parse_twist_.ti);
        printf("Reading linear velocity: %f and angular : %f \n", vel_lin, vel_ang);
        
        
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

void HighRateControlLoop(void) {
    
    // 1. Publish the twist to State in rosfsw (it is like sending a command to the robot)
    // (we should use another name, telemetry is not supposed to command anything)

    { 
     // Update telemetry data
     uint8_t* pose_msg = create_msg(parse_pose_.ti);

     // Fill data
     set_float64(pose_msg, parse_pose_.ti, "position.x", 0.1);
     set_float64(pose_msg, parse_pose_.ti, "position.y", 0.2);
     set_float64(pose_msg, parse_pose_.ti, "position.z", 0.3);

     set_float64(pose_msg, parse_pose_.ti, "orientation.z", 0.7);
     set_float64(pose_msg, parse_pose_.ti, "orientation.w", 0.7);

     debug_parse_buffer(pose_msg, parse_pose_.ti);

     // Convert data to serialized version
     uint8_t* test_tlm_data = NULL;
    from_msg_pointer_to_uint_buffer(pose_msg, parse_pose_.ts, parse_pose_.ti, test_tlm_data);
    CFE_SB_TimeStampMsg(&tlm_pose.TlmHeader.Msg);
    CFE_SB_TransmitMsg(&tlm_pose.TlmHeader.Msg, true);    
    }
    
    // 2. Update the telemetry information        
    GatewayAppOdometry_t *st = &lastOdomMsg; //GatewayAppGoal.StateTlm;

    GatewayAppData.HkTlm.Payload.state.pose.x = st->pose.x;
    GatewayAppData.HkTlm.Payload.state.pose.y = st->pose.y;

    GatewayAppData.HkTlm.Payload.state.twist.linear_x = st->twist.linear_x;
    GatewayAppData.HkTlm.Payload.state.twist.linear_y = st->twist.linear_y;

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
