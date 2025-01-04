/*******************************************************************************
**
** File: edoras_app.c
**
** Purpose:
**   This file contains the source code for the ros App.
**
*******************************************************************************/

/*
** Include Files:
*/
#include "edoras_app_events.h"
#include "edoras_app_version.h"
#include "edoras_app.h"
#include "edoras_app_table.h"

#include <string.h>

#include <math.h>

// To communicate with the actual robot locally
#include "robot_comm_udp_gateway_big_arm.h"

#define ROBOT_PORT 8585
#define CFS_PORT 8080


CommData_t commData;

ParseData_t parse_pose_; // Receive command
ParseData_t parse_joint_state_; // Send telemetry

typedef struct
{
    CFE_MSG_TelemetryHeader_t  TlmHeader;
    uint8_t data[260]; // 56 for c ros struct, 76 serialized
} JointStateData_t;

JointStateData_t tlm_joint_state;

// To delete
EdorasAppData_t EdorasAppData;

void HighRateControlLoop(void);

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  * *  * * * * **/
/* EdorasAppMain() -- Application entry point and main process loop         */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  * *  * * * * **/
void EdorasAppMain(void)
{
    int32            status;
    CFE_SB_Buffer_t *SBBufPtr;

    CFE_ES_PerfLogEntry(EDORAS_APP_PERF_ID);

    /*
    ** Perform application specific initialization
    ** If the Initialization fails, set the RunStatus to
    ** CFE_ES_RunStatus_APP_ERROR and the App will not enter the RunLoop
    */
    // 0: CFE_ES_RunStatus_UNDEFINED, 1: CFE_ES_RunStatus_APP_RUN, 2: CFE_ES_RunStatus_APP_EXIT, 3: CFE_ES_RunStatus_APP_ERROR
    status = EdorasAppInit();
    if (status != CFE_SUCCESS)
    {   printf("Setting run status to be error !!!!!!XXXXXXXXXXXXX!!!!!!!\n");
        EdorasAppData.RunStatus = CFE_ES_RunStatus_APP_ERROR;
    }

    // Start comm
    if(!setupComm(&commData, CFS_PORT, ROBOT_PORT))
    {
       perror("Error setting up communication to the robot using sockets");
    }

    // Run loop
    while (CFE_ES_RunLoop(&EdorasAppData.RunStatus) == true)
    {   
        // Performance Log Exit Stamp
        CFE_ES_PerfLogExit(EDORAS_APP_PERF_ID);


        /* Pend on receipt of command packet */
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

        CFE_ES_PerfLogEntry(EDORAS_APP_PERF_ID);
    }

    // Performance Log Exit Stamp
    CFE_ES_PerfLogExit(EDORAS_APP_PERF_ID);

    CFE_ES_ExitApp(EdorasAppData.RunStatus);

}

/**
 * @function initializeParserData
 */
void initializeParseData(const char* _interface_name, const char* _interface_type, ParseData_t *_parse_data )
{
    _parse_data->interface_type = _interface_name;
    _parse_data->interface_name = _interface_type;
    _parse_data->ti = get_type_info(_parse_data->interface_type, _parse_data->interface_name);

    _parse_data->ts_library = get_type_support_library(_parse_data->interface_type, _parse_data->interface_name);
    _parse_data->ts = get_type_support(_parse_data->interface_type, _parse_data->interface_name, _parse_data->ts_library);
}

/**
 * @function getSizeWithTlmHeader
 */
size_t getSizeWithTlmHeader(ParseData_t *_parse_data)
{
   size_t size_msg = _parse_data->ti->size_of_;
   size_t size_tlm_hdr = sizeof(CFE_MSG_TelemetryHeader_t);
   printf("Size msg: %ld size tlm hdr: %ld \n", size_msg, size_tlm_hdr);
   return size_msg + size_tlm_hdr;
}

/**
 * @function EdorasAppInit
 * @brief Initialize app
 */
int32 EdorasAppInit(void)
{
    // Init ROS message stuff
    initializeParseData("geometry_msgs", "Pose", &parse_pose_);
    initializeParseData("sensor_msgs", "JointState", &parse_joint_state_);

    int32 status;

    EdorasAppData.RunStatus = CFE_ES_RunStatus_APP_RUN;

    // Initialize app command execution counters
    EdorasAppData.CmdCounter = 0;
    EdorasAppData.ErrCounter = 0;
    EdorasAppData.square_counter = 0;
    EdorasAppData.hk_counter = 0;

    EdorasAppData.HkTlm.Payload.state.pose.x = 0.0;

    // Initialize app configuration data
    EdorasAppData.PipeDepth = EDORAS_APP_PIPE_DEPTH;

    strncpy(EdorasAppData.PipeName, "EDORAS_APP_PIPE", sizeof(EdorasAppData.PipeName));
    EdorasAppData.PipeName[sizeof(EdorasAppData.PipeName) - 1] = 0;

    /*
    ** Initialize event filter table...
    */
    EdorasAppData.EventFilters[0].EventID = EDORAS_APP_STARTUP_INF_EID;
    EdorasAppData.EventFilters[0].Mask    = 0x0000;
    EdorasAppData.EventFilters[1].EventID = EDORAS_APP_COMMAND_ERR_EID;
    EdorasAppData.EventFilters[1].Mask    = 0x0000;
    EdorasAppData.EventFilters[2].EventID = EDORAS_APP_COMMANDNOP_INF_EID;
    EdorasAppData.EventFilters[2].Mask    = 0x0000;
    EdorasAppData.EventFilters[3].EventID = EDORAS_APP_COMMANDTWIST_INF_EID;
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

    // Subscribe to flight odom data
    status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(EDORAS_APP_CMD_ODOM_MID), EdorasAppData.CommandPipe);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("Edoras App: Error Subscribing to Odom data, RC = 0x%08lX\n", (unsigned long)status);

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
    printf("DEBUG -- Returning cfe success from init... \n");
    return (CFE_SUCCESS);

} 


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*  Name:  EdorasAppProcessCommandPacket                                    */
/*                                                                            */
/*  Purpose:                                                                  */
/*     This routine will process any packet that is received on the ros    */
/*     command pipe.                                                          */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * *  * * * * * * *  * *  * * * * */
void EdorasAppProcessCommandPacket(CFE_SB_Buffer_t *SBBufPtr)
{
    CFE_SB_MsgId_t MsgId = CFE_SB_INVALID_MSG_ID;

    CFE_MSG_GetMsgId(&SBBufPtr->Msg, &MsgId);
    //printf("DEBUG: -- we're processing the cmd from MID: 0x%04x\n", CFE_SB_MsgIdToValue(MsgId));
    switch (CFE_SB_MsgIdToValue(MsgId))
    {
        case EDORAS_APP_CMD_MID:
            EdorasAppProcessGroundCommand(SBBufPtr);
            break;

        case EDORAS_APP_SEND_HK_MID:
            EdorasAppReportHousekeeping((CFE_MSG_CommandHeader_t *)SBBufPtr);
            break;

        case EDORAS_APP_CMD_ODOM_MID:
            EdorasAppProcessFlightOdom(SBBufPtr);
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
            printf("Noop...\n");
            if (EdorasAppVerifyCmdLength(&SBBufPtr->Msg, sizeof(EdorasAppNoopCmd_t)))
            {
                EdorasAppNoop((EdorasAppNoopCmd_t *)SBBufPtr);
            }

            break;

        case EDORAS_APP_SET_TWIST_CC:
        {
            //if (EdorasAppVerifyCmdLength(&SBBufPtr->Msg, sizeof(EdorasAppTwistCmd_t)))
            //{
              // Let's see if we can deserialize
              
            // You know the first 8 bytes are the header
            size_t offset = 0;
            unsigned char header[8];
            memcpy(&header, SBBufPtr + offset, sizeof(header));
            
            // DEBUG
            //printf("Got command data: Header: %02x, %02x, %02x, %02x, %02x, %02x, %02x, %02x \n", 
            //       header[0], header[1], header[2], header[3], header[4], header[5], header[6], header[7]);
  
            size_t actual_length = 0;
            CFE_MSG_GetSize(&SBBufPtr->Msg, &actual_length);
            //printf("******** DEBUG: Actual length of command: %ld. Minus header: %ld \n", actual_length, actual_length - 8);
        
        // Parse the information  
        offset = 8;
        uint8_t* msg_pointer = NULL;
        size_t buffer_size;
        msg_pointer = from_uint_buffer_to_msg_pointer( (uint8_t*)SBBufPtr, offset, parse_pose_.ts, parse_pose_.ti, &buffer_size);
        
        // Get data
        double pos_x, pos_y, pos_z, orient_x, orient_y, orient_z, orient_w;
        get_float64(msg_pointer, parse_pose_.ti, "position.x", &pos_x);
        get_float64(msg_pointer, parse_pose_.ti, "position.y", &pos_y);
        get_float64(msg_pointer, parse_pose_.ti, "position.z", &pos_z);
        get_float64(msg_pointer, parse_pose_.ti, "orientation.x", &orient_x);
        get_float64(msg_pointer, parse_pose_.ti, "orientation.y", &orient_y);
        get_float64(msg_pointer, parse_pose_.ti, "orientation.z", &orient_z);
        get_float64(msg_pointer, parse_pose_.ti, "orientation.w", &orient_w);                                
        //debug_parse_buffer(msg_pointer, parse_twist_.ti);
        //printf("Reading linear velocity: %f and angular : %f \n", vel_lin, vel_ang);
        
        // Send data to robot!!!
        sendPoseCmd(&commData, pos_x, pos_y, pos_z, orient_x, orient_y, orient_z, orient_w);
       }
             break;

        /* default case already found during FC vs length test */
        default:
            CFE_EVS_SendEvent(EDORAS_APP_COMMAND_ERR_EID, CFE_EVS_EventType_ERROR,
                              "Invalid ground command code: CC = %d", CommandCode);
            break;
    }


    return;

} /* End of EdorasAppProcessFlightOdom() */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/* EdorasAppProcessFlightOdom() -- Edoras App flight odometry                   */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
void EdorasAppProcessFlightOdom(CFE_SB_Buffer_t *SBBufPtr)
{
    CFE_MSG_FcnCode_t CommandCode = 0;

    CFE_MSG_GetFcnCode(&SBBufPtr->Msg, &CommandCode);

    printf("EdorasAppProcessGroundCommand() -- we're getting a flight odometry message ...%d\n", CommandCode);

    // Read
    if (EdorasAppVerifyCmdLength(&SBBufPtr->Msg, sizeof(EdorasAppCmdRobotState_t)))
    {
       //EdorasAppCmdRobotState_t* state = (EdorasAppCmdRobotState_t *)SBBufPtr;   
    }


    return;

} /* End of EdorasAppProcessFlightCommand() */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*  Name:  EdorasAppReportHousekeeping                                          */
/*                                                                            */
/*  Purpose:                                                                  */
/*         This function is triggered in response to a task telemetry request */
/*         from the housekeeping task. This function will gather the Apps     */
/*         telemetry, packetize it and send it to the housekeeping task via   */
/*         the software bus                                                   */
/* * * * * * * * * * * * * * * * * * * * * * * *  * * * * * * *  * *  * * * * */
int32 EdorasAppReportHousekeeping(const CFE_MSG_CommandHeader_t *Msg)
{
    { 
     // Read telemetry, if any
     double joints[7];
                    
     if(!receiveJointStateTlm(&commData, joints))
       return CFE_SUCCESS;
       
     // If data received from robot update telemetry data
     // to send back to ground
     uint8_t* js_msg = create_msg(parse_joint_state_.ti);

     // Fill data
     resize_sequence(js_msg, parse_joint_state_.ti, "position", 7);
          
     set_float64(js_msg, parse_joint_state_.ti, "position.0", joints[0]);
     set_float64(js_msg, parse_joint_state_.ti, "position.1", joints[1]);
     set_float64(js_msg, parse_joint_state_.ti, "position.2", joints[2]);
     set_float64(js_msg, parse_joint_state_.ti, "position.3", joints[3]);
     set_float64(js_msg, parse_joint_state_.ti, "position.4", joints[4]);
     set_float64(js_msg, parse_joint_state_.ti, "position.5", joints[5]);
     set_float64(js_msg, parse_joint_state_.ti, "position.6", joints[6]);

     resize_sequence(js_msg, parse_joint_state_.ti, "name", 7);

     set_const_char(js_msg, parse_joint_state_.ti, "name.0", "big_arm_joint_2");
     set_const_char(js_msg, parse_joint_state_.ti, "name.1", "big_arm_joint_3");
     set_const_char(js_msg, parse_joint_state_.ti, "name.2", "big_arm_joint_4");
     set_const_char(js_msg, parse_joint_state_.ti, "name.3", "big_arm_joint_5");
     set_const_char(js_msg, parse_joint_state_.ti, "name.4", "big_arm_joint_6");
     set_const_char(js_msg, parse_joint_state_.ti, "name.5", "big_arm_joint_7");
     set_const_char(js_msg, parse_joint_state_.ti, "name.6", "big_arm_joint_8");

     // DEBUG 
     debug_parse_buffer(js_msg, parse_joint_state_.ti);

     // Convert data to serialized version
     uint8_t* tlm_data = NULL;
     size_t tlm_data_size;
     tlm_data = from_msg_pointer_to_uint_buffer(js_msg, parse_joint_state_.ts, parse_joint_state_.ti, &tlm_data_size);
     
     printf("Size of parse_joint_state element: %ld. Size of tlm header: %ld, size of Tlm data size: %ld \n", 
            sizeof(tlm_joint_state), sizeof(CFE_MSG_TelemetryHeader_t), tlm_data_size);
 
     // See header
     //printf("*** Tlm Header sent to ground: ");
     //for(size_t i = 0; i < 8; i++)
     //   printf("%02x ", tlm_joint_state.TlmHeader.Msg.Byte[i]);     
     //printf("\n");

     // Fill
     for(size_t i = 0; i < tlm_data_size; i++)
        memcpy(&tlm_joint_state.data[i], (uint8_t*)(tlm_data) + i, sizeof(uint8_t)); 
    
     // Debug
     //printBuffer(tlm_data, tlm_data_size, "Data to send back to ground: ");
     
     CFE_SB_TimeStampMsg(&tlm_joint_state.TlmHeader.Msg);
     // update_header: If true, the sequence counter bit in the primary header will increase each time
     // If false, it will remain zero.
     bool update_header = true;
     CFE_SB_TransmitMsg(&tlm_joint_state.TlmHeader.Msg, update_header); 
     
     // Clean up
     free(js_msg);
     free(tlm_data);
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

} /* End of EdorasAppReportHousekeeping() */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/* EdorasAppNoop -- ROS NOOP commands                                          */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
int32 EdorasAppNoop(const EdorasAppNoopCmd_t *Msg)
{
    CFE_EVS_SendEvent(EDORAS_APP_COMMANDNOP_INF_EID, CFE_EVS_EventType_INFORMATION, "Edoras App: NOOP command %s",
                      EDORAS_APP_VERSION);

    return CFE_SUCCESS;
} /* End of EdorasAppNoop */


int32 EdorasAppCmdTwist(const EdorasAppTwistCmd_t *Msg)
{
    EdorasAppData.LastTwist.twist.linear_x = Msg->twist.linear_x;
    EdorasAppData.LastTwist.twist.linear_y = Msg->twist.linear_y; 
    EdorasAppData.LastTwist.twist.linear_z = Msg->twist.linear_z;
    EdorasAppData.LastTwist.twist.angular_x = Msg->twist.angular_x;
    EdorasAppData.LastTwist.twist.angular_y = Msg->twist.angular_y;
    EdorasAppData.LastTwist.twist.angular_z = Msg->twist.angular_z;


    CFE_EVS_SendEvent(EDORAS_APP_COMMANDTWIST_INF_EID, CFE_EVS_EventType_INFORMATION, "Edoras App: twist command %s",
                      EDORAS_APP_VERSION);

    return CFE_SUCCESS;
    
}

void HighRateControlLoop(void) {
        
    // 2. Update the telemetry information        
    //EdorasAppOdometry_t *st = &lastOdomMsg; //EdorasAppGoal.StateTlm;

    //EdorasAppData.HkTlm.Payload.state.pose.x = st->pose.x;
    //EdorasAppData.HkTlm.Payload.state.pose.y = st->pose.y;

    //EdorasAppData.HkTlm.Payload.state.twist.linear_x = st->twist.linear_x;
    //EdorasAppData.HkTlm.Payload.state.twist.linear_y = st->twist.linear_y;

    // This data is sent when a Housekeeping request is received, 
    // (usually, at a low rate) so nothing sent here
    //memcpy(&st->joints, &EdorasAppData.HkTlm.Payload.state, sizeof(EdorasAppSSRMS_t) );
    
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/* EdorasAppVerifyCmdLength() -- Verify command packet length                   */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
bool EdorasAppVerifyCmdLength(CFE_MSG_Message_t *MsgPtr, size_t ExpectedLength)
{
    bool              result       = true;
    size_t            ActualLength = 0;
    CFE_SB_MsgId_t    MsgId        = CFE_SB_INVALID_MSG_ID;
    CFE_MSG_FcnCode_t FcnCode      = 0;

    printf("EdorasAppVerifyCmdLength() --\n");

    CFE_MSG_GetSize(MsgPtr, &ActualLength);

    /*
    ** Verify the command packet length.
    */
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

} /* End of EdorasAppVerifyCmdLength() */
