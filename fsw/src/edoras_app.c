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

// Global data
EdorasAppData_t EdorasAppData;

ParseData_t parse_pose_; // Receive command
ParseData_t parse_twist_; // Send telemetry

typedef struct
{
    CFE_MSG_TelemetryHeader_t  TlmHeader;
    uint8_t data[120]; // 56 for c ros struct, 100 serialized = 84 + 8 length + 8 capacity
} PoseData_t;

typedef struct
{
    CFE_MSG_TelemetryHeader_t  TlmHeader;
    uint8_t data[80]; // 56 for c ros struct, 68 serialized = 52 + 8 length + 8 capacity
} TwistData_t;

PoseData_t tlm_pose_1;
PoseData_t tlm_pose_2;
TwistData_t cmd_twist_1;
TwistData_t cmd_twist_2;

// To delete
EdorasAppData_t EdorasAppData;

/**
 * @function initializeParseData
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

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  * *  * * * * **/
/* EdorasAppMain() -- Application entry point and main process loop         */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  * *  * * * * **/
void EdorasAppMain(void)
{
    int32            status;
    CFE_SB_Buffer_t *SBBufPtr;

    // Create the first Performance Log entry
    CFE_ES_PerfLogEntry(EDORAS_APP_PERF_ID);

    // Perform application specific initialization
    // 0: CFE_ES_RunStatus_UNDEFINED, 1: CFE_ES_RunStatus_APP_RUN, 2: CFE_ES_RunStatus_APP_EXIT, 3: CFE_ES_RunStatus_APP_ERROR
    status = EdorasAppInit();
    if (status != CFE_SUCCESS)
    {   printf("Setting run status to be error !!!!!!!!\n");
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
 * @function EdorasAppInit
 * @brief Initialize app
 */
int32 EdorasAppInit(void)
{
    // Init ROS message stuff
    initializeParseData("geometry_msgs", "PoseStamped", &parse_pose_);
    initializeParseData("geometry_msgs", "Twist", &parse_twist_);

    int32 status;

    EdorasAppData.RunStatus = CFE_ES_RunStatus_APP_RUN;

    // Initialize app command execution counters
    EdorasAppData.CmdCounter = 0;
    EdorasAppData.ErrCounter = 0;
    EdorasAppData.square_counter = 0;
    EdorasAppData.hk_counter = 0;

    EdorasAppData.rcvd_twist_1 = false;
    EdorasAppData.rcvd_twist_2 = false;
    
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
    
    // Initialize data coming out of fsw (going to ground as telemetry, or going to rosfsw as commands)
    // CFE_MSG_Init Clears memory of message size and sets up the respective bytes of the telemetry headers
    CFE_MSG_Init(&tlm_pose_1.TlmHeader.Msg, CFE_SB_ValueToMsgId(EDORAS_APP_TLM_1_GROUND_MID), sizeof(tlm_pose_1));
    CFE_MSG_Init(&tlm_pose_2.TlmHeader.Msg, CFE_SB_ValueToMsgId(EDORAS_APP_TLM_2_GROUND_MID), sizeof(tlm_pose_2));
    
    CFE_MSG_Init(&cmd_twist_1.TlmHeader.Msg, CFE_SB_ValueToMsgId(EDORAS_APP_CMD_1_FLIGHT_MID), sizeof(cmd_twist_1) );
    CFE_MSG_Init(&cmd_twist_2.TlmHeader.Msg, CFE_SB_ValueToMsgId(EDORAS_APP_CMD_2_FLIGHT_MID), sizeof(cmd_twist_2) );    

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

    // Subscribe to HR wakeup
    status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(EDORAS_APP_HR_CONTROL_MID), EdorasAppData.CommandPipe);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("Edoras App: Error Subscribing to HR Wakeup Command, RC = 0x%08lX\n", (unsigned long)status);
        return (status);
    }


    // Subscribe to command packets coming to fsw
    // Either they come from the ground (commands)
    // Or they are telemetry coming from the flightside (rosfsw)
    status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(EDORAS_APP_CMD_1_GROUND_MID), EdorasAppData.CommandPipe);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("Edoras App: Error Subscribing to Ground Twist 1, RC = 0x%08lX\n", (unsigned long)status);
        return (status);
    }
    status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(EDORAS_APP_CMD_2_GROUND_MID), EdorasAppData.CommandPipe);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("Edoras App: Error Subscribing to Ground Twist 2, RC = 0x%08lX\n", (unsigned long)status);
        return (status);
    }
    status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(EDORAS_APP_TLM_1_FLIGHT_MID), EdorasAppData.CommandPipe);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("Edoras App: Error Subscribing to Flight Pose 1, RC = 0x%08lX\n", (unsigned long)status);
        return (status);
    }
    status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(EDORAS_APP_TLM_2_FLIGHT_MID), EdorasAppData.CommandPipe);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("Edoras App: Error Subscribing to Flight Pose 2, RC = 0x%08lX\n", (unsigned long)status);
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
    //printf("DEBUG: -- we're processing the cmd from MID: 0x%04x .\n", CFE_SB_MsgIdToValue(MsgId));
    switch (CFE_SB_MsgIdToValue(MsgId))
    {
        case EDORAS_APP_SEND_HK_MID:
            EdorasAppReportHousekeeping();
            break;

        case EDORAS_APP_HR_CONTROL_MID:
            //HighRateControlLoop();
            break;

        case EDORAS_APP_CMD_1_GROUND_MID:
            EdorasAppProcessGroundTwist(SBBufPtr, 1);
            break;

        case EDORAS_APP_CMD_2_GROUND_MID:
            EdorasAppProcessGroundTwist(SBBufPtr, 2);
            break;

        case EDORAS_APP_TLM_1_FLIGHT_MID:
            EdorasAppProcessFlightPose(SBBufPtr, 1);
            break;

        case EDORAS_APP_TLM_2_FLIGHT_MID:
            EdorasAppProcessFlightPose(SBBufPtr, 2);
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
void EdorasAppProcessGroundTwist(CFE_SB_Buffer_t *SBBufPtr, int _robot_id)
{
    CFE_MSG_FcnCode_t CommandCode = 0;
    CFE_MSG_GetFcnCode(&SBBufPtr->Msg, &CommandCode);

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
          
    // Send command to robot using ROS2 on the flight side
    // Put the information directly in the twist that we'll send to the robot on the flight side
    // (message will be the same, only difference is the header)
    // offset: 8 (command header size)
    // update_header: If true, the sequence counter bit in the primary header will increase each time
    // If false, it will remain zero.
    offset += sizeof(header);
    size_t data_size = actual_length - offset;
    if(_robot_id == 1)
    { //printf("Received twist command from ground for robot 1, sending it to flight side \n");
      EdorasAppData.rcvd_twist_1 = true;
      memcpy(&cmd_twist_1.data, (uint8_t*) (SBBufPtr) + offset, data_size*sizeof(uint8_t));
    }
    else if(_robot_id == 2)
    {
      EdorasAppData.rcvd_twist_2 = true;
      memcpy(&cmd_twist_2.data, (uint8_t*) (SBBufPtr) + offset, data_size*sizeof(uint8_t));
    } 
    
    return;
}


/**                                   
 * @function EdorasAppProcessFlightPose
 * @brief Edoras App ground commands               
 */
void EdorasAppProcessFlightPose(CFE_SB_Buffer_t *SBBufPtr, int _robot_id)
{
    CFE_MSG_FcnCode_t CommandCode = 0;
    CFE_MSG_GetFcnCode(&SBBufPtr->Msg, &CommandCode);

    //if (EdorasAppVerifyCmdLength(&SBBufPtr->Msg, sizeof(EdorasAppTwistCmd_t)))
    //{
       // Let's see if we can deserialize
              
    // You know the first 8 bytes are the header
    size_t offset = 0;
    uint8_t header[8];
    memcpy(&header, SBBufPtr + offset, sizeof(header));
            
    // DEBUG
    //printf("Got command data flight pose: Header: %02x, %02x, %02x, %02x, %02x, %02x, %02x, %02x \n", 
    //       header[0], header[1], header[2], header[3], header[4], header[5], header[6], header[7]);

    size_t actual_length = 0;
    CFE_MSG_GetSize(&SBBufPtr->Msg, &actual_length);
    
    //printf("******** DEBUG: Actual length of Pose command: %ld. Minus header: %ld \n", actual_length, actual_length - 8);
        
          
    // Send command to robot using ROS2 on the flight side
    // Put the information directly in the twist that we'll send to the robot on the flight side
    // (message will be the same, only difference is the header)
    // offset: 8 (command header size)
    // update_header: If true, the sequence counter bit in the primary header will increase each time
    // If false, it will remain zero.
    offset = 8;
    bool update_header = true;
    size_t data_size = actual_length - offset;
    if(_robot_id == 1)
    {
      memcpy(&tlm_pose_1.data, (uint8_t*) (SBBufPtr) + offset, data_size);
      CFE_SB_TimeStampMsg(&tlm_pose_1.TlmHeader.Msg);
      CFE_SB_TransmitMsg(&tlm_pose_1.TlmHeader.Msg, update_header); 
    }
    else if(_robot_id == 2)
    {
      memcpy(&tlm_pose_2.data, (uint8_t*) (SBBufPtr) + offset, data_size);
      CFE_SB_TimeStampMsg(&tlm_pose_2.TlmHeader.Msg);
      CFE_SB_TransmitMsg(&tlm_pose_2.TlmHeader.Msg, update_header);      
    }     
    return;
}

/**
 *  Name:  EdorasAppReportHousekeeping
 */
int32 EdorasAppReportHousekeeping(void)
{

  bool update_header = true;
  if(EdorasAppData.rcvd_twist_1 == true)
  {
     //OS_printf("******** DEBUG: Sending twist to flight side command 1  ** \n");
     CFE_SB_TimeStampMsg(&cmd_twist_1.TlmHeader.Msg);
     CFE_Status_t  res;
     res = CFE_SB_TransmitMsg(&cmd_twist_1.TlmHeader.Msg, update_header);
     if(res != CFE_SUCCESS)
       OS_printf("Error in trying to transmit twist to flight side: %d \n", res); 
     EdorasAppData.rcvd_twist_1 = false;
  }
  if(EdorasAppData.rcvd_twist_2 == true)
  {
     //printf("******** DEBUG: Sending twist to flight side command 2 \n");  
     CFE_SB_TimeStampMsg(&cmd_twist_2.TlmHeader.Msg);
     CFE_SB_TransmitMsg(&cmd_twist_2.TlmHeader.Msg, update_header);
     EdorasAppData.rcvd_twist_2 = false;  
  }

    return CFE_SUCCESS;

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
