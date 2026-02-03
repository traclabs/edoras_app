/**
 * @file robot_comm_udp_mobile_servicing_system.c
 */
#include "robot_comm_udp_mobile_servicing_system.h"

#include <arpa/inet.h>
#include <string.h>
#include <stdlib.h>

bool setupComm( CommData_t* _cd, int _cfs_port, int _robot_port, const char* _cfs_ip, const char* _robot_ip)
{
 // Create socket
 _cd->sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
 if(_cd->sock_fd < 0)
 {
    perror("Socket creation failed \n");
    return false;
 }
 
  memset(&_cd->own_address, 0, sizeof(_cd->own_address));
  memset(&_cd->other_address, 0, sizeof(_cd->other_address));
  
  // Fill server information
  _cd->own_address.sin_family = AF_INET;
  _cd->own_address.sin_addr.s_addr = inet_addr(_cfs_ip); //inet_addr("127.0.0.1"); //INADDR_ANY;
  _cd->own_address.sin_port = htons(_cfs_port);
  
  // Bind the socket
  int res = bind(_cd->sock_fd, (const struct sockaddr*)&_cd->own_address, sizeof(_cd->own_address));
  if( res < 0 )
  {
     perror("Error in binding  socket to talk to robot");
     return false;
  }
  
  _cd->other_address.sin_family = AF_INET;
  _cd->other_address.sin_addr.s_addr = inet_addr(_robot_ip); //inet_addr("127.0.0.1"); //INADDR_ANY;
  _cd->other_address.sin_port = htons(_robot_port);
  
  return true;
}

bool sendPoseCmd( CommData_t* _cd, float _pos[3], float _rot[4])
{
    uint8_t* buf     = NULL;
    float   pose_data[7] = {_pos[0], _pos[1], _pos[2], _rot[0], _rot[1], _rot[2], _rot[3]};
    size_t   bufSize = 7*sizeof(float);
    
    buf = (uint8_t*)malloc(bufSize);
    size_t offset = 0;
    for(int i = 0; i < 7; ++i)
    {
      memcpy(buf + offset, &pose_data[i], sizeof(float));
      offset += sizeof(float);
    }
 
   // DEBUG
   /*size_t ofi = 0;
   printf("* Pose command to be sent: ");
   for(int i =0; i < 7; ++i)
   {
      double val;
      memcpy(&val, buf + ofi, sizeof(double));
      ofi += sizeof(double);
      printf("%f ", val);   
   } printf("\n");*/
     
    int res = sendto(_cd->sock_fd, buf, bufSize, 0, (const struct sockaddr *)&_cd->other_address, sizeof(_cd->other_address));
 
    // Clean up
    free(buf);
    
    return (res > 0);
}

/**
 * @function sendGroupCmd
 */
bool sendGroupCmd( CommData_t* _cd, char _group[30], char _state[30])
{
    uint8_t* buf     = NULL;
    size_t   bufSize = 60*sizeof(char);
    
    buf = (uint8_t*)malloc(bufSize);
    
    size_t offset = 0;
    memcpy(buf + offset, _group, 30*sizeof(char));
    offset += 30*sizeof(char);

    memcpy(buf + offset, _state, 30*sizeof(char));
    offset += 30*sizeof(char);
     
    int res = sendto(_cd->sock_fd, buf, bufSize, 0, (const struct sockaddr *)&_cd->other_address, sizeof(_cd->other_address));
 
    // Clean up
    free(buf);
    
    return (res > 0);
}


/**
 * @function receivePoseTlm
 */
bool receiveJointStateTlm(CommData_t* _cd, 
     float _js_canadarm[7], 
     float _js_dextre_arm_1[6], float _js_dextre_arm_2[6], float* _js_dextre_body,
     float* _js_mbs,
     float _port_bga[4], float* _port_sarj,
     float _starboard_bga[4], float* _starboard_sarj,
     int32_t* _sec, uint32_t* _nanosec)
{
     ssize_t buffer_rcvd_size; 
     const int MAXLINE = 1024;
     uint8_t buffer_rcvd[MAXLINE];
     uint8_t* bp = &buffer_rcvd[0];
     
   // Joint order
   // joint_canadarm2_1, joint_canadarm2_2, joint_canadarm2_3, joint_canadarm2_4, joint_canadarm2_5, joint_canadarm2_6, joint_canadarm2_7
   // joint_dextre_arm_1_elbow_pitch, joint_dextre_arm_1_shoulder_pitch, joint_dextre_arm_1_shoulder_roll
   // joint_dextre_arm_1_shoulder_yaw, joint_dextre_arm_1_wrist_pitch_yaw, joint_dextre_arm_1_wrist_roll
   // joint_dextre_arm_2_elbow_pitch, joint_dextre_arm_2_shoulder_pitch, joint_dextre_arm_2_shoulder_roll
   // joint_dextre_arm_2_shoulder_yaw, joint_dextre_arm_2_wrist_pitch_yaw, joint_dextre_arm_2_wrist_roll
   // joint_dextre_body
   // joint_mbs
   // joint_port_bga_1, joint_port_bga_2, joint_port_bga_3, joint_port_bga_4
   // joint_port_sarj
   // joint_starboard_bga_1, joint_starboard_bga_2, joint_starboard_bga_3, joint_starboard_bga_4
   // joint_starboard_sarj

     
     // Receive............
    buffer_rcvd_size = recvfrom(_cd->sock_fd, (uint8_t*) buffer_rcvd, MAXLINE, MSG_DONTWAIT, (struct sockaddr*)NULL, NULL);
    if(buffer_rcvd_size > 0)
    {
      size_t offset = 0;
      // Fill fields
      for(int i = 0; i < 7; ++i)  
      {
       memcpy(&_js_canadarm[i], bp + offset, sizeof(float));
       offset += sizeof(float);
      }
      for(int i = 0; i < 6; ++i)  
      {
       memcpy(&_js_dextre_arm_1[i], bp + offset, sizeof(float));
       offset += sizeof(float);
      }
      for(int i = 0; i < 6; ++i)  
      {
       memcpy(&_js_dextre_arm_2[i], bp + offset, sizeof(float));
       offset += sizeof(float);
      }
      
       memcpy(_js_dextre_body, bp + offset, sizeof(float));
       offset += sizeof(float);

       memcpy(_js_mbs, bp + offset, sizeof(float));
       offset += sizeof(float);
     
      for(int i = 0; i < 4; ++i)  
      {
       memcpy(&_port_bga[i], bp + offset, sizeof(float));
       offset += sizeof(float);
      }      
       memcpy(&_port_sarj, bp + offset, sizeof(float));
       offset += sizeof(float);     

      for(int i = 0; i < 4; ++i)  
      {
       memcpy(&_starboard_bga[i], bp + offset, sizeof(float));
       offset += sizeof(float);
      }      
       memcpy(&_starboard_sarj, bp + offset, sizeof(float));
       offset += sizeof(float);

      // Sec
      memcpy(_sec, bp + offset, sizeof(int32_t));
      offset += sizeof(int32_t);
      memcpy(_nanosec, bp + offset, sizeof(uint32_t));
      offset += sizeof(uint32_t);

      return true;  
    }
    
  return false; 
}
