/**
 * @file robot_comm_udp_mobile_servicing_system.c
 */
#include "robot_comm_udp_lunar_exploration.h"

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

bool sendTwistCmd( CommData_t* _cd, float _vlin, float _vang)
{
    uint8_t* buf     = NULL;
    size_t   bufSize = sizeof(uint8_t) + 2*sizeof(float);
    
    buf = (uint8_t*)malloc(bufSize);
    size_t offset = 0;

    uint8_t code = 1;
    float vlin = _vlin;
    float vang = _vang;

    memcpy(buf + offset, &code, sizeof(uint8_t));
    offset += sizeof(uint8_t);
 
    memcpy(buf + offset, &vlin, sizeof(float));
    offset += sizeof(float);

    memcpy(buf + offset, &vang, sizeof(float));
    offset += sizeof(float);
     
    int res = sendto(_cd->sock_fd, buf, bufSize, 0, (const struct sockaddr *)&_cd->other_address, sizeof(_cd->other_address));
 
    // Clean up
    free(buf);
    
    return (res > 0);
}

/**
 * @function sendGroupCmd
 */
bool sendCameraCmd( CommData_t* _cd, float _joint_1, float _joint_2)
{
    uint8_t* buf     = NULL;
    size_t   bufSize = sizeof(uint8_t) + 2*sizeof(float);
    
    buf = (uint8_t*)malloc(bufSize);
    size_t offset = 0;

    uint8_t code = 2;
    float joint_1 = _joint_1;
    float joint_2 = _joint_2;

    memcpy(buf + offset, &code, sizeof(uint8_t));
    offset += sizeof(uint8_t);
 
    memcpy(buf + offset, &joint_1, sizeof(float));
    offset += sizeof(float);

    memcpy(buf + offset, &joint_2, sizeof(float));
    offset += sizeof(float);
     
    int res = sendto(_cd->sock_fd, buf, bufSize, 0, (const struct sockaddr *)&_cd->other_address, sizeof(_cd->other_address));
 
    // Clean up
    free(buf);
    
    return (res > 0);

}


/**
 * @function receivePoseTlm
 */
bool receiveJointStateTlm(CommData_t* _cd, 
     float _joints[17], 
     float* _x, float* _y, float* _z, float* _qx, float* _qy,float* _qz,float* _qw, 
     int32_t* _sec, uint32_t* _nanosec)
{
     ssize_t buffer_rcvd_size; 
     const int MAXLINE = 1024;
     uint8_t buffer_rcvd[MAXLINE];
     uint8_t* bp = &buffer_rcvd[0];
     
   // Joint order
     
     // Receive............
    buffer_rcvd_size = recvfrom(_cd->sock_fd, (uint8_t*) buffer_rcvd, MAXLINE, MSG_DONTWAIT, (struct sockaddr*)NULL, NULL);
    if(buffer_rcvd_size > 0)
    {
      size_t offset = 0;
      // Fill fields
      for(int i = 0; i < 17; ++i)  
      {
       memcpy(&_joints[i], bp + offset, sizeof(float));
       offset += sizeof(float);
      }
      
       memcpy(_x, bp + offset, sizeof(float));
       offset += sizeof(float);

       memcpy(_y, bp + offset, sizeof(float));
       offset += sizeof(float);

       memcpy(_z, bp + offset, sizeof(float));
       offset += sizeof(float);

       memcpy(_qx, bp + offset, sizeof(float));
       offset += sizeof(float);

       memcpy(_qy, bp + offset, sizeof(float));
       offset += sizeof(float);

       memcpy(_qz, bp + offset, sizeof(float));
       offset += sizeof(float);

       memcpy(_qw, bp + offset, sizeof(float));
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
