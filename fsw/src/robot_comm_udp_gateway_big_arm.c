/**
 * @file robot_comm_udp_gateway_big_arm.c
 */
#include "robot_comm_udp_gateway_big_arm.h"

#include <arpa/inet.h>
#include <string.h>
#include <stdlib.h>

bool setupComm( CommData_t* _cd, int _cfs_port, int _robot_port)
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
  _cd->own_address.sin_addr.s_addr = inet_addr("127.0.0.1"); //INADDR_ANY;
  _cd->own_address.sin_port = htons(_cfs_port);
  
  // Bind the socket
  int res = bind(_cd->sock_fd, (const struct sockaddr*)&_cd->own_address, sizeof(_cd->own_address));
  if( res < 0 )
  {
     perror("Error in binding  socket to talk to robot");
     return false;
  }
  
  _cd->other_address.sin_family = AF_INET;
  _cd->other_address.sin_addr.s_addr = inet_addr("127.0.0.1"); //INADDR_ANY;
  _cd->other_address.sin_port = htons(_robot_port);
  
  return true;
}

bool sendPoseCmd( CommData_t* _cd, double _pos_x, double _pos_y, double _pos_z, double _orient_x, double _orient_y, double _orient_z, double _orient_w)
{
    uint8_t* buf     = NULL;
    double   pose_data[7] = {_pos_x, _pos_y, _pos_z, _orient_x, _orient_y, _orient_z, _orient_w};
    size_t   bufSize = 7*sizeof(double);
    
    buf = (uint8_t*)malloc(bufSize);
    size_t offset = 0;
    for(int i = 0; i < 7; ++i)
    {
      memcpy(buf + offset, &pose_data[i], sizeof(double));
      offset += sizeof(double);
    }
 
   // DEBUG
   size_t ofi = 0;
   printf("* Pose command to be sent: ");
   for(int i =0; i < 7; ++i)
   {
      double val;
      memcpy(&val, buf + ofi, sizeof(double));
      ofi += sizeof(double);
      printf("%f ", val);   
   } printf("\n");
     
    int res = sendto(_cd->sock_fd, buf, bufSize, 0, (const struct sockaddr *)&_cd->other_address, sizeof(_cd->other_address));
 
    // Clean up
    free(buf);
    
    return (res > 0);
}

/**
 * @function receivePoseTlm
 */
bool receiveJointStateTlm(CommData_t* _cd, double _js[7], int32_t* _sec, uint32_t* _nanosec)
{
     ssize_t buffer_rcvd_size; 
     const int MAXLINE = 1024;
     uint8_t buffer_rcvd[MAXLINE];
     uint8_t* bp = &buffer_rcvd[0];
     
     // Receive............
    buffer_rcvd_size = recvfrom(_cd->sock_fd, (uint8_t*) buffer_rcvd, MAXLINE, MSG_DONTWAIT, (struct sockaddr*)NULL, NULL);
    if(buffer_rcvd_size > 0)
    {
      size_t offset = 0;
      // Fill fields
      for(int i = 0; i < 7; ++i)  
      {
       memcpy(&_js[i], bp + offset, sizeof(double));
       offset += sizeof(double);
      }

      // Sec
      memcpy(_sec, bp + offset, sizeof(int32_t));
      offset += sizeof(int32_t);
      memcpy(_nanosec, bp + offset, sizeof(uint32_t));
      offset += sizeof(uint32_t);

     printf("* Tlm pose received from robot: %f %f %f %f %f %f %f - Time: sec: %d nanosec: %d \n", _js[0], _js[1], _js[2], 
        _js[3], _js[4], _js[5], _js[6], *_sec, *_nanosec);
     
      return true;  
    }
    
  return false; 
}
