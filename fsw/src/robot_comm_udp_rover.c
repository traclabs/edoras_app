/**
 * @file robot_comm_udp_rover.c
 */
#include "robot_comm_udp_rover.h"

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

bool sendTwistCmd( CommData_t* _cd, double _lin_x, double _lin_y, double _lin_z, double _ang_x, double _ang_y, double _ang_z)
{
    uint8_t* buf     = NULL;
    double   twist_data[6] = {_lin_x, _lin_y, _lin_z, _ang_x, _ang_y, _ang_z};
    size_t   bufSize = 6*sizeof(double);
    
    buf = (uint8_t*)malloc(bufSize);
    size_t offset = 0;
    for(int i = 0; i < 6; ++i)
    {
      memcpy(buf + offset, &twist_data[i], sizeof(double));
      offset += sizeof(double);
    }
 
   // DEBUG
   size_t ofi = 0;
   printf("* Twist command to be sent: ");
   for(int i =0; i < 6; ++i)
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
bool receivePoseTlm(CommData_t* _cd, double _position[3], double _orientation[4])
{
     ssize_t buffer_rcvd_size; 
     const int MAXLINE = 1024;
     uint8_t buffer_rcvd[MAXLINE];
     uint8_t* bp = &buffer_rcvd[0];
     
     // Receive............
    buffer_rcvd_size = recvfrom(_cd->sock_fd, (uint8_t*) buffer_rcvd, MAXLINE, MSG_DONTWAIT, (struct sockaddr*)NULL, NULL);
    if(buffer_rcvd_size > 0)
    {
      double data[7];
      size_t offset = 0;
      for(int i = 0; i < 7; ++i)  
      {
       memcpy(&data[i], bp + offset, sizeof(double));
       offset += sizeof(double);
      }

      // Fill fields
      _position[0] = data[0]; _position[1] = data[1]; _position[2] = data[2];
      _orientation[0] = data[3]; _orientation[1] = data[4]; _orientation[2] = data[5]; _orientation[3] = data[6];

     printf("* Tlm pose received from robot: %f %f %f -- %f %f %f %f \n", _position[0], _position[1], _position[2], 
        _orientation[0], _orientation[1], _orientation[2], _orientation[3]);
     
      return true;  
    }
    
  return false; 
}
