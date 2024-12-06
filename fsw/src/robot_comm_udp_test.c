/**
 * @file robot_comm_udp_test.c
 */
#include "robot_comm_udp_test.h"

#include <arpa/inet.h>


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

bool sendGoalPose( CommData_t* _cd, double _x, double _y, double _z, double _roll, double _pitch, double _yaw)
{
 return true;
}

bool receiveJointState(CommData_t* _cd, double _joint[7])
{
 return true;
}
