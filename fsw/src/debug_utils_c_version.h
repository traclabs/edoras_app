#ifndef __GATEWAY_APP_DEBUG_UTILS_C_VERSION_H__
#define __GATEWAY_APP_DEBUG_UTILS_C_VERSION_H__

#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <stdbool.h>
#include <string.h>

typedef struct 
{
  int sock_fd;
  char buffer[1024];
  struct sockaddr_in own_address;
  struct sockaddr_in other_address;
  
} CommData_t;

bool setupComm( CommData_t* _cd, int _cfs_port, int _robot_port);
bool sendGoalPose( CommData_t* _cd, double _x, double _y, double _z, double _roll, double _pitch, double _yaw);
bool receiveJointState(CommData_t* _cd, double _joint[7]);

#endif // __GATEWAY_APP_DEBUG_UTILS_C_VERSION_H__
