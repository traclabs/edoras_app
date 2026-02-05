#ifndef __ROBOT_COMM_UDP_LUNAR_EXPLORATION_H__
#define __ROBOT_COMM_UDP_LUNAR_EXPLORATION_H__

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

bool setupComm( CommData_t* _cd, int _cfs_port, int _robot_port, const char* _cfs_ip, const char* _robot_ip);
bool sendPoseCmd( CommData_t* _cd, float _pos[3], float _rot[4]);
bool sendTwistCmd( CommData_t* _cd, float _vlin, float _vang);
bool sendCameraCmd( CommData_t* _cd, float _j1, float _j2);

bool receiveJointStateTlm(CommData_t* _cd, 
  float _joints[17], 
  float* _x, float* _y, float* _z, float* _qx, float* _qy,float* _qz,float* _qw, 
  int32_t* _sec, uint32_t* _nanosec);

#endif // __ROBOT_COMM_UDP_LUNAR_EXPLORATION_H__
