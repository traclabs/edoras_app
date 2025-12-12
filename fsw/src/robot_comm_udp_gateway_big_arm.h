#ifndef __ROBOT_COMM_UDP_GATEWAY_BIG_ARM_H__
#define __ROBOT_COMM_UDP_GATEWAY_BIG_ARM_H__

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
bool receiveJointStateTlm(CommData_t* _cd, float _js[7], int32_t* _sec, uint32_t* _nanosec);

#endif // __ROBOT_COMM_UDP_ROVER_H__
