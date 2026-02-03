#ifndef __ROBOT_COMM_UDP_MOBILE_SERVICING_SYSTEM_H__
#define __ROBOT_COMM_UDP_MOBILE_SERVICING_SYSTEM_H__

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
bool sendGroupCmd( CommData_t* _cd, char _group[30], char _state[30]);
bool receiveJointStateTlm(CommData_t* _cd, 
  float _js_canadarm[7], 
  float _js_dextre_arm_1[6], float _js_dextre_arm_2[6], float* _js_dextre_body,
  float* _js_mbs,
  float _port_bga[4], float* _port_sarj,
  float _starboard_bga[4], float* _starboard_sarj,
  int32_t* _sec, uint32_t* _nanosec);

#endif // __ROBOT_COMM_UDP_ROVER_H__
