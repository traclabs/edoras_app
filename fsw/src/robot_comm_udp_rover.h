#ifndef __ROBOT_COMM_UDP_ROVER_H__
#define __ROBOT_COMM_UDP_ROVER_H__

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
bool sendTwistCmd( CommData_t* _cd, double _lin_x, double _lin_y, double _lin_z, double _ang_x, double _ang_y, double _ang_z);
bool receivePoseTlm(CommData_t* _cd, double _position[3], double _orientation[4], int32_t* _sec, uint32_t* _nanosec );

#endif // __ROBOT_COMM_UDP_ROVER_H__
