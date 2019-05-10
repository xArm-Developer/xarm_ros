/* Copyright 2017 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jimy Zhang <jimy92@163.com>
 ============================================================================*/
#ifndef LINUX_NETWORK_H_
#define LINUX_NETWORK_H_

#include "xarm/common/data_type.h"

int socket_init(char *local_ip, int port, int is_server);
int socket_send_data(int client_fp, unsigned char *data, int len);
int socket_connect_server(int *socket, char server_ip[], int server_port);

#endif
