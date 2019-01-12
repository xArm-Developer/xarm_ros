/* Copyright 2017 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jimy Zhang <jimy92@163.com>
 ============================================================================*/
#ifndef LINUX_NETWORK_H_
#define LINUX_NETWORK_H_

#include "xarm/common/data_type.h"

int socket_init(char *local_ip, int port, u8 is_server);
s8 socket_send_data(int client_fp, u8 *data, u16 len);
s8 socket_connect_server(int *socket, char server_ip[], int server_port);

#endif
