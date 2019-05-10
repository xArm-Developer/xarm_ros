/* Copyright 2017 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jimy Zhang <jimy92@163.com>
 ============================================================================*/
#ifndef XARM_CONNECT_H_
#define XARM_CONNECT_H_

#include "xarm/instruction/uxbus_cmd_ser.h"
#include "xarm/instruction/uxbus_cmd_tcp.h"

UxbusCmdSer *connect_rs485_control(const char *com);
UxbusCmdTcp *connect_tcp_control(char *server_ip);
SocketPort *connext_tcp_report_norm(char *server_ip);
SocketPort *connext_tcp_report_rich(char *server_ip);
SocketPort *connext_tcp_report_devl(char *server_ip);

#endif
