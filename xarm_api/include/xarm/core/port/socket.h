/* Copyright 2017 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jimy Zhang <jimy92@163.com>
 ============================================================================*/
#ifndef CORE_PORT_SOCKET_H_
#define CORE_PORT_SOCKET_H_

#include <iostream>
#include <thread>
#include "xarm/core/common/data_type.h"
#include "xarm/core/common/queue_memcpy.h"

class SocketPort {
public:
	SocketPort(char *server_ip, int server_port, int que_num, int que_maxlen);
	~SocketPort(void);
	int is_ok(void);
	void flush(void);
	void recv_proc(void);
	int write_frame(unsigned char *data, int len);
	int read_frame(unsigned char *data);
	void close_port(void);
	int que_maxlen_;

private:
	int port_;
	int fp_;
	int state_;
	int que_num_;
	QueueMemcpy *rx_que_;
};

#endif
