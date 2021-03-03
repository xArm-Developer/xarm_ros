/*
# Software License Agreement (MIT License)
#
# Copyright (c) 2020, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
*/
#include <string.h>
#include <errno.h>

#ifndef WIN32
#include <sys/socket.h>
#include <unistd.h>
#else
#include <ws2tcpip.h>
static int close(int fd)
{
  return closesocket(fd);
}
// #define errno WSAGetLastError()
#endif

#include "xarm/core/port/socket.h"
#include "xarm/core/os/network.h"

void SocketPort::recv_proc(void) {
	int ret;
	int failed_cnt = 0;
	int num;
	unsigned char *recv_data = new unsigned char[que_maxlen_];
	while (state_ == 0) {
		memset(recv_data, 0, que_maxlen_);
		num = recv(fp_, (char *)&recv_data[4], que_maxlen_ - 4, 0);
		if (num <= 0 && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)) {
			printf("EINTR occured, errno=%d\n", errno);
			continue;
		}
		if (num <= 0) {
			printf("socket read failed, fp=%d, errno=%d, exit\n", fp_, errno);
			close_port();
			break;
		}
		bin32_to_8(num, &recv_data[0]);
		ret = rx_que_->push(recv_data);
		failed_cnt = 0;
		while (ret != 0 && state_ == 0 && failed_cnt < 1500)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(2));
			ret = rx_que_->push(recv_data);
			failed_cnt += 1;
		}
		if (ret != 0) {
			close_port();
			printf("socket push data failed, exit, %d\n", fp_);
			break;
		};
	}
	delete[] recv_data;
	delete rx_que_;
}

static void *recv_proc_(void *arg) {
	SocketPort *my_this = (SocketPort *)arg;

	my_this->recv_proc();
	return (void *)0;
}

SocketPort::SocketPort(char *server_ip, int server_port, int que_num,
	int que_maxlen) {
	que_num_ = que_num;
	que_maxlen_ = que_maxlen;
	state_ = -1;
	rx_que_ = new QueueMemcpy(que_num_, que_maxlen_);
	fp_ = socket_init((char *)" ", 0, 0);
	if (fp_ == -1) { 
		delete rx_que_;
		return;
	}

	int ret = socket_connect_server(&fp_, server_ip, server_port);
	if (ret == -1) { 
		delete rx_que_;
		return;
	}

	state_ = 0;
	flush();
	std::thread th(recv_proc_, this);
	th.detach();
}

SocketPort::~SocketPort(void) {
	state_ = -1;
	close_port();
}

int SocketPort::is_ok(void) { return state_; }

void SocketPort::flush(void) { rx_que_->flush(); }

int SocketPort::read_frame(unsigned char *data) {
	if (state_ != 0) { return -1; }

	if (rx_que_->size() == 0) { return -1; }

	rx_que_->pop(data);
	return 0;
}

int SocketPort::write_frame(unsigned char *data, int len) {
	int ret = socket_send_data(fp_, data, len);
	return ret;
}

void SocketPort::close_port(void) {
	close(fp_);
	state_ = -1;
}
