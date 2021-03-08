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
#include "xarm/core/port/socket.h"
#include "xarm/core/os/network.h"

#ifdef _WIN32
#include <ws2tcpip.h>
static int close(int fd)
{
	return closesocket(fd);
}

static bool is_ignore_errno(int fp, int port)
{
	if (WSAGetLastError() == WSAEINTR || WSAGetLastError() == WSAEWOULDBLOCK) {
		printf("EINTR occured, port=%d, fp=%d, errno=%d\n", port, fp, WSAGetLastError());
		return true;
	}
	printf("socket read failed, port=%d, fp=%d, errno=%d, exit\n", port, fp, WSAGetLastError());
	return false;
}
#else
#include <sys/socket.h>
#include <unistd.h>
static bool is_ignore_errno(int fp, int port)
{
	if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
		printf("EINTR occured, port=%d, fp=%d, errno=%d\n", port, fp, errno);
		return true;
	}
	printf("socket read failed, port=%d, fp=%d, errno=%d, exit\n", port, fp, errno);
	return false;
}
#endif


void SocketPort::recv_proc(void) {
	int ret;
	int failed_cnt = 0;
	int num;
	unsigned char *recv_data = new unsigned char[que_maxlen_];
	while (state_ == 0) {
		memset(recv_data, 0, que_maxlen_);
		num = recv(fp_, (char *)&recv_data[4], que_maxlen_ - 4, 0);
		if (num <= 0) {
			if (is_ignore_errno(fp_, port_)) {
				continue;
			}
			else {
				close_port();
				break;
			}
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
			if (state_ == 0)
				printf("socket push data failed, exit, port=%d, fp=%d\n", port_, fp_);
			close_port();
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
	port_ = server_port;
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
	state_ = -1;
	flush();
	close(fp_);
}
