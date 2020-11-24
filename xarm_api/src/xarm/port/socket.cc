/* Copyright 2017 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jimy Zhang <jimy92@163.com>
 ============================================================================*/
#include "xarm/port/socket.h"

#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include <errno.h>

#include "xarm/linux/network.h"
#include "xarm/linux/thread.h"

void SocketPort::recv_proc(void) {
  int num;
  unsigned char recv_data[que_maxlen_];
  while (state_ == 0) {
    bzero(recv_data, que_maxlen_);
    num = recv(fp_, (void *)&recv_data[4], que_maxlen_ - 1, 0);
    if (num <= 0) {
      // in case recv() blocking call is interrupted by a system signal, this should not be considered as socket failure.
      if(errno == EINTR)
      {
        printf("EINTR occured\n");
        continue;
      }

      close_port();
      printf("SocketPort::recv_proc exit, %d\n", fp_);
      pthread_exit(0);
    }
    bin32_to_8(num, &recv_data[0]);
    rx_que_->push(recv_data);
  }
}

static void *recv_proc_(void *arg) {
  SocketPort *my_this = (SocketPort *)arg;

  my_this->recv_proc();

  pthread_exit(0);
}

SocketPort::SocketPort(char *server_ip, int server_port, int que_num,
                       int que_maxlen) {
  que_num_ = que_num;
  que_maxlen_ = que_maxlen;
  state_ = -1;
  rx_que_ = new QueueMemcpy(que_num_, que_maxlen_);
  fp_ = socket_init((char *)" ", 0, 0);
  if (fp_ == -1) { return; }

  int ret = socket_connect_server(&fp_, server_ip, server_port);
  if (ret == -1) { return; }

  state_ = 0;
  flush();
  thread_id_ = thread_init(recv_proc_, this);
}

SocketPort::~SocketPort(void) {
  state_ = -1;
  delete rx_que_;
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
  delete rx_que_;
}
