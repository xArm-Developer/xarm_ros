/* Copyright 2017 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jimy Zhang <jimy92@163.com>
 ============================================================================*/
#ifndef PORT_SOCKET_H_
#define PORT_SOCKET_H_

#include <pthread.h>

#include "xarm/common/data_type.h"
#include "xarm/common/queue_memcpy.h"

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
  int fp_;
  int state_;
  int que_num_;
  QueueMemcpy *rx_que_;
  pthread_t thread_id_;
};

#endif
