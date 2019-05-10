/* Copyright 2017 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jimy Zhang <jimy92@163.com>
 ============================================================================*/
#ifndef PORT_SERIAL_H_
#define PORT_SERIAL_H_

#include <pthread.h>

#include "xarm/common/data_type.h"
#include "xarm/common/queue_memcpy.h"

class SerialPort {
 public:
  SerialPort(const char *port, int baud, int que_num, int que_maxlen);
  ~SerialPort(void);
  int is_ok(void);
  void flush(void);
  void recv_proc(void);
  int write_frame(unsigned char *data, int len);
  int read_frame(unsigned char *data);
  void close_port(void);
  int que_maxlen_;
  int que_num_;

 private:
  int fp_;
  int state_;
  pthread_t thread_id_;
  QueueMemcpy *rx_que_;
  int init_serial(const char *port, int baud);
  int read_char(unsigned char *ch);
  int read_str(unsigned char *data, char eol, int len);
  int write_char(unsigned char ch);
  void parse_put(unsigned char *data, int len);

  typedef enum _UXBUS_RECV_STATE {
    UXBUS_START_FROMID = 0,
    UXBUS_START_TOOID = 1,
    UXBUS_STATE_LENGTH = 2,
    UXBUS_STATE_DATA = 3,
    UXBUS_STATE_CRC1 = 4,
    UXBUS_STATE_CRC2 = 5,
  } UXBUS_RECV_STATE;

  unsigned char UXBUS_PROT_FROMID_;
  unsigned char UXBUS_PROT_TOID_;

  int rx_data_idx_;
  int rx_state_;
  unsigned char rx_buf_[128];
  int rx_length_;
};

#endif
