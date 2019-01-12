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
  int write_frame(u8 *data, int len);
  int read_frame(u8 *data);
  void close_port(void);
  int que_maxlen_;
  int que_num_;

 private:
  int fp_;
  int state_;
  pthread_t thread_id_;
  QueueMemcpy *rx_que_;
  int init_serial(const char *port, int baud);
  int read_char(u8 *ch);
  int read_str(u8 *data, char eol, u8 len);
  int write_char(u8 ch);
  void parse_put(u8 *data, u16 len);

  typedef enum _UXBUS_RECV_STATE {
    UXBUS_START_FROMID = 0,
    UXBUS_START_TOOID = 1,
    UXBUS_STATE_LENGTH = 2,
    UXBUS_STATE_DATA = 3,
    UXBUS_STATE_CRC1 = 4,
    UXBUS_STATE_CRC2 = 5,
  } UXBUS_RECV_STATE;

  u8 UXBUS_PROT_FROMID_;
  u8 UXBUS_PROT_TOID_;

  u16 rx_data_idx_;
  u8 rx_state_;
  u8 rx_buf_[128];
  u16 rx_length_;
};

#endif
