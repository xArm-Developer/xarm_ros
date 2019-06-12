/* Copyright 2017 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jimy Zhang <jimy92@163.com>
 ============================================================================*/
#include "xarm/port/serial.h"

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/shm.h>
#include <sys/time.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#include "xarm/common/crc16.h"
#include "xarm/linux/thread.h"

void SerialPort::recv_proc(void) {
  unsigned char ch;
  int ret;
  while (state_ == 0) {
    ret = read_char(&ch);

    if (ret >= 0) {
      parse_put(&ch, 1);
      continue;
    }
    usleep(1000);
  }
}

static void *recv_proc_(void *arg) {
  SerialPort *my_this = (SerialPort *)arg;

  my_this->recv_proc();

  pthread_exit(0);
}

SerialPort::SerialPort(const char *port, int baud, int que_num,
                       int que_maxlen) {
  que_num_ = que_num;
  que_maxlen_ = que_maxlen;
  rx_que_ = new QueueMemcpy(que_num_, que_maxlen_);

  int ret = init_serial(port, baud);
  if (ret == -1)
  { state_ = -1; }
  else
  { state_ = 0; }

  if (state_ == -1) { return; }

  UXBUS_PROT_FROMID_ = 0x55;
  UXBUS_PROT_TOID_ = 0xAA;
  flush();
  thread_id_ = thread_init(recv_proc_, this);
}

SerialPort::~SerialPort(void) {
  state_ = -1;
  delete rx_que_;
}

int SerialPort::is_ok(void) { return state_; }

void SerialPort::flush(void) {
  rx_que_->flush();
  rx_data_idx_ = 0;
  rx_state_ = UXBUS_START_FROMID;
}

int SerialPort::read_char(unsigned char *ch) { return (read(fp_, ch, 1) == 1) ? 0 : -1; }

int SerialPort::read_frame(unsigned char *data) {
  if (state_ != 0) { return -1; }

  if (rx_que_->size() == 0) { return -1; }

  rx_que_->pop(data);
  return 0;
}

int SerialPort::write_char(unsigned char ch) {
  return ((write(fp_, &ch, 1) == 1) ? 0 : -1);
}

int SerialPort::write_frame(unsigned char *data, int len) {
  if (write(fp_, data, len) != len) { return -1; }
  return 0;
}

void SerialPort::close_port(void) {
  state_ = -1;
  close(fp_);
  delete rx_que_;
}

void SerialPort::parse_put(unsigned char *data, int len) {
  unsigned char ch;

  for (int i = 0; i < len; i++) {
    ch = data[i];
    // printf("---state = %d, ch = %x\n", rx_state_, ch);
    switch (rx_state_) {
    case UXBUS_START_FROMID:
      if (UXBUS_PROT_FROMID_ == ch) {
        rx_buf_[0] = ch;
        rx_state_ = UXBUS_START_TOOID;
      }
      break;

    case UXBUS_START_TOOID:
      if (UXBUS_PROT_TOID_ == ch) {
        rx_buf_[1] = ch;
        rx_state_ = UXBUS_STATE_LENGTH;
      } else {
        rx_state_ = UXBUS_START_FROMID;
      }
      break;

    case UXBUS_STATE_LENGTH:
      if (0 < ch && ch < (que_maxlen_ - 5)) {
        rx_buf_[2] = ch;
        rx_length_ = ch;
        rx_data_idx_ = 3;
        rx_state_ = UXBUS_STATE_DATA;
      } else {
        rx_state_ = UXBUS_START_FROMID;
      }
      break;

    case UXBUS_STATE_DATA:
      if (rx_data_idx_ < rx_length_ + 3) {
        rx_buf_[rx_data_idx_++] = ch;
        if (rx_data_idx_ == rx_length_ + 3) {
          rx_state_ = UXBUS_STATE_CRC1;
        }
      } else {
        rx_state_ = UXBUS_START_FROMID;
      }
      break;

    case UXBUS_STATE_CRC1:
      rx_buf_[rx_length_ + 3] = ch;
      rx_state_ = UXBUS_STATE_CRC2;
      break;

    case UXBUS_STATE_CRC2:
      int crc, crc_r;
      rx_buf_[rx_length_ + 4] = ch;
      crc = modbus_crc(rx_buf_, rx_length_ + 3);
      crc_r = (rx_buf_[rx_length_ + 4] << 8) + rx_buf_[rx_length_ + 3];
      if (crc == crc_r) {
        rx_que_->push(rx_buf_);
      }
      rx_state_ = UXBUS_START_FROMID;
      break;

    default:
      rx_state_ = UXBUS_START_FROMID;
      break;
    }
  }
}

int SerialPort::init_serial(const char *port, int baud) {
  speed_t speed;
  struct termios options;

  fp_ = open((const char *)port, O_RDWR | O_NOCTTY);
  if (-1 == fp_) { return -1; }

  fcntl(fp_, F_SETFL, FNDELAY);
  tcgetattr(fp_, &options);
  bzero(&options, sizeof(options));

  switch (baud) {
  case 110:
    speed = B110;
    break;
  case 300:
    speed = B300;
    break;
  case 600:
    speed = B600;
    break;
  case 1200:
    speed = B1200;
    break;
  case 2400:
    speed = B2400;
    break;
  case 4800:
    speed = B4800;
    break;
  case 9600:
    speed = B9600;
    break;
  case 19200:
    speed = B19200;
    break;
  case 38400:
    speed = B38400;
    break;
  case 57600:
    speed = B57600;
    break;
  case 115200:
    speed = B115200;
    break;
  case 921600:
    speed = B921600;
    break;
  }

  cfsetispeed(&options, speed);
  cfsetospeed(&options, speed);

  options.c_oflag &= ~OPOST;
  options.c_cc[VTIME] = 200;
  options.c_cc[VMIN] = 10;
  tcsetattr(fp_, TCSANOW, &options);
  return 0;
}
