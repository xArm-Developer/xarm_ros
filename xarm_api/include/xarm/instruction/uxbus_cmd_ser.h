/* Copyright 2017 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jimy Zhang <jimy92@163.com>
 ============================================================================*/
#ifndef INSTRUCTION_UXBUS_CMD_SER_H_
#define INSTRUCTION_UXBUS_CMD_SER_H_

#include "xarm/instruction/uxbus_cmd.h"
#include "xarm/port/serial.h"

class UxbusCmdSer : public UxbusCmd {
 public:
  UxbusCmdSer(SerialPort *arm_port);
  ~UxbusCmdSer(void);

  int check_xbus_prot(unsigned char *datas, int funcode);
  int send_pend(int funcode, int num, int timeout, unsigned char *ret_data);
  int send_xbus(int funcode, unsigned char *datas, int num);
  void close(void);

 private:
  SerialPort *arm_port_;
};

#endif
