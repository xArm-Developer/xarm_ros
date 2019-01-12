/* Copyright 2017 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jimy Zhang <jimy92@163.com>
 ============================================================================*/
#ifndef INSTRUCTION_UXBUS_CMD_TCP_H_
#define INSTRUCTION_UXBUS_CMD_TCP_H_

#include "xarm/instruction/uxbus_cmd.h"
#include "xarm/port/socket.h"

class UxbusCmdTcp : public UxbusCmd {
 public:
  UxbusCmdTcp(SocketPort *arm_port);
  ~UxbusCmdTcp(void);

  int check_xbus_prot(u8 *datas, u8 funcode);
  int send_pend(u8 funcode, int num, int timeout, u8 *ret_data);
  int send_xbus(u8 funcode, u8 *datas, int num);
  void close(void);

 private:
  SocketPort *arm_port_;
  int bus_flag_;
  int prot_flag_;
  int TX2_PROT_CON_ = 2;         // tcp cmd prot
  int TX2_PROT_HEAT_ = 1;        // tcp heat prot
  int TX2_BUS_FLAG_MIN_ = 1;     // cmd序号 起始值
  int TX2_BUS_FLAG_MAX_ = 5000;  // cmd序号 最大值
};

#endif
