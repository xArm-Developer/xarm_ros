/* Copyright 2017 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jimy Zhang <jimy92@163.com>
 ============================================================================*/
class XARM_CONF {
public:
  XARM_CONF() {}
  ~XARM_CONF() {}

  static const int AXIS_NUM = 7;
  static const int GRIPPER_ID = 8;
  static const int GPIO_ID = 9;
  static const int SERIAL_BAUD = 921600;
  static const int TCP_PORT_CONTROL = 502;
  static const int TCP_PORT_REPORT_NORM = 30001;
  static const int TCP_PORT_REPORT_RICH = 30002;
  static const int TCP_PORT_REPORT_DEVL = 30003;
};
