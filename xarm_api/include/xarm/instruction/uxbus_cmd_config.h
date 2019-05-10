/* Copyright 2017 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jimy Zhang <jimy92@163.com>
 ============================================================================*/
#ifndef INSTRUCTION_UXBUS_CMD_CONFIG_H_
#define INSTRUCTION_UXBUS_CMD_CONFIG_H_
class UXBUS_RG {
 public:
  UXBUS_RG(void) {}
  ~UXBUS_RG(void) {}

  static const unsigned char GET_VERSION = 1;
  static const unsigned char SHUTDOWN_SYSTEM = 10;

  static const unsigned char MOTION_EN = 11;
  static const unsigned char SET_STATE = 12;
  static const unsigned char GET_STATE = 13;
  static const unsigned char GET_CMDNUM = 14;
  static const unsigned char GET_ERROR = 15;
  static const unsigned char CLEAN_ERR = 16;
  static const unsigned char CLEAN_WAR = 17;
  static const unsigned char SET_BRAKE = 18;
  static const unsigned char SET_MODE = 19;

  static const unsigned char MOVE_LINE = 21;
  static const unsigned char MOVE_LINEB = 22;
  static const unsigned char MOVE_JOINT = 23;
  static const unsigned char MOVE_HOME = 25;
  static const unsigned char SLEEP_INSTT = 26;
  static const unsigned char MOVE_CIRCLE = 27;
  static const unsigned char MOVE_SERVOJ = 29;

  static const unsigned char SET_TCP_JERK = 31;
  static const unsigned char SET_TCP_MAXACC = 32;
  static const unsigned char SET_JOINT_JERK = 33;
  static const unsigned char SET_JOINT_MAXACC = 34;
  static const unsigned char SET_TCP_OFFSET = 35;
  static const unsigned char SET_LOAD_PARAM = 36;
  static const unsigned char SET_COLLIS_SENS = 37;
  static const unsigned char SET_TEACH_SENS = 38;
  static const unsigned char CLEAN_CONF = 39;
  static const unsigned char SAVE_CONF = 40;
  static const unsigned char SET_GRAVITY_DIR = 51;

  static const unsigned char GET_TCP_POSE = 41;
  static const unsigned char GET_JOINT_POS = 42;
  static const unsigned char GET_IK = 43;
  static const unsigned char GET_FK = 44;
  static const unsigned char IS_JOINT_LIMIT = 45;
  static const unsigned char IS_TCP_LIMIT = 46;

  static const unsigned char SERVO_W16B = 101;
  static const unsigned char SERVO_R16B = 102;
  static const unsigned char SERVO_W32B = 103;
  static const unsigned char SERVO_R32B = 104;
  static const unsigned char SERVO_ZERO = 105;
  static const unsigned char SERVO_DBMSG = 106;

  static const unsigned char GPGET_ERR = 125;
  static const unsigned char GRIPP_W16B = 127;
  static const unsigned char GRIPP_R16B = 128;
  static const unsigned char GRIPP_W32B = 129;
  static const unsigned char GRIPP_R32B = 130;

  static const unsigned char CGPIO_GET_DIGIT = 131;
  static const unsigned char CGPIO_GET_ANALOG1 = 132;
  static const unsigned char CGPIO_GET_ANALOG2 = 133;
  static const unsigned char CGPIO_SET_DIGIT = 134;
  static const unsigned char CGPIO_SET_ANALOG1 = 135;
  static const unsigned char CGPIO_SET_ANALOG2 = 136;
  static const unsigned char CGPIO_SET_IN_FUN = 137;
  static const unsigned char CGPIO_SET_OUT_FUN = 138;
  static const unsigned char CGPIO_GET_STATE = 139;
};

class UXBUS_STATE {
 public:
  UXBUS_STATE(void) {}
  ~UXBUS_STATE(void) {}
  static const int ERR_CODE = 1;
  static const int WAR_CODE = 2;
  static const int ERR_TOUT = 3;
  static const int ERR_LENG = 4;
  static const int ERR_NUM = 5;
  static const int ERR_PROT = 6;
  static const int ERR_FUN = 7;
  static const int ERR_NOTTCP = 8;
  static const int ERR_OTHER = 11;
};

class UXBUS_CONF {
 public:
  UXBUS_CONF(void) {}
  ~UXBUS_CONF(void) {}

  static const int SET_TIMEOUT = 1000;  // ms
  static const int GET_TIMEOUT = 1000;  // ms
  static const int GRIPPER_ID = 8;
  static const int GPIO_ID = 9;
  static const int MASTER_ID = 0xAA;
  static const int SLAVE_ID = 0x55;
};

class XARM_MODE {
 public:
  XARM_MODE(void) {}
  ~XARM_MODE(void) {}

  static const int POSE = 0;
  static const int SERVO = 1;
  static const int TEACH_JOINT = 2;
  static const int TEACH_CART = 3;
};

class XARM_STATE {
 public:
  XARM_STATE(void) {}
  ~XARM_STATE(void) {}

  static const int START = 0;
  static const int PAUSE = 3;
  static const int STOP = 4;
};

#endif
