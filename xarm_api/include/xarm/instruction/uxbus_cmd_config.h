/* Copyright 2017 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jimy Zhang <jimy92@163.com>
 ============================================================================*/
#ifndef CORE_INSTRUCTION_UXBUS_CMD_CONFIG_H_
#define CORE_INSTRUCTION_UXBUS_CMD_CONFIG_H_

class UXBUS_RG {
public:
	UXBUS_RG(void) {}
	~UXBUS_RG(void) {}

	static const unsigned char GET_VERSION = 1;
	static const unsigned char GET_ROBOT_SN = 2;
	static const unsigned char CHECK_VERIFY = 3;
	static const unsigned char RELOAD_DYNAMICS = 4;
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
	static const unsigned char MOVE_LINE_TOOL = 28;
	static const unsigned char MOVE_SERVOJ = 29;
	static const unsigned char MOVE_SERVO_CART = 30;

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

	static const unsigned char GET_TCP_POSE = 41;
	static const unsigned char GET_JOINT_POS = 42;
	static const unsigned char GET_IK = 43;
	static const unsigned char GET_FK = 44;
	static const unsigned char IS_JOINT_LIMIT = 45;
	static const unsigned char IS_TCP_LIMIT = 46;

	static const unsigned char SET_REDUCED_TRSV = 47;
	static const unsigned char SET_REDUCED_P2PV = 48;
	static const unsigned char GET_REDUCED_MODE = 49;
	static const unsigned char SET_REDUCED_MODE = 50;
	static const unsigned char SET_GRAVITY_DIR = 51;
	static const unsigned char SET_LIMIT_XYZ = 52;
	static const unsigned char GET_REDUCED_STATE = 53;

	static const unsigned char SET_SERVOT = 54;
	static const unsigned char GET_JOINT_TAU = 55;
	static const unsigned char SET_SAFE_LEVEL = 56;
	static const unsigned char GET_SAFE_LEVEL = 57;

	static const unsigned char SET_REDUCED_JRANGE = 58;
	static const unsigned char SET_FENSE_ON = 59;
	static const unsigned char SET_COLLIS_REB = 60;

	static const unsigned char SET_TRAJ_RECORD = 61;
	static const unsigned char SAVE_TRAJ = 62;
	static const unsigned char LOAD_TRAJ = 63;
	static const unsigned char PLAY_TRAJ = 64;
	static const unsigned char GET_TRAJ_RW_STATUS = 65;

	static const unsigned char SET_TIMER = 71;
	static const unsigned char CANCEL_TIMER = 72;
	static const unsigned char SET_WORLD_OFFSET = 73;
	static const unsigned char CNTER_RESET = 74;
	static const unsigned char CNTER_PLUS = 75;

	static const unsigned char CAL_POSE_OFFSET = 76;
  static const unsigned char GET_TCP_POSE_AA = 91;
  static const unsigned char MOVE_LINE_AA = 92;
  static const unsigned char MOVE_SERVO_CART_AA = 93;

	static const unsigned char SERVO_W16B = 101;
	static const unsigned char SERVO_R16B = 102;
	static const unsigned char SERVO_W32B = 103;
	static const unsigned char SERVO_R32B = 104;
	static const unsigned char SERVO_ZERO = 105;
	static const unsigned char SERVO_DBMSG = 106;

	static const unsigned char TGPIO_MB_TIOUT = 123;
	static const unsigned char TGPIO_MODBUS = 124;
	static const unsigned char TGPIO_ERR = 125;
	static const unsigned char TGPIO_W16B = 127;
	static const unsigned char TGPIO_R16B = 128;
	static const unsigned char TGPIO_W32B = 129;
	static const unsigned char TGPIO_R32B = 130;

	static const unsigned char CGPIO_GET_DIGIT = 131;
	static const unsigned char CGPIO_GET_ANALOG1 = 132;
	static const unsigned char CGPIO_GET_ANALOG2 = 133;
	static const unsigned char CGPIO_SET_DIGIT = 134;
	static const unsigned char CGPIO_SET_ANALOG1 = 135;
	static const unsigned char CGPIO_SET_ANALOG2 = 136;
	static const unsigned char CGPIO_SET_IN_FUN = 137;
	static const unsigned char CGPIO_SET_OUT_FUN = 138;
	static const unsigned char CGPIO_GET_STATE = 139;

	static const unsigned char GET_HD_TYPES = 141;
	static const unsigned char DELAYED_CGPIO_SET = 142;
	static const unsigned char DELAYED_TGPIO_SET = 143;
	static const unsigned char POSITION_CGPIO_SET = 144;
	static const unsigned char POSITION_TGPIO_SET = 145;
	static const unsigned char SET_IO_STOP_RESET = 146;
};

class UXBUS_STATE {
public:
	UXBUS_STATE(void) {}
	~UXBUS_STATE(void) {}
	static const int NOT_CONNECTED = -1;
	static const int NOT_READY = -2;
	static const int API_EXCEPTION = -3;
	static const int CMD_NOT_EXIST = -4;
	static const int TCP_LIMIT = -6;
	static const int JOINT_LIMIT = -7;
	static const int OUT_OF_RANGE = -8;
	static const int EMERGENCY_STOP = -9;
	static const int SERVO_NOT_EXIST = -10;
	static const int CONVERT_FAILED = -11;
	static const int ERR_CODE = 1;
	static const int WAR_CODE = 2;
	static const int ERR_TOUT = 3;
	static const int ERR_LENG = 4;
	static const int ERR_NUM = 5;
	static const int ERR_PROT = 6;
	static const int ERR_FUN = 7;
	static const int ERR_NOTTCP = 8;
	static const int ERR_OTHER = 11;
	static const int ERR_PARAM = 12;
	static const int TRAJ_RW_FAILED = 31;
	static const int TRAJ_RW_TOUT = 32;
	static const int TRAJ_PLAYBACK_TOUT = 33;
	static const int SUCTION_CUP_TOUT = 41;
};

class TRAJ_STATE {
public:
	TRAJ_STATE(void) {}
	~TRAJ_STATE(void) {}

	static const int IDLE = 0;
	static const int LOADING = 1;
	static const int LOAD_SUCCESS = 2;
	static const int LOAD_FAIL = 3;
	static const int SAVING = 4;
	static const int SAVE_SUCCESS = 5;
	static const int SAVE_FAIL = 6;
};

class UXBUS_CONF {
public:
	UXBUS_CONF(void) {}
	~UXBUS_CONF(void) {}

	static const int SET_TIMEOUT = 1000;  // ms
	static const int GET_TIMEOUT = 1000;  // ms
	static const int GRIPPER_ID = 8;
	static const int TGPIO_ID = 9;
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
