
/* Copyright 2017 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jimy Zhang <jimy92@163.com>
 ============================================================================*/
#ifndef CORE_INSTRUCTION_SERVO3_CONFIG_H_
#define CORE_INSTRUCTION_SERVO3_CONFIG_H_

class SERVO3_RG {
public:
  static const unsigned short CON_EN = 0x0100;
  static const unsigned short CON_MODE = 0x0101;
  static const unsigned short CON_DIR = 0x0102;
  static const unsigned short SV3MOD_POS = 0;
  static const unsigned short SV3MOD_SPD = 1;
  static const unsigned short SV3MOD_FOS = 2;
  static const unsigned short SV3_SAVE = 0x1000;
  static const unsigned short BRAKE = 0x0104;
  static const unsigned short GET_TEMP = 0x000E;
  static const unsigned short ERR_CODE = 0x000F;
  static const unsigned short OVER_TEMP = 0x0108;
  static const unsigned short CURR_CURR = 0x0001;
  static const unsigned short POS_KP = 0x0200;
  static const unsigned short POS_FWDKP = 0x0201;
  static const unsigned short POS_PWDTC = 0x0202;
  static const unsigned short SPD_KP = 0x0203;
  static const unsigned short SPD_KI = 0x0204;
  static const unsigned short CURR_KP = 0x090C;
  static const unsigned short CURR_KI = 0x090D;
  static const unsigned short SPD_IFILT = 0x030C;
  static const unsigned short SPD_OFILT = 0x030D;
  static const unsigned short POS_CMDILT = 0x030E;
  static const unsigned short CURR_IFILT = 0x0401;
  static const unsigned short POS_KD = 0x0205;
  static const unsigned short POS_ACCT = 0x0300;
  static const unsigned short POS_DECT = 0x0301;
  static const unsigned short POS_STHT = 0x0302;
  static const unsigned short POS_SPD = 0x0303;
  static const unsigned short MT_ID = 0x1600;
  static const unsigned short BAUDRATE = 0x0601;
  static const unsigned short SOFT_REBOOT = 0x0607;
  static const unsigned short TAGET_TOQ = 0x050a;
  static const unsigned short CURR_TOQ = 0x050c;
  static const unsigned short TOQ_SPD = 0x050e;
  static const unsigned short TAGET_POS = 0x0700;
  static const unsigned short CURR_POS = 0x0702;
  static const unsigned short HARD_VER = 0x0800;
  static const unsigned short SOFT_VER = 0x0801;
  static const unsigned short MT_TYPE = 0x0802;
  static const unsigned short MT_ZERO = 0x0817;
  static const unsigned short RESET_PVL = 0x0813;
  static const unsigned short CAL_ZERO = 0x080C;
  static const unsigned short ERR_SWITCH = 0x0910;
  static const unsigned short RESET_ERR = 0x0109;
  static const unsigned short SV3_BRO_ID = 0xFF;

  static const unsigned short MODBUS_BAUDRATE = 0x0A0B;
  static const unsigned short DIGITAL_IN = 0x0A14;
  static const unsigned short DIGITAL_OUT = 0x0A15;
  static const unsigned short ANALOG_IO1 = 0x0A16;
  static const unsigned short ANALOG_IO2 = 0x0A17;
};

#endif