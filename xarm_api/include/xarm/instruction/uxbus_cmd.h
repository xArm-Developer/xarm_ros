/* Copyright 2017 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jimy Zhang <jimy92@163.com>
 ============================================================================*/
#ifndef INSTRUCTION_UXBUS_CMD_H_
#define INSTRUCTION_UXBUS_CMD_H_

#include "xarm/common/data_type.h"

class UxbusCmd {
public:
  UxbusCmd(void);
  ~UxbusCmd(void);

  int get_version(u8 rx_data[40]);
  int motion_en(u8 id, u8 value);
  int set_state(u8 value);
  int get_state(u8 *rx_data);
  int get_cmdnum(u16 *rx_data);
  int get_errcode(u8 *rx_data);
  int clean_err(void);
  int clean_war(void);
  int set_brake(u8 axis, u8 en);
  int set_mode(u8 value);
  int move_line(fp32 mvpose[6], fp32 mvvelo, fp32 mvacc, fp32 mvtime);
  int move_lineb(fp32 mvpose[6], fp32 mvvelo, fp32 mvacc, fp32 mvtime,
                 fp32 mvradii);
  int move_joint(fp32 mvjoint[7], fp32 mvvelo, fp32 mvacc, fp32 mvtime);
  int move_gohome(fp32 mvvelo, fp32 mvacc, fp32 mvtime);
  int move_servoj(fp32 mvjoint[7], fp32 mvvelo, fp32 mvacc, fp32 mvtime);
  int sleep_instruction(fp32 sltime);
  int set_tcp_jerk(fp32 jerk);
  int set_tcp_maxacc(fp32 maxacc);
  int set_joint_jerk(fp32 jerk);
  int set_joint_maxacc(fp32 maxacc);
  int set_tcp_offset(fp32 pose_offset[6]);
  int clean_conf(void);
  int save_conf(void);
  int get_tcp_pose(fp32 pose[6]);
  int get_joint_pose(fp32 angles[7]);
  int get_ik(fp32 pose[6], fp32 angles[7]);
  int get_fk(fp32 angles[7], fp32 pose[6]);
  int is_joint_limit(fp32 joint[7], int *value);
  int is_tcp_limit(fp32 pose[6], int *value);
  int gripper_addr_w16(u8 id, u16 addr, fp32 value);
  int gripper_addr_r16(u8 id, u16 addr, fp32 *value);
  int gripper_addr_w32(u8 id, u16 addr, fp32 value);
  int gripper_addr_r32(u8 id, u16 addr, fp32 *value);
  int gripper_set_en(u16 value);
  int gripper_set_mode(u16 value);
  int gripper_set_zero(void);
  int gripper_get_pos(fp32 *pulse);
  int gripper_set_pos(fp32 pulse);
  int gripper_set_posspd(fp32 speed);
  int gripper_get_errcode(u8 rx_data[2]);
  int gripper_clean_err(void);

  int gpio_get_digital(int *io1, int *io2);
  int gpio_set_digital(int ionum, int value);
  int gpio_get_analog1(float *value);
  int gpio_get_analog2(float *value);

  int servo_set_zero(u8 id);
  int servo_get_dbmsg(u8 rx_data[16]);
  int servo_addr_w16(u8 id, u16 addr, fp32 value);
  int servo_addr_r16(u8 id, u16 addr, fp32 *value);
  int servo_addr_w32(u8 id, u16 addr, fp32 value);
  int servo_addr_r32(u8 id, u16 addr, fp32 *value);
  virtual void close(void);

private:
  virtual int check_xbus_prot(u8 *data, u8 funcode);
  virtual int send_pend(u8 funcode, int num, int timeout, u8 *rx_data);
  virtual int send_xbus(u8 funcode, u8 *txdata, int num);
  int set_nu8(u8 funcode, u8 *datas, int num);
  int get_nu8(u8 funcode, u8 num, u8 *rx_data);
  int get_nu16(u8 funcode, u8 num, u16 *rx_data);
  int set_nfp32(u8 funcode, fp32 *datas, u8 num);
  int get_nfp32(u8 funcode, u8 num, fp32 *rx_data);
  int swop_nfp32(u8 funcode, fp32 tx_datas[], u8 txn, fp32 *rx_data, u8 rxn);
  int is_nfp32(u8 funcode, fp32 datas[], u8 txn, int *value);
};

#endif
