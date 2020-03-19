/* Copyright 2017 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jimy Zhang <jimy92@163.com>
 ============================================================================*/
#ifndef CORE_INSTRUCTION_UXBUS_CMD_H_
#define CORE_INSTRUCTION_UXBUS_CMD_H_

#include "xarm/common/data_type.h"

class UxbusCmd {
 public:
  UxbusCmd(void);
  ~UxbusCmd(void);

  int get_version(unsigned char rx_data[40]);
  int get_robot_sn(unsigned char rx_data[40]);
  int shutdown_system(int value);
  int motion_en(int id, int value);
  int set_state(int value);
  int get_state(int *rx_data);
  int get_cmdnum(int *rx_data);
  int get_err_code(int *rx_data);
  int clean_err(void);
  int clean_war(void);
  int set_brake(int axis, int en);
  int set_mode(int value);
  int move_line(float mvpose[6], float mvvelo, float mvacc, float mvtime);
  int move_lineb(float mvpose[6], float mvvelo, float mvacc, float mvtime,
                 float mvradii);
  int move_joint(float mvjoint[7], float mvvelo, float mvacc, float mvtime);
  int move_gohome(float mvvelo, float mvacc, float mvtime);
  int move_servoj(float mvjoint[7], float mvvelo, float mvacc, float mvtime);
  int move_servo_cartesian(float mvpose[6], float mvvelo, float mvacc, float mvtime);
  int sleep_instruction(float sltime);
  int move_circle(float pose1[6], float pose2[6], float mvvelo, float mvacc, float mvtime, float percent);
  int set_tcp_jerk(float jerk);
  int set_tcp_maxacc(float maxacc);
  int set_joint_jerk(float jerk);
  int set_joint_maxacc(float maxacc);
  int set_tcp_offset(float pose_offset[6]);
  int set_tcp_load(float mass, float load_offset[3]);
  int set_collis_sens(int value);
  int set_teach_sens(int value);
  int set_gravity_dir(float gravity_dir[3]);
  int clean_conf(void);
  int save_conf(void);

  int get_tcp_pose(float pose[6]);
  int get_joint_pose(float angles[7]);
  int get_ik(float pose[6], float angles[7]);
  int get_fk(float angles[7], float pose[6]);
  int is_joint_limit(float joint[7], int *value);
  int is_tcp_limit(float pose[6], int *value);
  int gripper_addr_w16(int addr, float value);
  int gripper_addr_r16(int addr, float *value);
  int gripper_addr_w32(int addr, float value);
  int gripper_addr_r32(int addr, float *value);
  int gripper_set_en(int value);
  int gripper_set_mode(int value);
  int gripper_set_zero(void);
  int gripper_get_pos(float *pulse);
  int gripper_set_pos(float pulse);
  int gripper_set_posspd(float speed);
  int gripper_get_errcode(int rx_data[2]);
  int gripper_clean_err(void);

  int tgpio_addr_w16(int addr, float value);
  int tgpio_addr_r16(int addr, float *value);
  int tgpio_addr_w32(int addr, float value);
  int tgpio_addr_r32(int addr, float *value);
  int tgpio_get_digital(int *io1, int *io2);
  int tgpio_set_digital(int ionum, int value);
  int tgpio_get_analog1(float *value);
  int tgpio_get_analog2(float *value);

  int tgpio_set_modbus(unsigned char *modbus_t, int len_t, unsigned char *ret_data);
  int gripper_modbus_w16s(int addr, float value, int len);
  int gripper_modbus_r16s(int addr, int len, unsigned char *rx_data);
  int gripper_modbus_set_en(int value);
  int gripper_modbus_set_mode(int value);
  int gripper_modbus_set_zero(void);
  int gripper_modbus_get_pos(float *pulse);
  int gripper_modbus_set_pos(float pulse);
  int gripper_modbus_set_posspd(float speed);
  int gripper_modbus_get_errcode(int *err);
  int gripper_modbus_clean_err(void);

  int servo_set_zero(int id);
  int servo_get_dbmsg(int rx_data[16]);
  int servo_addr_w16(int id, int addr, float value);
  int servo_addr_r16(int id, int addr, float *value);
  int servo_addr_w32(int id, int addr, float value);
  int servo_addr_r32(int id, int addr, float *value);


  int cgpio_get_auxdigit(int *value);
  int cgpio_get_analog1(float *value);
  int cgpio_get_analog2(float *value);
  int cgpio_set_auxdigit(int ionum, int value);
  int cgpio_set_analog1(int value);
  int cgpio_set_analog2(int value);
  int cgpio_set_infun(int num, int fun);
  int cgpio_set_outfun(int num, int fun);
  int cgpio_get_state(int *state, int *digit_io, float *analog, int *input_conf, int *output_conf);

  virtual void close(void);

 private:
  virtual int check_xbus_prot(unsigned char *data, int funcode);
  virtual int send_pend(int funcode, int num, int timeout, unsigned char *rx_data);
  virtual int send_xbus(int funcode, unsigned char *txdata, int num);
  int set_nu8(int funcode, int *datas, int num);
  int get_nu8(int funcode, int *rx_data, int num);
  int get_nu8(int funcode, unsigned char *rx_data, int num);
  int set_nu16(int funcode, int *datas, int num);
  int get_nu16(int funcode, int *rx_data, int num);
  int set_nfp32(int funcode, float *datas, int num);
  int get_nfp32(int funcode, float *rx_data, int num);
  int swop_nfp32(int funcode, float tx_datas[], int txn, float *rx_data, int rxn);
  int is_nfp32(int funcode, float datas[], int txn, int *value);
};

#endif