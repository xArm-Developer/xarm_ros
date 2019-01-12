/* Copyright 2017 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jimy Zhang <jimy92@163.com>
 ============================================================================*/
#include "xarm/instruction/uxbus_cmd.h"
#include "xarm/instruction/servo3_config.h"
#include "xarm/instruction/uxbus_cmd_config.h"

UxbusCmd::UxbusCmd(void) {}

UxbusCmd::~UxbusCmd(void) {}

int UxbusCmd::check_xbus_prot(u8 *data, u8 funcode) { return -11; }

int UxbusCmd::send_pend(u8 funcode, int num, int timeout, u8 *rx_data) {
  return -11;
}

int UxbusCmd::send_xbus(u8 funcode, u8 *txdata, int num) { return -11; }

void UxbusCmd::close(void) {}

int UxbusCmd::set_nu8(u8 funcode, u8 *datas, int num) {
  int ret = send_xbus(funcode, datas, num);
  if (ret != 0) return UXBUS_STATE::ERR_NOTTCP;
  ret = send_pend(funcode, 0, UXBUS_CONF::SET_TIMEOUT, NULL);

  return ret;
}

int UxbusCmd::get_nu8(u8 funcode, u8 num, u8 *rx_data) {
  int ret = send_xbus(funcode, 0, 0);
  if (ret != 0) return UXBUS_STATE::ERR_NOTTCP;
  ret = send_pend(funcode, num, UXBUS_CONF::GET_TIMEOUT, rx_data);

  return ret;
}

int UxbusCmd::get_nu16(u8 funcode, u8 num, u16 *rx_data) {
  u8 datas[num * 2];
  int ret = send_xbus(funcode, 0, 0);
  if (ret != 0) return UXBUS_STATE::ERR_NOTTCP;

  ret = send_pend(funcode, num * 2, UXBUS_CONF::GET_TIMEOUT, datas);
  for (int i = 0; i < num; i++) rx_data[i] = bin8_to_16(&datas[i * 2]);

  return ret;
}

int UxbusCmd::set_nfp32(u8 funcode, fp32 *datas, u8 num) {
  u8 hexdata[num * 4] = {0};

  nfp32_to_hex(datas, hexdata, num);
  int ret = send_xbus(funcode, hexdata, num * 4);
  if (0 != ret) return UXBUS_STATE::ERR_NOTTCP;
  ret = send_pend(funcode, 0, UXBUS_CONF::SET_TIMEOUT, NULL);

  return ret;
}

int UxbusCmd::get_nfp32(u8 funcode, u8 num, fp32 *rx_data) {
  u8 datas[num * 4] = {0};

  int ret = send_xbus(funcode, 0, 0);
  if (0 != ret) return UXBUS_STATE::ERR_NOTTCP;
  ret = send_pend(funcode, num * 4, UXBUS_CONF::GET_TIMEOUT, datas);
  hex_to_nfp32(datas, rx_data, num);

  return ret;
}

int UxbusCmd::swop_nfp32(u8 funcode, fp32 tx_datas[], u8 txn, fp32 *rx_data,
                         u8 rxn) {
  u8 hexdata[128] = {0};

  nfp32_to_hex(tx_datas, hexdata, txn);
  int ret = send_xbus(funcode, hexdata, txn * 4);
  if (0 != ret) return UXBUS_STATE::ERR_NOTTCP;
  ret = send_pend(funcode, rxn * 4, UXBUS_CONF::GET_TIMEOUT, hexdata);
  hex_to_nfp32(hexdata, rx_data, rxn);

  return ret;
}

int UxbusCmd::is_nfp32(u8 funcode, fp32 datas[], u8 txn, int *value) {
  u8 hexdata[txn * 4] = {0};

  nfp32_to_hex(datas, hexdata, txn);
  int ret = send_xbus(funcode, hexdata, txn * 4);
  if (0 != ret) return UXBUS_STATE::ERR_NOTTCP;
  ret = send_pend(funcode, 1, UXBUS_CONF::GET_TIMEOUT, hexdata);
  *value = hexdata[0];

  return ret;
}

int UxbusCmd::get_version(u8 rx_data[40]) {
  return get_nu8(UXBUS_RG::GET_VERSION, 40, rx_data);
}

int UxbusCmd::motion_en(u8 id, u8 value) {
  u8 txdata[2];
  txdata[0] = id;
  txdata[1] = value;
  int ret = set_nu8(UXBUS_RG::MOTION_EN, txdata, 2);
  return ret;
}

int UxbusCmd::set_state(u8 value) {
  u8 txdata[1];
  txdata[0] = value;
  int ret = set_nu8(UXBUS_RG::SET_STATE, txdata, 1);
  return ret;
}

int UxbusCmd::get_state(u8 *rx_data) {
  int ret = get_nu8(UXBUS_RG::GET_STATE, 1, rx_data);
  return ret;
}

int UxbusCmd::get_cmdnum(u16 *rx_data) {
  int ret = get_nu16(UXBUS_RG::GET_CMDNUM, 1, rx_data);
  return ret;
}

int UxbusCmd::get_errcode(u8 *rx_data) {
  int ret = get_nu8(UXBUS_RG::GET_ERROR, 2, rx_data);
  return ret;
}

int UxbusCmd::clean_err(void) {
  u8 txdata[1];
  txdata[0] = 0;
  int ret = set_nu8(UXBUS_RG::CLEAN_ERR, txdata, 0);
  return ret;
}

int UxbusCmd::clean_war(void) {
  u8 txdata[1];
  txdata[0] = 0;
  int ret = set_nu8(UXBUS_RG::CLEAN_WAR, txdata, 0);
  return ret;
}

int UxbusCmd::set_brake(u8 axis, u8 en) {
  u8 txdata[2] = {0};
  txdata[0] = axis;
  txdata[1] = en;
  int ret = set_nu8(UXBUS_RG::SET_BRAKE, txdata, 2);
  return ret;
}

int UxbusCmd::set_mode(u8 value) {
  u8 txdata[1];
  txdata[0] = value;
  int ret = set_nu8(UXBUS_RG::SET_MODE, txdata, 1);
  return ret;
}

int UxbusCmd::move_line(fp32 mvpose[6], fp32 mvvelo, fp32 mvacc, fp32 mvtime) {
  int i;
  fp32 txdata[9] = {0};
  for (i = 0; i < 6; i++) txdata[i] = mvpose[i];
  txdata[6] = mvvelo;
  txdata[7] = mvacc;
  txdata[8] = mvtime;
  int ret = set_nfp32(UXBUS_RG::MOVE_LINE, txdata, 9);
  return ret;
}

int UxbusCmd::move_lineb(fp32 mvpose[6], fp32 mvvelo, fp32 mvacc, fp32 mvtime,
                         fp32 mvradii) {
  int i;
  fp32 txdata[10] = {0};
  for (i = 0; i < 6; i++) txdata[i] = mvpose[i];
  txdata[6] = mvvelo;
  txdata[7] = mvacc;
  txdata[8] = mvtime;
  txdata[9] = mvradii;

  int ret = set_nfp32(UXBUS_RG::MOVE_LINEB, txdata, 10);
  return ret;
}

int UxbusCmd::move_joint(fp32 mvjoint[7], fp32 mvvelo, fp32 mvacc,
                         fp32 mvtime) {
  int i;
  fp32 txdata[10] = {0};
  for (i = 0; i < 7; i++) txdata[i] = mvjoint[i];
  txdata[7] = mvvelo;
  txdata[8] = mvacc;
  txdata[9] = mvtime;
  int ret = set_nfp32(UXBUS_RG::MOVE_JOINT, txdata, 10);
  return ret;
}

int UxbusCmd::move_gohome(fp32 mvvelo, fp32 mvacc, fp32 mvtime) {
  fp32 txdata[3] = {0};
  txdata[0] = mvvelo;
  txdata[1] = mvacc;
  txdata[2] = mvtime;
  int ret = set_nfp32(UXBUS_RG::MOVE_HOME, txdata, 3);
  return ret;
}

int UxbusCmd::move_servoj(fp32 mvjoint[7], fp32 mvvelo, fp32 mvacc,
                          fp32 mvtime) {
  int i;
  fp32 txdata[10] = {0};
  for (i = 0; i < 7; i++) txdata[i] = mvjoint[i];
  txdata[7] = mvvelo;
  txdata[8] = mvacc;
  txdata[9] = mvtime;
  int ret = set_nfp32(UXBUS_RG::MOVE_SERVOJ, txdata, 10);
  return ret;
}

int UxbusCmd::sleep_instruction(fp32 sltime) {
  fp32 txdata[1] = {0};
  txdata[0] = sltime;
  int ret = set_nfp32(UXBUS_RG::SLEEP_INSTT, txdata, 1);
  return ret;
}

int UxbusCmd::set_tcp_jerk(fp32 jerk) {
  fp32 txdata[1] = {0};
  txdata[0] = jerk;
  int ret = set_nfp32(UXBUS_RG::SET_TCP_JERK, txdata, 1);
  return ret;
}

int UxbusCmd::set_tcp_maxacc(fp32 maxacc) {
  fp32 txdata[1] = {0};
  txdata[0] = maxacc;
  int ret = set_nfp32(UXBUS_RG::SET_TCP_MAXACC, txdata, 1);
  return ret;
}

int UxbusCmd::set_joint_jerk(fp32 jerk) {
  fp32 txdata[1] = {0};
  txdata[0] = jerk;
  int ret = set_nfp32(UXBUS_RG::SET_JOINT_JERK, txdata, 1);
  return ret;
}

int UxbusCmd::set_joint_maxacc(fp32 maxacc) {
  fp32 txdata[1] = {0};
  txdata[0] = maxacc;
  int ret = set_nfp32(UXBUS_RG::SET_JOINT_MAXACC, txdata, 1);
  return ret;
}

int UxbusCmd::set_tcp_offset(fp32 pose_offset[6]) {
  return set_nfp32(UXBUS_RG::SET_TCP_OFFSET, pose_offset, 6);
}

int UxbusCmd::clean_conf() { return set_nu8(UXBUS_RG::CLEAN_CONF, 0, 0); }

int UxbusCmd::save_conf() { return set_nu8(UXBUS_RG::SAVE_CONF, 0, 0); }

int UxbusCmd::get_tcp_pose(fp32 pose[6]) {
  return get_nfp32(UXBUS_RG::GET_TCP_POSE, 6, pose);
}

int UxbusCmd::get_joint_pose(fp32 angles[7]) {
  return get_nfp32(UXBUS_RG::GET_JOINT_POS, 7, angles);
}

int UxbusCmd::get_ik(fp32 pose[6], fp32 angles[7]) {
  return swop_nfp32(UXBUS_RG::GET_IK, pose, 6, angles, 7);
}

int UxbusCmd::get_fk(fp32 angles[7], fp32 pose[6]) {
  return swop_nfp32(UXBUS_RG::GET_FK, angles, 7, pose, 6);
}

int UxbusCmd::is_joint_limit(fp32 joint[7], int *value) {
  return is_nfp32(UXBUS_RG::IS_JOINT_LIMIT, joint, 7, value);
}

int UxbusCmd::is_tcp_limit(fp32 pose[6], int *value) {
  return is_nfp32(UXBUS_RG::IS_TCP_LIMIT, pose, 6, value);
}

int UxbusCmd::gripper_addr_w16(u8 id, u16 addr, fp32 value) {
  u8 txdata[7];
  txdata[0] = id;
  bin16_to_8(addr, &txdata[1]);
  fp32_to_hex(value, &txdata[3]);
  int ret = send_xbus(UXBUS_RG::GRIPP_W16B, txdata, 7);
  if (0 != ret) return UXBUS_STATE::ERR_NOTTCP;

  ret = send_pend(UXBUS_RG::GRIPP_W16B, 0, UXBUS_CONF::GET_TIMEOUT, NULL);
  return ret;
}

int UxbusCmd::gripper_addr_r16(u8 id, u16 addr, fp32 *value) {
  u8 txdata[3], rx_data[4];
  txdata[0] = id;
  bin16_to_8(addr, &txdata[1]);
  int ret = send_xbus(UXBUS_RG::GRIPP_R16B, txdata, 3);
  if (0 != ret) return UXBUS_STATE::ERR_NOTTCP;

  ret = send_pend(UXBUS_RG::GRIPP_R16B, 4, UXBUS_CONF::GET_TIMEOUT, rx_data);
  *value = bin8_to_16(rx_data);
  return ret;
}

int UxbusCmd::gripper_addr_w32(u8 id, u16 addr, fp32 value) {
  u8 txdata[7];
  txdata[0] = id;
  bin16_to_8(addr, &txdata[1]);
  fp32_to_hex(value, &txdata[3]);
  int ret = send_xbus(UXBUS_RG::GRIPP_W32B, txdata, 7);
  if (0 != ret) return UXBUS_STATE::ERR_NOTTCP;

  ret = send_pend(UXBUS_RG::GRIPP_W32B, 0, UXBUS_CONF::GET_TIMEOUT, NULL);
  return ret;
}

int UxbusCmd::gripper_addr_r32(u8 id, u16 addr, fp32 *value) {
  u8 txdata[3], rx_data[4];
  txdata[0] = id;
  bin16_to_8(addr, &txdata[1]);
  int ret = send_xbus(UXBUS_RG::GRIPP_R32B, txdata, 3);
  if (0 != ret) return UXBUS_STATE::ERR_NOTTCP;

  ret = send_pend(UXBUS_RG::GRIPP_R32B, 4, UXBUS_CONF::GET_TIMEOUT, rx_data);
  *value = bin8_to_32(rx_data);
  return ret;
}

int UxbusCmd::gripper_set_en(u16 value) {
  return gripper_addr_w16(UXBUS_CONF::GRIPPER_ID, SERVO3_RG::CON_EN, value);
}

int UxbusCmd::gripper_set_mode(u16 value) {
  return gripper_addr_w16(UXBUS_CONF::GRIPPER_ID, SERVO3_RG::CON_MODE, value);
}

int UxbusCmd::gripper_set_zero() {
  return gripper_addr_w16(UXBUS_CONF::GRIPPER_ID, SERVO3_RG::MT_ZERO, 1);
}

int UxbusCmd::gripper_get_pos(fp32 *pulse) {
  return gripper_addr_r32(UXBUS_CONF::GRIPPER_ID, SERVO3_RG::CURR_POS, pulse);
}

int UxbusCmd::gripper_set_pos(fp32 pulse) {
  return gripper_addr_w32(UXBUS_CONF::GRIPPER_ID, SERVO3_RG::TAGET_POS, pulse);
}

int UxbusCmd::gripper_set_posspd(fp32 speed) {
  return gripper_addr_w16(UXBUS_CONF::GRIPPER_ID, SERVO3_RG::POS_SPD, speed);
}

int UxbusCmd::gripper_get_errcode(u8 rx_data[2]) {
  return get_nu8(UXBUS_RG::GPGET_ERR, 2, rx_data);
}

int UxbusCmd::gripper_clean_err() {
  return gripper_addr_w16(UXBUS_CONF::GRIPPER_ID, SERVO3_RG::RESET_ERR, 1);
}

int UxbusCmd::servo_set_zero(u8 id) {
  u8 txdata[1];
  txdata[0] = id;
  int ret = set_nu8(UXBUS_RG::SERVO_ZERO, txdata, 1);
  return ret;
}

int UxbusCmd::servo_get_dbmsg(u8 rx_data[16]) {
  int ret = get_nu8(UXBUS_RG::SERVO_DBMSG, 16, rx_data);
  return ret;
}

int UxbusCmd::servo_addr_w16(u8 id, u16 addr, fp32 value) {
  u8 txdata[7];
  txdata[0] = id;
  bin16_to_8(addr, &txdata[1]);
  fp32_to_hex(value, &txdata[3]);
  int ret = send_xbus(UXBUS_RG::SERVO_W16B, txdata, 7);
  if (0 != ret) return UXBUS_STATE::ERR_NOTTCP;

  ret = send_pend(UXBUS_RG::SERVO_W16B, 0, UXBUS_CONF::GET_TIMEOUT, NULL);
  return ret;
}

int UxbusCmd::servo_addr_r16(u8 id, u16 addr, fp32 *value) {
  u8 txdata[3], rx_data[4];
  txdata[0] = id;
  bin16_to_8(addr, &txdata[1]);
  int ret = send_xbus(UXBUS_RG::SERVO_R16B, txdata, 3);
  if (0 != ret) return UXBUS_STATE::ERR_NOTTCP;

  ret = send_pend(UXBUS_RG::SERVO_R16B, 4, UXBUS_CONF::GET_TIMEOUT, rx_data);
  *value = bin8_to_16(rx_data);
  return ret;
}

int UxbusCmd::servo_addr_w32(u8 id, u16 addr, fp32 value) {
  u8 txdata[7];
  txdata[0] = id;
  bin16_to_8(addr, &txdata[1]);
  fp32_to_hex(value, &txdata[3]);
  int ret = send_xbus(UXBUS_RG::SERVO_W32B, txdata, 7);
  if (0 != ret) return UXBUS_STATE::ERR_NOTTCP;

  ret = send_pend(UXBUS_RG::SERVO_W32B, 0, UXBUS_CONF::GET_TIMEOUT, NULL);
  return ret;
}

int UxbusCmd::servo_addr_r32(u8 id, u16 addr, fp32 *value) {
  u8 txdata[3], rx_data[4];
  txdata[0] = id;
  bin16_to_8(addr, &txdata[1]);
  int ret = send_xbus(UXBUS_RG::SERVO_R32B, txdata, 3);
  if (0 != ret) return UXBUS_STATE::ERR_NOTTCP;

  ret = send_pend(UXBUS_RG::SERVO_R32B, 4, UXBUS_CONF::GET_TIMEOUT, rx_data);
  *value = bin8_to_32(rx_data);
  return ret;
}
