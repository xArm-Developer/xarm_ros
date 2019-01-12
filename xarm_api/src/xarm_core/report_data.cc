/* Copyright 2017 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jimy Zhang <jimy92@163.com>
 ============================================================================*/
#include "xarm/report_data.h"

#include <stdlib.h>
#include <string.h>

#include "xarm/debug/debug_print.h"

ReportDataNorm::ReportDataNorm(void) {
  runing_ = 0;
  mode_ = 0;
  mt_brake_ = 0;
  mt_able_ = 0;
  err_ = 0;
  war_ = 0;
  cmdnum_ = 0;
  total_num_ = 0;

  memset(angle_, 0, sizeof(angle_));
  memset(pose_, 0, sizeof(pose_));
  memset(tcp_offset_, 0, sizeof(tcp_offset_));
}

ReportDataNorm::~ReportDataNorm(void) {}

int ReportDataNorm::flush_data(u8 *rx_data) {
  u8 *data_fp = &rx_data[4];
  int sizeof_data = bin8_to_32(rx_data);
  if (sizeof_data < 87) 
  {
      printf(" len = %d \n", sizeof_data);
      return -1;
  }

  total_num_ = bin8_to_32(data_fp);
  if (total_num_ != 87) return -2;

  runing_ = data_fp[4] & 0x0F;
  mode_ = data_fp[4] >> 4;
  mt_brake_ = data_fp[5];
  mt_able_ = data_fp[6];
  err_ = data_fp[7];
  war_ = data_fp[8];

  hex_to_nfp32(&data_fp[9], angle_, 7);
  hex_to_nfp32(&data_fp[37], pose_, 6);
  cmdnum_ = bin8_to_16(&data_fp[61]);
  hex_to_nfp32(&data_fp[63], tcp_offset_, 6);
  return 0;
}

void ReportDataNorm::print_data(void) {
  printf("total   = %d\n", total_num_);
  printf("runing  = %d\n", runing_);
  printf("mode    = %d\n", mode_);
  printf("mt_brake= %x\n", mt_brake_);
  printf("mt_able = %x\n", mt_able_);
  printf("err&war = %d %d\n", err_, war_);
  printf("cmdnum  = %d\n", cmdnum_);
  print_nvect("angle   = ", angle_, 7);
  print_nvect("pose    = ", pose_, 6);
  print_nvect("offset  = ", tcp_offset_, 6);
}

ReportDataRich::ReportDataRich(void) {
  runing_ = 0;
  mode_ = 0;
  mt_brake_ = 0;
  mt_able_ = 0;
  err_ = 0;
  war_ = 0;
  cmdnum_ = 0;
  total_num_ = 0;

  memset(angle_, 0, sizeof(angle_));
  memset(pose_, 0, sizeof(pose_));
  memset(tcp_offset_, 0, sizeof(tcp_offset_));
}

ReportDataRich::~ReportDataRich(void) {}

int ReportDataRich::flush_data(u8 *rx_data) {
  u8 *data_fp = &rx_data[4];
  int sizeof_data = bin8_to_32(rx_data);
  if (sizeof_data < 187) {
    printf("sizeof_data = %d\n", sizeof_data);
    return -1;
  }

  total_num_ = bin8_to_32(data_fp);
  if (total_num_ != 187) return -2;

  runing_ = data_fp[4] & 0x0F;
  mode_ = data_fp[4] >> 4;
  mt_brake_ = data_fp[5];
  mt_able_ = data_fp[6];
  err_ = data_fp[7];
  war_ = data_fp[8];

  hex_to_nfp32(&data_fp[9], angle_, 7);
  hex_to_nfp32(&data_fp[37], pose_, 6);
  cmdnum_ = bin8_to_16(&data_fp[61]);
  hex_to_nfp32(&data_fp[63], tcp_offset_, 6);

  arm_type_ = data_fp[87];
  axis_num_ = data_fp[88];
  master_id_ = data_fp[89];
  slave_id_ = data_fp[90];
  motor_tid_ = data_fp[91];
  motor_fid_ = data_fp[92];
  memcpy(versions_, &data_fp[93], 30);

  hex_to_nfp32(&data_fp[123], trs_msg_, 5);
  trs_jerk_ = trs_msg_[0];
  trs_accmin_ = trs_msg_[1];
  trs_accmax_ = trs_msg_[2];
  trs_velomin_ = trs_msg_[3];
  trs_velomax_ = trs_msg_[4];

  hex_to_nfp32(&data_fp[143], p2p_msg_, 5);
  p2p_jerk_ = p2p_msg_[0];
  p2p_accmin_ = p2p_msg_[1];
  p2p_accmax_ = p2p_msg_[2];
  p2p_velomin_ = p2p_msg_[3];
  p2p_velomax_ = p2p_msg_[4];

  hex_to_nfp32(&data_fp[163], rot_msg_, 2);
  rot_jerk_ = rot_msg_[0];
  rot_accmax_ = rot_msg_[1];

  for (int i = 0; i < 17; i++) sv3msg_[i] = data_fp[171 + i];

  return 0;
}

void ReportDataRich::print_data(void) {
  printf("versions= %s\n", versions_);
  printf("total   = %d\n", total_num_);
  printf("runing  = %d\n", runing_);
  printf("mode    = %d\n", mode_);
  printf("mt_brake= %x\n", mt_brake_);
  printf("mt_able = %x\n", mt_able_);
  printf("err&war = %d %d\n", err_, war_);
  printf("cmdnum  = %d\n", cmdnum_);
  print_nvect("angle   = ", angle_, 7);
  print_nvect("pose    = ", pose_, 6);
  print_nvect("offset  = ", tcp_offset_, 6);

  printf("xarm_type = %d(axis%d)\n", arm_type_, axis_num_);
  printf("xarm_msid = 0x%X 0x%X\n", master_id_, slave_id_);
  printf("motor_tfid = 0x%X 0x%X\n", motor_tid_, motor_fid_);

  print_nvect("trs_msg = ", trs_msg_, 5);
  print_nvect("p2p_msg = ", p2p_msg_, 5);
  print_nvect("ros_msg = ", rot_msg_, 2);

  printf("ID   执行状态  错误代码\n");
  for (int i = 0; i < 8; i++) {
    printf("%d      %d        0x%X\n", i + 1, sv3msg_[i * 2],
           sv3msg_[i * 2 + 1]);
  }
}
