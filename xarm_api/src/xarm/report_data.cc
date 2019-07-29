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

ReportDataDevelop::ReportDataDevelop(void) {
  runing_ = 0;
  mode_ = 0;
  cmdnum_ = 0;
  total_num_ = 0;

  memset(angle_, 0, sizeof(angle_));
  memset(pose_, 0, sizeof(pose_));
  memset(tau_, 0, sizeof(tau_));
}

ReportDataDevelop::~ReportDataDevelop(void) {}

int ReportDataDevelop::flush_data(unsigned char *rx_data) {
  unsigned char *data_fp = &rx_data[4];
  int sizeof_data = bin8_to_32(rx_data);
  if (sizeof_data < 87) { return -1; }

  total_num_ = bin8_to_32(data_fp);
  if (total_num_ != 87) { return -2; }

  runing_ = data_fp[4] & 0x0F;
  mode_ = data_fp[4] >> 4;
  cmdnum_ = bin8_to_16(&data_fp[5]);

  hex_to_nfp32(&data_fp[7], angle_, 7);
  hex_to_nfp32(&data_fp[35], pose_, 6);
  hex_to_nfp32(&data_fp[59], tau_, 7);
  return 0;
}

void ReportDataDevelop::print_data(void) {
  printf("total  = %d\n", total_num_);
  printf("runing = %d\n", runing_);
  printf("mode   = %d\n", mode_);
  printf("cmdnum = %d\n", cmdnum_);
  print_nvect("angle  = ", angle_, 7);
  print_nvect("pose   = ", pose_, 6);
  print_nvect("tau    = ", tau_, 7);
}



ReportDataNorm::ReportDataNorm(void) {
  runing_ = 0;
  mode_ = 0;
  cmdnum_ = 0;
  memset(angle_, 0, sizeof(angle_));
  memset(pose_, 0, sizeof(pose_));
  memset(tau_, 0, sizeof(tau_));

  mt_brake_ = 0;
  mt_able_ = 0;
  err_ = 0;
  war_ = 0;
  memset(tcp_offset_, 0, sizeof(tcp_offset_));
  memset(tcp_load_, 0, sizeof(tcp_load_));
  collis_sens_ = 0;
  teach_sens_ = 0;
  total_num_ = 0;
}

ReportDataNorm::~ReportDataNorm(void) {}

int ReportDataNorm::flush_data(unsigned char *rx_data) {
  unsigned char *data_fp = &rx_data[4];
  int sizeof_data = bin8_to_32(rx_data);
  if (sizeof_data < 133) {
    printf("sizeof_data = %d\n", sizeof_data);
    return -1;
  }

  total_num_ = bin8_to_32(data_fp);
  // 133: legacy bug, actuall total number (data part) is 145 bytes 
  if (total_num_ != 133 && total_num_ != 145) { return -2; }

  runing_ = data_fp[4] & 0x0F;
  mode_ = data_fp[4] >> 4;
  cmdnum_ = bin8_to_16(&data_fp[5]);
  hex_to_nfp32(&data_fp[7], angle_, 7);
  hex_to_nfp32(&data_fp[35], pose_, 6);
  hex_to_nfp32(&data_fp[59], tau_, 7);

  mt_brake_ = data_fp[87];
  mt_able_ = data_fp[88];
  err_ = data_fp[89];
  war_ = data_fp[90];

  hex_to_nfp32(&data_fp[91], tcp_offset_, 6);
  hex_to_nfp32(&data_fp[115], tcp_load_, 4);

  collis_sens_ = data_fp[131];
  teach_sens_ = data_fp[132];
  hex_to_nfp32(&data_fp[133], gravity_dir_, 3); // length is 12, not taken into account in legacy version.(before firmware v1.1)
  
  return 0;
}

void ReportDataNorm::print_data(void) {
  printf("total   = %d\n", total_num_);
  printf("runing  = %d\n", runing_);
  printf("mode    = %d\n", mode_);
  printf("cmdnum  = %d\n", cmdnum_);
  print_nvect("angle   = ", angle_, 7);
  print_nvect("pose    = ", pose_, 6);
  print_nvect("tau     = ", tau_, 7);


  printf("mode    = %d\n", mode_);
  printf("mt_brake= %x\n", mt_brake_);
  printf("mt_able = %x\n", mt_able_);
  printf("err&war = %d %d\n", err_, war_);
  print_nvect("tcp_off = ", tcp_offset_, 6);
  print_nvect("tap_load= ", tcp_load_, 4);
  printf("coll_sen= %d\n", collis_sens_);
  printf("teac_sen= %d\n", teach_sens_);
  print_nvect("gravity_dir_= ", gravity_dir_, 3);
}



ReportDataRich::ReportDataRich(void) {
  runing_ = 0;
  mode_ = 0;
  cmdnum_ = 0;
  memset(angle_, 0, sizeof(angle_));
  memset(pose_, 0, sizeof(pose_));
  memset(tau_, 0, sizeof(tau_));

  mt_brake_ = 0;
  mt_able_ = 0;
  err_ = 0;
  war_ = 0;
  memset(tcp_offset_, 0, sizeof(tcp_offset_));
  memset(tcp_load_, 0, sizeof(tcp_load_));
  collis_sens_ = 0;
  teach_sens_ = 0;

  total_num_ = 0;
}

ReportDataRich::~ReportDataRich(void) {}

int ReportDataRich::flush_data(unsigned char *rx_data) {
  unsigned char *data_fp = &rx_data[4];
  int sizeof_data = bin8_to_32(rx_data);
  if (sizeof_data < 233) {
    printf("sizeof_data = %d\n", sizeof_data);
    return -1;
  }

  total_num_ = bin8_to_32(data_fp);
  if (total_num_ != 233) { return -2; }

  runing_ = data_fp[4] & 0x0F;
  mode_ = data_fp[4] >> 4;
  cmdnum_ = bin8_to_16(&data_fp[5]);
  hex_to_nfp32(&data_fp[7], angle_, 7);
  hex_to_nfp32(&data_fp[35], pose_, 6);
  hex_to_nfp32(&data_fp[59], tau_, 7);

  mt_brake_ = data_fp[87];
  mt_able_ = data_fp[88];
  err_ = data_fp[89];
  war_ = data_fp[90];
  hex_to_nfp32(&data_fp[91], tcp_offset_, 6);
  hex_to_nfp32(&data_fp[115], tcp_load_, 4);
  collis_sens_ = data_fp[131];
  teach_sens_ = data_fp[132];
  hex_to_nfp32(&data_fp[133], gravity_dir_, 3);

  arm_type_ = data_fp[145];
  axis_num_ = data_fp[146];
  master_id_ = data_fp[147];
  slave_id_ = data_fp[148];
  motor_tid_ = data_fp[149];
  motor_fid_ = data_fp[150];
  memcpy(versions_, &data_fp[151], 30);

  hex_to_nfp32(&data_fp[181], trs_msg_, 5);
  trs_jerk_ = trs_msg_[0];
  trs_accmin_ = trs_msg_[1];
  trs_accmax_ = trs_msg_[2];
  trs_velomin_ = trs_msg_[3];
  trs_velomax_ = trs_msg_[4];

  hex_to_nfp32(&data_fp[201], p2p_msg_, 5);
  p2p_jerk_ = p2p_msg_[0];
  p2p_accmin_ = p2p_msg_[1];
  p2p_accmax_ = p2p_msg_[2];
  p2p_velomin_ = p2p_msg_[3];
  p2p_velomax_ = p2p_msg_[4];

  hex_to_nfp32(&data_fp[221], rot_msg_, 2);
  rot_jerk_ = rot_msg_[0];
  rot_accmax_ = rot_msg_[1];

  for (int i = 0; i < 17; i++) { sv3msg_[i] = data_fp[229 + i]; }

  return 0;
}

void ReportDataRich::print_data(void) {
  printf("versions= %s\n", versions_);
  printf("total   = %d\n", total_num_);
  printf("runing  = %d\n", runing_);
  printf("mode    = %d\n", mode_);
  printf("cmdnum  = %d\n", cmdnum_);
  print_nvect("angle   = ", angle_, 7);
  print_nvect("pose    = ", pose_, 6);
  print_nvect("tau     = ", tau_, 7);

  printf("mode    = %d\n", mode_);
  printf("mt_brake= %x\n", mt_brake_);
  printf("mt_able = %x\n", mt_able_);
  printf("err&war = %d %d\n", err_, war_);
  print_nvect("tcp_off = ", tcp_offset_, 6);
  print_nvect("tap_load= ", tcp_load_, 4);
  printf("coll_sen= %d\n", collis_sens_);
  printf("teac_sen= %d\n", teach_sens_);
  print_nvect("gravity_dir_= ", gravity_dir_, 3);

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
