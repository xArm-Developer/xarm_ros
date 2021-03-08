/* Copyright 2017 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jimy Zhang <jimy92@163.com>
 ============================================================================*/
#include <stdlib.h>
#include <string.h>
#include "xarm/core/report_data.h"
#include "xarm/core/debug/debug_print.h"

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
  if (total_num_ != 133 && total_num_ != 145) { 
    printf("total_num=%d, sizeof_data=%d\n", total_num_, sizeof_data);
    return -2; 
  }

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

XArmReportData::XArmReportData(void) 
{
  total_num = 0;
  // normal report data
  state = 4;
  mode = 0;
  cmdnum = 0;
  memset(angle, 0, sizeof(angle));
  memset(pose, 0, sizeof(pose));
  memset(tau, 0, sizeof(tau));
  mt_brake = 0;
  mt_able = 0;
  err = 0;
  war = 0;
  memset(tcp_offset, 0, sizeof(tcp_offset));
  memset(tcp_load, 0, sizeof(tcp_load));
  collis_sens = 0;
  teach_sens = 0;
  memset(gravity_dir, 0, sizeof(gravity_dir));

  // rich report data
  arm_type = 0;
  axis_num = 0;
  master_id = 0;
  slave_id = 0;
  motor_tid = 0;
  motor_fid = 0;
  memset(versions, 0, sizeof(versions));
  trs_jerk = 0;
  trs_accmin = 0;
  trs_accmax = 0;
  trs_velomin = 0;
  trs_velomax = 0;
  p2p_jerk = 0;
  p2p_accmin = 0;
  p2p_accmax = 0;
  p2p_velomin = 0;
  p2p_velomax = 0;
  rot_jerk = 0;
  rot_accmax = 0;
  memset(sv3msg_, 0, sizeof(sv3msg_));
  memset(trs_msg_, 0, sizeof(trs_msg_));
  memset(p2p_msg_, 0, sizeof(p2p_msg_));
  memset(rot_msg_, 0, sizeof(rot_msg_));

  memset(temperatures, 0, sizeof(temperatures));
  rt_tcp_spd = 0;
  memset(rt_joint_spds, 0, sizeof(rt_joint_spds));
  count = 0;
  memset(world_offset, 0, sizeof(world_offset));
  memset(gpio_reset_conf, 0, sizeof(gpio_reset_conf));
  simulation_mode = 0;
  collision_detection = 0;
  collision_tool_type = 0;
  memset(collision_model_params, 0, sizeof(collision_model_params));
  memset(voltages, 0, sizeof(voltages));
  memset(currents, 0, sizeof(currents));
  
  cgpio_state = 0;
  cgpio_code = 0;
  memset(cgpio_input_digitals, 0, sizeof(cgpio_input_digitals));
  memset(cgpio_output_digitals, 0, sizeof(cgpio_output_digitals));
  memset(cgpio_input_analogs, 0, sizeof(cgpio_input_analogs));
  memset(cgpio_output_analogs, 0, sizeof(cgpio_output_analogs));
  memset(cgpio_input_conf, 0, sizeof(cgpio_input_conf));
  memset(cgpio_output_conf, 0, sizeof(cgpio_output_conf));
}

XArmReportData::~XArmReportData(void) {}

int XArmReportData::flush_data(unsigned char *rx_data)
{
  unsigned char *data_fp = &rx_data[4];
  total_num = bin8_to_32(data_fp);

  // develop 
  state = data_fp[4] & 0x0F;
  mode = data_fp[4] >> 4;
  cmdnum = bin8_to_16(&data_fp[5]);
  hex_to_nfp32(&data_fp[7], angle, 7);
  hex_to_nfp32(&data_fp[35], pose, 6);
  hex_to_nfp32(&data_fp[59], tau, 7);

  // normal
  if (total_num >= 133) {
    mt_brake = data_fp[87];
    mt_able = data_fp[88];
    err = data_fp[89];
    war = data_fp[90];
    hex_to_nfp32(&data_fp[91], tcp_offset, 6);
    hex_to_nfp32(&data_fp[115], tcp_load, 4);
    collis_sens = data_fp[131];
    teach_sens = data_fp[132];
    hex_to_nfp32(&data_fp[133], gravity_dir, 3);
  }

  // rich
  if (total_num >= 245) {
    arm_type = data_fp[145];
    axis_num = data_fp[146];
    master_id = data_fp[147];
    slave_id = data_fp[148];
    motor_tid = data_fp[149];
    motor_fid = data_fp[150];
    memcpy(versions, &data_fp[151], 30);

    hex_to_nfp32(&data_fp[181], trs_msg_, 5);
    trs_jerk = trs_msg_[0];
    trs_accmin = trs_msg_[1];
    trs_accmax = trs_msg_[2];
    trs_velomin = trs_msg_[3];
    trs_velomax = trs_msg_[4];

    hex_to_nfp32(&data_fp[201], p2p_msg_, 5);
    p2p_jerk = p2p_msg_[0];
    p2p_accmin = p2p_msg_[1];
    p2p_accmax = p2p_msg_[2];
    p2p_velomin = p2p_msg_[3];
    p2p_velomax = p2p_msg_[4];

    hex_to_nfp32(&data_fp[221], rot_msg_, 2);
    rot_jerk = rot_msg_[0];
    rot_accmax = rot_msg_[1];

    for (int i = 0; i < 17; i++) { sv3msg_[i] = data_fp[229 + i]; }

    if (total_num >= 252) {
      for (int i = 0; i < 17; i++) { temperatures[i] = data_fp[245 + i]; }
    }
    if (total_num >= 284) {
      float tcp_spd[1];
      hex_to_nfp32(&data_fp[252], tcp_spd, 1);
      rt_tcp_spd = tcp_spd[0];
      hex_to_nfp32(&data_fp[256], rt_joint_spds, 7);
    }
    if (total_num >= 288) {
      count = bin8_to_32(&data_fp[284]);
    }
    if (total_num >= 312) {
      hex_to_nfp32(&data_fp[288], world_offset, 6);
    }
    if (total_num >= 314) {
      gpio_reset_conf[0] = data_fp[312];
      gpio_reset_conf[1] = data_fp[313];
    }
    if (total_num >= 417) {
      simulation_mode = data_fp[314];
      collision_detection = data_fp[315];
      collision_tool_type = data_fp[316];
      hex_to_nfp32(&data_fp[317], collision_model_params, 6);
      for (int i = 0; i < 17; i++) { voltages[i] = (float)bin8_to_16(&data_fp[341 + 2 * i]) / 100; }
      hex_to_nfp32(&data_fp[355], currents, 7);
      
      cgpio_state = data_fp[383];
      cgpio_code = data_fp[384];
      cgpio_input_digitals[0] = bin8_to_16(&data_fp[385]);
      cgpio_input_digitals[1] = bin8_to_16(&data_fp[387]);
      cgpio_output_digitals[0] = bin8_to_16(&data_fp[389]);
      cgpio_output_digitals[1] = bin8_to_16(&data_fp[391]);
      cgpio_input_analogs[0] = (float)(bin8_to_16(&data_fp[393])) / 4095 * 10;
			cgpio_input_analogs[1] = (float)(bin8_to_16(&data_fp[395])) / 4095 * 10;
			cgpio_output_analogs[0] = (float)(bin8_to_16(&data_fp[397])) / 4095 * 10;
			cgpio_output_analogs[1] = (float)(bin8_to_16(&data_fp[399])) / 4095 * 10;

      for (int i = 0; i < 8; i++) {
        cgpio_input_conf[i] = data_fp[401 + i];
        cgpio_output_conf[i] = data_fp[409 + i];
      };
      if (total_num >= 433) {
        for (int i = 0; i < 8; i++) {
          cgpio_input_conf[i+8] = data_fp[417 + i];
          cgpio_output_conf[i+8] = data_fp[425 + i];
        };
      }
    }
  }
  return 0;
}

void XArmReportData::print_data(void)
{
  printf("total   = %d\n", total_num);
  printf("state  = %d\n", state);
  printf("mode    = %d\n", mode);
  printf("cmdnum  = %d\n", cmdnum);
  print_nvect("angle   = ", angle, 7);
  print_nvect("pose    = ", pose, 6);
  print_nvect("tau     = ", tau, 7);

  if (total_num >= 133) {
    printf("mt_brake= %x\n", mt_brake);
    printf("mt_able = %x\n", mt_able);
    printf("err&war = %d %d\n", err, war);
    print_nvect("tcp_off = ", tcp_offset, 6);
    print_nvect("tcp_load= ", tcp_load, 4);
    printf("coll_sen= %d\n", collis_sens);
    printf("teac_sen= %d\n", teach_sens);
    print_nvect("gravity_dir= ", gravity_dir, 3);
  }

  if (total_num >= 245) {
    printf("xarm_type = %d(axis%d)\n", arm_type, axis_num);
    printf("xarm_msid = 0x%X 0x%X\n", master_id, slave_id);
    printf("motor_tfid = 0x%X 0x%X\n", motor_tid, motor_fid);

    printf("versions= %s\n", versions);
    
    print_nvect("trs_msg = ", rot_msg_, 5);
    print_nvect("p2p_msg = ", rot_msg_, 5);
    print_nvect("ros_msg = ", rot_msg_, 2);

    printf("ID   执行状态  错误代码\n");
    for (int i = 0; i < 8; i++) {
        printf("%d      %d        0x%X\n", i + 1, sv3msg_[i * 2], sv3msg_[i * 2 + 1]);
    }
  }
}
