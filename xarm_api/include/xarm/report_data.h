/* Copyright 2017 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jimy Zhang <jimy92@163.com>
 ============================================================================*/
#ifndef XARM_REPORT_DATA_H_
#define XARM_REPORT_DATA_H_

#include "xarm/common/data_type.h"


class ReportDataDevelop {
 public:
  ReportDataDevelop(void);
  ~ReportDataDevelop(void);

  int flush_data(unsigned char *rx_data);
  void print_data(void);

 private:
  int runing_;
  int mode_;
  int cmdnum_;
  float angle_[7];
  float pose_[6];
  float tau_[7];
  int total_num_;
};



class ReportDataNorm {
 public:
  ReportDataNorm(void);
  ~ReportDataNorm(void);

  int flush_data(unsigned char *rx_data);
  void print_data(void);

 // private:
  int runing_;
  int mode_;
  int cmdnum_;
  float angle_[7];
  float pose_[6];
  float tau_[7];

  int mt_brake_;
  int mt_able_;
  int err_;
  int war_;
  float tcp_offset_[6];
  float tcp_load_[4];
  int collis_sens_;
  int teach_sens_;
  float gravity_dir_[3];
  int total_num_;
};



class ReportDataRich {
 public:
  ReportDataRich(void);
  ~ReportDataRich(void);

  int flush_data(unsigned char *rx_data);
  void print_data(void);

 private:
  int runing_;
  int mode_;
  int cmdnum_;
  float angle_[7];
  float pose_[6];
  float tau_[7];

  int mt_brake_;
  int mt_able_;
  int err_;
  int war_;
  float tcp_offset_[6];
  float tcp_load_[4];
  int collis_sens_;
  int teach_sens_;
  float gravity_dir_[3];
  int total_num_;

  int arm_type_;
  int axis_num_;
  int master_id_;
  int slave_id_;
  int motor_tid_;
  int motor_fid_;
  unsigned char versions_[30];
  float trs_jerk_;
  float trs_accmin_;
  float trs_accmax_;
  float trs_velomin_;
  float trs_velomax_;
  float p2p_jerk_;
  float p2p_accmin_;
  float p2p_accmax_;
  float p2p_velomin_;
  float p2p_velomax_;
  float rot_jerk_;
  float rot_accmax_;
  int sv3msg_[16];
  float trs_msg_[5];
  float p2p_msg_[5];
  float rot_msg_[2];
};

#endif
