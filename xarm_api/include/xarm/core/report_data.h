/* Copyright 2017 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jimy Zhang <jimy92@163.com>
 ============================================================================*/
#ifndef XARM_REPORT_DATA_H_
#define XARM_REPORT_DATA_H_

#include <string>
#include "xarm/core/common/data_type.h"


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

class XArmReportData {
public:
  XArmReportData(std::string report_type = "normal");
  ~XArmReportData(void);

  int flush_data(unsigned char *rx_data);
  void print_data(void);

private:
  int __flush_common_data(unsigned char *rx_data);
  void __print_common_data(void);
  int _flush_dev_data(unsigned char *rx_data);
  void _print_dev_data(void);
  int _flush_normal_data(unsigned char *rx_data);
  void _print_normal_data(void);
  int _flush_rich_data(unsigned char *rx_data);
  void _print_rich_data(void);

public:
  int total_num;
  // dev/normal/rich report data
  int state;
  int mode;
  int cmdnum;
  float angle[7];
  float pose[6];
  float tau[7];

  // normal/rich report data
  int mt_brake;
  int mt_able;
  int err;
  int war;
  float tcp_offset[6];
  float tcp_load[4];
  int collis_sens;
  int teach_sens;
  float gravity_dir[3];

  // rich report data
  int arm_type;
  int axis_num;
  int master_id;
  int slave_id;
  int motor_tid;
  int motor_fid;
  unsigned char versions[30];
  float trs_jerk;
  float trs_accmin;
  float trs_accmax;
  float trs_velomin;
  float trs_velomax;
  float p2p_jerk;
  float p2p_accmin;
  float p2p_accmax;
  float p2p_velomin;
  float p2p_velomax;
  float rot_jerk;
  float rot_accmax;

  int temperatures[7];
  float rt_tcp_spd;
  float rt_joint_spds[7];
  int count;
  float world_offset[6];
  int gpio_reset_conf[2];
  int simulation_mode;
  int collision_detection;
  int collision_tool_type;
  float collision_model_params[6];
  float voltages[7];
  float currents[7];
  
  int cgpio_state;
  int cgpio_code;
  int cgpio_input_digitals[2];
  int cgpio_output_digitals[2];
  float cgpio_input_analogs[2];
  float cgpio_output_analogs[2];
  unsigned char cgpio_input_conf[16];
  unsigned char cgpio_output_conf[16];

private:
  std::string report_type;
  unsigned char *data_fp;
  int sv3msg_[16];
  float trs_msg_[5];
  float p2p_msg_[5];
  float rot_msg_[2];
};

#endif // XARM_REPORT_DATA_H_
