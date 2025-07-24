/* Copyright 2018 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jason Peng <jason@ufactory.cc>
 ============================================================================*/
#include "xarm_api/xarm_driver.h"
#define CMD_HEARTBEAT_SEC 30 // 30s

#define DEBUG_MODE 1
#define PARAM_ERROR 997


// void* cmd_heart_beat(void* args)
// {
//   xarm_api::XArmDriver *my_driver = (xarm_api::XArmDriver *) args;
//   int cmdnum;
//   int cnt = 0;
//   int max_cnt = CMD_HEARTBEAT_SEC * 2;
//   while(my_driver->arm->is_connected())
//   {
//     ros::Duration(0.5).sleep(); // non-realtime
//     cnt += 1;
//     if (cnt >= max_cnt) {
//       cnt = 0;
//       my_driver->arm->get_cmdnum(&cmdnum);
//     }
//   }
//   ROS_ERROR("xArm Control Connection Failed! Please Shut Down (Ctrl-C) and Retry ...");
//   return (void*)0;
// }

namespace xarm_api
{

XArmDriver::~XArmDriver()
{   
  arm->set_mode(XARM_MODE::POSE);
  arm->disconnect();
  spinner_.stop();
}

bool XArmDriver::_get_wait_param(void) 
{
  bool wait;
  nh_.getParam("wait_for_finish", wait);
  return wait;
}

void XArmDriver::_report_connect_changed_callback(bool connected, bool reported)
{
  ROS_INFO("[TCP STATUS] CONTROL: %d, REPORT: %d", connected, reported);
}

void XArmDriver::_report_data_callback(XArmReportData *report_data_ptr)
{
  // ROS_INFO("[1] state: %d, error_code: %d", report_data_ptr->state, report_data_ptr->err);
  curr_state = report_data_ptr->state;
  curr_err = report_data_ptr->err;
  curr_mode = report_data_ptr->mode;
  curr_cmdnum = report_data_ptr->cmdnum;

  ros::Time now = ros::Time::now();
  bool use_new = _firmware_version_is_ge(1, 8, 103);
  if (!use_new)
  {
    for(int i = 0; i < dof_; i++)
    {
      // joint_state_msg_.position[i] = (double)report_data_ptr->angle[i];
      if (!in_ros_control_)
        joint_state_msg_.velocity[i] = (double)report_data_ptr->rt_joint_spds[i];
      joint_state_msg_.effort[i] = (double)report_data_ptr->tau[i];
    }
  }

  xarm_state_msg_.state = report_data_ptr->state;
  xarm_state_msg_.mode = report_data_ptr->mode;
  xarm_state_msg_.cmdnum = report_data_ptr->cmdnum;
  xarm_state_msg_.err = report_data_ptr->err;
  xarm_state_msg_.warn = report_data_ptr->war;
  xarm_state_msg_.mt_brake = report_data_ptr->mt_brake;
  xarm_state_msg_.mt_able = report_data_ptr->mt_able;

  for(int i = 0; i < dof_; i++)
  {
    xarm_state_msg_.angle[i] = (double)report_data_ptr->angle[i];
  }
  for(int i = 0; i < 6; i++)
  {
    xarm_state_msg_.pose[i] = report_data_ptr->pose[i];
    xarm_state_msg_.offset[i] = report_data_ptr->tcp_offset[i];
  }
  xarm_state_msg_.header.stamp = now;
  pub_robot_msg(xarm_state_msg_);
  if (report_data_ptr->total_num >= 417) {
    cgpio_state_msg_.header.stamp = now;
    cgpio_state_msg_.state = report_data_ptr->cgpio_state;
    cgpio_state_msg_.code = report_data_ptr->cgpio_code;
    cgpio_state_msg_.input_digitals[0] = report_data_ptr->cgpio_input_digitals[0];
    cgpio_state_msg_.input_digitals[1] = report_data_ptr->cgpio_input_digitals[1];
    cgpio_state_msg_.output_digitals[0] = report_data_ptr->cgpio_output_digitals[0];
    cgpio_state_msg_.output_digitals[1] = report_data_ptr->cgpio_output_digitals[1];

    cgpio_state_msg_.input_analogs[0] = report_data_ptr->cgpio_input_analogs[0];
    cgpio_state_msg_.input_analogs[1] = report_data_ptr->cgpio_input_analogs[1];
    cgpio_state_msg_.output_analogs[0] = report_data_ptr->cgpio_output_analogs[0];
    cgpio_state_msg_.output_analogs[1] = report_data_ptr->cgpio_output_analogs[1];

    for (int i = 0; i < 16; ++i) {
      cgpio_state_msg_.input_conf[i] = report_data_ptr->cgpio_input_conf[i];
      cgpio_state_msg_.output_conf[i] = report_data_ptr->cgpio_output_conf[i];
    }
    pub_cgpio_state(cgpio_state_msg_);
  }
  
  if ((report_type_ == "dev" && report_data_ptr->total_num >= 135) 
    || (report_type_ == "rich" && report_data_ptr->total_num >= 481)) {
    ftsensor_msg_.header.stamp = now;
    ftsensor_msg_.header.frame_id = "uf_ft_sensor_ext_data";
    ftsensor_msg_.wrench.force.x = report_data_ptr->ft_ext_force[0];
    ftsensor_msg_.wrench.force.y = report_data_ptr->ft_ext_force[1];
    ftsensor_msg_.wrench.force.z = report_data_ptr->ft_ext_force[2];
    ftsensor_msg_.wrench.torque.x = report_data_ptr->ft_ext_force[3];
    ftsensor_msg_.wrench.torque.y = report_data_ptr->ft_ext_force[4];
    ftsensor_msg_.wrench.torque.z = report_data_ptr->ft_ext_force[5];
    pub_ftsensor_ext_state(ftsensor_msg_);
    ftsensor_msg_.header.frame_id = "uf_ft_sensor_raw_data";
    ftsensor_msg_.wrench.force.x = report_data_ptr->ft_raw_force[0];
    ftsensor_msg_.wrench.force.y = report_data_ptr->ft_raw_force[1];
    ftsensor_msg_.wrench.force.z = report_data_ptr->ft_raw_force[2];
    ftsensor_msg_.wrench.torque.x = report_data_ptr->ft_raw_force[3];
    ftsensor_msg_.wrench.torque.y = report_data_ptr->ft_raw_force[4];
    ftsensor_msg_.wrench.torque.z = report_data_ptr->ft_raw_force[5];
    pub_ftsensor_raw_state(ftsensor_msg_);
  }
}

void XArmDriver::_init_service(void)
{
  // api command services:
  motion_ctrl_server_ = nh_.advertiseService("motion_ctrl", &XArmDriver::MotionCtrlCB, this);
  set_mode_server_ = nh_.advertiseService("set_mode", &XArmDriver::SetModeCB, this);
  set_state_server_ = nh_.advertiseService("set_state", &XArmDriver::SetStateCB, this);
  set_tcp_offset_server_ = nh_.advertiseService("set_tcp_offset", &XArmDriver::SetTCPOffsetCB, this);
  set_load_server_ = nh_.advertiseService("set_load", &XArmDriver::SetLoadCB, this);

  go_home_server_ = nh_.advertiseService("go_home", &XArmDriver::GoHomeCB, this);
  move_joint_server_ = nh_.advertiseService("move_joint", &XArmDriver::MoveJointCB, this);
  move_jointb_server_ = nh_.advertiseService("move_jointb", &XArmDriver::MoveJointbCB, this);
  move_lineb_server_ = nh_.advertiseService("move_lineb", &XArmDriver::MoveLinebCB, this);
  move_line_server_ = nh_.advertiseService("move_line", &XArmDriver::MoveLineCB, this);
  move_line_tool_server_ = nh_.advertiseService("move_line_tool", &XArmDriver::MoveLineToolCB, this);
  move_servoj_server_ = nh_.advertiseService("move_servoj", &XArmDriver::MoveServoJCB, this);
  move_servo_cart_server_ = nh_.advertiseService("move_servo_cart", &XArmDriver::MoveServoCartCB, this);        
  clear_err_server_ = nh_.advertiseService("clear_err", &XArmDriver::ClearErrCB, this);
  moveit_clear_err_server_ = nh_.advertiseService("moveit_clear_err", &XArmDriver::MoveitClearErrCB, this);
  get_err_server_ = nh_.advertiseService("get_err", &XArmDriver::GetErrCB, this);

  // axis-angle motion:
  move_line_aa_server_ = nh_.advertiseService("move_line_aa", &XArmDriver::MoveLineAACB, this);
  move_servo_cart_aa_server_ = nh_.advertiseService("move_servo_cart_aa", &XArmDriver::MoveServoCartAACB, this);

  // tool io:
  set_end_io_server_ = nh_.advertiseService("set_digital_out", &XArmDriver::SetDigitalIOCB, this);
  get_digital_in_server_ = nh_.advertiseService("get_digital_in", &XArmDriver::GetDigitalIOCB, this);
  get_analog_in_server_ = nh_.advertiseService("get_analog_in", &XArmDriver::GetAnalogIOCB, this);
  config_modbus_server_ = nh_.advertiseService("config_tool_modbus", &XArmDriver::ConfigModbusCB, this);
  set_modbus_server_ = nh_.advertiseService("set_tool_modbus", &XArmDriver::SetModbusCB, this);

  gripper_config_server_ = nh_.advertiseService("gripper_config", &XArmDriver::GripperConfigCB, this);
  gripper_move_server_ = nh_.advertiseService("gripper_move", &XArmDriver::GripperMoveCB, this);
  gripper_state_server_ = nh_.advertiseService("gripper_state", &XArmDriver::GripperStateCB, this);

  set_vacuum_gripper_server_old_ = nh_.advertiseService("vacuum_gripper_set", &XArmDriver::VacuumGripperCB_OLD, this);
  set_vacuum_gripper_server_ = nh_.advertiseService("set_vacuum_gripper", &XArmDriver::VacuumGripperCB, this);

  // controller_io (digital):
  set_controller_dout_server_ = nh_.advertiseService("set_controller_dout", &XArmDriver::SetControllerDOutCB, this);
  get_controller_din_server_ = nh_.advertiseService("get_controller_din", &XArmDriver::GetControllerDInCB, this);
  set_controller_aout_server_ = nh_.advertiseService("set_controller_aout", &XArmDriver::SetControllerAOutCB, this);
  get_controller_ain_server_ = nh_.advertiseService("get_controller_ain", &XArmDriver::GetControllerAInCB, this);

  // velocity control:
  vc_set_jointv_server_ = nh_.advertiseService("velo_move_joint", &XArmDriver::VeloMoveJointCB, this);
  vc_set_linev_server_ = nh_.advertiseService("velo_move_line", &XArmDriver::VeloMoveLineVCB, this);
  // velocity control (with duration parameter)
  vdc_set_jointv_server_ = nh_.advertiseService("velo_move_joint_timed", &XArmDriver::VCSetJointVelocityCB, this);
  vdc_set_linev_server_ = nh_.advertiseService("velo_move_line_timed", &XArmDriver::VCSetCartesianVelocityCB, this);
  
  set_max_jacc_server_ = nh_.advertiseService("set_max_acc_joint", &XArmDriver::SetMaxJAccCB, this);
  set_max_lacc_server_ = nh_.advertiseService("set_max_acc_line", &XArmDriver::SetMaxLAccCB, this);

  // trajectory recording and playback (beta):
  traj_record_server_ = nh_.advertiseService("set_recording", &XArmDriver::SetRecordingCB, this); // start(1)/stop(0) recording
  traj_save_server_ = nh_.advertiseService("save_traj", &XArmDriver::SaveTrajCB, this);
  traj_play_server_ = nh_.advertiseService("play_traj", &XArmDriver::LoadNPlayTrajCB, this); // load and playback recorded trajectory
  
  set_rebound_server_ = nh_.advertiseService("set_collision_rebound", &XArmDriver::SetReboundCB, this); // set collision rebound
  set_coll_sens_server_ = nh_.advertiseService("set_collision_sensitivity", &XArmDriver::SetCollSensCB, this); // set collision sensitivity
  set_teach_sens_server_ = nh_.advertiseService("set_teach_sensitivity", &XArmDriver::SetTeachSensCB, this); // set teach sensitivity

  set_world_offset_server_ = nh_.advertiseService("set_world_offset", &XArmDriver::SetWorldOffsetCB, this);;
  set_fence_mode_server_ = nh_.advertiseService("set_fence_mode", &XArmDriver::SetFenceModeCB, this);
  set_reduced_mode_server_ = nh_.advertiseService("set_reduced_mode", &XArmDriver::SetReducedModeCB, this);
  set_tcp_jerk_server_ = nh_.advertiseService("set_tcp_jerk", &XArmDriver::SetTcpJerkCB, this);
  set_joint_jerk_server_ = nh_.advertiseService("set_joint_jerk", &XArmDriver::SetJointJerkCB, this);
  get_servo_angle_ = nh_.advertiseService("get_servo_angle", &XArmDriver::GetServoAngleCB, this);
  get_position_rpy_server_ = nh_.advertiseService("get_position_rpy", &XArmDriver::GetPositionRPYCB, this);
  get_position_aa_server_ = nh_.advertiseService("get_position_axis_angle", &XArmDriver::GetPositionAACB, this); // aa for axis-angle
  get_tgpio_baudrate_server_ = nh_.advertiseService("get_tgpio_modbus_baudrate", &XArmDriver::GetTgpioBaudRateCB, this);
  
  set_modbus_timeout_server_ = nh_.advertiseService("set_tgpio_modbus_timeout", &XArmDriver::SetModbusToutCB, this);
  getset_tgpio_modbus_server_ = nh_.advertiseService("getset_tgpio_modbus_data", &XArmDriver::GetSetModbusCB, this);
  set_tgpio_modbus_use_503_server_ = nh_.advertiseService("set_tgpio_modbus_use_503_port", &XArmDriver::SetModbusUsePort503CB, this);

  set_ft_sensor_enable_server_ = nh_.advertiseService("set_ft_sensor_enable", &XArmDriver::FtSensorEnable, this);
  set_ft_sensor_mode_server_ = nh_.advertiseService("set_ft_sensor_mode", &XArmDriver::FtSensorSetMode, this);
  set_ft_sensor_zero_server_ = nh_.advertiseService("set_ft_sensor_zero", &XArmDriver::FtSensorSetZero, this);
  iden_ft_sensor_load_offset_server_ = nh_.advertiseService("iden_ft_sensor_load_offset", &XArmDriver::FtSensorIdenLoad, this);
  set_ft_sensor_load_offset_server_ = nh_.advertiseService("set_ft_sensor_load_offset", &XArmDriver::FtSensorCaliLoad, this);
  get_ft_sensor_error_server_ = nh_.advertiseService("get_ft_sensor_error", &XArmDriver::GetFtSensorError, this);
  // OLD SERVICE
  ft_sensor_enable_server_ = nh_.advertiseService("ft_sensor_enable", &XArmDriver::FtSensorEnable, this);
  ft_sensor_app_set_server_ = nh_.advertiseService("ft_sensor_app_set", &XArmDriver::FtSensorSetMode, this);
  ft_sensor_set_zero_server_ = nh_.advertiseService("ft_sensor_set_zero", &XArmDriver::FtSensorSetZero, this);
  ft_sensor_iden_load_server_ = nh_.advertiseService("ft_sensor_iden_load", &XArmDriver::FtSensorIdenLoad, this);
  ft_sensor_cali_load_server_ = nh_.advertiseService("ft_sensor_cali_load", &XArmDriver::FtSensorCaliLoad, this);

  open_lite6_gripper_server_ = nh_.advertiseService("open_lite6_gripper", &XArmDriver::OpenLite6Gripper, this);
  close_lite6_gripper_server_ = nh_.advertiseService("close_lite6_gripper", &XArmDriver::CloseLite6Gripper, this);
  stop_lite6_gripper_server_ = nh_.advertiseService("stop_lite6_gripper", &XArmDriver::StopLite6Gripper, this);
}

void XArmDriver::_init_publisher(void)
{
  joint_state_msg_.header.frame_id = "joint-state data";
  joint_state_msg_.name.resize(dof_);
  joint_state_msg_.position.resize(dof_);
  joint_state_msg_.velocity.resize(dof_, 0);
  joint_state_msg_.effort.resize(dof_, 0);
  for(int i = 0; i < dof_; i++)
  {
    joint_state_msg_.name[i] = joint_names_[i];
  }
  xarm_state_msg_.angle.resize(dof_);

  // state feedback topics:
  joint_state_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 10, true);
  robot_rt_state_ = nh_.advertise<xarm_msgs::RobotMsg>("xarm_states", 10, true);
  cgpio_state_ = nh_.advertise<xarm_msgs::CIOState>("controller_gpio_states", 10, true);
  ftsensor_ext_state_ = nh_.advertise<geometry_msgs::WrenchStamped>("uf_ftsensor_ext_states", 10, true);
  ftsensor_raw_state_ = nh_.advertise<geometry_msgs::WrenchStamped>("uf_ftsensor_raw_states", 10, true);
}

void XArmDriver::_init_subscriber(void)
{
  sleep_sub_ = nh_.subscribe("sleep_sec", 1, &XArmDriver::SleepTopicCB, this);
  velo_move_joint_sub_ = nh_.subscribe("velo_move_joint_timed", 1, &XArmDriver::VeloMoveJointTopicCB, this);
  velo_move_line_sub_ = nh_.subscribe("velo_move_line_timed", 1, &XArmDriver::VeloMoveLineTopicCB, this);
}

void XArmDriver::SleepTopicCB(const std_msgs::Float32ConstPtr& msg)
{
  if(msg->data>0)
    arm->set_pause_time(msg->data);
}

void XArmDriver::VeloMoveJointTopicCB(const xarm_msgs::VeloMoveMsgConstPtr& msg)
{
  float jnt_v[7]={0};
  int index = 0;
  if(msg->speeds.size() < dof_)
  {
    ROS_ERROR("The speeds parameter length is incorrect. size=%ld (Expected=%d)", msg->speeds.size(), dof_);
    return;
  }
  else
  {
    for(index = 0; index < 7; index++) // should always send 7 joint commands, whatever current DOF is.
    {
      if(index < msg->speeds.size())
        jnt_v[index] = msg->speeds[index];
      else
        jnt_v[index] = 0;
    }
  }

  arm->vc_set_joint_velocity(jnt_v, msg->is_sync, msg->duration);
}

void XArmDriver::VeloMoveLineTopicCB(const xarm_msgs::VeloMoveMsgConstPtr& msg)
{
  // ROS_INFO("msg: is_sync=%d, is_tool_coord=%d, duration=%f", msg->is_sync, msg->is_tool_coord, msg->duration);

  float line_v[6];
  int index = 0;
  if(msg->speeds.size() < 6)
  {
    ROS_ERROR("The speeds parameter length is incorrect. size=%ld (Expected=6)", msg->speeds.size());
    return;
  }
  else
  {
    for(index = 0; index < 6; index++)
    {
      line_v[index] = msg->speeds[index];
    }
  }

  arm->vc_set_cartesian_velocity(line_v, msg->is_tool_coord, msg->duration);
}

void XArmDriver::init(ros::NodeHandle& root_nh, std::string &server_ip, bool in_ros_control)
{
  curr_err = 0;
  curr_state = 4;
  curr_mode = 0;
  curr_cmdnum = 0;
  arm = NULL;
  in_ros_control_ = in_ros_control;

  nh_ = root_nh;

  nh_.getParam("DOF", dof_);
  nh_.getParam("prefix", prefix_);
  nh_.getParam("joint_names", joint_names_);
  nh_.getParam("xarm_report_type", report_type_);

  for (int i = 0; i < dof_; i++) {
    joint_names_[i] = prefix_ + joint_names_[i];
  }
  
  bool baud_checkset = true;
  if (nh_.hasParam("baud_checkset")) {
    nh_.getParam("baud_checkset", baud_checkset);
  }
  int default_gripper_baud = 2000000;
  if (nh_.hasParam("default_gripper_baud")) {
    nh_.getParam("default_gripper_baud", default_gripper_baud);
  }

  joint_states_rate_ = -1;
  if (nh_.hasParam("joint_states_rate")) {
    nh_.getParam("joint_states_rate", joint_states_rate_);
  }

  joint_state_flags_ = -1;
  if (nh_.hasParam("joint_state_flags")) {
    nh_.getParam("joint_state_flags", joint_state_flags_);
  }

  _init_publisher();

  arm = new XArmAPI(
    server_ip, 
    true, // is_radian
    true, // do_not_open
    true, // check_tcp_limit
    true, // check_joint_limit
    true, // check_cmdnum_limit
    false, // check_robot_sn
    true, // check_is_ready
    true, // check_is_pause
    0, // max_callback_thread_count
    512, // max_cmdnum
    dof_, // init_axis
    DEBUG_MODE, // debug
    report_type_ // report_type
  );
  arm->set_baud_checkset_enable(baud_checkset);
  arm->set_checkset_default_baud(1, default_gripper_baud);
  arm->release_connect_changed_callback(true);
  arm->release_report_data_callback(true);
  arm->register_connect_changed_callback(std::bind(&XArmDriver::_report_connect_changed_callback, this, std::placeholders::_1, std::placeholders::_2));
  arm->register_report_data_callback(std::bind(&XArmDriver::_report_data_callback, this, std::placeholders::_1));
  arm->connect();

  int err_warn[2] = {0};
  int ret = arm->get_err_warn_code(err_warn);
  if (err_warn[0] != 0) {
    ROS_WARN("xArmErrorCode: C%d: [ %s ]", err_warn[0], controller_error_interpreter(err_warn[0]).c_str());
  }
  
  // std::thread th(cmd_heart_beat, this);
  // th.detach();
  int dbg_msg[16] = {0};
  arm->core->servo_get_dbmsg(dbg_msg);

  for(int i=0; i<dof_; i++)
  {
    if((dbg_msg[i*2]==1)&&(dbg_msg[i*2+1]==40))
    {
      arm->clean_error();
      ROS_WARN("Cleared low-voltage error of joint %d", i+1);
    }
    else if((dbg_msg[i*2]==1))
    {
      arm->clean_error();
      ROS_WARN("There is servo error code:(0x%x) in joint %d, trying to clear it..", dbg_msg[i*2+1], i+1);
    }
  }

  if (!in_ros_control_)
  {
    std::thread([this]() {
      float position[7] = {0};
      float velocity[7] = {0};
      float effort[7] = {0};
      if (joint_states_rate_ < 0) {
        joint_states_rate_ = report_type_ == "dev" ? 100 : 5;
      }
      bool use_new = _firmware_version_is_ge(1, 8, 103);
      int microseconds = 1000000 / joint_states_rate_;
      int num = 3;
      if (_firmware_version_is_ge(2, 6, 107)) {
        if (joint_state_flags_ >= 0){
          num = ((joint_state_flags_ & 0x0F) << 4) + num;
        }
      }
      while (arm->is_connected())
      {
        if (use_new)
          arm->get_joint_states(position, velocity, effort, num);
        else
          arm->get_servo_angle(position);

        joint_state_msg_.header.stamp = ros::Time::now();
        for(int i = 0; i < dof_; i++)
        {
          joint_state_msg_.position[i] = (double)position[i];
          if (use_new)
          {
            joint_state_msg_.velocity[i] = (double)velocity[i];
            joint_state_msg_.effort[i] = (double)effort[i];
          }
        }
        pub_joint_state(joint_state_msg_);
        std::this_thread::sleep_for(std::chrono::microseconds(microseconds));
      }
      ROS_ERROR("xArm Control Connection Failed! Please Shut Down (Ctrl-C) and Retry ...");
    }).detach();
  }

  _init_service();
  _init_subscriber();
  _init_xarm_gripper();
  _init_bio_gripper();
}

sensor_msgs::JointState* XArmDriver::get_joint_states()
{
  return &joint_state_msg_;
}

int XArmDriver::update_joint_states(bool initialized, int flag)
{
  static ros::Time prev_time;
  static ros::Time curr_time;
  static float prev_position[7] = {0};
  static float curr_position[7] = {0};
  static float curr_velocity[7] = {0};
  static float curr_effort[7] = {0};

  int num = 3;
  if (_firmware_version_is_ge(2, 6, 107)) {
    if (flag >= 0) {
      num = ((flag & 0x0F) << 4) + num;
    }
    else if (joint_state_flags_ >= 0){
      num = ((joint_state_flags_ & 0x0F) << 4) + num;
    }
  }

  bool use_new = _firmware_version_is_ge(1, 8, 103);
  int ret;
  if (use_new)
    ret = arm->get_joint_states(curr_position, curr_velocity, curr_effort, num);
  else
    ret = arm->get_servo_angle(curr_position);
  curr_time = ros::Time::now();

  joint_state_msg_.header.stamp = curr_time;
  for(int i = 0; i < joint_state_msg_.position.size(); i++)
  {
    joint_state_msg_.position[i] = (double)curr_position[i];
    if (use_new) {
      joint_state_msg_.velocity[i] = (double)curr_velocity[i];
      joint_state_msg_.effort[i] = (double)curr_effort[i];
    }
    else {
      curr_velocity[i] = !initialized ? 0.0 : (curr_position[i] - prev_position[i]) / (curr_time.toSec() - prev_time.toSec());
      joint_state_msg_.velocity[i] = (double)curr_velocity[i];
    }
  }
  pub_joint_state(joint_state_msg_);
  memcpy(prev_position, curr_position, sizeof(float) * 7);
  prev_time = curr_time;
  return ret;
}

bool XArmDriver::_firmware_version_is_ge(int major, int minor, int revision)
{
  return arm->version_number[0] > major || (arm->version_number[0] == major && arm->version_number[1] > minor) || (arm->version_number[0] == major && arm->version_number[1] == minor && arm->version_number[2] >= revision);
}

inline float XArmDriver::_xarm_gripper_pos_convert(float pos, bool reversed)
{
  if (reversed) {
    return fabs(xarm_gripper_max_pos - pos * 1000.0);
  }
  else {
    return fabs(xarm_gripper_max_pos - pos) / 1000.0;
  }
}

void XArmDriver::_init_xarm_gripper(void)
{
  std::string uf_model;
  nh_.getParam("uf_model", uf_model);
  xarm_gripper_added_ = false;
  // ROS_INFO("UF_MODEL: %s\n", uf_model.c_str());
  if(uf_model != "XARM")
  {
    return; // only for xArm Gripper
  }

  ros::NodeHandle gripper_node("xarm_gripper");

  xarm_gripper_joint_state_msg_.header.stamp = ros::Time::now();
  xarm_gripper_joint_state_msg_.header.frame_id = "xarm-gripper-joint-state data";
  xarm_gripper_joint_state_msg_.name.resize(6);
  xarm_gripper_joint_state_msg_.position.resize(6, std::numeric_limits<double>::quiet_NaN());
  xarm_gripper_joint_state_msg_.velocity.resize(6, std::numeric_limits<double>::quiet_NaN());
  xarm_gripper_joint_state_msg_.effort.resize(6, std::numeric_limits<double>::quiet_NaN());
  xarm_gripper_joint_state_msg_.name[0] = prefix_ + "drive_joint";
  xarm_gripper_joint_state_msg_.name[1] = prefix_ + "left_finger_joint";
  xarm_gripper_joint_state_msg_.name[2] = prefix_ + "left_inner_knuckle_joint";
  xarm_gripper_joint_state_msg_.name[3] = prefix_ + "right_outer_knuckle_joint";
  xarm_gripper_joint_state_msg_.name[4] = prefix_ + "right_finger_joint";
  xarm_gripper_joint_state_msg_.name[5] = prefix_ + "right_inner_knuckle_joint";
  xarm_gripper_action_server_.reset(new actionlib::ActionServer<control_msgs::GripperCommandAction>(gripper_node, "gripper_action",
    std::bind(&XArmDriver::_handle_xarm_gripper_action_goal, this, std::placeholders::_1),
    std::bind(&XArmDriver::_handle_xarm_gripper_action_cancel, this, std::placeholders::_1),
    false));
  xarm_gripper_action_server_->start(); 

  bool add_gripper = false;
  bool rtt = nh_.getParam("add_gripper", add_gripper);
  // has "add_gripper" parameter and its value is true.
  if(rtt && add_gripper)
  {   
    xarm_gripper_added_ = true;
    
    // int ret_grip = arm->get_gripper_position(&xarm_gripper_pos);
    // if(ret_grip || xarm_gripper_pos<0 || xarm_gripper_pos>xarm_gripper_max_pos)
    //   ROS_ERROR("Abnormal when update xArm Gripper initial position, ret = %d, pos = %f, please check the gripper connection!", ret_grip, xarm_gripper_pos);
    
    xarm_gripper_init_loop_ = false;
    std::thread([this]() {
      int curr_pos;
      int ret_grip = arm->get_gripper_position(&curr_pos);
      while (ros::ok() && !xarm_gripper_init_loop_)
      {
        ros::Duration(0.1).sleep();
        _pub_xarm_gripper_joint_states(curr_pos);
      }
    }).detach();
  }
}

void XArmDriver::_pub_xarm_gripper_joint_states(int pos)
{
  xarm_gripper_joint_state_msg_.header.stamp = ros::Time::now();
  float p = _xarm_gripper_pos_convert(pos);
  for (int i = 0; i < 6; i++) {
    xarm_gripper_joint_state_msg_.position[i] = p;
  }
  pub_joint_state(xarm_gripper_joint_state_msg_);
}

void XArmDriver::_handle_xarm_gripper_action_goal(actionlib::ActionServer<control_msgs::GripperCommandAction>::GoalHandle gh)
{
  xarm_gripper_init_loop_ = true;
  ros::Rate loop_rate(10);
  const auto goal = gh.getGoal();
  ROS_INFO("gripper_action_goal handle, position: %f", goal->command.position);
  gh.setAccepted();
  control_msgs::GripperCommandResult result;
  control_msgs::GripperCommandFeedback feedback;

  int ret;
  int curr_pos = 0;
  int err = 0;
  ret = arm->get_gripper_err_code(&err);
  if (err != 0) {
    try {
      gh.setCanceled(result);
    } catch (std::exception &e) {
      ROS_ERROR("goal_handle setCanceled exception, ex=%s", e.what());
    }
    ROS_ERROR("get_gripper_err_code, ret=%d, err=%d", ret, err);
    return;
  }
  ret = arm->get_gripper_position(&curr_pos);
  _pub_xarm_gripper_joint_states(curr_pos);

  ret = arm->set_gripper_mode(0);
  if (ret != 0) {
    result.position = _xarm_gripper_pos_convert(curr_pos);
    try {
      gh.setCanceled(result);
    } catch (std::exception &e) {
      ROS_ERROR("goal_handle setCanceled exception, ex=%s", e.what());
    }
    ret = arm->get_gripper_err_code(&err);
    ROS_WARN("set_gripper_mode, ret=%d, err=%d, curr_pos=%d", ret, err, curr_pos);
    return;
  }
  ret = arm->set_gripper_enable(true);
  if (ret != 0) {
    result.position = _xarm_gripper_pos_convert(curr_pos);
    try {
      gh.setCanceled(result);
    } catch (std::exception &e) {
      ROS_ERROR("goal_handle setCanceled exception, ex=%s", e.what());
    }
    ret = arm->get_gripper_err_code(&err);
    ROS_WARN("set_gripper_enable, ret=%d, err=%d, curr_pos=%d", ret, err, curr_pos);
    return;
  }
  ret = arm->set_gripper_speed(3000);
  if (ret != 0) {
    result.position = _xarm_gripper_pos_convert(curr_pos);
    try {
      gh.setCanceled(result);
    } catch (std::exception &e) {
      ROS_ERROR("goal_handle setCanceled exception, ex=%s", e.what());
    }
    ret = arm->get_gripper_err_code(&err);
    ROS_WARN("set_gripper_speed, ret=%d, err=%d, curr_pos=%d", ret, err, curr_pos);
    return;
  }

  float target_pos = _xarm_gripper_pos_convert(goal->command.position, true);
  bool is_move = true;
  std::thread([this, &target_pos, &is_move, &curr_pos]() {
    is_move = true;
    int ret2 = arm->set_gripper_position((int)target_pos, true, -1, false); // set wait_motion=false
    int err;
    arm->get_gripper_err_code(&err);
    ROS_INFO("set_gripper_position, ret=%d, err=%d, curr_pos=%d", ret2, err, curr_pos);
    is_move = false;
  }).detach();
  while (is_move && ros::ok())
  {
    loop_rate.sleep();
    ret = arm->get_gripper_position(&curr_pos);
    if (ret == 0) {
      feedback.position = _xarm_gripper_pos_convert(curr_pos);
      try {
        gh.publishFeedback(feedback);
      } catch (std::exception &e) {
        ROS_ERROR("goal_handle publishFeedback exception, ex=%s", e.what());
      }
      _pub_xarm_gripper_joint_states(curr_pos);
    }
  }
  arm->get_gripper_position(&curr_pos);
  ROS_INFO("move finish, curr_pos=%d", curr_pos);
  if (ros::ok()) {
    result.position = _xarm_gripper_pos_convert(curr_pos);
    try {
      gh.setSucceeded(result);
    } catch (std::exception &e) {
      ROS_ERROR("goal_handle setSucceeded exception, ex=%s", e.what());
    }
    ROS_INFO("Goal succeeded");
  }
}

void XArmDriver::_handle_xarm_gripper_action_cancel(actionlib::ActionServer<control_msgs::GripperCommandAction>::GoalHandle gh)
{
  ROS_INFO("xarm gripper cancel, not support");
}

inline float XArmDriver::_bio_gripper_pos_convert(float pos, bool reversed)
{
  if (reversed) {
    return fabs(pos * 1000 * 2 + 71);
  }
  else {
    return -fabs(pos - 71) / 1000 / 2;
  }
}

void XArmDriver::_init_bio_gripper(void)
{
  std::string uf_model;
  nh_.getParam("uf_model", uf_model);
  bio_gripper_added_ = false;
  // ROS_INFO("UF_MODEL: %s\n", uf_model.c_str());
  if(uf_model != "XARM")
  {
    return; // only for xArm Gripper
  }

  ros::NodeHandle gripper_node("bio_gripper");

  bio_gripper_joint_state_msg_.header.stamp = ros::Time::now();
  bio_gripper_joint_state_msg_.header.frame_id = "bio-gripper-joint-state data";
  bio_gripper_joint_state_msg_.name.resize(2);
  bio_gripper_joint_state_msg_.position.resize(2, std::numeric_limits<double>::quiet_NaN());
  bio_gripper_joint_state_msg_.velocity.resize(2, std::numeric_limits<double>::quiet_NaN());
  bio_gripper_joint_state_msg_.effort.resize(2, std::numeric_limits<double>::quiet_NaN());
  bio_gripper_joint_state_msg_.name[0] = prefix_ + "left_finger_joint";
  bio_gripper_joint_state_msg_.name[1] = prefix_ + "right_finger_joint";
  bio_gripper_action_server_.reset(new actionlib::ActionServer<control_msgs::GripperCommandAction>(gripper_node, "gripper_action",
    std::bind(&XArmDriver::_handle_bio_gripper_action_goal, this, std::placeholders::_1),
    std::bind(&XArmDriver::_handle_bio_gripper_action_cancel, this, std::placeholders::_1),
    false));
  bio_gripper_action_server_->start(); 

  bool add_bio_gripper = false;
  bool rtt = nh_.getParam("add_bio_gripper", add_bio_gripper);
  // has "add_bio_gripper" parameter and its value is true.
  if(rtt && add_bio_gripper)
  {   
    bio_gripper_added_ = true;
    // int ret_grip = arm->get_bio_gripper_position(&bio_gripper_pos);
    // if(ret_grip || bio_gripper_pos<0 || bio_gripper_pos>bio_gripper_max_pos)
    //   ROS_ERROR("Abnormal when update BIO Gripper initial position, ret = %d, pos = %f, please check the gripper connection!", ret_grip, bio_gripper_pos);
    
    bio_gripper_init_loop_ = false;
    std::thread([this]() {
      int curr_pos;
      int ret_grip = arm->get_bio_gripper_position(&curr_pos);
      while (ros::ok() && !bio_gripper_init_loop_)
      {
        ros::Duration(0.1).sleep();
        _pub_bio_gripper_joint_states(curr_pos);
      }
    }).detach();
  }
}

void XArmDriver::_pub_bio_gripper_joint_states(int pos)
{
  bio_gripper_joint_state_msg_.header.stamp = ros::Time::now();
  float p = _bio_gripper_pos_convert(pos);
  for (int i = 0; i < 6; i++) {
    bio_gripper_joint_state_msg_.position[i] = p;
  }
  pub_joint_state(bio_gripper_joint_state_msg_);
}

void XArmDriver::_handle_bio_gripper_action_goal(actionlib::ActionServer<control_msgs::GripperCommandAction>::GoalHandle gh)
{
  bio_gripper_init_loop_ = true;
  ros::Rate loop_rate(10);
  const auto goal = gh.getGoal();
  ROS_INFO("gripper_action_goal handle, position: %f", goal->command.position);
  gh.setAccepted();
  control_msgs::GripperCommandResult result;
  control_msgs::GripperCommandFeedback feedback;

  int ret;
  int curr_pos = 0;
  int err = 0;
  ret = arm->get_bio_gripper_error(&err);
  if (ret != 0 || err != 0) {
    arm->clean_bio_gripper_error();
    ret = arm->get_bio_gripper_error(&err);
  }
  if (ret != 0 || err != 0) {
    try {
      gh.setCanceled(result);
    } catch (std::exception &e) {
      ROS_ERROR("bio goal_handle setCanceled exception, ex=%s", e.what());
    }
    ROS_ERROR("get_bio_gripper_error, ret=%d, err=%d", ret, err);
    return;
  }
  ret = arm->get_bio_gripper_position(&curr_pos);
  _pub_bio_gripper_joint_states(curr_pos);

  // ret = arm->set_bio_gripper_enable(true);
  // if (ret != 0) {
  //   result.position = _bio_gripper_pos_convert(curr_pos);
  //   try {
  //     gh.setCanceled(result);
  //   } catch (std::exception &e) {
  //     ROS_ERROR("bio goal_handle setCanceled exception, ex=%s", e.what());
  //   }
  //   ret = arm->get_bio_gripper_error(&err);
  //   ROS_WARN("set_bio_gripper_enable, ret=%d, err=%d, curr_pos=%d", ret, err, curr_pos);
  //   return;
  // }
  // ret = arm->set_bio_gripper_speed(3000);
  // if (ret != 0) {
  //   result.position = _bio_gripper_pos_convert(curr_pos);
  //   try {
  //     gh.setCanceled(result);
  //   } catch (std::exception &e) {
  //     ROS_ERROR("bio goal_handle setCanceled exception, ex=%s", e.what());
  //   }
  //   ret = arm->get_bio_gripper_error(&err);
  //   ROS_WARN("set_bio_gripper_speed, ret=%d, err=%d, curr_pos=%d", ret, err, curr_pos);
  //   return;
  // }

  float target_pos = _bio_gripper_pos_convert(goal->command.position, true);
  bool is_move = true;
  std::thread([this, &target_pos, &is_move, &curr_pos]() {
    is_move = true;
    int ret2;
    if (target_pos >= 100)
        ret2 = arm->open_bio_gripper(2000, true, 5, false); // set wait_motion=false
    else
        ret2 = arm->close_bio_gripper(2000, true, 5, false); // set wait_motion=false
    int err;
    arm->get_bio_gripper_error(&err);
    ROS_INFO("set_bio_gripper_position, ret=%d, err=%d, curr_pos=%d", ret2, err, curr_pos);
    is_move = false;
  }).detach();
  while (is_move && ros::ok())
  {
    loop_rate.sleep();
    ret = arm->get_bio_gripper_position(&curr_pos);
    if (ret == 0) {
      feedback.position = _bio_gripper_pos_convert(curr_pos);
      try {
        gh.publishFeedback(feedback);
      } catch (std::exception &e) {
        ROS_ERROR("bio goal_handle publishFeedback exception, ex=%s", e.what());
      }
      _pub_bio_gripper_joint_states(curr_pos);
    }
  }
  arm->get_bio_gripper_position(&curr_pos);
  ROS_INFO("bio move finish, curr_pos=%d", curr_pos);
  if (ros::ok()) {
    result.position = _bio_gripper_pos_convert(curr_pos);
    try {
      gh.setSucceeded(result);
    } catch (std::exception &e) {
      ROS_ERROR("bio goal_handle setSucceeded exception, ex=%s", e.what());
    }
    ROS_INFO("Goal succeeded");
  }
}

void XArmDriver::_handle_bio_gripper_action_cancel(actionlib::ActionServer<control_msgs::GripperCommandAction>::GoalHandle gh)
{
  ROS_INFO("bio gripper cancel, not support");
}

bool XArmDriver::ClearErrCB(xarm_msgs::ClearErr::Request& req, xarm_msgs::ClearErr::Response& res)
{
  // First clear controller warning and error:
  if(xarm_gripper_added_)
    int ret1 = arm->clean_gripper_error();

  int ret2 = arm->clean_error(); 
  int ret3 = arm->clean_warn();

  // Then try to enable motor again:
  res.ret = arm->motion_enable(true, 8);

  if(res.ret)
  {
    res.message = "clear err, ret = "  + std::to_string(res.ret);
  }
  return true;

  // After calling this service, user should check '/xarm_states' again to make sure 'err' field is 0, to confirm success.
}

bool XArmDriver::MoveitClearErrCB(xarm_msgs::ClearErr::Request& req, xarm_msgs::ClearErr::Response& res)
{
  if(ClearErrCB(req, res))
  {
    bool v_control = false;
    nh_.getParam("velocity_control", v_control);

    arm->set_mode(v_control ? XARM_MODE::VELO_JOINT : XARM_MODE::SERVO);
    res.ret = arm->set_state(XARM_STATE::START);
    return true;
  }
  return false;

  // After calling this service, user should check '/xarm_states' again to make sure 'err' field is 0, to confirm success.
}

bool XArmDriver::GetErrCB(xarm_msgs::GetErr::Request & req, xarm_msgs::GetErr::Response & res)
{
  res.err = curr_err;
  res.message = "current error code C"  + std::to_string(res.err) + ": [ " + controller_error_interpreter() + " ]";
  return true;
}

bool XArmDriver::MotionCtrlCB(xarm_msgs::SetAxis::Request& req, xarm_msgs::SetAxis::Response& res)
{
  res.ret = arm->motion_enable(req.data, req.id);
  if(req.data == 1)
  {
    res.message = "motion enable, ret = "  + std::to_string(res.ret);
  }
  else
  {
    res.message = "motion disable, ret = " + std::to_string(res.ret);
  }
  return true;
}

bool XArmDriver::SetModeCB(xarm_msgs::SetInt16::Request& req, xarm_msgs::SetInt16::Response& res)
{
  /* for successful none-zero mode switch, must happen at STOP state */
  arm->set_state(XARM_STATE::STOP);
  ros::Duration(0.01).sleep();
  
  res.ret = arm->set_mode(req.data);
  switch(req.data)
  {
    case XARM_MODE::POSE:
    {
      res.message = "pose mode, ret = " + std::to_string(res.ret);
    }break;
    case XARM_MODE::SERVO:
    {
      res.message = "servo mode, ret = " + std::to_string(res.ret);
    }break;
    case XARM_MODE::TEACH_CART:
    {
      res.message = "cartesian teach, ret = " + std::to_string(res.ret);
    } break;
    case XARM_MODE::TEACH_JOINT:
    {
      res.message = "joint teach, ret = " + std::to_string(res.ret);
    } break;
    case XARM_MODE::VELO_JOINT:
    {
      res.message = "joint velocity, ret = " + std::to_string(res.ret);
    } break;
    case XARM_MODE::VELO_CART:
    {
      res.message = "cartesian velocity, ret = " + std::to_string(res.ret);
    } break;
    case 6:
    {
      res.message = "joint online, ret = " + std::to_string(res.ret);
    } break;
    case 7:
    {
      res.message = "cartesian online, ret = " + std::to_string(res.ret);
    } break;
    default:
    {
      res.message = "the failed mode, ret = " + std::to_string(res.ret);
    }
  }

  return true;
}

bool XArmDriver::SetStateCB(xarm_msgs::SetInt16::Request& req, xarm_msgs::SetInt16::Response& res)
{
  res.ret = arm->set_state(req.data);
  switch(req.data)
  {
    case XARM_STATE::START:
    {
      res.message = "start, ret = " + std::to_string(res.ret);
    }break;
    case XARM_STATE::PAUSE:
    {
      res.message = "pause, ret = " + std::to_string(res.ret);
    }break;
    case XARM_STATE::STOP:
    {
      res.message = "stop, ret = " + std::to_string(res.ret);
    }break;
    default:
    {
      res.message = "the failed state, ret = " + std::to_string(res.ret);
    }
  }

  return true;
}

bool XArmDriver::SetTCPOffsetCB(xarm_msgs::TCPOffset::Request &req, xarm_msgs::TCPOffset::Response &res)
{
  float offsets[6] = {req.x, req.y, req.z, req.roll, req.pitch, req.yaw};
  res.ret = arm->set_tcp_offset(offsets);
  if (res.ret >= 0)
    arm->save_conf();
  res.message = "set tcp offset: ret = " + std::to_string(res.ret); 
  return true;
}

bool XArmDriver::SetLoadCB(xarm_msgs::SetLoad::Request &req, xarm_msgs::SetLoad::Response &res)
{   
  float Mass = req.mass;
  float CoM[3] = {req.xc, req.yc, req.zc};
  res.ret = arm->set_tcp_load(Mass, CoM);
  if (res.ret >= 0)
    arm->save_conf();
  res.message = "set load: ret = " + std::to_string(res.ret); 
  return true;
}

bool XArmDriver::SetControllerDOutCB(xarm_msgs::SetDigitalIO::Request &req, xarm_msgs::SetDigitalIO::Response &res)
{
  if(req.io_num>=1 && req.io_num<=16)
  {
    res.ret = arm->set_cgpio_digital(req.io_num-1, req.value);
    res.message = "set Controller digital Output "+ std::to_string(req.io_num) +" to "+ std::to_string(req.value) + " : ret = " + std::to_string(res.ret); 
    return true;
  }
  ROS_WARN("Controller Digital IO io_num: from 1 to 16");
  res.ret = PARAM_ERROR;
  res.message = "Controller Digital IO io_num: from 1 to 16";
  return true;
}

bool XArmDriver::GetControllerDInCB(xarm_msgs::GetControllerDigitalIO::Request &req, xarm_msgs::GetControllerDigitalIO::Response &res)
{
  if(req.io_num>=1 && req.io_num<=16)
  {
    int all_status;
    int digitals[8];
    int *digitals2 = NULL;
    if (req.io_num > 8)
      digitals2 = new int[8];
    res.ret = arm->get_cgpio_digital(digitals, digitals2);
    if (req.io_num > 8) {
      res.value = digitals2[req.io_num - 9];
      delete[] digitals2;
    }
    else
      res.value = digitals[req.io_num - 1];
    res.message = "get Controller digital Input ret = " + std::to_string(res.ret);
    return res.ret >= 0;
  }
  ROS_WARN("Controller Digital IO io_num: from 1 to 16");
  res.ret = PARAM_ERROR;
  res.message = "Controller Digital IO io_num: from 1 to 16";
  return true;
}

bool XArmDriver::GetControllerAInCB(xarm_msgs::GetAnalogIO::Request &req, xarm_msgs::GetAnalogIO::Response &res)
{
  res.ret = PARAM_ERROR;
  switch (req.port_num)
  {
    case 1:
    case 2:
      res.ret = arm->get_cgpio_analog(req.port_num-1, &res.analog_value);
      break;
    default:
      res.message = "GetAnalogIO Fail: port number incorrect ! Must be 1 or 2";
      return true;
  }
  res.message = "get controller analog port " + std::to_string(req.port_num) + ", ret = " + std::to_string(res.ret); 
  return true;
}

bool XArmDriver::SetControllerAOutCB(xarm_msgs::SetControllerAnalogIO::Request &req, xarm_msgs::SetControllerAnalogIO::Response &res)
{
  res.ret = PARAM_ERROR;
  switch (req.port_num)
  {
    case 1:
    case 2:
      res.ret = arm->set_cgpio_analog(req.port_num-1, req.analog_value);
      break;
    default:
      res.message = "SetAnalogIO Fail: port number incorrect ! Must be 1 or 2";
      return true;
  }
  res.message = "Set controller analog port " + std::to_string(req.port_num) + ", ret = " + std::to_string(res.ret); 
  return true;
}

bool XArmDriver::SetDigitalIOCB(xarm_msgs::SetDigitalIO::Request &req, xarm_msgs::SetDigitalIO::Response &res)
{
  res.ret = arm->set_tgpio_digital(req.io_num-1, req.value);
  res.message = "set Digital port "+ std::to_string(req.io_num) +" to "+ std::to_string(req.value) + " : ret = " + std::to_string(res.ret); 
  return true;
}

bool XArmDriver::GetDigitalIOCB(xarm_msgs::GetDigitalIO::Request &req, xarm_msgs::GetDigitalIO::Response &res)
{
  res.ret = arm->get_tgpio_digital(&res.digital_1, &res.digital_2);
  res.message = "get Digital port ret = " + std::to_string(res.ret); 
  return true;
}

bool XArmDriver::GetAnalogIOCB(xarm_msgs::GetAnalogIO::Request &req, xarm_msgs::GetAnalogIO::Response &res)
{
  res.ret = PARAM_ERROR;
  switch (req.port_num)
  {
    case 1:
    case 2:
      res.ret = arm->get_tgpio_analog(req.port_num-1, &res.analog_value);
      break;
    default:
      res.message = "GetAnalogIO Fail: port number incorrect ! Must be 1 or 2";
      return true;
  }
  res.message = "get tool analog port " + std::to_string(req.port_num) + ", ret = " + std::to_string(res.ret); 
  return true;
}

bool XArmDriver::SetModbusCB(xarm_msgs::SetToolModbus::Request &req, xarm_msgs::SetToolModbus::Response &res)
{
  int send_len = req.send_data.size();
  int recv_len = req.respond_len;
  unsigned char * tx_data = new unsigned char [send_len]{0};
  unsigned char * rx_data = new unsigned char [recv_len]{0};

  for(int i=0; i<send_len; i++)
  {
    tx_data[i] = req.send_data[i];
  }
  res.ret = arm->getset_tgpio_modbus_data(tx_data, send_len, rx_data, recv_len);
  for(int i=0; i<recv_len; i++)
  {
    res.respond_data.push_back(rx_data[i]);
  }

  delete [] tx_data;
  delete [] rx_data;
  return true;
}

bool XArmDriver::ConfigModbusCB(xarm_msgs::ConfigToolModbus::Request &req, xarm_msgs::ConfigToolModbus::Response &res)
{
  res.message = "";
  if(curr_err)
  {
    arm->set_state(XARM_STATE::START);
    ROS_WARN("Cleared Existing Error Code %d", curr_err);
  }

  int ret = arm->set_tgpio_modbus_baudrate(req.baud_rate);
  int ret2 = arm->set_tgpio_modbus_timeout(req.timeout_ms);
  res.ret = ret == 0 ? ret2 : ret;
  res.message = "set_modbus_baudrate, ret="+ std::to_string(ret);
  res.message += (std::string(" | set_modbus_timeout, ret=") + std::to_string(ret2));

  return true;
}

bool XArmDriver::GoHomeCB(xarm_msgs::Move::Request &req, xarm_msgs::Move::Response &res)
{
  res.ret = arm->move_gohome(req.mvvelo, req.mvacc, req.mvtime, _get_wait_param());
  res.message = "go home, ret = " + std::to_string(res.ret);
  return true;
}

bool XArmDriver::MoveJointCB(xarm_msgs::Move::Request &req, xarm_msgs::Move::Response &res)
{
  float joint[7]={0};
  int index = 0;
  if(req.pose.size() != dof_)
  {
    res.ret = PARAM_ERROR;
    res.message = "pose parameters incorrect! Expected: "+std::to_string(dof_);
    return false;
  }
  else
  {
    for(index = 0; index < 7; index++) // should always send 7 joint commands, whatever current DOF is.
    {
      // joint[0][index] = req.pose[index];
      if(index<req.pose.size())
        joint[index] = req.pose[index];
      else
        joint[index] = 0;
    }
  }
  res.ret = arm->set_servo_angle(joint, req.mvvelo, req.mvacc, req.mvtime, _get_wait_param());
  res.message = "move joint, ret = " + std::to_string(res.ret);
  return true;
}

bool XArmDriver::MoveLineCB(xarm_msgs::Move::Request &req, xarm_msgs::Move::Response &res)
{
  float pose[6];
  int index = 0;
  if(req.pose.size() != 6)
  {
    res.ret = PARAM_ERROR;
    res.message = "number of parameters incorrect!";
    return true;
  }
  else
  {
    for(index = 0; index < 6; index++)
    {
      pose[index] = req.pose[index];
    }
  }
  res.ret = arm->set_position(pose, -1, req.mvvelo, req.mvacc, req.mvtime, _get_wait_param());
  res.message = "move line, ret = " + std::to_string(res.ret);
  return true;
}

bool XArmDriver::MoveLineToolCB(xarm_msgs::Move::Request &req, xarm_msgs::Move::Response &res)
{
  float pose[6];
  int index = 0;
  if(req.pose.size() != 6)
  {
    res.ret = PARAM_ERROR;
    res.message = "number of parameters incorrect!";
    return true;
  }
  else
  {
    for(index = 0; index < 6; index++)
    {
      pose[index] = req.pose[index];
    }
  }
  res.ret = arm->set_tool_position(pose, req.mvvelo, req.mvacc, req.mvtime, _get_wait_param());
  res.message = "move line tool, ret = " + std::to_string(res.ret);
  return true;
}

bool XArmDriver::MoveLinebCB(xarm_msgs::Move::Request &req, xarm_msgs::Move::Response &res)
{
  float pose[6];
  int index = 0;
  if(req.pose.size() != 6)
  {
    res.ret = PARAM_ERROR;
    res.message = "number of parameters incorrect!";
    return true;
  }
  else
  {
    for(index = 0; index < 6; index++)
    {
      pose[index] = req.pose[index];
    }
  }
  float mvradii = req.mvradii >= 0 ? req.mvradii : 0;
  res.ret = arm->set_position(pose, mvradii, req.mvvelo, req.mvacc, req.mvtime);        
  res.message = "move lineb, ret = " + std::to_string(res.ret);
  return true;
}

bool XArmDriver::MoveJointbCB(xarm_msgs::Move::Request &req, xarm_msgs::Move::Response &res)
{
  float joint[7]={0};
  int index = 0;
  if(req.pose.size() != dof_)
  {
    res.ret = PARAM_ERROR;
    res.message = "number of joint parameters incorrect! Expected: "+std::to_string(dof_);
    return true;
  }
  else
  {
    for(index = 0; index < 7; index++) // should always send 7 joint commands, whatever current DOF is.
    {
      if(index<req.pose.size())
        joint[index] = req.pose[index];
      else
        joint[index] = 0;
    }
  }
  float mvradii = req.mvradii >= 0 ? req.mvradii : 0;
  res.ret = arm->set_servo_angle(joint, req.mvvelo, req.mvacc, req.mvtime, _get_wait_param(), 0, mvradii);
  res.message = "move jointB, ret = " + std::to_string(res.ret);
  return true;
}

bool XArmDriver::MoveServoJCB(xarm_msgs::Move::Request &req, xarm_msgs::Move::Response &res)
{
  float pose[7]={0};
  int index = 0;
  if(req.pose.size() != dof_)
  {
    res.ret = PARAM_ERROR;
    res.message = "pose parameters incorrect! Expected: "+std::to_string(dof_);
    return true;
  }
  else
  {
    for(index = 0; index < 7; index++) // should always send 7 joint commands, whatever current DOF is.
    {
      if(index<req.pose.size())
        pose[index] = req.pose[index];
      else
        pose[index] = 0;
    }
  }

  res.ret = arm->set_servo_angle_j(pose, req.mvvelo, req.mvacc, req.mvtime);
  res.message = "move servoj, ret = " + std::to_string(res.ret);
  return true;
}

bool XArmDriver::MoveServoCartCB(xarm_msgs::Move::Request &req, xarm_msgs::Move::Response &res)
{
  float pose[6];
  int index = 0;
  if(req.pose.size() != 6)
  {
    res.ret = PARAM_ERROR;
    res.message = "MoveServoCartCB parameters incorrect!";
    return true;
  }
  else
  {
    for(index = 0; index < 6; index++)
    {
      pose[index] = req.pose[index];
    }
  }
  bool is_tool_coord = (req.mvtime != 0.0);
  res.ret = arm->set_servo_cartesian(pose, req.mvvelo, req.mvacc, req.mvtime, is_tool_coord);
  res.message = "move servo_cartesian, ret = " + std::to_string(res.ret);
  return true;
}

bool XArmDriver::MoveLineAACB(xarm_msgs::MoveAxisAngle::Request &req, xarm_msgs::MoveAxisAngle::Response &res)
{
  float pose[6];
  int index = 0;
  if(req.pose.size() != 6)
  {
    res.ret = PARAM_ERROR;
    res.message = "MoveServoCartCB parameters incorrect!";
    return true;
  }
  else
  {
    for(index = 0; index < 6; index++)
    {
      pose[index] = req.pose[index];
    }
  }
  res.ret = arm->set_position_aa(pose, req.mvvelo, req.mvacc, req.mvtime, req.coord, req.relative, _get_wait_param());
  res.message = "move_line_aa, ret = " + std::to_string(res.ret);
  return true;
}

bool XArmDriver::MoveServoCartAACB(xarm_msgs::MoveAxisAngle::Request &req, xarm_msgs::MoveAxisAngle::Response &res)
{
  float pose[6];
  int index = 0;
  if(req.pose.size() != 6)
  {
    res.ret = PARAM_ERROR;
    res.message = "MoveServoCartAACB parameters incorrect!";
    return true;
  }
  else
  {
    for(index = 0; index < 6; index++)
    {
      pose[index] = req.pose[index];
    }
  }
  res.ret = arm->set_servo_cartesian_aa(pose, req.mvvelo, req.mvacc, req.coord, req.relative);
  res.message = "move_servo_cart_aa, ret = " + std::to_string(res.ret);
  return true;
}

bool XArmDriver::VeloMoveJointCB(xarm_msgs::MoveVelo::Request &req, xarm_msgs::MoveVelo::Response &res)
{
  float jnt_v[7]={0};
  int index = 0;
  if(req.velocities.size() < dof_)
  {
    res.ret = PARAM_ERROR;
    res.message = "pose parameters incorrect! Expected: "+std::to_string(dof_);
    return true;
  }
  else
  {
    for(index = 0; index < 7; index++) // should always send 7 joint commands, whatever current DOF is.
    {
      // jnt_v[0][index] = req.velocities[index];
      if(index < req.velocities.size())
        jnt_v[index] = req.velocities[index];
      else
        jnt_v[index] = 0;
    }
  }

  res.ret = arm->vc_set_joint_velocity(jnt_v, req.jnt_sync);
  res.message = "velocity move joint, ret = " + std::to_string(res.ret);
  return true;
}

bool XArmDriver::VeloMoveLineVCB(xarm_msgs::MoveVelo::Request &req, xarm_msgs::MoveVelo::Response &res)
{
  float line_v[6];
  int index = 0;
  if(req.velocities.size() < 6)
  {
    res.ret = PARAM_ERROR;
    res.message = "number of parameters incorrect!";
    return true;
  }
  else
  {
    for(index = 0; index < 6; index++)
    {
      line_v[index] = req.velocities[index];
    }
  }

  res.ret = arm->vc_set_cartesian_velocity(line_v, req.coord);
  res.message = "velocity move line, ret = " + std::to_string(res.ret);
  return true;
}

bool XArmDriver::VCSetJointVelocityCB(xarm_msgs::MoveVelocity::Request &req, xarm_msgs::MoveVelocity::Response &res)
{
  float jnt_v[7]={0};
  int index = 0;
  if(req.speeds.size() < dof_)
  {
    res.ret = PARAM_ERROR;
    res.message = "pose parameters incorrect! Expected: "+std::to_string(dof_);
    return true;
  }
  else
  {
    for(index = 0; index < 7; index++) // should always send 7 joint commands, whatever current DOF is.
    {
      // jnt_v[0][index] = req.velocities[index];
      if(index < req.speeds.size())
        jnt_v[index] = req.speeds[index];
      else
        jnt_v[index] = 0;
    }
  }

  res.ret = arm->vc_set_joint_velocity(jnt_v, req.is_sync, req.duration);
  res.message = "velocity move joint, ret = " + std::to_string(res.ret);
  return true;
}

bool XArmDriver::VCSetCartesianVelocityCB(xarm_msgs::MoveVelocity::Request &req, xarm_msgs::MoveVelocity::Response &res)
{
  float line_v[6];
  int index = 0;
  if(req.speeds.size() < 6)
  {
    res.ret = PARAM_ERROR;
    res.message = "number of parameters incorrect!";
    return true;
  }
  else
  {
    for(index = 0; index < 6; index++)
    {
      line_v[index] = req.speeds[index];
    }
  }

  res.ret = arm->vc_set_cartesian_velocity(line_v, req.is_tool_coord, req.duration);
  res.message = "velocity move line, ret = " + std::to_string(res.ret);
  return true;
}

bool XArmDriver::SetMaxJAccCB(xarm_msgs::SetFloat32::Request &req, xarm_msgs::SetFloat32::Response &res)
{
  if(req.data<0 || req.data>20.0)
  {
    res.ret = PARAM_ERROR;
    res.message = "set max joint acc: " + std::to_string(req.data) + "error! Proper range is: 0-20.0 rad/s^2";
    return true;
  }
  res.ret = arm->set_joint_maxacc(req.data);
  res.message = "set max joint acc: " + std::to_string(req.data) + " ret = " + std::to_string(res.ret);
  return true;
}

bool XArmDriver::SetMaxLAccCB(xarm_msgs::SetFloat32::Request &req, xarm_msgs::SetFloat32::Response &res)
{
  if(req.data<0 || req.data>50000.0)
  {
    res.ret = PARAM_ERROR;
    res.message = "set max linear acc: " + std::to_string(req.data) + "error! Proper range is: 0-50000.0 mm/s^2";
    return true;
  }
  res.ret = arm->set_tcp_maxacc(req.data);
  res.message = "set max linear acc: " + std::to_string(req.data) + " ret = " + std::to_string(res.ret);
  return true;
}

bool XArmDriver::GripperConfigCB(xarm_msgs::GripperConfig::Request &req, xarm_msgs::GripperConfig::Response &res)
{
  if(req.pulse_vel>5000)
    req.pulse_vel = 5000;
  else if(req.pulse_vel<0)
    req.pulse_vel = 0;

  
  int ret1 = arm->set_gripper_mode(0);
  int ret2 = arm->set_gripper_enable(true);
  int ret3 = arm->set_gripper_speed(req.pulse_vel);

  res.ret = (ret1 == 0 && ret2 == 0) ? ret3 : (ret1 == 0 && ret3 == 0) ? ret2 : ret1;
  res.message = "gripper_config, ret = " + std::to_string(res.ret);
  return true;
}

bool XArmDriver::GripperMoveCB(xarm_msgs::GripperMove::Request &req, xarm_msgs::GripperMove::Response &res)
{
  if(req.pulse_pos>850)
    req.pulse_pos = 850;
  else if(req.pulse_pos<-100)
    req.pulse_pos = -100;

  res.ret = arm->set_gripper_position(req.pulse_pos);
  res.message = "gripper_move, ret = " + std::to_string(res.ret);
  return true;
}

bool XArmDriver::GripperStateCB(xarm_msgs::GripperState::Request &req, xarm_msgs::GripperState::Response &res)
{   
  int err_code = 0;
  float pos_now = 0;
  int ret1 = arm->get_gripper_err_code(&err_code);
  int ret2 = arm->get_gripper_position(&pos_now);

  res.ret = ret1 == 0 ? ret2 : ret1;
  res.err_code = err_code;
  res.curr_pos = pos_now;
  // fprintf(stderr, "gripper_pos: %f, gripper_err: %d\n", res.curr_pos, res.err_code);
  return true;
}

bool XArmDriver::VacuumGripperCB_OLD(xarm_msgs::SetInt16::Request &req, xarm_msgs::SetInt16::Response &res)
{
  res.ret = arm->set_vacuum_gripper(req.data);
  res.message = "set vacuum gripper: " + std::to_string(req.data) + " ret = " + std::to_string(res.ret);
  return true;
}

bool XArmDriver::VacuumGripperCB(xarm_msgs::VacuumGripperCtrl::Request &req, xarm_msgs::VacuumGripperCtrl::Response &res)
{
  res.ret = arm->set_vacuum_gripper(req.on, req.wait, req.timeout, req.delay_sec, req.sync, req.hardware_version);
  res.message = "set vacuum gripper: " + std::to_string(req.on) + ", hardware_version=" + std::to_string(req.hardware_version) + ", ret = " + std::to_string(res.ret);
  return true;
}

bool XArmDriver::SetRecordingCB(xarm_msgs::SetInt16::Request &req, xarm_msgs::SetInt16::Response &res)
{  
  if(req.data)
    res.ret = arm->start_record_trajectory(); // start recording
  else
    res.ret = arm->stop_record_trajectory(); // stop recording
  res.message = "set trajectory recording: " + std::to_string(req.data) + " ret = " + std::to_string(res.ret);
  return true;
}

bool XArmDriver::SaveTrajCB(xarm_msgs::SetString::Request &req, xarm_msgs::SetString::Response &res)
{
  if(req.str_data.size()>80)
  {
    res.ret = PARAM_ERROR;
    res.message = "Save Trajectory ERROR: name length should be within 80 characrters!";
    return true;
  }
  char file_name[81]={0};
  req.str_data.copy(file_name, req.str_data.size(), 0);
  float timeout = req.timeout;
  if (timeout <= 0.1) timeout = 10;
  res.ret = arm->save_record_trajectory(file_name, timeout);
  res.message = "save trajectory file: " + req.str_data + " ret = " + std::to_string(res.ret);
  return true;
}

bool XArmDriver::LoadNPlayTrajCB(xarm_msgs::PlayTraj::Request &req, xarm_msgs::PlayTraj::Response &res)
{
  /* LOAD: */
  if(req.traj_file.size()>80)
  {
    res.ret = PARAM_ERROR;
    res.message = "Load Trajectory ERROR: name length should be within 80 characrters!";
    ROS_ERROR("%s", res.message.c_str());
    return true;
  }

  if(req.speed_factor != 1 && req.speed_factor != 2 && req.speed_factor != 4)
  {
    res.ret = PARAM_ERROR;
    res.message = "PlayBack Trajectory ERROR: please check given speed_factor (int: 1, 2 or 4)";
    ROS_ERROR("%s", res.message.c_str());
    return true;
  }

  char file_name[81]={0};
  req.traj_file.copy(file_name, req.traj_file.size(), 0);

  res.ret = arm->playback_trajectory(req.repeat_times, file_name, true, req.speed_factor);

  res.message = "PlayBack Trajectory, ret = " + std::to_string(res.ret);
  return true;

}

bool XArmDriver::SetReboundCB(xarm_msgs::SetInt16::Request& req, xarm_msgs::SetInt16::Response& res)
{
  res.ret = arm->set_collision_rebound((bool)req.data); 
  return true;
}

bool XArmDriver::SetCollSensCB(xarm_msgs::SetInt16::Request& req, xarm_msgs::SetInt16::Response& res)
{
  res.ret = arm->set_collision_sensitivity(req.data); 
  return true;
}

bool XArmDriver::SetTeachSensCB(xarm_msgs::SetInt16::Request& req, xarm_msgs::SetInt16::Response& res)
{
  res.ret = arm->set_teach_sensitivity(req.data); 
  return true;
}

bool XArmDriver::SetWorldOffsetCB(xarm_msgs::TCPOffset::Request &req, xarm_msgs::TCPOffset::Response &res)
{
  float offsets[6] = {req.x, req.y, req.z, req.roll, req.pitch, req.yaw};
  res.ret = arm->set_world_offset(offsets);
  if (res.ret >= 0)
    arm->save_conf();
  res.message = "set world offset: ret = " + std::to_string(res.ret); 
  return true;
}

bool XArmDriver::SetFenceModeCB(xarm_msgs::SetInt16::Request& req, xarm_msgs::SetInt16::Response& res)
{
  res.ret = arm->set_fence_mode((bool)req.data);
  return true;
}

bool XArmDriver::SetReducedModeCB(xarm_msgs::SetInt16::Request& req, xarm_msgs::SetInt16::Response& res)
{
  res.ret = arm->set_reduced_mode((bool)req.data); 
  return true;
}

bool XArmDriver::SetTcpJerkCB(xarm_msgs::SetFloat32::Request &req, xarm_msgs::SetFloat32::Response &res)
{
  res.ret = arm->set_tcp_jerk(req.data);
  res.message = "set tcp jerk: " + std::to_string(req.data) + " ret = " + std::to_string(res.ret);
  return true;
}

bool XArmDriver::SetJointJerkCB(xarm_msgs::SetFloat32::Request &req, xarm_msgs::SetFloat32::Response &res)
{
  res.ret = arm->set_joint_jerk(req.data);
  res.message = "set joint jerk: " + std::to_string(req.data) + " ret = " + std::to_string(res.ret);
  return true;
}

bool XArmDriver::GetServoAngleCB(xarm_msgs::GetFloat32List::Request &req, xarm_msgs::GetFloat32List::Response &res)
{
  res.datas.resize(7);
  res.ret = arm->get_servo_angle(&res.datas[0]);
  std::string tmp = "";
  for (int i = 0; i < res.datas.size(); i++) {
    tmp += (i == 0 ? "" : ", ") + std::to_string(res.datas[i]);
  }
  res.message = "angles=[ " + tmp + " ]";
  return true;
}

bool XArmDriver::GetPositionRPYCB(xarm_msgs::GetFloat32List::Request &req, xarm_msgs::GetFloat32List::Response &res)
{
  res.datas.resize(6);
  res.ret = arm->get_position(&res.datas[0]);
  std::string tmp = "";
  for (int i = 0; i < res.datas.size(); i++) {
    tmp += (i == 0 ? "" : ", ") + std::to_string(res.datas[i]);
  }
  res.message = "position=[ " + tmp + " ]";
  return true;
}

bool XArmDriver::GetPositionAACB(xarm_msgs::GetFloat32List::Request &req, xarm_msgs::GetFloat32List::Response &res)
{
  res.datas.resize(6);
  res.ret = arm->get_position_aa(&res.datas[0]);
  std::string tmp = "";
  for (int i = 0; i < res.datas.size(); i++) {
    tmp += (i == 0 ? "" : ", ") + std::to_string(res.datas[i]);
  }
  res.message = "position=[ " + tmp + " ]";
  return true;
}

bool XArmDriver::GetTgpioBaudRateCB(xarm_msgs::GetInt32::Request &req, xarm_msgs::GetInt32::Response &res)
{
  int baud_rate = 0;
  res.ret = arm->get_tgpio_modbus_baudrate(&baud_rate);
  res.data = baud_rate;
  res.message = "ret = " + std::to_string(res.ret) + ", current baud_rate = " + std::to_string(baud_rate);
  return true;
}

bool XArmDriver::SetModbusToutCB(xarm_msgs::SetModbusTimeout::Request &req, xarm_msgs::SetModbusTimeout::Response &res)
{
  res.ret = arm->set_tgpio_modbus_timeout(req.timeout_ms, req.is_transparent_transmission);
  res.message = "set_tgpio_modbus_timeout, ret="+ std::to_string(res.ret);
  return true;
}

bool XArmDriver::GetSetModbusCB(xarm_msgs::GetSetModbusData::Request &req, xarm_msgs::GetSetModbusData::Response &res)
{
  int send_len = req.send_data.size();
  int recv_len = req.respond_len;
  unsigned char * tx_data = new unsigned char [send_len]{0};
  unsigned char * rx_data = new unsigned char [recv_len]{0};

  for(int i=0; i<send_len; i++)
  {
    tx_data[i] = req.send_data[i];
  }
  res.ret = arm->getset_tgpio_modbus_data(tx_data, send_len, rx_data, recv_len, req.host_id, req.is_transparent_transmission, req.use_503_port);
  for(int i=0; i<recv_len; i++)
  {
    res.respond_data.push_back(rx_data[i]);
  }

  delete [] tx_data;
  delete [] rx_data;
  return true;
}

bool XArmDriver::SetModbusUsePort503CB(xarm_msgs::SetInt16::Request& req, xarm_msgs::SetInt16::Response& res)
{
  res.ret = arm->set_tgpio_modbus_use_503_port(req.data);
  res.message = "set_tgpio_modbus_use_503_port, ret = " + std::to_string(res.ret);
  return true;
}

bool XArmDriver::FtSensorEnable(xarm_msgs::SetInt16::Request& req, xarm_msgs::SetInt16::Response& res)
{
  res.ret = arm->set_ft_sensor_enable(req.data);
  res.message = "set_ft_sensor_enable, ret = " + std::to_string(res.ret);
  return true;
}

bool XArmDriver::FtSensorSetMode(xarm_msgs::SetInt16::Request& req, xarm_msgs::SetInt16::Response& res)
{
  res.ret = arm->set_ft_sensor_mode(req.data);
  res.message = "set_ft_sensor_mode, ret = " + std::to_string(res.ret);
  return true;
}

bool XArmDriver::FtSensorSetZero(xarm_msgs::Call::Request& req, xarm_msgs::Call::Response& res)
{
  res.ret = arm->set_ft_sensor_zero();
  res.message = "set_ft_sensor_zero, ret = " + std::to_string(res.ret);
  return true;
}

bool XArmDriver::FtSensorCaliLoad(xarm_msgs::FtCaliLoad::Request& req, xarm_msgs::FtCaliLoad::Response& res)
{
  if (req.datas.size() < 10) {
    res.ret = PARAM_ERROR;
    return true;
  }

  res.ret = arm->set_ft_sensor_load_offset(&req.datas[0], req.association_setting_tcp_load);
  if (res.ret >= 0)
    arm->save_conf();
  // res.ret = arm->set_ft_sensor_load_offset(&req.datas[0], req.association_setting_tcp_load, req.m, req.x, req.y, req.z);
  res.message = "set_ft_sensor_load_offset, ret = " + std::to_string(res.ret);
  return true;
}

bool XArmDriver::FtSensorIdenLoad(xarm_msgs::FtIdenLoad::Request& req, xarm_msgs::FtIdenLoad::Response& res)
{
  res.datas.resize(10);
  res.ret = arm->iden_ft_sensor_load_offset(&res.datas[0]);
  res.message = "iden_ft_sensor_load_offset, ret = " + std::to_string(res.ret);
  return true;
}

bool XArmDriver::GetFtSensorError(xarm_msgs::GetErr::Request& req, xarm_msgs::GetErr::Response& res)
{
  int err;
  res.ret = arm->get_ft_sensor_error(&err);
  res.err = (signed short)err;
  res.message = "get_ft_sensor_error, ret = " + std::to_string(res.ret) + ", err = " + std::to_string(res.err);
  return true;
}

bool XArmDriver::OpenLite6Gripper(xarm_msgs::Call::Request& req, xarm_msgs::Call::Response& res)
{
  res.ret = arm->open_lite6_gripper();
  res.message = "open_lite6_gripper, ret = " + std::to_string(res.ret);
  return true;
}

bool XArmDriver::CloseLite6Gripper(xarm_msgs::Call::Request& req, xarm_msgs::Call::Response& res)
{
  res.ret = arm->close_lite6_gripper();
  res.message = "close_lite6_gripper, ret = " + std::to_string(res.ret);
  return true;
}

bool XArmDriver::StopLite6Gripper(xarm_msgs::Call::Request& req, xarm_msgs::Call::Response& res)
{
  res.ret = arm->stop_lite6_gripper();
  res.message = "stop_lite6_gripper, ret = " + std::to_string(res.ret);
  return true;
}

void XArmDriver::pub_robot_msg(xarm_msgs::RobotMsg &rm_msg)
{   
  robot_rt_state_.publish(rm_msg);
}

void XArmDriver::pub_joint_state(sensor_msgs::JointState &js_msg)
{
  joint_state_.publish(js_msg);
}

void XArmDriver::pub_cgpio_state(xarm_msgs::CIOState &cio_msg)
{
  cgpio_state_.publish(cio_msg);
}

void XArmDriver::pub_ftsensor_ext_state(geometry_msgs::WrenchStamped &wrench_msg)
{
  ftsensor_ext_state_.publish(wrench_msg);
}

void XArmDriver::pub_ftsensor_raw_state(geometry_msgs::WrenchStamped &wrench_msg)
{
  ftsensor_raw_state_.publish(wrench_msg);
}

bool XArmDriver::is_connected(void) {
  return arm == NULL ? false : arm->is_connected();
}

std::string XArmDriver::controller_error_interpreter(int err)
{
  err = (err==-1) ? curr_err : err;
  switch(err)
  {
    case 0:
      return "Everything OK";
    case 1:
      return "Hardware Emergency STOP effective";
    case 2:
      return "Emergency IO of Control Box is triggered";
    case 3:
      return "Emergency Stop of Three-state Switch triggered";
    case 11 ... 17:
      return std::string("Servo Motor Error of Joint ") + std::to_string(err-10); 
    case 19:
      return "End Module Communication Error";
    case 21:
      return "Kinematic Error";
    case 22:
      return "Self-collision Error";
    case 23:
      return "Joint Angle Exceed Limit";
    case 24:
      return "Speed Exceeds Limit";
    case 25:
      return "Planning Error";
    case 26:
      return "System Real Time Error";
    case 27:
      return "Command Reply Error";
    case 29:
      return "Other Errors, please contact technical support";
    case 30:
      return "Feedback Speed Exceeds limit";
    case 31:
      return "Collision Caused Abnormal Joint Current";
    case 32:
      return "Circle Calculation Error";
    case 33:
      return "Controller GPIO Error";
    case 34:
      return "Trajectory Recording Timeout";
    case 35:
      return "Exceed Safety Boundary";
    case 36:
      return "Number of Delayed Command Exceed Limit";
    case 37:
      return "Abnormal Motion in Manual Mode";
    case 38: 
      return "Abnormal Joint Angle";
    case 39:
      return "Abnormal Communication Between Master and Slave IC of Power Board";
    case 50:
      return "Tool Force/Torque Sensor Error";
    case 51:
      return "Tool Force Torque Sensor Mode Setting Error";
    case 52:
      return "Tool Force Torque Sensor Zero Setting Error";
    case 53:
      return "Tool Force Torque Sensor Overload";
    case 110:
      return "Robot Arm Base Board Communication Error";
    case 111:
      return "Control Box External RS485 Device Communication Error";

    default:
      return "Abnormal Error Code, please contact support!";

  }
}
}
