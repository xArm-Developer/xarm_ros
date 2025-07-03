/* Copyright 2018 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jason Peng <jason@ufactory.cc>
 ============================================================================*/

#include "xarm_controller/xarm_hw.h"

#define SERVICE_CALL_FAILED 999
#define SERVICE_IS_PERSISTENT_BUT_INVALID 998
#define XARM_IS_DISCONNECTED -1
#define VELO_DURATION 1

namespace xarm_control
{
  void XArmHW::_register_joint_limits(ros::NodeHandle &root_nh, std::string joint_name, const ControlMethod ctrl_method)
  {
    joint_limits_interface::JointLimits joint_limits;
    joint_limits_interface::SoftJointLimits soft_joint_limits;
    bool has_limits = false;
    bool has_soft_limits = false;
    
    urdf::JointConstSharedPtr urdf_joint = model_ptr_->getJoint(joint_name);
    if (urdf_joint != NULL) 
    {
      // Get limits from the URDF file.
      if (joint_limits_interface::getJointLimits(urdf_joint, joint_limits))
        has_limits = true;
      if (joint_limits_interface::getSoftJointLimits(urdf_joint, soft_joint_limits))
        has_soft_limits = true;
    }
    // Get limits from the parameter server.
    if (joint_limits_interface::getJointLimits(joint_name, root_nh, joint_limits))
      has_limits = true;

    if (!has_limits)
      return;

    switch (ctrl_method)
    {
      case EFFORT:
      {
        if (has_soft_limits)
          ej_limits_interface_.registerHandle(joint_limits_interface::EffortJointSoftLimitsHandle(ej_interface_.getHandle(joint_name), joint_limits, soft_joint_limits));
        else
          ej_sat_interface_.registerHandle(joint_limits_interface::EffortJointSaturationHandle(ej_interface_.getHandle(joint_name), joint_limits));
        break;
      }
      case VELOCITY:
      {
        if (has_soft_limits)
          vj_limits_interface_.registerHandle(joint_limits_interface::VelocityJointSoftLimitsHandle(vj_interface_.getHandle(joint_name), joint_limits, soft_joint_limits));
        else
          vj_sat_interface_.registerHandle(joint_limits_interface::VelocityJointSaturationHandle(vj_interface_.getHandle(joint_name), joint_limits));
        break;
      }
      case POSITION:
      default:
      {
        if (has_soft_limits)
          pj_limits_interface_.registerHandle(joint_limits_interface::PositionJointSoftLimitsHandle(pj_interface_.getHandle(joint_name), joint_limits, soft_joint_limits));
        else
          pj_sat_interface_.registerHandle(joint_limits_interface::PositionJointSaturationHandle(pj_interface_.getHandle(joint_name), joint_limits));
        break;
      }
    }
    ROS_INFO("[%s] %s, ctrl_method=%d, has_soft_limits=%d, has_velocity_limits=%d, max_velocity=%f, has_position_limits=%d, min_position=%f, max_position=%f\n", 
      robot_ip_.c_str(), joint_name.c_str(), ctrl_method, has_soft_limits, joint_limits.has_velocity_limits, joint_limits.max_velocity, 
      joint_limits.has_position_limits, joint_limits.min_position, joint_limits.max_position);
  }

  void XArmHW::clientInit(const std::string& robot_ip, ros::NodeHandle& root_nh)
  {
    // prev_cmds_float_.resize(dof_);
    // cmds_float_.resize(dof_); // command vector must have 7 dimention!

    memset(prev_cmds_float_, 0, sizeof(float) * 7);
    memset(cmds_float_, 0, sizeof(float) * 7);

    position_cmds_.resize(dof_);
    velocity_cmds_.resize(dof_);
    effort_cmds_.resize(dof_);

    position_states_.resize(dof_);
    velocity_states_.resize(dof_);
    effort_states_.resize(dof_);

    curr_err_ = 0;
    curr_state_ = 4;
    curr_mode_ = 0;
    read_code_ = 0;
    write_code_ = 0;

    // pos_sub_ = root_nh.subscribe(jnt_state_topic, 100, &XArmHW::pos_fb_cb, this);
    // state_sub_ = root_nh.subscribe(xarm_state_topic, 100, &XArmHW::state_fb_cb, this);
    // wrench_sub_ = root_nh.subscribe(xarm_ftsensor_states_topic, 100, &XArmHW::ftsensor_fb_cb, this);

    for(unsigned int j=0; j < dof_; j++)
    {
      // Create joint state interface for all joints
      js_interface_.registerHandle(hardware_interface::JointStateHandle(jnt_names_[j], &position_states_[j], &velocity_states_[j], &effort_states_[j]));
      switch (ctrl_method_)
      {
        case EFFORT:
        {
          ej_interface_.registerHandle(hardware_interface::JointHandle(js_interface_.getHandle(jnt_names_[j]), &effort_cmds_[j]));
          break;
        }
        case VELOCITY:
        {
          vj_interface_.registerHandle(hardware_interface::JointHandle(js_interface_.getHandle(jnt_names_[j]), &velocity_cmds_[j]));
          break;
        }
        case POSITION:
        default:
        {
          ctrl_method_ = POSITION;
          pj_interface_.registerHandle(hardware_interface::JointHandle(js_interface_.getHandle(jnt_names_[j]), &position_cmds_[j]));
          break;
        }
      }
      _register_joint_limits(root_nh, jnt_names_[j], ctrl_method_);
    }

    // fts_interface_.registerHandle(hardware_interface::ForceTorqueSensorHandle(
    // force_torque_sensor_name_, force_torque_sensor_frame_id_, force_, torque_));

    registerInterface(&js_interface_);
    switch (ctrl_method_)
    {
      case EFFORT:
        registerInterface(&ej_interface_);
        break;
      case VELOCITY:
        registerInterface(&vj_interface_);
        break;
      case POSITION:
      default:
        registerInterface(&pj_interface_);
        break;
    }
    // registerInterface(&fts_interface_);
      
    xarm_driver_.arm->clean_error();
    xarm_driver_.arm->clean_warn();
    xarm_driver_.arm->motion_enable(true);
    xarm_driver_.arm->set_mode(ctrl_method_ == VELOCITY ? XARM_MODE::VELO_JOINT : XARM_MODE::SERVO);
    xarm_driver_.arm->set_state(XARM_STATE::START);
  }

  bool XArmHW::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh)
  {
    hw_nh_ = robot_hw_nh;
    root_nh_ = root_nh;
    bool velocity_control = false;
    robot_hw_nh.getParam("velocity_control", velocity_control);
    // ctrl_method_ = EFFORT; // INVALID
    // ctrl_method_ = VELOCITY; // INVALID
    if (velocity_control) {
      ctrl_method_ = VELOCITY;
    }
    else {
      ctrl_method_ = POSITION; // default
    }

    hw_ns_ = robot_hw_nh.getNamespace() + "/";
    std::vector<std::string> jnt_names;
    int xarm_dof = 0;

    if(!robot_hw_nh.hasParam("DOF"))
    {
      ROS_ERROR("ROS Parameter xarm_dof not specified!");
      return false;
    }
    if(!robot_hw_nh.hasParam("xarm_robot_ip"))
    {
      ROS_ERROR("ROS Parameter xarm_robot_ip not specified!");
      return false;
    }
    // commented because now read() will check integrity before write() 
    // If there is no /robot_description parameter, moveit controller may send zero command even controller fails to initialize
    // if(!robot_hw_nh.hasParam("/robot_description"))
    // {
    //   ROS_ERROR("ROS Parameter /robot_description not specified!");
    //   return false;
    // }

    /* getParam forbids to change member */
    robot_hw_nh.getParam("DOF", xarm_dof);
    std::string prefix = "";
    robot_hw_nh.getParam("prefix", prefix);
    robot_hw_nh.getParam("xarm_robot_ip", robot_ip_);
    robot_hw_nh.getParam("joint_names", jnt_names);
    for (int i = 0; i < xarm_dof; i++) {
      jnt_names[i] = prefix + jnt_names[i];
    }
    // robot_hw_nh.param<std::string>("force_torque_sensor_name", force_torque_sensor_name_, "ft_sensor");
    // force_torque_sensor_frame_id_ = "ft_sensor_data";

    locked_ip_key_ = "/uf_robot/" + robot_ip_;
    std::string::size_type pos = 0;
    while ((pos = locked_ip_key_.find(".")) != std::string::npos) {
      locked_ip_key_.replace(pos, 1, "_");
    }

    bool ip_locked = false;

    if (root_nh.hasParam(locked_ip_key_)) {
      root_nh.getParam(locked_ip_key_, ip_locked);
    }
    if (ip_locked) {
      ROS_ERROR("[%s] The same robotic arm can only be controlled by one controller, please check whether multiple controllers are started, the current controller has exited.", robot_ip_.c_str());
      return false;
    }
    else {
      root_nh.setParam(locked_ip_key_, true);
    }

    xarm_driver_.init(robot_hw_nh, robot_ip_, true);
    // 20250613, get joint_states msg reference from xarm_driver
    joint_state_msg_ = xarm_driver_.get_joint_states();

    dof_ = xarm_dof;
    jnt_names_ = jnt_names;
    initial_write_ = true;
    pos_fdb_called_ = false;
    stat_fdb_called_ = false;

    read_cnts_ = 0;
    read_max_time_ = 0;
    read_total_time_ = 0;
    read_duration_ = ros::Duration(0);
    read_failed_cnts_ = 0;

    initialized_ = false;
    read_ready_ = false;

    enforce_limits_ = true;
    if (robot_hw_nh.hasParam("enforce_limits")) {
      robot_hw_nh.getParam("enforce_limits", enforce_limits_);
    }

    std::string hw_ns;
    root_nh.getParam("hw_ns", hw_ns);
    std::string robot_description = "";
    std::string robot_description_path = hw_ns_.substr(0, hw_ns_.length() - hw_ns.length() - 1) + "robot_description";
    root_nh.getParam(robot_description_path, robot_description);
    if (robot_description == "")
      root_nh.getParam("/robot_description", robot_description);
    if (robot_description == "") {
      ROS_ERROR("robot_description is empty!");
      return false;
    }
    model_ptr_ = urdf::parseURDF(robot_description);

    clientInit(robot_ip_, robot_hw_nh);
    return true;
  }

  XArmHW::~XArmHW()
  {
    xarm_driver_.arm->set_mode(XARM_MODE::POSE);
    root_nh_.setParam(locked_ip_key_, false);
  }

  void XArmHW::pos_fb_cb(const sensor_msgs::JointState::ConstPtr& data)
  {
    if (data->header.stamp <= last_joint_state_stamp_) return;
    if (data->name[0]!=jnt_names_[0]) 
    {	
      // in case that the gripper joints are independently published, 
      // then the index j will not be valid for xArm joints  
      return; 
    }

    // static int call_cnt = 0;
    std::lock_guard<std::mutex> locker(mutex_);
    for(int j=0; j<dof_; j++)
    {
      position_states_[j] = data->position[j];
      velocity_states_[j] = data->velocity[j];
      effort_states_[j] = data->effort[j];
    }
    last_joint_state_stamp_ = data->header.stamp;

    if(!pos_fdb_called_)
    {
      pos_fdb_called_ = true;
    }
  }

  void XArmHW::state_fb_cb(const xarm_msgs::RobotMsg::ConstPtr& data)
  {
    curr_mode_ = data->mode;
    curr_state_ = data->state;
    curr_err_ = data->err;
    
    if(!stat_fdb_called_)
      stat_fdb_called_ = true;
  }

  bool XArmHW::wait_fbk_start(ros::Duration timeout)
  {
    if(timeout.isZero())
      return true;

    bool started = false;
    ros::Time end = ros::Time::now() + timeout;
    while(ros::ok() && ros::Time::now() < end)
    {
      started = pos_fdb_called_ && stat_fdb_called_;
      if(started)
        break;
      ros::Duration(0.1).sleep();
    }
    return started;
  }

  // void XArmHW::ftsensor_fb_cb(const geometry_msgs::WrenchStamped::ConstPtr& data)
  // {
  // 	if (data->header.stamp <= last_ftsensor_stamp_) return;
  // 	if (data->header.frame_id != force_torque_sensor_frame_id_) return;
    
  // 	force_[0] = data->wrench.force.x;
  // 	force_[1] = data->wrench.force.y;
  // 	force_[2] = data->wrench.force.z;
  // 	torque_[0] = data->wrench.torque.x;
  // 	torque_[1] = data->wrench.torque.y;
  // 	torque_[2] = data->wrench.torque.z;
  // 	last_ftsensor_stamp_ = data->header.stamp;
  // }

  void XArmHW::_reset_limits(void)
  {
    if (!enforce_limits_) return;
    switch (ctrl_method_)
    {
      case EFFORT:
        // no reset() interface
        break;
      case VELOCITY:
        // no reset() interface
        break;
      case POSITION:
      default:
      {
        pj_sat_interface_.reset();
        pj_limits_interface_.reset();
      }
      break;
    }
  }

  // Keep velocity and position within Moveit "joint_limits" configuration
  void XArmHW::_enforce_limits(const ros::Duration& period)
  {
    if (!enforce_limits_) return;
    switch (ctrl_method_)
    {
      case EFFORT:
      {
        ej_sat_interface_.enforceLimits(period);
        ej_limits_interface_.enforceLimits(period);
        break;
      }
      case VELOCITY:
      {
        vj_sat_interface_.enforceLimits(period);
        vj_limits_interface_.enforceLimits(period);
        break;
      }
      case POSITION:
      default:
      {
        pj_sat_interface_.enforceLimits(period);
        pj_limits_interface_.enforceLimits(period);
        break;
      }
    }
  }

  void XArmHW::read(const ros::Time& time, const ros::Duration& period)
  {
    read_cnts_ += 1;
    ros::Time start = ros::Time::now();
    read_ready_ = _xarm_is_ready_read();
    read_code_ = xarm_driver_.update_joint_states(initialized_);
    // double time_sec = (joint_state_msg_->header.stamp - start).toSec();
    // read_total_time_ += time_sec;
    // if (time_sec > read_max_time_) {
    //   read_max_time_ = time_sec;
    // }
    // if (read_cnts_ % 6000 == 0) {
    //   ROS_INFO("[%s] [READ] cnt: %lld, max: %f, mean: %f, failed: %lld", robot_ip_.c_str(), read_cnts_, read_max_time_, read_total_time_ / read_cnts_, read_failed_cnts_);
    // }
    if (read_code_ == 0 && read_ready_) {
      for (int j = 0; j < dof_; j++) {
        position_states_[j] = joint_state_msg_->position[j];
        velocity_states_[j] = joint_state_msg_->velocity[j];
        // effort_states_[j] = joint_state_msg_->effort[j];
      }
      if (!initialized_) {
        for (uint i = 0; i < position_states_.size(); i++) {
          position_cmds_[i] = position_states_[i];
          velocity_cmds_[i] = 0.0;
        }
      }
    }
    else {
      // initialized_ = read_ready_ && _xarm_is_ready_write();
      if (read_code_) {
        read_failed_cnts_ += 1;
        ROS_ERROR("[%s] Read() returns: %d", robot_ip_.c_str(), read_code_);
        if (read_code_ == XARM_IS_DISCONNECTED) {
          ROS_ERROR("[%s] Robot is disconnected, ros shutdown", robot_ip_.c_str());
          ros::shutdown();
          exit(1);
        }
      }
    }
  }

  void XArmHW::write(const ros::Time& time, const ros::Duration& period)
  {
    write_duration_ += period;
    if (need_reset()) {
      initialized_ = false;
      _reset_limits();
      return;
    }
    initialized_ = true;

    _enforce_limits(period);

    int cmd_ret = 0;
    switch (ctrl_method_)
    {
      case VELOCITY:
      {
        for (int k = 0; k < dof_; k++) { cmds_float_[k] = (float)velocity_cmds_[k]; }
        float velo_duration = VELO_DURATION;
        hw_nh_.param<float>("velo_duration", velo_duration, VELO_DURATION);
        cmd_ret = xarm_driver_.arm->vc_set_joint_velocity(cmds_float_, true, velo_duration);
        break;
      }
      case POSITION:
      default:
      {
        for (int k = 0; k < dof_; k++) { cmds_float_[k] = (float)position_cmds_[k]; }
        if (write_duration_.toSec() > 1 || _check_cmds_is_change(prev_cmds_float_, cmds_float_)) {
          cmd_ret = xarm_driver_.arm->set_servo_angle_j(cmds_float_, 0, 0, 0);
          if (cmd_ret == 0) {
            write_duration_ -= write_duration_;
            for (int i = 0; i < dof_; i++) { 
              prev_cmds_float_[i] = (float)cmds_float_[i];
            }
          }
        }
        break;
      }
    }

    if (cmd_ret != 0 && cmd_ret != UXBUS_STATE::WAR_CODE) {
      // to reset controller, preempt current goal
      write_code_ = cmd_ret;
    }
  }

  bool XArmHW::_check_cmds_is_change(float *prev, float *cur, double threshold)
  {
    for (int i = 0; i < dof_; i++) {
      if (std::abs(cur[i] - prev[i]) > threshold) return true;
    }
    return false;
  }

  bool XArmHW::_xarm_is_ready_read(void)
  {
    static int last_err = xarm_driver_.curr_err;
    int curr_err = xarm_driver_.curr_err;
    if (curr_err != 0) {
      if (last_err != curr_err) {
        ROS_ERROR("[%s] [ns: %s] UFACTORY Error detected! C%d: [ %s ]", robot_ip_.c_str(), hw_ns_.c_str(), curr_err, xarm_driver_.controller_error_interpreter().c_str());
      }
    }
    last_err = curr_err;
    return last_err == 0;
  }

  bool XArmHW::_xarm_is_ready_write(void)
  {
    static bool last_not_ready = false;
    static int last_state = xarm_driver_.curr_state;
    static int last_mode = xarm_driver_.curr_mode;
    int curr_mode = xarm_driver_.curr_mode;
    int curr_state = xarm_driver_.curr_state;

    if (!_xarm_is_ready_read()) {
      last_not_ready = true;
      return false;
    }

    if (curr_state > 2) {
      if (last_state != curr_state) {
        last_state = curr_state;
        ROS_WARN("[%s] [ns: %s] Robot State detected! State: %d", robot_ip_.c_str(), hw_ns_.c_str(), curr_state);
      }
      last_not_ready = true;
      return false;
    }
    last_state = curr_state;

    if (!(ctrl_method_ == VELOCITY ? curr_mode == 4 : curr_mode == 1)) {
      if (last_mode != curr_mode) {
        last_mode = curr_mode;
        ROS_WARN("[%s] [ns: %s] Robot Mode detected! Mode: %d", robot_ip_.c_str(), hw_ns_.c_str(), curr_mode);
      }
      last_not_ready = true;
      return false;
    }
    last_mode = curr_mode;

    if (last_not_ready) {
      ROS_INFO("[%s] [ns: %s] Robot is Ready", robot_ip_.c_str(), hw_ns_.c_str());
    }
    last_not_ready = false;
    return true;
  }

  bool XArmHW::_firmware_version_is_ge(int major, int minor, int revision)
  {
    return xarm_driver_.arm->version_number[0] > major || (xarm_driver_.arm->version_number[0] == major && xarm_driver_.arm->version_number[1] > minor) || (xarm_driver_.arm->version_number[0] == major && xarm_driver_.arm->version_number[1] == minor && xarm_driver_.arm->version_number[2] >= revision);
  }

  void XArmHW::get_status(int state_mode_err[3])
  {
    state_mode_err[0] = xarm_driver_.curr_state;
    state_mode_err[1] = xarm_driver_.curr_mode;
    state_mode_err[2] = xarm_driver_.curr_err;
  }

  bool XArmHW::need_reset()
  {
    bool is_not_ready = !_xarm_is_ready_write();
    bool write_succeed = write_code_ == 0;
    if (!write_succeed) {
      // int ret = xarm_driver_.arm->set_state(XARM_STATE::STOP);
      // ROS_ERROR("[%s] Write() failed, failed_ret=%d !, Setting Robot State to STOP... (ret: %d)", robot_ip_.c_str(), write_code_, ret);
      ROS_ERROR("[%s] Write() failed, failed_ret=%d !", robot_ip_.c_str(), write_code_);
      if (write_code_ == SERVICE_IS_PERSISTENT_BUT_INVALID || write_code_ == SERVICE_CALL_FAILED) {
        ROS_ERROR("[%s] Service is invaild, ros shutdown", robot_ip_.c_str());
        ros::shutdown();
        exit(1);
      }
      else if (write_code_ == XARM_IS_DISCONNECTED) {
        ROS_ERROR("[%s] Robot is disconnected, ros shutdown", robot_ip_.c_str());
        ros::shutdown();
        exit(1);
      }
      write_code_ = 0;
    }
    return is_not_ready || !write_succeed || read_code_ != 0 || !read_ready_;
  }
}

PLUGINLIB_EXPORT_CLASS(xarm_control::XArmHW, hardware_interface::RobotHW)