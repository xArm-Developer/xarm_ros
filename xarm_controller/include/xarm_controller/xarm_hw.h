/* Copyright 2018 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jason Peng <jason@ufactory.cc>
 ============================================================================*/

#ifndef __XARM_HARDWARE_INTERFACE_H__
#define __XARM_HARDWARE_INTERFACE_H__

// ros_control
#include <control_toolbox/pid.h>
#include <hardware_interface/joint_state_interface.h>
// #include <hardware_interface/force_torque_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <urdf_parser/urdf_parser.h>
#include <urdf/model.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
// ROS
#include <ros/ros.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/JointState.h>
// #include <geometry_msgs/WrenchStamped.h>
// for mutex
#include <mutex>
#include <thread>
// xarm
#include "xarm/core/instruction/uxbus_cmd_config.h"
// #include "xarm_api/xarm_ros_client.h"
#include "xarm_api/xarm_driver.h"


namespace xarm_control
{
  const std::string jnt_state_topic = "joint_states";
  const std::string xarm_state_topic = "xarm_states";
  // const std::string xarm_ftsensor_states_topic = "xarm_ftsensor_states";

  class XArmHW : public hardware_interface::RobotHW
  {
  public:
    XArmHW(){};
    ~XArmHW();
    virtual bool init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh);
    virtual void read(const ros::Time& time, const ros::Duration& period);
    virtual void write(const ros::Time& time, const ros::Duration& period);

    /* TODO:
    virtual bool prepareSwitch(const std::list<ControllerInfo>& start_list,
                               const std::list<ControllerInfo>& stop_list) { return true; }
    virtual void doSwitch(const std::list<ControllerInfo>& ,
                          const std::list<ControllerInfo>& ) {} */
    
    /* get current arm status: in the order of state, mode and error_code */
    void get_status(int state_mode_err[3]);
    /* check whether the controller needs to be reset due to error or mode change */
    bool need_reset();
    bool wait_fbk_start(ros::Duration timeout);
  
  protected:
    enum ControlMethod {POSITION, VELOCITY, EFFORT};

  private:
    int curr_state_;
    int curr_mode_;
    int curr_err_;
    int read_code_;
    int write_code_;

    unsigned int dof_;
    std::string robot_ip_;
    std::vector<std::string> jnt_names_;

    // std::vector<float> prev_cmds_float_;
        // std::vector<float> cmds_float_;
    float prev_cmds_float_[7];
    float cmds_float_[7];
    std::vector<double> position_cmds_;
    std::vector<double> velocity_cmds_;
    std::vector<double> effort_cmds_;
    std::vector<double> position_states_;
    std::vector<double> velocity_states_;
    std::vector<double> effort_states_;

    bool initialized_;
    bool read_ready_;

    unsigned long long read_cnts_;
    unsigned long long read_failed_cnts_;
    double read_max_time_;
    double read_total_time_;

    float prev_read_position_[7];
    float curr_read_position_[7];
    float curr_read_velocity_[7];
    float curr_read_effort_[7];

    ros::Duration read_duration_;
    ros::Duration write_duration_;

    ros::NodeHandle root_nh_;
    ros::NodeHandle hw_nh_;
    
    bool enforce_limits_;
    std::string locked_ip_key_;

    // double force_[3];
    // double torque_[3];

    bool initial_write_;		
    std::mutex mutex_;
    std::string hw_ns_;
    bool pos_fdb_called_;
    bool stat_fdb_called_;
    // std::string force_torque_sensor_name_;
    // std::string force_torque_sensor_frame_id_;
    
    ros::Time last_joint_state_stamp_;
    // ros::Time last_ftsensor_stamp_;
    
    // xarm_api::XArmROSClient xarm;
    xarm_api::XArmDriver xarm_driver_;
    sensor_msgs::JointState *joint_state_msg_;

    urdf::ModelInterfaceSharedPtr model_ptr_;
    ControlMethod ctrl_method_;

    hardware_interface::JointStateInterface    js_interface_;
    hardware_interface::EffortJointInterface   ej_interface_;
    hardware_interface::PositionJointInterface pj_interface_;
    hardware_interface::VelocityJointInterface vj_interface_;
    // hardware_interface::ForceTorqueSensorInterface fts_interface_;

    joint_limits_interface::EffortJointSaturationInterface   ej_sat_interface_;
    joint_limits_interface::EffortJointSoftLimitsInterface   ej_limits_interface_;
    joint_limits_interface::PositionJointSaturationInterface pj_sat_interface_;
    joint_limits_interface::PositionJointSoftLimitsInterface pj_limits_interface_;
    joint_limits_interface::VelocityJointSaturationInterface vj_sat_interface_;
    joint_limits_interface::VelocityJointSoftLimitsInterface vj_limits_interface_;

    ros::Subscriber pos_sub_, vel_sub_, effort_sub_, state_sub_;
    // ros::Subscriber wrench_sub_;

    void clientInit(const std::string& robot_ip, ros::NodeHandle &root_nh);
    void pos_fb_cb(const sensor_msgs::JointState::ConstPtr& data);
    void state_fb_cb(const xarm_msgs::RobotMsg::ConstPtr& data);
    // void ftsensor_fb_cb(const geometry_msgs::WrenchStamped::ConstPtr& data);

    void _register_joint_limits(ros::NodeHandle &root_nh, std::string joint_name, const ControlMethod ctrl_method);
    void _reset_limits(void);
    void _enforce_limits(const ros::Duration& period);
    bool _check_cmds_is_change(float *prev, float *cur, double threshold = 0.0001);
    bool _xarm_is_ready_read(void);
    bool _xarm_is_ready_write(void);
    bool _firmware_version_is_ge(int major, int minor, int revision);
  };

}

#endif
