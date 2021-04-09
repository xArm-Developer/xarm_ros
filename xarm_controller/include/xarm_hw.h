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
// for mutex
#include <mutex>
// xarm
#include "xarm/core/instruction/uxbus_cmd_config.h"
#include "xarm_ros_client.h"


namespace xarm_control
{
	const std::string jnt_state_topic = "joint_states";
	const std::string xarm_state_topic = "xarm_states";

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
	                        const std::list<ControllerInfo>& ) {}*/
		
		/* get current arm status: in the order of state, mode and error_code */
		void get_status(int state_mode_err[3]);
		/* check whether the controller needs to be reset due to error or mode change */
		bool need_reset();
	
	protected:
		enum ControlMethod {POSITION, VELOCITY, EFFORT};

	private:
		int curr_state;
		int curr_mode;
		int curr_err;

		unsigned int dof_;
		std::vector<std::string> jnt_names_;
		std::vector<double> position_cmd_;
		std::vector<float> position_cmd_float_;
		std::vector<double> velocity_cmd_;
		std::vector<float> velocity_cmd_float_;
		std::vector<double> effort_cmd_;

		std::vector<double> position_fdb_;
		std::vector<double> velocity_fdb_;
		std::vector<double> effort_fdb_;

		bool initial_write_;		
		std::mutex mutex_;
		std::string hw_ns_;
		
		xarm_api::XArmROSClient xarm;

		urdf::ModelInterfaceSharedPtr model_ptr_;
		ControlMethod ctrl_method_;

		hardware_interface::JointStateInterface    js_interface_;
		hardware_interface::EffortJointInterface   ej_interface_;
		hardware_interface::PositionJointInterface pj_interface_;
		hardware_interface::VelocityJointInterface vj_interface_;

		joint_limits_interface::EffortJointSaturationInterface   ej_sat_interface_;
		joint_limits_interface::EffortJointSoftLimitsInterface   ej_limits_interface_;
		joint_limits_interface::PositionJointSaturationInterface pj_sat_interface_;
		joint_limits_interface::PositionJointSoftLimitsInterface pj_limits_interface_;
		joint_limits_interface::VelocityJointSaturationInterface vj_sat_interface_;
		joint_limits_interface::VelocityJointSoftLimitsInterface vj_limits_interface_;

		ros::Subscriber pos_sub_, vel_sub_, effort_sub_, state_sub_;

		void clientInit(const std::string& robot_ip, ros::NodeHandle &root_nh);
		void pos_fb_cb(const sensor_msgs::JointState::ConstPtr& data);
		void state_fb_cb(const xarm_msgs::RobotMsg::ConstPtr& data);

		void _register_joint_limits(ros::NodeHandle &root_nh, std::string joint_name, const ControlMethod ctrl_method);
		void _reset_limits(void);
		void _enforce_limits(const ros::Duration& period);
	};

}

#endif
