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
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
// ROS
#include <ros/ros.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/JointState.h>
// for mutex
#include <pthread.h>
// xarm
#include "xarm/instruction/uxbus_cmd_config.h"
#include "xarm_ros_client.h"

namespace xarm_control
{

const std::string jnt_state_topic = "joint_states";

class XArmHWInterface : public hardware_interface::RobotHW
{
public:
	XArmHWInterface(unsigned int dof, std::vector<std::string>& jnt_names, const std::string& robot_ip, ros::NodeHandle &root_nh):dof_(dof),xarm(root_nh)
	{
		jnt_names_ = jnt_names;
		clientInit(robot_ip, root_nh);
	};
	~XArmHWInterface();
	void read();
	void write();

	/* TODO:
	virtual bool prepareSwitch(const std::list<ControllerInfo>& start_list,
                             const std::list<ControllerInfo>& stop_list) { return true; }
  	virtual void doSwitch(const std::list<ControllerInfo>& ,
                        const std::list<ControllerInfo>& ) {}*/

private:
	unsigned int dof_;
	std::vector<std::string> jnt_names_;
	std::vector<double> position_cmd_;
	std::vector<float> position_cmd_float_;
	std::vector<double> velocity_cmd_;
	std::vector<double> effort_cmd_;

	std::vector<double> position_fdb_;
	std::vector<double> velocity_fdb_;
	std::vector<double> effort_fdb_;

	xarm_api::XArmROSClient xarm;

	hardware_interface::JointStateInterface    js_interface_;
  	hardware_interface::EffortJointInterface   ej_interface_;
  	hardware_interface::PositionJointInterface pj_interface_;
  	hardware_interface::VelocityJointInterface vj_interface_;

	ros::Subscriber pos_sub_, vel_sub_, effort_sub_;

	void clientInit(const std::string& robot_ip, ros::NodeHandle &root_nh);
	void pos_fb_cb(const sensor_msgs::JointState::ConstPtr& data);

};

void XArmHWInterface::clientInit(const std::string& robot_ip, ros::NodeHandle &root_nh)
{
	position_cmd_.resize(dof_);
	position_cmd_float_.resize(7); // command vector must have 7 dimention!
	position_fdb_.resize(dof_);
	velocity_cmd_.resize(dof_);
	velocity_fdb_.resize(dof_);
	effort_cmd_.resize(dof_);
	effort_fdb_.resize(dof_);

	pos_sub_ = root_nh.subscribe(jnt_state_topic, 100, &XArmHWInterface::pos_fb_cb, this);

	for(unsigned int j=0; j < dof_; j++)
  	{
  		// Create joint state interface for all joints
    	js_interface_.registerHandle(hardware_interface::JointStateHandle(jnt_names_[j], &position_fdb_[j], &velocity_fdb_[j], &effort_fdb_[j]));

    	hardware_interface::JointHandle joint_handle;
    	joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(jnt_names_[j]),&position_cmd_[j]);
      	pj_interface_.registerHandle(joint_handle);
  	}

  	registerInterface(&js_interface_);
  	registerInterface(&pj_interface_);
  	
  	int ret1 = xarm.motionEnable(1);
  	int ret2 = xarm.setMode(XARM_MODE::SERVO);
  	int ret3 = xarm.setState(XARM_STATE::START);

  	if(ret1 || ret2 || ret3)
  	{
  		ROS_ERROR("The Xarm may not be properly connected or hardware error exists, PLEASE CHECK or RESTART HARDWARE!!!");
  		ROS_ERROR(" ");
  		ROS_ERROR("Did you specify the correct ros param xarm_robot_ip ? Exitting...");
  		exit(1);
  	}

}

XArmHWInterface::~XArmHWInterface()
{
	xarm.setMode(XARM_MODE::POSE);
}

void XArmHWInterface::pos_fb_cb(const sensor_msgs::JointState::ConstPtr& data)
{

	for(int j=0; j<dof_; j++)
	{
		position_fdb_[j] = data->position[j];
		velocity_fdb_[j] = data->velocity[j];
		effort_fdb_[j] = data->effort[j];
	}
}

void XArmHWInterface::read()
{

}

void XArmHWInterface::write()
{
	for(int k=0; k<dof_; k++)
	{
		position_cmd_float_[k] = (float)position_cmd_[k];
	}

	xarm.setServoJ(position_cmd_float_);
}

} // namespace xarm_control


int main(int argc, char**argv)
{
	ros::init(argc, argv, "xarm_controller");
	ros::NodeHandle nh;
	ros::Rate r(100); // ServoJ mode can not handle update rate greater than 100Hz
	std::string ip;
	std::vector<std::string> jnt_names;
	int xarm_dof = 0;
	if(!nh.hasParam("DOF"))
	{
		ROS_ERROR("ROS Parameter xarm_dof not specified!");
	}
	if(!nh.hasParam("xarm_robot_ip"))
	{
		ROS_ERROR("ROS Parameter xarm_robot_ip not specified!");
	}
	
	nh.getParam("DOF", xarm_dof);
	nh.getParam("xarm_robot_ip", ip);
	nh.getParam("joint_names", jnt_names);

	ros::service::waitForService("motion_ctrl");
  	ros::service::waitForService("set_state");
  	ros::service::waitForService("set_mode");
  	ros::service::waitForService("move_servoj");
	ros::Duration(1.0).sleep();
	xarm_control::XArmHWInterface xarm_hw(xarm_dof, jnt_names, ip, nh);
	controller_manager::ControllerManager cm(&xarm_hw, nh);

  	ros::AsyncSpinner spinner(4);
	spinner.start();

	// IMPORTANT: DO NOT REMOVE THIS DELAY !!!
	/* Wait for correct initial position to be updated to ros_controller */
	ros::Duration(2.0).sleep();

	ros::Time ts = ros::Time::now();
	while (ros::ok())
	{	
	   ros::Duration elapsed = ros::Time::now() - ts;
	   ts = ros::Time::now();
	   // xarm_hw.read();
	   cm.update(ts, elapsed);
	   xarm_hw.write();
	   r.sleep();
	}
	spinner.stop();
	return 0;
}

#endif
