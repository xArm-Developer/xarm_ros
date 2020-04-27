#ifndef __XARM_ROS_CLIENT_H__
#define __XARM_ROS_CLIENT_H__

#include "ros/ros.h"
#include <xarm_driver.h>

namespace xarm_api{

class XArmROSClient
{
public:
	XArmROSClient(){};
	void init(ros::NodeHandle& nh);
	~XArmROSClient(){};

	int motionEnable(short en);
	int setState(short state);
	int setMode(short mode);
	int setTCPOffset(const std::vector<float>& tcp_offset);
	int setLoad(float mass, const std::vector<float>& center_of_mass);
	int setServoJ(const std::vector<float>& joint_cmd);
	int setServoCartisian(const std::vector<float>& cart_cmd);
	int goHome(float jnt_vel_rad, float jnt_acc_rad=15);
	int moveJoint(const std::vector<float>& joint_cmd, float jnt_vel_rad, float jnt_acc_rad=15);
	int moveLine(const std::vector<float>& cart_cmd, float cart_vel_mm, float cart_acc_mm=500);
	int moveLineB(int num_of_pnts, const std::vector<float> cart_cmds[], float cart_vel_mm, float cart_acc_mm=500, float radii=0);

private:
	ros::ServiceClient motion_ctrl_client_;
	ros::ServiceClient set_mode_client_;
	ros::ServiceClient set_state_client_;
  	ros::ServiceClient go_home_client_;
	ros::ServiceClient move_lineb_client_;
	ros::ServiceClient move_servoj_client_;
	ros::ServiceClient move_servo_cart_client_;
	ros::ServiceClient move_line_client_;
	ros::ServiceClient move_joint_client_;
	ros::ServiceClient set_tcp_offset_client_;
	ros::ServiceClient set_load_client_;

    xarm_msgs::SetAxis set_axis_srv_;
    xarm_msgs::SetInt16 set_int16_srv_;
    xarm_msgs::TCPOffset offset_srv_;
    xarm_msgs::SetLoad set_load_srv_;
    xarm_msgs::Move move_srv_;
    xarm_msgs::Move servoj_msg_;
    xarm_msgs::Move servo_cart_msg_;

    ros::NodeHandle nh_;
};

}

#endif
