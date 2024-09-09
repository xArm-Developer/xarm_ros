/* Copyright 2021 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Vinman <vinman.cub@gmail.com>
 ============================================================================*/

#ifndef __XARM_JOY_STICK_INPUT_H__
#define __XARM_JOY_STICK_INPUT_H__

#include <moveit_servo/make_shared_from_pool.h>
#include "geometry_msgs/TwistStamped.h"
#include "control_msgs/JointJog.h"
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"


namespace xarm_moveit_servo
{

static const int NUM_SPINNERS = 1;

class JoyToServoPub 
{
public:
    JoyToServoPub(ros::NodeHandle& nh);
private:
    bool _convert_xbox360_joy_to_cmd(
        const std::vector<float>& axes, const std::vector<int>& buttons,
        boost::shared_ptr<geometry_msgs::TwistStamped>& twist,
        boost::shared_ptr<control_msgs::JointJog>& joint);
    bool _convert_spacemouse_wireless_joy_to_cmd(const std::vector<float>& axes, const std::vector<int>& buttons,
        boost::shared_ptr<geometry_msgs::TwistStamped>& twist);

    void _filter_twist_msg(boost::shared_ptr<geometry_msgs::TwistStamped>& twist, double filter_coeff=0.4, double zero_threshold=0.05);
    void _joy_callback(const sensor_msgs::Joy::ConstPtr& msg);

    ros::Subscriber joy_sub_;
    ros::Publisher twist_pub_;
    ros::Publisher joint_pub_;

    int dof_;
    int ros_queue_size_;
    int joystick_type_;
    int initialized_status_;

    std::string joy_topic_;
    std::string cartesian_command_in_topic_;
    std::string joint_command_in_topic_;

    std::string robot_link_command_frame_;
    std::string ee_frame_name_;

    std::string planning_frame_;

    ros::NodeHandle nh_;
};
}


#endif // __XARM_JOY_STICK_INPUT_H__
