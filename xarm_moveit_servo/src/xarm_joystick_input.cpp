/* Copyright 2021 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Vinman <vinman.cub@gmail.com>
 ============================================================================*/

#include "xarm_moveit_servo/xarm_joystick_input.h"
#include "controller_manager_msgs/ListControllers.h"

namespace xarm_moveit_servo
{

static double prev_twist_cmd[6] = {0};

enum JOYSTICK_TYPE
{
    JOYSTICK_XBOX360_WIRED = 1,
    JOYSTICK_XBOX360_WIRELESS = 2,
    JOYSTICK_SPACEMOUSE_WIRELESS = 3
};

enum XBOX360_WIRED_CONTROLLER_AXIS
{
    XBOX360_WIRED_LEFT_STICK_LR = 0,
    XBOX360_WIRED_LEFT_STICK_FB = 1,
    XBOX360_WIRED_LEFT_TRIGGER = 2,
    XBOX360_WIRED_RIGHT_STICK_LR = 3,
    XBOX360_WIRED_RIGHT_STICK_FB = 4,
    XBOX360_WIRED_RIGHT_TRIGGER = 5,
    XBOX360_WIRED_CROSS_KEY_LR = 6,
    XBOX360_WIRED_CROSS_KEY_FB = 7
};

enum XBOX360_WIRELESS_CONTROLLER_AXIS
{
    XBOX360_WIRELESS_LEFT_STICK_LR = 0,
    XBOX360_WIRELESS_LEFT_STICK_FB = 1,
    XBOX360_WIRELESS_RIGHT_STICK_LR = 2,
    XBOX360_WIRELESS_RIGHT_STICK_FB = 3,
    XBOX360_WIRELESS_LEFT_TRIGGER = 4,
    XBOX360_WIRELESS_RIGHT_TRIGGER = 5,
    XBOX360_WIRELESS_CROSS_KEY_LR = 6,
    XBOX360_WIRELESS_CROSS_KEY_FB = 7
};

enum XBOX360_CONTROLLER_BUTTON
{
    XBOX360_BTN_A = 0,
    XBOX360_BTN_B = 1,
    XBOX360_BTN_X = 2,
    XBOX360_BTN_Y = 3,
    XBOX360_BTN_LB = 4,
    XBOX360_BTN_RB = 5,
    XBOX360_BTN_BACK = 6,
    XBOX360_BTN_START = 7,
    XBOX360_BTN_POWER = 8,
    XBOX360_BTN_STICK_LEFT = 9,
    XBOX360_BTN_STICK_RIGHT = 10
};

enum SPACEMOUSE_WIRELESS_AXIS
{
    SPM_STICK_Y = 0,
    SPM_STICK_X = 1,
    SPM_STICK_Z = 2,
    SPM_STICK_PITCH = 3,
    SPM_STICK_ROLL = 4,
    SPM_STICK_YAW = 5
};

enum SPACEMOUSE_WIRELESS_BUTTON
{
    SPM_BTN_LEFT = 0,
    SPM_BTN_RIGHT = 1
};

JoyToServoPub::JoyToServoPub(ros::NodeHandle& nh)
  : nh_(nh), 
    dof_(7), ros_queue_size_(10), joystick_type_(JOYSTICK_XBOX360_WIRED), initialized_status_(10),
    joy_topic_("/joy"),
    cartesian_command_in_topic_("/servo_server/delta_twist_cmds"), 
    joint_command_in_topic_("/servo_server/delta_joint_cmds"), 
    robot_link_command_frame_("link_base"), 
    ee_frame_name_("link_eef"),
    planning_frame_("link_base")
{
    // init parameter from node
    nh_.param<int>("dof", dof_, dof_);
    nh_.param<int>("ros_queue_size", ros_queue_size_, ros_queue_size_);
    nh_.param<int>("joystick_type", joystick_type_, joystick_type_);
    nh_.param<std::string>("joy_topic", joy_topic_, joy_topic_);
    nh_.param<std::string>("/servo_server/cartesian_command_in_topic", cartesian_command_in_topic_, cartesian_command_in_topic_);
    nh_.param<std::string>("/servo_server/joint_command_in_topic", joint_command_in_topic_, joint_command_in_topic_);
    nh_.param<std::string>("/servo_server/robot_link_command_frame", robot_link_command_frame_, robot_link_command_frame_);
    nh_.param<std::string>("/servo_server/ee_frame_name", ee_frame_name_, ee_frame_name_);
    nh_.param<std::string>("/servo_server/planning_frame", planning_frame_, planning_frame_);

    if (cartesian_command_in_topic_.rfind("/servo_server/", 0) != 0) {
        cartesian_command_in_topic_ = "/servo_server/" + cartesian_command_in_topic_;
    }
    if (joint_command_in_topic_.rfind("/servo_server/", 0) != 0) {
        joint_command_in_topic_ = "/servo_server/" + joint_command_in_topic_;
    }

    ros::service::waitForService("controller_manager/list_controllers");
    controller_manager_msgs::ListControllers list_ctrlers_srv;
    int ctrllers_num = 0;
    bool ret=false, all_controllers_running=true;
    std::string ns = ros::this_node::getNamespace();
    do
    {
        ros::ServiceClient list_ctrlers_client = nh_.serviceClient<controller_manager_msgs::ListControllers>(ns+"/"+"controller_manager/list_controllers");
        ret = list_ctrlers_client.call(list_ctrlers_srv);
        // ROS_WARN("[%s] Calling List controllers in code: ret = %d", ns.c_str(), ret);
        ctrllers_num = list_ctrlers_srv.response.controller.size();
        for(int ii=0; ii<ctrllers_num; ii++)
        {
            if(list_ctrlers_srv.response.controller[ii].state != std::string("running"))
            {
                all_controllers_running = false;
            }
            ROS_INFO("controller number %d, name: %s, state: %s", ii+1, list_ctrlers_srv.response.controller[ii].name.c_str(), list_ctrlers_srv.response.controller[ii].state.c_str());
        }
        ros::Duration(0.5).sleep();
    }
    while(ctrllers_num==0 || !all_controllers_running);

    // Setup pub/sub
    joy_sub_ = nh_.subscribe(joy_topic_, ros_queue_size_, &JoyToServoPub::_joy_callback, this);
    twist_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(cartesian_command_in_topic_, ros_queue_size_);
    joint_pub_ = nh_.advertise<control_msgs::JointJog>(joint_command_in_topic_, ros_queue_size_);
}

void JoyToServoPub::_filter_twist_msg(boost::shared_ptr<geometry_msgs::TwistStamped>& twist, double filter_coeff, double zero_threshold)
{   
    filter_coeff = std::min(std::max(0.0, filter_coeff), 1.0);
    twist->twist.linear.x = twist->twist.linear.x + filter_coeff * (prev_twist_cmd[0]-twist->twist.linear.x);
    twist->twist.linear.y = twist->twist.linear.y + filter_coeff * (prev_twist_cmd[1]-twist->twist.linear.y);
    twist->twist.linear.z = twist->twist.linear.z + filter_coeff * (prev_twist_cmd[2]-twist->twist.linear.z);
    twist->twist.angular.x = twist->twist.angular.x + filter_coeff * (prev_twist_cmd[3]-twist->twist.angular.x);
    twist->twist.angular.y = twist->twist.angular.y + filter_coeff * (prev_twist_cmd[4]-twist->twist.angular.y);
    twist->twist.angular.z = twist->twist.angular.z + filter_coeff * (prev_twist_cmd[5]-twist->twist.angular.z);

    prev_twist_cmd[0] = twist->twist.linear.x;
    prev_twist_cmd[1] = twist->twist.linear.y;
    prev_twist_cmd[2] = twist->twist.linear.z;
    prev_twist_cmd[3] = twist->twist.angular.x;
    prev_twist_cmd[4] = twist->twist.angular.y;
    prev_twist_cmd[5] = twist->twist.angular.z;

    if (abs(twist->twist.linear.x) < zero_threshold) {
        twist->twist.linear.x = 0;
    }
    if (abs(twist->twist.linear.y) < zero_threshold) {
        twist->twist.linear.y = 0;
    }
    if (abs(twist->twist.linear.z) < zero_threshold) {
        twist->twist.linear.z = 0;
    }
    if (abs(twist->twist.angular.x) < zero_threshold) {
        twist->twist.angular.x = 0;
    }
    if (abs(twist->twist.angular.y) < zero_threshold) {
        twist->twist.angular.y = 0;
    }
    if (abs(twist->twist.angular.z) < zero_threshold) {
        twist->twist.angular.z = 0;
    }
}

bool JoyToServoPub::_convert_xbox360_joy_to_cmd(
    const std::vector<float>& axes, const std::vector<int>& buttons,
    boost::shared_ptr<geometry_msgs::TwistStamped>& twist,
    boost::shared_ptr<control_msgs::JointJog>& joint)
{
    // xbox360 wired axis
    int left_stick_lr = XBOX360_WIRED_LEFT_STICK_LR;
    int left_stick_fb = XBOX360_WIRED_LEFT_STICK_FB;
    int right_stick_lr = XBOX360_WIRED_RIGHT_STICK_LR;
    int right_stick_fb = XBOX360_WIRED_RIGHT_STICK_FB;
    int left_trigger = XBOX360_WIRED_LEFT_TRIGGER;
    int right_trigger = XBOX360_WIRED_RIGHT_TRIGGER;
    int cross_key_lr = XBOX360_WIRED_CROSS_KEY_LR;
    int cross_key_fb = XBOX360_WIRED_CROSS_KEY_FB;
    
    if (joystick_type_ == JOYSTICK_XBOX360_WIRELESS) {
        // xbox360 wireless axis
        left_stick_lr = XBOX360_WIRELESS_LEFT_STICK_LR;
        left_stick_fb = XBOX360_WIRELESS_LEFT_STICK_FB;
        right_stick_lr = XBOX360_WIRELESS_RIGHT_STICK_LR;
        right_stick_fb = XBOX360_WIRELESS_RIGHT_STICK_FB;
        left_trigger = XBOX360_WIRELESS_LEFT_TRIGGER;
        right_trigger = XBOX360_WIRELESS_RIGHT_TRIGGER;
        cross_key_lr = XBOX360_WIRELESS_CROSS_KEY_LR;
        cross_key_fb = XBOX360_WIRELESS_CROSS_KEY_FB;
    }

    if (buttons[XBOX360_BTN_BACK] && planning_frame_ == ee_frame_name_) {
        planning_frame_ = robot_link_command_frame_;
    }
    else if (buttons[XBOX360_BTN_START] && planning_frame_ == robot_link_command_frame_) {
        planning_frame_ = ee_frame_name_;
    }
    
    if (buttons[XBOX360_BTN_A] || buttons[XBOX360_BTN_B] 
        || buttons[XBOX360_BTN_X] || buttons[XBOX360_BTN_Y] 
        || axes[cross_key_lr] || axes[cross_key_fb])
    {
        // Map the D_PAD to the proximal joints
        joint->joint_names.push_back("joint1");
        joint->velocities.push_back(axes[cross_key_lr] * 1);
        joint->joint_names.push_back("joint2");
        joint->velocities.push_back(axes[cross_key_fb] * 1);

        // Map the diamond to the distal joints
        joint->joint_names.push_back("joint" + std::to_string(dof_));
        joint->velocities.push_back((buttons[XBOX360_BTN_B] - buttons[XBOX360_BTN_X]) * 1);
        joint->joint_names.push_back("joint" + std::to_string(dof_ - 1));
        joint->velocities.push_back((buttons[XBOX360_BTN_Y] - buttons[XBOX360_BTN_A]) * 1);
        return false;
    }

    // The bread and butter: map buttons to twist commands
    twist->twist.linear.x = axes[left_stick_fb];
    twist->twist.linear.y = axes[left_stick_lr];
    twist->twist.linear.z = -1 * (axes[left_trigger] - axes[right_trigger]);
    twist->twist.angular.y = axes[right_stick_fb];
    twist->twist.angular.x = axes[right_stick_lr];
    twist->twist.angular.z = buttons[XBOX360_BTN_LB] - buttons[XBOX360_BTN_RB];

    return true;
}

bool JoyToServoPub::_convert_spacemouse_wireless_joy_to_cmd(const std::vector<float>& axes, const std::vector<int>& buttons,
    boost::shared_ptr<geometry_msgs::TwistStamped>& twist)
{
    twist->twist.linear.x = axes[SPM_STICK_X];
    twist->twist.linear.y = axes[SPM_STICK_Y];
    twist->twist.linear.z = axes[SPM_STICK_Z];
    
    twist->twist.angular.x = axes[SPM_STICK_ROLL];
    twist->twist.angular.y = axes[SPM_STICK_PITCH];
    twist->twist.angular.z = axes[SPM_STICK_YAW];

    if (buttons[SPM_BTN_LEFT]) {
        twist->twist.angular.x = 0;
        twist->twist.angular.y = 0;
        twist->twist.angular.z = 0;
    }
    if (buttons[SPM_BTN_RIGHT]) {
        twist->twist.linear.x = 0;
        twist->twist.linear.y = 0;
        twist->twist.linear.z = 0;
    }
    return true;
}

void JoyToServoPub::_joy_callback(const sensor_msgs::Joy::ConstPtr& msg)
{

    // Create the messages we might publish
    auto twist_msg = moveit::util::make_shared_from_pool<geometry_msgs::TwistStamped>();
    auto joint_msg = moveit::util::make_shared_from_pool<control_msgs::JointJog>();

    if (dof_ == 7 && initialized_status_) {
        initialized_status_ -= 1;
        joint_msg->joint_names.push_back("joint1");
        joint_msg->velocities.push_back(initialized_status_ > 0 ? 0.01 : 0);

        joint_msg->header.stamp = ros::Time::now();
        joint_msg->header.frame_id = "joint1";
        joint_pub_.publish(std::move(joint_msg));

        return;
    }

    bool pub_twist = false;

    switch (joystick_type_) {
        case JOYSTICK_XBOX360_WIRED: // xbox360 wired
        case JOYSTICK_XBOX360_WIRELESS: // xbox360 wireless
            if (msg->axes.size() != 8 || msg->buttons.size() != 11)
                return;
            pub_twist = _convert_xbox360_joy_to_cmd(msg->axes, msg->buttons, twist_msg, joint_msg);
            break;
        case JOYSTICK_SPACEMOUSE_WIRELESS: // spacemouse wireless
            if (msg->axes.size() != 6 || msg->buttons.size() != 2)
                return;
            pub_twist = _convert_spacemouse_wireless_joy_to_cmd(msg->axes, msg->buttons, twist_msg);
            break;
        default:
            return;
    }
    if (pub_twist) {
        // filter and publish the TwistStamped
        // Tune the filter parameters "filter_coeff" and "zero_threshold" if needed!
        _filter_twist_msg(twist_msg, 0.5, 0.1);
      
        twist_msg->header.frame_id = planning_frame_;
        twist_msg->header.stamp = ros::Time::now();
        twist_pub_.publish(std::move(twist_msg));
    }
    else {
        // publish the JointJog
        joint_msg->header.stamp = ros::Time::now();
        joint_msg->header.frame_id = "joint";
        joint_pub_.publish(std::move(joint_msg));
    }
}

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "JoyToServoPubNode");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh("~");

  xarm_moveit_servo::JoyToServoPub joy_to_servo_pub(nh);

  ros::waitForShutdown();

  return 0;
}
