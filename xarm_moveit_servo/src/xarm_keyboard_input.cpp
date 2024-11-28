/* Copyright 2021 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Vinman <vinman.cub@gmail.com>
 ============================================================================*/

#include <signal.h>
#include <stdio.h>
#include <unistd.h>
#include "geometry_msgs/TwistStamped.h"
#include "control_msgs/JointJog.h"
#include <moveit_servo/make_shared_from_pool.h>
#include "xarm_moveit_servo/xarm_keyboard_input.h"
/****************************** Jason added 20240612 ***************************************/
#include "controller_manager_msgs/ListControllers.h"
/****************************** Jason added 20240612 ***************************************/

// Define used keys
#define KEYCODE_RIGHT 0x43
#define KEYCODE_LEFT 0x44
#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42
#define KEYCODE_PERIOD 0x2E
#define KEYCODE_SEMICOLON 0x3B
#define KEYCODE_1 0x31
#define KEYCODE_2 0x32
#define KEYCODE_3 0x33
#define KEYCODE_4 0x34
#define KEYCODE_5 0x35
#define KEYCODE_6 0x36
#define KEYCODE_7 0x37
#define KEYCODE_Q 0x71
#define KEYCODE_W 0x77
#define KEYCODE_E 0x65
#define KEYCODE_R 0x72

KeyboardReader keyboard_reader_;


KeyboardServoPub::KeyboardServoPub(ros::NodeHandle& nh)
: nh_(nh), dof_(6), ros_queue_size_(10),
cartesian_command_in_topic_("/servo_server/delta_twist_cmds"), 
joint_command_in_topic_("/servo_server/delta_joint_cmds"), 
robot_link_command_frame_("link_base"), 
ee_frame_name_("link_eef"),
planning_frame_("link_base"),
joint_vel_cmd_(1.0),
linear_pos_cmd_(0.5)
{
    // init parameter from node
    nh_.param<int>("dof", dof_, dof_);
    nh_.param<int>("ros_queue_size", ros_queue_size_, ros_queue_size_);
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

    // Setup pub/sub
    twist_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(cartesian_command_in_topic_, ros_queue_size_);
    joint_pub_ = nh_.advertise<control_msgs::JointJog>(joint_command_in_topic_, ros_queue_size_);
}

void KeyboardServoPub::keyLoop()
{
    char c;
    bool publish_twist = false;
    bool publish_joint = false;

    /****************************** Jason added 20240612 ***************************************/
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
    /****************************** Jason added 20240612 ***************************************/

    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use arrow keys and the '.' and ';' keys to Cartesian jog");
    puts("Use 'W' to Cartesian jog in the world frame, and 'E' for the End-Effector frame");
    puts("Use 1|2|3|4|5|6|7 keys to joint jog. 'R' to reverse the direction of jogging.");
    puts("'Q' to quit.");
    
    for (;;) {
        try {
            keyboard_reader_.readOne(&c);
        }
        catch (const std::runtime_error&) {
            perror("read():");
            return;
        }
        ROS_DEBUG("value: 0x%02X", c);

        auto twist_msg = moveit::util::make_shared_from_pool<geometry_msgs::TwistStamped>();
        auto joint_msg = moveit::util::make_shared_from_pool<control_msgs::JointJog>();

        // Use read key-press
        switch (c)
        {
        case KEYCODE_LEFT:
            ROS_DEBUG("LEFT");
            twist_msg->twist.linear.y = linear_pos_cmd_;
            publish_twist = true;
            break;
        case KEYCODE_RIGHT:
            ROS_DEBUG("RIGHT");
            twist_msg->twist.linear.y = -linear_pos_cmd_;
            publish_twist = true;
            break;
        case KEYCODE_UP:
            ROS_DEBUG("UP");
            twist_msg->twist.linear.x = linear_pos_cmd_;
            publish_twist = true;
            break;
        case KEYCODE_DOWN:
            ROS_DEBUG("DOWN");
            twist_msg->twist.linear.x = -linear_pos_cmd_;
            publish_twist = true;
            break;
        case KEYCODE_PERIOD:
            ROS_DEBUG("PERIOD");
            twist_msg->twist.linear.z = -linear_pos_cmd_;
            publish_twist = true;
            break;
        case KEYCODE_SEMICOLON:
            ROS_DEBUG("SEMICOLON");
            twist_msg->twist.linear.z = linear_pos_cmd_;
            publish_twist = true;
            break;
        case KEYCODE_E:
            ROS_DEBUG("E");
            planning_frame_ = ee_frame_name_;
            break;
        case KEYCODE_W:
            ROS_DEBUG("W");
            planning_frame_ = robot_link_command_frame_;
            break;
        case KEYCODE_1:
            ROS_DEBUG("1");
            joint_msg->joint_names.push_back("joint1");
            joint_msg->velocities.push_back(joint_vel_cmd_);
            publish_joint = true;
            break;
        case KEYCODE_2:
            ROS_DEBUG("2");
            joint_msg->joint_names.push_back("joint2");
            joint_msg->velocities.push_back(joint_vel_cmd_);
            publish_joint = true;
            break;
        case KEYCODE_3:
            ROS_DEBUG("3");
            joint_msg->joint_names.push_back("joint3");
            joint_msg->velocities.push_back(joint_vel_cmd_);
            publish_joint = true;
            break;
        case KEYCODE_4:
            ROS_DEBUG("4");
            joint_msg->joint_names.push_back("joint4");
            joint_msg->velocities.push_back(joint_vel_cmd_);
            publish_joint = true;
            break;
        case KEYCODE_5:
            ROS_DEBUG("5");
            joint_msg->joint_names.push_back("joint5");
            joint_msg->velocities.push_back(joint_vel_cmd_);
            publish_joint = true;
            break;
        case KEYCODE_6:
            ROS_DEBUG("6");
            joint_msg->joint_names.push_back("joint6");
            joint_msg->velocities.push_back(joint_vel_cmd_);
            publish_joint = true;
            break;
        case KEYCODE_7:
            ROS_DEBUG("7");
            joint_msg->joint_names.push_back("joint7");
            joint_msg->velocities.push_back(joint_vel_cmd_);
            publish_joint = true;
            break;
        case KEYCODE_R:
            ROS_DEBUG("R");
            joint_vel_cmd_ *= -1;
            break;
        case KEYCODE_Q:
            ROS_DEBUG("quit");
            return;
        }
        
        // If a key requiring a publish was pressed, publish the message now
        if (publish_twist)
        {
            twist_msg->header.stamp = ros::Time::now();
            twist_msg->header.frame_id = planning_frame_;
            twist_pub_.publish(std::move(twist_msg));
            publish_twist = false;
        }
        else if (publish_joint)
        {
            joint_msg->header.stamp = ros::Time::now();
            joint_msg->header.frame_id = "joint";
            joint_pub_.publish(std::move(joint_msg));
            publish_joint = false;
        }
    }
}

void exit_sig_handler(int sig)
{
  (void)sig;
  keyboard_reader_.shutdown();
  ros::shutdown();
  exit(-1);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "KeyboardToServeNode");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle nh("~");

    KeyboardServoPub keyboard_servo_pub(nh);
    signal(SIGINT, exit_sig_handler);
    keyboard_servo_pub.keyLoop();
    keyboard_reader_.shutdown();
    
    ros::shutdown();

    ROS_INFO("xarm_moveit_servo_keyboard_node over");

    return 0;
}
