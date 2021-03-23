/* Copyright 2021 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jason <jason@ufactory.cc>
           Vinman <vinman@ufactory.cc> 
 ============================================================================*/
#include "ros/ros.h"
#include <xarm_driver.h>

int motion_enable(bool enable, xarm_msgs::SetAxis srv, ros::ServiceClient client)
{
    srv.request.id = 8;
    srv.request.data = enable ? 1 : 0;
    if(client.call(srv))
    {
        ROS_INFO("%s\n", srv.response.message.c_str());
    }
    else
    {
        ROS_ERROR("Failed to call service motion_ctrl");
        return 1;
    }
    return 0;
}

int switch_mode(int mode, xarm_msgs::SetInt16 srv, ros::ServiceClient set_mode_client, ros::ServiceClient set_state_client)
{
    srv.request.data = mode;
    if(set_mode_client.call(srv))
    {
        ROS_INFO("%s\n", srv.response.message.c_str());
    }
    else
    {
        ROS_ERROR("Failed to call service set_mode");
        return 1;
    }  

    srv.request.data = 0;
    if(set_state_client.call(srv))
    {
        ROS_INFO("%s\n", srv.response.message.c_str());
    }
    else
    {
        ROS_ERROR("Failed to call service set_state");
        return 1;
    }
    return 0;
}

int velo_move_test(xarm_msgs::MoveVelo srv, ros::ServiceClient client)
{
    if(client.call(srv))
    {
        ROS_INFO("%s\n", srv.response.message.c_str());
    }
    else
    {
        ROS_ERROR("Failed to call service velo move");
        return 1;
    }
    return 0;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "xarm_move_test");
	ros::NodeHandle nh;
	
    ros::ServiceClient motion_ctrl_client_ = nh.serviceClient<xarm_msgs::SetAxis>("/xarm/motion_ctrl");
	ros::ServiceClient set_mode_client_ = nh.serviceClient<xarm_msgs::SetInt16>("/xarm/set_mode");
	ros::ServiceClient set_state_client_ = nh.serviceClient<xarm_msgs::SetInt16>("/xarm/set_state");
	ros::ServiceClient velo_move_joint_client_ = nh.serviceClient<xarm_msgs::MoveVelo>("/xarm/velo_move_joint");
	ros::ServiceClient velo_move_line_client_ = nh.serviceClient<xarm_msgs::MoveVelo>("/xarm/velo_move_line");

    xarm_msgs::SetAxis set_axis_srv_;
    xarm_msgs::SetInt16 set_int16_srv_;
    xarm_msgs::MoveVelo move_joint_velo_srv_;
    xarm_msgs::MoveVelo move_line_velo_srv_;

    if(motion_enable(true, set_axis_srv_, motion_ctrl_client_) == 1)
        return 1;

    std::vector<float> jnt_v = { 1, 0, 0, 0, 0, 0, 0 };
    std::vector<float> line_v = { 100, 0, 0, 0, 0, 0};
    move_joint_velo_srv_.request.velocities = jnt_v;
    move_joint_velo_srv_.request.jnt_sync = 1;
    move_line_velo_srv_.request.velocities = line_v;
    move_line_velo_srv_.request.coord = 0;

    // velocity move joint
    if(switch_mode(4, set_int16_srv_, set_mode_client_, set_state_client_) == 1)
        return 1;
    if(velo_move_test(move_joint_velo_srv_, velo_move_joint_client_) == 1)
        return 1;
    sleep_milliseconds(3000); // sleep 3s
    move_joint_velo_srv_.request.velocities[0] = -1;
    if(velo_move_test(move_joint_velo_srv_, velo_move_joint_client_) == 1)
        return 1;
    sleep_milliseconds(3100); // sleep 3s
    move_joint_velo_srv_.request.velocities[0] = 0;
    if(velo_move_test(move_joint_velo_srv_, velo_move_joint_client_) == 1)
        return 1;
    
    // velocity move line
    if(switch_mode(5, set_int16_srv_, set_mode_client_, set_state_client_) == 1)
        return 1;
    if(velo_move_test(move_line_velo_srv_, velo_move_line_client_) == 1)
        return 1;
    sleep_milliseconds(3000); // sleep 3s
    move_line_velo_srv_.request.velocities[0] = -100;
    if(velo_move_test(move_line_velo_srv_, velo_move_line_client_) == 1)
        return 1;
    sleep_milliseconds(3000); // sleep 3s
    move_line_velo_srv_.request.velocities[0] = 0;
    if(velo_move_test(move_line_velo_srv_, velo_move_line_client_) == 1)
        return 1;
}