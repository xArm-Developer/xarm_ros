/* Copyright 2018 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jason Peng <jason@ufactory.cc>
 ============================================================================*/
#include "ros/ros.h"
#include <std_msgs/Bool.h>
#include <xarm_msgs/SetInt16.h>
#include <xarm_msgs/SetAxis.h>
#include <xarm_msgs/SetString.h>
#include <xarm_msgs/PlayTraj.h>
#include <stdlib.h>
#include <vector>

int main(int argc, char** argv)
{	
	
	ros::init(argc, argv, "xarm_traj_plays");
	ros::NodeHandle nh;
	ros::ServiceClient play_client = nh.serviceClient<xarm_msgs::PlayTraj>("xarm/play_traj");
	ros::ServiceClient client_state = nh.serviceClient<xarm_msgs::SetInt16>("xarm/set_state");
	ros::ServiceClient client_mode = nh.serviceClient<xarm_msgs::SetInt16>("xarm/set_mode");
	ros::ServiceClient client_motion = nh.serviceClient<xarm_msgs::SetAxis>("xarm/motion_ctrl");

	xarm_msgs::PlayTraj play_srv;
	xarm_msgs::SetInt16 srv_mode;
	xarm_msgs::SetInt16 srv_state;
	xarm_msgs::SetAxis srv_enable;

	ros::service::waitForService("xarm/motion_ctrl");
	ros::service::waitForService("xarm/set_state");
	ros::service::waitForService("xarm/play_traj");
	ROS_WARN("outof wait");

	// motion enable:
	srv_enable.request.id = 8;
	srv_enable.request.data = 1;
	client_motion.call(srv_enable);
	ros::Duration(1.0).sleep();


	while(ros::ok())
	{
		srv_state.request.data = 0;
		srv_mode.request.data = 0;
		client_mode.call(srv_mode);
		client_state.call(srv_state);
		
		ROS_INFO("Setting mode state... ");

		ros::Duration(1.0).sleep();

		play_srv.request.traj_file = "my_recording.traj";
		play_srv.request.repeat_times = 1;
		play_srv.request.speed_factor = 1;
		if(!play_client.call(play_srv))
		{
			ROS_ERROR("Failed to call service play_traj");
			break;
		}
		
		ROS_INFO("exec over");
		
	}

	ros::shutdown();
	return 0;

}

