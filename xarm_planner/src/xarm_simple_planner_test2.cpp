/* Copyright 2018 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jason Peng <jason@ufactory.cc>
 ============================================================================*/
#include "ros/ros.h"
#include <std_msgs/Bool.h>
#include <xarm_planner/pose_plan.h>
#include <xarm_planner/joint_plan.h>
#include <xarm_planner/exec_plan.h>
#include <xarm_planner/single_straight_plan.h>
#include <stdlib.h>
#include <vector>

bool request_plan(ros::ServiceClient& client, xarm_planner::joint_plan& srv)
{
	if(client.call(srv))
	{
		return srv.response.success;
	}
	else
	{
		ROS_ERROR("Failed to call service joint_plan");
		return false;
	}
}

bool request_plan(ros::ServiceClient& client, xarm_planner::pose_plan& srv)
{
	if(client.call(srv))
	{
		return srv.response.success;
	}
	else
	{
		ROS_ERROR("Failed to call service pose_plan");
		return false;
	}
}

bool request_plan(ros::ServiceClient& client, xarm_planner::single_straight_plan& srv)
{
	if(client.call(srv))
	{
		return srv.response.success;
	}
	else
	{
		ROS_ERROR("Failed to call service single_straight_plan");
		return false;
	}
}

bool request_exec(ros::ServiceClient& client, xarm_planner::exec_plan& srv)
{
	if(client.call(srv))
	{
		return srv.response.success;
	}
	else
	{
		ROS_ERROR("Failed to call service exec_plan");
		return false;
	}
}


int main(int argc, char** argv)
{	
	std::vector<double> tar_joint1 = {-1.0, -0.75, 0.0, -0.5, 0.0, 0.3, 0.0};
	std::vector<double> tar_joint2 = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	std::vector<double> tar_joint3 = {1.0, -0.75, 0.0, -0.5, 0.0, -0.3, 0.0};

	ros::init(argc, argv, "xarm_simple_planner_client");
	ros::NodeHandle nh;
	ros::ServiceClient client = nh.serviceClient<xarm_planner::joint_plan>("xarm_joint_plan");
	ros::ServiceClient client2 = nh.serviceClient<xarm_planner::pose_plan>("xarm_pose_plan");
	ros::ServiceClient client_exec = nh.serviceClient<xarm_planner::exec_plan>("xarm_exec_plan");

	ros::Publisher exec_pub = nh.advertise<std_msgs::Bool>("xarm_planner_exec", 10);
	std_msgs::Bool msg;
	xarm_planner::joint_plan srv;
	xarm_planner::pose_plan srv2;
	xarm_planner::exec_plan srv_exec;



	//********************************************************************
	ros::ServiceClient client22 = nh.serviceClient<xarm_planner::pose_plan>("xarm_straight_plan");
	std::vector<double> tar_joint22 = {-19.9/57.3, 16.9/57.3, 0.8/57.3, 21.5/57.3, -2.9/57.3, 4.6/57.3, -16.3/57.3};
	xarm_planner::single_straight_plan srv22;

	double slp_t = 0.5;
	geometry_msgs::Pose target1;
	target1.position.x = 0.3;
	target1.position.y = -0.1;
	target1.position.z = 0.2;

	target1.orientation.x = 1;
	target1.orientation.y = 0;
	target1.orientation.z = 0;
	target1.orientation.w = 0;

	geometry_msgs::Pose target2;
	target2.position.x = 0.3;
	target2.position.y = 0.1;
	target2.position.z = 0.2;

	target2.orientation.x = 1;
	target2.orientation.y = 0;
	target2.orientation.z = 0;
	target2.orientation.w = 0;

	geometry_msgs::Pose target3;
	target3.position.x = 0.3;
	target3.position.y = 0.1;
	target3.position.z = 0.4;

	target3.orientation.x = 1;
	target3.orientation.y = 0;
	target3.orientation.z = 0;
	target3.orientation.w = 0;

	geometry_msgs::Pose target4;
	target4.position.x = 0.3;
	target4.position.y = -0.1;
	target4.position.z = 0.4;

	target4.orientation.x = 1;
	target4.orientation.y = 0;
	target4.orientation.z = 0;
	target4.orientation.w = 0;

	
	srv.request.target = tar_joint22;
	if(request_plan(client, srv))
	{
		ROS_INFO("Plan SUCCESS! Executing... ");
		// msg.data = true;
		// ros::Duration(1.0).sleep();
		// exec_pub.publish(msg);
		srv_exec.request.exec = true;
		request_exec(client_exec, srv_exec);
	}
	ros::Duration(slp_t).sleep(); // Wait for last execution to finish

	int i=0;
	for(i=0; i<10; i++)
	{
		srv22.request.target = target1;
		if(request_plan(client22, srv22))
		{
			ROS_INFO("Plan SUCCESS! Executing... ");
			// msg.data = true;
			// ros::Duration(1.0).sleep();
			// exec_pub.publish(msg);
			srv_exec.request.exec = true;
			request_exec(client_exec, srv_exec);
		}
		else
			break;

		srv22.request.target = target2;
		if(request_plan(client22, srv22))
		{
			ROS_INFO("Plan SUCCESS! Executing... ");
			// msg.data = true;
			// ros::Duration(1.0).sleep();
			// exec_pub.publish(msg);
			srv_exec.request.exec = true;
			request_exec(client_exec, srv_exec);
		}
		else
			break;
		srv22.request.target = target3;
		if(request_plan(client22, srv22))
		{
			ROS_INFO("Plan SUCCESS! Executing... ");
			// msg.data = true;
			// ros::Duration(1.0).sleep();
			// exec_pub.publish(msg);
			srv_exec.request.exec = true;
			request_exec(client_exec, srv_exec);
		}
		else
			break;
		srv22.request.target = target4;
		if(request_plan(client22, srv22))
		{
			ROS_INFO("Plan SUCCESS! Executing... ");
			// msg.data = true;
			// ros::Duration(1.0).sleep();
			// exec_pub.publish(msg);
			srv_exec.request.exec = true;
			request_exec(client_exec, srv_exec);
		}
		else
			break;

	}

	if(i<10)
		ROS_ERROR("Execution Failed at loop: %d!", i+1);

	// ros::Duration(4.0).sleep(); // Wait for last execution to finish


	//********************************************************************


	// geometry_msgs::Pose target1;
	// target1.position.x = 0.28;
	// target1.position.y = 0.2;
	// target1.position.z = 0.2;

	// target1.orientation.x = 1;
	// target1.orientation.y = 0;
	// target1.orientation.z = 0;
	// target1.orientation.w = 0;


	// srv2.request.target = target1;
	// if(request_plan(client2, srv2))
	// {
	// 	ROS_INFO("Plan SUCCESS! Executing... ");
	// 	// msg.data = true;
	// 	// ros::Duration(1.0).sleep();
	// 	// exec_pub.publish(msg);
	// 	srv_exec.request.exec = true;
	// 	request_exec(client_exec, srv_exec);
	// }

	// // ros::Duration(4.0).sleep(); // Wait for last execution to finish



	// srv.request.target = tar_joint2;
	// if(request_plan(client, srv))
	// {
	// 	ROS_INFO("Plan SUCCESS! Executing... ");
	// 	// msg.data = true;
	// 	// ros::Duration(1.0).sleep();
	// 	// exec_pub.publish(msg);
	// 	srv_exec.request.exec = true;
	// 	request_exec(client_exec, srv_exec);
	// }

	// // ros::Duration(4.0).sleep(); // Wait for last execution to finish

	// srv2.request.target = target1;
	// if(request_plan(client2, srv2))
	// {
	// 	ROS_INFO("Plan SUCCESS! Executing... ");
	// 	// msg.data = true;
	// 	// ros::Duration(1.0).sleep();
	// 	// exec_pub.publish(msg);
	// 	srv_exec.request.exec = true;
	// 	request_exec(client_exec, srv_exec);
	// }

	// // ros::Duration(4.0).sleep(); // Wait for last execution to finish

	// srv.request.target = tar_joint2;
	// if(request_plan(client, srv))
	// {
	// 	ROS_INFO("Plan SUCCESS! Executing... ");
	// 	// msg.data = true;
	// 	// ros::Duration(1.0).sleep();
	// 	// exec_pub.publish(msg);
	// 	srv_exec.request.exec = true;
	// 	request_exec(client_exec, srv_exec);
	// }

	return 0;

}

