/* Copyright 2018 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jason Peng <jason@ufactory.cc>
 ============================================================================*/
#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <std_msgs/Bool.h>
#include <xarm_planner/pose_plan.h>
#include <xarm_planner/joint_plan.h>
#include <xarm_planner/exec_plan.h>
#include <xarm_planner/single_straight_plan.h>
#include <xarm_msgs/GripperMove.h>
#include <xarm_msgs/GripperConfig.h>
#include <stdlib.h>
#include <vector>

static const std::string target_frame = "/link_base";
static const std::string source_frame = "/object_17";

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
	ros::init(argc, argv, "xarm_simple_planner_client");
	ros::NodeHandle nh;

	// wait for xarm motion service becomming available
	// ros::service::waitForService("xarm/move_line");

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
	xarm_planner::single_straight_plan srv22;

	ros::ServiceClient gripper_config_client = nh.serviceClient<xarm_msgs::GripperConfig>("xarm/gripper_config");
	xarm_msgs::GripperConfig gripper_config_srv; 
	gripper_config_srv.request.pulse_vel = 3000.0;
	gripper_config_client.call(gripper_config_srv);

	ros::ServiceClient gripper_move_client = nh.serviceClient<xarm_msgs::GripperMove>("xarm/gripper_move");
	xarm_msgs::GripperMove gripper_srv; 
	gripper_srv.request.pulse_pos = 850.0;
	gripper_move_client.call(gripper_srv);


	tf::TransformListener listener;
	tf::StampedTransform transform;

	try {
	    listener.waitForTransform(target_frame, source_frame, ros::Time(0), ros::Duration(15.0) );
	    listener.lookupTransform( target_frame, source_frame, ros::Time(0), transform);
	} catch (tf::TransformException ex) {
	    ROS_ERROR("%s",ex.what());
	    exit(-1);
	}

	double x = transform.getOrigin().x();
	double y = transform.getOrigin().y();
	double z = transform.getOrigin().z();

	tf::Quaternion q = transform.getRotation();
	double qx = q.getX();
	double qy = q.getY();
	double qz = q.getZ();
	double qw = q.getW();
	ROS_WARN("listener: x: %lf, y: %lf, z: %lf, q: [%lf, %lf, %lf, %lf]", x,y,z, qw, qx, qy, qz);

	// double slp_t = 0.5;
	geometry_msgs::Pose target1;
	target1.position.x = x;
	target1.position.y = y;
	target1.position.z = z+0.04;

	target1.orientation.x = 0;
	target1.orientation.y = 1;
	target1.orientation.z = 0;
	target1.orientation.w = 0;

	srv22.request.target = target1;
	if(request_plan(client22, srv22))
	{
		ROS_INFO("Plan SUCCESS! Executing... ");
		srv_exec.request.exec = true;
		request_exec(client_exec, srv_exec);
	}
	ros::Duration(1.0).sleep();

	target1.position.z = z-0.01;
	srv22.request.target = target1;
	if(request_plan(client22, srv22))
	{
		ROS_INFO("Plan SUCCESS! Executing... ");
		srv_exec.request.exec = true;
		request_exec(client_exec, srv_exec);
	}

	gripper_srv.request.pulse_pos = 120.0;
	gripper_move_client.call(gripper_srv);

	ros::Duration(1.0).sleep();

	target1.position.x = -0.21;
	target1.position.y = 0.3;
	target1.position.z = 0.23;

	srv22.request.target = target1;
	if(request_plan(client22, srv22))
	{
		ROS_INFO("Plan SUCCESS! Executing... ");
		srv_exec.request.exec = true;
		request_exec(client_exec, srv_exec);
	}

	return 0;

}

