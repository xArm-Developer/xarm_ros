/* Copyright 2021 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jason Peng <jason@ufactory.cc>
 ============================================================================*/
#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <std_msgs/Bool.h>
#include <xarm_api/xarm_ros_client.h>
#include <vector>

// Caution: motion services from "xarm_api" did not verify the trajectory (collision and singularity check) in advance，
// Thus motion may fail during execution.

static const std::string target_frame = "/link_base";
static const std::string source_frame = "/object_1";

int main(int argc, char** argv)
{	
	ros::init(argc, argv, "xarm_simple_planner_client");
	ros::NodeHandle nh;

	// should run this node under "xarm" namespace !
	// *********** configuration:
	xarm_api::XArmROSClient xarm_c;
	xarm_c.init(nh);

	xarm_c.motionEnable(1);
	xarm_c.gripperConfig(5000);
	const std::vector<float> gripper_tcp_offset = {0,0,175,0,0,0};
	xarm_c.setTCPOffset(gripper_tcp_offset);

	xarm_c.setMode(0);
	xarm_c.setState(0);

	// *********** First, open the xArm gripper and go to start pose:
	xarm_c.gripperMove(850);
	// here, give the start pose:
	std::vector<float> prep_pos = {-258, 405, 145, M_PI, 0, M_PI};
	xarm_c.moveLine(prep_pos, 160, 1000);

	// *********** Second, wait for the recognition result from find_object_3d:
	tf::TransformListener listener;
	tf::StampedTransform transform;

	try {
		// timeout here is 15 seconds. Default object frame name: "object_1"
	    listener.waitForTransform(target_frame, source_frame, ros::Time(0), ros::Duration(15.0) );
	    listener.lookupTransform( target_frame, source_frame, ros::Time(0), transform);
	} catch (tf::TransformException ex) {
	    ROS_ERROR("%s",ex.what());
	    exit(-1);
	}

	float x = transform.getOrigin().x()*1000.0;
	float y = transform.getOrigin().y()*1000.0;
	float z = transform.getOrigin().z()*1000.0;

	tf::Quaternion q = transform.getRotation();
	float qx = q.getX();
	float qy = q.getY();
	float qz = q.getZ();
	float qw = q.getW();
	ROS_WARN("listener: x: %lf, y: %lf, z: %lf, q: [%lf, %lf, %lf, %lf]", x,y,z, qw, qx, qy, qz);

	// *********** Third, go to 40mm above target:
	std::vector<float> tar_pos1 = {x, y, z+40, M_PI, 0, M_PI};
	xarm_c.moveLine(tar_pos1, 160, 1000);

	// *********** Fourth, go to -10mm below recognized target, for grasp:
	tar_pos1[2] = z-10;
	xarm_c.moveLine(tar_pos1, 160, 1000);

	// *********** Fifth step, grasp:
	ros::Duration(1.0).sleep();
	xarm_c.gripperMove(120);

	// *********** Last step, go back to start pose:
	ros::Duration(1.0).sleep();
	xarm_c.moveLine(prep_pos, 160, 1000);

	return 0;

}

