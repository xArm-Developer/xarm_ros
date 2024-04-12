/* Copyright 2021 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jason <jason@ufactory.cc>
           Vinman <vinman@ufactory.cc> 
 ============================================================================*/
#include "ros/ros.h"
#include "xarm_api/xarm_ros_client.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "xarm_ros_client");
  ros::NodeHandle nh("xarm");

  ROS_INFO("test_xarm_ros_client start");

  xarm_api::XArmROSClient client;
  client.init(nh);

  int ret;

  client.motionEnable(1);
  client.setMode(0);
  client.setState(0);

  ROS_INFO("test_xarm_ros_client over");

  return 0;
}