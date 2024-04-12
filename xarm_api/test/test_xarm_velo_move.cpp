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
  ros::init(argc, argv, "xarm_move_test");
  ros::NodeHandle nh("xarm");

  xarm_api::XArmROSClient client;
  client.init(nh);

  int ret;

  client.motionEnable(1);
  client.setMode(4);
  client.setState(0);

  std::vector<float> jnt_v = { 1, 0, 0, 0, 0, 0, 0 };
  ret = client.veloMoveJoint(jnt_v);
  ROS_INFO("velo_move_joint: %d", ret);
  ros::Duration(2).sleep();
  jnt_v[0] = -1;
  ret = client.veloMoveJoint(jnt_v);
  ROS_INFO("velo_move_joint: %d", ret);
  ros::Duration(2).sleep();
  // stop
  jnt_v[0] = 0;
  ret = client.veloMoveJoint(jnt_v);
  ROS_INFO("velo_move_joint: %d", ret);

  std::vector<float> line_v = { 100, 0, 0, 0, 0, 0};
  client.setMode(5);
  client.setState(0);
  ret = client.veloMoveLine(line_v);
  ROS_INFO("velo_move_line: %d", ret);
  ros::Duration(2).sleep();
  line_v[0] = -100;
  ret = client.veloMoveLine(line_v);
  ROS_INFO("velo_move_line: %d", ret);
  ros::Duration(2).sleep();
  // stop
  line_v[0] = 0;
  ret = client.veloMoveLine(line_v);
  ROS_INFO("velo_move_line: %d", ret);

  return 0;
}