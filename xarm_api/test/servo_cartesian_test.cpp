/* Copyright 2020 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: jason <jason@ufactory.cc>
 ============================================================================*/
#include "ros/ros.h"
#include <xarm_api/xarm_driver.h>

int go_home_test(xarm_msgs::Move &srv, ros::ServiceClient &client)
{
  srv.request.mvvelo = 20.0 * M_PI / 180.0;
  srv.request.mvacc = 1000;
  srv.request.mvtime = 0;
  if(client.call(srv))
  {
    ROS_INFO("%s\n", srv.response.message.c_str());
  }
  else
  {
    ROS_ERROR("Failed to call service go home");
    return 1;
  }
  return 0;
}

int servo_cart_test(std::vector<float> &start_pos, xarm_msgs::Move &srv, ros::ServiceClient &client)
{
  float inc = 0.3; 

  srv.request.mvvelo = 0;
  srv.request.mvacc = 0;
  srv.request.mvtime = 0;

  srv.request.pose = start_pos;

  for (int i = 0; i < 500; i++) 
  {   
    srv.request.pose[0] += inc;

    if(client.call(srv))
    {
      ROS_INFO("%s\n", srv.response.message.c_str());
    }
    else
    {
      ROS_ERROR("Failed to call service move_servo_cart");
      return 1;
    }
    ros::Duration(0.01).sleep(); //10ms

  }
  return 0;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "xarm_move_test");
  ros::NodeHandle nh;
  nh.setParam("/xarm/wait_for_finish", true); // return after motion service finish
  ros::ServiceClient motion_ctrl_client_ = nh.serviceClient<xarm_msgs::SetAxis>("/xarm/motion_ctrl");
  ros::ServiceClient set_mode_client_ = nh.serviceClient<xarm_msgs::SetInt16>("/xarm/set_mode");
  ros::ServiceClient set_state_client_ = nh.serviceClient<xarm_msgs::SetInt16>("/xarm/set_state");
  ros::ServiceClient go_home_client_ = nh.serviceClient<xarm_msgs::Move>("/xarm/go_home");
  ros::ServiceClient servo_cart_client_ = nh.serviceClient<xarm_msgs::Move>("/xarm/move_servo_cart");
  ros::ServiceClient get_position_client_ = nh.serviceClient<xarm_msgs::GetFloat32List>("/xarm/get_position_rpy");

  xarm_msgs::SetAxis set_axis_srv_;
  xarm_msgs::SetInt16 set_int16_srv_;
  xarm_msgs::Move move_srv_;
  xarm_msgs::GetFloat32List get_position_srv_;
  

  // STEP 1: Motion Enable
  set_axis_srv_.request.id = 8;
  set_axis_srv_.request.data = 1;
  
  if(motion_ctrl_client_.call(set_axis_srv_))
  {
    ROS_INFO("%s\n", set_axis_srv_.response.message.c_str());
  }
  else
  {
    ROS_ERROR("Failed to call service motion_ctrl");
    return 1;
  }

  // STEP 2: Set Mode to 0
  set_int16_srv_.request.data = 0;
  if(set_mode_client_.call(set_int16_srv_))
  {
    ROS_INFO("%s\n", set_int16_srv_.response.message.c_str());
  }
  else
  {
    ROS_ERROR("Failed to call service set_mode");
    return 1;
  }  

  // STEP 3: Set State to 0
  set_int16_srv_.request.data = 0;
  if(set_state_client_.call(set_int16_srv_))
  {
    ROS_INFO("%s\n", set_int16_srv_.response.message.c_str());
  }
  else
  {
    ROS_ERROR("Failed to call service set_state");
    return 1;
  }

  // STEP 4: Go to Home Position
  if(go_home_test(move_srv_, go_home_client_) == 1) 
    return 1;


  // STEP 5: Set Mode to 1
  set_int16_srv_.request.data = 1;
  if(set_mode_client_.call(set_int16_srv_))
  {
    ROS_INFO("%s\n", set_int16_srv_.response.message.c_str());
  }
  else
  {
    ROS_ERROR("Failed to call service set_mode");
    return 1;
  }

  // STEP 6: Set state to 0
  set_int16_srv_.request.data = 0;
  if(set_state_client_.call(set_int16_srv_))
  {
    ROS_INFO("%s\n", set_int16_srv_.response.message.c_str());
  }
  else
  {
    ROS_ERROR("Failed to call service set_state");
    return 1;
  }

  // STEP 7: Get current position
  get_position_client_.call(get_position_srv_);
  std::vector<float> curr_pos(get_position_srv_.response.datas);

  // STEP 8: Call SERVO_CARTESIAN service in a loop
  return servo_cart_test(curr_pos, move_srv_, servo_cart_client_);

}