/* Copyright 2018 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: waylon <weile.wang@ufactory.cc>
           Jason <jason@ufactory.cc> 
 ============================================================================*/
#include "ros/ros.h"
#include <xarm_api/xarm_driver.h>

int go_home_test(xarm_msgs::Move srv, ros::ServiceClient client)
{
  srv.request.mvvelo = 20.0 / 57.0;
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

int move_lineb_test(xarm_msgs::Move srv, ros::ServiceClient client)
{
  std::vector<float> pose[5] = {  {300, 0, 100, -3.14, 0, 0},
                                  {300, 100, 100, -3.14, 0, 0},
                                  {400, 100, 100, -3.14, 0, 0},
                                  {400, -100, 100, -3.14, 0, 0},
                                  {300, -100, 100, -3.14, 0, 0}};
  srv.request.mvvelo = 100;
  srv.request.mvacc = 1000;
  srv.request.mvtime = 0;
  srv.request.mvradii = 20;

  for(int k=0; k<3; k++)
  {
    for(int i = 0; i < 5; i++) 
    {
      srv.request.pose = pose[i];
      if(client.call(srv))
      {
        ROS_INFO("%s\n", srv.response.message.c_str());
      }
      else
      {
        ROS_ERROR("Failed to call service move_lineb");
        return 1;
      }
    }
  }
  return 0;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "xarm_move_test");
  ros::NodeHandle nh;
  
  nh.setParam("/xarm/wait_for_finish", true); // return after motion service finish
  ros::Publisher sleep_pub_ = nh.advertise<std_msgs::Float32>("/xarm/sleep_sec", 1);
  ros::ServiceClient motion_ctrl_client_ = nh.serviceClient<xarm_msgs::SetAxis>("/xarm/motion_ctrl");
  ros::ServiceClient set_mode_client_ = nh.serviceClient<xarm_msgs::SetInt16>("/xarm/set_mode");
  ros::ServiceClient set_state_client_ = nh.serviceClient<xarm_msgs::SetInt16>("/xarm/set_state");
  ros::ServiceClient go_home_client_ = nh.serviceClient<xarm_msgs::Move>("/xarm/go_home");
  ros::ServiceClient move_lineb_client_ = nh.serviceClient<xarm_msgs::Move>("/xarm/move_lineb");
  ros::ServiceClient move_servoj_client_ = nh.serviceClient<xarm_msgs::Move>("/xarm/move_servoj");

  xarm_msgs::SetAxis set_axis_srv_;
  xarm_msgs::SetInt16 set_int16_srv_;
  xarm_msgs::Move move_srv_;
  
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

  if(go_home_test(move_srv_, go_home_client_) == 1) 
    return 1;

  // MOVE_LINEB need some additional configurations: wait=False, sleep before sending commands
  nh.setParam("/xarm/wait_for_finish", false); // This configuration is CRITICAL for move_lineb!
  std_msgs::Float32 sleep_msg;
  sleep_msg.data = 1.0;
  sleep_pub_.publish(sleep_msg);
  if(move_lineb_test(move_srv_, move_lineb_client_) == 1)
    return 1;
  nh.setParam("/xarm/wait_for_finish", true);

}