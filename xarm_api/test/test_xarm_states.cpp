/* Copyright 2021 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jason <jason@ufactory.cc>
           Vinman <vinman@ufactory.cc> 
 ============================================================================*/
#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include "xarm_api/xarm_msgs.h"


void joint_states_callback(const sensor_msgs::JointState::ConstPtr& states)
{
  std::string pos_str = "[ ";
  for (int i = 0; i < states->position.size(); i++) { 
    pos_str += std::to_string(states->position[i]); 
    pos_str += " ";
  }
  pos_str += "]";
  printf("[JOINT_STATE]\n");
  printf("    positon: %s\n", pos_str.c_str());
}

void xarm_states_callback(const xarm_msgs::RobotMsg::ConstPtr& states)
{
  printf("[XARM_STATE]\n");
  printf("    state: %d, error: %d\n", states->state, states->err);
}

void xarm_cgpio_states_callback(const xarm_msgs::CIOState::ConstPtr& states)
{
  printf("[XARM_CGPIO_STATE]\n");
  printf("    [INPUT]\n");
  printf("        [CI] ");
  for (int i = 0; i < 8; ++i) { printf("CI%d=%d, ", i, (states->input_digitals[1] >> i) & 0x0001); }
  printf("\n");
  // printf("        [DI] ");
  // for (int i = 8; i < 16; ++i) { printf("DI%d=%d, ", i-8, (states->input_digitals[1] >> i) & 0x0001); }
  // printf("\n");
  printf("        [AI] AI0=%f, AI1=%f\n", states->input_analogs[0], states->input_analogs[1]);

  printf("    [OUTPUT]\n");
  printf("        [CO] ");
  for (int i = 0; i < 8; ++i) { printf("CO%d=%d, ", i, (states->output_digitals[1] >> i) & 0x0001); }
  printf("\n");
  // printf("        [DO] ");
  // for (int i = 8; i < 16; ++i) { printf("DO%d=%d, ", i-8, (states->output_digitals[1] >> i) & 0x0001); }
  // printf("\n");
  printf("        [AO] AO0=%f, AO1=%f\n", states->output_analogs[0], states->output_analogs[1]);


  printf("    [CONF]\n");
  printf("        [CI] ");
    for (int i = 0; i < 8; ++i) { printf("CI%d=%d, ", i, states->input_conf[i]); }
  printf("\n");
  // printf("        [DI] ");
  //   for (int i = 8; i < 16; ++i) { printf("DI%d=%d, ", i-8, states->input_conf[i]); }
  // printf("\n");
  printf("        [CO] ");
    for (int i = 0; i < 8; ++i) { printf("CO%d=%d, ", i, states->output_conf[i]); }
  printf("\n");
  // printf("        [DO] ");
  //   for (int i = 8; i < 16; ++i) { printf("DO%d=%d, ", i-8, states->output_conf[i]); }
  // printf("\n");

  printf("======================================\n");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_xarm_states");
  ros::NodeHandle nh("xarm");

  ROS_INFO("namespace: %s", nh.getNamespace().c_str());
  ROS_INFO("test_xarm_states start");

  ros::Subscriber joint_state_sub = nh.subscribe("joint_states", 100, joint_states_callback);
  ros::Subscriber xarm_state_sub = nh.subscribe("xarm_states", 100, xarm_states_callback);
  ros::Subscriber cgpio_sub = nh.subscribe("controller_gpio_states", 100, xarm_cgpio_states_callback);
  ros::spin();

  ROS_INFO("test_xarm_states over");
  return 0;
}
