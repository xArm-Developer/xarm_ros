/* Copyright 2018 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jason Peng <jason@ufactory.cc>
 ============================================================================*/
#include <signal.h>
#include "xarm_api/xarm_driver.h"


void exit_sig_handler(int signum)
{
  fprintf(stderr, "[xarm_driver] Ctrl-C caught, exit process...\n");
  exit(-1);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "xarm_driver_node");

  // with namespace (ns) specified in the calling launch file (xarm by default)
  ros::NodeHandle node;
  ROS_INFO("namespace: %s", node.getNamespace().c_str());

  std::string robot_ip = "192.168.1.121";
  if (!node.hasParam("xarm_robot_ip"))
  {
    ROS_ERROR("No param named 'xarm_robot_ip'");
    ROS_ERROR("Use default robot ip = 192.168.1.121");
  }
  else
  {
    node.getParam("xarm_robot_ip", robot_ip);
  }

  ROS_INFO("xarm_driver_node start");
  xarm_api::XArmDriver xarm_driver;
  xarm_driver.init(node, robot_ip);

  signal(SIGINT, exit_sig_handler);
  ros::waitForShutdown();

  ROS_INFO("xarm_driver_node over");

  return 0;
}
