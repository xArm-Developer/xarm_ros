/* Copyright 2020 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jason Peng <jason@ufactory.cc>
 ============================================================================*/
#include <ros/ros.h>
#include <combined_robot_hw/combined_robot_hw.h>
#include "xarm_controller/xarm_hw.h"

namespace xarm_control
{

class XArmCombinedHW : public combined_robot_hw::CombinedRobotHW
{
public:
  XArmCombinedHW(){};
  ~XArmCombinedHW(){};
  
  bool need_reset();
  bool wait_fbk_start(ros::Duration timeout);
};

}