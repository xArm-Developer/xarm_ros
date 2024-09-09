/* Copyright 2020 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jason Peng <jason@ufactory.cc>
 ============================================================================*/

#include "xarm_controller/xarm_hw.h"

int main(int argc, char**argv)
{
  ros::init(argc, argv, "xarm_controller");
  ros::NodeHandle nh;

  ros::Duration(1.0).sleep();

  xarm_control::XArmHW xarm_hw;
  if(!xarm_hw.init(nh, nh)) exit(-1);

  controller_manager::ControllerManager cm(&xarm_hw, nh);

  ros::AsyncSpinner spinner(4);
  spinner.start();

  // IMPORTANT: DO NOT REMOVE THIS DELAY !!!
  /* Wait for correct initial position to be updated to ros_controller */
  ros::Duration(2.0).sleep();

  int update_rate = 100;
  ros::Time ts1 = ros::Time::now();
  ros::Time ts2 = ros::Time::now();
  ros::Time ts3 = ros::Time::now();
  bool is_first = true;
  ros::Duration elapsed = ros::Duration(1.0 / update_rate);

  long int cnts = 0;
  ros::Duration total_duration = ros::Duration(0);
  ros::Duration max_duration = ros::Duration(0);

  bool last_need_reset = false;
  bool curr_need_reset;

  while (ros::ok())
  {
    cnts += 1;
    ros::Time start_time = ros::Time::now();
    
    std::chrono::system_clock::time_point begin = std::chrono::system_clock::now();
    elapsed = is_first ? elapsed : (ros::Time::now() - ts1);
    ts1 = ros::Time::now();
    xarm_hw.read(ts1, elapsed);
    elapsed = is_first ? elapsed : (ros::Time::now() - ts2);
    ts2 = ros::Time::now();
    curr_need_reset = xarm_hw.need_reset();
    cm.update(ts2, elapsed, last_need_reset && !curr_need_reset); // reset_controllers=true: preempt and cancel current goal
    last_need_reset = curr_need_reset;
    elapsed = is_first ? elapsed : (ros::Time::now() - ts3);
    ts3 = ros::Time::now();
    xarm_hw.write(ts3, elapsed);
    ros::Time end_time = ros::Time::now();
    elapsed = end_time - start_time;
    total_duration += elapsed;
    if (elapsed > max_duration) {
      max_duration = elapsed;
    }
    // if (cnts % 6000 == 0) {
    //   ROS_INFO("[CTRL] cnt: %ld, max: %f, mean: %f", cnts, max_duration.toSec(), total_duration.toSec() / cnts);
    // }

    std::chrono::system_clock::time_point end = std::chrono::system_clock::now();
    std::this_thread::sleep_for(
      std::max(
        std::chrono::nanoseconds(0),
        std::chrono::nanoseconds(1000000000 / update_rate) -
        std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin)));
    if (is_first) is_first = false;
  }

  spinner.stop();
  return 0;
}