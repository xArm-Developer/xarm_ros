#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <xarm_gripper/MoveAction.h>


int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_gripper");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<xarm_gripper::MoveAction> ac("xarm/gripper_move", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  xarm_gripper::MoveGoal goal;
  if(argc<3)
  {
    ROS_WARN("argment 1: target_pulse, argument 2: pulse_speed, Not fully specified.");
    ROS_INFO("Use default target_pulse: 450, pulse_speed: 1500");
    goal.target_pulse = 450;
    goal.pulse_speed = 1500;
  }
  else
  {
    goal.target_pulse = std::atof(argv[1]);
    goal.pulse_speed = std::atof(argv[2]);
  }
  ac.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(5.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
  {
    ROS_INFO("Action did not finish before the time out.");
    ac.cancelAllGoals();
  }
  //exit
  return 0;
}