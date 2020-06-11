/* Copyright 2018 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jason Peng <jason@ufactory.cc>
 ============================================================================*/
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <std_msgs/Bool.h>
#include <xarm_planner/joint_plan.h>
#include <xarm_planner/exec_plan.h>

#define SPINNER_THREAD_NUM 2

/* Used for Cartesian path computation, please modify as needed: */
const double jump_threshold = 0.0;
const double eef_step = 0.005;
const double maxV_scale_factor = 0.3; // check!!


namespace rvt = rviz_visual_tools;

class XArmGripperPlanner
{
  public:
    XArmGripperPlanner(const std::string plan_group_name):spinner(SPINNER_THREAD_NUM), group(plan_group_name){init();};
    XArmGripperPlanner():spinner(SPINNER_THREAD_NUM),group(PLANNING_GROUP){init();};
    ~XArmGripperPlanner(){ delete visual_tools;};
    void start();
    void stop();

    static std::string PLANNING_GROUP; // declaration of static class member

  private:
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    std::vector<std::string> joint_names;
    moveit::planning_interface::MoveGroupInterface group;
    moveit::planning_interface::MoveGroupInterface::Plan my_xarm_plan;
    moveit_visual_tools::MoveItVisualTools *visual_tools;

    ros::Publisher display_path;
    // ros::ServiceServer plan_pose_srv;
    ros::ServiceServer plan_joint_srv;
    // ros::ServiceServer sing_cart_srv;
    ros::Subscriber exec_plan_sub; /* non-blocking*/
    ros::ServiceServer exec_plan_srv; /* blocking with result feedback */

    void init();
    // bool do_pose_plan(xarm_planner::pose_plan::Request &req, xarm_planner::pose_plan::Response &res);
    bool do_joint_plan(xarm_planner::joint_plan::Request &req, xarm_planner::joint_plan::Response &res);
    // bool do_single_cartesian_plan(xarm_planner::single_straight_plan::Request &req, xarm_planner::single_straight_plan::Response &res);
    bool exec_plan_cb(xarm_planner::exec_plan::Request &req, xarm_planner::exec_plan::Response &res);
    void execute_plan_topic(const std_msgs::Bool::ConstPtr& exec);
};

void XArmGripperPlanner::init()
{
  joint_names = group.getJointNames();
  ROS_INFO("Joint names(%lu): %s", joint_names.size(), joint_names[0].c_str());

  display_path = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true); /*necessary?*/

  ROS_INFO_NAMED("move_group_planner", "Reference frame: %s", group.getPlanningFrame().c_str());

  ROS_INFO_NAMED("move_group_planner", "End effector link: %s", group.getEndEffectorLink().c_str());

  /* Notice: the correct way to specify member function as callbacks */
  plan_joint_srv = node_handle.advertiseService("gripper_joint_plan", &XArmGripperPlanner::do_joint_plan, this);

  exec_plan_sub = node_handle.subscribe("gripper_planner_exec", 10, &XArmGripperPlanner::execute_plan_topic, this);
  exec_plan_srv = node_handle.advertiseService("gripper_exec_plan", &XArmGripperPlanner::exec_plan_cb, this);

}

void XArmGripperPlanner::start()
{
  ROS_INFO("Spinning");
  spinner.start();
}

void XArmGripperPlanner::stop()
{
  spinner.stop();
}


bool XArmGripperPlanner::do_joint_plan(xarm_planner::joint_plan::Request &req, xarm_planner::joint_plan::Response &res)
{
  ROS_INFO("move_group_planner received new plan Request target: %lu, %lf", req.target.size(), req.target[0]);

  std::vector<double> mimic_target(6,req.target[0]);
  group.setJointValueTarget(mimic_target);
  
  bool success = (group.plan(my_xarm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  res.success = success;
  ROS_INFO_NAMED("move_group_planner", "This plan (joint goal) %s", success ? "SUCCEEDED" : "FAILED");
  
  return success;
}

bool XArmGripperPlanner::exec_plan_cb(xarm_planner::exec_plan::Request &req, xarm_planner::exec_plan::Response &res)
{
  if(req.exec)
  {
    ROS_INFO("Received Execution Service Request");
    bool finish_ok = (group.execute(my_xarm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS); /* return after execution finish */
    res.success = finish_ok;
    return finish_ok;
  }

  res.success = false;
  return false;
}

/* execution subscriber call-back function */
void XArmGripperPlanner::execute_plan_topic(const std_msgs::Bool::ConstPtr& exec)
{
  if(exec->data)
  { 
    ROS_INFO("Received Execution Command !!!!!");
    group.asyncExecute(my_xarm_plan); /* return without waiting for finish */
  }
}


std::string XArmGripperPlanner::PLANNING_GROUP; // Definition of static class member

int main(int argc, char** argv)
{
  ros::init(argc, argv, "xarm_gripper_planner");
  ros::NodeHandle nh;
  int jnt_num;
  
  XArmGripperPlanner::PLANNING_GROUP = "xarm_gripper";

  XArmGripperPlanner planner;

  planner.start();

  ROS_INFO("Waiting for \'pose_plan\' or \'joint_plan\' service Request ...");

  /* necessary: because AsyncSpinner is not operating in the same thread */
  ros::waitForShutdown();
  return 0;
}

