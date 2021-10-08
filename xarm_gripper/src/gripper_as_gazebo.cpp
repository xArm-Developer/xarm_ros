/**
 * Action server for gripper in gazebo
 * Uses MoveIt to control gripper opening and closing
 */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <actionlib/server/simple_action_server.h>
#include <xarm_gripper/MoveAction.h>

namespace xarm_control
{
	class GripperGazeboAction
	{
	protected:
	ros::NodeHandle nh_;

	// moveit
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	moveit::planning_interface::MoveGroupInterface gripper;
	moveit::planning_interface::MoveGroupInterface::Plan my_xarm_plan;

	// action server
	actionlib::SimpleActionServer<xarm_gripper::MoveAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
	std::string action_name_ = "gripper_action_gazebo";

	// create messages that are used to published feedback/result
	xarm_gripper::MoveFeedback feedback_;
	xarm_gripper::MoveResult result_;

	public:

	GripperGazeboAction(std::string name) :
		as_(nh_, "/xarm/gripper_move", boost::bind(&GripperGazeboAction::executeCB, this, _1), false),
		gripper("xarm_gripper")
	{
		as_.start();
		std::cout << ">> Gripper server for gazebo started " << std::endl;

	}

	~GripperGazeboAction(void)
	{
	}

	void executeCB(const xarm_gripper::MoveGoalConstPtr &goal);

	};

	void GripperGazeboAction::executeCB(const xarm_gripper::MoveGoalConstPtr &goal)
	{
		ros::Rate r(10);
		int ret = 0;
		// feedback_.message = "something";

		// publish info to the console for the user
		ROS_INFO("Executing, creating MoveAction ");

		// start executing the action
		// target_pulse = 0 : close => joint = 49 deg (moveit)
		// target_pulse = 850 : open => joint = 0 deg (moveit)
		
		// TODO: include pulse_speed
		gripper.setMaxVelocityScalingFactor(0.5);

		if(goal->target_pulse == 0)
		{
			// feedback_.message = "Closing gripper";
			// as_.publishFeedback(feedback_);
			std::cout << "Close gripper" << std::endl;
			gripper.setNamedTarget("close");
		}
		else if (goal->target_pulse == 850)
		{
			// feedback_.message = "Opening gripper";
			// as_.publishFeedback(feedback_);
			std::cout << "Open gripper" << std::endl;
			gripper.setNamedTarget("open");
		}
		else 
		{
			float joint_value = (49 - (goal->target_pulse / 850 * 49 )) / 180 * 3.142;
			auto joint_values = gripper.getCurrentJointValues();
			std::cout << "Joint value: " << joint_value << std::endl;
			for (int i = 0; i < 6; i ++)
			{
				// std::cout << joint_values[i] << " ";
				joint_values[i] = joint_value;
			}
			gripper.setJointValueTarget(joint_values);
			
		}

		bool success = (gripper.plan(my_xarm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO("Planning %s", success ? "" : "FAILED");

		bool move_success = (gripper.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
		ROS_INFO("Motion %s", success ? "success" : "FAILED");

		// check if premmpted
   	    if (as_.isPreemptRequested() || !ros::ok())
	    {
			result_.success = false;
	        ROS_INFO("%s: Preempted", action_name_.c_str());
	        // set the action state to preempted
	        as_.setPreempted(result_);
	        return;
	    }
		r.sleep();

		// report success
		if (move_success)
		{
			result_.success = true;
			ROS_INFO("%s: Succeeded", action_name_.c_str());
			as_.setSucceeded(result_);
		}
		else
		{	
			result_.success = false;
			as_.setAborted(result_);
			return;
		}
		
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "xarm_gripper_node");
	// std::cout << "here" << std::endl;

	xarm_control::GripperGazeboAction gripper_gazebo_action("gripper_move");

	ros::spin();

	return 0;
}