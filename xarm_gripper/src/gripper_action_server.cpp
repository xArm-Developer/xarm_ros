#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <xarm_gripper/MoveAction.h>
#include <xarm_api/xarm_ros_client.h>

// Please run "export ROS_NAMESPACE=/xarm" first

namespace xarm_control
{
	class GripperAction
	{
	protected:

	  ros::NodeHandle nh_;
	  actionlib::SimpleActionServer<xarm_gripper::MoveAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
	  std::string action_name_;
	  // create messages that are used to published feedback/result
	  xarm_gripper::MoveFeedback feedback_;
	  xarm_gripper::MoveResult result_;

	  xarm_api::XArmROSClient xarm;

	public:

	  GripperAction(std::string name) :
	    as_(nh_, name, boost::bind(&GripperAction::executeCB, this, _1), false),
	    action_name_(name)
	  {
	  	xarm.init(nh_);
	    as_.start();
	  }

	  ~GripperAction(void)
	  {
	  }

	  void executeCB(const xarm_gripper::MoveGoalConstPtr &goal);

	};

	void GripperAction::executeCB(const xarm_gripper::MoveGoalConstPtr &goal)
	  {
	    ros::Rate r(10);
	    int ret = 0;
	    feedback_.current_pulse = 0;

	    // publish info to the console for the user
	    ROS_INFO("Executing, creating MoveAction ");

	    // start executing the action
	    if(xarm.gripperConfig(goal->pulse_speed))
	   	{
	   		ROS_INFO("%s: Aborted, not ready", action_name_.c_str());
	        as_.setAborted();
	        return;
	   	}

	   	if(xarm.gripperMove(goal->target_pulse))
	   	{
	   		ROS_INFO("%s: Aborted, not ready", action_name_.c_str());
	        as_.setAborted();
	        return;
	   	}

	   	float fdb_pulse = 0; int fdb_err = 0;
	   	ret = xarm.getGripperState(&fdb_pulse, &fdb_err);

	   	while(!ret && fabs(fdb_pulse-goal->target_pulse)>10)
	   	{
	   		
	   		feedback_.current_pulse = fdb_pulse;
		    as_.publishFeedback(feedback_);

	   	    if (as_.isPreemptRequested() || !ros::ok())
		    {
		        ROS_INFO("%s: Preempted", action_name_.c_str());
		        // set the action state to preempted
		        xarm.gripperMove(fdb_pulse);
		        as_.setPreempted();
		        return;
		    }

		    r.sleep();

		    ret = xarm.getGripperState(&fdb_pulse, &fdb_err);
		    
	   	}

	    if(!ret)
	    {
	      result_.success = true;
	      result_.err_code = fdb_err;
	      ROS_INFO("%s: Succeeded, err_code: %d", action_name_.c_str(), fdb_err);
	      as_.setSucceeded(result_);
	    }

	    else
	    {
	    	result_.success = false;
	      	result_.err_code = fdb_err;
	        ROS_INFO("%s: Failed, ret = %d, err_code: %d", action_name_.c_str(), ret, fdb_err);
	    	as_.setAborted();
	    }
	  }

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "xarm_gripper_node");
    
    xarm_control::GripperAction ga("gripper_move");

  	ros::spin();

	return 0;
}