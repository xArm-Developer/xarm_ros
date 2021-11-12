/* Copyright 2018 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jason Peng <jason@ufactory.cc>
 ============================================================================*/

#include "xarm_controller/xarm_hw.h"

#define SERVICE_CALL_FAILED 999
#define SERVICE_IS_PERSISTENT_BUT_INVALID 998
#define VELO_DURATION 1

namespace xarm_control
{
	void XArmHW::_register_joint_limits(ros::NodeHandle &root_nh, std::string joint_name, const ControlMethod ctrl_method)
	{
		joint_limits_interface::JointLimits joint_limits;
		joint_limits_interface::SoftJointLimits soft_joint_limits;
		bool has_limits = false;
		bool has_soft_limits = false;
		
		urdf::JointConstSharedPtr urdf_joint = model_ptr_->getJoint(joint_name);
		if (urdf_joint != NULL) 
		{
			// Get limits from the URDF file.
			if (joint_limits_interface::getJointLimits(urdf_joint, joint_limits))
				has_limits = true;
			if (joint_limits_interface::getSoftJointLimits(urdf_joint, soft_joint_limits))
				has_soft_limits = true;
		}
		// Get limits from the parameter server.
		if (joint_limits_interface::getJointLimits(joint_name, root_nh, joint_limits))
			has_limits = true;

		if (!has_limits)
			return;

		switch (ctrl_method)
		{
		case EFFORT:
			{
				if (has_soft_limits)
					ej_limits_interface_.registerHandle(joint_limits_interface::EffortJointSoftLimitsHandle(ej_interface_.getHandle(joint_name), joint_limits, soft_joint_limits));
				else
					ej_sat_interface_.registerHandle(joint_limits_interface::EffortJointSaturationHandle(ej_interface_.getHandle(joint_name), joint_limits));
			}
			break;
		case VELOCITY:
			{
				if (has_soft_limits)
					vj_limits_interface_.registerHandle(joint_limits_interface::VelocityJointSoftLimitsHandle(vj_interface_.getHandle(joint_name), joint_limits, soft_joint_limits));
				else
					vj_sat_interface_.registerHandle(joint_limits_interface::VelocityJointSaturationHandle(vj_interface_.getHandle(joint_name), joint_limits));
			}
			break;
		case POSITION:
		default:
			{
				if (has_soft_limits)
					pj_limits_interface_.registerHandle(joint_limits_interface::PositionJointSoftLimitsHandle(pj_interface_.getHandle(joint_name), joint_limits, soft_joint_limits));
				else
					pj_sat_interface_.registerHandle(joint_limits_interface::PositionJointSaturationHandle(pj_interface_.getHandle(joint_name), joint_limits));
			}
			break;
		}
		printf("%s, ctrl_method=%d, has_soft_limits=%d, has_velocity_limits=%d, max_velocity=%f, has_position_limits=%d, min_position=%f, max_position=%f\n", 
				joint_name.c_str(), ctrl_method, has_soft_limits, joint_limits.has_velocity_limits, joint_limits.max_velocity, 
				joint_limits.has_position_limits, joint_limits.min_position, joint_limits.max_position);
	}

	void XArmHW::clientInit(const std::string& robot_ip, ros::NodeHandle& root_nh)
	{
		position_cmd_.resize(dof_);
		position_cmd_float_.resize(dof_); // command vector must have 7 dimention!
		position_fdb_.resize(dof_);
		velocity_cmd_.resize(dof_);
		velocity_cmd_float_.resize(dof_);
		velocity_fdb_.resize(dof_);
		effort_cmd_.resize(dof_);
		effort_fdb_.resize(dof_);

		prev_cmds_float_.resize(dof_);

		curr_err_ = 0;
		curr_state_ = 4;
		curr_mode_ = 0;
		read_code_ = 0;
		write_code_ = 0;

		// pos_sub_ = root_nh.subscribe(jnt_state_topic, 100, &XArmHW::pos_fb_cb, this);
		state_sub_ = root_nh.subscribe(xarm_state_topic, 100, &XArmHW::state_fb_cb, this);
		// wrench_sub_ = root_nh.subscribe(xarm_ftsensor_states_topic, 100, &XArmHW::ftsensor_fb_cb, this);

		for(unsigned int j=0; j < dof_; j++)
	  	{
			// Create joint state interface for all joints
			js_interface_.registerHandle(hardware_interface::JointStateHandle(jnt_names_[j], &position_fdb_[j], &velocity_fdb_[j], &effort_fdb_[j]));
			switch (ctrl_method_)
			{
			case EFFORT:
				{
					ej_interface_.registerHandle(hardware_interface::JointHandle(js_interface_.getHandle(jnt_names_[j]), &effort_cmd_[j]));
				}
				break;
			case VELOCITY:
				{
					vj_interface_.registerHandle(hardware_interface::JointHandle(js_interface_.getHandle(jnt_names_[j]), &velocity_cmd_[j]));
				}
				break;
			case POSITION:
			default:
				{
					ctrl_method_ = POSITION;
					pj_interface_.registerHandle(hardware_interface::JointHandle(js_interface_.getHandle(jnt_names_[j]), &position_cmd_[j]));
				}
				break;
			}
			_register_joint_limits(root_nh, jnt_names_[j], ctrl_method_);
	  	}

		// fts_interface_.registerHandle(hardware_interface::ForceTorqueSensorHandle(
		// 	force_torque_sensor_name_, force_torque_sensor_frame_id_, force_, torque_));

		registerInterface(&js_interface_);
		switch (ctrl_method_)
		{
		case EFFORT:
			registerInterface(&ej_interface_);
			break;
		case VELOCITY:
			registerInterface(&vj_interface_);
			break;
		case POSITION:
		default:
			registerInterface(&pj_interface_);
			break;
		}
		// registerInterface(&fts_interface_);
	  	
	  	int ret1 = xarm.motionEnable(1);
	  	int ret2 = xarm.setMode(ctrl_method_ == VELOCITY ? XARM_MODE::VELO_JOINT : XARM_MODE::SERVO);
	  	int ret3 = xarm.setState(XARM_STATE::START);
	}

	bool XArmHW::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh)
	{
		bool velocity_control = false;
		robot_hw_nh.getParam("velocity_control", velocity_control);
		// ctrl_method_ = EFFORT; // INVALID
		// ctrl_method_ = VELOCITY; // INVALID
		if (velocity_control) {
			ctrl_method_ = VELOCITY;
		}
		else {
			ctrl_method_ = POSITION; // default
		}

		hw_ns_ = robot_hw_nh.getNamespace() + "/";
		ros::service::waitForService(hw_ns_+"motion_ctrl");
	  	ros::service::waitForService(hw_ns_+"set_state");
	  	ros::service::waitForService(hw_ns_+"set_mode");
		if (ctrl_method_ == VELOCITY)
			ros::service::waitForService(hw_ns_+"velo_move_joint");
		else
	  		ros::service::waitForService(hw_ns_+"move_servoj");
		xarm.init(robot_hw_nh);
		std::string robot_ip;
		std::vector<std::string> jnt_names;
		int xarm_dof = 0;

		if(!robot_hw_nh.hasParam("DOF"))
		{
			ROS_ERROR("ROS Parameter xarm_dof not specified!");
			return false;
		}
		if(!robot_hw_nh.hasParam("xarm_robot_ip"))
		{
			ROS_ERROR("ROS Parameter xarm_robot_ip not specified!");
			return false;
		}
		// commented because now read() will check integrity before write() 
		// If there is no /robot_description parameter, moveit controller may send zero command even controller fails to initialize
		// if(!robot_hw_nh.hasParam("/robot_description"))
		// {
		// 	ROS_ERROR("ROS Parameter /robot_description not specified!");
		// 	return false;
		// }

		/* getParam forbids to change member */
		robot_hw_nh.getParam("DOF", xarm_dof);
		robot_hw_nh.getParam("xarm_robot_ip", robot_ip);
		robot_hw_nh.getParam("joint_names", jnt_names);
		// robot_hw_nh.param<std::string>("force_torque_sensor_name", force_torque_sensor_name_, "ft_sensor");
		// force_torque_sensor_frame_id_ = "ft_sensor_data";

		dof_ = xarm_dof;
		jnt_names_ = jnt_names;
		initial_write_ = true;
		pos_fdb_called_ = false;
		stat_fdb_called_ = false;

		first_read_ = true;
		read_cnts_ = 0;
		read_max_time_ = 0;
		read_total_time_ = 0;
		std::vector<float> prev_read_angles_;
		std::vector<float> curr_read_angles_;
		prev_read_angles_.resize(dof_);
		curr_read_angles_.resize(dof_);
		read_duration_ = ros::Duration(0);
		read_failed_cnts_ = 0;
		initital_check_ = false;

		enforce_limits_ = true;
		if (robot_hw_nh.hasParam("enforce_limits")) {
			robot_hw_nh.getParam("enforce_limits", enforce_limits_);
		}

		std::string robot_description;
		root_nh.getParam("/robot_description", robot_description);
		model_ptr_ = urdf::parseURDF(robot_description);

		clientInit(robot_ip, robot_hw_nh);
		return true;
	}

	XArmHW::~XArmHW()
	{
		xarm.setMode(XARM_MODE::POSE);
	}

	void XArmHW::pos_fb_cb(const sensor_msgs::JointState::ConstPtr& data)
	{
		if (data->header.stamp <= last_joint_state_stamp_) return;
		if (data->name[0]!=jnt_names_[0]) 
		{	
			// in case that the gripper joints are independently published, 
			// then the index j will not be valid for xArm joints  
			return; 
		}

		// static int call_cnt = 0;
		std::lock_guard<std::mutex> locker(mutex_);
		for(int j=0; j<dof_; j++)
		{
			position_fdb_[j] = data->position[j];
			velocity_fdb_[j] = data->velocity[j];
			effort_fdb_[j] = data->effort[j];
		}
		last_joint_state_stamp_ = data->header.stamp;

		if(!pos_fdb_called_)
		{
			pos_fdb_called_ = true;
		}
	}

	void XArmHW::state_fb_cb(const xarm_msgs::RobotMsg::ConstPtr& data)
	{
		curr_mode_ = data->mode;
		curr_state_ = data->state;
		curr_err_ = data->err;
		
		if(!stat_fdb_called_)
			stat_fdb_called_ = true;
	}

	bool XArmHW::wait_fbk_start(ros::Duration timeout)
	{
		if(timeout.isZero())
			return true;

		bool started = false;
		ros::Time end = ros::Time::now() + timeout;
		while(ros::ok() && ros::Time::now() < end)
		{
			started = pos_fdb_called_ && stat_fdb_called_;
			if(started)
				break;
			ros::Duration(0.1).sleep();
		}
		return started;
	}

	bool XArmHW::_wait_xarm_ready(double timeout)
	{
		if(timeout <= 0)
			return ros::ok() && !_xarm_is_not_ready();

		ros::Time end = ros::Time::now() + ros::Duration(timeout);
		int ready_cnts = 0;
		while(ros::ok() && ros::Time::now() < end)
		{
			if (_xarm_is_not_ready()) {
				ready_cnts = 0;
			}
			else {
				ready_cnts += 1;
				if (ready_cnts >= 20) {
					break;
				}
			}
			ros::Duration(0.1).sleep();
		}
		return ros::ok() && !_xarm_is_not_ready();
	}

	// void XArmHW::ftsensor_fb_cb(const geometry_msgs::WrenchStamped::ConstPtr& data)
	// {
	// 	if (data->header.stamp <= last_ftsensor_stamp_) return;
	// 	if (data->header.frame_id != force_torque_sensor_frame_id_) return;
		
	// 	force_[0] = data->wrench.force.x;
	// 	force_[1] = data->wrench.force.y;
	// 	force_[2] = data->wrench.force.z;
	// 	torque_[0] = data->wrench.torque.x;
	// 	torque_[1] = data->wrench.torque.y;
	// 	torque_[2] = data->wrench.torque.z;
	// 	last_ftsensor_stamp_ = data->header.stamp;
	// }

	void XArmHW::_reset_limits(void)
	{
		if (!enforce_limits_) return;
		switch (ctrl_method_)
		{
		case EFFORT:
			// no reset() interface
			break;
		case VELOCITY:
			// no reset() interface
			break;
		case POSITION:
		default:
			{
				pj_sat_interface_.reset();
				pj_limits_interface_.reset();
			}
			break;
		}
	}

	// Keep velocity and position within Moveit "joint_limits" configuration
	void XArmHW::_enforce_limits(const ros::Duration& period)
	{
		if (!enforce_limits_) return;
		switch (ctrl_method_)
		{
		case EFFORT:
			{
				ej_sat_interface_.enforceLimits(period);
				ej_limits_interface_.enforceLimits(period);
			}
			break;
		case VELOCITY:
			{
				vj_sat_interface_.enforceLimits(period);
				vj_limits_interface_.enforceLimits(period);
			}
			break;
		case POSITION:
		default:
			{
				pj_sat_interface_.enforceLimits(period);
				pj_limits_interface_.enforceLimits(period);
			}
			break;
		}
	}

	void XArmHW::read(const ros::Time& time, const ros::Duration& period)
	{
		if (!initital_check_) {
			initital_check_ = true;
			if (!_wait_xarm_ready(5.0)) {
				ROS_ERROR("The Xarm may not be properly connected (ret = 3) or hardware Error/Warning (ret = 1 or 2) exists, PLEASE CHECK or RESTART HARDWARE!!!");
				ROS_ERROR(" ");
				ROS_ERROR("Did you specify the correct ros param xarm_robot_ip ? Exitting...");
				ros::shutdown();
				exit(1);
			}
		}

		read_cnts_ += 1;
		ros::Time start = ros::Time::now();
		read_code_ = xarm.getServoAngle(curr_read_angles_);
		double time_sec = (ros::Time::now() - start).toSec();
		read_total_time_ += time_sec;
		if (time_sec > read_max_time_) {
			read_max_time_ = time_sec;
		}
		// if (read_cnts_ % 6000 == 0) {
		// 	ROS_INFO("[READ] cnt: %ld, max: %f, mean: %f, failed: %ld", read_cnts_, read_max_time_, read_total_time_ / read_cnts_, read_failed_cnts_);
		// }
		read_duration_ += period;
		if (read_code_ == 0) {
			for (int j = 0; j < dof_; j++) {
				position_fdb_[j] = curr_read_angles_[j];
				velocity_fdb_[j] = first_read_ ? 0.0 : (curr_read_angles_[j] - prev_read_angles_[j]) / read_duration_.toSec();
				effort_fdb_[j] = 0.0;
			}
			first_read_ = false;
			prev_read_angles_.swap(curr_read_angles_);
			read_duration_ -= read_duration_;
		}
		else {
			read_failed_cnts_ += 1;
			ROS_ERROR("xArmHW::Read() returns: %d", read_code_);
		}
	}

	void XArmHW::write(const ros::Time& time, const ros::Duration& period)
	{
		if (need_reset())
		{
			_reset_limits();
			return;
		}

		_enforce_limits(period);

		int cmd_ret = 0;
		switch (ctrl_method_)
		{
		case VELOCITY:
			{
				for (int k = 0; k < dof_; k++) { velocity_cmd_float_[k] = (float)velocity_cmd_[k]; }
				cmd_ret = xarm.veloMoveJoint(velocity_cmd_float_, true, VELO_DURATION);
			}
			break;
		case POSITION:
		default:
			{
				for (int k = 0; k < dof_; k++) { position_cmd_float_[k] = (float)position_cmd_[k]; }
				cur_time_ = ros::Time::now();
				elapsed_ = cur_time_ - prev_time_;
				if (elapsed_.toSec() > 1 || _check_cmds_is_change(prev_cmds_float_, position_cmd_float_)) {
					cmd_ret = xarm.setServoJ(position_cmd_float_);
					if (cmd_ret == 0) {
						prev_time_ = cur_time_;
						for (int i = 0; i < prev_cmds_float_.size(); i++) { 
							prev_cmds_float_[i] = (float)position_cmd_float_[i];
						}
					}
				}	
			}
			break;
		}

		if (cmd_ret != 0 && cmd_ret != UXBUS_STATE::WAR_CODE) {
			// to reset controller, preempt current goal
			write_code_ = cmd_ret;
		}
	}

	bool XArmHW::_check_cmds_is_change(std::vector<float> prev, std::vector<float> cur, double threshold)
    {
        for (int i = 0; i < cur.size(); i++) {
            if (std::abs(cur[i] - prev[i]) > threshold) return true;
        }
        return false;
    }

	void XArmHW::get_status(int state_mode_err[3])
	{
		state_mode_err[0] = curr_state_;
		state_mode_err[1] = curr_mode_;
		state_mode_err[2] = curr_err_;
	}

	bool XArmHW::_xarm_is_not_ready(void)
	{
		static int last_err = 0;
		if (curr_err_ != 0) {
			// not ready because error_code != 0
			if (last_err != curr_err_) {
				ROS_ERROR("[ns: %s] xArm Error detected! Code: %d", hw_ns_.c_str(), curr_err_);
				last_err = curr_err_;
			}
			return true;
		}
		last_err = 0;
		if (curr_state_ != 0 && curr_state_ != 1 && curr_state_ != 2) {
			// not ready because current state can not motion
			return true; 
		}
		if ((ctrl_method_ == VELOCITY ? curr_mode_ != XARM_MODE::VELO_JOINT : curr_mode_ != XARM_MODE::SERVO)) {
			// not ready because current mode is not correct
			return true;
		}
		return false;
	}

	bool XArmHW::need_reset()
	{
		bool is_not_ready = _xarm_is_not_ready();
		bool write_succeed = write_code_ == 0;
		if (!write_succeed) {
			int ret = xarm.setState(XARM_STATE::STOP);
			ROS_ERROR("XArmHW::Write() failed, failed_ret=%d !, Setting Robot State to STOP... (ret: %d)", write_code_, ret);
			if (write_code_ == SERVICE_IS_PERSISTENT_BUT_INVALID || write_code_ == SERVICE_CALL_FAILED) {
				ROS_ERROR("service is invaild, ros shutdown");
				ros::shutdown();
				exit(1);
			}
			write_code_ = 0;
		}
		return is_not_ready || !write_succeed || read_code_ != 0;
	}
}

PLUGINLIB_EXPORT_CLASS(xarm_control::XArmHW, hardware_interface::RobotHW)