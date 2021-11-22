#ifndef __XARM_DRIVER_H
#define __XARM_DRIVER_H

#include <thread>
#include <ros/ros.h>
#include <control_msgs/GripperCommandAction.h>
#include <actionlib/server/action_server.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/JointState.h>
// #include <geometry_msgs/WrenchStamped.h>
#include "xarm_api/xarm_msgs.h"
#include "xarm/wrapper/xarm_api.h"

namespace xarm_api
{
    class XArmDriver
    {
    public:
        XArmDriver():spinner_(4){ spinner_.start(); };
        ~XArmDriver();
        void init(ros::NodeHandle& root_nh, std::string &server_ip);

        // provide a list of services:
        bool MotionCtrlCB(xarm_msgs::SetAxis::Request &req, xarm_msgs::SetAxis::Response &res);
        bool SetModeCB(xarm_msgs::SetInt16::Request& req, xarm_msgs::SetInt16::Response& res);
        bool SetStateCB(xarm_msgs::SetInt16::Request &req, xarm_msgs::SetInt16::Response &res);
        bool SetTCPOffsetCB(xarm_msgs::TCPOffset::Request &req, xarm_msgs::TCPOffset::Response &res);
        bool SetLoadCB(xarm_msgs::SetLoad::Request &req, xarm_msgs::SetLoad::Response &res);
        bool SetDigitalIOCB(xarm_msgs::SetDigitalIO::Request &req, xarm_msgs::SetDigitalIO::Response &res);
        bool GetDigitalIOCB(xarm_msgs::GetDigitalIO::Request &req, xarm_msgs::GetDigitalIO::Response &res);
        bool GetAnalogIOCB(xarm_msgs::GetAnalogIO::Request &req, xarm_msgs::GetAnalogIO::Response &res);
        bool ClearErrCB(xarm_msgs::ClearErr::Request& req, xarm_msgs::ClearErr::Response& res);
        bool MoveitClearErrCB(xarm_msgs::ClearErr::Request& req, xarm_msgs::ClearErr::Response& res);
        bool GetErrCB(xarm_msgs::GetErr::Request & req, xarm_msgs::GetErr::Response & res);
        bool GoHomeCB(xarm_msgs::Move::Request &req, xarm_msgs::Move::Response &res);
        bool MoveJointCB(xarm_msgs::Move::Request &req, xarm_msgs::Move::Response &res);
        bool MoveJointbCB(xarm_msgs::Move::Request &req, xarm_msgs::Move::Response &res);
        bool MoveLinebCB(xarm_msgs::Move::Request &req, xarm_msgs::Move::Response &res);
        bool MoveLineCB(xarm_msgs::Move::Request &req, xarm_msgs::Move::Response &res);
        bool MoveLineToolCB(xarm_msgs::Move::Request &req, xarm_msgs::Move::Response &res);
        bool MoveServoJCB(xarm_msgs::Move::Request &req, xarm_msgs::Move::Response &res);
        bool MoveServoCartCB(xarm_msgs::Move::Request &req, xarm_msgs::Move::Response &res);
        bool MoveLineAACB(xarm_msgs::MoveAxisAngle::Request &req, xarm_msgs::MoveAxisAngle::Response &res);
        bool MoveServoCartAACB(xarm_msgs::MoveAxisAngle::Request &req, xarm_msgs::MoveAxisAngle::Response &res);
        bool GripperConfigCB(xarm_msgs::GripperConfig::Request &req, xarm_msgs::GripperConfig::Response &res);
        bool GripperMoveCB(xarm_msgs::GripperMove::Request &req, xarm_msgs::GripperMove::Response &res);
        bool GripperStateCB(xarm_msgs::GripperState::Request &req, xarm_msgs::GripperState::Response &res);
        bool VacuumGripperCB(xarm_msgs::SetInt16::Request &req, xarm_msgs::SetInt16::Response &res);
        bool SetModbusCB(xarm_msgs::SetToolModbus::Request &req, xarm_msgs::SetToolModbus::Response &res);
        bool ConfigModbusCB(xarm_msgs::ConfigToolModbus::Request &req, xarm_msgs::ConfigToolModbus::Response &res);
        bool SetControllerDOutCB(xarm_msgs::SetDigitalIO::Request &req, xarm_msgs::SetDigitalIO::Response &res);
        bool GetControllerDInCB(xarm_msgs::GetControllerDigitalIO::Request &req, xarm_msgs::GetControllerDigitalIO::Response &res);
        bool SetControllerAOutCB(xarm_msgs::SetControllerAnalogIO::Request &req, xarm_msgs::SetControllerAnalogIO::Response &res);
        bool GetControllerAInCB(xarm_msgs::GetAnalogIO::Request &req, xarm_msgs::GetAnalogIO::Response &res);
        void SleepTopicCB(const std_msgs::Float32ConstPtr& msg);
        bool VeloMoveJointCB(xarm_msgs::MoveVelo::Request &req, xarm_msgs::MoveVelo::Response &res);
        bool VeloMoveLineVCB(xarm_msgs::MoveVelo::Request &req, xarm_msgs::MoveVelo::Response &res);
        bool VCSetJointVelocityCB(xarm_msgs::MoveVelocity::Request &req, xarm_msgs::MoveVelocity::Response &res);
        bool VCSetCartesianVelocityCB(xarm_msgs::MoveVelocity::Request &req, xarm_msgs::MoveVelocity::Response &res);
        bool SetMaxJAccCB(xarm_msgs::SetFloat32::Request &req, xarm_msgs::SetFloat32::Response &res);
        bool SetMaxLAccCB(xarm_msgs::SetFloat32::Request &req, xarm_msgs::SetFloat32::Response &res);

        bool SetRecordingCB(xarm_msgs::SetInt16::Request &req, xarm_msgs::SetInt16::Response &res);
        bool SaveTrajCB(xarm_msgs::SetString::Request &req, xarm_msgs::SetString::Response &res);
        bool LoadNPlayTrajCB(xarm_msgs::PlayTraj::Request &req, xarm_msgs::PlayTraj::Response &res);

        bool SetReboundCB(xarm_msgs::SetInt16::Request& req, xarm_msgs::SetInt16::Response& res);
        bool SetCollSensCB(xarm_msgs::SetInt16::Request& req, xarm_msgs::SetInt16::Response& res);
        bool SetTeachSensCB(xarm_msgs::SetInt16::Request& req, xarm_msgs::SetInt16::Response& res);

        bool SetWorldOffsetCB(xarm_msgs::TCPOffset::Request &req, xarm_msgs::TCPOffset::Response &res);
        bool SetFenceModeCB(xarm_msgs::SetInt16::Request& req, xarm_msgs::SetInt16::Response& res);
        bool SetReducedModeCB(xarm_msgs::SetInt16::Request& req, xarm_msgs::SetInt16::Response& res);
        bool SetTcpJerkCB(xarm_msgs::SetFloat32::Request &req, xarm_msgs::SetFloat32::Response &res);
        bool SetJointJerkCB(xarm_msgs::SetFloat32::Request &req, xarm_msgs::SetFloat32::Response &res);

        bool GetServoAngleCB(xarm_msgs::GetFloat32List::Request &req, xarm_msgs::GetFloat32List::Response &res);

        void pub_robot_msg(xarm_msgs::RobotMsg &rm_msg);
        void pub_joint_state(sensor_msgs::JointState &js_msg);
        void pub_cgpio_state(xarm_msgs::CIOState &cio_msg);
        // void pub_ftsensor_state(geometry_msgs::WrenchStamped &wrench_msg);

    private:
        void _report_connect_changed_callback(bool connected, bool reported);
        void _report_data_callback(XArmReportData *report_data_ptr);
        bool _get_wait_param(void);

        void _handle_gripper_action_goal(actionlib::ActionServer<control_msgs::GripperCommandAction>::GoalHandle gh);
        void _handle_gripper_action_cancel(actionlib::ActionServer<control_msgs::GripperCommandAction>::GoalHandle gh);
        void _pub_gripper_joint_states(float pos);
    
    public:
        XArmAPI *arm;
        const static int max_gripper_pos = 850;

    private:
        std::string report_type_;
        ros::AsyncSpinner spinner_;
        int dof_;
        int curr_state_;
        int curr_err_;
        int curr_cmd_num_;
        int curr_mode_;
        float init_gripper_pos_;
        bool gripper_init_loop_;
        bool gripper_added_;

        ros::NodeHandle nh_;
        ros::ServiceServer go_home_server_;
        ros::ServiceServer move_joint_server_;
        ros::ServiceServer move_jointb_server_;
        ros::ServiceServer motion_ctrl_server_;
        ros::ServiceServer set_state_server_;
        ros::ServiceServer set_mode_server_;
        ros::ServiceServer move_lineb_server_;
        ros::ServiceServer move_line_server_;
        ros::ServiceServer move_line_tool_server_;
        ros::ServiceServer move_servoj_server_;
        ros::ServiceServer move_servo_cart_server_;
        ros::ServiceServer move_line_aa_server_;
        ros::ServiceServer move_servo_cart_aa_server_;
        ros::ServiceServer set_tcp_offset_server_;
        ros::ServiceServer set_load_server_;
        ros::ServiceServer set_end_io_server_;
        ros::ServiceServer get_digital_in_server_;
        ros::ServiceServer get_analog_in_server_;
        ros::ServiceServer clear_err_server_;
        ros::ServiceServer moveit_clear_err_server_;
        ros::ServiceServer get_err_server_;
        ros::ServiceServer gripper_config_server_;
        ros::ServiceServer gripper_move_server_;
        ros::ServiceServer gripper_state_server_;
        ros::ServiceServer set_vacuum_gripper_server_;
        ros::ServiceServer set_modbus_server_;
        ros::ServiceServer config_modbus_server_;
        ros::ServiceServer set_controller_dout_server_;
        ros::ServiceServer get_controller_din_server_;
        ros::ServiceServer set_controller_aout_server_;
        ros::ServiceServer get_controller_ain_server_;

        // ros::ServiceServer tgpio_delay_set_digital_server_;
        // ros::ServiceServer cgpio_delay_set_digital_server_;
        // ros::ServiceServer tgpio_position_set_digital_server_;
        // ros::ServiceServer cgpio_position_set_digital_server_;
        // ros::ServiceServer cgpio_position_set_analog_server_;
        ros::ServiceServer vc_set_jointv_server_;
        ros::ServiceServer vc_set_linev_server_;
        ros::ServiceServer vdc_set_jointv_server_;
        ros::ServiceServer vdc_set_linev_server_;
        ros::ServiceServer set_max_jacc_server_;
        ros::ServiceServer set_max_lacc_server_;

        ros::ServiceServer traj_record_server_;
        ros::ServiceServer traj_save_server_;
        ros::ServiceServer traj_play_server_;

        ros::ServiceServer set_rebound_server_;
        ros::ServiceServer set_coll_sens_server_;
        ros::ServiceServer set_teach_sens_server_;

        ros::ServiceServer set_world_offset_server_;
        ros::ServiceServer set_fence_mode_server_;
        ros::ServiceServer set_reduced_mode_server_;
        ros::ServiceServer set_tcp_jerk_server_;
        ros::ServiceServer set_joint_jerk_server_;

        ros::ServiceServer get_servo_angle_;

        ros::Publisher joint_state_;
        ros::Publisher robot_rt_state_; 
        ros::Publisher end_input_state_;
        ros::Publisher cgpio_state_;
        // ros::Publisher ftsensor_state_;

        ros::Subscriber sleep_sub_;

        std::shared_ptr<actionlib::ActionServer<control_msgs::GripperCommandAction>> gripper_action_server_;
        sensor_msgs::JointState gripper_joint_state_msg_;

    // private:
    //     void _init_xarm_api_server(void);

    // public:
    //     bool MotionEnableCallback(xarm_msgs::SetAxis::Request &req, xarm_msgs::SetAxis::Response &res);
    //     bool SetModeCallback(xarm_msgs::SetInt16::Request& req, xarm_msgs::SetInt16::Response& res);
    //     bool SetStateCallback(xarm_msgs::SetInt16::Request &req, xarm_msgs::SetInt16::Response &res);
    //     bool SetTcpLoadCallback(xarm_msgs::SetLoad::Request& req, xarm_msgs::SetLoad::Response& res);
    //     bool SetTcpOffsetCallback(xarm_msgs::TCPOffset::Request &req, xarm_msgs::TCPOffset::Response &res);

    //     bool GetErrWarnCodeCallback(xarm_msgs::GetErr::Request & req, xarm_msgs::GetErr::Response & res);
    //     bool CleanErrorCallback(xarm_msgs::ClearErr::Request& req, xarm_msgs::ClearErr::Response& res);
    //     bool MoveitClearErrorCallback(xarm_msgs::ClearErr::Request& req, xarm_msgs::ClearErr::Response& res);

    //     bool MoveGoHomeCallback(xarm_msgs::Move::Request &req, xarm_msgs::Move::Response &res);
    //     bool MoveJointCallback(xarm_msgs::Move::Request &req, xarm_msgs::Move::Response &res);
    //     bool MoveArcJointCallback(xarm_msgs::Move::Request &req, xarm_msgs::Move::Response &res);
    //     bool MoveLineCallback(xarm_msgs::Move::Request &req, xarm_msgs::Move::Response &res);
    //     bool MoveArcLineCallback(xarm_msgs::Move::Request &req, xarm_msgs::Move::Response &res);
    //     bool MoveServoJCallback(xarm_msgs::Move::Request &req, xarm_msgs::Move::Response &res);
    //     bool MoveServoCartCallback(xarm_msgs::Move::Request &req, xarm_msgs::Move::Response &res);
    //     bool MoveToolLineCallback(xarm_msgs::Move::Request &req, xarm_msgs::Move::Response &res);
    //     bool MoveLineAACallback(xarm_msgs::MoveAxisAngle::Request &req, xarm_msgs::MoveAxisAngle::Response &res);
    //     bool MoveServoCartAACallback(xarm_msgs::MoveAxisAngle::Request &req, xarm_msgs::MoveAxisAngle::Response &res);
    //     bool VeloMoveJointCallback(xarm_msgs::MoveVelo::Request &req, xarm_msgs::MoveVelo::Response &res);
    //     bool VeloMoveLineCallback(xarm_msgs::MoveVelo::Request &req, xarm_msgs::MoveVelo::Response &res);
    //     bool SetJointMaxAccCallback(xarm_msgs::SetFloat32::Request &req, xarm_msgs::SetFloat32::Response &res);
    //     bool SetTcpMaxAccCallback(xarm_msgs::SetFloat32::Request &req, xarm_msgs::SetFloat32::Response &res);

    //     bool SetTGPIODigitalCallback(xarm_msgs::SetDigitalIO::Request &req, xarm_msgs::SetDigitalIO::Response &res);
    //     bool GetTGPIODigitalCallback(xarm_msgs::GetDigitalIO::Request &req, xarm_msgs::GetDigitalIO::Response &res);
    //     bool GetTGPIOAnalogCallback(xarm_msgs::GetAnalogIO::Request &req, xarm_msgs::GetAnalogIO::Response &res);

    //     bool SetModbusParamsCallback(xarm_msgs::ConfigToolModbus::Request &req, xarm_msgs::ConfigToolModbus::Response &res);
    //     bool SetModbusDataCallback(xarm_msgs::SetToolModbus::Request &req, xarm_msgs::SetToolModbus::Response &res);

    //     bool SetVacuumGripperCallback(xarm_msgs::SetInt16::Request &req, xarm_msgs::SetInt16::Response &res);
    //     bool SetCGPIODigitalCallback(xarm_msgs::SetDigitalIO::Request &req, xarm_msgs::SetDigitalIO::Response &res);
    //     bool GetCGPIODigitalCallback(xarm_msgs::GetControllerDigitalIO::Request &req, xarm_msgs::GetControllerDigitalIO::Response &res);
    //     bool GetCGPIOAnalogCallback(xarm_msgs::GetAnalogIO::Request &req, xarm_msgs::GetAnalogIO::Response &res);
    //     bool SetCGPIOAnalogCallback(xarm_msgs::SetControllerAnalogIO::Request &req, xarm_msgs::SetControllerAnalogIO::Response &res);

    //     bool CleanGripperErrorCallback(xarm_msgs::ClearErr::Request& req, xarm_msgs::ClearErr::Response& res);
    //     bool SetGripperParamsCallback(xarm_msgs::GripperConfig::Request &req, xarm_msgs::GripperConfig::Response &res);
    //     bool SetGripperMoveCallback(xarm_msgs::GripperMove::Request &req, xarm_msgs::GripperMove::Response &res);
    //     bool GetGripperState(xarm_msgs::GripperState::Request &req, xarm_msgs::GripperState::Response &res);

    };
}

#endif
