#ifndef __XARM_DRIVER_H
#define __XARM_DRIVER_H

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <xarm_msgs/SetInt16.h>
#include <xarm_msgs/TCPOffset.h>
#include <xarm_msgs/SetLoad.h>
#include <xarm_msgs/SetAxis.h>
#include <xarm_msgs/Move.h>
#include <xarm_msgs/RobotMsg.h>
#include <xarm_msgs/IOState.h>
#include <xarm_msgs/SetDigitalIO.h>
#include <xarm_msgs/GetDigitalIO.h>
#include <xarm_msgs/GetControllerDigitalIO.h>
#include <xarm_msgs/GetAnalogIO.h>
#include <xarm_msgs/ClearErr.h>
#include <xarm_msgs/GetErr.h>
#include <xarm_msgs/GripperConfig.h>
#include <xarm_msgs/GripperMove.h>
#include <xarm_msgs/GripperState.h>
#include <xarm_msgs/SetToolModbus.h>
#include <xarm_msgs/ConfigToolModbus.h>
#include <sensor_msgs/JointState.h>
#include <xarm/common/data_type.h>
#include <xarm/linux/thread.h>
#include "xarm/connect.h"
#include "xarm/report_data.h"

namespace xarm_api
{
    class XARMDriver
    {
        public:
            XARMDriver():spinner(4){spinner.start();};
            ~XARMDriver();
            void XARMDriverInit(ros::NodeHandle& root_nh, char *server_ip);
            void Heartbeat(void);
            bool isConnectionOK(void);

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
            bool MoveLinebCB(xarm_msgs::Move::Request &req, xarm_msgs::Move::Response &res);
            bool MoveLineCB(xarm_msgs::Move::Request &req, xarm_msgs::Move::Response &res);
            bool MoveLineToolCB(xarm_msgs::Move::Request &req, xarm_msgs::Move::Response &res);
            bool MoveServoJCB(xarm_msgs::Move::Request &req, xarm_msgs::Move::Response &res);
            bool MoveServoCartCB(xarm_msgs::Move::Request &req, xarm_msgs::Move::Response &res);
            bool GripperConfigCB(xarm_msgs::GripperConfig::Request &req, xarm_msgs::GripperConfig::Response &res);
            bool GripperMoveCB(xarm_msgs::GripperMove::Request &req, xarm_msgs::GripperMove::Response &res);
            bool GripperStateCB(xarm_msgs::GripperState::Request &req, xarm_msgs::GripperState::Response &res);
            bool VacuumGripperCB(xarm_msgs::SetInt16::Request &req, xarm_msgs::SetInt16::Response &res);
            bool SetModbusCB(xarm_msgs::SetToolModbus::Request &req, xarm_msgs::SetToolModbus::Response &res);
            bool ConfigModbusCB(xarm_msgs::ConfigToolModbus::Request &req, xarm_msgs::ConfigToolModbus::Response &res);
            bool SetControllerDOutCB(xarm_msgs::SetDigitalIO::Request &req, xarm_msgs::SetDigitalIO::Response &res);
            bool GetControllerDInCB(xarm_msgs::GetControllerDigitalIO::Request &req, xarm_msgs::GetControllerDigitalIO::Response &res);
            void SleepTopicCB(const std_msgs::Float32ConstPtr& msg);

            void pub_robot_msg(xarm_msgs::RobotMsg &rm_msg);
            void pub_joint_state(sensor_msgs::JointState &js_msg);
            void pub_io_state();

            int get_frame(void);
            int get_rich_data(ReportDataNorm &norm_data);

        private:
            SocketPort *arm_report_;
            ReportDataNorm norm_data_;
            UxbusCmd *arm_cmd_;
            unsigned char rx_data_[1280];
            std::string ip;
            pthread_t thread_id_;
            ros::AsyncSpinner spinner;
            int dof_;
            int curr_state_;
            int curr_err_;
            xarm_msgs::IOState io_msg;

            ros::NodeHandle nh_;
            ros::ServiceServer go_home_server_;
            ros::ServiceServer move_joint_server_;
            ros::ServiceServer motion_ctrl_server_;
            ros::ServiceServer set_state_server_;
            ros::ServiceServer set_mode_server_;
            ros::ServiceServer move_lineb_server_;
            ros::ServiceServer move_line_server_;
            ros::ServiceServer move_line_tool_server_;
            ros::ServiceServer move_servoj_server_;
            ros::ServiceServer move_servo_cart_server_;
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

            ros::Publisher joint_state_;
            ros::Publisher robot_rt_state_; 
            ros::Publisher end_input_state_;

            ros::Subscriber sleep_sub_;

            int wait_for_finish();
    };
}

#endif
