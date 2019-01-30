#ifndef __XARM_DRIVER_H
#define __XARM_DRIVER_H

#include "ros/ros.h"
#include <xarm_msgs/SetInt16.h>
#include <xarm_msgs/TCPOffset.h>
#include <xarm_msgs/SetAxis.h>
#include <xarm_msgs/Move.h>
#include <xarm_msgs/RobotMsg.h>
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

            bool MotionCtrlCB(xarm_msgs::SetAxis::Request &req, xarm_msgs::SetAxis::Response &res);
            bool SetModeCB(xarm_msgs::SetInt16::Request& req, xarm_msgs::SetInt16::Response& res);
            bool SetStateCB(xarm_msgs::SetInt16::Request &req, xarm_msgs::SetInt16::Response &res);
            bool SetTCPOffsetCB(xarm_msgs::TCPOffset::Request &req, xarm_msgs::TCPOffset::Response &res);
            bool GoHomeCB(xarm_msgs::Move::Request &req, xarm_msgs::Move::Response &res);
            bool MoveJointCB(xarm_msgs::Move::Request &req, xarm_msgs::Move::Response &res);
            bool MoveLinebCB(xarm_msgs::Move::Request &req, xarm_msgs::Move::Response &res);
            bool MoveLineCB(xarm_msgs::Move::Request &req, xarm_msgs::Move::Response &res);
            bool MoveServoJCB(xarm_msgs::Move::Request &req, xarm_msgs::Move::Response &res);

            void pub_robot_msg(xarm_msgs::RobotMsg rm_msg);
            void pub_joint_state(sensor_msgs::JointState js_msg);

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

            ros::NodeHandle nh_;
            ros::ServiceServer go_home_server_;
            ros::ServiceServer move_joint_server_;
            ros::ServiceServer motion_ctrl_server_;
            ros::ServiceServer set_state_server_;
            ros::ServiceServer set_mode_server_;
            ros::ServiceServer move_lineb_server_;
            ros::ServiceServer move_line_server_;
            ros::ServiceServer move_servoj_server_;
            ros::ServiceServer set_tcp_offset_server_;

            ros::Publisher joint_state_;
            ros::Publisher robot_rt_state_; 
    };
}

#endif
