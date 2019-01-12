/* Copyright 2018 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: waylon <weile.wang@ufactory.cc>
 ============================================================================*/
#include <xarm_driver.h>
#include "xarm/instruction/uxbus_cmd_config.h"

namespace xarm_api
{   
    XARMDriver::~XARMDriver()
    {
        arm_cmd_->set_mode(XARM_MODE::POSE);
        arm_cmd_->close();
        spinner.stop();
    }

    void XARMDriver::XARMDriverInit(ros::NodeHandle& root_nh, char *server_ip)
    {   
        nh_ = root_nh;
        // api command services:
        motion_ctrl_server_ = nh_.advertiseService("motion_ctrl", &XARMDriver::MotionCtrlCB, this);
        set_mode_server_ = nh_.advertiseService("set_mode", &XARMDriver::SetModeCB, this);
        set_state_server_ = nh_.advertiseService("set_state", &XARMDriver::SetStateCB, this);
        set_tcp_offset_server_ = nh_.advertiseService("set_tcp_offset", &XARMDriver::SetTCPOffsetCB, this);
        go_home_server_ = nh_.advertiseService("go_home", &XARMDriver::GoHomeCB, this);
        move_joint_server_ = nh_.advertiseService("move_joint", &XARMDriver::MoveJointCB, this);
        move_lineb_server_ = nh_.advertiseService("move_lineb", &XARMDriver::MoveLinebCB, this);
        move_line_server_ = nh_.advertiseService("move_line", &XARMDriver::MoveLineCB, this);
        move_servoj_server_ = nh_.advertiseService("move_servoj", &XARMDriver::MoveServoJCB, this);

        // state feedback topics:
        joint_state_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 10, true);
        robot_rt_state_ = nh_.advertise<xarm_msgs::RobotMsg>("xarm_states", 10, true);

        arm_report_ = connect_tcp_report_norm(server_ip);
        // ReportDataNorm norm_data_;
        arm_cmd_ = connect_tcp_control(server_ip);  
        if (arm_cmd_ == NULL)
            ROS_ERROR("Xarm Connection Failed!");
    }

    bool XARMDriver::MotionCtrlCB(xarm_msgs::SetAxis::Request& req, xarm_msgs::SetAxis::Response& res)
    {
        res.ret = arm_cmd_->motion_en(req.id, req.data);
        if(req.data == 1)
        {
            res.message = "motion enable, ret = "  + std::to_string(res.ret);
        }
        else
        {
            res.message = "motion disable, ret = " + std::to_string(res.ret);
        }
        return true;
    }

    bool XARMDriver::SetModeCB(xarm_msgs::SetInt16::Request& req, xarm_msgs::SetInt16::Response& res)
    {
        res.ret = arm_cmd_->set_mode(req.data);
        switch(req.data)
        {
            case XARM_MODE::POSE:
            {
               res.message = "pose mode, ret = " + std::to_string(res.ret);
            }break;
            case XARM_MODE::SERVO:
			{
				res.message = "servo mode, ret = " + std::to_string(res.ret);
			}break;
            case XARM_MODE::TEACH_CART:
			{
				res.message = "cartesian teach, ret = " + std::to_string(res.ret);
			} break;
            case XARM_MODE::TEACH_JOINT:
			{
				res.message = "joint teach , ret = " + std::to_string(res.ret);
			} break;
            default:
            {
                res.message = "the failed mode, ret = " + std::to_string(res.ret);
            }
        }

        return true;
    }

    bool XARMDriver::SetStateCB(xarm_msgs::SetInt16::Request& req, xarm_msgs::SetInt16::Response& res)
    {
        res.ret = arm_cmd_->set_state(req.data);
        switch(req.data)
        {
            case XARM_STATE::START:
            {
               res.message = "start, ret = " + std::to_string(res.ret);
            }break;
            case XARM_STATE::PAUSE:
            {
               res.message = "pause, ret = " + std::to_string(res.ret);
            }break;
            case XARM_STATE::STOP:
            {
               res.message = "stop, ret = " + std::to_string(res.ret);
            }break;
            default:
            {
                res.message = "the failed state, ret = " + std::to_string(res.ret);
            }
        }

        return true;
    }

    bool XARMDriver::SetTCPOffsetCB(xarm_msgs::TCPOffset::Request &req, xarm_msgs::TCPOffset::Response &res)
    {
        float offsets[6] = {req.x, req.y, req.z, req.roll, req.pitch, req.yaw};
        res.ret = arm_cmd_->set_tcp_offset(offsets);
        res.message = "set tcp offset: ret = " + std::to_string(res.ret); 
        return true;
    }

    bool XARMDriver::GoHomeCB(xarm_msgs::Move::Request &req, xarm_msgs::Move::Response &res)
    {
        res.ret = arm_cmd_->move_gohome(req.mvvelo, req.mvacc, req.mvtime);
        res.message = "go home, ret = " + std::to_string(res.ret);
        return true;
    }

    bool XARMDriver::MoveJointCB(xarm_msgs::Move::Request &req, xarm_msgs::Move::Response &res)
    {
        float joint[1][7];
        int index = 0;
        if(req.pose.size() != 7)
        {
            res.ret = req.pose.size();
            res.message = "pose parameters incorrect!";
            return true;
        }
        else
        {
            for(index = 0; index < 7; index++)
            {
                joint[0][index] = req.pose[index];
            }
        }

        res.ret = arm_cmd_->move_joint(joint[0], req.mvvelo, req.mvacc, req.mvtime);
        res.message = "move joint, ret = " + std::to_string(res.ret);
        return true;
    }

    bool XARMDriver::MoveLineCB(xarm_msgs::Move::Request &req, xarm_msgs::Move::Response &res)
    {
        float pose[1][6];
        int index = 0;
        if(req.pose.size() != 6)
        {
            res.ret = -1;
            res.message = "parameters incorrect!";
            return true;
        }
        else
        {
            for(index = 0; index < 6; index++)
            {
                pose[0][index] = req.pose[index];
            }
        }

        res.ret = arm_cmd_->move_line(pose[0], req.mvvelo, req.mvacc, req.mvtime);
        res.message = "move line, ret = " + std::to_string(res.ret);
        return true;
    }

    bool XARMDriver::MoveLinebCB(xarm_msgs::Move::Request &req, xarm_msgs::Move::Response &res)
    {
        float pose[1][6];
        int index = 0;
        if(req.pose.size() != 6)
        {
            res.ret = -1;
            res.message = "parameters incorrect!";
            return true;
        }
        else
        {
            for(index = 0; index < 6; index++)
            {
                pose[0][index] = req.pose[index];
            }
        }

        res.ret = arm_cmd_->move_lineb(pose[0], req.mvvelo, req.mvacc, req.mvtime, req.mvradii);
        res.message = "move lineb, ret = " + std::to_string(res.ret);
        return true;
    }

    bool XARMDriver::MoveServoJCB(xarm_msgs::Move::Request &req, xarm_msgs::Move::Response &res)
    {
        float pose[1][7];
        int index = 0;
        if(req.pose.size() != 7)
        {
            res.ret = -1;
            res.message = "parameters incorrect!";
            return true;
        }
        else
        {
            for(index = 0; index < 7; index++)
            {
                pose[0][index] = req.pose[index];
            }
        }

        res.ret = arm_cmd_->move_servoj(pose[0], req.mvvelo, req.mvacc, req.mvtime);
        res.message = "move servoj, ret = " + std::to_string(res.ret);
        return true;
    }

    void XARMDriver::pub_robot_msg(xarm_msgs::RobotMsg rm_msg)
    {
        robot_rt_state_.publish(rm_msg);
    }
    
    void XARMDriver::pub_joint_state(sensor_msgs::JointState js_msg)
    {
        joint_state_.publish(js_msg);
    }

    int XARMDriver::get_frame(void)
    {
        int ret;
        ret = arm_report_->read_frame(rx_data_);
        return ret;
    }

    int XARMDriver::get_rich_data(ReportDataNorm &norm_data)
    {
        int ret;
        ret = norm_data_.flush_data(rx_data_);
        norm_data = norm_data_;
        return ret;
    }
}
