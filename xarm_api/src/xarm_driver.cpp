/* Copyright 2018 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jason Peng <jason@ufactory.cc>
 ============================================================================*/
#include "xarm_driver.h"
#define CMD_HEARTBEAT_SEC 30 // 30s

#define DEBUG_MODE 1


void* cmd_heart_beat(void* args)
{
    xarm_api::XArmDriver *my_driver = (xarm_api::XArmDriver *) args;
    int cmdnum;
    int cnt = 0;
    int max_cnt = CMD_HEARTBEAT_SEC * 2;
    while(my_driver->arm->is_connected())
    {
        ros::Duration(0.5).sleep(); // non-realtime
        cnt += 1;
        if (cnt >= max_cnt) {
            cnt = 0;
            my_driver->arm->get_cmdnum(&cmdnum);
        }
    }
    ROS_ERROR("xArm Control Connection Failed! Please Shut Down (Ctrl-C) and Retry ...");
    return (void*)0;
}

namespace xarm_api
{   
    XArmDriver::~XArmDriver()
    {   
        arm->set_mode(XARM_MODE::POSE);
        arm->disconnect();
        spinner_.stop();
    }

    bool XArmDriver::_get_wait_param(void) 
    {
        bool wait;
        nh_.getParam("wait_for_finish", wait);
        return wait;
    }

    void XArmDriver::_report_connect_changed_callback(bool connected, bool reported)
    {
        ROS_INFO("[TCP STATUS] CONTROL: %d, REPORT: %d", connected, reported);
    }

    void XArmDriver::_report_data_callback(XArmReportData *report_data_ptr)
    {
        // ROS_INFO("[1] state: %d, error_code: %d", report_data_ptr->state, report_data_ptr->err);
        curr_state_ = report_data_ptr->state;
        curr_err_ = report_data_ptr->err;
    }

    void XArmDriver::init(ros::NodeHandle& root_nh, std::string &server_ip)
    {   
        nh_ = root_nh;
        nh_.getParam("DOF",dof_);
        root_nh.getParam("xarm_report_type", report_type_);
        
        arm = new XArmAPI(
            server_ip, 
            true, // is_radian
            false, // do_not_open
            true, // check_tcp_limit
            true, // check_joint_limit
            true, // check_cmdnum_limit
            false, // check_robot_sn
            true, // check_is_ready
            true, // check_is_pause
            0, // max_callback_thread_count
            512, // max_cmdnum
            dof_, // init_axis
            DEBUG_MODE, // debug
            report_type_ // report_type
        );
        arm->release_connect_changed_callback(true);
        arm->release_report_data_callback(true);
        // arm->register_connect_changed_callback(std::bind(&XArmDriver::_report_connect_changed_callback, this, std::placeholders::_1, std::placeholders::_2));
        arm->register_report_data_callback(std::bind(&XArmDriver::_report_data_callback, this, std::placeholders::_1));
        // arm->connect();

        int err_warn[2] = {0};
        int ret = arm->get_err_warn_code(err_warn);
        if (err_warn[0] != 0) {
            ROS_WARN("xArmErrorCode: %d", err_warn[0]);
        }
        
        std::thread th(cmd_heart_beat, this);
        th.detach();
        int dbg_msg[16] = {0};
        arm->core->servo_get_dbmsg(dbg_msg);

        for(int i=0; i<dof_; i++)
        {
            if((dbg_msg[i*2]==1)&&(dbg_msg[i*2+1]==40))
            {
                arm->clean_error();
                ROS_WARN("Cleared low-voltage error of joint %d", i+1);
            }
            else if((dbg_msg[i*2]==1))
            {
                arm->clean_error();
                ROS_WARN("There is servo error code:(0x%x) in joint %d, trying to clear it..", dbg_msg[i*2+1], i+1);
            }
        }

        // api command services:
        motion_ctrl_server_ = nh_.advertiseService("motion_ctrl", &XArmDriver::MotionCtrlCB, this);
        set_mode_server_ = nh_.advertiseService("set_mode", &XArmDriver::SetModeCB, this);
        set_state_server_ = nh_.advertiseService("set_state", &XArmDriver::SetStateCB, this);
        set_tcp_offset_server_ = nh_.advertiseService("set_tcp_offset", &XArmDriver::SetTCPOffsetCB, this);
        set_load_server_ = nh_.advertiseService("set_load", &XArmDriver::SetLoadCB, this);

        go_home_server_ = nh_.advertiseService("go_home", &XArmDriver::GoHomeCB, this);
        move_joint_server_ = nh_.advertiseService("move_joint", &XArmDriver::MoveJointCB, this);
        move_jointb_server_ = nh_.advertiseService("move_jointb", &XArmDriver::MoveJointbCB, this);
        move_lineb_server_ = nh_.advertiseService("move_lineb", &XArmDriver::MoveLinebCB, this);
        move_line_server_ = nh_.advertiseService("move_line", &XArmDriver::MoveLineCB, this);
        move_line_tool_server_ = nh_.advertiseService("move_line_tool", &XArmDriver::MoveLineToolCB, this);
        move_servoj_server_ = nh_.advertiseService("move_servoj", &XArmDriver::MoveServoJCB, this);
        move_servo_cart_server_ = nh_.advertiseService("move_servo_cart", &XArmDriver::MoveServoCartCB, this);        
        clear_err_server_ = nh_.advertiseService("clear_err", &XArmDriver::ClearErrCB, this);
        moveit_clear_err_server_ = nh_.advertiseService("moveit_clear_err", &XArmDriver::MoveitClearErrCB, this);
        get_err_server_ = nh_.advertiseService("get_err", &XArmDriver::GetErrCB, this);

        // axis-angle motion:
        move_line_aa_server_ = nh_.advertiseService("move_line_aa", &XArmDriver::MoveLineAACB, this);
        move_servo_cart_aa_server_ = nh_.advertiseService("move_servo_cart_aa", &XArmDriver::MoveServoCartAACB, this);

        // tool io:
        set_end_io_server_ = nh_.advertiseService("set_digital_out", &XArmDriver::SetDigitalIOCB, this);
        get_digital_in_server_ = nh_.advertiseService("get_digital_in", &XArmDriver::GetDigitalIOCB, this);
        get_analog_in_server_ = nh_.advertiseService("get_analog_in", &XArmDriver::GetAnalogIOCB, this);
        config_modbus_server_ = nh_.advertiseService("config_tool_modbus", &XArmDriver::ConfigModbusCB, this);
        set_modbus_server_ = nh_.advertiseService("set_tool_modbus", &XArmDriver::SetModbusCB, this);

        gripper_config_server_ = nh_.advertiseService("gripper_config", &XArmDriver::GripperConfigCB, this);
        gripper_move_server_ = nh_.advertiseService("gripper_move", &XArmDriver::GripperMoveCB, this);
        gripper_state_server_ = nh_.advertiseService("gripper_state", &XArmDriver::GripperStateCB, this);

        set_vacuum_gripper_server_ = nh_.advertiseService("vacuum_gripper_set", &XArmDriver::VacuumGripperCB, this);

        // controller_io (digital):
        set_controller_dout_server_ = nh_.advertiseService("set_controller_dout", &XArmDriver::SetControllerDOutCB, this);
        get_controller_din_server_ = nh_.advertiseService("get_controller_din", &XArmDriver::GetControllerDInCB, this);
        set_controller_aout_server_ = nh_.advertiseService("set_controller_aout", &XArmDriver::SetControllerAOutCB, this);
        get_controller_ain_server_ = nh_.advertiseService("get_controller_ain", &XArmDriver::GetControllerAInCB, this);

        // velocity control:
        vc_set_jointv_server_ = nh_.advertiseService("velo_move_joint", &XArmDriver::VeloMoveJointCB, this);
        vc_set_linev_server_ = nh_.advertiseService("velo_move_line", &XArmDriver::VeloMoveLineVCB, this);
        set_max_jacc_server_ = nh_.advertiseService("set_max_acc_joint", &XArmDriver::SetMaxJAccCB, this);
        set_max_lacc_server_ = nh_.advertiseService("set_max_acc_line", &XArmDriver::SetMaxLAccCB, this);

        // state feedback topics:
        joint_state_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 10, true);
        robot_rt_state_ = nh_.advertise<xarm_msgs::RobotMsg>("xarm_states", 10, true);
        cgpio_state_ = nh_.advertise<xarm_msgs::CIOState>("xarm_cgpio_states", 10, true);

        // subscribed topics
        sleep_sub_ = nh_.subscribe("sleep_sec", 1, &XArmDriver::SleepTopicCB, this);

    }

    void XArmDriver::SleepTopicCB(const std_msgs::Float32ConstPtr& msg)
    {
        if(msg->data>0)
            arm->set_pause_time(msg->data);
    }

    bool XArmDriver::ClearErrCB(xarm_msgs::ClearErr::Request& req, xarm_msgs::ClearErr::Response& res)
    {
        // First clear controller warning and error:
        int ret1 = arm->clean_gripper_error();
        int ret2 = arm->clean_error(); 
        int ret3 = arm->clean_warn();

        // Then try to enable motor again:
        res.ret = arm->motion_enable(true, 8);

        if(res.ret)
        {
            res.message = "clear err, ret = "  + std::to_string(res.ret);
        }
        return res.ret >= 0;

        // After calling this service, user should check '/xarm_states' again to make sure 'err' field is 0, to confirm success.
    }

    bool XArmDriver::MoveitClearErrCB(xarm_msgs::ClearErr::Request& req, xarm_msgs::ClearErr::Response& res)
    {
        if(ClearErrCB(req, res))
        {
            arm->set_mode(XARM_MODE::SERVO);
            int ret = arm->set_state(XARM_STATE::START);
            return ret == 0;
        }
        return false;

        // After calling this service, user should check '/xarm_states' again to make sure 'err' field is 0, to confirm success.
    }
    
    bool XArmDriver::GetErrCB(xarm_msgs::GetErr::Request & req, xarm_msgs::GetErr::Response & res)
    {
        res.err = curr_err_;
        res.message = "current error code = "  + std::to_string(res.err);
        return true;
    }

    bool XArmDriver::MotionCtrlCB(xarm_msgs::SetAxis::Request& req, xarm_msgs::SetAxis::Response& res)
    {
        res.ret = arm->motion_enable(req.data, req.id);
        if(req.data == 1)
        {
            res.message = "motion enable, ret = "  + std::to_string(res.ret);
        }
        else
        {
            res.message = "motion disable, ret = " + std::to_string(res.ret);
        }
        return res.ret >= 0;
    }

    bool XArmDriver::SetModeCB(xarm_msgs::SetInt16::Request& req, xarm_msgs::SetInt16::Response& res)
    {
        /* for successful none-zero mode switch, must happen at STOP state */
        arm->set_state(XARM_STATE::STOP);
        ros::Duration(0.01).sleep();
        
        res.ret = arm->set_mode(req.data);
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
				res.message = "joint teach, ret = " + std::to_string(res.ret);
			} break;
            case XARM_MODE::VELO_JOINT:
			{
				res.message = "joint velocity, ret = " + std::to_string(res.ret);
			} break;
            case XARM_MODE::VELO_CART:
			{
				res.message = "cartesian velocity, ret = " + std::to_string(res.ret);
			} break;
            default:
            {
                res.message = "the failed mode, ret = " + std::to_string(res.ret);
            }
        }

        return res.ret >= 0;
    }

    bool XArmDriver::SetStateCB(xarm_msgs::SetInt16::Request& req, xarm_msgs::SetInt16::Response& res)
    {
        res.ret = arm->set_state(req.data);
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

        return res.ret >= 0;
    }

    bool XArmDriver::SetTCPOffsetCB(xarm_msgs::TCPOffset::Request &req, xarm_msgs::TCPOffset::Response &res)
    {
        float offsets[6] = {req.x, req.y, req.z, req.roll, req.pitch, req.yaw};
        res.ret = arm->set_tcp_offset(offsets);
        if (res.ret >= 0)
            arm->save_conf();
        res.message = "set tcp offset: ret = " + std::to_string(res.ret); 
        return res.ret >= 0;
    }

    bool XArmDriver::SetLoadCB(xarm_msgs::SetLoad::Request &req, xarm_msgs::SetLoad::Response &res)
    {   
        float Mass = req.mass;
        float CoM[3] = {req.xc, req.yc, req.zc};
        res.ret = arm->set_tcp_load(Mass, CoM);
        if (res.ret >= 0)
            arm->save_conf();
        res.message = "set load: ret = " + std::to_string(res.ret); 
        return res.ret >= 0;
    }

    bool XArmDriver::SetControllerDOutCB(xarm_msgs::SetDigitalIO::Request &req, xarm_msgs::SetDigitalIO::Response &res)
    {
        if(req.io_num>=1 && req.io_num<=16)
        {
            res.ret = arm->set_cgpio_digital(req.io_num-1, req.value);
            res.message = "set Controller digital Output "+ std::to_string(req.io_num) +" to "+ std::to_string(req.value) + " : ret = " + std::to_string(res.ret); 
            return res.ret >= 0;
        }
        ROS_WARN("Controller Digital IO io_num: from 1 to 16");
        return false;
    }

    bool XArmDriver::GetControllerDInCB(xarm_msgs::GetControllerDigitalIO::Request &req, xarm_msgs::GetControllerDigitalIO::Response &res)
    {
        if(req.io_num>=1 && req.io_num<=16)
        {
            int all_status;
            int digitals[8];
            int *digitals2 = NULL;
            if (req.io_num > 8)
                digitals2 = new int[8];
            res.ret = arm->get_cgpio_digital(digitals, digitals2);
            if (req.io_num > 8) {
                res.value = digitals2[req.io_num - 9];
                delete[] digitals2;
            }
            else
                res.value = digitals[req.io_num - 1];
            res.message = "get Controller digital Input ret = " + std::to_string(res.ret);
            return res.ret >= 0;
        }
        ROS_WARN("Controller Digital IO io_num: from 1 to 8");
        return false;
    }

    bool XArmDriver::GetControllerAInCB(xarm_msgs::GetAnalogIO::Request &req, xarm_msgs::GetAnalogIO::Response &res)
    {
        switch (req.port_num)
        {
            case 1:
            case 2:
                res.ret = arm->get_cgpio_analog(req.port_num-1, &res.analog_value);
                break;
            default:
                res.message = "GetAnalogIO Fail: port number incorrect ! Must be 1 or 2";
                return false;
        }
        res.message = "get controller analog port " + std::to_string(req.port_num) + ", ret = " + std::to_string(res.ret); 
        return res.ret >= 0;
    }

    bool XArmDriver::SetControllerAOutCB(xarm_msgs::SetControllerAnalogIO::Request &req, xarm_msgs::SetControllerAnalogIO::Response &res)
    {
        switch (req.port_num)
        {
            case 1:
            case 2:
                res.ret = arm->set_cgpio_analog(req.port_num-1, req.analog_value);
                break;
            default:
                res.message = "SetAnalogIO Fail: port number incorrect ! Must be 1 or 2";
                return false;
        }
        res.message = "Set controller analog port " + std::to_string(req.port_num) + ", ret = " + std::to_string(res.ret); 
        return res.ret >= 0;
    }

    bool XArmDriver::SetDigitalIOCB(xarm_msgs::SetDigitalIO::Request &req, xarm_msgs::SetDigitalIO::Response &res)
    {
        res.ret = arm->set_tgpio_digital(req.io_num-1, req.value);
        res.message = "set Digital port "+ std::to_string(req.io_num) +" to "+ std::to_string(req.value) + " : ret = " + std::to_string(res.ret); 
        return res.ret >= 0;
    }

    bool XArmDriver::GetDigitalIOCB(xarm_msgs::GetDigitalIO::Request &req, xarm_msgs::GetDigitalIO::Response &res)
    {
        res.ret = arm->get_tgpio_digital(&res.digital_1, &res.digital_2);
        res.message = "get Digital port ret = " + std::to_string(res.ret); 
        return res.ret >= 0;
    }

    bool XArmDriver::GetAnalogIOCB(xarm_msgs::GetAnalogIO::Request &req, xarm_msgs::GetAnalogIO::Response &res)
    {
        switch (req.port_num)
        {
            case 1:
            case 2:
                res.ret = arm->get_tgpio_analog(req.port_num-1, &res.analog_value);
                break;
            default:
                res.message = "GetAnalogIO Fail: port number incorrect ! Must be 1 or 2";
                return false;
        }
        res.message = "get tool analog port " + std::to_string(req.port_num) + ", ret = " + std::to_string(res.ret); 
        return res.ret >= 0;
    }

    bool XArmDriver::SetModbusCB(xarm_msgs::SetToolModbus::Request &req, xarm_msgs::SetToolModbus::Response &res)
    {
        int send_len = req.send_data.size();
        int recv_len = req.respond_len;
        unsigned char * tx_data = new unsigned char [send_len]{0};
        unsigned char * rx_data = new unsigned char [recv_len]{0};

        for(int i=0; i<send_len; i++)
        {
            tx_data[i] = req.send_data[i];
        }
        res.ret = arm->getset_tgpio_modbus_data(tx_data, send_len, rx_data, recv_len);
        for(int i=0; i<recv_len; i++)
        {
           res.respond_data.push_back(rx_data[i]);
        }

        delete [] tx_data;
        delete [] rx_data;
        return res.ret >= 0;
    }

    bool XArmDriver::ConfigModbusCB(xarm_msgs::ConfigToolModbus::Request &req, xarm_msgs::ConfigToolModbus::Response &res)
    {
        res.message = "";
        if(curr_err_)
        {
            arm->set_state(XARM_STATE::START);
            ROS_WARN("Cleared Existing Error Code %d", curr_err_);
        }

        int ret = arm->set_tgpio_modbus_baudrate(req.baud_rate);
        int ret2 = arm->set_tgpio_modbus_timeout(req.timeout_ms);
        res.ret = ret == 0 ? ret2 : ret;
        res.message = "set_modbus_baudrate, ret="+ std::to_string(ret);
        res.message += (std::string(" | set_modbus_timeout, ret=") + std::to_string(ret2));

        return res.ret >= 0;
    }

    bool XArmDriver::GoHomeCB(xarm_msgs::Move::Request &req, xarm_msgs::Move::Response &res)
    {
        res.ret = arm->move_gohome(req.mvvelo, req.mvacc, req.mvtime, _get_wait_param());
        res.message = "go home, ret = " + std::to_string(res.ret);
        return res.ret >= 0;
    }

    bool XArmDriver::MoveJointCB(xarm_msgs::Move::Request &req, xarm_msgs::Move::Response &res)
    {
        float joint[7]={0};
        int index = 0;
        if(req.pose.size() != dof_)
        {
            res.ret = req.pose.size();
            res.message = "pose parameters incorrect! Expected: "+std::to_string(dof_);
            return false;
        }
        else
        {
            for(index = 0; index < 7; index++) // should always send 7 joint commands, whatever current DOF is.
            {
                // joint[0][index] = req.pose[index];
                if(index<req.pose.size())
                    joint[index] = req.pose[index];
                else
                    joint[index] = 0;
            }
        }
        res.ret = arm->set_servo_angle(joint, req.mvvelo, req.mvacc, req.mvtime, _get_wait_param());
        res.message = "move joint, ret = " + std::to_string(res.ret);
        return res.ret >= 0;
    }

    bool XArmDriver::MoveLineCB(xarm_msgs::Move::Request &req, xarm_msgs::Move::Response &res)
    {
        float pose[6];
        int index = 0;
        if(req.pose.size() != 6)
        {
            res.ret = -1;
            res.message = "number of parameters incorrect!";
            return false;
        }
        else
        {
            for(index = 0; index < 6; index++)
            {
                pose[index] = req.pose[index];
            }
        }
        res.ret = arm->set_position(pose, -1, req.mvvelo, req.mvacc, req.mvtime, _get_wait_param());
        res.message = "move line, ret = " + std::to_string(res.ret);
        return res.ret >= 0;
    }

    bool XArmDriver::MoveLineToolCB(xarm_msgs::Move::Request &req, xarm_msgs::Move::Response &res)
    {
        float pose[6];
        int index = 0;
        if(req.pose.size() != 6)
        {
            res.ret = -1;
            res.message = "number of parameters incorrect!";
            return false;
        }
        else
        {
            for(index = 0; index < 6; index++)
            {
                pose[index] = req.pose[index];
            }
        }
        res.ret = arm->set_tool_position(pose, req.mvvelo, req.mvacc, req.mvtime, _get_wait_param());
        res.message = "move line tool, ret = " + std::to_string(res.ret);
        return res.ret >= 0;
    }

    bool XArmDriver::MoveLinebCB(xarm_msgs::Move::Request &req, xarm_msgs::Move::Response &res)
    {
        float pose[6];
        int index = 0;
        if(req.pose.size() != 6)
        {
            res.ret = -1;
            res.message = "number of parameters incorrect!";
            return false;
        }
        else
        {
            for(index = 0; index < 6; index++)
            {
                pose[index] = req.pose[index];
            }
        }
        float mvradii = req.mvradii >= 0 ? req.mvradii : 0;
        res.ret = arm->set_position(pose, mvradii, req.mvvelo, req.mvacc, req.mvtime);        
        res.message = "move lineb, ret = " + std::to_string(res.ret);
        return res.ret >= 0;
    }

    bool XArmDriver::MoveJointbCB(xarm_msgs::Move::Request &req, xarm_msgs::Move::Response &res)
    {
        float joint[7]={0};
        int index = 0;
        if(req.pose.size() != dof_)
        {
            res.ret = req.pose.size();
            res.message = "number of joint parameters incorrect! Expected: "+std::to_string(dof_);
            return false;
        }
        else
        {
            for(index = 0; index < 7; index++) // should always send 7 joint commands, whatever current DOF is.
            {
                if(index<req.pose.size())
                    joint[index] = req.pose[index];
                else
                    joint[index] = 0;
            }
        }
        float mvradii = req.mvradii >= 0 ? req.mvradii : 0;
        res.ret = arm->set_servo_angle(joint, req.mvvelo, req.mvacc, req.mvtime, _get_wait_param(), 0, mvradii);
        res.message = "move jointB, ret = " + std::to_string(res.ret);
        return res.ret >= 0;
    }

    bool XArmDriver::MoveServoJCB(xarm_msgs::Move::Request &req, xarm_msgs::Move::Response &res)
    {
        float pose[7]={0};
        int index = 0;
        if(req.pose.size() != dof_)
        {
            res.ret = req.pose.size();
            res.message = "pose parameters incorrect! Expected: "+std::to_string(dof_);
            return false;
        }
        else
        {
            for(index = 0; index < 7; index++) // should always send 7 joint commands, whatever current DOF is.
            {
                if(index<req.pose.size())
                    pose[index] = req.pose[index];
                else
                    pose[index] = 0;
            }
        }

        res.ret = arm->set_servo_angle_j(pose, req.mvvelo, req.mvacc, req.mvtime);
        res.message = "move servoj, ret = " + std::to_string(res.ret);
        return res.ret >= 0;
    }

    bool XArmDriver::MoveServoCartCB(xarm_msgs::Move::Request &req, xarm_msgs::Move::Response &res)
    {
        float pose[6];
        int index = 0;
        if(req.pose.size() != 6)
        {
            res.ret = -1;
            res.message = "MoveServoCartCB parameters incorrect!";
            return false;
        }
        else
        {
            for(index = 0; index < 6; index++)
            {
                pose[index] = req.pose[index];
            }
        }

        res.ret = arm->set_servo_cartesian(pose, req.mvvelo, req.mvacc, req.mvtime);
        res.message = "move servo_cartesian, ret = " + std::to_string(res.ret);
        return res.ret >= 0;
    }

    bool XArmDriver::MoveLineAACB(xarm_msgs::MoveAxisAngle::Request &req, xarm_msgs::MoveAxisAngle::Response &res)
    {
        float pose[6];
        int index = 0;
        if(req.pose.size() != 6)
        {
            res.ret = -1;
            res.message = "MoveServoCartCB parameters incorrect!";
            return false;
        }
        else
        {
            for(index = 0; index < 6; index++)
            {
                pose[index] = req.pose[index];
            }
        }
        res.ret = arm->set_position_aa(pose, req.mvvelo, req.mvacc, req.mvtime, req.coord, req.relative, _get_wait_param());
        res.message = "move_line_aa, ret = " + std::to_string(res.ret);
        return res.ret >= 0;
    }

    bool XArmDriver::MoveServoCartAACB(xarm_msgs::MoveAxisAngle::Request &req, xarm_msgs::MoveAxisAngle::Response &res)
    {
        float pose[6];
        int index = 0;
        if(req.pose.size() != 6)
        {
            res.ret = -1;
            res.message = "MoveServoCartAACB parameters incorrect!";
            return false;
        }
        else
        {
            for(index = 0; index < 6; index++)
            {
                pose[index] = req.pose[index];
            }
        }
        res.ret = arm->set_servo_cartesian_aa(pose, req.mvvelo, req.mvacc, req.coord, req.relative);
        res.message = "move_servo_cart_aa, ret = " + std::to_string(res.ret);
        return res.ret >= 0;
    }

    bool XArmDriver::VeloMoveJointCB(xarm_msgs::MoveVelo::Request &req, xarm_msgs::MoveVelo::Response &res)
    {
        float jnt_v[7]={0};
        int index = 0;
        if(req.velocities.size() < dof_)
        {
            res.ret = req.velocities.size();
            res.message = "pose parameters incorrect! Expected: "+std::to_string(dof_);
            return false;
        }
        else
        {
            for(index = 0; index < 7; index++) // should always send 7 joint commands, whatever current DOF is.
            {
                // jnt_v[0][index] = req.velocities[index];
                if(index < req.velocities.size())
                    jnt_v[index] = req.velocities[index];
                else
                    jnt_v[index] = 0;
            }
        }

        res.ret = arm->vc_set_joint_velocity(jnt_v, req.jnt_sync);
        res.message = "velocity move joint, ret = " + std::to_string(res.ret);
        return res.ret >= 0;
    }

    bool XArmDriver::VeloMoveLineVCB(xarm_msgs::MoveVelo::Request &req, xarm_msgs::MoveVelo::Response &res)
    {
        float line_v[6];
        int index = 0;
        if(req.velocities.size() < 6)
        {
            res.ret = -1;
            res.message = "number of parameters incorrect!";
            return false;
        }
        else
        {
            for(index = 0; index < 6; index++)
            {
                line_v[index] = req.velocities[index];
            }
        }

        res.ret = arm->vc_set_cartesian_velocity(line_v, req.coord);
        res.message = "velocity move line, ret = " + std::to_string(res.ret);
        return res.ret >= 0;
    }

    bool XArmDriver::SetMaxJAccCB(xarm_msgs::SetFloat32::Request &req, xarm_msgs::SetFloat32::Response &res)
    {
        if(req.data<0 || req.data>20.0)
        {
            res.ret = -1;
            res.message = "set max joint acc: " + std::to_string(req.data) + "error! Proper range is: 0-20.0 rad/s^2";
            return false;
        }
        res.ret = arm->set_joint_maxacc(req.data);
        res.message = "set max joint acc: " + std::to_string(req.data) + " ret = " + std::to_string(res.ret);
        return res.ret >= 0;
    }

    bool XArmDriver::SetMaxLAccCB(xarm_msgs::SetFloat32::Request &req, xarm_msgs::SetFloat32::Response &res)
    {
        if(req.data<0 || req.data>50000.0)
        {
            res.ret = -1;
            res.message = "set max linear acc: " + std::to_string(req.data) + "error! Proper range is: 0-50000.0 mm/s^2";
            return false;
        }
        res.ret = arm->set_tcp_maxacc(req.data);
        res.message = "set max linear acc: " + std::to_string(req.data) + " ret = " + std::to_string(res.ret);
        return res.ret >= 0;
    }

    bool XArmDriver::GripperConfigCB(xarm_msgs::GripperConfig::Request &req, xarm_msgs::GripperConfig::Response &res)
    {
        if(req.pulse_vel>5000)
            req.pulse_vel = 5000;
        else if(req.pulse_vel<0)
            req.pulse_vel = 0;

        
        int ret1 = arm->set_gripper_mode(0);
        int ret2 = arm->set_gripper_enable(true);
        int ret3 = arm->set_gripper_speed(req.pulse_vel);

        res.ret = (ret1 == 0 && ret2 == 0) ? ret3 : (ret1 == 0 && ret3 == 0) ? ret2 : ret1;
        res.message = "gripper_config, ret = " + std::to_string(res.ret);
        return res.ret >= 0;
    }

    bool XArmDriver::GripperMoveCB(xarm_msgs::GripperMove::Request &req, xarm_msgs::GripperMove::Response &res)
    {
        if(req.pulse_pos>850)
            req.pulse_pos = 850;
        else if(req.pulse_pos<-100)
            req.pulse_pos = -100;

        res.ret = arm->set_gripper_position(req.pulse_pos);
        res.message = "gripper_move, ret = " + std::to_string(res.ret);
        return res.ret >= 0;
    }

    bool XArmDriver::GripperStateCB(xarm_msgs::GripperState::Request &req, xarm_msgs::GripperState::Response &res)
    {   
        int err_code = 0;
        float pos_now = 0;
        int ret1 = arm->get_gripper_err_code(&err_code);
        int ret2 = arm->get_gripper_position(&pos_now);

        res.ret = ret1 == 0 ? ret2 : ret1;
        res.err_code = err_code;
        res.curr_pos = pos_now;
        // fprintf(stderr, "gripper_pos: %f, gripper_err: %d\n", res.curr_pos, res.err_code);
        return res.ret >= 0;
    }

    bool XArmDriver::VacuumGripperCB(xarm_msgs::SetInt16::Request &req, xarm_msgs::SetInt16::Response &res)
    {
        res.ret = arm->set_vacuum_gripper(req.data);
        res.message = "set vacuum gripper: " + std::to_string(req.data) + " ret = " + std::to_string(res.ret);
        return res.ret >= 0;
    }

    void XArmDriver::pub_robot_msg(xarm_msgs::RobotMsg &rm_msg)
    {
        curr_err_ = rm_msg.err;
        curr_state_ = rm_msg.state;
        robot_rt_state_.publish(rm_msg);
    }
    
    void XArmDriver::pub_joint_state(sensor_msgs::JointState &js_msg)
    {
        joint_state_.publish(js_msg);
    }

    void XArmDriver::pub_cgpio_state(xarm_msgs::CIOState &cio_msg)
    {
        cgpio_state_.publish(cio_msg);
    }
}
