/* Copyright 2018 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jason Peng <jason@ufactory.cc>
 ============================================================================*/
#include <xarm_driver.h>
#include "xarm/instruction/uxbus_cmd_config.h"
#include "xarm/linux/thread.h"

#define CMD_HEARTBEAT_US 3e7 // 30s

void* cmd_heart_beat(void* args)
{
    xarm_api::XARMDriver *my_driver = (xarm_api::XARMDriver *) args;
    while(true)
    {
        usleep(CMD_HEARTBEAT_US); // non-realtime
        my_driver->Heartbeat();
    }
    pthread_exit(0);
}

namespace xarm_api
{   
    XARMDriver::~XARMDriver()
    {   
        // pthread_cancel(thread_id_);  // heartbeat related
        arm_cmd_->set_mode(XARM_MODE::POSE);
        arm_cmd_->close();
        spinner.stop();
    }

    void XARMDriver::XARMDriverInit(ros::NodeHandle& root_nh, char *server_ip)
    {   
        nh_ = root_nh;
        nh_.getParam("DOF",dof_);

        arm_report_ = connext_tcp_report_norm(server_ip);
        // ReportDataNorm norm_data_;
        arm_cmd_ = connect_tcp_control(server_ip);  
        if (arm_cmd_ == NULL)
            ROS_ERROR("Xarm Connection Failed!");
        else // clear unimportant errors
        {
            // thread_id_ = thread_init(cmd_heart_beat, this); // heartbeat related
            int dbg_msg[16] = {0};
            arm_cmd_->servo_get_dbmsg(dbg_msg);

            for(int i=0; i<dof_; i++)
            {
                if((dbg_msg[i*2]==1)&&(dbg_msg[i*2+1]==40))
                {
                    arm_cmd_->clean_err();
                    ROS_WARN("Cleared low-voltage error of joint %d", i+1);
                }
                else if((dbg_msg[i*2]==1))
                {
                    arm_cmd_->clean_err();
                    ROS_WARN("There is servo error code:(0x%x) in joint %d, trying to clear it..", dbg_msg[i*2+1], i+1);
                }
            }

        }

        // api command services:
        motion_ctrl_server_ = nh_.advertiseService("motion_ctrl", &XARMDriver::MotionCtrlCB, this);
        set_mode_server_ = nh_.advertiseService("set_mode", &XARMDriver::SetModeCB, this);
        set_state_server_ = nh_.advertiseService("set_state", &XARMDriver::SetStateCB, this);
        set_tcp_offset_server_ = nh_.advertiseService("set_tcp_offset", &XARMDriver::SetTCPOffsetCB, this);
        set_load_server_ = nh_.advertiseService("set_load", &XARMDriver::SetLoadCB, this);

        go_home_server_ = nh_.advertiseService("go_home", &XARMDriver::GoHomeCB, this);
        move_joint_server_ = nh_.advertiseService("move_joint", &XARMDriver::MoveJointCB, this);
        move_lineb_server_ = nh_.advertiseService("move_lineb", &XARMDriver::MoveLinebCB, this);
        move_line_server_ = nh_.advertiseService("move_line", &XARMDriver::MoveLineCB, this);
        move_line_tool_server_ = nh_.advertiseService("move_line_tool", &XARMDriver::MoveLineToolCB, this);
        move_servoj_server_ = nh_.advertiseService("move_servoj", &XARMDriver::MoveServoJCB, this);
        move_servo_cart_server_ = nh_.advertiseService("move_servo_cart", &XARMDriver::MoveServoCartCB, this);        
        clear_err_server_ = nh_.advertiseService("clear_err", &XARMDriver::ClearErrCB, this);
        moveit_clear_err_server_ = nh_.advertiseService("moveit_clear_err", &XARMDriver::MoveitClearErrCB, this);
        get_err_server_ = nh_.advertiseService("get_err", &XARMDriver::GetErrCB, this);

        // tool io:
        set_end_io_server_ = nh_.advertiseService("set_digital_out", &XARMDriver::SetDigitalIOCB, this);
        get_digital_in_server_ = nh_.advertiseService("get_digital_in", &XARMDriver::GetDigitalIOCB, this);
        get_analog_in_server_ = nh_.advertiseService("get_analog_in", &XARMDriver::GetAnalogIOCB, this);
        config_modbus_server_ = nh_.advertiseService("config_tool_modbus", &XARMDriver::ConfigModbusCB, this);
        set_modbus_server_ = nh_.advertiseService("set_tool_modbus", &XARMDriver::SetModbusCB, this);

        gripper_config_server_ = nh_.advertiseService("gripper_config", &XARMDriver::GripperConfigCB, this);
        gripper_move_server_ = nh_.advertiseService("gripper_move", &XARMDriver::GripperMoveCB, this);
        gripper_state_server_ = nh_.advertiseService("gripper_state", &XARMDriver::GripperStateCB, this);

        set_vacuum_gripper_server_ = nh_.advertiseService("vacuum_gripper_set", &XARMDriver::VacuumGripperCB, this);

        // controller_io (digital):
        set_controller_dout_server_ = nh_.advertiseService("set_controller_dout", &XARMDriver::SetControllerDOutCB, this);
        get_controller_din_server_ = nh_.advertiseService("get_controller_din", &XARMDriver::GetControllerDInCB, this);

        // state feedback topics:
        joint_state_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 10, true);
        robot_rt_state_ = nh_.advertise<xarm_msgs::RobotMsg>("xarm_states", 10, true);
        // end_input_state_ = nh_.advertise<xarm_msgs::IOState>("xarm_input_states", 10, true);

        // subscribed topics
        sleep_sub_ = nh_.subscribe("sleep_sec", 1, &XARMDriver::SleepTopicCB, this);

    }

    void XARMDriver::Heartbeat(void)
    {   
        int cmd_num;
        int ret = arm_cmd_->get_cmdnum(&cmd_num);
        // if(ret)
        // {
        //     ROS_ERROR("xArm Heartbeat error! ret = %d", ret);
        // }
        // ROS_INFO("xArm Heartbeat! %d", cmd_num);
    }

    bool XARMDriver::isConnectionOK(void)
    {
        return !arm_report_->is_ok(); // is_ok will return 0 if connection is normal
    }

    void XARMDriver::SleepTopicCB(const std_msgs::Float32ConstPtr& msg)
    {
        if(msg->data>0)
            arm_cmd_->sleep_instruction(msg->data);
    }

    bool XARMDriver::ClearErrCB(xarm_msgs::ClearErr::Request& req, xarm_msgs::ClearErr::Response& res)
    {
        // First clear controller warning and error:
        int ret1 = arm_cmd_->gripper_modbus_clean_err();
        int ret2 = arm_cmd_->clean_war(); 
        int ret3 = arm_cmd_->clean_err();

        // Then try to enable motor again:
        res.ret = arm_cmd_->motion_en(8, 1);

        if(res.ret)
        {
            res.message = "clear err, ret = "  + std::to_string(res.ret);
        }
        return true;

        // After calling this service, user should check '/xarm_states' again to make sure 'err' field is 0, to confirm success.
    }

    bool XARMDriver::MoveitClearErrCB(xarm_msgs::ClearErr::Request& req, xarm_msgs::ClearErr::Response& res)
    {
        if(ClearErrCB(req, res))
        {
            arm_cmd_->set_mode(1);
            int ret = arm_cmd_->set_state(0);
            
            if(!ret)
                return true;
            return false;
        }
        return false;

        // After calling this service, user should check '/xarm_states' again to make sure 'err' field is 0, to confirm success.
    }
    
    bool XARMDriver::GetErrCB(xarm_msgs::GetErr::Request & req, xarm_msgs::GetErr::Response & res)
    {
        res.err = curr_err_;
        res.message = "current error code = "  + std::to_string(res.err);
        return true;
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
        res.ret = arm_cmd_->set_tcp_offset(offsets) | arm_cmd_->save_conf();
        res.message = "set tcp offset: ret = " + std::to_string(res.ret); 
        return true;
    }

    bool XARMDriver::SetLoadCB(xarm_msgs::SetLoad::Request &req, xarm_msgs::SetLoad::Response &res)
    {   
        float Mass = req.mass;
        float CoM[3] = {req.xc, req.yc, req.zc};
        res.ret = arm_cmd_->set_tcp_load(Mass, CoM) | arm_cmd_->save_conf();
        res.message = "set load: ret = " + std::to_string(res.ret); 
        return true;
    }

    bool XARMDriver::SetControllerDOutCB(xarm_msgs::SetDigitalIO::Request &req, xarm_msgs::SetDigitalIO::Response &res)
    {
        if(req.io_num>=1 && req.io_num<=8)
        {
            res.ret = arm_cmd_->cgpio_set_auxdigit(req.io_num-1, req.value);
            res.message = "set Controller digital Output "+ std::to_string(req.io_num) +" to "+ std::to_string(req.value) + " : ret = " + std::to_string(res.ret); 
            return true;
        }
        ROS_WARN("Controller IO io_num: from 1 to 8");
        return false;
    }

    bool XARMDriver::GetControllerDInCB(xarm_msgs::GetControllerDigitalIO::Request &req, xarm_msgs::GetControllerDigitalIO::Response &res)
    {
        if(req.io_num>=1 && req.io_num<=8)
        {
            int all_status;
            res.ret = arm_cmd_->cgpio_get_auxdigit(&all_status);
            res.value = (all_status >> (req.io_num - 1)) & 0x0001;
            res.message = "get Controller digital Input ret = " + std::to_string(res.ret);
            return true;
        }
        ROS_WARN("Controller IO io_num: from 1 to 8");
        return false;
    }

    bool XARMDriver::SetDigitalIOCB(xarm_msgs::SetDigitalIO::Request &req, xarm_msgs::SetDigitalIO::Response &res)
    {
        res.ret = arm_cmd_->tgpio_set_digital(req.io_num, req.value);
        res.message = "set Digital port "+ std::to_string(req.io_num) +" to "+ std::to_string(req.value) + " : ret = " + std::to_string(res.ret); 
        return true;
    }

    bool XARMDriver::GetDigitalIOCB(xarm_msgs::GetDigitalIO::Request &req, xarm_msgs::GetDigitalIO::Response &res)
    {
        res.ret = arm_cmd_->tgpio_get_digital(&res.digital_1, &res.digital_2);
        res.message = "get Digital port ret = " + std::to_string(res.ret); 
        return true;
    }

    bool XARMDriver::GetAnalogIOCB(xarm_msgs::GetAnalogIO::Request &req, xarm_msgs::GetAnalogIO::Response &res)
    {
        switch (req.port_num)
        {
            case 1:
                res.ret = arm_cmd_->tgpio_get_analog1(&res.analog_value);
                break;
            case 2:
                res.ret = arm_cmd_->tgpio_get_analog2(&res.analog_value);
                break;

            default:
                res.message = "GetAnalogIO Fail: port number incorrect !";
                return false;
        }
        res.message = "get analog port ret = " + std::to_string(res.ret); 
        return true;
    }

    bool XARMDriver::SetModbusCB(xarm_msgs::SetToolModbus::Request &req, xarm_msgs::SetToolModbus::Response &res)
    {
        int send_len = req.send_data.size();
        int recv_len = req.respond_len;
        unsigned char tx_data[send_len]={0}, rx_data[recv_len]={0};

        for(int i=0; i<send_len; i++)
        {
            tx_data[i] = req.send_data[i];
        }
        res.ret = arm_cmd_->tgpio_set_modbus(tx_data, send_len, rx_data);
        for(int i=0; i<recv_len; i++)
        {
           res.respond_data.push_back(rx_data[i]);
        }

        return true;
    }

    bool XARMDriver::ConfigModbusCB(xarm_msgs::ConfigToolModbus::Request &req, xarm_msgs::ConfigToolModbus::Response &res)
    {
        res.message = "";
        if(curr_err_)
        {
            arm_cmd_->set_state(0);
            ROS_WARN("Cleared Existing Error Code %d", curr_err_);
        }

        int ret = arm_cmd_->set_modbus_baudrate(req.baud_rate);
        if(ret)
        {
            res.message = "set baud_rate, ret = "+ std::to_string(ret);
            if(ret==1)
                res.message += " controller error exists, please check and run clear_err service first!";
        }
        ret = arm_cmd_->set_modbus_timeout(req.timeout_ms);
        if(ret)
        {
            res.message += (std::string(" set baud_rate, ret = ") + std::to_string(ret));
            if(ret==1)
                res.message += " controller error exists, please check and run clear_err service first!";
        }
        
        if(res.message.size())
            res.ret = -1;
        else
            res.message = "Modbus configuration OK";

        return true;
    }

    bool XARMDriver::GoHomeCB(xarm_msgs::Move::Request &req, xarm_msgs::Move::Response &res)
    {
        res.ret = arm_cmd_->move_gohome(req.mvvelo, req.mvacc, req.mvtime);
        if(!res.ret)
        {
            res.ret = wait_for_finish();
        }
        res.message = "go home, ret = " + std::to_string(res.ret);
        return true;
    }

    bool XARMDriver::MoveJointCB(xarm_msgs::Move::Request &req, xarm_msgs::Move::Response &res)
    {
        float joint[1][7]={0};
        int index = 0;
        if(req.pose.size() != dof_)
        {
            res.ret = req.pose.size();
            res.message = "pose parameters incorrect! Expected: "+std::to_string(dof_);
            return true;
        }
        else
        {
            for(index = 0; index < 7; index++) // should always send 7 joint commands, whatever current DOF is.
            {
                // joint[0][index] = req.pose[index];
                if(index<req.pose.size())
                    joint[0][index] = req.pose[index];
                else
                    joint[0][index] = 0;
            }
        }

        res.ret = arm_cmd_->move_joint(joint[0], req.mvvelo, req.mvacc, req.mvtime);
        if(!res.ret)
        {
            res.ret = wait_for_finish();
        }
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
        if(!res.ret)
        {
            res.ret = wait_for_finish();
        }
        res.message = "move line, ret = " + std::to_string(res.ret);
        return true;
    }

    bool XARMDriver::MoveLineToolCB(xarm_msgs::Move::Request &req, xarm_msgs::Move::Response &res)
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

        res.ret = arm_cmd_->move_line_tool(pose[0], req.mvvelo, req.mvacc, req.mvtime);
        if(!res.ret)
        {
            res.ret = wait_for_finish();
        }
        res.message = "move line tool, ret = " + std::to_string(res.ret);
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
        float pose[1][7]={0};
        int index = 0;
        if(req.pose.size() != dof_)
        {
            res.ret = req.pose.size();
            res.message = "pose parameters incorrect! Expected: "+std::to_string(dof_);
            return true;
        }
        else
        {
            for(index = 0; index < 7; index++) // should always send 7 joint commands, whatever current DOF is.
            {
                if(index<req.pose.size())
                    pose[0][index] = req.pose[index];
                else
                    pose[0][index] = 0;
            }
        }

        res.ret = arm_cmd_->move_servoj(pose[0], req.mvvelo, req.mvacc, req.mvtime);
        res.message = "move servoj, ret = " + std::to_string(res.ret);
        return true;
    }

    bool XARMDriver::MoveServoCartCB(xarm_msgs::Move::Request &req, xarm_msgs::Move::Response &res)
    {
        float pose[1][6];
        int index = 0;
        if(req.pose.size() != 6)
        {
            res.ret = -1;
            res.message = "MoveServoCartCB parameters incorrect!";
            return true;
        }
        else
        {
            for(index = 0; index < 6; index++)
            {
                pose[0][index] = req.pose[index];
            }
        }

        res.ret = arm_cmd_->move_servo_cartesian(pose[0], req.mvvelo, req.mvacc, req.mvtime);
        res.message = "move servo_cartesian, ret = " + std::to_string(res.ret);
        return true;
    }

    bool XARMDriver::GripperConfigCB(xarm_msgs::GripperConfig::Request &req, xarm_msgs::GripperConfig::Response &res)
    {
        if(req.pulse_vel>5000)
            req.pulse_vel = 5000;
        else if(req.pulse_vel<0)
            req.pulse_vel = 0;

        
        int ret1 = arm_cmd_->gripper_modbus_set_mode(0);
        int ret2 = arm_cmd_->gripper_modbus_set_en(1);
        int ret3 = arm_cmd_->gripper_modbus_set_posspd(req.pulse_vel);

        if(ret1 || ret2 || ret3)
        {
            res.ret = ret3;
        }
        else
        {
            res.ret = 0;
        }
        res.message = "gripper_config, ret = " + std::to_string(res.ret);
        return true;
    }

    bool XARMDriver::GripperMoveCB(xarm_msgs::GripperMove::Request &req, xarm_msgs::GripperMove::Response &res)
    {
        if(req.pulse_pos>850)
            req.pulse_pos = 850;
        else if(req.pulse_pos<-100)
            req.pulse_pos = -100;

        res.ret = arm_cmd_->gripper_modbus_set_pos(req.pulse_pos);
        res.message = "gripper_move, ret = " + std::to_string(res.ret);
        return true;
    }

    bool XARMDriver::GripperStateCB(xarm_msgs::GripperState::Request &req, xarm_msgs::GripperState::Response &res)
    {   
        int err_code = 0;
        float pos_now = 0;     
        if(arm_cmd_->gripper_modbus_get_errcode(&err_code))
            return false;

        if(arm_cmd_->gripper_modbus_get_pos(&pos_now))
            return false;
        
        res.err_code = err_code;
        res.curr_pos = pos_now;
        // fprintf(stderr, "gripper_pos: %f, gripper_err: %d\n", res.curr_pos, res.err_code);
        return true;
    }

    bool XARMDriver::VacuumGripperCB(xarm_msgs::SetInt16::Request &req, xarm_msgs::SetInt16::Response &res)
    {
        if(req.data)
        {
            res.ret = arm_cmd_->tgpio_set_digital(1, 1);
            res.ret = arm_cmd_->tgpio_set_digital(2, 0);
        }
        else
        {
            res.ret = arm_cmd_->tgpio_set_digital(1, 0);
            res.ret = arm_cmd_->tgpio_set_digital(2, 1);
        }
        res.message = "set vacuum gripper: " + std::to_string(req.data) + " ret = " + std::to_string(res.ret);
        return true;
    }

    void XARMDriver::pub_robot_msg(xarm_msgs::RobotMsg &rm_msg)
    {
        curr_err_ = rm_msg.err;
        curr_state_ = rm_msg.state;
        robot_rt_state_.publish(rm_msg);
    }
    
    void XARMDriver::pub_joint_state(sensor_msgs::JointState &js_msg)
    {
        joint_state_.publish(js_msg);
    }

    void XARMDriver::pub_io_state()
    {
        arm_cmd_->tgpio_get_digital(&io_msg.digital_1, &io_msg.digital_2);
        arm_cmd_->tgpio_get_analog1(&io_msg.analog_1);
        arm_cmd_->tgpio_get_analog2(&io_msg.analog_2);

        end_input_state_.publish(io_msg);
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

    int XARMDriver::wait_for_finish()
    {
        bool wait;
        int ret = 0;

        nh_.getParam("wait_for_finish", wait);

        if(!wait)
            return ret;

        ros::Duration(0.2).sleep(); // delay 0.2s, for 5Hz state update
        ros::Rate sleep_rate(10); // 10Hz

        while(curr_state_== 1) // in MOVE state
        {
            if(curr_err_)
            {
                ret = UXBUS_STATE::ERR_CODE;
                break;
            }
            sleep_rate.sleep();
        }

        if(!ret)
        {
            int err_warn[2] = {0};
            arm_cmd_->get_err_code(err_warn);
            if(err_warn[0])
            {
                ROS_ERROR("XARM ERROR CODE: %d ", err_warn[0]);
                ret = UXBUS_STATE::ERR_CODE;
            }
        }

        return ret;
    }

}
