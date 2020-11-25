/* Copyright 2018 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jason Peng <jason@ufactory.cc>
 ============================================================================*/
#include <xarm_ros_client.h>

namespace xarm_api{

void XArmROSClient::init(ros::NodeHandle& nh)
{   
    nh_ = nh;
    std::string client_ns = nh_.getNamespace() + "/";
    ros::service::waitForService(client_ns+"motion_ctrl");
    ros::service::waitForService(client_ns+"set_state");
    ros::service::waitForService(client_ns+"set_mode");
    ros::service::waitForService(client_ns+"move_servoj");

	motion_ctrl_client_ = nh_.serviceClient<xarm_msgs::SetAxis>("motion_ctrl");
	set_mode_client_ = nh_.serviceClient<xarm_msgs::SetInt16>("set_mode");
	set_state_client_ = nh_.serviceClient<xarm_msgs::SetInt16>("set_state");
    set_tcp_offset_client_ = nh_.serviceClient<xarm_msgs::TCPOffset>("set_tcp_offset");
    set_load_client_ = nh_.serviceClient<xarm_msgs::SetLoad>("set_load");
    clear_err_client_ = nh_.serviceClient<xarm_msgs::ClearErr>("clear_err");
    get_err_client_ = nh_.serviceClient<xarm_msgs::GetErr>("get_err");
  	go_home_client_ = nh_.serviceClient<xarm_msgs::Move>("go_home");
	move_lineb_client_ = nh_.serviceClient<xarm_msgs::Move>("move_lineb");
    move_line_client_ = nh_.serviceClient<xarm_msgs::Move>("move_line");
    move_joint_client_ = nh_.serviceClient<xarm_msgs::Move>("move_joint");
	move_servoj_client_ = nh_.serviceClient<xarm_msgs::Move>("move_servoj",true); // persistent connection for servoj
    move_servo_cart_client_ = nh_.serviceClient<xarm_msgs::Move>("move_servo_cart",true); // persistent connection for servo_cartesian

    //xarm gripper:
    gripper_move_client_ = nh_.serviceClient<xarm_msgs::GripperMove>("gripper_move");
    gripper_config_client_ = nh_.serviceClient<xarm_msgs::GripperConfig>("gripper_config");
    gripper_state_client_ = nh_.serviceClient<xarm_msgs::GripperState>("gripper_state");

    //tool modbus:
    config_modbus_client_ = nh_.serviceClient<xarm_msgs::ConfigToolModbus>("config_tool_modbus");
    send_modbus_client_ = nh_.serviceClient<xarm_msgs::SetToolModbus>("set_tool_modbus");
}

int XArmROSClient::motionEnable(short en)
{
	set_axis_srv_.request.id = 8;
    set_axis_srv_.request.data = en;
    if(motion_ctrl_client_.call(set_axis_srv_))
    {
        ROS_INFO("%s\n", set_axis_srv_.response.message.c_str());
        return set_axis_srv_.response.ret;
    }
    else
    {
        ROS_ERROR("Failed to call service motion_ctrl");
        return 1;
    }

}

int XArmROSClient::setState(short state)
{
	set_int16_srv_.request.data = state;
    if(set_state_client_.call(set_int16_srv_))
    {
        ROS_INFO("%s\n", set_int16_srv_.response.message.c_str());
        return set_int16_srv_.response.ret;
    }
    else
    {
        ROS_ERROR("Failed to call service set_state");
        return 1;
    }
}

int XArmROSClient::setMode(short mode)
{
	set_int16_srv_.request.data = mode;
    if(set_mode_client_.call(set_int16_srv_))
    {
        ROS_INFO("%s\n", set_int16_srv_.response.message.c_str());
        return set_int16_srv_.response.ret;
    }
    else
    {
        ROS_ERROR("Failed to call service set_mode");
        return 1;
    }  

}

int XArmROSClient::clearErr()
{
    if(clear_err_client_.call(clear_err_srv_))
    {
        ROS_INFO("%s\n", clear_err_srv_.response.message.c_str());
        return clear_err_srv_.response.ret;
    }
    else
    {
        ROS_ERROR("Failed to call service clear_err");
        return 1;
    }

}

int XArmROSClient::getErr()
{
    if(get_err_client_.call(get_err_srv_))
    {
        ROS_INFO("%s\n", get_err_srv_.response.message.c_str());
        return get_err_srv_.response.err;
    }
    else
    {
        ROS_ERROR("Failed to call service get_err");
        return 1;
    }

}

int XArmROSClient::setServoJ(const std::vector<float>& joint_cmd)
{
	servoj_msg_.request.mvvelo = 0;
    servoj_msg_.request.mvacc = 0;
    servoj_msg_.request.mvtime = 0;
    servoj_msg_.request.pose = joint_cmd;

    if(move_servoj_client_.call(servoj_msg_))
    {
        // ROS_INFO("%s\n", servoj_msg_.response.message.c_str());
        return servoj_msg_.response.ret;
    }
    else
    {
        ROS_ERROR("Failed to call service move_servoj");
        return 1;
    }
}

int XArmROSClient::setServoCartisian(const std::vector<float>& cart_cmd)
{
    servo_cart_msg_.request.mvvelo = 0;
    servo_cart_msg_.request.mvacc = 0;
    servo_cart_msg_.request.mvtime = 0;
    servo_cart_msg_.request.pose = cart_cmd;


    if(move_servo_cart_client_.call(servo_cart_msg_))
    {
        // ROS_INFO("%s\n", servo_cart_msg_.response.message.c_str());
        return servo_cart_msg_.response.ret;
    }
    else
    {
        ROS_ERROR("Failed to call service move_servo_cart");
        return 1;
    }
}

int XArmROSClient::setTCPOffset(const std::vector<float>& tcp_offset)
{
    if(tcp_offset.size() != 6)
    {
        ROS_ERROR("Set tcp offset service parameter should be 6-element Cartesian offset!");
        return 1;
    }
    
    offset_srv_.request.x = tcp_offset[0];
    offset_srv_.request.y = tcp_offset[1];
    offset_srv_.request.z = tcp_offset[2];
    offset_srv_.request.roll = tcp_offset[3];
    offset_srv_.request.pitch = tcp_offset[4];
    offset_srv_.request.yaw = tcp_offset[5];
    
    if(set_tcp_offset_client_.call(offset_srv_))
    {
       return offset_srv_.response.ret;
    }
    else
    {
        ROS_ERROR("Failed to call service set_tcp_offset");
        return 1;
    }

}

int XArmROSClient::setLoad(float mass, const std::vector<float>& center_of_mass)
{
    set_load_srv_.request.mass = mass;
    set_load_srv_.request.xc = center_of_mass[0];
    set_load_srv_.request.yc = center_of_mass[1];
    set_load_srv_.request.zc = center_of_mass[2];

    if(set_load_client_.call(set_load_srv_))
    {
       return set_load_srv_.response.ret;
    }
    else
    {
        ROS_ERROR("Failed to call service set_load");
        return 1;
    }
}

int XArmROSClient::goHome(float jnt_vel_rad, float jnt_acc_rad)
{
    move_srv_.request.mvvelo = jnt_vel_rad;
    move_srv_.request.mvacc = jnt_acc_rad;
    move_srv_.request.mvtime = 0;

    if(go_home_client_.call(move_srv_))
    {
        return move_srv_.response.ret;
    }
    else
    {
        ROS_ERROR("Failed to call service go_home");
        return 1;
    }
}

int XArmROSClient::moveJoint(const std::vector<float>& joint_cmd, float jnt_vel_rad, float jnt_acc_rad)
{
    move_srv_.request.mvvelo = jnt_vel_rad;
    move_srv_.request.mvacc = jnt_acc_rad;
    move_srv_.request.mvtime = 0;
    move_srv_.request.pose = joint_cmd;

    if(move_joint_client_.call(move_srv_))
    {
        return move_srv_.response.ret;
    }
    else
    {
        ROS_ERROR("Failed to call service move_joint");
        return 1;
    }
}

int XArmROSClient::moveLine(const std::vector<float>& cart_cmd, float cart_vel_mm, float cart_acc_mm)
{
    move_srv_.request.mvvelo = cart_vel_mm;
    move_srv_.request.mvacc = cart_acc_mm;
    move_srv_.request.mvtime = 0;
    move_srv_.request.pose = cart_cmd;

    if(move_line_client_.call(move_srv_))
    {
        return move_srv_.response.ret;
    }
    else
    {
        ROS_ERROR("Failed to call service move_line");
        return 1;
    }
}

int XArmROSClient::moveLineB(int num_of_pnts, const std::vector<float> cart_cmds[], float cart_vel_mm, float cart_acc_mm, float radii)
{
    move_srv_.request.mvvelo = cart_vel_mm;
    move_srv_.request.mvacc = cart_acc_mm;
    move_srv_.request.mvtime = 0;
    move_srv_.request.mvradii = radii;
    
    for(int i=0; i<num_of_pnts; i++)
    {
        move_srv_.request.pose = cart_cmds[i];

        if(move_lineb_client_.call(move_srv_))
        {
            if(move_srv_.response.ret)
                return 1; // move_lineb() returns non-zero value.
        }
        else
        {
            ROS_ERROR("Failed to call service move_lineb");
            return 1;
        }
    }

    return 0;
}

int XArmROSClient::config_tool_modbus(int baud_rate, int time_out_ms)
{
    cfg_modbus_msg_.request.baud_rate = baud_rate;
    cfg_modbus_msg_.request.timeout_ms = time_out_ms;
    if(config_modbus_client_.call(cfg_modbus_msg_))
    {
        return cfg_modbus_msg_.response.ret;
    }
    else
    {
        ROS_ERROR("Failed to call service config_tool_modbus");
        return 1;
    }
}

int XArmROSClient::send_tool_modbus(unsigned char* data, int send_len, unsigned char* recv_data, int recv_len)
{
    for(int i=0; i<send_len; i++)
    {
        set_modbus_msg_.request.send_data.push_back(data[i]);
    }

    set_modbus_msg_.request.respond_len = recv_len;

    if(send_modbus_client_.call(set_modbus_msg_))
    {   
        if(recv_len)
        {
           for(int j=0; j<recv_len; j++)
           {
             recv_data[j] = set_modbus_msg_.response.respond_data[j];
           }
        }

        set_modbus_msg_.request.send_data.clear();
        set_modbus_msg_.response.respond_data.clear();
        
        return 0;
    }
    else
    {
        ROS_ERROR("Failed to call service send_tool_modbus");
        set_modbus_msg_.request.send_data.clear();
        set_modbus_msg_.response.respond_data.clear();
        return 1;
    }
}

int XArmROSClient::gripperMove(float pulse)
{
    gripper_move_msg_.request.pulse_pos = pulse;
    if(gripper_move_client_.call(gripper_move_msg_))
    {
        ROS_INFO("gripper_move: %f\n", pulse);
        return gripper_move_msg_.response.ret;
    }
    else
    {
        ROS_ERROR("Failed to call service gripper_move");
        return 1;
    }

}

int XArmROSClient::gripperConfig(float pulse_vel)
{
    gripper_config_msg_.request.pulse_vel = pulse_vel;
    if(gripper_config_client_.call(gripper_config_msg_))
    {
        // ROS_INFO("gripper_vel: %f\n", pulse_vel);
        return gripper_config_msg_.response.ret;
    }
    else
    {
        ROS_ERROR("Failed to call service gripper_move");
        return 1;
    }

}

int XArmROSClient::getGripperState(float *curr_pulse, int *curr_err)
{
    if(gripper_state_client_.call(gripper_state_msg_))
    {
        *curr_pulse = gripper_state_msg_.response.curr_pos;
        *curr_err = gripper_state_msg_.response.err_code;
        return 0;
    }
    else
    {
        ROS_ERROR("Failed to call service gripper_move");
        return 1;
    }

}

}// namespace xarm_api
