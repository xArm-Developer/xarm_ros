/* Copyright 2018 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jason Peng <jason@ufactory.cc>
 ============================================================================*/
#include <xarm_api/xarm_ros_client.h>

#define SERVICE_CALL_FAILED 999
#define SERVICE_IS_PERSISTENT_BUT_INVALID 998
#define PARAMS_ERROR 997

namespace xarm_api {

XArmROSClient::XArmROSClient(){};
XArmROSClient::~XArmROSClient(){};

void XArmROSClient::init(ros::NodeHandle& nh)
{   
  nh_ = nh;
  std::string client_ns = nh_.getNamespace() + "/";
  ros::service::waitForService(client_ns+"motion_ctrl");
  ros::service::waitForService(client_ns+"set_state");
  ros::service::waitForService(client_ns+"set_mode");
  ros::service::waitForService(client_ns+"move_servoj");
  ros::service::waitForService(client_ns+"get_servo_angle");
  // ros::service::waitForService(client_ns+"controller_gpio_states"); // last one in driver

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

  // velocity control
  velo_move_joint_client_ = nh_.serviceClient<xarm_msgs::MoveVelocity>("velo_move_joint_timed",true);
  velo_move_line_client_ = nh_.serviceClient<xarm_msgs::MoveVelocity>("velo_move_line_timed");

  traj_record_client_ = nh_.serviceClient<xarm_msgs::SetInt16>("set_recording");
  traj_save_client_ = nh_.serviceClient<xarm_msgs::SetString>("save_traj");
  traj_play_client_ = nh_.serviceClient<xarm_msgs::PlayTraj>("play_traj");

  set_coll_rebound_client_ = nh_.serviceClient<xarm_msgs::SetInt16>("set_collision_rebound");
  set_coll_sens_client_ = nh_.serviceClient<xarm_msgs::SetInt16>("set_collision_sensitivity");
  set_teach_sens_client_ = nh_.serviceClient<xarm_msgs::SetInt16>("set_teach_sensitivity");

  set_world_offset_client_ = nh_.serviceClient<xarm_msgs::TCPOffset>("set_world_offset");
  set_fence_mode_client_ = nh_.serviceClient<xarm_msgs::SetInt16>("set_fence_mode");
  set_reduced_mode_client_ = nh_.serviceClient<xarm_msgs::SetInt16>("set_reduced_mode");
  set_tcp_jerk_client_ = nh_.serviceClient<xarm_msgs::SetFloat32>("set_tcp_jerk");
  set_joint_jerk_client_ = nh_.serviceClient<xarm_msgs::SetFloat32>("set_joint_jerk");
  set_tcp_maxacc_client_ = nh_.serviceClient<xarm_msgs::SetFloat32>("set_tcp_maxacc");
  set_joint_maxacc_client_ = nh_.serviceClient<xarm_msgs::SetFloat32>("set_joint_maxacc");
  get_servo_angle_client_ = nh_.serviceClient<xarm_msgs::GetFloat32List>("get_servo_angle",true);
  get_position_rpy_client_ = nh_.serviceClient<xarm_msgs::GetFloat32List>("get_position_rpy");
  get_position_aa_client_ = nh_.serviceClient<xarm_msgs::GetFloat32List>("get_position_axis_angle");
}

template<typename ServiceSrv>
int XArmROSClient::_call_service(ros::ServiceClient &client, ServiceSrv &srv)
{
  if (client.isPersistent() && !client.isValid()) return SERVICE_IS_PERSISTENT_BUT_INVALID;
  if(client.call(srv))
  {
    // ROS_INFO("call service %s, ret=%d, message=%s", 
    //   client.getService().c_str(), srv.response.ret, srv.response.message.c_str());
    return srv.response.ret;
  }
  else
  {
    ROS_ERROR("Failed to call service %s", client.getService().c_str());
    return SERVICE_CALL_FAILED;
  }
}

int XArmROSClient::motionEnable(short en)
{
  set_axis_srv_.request.id = 8;
  set_axis_srv_.request.data = en;
  return _call_service(motion_ctrl_client_, set_axis_srv_);
}

int XArmROSClient::setState(short state)
{
  set_int16_srv_.request.data = state;
  return _call_service(set_state_client_, set_int16_srv_);
}

int XArmROSClient::setMode(short mode)
{
  set_int16_srv_.request.data = mode;
  return _call_service(set_mode_client_, set_int16_srv_);
}

int XArmROSClient::clearErr()
{
  return _call_service(clear_err_client_, clear_err_srv_);
}

int XArmROSClient::getErr()
{
  int ret = _call_service(get_err_client_, get_err_srv_);
  if (ret != SERVICE_CALL_FAILED)
    return get_err_srv_.response.err;
  return ret;
}

int XArmROSClient::setServoJ(const std::vector<float>& joint_cmd)
{
  servoj_msg_.request.mvvelo = 0;
  servoj_msg_.request.mvacc = 0;
  servoj_msg_.request.mvtime = 0;
  servoj_msg_.request.pose = joint_cmd;
  return _call_service(move_servoj_client_, servoj_msg_);
}

int XArmROSClient::setServoCartisian(const std::vector<float>& cart_cmd)
{
  servo_cart_msg_.request.mvvelo = 0;
  servo_cart_msg_.request.mvacc = 0;
  servo_cart_msg_.request.mvtime = 0;
  servo_cart_msg_.request.pose = cart_cmd;
  return _call_service(move_servo_cart_client_, servo_cart_msg_);
}

int XArmROSClient::setTCPOffset(const std::vector<float>& tcp_offset)
{
  if(tcp_offset.size() != 6)
  {
    ROS_ERROR("Set tcp offset service parameter should be 6-element Cartesian offset!");
    return PARAMS_ERROR;
  }
  
  offset_srv_.request.x = tcp_offset[0];
  offset_srv_.request.y = tcp_offset[1];
  offset_srv_.request.z = tcp_offset[2];
  offset_srv_.request.roll = tcp_offset[3];
  offset_srv_.request.pitch = tcp_offset[4];
  offset_srv_.request.yaw = tcp_offset[5];
  return _call_service(set_tcp_offset_client_, offset_srv_);
}

int XArmROSClient::setLoad(float mass, const std::vector<float>& center_of_mass)
{
  set_load_srv_.request.mass = mass;
  set_load_srv_.request.xc = center_of_mass[0];
  set_load_srv_.request.yc = center_of_mass[1];
  set_load_srv_.request.zc = center_of_mass[2];
  return _call_service(set_load_client_, set_load_srv_);
}

int XArmROSClient::goHome(float jnt_vel_rad, float jnt_acc_rad)
{
  move_srv_.request.mvvelo = jnt_vel_rad;
  move_srv_.request.mvacc = jnt_acc_rad;
  move_srv_.request.mvtime = 0;
  return _call_service(go_home_client_, move_srv_);
}

int XArmROSClient::moveJoint(const std::vector<float>& joint_cmd, float jnt_vel_rad, float jnt_acc_rad)
{
  move_srv_.request.mvvelo = jnt_vel_rad;
  move_srv_.request.mvacc = jnt_acc_rad;
  move_srv_.request.mvtime = 0;
  move_srv_.request.pose = joint_cmd;
  return _call_service(move_joint_client_, move_srv_);
}

int XArmROSClient::moveLine(const std::vector<float>& cart_cmd, float cart_vel_mm, float cart_acc_mm)
{
  move_srv_.request.mvvelo = cart_vel_mm;
  move_srv_.request.mvacc = cart_acc_mm;
  move_srv_.request.mvtime = 0;
  move_srv_.request.pose = cart_cmd;
  return _call_service(move_line_client_, move_srv_);
}

int XArmROSClient::moveLineB(int num_of_pnts, const std::vector<float> cart_cmds[], float cart_vel_mm, float cart_acc_mm, float radii)
{
  move_srv_.request.mvvelo = cart_vel_mm;
  move_srv_.request.mvacc = cart_acc_mm;
  move_srv_.request.mvtime = 0;
  move_srv_.request.mvradii = radii;
  
  int ret;
  for(int i=0; i<num_of_pnts; i++)
  {
    move_srv_.request.pose = cart_cmds[i];
    ret = _call_service(move_lineb_client_, move_srv_);
    if (ret != 0)
      return ret;
  }
  return 0;
}

int XArmROSClient::config_tool_modbus(int baud_rate, int time_out_ms)
{
  cfg_modbus_msg_.request.baud_rate = baud_rate;
  cfg_modbus_msg_.request.timeout_ms = time_out_ms;
  return _call_service(config_modbus_client_, cfg_modbus_msg_);
}

int XArmROSClient::send_tool_modbus(unsigned char* data, int send_len, unsigned char* recv_data, int recv_len)
{
  for(int i=0; i<send_len; i++)
  {
    set_modbus_msg_.request.send_data.push_back(data[i]);
  }

  set_modbus_msg_.request.respond_len = recv_len;

  int ret = _call_service(send_modbus_client_, set_modbus_msg_);
  if (ret != SERVICE_CALL_FAILED) {
    if (recv_len) {
      for(int j=0; j<recv_len; j++) {
        recv_data[j] = set_modbus_msg_.response.respond_data[j];
      }
    }
  }
  set_modbus_msg_.request.send_data.clear();
  set_modbus_msg_.response.respond_data.clear();
  return ret;
}

int XArmROSClient::gripperMove(float pulse)
{
  gripper_move_msg_.request.pulse_pos = pulse;
  return _call_service(gripper_move_client_, gripper_move_msg_);
}

int XArmROSClient::gripperConfig(float pulse_vel)
{
  gripper_config_msg_.request.pulse_vel = pulse_vel;
  return _call_service(gripper_config_client_, gripper_config_msg_);
}

int XArmROSClient::getGripperState(float *curr_pulse, int *curr_err)
{
  int ret = _call_service(gripper_state_client_, gripper_state_msg_);
  if (ret != SERVICE_CALL_FAILED) {
    *curr_pulse = gripper_state_msg_.response.curr_pos;
    *curr_err = gripper_state_msg_.response.err_code;
  }
  return ret;
}

int XArmROSClient::veloMoveJoint(const std::vector<float>& jnt_v, bool is_sync, float duration) 
{
  // move_velo_srv_.request.velocities = jnt_v;
  // move_velo_srv_.request.jnt_sync = is_sync ? 1 : 0;
  move_velo_srv_.request.speeds = jnt_v;
  move_velo_srv_.request.is_sync = is_sync;
  move_velo_srv_.request.duration = duration;
  return _call_service(velo_move_joint_client_, move_velo_srv_);
}

int XArmROSClient::veloMoveLine(const std::vector<float>& line_v, bool is_tool_coord, float duration)
{
  // move_velo_srv_.request.velocities = line_v;
  // move_velo_srv_.request.coord = is_tool_coord ? 1 : 0;
  move_velo_srv_.request.speeds = line_v;
  move_velo_srv_.request.is_tool_coord = is_tool_coord;
  move_velo_srv_.request.duration = duration;
  return _call_service(velo_move_line_client_, move_velo_srv_);
}

int XArmROSClient::trajRecord(short on)
{
  set_int16_srv_.request.data = on;
  return _call_service(traj_record_client_, set_int16_srv_);
}

int XArmROSClient::trajSave(std::string filename, float timeout)
{
  set_string_srv_.request.str_data = filename;
  set_string_srv_.request.timeout = timeout;
  return _call_service(traj_save_client_, set_string_srv_);
}

int XArmROSClient::trajPlay(std::string filename, int times, int double_speed, bool wait)
{
  play_traj_srv_.request.traj_file = filename;
  play_traj_srv_.request.repeat_times = times;
  play_traj_srv_.request.speed_factor = double_speed;
  return _call_service(traj_play_client_, play_traj_srv_);
}

int XArmROSClient::setCollisionRebound(bool on)
{
  set_int16_srv_.request.data = (int)on;
  return _call_service(set_coll_rebound_client_, set_int16_srv_);
}

int XArmROSClient::setCollSens(int sens)
{
  set_int16_srv_.request.data = sens;
  return _call_service(set_coll_sens_client_, set_int16_srv_);
}

int XArmROSClient::setTeachSens(int sens)
{
  set_int16_srv_.request.data = sens;
  return _call_service(set_teach_sens_client_, set_int16_srv_);
}

int XArmROSClient::setWorldOffset(const std::vector<float>& world_offset)
{
  if(world_offset.size() != 6)
  {
    ROS_ERROR("Set world offset service parameter should be 6-element Cartesian offset!");
    return PARAMS_ERROR;
  }
  
  offset_srv_.request.x = world_offset[0];
  offset_srv_.request.y = world_offset[1];
  offset_srv_.request.z = world_offset[2];
  offset_srv_.request.roll = world_offset[3];
  offset_srv_.request.pitch = world_offset[4];
  offset_srv_.request.yaw = world_offset[5];
  return _call_service(set_world_offset_client_, offset_srv_);
}

int XArmROSClient::setFenceMode(bool on)
{
  set_int16_srv_.request.data = (int)on;
  return _call_service(set_fence_mode_client_, set_int16_srv_);
}

int XArmROSClient::setReducedMode(bool on)
{
  set_int16_srv_.request.data = (int)on;
  return _call_service(set_reduced_mode_client_, set_int16_srv_);
}

int XArmROSClient::setTcpJerk(float jerk)
{
  set_float32_srv_.request.data = jerk;
  return _call_service(set_tcp_jerk_client_, set_float32_srv_);
}

int XArmROSClient::setJointJerk(float jerk)
{
  set_float32_srv_.request.data = jerk;
  return _call_service(set_joint_jerk_client_, set_float32_srv_);
}

int XArmROSClient::setTcpMaxAcc(float maxacc)
{
  set_float32_srv_.request.data = maxacc;
  return _call_service(set_tcp_maxacc_client_, set_float32_srv_);
}

int XArmROSClient::setJointMaxAcc(float maxacc)
{
  set_float32_srv_.request.data = maxacc;
  return _call_service(set_joint_maxacc_client_, set_float32_srv_);
}

int XArmROSClient::getServoAngle(std::vector<float>& angles)
{
  int ret = _call_service(get_servo_angle_client_, get_float32_list_srv_);
  angles.resize(7);
  angles.swap(get_float32_list_srv_.response.datas);
  get_float32_list_srv_.response.datas.clear();
  return ret;
}

int XArmROSClient::getPositionRPY(std::vector<float>& pos)
{
  int ret = _call_service(get_position_rpy_client_, get_float32_list_srv_);
  pos.resize(6);
  pos.swap(get_float32_list_srv_.response.datas);
  get_float32_list_srv_.response.datas.clear();
  return ret;
}

int XArmROSClient::getPositionAxisAngle(std::vector<float>& pos)
{
  int ret = _call_service(get_position_aa_client_, get_float32_list_srv_);
  pos.resize(6);
  pos.swap(get_float32_list_srv_.response.datas);
  get_float32_list_srv_.response.datas.clear();
  return ret;
}

}// namespace xarm_api
