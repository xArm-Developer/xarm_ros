#ifndef __XARM_ROS_CLIENT_H__
#define __XARM_ROS_CLIENT_H__

#include "ros/ros.h"
#include "xarm_api/xarm_msgs.h"
#include "xarm_api/visibility_control.h"

namespace xarm_api
{

class XARM_API_PUBLIC XArmROSClient
{
public:
  XArmROSClient();
  void init(ros::NodeHandle& nh);
  ~XArmROSClient();

  int motionEnable(short en);
  int setState(short state);
  int setMode(short mode);
  int clearErr(void);
  int getErr(void);
  int setTCPOffset(const std::vector<float>& tcp_offset);
  int setLoad(float mass, const std::vector<float>& center_of_mass);
  int setServoJ(const std::vector<float>& joint_cmd);
  int setServoCartisian(const std::vector<float>& cart_cmd);
  int goHome(float jnt_vel_rad, float jnt_acc_rad=15);
  int moveJoint(const std::vector<float>& joint_cmd, float jnt_vel_rad, float jnt_acc_rad=15);
  int moveLine(const std::vector<float>& cart_cmd, float cart_vel_mm, float cart_acc_mm=500);
  int moveLineB(int num_of_pnts, const std::vector<float> cart_cmds[], float cart_vel_mm, float cart_acc_mm=500, float radii=0);
  int getGripperState(float *curr_pulse, int *curr_err);
  int gripperConfig(float pulse_vel);
  int gripperMove(float pulse);
  
  int config_tool_modbus(int baud_rate, int time_out_ms);
  int send_tool_modbus(unsigned char* data, int send_len, unsigned char* recv_data=NULL, int recv_len=0);

  int veloMoveJoint(const std::vector<float>& jnt_v, bool is_sync = true, float duration = -1.0);
  int veloMoveLine(const std::vector<float>& line_v, bool is_tool_coord = false, float duration = -1.0);

  int trajRecord(short on);
  int trajSave(std::string filename, float timeout = 10);
  int trajPlay(std::string filename, int times = 1, int double_speed = 1, bool wait = false);

  int setCollisionRebound(bool on);
  int setCollSens(int sens);
  int setTeachSens(int sens);

  int setWorldOffset(const std::vector<float>& world_offset);
  int setFenceMode(bool on);
  int setReducedMode(bool on);
  int setTcpJerk(float jerk);
  int setJointJerk(float jerk);
  int setTcpMaxAcc(float maxacc);
  int setJointMaxAcc(float maxacc);
  int getServoAngle(std::vector<float>& angles);
  int getPositionRPY(std::vector<float>& pos);
  int getPositionAxisAngle(std::vector<float>& pos);

private:
  template<typename ServiceSrv>
  int _call_service(ros::ServiceClient &client, ServiceSrv &srv);

private:
  ros::ServiceClient motion_ctrl_client_;
  ros::ServiceClient set_mode_client_;
  ros::ServiceClient set_state_client_;
  ros::ServiceClient go_home_client_;
  ros::ServiceClient move_lineb_client_;
  ros::ServiceClient move_servoj_client_;
  ros::ServiceClient move_servo_cart_client_;
  ros::ServiceClient move_line_client_;
  ros::ServiceClient move_joint_client_;
  ros::ServiceClient set_tcp_offset_client_;
  ros::ServiceClient set_load_client_;
  ros::ServiceClient clear_err_client_;
  ros::ServiceClient get_err_client_;
  ros::ServiceClient config_modbus_client_;
  ros::ServiceClient send_modbus_client_;
  ros::ServiceClient gripper_move_client_;
  ros::ServiceClient gripper_config_client_;
  ros::ServiceClient gripper_state_client_;
  ros::ServiceClient velo_move_joint_client_;
  ros::ServiceClient velo_move_line_client_;
  ros::ServiceClient traj_record_client_;
  ros::ServiceClient traj_save_client_;
  ros::ServiceClient traj_play_client_;
  ros::ServiceClient set_coll_rebound_client_;
  ros::ServiceClient set_coll_sens_client_;
  ros::ServiceClient set_teach_sens_client_;

  ros::ServiceClient set_world_offset_client_;
  ros::ServiceClient set_fence_mode_client_;
  ros::ServiceClient set_reduced_mode_client_;
  ros::ServiceClient set_tcp_jerk_client_;
  ros::ServiceClient set_joint_jerk_client_;
  ros::ServiceClient set_tcp_maxacc_client_;
  ros::ServiceClient set_joint_maxacc_client_;
  ros::ServiceClient get_servo_angle_client_;
  ros::ServiceClient get_position_rpy_client_;
  ros::ServiceClient get_position_aa_client_;

  xarm_msgs::SetAxis set_axis_srv_;
  xarm_msgs::SetInt16 set_int16_srv_;
  xarm_msgs::TCPOffset offset_srv_;
  xarm_msgs::SetLoad set_load_srv_;
  xarm_msgs::ClearErr clear_err_srv_;
  xarm_msgs::GetErr get_err_srv_;
  xarm_msgs::Move move_srv_;
  xarm_msgs::Move servoj_msg_;
  xarm_msgs::Move servo_cart_msg_;
  xarm_msgs::ConfigToolModbus cfg_modbus_msg_;
  xarm_msgs::SetToolModbus set_modbus_msg_;
  xarm_msgs::GripperConfig gripper_config_msg_;
  xarm_msgs::GripperMove gripper_move_msg_;
  xarm_msgs::GripperState gripper_state_msg_;
  xarm_msgs::MoveVelocity move_velo_srv_;
  xarm_msgs::SetString set_string_srv_;
  xarm_msgs::PlayTraj play_traj_srv_;
  xarm_msgs::SetFloat32 set_float32_srv_;
  xarm_msgs::GetFloat32List get_float32_list_srv_;

  ros::NodeHandle nh_;
};

}

#endif
