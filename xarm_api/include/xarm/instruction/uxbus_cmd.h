/* Copyright 2017 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jimy Zhang <jimy92@163.com>
 ============================================================================*/
#ifndef CORE_INSTRUCTION_UXBUS_CMD_H_
#define CORE_INSTRUCTION_UXBUS_CMD_H_

#include <mutex>
#include "xarm/common/data_type.h"

class UxbusCmd {
public:
	UxbusCmd(void);
	~UxbusCmd(void);

	int get_version(unsigned char rx_data[40]);
	int get_robot_sn(unsigned char rx_data[40]);
	int check_verification(int *rx_data);
	int shutdown_system(int value);
	int set_record_traj(int value);
	int save_traj(char filename[81]);
	int load_traj(char filename[81]);
	int playback_traj(int times, int spdx = 1);
	int playback_traj_old(int times);
	int get_traj_rw_status(int *rx_data);
	int set_reduced_mode(int on_off);
	int set_reduced_linespeed(float lspd_mm);
	int set_reduced_jointspeed(float jspd_rad);
	int get_reduced_mode(int *rx_data);
	int get_reduced_states(int *on, int xyz_list[6], float *tcp_speed, float *joint_speed, float jrange_rad[14] = NULL, int *fense_is_on = NULL, int *collision_rebound_is_on = NULL, int length = 21);
	int set_xyz_limits(int xyz_list[6]);
	int set_world_offset(float pose_offset[6]);
	int cnter_reset(void);
	int cnter_plus(void);
	int set_reduced_jrange(float jrange_rad[14]);
	int set_fense_on(int on_off);
	int set_collis_reb(int on_off);
	int motion_en(int id, int value);
	int set_state(int value);
	int get_state(int *rx_data);
	int get_cmdnum(int *rx_data);
	int get_err_code(int *rx_data);
	int get_hd_types(int *rx_data);
	int reload_dynamics(void);
	int clean_err(void);
	int clean_war(void);
	int set_brake(int axis, int en);
	int set_mode(int value);
	int move_line(float mvpose[6], float mvvelo, float mvacc, float mvtime);
	int move_lineb(float mvpose[6], float mvvelo, float mvacc, float mvtime,
		float mvradii);
	int move_joint(float mvjoint[7], float mvvelo, float mvacc, float mvtime);
	int move_line_tool(float mvpose[6], float mvvelo, float mvacc, float mvtime);
	int move_gohome(float mvvelo, float mvacc, float mvtime);
	int move_servoj(float mvjoint[7], float mvvelo, float mvacc, float mvtime);
	int move_servo_cartesian(float mvpose[6], float mvvelo, float mvacc, float mvtime);
	int set_servot(float jnt_taus[7]);
	int get_joint_tau(float jnt_taus[7]);
	int set_safe_level(int level);
	int get_safe_level(int *level);
	int sleep_instruction(float sltime);
	int move_circle(float pose1[6], float pose2[6], float mvvelo, float mvacc, float mvtime, float percent);
	int set_tcp_jerk(float jerk);
	int set_tcp_maxacc(float maxacc);
	int set_joint_jerk(float jerk);
	int set_joint_maxacc(float maxacc);
	int set_tcp_offset(float pose_offset[6]);
	int set_tcp_load(float mass, float load_offset[3]);
	int set_collis_sens(int value);
	int set_teach_sens(int value);
	int set_gravity_dir(float gravity_dir[3]);
	int clean_conf(void);
	int save_conf(void);

	int get_tcp_pose(float pose[6]);
	int get_joint_pose(float angles[7]);
	int get_ik(float pose[6], float angles[7]);
	int get_fk(float angles[7], float pose[6]);
	int is_joint_limit(float joint[7], int *value);
	int is_tcp_limit(float pose[6], int *value);
	int gripper_addr_w16(int addr, float value);
	int gripper_addr_r16(int addr, float *value);
	int gripper_addr_w32(int addr, float value);
	int gripper_addr_r32(int addr, float *value);
	int gripper_set_en(int value);
	int gripper_set_mode(int value);
	int gripper_set_zero(void);
	int gripper_get_pos(float *pulse);
	int gripper_set_pos(float pulse);
	int gripper_set_posspd(float speed);
	int gripper_get_errcode(int rx_data[2]);
	int gripper_clean_err(void);

	int tgpio_addr_w16(int addr, float value);
	int tgpio_addr_r16(int addr, float *value);
	int tgpio_addr_w32(int addr, float value);
	int tgpio_addr_r32(int addr, float *value);
	int tgpio_get_digital(int *io1, int *io2);
	int tgpio_set_digital(int ionum, int value);
	int tgpio_get_analog1(float *value);
	int tgpio_get_analog2(float *value);

	int set_modbus_timeout(int value);
	int set_modbus_baudrate(int baud);
	int tgpio_set_modbus(unsigned char *send_data, int length, unsigned char *recv_data);
	int gripper_modbus_w16s(int addr, float value, int len);
	int gripper_modbus_r16s(int addr, int len, unsigned char *rx_data);
	int gripper_modbus_set_en(int value);
	int gripper_modbus_set_mode(int value);
	int gripper_modbus_set_zero(void);
	int gripper_modbus_get_pos(float *pulse);
	int gripper_modbus_set_pos(float pulse);
	int gripper_modbus_set_posspd(float speed);
	int gripper_modbus_get_errcode(int *err);
	int gripper_modbus_clean_err(void);

	int servo_set_zero(int id);
	int servo_get_dbmsg(int rx_data[16]);
	int servo_addr_w16(int id, int addr, float value);
	int servo_addr_r16(int id, int addr, float *value);
	int servo_addr_w32(int id, int addr, float value);
	int servo_addr_r32(int id, int addr, float *value);


	int cgpio_get_auxdigit(int *value);
	int cgpio_get_analog1(float *value);
	int cgpio_get_analog2(float *value);
	int cgpio_set_auxdigit(int ionum, int value);
	int cgpio_set_analog1(int value);
	int cgpio_set_analog2(int value);
	int cgpio_set_infun(int ionum, int fun);
	int cgpio_set_outfun(int ionum, int fun);
	int cgpio_get_state(int *state, int *digit_io, float *analog, int *input_conf, int *output_conf);

	int get_pose_offset(float pose1[6], float pose2[6], float offset[6], int orient_type_in=0, int orient_type_out=0);
	int get_position_aa(float pose[6]);
	int move_line_aa(float mvpose[6], float mvvelo, float mvacc, float mvtime, int mvcoord=0, int relative=0);
	int move_servo_cart_aa(float mvpose[6], float mvvelo, float mvacc, int tool_coord=0, int relative=0);

	int tgpio_delay_set_digital(int ionum, int value, float delay_sec);
	int cgpio_delay_set_digital(int ionum, int value, float delay_sec);
	int tgpio_position_set_digital(int ionum, int value, float xyz[3], float tol_r);
	int cgpio_position_set_digital(int ionum, int value, float xyz[3], float tol_r);
	int config_io_stop_reset(int io_type, int val);

	virtual void close(void);

private:
	virtual int check_xbus_prot(unsigned char *data, int funcode);
	virtual int send_pend(int funcode, int num, int timeout, unsigned char *rx_data);
	virtual int send_xbus(int funcode, unsigned char *txdata, int num);
	int set_nu8(int funcode, int *datas, int num);
	int get_nu8(int funcode, int *rx_data, int num);
	int get_nu8(int funcode, unsigned char *rx_data, int num);
	int set_nu16(int funcode, int *datas, int num);
	int get_nu16(int funcode, int *rx_data, int num);
	int set_nfp32(int funcode, float *datas, int num);
	int set_nint32(int funcode, int *datas, int num);
	int get_nfp32(int funcode, float *rx_data, int num);
	int swop_nfp32(int funcode, float tx_datas[], int txn, float *rx_data, int rxn);
	int is_nfp32(int funcode, float datas[], int txn, int *value);
	int set_nfp32_with_bytes(int funcode, float *datas, int num, char *additional, int n);

private:
	std::mutex mutex_;
};
#endif
