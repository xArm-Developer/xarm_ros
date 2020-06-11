/* Copyright 2017 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jimy Zhang <jimy92@163.com>
 ============================================================================*/
#include <unistd.h>
#include "xarm/instruction/uxbus_cmd.h"
#include "xarm/instruction/servo3_config.h"
#include "xarm/instruction/uxbus_cmd_config.h"
#include "xarm/debug/debug_print.h"

static int BAUDRATES[13] = { 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600, 1000000, 1500000, 2000000, 2500000 };

static int get_baud_inx(int baud) {
	for (int i = 0; i < 13; i++) { if (BAUDRATES[i] == baud) return i; }
	return -1;
}

UxbusCmd::UxbusCmd(void) {}

UxbusCmd::~UxbusCmd(void) {}

int UxbusCmd::check_xbus_prot(unsigned char *data, int funcode) { return -11; }

int UxbusCmd::send_pend(int funcode, int num, int timeout, unsigned char *rx_data) {
	return -11;
}

int UxbusCmd::send_xbus(int funcode, unsigned char *txdata, int num) { return -11; }

void UxbusCmd::close(void) {}


/*******************************************************
 * Uxbus generic protocol function
 *******************************************************/

int UxbusCmd::set_nu8(int funcode, int *datas, int num) {
	std::lock_guard<std::mutex> locker(mutex_);
	//unsigned char send_data[num];
	unsigned char *send_data = new unsigned char[num];
	for (int i = 0; i < num; i++)
	{
		send_data[i] = (unsigned char)datas[i];
	}

	int ret = send_xbus(funcode, send_data, num);
	delete send_data;
	if (ret != 0) { return UXBUS_STATE::ERR_NOTTCP; }
	ret = send_pend(funcode, 0, funcode == UXBUS_RG::MOTION_EN ? UXBUS_CONF::SET_TIMEOUT * 2 : UXBUS_CONF::SET_TIMEOUT, NULL);
	return ret;
}

int UxbusCmd::get_nu8(int funcode, int *rx_data, int num) {
	// unsigned char secv_data[num];
	unsigned char *send_data = new unsigned char[num];
	int ret = get_nu8(funcode, send_data, num);

	for (int i = 0; i < num; i++) {
		rx_data[i] = send_data[i];
	}
	delete send_data;
	return ret;
}

int UxbusCmd::get_nu8(int funcode, unsigned char *rx_data, int num) {
	std::lock_guard<std::mutex> locker(mutex_);
	int ret = send_xbus(funcode, 0, 0);
	if (ret != 0) { return UXBUS_STATE::ERR_NOTTCP; }
	return send_pend(funcode, num, UXBUS_CONF::GET_TIMEOUT, rx_data);
}

int UxbusCmd::set_nu16(int funcode, int *datas, int num) {
	std::lock_guard<std::mutex> locker(mutex_);
	//unsigned char send_data[num * 2];
	unsigned char *send_data = new unsigned char[num * 2];
	for (int i = 0; i < num; i++) {
		bin16_to_8(datas[i], &send_data[i * 2]);
	}
	int ret = send_xbus(funcode, send_data, num * 2);
	delete send_data;
	if (ret != 0) { return UXBUS_STATE::ERR_NOTTCP; }
	ret = send_pend(funcode, 0, UXBUS_CONF::SET_TIMEOUT, NULL);

	return ret;
}
int UxbusCmd::get_nu16(int funcode, int *rx_data, int num) {
	std::lock_guard<std::mutex> locker(mutex_);
	//unsigned char datas[num * 2];
	unsigned char *datas = new unsigned char[num * 2];
	int ret = send_xbus(funcode, 0, 0);
	if (ret != 0) {
		delete datas;
		return UXBUS_STATE::ERR_NOTTCP;
	}

	ret = send_pend(funcode, num * 2, UXBUS_CONF::GET_TIMEOUT, datas);
	for (int i = 0; i < num; i++) {
		rx_data[i] = bin8_to_16(&datas[i * 2]);
	}
	delete datas;

	return ret;
}

int UxbusCmd::set_nfp32(int funcode, float *datas, int num) {
	std::lock_guard<std::mutex> locker(mutex_);
	// unsigned char hexdata[num * 4] = {0};
	unsigned char *hexdata = new unsigned char[num * 4];
	nfp32_to_hex(datas, hexdata, num);
	int ret = send_xbus(funcode, hexdata, num * 4);
	delete hexdata;
	if (0 != ret) { return UXBUS_STATE::ERR_NOTTCP; }
	ret = send_pend(funcode, 0, UXBUS_CONF::SET_TIMEOUT, NULL);

	return ret;
}

int UxbusCmd::set_nint32(int funcode, int *datas, int num) {
	std::lock_guard<std::mutex> locker(mutex_);
	//unsigned char hexdata[num * 4] = {0};  \\??
	unsigned char *hexdata = new unsigned char[num * 4];
	nint32_to_hex(datas, hexdata, num);
	int ret = send_xbus(funcode, hexdata, num * 4);
	delete hexdata;
	if (0 != ret) { return UXBUS_STATE::ERR_NOTTCP; }
	ret = send_pend(funcode, 0, UXBUS_CONF::SET_TIMEOUT, NULL);

	return ret;
}

int UxbusCmd::get_nfp32(int funcode, float *rx_data, int num) {
	std::lock_guard<std::mutex> locker(mutex_);
	//unsigned char datas[num * 4] = {0};
	unsigned char *datas = new unsigned char[num * 4];
	int ret = send_xbus(funcode, 0, 0);
	if (0 != ret) {
		delete datas;
		return UXBUS_STATE::ERR_NOTTCP;
	}
	ret = send_pend(funcode, num * 4, UXBUS_CONF::GET_TIMEOUT, datas);
	hex_to_nfp32(datas, rx_data, num);
	delete datas;
	return ret;
}

int UxbusCmd::swop_nfp32(int funcode, float tx_datas[], int txn, float *rx_data, int rxn) {
	std::lock_guard<std::mutex> locker(mutex_);
	// unsigned char hexdata[128] = { 0 };
	unsigned char *hexdata = new unsigned char[128];

	nfp32_to_hex(tx_datas, hexdata, txn);
	int ret = send_xbus(funcode, hexdata, txn * 4);
	if (0 != ret) { return UXBUS_STATE::ERR_NOTTCP; }
	ret = send_pend(funcode, rxn * 4, UXBUS_CONF::GET_TIMEOUT, hexdata);
	hex_to_nfp32(hexdata, rx_data, rxn);

	return ret;
}

int UxbusCmd::is_nfp32(int funcode, float datas[], int txn, int *value) {
	std::lock_guard<std::mutex> locker(mutex_);
	//unsigned char hexdata[txn * 4] = {0};
	unsigned char *hexdata = new unsigned char[txn * 4];

	nfp32_to_hex(datas, hexdata, txn);
	int ret = send_xbus(funcode, hexdata, txn * 4);
	if (0 != ret) {
		delete hexdata;
		return UXBUS_STATE::ERR_NOTTCP;
	}
	ret = send_pend(funcode, 1, UXBUS_CONF::GET_TIMEOUT, hexdata);
	*value = hexdata[0];
	delete hexdata;

	return ret;
}

int UxbusCmd::set_nfp32_with_bytes(int funcode, float *datas, int num, char *additional, int len) {
	std::lock_guard<std::mutex> locker(mutex_);
	unsigned char *hexdata = new unsigned char[num * 4 + len];
	nfp32_to_hex(datas, hexdata, num);
	for (int i = 0; i < len; i++) { hexdata[num * 4 + i] = additional[i]; }
	int ret = send_xbus(funcode, hexdata, num * 4 + len);
	delete hexdata;
	if (0 != ret) { return UXBUS_STATE::ERR_NOTTCP; }
	ret = send_pend(funcode, 0, UXBUS_CONF::SET_TIMEOUT, NULL);

	return ret;
}

/*******************************************************
 * controler setting
 *******************************************************/
int UxbusCmd::get_version(unsigned char rx_data[40]) {
	return get_nu8(UXBUS_RG::GET_VERSION, rx_data, 40);
}

int UxbusCmd::get_robot_sn(unsigned char rx_data[40]) {
	return get_nu8(UXBUS_RG::GET_ROBOT_SN, rx_data, 40);
}

int UxbusCmd::check_verification(int *rx_data) {
	return get_nu8(UXBUS_RG::CHECK_VERIFY, rx_data, 1);
}

int UxbusCmd::shutdown_system(int value) {
	return set_nu8(UXBUS_RG::SHUTDOWN_SYSTEM, &value, 1);
}

int UxbusCmd::set_record_traj(int value) {
	int txdata[1] = { value };
	return set_nu8(UXBUS_RG::SET_TRAJ_RECORD, txdata, 1);
}

int UxbusCmd::playback_traj(int times, int spdx) {
	int txdata[2] = { times, spdx };
	return set_nint32(UXBUS_RG::PLAY_TRAJ, txdata, 2);
}

int UxbusCmd::playback_traj_old(int times) {
	int txdata[1] = { times };
	return set_nint32(UXBUS_RG::PLAY_TRAJ, txdata, 1);
}

int UxbusCmd::save_traj(char filename[81]) {
	return set_nu8(UXBUS_RG::SAVE_TRAJ, (int *)filename, 81);
}

int UxbusCmd::load_traj(char filename[81]) {
	return set_nu8(UXBUS_RG::LOAD_TRAJ, (int *)filename, 81);
}

int UxbusCmd::get_traj_rw_status(int *rx_data) {
	return get_nu8(UXBUS_RG::GET_TRAJ_RW_STATUS, rx_data, 1);
}

int UxbusCmd::set_reduced_mode(int on_off) {
	int txdata[1] = { on_off };
	return set_nu8(UXBUS_RG::SET_REDUCED_MODE, txdata, 1);
}

int UxbusCmd::set_reduced_linespeed(float lspd_mm) {
	float txdata[1] = { lspd_mm };
	return set_nfp32(UXBUS_RG::SET_REDUCED_TRSV, txdata, 1);
}

int UxbusCmd::set_reduced_jointspeed(float jspd_rad) {
	float txdata[1] = { jspd_rad };
	return set_nfp32(UXBUS_RG::SET_REDUCED_P2PV, txdata, 1);
}

int UxbusCmd::get_reduced_mode(int *rx_data) {
	return get_nu8(UXBUS_RG::GET_REDUCED_MODE, rx_data, 1);
}

int UxbusCmd::get_reduced_states(int *on, int xyz_list[6], float *tcp_speed, float *joint_speed, float jrange_rad[14], int *fense_is_on, int *collision_rebound_is_on, int length) {
	//unsigned char rx_data[length] = {0};
	unsigned char *rx_data = new unsigned char[length];
	int ret = get_nu8(UXBUS_RG::GET_REDUCED_STATE, rx_data, length);
	*on = rx_data[0];
	bin8_to_ns16(&rx_data[1], xyz_list, 6);
	*tcp_speed = hex_to_fp32(&rx_data[13]);
	*joint_speed = hex_to_fp32(&rx_data[17]);
	if (length == 79) {
		if (jrange_rad != NULL) { hex_to_nfp32(&rx_data[21], jrange_rad, 14); }
		if (fense_is_on != NULL) { *fense_is_on = rx_data[77]; }
		if (collision_rebound_is_on != NULL) { *collision_rebound_is_on = rx_data[78]; }
	}
	delete rx_data;
	return ret;
}

int UxbusCmd::set_xyz_limits(int xyz_list[6]) {
	return set_nint32(UXBUS_RG::SET_LIMIT_XYZ, xyz_list, 6);
}

int UxbusCmd::set_world_offset(float pose_offset[6]) {
	return set_nfp32(UXBUS_RG::SET_WORLD_OFFSET, pose_offset, 6);
}

int UxbusCmd::cnter_reset(void) {
	return set_nu8(UXBUS_RG::CNTER_RESET, 0, 0);
}

int UxbusCmd::cnter_plus(void) {
	return set_nu8(UXBUS_RG::CNTER_PLUS, 0, 0);
}

int UxbusCmd::set_reduced_jrange(float jrange_rad[14]) {
	return set_nfp32(UXBUS_RG::SET_REDUCED_JRANGE, jrange_rad, 14);
}

int UxbusCmd::set_fense_on(int on_off) {
	int txdata[1] = { on_off };
	return set_nu8(UXBUS_RG::SET_FENSE_ON, txdata, 1);
}

int UxbusCmd::set_collis_reb(int on_off) {
	int txdata[1] = { on_off };
	return set_nu8(UXBUS_RG::SET_COLLIS_REB, txdata, 1);
}

int UxbusCmd::motion_en(int id, int value) {
	int txdata[2] = { id, value };
	return set_nu8(UXBUS_RG::MOTION_EN, txdata, 2);
}

int UxbusCmd::set_state(int value) {
	return set_nu8(UXBUS_RG::SET_STATE, &value, 1);
}

int UxbusCmd::get_state(int *rx_data) {
	return get_nu8(UXBUS_RG::GET_STATE, rx_data, 1);
}

int UxbusCmd::get_cmdnum(int *rx_data) {
	return get_nu16(UXBUS_RG::GET_CMDNUM, rx_data, 1);
}

int UxbusCmd::get_err_code(int * rx_data) {
	return get_nu8(UXBUS_RG::GET_ERROR, rx_data, 2);
}

int UxbusCmd::get_hd_types(int *rx_data) {
	return get_nu8(UXBUS_RG::GET_HD_TYPES, rx_data, 2);
}

int UxbusCmd::reload_dynamics(void) {
	int txdata[1] = { 0 };
	return set_nu8(UXBUS_RG::RELOAD_DYNAMICS, txdata, 0);
}

int UxbusCmd::clean_err(void) {
	int txdata[1] = { 0 };
	return set_nu8(UXBUS_RG::CLEAN_ERR, txdata, 0);
}

int UxbusCmd::clean_war(void) {
	int txdata[1] = { 0 };
	return set_nu8(UXBUS_RG::CLEAN_WAR, txdata, 0);
}

int UxbusCmd::set_brake(int axis, int en) {
	int txdata[2] = { axis, en };
	return set_nu8(UXBUS_RG::SET_BRAKE, txdata, 2);
}

int UxbusCmd::set_mode(int value) {
	int txdata[1] = { value };
	return set_nu8(UXBUS_RG::SET_MODE, txdata, 1);
}

/*******************************************************
 * controler motion
 *******************************************************/
int UxbusCmd::move_line(float mvpose[6], float mvvelo, float mvacc, float mvtime) {
	float txdata[9] = { 0 };
	for (int i = 0; i < 6; i++) { txdata[i] = mvpose[i]; }
	txdata[6] = mvvelo;
	txdata[7] = mvacc;
	txdata[8] = mvtime;
	return set_nfp32(UXBUS_RG::MOVE_LINE, txdata, 9);
}

int UxbusCmd::move_lineb(float mvpose[6], float mvvelo, float mvacc, float mvtime,
	float mvradii) {
	float txdata[10] = { 0 };
	for (int i = 0; i < 6; i++) { txdata[i] = mvpose[i]; }
	txdata[6] = mvvelo;
	txdata[7] = mvacc;
	txdata[8] = mvtime;
	txdata[9] = mvradii;

	return set_nfp32(UXBUS_RG::MOVE_LINEB, txdata, 10);
}

int UxbusCmd::move_joint(float mvjoint[7], float mvvelo, float mvacc,
	float mvtime) {
	float txdata[10] = { 0 };
	for (int i = 0; i < 7; i++) { txdata[i] = mvjoint[i]; }
	txdata[7] = mvvelo;
	txdata[8] = mvacc;
	txdata[9] = mvtime;
	return set_nfp32(UXBUS_RG::MOVE_JOINT, txdata, 10);
}

int UxbusCmd::move_line_tool(float mvpose[6], float mvvelo, float mvacc, float mvtime) {
	float txdata[9] = { 0 };
	for (int i = 0; i < 6; i++) { txdata[i] = mvpose[i]; }
	txdata[6] = mvvelo;
	txdata[7] = mvacc;
	txdata[8] = mvtime;
	return set_nfp32(UXBUS_RG::MOVE_LINE_TOOL, txdata, 9);
}

int UxbusCmd::move_gohome(float mvvelo, float mvacc, float mvtime) {
	float txdata[3] = { 0 };
	txdata[0] = mvvelo;
	txdata[1] = mvacc;
	txdata[2] = mvtime;
	return set_nfp32(UXBUS_RG::MOVE_HOME, txdata, 3);
}

int UxbusCmd::move_servoj(float mvjoint[7], float mvvelo, float mvacc, float mvtime) {
	float txdata[10] = { 0 };
	for (int i = 0; i < 7; i++) { txdata[i] = mvjoint[i]; }
	txdata[7] = mvvelo;
	txdata[8] = mvacc;
	txdata[9] = mvtime;
	return set_nfp32(UXBUS_RG::MOVE_SERVOJ, txdata, 10);
}

int UxbusCmd::move_servo_cartesian(float mvpose[6], float mvvelo, float mvacc, float mvtime) {
	float txdata[9] = { 0 };
	for (int i = 0; i < 6; i++) { txdata[i] = mvpose[i]; }
	txdata[6] = mvvelo;
	txdata[7] = mvacc;
	txdata[8] = mvtime;
	return set_nfp32(UXBUS_RG::MOVE_SERVO_CART, txdata, 9);
}

int UxbusCmd::set_servot(float jnt_taus[7]) {
	float txdata[7] = { 0 };
	for (int i = 0; i < 7; i++) { txdata[i] = jnt_taus[i]; }
	return set_nfp32(UXBUS_RG::SET_SERVOT, txdata, 7);
}

int UxbusCmd::get_joint_tau(float jnt_taus[7]) {
	return get_nfp32(UXBUS_RG::GET_JOINT_TAU, jnt_taus, 7);
}

int UxbusCmd::set_safe_level(int level) {
	int txdata[1] = { level };
	return set_nu8(UXBUS_RG::SET_SAFE_LEVEL, txdata, 1);
}

int UxbusCmd::get_safe_level(int *level) {
	return get_nu8(UXBUS_RG::GET_SAFE_LEVEL, level, 1);
}

int UxbusCmd::sleep_instruction(float sltime) {
	float txdata[1] = { sltime };
	return set_nfp32(UXBUS_RG::SLEEP_INSTT, txdata, 1);
}

int UxbusCmd::move_circle(float pose1[6], float pose2[6], float mvvelo, float mvacc, float mvtime, float percent) {

	float txdata[16] = { 0 };
	for (int i = 0; i < 6; i++) {
		txdata[i] = pose1[i];
		txdata[6 + i] = pose2[i];
	}
	txdata[12] = mvvelo;
	txdata[13] = mvacc;
	txdata[14] = mvtime;
	txdata[15] = percent;

	return set_nfp32(UXBUS_RG::MOVE_CIRCLE, txdata, 16);
}

int UxbusCmd::set_tcp_jerk(float jerk) {
	float txdata[1] = { jerk };
	return set_nfp32(UXBUS_RG::SET_TCP_JERK, txdata, 1);
}

int UxbusCmd::set_tcp_maxacc(float maxacc) {
	float txdata[1] = { maxacc };
	return set_nfp32(UXBUS_RG::SET_TCP_MAXACC, txdata, 1);
}

int UxbusCmd::set_joint_jerk(float jerk) {
	float txdata[1] = { jerk };
	return set_nfp32(UXBUS_RG::SET_JOINT_JERK, txdata, 1);
}

int UxbusCmd::set_joint_maxacc(float maxacc) {
	float txdata[1] = { maxacc };
	return set_nfp32(UXBUS_RG::SET_JOINT_MAXACC, txdata, 1);
}

int UxbusCmd::set_tcp_offset(float pose_offset[6]) {
	return set_nfp32(UXBUS_RG::SET_TCP_OFFSET, pose_offset, 6);
}

int UxbusCmd::set_tcp_load(float mass, float load_offset[3]) {
	float txdata[4] = { mass, load_offset[0], load_offset[1], load_offset[2] };
	return set_nfp32(UXBUS_RG::SET_LOAD_PARAM, txdata, 4);
}

int UxbusCmd::set_collis_sens(int value) {
	return set_nu8(UXBUS_RG::SET_TEACH_SENS, &value, 1);
}

int UxbusCmd::set_teach_sens(int value) {
	return set_nu8(UXBUS_RG::SET_TEACH_SENS, &value, 1);
}

int UxbusCmd::set_gravity_dir(float gravity_dir[3]) {
	return set_nfp32(UXBUS_RG::SET_GRAVITY_DIR, gravity_dir, 3);
}

int UxbusCmd::clean_conf() {
	return set_nu8(UXBUS_RG::CLEAN_CONF, 0, 0);
}

int UxbusCmd::save_conf() {
	return set_nu8(UXBUS_RG::SAVE_CONF, 0, 0);
}

int UxbusCmd::get_tcp_pose(float pose[6]) {
	return get_nfp32(UXBUS_RG::GET_TCP_POSE, pose, 6);
}

int UxbusCmd::get_joint_pose(float angles[7]) {
	return get_nfp32(UXBUS_RG::GET_JOINT_POS, angles, 7);
}

int UxbusCmd::get_ik(float pose[6], float angles[7]) {
	return swop_nfp32(UXBUS_RG::GET_IK, pose, 6, angles, 7);
}

int UxbusCmd::get_fk(float angles[7], float pose[6]) {
	return swop_nfp32(UXBUS_RG::GET_FK, angles, 7, pose, 6);
}

int UxbusCmd::is_joint_limit(float joint[7], int *value) {
	return is_nfp32(UXBUS_RG::IS_JOINT_LIMIT, joint, 7, value);
}

int UxbusCmd::is_tcp_limit(float pose[6], int *value) {
	return is_nfp32(UXBUS_RG::IS_TCP_LIMIT, pose, 6, value);
}

/*******************************************************
 * gripper
 *******************************************************/
int UxbusCmd::gripper_addr_w16(int addr, float value) {
	// unsigned char txdata[7];
	std::lock_guard<std::mutex> locker(mutex_);
	unsigned char *txdata = new unsigned char[7];
	txdata[0] = UXBUS_CONF::GRIPPER_ID;
	bin16_to_8(addr, &txdata[1]);
	fp32_to_hex(value, &txdata[3]);
	int ret = send_xbus(UXBUS_RG::TGPIO_W16B, txdata, 7);
	if (0 != ret) { return UXBUS_STATE::ERR_NOTTCP; }

	return send_pend(UXBUS_RG::TGPIO_W16B, 0, UXBUS_CONF::GET_TIMEOUT, NULL);
}

int UxbusCmd::gripper_addr_r16(int addr, float *value) {
	std::lock_guard<std::mutex> locker(mutex_);
	// unsigned char txdata[3], rx_data[4];
	unsigned char *txdata = new unsigned char[3];
	unsigned char *rx_data = new unsigned char[4];
	txdata[0] = UXBUS_CONF::GRIPPER_ID;
	bin16_to_8(addr, &txdata[1]);
	int ret = send_xbus(UXBUS_RG::TGPIO_R16B, txdata, 3);
	if (0 != ret) { return UXBUS_STATE::ERR_NOTTCP; }

	return send_pend(UXBUS_RG::TGPIO_R16B, 4, UXBUS_CONF::GET_TIMEOUT, rx_data);
	*value = (float)bin8_to_32(rx_data);
	return ret;
}

int UxbusCmd::gripper_addr_w32(int addr, float value) {
	std::lock_guard<std::mutex> locker(mutex_);
	// unsigned char txdata[7];
	unsigned char *txdata = new unsigned char[7];
	txdata[0] = UXBUS_CONF::GRIPPER_ID;
	bin16_to_8(addr, &txdata[1]);
	fp32_to_hex(value, &txdata[3]);
	int ret = send_xbus(UXBUS_RG::TGPIO_W32B, txdata, 7);
	if (0 != ret) { return UXBUS_STATE::ERR_NOTTCP; }

	return send_pend(UXBUS_RG::TGPIO_W32B, 0, UXBUS_CONF::GET_TIMEOUT, NULL);
}

int UxbusCmd::gripper_addr_r32(int addr, float *value) {
	std::lock_guard<std::mutex> locker(mutex_);
	// unsigned char txdata[3], rx_data[4];
	unsigned char *txdata = new unsigned char[3];
	unsigned char *rx_data = new unsigned char[4];
	txdata[0] = UXBUS_CONF::GRIPPER_ID;
	bin16_to_8(addr, &txdata[1]);
	int ret = send_xbus(UXBUS_RG::TGPIO_R32B, txdata, 3);
	if (0 != ret) { return UXBUS_STATE::ERR_NOTTCP; }

	ret = send_pend(UXBUS_RG::TGPIO_R32B, 4, UXBUS_CONF::GET_TIMEOUT, rx_data);
	*value = (float)bin8_to_32(rx_data);
	return ret;
}

int UxbusCmd::gripper_set_en(int value) {
	return gripper_addr_w16(SERVO3_RG::CON_EN, (float)value);
}

int UxbusCmd::gripper_set_mode(int value) {
	return gripper_addr_w16(SERVO3_RG::CON_MODE, (float)value);
}

int UxbusCmd::gripper_set_zero() {
	return gripper_addr_w16(SERVO3_RG::MT_ZERO, 1);
}

int UxbusCmd::gripper_get_pos(float *pulse) {
	return gripper_addr_r32(SERVO3_RG::CURR_POS, pulse);
}

int UxbusCmd::gripper_set_pos(float pulse) {
	return gripper_addr_w32(SERVO3_RG::TAGET_POS, pulse);
}

int UxbusCmd::gripper_set_posspd(float speed) {
	return gripper_addr_w16(SERVO3_RG::POS_SPD, speed);
}

int UxbusCmd::gripper_get_errcode(int rx_data[2]) {
	return get_nu8(UXBUS_RG::TGPIO_ERR, rx_data, 2);
}

int UxbusCmd::gripper_clean_err() {
	return gripper_addr_w16(SERVO3_RG::RESET_ERR, 1);
}

/*******************************************************
 * tool gpio
 *******************************************************/
int UxbusCmd::tgpio_addr_w16(int addr, float value) {
	// unsigned char txdata[7];
	unsigned char *txdata = new unsigned char[7];
	txdata[0] = UXBUS_CONF::TGPIO_ID;
	bin16_to_8(addr, &txdata[1]);
	fp32_to_hex(value, &txdata[3]);
	std::lock_guard<std::mutex> locker(mutex_);
	int ret = send_xbus(UXBUS_RG::TGPIO_W16B, txdata, 7);
	if (0 != ret) { return UXBUS_STATE::ERR_NOTTCP; }

	return send_pend(UXBUS_RG::TGPIO_W16B, 0, UXBUS_CONF::GET_TIMEOUT, NULL);
}

int UxbusCmd::tgpio_addr_r16(int addr, float *value) {
	// unsigned char txdata[3], rx_data[4];
	unsigned char *txdata = new unsigned char[3];
	unsigned char *rx_data = new unsigned char[4];
	txdata[0] = UXBUS_CONF::TGPIO_ID;
	bin16_to_8(addr, &txdata[1]);
	std::lock_guard<std::mutex> locker(mutex_);
	int ret = send_xbus(UXBUS_RG::TGPIO_R16B, txdata, 3);
	if (0 != ret) { return UXBUS_STATE::ERR_NOTTCP; }

	ret = send_pend(UXBUS_RG::TGPIO_R16B, 4, UXBUS_CONF::GET_TIMEOUT, rx_data);
	*value = (float)bin8_to_32(rx_data);
	return ret;
}
int UxbusCmd::tgpio_addr_w32(int addr, float value) {
	// unsigned char txdata[7];
	unsigned char *txdata = new unsigned char[7];
	txdata[0] = UXBUS_CONF::TGPIO_ID;
	bin16_to_8(addr, &txdata[1]);
	fp32_to_hex(value, &txdata[3]);
	std::lock_guard<std::mutex> locker(mutex_);
	int ret = send_xbus(UXBUS_RG::TGPIO_W32B, txdata, 7);
	if (0 != ret) { return UXBUS_STATE::ERR_NOTTCP; }

	return send_pend(UXBUS_RG::TGPIO_W32B, 0, UXBUS_CONF::GET_TIMEOUT, NULL);
}

int UxbusCmd::tgpio_addr_r32(int addr, float *value) {
	// unsigned char txdata[3], rx_data[4];
	unsigned char *txdata = new unsigned char[3];
	unsigned char *rx_data = new unsigned char[4];
	txdata[0] = UXBUS_CONF::TGPIO_ID;
	bin16_to_8(addr, &txdata[1]);
	std::lock_guard<std::mutex> locker(mutex_);
	int ret = send_xbus(UXBUS_RG::TGPIO_R32B, txdata, 3);
	if (0 != ret) { return UXBUS_STATE::ERR_NOTTCP; }

	return send_pend(UXBUS_RG::TGPIO_R32B, 4, UXBUS_CONF::GET_TIMEOUT, rx_data);
	*value = (float)bin8_to_32(rx_data);
	return ret;
}

int UxbusCmd::tgpio_get_digital(int *io1, int *io2) {
	float tmp;
	int ret = tgpio_addr_r16(SERVO3_RG::DIGITAL_IN, &tmp);

	*io1 = (int)tmp & 0x0001;
	*io2 = ((int)tmp & 0x0002) >> 1;
	return ret;
}

int UxbusCmd::tgpio_set_digital(int ionum, int value) {
	int tmp = 0;
	if (ionum == 1) {
		tmp = tmp | 0x0100;
		if (value)
		{
			tmp = tmp | 0x0001;
		}
	}
	else if (ionum == 2) {
		tmp = tmp | 0x0200;
		if (value)
		{
			tmp = tmp | 0x0002;
		}
	}
	else {
		return -1;
	}
	return tgpio_addr_w16(SERVO3_RG::DIGITAL_OUT, (float)tmp);
}

int UxbusCmd::tgpio_get_analog1(float * value) {
	float tmp;
	int ret = tgpio_addr_r16(SERVO3_RG::ANALOG_IO1, &tmp);
	*value = (float)(tmp * 3.3 / 4096.0);
	return ret;
}

int UxbusCmd::tgpio_get_analog2(float * value) {
	float tmp;
	int ret = tgpio_addr_r16(SERVO3_RG::ANALOG_IO2, &tmp);
	// printf("tmp = %f\n", tmp);
	*value = (float)(tmp * 3.3 / 4096.0);
	return ret;
}

/*******************************************************
 * tgpio modbus
 *******************************************************/

int UxbusCmd::set_modbus_timeout(int value) {
	return set_nu16(UXBUS_RG::TGPIO_MB_TIOUT, &value, 1);
}

int UxbusCmd::set_modbus_baudrate(int baud) {
	float val;
	int baud_inx = get_baud_inx(baud);
	if (baud_inx == -1) return -1;
	int ret = tgpio_addr_r16(SERVO3_RG::MODBUS_BAUDRATE & 0x0FFF, &val);
	if (ret == 0) {
		int baud_i = (int)val;
		if (baud_i != baud_inx) {
			tgpio_addr_w16(SERVO3_RG::MODBUS_BAUDRATE, baud_inx);
			tgpio_addr_w16((0x1000 | SERVO3_RG::MODBUS_BAUDRATE), baud_inx);
			usleep(1e4); // 10ms
			return tgpio_addr_w16(SERVO3_RG::SOFT_REBOOT, 1);
		}
	}
	return ret;
}

int UxbusCmd::tgpio_set_modbus(unsigned char *modbus_t, int len_t, unsigned char *rx_data) {
	//unsigned char txdata[len_t + 1];
	unsigned char *txdata = new unsigned char[len_t + 1];
	txdata[0] = UXBUS_CONF::TGPIO_ID;
	for (int i = 0; i < len_t; i++) { txdata[i + 1] = modbus_t[i]; }
	std::lock_guard<std::mutex> locker(mutex_);
	int ret = send_xbus(UXBUS_RG::TGPIO_MODBUS, txdata, len_t + 1);
	delete txdata;
	if (0 != ret) { return UXBUS_STATE::ERR_NOTTCP; }

	ret = send_pend(UXBUS_RG::TGPIO_MODBUS, -1, UXBUS_CONF::SET_TIMEOUT, rx_data);
	return ret;
}

int UxbusCmd::gripper_modbus_w16s(int addr, float value, int len) {
	// unsigned char txdata[9], rx_data[254];
	unsigned char *txdata = new unsigned char[9];
	unsigned char *rx_data = new unsigned char[254];
	txdata[0] = UXBUS_CONF::GRIPPER_ID;
	txdata[1] = 0x10;
	bin16_to_8(addr, &txdata[2]);
	bin16_to_8(len, &txdata[4]);
	txdata[6] = len * 2;
	fp32_to_hex(value, &txdata[7]);
	return tgpio_set_modbus(txdata, len * 2 + 7, rx_data);
}

int UxbusCmd::gripper_modbus_r16s(int addr, int len, unsigned char *rx_data) {
	// unsigned char txdata[6];
	unsigned char *txdata = new unsigned char[6];
	txdata[0] = UXBUS_CONF::GRIPPER_ID;
	txdata[1] = 0x03;
	bin16_to_8(addr, &txdata[2]);
	bin16_to_8(len, &txdata[4]);
	return tgpio_set_modbus(txdata, 6, rx_data);
}

int UxbusCmd::gripper_modbus_set_en(int value) {
	// unsigned char txdata[2] = { 0 };
	unsigned char *txdata = new unsigned char[2];
	bin16_to_8(value, &txdata[0]);
	float _value = hex_to_fp32(txdata);
	return gripper_modbus_w16s(SERVO3_RG::CON_EN, _value, 1);
}

int UxbusCmd::gripper_modbus_set_mode(int value) {
	// unsigned char txdata[2];
	unsigned char *txdata = new unsigned char[2];
	bin16_to_8(value, &txdata[0]);
	float _value = hex_to_fp32(txdata);
	return gripper_modbus_w16s(SERVO3_RG::CON_MODE, _value, 1);
}

int UxbusCmd::gripper_modbus_set_zero(void) {
	return gripper_modbus_w16s(SERVO3_RG::MT_ZERO, 1, 1);
}

int UxbusCmd::gripper_modbus_get_pos(float *pulse) {
	// unsigned char rx_data[254];
	unsigned char *rx_data = new unsigned char[254];
	int ret = gripper_modbus_r16s(SERVO3_RG::CURR_POS, 2, rx_data);
	*pulse = (float)bin8_to_32(&rx_data[4]);
	return ret;
}

int UxbusCmd::gripper_modbus_set_pos(float pulse) {
	// unsigned char txdata[4];
	unsigned char *txdata = new unsigned char[4];
	txdata[0] = ((int)pulse >> 24) & 0xFF;
	txdata[1] = ((int)pulse >> 16) & 0xFF;
	txdata[2] = ((int)pulse >> 8) & 0xFF;
	txdata[3] = (int)pulse & 0xFF;
	float value = hex_to_fp32(txdata);
	return gripper_modbus_w16s(SERVO3_RG::TAGET_POS, value, 2);
}

int UxbusCmd::gripper_modbus_set_posspd(float speed) {
	// unsigned char txdata[2];
	unsigned char *txdata = new unsigned char[2];
	bin16_to_8((int)speed, &txdata[0]);
	float value = hex_to_fp32(txdata);
	return gripper_modbus_w16s(SERVO3_RG::POS_SPD, value, 1);
}

int UxbusCmd::gripper_modbus_get_errcode(int *err) {
	// unsigned char rx_data[254];
	unsigned char *rx_data = new unsigned char[254];
	int ret = gripper_modbus_r16s(SERVO3_RG::ERR_CODE, 1, rx_data);
	*err = bin8_to_16(&rx_data[4]);
	return ret;
}

int UxbusCmd::gripper_modbus_clean_err(void) {
	return gripper_modbus_w16s(SERVO3_RG::RESET_ERR, 1, 1);
}

/*******************************************************
 * uservo
 *******************************************************/
int UxbusCmd::servo_set_zero(int id) {
	return set_nu8(UXBUS_RG::SERVO_ZERO, &id, 1);
}

int UxbusCmd::servo_get_dbmsg(int rx_data[16]) {
	return get_nu8(UXBUS_RG::SERVO_DBMSG, rx_data, 16);
}

int UxbusCmd::servo_addr_w16(int id, int addr, float value) {
	// unsigned char txdata[7];
	std::lock_guard<std::mutex> locker(mutex_);
	unsigned char *txdata = new unsigned char[7];
	txdata[0] = id;
	bin16_to_8(addr, &txdata[1]);
	fp32_to_hex(value, &txdata[3]);
	int ret = send_xbus(UXBUS_RG::SERVO_W16B, txdata, 7);
	if (0 != ret) { return UXBUS_STATE::ERR_NOTTCP; }

	return send_pend(UXBUS_RG::SERVO_W16B, 0, UXBUS_CONF::SET_TIMEOUT, NULL);
}

int UxbusCmd::servo_addr_r16(int id, int addr, float *value) {
	std::lock_guard<std::mutex> locker(mutex_);
	// unsigned char txdata[3], rx_data[4];
	unsigned char *txdata = new unsigned char[3];
	unsigned char *rx_data = new unsigned char[4];
	txdata[0] = id;
	bin16_to_8(addr, &txdata[1]);
	int ret = send_xbus(UXBUS_RG::SERVO_R16B, txdata, 3);
	if (0 != ret) { return UXBUS_STATE::ERR_NOTTCP; }

	ret = send_pend(UXBUS_RG::SERVO_R16B, 4, UXBUS_CONF::GET_TIMEOUT, rx_data);
	*value = (float)bin8_to_32(rx_data);
	return ret;
}

int UxbusCmd::servo_addr_w32(int id, int addr, float value) {
	std::lock_guard<std::mutex> locker(mutex_);
	// unsigned char txdata[7];
	unsigned char *txdata = new unsigned char[7];
	txdata[0] = id;
	bin16_to_8(addr, &txdata[1]);
	fp32_to_hex(value, &txdata[3]);
	int ret = send_xbus(UXBUS_RG::SERVO_W32B, txdata, 7);
	if (0 != ret) { return UXBUS_STATE::ERR_NOTTCP; }

	return send_pend(UXBUS_RG::SERVO_W32B, 0, UXBUS_CONF::SET_TIMEOUT, NULL);
}

int UxbusCmd::servo_addr_r32(int id, int addr, float *value) {
	std::lock_guard<std::mutex> locker(mutex_);
	// unsigned char txdata[3], rx_data[4];
	unsigned char *txdata = new unsigned char[3];
	unsigned char *rx_data = new unsigned char[4];
	txdata[0] = id;
	bin16_to_8(addr, &txdata[1]);
	int ret = send_xbus(UXBUS_RG::SERVO_R32B, txdata, 3);
	if (0 != ret) { return UXBUS_STATE::ERR_NOTTCP; }

	ret = send_pend(UXBUS_RG::SERVO_R32B, 4, UXBUS_CONF::GET_TIMEOUT, rx_data);
	*value = (float)bin8_to_32(rx_data);
	return ret;
}



/*******************************************************
 * controler gpio
 *******************************************************/
int UxbusCmd::cgpio_get_auxdigit(int *value) {
	return get_nu16(UXBUS_RG::CGPIO_GET_DIGIT, value, 1);
}
int UxbusCmd::cgpio_get_analog1(float *value) {
	int tmp;
	int ret = get_nu16(UXBUS_RG::CGPIO_GET_ANALOG1, &tmp, 1);
	*value = (float)(tmp * 10.0 / 4096.0);
	return ret;
}
int UxbusCmd::cgpio_get_analog2(float *value) {
	int tmp;
	int ret = get_nu16(UXBUS_RG::CGPIO_GET_ANALOG2, &tmp, 1);
	*value = (float)(tmp * 10.0 / 4096.0);
	return ret;
}
/**
 *
 * @method cgpio_set_auxdigit
 * @param  ionum              [0~7]
 * @param  value              [0 / 1]
 * @return                    [description]
 */

int UxbusCmd::cgpio_set_auxdigit(int ionum, int value) {
	if (ionum > 7)
	{
		return -99;
	}

	int tmp = 0;
	tmp = tmp | (0x0100 << ionum);
	if (value)
	{
		tmp = tmp | (0x0001 << ionum);
	}
	return set_nu16(UXBUS_RG::CGPIO_SET_DIGIT, &tmp, 1);
}
int UxbusCmd::cgpio_set_analog1(int value) {
	value = (int)(value / 10.0 * 4096.0);
	return set_nu16(UXBUS_RG::CGPIO_SET_ANALOG1, &value, 1);
}
int UxbusCmd::cgpio_set_analog2(int value) {
	value = (int)(value / 10.0 * 4096.0);
	return set_nu16(UXBUS_RG::CGPIO_SET_ANALOG2, &value, 1);
}

int UxbusCmd::cgpio_set_infun(int num, int fun) {
	int txdata[2] = { num, fun };
	return set_nu8(UXBUS_RG::CGPIO_SET_IN_FUN, txdata, 2);
}
int UxbusCmd::cgpio_set_outfun(int num, int fun) {
	int txdata[2] = { num, fun };
	return set_nu8(UXBUS_RG::CGPIO_SET_OUT_FUN, txdata, 2);
}

/**
 *　get controler gpio all state infomation
 * @method UxbusCmd::cgpio_get_state
 * @param  state                     [description]
 * @param  digit_io                  [digital input functional gpio state,
									  digital input configuring gpio state,
									  digital output functional gpio state,
									  digital output configuring gpio state]
 * @param  analog                    [analog-1 input value,
									  analog-2 input value,
									  analog-1 output value,
									  analog-2 output value]
 * @param  input_conf                [digital input functional info]
 * @param  output_conf               [digital output functional info]
 * @return                           [description]
 */

int UxbusCmd::cgpio_get_state(int *state, int *digit_io, float *analog, int *input_conf, int *output_conf) {
	// unsigned char rx_data[34] = { 0 };
	unsigned char *rx_data = new unsigned char[34];
	int ret = get_nu8(UXBUS_RG::CGPIO_GET_STATE, rx_data, 34);

	state[0] = rx_data[0];
	state[1] = rx_data[1];
	for (int i = 0; i < 4; i++) {
		digit_io[i] = bin8_to_16(&rx_data[2 + i * 2]);
		analog[i] = (float)(bin8_to_16(&rx_data[10 + i * 2]) / 4096.0 * 10.0);
	}
	for (int i = 0; i < 8; i++) {
		input_conf[i] = rx_data[18 + i];
		output_conf[i] = rx_data[26 + i];
	}
	return ret;
}

int UxbusCmd::get_pose_offset(float pose1[6], float pose2[6], float offset[6], int orient_type_in, int orient_type_out) {
	float txdata[14] = { 0 };
	for (int i = 0; i < 6; i++) { txdata[i] = pose1[i]; }
	for (int i = 0; i < 6; i++) { txdata[6+i] = pose2[i]; }
	unsigned char *hexdata = new unsigned char[50];
	nfp32_to_hex(txdata, hexdata, 12);
	hexdata[48] = orient_type_in;
	hexdata[49] = orient_type_out;
	std::lock_guard<std::mutex> locker(mutex_);
	int ret = send_xbus(UXBUS_RG::MOVE_LINE_AA, hexdata, 50);
	delete hexdata;
	if (0 != ret) { return UXBUS_STATE::ERR_NOTTCP; }
	unsigned char *datas = new unsigned char[24];
	ret = send_pend(UXBUS_RG::MOVE_LINE_AA, 24, UXBUS_CONF::GET_TIMEOUT, datas);
	hex_to_nfp32(datas, offset, 6);
	delete datas;
	return ret;
}

int UxbusCmd::get_position_aa(float pose[6]) {
	return get_nfp32(UXBUS_RG::GET_TCP_POSE_AA, pose, 6);
}

int UxbusCmd::move_line_aa(float mvpose[6], float mvvelo, float mvacc, float mvtime, int mvcoord, int relative) {
	float txdata[9] = { 0 };
	for (int i = 0; i < 6; i++) { txdata[i] = mvpose[i]; }
	txdata[6] = mvvelo;
	txdata[7] = mvacc;
	txdata[8] = mvtime;
	char additional[2] = { (char)mvcoord, (char)relative };
	return set_nfp32_with_bytes(UXBUS_RG::MOVE_LINE_AA, txdata, 9, additional, 2);
}

int UxbusCmd::move_servo_cart_aa(float mvpose[6], float mvvelo, float mvacc, int tool_coord, int relative) {
	float txdata[9] = { 0 };
	for (int i = 0; i < 6; i++) { txdata[i] = mvpose[i]; }
	txdata[6] = mvvelo;
	txdata[7] = mvacc;
	txdata[8] = (char)tool_coord;
	char additional[1] = { (char)relative };
	return set_nfp32_with_bytes(UXBUS_RG::MOVE_SERVO_CART_AA, txdata, 9, additional, 1);
}

int UxbusCmd::tgpio_delay_set_digital(int ionum, int value, float delay_sec) {
	unsigned char *txdata = new unsigned char[6];
	txdata[0] = ionum;
	txdata[1] = value;
	fp32_to_hex(delay_sec, &txdata[2]);
	std::lock_guard<std::mutex> locker(mutex_);
	int ret = send_xbus(UXBUS_RG::DELAYED_TGPIO_SET, txdata, 6);
	if (0 != ret) { return UXBUS_STATE::ERR_NOTTCP; }

	return send_pend(UXBUS_RG::DELAYED_TGPIO_SET, 0, UXBUS_CONF::SET_TIMEOUT, NULL);
}

int UxbusCmd::cgpio_delay_set_digital(int ionum, int value, float delay_sec) {
	unsigned char *txdata = new unsigned char[6];
	txdata[0] = ionum;
	txdata[1] = value;
	fp32_to_hex(delay_sec, &txdata[2]);
	std::lock_guard<std::mutex> locker(mutex_);
	int ret = send_xbus(UXBUS_RG::DELAYED_CGPIO_SET, txdata, 6);
	if (0 != ret) { return UXBUS_STATE::ERR_NOTTCP; }

	return send_pend(UXBUS_RG::DELAYED_CGPIO_SET, 0, UXBUS_CONF::SET_TIMEOUT, NULL);
}

int UxbusCmd::tgpio_position_set_digital(int ionum, int value, float xyz[3], float tol_r) {
	unsigned char *txdata = new unsigned char[18];
	txdata[0] = ionum;
	txdata[1] = value;
	nfp32_to_hex(xyz, &txdata[2], 3);
	fp32_to_hex(tol_r, &txdata[14]);
	std::lock_guard<std::mutex> locker(mutex_);
	int ret = send_xbus(UXBUS_RG::POSITION_TGPIO_SET, txdata, 6);
	if (0 != ret) { return UXBUS_STATE::ERR_NOTTCP; }

	return send_pend(UXBUS_RG::POSITION_TGPIO_SET, 0, UXBUS_CONF::SET_TIMEOUT, NULL);
}

int UxbusCmd::cgpio_position_set_digital(int ionum, int value, float xyz[3], float tol_r) {
	unsigned char *txdata = new unsigned char[18];
	txdata[0] = ionum;
	txdata[1] = value;
	nfp32_to_hex(xyz, &txdata[2], 3);
	fp32_to_hex(tol_r, &txdata[14]);
	std::lock_guard<std::mutex> locker(mutex_);
	int ret = send_xbus(UXBUS_RG::POSITION_CGPIO_SET, txdata, 6);
	if (0 != ret) { return UXBUS_STATE::ERR_NOTTCP; }

	return send_pend(UXBUS_RG::POSITION_CGPIO_SET, 0, UXBUS_CONF::SET_TIMEOUT, NULL);
}

int UxbusCmd::config_io_stop_reset(int io_type, int val) {
	int txdata[2] = { io_type, val };
	return set_nu8(UXBUS_RG::SET_IO_STOP_RESET, txdata, 2);
}
