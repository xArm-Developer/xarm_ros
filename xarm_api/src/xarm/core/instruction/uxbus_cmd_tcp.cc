/*
# Software License Agreement (MIT License)
#
# Copyright (c) 2020, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
*/

#include "xarm/core/instruction/uxbus_cmd_tcp.h"
#include "xarm/core/debug/debug_print.h"
#include "xarm/core/instruction/uxbus_cmd_config.h"

UxbusCmdTcp::UxbusCmdTcp(SocketPort *arm_port) {
	arm_port_ = arm_port;
	bus_flag_ = TX2_BUS_FLAG_MIN_;
	prot_flag_ = TX2_PROT_CON_;
}

UxbusCmdTcp::~UxbusCmdTcp(void) {}

int UxbusCmdTcp::check_xbus_prot(unsigned char *datas, int funcode) {
	unsigned char *data_fp = &datas[4];

	int sizeof_data = bin8_to_32(datas);
	if (sizeof_data < 8 || sizeof_data >= arm_port_->que_maxlen_)
	{
		return UXBUS_STATE::ERR_LENG;
	}

	int num = bin8_to_16(&data_fp[0]);
	int prot = bin8_to_16(&data_fp[2]);
	int len = bin8_to_16(&data_fp[4]);
	int fun = data_fp[6];
	int state = data_fp[7];

	int bus_flag = bus_flag_;
	if (bus_flag == TX2_BUS_FLAG_MIN_)
	{
		bus_flag = TX2_BUS_FLAG_MAX_;
	}
	else
	{
		bus_flag -= 1;
	}

	if (num != bus_flag) { return UXBUS_STATE::ERR_NUM; }
	if (prot != TX2_PROT_CON_) { return UXBUS_STATE::ERR_PROT; }
	if (fun != funcode) { return UXBUS_STATE::ERR_FUN; }
	state_is_ready = !(state & 0x10);
	if (state & 0x40) { return UXBUS_STATE::ERR_CODE; }
	if (state & 0x20) { return UXBUS_STATE::WAR_CODE; }
	if (sizeof_data != len + 6) { return UXBUS_STATE::ERR_LENG; }
	// if (state & 0x10) { return UXBUS_STATE::STATE_NOT_READY; }
	return 0;
}

int UxbusCmdTcp::send_pend(int funcode, int num, int timeout, unsigned char *ret_data) {
	int i;
	int ret = UXBUS_STATE::ERR_TOUT;
	int ret2;
	// unsigned char rx_data[arm_port_->que_maxlen_] = {0};
	unsigned char *rx_data = new unsigned char[arm_port_->que_maxlen_];
	long long expired = get_system_time() + (long long)timeout;
	while (get_system_time() < expired) {
		ret2 = arm_port_->read_frame(rx_data);
		if (ret2 != -1) {
			// print_hex("recv:", rx_data, arm_port_->que_maxlen_);
			ret = check_xbus_prot(rx_data, funcode);
			if (ret == 0 || ret == UXBUS_STATE::ERR_CODE || ret == UXBUS_STATE::WAR_CODE) {
				int n = num;
				if (num == -1) {
					n = rx_data[9] - 2;
				}
				for (i = 0; i < n; i++) { ret_data[i] = rx_data[i + 8 + 4]; }
				// print_hex(" 3", rx_data, num + 8 + 4);
				break;
			}
			else if (ret != UXBUS_STATE::ERR_NUM) {
				break;
			}
		}
		sleep_milliseconds(1);
	}
	delete[] rx_data;
	return ret;
}

int UxbusCmdTcp::send_xbus(int funcode, unsigned char *datas, int num) {
	int len = num + 7;
	// unsigned char send_data[len];
	unsigned char *send_data = new unsigned char[len];

	bin16_to_8(bus_flag_, &send_data[0]);
	bin16_to_8(prot_flag_, &send_data[2]);
	bin16_to_8(num + 1, &send_data[4]);
	send_data[6] = funcode;

	for (int i = 0; i < num; i++) { send_data[7 + i] = datas[i]; }
	arm_port_->flush();
	// print_hex("send:", send_data, num + 7);
	int ret = arm_port_->write_frame(send_data, len);
	delete[] send_data;
	if (ret != len) { return -1; }

	bus_flag_ += 1;
	if (bus_flag_ > TX2_BUS_FLAG_MAX_) { bus_flag_ = TX2_BUS_FLAG_MIN_; }

	return 0;
}

void UxbusCmdTcp::close(void) { arm_port_->close_port(); }

int UxbusCmdTcp::is_ok(void) { return arm_port_->is_ok(); }
