/* Copyright 2017 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jimy Zhang <jimy92@163.com>
 ============================================================================*/
#ifndef COMMON_CRC16_H_
#define COMMON_CRC16_H_

#include "xarm/common/data_type.h"

u16 modbus_crc(u8 *data, u16 len);

#endif
