/* Copyright 2017 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jimy Zhang <jimy92@163.com>
 ============================================================================*/
#ifndef CORE_COMMON_CRC16_H_
#define CORE_COMMON_CRC16_H_

#include "xarm/core/common/data_type.h"

int modbus_crc(unsigned char *data, int len);

#endif
