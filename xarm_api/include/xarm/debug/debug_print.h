/* Copyright 2017 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jimy Zhang <jimy92@163.com>
 ============================================================================*/
#ifndef DEBUG_DEBUG_PRINT_H_
#define DEBUG_DEBUG_PRINT_H_

#include "xarm/common/data_type.h"

void print_log(const char *format, ...);
void print_nvect(const char *str, fp64 vect[], u8 n);
void print_nvect(const char *str, float *vect, u8 n);
void print_nvect(const char *str, u8 vect[], u8 n);
void print_nvect(const char *str, u16 vect[], u8 n);
void print_nvect(const char *str, u32 vect[], u8 n);
void print_hex(const char *str, u8 *hex, u8 len);

#endif
