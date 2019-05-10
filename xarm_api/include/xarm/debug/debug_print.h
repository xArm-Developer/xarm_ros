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
void print_nvect(const char *str, double vect[], int n);
void print_nvect(const char *str, float *vect, int n);
void print_nvect(const char *str, unsigned char vect[], int n);
void print_nvect(const char *str, int vect[], int n);
void print_hex(const char *str, unsigned char *hex, int len);

#endif
