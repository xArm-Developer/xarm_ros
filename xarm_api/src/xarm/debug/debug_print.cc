/* Copyright 2017 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jimy Zhang <jimy92@163.com>
 ============================================================================*/
#include "xarm/debug/debug_print.h"

#include <stdarg.h>

#include "xarm/common/data_type.h"

#define DB_FLG "[deprint ] "
#define PRINTF_NUM_MAX 128
#define log_put printf

void print_log(const char *format, ...) {
  char buffer[PRINTF_NUM_MAX] = {0};
  va_list arg;
  va_start(arg, format);
  vsnprintf(buffer, PRINTF_NUM_MAX, format, arg);
  printf("%s", buffer);
  va_end(arg);
}

void print_nvect(const char *str, double vect[], int n) {
  print_log("%s", str);
  for (int i = 0; i < n; ++i) { print_log("%0.3f ", vect[i]); }
  print_log("\n");
}

void print_nvect(const char *str, float *vect, int n) {
  print_log("%s", str);
  for (int i = 0; i < n; ++i) { print_log("%0.3f ", vect[i]); }
  print_log("\n");
}

void print_nvect(const char *str, unsigned char vect[], int n) {
  print_log("%s", str);
  for (int i = 0; i < n; ++i) { print_log("%d ", vect[i]); }
  print_log("\n");
}

void print_nvect(const char *str, int vect[], int n) {
  print_log("%s", str);
  for (int i = 0; i < n; ++i) { print_log("%d ", vect[i]); }
  print_log("\n");
}

void print_hex(const char *str, unsigned char *hex, int len) {
  char buf[len * 3 + 1] = {'\0'};
  long i;
  for (i = 0; i < len; ++i) { sprintf((char *)&buf[i * 3], "%02x ", hex[i]); }

  printf("%s %s\n", str, buf);
}
