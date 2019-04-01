/* Copyright 2017 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jimy Zhang <jimy92@163.com>
 ============================================================================*/
#include "xarm/linux/thread.h"

#include <stdio.h>

#define PRINT_ERR printf

pthread_t thread_init(fun_point_t fun_point, void *arg) {
  pthread_t id;
  pthread_attr_t attr;

  pthread_attr_init(&attr);
  pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
  char ret = pthread_create(&id, &attr, fun_point, arg);
  if (0 != ret) PRINT_ERR("error: pthread create failes\n");

  return id;
}

void thread_delete(pthread_t id) { pthread_cancel(id); }
