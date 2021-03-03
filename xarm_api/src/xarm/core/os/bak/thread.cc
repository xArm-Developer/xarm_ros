/* Copyright 2017 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jimy Zhang <jimy92@163.com>
 ============================================================================*/


#include <stdio.h>
#include "xarm/core/os/thread.h"

#define PRINT_ERR printf

#ifdef _WIN32
HANDLE thread_init(fun_point_t fun_point, void *arg) {
	HANDLE m_handle;
	/*
	pthread_attr_t attr;

	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
	char ret = pthread_create(&id, &attr, fun_point, arg);
	*/
	m_handle = (HANDLE)_beginthreadex(NULL, 0, fun_point, arg, 0, NULL);
	if (NULL == m_handle) PRINT_ERR("error: pthread create failes\n");

	return m_handle;
}

void thread_delete(HANDLE m_handle) { CloseHandle(m_handle); }

#else
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
#endif



