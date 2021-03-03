/* Copyright 2017 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jimy Zhang <jimy92@163.com>
 ============================================================================*/
 /*#ifndef CORE_LINUX_THREAD_H_
 #define CORE_LINUX_THREAD_H_

 #include <pthread.h>

 typedef void *(*fun_point_t)(void *);

 void thread_delete(pthread_t id);
 pthread_t thread_init(fun_point_t fun_point, void *arg);

 #endif
 */

#ifndef THREAD_H_
#define THREAD_H_

#ifdef _WIN32
#include <windows.h>
#include <process.h>
#else
#include <pthread.h>
#endif


#ifdef WIN32
//typedef unsigned __stdcall *(*fun_point_t)(void *);
typedef unsigned __stdcall fun_point_t(void*);
void thread_delete(HANDLE m_handle);
HANDLE thread_init(fun_point_t fun_point, void *arg);
#else
typedef void *(*fun_point_t)(void *);
void thread_delete(pthread_t id);
pthread_t thread_init(fun_point_t fun_point, void *arg);
#endif

#endif // THREAD_H_