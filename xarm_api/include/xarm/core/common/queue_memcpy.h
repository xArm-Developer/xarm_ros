/* Copyright 2017 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jimy Zhang <jimy92@163.com>
 ============================================================================*/
#ifndef CORE_COMMON_QUEUE_MEMCPY_H_
#define CORE_COMMON_QUEUE_MEMCPY_H_

#include <iostream>
#include <functional>
#include <thread>
#include <mutex>

#ifdef _WIN32
#include <Windows.h>
#else
#include <pthread.h>
#endif

class QueueMemcpy {
public:
	QueueMemcpy(long n, long n_size);
	~QueueMemcpy(void);
	char flush(void);
	char push(void *data);
	char pop(void *data);
	char get(void *data);
	long size(void);
	long node_size(void);
	int is_full(void);

protected:
private:
	long total_;
	long annode_size_;

	long cnt_;
	long head_;
	long tail_;
	char *buf_;
	//pthread_mutex_t mutex_;

	std::thread report_thread_;
	std::mutex mutex_;
	/*
	#ifdef WIN32

	CRITICAL_SECTION m_cs;
	void init_lock() { InitializeCriticalSection(&m_cs); }

	void lock() { EnterCriticalSection(&m_cs); }
	void unlock() { LeaveCriticalSection(&m_cs); }

	#else
	  pthread_mutex_t m_mutex;
	  void init_lock() { pthread_mutex_init(&m_mutex, NULL); }
	  void lock() { pthread_mutex_lock(&m_mutex); }
	  void unlock() { pthread_mutex_unlock(&m_mutex); }
	  };
	#endif
	*/

};
#endif
