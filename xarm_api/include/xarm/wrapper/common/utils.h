/*
# Software License Agreement (MIT License)
#
# Copyright (c) 2019, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
*/
#ifndef WRAPPER_COMMON_UTILS_H_
#define WRAPPER_COMMON_UTILS_H_
#include <sys/timeb.h>
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#include <time.h>
#endif

inline void sleep_milliseconds(unsigned long milliseconds) {
#ifdef _WIN32
	Sleep(milliseconds); // 100 ms
#else
	usleep(milliseconds * 1000); // 100 ms
#endif
}

inline long long get_system_time()
{
#ifdef _WIN32
	struct timeb t;
	ftime(&t);
	return 1000 * t.time + t.millitm;
#else
	struct timespec t;
	clock_gettime(CLOCK_REALTIME, &t);
	return 1000 * t.tv_sec + t.tv_nsec / 1000000;
#endif
}

inline std::vector<std::string> split(const std::string &str, const std::string &pattern)
{

    std::vector<std::string> resVec;

    if ("" == str)
    {
        return resVec;
    }

	std::string strs = str + pattern;

    size_t pos = strs.find(pattern);
    size_t size = strs.size();

    while (pos != std::string::npos)
    {
        std::string x = strs.substr(0, pos);
        resVec.push_back(x);
        strs = strs.substr(pos + 1, size);
        pos = strs.find(pattern);
    }

    return resVec;
}

#endif
