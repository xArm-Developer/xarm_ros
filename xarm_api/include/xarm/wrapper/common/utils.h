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

#include <iostream>
#include <vector>

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
