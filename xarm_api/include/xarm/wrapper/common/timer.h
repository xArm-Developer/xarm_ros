/*
# Software License Agreement (MIT License)
#
# Copyright (c) 2018, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
*/
#ifndef WRAPPER_COMMON_TIMER_H_
#define WRAPPER_COMMON_TIMER_H_

#include <iostream>
#include <functional>
#include <chrono>
#include <thread>
#include <atomic>
#include <memory>
#include <mutex>
#include <condition_variable>
#include <vector>
#include <queue>

class Timer {
public:
	Timer() :expired_(true), try_to_expire_(false) {
	}

	Timer(const Timer& t) {
		expired_ = t.expired_.load();
		try_to_expire_ = t.try_to_expire_.load();
	}
	~Timer() {
		Expire();
		// std::cout << "timer destructed!" << std::endl;
	}

	void StartTimer(int interval, std::function<void()> task) {
		if (expired_ == false) {
			// std::cout << "timer is currently running, please expire it first..." << std::endl;
			return;
		}
		expired_ = false;
		std::thread([this, interval, task]() {
			while (!try_to_expire_) {
				std::this_thread::sleep_for(std::chrono::milliseconds(interval));
				task();
			}
			// std::cout << "stop task..." << std::endl;
			{
				std::lock_guard<std::mutex> locker(mutex_);
				expired_ = true;
				expired_cond_.notify_one();
			}
		}).detach();
	}

	void Expire() {
		if (expired_) {
			return;
		}

		if (try_to_expire_) {
			// std::cout << "timer is trying to expire, please wait..." << std::endl;
			return;
		}
		try_to_expire_ = true;
		{
			std::unique_lock<std::mutex> locker(mutex_);
			expired_cond_.wait(locker, [this] {return expired_ == true; });
			if (expired_ == true) {
				// std::cout << "timer expired!" << std::endl;
				try_to_expire_ = false;
			}
		}
	}

	template<typename callable, class... arguments>
	void SyncWait(int after, callable&& f, arguments&&... args) {

		std::function<typename std::result_of<callable(arguments...)>::type()> task
		(std::bind(std::forward<callable>(f), std::forward<arguments>(args)...));
		std::this_thread::sleep_for(std::chrono::milliseconds(after));
		task();
	}
	template<typename callable, class... arguments>
	void AsyncWait(int after, callable&& f, arguments&&... args) {
		std::function<typename std::result_of<callable(arguments...)>::type()> task
		(std::bind(std::forward<callable>(f), std::forward<arguments>(args)...));

		std::thread([after, task]() {
			std::this_thread::sleep_for(std::chrono::milliseconds(after));
			task();
		}).detach();
	}

private:
	std::atomic<bool> expired_;
	std::atomic<bool> try_to_expire_;
	std::mutex mutex_;
	std::condition_variable expired_cond_;
};

class ThreadPool {
public:
	ThreadPool(int max_thread_count = 10) : max_thread_count_(max_thread_count), total_thread_count_(0), free_thread_count_(0) {};
	~ThreadPool() {
		stop();
	};

	static void thread_handle(void *arg) {
		ThreadPool *pool = (ThreadPool *)arg;
		pool->_thread_process();
	};

	void _thread_process(void) {
		std::unique_lock<std::mutex> locker(mutex_);
		total_thread_count_ += 1;
		free_thread_count_ += 1;
		// int thread_inx = total_thread_count_;
		locker.unlock();

		// std::cout << "callback thread start, thread_index=" << thread_inx << ", thread_id=" << std::this_thread::get_id() << std::endl;
		while (!stoped_) {
			std::function<void()> task;
			locker.lock();
			task_cond_.wait(locker, [this] {
				return stoped_ || !task_que_.empty();
			});
			if (stoped_ && task_que_.empty()) {
				locker.unlock();
				break;
			}
			free_thread_count_ -= 1;
			task = std::move(task_que_.front());
			task_que_.pop();
			locker.unlock();
			task();
			locker.lock();
			free_thread_count_ += 1;
			locker.unlock();
		}
		locker.lock();
		total_thread_count_ -= 1;
		free_thread_count_ -= 1;
		locker.unlock();
		// std::cout << "callback thread finished, thread_index=" << thread_inx << ", thread_id=" << std::this_thread::get_id() << std::endl;
	};

	void stop() {
		stoped_.store(true);
		task_cond_.notify_all();
		for (std::thread& thread : pool_) {
			try {
				if (thread.joinable()) thread.join();
			}
			catch (...) {}
		}
	};

	void set_max_thread_count(int max_thread_count) {
		max_thread_count_ = max_thread_count;
	};

	template<typename callable, class... arguments>
	void dispatch(callable&& f, arguments&&... args) {
		std::function<typename std::result_of<callable(arguments...)>::type()> task
		(std::bind(std::forward<callable>(f), std::forward<arguments>(args)...));
		{
			std::lock_guard<std::mutex> lock{ mutex_ };
			task_que_.emplace([task]() {
				task();
			});
			_thread_check();
			task_cond_.notify_one();
		}
	}

	template<typename callable, class... arguments>
	void commit(callable&& f, arguments&&... args) {
		std::function<typename std::result_of<callable(arguments...)>::type()> task
		(std::bind(std::forward<callable>(f), std::forward<arguments>(args)...));
		task();
	}

private:
	void _thread_check(void) {
		if (free_thread_count_ == 0 && (max_thread_count_ < 0 || total_thread_count_ < max_thread_count_)) {
			pool_.emplace_back(std::thread(thread_handle, this));
		}
	}

private:
	using Task = std::function<void()>;
	std::queue<Task> task_que_;
	std::vector<std::thread> pool_;
	std::mutex mutex_;
	std::condition_variable task_cond_;

	int max_thread_count_;
	int total_thread_count_;
	int free_thread_count_;
	std::atomic<bool> stoped_;
};

#endif // WRAPPER_COMMON_TIMER_H_
