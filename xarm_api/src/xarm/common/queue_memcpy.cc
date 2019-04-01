/* Copyright 2017 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jimy Zhang <jimy92@163.com>
 ============================================================================*/
#include "xarm/common/queue_memcpy.h"

#include <string.h>

QueueMemcpy::QueueMemcpy(long n, long n_size) {
  total_ = n;
  annode_size_ = n_size;
  buf_ = new char[total_ * annode_size_];
  pthread_mutex_init(&mutex_, NULL);
  flush();
}

QueueMemcpy::~QueueMemcpy(void) { delete[] buf_; }

char QueueMemcpy::flush(void) {
  cnt_ = 0;
  head_ = 0;
  tail_ = 0;
  memset(buf_, 0, annode_size_ * total_);

  return 0;
}

long QueueMemcpy::size(void) { return cnt_; }

int QueueMemcpy::is_full(void) {
  if (total_ <= cnt_)
    return 1;
  else
    return 0;
}

long QueueMemcpy::node_size(void) { return annode_size_; }

char QueueMemcpy::pop(void *data) {
  pthread_mutex_lock(&mutex_);
  if (0 >= cnt_) {
    pthread_mutex_unlock(&mutex_);
    return -1;
  }
  if (total_ <= tail_) tail_ = 0;

  memcpy(data, &buf_[tail_ * annode_size_], annode_size_);
  tail_++;
  cnt_--;
  pthread_mutex_unlock(&mutex_);
  return 0;
}

char QueueMemcpy::get(void *data) {
  pthread_mutex_lock(&mutex_);
  if (0 >= cnt_) {
    pthread_mutex_unlock(&mutex_);
    return -1;
  }
  if (total_ <= tail_) tail_ = 0;

  memcpy(data, &buf_[tail_ * annode_size_], annode_size_);
  pthread_mutex_unlock(&mutex_);

  return 0;
}

char QueueMemcpy::push(void *data) {
  pthread_mutex_lock(&mutex_);
  if (total_ <= cnt_) {
    pthread_mutex_unlock(&mutex_);
    return -1;
  }
  if (total_ <= head_) head_ = 0;

  memcpy(&buf_[head_ * annode_size_], data, annode_size_);
  head_++;
  cnt_++;
  pthread_mutex_unlock(&mutex_);
  return 0;
}
