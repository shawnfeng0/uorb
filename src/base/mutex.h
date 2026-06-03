//
// Copyright (c) 2021 shawnfeng. All rights reserved.
//
#pragma once

//---------------------------------------------------------
// Reference:
// https://github.com/google/glog/blob/master/src/base/mutex.h
//---------------------------------------------------------

#include <pthread.h>

namespace uorb {
namespace base {

/// The standard Mutex type.
class Mutex {
 public:
  Mutex(const Mutex &) = delete;
  Mutex(Mutex &&) = delete;
  Mutex &operator=(const Mutex &) = delete;
  Mutex &operator=(Mutex &&) = delete;
#ifdef PTHREAD_MUTEX_INITIALIZER
  constexpr Mutex() noexcept = default;
  ~Mutex() = default;
#else
  Mutex() noexcept { pthread_mutex_init(&mutex_, nullptr); }
  ~Mutex() noexcept { pthread_mutex_destroy(&mutex_); }
#endif
  void lock() { pthread_mutex_lock(&mutex_); }
  void unlock() { pthread_mutex_unlock(&mutex_); }

  bool try_lock() noexcept { return 0 == pthread_mutex_trylock(&mutex_); }

  pthread_mutex_t *native_handle() noexcept { return &mutex_; }

 private:
#ifdef PTHREAD_MUTEX_INITIALIZER
  pthread_mutex_t mutex_ = PTHREAD_MUTEX_INITIALIZER;
#else
  pthread_mutex_t mutex_{};
#endif
};

/**
 * @brief A simple scoped lock type.
 *
 * A LockGuard controls Mutex ownership within a scope, releasing
 * ownership in the destructor.
 */
template <typename MutexType = Mutex>
class LockGuard {
 public:
  explicit LockGuard(MutexType &m) : mutex_(m) { mutex_.lock(); }
  ~LockGuard() { mutex_.unlock(); }

  LockGuard(const LockGuard &) = delete;
  LockGuard &operator=(const LockGuard &) = delete;

 private:
  MutexType &mutex_;
};

}  // namespace base
}  // namespace uorb

