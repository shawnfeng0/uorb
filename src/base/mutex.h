//
// Copyright (c) 2021 shawnfeng. All rights reserved.
//
#pragma once

//---------------------------------------------------------
// Reference:
// https://github.com/google/glog/blob/master/src/base/mutex.h
//---------------------------------------------------------

#include <pthread.h>

#include "uorb/internal/noncopyable.h"

namespace uorb {
namespace base {

/// The standard Mutex type.
class Mutex : internal::Noncopyable {
#define SAFE_PTHREAD_MUTEX(fncall)          \
  do { /* run fncall if is_safe_ is true */ \
    if (is_safe_) fncall(&mutex_);          \
  } while (0)

 public:
#ifdef PTHREAD_MUTEX_INITIALIZER
  constexpr Mutex() noexcept = default;
  ~Mutex() = default;
#else
  Mutex() noexcept {
    SetIsSafe();
    pthread_mutex_init(&mutex_, nullptr);
  }
  ~Mutex() noexcept { SAFE_PTHREAD_MUTEX(pthread_mutex_destroy); }
#endif
  Mutex(const Mutex &) = delete;
  Mutex &operator=(const Mutex &) = delete;

  void lock() { SAFE_PTHREAD_MUTEX(pthread_mutex_lock); }
  void unlock() { SAFE_PTHREAD_MUTEX(pthread_mutex_unlock); }

  bool try_lock() noexcept {
    return is_safe_ ? 0 == pthread_mutex_trylock(&mutex_) : true;
  }

  pthread_mutex_t *native_handle() noexcept { return &mutex_; }

 private:
#ifdef PTHREAD_MUTEX_INITIALIZER
  pthread_mutex_t mutex_ = PTHREAD_MUTEX_INITIALIZER;
  volatile bool is_safe_{true};
#else
  pthread_mutex_t mutex_{};

  // We want to make sure that the compiler sets is_safe_ to true only
  // when we tell it to, and never makes assumptions is_safe_ is
  // always true.  volatile is the most reliable way to do that.
  volatile bool is_safe_{};
  inline void SetIsSafe() { is_safe_ = true; }
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
