//
// Copyright (c) 2021 shawnfeng. All rights reserved.
//
#pragma once

#include <pthread.h>
#include <stdint.h>
#include <time.h>

#include "base/mutex.h"
#include "uorb/internal/noncopyable.h"

namespace uorb {
namespace base {

class ConditionVariableTest;

class ConditionVariable : public internal::Noncopyable {
 public:
  ConditionVariable(const ConditionVariable &) = delete;
  ConditionVariable &operator=(const ConditionVariable &) = delete;

  ConditionVariable() noexcept {
#ifdef __APPLE__
    pthread_cond_init(&cond_, nullptr);
#else
    pthread_condattr_t attr;
    pthread_condattr_init(&attr);
    pthread_condattr_setclock(&attr, kWhichClock);
    pthread_cond_init(&cond_, &attr);
#endif
  }

  ~ConditionVariable() noexcept {
    // https://chromium.googlesource.com/v8/v8/+/refs/tags/11.5.129/src/base/platform/condition-variable.cc#42
#ifdef __APPLE__
    // This hack is necessary to avoid a fatal pthreads subsystem bug in the
    // Darwin kernel. http://crbug.com/517681.
    {
      Mutex lock;
      LockGuard<Mutex> l(lock);
      struct timespec ts {};
      ts.tv_sec = 0;
      ts.tv_nsec = 1;
      pthread_cond_timedwait_relative_np(&cond_, lock.native_handle(), &ts);
    }
#endif
    pthread_cond_destroy(&cond_);
  }

  void notify_one() noexcept { pthread_cond_signal(&cond_); }

  void notify_all() noexcept { pthread_cond_broadcast(&cond_); }

  void wait(Mutex &lock) noexcept {  // NOLINT
    pthread_cond_wait(&cond_, lock.native_handle());
  }

  template <typename Predicate>
  void wait(Mutex &lock, Predicate p) {  // NOLINT
    while (!p()) wait(lock);
  }

  // Return true if successful
  bool wait_for(Mutex &lock, uint32_t time_ms) {  // NOLINT
#ifdef __APPLE__
    struct timespec rel_ts = {.tv_sec = time_ms / 1000,
                              .tv_nsec = (time_ms % 1000) * 1000000};
    // OS X do support waiting on a condition variable with a relative timeout.
    auto ret = pthread_cond_timedwait_relative_np(&cond_, lock.native_handle(),
                                                  &rel_ts) == 0;
    return ret;
#else
    struct timespec until_time = timespec_get_after(get_now_time(), time_ms);
    auto ret =
        pthread_cond_timedwait(&cond_, lock.native_handle(), &until_time) == 0;
    return ret;
#endif
  }

  // Return true if successful
  template <typename Predicate>
  bool wait_for(Mutex &lock, uint32_t time_ms, Predicate p) {  // NOLINT
    // Not returned until timeout or other error
    while (!p())
      if (!wait_for(lock, time_ms)) return p();
    return true;
  }

  pthread_cond_t *native_handle() { return &cond_; }

 private:
  friend class ConditionVariableTest;

  static inline struct timespec get_now_time() {
    struct timespec result {};
    clock_gettime(kWhichClock, &result);
    return result;
  }
  static inline struct timespec timespec_get_after(const struct timespec &now,
                                                   uint32_t time_ms) {
    static const auto kNSecPerS = 1000 * 1000 * 1000;

    struct timespec result = now;

    result.tv_sec += time_ms / 1000;
    result.tv_nsec += (time_ms % 1000) * 1000 * 1000;

    if (result.tv_nsec >= kNSecPerS) {
      result.tv_sec += result.tv_nsec / kNSecPerS;
      result.tv_nsec %= kNSecPerS;
    }

    return result;
  }

  pthread_cond_t cond_{};

  // The C++ specification defines std::condition_variable::wait_for in terms of
  // std::chrono::steady_clock, which is closest to CLOCK_MONOTONIC.
  static const clockid_t kWhichClock = CLOCK_MONOTONIC;
};

class SimpleSemaphore {
 public:
  explicit SimpleSemaphore(unsigned int count = 0) : count_(count) {}
  SimpleSemaphore(const SimpleSemaphore &) = delete;
  SimpleSemaphore &operator=(const SimpleSemaphore &) = delete;

  // increments the internal counter and unblocks acquirers
  void release() {
    LockGuard<decltype(mutex_)> lock(mutex_);
    ++count_;
    // The previous semaphore was 0, and there may be waiting tasks
    if (count_ == 1) condition_.notify_one();
  }

  // decrements the internal counter or blocks until it can
  void acquire() {
    LockGuard<decltype(mutex_)> lock(mutex_);
    condition_.wait(mutex_, [&] { return count_ > 0; });
    --count_;
  }

  // tries to decrement the internal counter without blocking
  bool try_acquire() {
    LockGuard<decltype(mutex_)> lock(mutex_);
    if (count_) {
      --count_;
      return true;
    }
    return false;
  }

  // tries to decrement the internal counter, blocking for up to a duration time
  bool try_acquire_for(uint32_t time_ms) {
    LockGuard<decltype(mutex_)> lock(mutex_);
    bool finished =
        condition_.wait_for(mutex_, time_ms, [&] { return count_ > 0; });
    if (finished) --count_;
    return finished;
  }

  unsigned int get_value() {
    LockGuard<decltype(mutex_)> lock(mutex_);
    return count_;
  }

 private:
  Mutex mutex_;
  ConditionVariable condition_;
  unsigned int count_;
};

}  // namespace base
}  // namespace uorb
