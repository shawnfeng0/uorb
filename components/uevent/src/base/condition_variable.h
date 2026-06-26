//
// Copyright (c) 2021 shawnfeng. All rights reserved.
//
#pragma once

#include <pthread.h>
#include <stdint.h>
#include <time.h>

#include "base/mutex.h"

namespace uevent {
namespace base {

class ConditionVariableTest;

class ConditionVariable {
 public:
  ConditionVariable(const ConditionVariable &) = delete;
  ConditionVariable(ConditionVariable &&) = delete;
  ConditionVariable &operator=(const ConditionVariable &) = delete;
  ConditionVariable &operator=(ConditionVariable &&) = delete;

  ConditionVariable() noexcept {
#ifdef __APPLE__
    pthread_cond_init(&cond_, nullptr);
#else
    pthread_condattr_t attr;
    const bool attr_ready = (pthread_condattr_init(&attr) == 0);
    const bool use_custom_clock =
        attr_ready && (pthread_condattr_setclock(&attr, kWhichClock) == 0);
    pthread_cond_init(&cond_, use_custom_clock ? &attr : nullptr);
    if (attr_ready) {
      pthread_condattr_destroy(&attr);
    }
#endif
  }

  ~ConditionVariable() noexcept {
#ifdef __APPLE__
    {
      Mutex lock;
      LockGuard<Mutex> l(lock);
      struct timespec ts{};
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

  bool wait_for(Mutex &lock, uint32_t time_ms) {  // NOLINT
#ifdef __APPLE__
    struct timespec rel_ts = {.tv_sec = time_ms / 1000,
                              .tv_nsec = (time_ms % 1000) * 1000000};
    return pthread_cond_timedwait_relative_np(&cond_, lock.native_handle(),
                                               &rel_ts) == 0;
#else
    struct timespec until_time = timespec_add_ms(get_now(), time_ms);
    return pthread_cond_timedwait(&cond_, lock.native_handle(), &until_time) == 0;
#endif
  }

  template <typename Predicate>
  bool wait_for(Mutex &lock, uint32_t time_ms, Predicate p) {  // NOLINT
    struct timespec deadline = timespec_add_ms(get_now(), time_ms);
    while (!p()) {
      struct timespec now = get_now();
      if (timespec_ge(now, deadline)) {
        return p();
      }
      uint32_t remaining_ms = timespec_diff_ms(now, deadline);
      if (remaining_ms == 0) remaining_ms = 1;
      if (!wait_for(lock, remaining_ms)) {
        return p();
      }
    }
    return true;
  }

  pthread_cond_t *native_handle() { return &cond_; }

  // Utility: get current monotonic time
  static struct timespec get_now() {
    struct timespec ts{};
    clock_gettime(kWhichClock, &ts);
    return ts;
  }

  // Utility: add milliseconds to a timespec
  static struct timespec timespec_add_ms(const struct timespec &ts, uint32_t ms) {
    static const int64_t kNSecPerS = 1000000000LL;
    struct timespec result = ts;
    result.tv_sec += ms / 1000;
    result.tv_nsec += (ms % 1000) * 1000000LL;
    if (result.tv_nsec >= kNSecPerS) {
      result.tv_sec += result.tv_nsec / kNSecPerS;
      result.tv_nsec %= kNSecPerS;
    }
    return result;
  }

  // Utility: compare two timespecs (a >= b)
  static bool timespec_ge(const struct timespec &a, const struct timespec &b) {
    if (a.tv_sec != b.tv_sec) return a.tv_sec > b.tv_sec;
    return a.tv_nsec >= b.tv_nsec;
  }

  // Utility: difference in ms (b - a), clamped to uint32_t
  static uint32_t timespec_diff_ms(const struct timespec &a, const struct timespec &b) {
    int64_t diff_ns = (int64_t)(b.tv_sec - a.tv_sec) * 1000000000LL +
                       (b.tv_nsec - a.tv_nsec);
    if (diff_ns <= 0) return 0;
    return (uint32_t)((diff_ns + 999999) / 1000000);
  }

 private:
  friend class ConditionVariableTest;
  pthread_cond_t cond_{};
  static const clockid_t kWhichClock = CLOCK_MONOTONIC;
};

}  // namespace base
}  // namespace uevent
