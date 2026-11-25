//
// Copyright (c) 2021 shawnfeng. All rights reserved.
//
#pragma once

#include <pthread.h>
#include <stdint.h>
#include <time.h>

#include "base/mutex.h"

namespace uorb {
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
    // https://chromium.googlesource.com/v8/v8/+/refs/tags/11.5.129/src/base/platform/condition-variable.cc#42
#ifdef __APPLE__
    // This hack is necessary to avoid a fatal pthreads subsystem bug in the
    // Darwin kernel. http://crbug.com/517681.
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
    const struct timespec deadline =
        timespec_get_after(get_now_time(), time_ms);

    while (!p()) {
      const struct timespec now = get_now_time();
      if (timespec_ge(now, deadline)) {
        return p();
      }

      uint32_t remaining_ms = timespec_diff_ms(now, deadline);
      if (remaining_ms == 0) {
        remaining_ms = 1;
      }

      if (!wait_for(lock, remaining_ms)) {
        return p();
      }
    }
    return true;
  }

  pthread_cond_t *native_handle() { return &cond_; }

 private:
  friend class ConditionVariableTest;

  static inline struct timespec get_now_time() {
    struct timespec result{};
    clock_gettime(kWhichClock, &result);
    return result;
  }

  static inline struct timespec timespec_get_after(const struct timespec &now,
                                                   uint32_t time_ms) {
    static const int64_t kNSecPerS = 1000000000LL;

    struct timespec result = now;

    result.tv_sec += time_ms / 1000;
    result.tv_nsec += static_cast<long>((time_ms % 1000) * 1000000LL);

    if (result.tv_nsec >= kNSecPerS) {
      result.tv_sec += result.tv_nsec / kNSecPerS;
      result.tv_nsec %= kNSecPerS;
    }

    return result;
  }

  static inline bool timespec_ge(const struct timespec &a,
                                 const struct timespec &b) {
    if (a.tv_sec != b.tv_sec) return a.tv_sec > b.tv_sec;
    return a.tv_nsec >= b.tv_nsec;
  }

  static inline uint32_t timespec_diff_ms(const struct timespec &a,
                                          const struct timespec &b) {
    const int64_t diff_ns =
        static_cast<int64_t>(b.tv_sec - a.tv_sec) * 1000000000LL +
        static_cast<int64_t>(b.tv_nsec - a.tv_nsec);
    if (diff_ns <= 0) return 0;
    return static_cast<uint32_t>((diff_ns + 999999LL) / 1000000LL);
  }

  pthread_cond_t cond_{};

  // The C++ specification defines std::condition_variable::wait_for in terms of
  // std::chrono::steady_clock, which is closest to CLOCK_MONOTONIC.
  static const clockid_t kWhichClock = CLOCK_MONOTONIC;
};

}  // namespace base
}  // namespace uorb
