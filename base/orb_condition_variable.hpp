#pragma once

#include <pthread.h>
#include <time.h>

#include "orb_errno.h"
#include "orb_mutex.hpp"

namespace uORB {

template <clockid_t clock_id>
class ConditionVariable {
 public:
  ConditionVariable() noexcept {
    pthread_condattr_t attr;
    pthread_condattr_init(&attr);
    pthread_condattr_setclock(&attr, (clock_id));
    pthread_cond_init((&cond_), &attr);
  }

  ~ConditionVariable() noexcept { pthread_cond_destroy(&cond_); }

 public:
  ConditionVariable(const ConditionVariable &) = delete;
  ConditionVariable &operator=(const ConditionVariable &) = delete;

  void notify_one() noexcept { pthread_cond_signal(&cond_); }

  void notify_all() noexcept { pthread_cond_broadcast(&cond_); }

  void wait(Mutex &lock) noexcept {
    pthread_cond_wait(&cond_, lock.native_handle());
  }

  template <typename _Predicate>
  void wait(Mutex &lock, _Predicate p) {
    while (!p()) wait(lock);
  }

  // Return true if successful
  bool wait_until(Mutex &lock, const struct timespec &atime) {
    return pthread_cond_timedwait(&cond_, lock.native_handle(), &atime) == 0;
  }

  // Return true if successful
  template <typename _Predicate>
  bool wait_until(Mutex &lock, const struct timespec &atime, _Predicate p) {
    // Not returned until timeout or other error
    while (!p())
      if (!wait_until(lock, atime)) return p();
    return true;
  }

  // Return true if successful
  bool wait_for(Mutex &lock, unsigned long time_ms) {
    struct timespec atime {};
    GenerateFutureTime(clock_id, time_ms, atime);
    return wait_until(lock, atime);
  }

  // Return true if successful
  template <typename _Predicate>
  bool wait_for(Mutex &lock, unsigned long time_ms, _Predicate p) {
    struct timespec atime {};
    GenerateFutureTime(clock_id, time_ms, atime);

    // Not returned until timeout or other error
    while (!p())
      if (!wait_until(lock, atime)) return p();
    return true;
  }

  pthread_cond_t *native_handle() { return &cond_; }

 private:
  // Increase time_ms time based on the current clockid time
  static void GenerateFutureTime(clockid_t clockid, unsigned long time_ms,
                                 struct timespec &out) {
    const decltype(out.tv_nsec) kSec2Nsec = 1000 * 1000 * 1000;
    clock_gettime(clockid, &out);
    out.tv_nsec += time_ms * 1000 * 1000;
    out.tv_sec += out.tv_nsec / kSec2Nsec;
    out.tv_nsec %= kSec2Nsec;
  }

  pthread_cond_t cond_{};
};

typedef ConditionVariable<CLOCK_MONOTONIC> MonoClockCond;
typedef ConditionVariable<CLOCK_REALTIME> RealClockCond;

}  // namespace uORB
