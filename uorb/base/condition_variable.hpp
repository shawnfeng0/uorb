#pragma once

#include <pthread.h>
#include <stdint.h>
#include <time.h>

#include "errno.h"
#include "mutex.hpp"

namespace uorb {
namespace base {

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
  inline void GenerateFutureTime(clockid_t clockid, unsigned long time_ms,
                                 struct timespec &out) {
    // Calculate an absolute time in the future
    const decltype(out.tv_nsec) kSec2Nsec = 1000 * 1000 * 1000;
    clock_gettime(clockid, &out);
    uint64_t nano_secs = out.tv_nsec + ((uint64_t)time_ms * 1000 * 1000);
    out.tv_nsec = nano_secs % kSec2Nsec;
    out.tv_sec += nano_secs / kSec2Nsec;
  }

  pthread_cond_t cond_{};
};

typedef ConditionVariable<CLOCK_MONOTONIC> MonoClockCond;
typedef ConditionVariable<CLOCK_REALTIME> RealClockCond;

template <clockid_t clock_id>
class SimpleSemaphore {
 public:
  explicit SimpleSemaphore(unsigned int count) : count_(count) {}
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
  bool try_acquire_for(int time_ms) {
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
  // Hide this function in case the client is not sure which clockid to use
  // tries to decrement the internal counter, blocking until a point in time
  bool try_acquire_until(const struct timespec &atime) {
    LockGuard<decltype(mutex_)> lock(mutex_);
    bool finished =
        condition_.wait_until(mutex_, atime, [&] { return count_ > 0; });
    if (finished) --count_;
    return finished;
  }

  Mutex mutex_;
  ConditionVariable<clock_id> condition_;
  unsigned int count_;
};

typedef SimpleSemaphore<CLOCK_REALTIME> RealClockSemaphore;
typedef SimpleSemaphore<CLOCK_MONOTONIC> MonoClockSemaphore;

}  // namespace base
}  // namespace uorb
