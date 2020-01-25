#pragma once

#include <pthread.h>

#include "orb_errno.h"
#include "orb_mutex.hpp"

namespace uORB {

typedef pthread_cond_t __orb_thread_cond_t;

#ifdef PTHREAD_COND_INITIALIZER
#define __ORB_THREAD_COND_INITIALIZER PTHREAD_COND_INITIALIZER
#endif
#define __ORB_THREAD_COND_INIT(x) pthread_cond_init((x), nullptr)
#define __ORB_THREAD_COND_INIT_CLOCK(x, clock_id) \
do { \
	pthread_condattr_t attr; \
	pthread_condattr_init(&attr); \
	pthread_condattr_setclock(&attr, (clock_id)); \
	pthread_cond_init((x), &attr); \
} while (0)
#define __ORB_THREAD_COND_DESTROY pthread_cond_destroy
#define __ORB_THREAD_COND_SIGNAL pthread_cond_signal
#define __ORB_THREAD_COND_BROADCAST pthread_cond_broadcast
#define __ORB_THREAD_COND_TIMEDWAIT pthread_cond_timedwait
#define __ORB_THREAD_COND_WAIT pthread_cond_wait

/// condition_variable
template <int clock_id>
class ConditionVariable {

#ifdef __ORB_THREAD_COND_INITIAIZER
  __orb_thread_cond_t cond_ = __ORB_THREAD_COND_INITIALIZER;
  condition_variable() noexcept = default;
#else
  __orb_thread_cond_t cond_{};

public:
  ConditionVariable() noexcept {
    __ORB_THREAD_COND_INIT_CLOCK(&cond_, clock_id);
  }

  ~ConditionVariable() noexcept {
    __ORB_THREAD_COND_DESTROY(&cond_);
  }
#endif

 public:
   ConditionVariable(const ConditionVariable &) = delete;
   ConditionVariable & operator=(const ConditionVariable &) = delete;

  int notify_one() noexcept { return __ORB_THREAD_COND_SIGNAL(&cond_); }

  int notify_all() noexcept { return __ORB_THREAD_COND_BROADCAST(&cond_); }

  int wait(Mutex &lock) noexcept {
    return __ORB_THREAD_COND_WAIT(&cond_, lock.native_handle());
  }

  template <typename _Predicate>
  int wait(Mutex &lock, _Predicate p) {
    int ret = 0;
    while (!p()) ret = wait(lock);
    return ret;
  }

  int wait_until(Mutex &lock, const struct timespec&atime) {
    return __ORB_THREAD_COND_TIMEDWAIT(&cond_, lock.native_handle(), &atime);
  }

  int wait_until(Mutex &lock, const struct timespec *atime) {
    return __ORB_THREAD_COND_TIMEDWAIT(&cond_, lock.native_handle(), atime);
  }

  template <typename _Predicate>
  bool wait_until(Mutex &lock, const struct timespec&atime,
                  _Predicate p) {
    while (!p())
      if (wait_until(lock, atime) == ETIMEDOUT) return p();
    return true;
  }

  int wait_for(Mutex &lock, long usec) {
    struct timespec req {};
    clock_gettime(CLOCK_MONOTONIC, &req);
    req.tv_nsec += usec;
    req.tv_sec += req.tv_nsec / (1e9);
    return wait_until(lock, req);
  }

  __orb_thread_cond_t* native_handle() { return &cond_; }
};

typedef ConditionVariable<CLOCK_MONOTONIC> MonoClockCond;
typedef ConditionVariable<CLOCK_REALTIME> RealClockCond;

}  // namespace uORB
