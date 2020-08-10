/**
 * @file orb_sem.hpp
 *
 * Synchronization primitive: Semaphore
 */

#pragma once

#include <pthread.h>

#include "orb_condition_variable.hpp"
#include "orb_errno.h"
#include "visibility.h"

//---------------------------------------------------------
// Reference:
// https://github.com/preshing/cpp11-on-multicore/blob/master/common/sema.h
// Semaphore (POSIX, Linux)
//---------------------------------------------------------

#if defined(__unix__)
#include <errno.h>
#endif
#include <semaphore.h>
#include <stdint.h>

namespace uorb {
namespace base {

class Semaphore {
 public:
  explicit Semaphore(unsigned int count) { sem_init(&m_sema, 0, count); }
  Semaphore() = delete;
  Semaphore(const Semaphore &other) = delete;
  Semaphore &operator=(const Semaphore &other) = delete;

  ~Semaphore() { sem_destroy(&m_sema); }

  // increments the internal counter and unblocks acquirers
  void release() { sem_post(&m_sema); }

  // decrements the internal counter or blocks until it can
  void acquire() {
    // http://stackoverflow.com/questions/2013181/gdb-causes-sem-wait-to-fail-with-eintr-error
#if defined(__unix__)
    int rc;
    do {
      rc = sem_wait(&m_sema);
    } while (rc == -1 && errno == EINTR);
#else
    sem_wait(&m_sema);
#endif
  }

  // tries to decrement the internal counter without blocking
  bool try_acquire() { return sem_trywait(&m_sema) == 0; }

  // tries to decrement the internal counter, blocking for up to a duration time
  bool try_acquire_for(int time_ms) {
    struct timespec abstime {};
    GenerateFutureTime(CLOCK_REALTIME, time_ms, abstime);
    return try_acquire_until(abstime);
  }

 private:
  sem_t m_sema{};

  // Hide this function in case the client is not sure which clockid to use
  // tries to decrement the internal counter, blocking until a point in time
  bool try_acquire_until(const struct timespec &abstime) {
    return sem_timedwait(&m_sema, &abstime) == 0;
  }

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
};

}  // namespace base
}  // namespace uorb
