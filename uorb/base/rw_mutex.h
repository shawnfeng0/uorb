#pragma once

//---------------------------------------------------------
// Reference:
// https://github.com/google/glog/blob/master/src/base/mutex.h
//---------------------------------------------------------

#include <pthread.h>

namespace uorb {
namespace base {

// The read/write mutex type.
class RwMutex {
#define SAFE_PTHREAD_MUTEX(fncall)          \
  do { /* run fncall if is_safe_ is true */ \
    if (is_safe_) fncall(&mutex_);          \
  } while (0)

 public:
#ifdef PTHREAD_RWLOCK_INITIALIZER
  constexpr RwMutex() noexcept = default;
  ~RwMutex() = default;
#else
  RwMutex() noexcept {
    SetIsSafe();
    pthread_rwlock_init((&mutex_), nullptr);
  }
  ~RwMutex() noexcept { SAFE_PTHREAD_MUTEX(pthread_rwlock_destroy); }
#endif
  RwMutex(const RwMutex &) = delete;
  RwMutex &operator=(const RwMutex &) = delete;

  void lock() { SAFE_PTHREAD_MUTEX(pthread_rwlock_wrlock); }
  void unlock() { SAFE_PTHREAD_MUTEX(pthread_rwlock_unlock); }

  bool tryLock() noexcept {
    return is_safe_ ? 0 == pthread_rwlock_trywrlock(&mutex_) : true;
  }

  void reader_lock() { SAFE_PTHREAD_MUTEX(pthread_rwlock_rdlock); }
  void reader_unlock() { SAFE_PTHREAD_MUTEX(pthread_rwlock_unlock); }

#undef SAFE_PTHREAD_MUTEX

 private:
#ifdef PTHREAD_RWLOCK_INITIALIZER
  pthread_rwlock_t mutex_ = PTHREAD_RWLOCK_INITIALIZER;
  volatile bool is_safe_{true};
#else
  pthread_rwlock_t mutex_{};

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
 * A LockGuard controls RwMutex ownership within a scope, releasing
 * ownership in the destructor.
 */
class ReaderLockGuard {
 public:
  explicit ReaderLockGuard(RwMutex &m) : mutex_(m) { mutex_.reader_lock(); }
  ~ReaderLockGuard() { mutex_.reader_unlock(); }

  ReaderLockGuard(const ReaderLockGuard &) = delete;
  ReaderLockGuard &operator=(const ReaderLockGuard &) = delete;

 private:
  RwMutex &mutex_;
};

class WriterLockGuard {
 public:
  explicit WriterLockGuard(RwMutex &m) : mutex_(m) { mutex_.lock(); }
  ~WriterLockGuard() { mutex_.unlock(); }

  WriterLockGuard(const WriterLockGuard &) = delete;
  WriterLockGuard &operator=(const WriterLockGuard &) = delete;

 private:
  RwMutex &mutex_;
};

}  // namespace base
}  // namespace uorb
