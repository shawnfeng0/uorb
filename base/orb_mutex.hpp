#pragma once

#include <pthread.h>

namespace uORB {

// Common base class for Mutex
class __mutex_base {
 protected:
#ifdef PTHREAD_MUTEX_INITIALIZER
  pthread_mutex_t mutex_ = PTHREAD_MUTEX_INITIALIZER;
  constexpr __mutex_base() noexcept = default;
#else
  pthread_mutex_t mutex_;

  __mutex_base() noexcept {
    // XXX EAGAIN, ENOMEM, EPERM, EBUSY(may), EINVAL(may)
    pthread_mutex_init((&mutex_), nullptr);
  }

  ~__mutex_base() noexcept { pthread_mutex_destroy(&mutex_); }
#endif

 public:
  __mutex_base(const __mutex_base &) = delete;
  __mutex_base &operator=(const __mutex_base &) = delete;
};

/// The standard Mutex type.
class Mutex : private __mutex_base {
 public:
#ifdef PTHREAD_MUTEX_INITIALIZER
  constexpr
#endif
  Mutex() noexcept = default;
  ~Mutex() = default;

  Mutex(const Mutex &) = delete;
  Mutex &operator=(const Mutex &) = delete;

  void lock() {
    // XXX EINVAL, EAGAIN, EBUSY, EINVAL, EDEADLK(may)
    pthread_mutex_lock(&mutex_);
  }

  bool try_lock() noexcept {
    // XXX EINVAL, EAGAIN, EBUSY
    return !pthread_mutex_trylock(&mutex_);
  }

  void unlock() {
    // XXX EINVAL, EAGAIN, EPERM
    pthread_mutex_unlock(&mutex_);
  }

  pthread_mutex_t *native_handle() noexcept { return &mutex_; }
};

/** @brief A simple scoped lock type.
 *
 * A LockGuard controls Mutex ownership within a scope, releasing
 * ownership in the destructor.
 */
template <typename _Mutex>
class LockGuard {
 public:
  typedef _Mutex MutexType;

  explicit LockGuard(MutexType &m) : mutex_(m) { mutex_.lock(); }

  ~LockGuard() { mutex_.unlock(); }

  LockGuard(const LockGuard &) = delete;
  LockGuard &operator=(const LockGuard &) = delete;

 private:
  MutexType &mutex_;
};

typedef LockGuard<Mutex> MutexGuard;

}  // namespace uORB
