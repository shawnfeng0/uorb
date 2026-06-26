//
// Copyright (c) 2021 shawnfeng. All rights reserved.
//
#pragma once

//---------------------------------------------------------
// Reference:
// https://github.com/google/glog/blob/master/src/base/mutex.h
//---------------------------------------------------------

#include <pthread.h>

namespace uevent {
namespace base {

/// The standard Mutex type.
class Mutex {
 public:
  Mutex(const Mutex &) = delete;
  Mutex(Mutex &&) = delete;
  Mutex &operator=(const Mutex &) = delete;
  Mutex &operator=(Mutex &&) = delete;
#ifdef PTHREAD_MUTEX_INITIALIZER
  constexpr Mutex() noexcept = default;
  ~Mutex() = default;
#else
  Mutex() noexcept { pthread_mutex_init(&mutex_, nullptr); }
  ~Mutex() noexcept { pthread_mutex_destroy(&mutex_); }
#endif
  void lock() { pthread_mutex_lock(&mutex_); }
  void unlock() { pthread_mutex_unlock(&mutex_); }

  bool try_lock() noexcept { return 0 == pthread_mutex_trylock(&mutex_); }

  pthread_mutex_t *native_handle() noexcept { return &mutex_; }

 private:
#ifdef PTHREAD_MUTEX_INITIALIZER
  pthread_mutex_t mutex_ = PTHREAD_MUTEX_INITIALIZER;
#else
  pthread_mutex_t mutex_{};
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

/**
 * @brief A movable scoped lock with manual unlock support.
 *
 * Like std::unique_lock, allows explicit unlock() before scope exit.
 * Destructor releases the lock if still held. Used by EventPoll::Wait()
 * where the lock must be held across condition variable waits but
 * released before early returns.
 */
template <typename MutexType = Mutex>
class UniqueLock {
 public:
  explicit UniqueLock(MutexType &m) : mutex_(&m), owns_(true) { m.lock(); }
  UniqueLock(MutexType &m, bool defer_lock) : mutex_(&m), owns_(false) {
    if (!defer_lock) {
      m.lock();
      owns_ = true;
    }
  }
  ~UniqueLock() {
    if (owns_) mutex_->unlock();
  }

  UniqueLock(const UniqueLock &) = delete;
  UniqueLock &operator=(const UniqueLock &) = delete;

  void lock() {
    mutex_->lock();
    owns_ = true;
  }

  void unlock() {
    if (owns_) {
      owns_ = false;
      mutex_->unlock();
    }
  }

  bool owns_lock() const { return owns_; }
  MutexType *mutex() { return mutex_; }

 private:
  MutexType *mutex_;
  bool owns_;
};

}  // namespace base
}  // namespace uevent

