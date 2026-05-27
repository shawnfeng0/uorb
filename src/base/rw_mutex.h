//
// Copyright (c) 2021 shawnfeng. All rights reserved.
//
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
 public:
#ifdef PTHREAD_RWLOCK_INITIALIZER
  constexpr RwMutex() noexcept = default;
  ~RwMutex() = default;
#else
  RwMutex() noexcept { pthread_rwlock_init(&mutex_, nullptr); }
  ~RwMutex() noexcept { pthread_rwlock_destroy(&mutex_); }
#endif
  RwMutex(const RwMutex &) = delete;
  RwMutex &operator=(const RwMutex &) = delete;

  void lock() { pthread_rwlock_wrlock(&mutex_); }
  void unlock() { pthread_rwlock_unlock(&mutex_); }

  bool try_lock() noexcept { return 0 == pthread_rwlock_trywrlock(&mutex_); }

  void reader_lock() { pthread_rwlock_rdlock(&mutex_); }
  void reader_unlock() { pthread_rwlock_unlock(&mutex_); }

 private:
#ifdef PTHREAD_RWLOCK_INITIALIZER
  pthread_rwlock_t mutex_ = PTHREAD_RWLOCK_INITIALIZER;
#else
  pthread_rwlock_t mutex_{};
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
