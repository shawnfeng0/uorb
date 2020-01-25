#pragma once

#include <pthread.h>

namespace uORB {

typedef pthread_mutex_t __orb_thread_mutex_t;

#ifdef PTHREAD_MUTEX_INITIALIZER
#define __ORB_THREAD_MUTEX_INITIALIZER PTHREAD_MUTEX_INITIALIZER
#endif
#define __ORB_THREAD_MUTEX_INIT(mutex) pthread_mutex_init((mutex), nullptr)
#define __ORB_THREAD_MUTEX_DESTROY pthread_mutex_destroy
#define __ORB_THREAD_MUTEX_LOCK pthread_mutex_lock
#define __ORB_THREAD_MUTEX_UNLOCK pthread_mutex_unlock
#define __ORB_THREAD_MUTEX_TRYLOCK pthread_mutex_trylock

// Common base class for Mutex
class __mutex_base
{
protected:
#ifdef __ORB_THREAD_MUTEX_INITIALIZER
  __orb_thread_mutex_t _M_mutex = __ORB_THREAD_MUTEX_INITIALIZER;

  constexpr __mutex_base() noexcept = default;
#else
  __orb_thread_mutex_t  _M_mutex;

    __mutex_base() noexcept
    {
      // XXX EAGAIN, ENOMEM, EPERM, EBUSY(may), EINVAL(may)
      __ORB_THREAD_MUTEX_INIT(&_M_mutex);
    }

    ~__mutex_base() noexcept { __ORB_THREAD_MUTEX_DESTROY(&_M_mutex); }
#endif

  __mutex_base(const __mutex_base&) = delete;
  __mutex_base& operator=(const __mutex_base&) = delete;
};

/// The standard Mutex type.
class Mutex : private __mutex_base
{
public:
#ifdef __ORB_THREAD_MUTEX_INITIALIZER
  constexpr
#endif
      Mutex() noexcept = default;
  ~Mutex() = default;

  Mutex(const Mutex &) = delete;
  Mutex & operator=(const Mutex &) = delete;

  void
  lock()
  {
    // EINVAL, EAGAIN, EBUSY, EINVAL, EDEADLK(may)
    __ORB_THREAD_MUTEX_LOCK(&_M_mutex);
  }

  bool
  try_lock() noexcept
  {
    // XXX EINVAL, EAGAIN, EBUSY
    return !__ORB_THREAD_MUTEX_TRYLOCK(&_M_mutex);
  }

  void
  unlock()
  {
    // XXX EINVAL, EAGAIN, EPERM
    __ORB_THREAD_MUTEX_UNLOCK(&_M_mutex);
  }

  __orb_thread_mutex_t *
  native_handle() noexcept
  { return &_M_mutex; }
};

/** @brief A simple scoped lock type.
 *
 * A LockGuard controls Mutex ownership within a scope, releasing
 * ownership in the destructor.
 */
template<typename _Mutex>
class LockGuard {
public:
  typedef _Mutex MutexType;

  explicit LockGuard(MutexType & __m) : _M_device(__m)
  { _M_device.lock(); }

  ~LockGuard()
  { _M_device.unlock(); }

  LockGuard(const LockGuard &) = delete;
  LockGuard & operator=(const LockGuard &) = delete;

private:
  MutexType &  _M_device;
};

typedef LockGuard<Mutex> MutexGuard;

} // namespace orb
