#pragma once

#include <uevent/uevent.h>

#include <atomic>
#include <cerrno>
#include <pthread.h>

namespace uevent {

class EventPoll;

/**
 * EventSource: internal abstract base for anything that can be polled by
 * EventPoll.
 *
 * Users do not subclass this directly — use uevent_source_create() (C API)
 * to create custom event sources. This class is an internal implementation
 * detail shared by CEventSource (C callback wrapper) and EventPoll.
 */
class EventSource {
 public:
#ifdef PTHREAD_MUTEX_INITIALIZER
  EventSource() = default;
  virtual ~EventSource() = default;
#else
  EventSource() { pthread_mutex_init(&notify_mu_, nullptr); }
  virtual ~EventSource() { pthread_mutex_destroy(&notify_mu_); }
#endif
  EventSource(const EventSource &) = delete;
  EventSource(EventSource &&) = delete;
  EventSource &operator=(const EventSource &) = delete;
  EventSource &operator=(EventSource &&) = delete;

  virtual bool SetWakeup(EventPoll *poll) {
    if (!poll) {
      errno = EINVAL;
      return false;
    }
    auto *expected = static_cast<EventPoll *>(nullptr);
    if (!wakeup_.compare_exchange_strong(expected, poll, std::memory_order_acq_rel)) {
      if (expected == poll) return true;
      errno = EBUSY;
      return false;
    }
    return true;
  }

  virtual bool RemoveWakeup() {
    OnRemoved();
    ClearWakeup();
    return true;
  }

  // Clear wakeup_ under notify_mu_. Blocks until any in-flight
  // notify_waiters() call completes, then sets wakeup_ to nullptr.
  // Must be called OUTSIDE EventPoll::mu_ — notify_waiters() calls
  // NotifyReady which acquires mu_, causing AB-BA deadlock if
  // called while mu_ is held.
  void ClearWakeup();

  // Called when the source is removed from an EventPoll. Override to perform
  // cleanup (e.g., unregister callbacks). This is called BEFORE wakeup_ is
  // cleared, so HasWakeup() still returns true. Can be called outside the
  // EventPoll lock to avoid lock-ordering deadlocks.
  virtual void OnRemoved() {}

  bool HasWakeup() const { return wakeup_.load(std::memory_order_acquire) != nullptr; }
  bool HasWakeup(const EventPoll *poll) const {
    return wakeup_.load(std::memory_order_acquire) == poll;
  }

  bool in_ready_queue() const {
    return in_ready_queue_.load(std::memory_order_relaxed);
  }
  void set_in_ready_queue(bool v) {
    in_ready_queue_.store(v, std::memory_order_release);
  }

  uevent_data_t user_data() const { return user_data_; }
  void set_user_data(uevent_data_t data) { user_data_ = data; }

  virtual bool is_ready() const = 0;

  // Wake the EventPoll blocked in Wait(). Call this from the event-producing
  // path (e.g., publisher thread). Thread-safe.
  void notify_waiters();

 private:
  friend class EventPoll;

  pthread_mutex_t notify_mu_
#ifdef PTHREAD_MUTEX_INITIALIZER
      = PTHREAD_MUTEX_INITIALIZER
#endif
  ;  // serializes notify_waiters() vs ClearWakeup()
  std::atomic<EventPoll *> wakeup_{nullptr};
  std::atomic<bool> in_ready_queue_{false};
  uevent_data_t user_data_{nullptr};

  // Intrusive doubly-linked list links owned by EventPoll (guarded by EventPoll::mu_).
  EventSource *poll_prev_{nullptr};
  EventSource *poll_next_{nullptr};
  bool in_poll_list_{false};
};

}  // namespace uevent
