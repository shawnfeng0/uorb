#pragma once

#include <atomic>
#include <cerrno>

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
  EventSource() = default;
  EventSource(const EventSource &) = delete;
  EventSource(EventSource &&) = delete;
  EventSource &operator=(const EventSource &) = delete;
  EventSource &operator=(EventSource &&) = delete;
  virtual ~EventSource() = default;

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
    wakeup_.store(nullptr, std::memory_order_release);
    return true;
  }

  // Clear wakeup_ atomically WITHOUT calling OnRemoved().
  // Used by EventPoll::Remove under the lock; call OnRemoved() after
  // releasing the lock to avoid lock-ordering deadlocks.
  void ClearWakeup() {
    wakeup_.store(nullptr, std::memory_order_release);
  }

  // Called when the source is removed from an EventPoll. Override to perform
  // cleanup (e.g., unregister callbacks). This is called BEFORE wakeup_ is
  // cleared, so HasWakeup() still returns true. Can be called outside the
  // EventPoll lock to avoid lock-ordering deadlocks.
  virtual void OnRemoved() {}

  bool HasWakeup() const { return wakeup_.load(std::memory_order_acquire) != nullptr; }
  bool HasWakeup(const EventPoll *poll) const {
    return wakeup_.load(std::memory_order_acquire) == poll;
  }

  virtual bool is_ready() const = 0;

  // Wake the EventPoll blocked in Wait(). Call this from the event-producing
  // path (e.g., publisher thread). Thread-safe.
  void notify_waiters();

 private:
  std::atomic<EventPoll *> wakeup_{nullptr};
};

}  // namespace uevent
