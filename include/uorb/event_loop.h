#pragma once

#include <uorb/uorb.h>

#include <functional>
#include <unordered_map>
#include <utility>
#include <vector>

namespace uorb {

// EventLoop: register callbacks on subscriptions and dispatch them when new
// data becomes available.
//
// Thread-safety:
//  - Quit() is thread-safe and may be called from any thread.
//  - All other member functions (RegisterCallback, UnregisterCallback,
//    Subscribe, PollOnce, Loop) must be invoked from a single thread
//    (typically the loop thread).
class EventLoop {
 public:
  EventLoop() : event_poll_(orb_event_poll_create()) {}
  ~EventLoop() {
    if (!event_poll_) return;
    for (auto &kv : entries_) {
      orb_subscription_t *sub = kv.first;
      orb_event_poll_remove(event_poll_, sub);
      if (kv.second.owned) {
        orb_destroy_subscription(&sub);
      }
    }
    orb_event_poll_destroy(&event_poll_);
  }

  EventLoop(const EventLoop &) = delete;
  EventLoop &operator=(const EventLoop &) = delete;
  EventLoop(EventLoop &&) = delete;
  EventLoop &operator=(EventLoop &&) = delete;

  // Whether the loop was successfully constructed.
  bool valid() const { return event_poll_ != nullptr; }
  explicit operator bool() const { return valid(); }

  // Register a callback on an externally managed subscription.
  // Returns false if the loop is invalid, the subscription is null, or the
  // subscription was already registered.
  template <typename Sub, typename F>
  bool RegisterCallback(Sub &sub_cpp, F &&cb) {
    using Msg = typename Sub::ValueType;
    return AddEntry<Msg>(sub_cpp.handle(), std::forward<F>(cb), /*owned=*/false);
  }

  // Unregister a callback previously registered with an external subscription.
  // Returns false if the subscription was not registered.
  template <typename Sub>
  bool UnregisterCallback(Sub &sub_cpp) {
    return RemoveEntry(sub_cpp.handle());
  }

  // Subscribe to a topic; the subscription is owned by EventLoop.
  // Returns false on failure (loop invalid, subscription allocation failure,
  // or a subscription for this topic has already been added by this call).
  template <const orb_metadata &meta, typename F>
  bool Subscribe(F &&cb) {
    if (!event_poll_) return false;
    orb_subscription_t *sub = orb_create_subscription(&meta);
    if (!sub) return false;
    using Msg = typename msg::TypeMap<meta>::type;
    if (!AddEntry<Msg>(sub, std::forward<F>(cb), /*owned=*/true)) {
      orb_destroy_subscription(&sub);
      return false;
    }
    return true;
  }

  [[nodiscard]] int PollOnce(const int timeout_ms = -1) {
    if (!event_poll_) return -1;
    if (entries_.empty()) return 0;

    const int number_of_event = orb_event_poll_wait(
        event_poll_, ready_buf_.data(), static_cast<int>(ready_buf_.size()), timeout_ms);

    for (int i = 0; i < number_of_event; ++i) {
      auto it = entries_.find(ready_buf_[i]);
      if (it != entries_.end()) it->second.callback();
    }

    return number_of_event;
  }

  void Loop() {
    // PollOnce(-1) blocks until either a subscription has data (returns > 0)
    // or Quit() is requested (returns < 0); it never returns 0 with an
    // infinite timeout.
    while (PollOnce(-1) > 0) {
    }
  }

  // Thread-safe: request the event loop to quit.
  void Quit() {
    if (event_poll_) orb_event_poll_quit(event_poll_);
  }

 private:
  struct Entry {
    std::function<void()> callback;
    bool owned;  // EventLoop created and owns the subscription
  };

  template <typename Msg, typename F>
  bool AddEntry(orb_subscription_t *sub, F &&cb, bool owned) {
    if (!event_poll_ || !sub) return false;
    if (entries_.find(sub) != entries_.end()) {
      // Already registered; refuse to double-register.
      return false;
    }
    if (!orb_event_poll_add(event_poll_, sub)) return false;

    std::function<void(const Msg &)> user_cb(std::forward<F>(cb));
    entries_.emplace(
        sub,
        Entry{[sub, cb = std::move(user_cb)] {
                Msg msg;
                if (orb_copy(sub, &msg)) cb(msg);
              },
              owned});
    // Keep ready_buf_ sized to match entries_ so PollOnce does not have to
    // resize on every call.
    ready_buf_.resize(entries_.size());
    return true;
  }

  bool RemoveEntry(orb_subscription_t *sub) {
    if (!event_poll_) return false;
    auto it = entries_.find(sub);
    if (it == entries_.end()) return false;
    orb_event_poll_remove(event_poll_, sub);
    // External entries are not owned; do not destroy the subscription here.
    entries_.erase(it);
    ready_buf_.resize(entries_.size());
    return true;
  }

  orb_event_poll_t *event_poll_;
  std::unordered_map<orb_subscription_t *, Entry> entries_;
  std::vector<orb_subscription_t *> ready_buf_;
};

}  // namespace uorb
