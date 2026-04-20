#pragma once

#include <uorb/uorb.h>

#include <algorithm>
#include <functional>
#include <vector>

namespace uorb {

class EventLoop {
 public:
  EventLoop() : event_poll_(orb_event_poll_create()) {}
  ~EventLoop() {
    for (auto &entry : entries_) {
      orb_event_poll_remove(event_poll_, entry.sub);
      if (entry.owned) {
        orb_destroy_subscription(&entry.sub);
      }
    }
    orb_event_poll_destroy(&event_poll_);
  }

  // Register a callback on an externally managed subscription.
  template <typename Sub>
  void RegisterCallback(Sub &sub_cpp, const std::function<void(const typename Sub::ValueType &)> &cb) {
    AddEntry(sub_cpp.handle(), cb, false);
  }

  // Unregister a callback previously registered with an external subscription.
  template <typename Sub>
  void UnRegisterCallback(Sub &sub_cpp) {
    RemoveEntry(sub_cpp.handle());
  }

  // Register a callback for a topic; the subscription is owned by EventLoop.
  template <const orb_metadata &meta>
  void RegisterCallback(const std::function<void(const typename msg::TypeMap<meta>::type &)> &cb) {
    orb_subscription_t *sub = orb_create_subscription(&meta);
    if (!sub) return;
    AddEntry(sub, cb, true);
  }

  int PollOnce(const int timeout_ms = -1) {
    ready_buf_.resize(entries_.size());
    const int number_of_event =
        orb_event_poll_wait(event_poll_, ready_buf_.data(), static_cast<int>(ready_buf_.size()), timeout_ms);

    for (int i = 0; i < number_of_event; ++i) {
      // Linear search is efficient for the small subscriber counts EventLoop
      // is designed for, and avoids the hash-table overhead of a map lookup.
      for (const auto &entry : entries_) {
        if (entry.sub == ready_buf_[i]) {
          entry.callback();
          break;
        }
      }
    }

    return number_of_event;
  }

  void Loop() {
    while (PollOnce(-1) >= 0) {
    }
  }

  // Quit the event loop (thread-safe, can be called from another thread)
  void Quit() const { orb_event_poll_quit(event_poll_); }

 private:
  struct Entry {
    orb_subscription_t *sub;
    std::function<void()> callback;
    bool owned;  // true when EventLoop created and owns the subscription

    Entry(orb_subscription_t *sub, std::function<void()> callback, bool owned)
        : sub(sub), callback(std::move(callback)), owned(owned) {}
  };

  template <typename Msg>
  void AddEntry(orb_subscription_t *sub, const std::function<void(const Msg &)> &cb, bool owned) {
    orb_event_poll_add(event_poll_, sub);
    entries_.emplace_back(sub,
                          [sub, cb] {
                            Msg msg;
                            if (orb_copy(sub, &msg)) {
                              cb(msg);
                            }
                          },
                          owned);
  }

  void RemoveEntry(orb_subscription_t *sub) {
    auto it = std::find_if(entries_.begin(), entries_.end(), [sub](const Entry &e) { return e.sub == sub; });
    if (it != entries_.end()) {
      orb_event_poll_remove(event_poll_, it->sub);
      // External entries are not owned; do not destroy the subscription here.
      entries_.erase(it);
    }
  }

  orb_event_poll_t *event_poll_;
  std::vector<Entry> entries_;
  std::vector<orb_subscription_t *> ready_buf_;
};

}  // namespace uorb
