#pragma once

#include <uorb/subscription.h>
#include <uorb/uorb.h>

#include <atomic>
#include <functional>
#include <thread>
#include <unordered_map>
#include <vector>

namespace uorb {

class EventLoop {
 public:
  EventLoop() : _poll(nullptr) { _poll = orb_event_poll_create(); }
  ~EventLoop() {
    if (_poll) {
      orb_event_poll_destroy(&_poll);
      _poll = nullptr;
    }
    for (auto &pair : _callbacks) {
      auto sub = pair.first;
      orb_destroy_subscription(&sub);
    }
  }

  template <typename Msg>
  void Listen(const orb_metadata &meta, std::function<void(const Msg &)> cb) {
    orb_subscription_t *sub = orb_create_subscription(&meta);
    if (!sub) return;
    orb_event_poll_add(_poll, sub);
    _callbacks[sub] = [cb, sub]() {
      Msg msg;
      if (orb_copy(sub, &msg)) {
        cb(msg);
      }
    };
  }

  void Loop(int timeout_ms = -1) {
    while (true) {
      std::vector<orb_subscription_t *> ready_subs(_callbacks.size(), nullptr);
      int n = orb_event_poll_wait(_poll, ready_subs.data(), static_cast<int>(ready_subs.size()), timeout_ms);
      if (n > 0) {
        for (int i = 0; i < n; ++i) {
          orb_subscription_t *sub = ready_subs[i];
          auto it = _callbacks.find(sub);
          if (it != _callbacks.end()) {
            it->second();
          }
        }
      } else if (n < 0) {
        // error or interrupted
        break;
      }
    }
  }

  // Quit the event loop (thread-safe, can be called from another thread)
  void Quit() const {
    if (_poll) {
      orb_event_poll_quit(_poll);
    }
  }

 private:
  orb_event_poll_t *_poll;
  std::unordered_map<orb_subscription_t *, std::function<void()>> _callbacks;
};

}  // namespace uorb
