#pragma once

#include <uorb/uorb.h>

#include <functional>
#include <unordered_map>
#include <vector>

namespace uorb {

class EventLoop {
 public:
  EventLoop() : event_poll_(orb_event_poll_create()) {}
  ~EventLoop() {
    for (const auto &pair : sub_callbacks_) {
      auto sub = pair.first;
      orb_event_poll_remove(event_poll_, sub);
      orb_destroy_subscription(&sub);
    }
    for (const auto &pair : external_sub_callbacks_) {
      const auto sub = pair.first;
      orb_event_poll_remove(event_poll_, sub);
    }
    orb_event_poll_destroy(&event_poll_);
  }

  template <typename Sub>
  void RegisterCallback(Sub &sub_cpp, const std::function<void(const typename Sub::ValueType &)> &cb) {
    RegisterCallback(external_sub_callbacks_, sub_cpp.handle(), cb);
  }

  template <typename Sub>
  void UnRegisterCallback(Sub &sub_cpp) {
    UnRegisterCallback(external_sub_callbacks_, sub_cpp.handle());
  }

  template <const orb_metadata &meta>
  void RegisterCallback(const std::function<void(const typename msg::TypeMap<meta>::type &)> &cb) {
    orb_subscription_t *sub = orb_create_subscription(&meta);
    if (!sub) return;

    RegisterCallback(sub_callbacks_, sub, cb);
  }

  int PollOnce(const int timeout_ms = -1) {
    std::vector<orb_subscription_t *> ready_subs(sub_callbacks_.size(), nullptr);
    const int number_of_event =
        orb_event_poll_wait(event_poll_, ready_subs.data(), static_cast<int>(ready_subs.size()), timeout_ms);

    for (int i = 0; i < number_of_event; ++i) {
      for (auto callbacks : {sub_callbacks_, external_sub_callbacks_}) {
        if (auto it = callbacks.find(ready_subs[i]); it != callbacks.end()) {
          it->second();
        }
      }
    }

    return number_of_event;
  }

  void Loop() { while (PollOnce(-1) >= 0); }

  // Quit the event loop (thread-safe, can be called from another thread)
  void Quit() const { orb_event_poll_quit(event_poll_); }

 private:
  template <typename Msg>
  void RegisterCallback(std::unordered_map<orb_subscription_t *, std::function<void()>> &callbacks,
                        orb_subscription_t *sub, const std::function<void(const Msg &)> &cb) {
    orb_event_poll_add(event_poll_, sub);
    callbacks[sub] = [sub, cb] {
      Msg msg;
      if (orb_copy(sub, &msg)) {
        cb(msg);
      }
    };
  }

  void UnRegisterCallback(std::unordered_map<orb_subscription_t *, std::function<void()>> &callbacks,
                          orb_subscription_t *sub) const {
    callbacks.erase(sub);
    orb_event_poll_remove(event_poll_, sub);
  }

  orb_event_poll_t *event_poll_;
  std::unordered_map<orb_subscription_t *, std::function<void()>> sub_callbacks_;
  std::unordered_map<orb_subscription_t *, std::function<void()>> external_sub_callbacks_;
};

}  // namespace uorb
