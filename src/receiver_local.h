//
// Copyright (c) 2021 shawnfeng. All rights reserved.
//
#pragma once

#include "device_node.h"

namespace uorb {

class ReceiverLocal final : public detail::ReceiverBase {
 public:
  explicit ReceiverLocal(DeviceNode &device_node) : ReceiverBase(), dev_(device_node) {
    last_generation_ = device_node.initial_generation();
    dev_.add_subscriber();
  }

  void notify_all() override {
    if (notifier_) notifier_->notify_all();
  }

  ~ReceiverLocal() override {
    RemoveNotifier();
    dev_.remove_subscriber();
  }

  bool Copy(void *buffer) { return dev_.Copy(buffer, &last_generation_); }
  unsigned updates_available() const { return dev_.updates_available(last_generation_); }
  bool is_ready() const { return updates_available() > 0; }

  bool SetNotifier(base::LiteNotifier *notifier) {
    if (!notifier) {
      errno = EINVAL;
      return false;
    }

    if (notifier_ == notifier) {
      return true;
    }

    if (notifier_) {
      errno = EBUSY;
      return false;
    }

    if (!dev_.RegisterCallback(this)) {
      return false;
    }

    notifier_ = notifier;
    return true;
  }

  bool RemoveNotifier() {
    if (!notifier_) {
      return true;
    }

    if (!dev_.UnregisterCallback(this)) {
      return false;
    }

    notifier_ = nullptr;
    return true;
  }

  bool HasNotifier() const { return notifier_ != nullptr; }
  bool HasNotifier(const base::LiteNotifier *notifier) const { return notifier_ == notifier; }

  intrusive_list::forward_list_node event_poll_node{};

 private:
  base::LiteNotifier *notifier_ = nullptr;
  DeviceNode &dev_;
  unsigned last_generation_{}; /**< last generation the subscriber has seen */
};
}  // namespace uorb
