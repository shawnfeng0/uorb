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
    if (notifier_) {
      dev_.UnregisterCallback(this);
    }
    dev_.remove_subscriber();
  }

  bool Copy(void *buffer) { return dev_.Copy(buffer, &last_generation_); }
  unsigned updates_available() const { return dev_.updates_available(last_generation_); }

  void SetNotifier(base::LiteNotifier *notifier) {
    notifier_ = notifier;
    dev_.RegisterCallback(this);
  }

  void RemoveNotifier() {
    dev_.UnregisterCallback(this);
    notifier_ = nullptr;
  }

 public:
  intrusive_list::forward_list_node event_poll_node{};

 private:
  base::LiteNotifier *notifier_ = nullptr;

  DeviceNode &dev_;
  unsigned last_generation_{}; /**< last generation the subscriber has seen */
};
}  // namespace uorb
