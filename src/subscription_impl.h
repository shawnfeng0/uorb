//
// Created by fs on 2020-09-07.
//

#pragma once

#include "device_node.h"

namespace uorb {

struct SubscriptionImpl {
  explicit SubscriptionImpl(DeviceNode &device_node) : dev_(device_node) {
    last_generation_ = device_node.initial_generation();
    dev_.add_subscriber();
  }

  ~SubscriptionImpl() { dev_.remove_subscriber(); }

  bool Copy(void *buffer) { return dev_.Copy(buffer, last_generation_); }
  unsigned updates_available() const {
    return dev_.updates_available(last_generation_);
  }

  void UnregisterCallback(DeviceNode::Callback *callback) {
    dev_.UnregisterCallback(callback);
  }

  bool RegisterCallback(DeviceNode::Callback *callback) {
    return dev_.RegisterCallback(callback);
  }

 private:
  DeviceNode &dev_;
  unsigned last_generation_{}; /**< last generation the subscriber has seen */
};
}  // namespace uorb
