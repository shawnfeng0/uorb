//
// Copyright (c) 2021 shawnfeng. All rights reserved.
//
#pragma once

#include <atomic>

#include "device_node.h"

namespace uorb {

class ReceiverLocal final : public detail::ReceiverBase {
 public:
  explicit ReceiverLocal(DeviceNode &device_node) : ReceiverBase(), dev_(device_node) {
    last_generation_ = device_node.initial_generation();
    dev_.add_subscriber();
  }

  void notify_all() override {
    auto *notifier = notifier_.load(std::memory_order_acquire);
    if (notifier) {
      notifier->notify_all();
    }
  }

  ~ReceiverLocal() override {
    RemoveNotifier();
    dev_.remove_subscriber();
  }

  DeviceNode *device_node() { return &dev_; }
  bool Copy(void *buffer) { return dev_.Copy(buffer, &last_generation_); }
  unsigned updates_available() const { return dev_.updates_available(last_generation_); }
  bool is_ready() const { return updates_available() > 0; }

  bool SetNotifier(base::LiteNotifier *notifier) {
    if (!notifier) {
      errno = EINVAL;
      return false;
    }

    auto *expected_notifier = static_cast<base::LiteNotifier *>(nullptr);
    if (!notifier_.compare_exchange_strong(expected_notifier, notifier, std::memory_order_acq_rel)) {
      if (expected_notifier == notifier) {
        return true;
      }

      errno = EBUSY;
      return false;
    }

    if (!dev_.RegisterCallback(this)) {
      notifier_.store(nullptr, std::memory_order_release);
      return false;
    }

    return true;
  }

  bool RemoveNotifier() {
    auto *notifier = notifier_.load(std::memory_order_acquire);
    if (!notifier) {
      return true;
    }

    if (!dev_.UnregisterCallback(this)) {
      return false;
    }

    notifier_.store(nullptr, std::memory_order_release);
    return true;
  }

  bool HasNotifier() const { return notifier_.load(std::memory_order_acquire) != nullptr; }
  bool HasNotifier(const base::LiteNotifier *notifier) const {
    return notifier_.load(std::memory_order_acquire) == notifier;
  }

  intrusive_list::forward_list_node event_poll_node{};

 private:
  std::atomic<base::LiteNotifier *> notifier_{nullptr};
  DeviceNode &dev_;
  unsigned last_generation_{}; /**< last generation the subscriber has seen */
};
}  // namespace uorb
