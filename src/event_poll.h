//
// Copyright (c) 2025 shawnfeng. All rights reserved.
//
#pragma once

#include "receiver_local.h"

namespace uorb {

// EventPoll: notifier-based wakeup + scan for ready receivers.
//
// Publish path: zero extra overhead (only notifier wakeup).
// Wait path: linear scan over the subscription list.
//
// Thread-safety:
//  - Stop() may be called from any thread.
//  - All other methods must be called from a single thread.
class EventPoll {
 public:
  ~EventPoll() {
    for (auto &receiver : receivers_) {
      receiver.RemoveNotifier();
    }
  }

  bool AddReceiver(ReceiverLocal &receiver) {
    if (receiver.HasNotifier(&notifier_)) {
      return true;
    }

    if (!receiver.SetNotifier(&notifier_)) {
      return false;
    }

    receivers_.push_front(receiver);
    return true;
  }

  bool RemoveReceiver(ReceiverLocal &receiver) {
    if (!receiver.HasNotifier(&notifier_)) {
      errno = receiver.HasNotifier() ? EBUSY : EINVAL;
      return false;
    }

    if (!receivers_.remove(receiver)) {
      errno = ENOENT;
      return false;
    }

    return receiver.RemoveNotifier();
  }

  int Wait(ReceiverLocal *ready[], int max_ready, const int timeout_ms) {
    if (!ready || max_ready < 0) {
      errno = EINVAL;
      return -1;
    }

    if (stop_) return -1;

    int count = ScanReady(ready, max_ready);
    if (count > 0) return count;

    if (timeout_ms > 0) {
      notifier_.wait_for(timeout_ms, [&] {
        count = ScanReady(ready, max_ready);
        return stop_ || count > 0;
      });
    } else if (timeout_ms < 0) {
      notifier_.wait([&] {
        count = ScanReady(ready, max_ready);
        return stop_ || count > 0;
      });
    }

    if (stop_) return -1;
    return count;
  }

  void Stop() {
    stop_ = true;
    notifier_.notify_all();
  }

 private:
  int ScanReady(ReceiverLocal *ready[], int max_ready) {
    int count = 0;
    for (auto &receiver : receivers_) {
      if (!receiver.is_ready()) {
        continue;
      }

      if (count < max_ready) {
        ready[count] = &receiver;
      }
      ++count;

      if (max_ready > 0 && count >= max_ready) {
        break;
      }
    }
    return count;
  }

  base::LiteNotifier notifier_;
  intrusive_list::forward_list<ReceiverLocal, &ReceiverLocal::event_poll_node> receivers_;
  std::atomic_bool stop_{false};
};
}  // namespace uorb
