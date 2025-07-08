//
// Copyright (c) 2025 shawnfeng. All rights reserved.
//
#pragma once

#include "receiver_local.h"

namespace uorb {

class EventPoll {
 public:
  ~EventPoll() {
    for (auto &receiver : event_poll_list_) {
      receiver.RemoveNotifier();
    }
  }
  void AddReceiver(ReceiverLocal &receiver) {
    receiver.SetNotifier(&notifier_);
    event_poll_list_.push_front(receiver);
  }
  void RemoveReceiver(ReceiverLocal &receiver) {
    receiver.RemoveNotifier();
    event_poll_list_.remove(receiver);
  }

  int Wait(ReceiverLocal *receivers[], int max_receivers, const int timeout_ms) {
    int number_of_new_data = 0;

    auto CheckDataUpdate = [&] {
      for (auto &item_sub : event_poll_list_) {
        if (item_sub.updates_available()) {
          receivers[number_of_new_data++] = &item_sub;
          if (number_of_new_data >= max_receivers) {
            break;  // Reached the maximum number of receivers
          }
        }
      }
      return number_of_new_data > 0;
    };

    if (CheckDataUpdate()) return number_of_new_data;

    if (timeout_ms > 0) {
      notifier_.wait_for(timeout_ms, CheckDataUpdate);
    } else if (timeout_ms < 0) {
      notifier_.wait(CheckDataUpdate);
    }

    return number_of_new_data;
  }

 private:
  base::LiteNotifier notifier_;
  intrusive_list::forward_list<ReceiverLocal, &ReceiverLocal::event_poll_node> event_poll_list_;
};
}  // namespace uorb
