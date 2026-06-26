//
// Copyright (c) 2025 shawnfeng. All rights reserved.
//

#include <uevent/uevent_source.h>

#include "event_poll.h"

namespace uevent {

void EventSource::notify_waiters() {
  std::lock_guard<std::mutex> lk(notify_mu_);
  auto *poll = wakeup_.load(std::memory_order_acquire);
  if (poll) poll->NotifyReady(this);
}

}  // namespace uevent
