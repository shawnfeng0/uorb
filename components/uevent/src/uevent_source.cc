//
// Copyright (c) 2025 shawnfeng. All rights reserved.
//

#include <uevent/uevent_source.h>

#include "event_poll.h"

namespace uevent {

void EventSource::notify_waiters() {
  notify_count_.fetch_add(1, std::memory_order_acq_rel);
  auto *poll = wakeup_.load(std::memory_order_acquire);
  if (poll) poll->NotifyReady(this);
  notify_count_.fetch_sub(1, std::memory_order_release);
}

}  // namespace uevent
