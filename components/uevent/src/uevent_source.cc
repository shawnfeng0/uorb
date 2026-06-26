//
// Copyright (c) 2025 shawnfeng. All rights reserved.
//

#include <uevent/uevent_source.h>

#include "event_poll.h"

namespace uevent {

void EventSource::notify_waiters() {
  pthread_mutex_lock(&notify_mu_);
  auto *poll = wakeup_.load(std::memory_order_acquire);
  if (poll) poll->NotifyReady(this);
  pthread_mutex_unlock(&notify_mu_);
}

void EventSource::ClearWakeup() {
  pthread_mutex_lock(&notify_mu_);
  wakeup_.store(nullptr, std::memory_order_release);
  pthread_mutex_unlock(&notify_mu_);
}

}  // namespace uevent
