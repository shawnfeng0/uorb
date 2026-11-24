//
// Copyright (c) 2025 shawnfeng. All rights reserved.
//

#include <uevent/uevent_source.h>

#include "event_poll.h"

namespace uevent {

void EventSource::notify_waiters() {
  if (!wakeup_.load(std::memory_order_acquire)) {
    return;
  }
  // Dekker fence: pairs with set_in_ready_queue(false) + seq_cst fence before
  // is_ready() in EventPoll::Wait(). Skips notify_mu_ and EventPoll::mu_ when
  // this source is already queued for delivery.
  std::atomic_thread_fence(std::memory_order_seq_cst);
  if (in_ready_queue_.load(std::memory_order_acquire)) {
    return;
  }
  pthread_mutex_lock(&notify_mu_);
  auto *poll = wakeup_.load(std::memory_order_relaxed);
  if (poll) poll->NotifyReady(this);
  pthread_mutex_unlock(&notify_mu_);
}

void EventSource::ClearWakeup() {
  pthread_mutex_lock(&notify_mu_);
  wakeup_.store(nullptr, std::memory_order_release);
  pthread_mutex_unlock(&notify_mu_);
}

}  // namespace uevent
