//
// Copyright (c) 2025 shawnfeng. All rights reserved.
//
#pragma once

#include <uevent/uevent_source.h>

#include <atomic>
#include <cstddef>

#include "uevent_base/mutex.h"
#include "uevent_base/condition_variable.h"

namespace uevent {

class EventPoll {
 public:
  EventPoll() = default;

  ~EventPoll() {
    for (;;) {
      EventSource *s = nullptr;
      {
        base::LockGuard<base::Mutex> lk(mu_);
        if (!sources_head_) {
          ready_head_ = nullptr;
          ready_tail_ = nullptr;
          ready_count_ = 0;
          deadline_count_ = 0;
          timeout_fired_count_ = 0;
          break;
        }
        s = sources_head_;
        UnlinkSource(s);
        RemoveReadyLocked(s);
        ClearTimeoutStateLocked(s);
      }
      s->RemoveWakeup();
      // Re-check under mu_ in case a racing notify_waiters() enqueued `s`
      // before RemoveWakeup() cleared wakeup_.
      {
        base::LockGuard<base::Mutex> lk(mu_);
        RemoveReadyLocked(s);
      }
    }
  }

  EventPoll(const EventPoll &) = delete;
  EventPoll &operator=(const EventPoll &) = delete;

  uevent_err Add(EventSource &source, int timeout_ms = 0) {
    return Add(source, timeout_ms, source.user_data());
  }

  uevent_err Add(EventSource &source, int timeout_ms, uevent_data_t data) {
    if (source.HasWakeup(this)) {
      base::LockGuard<base::Mutex> lk(mu_);
      source.set_user_data(data);
      return UEVENT_OK;
    }
    if (source.HasWakeup()) {
      return UEVENT_ERR_BUSY;
    }
    const uevent_err wakeup_err = source.SetWakeup(this);
    if (wakeup_err != UEVENT_OK) return wakeup_err;

    bool needs_notify = false;
    {
      base::LockGuard<base::Mutex> lk(mu_);
      source.set_user_data(data);
      LinkSource(&source);
      if (source.is_ready() && !source.in_ready_queue()) {
        EnqueueReadyLocked(&source);
        if (waiters_ > 0 && (!signaled_ || waiters_ > 1)) {
          signaled_ = true;
          needs_notify = true;
        }
      }
      const uint32_t effective_timeout_ms =
          (timeout_ms > 0) ? static_cast<uint32_t>(timeout_ms) : source.timer_period_ms();
      if (effective_timeout_ms > 0) {
        source.deadline_ = cv_.timespec_add_ms(cv_.get_now(), effective_timeout_ms);
        if (!source.has_deadline_) {
          source.has_deadline_ = true;
          ++deadline_count_;
        }
        deadlines_changed_ = true;
        if (waiters_ > 0 && (!signaled_ || waiters_ > 1)) {
          signaled_ = true;
          needs_notify = true;
        }
      }
    }
    if (needs_notify) {
      cv_.notify_one();
    }
    return UEVENT_OK;
  }

  uevent_err Remove(EventSource &source) {
    if (!source.HasWakeup(this)) {
      return source.HasWakeup() ? UEVENT_ERR_BUSY : UEVENT_ERR_INVALID;
    }
    {
      base::LockGuard<base::Mutex> lk(mu_);
      UnlinkSource(&source);
      RemoveReadyLocked(&source);
      ClearTimeoutStateLocked(&source);
    }
    source.ClearWakeup();
    {
      base::LockGuard<base::Mutex> lk(mu_);
      RemoveReadyLocked(&source);
    }
    source.OnRemoved();
    return UEVENT_OK;
  }

  int Wait(EventSource *ready[], int max_ready, const int timeout_ms) {
    if (!ready || max_ready <= 0) {
      return UEVENT_ERR_INVALID;
    }
    return WaitImpl(max_ready, timeout_ms, [ready](int idx, EventSource *s) {
      ready[idx] = s;
    });
  }

  // Copies both the source handle and user_data while mu_ is held, preventing
  // use-after-free if another thread removes and destroys the EventSource
  // immediately after Wait() releases mu_.
  int WaitSources(uevent_source_t *ready, int max_ready, const int timeout_ms) {
    if (!ready || max_ready <= 0) {
      return UEVENT_ERR_INVALID;
    }
    return WaitImpl(max_ready, timeout_ms, [ready](int idx, EventSource *s) {
      ready[idx]._handle = s;
      ready[idx].data = s->user_data();
    });
  }

  void Stop() {
    bool should_notify = false;
    {
      base::LockGuard<base::Mutex> lk(mu_);
      stop_ = true;
      if (waiters_ > 0) {
        signaled_ = true;
        should_notify = true;
      }
    }
    if (should_notify) {
      cv_.notify_all();
    }
  }

  void NotifyReady(EventSource *source) {
    bool needs_notify = false;
    {
      base::LockGuard<base::Mutex> lk(mu_);
      if (!source->in_ready_queue()) {
        EnqueueReadyLocked(source);
        if (waiters_ > 0 && (!signaled_ || waiters_ > 1)) {
          signaled_ = true;
          needs_notify = true;
        }
      }
    }
    if (needs_notify) {
      cv_.notify_one();
    }
  }

 private:
  template <typename EmitFn>
  int WaitImpl(int max_ready, const int timeout_ms, EmitFn &&emit) {
    base::UniqueLock<base::Mutex> lk(mu_);
    if (stop_) {
      stop_ = false;
      return UEVENT_ERR_CANCELED;
    }

    bool has_loop_deadline = false;
    struct timespec loop_deadline{};

    for (;;) {
      // Expire per-event deadlines (avoid clock_gettime when none exist).
      if (deadline_count_ > 0) {
        struct timespec now = cv_.get_now();
        for (EventSource *s = sources_head_; s != nullptr && deadline_count_ > 0; s = s->poll_next_) {
          if (s->has_deadline_ && cv_.timespec_ge(now, s->deadline_)) {
            s->has_deadline_ = false;
            --deadline_count_;
            if (!s->timeout_fired_) {
              s->timeout_fired_ = true;
              ++timeout_fired_count_;
            }
            // Automatically re-arm periodic timer sources (catch up if stalled to avoid storms).
            const uint32_t period_ms = s->timer_period_ms();
            if (s->timer_periodic() && period_ms > 0) {
              struct timespec next_dl = cv_.timespec_add_ms(s->deadline_, period_ms);
              if (cv_.timespec_ge(now, next_dl)) {
                next_dl = cv_.timespec_add_ms(now, period_ms);
              }
              s->deadline_ = next_dl;
              s->has_deadline_ = true;
              ++deadline_count_;
            }
          }
        }
      }

      // Collect ready sources.
      int count = 0;
      if (timeout_fired_count_ > 0) {
        for (EventSource *s = sources_head_;
             s != nullptr && count < max_ready && timeout_fired_count_ > 0;
             s = s->poll_next_) {
          if (s->timeout_fired_) {
            s->timeout_fired_ = false;
            --timeout_fired_count_;
            emit(count++, s);
            RemoveReadyLocked(s);
          }
        }
      }

      while (ready_head_ != nullptr && count < max_ready) {
        auto *s = DequeueReadyLocked();
        // Dekker fence: pairs with notify_waiters() fence before in_ready_queue_ load.
        std::atomic_thread_fence(std::memory_order_seq_cst);
        if (s->is_ready()) {
          emit(count++, s);
        }
      }

      // Level-triggered fallback scan when nothing collected and ready queue is empty.
      // Collect directly via emit() to avoid double is_ready() calls and queue churn.
      if (count == 0 && ready_head_ == nullptr) {
        for (EventSource *s = sources_head_; s != nullptr; s = s->poll_next_) {
          if (s->is_ready()) {
            if (count < max_ready) {
              emit(count++, s);
            } else {
              EnqueueReadyLocked(s);
            }
          }
        }
      }

      if (count > 0) {
        return count;
      }
      if (stop_) {
        stop_ = false;
        return -1;
      }
      if (timeout_ms == 0) {
        return 0;
      }

      // Lazily compute loop_deadline only when we actually need to wait.
      struct timespec now{};
      bool have_now = false;
      if (timeout_ms > 0) {
        now = cv_.get_now();
        have_now = true;
        if (!has_loop_deadline) {
          loop_deadline = cv_.timespec_add_ms(now, timeout_ms);
          has_loop_deadline = true;
        } else if (cv_.timespec_ge(now, loop_deadline)) {
          return 0;
        }
      }

      // Compute earliest wake time: min(loop deadline, earliest per-event deadline).
      bool has_wake_time = has_loop_deadline;
      struct timespec wake_time = has_loop_deadline ? loop_deadline : timespec{};
      if (deadline_count_ > 0) {
        for (EventSource *s = sources_head_; s != nullptr; s = s->poll_next_) {
          if (s->has_deadline_ &&
              (!has_wake_time || cv_.timespec_ge(wake_time, s->deadline_))) {
            wake_time = s->deadline_;
            has_wake_time = true;
          }
        }
      }

      auto wake_pred = [this] {
        return ready_count_ > 0 || timeout_fired_count_ > 0 || deadlines_changed_ || stop_;
      };

      deadlines_changed_ = false;
      if (!has_wake_time) {
        // Block forever until notified.
        ++waiters_;
        signaled_ = false;
        cv_.wait(mu_, wake_pred);
        --waiters_;
      } else {
        // Wait until the earliest deadline.
        if (!have_now) {
          now = cv_.get_now();
        }
        if (cv_.timespec_ge(now, wake_time)) continue;
        uint32_t remaining_ms = cv_.timespec_diff_ms(now, wake_time);
        if (remaining_ms == 0) remaining_ms = 1;
        ++waiters_;
        signaled_ = false;
        cv_.wait_for(mu_, remaining_ms, wake_pred);
        --waiters_;
      }

      if (stop_) {
        stop_ = false;
        return -1;
      }
    }
  }

  void LinkSource(EventSource *s) {
    if (s->in_poll_list_) return;
    s->poll_prev_ = sources_tail_;
    s->poll_next_ = nullptr;
    if (sources_tail_) {
      sources_tail_->poll_next_ = s;
    } else {
      sources_head_ = s;
    }
    sources_tail_ = s;
    s->in_poll_list_ = true;
    ++source_count_;
  }

  void UnlinkSource(EventSource *s) {
    if (!s->in_poll_list_) return;
    if (s->poll_prev_) {
      s->poll_prev_->poll_next_ = s->poll_next_;
    } else {
      sources_head_ = s->poll_next_;
    }
    if (s->poll_next_) {
      s->poll_next_->poll_prev_ = s->poll_prev_;
    } else {
      sources_tail_ = s->poll_prev_;
    }
    s->poll_prev_ = nullptr;
    s->poll_next_ = nullptr;
    s->in_poll_list_ = false;
    --source_count_;
  }

  void EnqueueReadyLocked(EventSource *s) {
    if (s->in_ready_queue()) return;
    s->ready_prev_ = ready_tail_;
    s->ready_next_ = nullptr;
    if (ready_tail_) {
      ready_tail_->ready_next_ = s;
    } else {
      ready_head_ = s;
    }
    ready_tail_ = s;
    s->set_in_ready_queue(true);
    ++ready_count_;
  }

  EventSource *DequeueReadyLocked() {
    EventSource *s = ready_head_;
    if (!s) return nullptr;
    ready_head_ = s->ready_next_;
    if (ready_head_) {
      ready_head_->ready_prev_ = nullptr;
    } else {
      ready_tail_ = nullptr;
    }
    s->ready_prev_ = nullptr;
    s->ready_next_ = nullptr;
    s->set_in_ready_queue(false);
    if (ready_count_ > 0) --ready_count_;
    return s;
  }

  void RemoveReadyLocked(EventSource *s) {
    if (!s->in_ready_queue()) return;
    if (s->ready_prev_) {
      s->ready_prev_->ready_next_ = s->ready_next_;
    } else {
      ready_head_ = s->ready_next_;
    }
    if (s->ready_next_) {
      s->ready_next_->ready_prev_ = s->ready_prev_;
    } else {
      ready_tail_ = s->ready_prev_;
    }
    s->ready_prev_ = nullptr;
    s->ready_next_ = nullptr;
    s->set_in_ready_queue(false);
    if (ready_count_ > 0) --ready_count_;
  }

  void ClearTimeoutStateLocked(EventSource *s) {
    if (s->has_deadline_) {
      s->has_deadline_ = false;
      if (deadline_count_ > 0) --deadline_count_;
    }
    if (s->timeout_fired_) {
      s->timeout_fired_ = false;
      if (timeout_fired_count_ > 0) --timeout_fired_count_;
    }
  }

  base::Mutex mu_;
  base::ConditionVariable cv_;
  EventSource *sources_head_{nullptr};
  EventSource *sources_tail_{nullptr};
  size_t source_count_{0};
  EventSource *ready_head_{nullptr};
  EventSource *ready_tail_{nullptr};
  size_t ready_count_{0};
  size_t deadline_count_{0};
  size_t timeout_fired_count_{0};
  size_t waiters_{0};
  bool signaled_{false};
  bool deadlines_changed_{false};
  std::atomic<bool> stop_{false};
};

}  // namespace uevent

