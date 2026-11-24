//
// Copyright (c) 2025 shawnfeng. All rights reserved.
//
#pragma once

#include <uevent/uevent_source.h>

#include <errno.h>
#include <atomic>
#include <cstddef>
#include <cstring>
#include <new>

#include "base/mutex.h"
#include "base/condition_variable.h"

namespace uevent {

class RingBuffer {
 public:
  RingBuffer() = default;
  ~RingBuffer() { delete[] buf_; }
  RingBuffer(const RingBuffer &) = delete;
  RingBuffer &operator=(const RingBuffer &) = delete;

  bool Reserve(size_t capacity) {
    if (capacity <= capacity_) return true;
    size_t new_cap = 16;
    while (new_cap < capacity) new_cap <<= 1;

    auto **new_buf = new (std::nothrow) EventSource *[new_cap]();
    if (!new_buf) return false;

    size_t count = size();
    if (capacity_ > 0) {
      size_t mask = capacity_ - 1;
      for (size_t i = 0; i < count; ++i) {
        new_buf[i] = buf_[(head_ + i) & mask];
      }
    }
    delete[] buf_;
    buf_ = new_buf;
    head_ = 0;
    tail_ = count;
    capacity_ = new_cap;
    return true;
  }

  bool empty() const { return head_ == tail_; }
  size_t size() const { return tail_ - head_; }

  bool push(EventSource *s) {
    if (tail_ - head_ >= capacity_) {
      if (!Reserve(capacity_ == 0 ? 16 : capacity_ * 2)) return false;
    }
    buf_[tail_ & (capacity_ - 1)] = s;
    ++tail_;
    return true;
  }

  EventSource *pop() {
    if (head_ == tail_) return nullptr;
    size_t idx = head_ & (capacity_ - 1);
    EventSource *s = buf_[idx];
    buf_[idx] = nullptr;
    ++head_;
    return s;
  }

  void remove(EventSource *s) {
    if (capacity_ == 0 || head_ == tail_) return;
    size_t mask = capacity_ - 1;
    for (size_t i = head_; i < tail_; ++i) {
      if (buf_[i & mask] == s) {
        buf_[i & mask] = nullptr;
      }
    }
  }

  void clear() {
    if (capacity_ > 0) {
      std::memset(buf_, 0, capacity_ * sizeof(EventSource *));
    }
    head_ = 0;
    tail_ = 0;
  }

 private:
  EventSource **buf_{nullptr};
  size_t head_{0};
  size_t tail_{0};
  size_t capacity_{0};
};

class EventPoll {
 public:
  EventPoll() = default;

  ~EventPoll() {
    for (;;) {
      EventSource *s = nullptr;
      {
        base::LockGuard<base::Mutex> lk(mu_);
        if (!sources_head_) {
          ready_queue_.clear();
          ready_count_ = 0;
          deadline_count_ = 0;
          timeout_fired_count_ = 0;
          break;
        }
        s = sources_head_;
        UnlinkSource(s);
        ClearTimeoutStateLocked(s);
      }
      s->RemoveWakeup();
      // Safe after RemoveWakeup(): no notify_waiters() for this poll is in flight.
      s->set_in_ready_queue(false);
    }
  }

  EventPoll(const EventPoll &) = delete;
  EventPoll &operator=(const EventPoll &) = delete;

  bool Add(EventSource &source, int timeout_ms = 0) {
    return Add(source, timeout_ms, source.user_data());
  }

  bool Add(EventSource &source, int timeout_ms, uevent_data_t data) {
    if (source.HasWakeup(this)) {
      base::LockGuard<base::Mutex> lk(mu_);
      source.set_user_data(data);
      return true;
    }
    if (source.HasWakeup()) {
      errno = EBUSY;
      return false;
    }
    if (!source.SetWakeup(this)) return false;

    bool needs_notify = false;
    bool reserved = false;
    {
      base::LockGuard<base::Mutex> lk(mu_);
      reserved = ready_queue_.Reserve(source_count_ + 1);
      if (reserved) {
        source.set_user_data(data);
        LinkSource(&source);
        if (source.is_ready() && !source.in_ready_queue()) {
          source.set_in_ready_queue(true);
          ready_queue_.push(&source);
          ++ready_count_;
          if (waiters_ > 0 && (!signaled_ || waiters_ > 1)) {
            signaled_ = true;
            needs_notify = true;
          }
        }
        if (timeout_ms > 0) {
          source.deadline_ = cv_.timespec_add_ms(cv_.get_now(), timeout_ms);
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
    }
    if (!reserved) {
      source.RemoveWakeup();
      source.set_in_ready_queue(false);
      errno = ENOMEM;
      return false;
    }
    if (needs_notify) {
      cv_.notify_one();
    }
    return true;
  }

  bool Remove(EventSource &source) {
    if (!source.HasWakeup(this)) {
      errno = source.HasWakeup() ? EBUSY : EINVAL;
      return false;
    }
    {
      base::LockGuard<base::Mutex> lk(mu_);
      UnlinkSource(&source);
      if (source.in_ready_queue()) {
        source.set_in_ready_queue(false);
        ready_queue_.remove(&source);
        if (ready_count_ > 0) --ready_count_;
      }
      ClearTimeoutStateLocked(&source);
    }
    source.ClearWakeup();
    {
      base::LockGuard<base::Mutex> lk(mu_);
      if (source.in_ready_queue()) {
        source.set_in_ready_queue(false);
        ready_queue_.remove(&source);
        if (ready_count_ > 0) --ready_count_;
      }
    }
    source.OnRemoved();
    return true;
  }

  int Wait(EventSource *ready[], int max_ready, const int timeout_ms) {
    if (!ready || max_ready <= 0) {
      errno = EINVAL;
      return -1;
    }

    base::UniqueLock<base::Mutex> lk(mu_);
    if (stop_) {
      stop_ = false;
      return -1;
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
            ready[count++] = s;
            if (s->in_ready_queue()) {
              s->set_in_ready_queue(false);
              ready_queue_.remove(s);
              if (ready_count_ > 0) --ready_count_;
            }
          }
        }
      }

      while (!ready_queue_.empty() && count < max_ready) {
        auto *s = ready_queue_.pop();
        if (!s) continue;
        if (!s->in_ready_queue()) continue;
        s->set_in_ready_queue(false);
        // Dekker fence: pairs with notify_waiters() fence before in_ready_queue_ load.
        std::atomic_thread_fence(std::memory_order_seq_cst);
        if (ready_count_ > 0) --ready_count_;
        if (s->is_ready()) {
          ready[count++] = s;
        }
      }

      // Level-triggered fallback scan when nothing collected and ready_queue_ is empty.
      // Collect directly into ready[] to avoid double is_ready() calls and queue churn.
      if (count == 0 && ready_queue_.empty()) {
        for (EventSource *s = sources_head_; s != nullptr; s = s->poll_next_) {
          if (s->is_ready()) {
            if (count < max_ready) {
              ready[count++] = s;
            } else if (!s->in_ready_queue()) {
              if (ready_queue_.push(s)) {
                s->set_in_ready_queue(true);
                ++ready_count_;
              }
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
        if (ready_queue_.push(source)) {
          source->set_in_ready_queue(true);
          ++ready_count_;
          if (waiters_ > 0 && (!signaled_ || waiters_ > 1)) {
            signaled_ = true;
            needs_notify = true;
          }
        }
      }
    }
    if (needs_notify) {
      cv_.notify_one();
    }
  }

 private:
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
  RingBuffer ready_queue_;
  size_t ready_count_{0};
  size_t deadline_count_{0};
  size_t timeout_fired_count_{0};
  size_t waiters_{0};
  bool signaled_{false};
  bool deadlines_changed_{false};
  std::atomic<bool> stop_{false};
};

}  // namespace uevent

