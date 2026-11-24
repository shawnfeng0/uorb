//
// Copyright (c) 2025 shawnfeng. All rights reserved.
//
#pragma once

#include <uevent/uevent_source.h>

#include <errno.h>
#include <algorithm>
#include <atomic>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "base/mutex.h"
#include "base/condition_variable.h"

namespace uevent {

class RingBuffer {
 public:
  void Reserve(size_t capacity) {
    if (capacity <= capacity_) return;
    size_t new_cap = 16;
    while (new_cap < capacity) new_cap <<= 1;

    std::vector<EventSource *> new_buf(new_cap, nullptr);
    size_t count = size();
    if (capacity_ > 0) {
      size_t mask = capacity_ - 1;
      for (size_t i = 0; i < count; ++i) {
        new_buf[i] = buf_[(head_ + i) & mask];
      }
    }
    buf_ = std::move(new_buf);
    head_ = 0;
    tail_ = count;
    capacity_ = new_cap;
  }

  bool empty() const { return head_ == tail_; }
  size_t size() const { return tail_ - head_; }

  void push(EventSource *s) {
    if (tail_ - head_ >= capacity_) {
      Reserve(capacity_ == 0 ? 16 : capacity_ * 2);
    }
    buf_[tail_ & (capacity_ - 1)] = s;
    ++tail_;
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
      std::fill(buf_.begin(), buf_.end(), nullptr);
    }
    head_ = 0;
    tail_ = 0;
  }

 private:
  std::vector<EventSource *> buf_;
  size_t head_{0};
  size_t tail_{0};
  size_t capacity_{0};
};

class EventPoll {
 public:
  EventPoll() = default;

  ~EventPoll() {
    std::vector<EventSource *> to_cleanup;
    {
      base::LockGuard<base::Mutex> lk(mu_);
      to_cleanup.reserve(source_count_);
      while (sources_head_) {
        EventSource *s = sources_head_;
        UnlinkSource(s);
        to_cleanup.push_back(s);
      }
      ready_queue_.clear();
      ready_count_ = 0;
      deadlines_.clear();
      timeout_fired_.clear();
    }
    for (auto *s : to_cleanup) {
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
    {
      base::LockGuard<base::Mutex> lk(mu_);
      source.set_user_data(data);
      LinkSource(&source);
      ready_queue_.Reserve(source_count_);
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
        deadlines_[&source] = cv_.timespec_add_ms(cv_.get_now(), timeout_ms);
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
      if (!deadlines_.empty()) deadlines_.erase(&source);
      if (!timeout_fired_.empty()) timeout_fired_.erase(&source);
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
      if (!deadlines_.empty()) {
        struct timespec now = cv_.get_now();
        for (auto it = deadlines_.begin(); it != deadlines_.end();) {
          if (cv_.timespec_ge(now, it->second)) {
            timeout_fired_.insert(it->first);
            it = deadlines_.erase(it);
          } else {
            ++it;
          }
        }
      }

      // Collect ready sources.
      int count = 0;
      if (!timeout_fired_.empty()) {
        for (auto it = timeout_fired_.begin();
             it != timeout_fired_.end() && count < max_ready;) {
          auto *s = *it;
          ready[count++] = s;
          if (s->in_ready_queue()) {
            s->set_in_ready_queue(false);
            ready_queue_.remove(s);
            if (ready_count_ > 0) --ready_count_;
          }
          it = timeout_fired_.erase(it);
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
              s->set_in_ready_queue(true);
              ready_queue_.push(s);
              ++ready_count_;
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
      if (!deadlines_.empty()) {
        for (const auto &kv : deadlines_) {
          if (!has_wake_time || cv_.timespec_ge(wake_time, kv.second)) {
            wake_time = kv.second;
            has_wake_time = true;
          }
        }
      }

      auto wake_pred = [this] {
        return ready_count_ > 0 || !timeout_fired_.empty() || deadlines_changed_ || stop_;
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
        source->set_in_ready_queue(true);
        ready_queue_.push(source);
        ++ready_count_;
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

  base::Mutex mu_;
  base::ConditionVariable cv_;
  EventSource *sources_head_{nullptr};
  EventSource *sources_tail_{nullptr};
  size_t source_count_{0};
  RingBuffer ready_queue_;
  size_t ready_count_{0};
  size_t waiters_{0};
  bool signaled_{false};
  bool deadlines_changed_{false};
  std::unordered_map<EventSource *, struct timespec> deadlines_;
  std::unordered_set<EventSource *> timeout_fired_;
  std::atomic<bool> stop_{false};
};

}  // namespace uevent

