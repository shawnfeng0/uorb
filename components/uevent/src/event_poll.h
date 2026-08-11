//
// Copyright (c) 2025 shawnfeng. All rights reserved.
//
#pragma once

#include <uevent/uevent_source.h>

#include <errno.h>
#include <algorithm>
#include <atomic>
#include <list>
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
    std::list<EventSource *> to_cleanup;
    {
      base::LockGuard<base::Mutex> lk(mu_);
      to_cleanup.swap(sources_);
      ready_queue_.clear();
      ready_count_ = 0;
      deadlines_.clear();
      timeout_fired_.clear();
    }
    for (auto *s : to_cleanup) s->RemoveWakeup();
  }

  EventPoll(const EventPoll &) = delete;
  EventPoll &operator=(const EventPoll &) = delete;

  bool Add(EventSource &source, int timeout_ms = 0) {
    if (source.HasWakeup(this)) return true;
    if (source.HasWakeup()) {
      errno = EBUSY;
      return false;
    }
    if (!source.SetWakeup(this)) return false;

    base::LockGuard<base::Mutex> lk(mu_);
    sources_.push_back(&source);
    ready_queue_.Reserve(sources_.size());
    if (source.is_ready() && !source.in_ready_queue()) {
      source.set_in_ready_queue(true);
      ready_queue_.push(&source);
      ++ready_count_;
    }
    if (timeout_ms > 0) {
      deadlines_[&source] = cv_.timespec_add_ms(cv_.get_now(), timeout_ms);
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
      sources_.remove(&source);
      if (source.in_ready_queue()) {
        source.set_in_ready_queue(false);
        ready_queue_.remove(&source);
        if (ready_count_ > 0) --ready_count_;
      }
      deadlines_.erase(&source);
      timeout_fired_.erase(&source);
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

    bool has_loop_deadline = timeout_ms > 0;
    struct timespec loop_deadline;
    if (has_loop_deadline) {
      loop_deadline = cv_.timespec_add_ms(cv_.get_now(), timeout_ms);
    }

    for (;;) {
      // If ready_queue_ is empty, scan sources_ for any source that is ready.
      if (ready_queue_.empty()) {
        for (auto *s : sources_) {
          if (s->is_ready() && !s->in_ready_queue()) {
            s->set_in_ready_queue(true);
            ready_queue_.push(s);
            ++ready_count_;
          }
        }
      }

      // Expire per-event deadlines.
      struct timespec now = cv_.get_now();
      for (auto it = deadlines_.begin(); it != deadlines_.end();) {
        if (cv_.timespec_ge(now, it->second)) {
          timeout_fired_.insert(it->first);
          if (!it->first->in_ready_queue()) {
            it->first->set_in_ready_queue(true);
            ready_queue_.push(it->first);
            ++ready_count_;
          }
          it = deadlines_.erase(it);
        } else {
          ++it;
        }
      }

      // Collect ready sources.
      int count = 0;
      for (auto it = timeout_fired_.begin();
           it != timeout_fired_.end() && count < max_ready;) {
        auto *s = *it;
        ready[count++] = s;
        if (s->in_ready_queue()) {
          s->set_in_ready_queue(false);
          if (ready_count_ > 0) --ready_count_;
        }
        it = timeout_fired_.erase(it);
      }

      while (!ready_queue_.empty() && count < max_ready) {
        auto *s = ready_queue_.pop();
        if (!s) continue;
        if (!s->in_ready_queue()) continue;
        s->set_in_ready_queue(false);
        if (ready_count_ > 0) --ready_count_;
        if (s->is_ready()) {
          ready[count++] = s;
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

      // If loop deadline has passed, return 0 (timeout).
      if (has_loop_deadline) {
        now = cv_.get_now();
        if (cv_.timespec_ge(now, loop_deadline)) {
          return 0;
        }
      }

      // Compute earliest wake time: min(loop deadline, earliest per-event deadline).
      bool has_wake_time = has_loop_deadline;
      struct timespec wake_time;
      if (has_loop_deadline) {
        wake_time = loop_deadline;
      } else {
        wake_time = {};
      }
      for (const auto &kv : deadlines_) {
        if (!has_wake_time || cv_.timespec_ge(wake_time, kv.second)) {
          wake_time = kv.second;
          has_wake_time = true;
        }
      }

      if (!has_wake_time) {
        // Block forever until notified.
        cv_.wait(mu_, [this] { return ready_count_ > 0 || stop_; });
      } else {
        // Wait until the earliest deadline.
        now = cv_.get_now();
        if (cv_.timespec_ge(now, wake_time)) continue;
        uint32_t remaining_ms = cv_.timespec_diff_ms(now, wake_time);
        if (remaining_ms == 0) remaining_ms = 1;
        cv_.wait_for(mu_, remaining_ms,
                     [this] { return ready_count_ > 0 || stop_; });
      }

      if (stop_) {
        stop_ = false;
        return -1;
      }
    }
  }

  void Stop() {
    base::LockGuard<base::Mutex> lk(mu_);
    stop_ = true;
    cv_.notify_one();
  }

  void NotifyReady(EventSource *source) {
    bool needs_notify = false;
    {
      base::LockGuard<base::Mutex> lk(mu_);
      if (!source->in_ready_queue()) {
        source->set_in_ready_queue(true);
        ready_queue_.push(source);
        ++ready_count_;
        needs_notify = true;
      }
    }
    if (needs_notify) {
      cv_.notify_one();
    }
  }

 private:
  base::Mutex mu_;
  base::ConditionVariable cv_;
  std::list<EventSource *> sources_;
  RingBuffer ready_queue_;
  size_t ready_count_{0};
  std::unordered_map<EventSource *, struct timespec> deadlines_;
  std::unordered_set<EventSource *> timeout_fired_;
  std::atomic<bool> stop_{false};
};

}  // namespace uevent

