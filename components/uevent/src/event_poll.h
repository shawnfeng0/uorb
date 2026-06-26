//
// Copyright (c) 2025 shawnfeng. All rights reserved.
//
#pragma once

#include <uevent/uevent_source.h>

#include <errno.h>
#include <list>
#include <unordered_map>
#include <unordered_set>

#include "base/mutex.h"
#include "base/condition_variable.h"

namespace uevent {

class EventPoll {
 public:
  EventPoll() = default;

  ~EventPoll() {
    std::list<EventSource *> to_cleanup;
    {
      base::LockGuard<base::Mutex> lk(mu_);
      to_cleanup.swap(sources_);
      ready_set_.clear();
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
    if (source.is_ready()) ready_set_.insert(&source);
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
      ready_set_.erase(&source);
      deadlines_.erase(&source);
      timeout_fired_.erase(&source);
      source.ClearWakeup();
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
      // Expire per-event deadlines.
      struct timespec now = cv_.get_now();
      for (auto it = deadlines_.begin(); it != deadlines_.end();) {
        if (cv_.timespec_ge(now, it->second)) {
          timeout_fired_.insert(it->first);
          ready_set_.insert(it->first);
          it = deadlines_.erase(it);
        } else {
          ++it;
        }
      }

      // Collect ready sources.
      int count = 0;
      for (auto it = timeout_fired_.begin();
           it != timeout_fired_.end() && count < max_ready;) {
        ready[count++] = *it;
        ready_set_.erase(*it);
        it = timeout_fired_.erase(it);
      }
      for (auto it = ready_set_.begin();
           count < max_ready && it != ready_set_.end();) {
        auto *s = *it;
        if (s->is_ready()) {
          ready[count++] = s;
          ++it;
        } else {
          it = ready_set_.erase(it);
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
        cv_.wait(mu_, [this] { return !ready_set_.empty() || stop_; });
      } else {
        // Wait until the earliest deadline.
        now = cv_.get_now();
        if (cv_.timespec_ge(now, wake_time)) continue;
        uint32_t remaining_ms = cv_.timespec_diff_ms(now, wake_time);
        if (remaining_ms == 0) remaining_ms = 1;
        cv_.wait_for(mu_, remaining_ms,
                     [this] { return !ready_set_.empty() || stop_; });
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
    base::LockGuard<base::Mutex> lk(mu_);
    ready_set_.insert(source);
    cv_.notify_one();
  }

 private:
  base::Mutex mu_;
  base::ConditionVariable cv_;
  std::list<EventSource *> sources_;
  std::unordered_set<EventSource *> ready_set_;
  std::unordered_map<EventSource *, struct timespec> deadlines_;
  std::unordered_set<EventSource *> timeout_fired_;
  bool stop_ = false;
};

}  // namespace uevent
