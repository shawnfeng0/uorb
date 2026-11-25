/**
 * @file async_dispatcher.cc
 * Non-template implementation of uorb::AsyncDispatcher (and EventLoop token definition).
 */

#include <sched.h>
#include <uorb/async_dispatcher.h>
#include <uorb/event_loop.h>

#include <cstring>

namespace uorb {
constexpr EventLoop::Token EventLoop::kInvalidToken;
constexpr AsyncDispatcher::Token AsyncDispatcher::kInvalidToken;

AsyncDispatcher::AsyncDispatcher() {
  pthread_mutex_init(&mu_, nullptr);
  pthread_cond_init(&done_cv_, nullptr);
  uevent_create(&base_);
}

AsyncDispatcher::~AsyncDispatcher() {
  Stop();
  EntryBase *cur = nullptr;
  pthread_mutex_lock(&mu_);
  cur = entries_head_;
  entries_head_ = nullptr;
  pthread_mutex_unlock(&mu_);

  while (cur != nullptr) {
    EntryBase *next = cur->next;
    DestroyEntry(cur);
    cur = next;
  }

  if (base_._handle) {
    uevent_destroy(&base_);
  }

#ifdef __APPLE__
  // Guard against Darwin pthread_cond_destroy race (crbug.com/517681).
  {
    pthread_mutex_lock(&mu_);
    struct timespec ts{};
    ts.tv_sec = 0;
    ts.tv_nsec = 1;
    pthread_cond_timedwait_relative_np(&done_cv_, &mu_, &ts);
    pthread_mutex_unlock(&mu_);
  }
#endif
  pthread_cond_destroy(&done_cv_);
  pthread_mutex_destroy(&mu_);
}

bool AsyncDispatcher::Start(const ThreadConfig &config) {
  if (!base_._handle) return false;

  pthread_mutex_lock(&mu_);
  while (state_ == WorkerState::kStopping) {
    pthread_cond_wait(&done_cv_, &mu_);
  }
  if (state_ == WorkerState::kRunning) {
    pthread_mutex_unlock(&mu_);
    return true;
  }
  if (state_ == WorkerState::kExitedPendingJoin) {
    const pthread_t old_tid = worker_tid_;
    state_ = WorkerState::kStopping;
    pthread_mutex_unlock(&mu_);

    pthread_join(old_tid, nullptr);

    pthread_mutex_lock(&mu_);
    state_ = WorkerState::kStopped;
    pthread_cond_broadcast(&done_cv_);
  }

  worker_name_[0] = '\0';
  if (config.name && config.name[0] != '\0') {
    std::strncpy(worker_name_, config.name, sizeof(worker_name_) - 1);
    worker_name_[sizeof(worker_name_) - 1] = '\0';
  }

  pthread_attr_t attr;
  if (pthread_attr_init(&attr) != 0) {
    pthread_mutex_unlock(&mu_);
    return false;
  }

  bool attr_configured = true;
  if (config.stack_size > 0) {
    if (pthread_attr_setstacksize(&attr, config.stack_size) != 0) {
      attr_configured = false;
    }
  }
  if (attr_configured && config.sched_policy >= 0) {
    struct sched_param param{};
    param.sched_priority = config.sched_priority;
    if (pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED) != 0 ||
        pthread_attr_setschedpolicy(&attr, config.sched_policy) != 0 ||
        pthread_attr_setschedparam(&attr, &param) != 0) {
      attr_configured = false;
    }
  }

  if (!attr_configured) {
    pthread_attr_destroy(&attr);
    pthread_mutex_unlock(&mu_);
    return false;
  }

  stop_requested_.store(false, std::memory_order_release);
  const int create_rc = pthread_create(&worker_tid_, &attr, &AsyncDispatcher::WorkerEntry, this);
  pthread_attr_destroy(&attr);
  if (create_rc != 0) {
    pthread_mutex_unlock(&mu_);
    return false;
  }
  state_ = WorkerState::kRunning;
  running_.store(true, std::memory_order_release);
  pthread_mutex_unlock(&mu_);
  return true;
}

void AsyncDispatcher::Stop() {
  pthread_t tid{};
  bool should_join = false;

  pthread_mutex_lock(&mu_);
  for (;;) {
    if (state_ == WorkerState::kStopped) {
      pthread_mutex_unlock(&mu_);
      return;
    }
    if (state_ == WorkerState::kStopping) {
      pthread_cond_wait(&done_cv_, &mu_);
      continue;
    }
    if (state_ == WorkerState::kRunning && pthread_equal(pthread_self(), worker_tid_)) {
      stop_requested_.store(true, std::memory_order_release);
      pthread_mutex_unlock(&mu_);
      if (base_._handle) uevent_loopbreak(&base_);
      return;
    }
    tid = worker_tid_;
    stop_requested_.store(true, std::memory_order_release);
    state_ = WorkerState::kStopping;
    should_join = true;
    break;
  }
  pthread_mutex_unlock(&mu_);

  if (should_join) {
    if (base_._handle) uevent_loopbreak(&base_);
    pthread_join(tid, nullptr);

    pthread_mutex_lock(&mu_);
    state_ = WorkerState::kStopped;
    running_.store(false, std::memory_order_release);
    pthread_cond_broadcast(&done_cv_);
    pthread_mutex_unlock(&mu_);
  }
}

bool AsyncDispatcher::Unsubscribe(Token token) {
  if (token == kInvalidToken) return false;

  EntryBase *entry = nullptr;
  bool defer_destroy = false;

  pthread_mutex_lock(&mu_);
  entry = UnlinkEntryLocked(token);
  if (!entry) {
    pthread_mutex_unlock(&mu_);
    return false;
  }
  if (entry->in_callback) {
    if (state_ != WorkerState::kStopped && pthread_equal(pthread_self(), worker_tid_)) {
      entry->pending_destroy = true;
      entry->stop_drain.store(true, std::memory_order_release);
      defer_destroy = true;
    } else {
      entry->waiting_unsubscribe = true;
      while (entry->in_callback) {
        pthread_cond_wait(&done_cv_, &mu_);
      }
      entry->waiting_unsubscribe = false;
    }
  }
  pthread_mutex_unlock(&mu_);

  if (!defer_destroy) {
    DestroyEntry(entry);
  }
  return true;
}

AsyncDispatcher::Token AsyncDispatcher::RegisterEntry(EntryBase *entry) {
  pthread_mutex_lock(&mu_);
  Token token = ++next_token_;
  if (token == kInvalidToken) {
    token = ++next_token_;
  }
  entry->token = token;
  entry->source.data.u64 = token;
  LinkEntryLocked(entry);
  const int add_rc = uevent_add(&base_, &entry->source, 0);
  if (add_rc != 0) {
    UnlinkEntryLocked(token);
    pthread_mutex_unlock(&mu_);
    if (entry->sub._handle) {
      orb_sub_destroy(&entry->sub);
    } else {
      uevent_source_destroy(&entry->source);
    }
    delete entry;
    return kInvalidToken;
  }
  pthread_mutex_unlock(&mu_);
  return token;
}

void *AsyncDispatcher::WorkerEntry(void *arg) {
  static_cast<AsyncDispatcher *>(arg)->WorkerLoop();
  return nullptr;
}

void AsyncDispatcher::WorkerLoop() {
  // Synchronize with Start() so worker_tid_, state_, and worker_name_ are visible.
  pthread_mutex_lock(&mu_);
  char name_copy[sizeof(worker_name_)];
  std::memcpy(name_copy, worker_name_, sizeof(worker_name_));
  pthread_mutex_unlock(&mu_);

  if (name_copy[0] != '\0') {
#if defined(__APPLE__)
    pthread_setname_np(name_copy);
#elif defined(__linux__)
    pthread_setname_np(pthread_self(), name_copy);
#endif
  }

  uevent_source_t ready[32];
  for (;;) {
    int n = uevent_loop(&base_, ready, 32, -1);
    if (n < 0) {
      if (stop_requested_.load(std::memory_order_acquire)) {
        break;
      }
      continue;
    }
    if (stop_requested_.load(std::memory_order_acquire)) {
      break;
    }

    for (int i = 0; i < n; ++i) {
      if (stop_requested_.load(std::memory_order_acquire)) {
        break;
      }
      const Token token = ready[i].data.u64;
      EntryBase *entry = nullptr;

      pthread_mutex_lock(&mu_);
      entry = FindEntryLocked(token);
      if (entry) {
        entry->in_callback = true;
      }
      pthread_mutex_unlock(&mu_);

      if (!entry) {
        continue;
      }

      entry->Dispatch(stop_requested_);

      bool destroy_now = false;
      pthread_mutex_lock(&mu_);
      entry->in_callback = false;
      if (entry->pending_destroy) {
        destroy_now = true;
      } else if (entry->waiting_unsubscribe) {
        pthread_cond_broadcast(&done_cv_);
      }
      pthread_mutex_unlock(&mu_);

      if (destroy_now) {
        DestroyEntry(entry);
      }
    }
  }

  pthread_mutex_lock(&mu_);
  if (state_ == WorkerState::kRunning) {
    state_ = WorkerState::kExitedPendingJoin;
  }
  running_.store(false, std::memory_order_release);
  pthread_mutex_unlock(&mu_);
}

void AsyncDispatcher::LinkEntryLocked(EntryBase *entry) {
  entry->next = entries_head_;
  entries_head_ = entry;
}

AsyncDispatcher::EntryBase *AsyncDispatcher::FindEntryLocked(Token token) const {
  for (EntryBase *cur = entries_head_; cur != nullptr; cur = cur->next) {
    if (cur->token == token) {
      return cur;
    }
  }
  return nullptr;
}

AsyncDispatcher::EntryBase *AsyncDispatcher::UnlinkEntryLocked(Token token) {
  EntryBase **pp = &entries_head_;
  while (*pp != nullptr) {
    if ((*pp)->token == token) {
      EntryBase *found = *pp;
      *pp = found->next;
      found->next = nullptr;
      return found;
    }
    pp = &((*pp)->next);
  }
  return nullptr;
}

void AsyncDispatcher::DestroyEntry(EntryBase *entry) {
  if (base_._handle) {
    uevent_remove(&base_, &entry->source);
  }
  if (entry->sub._handle) {
    orb_sub_destroy(&entry->sub);
  } else {
    uevent_source_destroy(&entry->source);
  }
  delete entry;
}

}  // namespace uorb
