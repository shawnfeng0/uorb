/**
 * @file async_dispatcher.h
 * Asynchronous background-thread topic dispatcher built on uorb + uevent.
 *
 * Decouples publish notification from user callback execution: publisher
 * threads only perform an O(1) wakeup signal while callbacks execute
 * sequentially on the dispatcher's worker thread with zero uORB locks held.
 *
 * This eliminates:
 *  - Reentrant / AB-BA deadlocks when callbacks publish other topics or lock
 *    application mutexes.
 *  - Stack overflow from deep synchronous callback chains (stack depth is O(1)).
 *  - Publisher stalls caused by slow subscribers.
 */

#pragma once

#ifdef __cplusplus

#include <pthread.h>
#include <sched.h>
#include <uevent/uevent.h>
#include <uorb/uorb.h>

#include <atomic>
#include <cstdint>
#include <cstring>
#include <new>
#include <type_traits>
#include <utility>

#ifndef UORB_UEVENT_API_DECLARED
extern "C" {
uevent_err uorb_subscriber_create_source(uevent_source_t *src, orb_subscriber_t sub);
void uorb_subscriber_destroy_source(uevent_source_t *src);
}
#endif

namespace uorb {

/**
 * AsyncDispatcher: thread-safe asynchronous callback dispatcher for uORB topics.
 *
 * Owns a single POSIX worker thread backed by uevent. Callbacks always execute
 * on the worker thread with no uORB or dispatcher locks held, so callbacks may
 * freely Publish() any topic (including the same topic), Copy() any topic,
 * acquire application mutexes, Subscribe() new topics, or Unsubscribe()
 * themselves.
 *
 * Compatible with C++14 and -fno-exceptions -fno-rtti (no STL containers,
 * std::thread, std::mutex, or std::function).
 */
class AsyncDispatcher {
 public:
  using Token = uint64_t;
  static constexpr Token kInvalidToken = 0;

  /// Optional POSIX worker thread configuration passed to Start(const ThreadConfig &).
  struct ThreadConfig {
    const char *name{nullptr};  ///< Optional thread name (truncated to 15 chars on Linux/macOS)
    size_t stack_size{0};       ///< Stack size in bytes (0 = OS default)
    int sched_policy{-1};       ///< <0 = inherit; otherwise e.g. SCHED_OTHER / SCHED_FIFO / SCHED_RR
    int sched_priority{0};      ///< Priority used when sched_policy >= 0
  };

  /// Per-subscription queue consumption policy when the worker wakes up.
  enum class ConsumePolicy : uint8_t {
    kSingle,      ///< Default: copy at most 1 sample per wakeup (fair across topics)
    kLatestOnly,  ///< Drain backlog and invoke callback once with the newest sample
    kDrainAll,    ///< Drain queued samples (bounded per turn) and invoke callback for each
  };

  /// Move-only RAII subscription guard that automatically unsubscribes on destruction.
  /// Must not outlive the AsyncDispatcher that created it.
  class ScopedSubscription {
   public:
    ScopedSubscription() noexcept = default;
    ScopedSubscription(AsyncDispatcher *dispatcher, Token token) noexcept
        : dispatcher_(token != kInvalidToken ? dispatcher : nullptr), token_(token) {}

    ScopedSubscription(const ScopedSubscription &) = delete;
    ScopedSubscription &operator=(const ScopedSubscription &) = delete;

    ScopedSubscription(ScopedSubscription &&other) noexcept
        : dispatcher_(other.dispatcher_), token_(other.token_) {
      other.dispatcher_ = nullptr;
      other.token_ = kInvalidToken;
    }

    ScopedSubscription &operator=(ScopedSubscription &&other) noexcept {
      if (this != &other) {
        Reset();
        dispatcher_ = other.dispatcher_;
        token_ = other.token_;
        other.dispatcher_ = nullptr;
        other.token_ = kInvalidToken;
      }
      return *this;
    }

    ~ScopedSubscription() { Reset(); }

    explicit operator bool() const noexcept {
      return dispatcher_ != nullptr && token_ != kInvalidToken;
    }

    Token token() const noexcept { return token_; }

    /// Unsubscribe immediately (no-op if empty). Blocks until any in-flight
    /// callback finishes when called from a thread other than the worker thread.
    void Reset() {
      if (dispatcher_ && token_ != kInvalidToken) {
        AsyncDispatcher *d = dispatcher_;
        const Token t = token_;
        dispatcher_ = nullptr;
        token_ = kInvalidToken;
        d->Unsubscribe(t);
      }
    }

    /// Detach ownership without unsubscribing and return the underlying Token.
    Token Release() noexcept {
      const Token t = token_;
      dispatcher_ = nullptr;
      token_ = kInvalidToken;
      return t;
    }

   private:
    AsyncDispatcher *dispatcher_{nullptr};
    Token token_{kInvalidToken};
  };

  AsyncDispatcher() {
    pthread_mutex_init(&mu_, nullptr);
    pthread_cond_init(&done_cv_, nullptr);
    uevent_create(&base_);
  }

  ~AsyncDispatcher() {
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

  AsyncDispatcher(const AsyncDispatcher &) = delete;
  AsyncDispatcher &operator=(const AsyncDispatcher &) = delete;

  explicit operator bool() const { return base_._handle != nullptr; }

  /// Start the background worker thread with default POSIX attributes.
  bool Start() { return Start(ThreadConfig{}); }

  /// Start the background worker thread with custom thread configuration.
  /// Idempotent (returns true if already started).
  /// Thread-safe: serializes with concurrent Stop() calls and reclaims a worker
  /// that previously exited via self-Stop() before spawning a new thread.
  bool Start(const ThreadConfig &config) {
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
    const bool attr_ok = (pthread_attr_init(&attr) == 0);
    if (!attr_ok) {
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

  /// Stop the background worker thread and join it. Safe to call multiple times,
  /// concurrently from multiple threads, or when not started. When called from
  /// inside a dispatched callback, requests stop asynchronously without joining.
  void Stop() {
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

  /// Returns true if the background worker thread is currently running.
  bool IsRunning() const { return running_.load(std::memory_order_acquire); }

  /// Subscribe to a uORB topic with a typed callback dispatched on the worker thread.
  /// Thread-safe: may be called before or after Start(), from any thread, or
  /// from inside another dispatched callback.
  /// Returns kInvalidToken on failure.
  template <const orb_metadata &meta, typename Callback>
  Token Subscribe(Callback &&cb, unsigned instance = 0,
                  ConsumePolicy policy = ConsumePolicy::kSingle) {
    if (!base_._handle) return kInvalidToken;

    orb_subscriber_t sub = ORB_SUBSCRIBER_INITIALIZER;
    if (orb_subscriber_create_multi(&sub, &meta, instance) != ORB_OK) {
      return kInvalidToken;
    }

    uevent_source_t source = UEVENT_SOURCE_INITIALIZER;
    if (uorb_subscriber_create_source(&source, sub) != 0) {
      orb_subscriber_destroy(&sub);
      return kInvalidToken;
    }

    using MsgType = typename msg::TypeMap<meta>::type;
    using Callable = typename std::decay<Callback>::type;
    auto *entry = new (std::nothrow)
        TypedEntry<MsgType, Callable>(sub, source, policy, std::forward<Callback>(cb));
    if (!entry) {
      uorb_subscriber_destroy_source(&source);
      orb_subscriber_destroy(&sub);
      return kInvalidToken;
    }

    return RegisterEntry(entry);
  }

  /// Convenience wrapper around Subscribe() returning a move-only RAII ScopedSubscription.
  template <const orb_metadata &meta, typename Callback>
  ScopedSubscription SubscribeScoped(Callback &&cb, unsigned instance = 0,
                                     ConsumePolicy policy = ConsumePolicy::kSingle) {
    const Token tok = Subscribe<meta>(std::forward<Callback>(cb), instance, policy);
    return ScopedSubscription(this, tok);
  }

  /// Member-function overload for Subscribe().
  template <const orb_metadata &meta, typename C, typename R>
  Token Subscribe(R (C::*method)(const typename msg::TypeMap<meta>::type &),
                  C *obj, unsigned instance = 0,
                  ConsumePolicy policy = ConsumePolicy::kSingle) {
    if (!obj) return kInvalidToken;
    return Subscribe<meta>(
        [obj, method](const typename msg::TypeMap<meta>::type &m) { (obj->*method)(m); },
        instance, policy);
  }

  /// Member-function overload for SubscribeScoped().
  template <const orb_metadata &meta, typename C, typename R>
  ScopedSubscription SubscribeScoped(R (C::*method)(const typename msg::TypeMap<meta>::type &),
                                     C *obj, unsigned instance = 0,
                                     ConsumePolicy policy = ConsumePolicy::kSingle) {
    return ScopedSubscription(this, Subscribe<meta>(method, obj, instance, policy));
  }

  /// Register a monotonic timer callback dispatched on the worker thread.
  /// Thread-safe: may be called before or after Start(), from any thread, or
  /// from inside another callback. Cancel via Unsubscribe(token).
  /// Returns kInvalidToken on failure (e.g. period_ms == 0).
  template <typename Callback>
  Token AddTimer(uint32_t period_ms, Callback &&cb, bool periodic = true) {
    if (!base_._handle || period_ms == 0) return kInvalidToken;

    uevent_source_t source = UEVENT_SOURCE_INITIALIZER;
    if (uevent_timer_source_create(&source, period_ms, periodic) != 0) {
      return kInvalidToken;
    }

    using Callable = typename std::decay<Callback>::type;
    auto *entry = new (std::nothrow) TimerEntry<Callable>(source, std::forward<Callback>(cb));
    if (!entry) {
      uevent_source_destroy(&source);
      return kInvalidToken;
    }

    return RegisterEntry(entry);
  }

  /// Convenience wrapper around AddTimer() returning a move-only RAII ScopedSubscription.
  template <typename Callback>
  ScopedSubscription AddTimerScoped(uint32_t period_ms, Callback &&cb, bool periodic = true) {
    const Token tok = AddTimer(period_ms, std::forward<Callback>(cb), periodic);
    return ScopedSubscription(this, tok);
  }

  /// Member-function overload for AddTimer().
  template <typename C, typename R>
  Token AddTimer(uint32_t period_ms, R (C::*method)(), C *obj, bool periodic = true) {
    if (!obj) return kInvalidToken;
    return AddTimer(period_ms, [obj, method]() { (obj->*method)(); }, periodic);
  }

  /// Member-function overload for AddTimerScoped().
  template <typename C, typename R>
  ScopedSubscription AddTimerScoped(uint32_t period_ms, R (C::*method)(), C *obj,
                                    bool periodic = true) {
    return ScopedSubscription(this, AddTimer(period_ms, method, obj, periodic));
  }

  /// Unsubscribe a previously registered callback by token.
  /// Thread-safe: may be called from any thread, including from inside the
  /// callback itself. When called from another thread while the callback is
  /// currently executing, blocks until the in-flight invocation completes so
  /// callers can safely destroy state captured by the callback afterwards.
  bool Unsubscribe(Token token) {
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

 private:
  static constexpr uint32_t kMaxDrainIterations = 64u;

  struct EntryBase {
    EntryBase(orb_subscriber_t s, uevent_source_t src, ConsumePolicy p)
        : sub(s), source(src), policy(p) {}
    virtual ~EntryBase() = default;
    virtual void Dispatch(const std::atomic<bool> &stop_requested) = 0;

    orb_subscriber_t sub;
    uevent_source_t source;
    Token token{kInvalidToken};
    EntryBase *next{nullptr};
    ConsumePolicy policy{ConsumePolicy::kSingle};
    bool in_callback{false};
    bool pending_destroy{false};
    bool waiting_unsubscribe{false};
    std::atomic<bool> stop_drain{false};
  };

  template <typename MsgType, typename Callable>
  struct TypedEntry final : EntryBase {
    template <typename G>
    TypedEntry(orb_subscriber_t s, uevent_source_t src, ConsumePolicy p, G &&g)
        : EntryBase(s, src, p), cb(std::forward<G>(g)) {}

    void Dispatch(const std::atomic<bool> &stop_requested) override {
      switch (policy) {
        case ConsumePolicy::kSingle: {
          if (orb_subscriber_check_update(sub)) {
            MsgType msg;
            if (orb_subscriber_copy(sub, &msg) == ORB_OK) {
              cb(msg);
            }
          }
          break;
        }
        case ConsumePolicy::kLatestOnly: {
          MsgType msg;
          bool has_msg = false;
          for (uint32_t i = 0; i < kMaxDrainIterations && orb_subscriber_check_update(sub); ++i) {
            if (orb_subscriber_copy(sub, &msg) == ORB_OK) {
              has_msg = true;
            } else {
              break;
            }
          }
          if (has_msg) {
            cb(msg);
          }
          break;
        }
        case ConsumePolicy::kDrainAll: {
          for (uint32_t i = 0; i < kMaxDrainIterations && orb_subscriber_check_update(sub); ++i) {
            MsgType msg;
            if (orb_subscriber_copy(sub, &msg) != ORB_OK) {
              break;
            }
            cb(msg);
            if (stop_requested.load(std::memory_order_acquire) ||
                stop_drain.load(std::memory_order_acquire)) {
              break;
            }
          }
          break;
        }
      }
    }

    Callable cb;
  };

  template <typename Callable>
  struct TimerEntry final : EntryBase {
    template <typename G>
    TimerEntry(uevent_source_t src, G &&g)
        : EntryBase(ORB_SUBSCRIBER_INITIALIZER, src, ConsumePolicy::kSingle),
          cb(std::forward<G>(g)) {}

    void Dispatch(const std::atomic<bool> &) override { cb(); }

    Callable cb;
  };

  Token RegisterEntry(EntryBase *entry) {
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
      uevent_source_destroy(&entry->source);
      if (entry->sub._handle) {
        orb_subscriber_destroy(&entry->sub);
      }
      delete entry;
      return kInvalidToken;
    }
    pthread_mutex_unlock(&mu_);
    return token;
  }

  static void *WorkerEntry(void *arg) {
    static_cast<AsyncDispatcher *>(arg)->WorkerLoop();
    return nullptr;
  }

  void WorkerLoop() {
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

  void LinkEntryLocked(EntryBase *entry) {
    entry->next = entries_head_;
    entries_head_ = entry;
  }

  EntryBase *FindEntryLocked(Token token) const {
    for (EntryBase *cur = entries_head_; cur != nullptr; cur = cur->next) {
      if (cur->token == token) {
        return cur;
      }
    }
    return nullptr;
  }

  EntryBase *UnlinkEntryLocked(Token token) {
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

  void DestroyEntry(EntryBase *entry) {
    if (base_._handle) {
      uevent_remove(&base_, &entry->source);
    }
    uevent_source_destroy(&entry->source);
    if (entry->sub._handle) {
      orb_subscriber_destroy(&entry->sub);
    }
    delete entry;
  }

  enum class WorkerState : uint8_t {
    kStopped,
    kRunning,
    kExitedPendingJoin,
    kStopping,
  };

  uevent_t base_ = UEVENT_INITIALIZER;
  pthread_mutex_t mu_{};
  pthread_cond_t done_cv_{};
  pthread_t worker_tid_{};
  EntryBase *entries_head_{nullptr};
  Token next_token_{0};
  WorkerState state_{WorkerState::kStopped};
  char worker_name_[16]{};
  std::atomic<bool> stop_requested_{false};
  std::atomic<bool> running_{false};
};

}  // namespace uorb

#endif  // __cplusplus
