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
#include <uevent/uevent.h>
#include <uorb/uorb.h>

#include <atomic>
#include <cstdint>
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

  AsyncDispatcher();
  ~AsyncDispatcher();

  AsyncDispatcher(const AsyncDispatcher &) = delete;
  AsyncDispatcher &operator=(const AsyncDispatcher &) = delete;

  explicit operator bool() const { return base_._handle != nullptr; }

  /// Start the background worker thread with default POSIX attributes.
  bool Start() { return Start(ThreadConfig{}); }

  /// Start the background worker thread with custom thread configuration.
  /// Idempotent (returns true if already started).
  /// Thread-safe: serializes with concurrent Stop() calls and reclaims a worker
  /// that previously exited via self-Stop() before spawning a new thread.
  bool Start(const ThreadConfig &config);

  /// Stop the background worker thread and join it. Safe to call multiple times,
  /// concurrently from multiple threads, or when not started. When called from
  /// inside a dispatched callback, requests stop asynchronously without joining.
  void Stop();

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
  bool Unsubscribe(Token token);

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

  Token RegisterEntry(EntryBase *entry);
  static void *WorkerEntry(void *arg);
  void WorkerLoop();
  void LinkEntryLocked(EntryBase *entry);
  EntryBase *FindEntryLocked(Token token) const;
  EntryBase *UnlinkEntryLocked(Token token);
  void DestroyEntry(EntryBase *entry);

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
