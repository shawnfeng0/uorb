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
int uorb_subscriber_create_source(uevent_source_t *src, orb_subscriber_t *sub);
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

  /// Start the background worker thread. Idempotent (returns true if already started).
  /// Thread-safe: serializes with concurrent Stop() calls and reclaims a worker
  /// that previously exited via self-Stop() before spawning a new thread.
  bool Start() {
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

    stop_requested_.store(false, std::memory_order_release);
    if (pthread_create(&worker_tid_, nullptr, &AsyncDispatcher::WorkerEntry, this) != 0) {
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
  Token Subscribe(Callback &&cb, unsigned instance = 0) {
    if (!base_._handle) return kInvalidToken;

    orb_subscriber_t sub = ORB_SUBSCRIBER_INITIALIZER;
    if (orb_subscriber_create_multi(&sub, &meta, instance) != ORB_OK) {
      return kInvalidToken;
    }

    uevent_source_t source = UEVENT_SOURCE_INITIALIZER;
    if (uorb_subscriber_create_source(&source, &sub) != 0) {
      orb_subscriber_destroy(&sub);
      return kInvalidToken;
    }

    using MsgType = typename msg::TypeMap<meta>::type;
    using Callable = typename std::decay<Callback>::type;
    auto *entry =
        new (std::nothrow) TypedEntry<MsgType, Callable>(sub, source, std::forward<Callback>(cb));
    if (!entry) {
      uorb_subscriber_destroy_source(&source);
      orb_subscriber_destroy(&sub);
      return kInvalidToken;
    }

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
      uorb_subscriber_destroy_source(&entry->source);
      orb_subscriber_destroy(&entry->sub);
      delete entry;
      return kInvalidToken;
    }
    pthread_mutex_unlock(&mu_);
    return token;
  }

  /// Convenience wrapper around Subscribe() returning a move-only RAII ScopedSubscription.
  template <const orb_metadata &meta, typename Callback>
  ScopedSubscription SubscribeScoped(Callback &&cb, unsigned instance = 0) {
    const Token tok = Subscribe<meta>(std::forward<Callback>(cb), instance);
    return ScopedSubscription(this, tok);
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
  struct EntryBase {
    EntryBase(orb_subscriber_t s, uevent_source_t src) : sub(s), source(src) {}
    virtual ~EntryBase() = default;
    virtual void Dispatch() = 0;

    orb_subscriber_t sub;
    uevent_source_t source;
    Token token{kInvalidToken};
    EntryBase *next{nullptr};
    bool in_callback{false};
    bool pending_destroy{false};
    bool waiting_unsubscribe{false};
  };

  template <typename MsgType, typename Callable>
  struct TypedEntry final : EntryBase {
    template <typename G>
    TypedEntry(orb_subscriber_t s, uevent_source_t src, G &&g)
        : EntryBase(s, src), cb(std::forward<G>(g)) {}

    void Dispatch() override {
      if (orb_subscriber_check_update(&sub)) {
        MsgType msg;
        if (orb_subscriber_copy(&sub, &msg) == ORB_OK) {
          cb(msg);
        }
      }
    }

    Callable cb;
  };

  static void *WorkerEntry(void *arg) {
    static_cast<AsyncDispatcher *>(arg)->WorkerLoop();
    return nullptr;
  }

  void WorkerLoop() {
    // Synchronize with Start() so worker_tid_ and state_ are visible.
    pthread_mutex_lock(&mu_);
    pthread_mutex_unlock(&mu_);

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

        entry->Dispatch();

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
    uorb_subscriber_destroy_source(&entry->source);
    orb_subscriber_destroy(&entry->sub);
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
  std::atomic<bool> stop_requested_{false};
  std::atomic<bool> running_{false};
};

}  // namespace uorb

#endif  // __cplusplus
