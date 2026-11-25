/**
 * @file uorb_uevent.h
 * Bridge between uORB subscriptions and the generic event loop.
 *
 * This is the only header that knows about both uorb.h and uevent.h.
 * uORB and uevent libraries are completely independent.
 */

#pragma once

#include <uorb/uorb.h>
#include <uevent/uevent.h>

#ifdef __cplusplus
extern "C" {
#endif

#define UORB_UEVENT_API_DECLARED 1

/**
 * Create an event source that wraps a uORB subscription.
 *
 * The returned event source can be added to an event base via uevent_add().
 * When the subscription receives new data (published to its topic), the
 * event source will be notified and reported as ready in uevent_loop().
 *
 * @param src event source handle to initialize
 * @param sub uORB subscription handle
 * @return 0 on success, -1 on error
 */
int uorb_subscriber_create_source(uevent_source_t *src, orb_subscriber_t *sub);

/**
 * Destroy an event source created by uorb_subscriber_create_source().
 *
 * The event source must be removed from the event base (via uevent_remove())
 * before calling this function.
 *
 * @param src event source handle (may be NULL)
 */
void uorb_subscriber_destroy_source(uevent_source_t *src);

#ifdef __cplusplus
}

#include <atomic>
#include <new>
#include <type_traits>
#include <utility>

namespace uorb {

/**
 * EventLoop: C++ wrapper around the generic event loop C API.
 * Provides type-safe callback dispatch for uORB subscriptions without
 * std::function or hash map lookup overhead.
 *
 * Thread safety: Not thread-safe, except for Quit() which is thread-safe
 * and may be called from any thread. Subscribe, RunOnce, Run, and the
 * destructor must be called from the same thread. The callback dispatch
 * runs on the caller's thread inside RunOnce/Run.
 */
class EventLoop {
 public:
  using Token = uint64_t;
  static constexpr Token kInvalidToken = 0;

  EventLoop() { uevent_create(&base_); }
  ~EventLoop() {
    EntryBase *cur = entries_head_;
    entries_head_ = nullptr;
    while (cur != nullptr) {
      EntryBase *next = cur->next;
      DestroyEntry(cur);
      cur = next;
    }
    if (base_._handle) {
      uevent_destroy(&base_);
    }
  }

  EventLoop(const EventLoop &) = delete;
  EventLoop &operator=(const EventLoop &) = delete;

  explicit operator bool() const { return base_._handle != nullptr; }

  template <const orb_metadata &meta, typename Callback>
  Token Subscribe(Callback &&cb, unsigned instance = 0) {
    if (!base_._handle) return kInvalidToken;

    orb_subscriber_t sub = ORB_SUBSCRIBER_INITIALIZER;
    if (orb_subscriber_create_multi(&sub, &meta, instance) != ORB_OK) return kInvalidToken;

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

    return RegisterEntry(entry);
  }

  /// Member-function overload for Subscribe().
  template <const orb_metadata &meta, typename C, typename R>
  Token Subscribe(R (C::*method)(const typename msg::TypeMap<meta>::type &),
                  C *obj, unsigned instance = 0) {
    if (!obj) return kInvalidToken;
    return Subscribe<meta>(
        [obj, method](const typename msg::TypeMap<meta>::type &m) { (obj->*method)(m); },
        instance);
  }

  /// Register a monotonic timer callback dispatched inside Run()/RunOnce().
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

  /// Member-function overload for AddTimer().
  template <typename C, typename R>
  Token AddTimer(uint32_t period_ms, R (C::*method)(), C *obj, bool periodic = true) {
    if (!obj) return kInvalidToken;
    return AddTimer(period_ms, [obj, method]() { (obj->*method)(); }, periodic);
  }

  /// Unsubscribe a topic or cancel a timer by token. Safe to call from inside a callback.
  bool Unsubscribe(Token token) {
    if (token == kInvalidToken) return false;
    EntryBase *entry = UnlinkEntry(token);
    if (!entry) return false;
    if (entry->in_callback) {
      entry->pending_destroy = true;
      return true;
    }
    DestroyEntry(entry);
    return true;
  }

  int RunOnce(int timeout_ms = -1) {
    if (!base_._handle || quit_requested_.load()) return -1;

    uevent_source_t ready[32];
    int n = uevent_loop(&base_, ready, 32, timeout_ms);
    if (n <= 0) return n;

    for (int i = 0; i < n; ++i) {
      EntryBase *entry = FindEntry(ready[i].data.u64);
      if (!entry) continue;
      entry->in_callback = true;
      entry->Dispatch();
      entry->in_callback = false;
      if (entry->pending_destroy) {
        DestroyEntry(entry);
      }
    }
    return n;
  }

  /// Run the loop until Quit() is requested or an error occurs.
  /// Returns true if the loop exited because of Quit(), false on error
  /// or if it was invoked on an invalid / empty EventLoop.
  /// Note: Run() returns false immediately if there are no subscriptions.
  /// Callers should add subscriptions via Subscribe() before calling Run().
  /// Run() resets quit_requested_ at the start, so it can be called again
  /// after a previous Quit(). Calling Quit() before Run() has no effect —
  /// the flag is cleared when Run() starts.
  bool Run() {
    quit_requested_.store(false);
    while (!quit_requested_.load()) {
      if (entries_head_ == nullptr) {
        return false;
      }
      int n = RunOnce(-1);
      if (n < 0) {
        return quit_requested_.load();
      }
    }
    return true;
  }

  void Quit() {
    quit_requested_.store(true);
    if (base_._handle) uevent_loopbreak(&base_);
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

  template <typename Callable>
  struct TimerEntry final : EntryBase {
    template <typename G>
    TimerEntry(uevent_source_t src, G &&g)
        : EntryBase(ORB_SUBSCRIBER_INITIALIZER, src), cb(std::forward<G>(g)) {}

    void Dispatch() override { cb(); }

    Callable cb;
  };

  Token RegisterEntry(EntryBase *entry) {
    Token tok = ++next_token_;
    if (tok == kInvalidToken) {
      tok = ++next_token_;
    }
    entry->token = tok;
    entry->source.data.u64 = tok;
    if (uevent_add(&base_, &entry->source, 0) != 0) {
      uevent_source_destroy(&entry->source);
      if (entry->sub._handle) {
        orb_subscriber_destroy(&entry->sub);
      }
      delete entry;
      return kInvalidToken;
    }
    entry->next = entries_head_;
    entries_head_ = entry;
    return tok;
  }

  EntryBase *FindEntry(Token token) const {
    for (EntryBase *cur = entries_head_; cur != nullptr; cur = cur->next) {
      if (cur->token == token) return cur;
    }
    return nullptr;
  }

  EntryBase *UnlinkEntry(Token token) {
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

  uevent_t base_ = UEVENT_INITIALIZER;
  EntryBase *entries_head_{nullptr};
  Token next_token_{0};
  std::atomic<bool> quit_requested_{false};
};

}  // namespace uorb

#include <uorb_uevent/async_dispatcher.h>

#endif  // __cplusplus
