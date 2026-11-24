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
#include <memory>
#include <new>
#include <type_traits>
#include <utility>
#include <vector>

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
  EventLoop() { uevent_create(&base_); }
  ~EventLoop() {
    if (!base_._handle) return;
    for (auto &entry : entries_) {
      uevent_remove(&base_, &entry->source);
      orb_subscriber_destroy(&entry->sub);
      uorb_subscriber_destroy_source(&entry->source);
    }
    uevent_destroy(&base_);
  }

  EventLoop(const EventLoop &) = delete;
  EventLoop &operator=(const EventLoop &) = delete;

  explicit operator bool() const { return base_._handle != nullptr; }

  template <const orb_metadata &meta, typename Callback>
  bool Subscribe(Callback &&cb) {
    if (!base_._handle) return false;

    orb_subscriber_t sub = ORB_SUBSCRIBER_INITIALIZER;
    if (orb_subscriber_create(&sub, &meta) != ORB_OK) return false;

    uevent_source_t source = UEVENT_SOURCE_INITIALIZER;
    if (uorb_subscriber_create_source(&source, &sub) != 0) {
      orb_subscriber_destroy(&sub);
      return false;
    }

    using MsgType = typename msg::TypeMap<meta>::type;
    using Callable = typename std::decay<Callback>::type;
    std::unique_ptr<EntryBase> entry(
        new (std::nothrow) TypedEntry<MsgType, Callable>(sub, source, std::forward<Callback>(cb)));
    if (!entry) {
      uorb_subscriber_destroy_source(&source);
      orb_subscriber_destroy(&sub);
      return false;
    }

    entry->source.data.ptr = entry.get();
    if (uevent_add(&base_, &entry->source, 0) != 0) {
      uorb_subscriber_destroy_source(&entry->source);
      orb_subscriber_destroy(&entry->sub);
      return false;
    }

    entries_.push_back(std::move(entry));
    return true;
  }

  int RunOnce(int timeout_ms = -1) {
    if (!base_._handle || quit_requested_.load()) return -1;

    uevent_source_t ready[32];
    int n = uevent_loop(&base_, ready, 32, timeout_ms);
    if (n <= 0) return n;

    for (int i = 0; i < n; ++i) {
      auto *entry = static_cast<EntryBase *>(ready[i].data.ptr);
      if (entry) {
        entry->Dispatch();
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
      if (entries_.empty()) {
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

  uevent_t base_ = UEVENT_INITIALIZER;
  std::vector<std::unique_ptr<EntryBase>> entries_;
  std::atomic<bool> quit_requested_{false};
};

}  // namespace uorb

#include <uorb_uevent/async_dispatcher.h>

#endif  // __cplusplus
