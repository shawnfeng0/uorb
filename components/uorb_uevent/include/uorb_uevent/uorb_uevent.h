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
#include <functional>
#include <unordered_map>
#include <utility>

namespace uorb {

/**
 * EventLoop: C++ wrapper around the generic event loop C API.
 * Provides type-safe callback dispatch for uORB subscriptions.
 *
 * Thread safety: Not thread-safe. All methods (Subscribe, RunOnce, Run,
 * Quit, destructor) must be called from the same thread. The callback
 * dispatch lambda is invoked from RunOnce/Run, so it also runs on the
 * caller's thread.
 */
class EventLoop {
 public:
  EventLoop() { uevent_create(&base_); }
  ~EventLoop() {
    if (!base_._handle) return;
    for (auto &kv : entries_) {
      uevent_remove(&base_, &kv.second.source);
      orb_subscriber_destroy(&kv.second.sub);
      uorb_subscriber_destroy_source(&kv.second.source);
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

    if (uevent_add(&base_, &source, 0) != 0) {
      uorb_subscriber_destroy_source(&source);
      orb_subscriber_destroy(&sub);
      return false;
    }

    auto dispatch = [sub, cb = std::forward<Callback>(cb)]() mutable {
      using MsgType = typename msg::TypeMap<meta>::type;
      MsgType msg;
      if (orb_subscriber_copy(&sub, &msg) == ORB_OK) {
        cb(msg);
      }
    };

    entries_.emplace(source._handle, Entry{sub, source, std::move(dispatch)});
    return true;
  }

  int RunOnce(int timeout_ms = -1) {
    if (!base_._handle || quit_requested_.load()) return -1;

    uevent_source_t ready[32];
    int n = uevent_loop(&base_, ready, 32, timeout_ms);
    if (n <= 0) return n;

    for (int i = 0; i < n; ++i) {
      auto it = entries_.find(ready[i]._handle);
      if (it != entries_.end()) {
        it->second.dispatch();
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
  struct Entry {
    orb_subscriber_t sub;
    uevent_source_t source;
    std::function<void()> dispatch;
  };

  uevent_t base_ = UEVENT_INITIALIZER;
  std::unordered_map<void*, Entry> entries_;
  std::atomic<bool> quit_requested_{false};
};

}  // namespace uorb

#endif  // __cplusplus
