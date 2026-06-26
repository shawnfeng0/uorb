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
 * The is_ready callback checks orb_check_update().
 * The register/unregister callbacks manage the subscription's publish
 * notification via orb_subscription_set_callback/clear_callback.
 *
 * @param sub uORB subscription handle
 * @return event source handle, or NULL on error
 */
uevent_source_t *uorb_subscription_create_source(orb_subscription_t *sub);

/**
 * Destroy an event source created by uorb_subscription_create_source().
 *
 * The event source must be removed from the event base (via uevent_remove())
 * before calling this function.
 *
 * @param source event source handle (may be NULL)
 */
void uorb_subscription_destroy_source(uevent_source_t *source);

#ifdef __cplusplus
}

#include <atomic>
#include <cerrno>
#include <functional>
#include <memory>
#include <unordered_map>
#include <utility>

namespace uorb {

/**
 * EventLoop: C++ wrapper around the generic event loop C API.
 * Provides type-safe callback dispatch for uORB subscriptions.
 */
class EventLoop {
 public:
  EventLoop() : base_(uevent_create()) {}
  ~EventLoop() {
    if (!base_) return;
    for (auto &entry : entries_) {
      uevent_remove(base_, entry.second.source);
      if (entry.second.owned) {
        orb_subscription_t *sub = entry.second.sub;
        orb_destroy_subscription(&sub);
      }
      uorb_subscription_destroy_source(entry.second.source);
    }
    uevent_destroy(base_);
  }

  EventLoop(const EventLoop &) = delete;
  EventLoop &operator=(const EventLoop &) = delete;

  // Check if the event loop was successfully created
  explicit operator bool() const { return base_ != nullptr; }

  /**
   * Subscribe to a topic with a callback.
   * The EventLoop owns the subscription and will destroy it on cleanup.
   * @tparam meta The orb_metadata for the topic
   * @tparam Callback The callback type (function pointer, lambda, or functor)
   * @param cb Callback invoked when new data is available
   * @return true on success, false on error
   */
  template <const orb_metadata &meta, typename Callback>
  bool Subscribe(Callback &&cb) {
    if (!base_) return false;

    orb_subscription_t *sub = orb_create_subscription(&meta);
    if (!sub) return false;

    uevent_source_t *source = uorb_subscription_create_source(sub);
    if (!source) {
      orb_destroy_subscription(&sub);
      return false;
    }

    if (uevent_add(base_, source, 0) != 0) {
      uorb_subscription_destroy_source(source);
      orb_destroy_subscription(&sub);
      return false;
    }

    auto dispatch = [sub, cb = std::forward<Callback>(cb)]() {
      using MsgType = typename msg::TypeMap<meta>::type;
      MsgType msg;
      if (orb_copy(sub, &msg)) {
        cb(msg);
      }
    };

    entries_.emplace(source, Entry{sub, source, true, std::move(dispatch)});
    return true;
  }

  /**
   * Add an existing subscription with a callback.
   * Accepts C++ wrapper objects (Subscription, SubscriptionData, etc.) that have
   * handle() and ValueType typedef. The callback receives the message struct.
   * @tparam Sub The subscription wrapper type (must have handle() and ValueType)
   * @tparam Callback The callback type (receives const ValueType&)
   * @param sub Subscription wrapper
   * @param cb Callback invoked when new data is available
   * @return true on success, false on error
   */
  template <typename Sub, typename Callback>
  bool AddSubscription(Sub &sub, Callback &&cb) {
    if (!base_) return false;

    orb_subscription_t *handle = sub.handle();
    if (!handle) return false;

    uevent_source_t *source = uorb_subscription_create_source(handle);
    if (!source) return false;

    // Check if already added
    if (entries_.find(source) != entries_.end()) {
      errno = EBUSY;
      return false;
    }

    if (uevent_add(base_, source, 0) != 0) {
      uorb_subscription_destroy_source(source);
      return false;
    }

    using MsgType = typename Sub::ValueType;
    auto dispatch = [handle, cb = std::forward<Callback>(cb)]() {
      MsgType msg;
      if (orb_copy(handle, &msg)) {
        cb(msg);
      }
    };

    entries_.emplace(source, Entry{handle, source, false, std::move(dispatch)});
    return true;
  }

  /**
   * Remove a subscription from the event loop.
   * Accepts C++ wrapper objects (Subscription, SubscriptionData, etc.) that have handle().
   * @tparam Sub The subscription wrapper type (must have handle())
   * @param sub Subscription wrapper to remove
   * @return true on success, false on error
   */
  template <typename Sub>
  bool RemoveSubscription(Sub &sub) {
    if (!base_) return false;

    orb_subscription_t *handle = sub.handle();
    if (!handle) return false;

    for (auto it = entries_.begin(); it != entries_.end(); ++it) {
      if (it->second.sub == handle) {
        uevent_remove(base_, it->second.source);
        uorb_subscription_destroy_source(it->second.source);
        entries_.erase(it);
        return true;
      }
    }
    return false;
  }

  /**
   * Run the event loop once.
   * @param timeout_ms Timeout in milliseconds (-1 = block indefinitely)
   * @return Number of ready sources, or -1 on error
   */
  int RunOnce(int timeout_ms = -1) {
    if (!base_ || quit_requested_.load()) return -1;

    uevent_source_t *ready[32];
    int n = uevent_loop(base_, ready, 32, timeout_ms);
    if (n <= 0) return n;

    for (int i = 0; i < n; ++i) {
      auto it = entries_.find(ready[i]);
      if (it != entries_.end()) {
        it->second.dispatch();
      }
    }
    return n;
  }

  /**
   * Run the event loop until Quit() is called.
   * @return true if Quit() was called, false on error or no entries
   * 
   * Note: After Quit() is called, any pending notifications that have not been
   * processed will be discarded.
   */
  bool Run() {
    while (!quit_requested_.load()) {
      if (entries_.empty()) {
        return false;  // No entries to wait for
      }
      int n = RunOnce(-1);
      if (n < 0) {
        return quit_requested_.load();  // Return true if Quit() was called
      }
    }
    return true;
  }

  /**
   * Request the event loop to quit.
   * Thread-safe.
   */
  void Quit() {
    quit_requested_.store(true);
    if (base_) uevent_loopbreak(base_);
  }

 private:
  struct Entry {
    orb_subscription_t *sub;
    uevent_source_t *source;
    bool owned;
    std::function<void()> dispatch;
  };

  uevent_t *base_;
  std::unordered_map<uevent_source_t *, Entry> entries_;
  std::atomic<bool> quit_requested_{false};
};

}  // namespace uorb

#endif  // __cplusplus
