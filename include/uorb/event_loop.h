#pragma once

#include <uorb/uorb.h>

#include <atomic>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

namespace uorb {

// EventLoop: register callbacks on subscriptions and dispatch them when new
// data becomes available.
//
// Thread-safety:
//  - Quit() is thread-safe and may be called from any thread.
//  - All other member functions (RegisterCallback, UnregisterCallback,
//    Subscribe, PollOnce, Loop) must be invoked from a single thread
//    (typically the loop thread).
//
// Lifetime:
//  - Quit() is sticky: once called, PollOnce() returns -1 immediately and
//    Loop() exits. The EventLoop cannot be restarted after Quit().
class EventLoop {
 public:
  EventLoop() : event_poll_(orb_event_poll_create()) {}
  ~EventLoop() {
    if (!event_poll_) return;
    // orb_event_poll_destroy() also unbinds remaining notifiers, but we still
    // remove each subscription here first: for owned subscriptions we need to
    // detach them from the poll set before we call orb_destroy_subscription(),
    // otherwise the subscription would be freed while still linked.
    for (auto &kv : entries_) {
      orb_subscription_t *sub = kv.first;
      orb_event_poll_remove(event_poll_, sub);
      if (kv.second->owned) {
        orb_destroy_subscription(&sub);
      }
    }
    orb_event_poll_destroy(&event_poll_);
  }

  EventLoop(const EventLoop &) = delete;
  EventLoop &operator=(const EventLoop &) = delete;
  EventLoop(EventLoop &&) = delete;
  EventLoop &operator=(EventLoop &&) = delete;

  // Whether the loop was successfully constructed.
  bool valid() const { return event_poll_ != nullptr; }
  explicit operator bool() const { return valid(); }

  // Register a callback on an externally managed subscription.
  // Returns false if the loop is invalid, the subscription is null, or the
  // subscription was already registered.
  //
  // The caller retains ownership of `sub_cpp`. It MUST remain alive until
  // either UnregisterCallback() is called for it or the EventLoop is
  // destroyed; otherwise the stored handle dangles and dispatch is UB.
  template <typename Sub, typename F>
  bool RegisterCallback(Sub &sub_cpp, F &&cb) {
    using Msg = typename Sub::ValueType;
    return AddEntry<Msg>(sub_cpp.handle(), std::forward<F>(cb), /*owned=*/false);
  }

  // Unregister a callback previously registered with an external subscription.
  // Returns false if the subscription was not registered.
  template <typename Sub>
  bool UnregisterCallback(Sub &sub_cpp) {
    return RemoveEntry(sub_cpp.handle());
  }

  // Subscribe to a topic; the subscription is owned by EventLoop.
  // Returns false if the loop is invalid or a subscription/entry could not
  // be allocated. (Each call creates a fresh subscription handle, so this
  // path never rejects as a duplicate.)
  template <const orb_metadata &meta, typename F>
  bool Subscribe(F &&cb) {
    if (!event_poll_) return false;
    orb_subscription_t *sub = orb_create_subscription(&meta);
    if (!sub) return false;
    using Msg = typename msg::TypeMap<meta>::type;
    if (!AddEntry<Msg>(sub, std::forward<F>(cb), /*owned=*/true)) {
      orb_destroy_subscription(&sub);
      return false;
    }
    return true;
  }

  [[nodiscard]] int PollOnce(const int timeout_ms = -1) {
    if (!event_poll_) return -1;
    if (entries_.empty()) return 0;

    const int number_of_event = orb_event_poll_wait(
        event_poll_, ready_buf_.data(), static_cast<int>(ready_buf_.size()), timeout_ms);

    for (int i = 0; i < number_of_event; ++i) {
      auto it = entries_.find(ready_buf_[i]);
      if (it != entries_.end()) it->second->Dispatch();
    }

    return number_of_event;
  }

  // Run the loop until Quit() is requested or a poll error occurs.
  // Returns true if the loop exited because of Quit(), false on poll error
  // or if it was invoked on an invalid / empty EventLoop.
  bool Loop() {
    // PollOnce(-1) blocks until either a subscription has data (returns > 0)
    // or Quit() is requested (returns < 0). It returns 0 only when there are
    // no registered entries, in which case we exit as false (nothing to do).
    for (;;) {
      const int n = PollOnce(-1);
      if (n < 0) return quit_requested_.load();
      if (n == 0) return false;
    }
  }

  // Thread-safe: request the event loop to quit.
  //
  // This is sticky: after Quit() returns, subsequent PollOnce() calls return
  // -1 immediately and the EventLoop cannot be restarted.
  void Quit() {
    quit_requested_ = true;
    if (event_poll_) orb_event_poll_quit(event_poll_);
  }

 private:
  // Type-erased entry. Each subscription is paired with a derived TypedEntry
  // that statically captures both the message type and the user callback's
  // concrete type, so we don't need std::function (and therefore neither the
  // <functional> header nor std::function's heap-allocation / SBO behavior).
  struct EntryBase {
    EntryBase(orb_subscription_t *s, bool o) : sub(s), owned(o) {}
    virtual ~EntryBase() = default;
    virtual void Dispatch() = 0;
    orb_subscription_t *sub;
    bool owned;  // EventLoop created and owns the subscription
  };

  template <typename Msg, typename F>
  struct TypedEntry final : EntryBase {
    template <typename G>
    TypedEntry(orb_subscription_t *s, bool o, G &&g)
        : EntryBase(s, o), cb(std::forward<G>(g)) {}
    void Dispatch() override {
      Msg msg;
      if (orb_copy(sub, &msg)) cb(msg);
    }
    F cb;
  };

  template <typename Msg, typename F>
  bool AddEntry(orb_subscription_t *sub, F &&cb, bool owned) {
    if (!event_poll_ || !sub) return false;
    if (entries_.find(sub) != entries_.end()) {
      // Already registered; refuse to double-register.
      return false;
    }
    if (!orb_event_poll_add(event_poll_, sub)) return false;

    // From here on, `sub` is registered on the poll set. If anything below
    // throws we must detach it again to avoid leaving a dangling binding.
    try {
      using Callable = typename std::decay<F>::type;
      std::unique_ptr<EntryBase> entry(
          new TypedEntry<Msg, Callable>(sub, owned, std::forward<F>(cb)));
      entries_.emplace(sub, std::move(entry));
      // Keep ready_buf_ sized to match entries_ so PollOnce does not have to
      // resize on every call. Only grow: shrinking on remove would churn the
      // allocation under add/remove cycles.
      if (ready_buf_.size() < entries_.size()) {
        ready_buf_.resize(entries_.size());
      }
    } catch (...) {
      orb_event_poll_remove(event_poll_, sub);
      // If emplace partially succeeded, roll it back too.
      auto it = entries_.find(sub);
      if (it != entries_.end()) entries_.erase(it);
      throw;
    }
    return true;
  }

  bool RemoveEntry(orb_subscription_t *sub) {
    if (!event_poll_) return false;
    auto it = entries_.find(sub);
    if (it == entries_.end()) return false;
    orb_event_poll_remove(event_poll_, sub);
    // External entries are not owned; do not destroy the subscription here.
    entries_.erase(it);
    // Intentionally do not shrink ready_buf_; it is an upper-bound scratch
    // buffer and shrinking would reallocate on churn.
    return true;
  }

  orb_event_poll_t *event_poll_;
  std::unordered_map<orb_subscription_t *, std::unique_ptr<EntryBase>> entries_;
  std::vector<orb_subscription_t *> ready_buf_;
  std::atomic<bool> quit_requested_{false};
};

}  // namespace uorb
