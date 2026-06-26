/**
 * @file uevent.cc
 * Implementation of the generic event loop C API.
 */

#include <uevent/uevent.h>

#include <new>

#include "event_poll.h"

namespace {

// CEventSource: EventSource backed by C callbacks.
class CEventSource final : public uevent::EventSource {
 public:
  CEventSource(uevent_ready_fn ready_fn,
               uevent_register_fn register_fn,
               uevent_unregister_fn unregister_fn,
               uevent_ctx_destroy_fn ctx_destroy_fn,
               void *ctx)
      : ready_fn_(ready_fn),
        register_fn_(register_fn),
        unregister_fn_(unregister_fn),
        ctx_destroy_fn_(ctx_destroy_fn),
        ctx_(ctx) {}

  ~CEventSource() override {
    if (ctx_destroy_fn_) ctx_destroy_fn_(ctx_);
  }

  bool is_ready() const override { return ready_fn_(ctx_); }

  bool SetWakeup(uevent::EventPoll *poll) override {
    if (!EventSource::SetWakeup(poll)) return false;
    if (register_fn_ && !register_fn_(ctx_)) {
      ClearWakeup();
      return false;
    }
    registered_ = true;
    return true;
  }

  void OnRemoved() override {
    if (registered_.exchange(false) && unregister_fn_) {
      unregister_fn_(ctx_);
    }
  }

 private:
  uevent_ready_fn ready_fn_;
  uevent_register_fn register_fn_;
  uevent_unregister_fn unregister_fn_;
  uevent_ctx_destroy_fn ctx_destroy_fn_;
  void *ctx_;
  std::atomic<bool> registered_{false};
};

}  // anonymous namespace

// ---- C API implementation ----

int uevent_create(uevent_t *ev) {
  if (!ev) {
    errno = EINVAL;
    return -1;
  }
  auto *poll = new (std::nothrow) uevent::EventPoll();
  if (!poll) {
    errno = ENOMEM;
    return -1;
  }
  ev->_handle = poll;
  return 0;
}

void uevent_destroy(uevent_t *ev) {
  if (!ev || !ev->_handle) return;
  auto *poll = reinterpret_cast<uevent::EventPoll *>(ev->_handle);
  delete poll;
  ev->_handle = nullptr;
}

int uevent_loop(uevent_t *ev, uevent_source_t *ready, int max_ready, int timeout_ms) {
  static_assert(sizeof(uevent_source_t) == sizeof(void *),
                "uevent_source_t must have same layout as void* for reinterpret_cast safety");
  if (!ev || !ev->_handle || !ready || max_ready <= 0) {
    errno = EINVAL;
    return -1;
  }
  auto *poll = reinterpret_cast<uevent::EventPoll *>(ev->_handle);
  // uevent_source_t is { void* _handle }, same layout as void*,
  // so reinterpret to EventSource** is safe.
  return poll->Wait(reinterpret_cast<uevent::EventSource **>(ready), max_ready, timeout_ms);
}

int uevent_loopbreak(uevent_t *ev) {
  if (!ev || !ev->_handle) {
    errno = EINVAL;
    return -1;
  }
  auto *poll = reinterpret_cast<uevent::EventPoll *>(ev->_handle);
  poll->Stop();
  return 0;
}

int uevent_source_create(uevent_source_t *src,
                         uevent_ready_fn ready_fn,
                         uevent_register_fn register_fn,
                         uevent_unregister_fn unregister_fn,
                         uevent_ctx_destroy_fn ctx_destroy_fn,
                         void *ctx) {
  if (!src || !ready_fn) {
    errno = EINVAL;
    return -1;
  }
  auto *source = new (std::nothrow) CEventSource(ready_fn, register_fn, unregister_fn, ctx_destroy_fn, ctx);
  if (!source) {
    errno = ENOMEM;
    return -1;
  }
  src->_handle = source;
  return 0;
}

void uevent_source_destroy(uevent_source_t *src) {
  if (!src || !src->_handle) return;
  delete reinterpret_cast<CEventSource *>(src->_handle);
  src->_handle = nullptr;
}

void uevent_source_notify(uevent_source_t *src) {
  if (!src || !src->_handle) return;
  reinterpret_cast<uevent::EventSource *>(src->_handle)->notify_waiters();
}

int uevent_add(uevent_t *ev, uevent_source_t *src, int timeout_ms) {
  if (!ev || !ev->_handle || !src || !src->_handle) {
    errno = EINVAL;
    return -1;
  }
  auto *poll = reinterpret_cast<uevent::EventPoll *>(ev->_handle);
  auto *es = reinterpret_cast<uevent::EventSource *>(src->_handle);
  return poll->Add(*es, timeout_ms) ? 0 : -1;
}

int uevent_remove(uevent_t *ev, uevent_source_t *src) {
  if (!ev || !ev->_handle || !src || !src->_handle) {
    errno = EINVAL;
    return -1;
  }
  auto *poll = reinterpret_cast<uevent::EventPoll *>(ev->_handle);
  auto *es = reinterpret_cast<uevent::EventSource *>(src->_handle);
  return poll->Remove(*es) ? 0 : -1;
}

bool uevent_source_is_bound(uevent_source_t *src) {
  if (!src || !src->_handle) return false;
  return reinterpret_cast<uevent::EventSource *>(src->_handle)->HasWakeup();
}
