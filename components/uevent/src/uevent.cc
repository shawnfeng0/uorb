/**
 * @file uevent_loop.cc
 * Implementation of the generic event loop C API.
 *
 * This file contains the EventPoll (internal C++ class) and CEventSource
 * (C wrapper), and implements the uevent_* C API functions declared in
 * uevent/uevent.h.
 */

#include <uevent/uevent.h>

#include "event_poll.h"

namespace {

// CEventSource: EventSource backed by C callbacks.
// Bridges the C API to the internal EventSource/EventPoll system.
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
      EventSource::RemoveWakeup();
      return false;
    }
    return true;
  }

  bool RemoveWakeup() override {
    if (!HasWakeup()) return true;
    if (unregister_fn_) unregister_fn_(ctx_);
    return EventSource::RemoveWakeup();
  }

 private:
  uevent_ready_fn ready_fn_;
  uevent_register_fn register_fn_;
  uevent_unregister_fn unregister_fn_;
  uevent_ctx_destroy_fn ctx_destroy_fn_;
  void *ctx_;
};

}  // anonymous namespace

// ---- C API implementation ----

uevent_t *uevent_create(void) {
  auto *poll = new (std::nothrow) uevent::EventPoll();
  if (!poll) {
    errno = ENOMEM;
    return nullptr;
  }
  return reinterpret_cast<uevent_t *>(poll);
}

void uevent_destroy(uevent_t *base) {
  if (!base) return;
  auto *poll = reinterpret_cast<uevent::EventPoll *>(base);
  delete poll;
}

int uevent_loop(uevent_t *base, uevent_source_t *ready[], int max_ready, int timeout_ms) {
  if (!base || !ready || max_ready <= 0) {
    errno = EINVAL;
    return -1;
  }
  auto *poll = reinterpret_cast<uevent::EventPoll *>(base);
  return poll->Wait(reinterpret_cast<uevent::EventSource **>(ready), max_ready, timeout_ms);
}

int uevent_loopbreak(uevent_t *base) {
  if (!base) {
    errno = EINVAL;
    return -1;
  }
  auto *poll = reinterpret_cast<uevent::EventPoll *>(base);
  poll->Stop();
  return 0;
}

uevent_source_t *uevent_source_create(uevent_ready_fn ready_fn,
                                       uevent_register_fn register_fn,
                                       uevent_unregister_fn unregister_fn,
                                       uevent_ctx_destroy_fn ctx_destroy_fn,
                                       void *ctx) {
  if (!ready_fn) {
    errno = EINVAL;
    return nullptr;
  }
  auto *source = new (std::nothrow) CEventSource(ready_fn, register_fn, unregister_fn, ctx_destroy_fn, ctx);
  if (!source) {
    errno = ENOMEM;
    return nullptr;
  }
  return reinterpret_cast<uevent_source_t *>(source);
}

void uevent_source_destroy(uevent_source_t *source) {
  if (!source) return;
  delete reinterpret_cast<CEventSource *>(source);
}

void uevent_source_notify(uevent_source_t *source) {
  if (!source) return;
  reinterpret_cast<uevent::EventSource *>(source)->notify_waiters();
}

int uevent_add(uevent_t *base, uevent_source_t *source, int timeout_ms) {
  if (!base || !source) {
    errno = EINVAL;
    return -1;
  }
  auto *poll = reinterpret_cast<uevent::EventPoll *>(base);
  auto *es = reinterpret_cast<uevent::EventSource *>(source);
  return poll->Add(*es, timeout_ms) ? 0 : -1;
}

int uevent_remove(uevent_t *base, uevent_source_t *source) {
  if (!base || !source) {
    errno = EINVAL;
    return -1;
  }
  auto *poll = reinterpret_cast<uevent::EventPoll *>(base);
  auto *es = reinterpret_cast<uevent::EventSource *>(source);
  return poll->Remove(*es) ? 0 : -1;
}

bool uevent_source_is_bound(uevent_source_t *source) {
  if (!source) return false;
  return reinterpret_cast<uevent::EventSource *>(source)->HasWakeup();
}
