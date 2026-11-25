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

  uevent_err SetWakeup(uevent::EventPoll *poll) override {
    const uevent_err err = EventSource::SetWakeup(poll);
    if (err != UEVENT_OK) return err;
    if (register_fn_ && !register_fn_(ctx_)) {
      ClearWakeup();
      return UEVENT_ERR_INVALID;
    }
    registered_ = true;
    return UEVENT_OK;
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

// ManualEventSource: built-in thread-safe software-triggered event source.
class ManualEventSource final : public uevent::EventSource {
 public:
  bool is_ready() const override {
    return ready_.load(std::memory_order_acquire);
  }

  bool Trigger() override {
    ready_.store(true, std::memory_order_release);
    notify_waiters();
    return true;
  }

  bool Reset() override {
    ready_.store(false, std::memory_order_release);
    return true;
  }

 private:
  std::atomic<bool> ready_{false};
};

// TimerEventSource: built-in monotonic one-shot or periodic timer source.
class TimerEventSource final : public uevent::EventSource {
 public:
  TimerEventSource(uint32_t period_ms, bool periodic)
      : period_ms_(period_ms), periodic_(periodic) {}

  bool is_ready() const override { return false; }
  uint32_t timer_period_ms() const override { return period_ms_; }
  bool timer_periodic() const override { return periodic_; }

 private:
  const uint32_t period_ms_;
  const bool periodic_;
};

}  // anonymous namespace

// ---- C API implementation ----

const char *uevent_strerror(uevent_err err) {
  switch (err) {
    case UEVENT_OK:
      return "OK";
    case UEVENT_ERR_CANCELED:
      return "Canceled";
    case UEVENT_ERR_INVALID:
      return "Invalid argument";
    case UEVENT_ERR_NO_MEM:
      return "Out of memory";
    case UEVENT_ERR_BUSY:
      return "Resource busy";
    default:
      return "Unknown error";
  }
}

uevent_err uevent_create(uevent_t *ev) {
  static_assert(sizeof(uevent_t) == sizeof(void *),
                "uevent_t must have same layout as void* for handle wrapper safety");
  if (!ev) {
    return UEVENT_ERR_INVALID;
  }
  auto *poll = new (std::nothrow) uevent::EventPoll();
  if (!poll) {
    return UEVENT_ERR_NO_MEM;
  }
  ev->_handle = poll;
  return UEVENT_OK;
}

void uevent_destroy(uevent_t *ev) {
  if (!ev || !ev->_handle) return;
  auto *poll = reinterpret_cast<uevent::EventPoll *>(ev->_handle);
  delete poll;
  ev->_handle = nullptr;
}

int uevent_loop(uevent_t *ev, uevent_source_t *ready, int max_ready, int timeout_ms) {
  if (!ev || !ev->_handle || !ready || max_ready <= 0) {
    return UEVENT_ERR_INVALID;
  }
  auto *poll = reinterpret_cast<uevent::EventPoll *>(ev->_handle);
  return poll->WaitSources(ready, max_ready, timeout_ms);
}

uevent_err uevent_loopbreak(uevent_t *ev) {
  if (!ev || !ev->_handle) {
    return UEVENT_ERR_INVALID;
  }
  auto *poll = reinterpret_cast<uevent::EventPoll *>(ev->_handle);
  poll->Stop();
  return UEVENT_OK;
}

uevent_err uevent_source_create(uevent_source_t *src,
                                uevent_ready_fn ready_fn,
                                uevent_register_fn register_fn,
                                uevent_unregister_fn unregister_fn,
                                uevent_ctx_destroy_fn ctx_destroy_fn,
                                void *ctx) {
  if (!src || !ready_fn) {
    return UEVENT_ERR_INVALID;
  }
  auto *source = new (std::nothrow) CEventSource(ready_fn, register_fn, unregister_fn, ctx_destroy_fn, ctx);
  if (!source) {
    return UEVENT_ERR_NO_MEM;
  }
  src->_handle = source;
  src->data.u64 = 0;
  return UEVENT_OK;
}

void uevent_source_destroy(uevent_source_t *src) {
  if (!src || !src->_handle) return;
  auto *es = reinterpret_cast<uevent::EventSource *>(src->_handle);
  if (es->is_owned_by_uevent()) {
    delete es;
  }
  src->_handle = nullptr;
  src->data.u64 = 0;
}

void uevent_source_notify(uevent_source_t *src) {
  if (!src || !src->_handle) return;
  reinterpret_cast<uevent::EventSource *>(src->_handle)->notify_waiters();
}

uevent_err uevent_add(uevent_t *ev, uevent_source_t *src, int timeout_ms) {
  if (!ev || !ev->_handle || !src || !src->_handle) {
    return UEVENT_ERR_INVALID;
  }
  auto *poll = reinterpret_cast<uevent::EventPoll *>(ev->_handle);
  auto *es = reinterpret_cast<uevent::EventSource *>(src->_handle);
  return poll->Add(*es, timeout_ms, src->data);
}

uevent_err uevent_remove(uevent_t *ev, uevent_source_t *src) {
  if (!ev || !ev->_handle || !src || !src->_handle) {
    return UEVENT_ERR_INVALID;
  }
  auto *poll = reinterpret_cast<uevent::EventPoll *>(ev->_handle);
  auto *es = reinterpret_cast<uevent::EventSource *>(src->_handle);
  return poll->Remove(*es);
}

bool uevent_source_is_bound(uevent_source_t *src) {
  if (!src || !src->_handle) return false;
  return reinterpret_cast<uevent::EventSource *>(src->_handle)->HasWakeup();
}

uevent_err uevent_manual_source_create(uevent_source_t *src) {
  if (!src) {
    return UEVENT_ERR_INVALID;
  }
  auto *source = new (std::nothrow) ManualEventSource();
  if (!source) {
    return UEVENT_ERR_NO_MEM;
  }
  src->_handle = source;
  src->data.u64 = 0;
  return UEVENT_OK;
}

uevent_err uevent_manual_source_trigger(uevent_source_t *src) {
  if (!src || !src->_handle) {
    return UEVENT_ERR_INVALID;
  }
  if (!reinterpret_cast<uevent::EventSource *>(src->_handle)->Trigger()) {
    return UEVENT_ERR_INVALID;
  }
  return UEVENT_OK;
}

uevent_err uevent_manual_source_reset(uevent_source_t *src) {
  if (!src || !src->_handle) {
    return UEVENT_ERR_INVALID;
  }
  if (!reinterpret_cast<uevent::EventSource *>(src->_handle)->Reset()) {
    return UEVENT_ERR_INVALID;
  }
  return UEVENT_OK;
}

uevent_err uevent_timer_source_create(uevent_source_t *src, uint32_t period_ms, bool periodic) {
  if (!src || period_ms == 0) {
    return UEVENT_ERR_INVALID;
  }
  auto *source = new (std::nothrow) TimerEventSource(period_ms, periodic);
  if (!source) {
    return UEVENT_ERR_NO_MEM;
  }
  src->_handle = source;
  src->data.u64 = 0;
  return UEVENT_OK;
}
