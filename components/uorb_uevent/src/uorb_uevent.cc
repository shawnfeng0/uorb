/**
 * @file uorb_uevent.cc
 * Bridge between uORB subscriptions and the generic event loop.
 */

#include <uorb_uevent/uorb_uevent.h>
#include <uevent/uevent.h>
#include <uevent/uevent_source.h>

#include <atomic>
#include <new>

namespace uorb {
constexpr EventLoop::Token EventLoop::kInvalidToken;
constexpr AsyncDispatcher::Token AsyncDispatcher::kInvalidToken;
}  // namespace uorb

namespace {

// Direct EventSource implementation for a uORB subscription:
// single heap allocation and direct callback/readiness dispatch (no C bridge indirection).
class UorbEventSource final : public uevent::EventSource {
 public:
  explicit UorbEventSource(orb_subscriber_t sub) : sub_(sub) {}

  bool is_ready() const override {
    return orb_subscriber_check_update(const_cast<orb_subscriber_t *>(&sub_));
  }

  uevent_err SetWakeup(uevent::EventPoll *poll) override {
    const uevent_err err = EventSource::SetWakeup(poll);
    if (err != UEVENT_OK) return err;
    orb_callback_ctx ctx{};
    ctx.ptr = this;
    const orb_err cb_err = orb_subscriber_set_callback(&sub_, &OnPublish, ctx);
    if (cb_err != ORB_OK) {
      ClearWakeup();
      return (cb_err == ORB_ERR_BUSY) ? UEVENT_ERR_BUSY : UEVENT_ERR_INVALID;
    }
    registered_.store(true, std::memory_order_release);
    return UEVENT_OK;
  }

  void OnRemoved() override {
    if (registered_.exchange(false, std::memory_order_acq_rel)) {
      orb_subscriber_clear_callback(&sub_);
    }
  }

 private:
  static void OnPublish(const void *, orb_callback_ctx ctx) {
    static_cast<UorbEventSource *>(ctx.ptr)->notify_waiters();
  }

  orb_subscriber_t sub_;
  std::atomic<bool> registered_{false};
};

}  // anonymous namespace

uevent_err uorb_subscriber_create_source(uevent_source_t *src, orb_subscriber_t *sub) {
  if (!src || !sub) {
    return UEVENT_ERR_INVALID;
  }

  auto *source = new (std::nothrow) UorbEventSource(*sub);
  if (!source) {
    return UEVENT_ERR_NO_MEM;
  }

  src->_handle = source;
  src->data.u64 = 0;
  return UEVENT_OK;
}

void uorb_subscriber_destroy_source(uevent_source_t *src) {
  if (!src) return;
  uevent_source_destroy(src);
}
