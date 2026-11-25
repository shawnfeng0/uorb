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

  bool SetWakeup(uevent::EventPoll *poll) override {
    if (!EventSource::SetWakeup(poll)) return false;
    orb_callback_ctx ctx{};
    ctx.ptr = this;
    if (orb_subscriber_set_callback(&sub_, &OnPublish, ctx) != ORB_OK) {
      ClearWakeup();
      return false;
    }
    registered_.store(true, std::memory_order_release);
    return true;
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

int uorb_subscriber_create_source(uevent_source_t *src, orb_subscriber_t *sub) {
  if (!src || !sub) {
    errno = EINVAL;
    return -1;
  }

  auto *source = new (std::nothrow) UorbEventSource(*sub);
  if (!source) {
    errno = ENOMEM;
    return -1;
  }

  src->_handle = source;
  src->data.u64 = 0;
  return 0;
}

void uorb_subscriber_destroy_source(uevent_source_t *src) {
  if (!src) return;
  uevent_source_destroy(src);
}
