/**
 * @file uorb_uevent.cc
 * Bridge between uORB subscriptions and the generic event loop.
 */

#include <uorb_uevent/uorb_uevent.h>
#include <uevent/uevent.h>

#include <cstdlib>

namespace {

struct UorbEventBridge {
  orb_subscriber_t sub;
  uevent_source_t source;
};

static bool bridge_is_ready(void *ctx) {
  auto *bridge = static_cast<UorbEventBridge *>(ctx);
  return orb_subscriber_check_update(&bridge->sub);
}

static void bridge_on_publish(const void *, orb_callback_ctx ctx) {
  auto *bridge = static_cast<UorbEventBridge *>(ctx.ptr);
  uevent_source_notify(&bridge->source);
}

static bool bridge_register(void *ctx) {
  auto *bridge = static_cast<UorbEventBridge *>(ctx);
  return orb_subscriber_set_callback(&bridge->sub, bridge_on_publish,
                                     orb_callback_ctx{bridge}) == ORB_OK;
}

static bool bridge_unregister(void *ctx) {
  auto *bridge = static_cast<UorbEventBridge *>(ctx);
  return orb_subscriber_clear_callback(&bridge->sub) == ORB_OK;
}

static void bridge_ctx_destroy(void *ctx) {
  std::free(static_cast<UorbEventBridge *>(ctx));
}

}  // anonymous namespace

int uorb_subscriber_create_source(uevent_source_t *src, orb_subscriber_t *sub) {
  if (!src || !sub) {
    errno = EINVAL;
    return -1;
  }

  auto *bridge = static_cast<UorbEventBridge *>(std::malloc(sizeof(UorbEventBridge)));
  if (!bridge) {
    errno = ENOMEM;
    return -1;
  }

  bridge->sub = *sub;

  int ret = uevent_source_create(
      src,
      bridge_is_ready,
      bridge_register,
      bridge_unregister,
      bridge_ctx_destroy,
      bridge);

  if (ret != 0) {
    std::free(bridge);
    return -1;
  }

  bridge->source = *src;
  return 0;
}

void uorb_subscriber_destroy_source(uevent_source_t *src) {
  if (!src) return;
  uevent_source_destroy(src);
}
