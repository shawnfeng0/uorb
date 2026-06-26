/**
 * @file uorb_uevent.cc
 * Bridge between uORB subscriptions and the generic event loop.
 */

#include <uorb_uevent/uorb_uevent.h>
#include <uevent/uevent.h>

#include <cstdlib>

namespace {

struct UorbEventBridge {
  orb_subscription_t *sub;
  uevent_source_t *source;
};

static bool bridge_is_ready(void *ctx) {
  auto *bridge = static_cast<UorbEventBridge *>(ctx);
  return orb_check_update(bridge->sub);
}

static void bridge_on_publish(void *ctx) {
  auto *bridge = static_cast<UorbEventBridge *>(ctx);
  uevent_source_notify(bridge->source);
}

static bool bridge_register(void *ctx) {
  auto *bridge = static_cast<UorbEventBridge *>(ctx);
  return orb_subscription_set_callback(bridge->sub, bridge_on_publish, bridge);
}

static bool bridge_unregister(void *ctx) {
  auto *bridge = static_cast<UorbEventBridge *>(ctx);
  return orb_subscription_clear_callback(bridge->sub);
}

static void bridge_ctx_destroy(void *ctx) {
  std::free(static_cast<UorbEventBridge *>(ctx));
}

}  // anonymous namespace

uevent_source_t *uorb_subscription_create_source(orb_subscription_t *sub) {
  if (!sub) {
    errno = EINVAL;
    return nullptr;
  }

  auto *bridge = static_cast<UorbEventBridge *>(std::malloc(sizeof(UorbEventBridge)));
  if (!bridge) {
    errno = ENOMEM;
    return nullptr;
  }

  bridge->sub = sub;
  bridge->source = uevent_source_create(
      bridge_is_ready,
      bridge_register,
      bridge_unregister,
      bridge_ctx_destroy,
      bridge);

  if (!bridge->source) {
    std::free(bridge);
    return nullptr;
  }

  return bridge->source;
}

void uorb_subscription_destroy_source(uevent_source_t *source) {
  if (!source) return;
  // The CEventSource destructor calls bridge_ctx_destroy which frees the bridge.
  uevent_source_destroy(source);
}
