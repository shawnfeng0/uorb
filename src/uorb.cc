/**
 * @file uorb.cpp
 * A lightweight object broker.
 */

#include <uorb/uorb.h>

#include <cerrno>
#include <new>

#include "device_master.h"
#include "device_node.h"

using namespace uorb;

// ReceiverLocal: internal struct representing a uORB subscription.
// Stores DeviceNode reference, generation tracking, and an optional
// publish callback (set by the bridge library via orb_subscription_set_callback).
struct ReceiverLocal {
  DeviceNode &dev;
  unsigned last_generation;
  orb_publish_callback_fn publish_cb;
  void *publish_cb_ctx;
  detail::CallbackEntry callback_entry;

  explicit ReceiverLocal(DeviceNode &device_node) : dev(device_node) {
    last_generation = device_node.initial_generation();
    device_node.add_subscriber();
    publish_cb = nullptr;
    publish_cb_ctx = nullptr;
    callback_entry.on_publish = [](void *ctx) {
      auto *self = static_cast<ReceiverLocal *>(ctx);
      if (self->publish_cb) self->publish_cb(self->publish_cb_ctx);
    };
    callback_entry.ctx = this;
  }

  ~ReceiverLocal() {
    dev.remove_subscriber();
  }

  bool Copy(void *buffer) { return dev.Copy(buffer, &last_generation); }
  unsigned updates_available() const { return dev.updates_available(last_generation); }
};

#ifndef UORB_GIT_TAG
#define UORB_GIT_TAG "v0.0.0-0-unknown"
#endif

#define ORB_CHECK_TRUE(condition, error_code, error_action) \
  ({                                                        \
    if (!static_cast<bool>(condition)) {                    \
      errno = error_code;                                   \
      error_action;                                         \
    }                                                       \
  })

orb_publication_t *orb_create_publication(const struct orb_metadata *meta) {
  return orb_create_publication_multi(meta, nullptr);
}

orb_publication_t *orb_create_publication_multi(const struct orb_metadata *meta, unsigned int *instance) {
  ORB_CHECK_TRUE(meta, EINVAL, return nullptr);
  auto &meta_ = *meta;
  auto &device_master = DeviceMaster::get_instance();
  auto *dev_ = device_master.CreateAdvertiser(meta_, instance);
  if (!dev_) {
    return nullptr;
  }

  return reinterpret_cast<orb_publication_t *>(dev_);
}

bool orb_destroy_publication(orb_publication_t **handle_ptr) {
  ORB_CHECK_TRUE(handle_ptr && *handle_ptr, EINVAL, return false);

  auto &publication_handle = *handle_ptr;

  auto *dev = reinterpret_cast<uorb::DeviceNode *>(publication_handle);
  dev->remove_publisher();

  publication_handle = nullptr;

  return true;
}

bool orb_publish(orb_publication_t *handle, const void *data) {
  ORB_CHECK_TRUE(handle && data, EINVAL, return false);

  auto &dev = *(uorb::DeviceNode *)handle;
  return dev.Publish(data);
}

bool orb_publish_once(const struct orb_metadata *meta, const void *data) {
  ORB_CHECK_TRUE(meta, EINVAL, return false);

  auto &device_master = DeviceMaster::get_instance();
  auto *dev = device_master.OpenDeviceNode(*meta, 0);
  if (!dev) {
    return false;
  }

  dev->mark_untracked_publisher();
  return dev->Publish(data);
}

orb_subscription_t *orb_create_subscription(const struct orb_metadata *meta) {
  return orb_create_subscription_multi(meta, 0);
}

orb_subscription_t *orb_create_subscription_multi(const struct orb_metadata *meta, unsigned instance) {
  ORB_CHECK_TRUE(meta, EINVAL, return nullptr);

  DeviceMaster &device_master = uorb::DeviceMaster::get_instance();

  auto *dev = device_master.OpenDeviceNode(*meta, instance);
  if (!dev) {
    return nullptr;
  }

  auto *subscriber = new (std::nothrow) ReceiverLocal(*dev);
  if (!subscriber) {
    errno = ENOMEM;
    return nullptr;
  }

  return reinterpret_cast<orb_subscription_t *>(subscriber);
}

bool orb_destroy_subscription(orb_subscription_t **handle_ptr) {
  ORB_CHECK_TRUE(handle_ptr && *handle_ptr, EINVAL, return false);

  auto *r = reinterpret_cast<ReceiverLocal *>(*handle_ptr);

  // Remove callback if registered
  if (r->publish_cb) {
    r->dev.UnregisterCallback(&r->callback_entry);
  }
  delete r;
  *handle_ptr = nullptr;
  return true;
}

bool orb_copy(orb_subscription_t *handle, void *buffer) {
  ORB_CHECK_TRUE(handle && buffer, EINVAL, return false);

  auto &sub = *reinterpret_cast<ReceiverLocal *>(handle);

  return sub.Copy(buffer);
}

bool orb_copy_once(const struct orb_metadata *meta, void *buffer) {
  ORB_CHECK_TRUE(meta, EINVAL, return false);

  auto &device_master = DeviceMaster::get_instance();
  auto *dev = device_master.OpenDeviceNode(*meta, 0);
  if (!dev) {
    return false;
  }

  dev->mark_untracked_subscriber();
  unsigned last_generation_ = dev->initial_generation();
  return dev->Copy(buffer, &last_generation_);
}

bool orb_check_update(orb_subscription_t *handle) {
  ORB_CHECK_TRUE(handle, EINVAL, return false);

  auto &sub = *reinterpret_cast<ReceiverLocal *>(handle);

  return sub.updates_available();
}

bool orb_subscription_set_callback(orb_subscription_t *sub, orb_publish_callback_fn cb, void *ctx) {
  if (!sub) {
    errno = EINVAL;
    return false;
  }
  auto *r = reinterpret_cast<ReceiverLocal *>(sub);
  if (r->publish_cb) {
    errno = EBUSY;
    return false;
  }
  r->publish_cb = cb;
  r->publish_cb_ctx = ctx;
  return r->dev.RegisterCallback(&r->callback_entry);
}

bool orb_subscription_clear_callback(orb_subscription_t *sub) {
  if (!sub) {
    errno = EINVAL;
    return false;
  }
  auto *r = reinterpret_cast<ReceiverLocal *>(sub);
  if (!r->publish_cb) return true;
  bool ok = r->dev.UnregisterCallback(&r->callback_entry);
  if (ok) {
    r->publish_cb = nullptr;
    r->publish_cb_ctx = nullptr;
  }
  return ok;
}

bool orb_exists(const struct orb_metadata *meta, unsigned int instance) {
  ORB_CHECK_TRUE(meta, EINVAL, return false);

  auto &master = DeviceMaster::get_instance();
  return master.TopicExists(*meta, instance);
}

unsigned int orb_group_count(const struct orb_metadata *meta) {
  ORB_CHECK_TRUE(meta, EINVAL, return false);

  unsigned int instance = 0;

  for (unsigned int i = 0; i < ORB_MULTI_MAX_INSTANCES; ++i) {
    if (orb_exists(meta, i)) {
      ++instance;
    }
  }
  return instance;
}

bool orb_get_topic_status(const struct orb_metadata *meta, unsigned int instance, struct orb_status *status) {
  ORB_CHECK_TRUE(meta, EINVAL, return false);

  auto &master = DeviceMaster::get_instance();
  return master.GetTopicStatus(*meta, instance, status);
}

const char *orb_version(void) { return UORB_GIT_TAG; }
