/**
 * @file uorb.cpp
 * A lightweight object broker.
 */

#include <uorb/uorb.h>

#include <new>

#include "device_master.h"
#include "device_node.h"

using namespace uorb;

// ReceiverLocal: internal struct representing a uORB subscription.
// Stores DeviceNode reference, generation tracking, and an optional
// publish callback (set by the bridge library via orb_subscriber_set_callback).
struct ReceiverLocal {
  DeviceNode &dev;
  unsigned last_generation;
  orb_subscriber_callback_fn publish_cb;
  orb_callback_ctx publish_cb_ctx;
  detail::CallbackEntry callback_entry;

  explicit ReceiverLocal(DeviceNode &device_node) : dev(device_node) {
    last_generation = device_node.initial_generation();
    device_node.add_subscriber();
    publish_cb = nullptr;
    publish_cb_ctx = {};
    callback_entry.on_publish = [](const void *msg, void *ctx) {
      auto *self = static_cast<ReceiverLocal *>(ctx);
      if (self->publish_cb) self->publish_cb(msg, self->publish_cb_ctx);
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

orb_err orb_publisher_create(orb_publisher_t *pub, const struct orb_metadata *meta) {
  return orb_publisher_create_multi(pub, meta, nullptr);
}

orb_err orb_publisher_create_multi(orb_publisher_t *pub, const struct orb_metadata *meta, unsigned int *instance) {
  if (!pub || !meta) {
    return ORB_ERR_INVALID;
  }
  auto &meta_ = *meta;
  auto &device_master = DeviceMaster::get_instance();
  DeviceNode *dev_ = nullptr;
  orb_err err = device_master.CreateAdvertiser(meta_, instance, &dev_);
  if (err != ORB_OK) {
    return err;
  }

  pub->_handle = reinterpret_cast<void *>(dev_);
  return ORB_OK;
}

orb_err orb_publisher_destroy(orb_publisher_t *pub) {
  if (!pub || !pub->_handle) {
    return ORB_ERR_INVALID;
  }

  auto *dev = reinterpret_cast<uorb::DeviceNode *>(pub->_handle);
  dev->remove_publisher();

  pub->_handle = nullptr;

  return ORB_OK;
}

orb_err orb_publisher_publish(orb_publisher_t *pub, const void *data) {
  if (!pub || !pub->_handle || !data) {
    return ORB_ERR_INVALID;
  }

  auto &dev = *reinterpret_cast<uorb::DeviceNode *>(pub->_handle);
  return dev.Publish(data) ? ORB_OK : ORB_ERR_UNKNOWN;
}

orb_err orb_publisher_publish_once(const struct orb_metadata *meta, const void *data) {
  if (!meta || !data) {
    return ORB_ERR_INVALID;
  }

  auto &device_master = DeviceMaster::get_instance();
  DeviceNode *dev = nullptr;
  orb_err err = device_master.OpenDeviceNode(*meta, 0, &dev);
  if (err != ORB_OK) {
    return err;
  }

  dev->mark_untracked_publisher();
  return dev->Publish(data) ? ORB_OK : ORB_ERR_UNKNOWN;
}

orb_err orb_subscriber_create(orb_subscriber_t *sub, const struct orb_metadata *meta) {
  return orb_subscriber_create_multi(sub, meta, 0);
}

orb_err orb_subscriber_create_multi(orb_subscriber_t *sub, const struct orb_metadata *meta, unsigned instance) {
  if (!sub || !meta) {
    return ORB_ERR_INVALID;
  }

  DeviceMaster &device_master = uorb::DeviceMaster::get_instance();

  DeviceNode *dev = nullptr;
  orb_err err = device_master.OpenDeviceNode(*meta, instance, &dev);
  if (err != ORB_OK) {
    return err;
  }

  auto *subscriber = new (std::nothrow) ReceiverLocal(*dev);
  if (!subscriber) {
    return ORB_ERR_NO_MEM;
  }

  sub->_handle = reinterpret_cast<void *>(subscriber);
  return ORB_OK;
}

orb_err orb_subscriber_destroy(orb_subscriber_t *sub) {
  if (!sub || !sub->_handle) {
    return ORB_ERR_INVALID;
  }

  auto *r = reinterpret_cast<ReceiverLocal *>(sub->_handle);

  // Always unregister — safe even if not in the list (forward_list::remove is a no-op).
  // This avoids a race where clear_callback sets publish_cb=nullptr before destroy
  // checks it, which would skip UnregisterCallback and leave a dangling entry.
  r->dev.UnregisterCallback(&r->callback_entry);
  delete r;
  sub->_handle = nullptr;
  return ORB_OK;
}

orb_err orb_subscriber_copy(orb_subscriber_t *sub, void *buffer) {
  if (!sub || !sub->_handle || !buffer) {
    return ORB_ERR_INVALID;
  }

  auto &r = *reinterpret_cast<ReceiverLocal *>(sub->_handle);

  return r.Copy(buffer) ? ORB_OK : ORB_ERR_UNKNOWN;
}

orb_err orb_subscriber_copy_once(const struct orb_metadata *meta, void *buffer) {
  if (!meta || !buffer) {
    return ORB_ERR_INVALID;
  }

  auto &device_master = DeviceMaster::get_instance();
  DeviceNode *dev = nullptr;
  orb_err err = device_master.OpenDeviceNode(*meta, 0, &dev);
  if (err != ORB_OK) {
    return err;
  }

  dev->mark_untracked_subscriber();
  unsigned last_generation_ = dev->initial_generation();
  return dev->Copy(buffer, &last_generation_) ? ORB_OK : ORB_ERR_UNKNOWN;
}

bool orb_subscriber_check_update(orb_subscriber_t *sub) {
  if (!sub || !sub->_handle) {
    return false;
  }

  auto &r = *reinterpret_cast<ReceiverLocal *>(sub->_handle);

  return r.updates_available() > 0;
}

orb_err orb_subscriber_set_callback(orb_subscriber_t *sub, orb_subscriber_callback_fn cb,
                                    orb_callback_ctx ctx) {
  if (!sub || !sub->_handle) {
    return ORB_ERR_INVALID;
  }
  if (!cb) {
    return ORB_ERR_INVALID;
  }
  auto *r = reinterpret_cast<ReceiverLocal *>(sub->_handle);
  if (r->publish_cb) {
    return ORB_ERR_BUSY;
  }
  r->publish_cb = cb;
  r->publish_cb_ctx = ctx;
  if (!r->dev.RegisterCallback(&r->callback_entry)) {
    r->publish_cb = nullptr;
    r->publish_cb_ctx = {};
    return ORB_ERR_UNKNOWN;
  }
  return ORB_OK;
}

orb_err orb_subscriber_clear_callback(orb_subscriber_t *sub) {
  if (!sub || !sub->_handle) {
    return ORB_ERR_INVALID;
  }
  auto *r = reinterpret_cast<ReceiverLocal *>(sub->_handle);
  if (!r->publish_cb) return ORB_OK;
  bool ok = r->dev.UnregisterCallback(&r->callback_entry);
  if (ok) {
    r->publish_cb = nullptr;
    r->publish_cb_ctx = {};
  }
  return ok ? ORB_OK : ORB_ERR_UNKNOWN;
}

bool orb_exists(const struct orb_metadata *meta, unsigned int instance) {
  if (!meta) {
    return false;
  }

  auto &master = DeviceMaster::get_instance();
  return master.TopicExists(*meta, instance);
}

unsigned int orb_group_count(const struct orb_metadata *meta) {
  if (!meta) {
    return 0;
  }

  unsigned int instance = 0;

  for (unsigned int i = 0; i < ORB_MULTI_MAX_INSTANCES; ++i) {
    if (orb_exists(meta, i)) {
      ++instance;
    }
  }
  return instance;
}

orb_err orb_get_topic_status(const struct orb_metadata *meta, unsigned int instance, struct orb_status *status) {
  if (!meta || !status) {
    return ORB_ERR_INVALID;
  }

  auto &master = DeviceMaster::get_instance();
  return master.GetTopicStatus(*meta, instance, status) ? ORB_OK : ORB_ERR_UNKNOWN;
}

const char *orb_version(void) { return UORB_GIT_TAG; }
