/**
 * @file uorb.cpp
 * A lightweight object broker.
 */

#include <uorb/uorb.h>

#include <new>

#include "base/allocator.h"
#include "device_master.h"
#include "device_node.h"

namespace uorb {
namespace base {
std::atomic<orb_alloc_fn> Allocator::alloc_fn_{nullptr};
std::atomic<orb_free_fn> Allocator::free_fn_{nullptr};
}  // namespace base
}  // namespace uorb

using namespace uorb;

// ReceiverLocal: internal struct representing a uORB subscription.
// Stores DeviceNode reference, generation tracking, and an optional
// publish callback (set by the bridge library via orb_subscriber_set_callback).
struct ReceiverLocal {
  DeviceNode &dev;
  unsigned last_generation;
  bool callback_registered;
  detail::CallbackEntry callback_entry;

  explicit ReceiverLocal(DeviceNode &device_node)
      : dev(device_node),
        last_generation(device_node.initial_generation()),
        callback_registered(false) {
    device_node.add_subscriber();
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
  static_assert(sizeof(orb_publisher_t) == sizeof(void *),
                "orb_publisher_t must have same layout as void*");
  static_assert(sizeof(orb_subscriber_t) == sizeof(void *),
                "orb_subscriber_t must have same layout as void*");
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

void orb_publisher_destroy(orb_publisher_t *pub) {
  if (!pub || !pub->_handle) {
    return;
  }

  auto *dev = reinterpret_cast<uorb::DeviceNode *>(pub->_handle);
  dev->remove_publisher();

  pub->_handle = nullptr;
}

orb_err orb_publisher_publish(orb_publisher_t pub, const void *data) {
  if (!pub._handle || !data) {
    return ORB_ERR_INVALID;
  }

  auto &dev = *reinterpret_cast<uorb::DeviceNode *>(pub._handle);
  return dev.Publish(data) ? ORB_OK : ORB_ERR_NO_MEM;
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
  return dev->Publish(data) ? ORB_OK : ORB_ERR_NO_MEM;
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

  auto *subscriber = base::Allocator::New<ReceiverLocal>(*dev);
  if (!subscriber) {
    return ORB_ERR_NO_MEM;
  }

  sub->_handle = reinterpret_cast<void *>(subscriber);
  return ORB_OK;
}

void orb_subscriber_destroy(orb_subscriber_t *sub) {
  if (!sub || !sub->_handle) {
    return;
  }

  auto *r = reinterpret_cast<ReceiverLocal *>(sub->_handle);

  if (r->callback_registered) {
    r->dev.UnregisterCallback(&r->callback_entry);
    r->callback_registered = false;
  }
  base::Allocator::Delete(r);
  sub->_handle = nullptr;
}

orb_err orb_subscriber_copy(orb_subscriber_t sub, void *buffer) {
  if (!sub._handle || !buffer) {
    return ORB_ERR_INVALID;
  }

  auto &r = *reinterpret_cast<ReceiverLocal *>(sub._handle);

  return r.Copy(buffer) ? ORB_OK : ORB_ERR_NO_ENTRY;
}

orb_err orb_subscriber_copy_once(const struct orb_metadata *meta, void *buffer) {
  if (!meta || !buffer) {
    return ORB_ERR_INVALID;
  }

  if (meta->o_durability == ORB_DURABILITY_VOLATILE) {
    return ORB_ERR_NO_ENTRY;
  }

  auto &device_master = DeviceMaster::get_instance();
  DeviceNode *dev = nullptr;
  orb_err err = device_master.OpenDeviceNode(*meta, 0, &dev);
  if (err != ORB_OK) {
    return err;
  }

  dev->mark_untracked_subscriber();
  unsigned last_generation_ = dev->initial_generation();
  return dev->Copy(buffer, &last_generation_) ? ORB_OK : ORB_ERR_NO_ENTRY;
}

bool orb_subscriber_check_update(orb_subscriber_t sub) {
  if (!sub._handle) {
    return false;
  }

  auto &r = *reinterpret_cast<ReceiverLocal *>(sub._handle);

  return r.updates_available() > 0;
}

orb_err orb_subscriber_set_callback(orb_subscriber_t sub, orb_subscriber_callback_fn cb,
                                    orb_callback_ctx ctx) {
  if (!sub._handle || !cb) {
    return ORB_ERR_INVALID;
  }
  auto *r = reinterpret_cast<ReceiverLocal *>(sub._handle);
  if (r->callback_registered) {
    return ORB_ERR_BUSY;
  }
  r->callback_entry.on_publish = cb;
  r->callback_entry.ctx = ctx;
  r->callback_registered = true;
  if (!r->dev.RegisterCallback(&r->callback_entry)) {
    r->callback_registered = false;
    r->callback_entry.on_publish = nullptr;
    r->callback_entry.ctx = {};
    return ORB_ERR_UNKNOWN;
  }
  return ORB_OK;
}

orb_err orb_subscriber_clear_callback(orb_subscriber_t sub) {
  if (!sub._handle) {
    return ORB_ERR_INVALID;
  }
  auto *r = reinterpret_cast<ReceiverLocal *>(sub._handle);
  if (!r->callback_registered) {
    return ORB_OK;
  }
  bool ok = r->dev.UnregisterCallback(&r->callback_entry);
  if (ok) {
    r->callback_registered = false;
    r->callback_entry.on_publish = nullptr;
    r->callback_entry.ctx = {};
  }
  return ok ? ORB_OK : ORB_ERR_UNKNOWN;
}

bool orb_exists(const struct orb_metadata *meta, unsigned int instance) {
  if (!meta || instance >= ORB_MULTI_MAX_INSTANCES) {
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
  if (!meta || !status || instance >= ORB_MULTI_MAX_INSTANCES) {
    return ORB_ERR_INVALID;
  }

  auto &master = DeviceMaster::get_instance();
  return master.GetTopicStatus(*meta, instance, status) ? ORB_OK : ORB_ERR_NO_ENTRY;
}

size_t orb_for_each_topic(orb_topic_visitor_fn visitor, void *ctx) {
  auto &master = DeviceMaster::get_instance();
  return master.ForEachTopic(visitor, ctx);
}

orb_err orb_set_allocator(orb_alloc_fn alloc_fn, orb_free_fn free_fn) {
  return base::Allocator::Set(alloc_fn, free_fn);
}

orb_err orb_topic_preallocate(const struct orb_metadata *meta, unsigned int instance) {
  if (!meta || instance >= ORB_MULTI_MAX_INSTANCES) {
    return ORB_ERR_INVALID;
  }
  auto &master = DeviceMaster::get_instance();
  DeviceNode *dev = nullptr;
  const orb_err err = master.OpenDeviceNode(*meta, instance, &dev);
  if (err != ORB_OK) {
    return err;
  }
  return dev->EnsureBuffer() ? ORB_OK : ORB_ERR_NO_MEM;
}

const char *orb_version(void) { return UORB_GIT_TAG; }

const char *orb_strerror(orb_err err) {
  switch (err) {
    case ORB_OK:
      return "OK";
    case ORB_ERR_INVALID:
      return "Invalid argument";
    case ORB_ERR_NO_MEM:
      return "Out of memory";
    case ORB_ERR_BUSY:
      return "Resource busy";
    case ORB_ERR_EXIST:
      return "Already exists";
    case ORB_ERR_NO_ENTRY:
      return "Not found / no data";
    case ORB_ERR_TIMEOUT:
      return "Timed out";
    case ORB_ERR_AGAIN:
      return "Try again";
    case ORB_ERR_UNKNOWN:
    default:
      return "Unknown error";
  }
}
