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
// Inherits uevent::EventSource directly so any subscriber handle can be added
// to a uevent loop via orb_sub_event_source() with zero extra allocation
// and zero synchronous user-callback hooks.
struct ReceiverLocal final : public uevent::EventSource {
  DeviceNode &dev;
  unsigned last_generation;
  detail::WaiterEntry waiter_entry;
  std::atomic<bool> waiter_registered{false};

  explicit ReceiverLocal(DeviceNode &device_node)
      : dev(device_node),
        last_generation(device_node.initial_generation()),
        waiter_entry{this, {}} {
    device_node.add_subscriber();
  }

  ~ReceiverLocal() override {
    if (waiter_registered.exchange(false, std::memory_order_acq_rel)) {
      dev.UnregisterWaiter(&waiter_entry);
    }
    dev.remove_subscriber();
  }

  bool is_owned_by_uevent() const override { return false; }

  bool is_ready() const override { return updates_available() > 0; }

  uevent_err SetWakeup(uevent::EventPoll *poll) override {
    const uevent_err err = EventSource::SetWakeup(poll);
    if (err != UEVENT_OK) {
      return err;
    }
    if (!waiter_registered.exchange(true, std::memory_order_acq_rel)) {
      if (!dev.RegisterWaiter(&waiter_entry)) {
        waiter_registered.store(false, std::memory_order_release);
        ClearWakeup();
        return UEVENT_ERR_INVALID;
      }
    }
    return UEVENT_OK;
  }

  void OnRemoved() override {
    if (waiter_registered.exchange(false, std::memory_order_acq_rel)) {
      dev.UnregisterWaiter(&waiter_entry);
    }
  }

  bool Copy(void *buffer) { return dev.Copy(buffer, &last_generation); }
  unsigned updates_available() const { return dev.updates_available(last_generation); }
};

#ifndef UORB_GIT_TAG
#define UORB_GIT_TAG "v0.0.0-0-unknown"
#endif

orb_err orb_pub_create(orb_pub_t *pub, const struct orb_metadata *meta) {
  static_assert(sizeof(orb_pub_t) == sizeof(void *),
                "orb_pub_t must have same layout as void*");
  static_assert(sizeof(orb_sub_t) == sizeof(void *),
                "orb_sub_t must have same layout as void*");
  return orb_pub_create_multi(pub, meta, nullptr);
}

orb_err orb_pub_create_multi(orb_pub_t *pub, const struct orb_metadata *meta, unsigned int *instance) {
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

void orb_pub_destroy(orb_pub_t *pub) {
  if (!pub || !pub->_handle) {
    return;
  }

  auto *dev = reinterpret_cast<uorb::DeviceNode *>(pub->_handle);
  dev->remove_publisher();

  pub->_handle = nullptr;
}

orb_err orb_pub_publish(orb_pub_t pub, const void *data) {
  if (!pub._handle || !data) {
    return ORB_ERR_INVALID;
  }

  auto &dev = *reinterpret_cast<uorb::DeviceNode *>(pub._handle);
  return dev.Publish(data) ? ORB_OK : ORB_ERR_NO_MEM;
}

orb_err orb_pub_loan(orb_pub_t pub, void **out_msg) {
  if (!pub._handle || !out_msg) {
    return ORB_ERR_INVALID;
  }

  auto &dev = *reinterpret_cast<uorb::DeviceNode *>(pub._handle);
  return dev.Loan(out_msg);
}

orb_err orb_pub_publish_loaned(orb_pub_t pub, void *loaned_msg) {
  if (!pub._handle || !loaned_msg) {
    return ORB_ERR_INVALID;
  }

  auto &dev = *reinterpret_cast<uorb::DeviceNode *>(pub._handle);
  return dev.PublishLoaned(loaned_msg);
}

void orb_pub_discard_loan(orb_pub_t pub, void *loaned_msg) {
  if (!pub._handle || !loaned_msg) {
    return;
  }

  auto &dev = *reinterpret_cast<uorb::DeviceNode *>(pub._handle);
  dev.DiscardLoan(loaned_msg);
}

orb_err orb_pub_publish_once(const struct orb_metadata *meta, const void *data) {
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

orb_err orb_sub_create(orb_sub_t *sub, const struct orb_metadata *meta) {
  return orb_sub_create_multi(sub, meta, 0);
}

orb_err orb_sub_create_multi(orb_sub_t *sub, const struct orb_metadata *meta, unsigned instance) {
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

void orb_sub_destroy(orb_sub_t *sub) {
  if (!sub || !sub->_handle) {
    return;
  }

  auto *r = reinterpret_cast<ReceiverLocal *>(sub->_handle);
  base::Allocator::Delete(r);
  sub->_handle = nullptr;
}

orb_err orb_sub_copy(orb_sub_t sub, void *buffer) {
  if (!sub._handle || !buffer) {
    return ORB_ERR_INVALID;
  }

  auto &r = *reinterpret_cast<ReceiverLocal *>(sub._handle);

  return r.Copy(buffer) ? ORB_OK : ORB_ERR_NO_ENTRY;
}

orb_err orb_sub_copy_once(const struct orb_metadata *meta, void *buffer) {
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

bool orb_sub_check_update(orb_sub_t sub) {
  if (!sub._handle) {
    return false;
  }

  auto &r = *reinterpret_cast<ReceiverLocal *>(sub._handle);

  return r.updates_available() > 0;
}

uevent_source_t orb_sub_event_source(orb_sub_t sub) {
  uevent_source_t src = UEVENT_SOURCE_INITIALIZER;
  if (!sub._handle) {
    return src;
  }
  auto *r = reinterpret_cast<ReceiverLocal *>(sub._handle);
  src._handle = static_cast<uevent::EventSource *>(r);
  return src;
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
