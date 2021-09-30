/**
 * @file uorb.cpp
 * A lightweight object broker.
 */

#include <uorb/uorb.h>

#include <cerrno>

#include "callback.h"
#include "device_master.h"
#include "device_node.h"
#include "subscription_impl.h"

using uorb::DeviceMaster;
using uorb::DeviceNode;
using uorb::SubscriptionImpl;

#define ORB_CHECK_TRUE(condition, error_code, error_action) \
  ({                                                        \
    if (!static_cast<bool>(condition)) {                    \
      errno = error_code;                                   \
      error_action;                                         \
    }                                                       \
  })

orb_publication_t *orb_create_publication(const struct orb_metadata *meta,
                                          unsigned int queue_size) {
  return orb_create_publication_multi(meta, nullptr, queue_size);
}

orb_publication_t *orb_create_publication_multi(const struct orb_metadata *meta,
                                                unsigned int *instance,
                                                unsigned int queue_size) {
  ORB_CHECK_TRUE(meta, EINVAL, return nullptr);
  auto &meta_ = *meta;
  auto &device_master = DeviceMaster::get_instance();
  auto *dev_ = device_master.CreateAdvertiser(meta_, instance, queue_size);

  ORB_CHECK_TRUE(dev_, ENOMEM, return nullptr);

  return reinterpret_cast<orb_publication_t *>(dev_);
}

bool orb_destroy_publication(orb_publication_t **handle_ptr) {
  ORB_CHECK_TRUE(handle_ptr && *handle_ptr, EINVAL, return false);

  auto &publication_handle = *handle_ptr;

  auto &dev = *(uorb::DeviceNode *)publication_handle;
  dev.remove_publisher();

  publication_handle = nullptr;

  return true;
}

bool orb_publish(orb_publication_t *handle, const void *data) {
  ORB_CHECK_TRUE(handle && data, EINVAL, return false);

  auto &dev = *(uorb::DeviceNode *)handle;
  return dev.Publish(data);
}

bool orb_publish_anonymous(const struct orb_metadata *meta, const void *data) {
  ORB_CHECK_TRUE(meta, EINVAL, return false);

  auto &device_master = DeviceMaster::get_instance();
  auto *dev = device_master.OpenDeviceNode(*meta, 0);
  ORB_CHECK_TRUE(dev, ENOMEM, return false);

  // Mark as an anonymous publisher, then copy the latest data
  dev->mark_anonymous_publisher();
  return dev->Publish(data);
}

orb_subscription_t *orb_create_subscription(const struct orb_metadata *meta) {
  return orb_create_subscription_multi(meta, 0);
}

orb_subscription_t *orb_create_subscription_multi(
    const struct orb_metadata *meta, unsigned instance) {
  ORB_CHECK_TRUE(meta, EINVAL, return nullptr);

  DeviceMaster &device_master = uorb::DeviceMaster::get_instance();

  auto *dev = device_master.OpenDeviceNode(*meta, instance);
  ORB_CHECK_TRUE(dev, ENOMEM, return nullptr);

  // Create a subscriber, if it fails, we don't have to release device_node (it
  // only increases but not decreases)
  auto *subscriber = new SubscriptionImpl(*dev);
  ORB_CHECK_TRUE(subscriber, ENOMEM, return nullptr);

  return reinterpret_cast<orb_subscription_t *>(subscriber);
}

bool orb_destroy_subscription(orb_subscription_t **handle_ptr) {
  ORB_CHECK_TRUE(handle_ptr && *handle_ptr, EINVAL, return false);

  auto &subscription_handle = *handle_ptr;

  delete reinterpret_cast<SubscriptionImpl *>(subscription_handle);
  subscription_handle = nullptr;  // Set the original pointer to null

  return true;
}

bool orb_copy(orb_subscription_t *handle, void *buffer) {
  ORB_CHECK_TRUE(handle && buffer, EINVAL, return false);

  auto &sub = *reinterpret_cast<SubscriptionImpl *>(handle);

  return sub.Copy(buffer);
}

bool orb_copy_anonymous(const struct orb_metadata *meta, void *buffer) {
  ORB_CHECK_TRUE(meta, EINVAL, return false);

  auto &device_master = DeviceMaster::get_instance();
  auto *dev = device_master.OpenDeviceNode(*meta, 0);
  ORB_CHECK_TRUE(dev, ENOMEM, return false);

  // Mark as anonymous subscription, then copy the latest data
  dev->mark_anonymous_subscriber();
  unsigned last_generation_ = dev->initial_generation();
  return dev->Copy(buffer, &last_generation_);
}

bool orb_check_update(orb_subscription_t *handle) {
  ORB_CHECK_TRUE(handle, EINVAL, return false);

  auto &sub = *reinterpret_cast<SubscriptionImpl *>(handle);

  return sub.updates_available();
}

bool orb_exists(const struct orb_metadata *meta, unsigned int instance) {
  ORB_CHECK_TRUE(meta, EINVAL, return false);

  auto &master = DeviceMaster::get_instance();
  auto *dev = master.GetDeviceNode(*meta, instance);

  return dev && dev->publisher_count();
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

bool orb_get_topic_status(const struct orb_metadata *meta,
                          unsigned int instance, struct orb_status *status) {
  ORB_CHECK_TRUE(meta, EINVAL, return false);

  auto &master = DeviceMaster::get_instance();
  auto *dev = master.GetDeviceNode(*meta, instance);

  if (!dev) return false;

  if (status) {
    status->queue_size = dev->queue_size();
    status->subscriber_count = dev->subscriber_count();
    status->has_anonymous_subscriber = dev->has_anonymous_subscriber();
    status->publisher_count = dev->publisher_count();
    status->has_anonymous_publisher = dev->has_anonymous_publisher();
    status->latest_data_index = dev->updates_available(0);
  }
  return true;
}

int orb_poll(struct orb_pollfd *fds, unsigned int nfds, int timeout_ms) {
  ORB_CHECK_TRUE(fds && nfds, EINVAL, return -1);

  int updated_num = 0;  // Number of new messages
  uorb::SemaphoreCallback semaphore_callback;

  for (unsigned i = 0; i < nfds; ++i) {
    auto &item = fds[i];
    if (!item.fd) {
      continue;
    }

    auto &item_sub = *reinterpret_cast<SubscriptionImpl *>(item.fd);
    item_sub.RegisterCallback(&semaphore_callback);

    if (item_sub.updates_available() > 0) {
      ++updated_num;
    }
  }

  // No new data, waiting for update
  if (updated_num == 0) {
    semaphore_callback.try_acquire_for(timeout_ms);

  } else {
    updated_num = 0;
  }

  for (unsigned i = 0; i < nfds; ++i) {
    auto &item = fds[i];
    if (!item.fd) {
      continue;
    }

    auto &item_sub = *reinterpret_cast<SubscriptionImpl *>(item.fd);
    item_sub.UnregisterCallback(&semaphore_callback);

    item.revents = 0;
    if (item_sub.updates_available()) {
      item.revents |= item.events & POLLIN;
      ++updated_num;
    }
  }

  return updated_num;
}
