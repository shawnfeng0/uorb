/**
 * @file uorb.cpp
 * A lightweight object broker.
 */

#include <uorb/uorb.h>

#include <cerrno>

#include "device_master.h"
#include "device_node.h"
#include "event_poll.h"
#include "receiver_local.h"

using namespace uorb;

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

orb_subscription_t *orb_create_subscription_multi(const struct orb_metadata *meta, unsigned instance) {
  ORB_CHECK_TRUE(meta, EINVAL, return nullptr);

  DeviceMaster &device_master = uorb::DeviceMaster::get_instance();

  auto *dev = device_master.OpenDeviceNode(*meta, instance);
  ORB_CHECK_TRUE(dev, ENOMEM, return nullptr);

  // Create a subscriber, if it fails, we don't have to release device_node (it
  // only increases but not decreases)
  auto *subscriber = new ReceiverLocal(*dev);
  ORB_CHECK_TRUE(subscriber, ENOMEM, return nullptr);

  return reinterpret_cast<orb_subscription_t *>(subscriber);
}

bool orb_destroy_subscription(orb_subscription_t **handle_ptr) {
  ORB_CHECK_TRUE(handle_ptr && *handle_ptr, EINVAL, return false);

  auto &subscription_handle = *handle_ptr;

  delete reinterpret_cast<ReceiverLocal *>(subscription_handle);
  subscription_handle = nullptr;  // Set the original pointer to null

  return true;
}

bool orb_copy(orb_subscription_t *handle, void *buffer) {
  ORB_CHECK_TRUE(handle && buffer, EINVAL, return false);

  auto &sub = *reinterpret_cast<ReceiverLocal *>(handle);

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

  auto &sub = *reinterpret_cast<ReceiverLocal *>(handle);

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

bool orb_get_topic_status(const struct orb_metadata *meta, unsigned int instance, struct orb_status *status) {
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
  for (unsigned i = 0; i < nfds; ++i) {
    ORB_CHECK_TRUE(fds[i].fd != nullptr, EINVAL, return -1);
  }

  int number_of_new_data = 0;

  auto CheckDataUpdate = [&] {
    number_of_new_data = 0;
    for (unsigned i = 0; i < nfds; ++i) {
      fds[i].revents = 0;
      auto &item_sub = *reinterpret_cast<ReceiverLocal *>(fds[i].fd);
      if (item_sub.updates_available()) {
        fds[i].revents |= fds[i].events & POLLIN;
        ++number_of_new_data;
      }
    }
    return number_of_new_data > 0;
  };

  if (CheckDataUpdate()) return number_of_new_data;

  uorb::base::LiteNotifier notifier;
  for (unsigned i = 0; i < nfds; ++i) {
    auto &item_sub = *reinterpret_cast<ReceiverLocal *>(fds[i].fd);
    item_sub.SetNotifier(&notifier);
  }

  if (timeout_ms > 0) {
    notifier.wait_for(timeout_ms, CheckDataUpdate);
  } else if (timeout_ms < 0) {
    notifier.wait(CheckDataUpdate);
  }

  for (unsigned i = 0; i < nfds; ++i) {
    auto &item_sub = *reinterpret_cast<ReceiverLocal *>(fds[i].fd);
    item_sub.RemoveNotifier();
  }

  return number_of_new_data;
}

orb_event_poll_t *orb_event_poll_create(void) {
  auto *cpp_poll = new uorb::EventPoll();
  return reinterpret_cast<orb_event_poll_t *>(cpp_poll);
}

bool orb_event_poll_destroy(orb_event_poll_t **handle_ptr) {
  ORB_CHECK_TRUE(handle_ptr && *handle_ptr, EINVAL, return false);
  auto *cpp_poll = reinterpret_cast<uorb::EventPoll *>(*handle_ptr);
  delete cpp_poll;
  *handle_ptr = nullptr;
  return true;
}

bool orb_event_poll_add(orb_event_poll_t *poll, orb_subscription_t *sub) {
  ORB_CHECK_TRUE(poll && sub, EINVAL, return false);
  auto *cpp_poll = reinterpret_cast<uorb::EventPoll *>(poll);
  auto *receiver = reinterpret_cast<uorb::ReceiverLocal *>(sub);
  cpp_poll->AddReceiver(*receiver);
  return true;
}

bool orb_event_poll_remove(orb_event_poll_t *poll, orb_subscription_t *sub) {
  ORB_CHECK_TRUE(poll && sub, EINVAL, return false);
  auto *cpp_poll = reinterpret_cast<uorb::EventPoll *>(poll);
  auto *receiver = reinterpret_cast<uorb::ReceiverLocal *>(sub);
  cpp_poll->RemoveReceiver(*receiver);
  return true;
}

int orb_event_poll_wait(orb_event_poll_t *poll, orb_subscription_t *subs[], int max_subs, int timeout_ms) {
  ORB_CHECK_TRUE(poll && subs && max_subs > 0, EINVAL, return -1);
  auto *cpp_poll = reinterpret_cast<uorb::EventPoll *>(poll);
  return cpp_poll->Wait(reinterpret_cast<uorb::ReceiverLocal **>(subs), max_subs, timeout_ms);
}

bool orb_event_poll_quit(orb_event_poll_t *poll) {
  ORB_CHECK_TRUE(poll, EINVAL, return false);
  auto *cpp_poll = reinterpret_cast<uorb::EventPoll *>(poll);
  cpp_poll->Stop();
  return true;
}
