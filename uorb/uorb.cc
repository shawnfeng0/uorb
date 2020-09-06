/****************************************************************************
 *
 *   Copyright (c) 2012-2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file uorb.cpp
 * A lightweight object broker.
 */

#include "uorb/uorb.h"

#include "uorb/base/orb_errno.h"
#include "uorb/device_master.h"
#include "uorb/device_node.h"
#include "uorb/subscription_interval.h"

using namespace uorb;

#define ORB_CHECK_TRUE(condition, error_code, error_action) \
  ({                                                        \
    if (!static_cast<bool>(condition)) {                    \
      orb_errno = error_code;                               \
      error_action;                                         \
    }                                                       \
  })

class SubscriberC : public SubscriptionInterval {
 public:
  explicit SubscriberC(const orb_metadata &meta, uint32_t interval_us = 0,
                       uint8_t instance = 0)
      : SubscriptionInterval(meta, interval_us, instance) {}
  auto get_node() -> decltype(node_) { return node_; }
};

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

  ORB_CHECK_TRUE(dev_, orb_errno, return nullptr);

  return (orb_publication_t *)dev_;
}

bool orb_destroy_publication(orb_publication_t **handle_ptr) {
  ORB_CHECK_TRUE(handle_ptr, EINVAL, return false);

  orb_publication_t *&handle = *handle_ptr;
  ORB_CHECK_TRUE(handle, EINVAL, return false);

  auto &dev = *(uorb::DeviceNode *)handle;
  dev.mark_as_unadvertised();

  handle = nullptr;

  return true;
}

bool orb_publish(orb_publication_t *handle, const void *data) {
  ORB_CHECK_TRUE(handle && data, EINVAL, return false);

  auto &dev = *(uorb::DeviceNode *)handle;
  return dev.Publish(data);
}

orb_subscription_t *orb_create_subscription(const struct orb_metadata *meta) {
  return orb_create_subscription_multi(meta, 0);
}

orb_subscription_t *orb_create_subscription_multi(const struct orb_metadata *meta,
                                            unsigned instance) {
  ORB_CHECK_TRUE(meta, EINVAL, return nullptr);

  auto *sub = new SubscriberC(*meta, 0, instance);
  ORB_CHECK_TRUE(sub, ENOMEM, return nullptr);

  return (orb_subscription_t *)sub;
}

bool orb_destroy_subscription(orb_subscription_t **handle_ptr) {
  ORB_CHECK_TRUE(handle_ptr, EINVAL, return false);

  orb_subscription_t *&handle = *handle_ptr;
  ORB_CHECK_TRUE(handle, EINVAL, return false);

  delete (SubscriberC *)handle;
  handle = nullptr;

  return true;
}

bool orb_copy(orb_subscription_t *handle, void *buffer) {
  ORB_CHECK_TRUE(handle && buffer, EINVAL, return false);

  auto &sub = *(SubscriberC *)handle;
  return sub.copy(buffer);
}

bool orb_check_updated(orb_subscription_t *handle) {
  ORB_CHECK_TRUE(handle, EINVAL, return false);
  auto &sub = *(SubscriberC *)handle;
  return sub.updated();
}

bool orb_exists(const struct orb_metadata *meta, unsigned int instance) {
  ORB_CHECK_TRUE(meta, EINVAL, return false);

  auto &master = DeviceMaster::get_instance();
  auto *dev = master.GetDeviceNode(*meta, instance);

  if (dev) return dev->is_advertised();

  return false;
}

unsigned int orb_group_count(const struct orb_metadata *meta) {
  ORB_CHECK_TRUE(meta, EINVAL, return false);

  unsigned int instance = 0;

  while (orb_exists(meta, instance)) {
    ++instance;
  }

  return instance;
}

int orb_poll(struct orb_pollfd *fds, unsigned int nfds, int timeout_ms) {
  ORB_CHECK_TRUE(fds && nfds, EINVAL, return -1);

  uorb::DeviceNode::SemaphoreCallback semaphore_callback;
  int updated_num = 0;  // Number of new messages

  for (unsigned i = 0; i < nfds; i++) {
    orb_subscription_t *&subscriber = fds[i].fd;
    if (!subscriber) continue;

    auto &sub = *((SubscriberC *)subscriber);
    if (!sub.get_node()) sub.subscribe();
    if (!sub.get_node()) continue;

    unsigned &event = fds[i].events;
    unsigned &revent = fds[i].revents;

    // Maybe there is new data before the callback is registered
    if (sub.updated()) {
      revent = event & POLLIN;
      ++updated_num;
    } else {
      sub.get_node()->RegisterCallback(&semaphore_callback);
      revent = 0;
    }
  }

  // No new data, waiting for update
  if (0 == updated_num) semaphore_callback.try_acquire_for(timeout_ms);

  // Cancel registration callback, re-count the number of new messages
  updated_num = 0;
  for (unsigned i = 0; i < nfds; i++) {
    orb_subscription_t *&subscriber = fds[i].fd;
    if (!subscriber) continue;

    auto &sub = *((SubscriberC *)subscriber);
    if (!sub.get_node()) sub.subscribe();
    if (!sub.get_node()) continue;

    unsigned &event = fds[i].events;
    unsigned &revent = fds[i].revents;

    sub.get_node()->UnregisterCallback(&semaphore_callback);

    if (sub.updated()) {
      revent |= event & POLLIN;
      ++updated_num;
    }
  }

  return updated_num;
}
