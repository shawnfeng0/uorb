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

#include "uorb/base/errno.h"
#include "uorb/device_master.h"
#include "uorb/device_node.h"
#include "uorb/subscription_interval.h"

using namespace uorb;

#ifdef ORB_STRICT

#include <cassert>
#define ORB_ASSERT(condition, false_action) assert(condition)

#else

#define ORB_ASSERT(expr, false_action, ...) \
  ({                                        \
    if (!static_cast<bool>(expr)) {         \
      false_action;                         \
      __VA_ARGS__;                          \
    }                                       \
  })

#endif

class SubscriberC : public SubscriptionInterval {
 public:
  explicit SubscriberC(const orb_metadata &meta, uint32_t interval_us = 0,
                       uint8_t instance = 0)
      : SubscriptionInterval(meta, interval_us, instance) {}
  auto get_node() { return node_; }
};

orb_advert_t orb_advertise(const struct orb_metadata *meta, const void *data) {
  return orb_advertise_multi_queue(meta, data, nullptr, 1);
}

orb_advert_t orb_advertise_queue(const struct orb_metadata *meta,
                                 const void *data, unsigned int queue_size) {
  return orb_advertise_multi_queue(meta, data, nullptr, queue_size);
}

orb_advert_t orb_advertise_multi(const struct orb_metadata *meta,
                                 const void *data, unsigned int *instance) {
  return orb_advertise_multi_queue(meta, data, instance, 1);
}

orb_advert_t orb_advertise_multi_queue(const struct orb_metadata *meta,
                                       const void *data, unsigned int *instance,
                                       unsigned int queue_size) {
  ORB_ASSERT(meta, return nullptr);
  auto &meta_ = *meta;
  auto &device_master = DeviceMaster::get_instance();
  auto *dev_ = device_master.CreateAdvertiser(meta_, instance, queue_size);

  ORB_ASSERT(dev_, return nullptr);

  if (data) dev_->Publish(meta_, data);

  return (orb_advert_t)dev_;
}

bool orb_unadvertise(orb_advert_t *handle_ptr) {
  ORB_ASSERT(handle_ptr, return false);

  orb_advert_t &handle = *handle_ptr;
  ORB_ASSERT(handle, return false);

  auto &dev = *(uorb::DeviceNode *)handle;
  dev.mark_as_unadvertised();

  handle = nullptr;

  return true;
}

bool orb_publish(const struct orb_metadata *meta, orb_advert_t handle,
                 const void *data) {
  ORB_ASSERT(meta && handle && data, return false);

  auto *dev = (uorb::DeviceNode *)handle;
  return dev->Publish(*meta, data);
}

orb_subscriber_t orb_subscribe(const struct orb_metadata *meta) {
  return orb_subscribe_multi(meta, 0);
}

orb_subscriber_t orb_subscribe_multi(const struct orb_metadata *meta,
                                     unsigned instance) {
  ORB_ASSERT(meta, return nullptr);

  auto *sub = new SubscriberC(*meta, 0, instance);
  ORB_ASSERT(sub, return nullptr);

  return (orb_subscriber_t)sub;
}

bool orb_unsubscribe(orb_subscriber_t *handle_ptr) {
  ORB_ASSERT(handle_ptr, return false);

  orb_subscriber_t &handle = *handle_ptr;
  ORB_ASSERT(handle, return false);

  delete (SubscriberC *)handle;

  handle = nullptr;

  return true;
}

bool orb_copy(const struct orb_metadata *meta, orb_subscriber_t handle,
              void *buffer) {
  ORB_ASSERT(meta && handle && buffer, return false);

  auto &sub = *(SubscriberC *)handle;
  return sub.copy(buffer);
}

bool orb_check_updated(orb_subscriber_t handle) {
  ORB_ASSERT(handle, return false);
  auto &sub = *(SubscriberC *)handle;
  return sub.updated();
}

bool orb_exists(const struct orb_metadata *meta, unsigned int instance) {
  ORB_ASSERT(meta, return false);

  auto &master = DeviceMaster::get_instance();
  auto *dev = master.GetDeviceNode(*meta, instance);

  if (dev) return dev->is_advertised();

  return false;
}

unsigned int orb_group_count(const struct orb_metadata *meta) {
  ORB_ASSERT(meta, return false);

  unsigned int instance = 0;

  while (orb_exists(meta, instance)) {
    ++instance;
  }

  return instance;
}

bool orb_set_interval(orb_subscriber_t handle, unsigned interval_ms) {
  ORB_ASSERT(handle, return false);
  auto &sub = *(SubscriberC *)handle;
  sub.set_interval_ms(interval_ms);
  return true;
}

unsigned int orb_get_interval(orb_subscriber_t handle) {
  ORB_ASSERT(handle, return false);
  auto &sub = *(SubscriberC *)handle;
  return sub.get_interval_ms();
}

int orb_poll(struct orb_pollfd *fds, unsigned int nfds, int timeout_ms) {
  ORB_ASSERT(fds && nfds, orb_errno = EINVAL; return -1);

  uorb::DeviceNode::SemaphoreCallback semaphore_callback(0);
  int updated_num = 0;  // Number of new messages

  for (unsigned i = 0; i < nfds; i++) {
    orb_subscriber_t &subscriber = fds[i].fd;
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
    orb_subscriber_t &subscriber = fds[i].fd;
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
