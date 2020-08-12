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
 * @file uORB.cpp
 * A lightweight object broker.
 */

#include "uorb/uORB.h"

#include "uorb/DeviceMaster.hpp"
#include "uorb/DeviceNode.hpp"
#include "uorb/SubscriptionInterval.h"
#include "uorb/base/errno.h"

using namespace uorb;

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
  if (!meta) {
    orb_errno = EINVAL;
    return nullptr;
  }
  auto &meta_ = *meta;

  auto &device_master = DeviceMaster::get_instance();
  auto *dev_ = device_master.CreateAdvertiser(meta_, instance, queue_size);

  if (!dev_) {
    return nullptr;
  }

  if (data) dev_->Publish(meta_, data);

  return (orb_advert_t)dev_;
}

bool orb_unadvertise(orb_advert_t *handle_ptr) {
  if (!handle_ptr) {
    orb_errno = EINVAL;
    return false;
  }

  orb_advert_t &handle = *handle_ptr;

  if (!handle) {
    orb_errno = EINVAL;
    return false;
  }

  auto &dev = *(DeviceNode *)handle;
  dev.mark_as_unadvertised();

  handle = nullptr;

  return true;
}

bool orb_publish(const struct orb_metadata *meta, orb_advert_t handle,
                const void *data) {
  if (!meta || !handle || !data) {
    orb_errno = EINVAL;
    return false;
  }

  auto *dev = (DeviceNode *)handle;

  return dev->Publish(*meta, data);
}

orb_subscriber_t orb_subscribe(const struct orb_metadata *meta) {
  return orb_subscribe_multi(meta, 0);
}

orb_subscriber_t orb_subscribe_multi(const struct orb_metadata *meta,
                                     unsigned instance) {
  if (!meta) {
    orb_errno = EINVAL;
    return nullptr;
  }

  auto *sub = new SubscriptionInterval(*meta, instance);
  if (!sub) {
    orb_errno = ENOMEM;
    return nullptr;
  }

  return (orb_subscriber_t)sub;
}

bool orb_unsubscribe(orb_subscriber_t *handle_ptr) {
  if (!handle_ptr) {
    orb_errno = EINVAL;
    return false;
  }

  orb_subscriber_t &handle = *handle_ptr;

  if (!handle) {
    orb_errno = EINVAL;
    return false;
  }

  delete (SubscriptionInterval *)handle;

  handle = nullptr;

  return true;
}

bool orb_copy(const struct orb_metadata *meta, orb_subscriber_t handle,
             void *buffer) {
  if (!meta || !handle || !buffer) {
    orb_errno = EINVAL;
    return false;
  }

  auto &sub = *(SubscriptionInterval *)handle;
  return sub.copy(buffer);
}

bool orb_check_updated(orb_subscriber_t handle) {
  if (!handle) {
    orb_errno = EINVAL;
    return false;
  }
  auto &sub = *(SubscriptionInterval *)handle;
  return sub.updated();
}

bool orb_exists(const struct orb_metadata *meta, unsigned int instance) {
  if (!meta) {
    orb_errno = EINVAL;
    return false;
  }

  auto &master = DeviceMaster::get_instance();
  return master.GetDeviceNode(*meta, instance) != nullptr;
}

unsigned int orb_group_count(const struct orb_metadata *meta) {
  if (!meta) {
    orb_errno = EINVAL;
    return false;
  }
  unsigned int instance = 0;

  while (orb_exists(meta, instance)) {
    ++instance;
  }

  return instance;
}

bool orb_set_interval(orb_subscriber_t handle, unsigned interval_ms) {
  if (!handle) {
    orb_errno = EINVAL;
    return false;
  }
  auto &sub = *(SubscriptionInterval *)handle;
  sub.set_interval_ms(interval_ms);
  return true;
}

unsigned int orb_get_interval(orb_subscriber_t handle) {
  if (!handle) {
    orb_errno = EINVAL;
    return false;
  }
  auto &sub = *(SubscriptionInterval *)handle;
  return sub.get_interval_ms();
}
