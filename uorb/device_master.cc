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

#include "uorb/device_master.h"

#include "uorb/base/errno.h"
#include "uorb/device_node.h"

uorb::DeviceMaster uorb::DeviceMaster::instance_;

uorb::DeviceNode *uorb::DeviceMaster::CreateAdvertiser(const orb_metadata &meta,
                                                       unsigned int *instance,
                                                       uint16_t queue_size) {
  const bool is_single_instance = !instance;
  const unsigned max_group_tries = is_single_instance ? 1 : ORB_MULTI_MAX_INSTANCES;

  DeviceNode *device_node;
  unsigned group_tries = 0;

  uorb::base::WriterLockGuard lg(lock_);

  // Find the following devices that can advertise:
  // - Unadvertised device
  // - Single instance device
  // - Unregistered device
  do {
    device_node = GetDeviceNodeLocked(meta, group_tries);
    if (device_node &&
        (!device_node->is_advertised() || is_single_instance)) {
      device_node->set_queue_size(queue_size);
      device_node->mark_as_advertised();
      break;  // Find a unadvertised device or single instance device
    }

    if (!device_node) {
      device_node = new DeviceNode(meta, group_tries, queue_size);
      if (!device_node) {
        orb_errno = ENOMEM;
        return nullptr;
      }
      device_node->mark_as_advertised();
      node_list_.Add(device_node);
      break;  // Create new device
    }
    group_tries++;
  } while (group_tries < max_group_tries);

  // All instances already exist
  if (group_tries >= max_group_tries) {
    orb_errno = EEXIST;
    return nullptr;
  }

  if (instance) *instance = group_tries;
  return device_node;
}

uorb::DeviceNode *uorb::DeviceMaster::GetDeviceNode(const orb_metadata &meta,
                                                    uint8_t instance) {
  uorb::base::ReaderLockGuard lg(lock_);
  return GetDeviceNodeLocked(meta, instance);
}

uorb::DeviceNode *uorb::DeviceMaster::GetDeviceNodeLocked(
    const orb_metadata &meta, uint8_t instance) const {
  // We can safely return the node that can be used by any thread, because a
  // DeviceNode never gets deleted.
  for (auto node : node_list_) {
    if (node->IsSameWith(meta, instance)) return node;
  }

  return nullptr;
}

uorb::DeviceNode *uorb::DeviceMaster::OpenDeviceNode(const orb_metadata &meta,
                                                     unsigned int instance) {
  if (instance >= ORB_MULTI_MAX_INSTANCES) {
    return nullptr;
  }

  uorb::base::WriterLockGuard lg(lock_);

  DeviceNode *device_node = GetDeviceNodeLocked(meta, instance);
  if (device_node) {
    return device_node;
  }

  device_node = new DeviceNode(meta, instance);

  if (!device_node) {
    orb_errno = ENOMEM;
    return nullptr;
  }

  node_list_.Add(device_node);

  return device_node;  // Create new device
}
