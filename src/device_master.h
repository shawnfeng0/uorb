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

#pragma once

#include <cstdint>
#include <list>

#include "base/intrusive_list.h"
#include "base/mutex.h"
#include "base/rw_mutex.h"
#include "uorb/uorb.h"

namespace uorb {
class DeviceNode;
class DeviceMaster;
}  // namespace uorb

/**
 * Master control device for uorb message device node.
 */
class uorb::DeviceMaster {
 public:
  /**
   * Method to get the singleton instance for the uorb::DeviceMaster.
   * @return DeviceMaster instance reference
   */
  static inline DeviceMaster &get_instance() { return instance_; }

  /**
   * Advertise as the publisher of a topic
   * @param meta The uORB metadata (usually from the *ORB_ID() macro) for the
   * topic.
   * @param instance  Pointer to an integer which will yield the instance ID
   * (0-based) of the publication. This is an output parameter and will be set
   * to the newly created instance, ie. 0 for the first advertiser, 1 for the
   * next and so on. If it is nullptr, it will only be created at 0.
   * @param queue_size Maximum number of buffered elements. If this is 1, no
   * queuing is used.
   * @return nullptr on error, and set errno to orb_errno. Otherwise returns a
   * DeviceNode that can be used to publish to the topic.
   */
  DeviceNode *CreateAdvertiser(const orb_metadata &meta, unsigned int *instance,
                               unsigned int queue_size);

  DeviceNode *OpenDeviceNode(const orb_metadata &meta, unsigned int instance);

  /**
   * Public interface for GetDeviceNodeLocked(). Takes care of synchronization.
   * @return node if exists, nullptr otherwise
   */
  DeviceNode *GetDeviceNode(const orb_metadata &meta, uint8_t instance) const;

 private:
  /**
   * Find a node give its name.
   * lock_ must already be held when calling this.
   * @return node if exists, nullptr otherwise
   */
  DeviceNode *GetDeviceNodeLocked(const orb_metadata &meta,
                                  uint8_t instance) const;

  // Private constructor, uorb::Manager takes care of its creation
  DeviceMaster() = default;
  ~DeviceMaster() = default;

  static DeviceMaster instance_;

  List<DeviceNode *> node_list_;
  mutable base::RwMutex lock_; /**< lock to protect access to all class members
                             (also for derived classes) */
};
