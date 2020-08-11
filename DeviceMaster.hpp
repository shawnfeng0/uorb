/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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

#include "base/mutex.hpp"
#include "uORB.h"

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
   * Method to get the singleton instance for the uORB::DeviceMaster.
   * @return uORB::DeviceMaster &
   */
  static inline uorb::DeviceMaster &get_instance() { return instance_; }

  DeviceNode *CreateAdvertiser(const orb_metadata &meta,
                               const unsigned int *instance,
                               uint16_t queue_size);
  /**
   * Public interface for GetDeviceNodeLocked(). Takes care of synchronization.
   * @return node if exists, nullptr otherwise
   */
  uorb::DeviceNode *GetDeviceNode(const orb_metadata &meta, uint8_t instance);

 private:
  uorb::DeviceNode *GetDeviceNodeLocked(const orb_metadata &meta,
                                        uint8_t instance) const;

  // Private constructor, uORB::Manager takes care of its creation
  DeviceMaster() = default;
  ~DeviceMaster() = default;

  static DeviceMaster instance_;

  std::list<uorb::DeviceNode *> _node_list;
  base::Mutex _lock; /**< lock to protect access to all class members
                              (also for derived classes) */
};
