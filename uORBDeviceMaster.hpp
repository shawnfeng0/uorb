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

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <list>

#include <sample/msg/uORB/topics/uORBTopics.hpp>

#include "base/atomic_bitset.h"
#include "base/orb_mutex.hpp"
#include "uORB.h"
#include "uORBCommon.hpp"

namespace uORB {
class DeviceNode;
class DeviceMaster;
class Manager;
}  // namespace uORB

/**
 * Master control device for ObjDev.
 *
 * Used primarily to create new objects via the ORBIOCCREATE
 * ioctl.
 */
class uORB::DeviceMaster {
 public:
  int advertise(const struct orb_metadata *meta, int *instance,
                ORB_PRIO priority);

  /**
   * Public interface for getDeviceNodeLocked(). Takes care of synchronization.
   * @return node if exists, nullptr otherwise
   */
  uORB::DeviceNode *getDeviceNode(const struct orb_metadata *meta,
                                  uint8_t instance);

  bool deviceNodeExists(ORB_ID id, uint8_t instance);

 private:
  // Private constructor, uORB::Manager takes care of its creation
  DeviceMaster() = default;
  ~DeviceMaster() = default;

  friend class uORB::Manager;

  /**
   * Find a node give its name.
   * _lock must already be held when calling this.
   * @return node if exists, nullptr otherwise
   */
  uORB::DeviceNode *getDeviceNodeLocked(const struct orb_metadata *meta,
                                        uint8_t instance);

  std::list<uORB::DeviceNode *> _node_list;
  uORB::base::AtomicBitset<ORB_TOPICS_COUNT>
      _node_exists[ORB_MULTI_MAX_INSTANCES];

  uORB::base::Mutex _lock; /**< lock to protect access to all class members
                              (also for derived classes) */
};
