/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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

#include <base/orb_errno.h>
#include <base/orb_log.h>
#include <sample/ulog/src/ulog_common.h>

#include <cstdint>

#include "base/orb_mutex.hpp"
#include "uORBDeviceMaster.hpp"
#include "uORBDeviceNode.hpp"

namespace uORB {
class Manager;
}

/**
 * This is implemented as a singleton.  This class manages creating the
 * uORB nodes for each uORB topics and also implements the behavior of the
 * uORB Api's.
 */
class uORB::Manager {
 public:
  // public interfaces for this class.

  /**
   * Method to get the singleton instance for the uORB::Manager.
   * @return uORB::Manager*
   */
  static inline uORB::Manager &get_instance() { return Instance; }

  /**
   * Get the DeviceMaster. If it does not exist,
   * it will be created and initialized.
   * Note: the first call to this is not thread-safe.
   * @return nullptr if initialization failed (and errno will be set)
   */
  uORB::DeviceMaster &get_device_master();

  // ==== uORB interface methods ====
  /**
   * Advertise as the publisher of a topic.
   *
   * This performs the initial advertisement of a topic; it creates the topic
   * node in /obj if required and publishes the initial data.
   *
   * Any number of advertisers may publish to a topic; publications are atomic
   * but co-ordination between publishers is not provided by the ORB.
   *
   * Internally this will call orb_advertise_multi with an instance of 0 and
   * default priority.
   *
   * @param meta    The uORB metadata (usually from the ORB_ID() macro)
   *      for the topic.
   * @param data    A pointer to the initial data to be published.
   *      For topics updated by interrupt handlers, the advertisement
   *      must be performed from non-interrupt context.
   * @param queue_size  Maximum number of buffered elements. If this is 1, no
   * queuing is used.
   * @return    nullptr on error, otherwise returns an object pointer
   *      that can be used to publish to the topic.
   *      If the topic in question is not known (due to an
   *      ORB_DEFINE with no corresponding ORB_DECLARE)
   *      this function will return nullptr and set errno to ENOENT.
   */
  orb_advert_t orb_advertise(const struct orb_metadata *meta, const void *data,
                             unsigned int queue_size = 1) {
    DeviceNode *dev = node_open(meta, nullptr);

    if (!dev) {
      ORB_ERR("%s advertise failed (%i)", meta->o_name, orb_errno);
      return nullptr;
    }

    /* Set the queue size. This must be done before the first publication; thus
     * it fails if this is not the first advertiser.
     */
    if (!dev->SetQueueSize(queue_size))
      ORB_WARN("orb_advertise_multi: failed to set queue size");

    /* the advertiser may perform an initial publish to initialise the object */
    if (data != nullptr) {
      if (!dev->Publish(*meta, data)) {
        ORB_ERR("orb_publish failed %s", meta->o_name);
        return nullptr;
      }
    }

    return dev;
  }

 private:  // class methods
  /**
   * Common implementation for orb_advertise and orb_subscribe.
   *
   * Handles creation of the object and the initial publication for
   * advertisers.
   */
  uORB::DeviceNode *node_open(const struct orb_metadata *meta,
                              unsigned int *instance);

 private:  // data members
  static Manager Instance;

  DeviceMaster _device_master{};

 private:  // class methods
  Manager() = default;
};
