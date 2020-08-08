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
#include "uORBManager.hpp"

#include "base/orb_defines.h"
#include "base/orb_errno.h"
#include "base/orb_log.h"
#include "uORBDeviceNode.hpp"
#include "uORBUtils.hpp"

uORB::Manager uORB::Manager::Instance;

uORB::DeviceMaster &uORB::Manager::get_device_master() {
  return _device_master;
}

orb_advert_t uORB::Manager::orb_advertise_multi(const struct orb_metadata *meta,
                                                const void *data, int *instance,
                                                unsigned int queue_size) {
  /* open the node as an advertiser */
  uORB::DeviceNode *dev = node_open(meta, instance);

  if (!dev) {
    ORB_ERR("%s advertise failed (%i)", meta->o_name, orb_errno);
    return nullptr;
  }

  /* Set the queue size. This must be done before the first publication; thus it
   * fails if this is not the first advertiser.
   */
  int result = ORB_OK;

  if (!dev->SetQueueSize(queue_size))
    ORB_WARN("orb_advertise_multi: failed to set queue size");

  /* the advertiser may perform an initial publish to initialise the object */
  if (data != nullptr) {
    result = orb_publish(meta, dev, data);

    if (result == ORB_ERROR) {
      ORB_ERR("orb_publish failed %s", meta->o_name);
      return nullptr;
    }
  }

  return dev;
}

int uORB::Manager::orb_publish(const struct orb_metadata *meta,
                               orb_advert_t handle, const void *data) {
  return uORB::DeviceNode::publish(meta, handle, data);
}

uORB::DeviceNode *uORB::Manager::node_open(const struct orb_metadata *meta,
                                           int *instance) {
  char path[orb_maxpath];
  uORB::DeviceNode *dev = nullptr;

  /*
   * If meta is null, the object was not defined, i.e. it is not
   * known to the system.  We can't advertise/subscribe such a thing.
   */
  if (nullptr == meta) {
    orb_errno = ENOENT;
    return nullptr;
  }

  /* if we have an instance and are an advertiser, we will generate a new node
   * and set the instance, so we do not need to open here */
  if (!instance) {
    /*
     * Generate the path to the node and try to open it.
     */
    int ret = uORB::Utils::node_mkpath(path, meta);

    if (ret != ORB_OK) {
      orb_errno = -ret;
      return dev;
    }

    /* open the path as either the advertiser or the subscriber */
    dev = _device_master.getDeviceNode(meta, 0);

  } else {
    *instance = 0;
  }

  /* we may need to advertise the node... */
  if (!dev) {
    int ret = _device_master.advertise(meta, instance);

    /* it's OK if it already exists */
    if ((ret != ORB_OK) && (EEXIST == orb_errno)) {
      ret = ORB_OK;
    }

    if (ret == ORB_OK) {
      /* update the path, as it might have been updated during the node
       * advertise call */
      ret = uORB::Utils::node_mkpath(path, meta, instance);

      /* on success, try to open again */
      if (ret == ORB_OK) {
        if (instance)
          dev = _device_master.getDeviceNode(meta, *instance);
        else
          dev = _device_master.getDeviceNode(meta, 0);
      } else {
        orb_errno = -ret;
        return nullptr;
      }
    }
  }

  if (!dev) {
    orb_errno = EIO;
  } else {
    dev->open();
  }

  return dev;
}
