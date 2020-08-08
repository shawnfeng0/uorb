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

#include "base/orb_errno.h"
#include "base/orb_log.h"
#include "uORBDeviceNode.hpp"

uORB::Manager uORB::Manager::Instance;

uORB::DeviceMaster &uORB::Manager::get_device_master() {
  return _device_master;
}

uORB::DeviceNode *uORB::Manager::node_open(const struct orb_metadata *meta,
                                           unsigned int *instance) {
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
    /* open the path as either the advertiser or the subscriber */
    dev = _device_master.getDeviceNode(meta, 0);

  } else {
    *instance = 0;
  }

  /* we may need to advertise the node... */
  if (!dev) {
    bool ret = _device_master.advertise(meta, instance);

    /* it's OK if it already exists */
    if (ret || EEXIST == orb_errno) {
      /* on success, try to open again */
      if (instance)
        dev = _device_master.getDeviceNode(meta, *instance);
      else
        dev = _device_master.getDeviceNode(meta, 0);
    }
  }

  if (!dev) {
    orb_errno = EIO;
  } else {
    dev->mark_as_advertised();
  }

  return dev;
}
