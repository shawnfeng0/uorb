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
#include "base/orb_posix.h"
#include "uORBDeviceNode.hpp"
#include "uORBUtils.hpp"

uORB::Manager uORB::Manager::Instance;

uORB::Manager::Manager() {
}

uORB::Manager::~Manager() = default;

uORB::DeviceMaster *uORB::Manager::get_device_master() {
  return &_device_master;
}

int uORB::Manager::orb_exists(const struct orb_metadata *meta, int instance) {
  int ret = ORB_ERROR;

  // instance valid range: [0, ORB_MULTI_MAX_INSTANCES)
  if ((instance < 0) || (instance > (ORB_MULTI_MAX_INSTANCES - 1))) {
    return ret;
  }

  uORB::DeviceNode *node = _device_master.getDeviceNode(meta, instance);

  if (node != nullptr) {
    if (node->is_advertised()) {
      return ORB_OK;
    }
  }

  return ret;
}

orb_advert_t uORB::Manager::orb_advertise_multi(const struct orb_metadata *meta,
                                                const void *data, int *instance,
                                                ORB_PRIO priority,
                                                unsigned int queue_size) {
  /* open the node as an advertiser */
  int fd = node_open(meta, true, instance, priority);

  if (fd == ORB_ERROR) {
    ORB_ERR("%s advertise failed (%i)", meta->o_name, orb_errno);
    return nullptr;
  }

  /* Set the queue size. This must be done before the first publication; thus it
   * fails if this is not the first advertiser.
   */
  int result = orb_ioctl(fd, ORBIOCSETQUEUESIZE, (unsigned long)queue_size);

  if (result < 0 && queue_size > 1) {
    ORB_WARN("orb_advertise_multi: failed to set queue size");
  }

  /* get the advertiser handle and close the node */
  orb_advert_t advertiser;

  result = orb_ioctl(fd, ORBIOCGADVERTISER, (unsigned long)&advertiser);
  orb_close(fd);

  if (result == ORB_ERROR) {
    ORB_WARN("orb_ioctl ORBIOCGADVERTISER failed. fd = %d", fd);
    return nullptr;
  }

  /* the advertiser may perform an initial publish to initialise the object */
  if (data != nullptr) {
    result = orb_publish(meta, advertiser, data);

    if (result == ORB_ERROR) {
      ORB_ERR("orb_publish failed %s", meta->o_name);
      return nullptr;
    }
  }

  return advertiser;
}

int uORB::Manager::orb_unadvertise(orb_advert_t handle) {
  return uORB::DeviceNode::unadvertise(handle);
}

int uORB::Manager::orb_subscribe(const struct orb_metadata *meta) {
  return node_open(meta, false);
}

int uORB::Manager::orb_subscribe_multi(const struct orb_metadata *meta,
                                       unsigned instance) {
  int inst = instance;
  return node_open(meta, false, &inst);
}

int uORB::Manager::orb_unsubscribe(int fd) { return orb_close(fd); }

int uORB::Manager::orb_publish(const struct orb_metadata *meta,
                               orb_advert_t handle, const void *data) {
  return uORB::DeviceNode::publish(meta, handle, data);
}

int uORB::Manager::orb_copy(const struct orb_metadata *meta, int handle,
                            void *buffer) {
  int ret;

  ret = orb_read(handle, buffer, meta->o_size);

  if (ret < 0) {
    return ORB_ERROR;
  }

  if (ret != (int)meta->o_size) {
    orb_errno = EIO;
    return ORB_ERROR;
  }

  return ORB_OK;
}

int uORB::Manager::orb_check(int handle, bool *updated) {
  /* Set to false here so that if `orb_ioctl` fails to false. */
  *updated = false;
  return orb_ioctl(handle, ORBIOCUPDATED, (unsigned long)(uintptr_t)updated);
}

int uORB::Manager::orb_priority(int handle, ORB_PRIO *priority) {
  return orb_ioctl(handle, ORBIOCGPRIORITY, (unsigned long)(uintptr_t)priority);
}

int uORB::Manager::orb_set_interval(int handle, unsigned interval) {
  return orb_ioctl(handle, ORBIOCSETINTERVAL, interval * 1000);
}

int uORB::Manager::orb_get_interval(int handle, unsigned *interval) {
  int ret = orb_ioctl(handle, ORBIOCGETINTERVAL, (unsigned long)interval);
  *interval /= 1000;
  return ret;
}

int uORB::Manager::node_open(const struct orb_metadata *meta, bool advertiser,
                             int *instance, ORB_PRIO priority) {
  char path[orb_maxpath];
  int fd = -1;
  int ret = ORB_ERROR;

  /*
   * If meta is null, the object was not defined, i.e. it is not
   * known to the system.  We can't advertise/subscribe such a thing.
   */
  if (nullptr == meta) {
    errno = ENOENT;
    return ORB_ERROR;
  }

  /* if we have an instance and are an advertiser, we will generate a new node
   * and set the instance, so we do not need to open here */
  if (!instance || !advertiser) {
    /*
     * Generate the path to the node and try to open it.
     */
    ret = uORB::Utils::node_mkpath(path, meta, instance);

    if (ret != ORB_OK) {
      errno = -ret;
      return ORB_ERROR;
    }

    /* open the path as either the advertiser or the subscriber */
    fd = orb_open(path, advertiser ? PX4_F_WRONLY : PX4_F_RDONLY);

  } else {
    *instance = 0;
  }

  /* we may need to advertise the node... */
  if (fd < 0) {
    ret = ORB_ERROR;

    if (get_device_master()) {
      ret = _device_master.advertise(meta, advertiser, instance, priority);
    }

    /* it's OK if it already exists */
    if ((ret != ORB_OK) && (EEXIST == errno)) {
      ret = ORB_OK;
    }

    if (ret == ORB_OK) {
      /* update the path, as it might have been updated during the node
       * advertise call */
      ret = uORB::Utils::node_mkpath(path, meta, instance);

      /* on success, try to open again */
      if (ret == ORB_OK) {
        fd = orb_open(path, (advertiser) ? PX4_F_WRONLY : PX4_F_RDONLY);

      } else {
        errno = -ret;
        return ORB_ERROR;
      }
    }
  }

  if (fd < 0) {
    errno = EIO;
    return ORB_ERROR;
  }

  /* everything has been OK, we can return the handle now */
  return fd;
}
