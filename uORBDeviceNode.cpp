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

#include "uORBDeviceNode.hpp"

#include <base/orb_defines.h>

#include "base/orb_errno.h"
#include "base/orb_log.h"
#include "uORBUtils.hpp"

uORB::DeviceNode::DeviceNode(const struct orb_metadata *meta, uint8_t instance,
                             uint8_t queue_size)
    : _meta(meta), _instance(instance), _queue_size(queue_size) {}

uORB::DeviceNode::~DeviceNode() { delete[] _data; }

int uORB::DeviceNode::open() {
  // Automatic mutex guarding
  uORB::base::MutexGuard lg(_lock);
  mark_as_advertised();

  /* now complete the open */
  return ORB_OK;
}

bool uORB::DeviceNode::copy_locked(void *dst, unsigned &generation) {
  bool updated = false;

  if ((dst != nullptr) && (_data != nullptr)) {
    unsigned current_generation = _generation.load();

    if (current_generation > generation + _queue_size) {
      // Reader is too far behind: some messages are lost
      _lost_messages += current_generation - (generation + _queue_size);
      generation = current_generation - _queue_size;
    }

    if ((current_generation == generation) && (generation > 0)) {
      /* The subscriber already read the latest message, but nothing new was
       * published yet. Return the previous message
       */
      --generation;
    }

    memcpy(dst, _data + (_meta->o_size * (generation % _queue_size)),
           _meta->o_size);

    if (generation < current_generation) {
      ++generation;
    }

    updated = true;
  }

  return updated;
}

bool uORB::DeviceNode::copy(void *dst, unsigned &generation) {
  // Automatic mutex guarding
  uORB::base::MutexGuard lg(_lock);

  bool updated = copy_locked(dst, generation);

  return updated;
}

ssize_t uORB::DeviceNode::write(const char *buffer, size_t buflen) {
  /*
   * Writes are legal from interrupt context as long as the
   * object has already been initialised from thread context.
   *
   * Writes outside interrupt context will allocate the object
   * if it has not yet been allocated.
   *
   * Note that filp will usually be NULL.
   */
  if (nullptr == _data) {
    {
      // Automatic mutex guarding
      uORB::base::MutexGuard lg(_lock);

      /* re-check size */
      if (nullptr == _data) {
        _data = new uint8_t[_meta->o_size * _queue_size];
      }
    }

    /* failed or could not allocate */
    if (nullptr == _data) {
      return -ENOMEM;
    }
  }

  /* If write size does not match, that is an error */
  if (_meta->o_size != buflen) {
    return -EIO;
  }

  /* Perform an atomic copy. */
  {
    // Automatic mutex guarding
    uORB::base::MutexGuard lg(_lock);

    /* wrap-around happens after ~49 days, assuming a publisher rate of 1 kHz */
    unsigned generation = _generation.fetch_add(1);

    memcpy(_data + (_meta->o_size * (generation % _queue_size)), buffer,
           _meta->o_size);
  }
  return _meta->o_size;
}

ssize_t uORB::DeviceNode::publish(const orb_metadata *meta, orb_advert_t handle,
                                  const void *data) {
  auto *devnode = (uORB::DeviceNode *)handle;
  int ret;

  /* check if the device handle is initialized and data is valid */
  if ((devnode == nullptr) || (meta == nullptr) || (data == nullptr)) {
    orb_errno = EFAULT;
    return ORB_ERROR;
  }

  /* check if the orb meta data matches the publication */
  if (devnode->_meta != meta) {
    orb_errno = EINVAL;
    return ORB_ERROR;
  }

  /* call the devnode write method with no file pointer */
  ret = devnode->write((const char *)data, meta->o_size);

  if (ret < 0) {
    orb_errno = -ret;
    return ORB_ERROR;
  }

  if (ret != (int)meta->o_size) {
    orb_errno = EIO;
    return ORB_ERROR;
  }

  return ORB_OK;
}

int uORB::DeviceNode::unadvertise(orb_advert_t handle) {
  if (handle == nullptr) {
    return -EINVAL;
  }

  auto *devnode = (uORB::DeviceNode *)handle;

  /*
   * We are cheating a bit here. First, with the current implementation, we can
   * only have multiple publishers for instance 0. In this case the caller will
   * have instance=nullptr and _published has no effect at all. Thus no
   * unadvertise is necessary. In case of multiple instances, we have at most 1
   * publisher per instance and we can signal an instance as 'free' by setting
   * _published to false. We never really free the DeviceNode, for this we would
   * need reference counting of subscribers and publishers. But we also do not
   * have a leak since future publishers reuse the same DeviceNode object.
   */
  devnode->_advertised = false;

  return ORB_OK;
}

void uORB::DeviceNode::add_internal_subscriber() {
  // Automatic mutex guarding
  uORB::base::MutexGuard lg(_lock);
  _subscriber_count++;
}

void uORB::DeviceNode::remove_internal_subscriber() {
  // Automatic mutex guarding
  uORB::base::MutexGuard lg(_lock);
  _subscriber_count--;
}

bool uORB::DeviceNode::update_queue_size_locked(unsigned int queue_size) {
  if (_queue_size == queue_size) {
    return true;
  }

  // queue size is limited to 255 for the single reason that we use uint8 to
  // store it
  if (_data || _queue_size > queue_size || queue_size > 255) {
    return false;
  }

  _queue_size = queue_size;
  return true;
}
