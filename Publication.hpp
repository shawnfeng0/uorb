/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
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
 * @file Publication.hpp
 *
 */

#pragma once

#include <base/orb_errno.h>
#include <base/orb_log.h>

#include "uORB.h"
#include "uORBDeviceNode.hpp"

namespace uORB {

class PublicationBase {
 public:
  bool advertised() const { return dev_ != nullptr; }

  bool unadvertise() {
    if (dev_) dev_->mark_as_unadvertised();
    return true;
  }

  orb_id_t get_topic() const { return meta_; }

 protected:
  explicit PublicationBase(const orb_metadata *meta) : meta_(meta) {}

  ~PublicationBase() {
    if (dev_ != nullptr) {
      // don't automatically unadvertise queued publications (eg
      // vehicle_command)
      if (dev_->get_queue_size() == 1) {
        unadvertise();
      }
    }
  }

  uORB::DeviceNode *dev_{nullptr};
  const orb_metadata *meta_;
};

/**
 * uORB publication wrapper class
 */
template <typename T, uint8_t ORB_QSIZE = 1>
class Publication : public PublicationBase {
 public:
  /**
   * Constructor
   *
   * @param meta The uORB metadata (usually from the ORB_ID() macro) for the
   * topic.
   */
  explicit Publication() : PublicationBase(T::get_metadata()) {}

  bool advertise() {
    if (!advertised()) {
      const orb_metadata &meta = *get_topic();
      DeviceMaster &master = DeviceMaster::get_instance();

      /* if we have an instance and are an advertiser, we will generate a new
       * node and set the instance, so we do not need to open here */
      /* open the path as either the advertiser or the subscriber */
      dev_ = master.GetDeviceNode(meta, 0);

      /* we may need to advertise the node... */
      if (!dev_) {
        /* it's OK if it already exists */
        dev_ = master.CreateAdvertiser(meta, nullptr, ORB_QSIZE);
      }

      if (!dev_) {
        ORB_ERR("%s advertise failed (%i)", meta.o_name, orb_errno);
        return false;
      }
    }

    return advertised();
  }

  /**
   * Publish the struct
   * @param data The uORB message struct we are updating.
   */
  bool publish(const T &data) {
    if (!advertised()) {
      advertise();
    }
    if (advertised()) {
      // don't automatically unadvertise queued publications (eg
      // vehicle_command)
      return dev_->Publish(*get_topic(), &data);
    }
    return false;
  }
};

/**
 * The publication class with data embedded.
 */
template <typename T>
class PublicationData : public Publication<T> {
 public:
  /**
   * Constructor
   *
   * @param meta The uORB metadata (usually from the ORB_ID() macro) for the
   * topic.
   */
  explicit PublicationData() : Publication<T>() {}

  T &get() { return _data; }
  void set(const T &data) { _data = data; }

  // Publishes the embedded struct.
  bool update() { return Publication<T>::publish(_data); }
  bool update(const T &data) {
    _data = data;
    return Publication<T>::publish(_data);
  }

 private:
  T _data{};
};

template <class T>
using PublicationQueued = Publication<T, T::ORB_QUEUE_LENGTH>;

}  // namespace uORB
