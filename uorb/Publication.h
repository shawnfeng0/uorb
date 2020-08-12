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

/**
 * @file Publication.hpp
 *
 */

#pragma once

#include "uorb/DeviceMaster.hpp"
#include "uorb/DeviceNode.hpp"
#include "uorb/base/errno.h"
#include "uorb/uORB.h"

namespace uorb {

class PublicationBase {
 protected:
  explicit PublicationBase(const orb_metadata &meta) : meta_(meta) {}

  ~PublicationBase() {
    if (dev_) dev_->mark_as_unadvertised();
  }

  uorb::DeviceNode *dev_{nullptr};
  const orb_metadata &meta_;
};

/**
 * uORB publication wrapper class
 */
template <typename T, uint8_t ORB_QSIZE = 1>
class Publication : public PublicationBase {
 public:
  Publication() : PublicationBase(T::get_metadata()) {}

  /**
   * Publish the struct
   * @param data The uORB message struct we are updating.
   */
  bool publish(const T &data) {
    if (!dev_) advertise();

    if (dev_) {
      return dev_->Publish(meta_, &data);
    }

    return false;
  }

 private:
  bool advertise() {
    auto &device_master = DeviceMaster::get_instance();
    dev_ = device_master.CreateAdvertiser(meta_, nullptr, ORB_QSIZE);

    return dev_ != nullptr;
  }
};

/**
 * The publication class with data embedded.
 */
template <typename T, uint8_t queue_size = 1>
class PublicationData : public Publication<T, queue_size> {
 public:
  PublicationData() = default;

  T &get() { return data_; }
  auto set(const T &data) -> decltype(*this) {
    data_ = data;
    return *this;
  }

  // Publishes the embedded struct.
  bool publish() { return Publication<T, queue_size>::publish(data_); }

 private:
  T data_{};
};

template <class T>
using PublicationQueued = Publication<T, T::ORB_QUEUE_LENGTH>;

}  // namespace uorb
