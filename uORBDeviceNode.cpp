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

#include "base/orb_errno.h"
#include "base/orb_log.h"

uORB::DeviceNode::DeviceNode(const struct orb_metadata &meta, uint8_t instance,
                             uint16_t queue_size)
    : meta_(meta),
      instance_(instance),
      queue_size_(GenerateQueueSize(queue_size)) {}

uORB::DeviceNode::~DeviceNode() { delete[] data_; }

template <typename T>
static inline bool is_in_range(const T &left, const T &value, const T &right) {
  if (right > left) {
    return left <= value && value <= right;
  } else {  // Maybe the data overflowed and a wraparound occurred
    return left <= value || value <= right;
  }
}

bool uORB::DeviceNode::Copy(void *dst, unsigned &sub_generation) {
  base::MutexGuard lg(lock_);

  if ((dst == nullptr) || (data_ == nullptr) || (generation_ == 0)) {
    return false;
  }

  if (generation_ == sub_generation) {
    /* The subscriber already read the latest message, but nothing new was
     * published yet. Return the previous message
     */
    --sub_generation;
  }

  // The queue is full and any value of "generation" is valid. If it is not in
  // the queue, it can only be considered as data loss.
  if (queue_is_full_) {
    const unsigned queue_start = generation_ - queue_size_;
    // If queue_size is 3 and cur_generation is 10, then 7, 8, 9 are in the
    // range, and others are not.
    if (!is_in_range(queue_start, sub_generation, generation_ - 1)) {
      // Reader is too far behind: some messages are lost
      lost_messages_ += queue_start - sub_generation;
      sub_generation = queue_start;
    }
  } else {
    if (sub_generation > generation_) {
      // Insufficient data generation, invalid input
      sub_generation = 0;
      ORB_WARN("generation is invalid: %u", sub_generation);
    }
  }

  memcpy(dst, data_ + (meta_.o_size * (sub_generation % queue_size_)),
         meta_.o_size);

  ++sub_generation;

  return true;
}

bool uORB::DeviceNode::Publish(const orb_metadata &meta, const void *data) {
  /* check if the device handle is initialized and data is valid */
  if (data == nullptr) {
    orb_errno = EFAULT;
    return false;
  }

  /* check if the orb meta data matches the publication */
  if (&meta_ != &meta) {
    orb_errno = EINVAL;
    return false;
  }

  base::MutexGuard lg(lock_);

  if (nullptr == data_) {
    data_ = new uint8_t[meta_.o_size * queue_size_];

    /* failed or could not allocate */
    if (nullptr == data_) {
      orb_errno = ENOMEM;
      return false;
    }
  }

  /* wrap-around happens after ~49 days, assuming a publisher rate of 1 kHz */
  memcpy(data_ + (meta_.o_size * (generation_ % queue_size_)),
         (const char *)data, meta_.o_size);

  if (!queue_is_full_ && (generation_ >= queue_size_ - 1)) {
    queue_is_full_ = true;
  }

  generation_++;

  return true;
}

bool uORB::DeviceNode::Unadvertise() {
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
  advertised_ = false;

  return true;
}

bool uORB::DeviceNode::SetQueueSize(uint16_t queue_size) {
  uORB::base::MutexGuard lg(lock_);
  if (queue_size_ == queue_size) {
    return true;
  }

  // queue size is limited to 255 for the single reason that we use uint8 to
  // store it
  if (data_ || queue_size_ > queue_size) {
    return false;
  }

  queue_size_ = GenerateQueueSize(queue_size);
  return true;
}

void uORB::DeviceNode::IncreaseSubscriberCount() {
  base::MutexGuard lg(lock_);
  subscriber_count_++;
}

void uORB::DeviceNode::ReduceSubscriberCount() {
  base::MutexGuard lg(lock_);
  subscriber_count_--;
}
