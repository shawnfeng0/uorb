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

uorb::DeviceNode::DeviceNode(const struct orb_metadata &meta, uint8_t instance,
                             uint16_t queue_size)
    : meta_(meta),
      instance_(instance),
      queue_size_(GenerateQueueSize(queue_size)) {}

uorb::DeviceNode::~DeviceNode() { delete[] data_; }

template <typename T>
static inline bool is_in_range(const T &left, const T &value, const T &right) {
  if (right > left) {
    return left <= value && value <= right;
  } else {  // Maybe the data overflowed and a wraparound occurred
    return left <= value || value <= right;
  }
}

bool uorb::DeviceNode::Copy(void *dst, unsigned &sub_generation) {
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

bool uorb::DeviceNode::Publish(const orb_metadata &meta, const void *data) {
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

void uorb::DeviceNode::IncreaseSubscriberCount() {
  base::MutexGuard lg(lock_);
  subscriber_count_++;
}

void uorb::DeviceNode::ReduceSubscriberCount() {
  base::MutexGuard lg(lock_);
  subscriber_count_--;
}

bool uorb::DeviceNode::IsSameWith(const orb_metadata &meta,
                                  uint8_t instance) const {
  return (strcmp(meta_.o_name, meta.o_name) == 0) && (instance_ == instance);
}
