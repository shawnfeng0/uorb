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

#include "device_node.h"

#include <cerrno>
#include <cstring>

static inline bool IsInRange(unsigned left, unsigned value, unsigned right) {
  if (right > left) {
    return (left <= value) && (value <= right);
  } else {  // Maybe the data overflowed and a wraparound occurred
    return (left <= value) || (value <= right);
  }
}

static inline uint16_t RoundPowOfTwo(uint16_t n) {
  if (n == 0) {
    return 1;
  }

  // Avoid is already a power of 2
  uint32_t value = n - 1;

  // Fill 1
  value |= value >> 1U;
  value |= value >> 2U;
  value |= value >> 4U;
  value |= value >> 8U;

  // Unable to round-up, take the value of round-down
  if (value == UINT16_MAX) {
    value >>= 1U;
  }

  return value + 1;
}

uorb::DeviceNode::DeviceNode(const struct orb_metadata &meta, uint8_t instance,
                             unsigned int queue_size)
    : meta_(meta),
      instance_(instance),
      queue_size_(
          RoundPowOfTwo(queue_size > UINT16_MAX ? UINT16_MAX : queue_size)) {}

uorb::DeviceNode::~DeviceNode() { delete[] data_; }

bool uorb::DeviceNode::Copy(void *dst, unsigned &sub_generation) const {
  if ((dst == nullptr) || (data_ == nullptr)) {
    return false;
  }

  base::ReaderLockGuard lg(lock_);

  /* The subscriber already read the latest message, but nothing new was
   * published yet. Return the previous message */
  if (generation_ == sub_generation) {
    --sub_generation;
  }

  // Before the queue is filled, if the incoming sub_generation points to
  // unpublished data, invalid data will be obtained. Such incorrect usage
  // should not be handled.
  const unsigned queue_start = generation_ - queue_size_;

  // If queue_size is 3 and cur_generation is 10, then 7, 8, 9 are in the
  // range, and others are not.
  if (!IsInRange(queue_start, sub_generation, generation_ - 1)) {
    // Reader is too far behind: some messages are lost
    sub_generation = queue_start;
  }

  memcpy(dst, data_ + (meta_.o_size * (sub_generation % queue_size_)),
         meta_.o_size);

  ++sub_generation;

  return true;
}

unsigned uorb::DeviceNode::updates_available(unsigned generation) const {
  return generation_ - generation;
}

bool uorb::DeviceNode::Publish(const void *data) {
  if (data == nullptr) {
    errno = EFAULT;
    return false;
  }

  base::WriterLockGuard lg(lock_);

  if (nullptr == data_) {
    data_ = new uint8_t[meta_.o_size * queue_size_];

    /* failed or could not allocate */
    if (nullptr == data_) {
      errno = ENOMEM;
      return false;
    }
  }

  memcpy(data_ + (meta_.o_size * (generation_ % queue_size_)),
         (const char *)data, meta_.o_size);

  generation_++;

  for (auto callback : callbacks_) {
    (*callback)();
  }

  return true;
}

void uorb::DeviceNode::add_subscriber() {
  base::WriterLockGuard lg(lock_);
  subscriber_count_++;
}

void uorb::DeviceNode::remove_subscriber() {
  base::WriterLockGuard lg(lock_);
  subscriber_count_--;
}

bool uorb::DeviceNode::RegisterCallback(Callback *callback) {
  if (!callback) {
    errno = EINVAL;
    return false;
  }

  base::WriterLockGuard lg(lock_);

  // prevent duplicate registrations
  for (auto existing_callback : callbacks_) {
    if (callback == existing_callback) {
      return true;
    }
  }

  return callbacks_.Add(callback);
}

void uorb::DeviceNode::UnregisterCallback(Callback *callback) {
  base::WriterLockGuard lg(lock_);
  callbacks_.Remove(callback);
}

bool uorb::DeviceNode::set_queue_size(unsigned int queue_size) {
  base::WriterLockGuard lg(lock_);

  if (data_ || queue_size_ > queue_size) {
    return false;
  }

  queue_size_ =
      RoundPowOfTwo(queue_size > UINT16_MAX ? UINT16_MAX : queue_size);
  return true;
}

unsigned uorb::DeviceNode::initial_generation() const {
  base::WriterLockGuard lg(lock_);

  // If there any previous publications allow the subscriber to read them
  return generation_ - (data_ && publisher_count() ? 1 : 0);
}

void uorb::DeviceNode::remove_publisher() {
  base::WriterLockGuard lg(lock_);
  publisher_count_--;
}

void uorb::DeviceNode::add_publisher() {
  base::WriterLockGuard lg(lock_);
  publisher_count_++;
}
