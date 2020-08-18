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
 * @file Subscription.hpp
 *
 */

#pragma once

#include "uorb/device_node.h"
#include "uorb/uorb.h"

namespace uorb {

// Base subscription wrapper class
class Subscription {
 public:
  /**
   * Constructor
   *
   * @param meta The uORB metadata (usually from the ORB_ID() macro) for the
   * topic.
   * @param instance The instance for multi sub.
   */
  explicit Subscription(const orb_metadata &meta, uint8_t instance = 0)
      : meta_(meta), instance_(instance) {}

  ~Subscription() {
    if (node_) node_->ReduceSubscriberCount();
  }

  bool subscribe();

  bool advertised() {
    if (!node_) {
      // try to initialize
      subscribe();
    }

    if (node_) {
      return node_->is_advertised();
    }

    return false;
  }

  /**
   * Check if there is a new update.
   */
  virtual bool updated() {
    return advertised() &&
           (node_->published_message_count() != last_generation_);
  }

  /**
   * Update the struct
   * @param data The uORB message struct we are updating.
   */
  virtual bool update(void *dst) { return updated() && copy(dst); }

  /**
   * Copy the struct
   * @param data The uORB message struct we are updating.
   */
  virtual bool copy(void *dst) {
    return advertised() && node_->Copy(dst, last_generation_);
  }

 protected:
  unsigned last_generation_{0}; /**< last generation the subscriber has seen */

  const orb_metadata &meta_;
  const uint8_t instance_{0};
  DeviceNode *node_{nullptr};
};

// Subscription wrapper class with data
template <class T>
class SubscriptionData : public Subscription {
 public:
  /**
   * Constructor
   *
   * @param instance The instance for multi sub.
   */
  explicit SubscriptionData(uint8_t instance = 0)
      : Subscription(T::get_metadata(), instance) {}

  ~SubscriptionData() = default;

  // no copy, assignment, move, move assignment
  SubscriptionData(const SubscriptionData &) = delete;
  SubscriptionData &operator=(const SubscriptionData &) = delete;
  SubscriptionData(SubscriptionData &&) = delete;
  SubscriptionData &operator=(SubscriptionData &&) = delete;

  // update the embedded struct.
  bool update() { return Subscription::update(&data_); }

  const T &get() const { return data_; }

 private:
  T data_{};
};

}  // namespace uorb
