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
 * @file subscription.h
 *
 */

#pragma once

#include "uorb/uorb.h"

namespace uorb {

template <const orb_metadata &meta>
class Subscription {
  using Type = typename msg::TypeMap<meta>::type;

 public:
  /**
   * Constructor
   *
   * @param meta The uORB metadata (usually from the ORB_ID() macro) for the
   * topic.
   * @param instance The instance for multi sub.
   */
  explicit Subscription(uint8_t instance = 0) : instance_(instance) {}

  // no copy, assignment, move, move assignment
  Subscription(const Subscription &) = delete;
  Subscription &operator=(const Subscription &) = delete;
  Subscription(Subscription &&) = delete;
  Subscription &operator=(Subscription &&) = delete;

  ~Subscription() { handle_ &&orb_destroy_subscription(&handle_); }

  bool Subscribed() {
    // check if already subscribed
    if (handle_) {
      return true;
    }
    return handle_ = orb_create_subscription_multi(&meta, instance_);
  }

  /**
   * Check if there is a new update.
   */
  virtual bool Updated() { return Subscribed() && orb_check_update(handle_); }

  /**
   * Update the struct
   * @param data The uORB message struct we are updating.
   */
  virtual bool Update(Type *dst) { return Updated() && Copy(dst); }

  /**
   * Copy the struct
   * @param data The uORB message struct we are updating.
   */
  virtual bool Copy(Type *dst) {
    return Subscribed() && orb_copy(handle_, dst);
  }

 protected:
  const uint8_t instance_{0};
  orb_subscription_t *handle_{nullptr};
};

// Subscription wrapper class with data
template <const orb_metadata &T>
class SubscriptionData : public Subscription<T> {
  using Type = typename msg::TypeMap<T>::type;

 public:
  explicit SubscriptionData(uint8_t instance = 0) : Subscription<T>(instance) {}

  ~SubscriptionData() = default;

  // no copy, assignment, move, move assignment
  SubscriptionData(const SubscriptionData &) = delete;
  SubscriptionData &operator=(const SubscriptionData &) = delete;
  SubscriptionData(SubscriptionData &&) = delete;
  SubscriptionData &operator=(SubscriptionData &&) = delete;

  // update the embedded struct.
  bool Update() { return Subscription<T>::Update(&data_); }

  const Type &get() const { return data_; }

 private:
  Type data_{};
};

}  // namespace uorb
