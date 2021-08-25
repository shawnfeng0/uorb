/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * @file SubscriptionInterval.hpp
 *
 */

#pragma once

#include "uorb/abs_time.h"
#include "uorb/subscription.h"
#include "uorb/uorb.h"

namespace uorb {

template <const orb_metadata &T>
class SubscriptionInterval : public Subscription<T> {
  using Type = typename msg::TypeMap<T>::type;

 private:
  template <typename Tp>
  constexpr Tp constrain(Tp val, Tp min_val, Tp max_val) const {
    return (val < min_val) ? min_val : ((val > max_val) ? max_val : val);
  }

 public:
  /**
   * Constructor
   *
   * @param meta The uORB metadata (usually from the ORB_ID() macro) for the
   * topic.
   * @param interval The requested maximum update interval in microseconds.
   * @param instance The instance for multi sub.
   */
  explicit SubscriptionInterval(uint32_t interval_us = 0,
                                uint8_t instance = 0) noexcept
      : Subscription<T>(instance), interval_us_(interval_us) {}

  ~SubscriptionInterval() = default;

  /**
   * Check if there is a new update.
   * */
  bool Updated() override {
    return Subscription<T>::Updated() &&
           (orb_elapsed_time_us(last_update_) >= interval_us_);
  }

  /**
   * Copy the struct if updated.
   * @param dst The destination pointer where the struct will be copied.
   * @return true only if topic was updated and copied successfully.
   */
  bool Update(Type *dst) override {
    if (Updated()) {
      return Copy(dst);
    }

    return false;
  }

  /**
   * Copy the struct
   * @param dst The destination pointer where the struct will be copied.
   * @return true only if topic was copied successfully.
   */
  bool Copy(Type *dst) override {
    if (Subscription<T>::Copy(dst)) {
      const orb_abstime_us now = orb_absolute_time_us();
      // shift last update time forward, but don't let it get further behind
      // than the interval
      last_update_ =
          constrain(last_update_ + interval_us_, now - interval_us_, now);
      return true;
    }

    return false;
  }

  uint32_t interval_us() const { return interval_us_; }
  uint32_t interval_ms() const { return interval_us_ / 1000; }
  void set_interval_us(uint32_t interval) { interval_us_ = interval; }
  void set_interval_ms(uint32_t interval) { interval_us_ = interval * 1000; }

 protected:
  orb_abstime_us last_update_{0};  // last update in microseconds
  uint32_t interval_us_{0};        // maximum update interval in microseconds
};

}  // namespace uorb
