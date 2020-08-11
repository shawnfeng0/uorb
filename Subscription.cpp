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
 * @file Subscription.cpp
 *
 */

#include "Subscription.hpp"
#include "uORBDeviceMaster.hpp"

namespace uorb {

bool Subscription::subscribe() {
  // check if already subscribed
  if (_node != nullptr) {
    return true;
  }

  DeviceMaster &device_master = uorb::DeviceMaster::get_instance();

  _node = device_master.GetDeviceNode(*get_topic(), _instance);

  if (_node == nullptr) {
    return false;
  }

  _node->IncreaseSubscriberCount();

  // If there were any previous publications, allow the subscriber to read
  // them
  const unsigned curr_gen = _node->published_message_count();
  const uint8_t q_size = _node->get_queue_size();

  if (q_size < curr_gen) {
    _last_generation = curr_gen - q_size;

  } else {
    _last_generation = 0;
  }

  return true;
}

void Subscription::unsubscribe() {
  if (_node != nullptr) {
    _node->ReduceSubscriberCount();
  }

  _node = nullptr;
  _last_generation = 0;
}

}  // namespace uorb
