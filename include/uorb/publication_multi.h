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

#include "uorb/uorb.h"

namespace uorb {

/**
 * uORB publication wrapper class
 */
template <const orb_metadata &meta, uint8_t queue_size = 1>
class PublicationMulti {
  using Type = typename msg::TypeMap<meta>::type;

 public:
  ~PublicationMulti() { handle_ &&orb_destroy_publication(&handle_); }

  /**
   * Publish the struct
   * @param data The uORB message struct we are updating.
   */
  bool Publish(const Type &data) {
    if (!handle_) {
      unsigned instance;
      handle_ = orb_create_publication_multi(&meta, &instance, queue_size);
    }

    return handle_ && orb_publish(handle_, &data);
  }

 private:
  orb_publication_t *handle_{nullptr};
};

/**
 * The publication class with data embedded.
 */
template <const orb_metadata &T, uint8_t queue_size = 1>
class PublicationMultiData : public PublicationMulti<T, queue_size> {
  using Type = typename msg::TypeMap<T>::type;

 public:
  PublicationMultiData() = default;

  Type &get() { return data_; }
  auto set(const Type &data) -> decltype(*this) {
    data_ = data;
    return *this;
  }

  // Publishes the embedded struct.
  bool Publish() { return PublicationMulti<T, queue_size>::Publish(data_); }

 private:
  Type data_{};
};

}  // namespace uorb
