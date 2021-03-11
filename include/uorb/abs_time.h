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
 * @file drv_hrt.h
 *
 * High-resolution timer with callouts and timekeeping.
 */

#pragma once

#include <inttypes.h>
#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Absolute time, in microsecond units.
 *
 * Absolute time is measured from some arbitrary epoch shortly after
 * system startup.  It should never wrap or go backwards.
 */
typedef uint64_t orb_abstime_us;

/**
 * Get absolute time in [us] (does not wrap).
 */
static inline orb_abstime_us orb_absolute_time_us() {
  struct timespec ts = {};
  orb_abstime_us result;

  clock_gettime(CLOCK_MONOTONIC, &ts);

  result = (orb_abstime_us)(ts.tv_sec) * 1000000;
  result += ts.tv_nsec / 1000;

  return result;
}

/**
 * Compute the delta between a timestamp taken in the past
 * and now.
 */
static inline orb_abstime_us orb_elapsed_time_us(const orb_abstime_us then) {
  return orb_absolute_time_us() - then;
}

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

namespace time_literals {

// User-defined integer literals for different time units.
// The base unit is orb_abstime_us in microseconds

constexpr orb_abstime_us operator"" _s(unsigned long long time) {
  return orb_abstime_us(time * 1000000ULL);
}

constexpr orb_abstime_us operator"" _ms(unsigned long long time) {
  return orb_abstime_us(time * 1000ULL);
}

constexpr orb_abstime_us operator"" _us(unsigned long long time) {
  return orb_abstime_us(time);
}

} /* namespace time_literals */

#endif /* __cplusplus */
