//
// Copyright (c) 2021 shawnfeng. All rights reserved.
//
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
