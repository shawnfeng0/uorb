/****************************************************************************
 *
 *   Copyright (c) 2012 - 2018 PX4 Development Team. All rights reserved.
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
 * @file drv_hrt.cpp
 *
 * High-resolution timer with callouts and timekeeping.
 */

#include <px4_time.h>
#include <drivers/drv_hrt.h>
#include <time.h>

#if defined(ENABLE_LOCKSTEP_SCHEDULER)
#include <lockstep_scheduler/lockstep_scheduler.h>
#endif

#ifndef __PX4_QURT
static hrt_abstime px4_timestart_monotonic = 0;
#else
static int32_t dsp_offset = 0;
#endif

#if defined(ENABLE_LOCKSTEP_SCHEDULER)
static LockstepScheduler *lockstep_scheduler = new LockstepScheduler();
#endif

#if defined(__PX4_APPLE_LEGACY)
#include <sys/time.h>

int px4_clock_gettime(clockid_t clk_id, struct timespec *tp)
{
	struct timeval now;
	int rv = gettimeofday(&now, nullptr);

	if (rv) {
		return rv;
	}

	tp->tv_sec = now.tv_sec;
	tp->tv_nsec = now.tv_usec * 1000;

	return 0;
}

int px4_clock_settime(clockid_t clk_id, struct timespec *tp)
{
	/* do nothing right now */
	return 0;
}

#elif defined(__PX4_QURT)

#include "dspal_time.h"

int px4_clock_settime(clockid_t clk_id, struct timespec *tp)
{
	/* do nothing right now */
	return 0;
}

int px4_clock_gettime(clockid_t clk_id, struct timespec *tp)
{
	return clock_gettime(clk_id, tp);
}

#endif

#ifndef __PX4_QURT
#endif

/*
 * Get absolute time.
 */
hrt_abstime hrt_absolute_time()
{
#if defined(ENABLE_LOCKSTEP_SCHEDULER)
	// optimized case (avoid ts_to_abstime) if lockstep scheduler is used
	const uint64_t abstime = lockstep_scheduler->get_absolute_time();
	return abstime - px4_timestart_monotonic;
#else // defined(ENABLE_LOCKSTEP_SCHEDULER)
	struct timespec ts;
	px4_clock_gettime(CLOCK_MONOTONIC, &ts);
#ifdef __PX4_QURT
	return ts_to_abstime(&ts) + dsp_offset;
#else
	return ts_to_abstime(&ts);
#endif
#endif // defined(ENABLE_LOCKSTEP_SCHEDULER)
}

#ifdef __PX4_QURT
int hrt_set_absolute_time_offset(int32_t time_diff_us)
{
	dsp_offset = time_diff_us;
	return 0;
}
#endif

/*
 * Convert a timespec to absolute time.
 */
hrt_abstime ts_to_abstime(const struct timespec *ts)
{
	hrt_abstime	result;

	result = (hrt_abstime)(ts->tv_sec) * 1000000;
	result += ts->tv_nsec / 1000;

	return result;
}

/*
 * Compute the delta between a timestamp taken in the past
 * and now.
 *
 * This function is safe to use even if the timestamp is updated
 * by an interrupt during execution.
 */
hrt_abstime hrt_elapsed_time_atomic(const volatile hrt_abstime *then)
{
	// This is not atomic as the value on the application layer of POSIX is limited.
	hrt_abstime delta = hrt_absolute_time() - *then;
	return delta;
}

#if !defined(__PX4_QURT)
int px4_clock_gettime(clockid_t clk_id, struct timespec *tp)
{
	return system_clock_gettime(clk_id, tp);
}
#endif // !defined(__PX4_QURT)
