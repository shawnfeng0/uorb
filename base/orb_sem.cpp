/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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
 * @file orb_sem.cpp
 *
 * PX4 Middleware Wrapper Linux Implementation
 */

#include <pthread.h>

#include "orb_log.h"
#include "orb_posix.h"
#include <errno.h>

#if !defined(ETIMEDOUT)
#define	ETIMEDOUT	110	/* Connection timed out */
#endif

int orb_sem_init(orb_sem_t *s, int pshared, unsigned value)
{
	// We do not used the process shared arg
	(void)pshared;
	s->value = value;
	pthread_cond_init(&(s->wait), nullptr);
#if !defined(__PX4_DARWIN)
	// We want to use CLOCK_MONOTONIC if possible but we can't on macOS
	// because it's not available.
	pthread_condattr_t attr;
	pthread_condattr_init(&attr);
	pthread_condattr_setclock(&attr, CLOCK_MONOTONIC);
	pthread_cond_init(&(s->wait), &attr);
#endif
	return 0;
}

int orb_sem_setprotocol(orb_sem_t *s, int protocol)
{
	return 0;
}

int orb_sem_wait(orb_sem_t *s)
{
  uORB::MutexGuard guard(s->lock);
	int ret;

	s->value--;

	if (s->value < 0) {
		ret = pthread_cond_wait(&(s->wait), s->lock.native_handle());

	} else {
		ret = 0;
	}

	if (ret) {
          ORB_WARN("orb_sem_wait failure");
	}

	return (ret);
}

int orb_sem_trywait(orb_sem_t *s)
{
  uORB::MutexGuard guard(s->lock);

  int ret = 0;

  if (s->value <= 0) {
    errno = EAGAIN;
    ret = -1;

  } else {
    s->value--;
  }

  return (ret);
}

int orb_sem_timedwait(orb_sem_t *s, const struct timespec *abstime)
{
  uORB::MutexGuard guard(s->lock);
  int ret = 0;

  s->value--;
  errno = 0;

  if (s->value < 0) {
    ret = pthread_cond_timedwait(&(s->wait), s->lock.native_handle(), abstime);

  } else {
    ret = 0;
  }

  errno = ret;

  if (ret != 0 && ret != ETIMEDOUT) {
#if defined(__unix__)
    const unsigned NAMELEN = 32;
    char thread_name[NAMELEN] = {};
    (void) pthread_getname_np(pthread_self(), thread_name, NAMELEN);
#else
    char thread_name[] = {"\"unknow name\""};
#endif
    ORB_WARN("%s: orb_sem_timedwait failure: ret: %d", thread_name, ret);
  }

  return ret ? -1 : 0;
}

int orb_sem_post(orb_sem_t *s)
{
  uORB::MutexGuard guard(s->lock);
	int ret = 0;

	s->value++;

	if (s->value <= 0) {
		ret = pthread_cond_signal(&(s->wait));

	} else {
		ret = 0;
	}

	if (ret) {
          ORB_WARN("orb_sem_post failure");
	}

	// return the cond signal failure if present,
	// else return the mutex status
	return ret;
}

int orb_sem_getvalue(orb_sem_t *s, int *sval)
{
  uORB::MutexGuard guard(s->lock);

  *sval = s->value;

  return 0;
}

int orb_sem_destroy(orb_sem_t *s)
{
  s->lock.lock();
  pthread_cond_destroy(&(s->wait));
  s->lock.unlock();

  return 0;
}
