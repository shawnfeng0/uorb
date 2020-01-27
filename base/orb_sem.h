/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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
 * @file orb_sem.h
 *
 * Synchronization primitive: Semaphore
 */

#pragma once

#include <pthread.h>

#include "orb_condition_variable.hpp"
#include "orb_mutex.hpp"
#include "visibility.h"

#define SEM_PRIO_NONE 0
#define SEM_PRIO_INHERIT 1
#define SEM_PRIO_PROTECT 2

__BEGIN_DECLS

typedef struct {
  uORB::Mutex lock;
  // We want to use CLOCK_MONOTONIC if possible but we can't on macOS
  // because it's not available.
  uORB::MonoClockCond wait;
  int value;
} orb_sem_t;

__EXPORT int orb_sem_init(orb_sem_t *s, int pshared, unsigned value);
__EXPORT int orb_sem_setprotocol(orb_sem_t *s, int protocol);
__EXPORT int orb_sem_wait(orb_sem_t *s);
__EXPORT int orb_sem_trywait(orb_sem_t *sem);
__EXPORT int orb_sem_timedwait(orb_sem_t *sem, const struct timespec *abstime);
__EXPORT int orb_sem_post(orb_sem_t *s);
__EXPORT int orb_sem_getvalue(orb_sem_t *s, int *sval);
__EXPORT int orb_sem_destroy(orb_sem_t *s);

__END_DECLS
