//
// Created by fs on 2020-01-22.
//

#pragma once

#include <pthread.h>
#include <sched.h>

#include "ulog/ulog.h"

/** Default scheduler type */
#define SCHED_DEFAULT SCHED_FIFO

#define SCHED_PRIORITY_MAX sched_get_priority_max(SCHED_FIFO)
#define SCHED_PRIORITY_MIN sched_get_priority_min(SCHED_FIFO)
#define SCHED_PRIORITY_DEFAULT \
  (((SCHED_PRIORITY_MAX - SCHED_PRIORITY_MIN) / 2) + SCHED_PRIORITY_MIN)

typedef int (*thread_entry_t)(int argc, char *argv[]);

#define ORB_DEBUG(...) (void *)0;
#define ORB_INFO LOGGER_INFO
#define ORB_ERROR LOGGER_ERROR

pthread_t task_spawn_cmd(const char *name, int scheduler, int priority,
                         int stack_size, thread_entry_t entry,
                         char *const *argv);
