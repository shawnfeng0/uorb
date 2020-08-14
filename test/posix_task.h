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
#define SCHED_PRIORITY_DEFAULT             \
  (((sched_get_priority_max(SCHED_FIFO) -  \
     sched_get_priority_min(SCHED_FIFO)) / \
    2) +                                   \
   sched_get_priority_min(SCHED_FIFO))

typedef int (*px4_main_t)(int argc, char *argv[]);

#define PX4_DEBUG(...) (void *)0;

#define PX4_INFO LOGGER_INFO
#define PX4_ERR LOGGER_DEBUG
#define PX4_STORAGEDIR "/tmp/"

pthread_t px4_task_spawn_cmd(const char *name, int scheduler, int priority,
                             int stack_size, px4_main_t entry,
                             char *const argv[]);
