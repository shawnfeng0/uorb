//
// Created by fs on 2020-01-22.
//

#pragma once

#include <pthread.h>
#include <sched.h>

#include "ulog/ulog.h"

typedef int (*thread_entry_t)(int argc, char *argv[]);

#define ORB_DEBUG(...) ((void *)0)
#define ORB_INFO LOGGER_INFO
#define ORB_ERROR LOGGER_ERROR

pthread_t task_spawn_cmd(const char* name, thread_entry_t entry,
                         char* const* argv);
