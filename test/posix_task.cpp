//
// Created by fs on 2020-01-22.
//

#include "posix_task.h"
#include "base/orb_errno.h"
#include "base/orb_log.h"

#include <climits>
#include <string>
#include <cstring>
#include <pthread.h>
#include <stdlib.h>

typedef struct {
  px4_main_t entry;
  char name[16]; //pthread_setname_np is restricted to 16 chars
  int argc;
  char *argv[];
  // strings are allocated after the struct data
} pthdata_t;

static void *entry_adapter(void *ptr)
{
  auto *data = (pthdata_t *) ptr;

  int rv;

  // set the threads name
#ifdef __PX4_DARWIN
  rv = pthread_setname_np(data->name);
#else
  rv = pthread_setname_np(pthread_self(), data->name);
#endif

  if (rv) {
    ORB_ERR("px4_task_spawn_cmd: failed to set name of thread %d %d\n", rv, errno);
  }

  data->entry(data->argc, data->argv);
  free(ptr);
  ORB_DEBUG("Before px4_task_exit");
  pthread_exit(nullptr);
}

pthread_t px4_task_spawn_cmd(const char *name, int scheduler, int priority, int stack_size, px4_main_t entry,
                              char *const argv[])
{
  int i;
  int argc = 0;
  unsigned int len = 0;
  struct sched_param param = {};
  char *p = (char *)argv;

  // Calculate argc
  while (p != (char *)nullptr) {
    p = argv[argc];

    if (p == (char *)nullptr) {
      break;
    }

    ++argc;
    len += strlen(p) + 1;
  }

  unsigned long structsize = sizeof(pthdata_t) + (argc + 1) * sizeof(char *);

  // not safe to pass stack data to the thread creation
  auto *taskdata = (pthdata_t *)malloc(structsize + len);

  if (taskdata == nullptr) {
    return -ENOMEM;
  }

  memset(taskdata, 0, structsize + len);
  unsigned long offset = ((unsigned long)taskdata) + structsize;

  strncpy(taskdata->name, name, 16);
  taskdata->name[15] = 0;
  taskdata->entry = entry;
  taskdata->argc = argc;

  for (i = 0; i < argc; i++) {
    ORB_DEBUG("arg %d %s\n", i, argv[i]);
    taskdata->argv[i] = (char *)offset;
    strcpy((char *)offset, argv[i]);
    offset += strlen(argv[i]) + 1;
  }

  // Must add NULL at end of argv
  taskdata->argv[argc] = (char *)nullptr;

  ORB_DEBUG("starting task %s", name);

  pthread_attr_t attr;
  int rv = pthread_attr_init(&attr);

  if (rv != 0) {
    ORB_ERR("px4_task_spawn_cmd: failed to init thread attrs");
    free(taskdata);
    return (rv < 0) ? rv : -rv;
  }

#ifndef __PX4_DARWIN

  if (stack_size < PTHREAD_STACK_MIN) {
    stack_size = PTHREAD_STACK_MIN;
  }

  rv = pthread_attr_setstacksize(&attr, stack_size);

  if (rv != 0) {
    ORB_ERR("pthread_attr_setstacksize to %d returned error (%d)", stack_size, rv);
    pthread_attr_destroy(&attr);
    free(taskdata);
    return (rv < 0) ? rv : -rv;
  }

#endif

  rv = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);

  if (rv != 0) {
    ORB_ERR("px4_task_spawn_cmd: failed to set inherit sched");
    pthread_attr_destroy(&attr);
    free(taskdata);
    return (rv < 0) ? rv : -rv;
  }

  rv = pthread_attr_setschedpolicy(&attr, scheduler);

  if (rv != 0) {
    ORB_ERR("px4_task_spawn_cmd: failed to set sched policy");
    pthread_attr_destroy(&attr);
    free(taskdata);
    return (rv < 0) ? rv : -rv;
  }

#ifdef __PX4_CYGWIN
  /* Priorities on Windows are defined a lot differently */
	priority = SCHED_PRIORITY_DEFAULT;
#endif

  param.sched_priority = priority;

  rv = pthread_attr_setschedparam(&attr, &param);

  if (rv != 0) {
    ORB_ERR("px4_task_spawn_cmd: failed to set sched param");
    ORB_INFO("%s", strerror(rv));
    LOG_TOKEN(priority);
    pthread_attr_destroy(&attr);
    free(taskdata);
    return (rv < 0) ? rv : -rv;
  }

  pthread_t taskid = 0;

  rv = pthread_create(&taskid, &attr, &entry_adapter, (void *) taskdata);

  if (rv != 0) {

    if (rv == EPERM) {
      //printf("WARNING: NOT RUNING AS ROOT, UNABLE TO RUN REALTIME THREADS\n");
      rv = pthread_create(&taskid, nullptr, &entry_adapter, (void *) taskdata);

      if (rv != 0) {
        ORB_ERR("px4_task_spawn_cmd: failed to create thread %d %d\n", rv, errno);
        pthread_attr_destroy(&attr);
        free(taskdata);
        return (rv < 0) ? rv : -rv;
      }

    } else {
      pthread_attr_destroy(&attr);
      free(taskdata);
      return (rv < 0) ? rv : -rv;
    }
  }

  pthread_attr_destroy(&attr);
  return taskid;
}
