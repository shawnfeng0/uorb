//
// Created by fs on 2020-01-22.
//

#include "posix_task.h"

#include <pthread.h>

#include <cerrno>
#include <climits>
#include <cstdlib>
#include <cstring>

typedef struct {
  thread_entry_t entry;
  char name[16];  // pthread_setname_np is restricted to 16 chars
  int argc;
  char *argv[];
  // strings are allocated after the struct data
} pthread_data_t;

static void *entry_adapter(void *ptr) {
  if (!ptr) {
    return nullptr;
  }
  auto &data = *(pthread_data_t *)ptr;

  // The method of thread name setting is not compatible
  int rv = pthread_setname_np(pthread_self(), data.name);

  if (rv) {
    ORB_ERROR("failed to set name of thread %d %d\n", rv, errno);
  }

  data.entry(data.argc, data.argv);
  free(ptr);
  pthread_exit(nullptr);
}

pthread_t task_spawn_cmd(const char *name, thread_entry_t entry,
                         char *const *argv) {
  int i;
  int argc = 0;
  unsigned int len = 0;
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

  unsigned long struct_size =
      sizeof(pthread_data_t) + (argc + 1) * sizeof(char *);

  // not safe to pass stack data to the thread creation
  auto *task_data = (pthread_data_t *)malloc(struct_size + len);

  if (task_data == nullptr) {
    ORB_ERROR("No memory");
    return -ENOMEM;
  }

  memset(task_data, 0, struct_size + len);
  unsigned long offset = ((unsigned long)task_data) + struct_size;

  strncpy(task_data->name, name, 16);
  task_data->name[15] = 0;
  task_data->entry = entry;
  task_data->argc = argc;

  for (i = 0; i < argc; i++) {
    ORB_DEBUG("arg %d %s\n", i, argv[i]);
    task_data->argv[i] = (char *)offset;
    strcpy((char *)offset, argv[i]);
    offset += strlen(argv[i]) + 1;
  }

  // Must add NULL at end of argv
  task_data->argv[argc] = (char *)nullptr;

  ORB_DEBUG("starting task %s", name);

  pthread_t task_id = 0;
  int rv = pthread_create(&task_id, nullptr, &entry_adapter, (void *)task_data);

  if (rv != 0) {
    ORB_ERROR("failed to create thread %d %d\n", rv, errno);
    free(task_data);
    return (rv < 0) ? rv : -rv;
  }
  return task_id;
}
