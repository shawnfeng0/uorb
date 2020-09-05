//
// Created by fs on 2020-01-20.
//
#include "uorb/base/orb_errno.h"

#include <pthread.h>

static pthread_key_t key;
static pthread_once_t init_done = PTHREAD_ONCE_INIT;
static int errno_aux;

static void free_errno(void *p_errno) { delete ((int *)p_errno); }

static void thread_init() { pthread_key_create(&key, free_errno); }

int *__orb_errno_location() {
  pthread_once(&init_done, thread_init);

  int *p_errno = (int *)pthread_getspecific(key);

  if (p_errno == nullptr) {
    p_errno = new int;
  }

  if (p_errno != nullptr) {
    pthread_setspecific(key, p_errno);
  } else {
    p_errno = &errno_aux;
  }

  return p_errno;
}
