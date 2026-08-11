/**
 * @file px4_compat.cc
 *
 * PX4-compatible uORB API shim — implementation.
 *
 * Architecture
 * ============
 * Subscriptions
 * -------------
 * PX4's API identifies subscriptions with a plain integer ("fd"). This shim
 * maintains a global table (g_sub_table) that maps those integers to the
 * native orb_subscriber_t handles plus per-subscription metadata (last-copy
 * generation, interval setting).  The table is protected by a single mutex.
 *
 * Poll
 * ----
 * px4_poll() needs to block until at least one of the polled subscriptions
 * has new data, or until a timeout expires.  This is implemented by
 * temporarily registering an orb_subscriber_callback_fn on each polled
 * subscription; the callback signals a shared condition variable.  Once
 * the wait completes the callbacks are removed and every subscription is
 * checked for available updates to populate revents.
 *
 * Publishers
 * ----------
 * PX4's orb_advert_t is a plain void*.  The native API uses orb_publisher_t
 * (an opaque struct).  We heap-allocate an orb_publisher_t and return its
 * address cast to void* so the two types map 1-to-1.
 */

#include <uorb_compat/px4_compat.h>

#include <errno.h>
#include <pthread.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>

#include <uorb/uorb.h>

/* -----------------------------------------------------------------------
 * Configuration
 * ----------------------------------------------------------------------- */

#ifndef ORB_PX4_COMPAT_MAX_SUBSCRIBERS
#define ORB_PX4_COMPAT_MAX_SUBSCRIBERS 128
#endif

/* -----------------------------------------------------------------------
 * Subscriber table
 * ----------------------------------------------------------------------- */

struct SubEntry {
  bool in_use;
  orb_subscriber_t sub;
  unsigned interval_ms; /* stored only; not enforced by this shim */
};

static struct SubEntry g_sub_table[ORB_PX4_COMPAT_MAX_SUBSCRIBERS];
static pthread_mutex_t g_sub_mutex = PTHREAD_MUTEX_INITIALIZER;

/** Convert a table index (0-based) to a public fd value (1-based). */
static inline int index_to_fd(int idx) { return idx + 1; }

/** Convert a public fd value to a table index; returns -1 if invalid. */
static inline int fd_to_index(int fd) {
  int idx = fd - 1;
  if (idx < 0 || idx >= ORB_PX4_COMPAT_MAX_SUBSCRIBERS) return -1;
  return idx;
}

/**
 * Allocate a slot in the table and return the new public fd, or -1 on error.
 * Caller must hold g_sub_mutex.
 */
static int alloc_sub_locked(orb_subscriber_t sub) {
  for (int i = 0; i < ORB_PX4_COMPAT_MAX_SUBSCRIBERS; ++i) {
    if (!g_sub_table[i].in_use) {
      g_sub_table[i].in_use = true;
      g_sub_table[i].sub = sub;
      g_sub_table[i].interval_ms = 0;
      return index_to_fd(i);
    }
  }
  return -1;
}

/* -----------------------------------------------------------------------
 * Publisher helpers
 * ----------------------------------------------------------------------- */

/** Retrieve the native publisher struct from a PX4 advert handle. */
static inline orb_publisher_t *pub_from_advert(orb_advert_t h) {
  return (orb_publisher_t *)h;
}

/* -----------------------------------------------------------------------
 * Publication API
 * ----------------------------------------------------------------------- */

orb_advert_t orb_advertise(const struct orb_metadata *meta, const void *data) {
  return orb_advertise_multi(meta, data, NULL);
}

orb_advert_t orb_advertise_queue(const struct orb_metadata *meta, const void *data,
                                 unsigned int queue_size) {
  (void)queue_size; /* queue size is defined at topic-definition time */
  return orb_advertise(meta, data);
}

orb_advert_t orb_advertise_multi(const struct orb_metadata *meta, const void *data,
                                 int *instance) {
  if (!meta || !data) return ORB_ADVERT_INVALID;

  orb_publisher_t *pub = (orb_publisher_t *)malloc(sizeof(orb_publisher_t));
  if (!pub) return ORB_ADVERT_INVALID;

  *pub = (orb_publisher_t)ORB_PUBLISHER_INITIALIZER;

  unsigned int ui_instance = 0;
  unsigned int *pinstance = instance ? &ui_instance : NULL;

  if (orb_publisher_create_multi(pub, meta, pinstance) != ORB_OK) {
    free(pub);
    return ORB_ADVERT_INVALID;
  }

  /* Publish the initial data as PX4's orb_advertise does. */
  if (orb_publisher_publish(pub, data) != ORB_OK) {
    orb_publisher_destroy(pub);
    free(pub);
    return ORB_ADVERT_INVALID;
  }

  if (instance) *instance = (int)ui_instance;

  return (orb_advert_t)pub;
}

orb_advert_t orb_advertise_multi_queue(const struct orb_metadata *meta, const void *data,
                                       int *instance, unsigned int queue_size) {
  (void)queue_size;
  return orb_advertise_multi(meta, data, instance);
}

int orb_publish(const struct orb_metadata *meta, orb_advert_t handle, const void *data) {
  (void)meta;
  if (!handle || !data) return -1;
  return orb_publisher_publish(pub_from_advert(handle), data) == ORB_OK ? 0 : -1;
}

int orb_unadvertise(orb_advert_t *handle) {
  if (!handle || !*handle) return -1;

  orb_publisher_t *pub = pub_from_advert(*handle);
  int rc = orb_publisher_destroy(pub) == ORB_OK ? 0 : -1;
  free(pub);
  *handle = ORB_ADVERT_INVALID;
  return rc;
}

/* -----------------------------------------------------------------------
 * Subscription API
 * ----------------------------------------------------------------------- */

int orb_subscribe(const struct orb_metadata *meta) {
  return orb_subscribe_multi(meta, 0);
}

int orb_subscribe_multi(const struct orb_metadata *meta, unsigned instance) {
  if (!meta) return -1;

  orb_subscriber_t sub = ORB_SUBSCRIBER_INITIALIZER;
  if (orb_subscriber_create_multi(&sub, meta, instance) != ORB_OK) return -1;

  pthread_mutex_lock(&g_sub_mutex);
  int fd = alloc_sub_locked(sub);
  pthread_mutex_unlock(&g_sub_mutex);

  if (fd < 0) {
    orb_subscriber_destroy(&sub);
    return -1;
  }

  return fd;
}

int orb_unsubscribe(int fd) {
  int idx = fd_to_index(fd);
  if (idx < 0) return -1;

  pthread_mutex_lock(&g_sub_mutex);
  if (!g_sub_table[idx].in_use) {
    pthread_mutex_unlock(&g_sub_mutex);
    return -1;
  }

  orb_subscriber_t sub = g_sub_table[idx].sub;
  g_sub_table[idx].in_use = false;
  g_sub_table[idx].sub = (orb_subscriber_t)ORB_SUBSCRIBER_INITIALIZER;
  pthread_mutex_unlock(&g_sub_mutex);

  return orb_subscriber_destroy(&sub) == ORB_OK ? 0 : -1;
}

int orb_copy(const struct orb_metadata *meta, int fd, void *buffer) {
  (void)meta;
  int idx = fd_to_index(fd);
  if (idx < 0 || !buffer) return -1;

  pthread_mutex_lock(&g_sub_mutex);
  if (!g_sub_table[idx].in_use) {
    pthread_mutex_unlock(&g_sub_mutex);
    return -1;
  }
  orb_subscriber_t sub = g_sub_table[idx].sub;
  pthread_mutex_unlock(&g_sub_mutex);

  return orb_subscriber_copy(&sub, buffer) == ORB_OK ? 0 : -1;
}

int orb_check(int fd, bool *updated) {
  if (!updated) return -1;
  int idx = fd_to_index(fd);
  if (idx < 0) return -1;

  pthread_mutex_lock(&g_sub_mutex);
  if (!g_sub_table[idx].in_use) {
    pthread_mutex_unlock(&g_sub_mutex);
    return -1;
  }
  orb_subscriber_t sub = g_sub_table[idx].sub;
  pthread_mutex_unlock(&g_sub_mutex);

  *updated = orb_subscriber_check_update(&sub);
  return 0;
}

int orb_set_interval(int fd, unsigned interval_ms) {
  int idx = fd_to_index(fd);
  if (idx < 0) return -1;

  pthread_mutex_lock(&g_sub_mutex);
  if (!g_sub_table[idx].in_use) {
    pthread_mutex_unlock(&g_sub_mutex);
    return -1;
  }
  g_sub_table[idx].interval_ms = interval_ms;
  pthread_mutex_unlock(&g_sub_mutex);

  return 0;
}

int orb_get_interval(int fd, unsigned *interval_ms) {
  if (!interval_ms) return -1;
  int idx = fd_to_index(fd);
  if (idx < 0) return -1;

  pthread_mutex_lock(&g_sub_mutex);
  if (!g_sub_table[idx].in_use) {
    pthread_mutex_unlock(&g_sub_mutex);
    return -1;
  }
  *interval_ms = g_sub_table[idx].interval_ms;
  pthread_mutex_unlock(&g_sub_mutex);

  return 0;
}

/* -----------------------------------------------------------------------
 * Poll implementation
 * ----------------------------------------------------------------------- */

/** Shared context signalled by publish callbacks during a px4_poll() call. */
struct PollCtx {
  pthread_mutex_t mutex;
  pthread_cond_t cond;
  int ready_count; /* number of fds that signalled so far */
};

/** Per-fd callback argument passed to orb_subscriber_set_callback(). */
struct PollCbArg {
  struct PollCtx *ctx;
  int fd; /* the px4 fd that triggered this callback */
};

static void poll_publish_cb(const void *msg, orb_callback_ctx ctx) {
  (void)msg;
  struct PollCbArg *arg = (struct PollCbArg *)ctx.ptr;
  struct PollCtx *pctx = arg->ctx;

  pthread_mutex_lock(&pctx->mutex);
  pctx->ready_count++;
  pthread_cond_signal(&pctx->cond);
  pthread_mutex_unlock(&pctx->mutex);
}

int px4_poll(px4_pollfd_struct_t *fds, unsigned int nfds, int timeout_ms) {
  if (!fds || nfds == 0) return -1;

  /* Clear revents. */
  for (unsigned i = 0; i < nfds; ++i) fds[i].revents = 0;

  /* --- Phase 1: check for already-available data. --- */
  int ready = 0;
  for (unsigned i = 0; i < nfds; ++i) {
    if (!(fds[i].events & POLLIN)) continue;
    bool updated = false;
    if (orb_check(fds[i].fd, &updated) == 0 && updated) {
      fds[i].revents = POLLIN;
      ready++;
    }
  }

  if (ready > 0 || timeout_ms == 0) return ready;

  /* --- Phase 2: register callbacks and wait. --- */

  struct PollCtx pctx;
  pthread_mutexattr_t mattr;
  pthread_mutexattr_init(&mattr);
  pthread_mutex_init(&pctx.mutex, &mattr);
  pthread_mutexattr_destroy(&mattr);

  pthread_condattr_t cattr;
  pthread_condattr_init(&cattr);
  pthread_condattr_setclock(&cattr, CLOCK_MONOTONIC);
  pthread_cond_init(&pctx.cond, &cattr);
  pthread_condattr_destroy(&cattr);
  pctx.ready_count = 0;

  /* Per-fd callback arguments. */
  struct PollCbArg *args =
      (struct PollCbArg *)malloc(nfds * sizeof(struct PollCbArg));
  if (!args) {
    pthread_mutex_destroy(&pctx.mutex);
    pthread_cond_destroy(&pctx.cond);
    return -1;
  }

  unsigned registered = 0;
  for (unsigned i = 0; i < nfds; ++i) {
    if (!(fds[i].events & POLLIN)) continue;

    int idx = fd_to_index(fds[i].fd);
    if (idx < 0) continue;

    pthread_mutex_lock(&g_sub_mutex);
    bool in_use = g_sub_table[idx].in_use;
    orb_subscriber_t sub = g_sub_table[idx].sub;
    pthread_mutex_unlock(&g_sub_mutex);

    if (!in_use) continue;

    args[i].ctx = &pctx;
    args[i].fd = fds[i].fd;
    orb_callback_ctx cb_ctx;
    cb_ctx.ptr = &args[i];

    if (orb_subscriber_set_callback(&sub, poll_publish_cb, cb_ctx) == ORB_OK) {
      /* Store back so we can clear later. */
      pthread_mutex_lock(&g_sub_mutex);
      g_sub_table[idx].sub = sub;
      pthread_mutex_unlock(&g_sub_mutex);
      registered++;
    }
  }

  /* Wait for at least one callback to fire (or timeout). */
  pthread_mutex_lock(&pctx.mutex);
  if (pctx.ready_count == 0) {
    if (timeout_ms < 0) {
      /* Block indefinitely; loop to handle spurious wakeups. */
      while (pctx.ready_count == 0) {
        pthread_cond_wait(&pctx.cond, &pctx.mutex);
      }
    } else {
      struct timespec ts;
      clock_gettime(CLOCK_MONOTONIC, &ts);
      ts.tv_sec += timeout_ms / 1000;
      ts.tv_nsec += (timeout_ms % 1000) * 1000000L;
      if (ts.tv_nsec >= 1000000000L) {
        ts.tv_sec++;
        ts.tv_nsec -= 1000000000L;
      }
      /* Loop to handle spurious wakeups. */
      while (pctx.ready_count == 0) {
        if (pthread_cond_timedwait(&pctx.cond, &pctx.mutex, &ts) != 0) {
          break; /* ETIMEDOUT or error */
        }
      }
    }
  }
  pthread_mutex_unlock(&pctx.mutex);

  /* Unregister all callbacks we registered. */
  for (unsigned i = 0; i < nfds; ++i) {
    if (!(fds[i].events & POLLIN)) continue;

    int idx = fd_to_index(fds[i].fd);
    if (idx < 0) continue;

    pthread_mutex_lock(&g_sub_mutex);
    bool in_use = g_sub_table[idx].in_use;
    orb_subscriber_t sub = g_sub_table[idx].sub;
    pthread_mutex_unlock(&g_sub_mutex);

    if (!in_use) continue;
    orb_subscriber_clear_callback(&sub);
    pthread_mutex_lock(&g_sub_mutex);
    g_sub_table[idx].sub = sub;
    pthread_mutex_unlock(&g_sub_mutex);
  }

  free(args);
  pthread_mutex_destroy(&pctx.mutex);
  pthread_cond_destroy(&pctx.cond);

  /* --- Phase 3: scan for ready fds. --- */
  ready = 0;
  for (unsigned i = 0; i < nfds; ++i) {
    if (!(fds[i].events & POLLIN)) continue;
    bool updated = false;
    if (orb_check(fds[i].fd, &updated) == 0 && updated) {
      fds[i].revents = POLLIN;
      ready++;
    }
  }

  return ready;
}
