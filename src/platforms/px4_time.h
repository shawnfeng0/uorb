#pragma once

#include <unistd.h>
#include <sys/types.h>
#include <time.h>
#include <pthread.h>
#include "visibility.h"

#if defined(__PX4_POSIX) || defined(__PX4_QURT)
__BEGIN_DECLS
__EXPORT int px4_clock_gettime(clockid_t clk_id, struct timespec *tp);
__END_DECLS
#else
#define px4_clock_gettime system_clock_gettime
#endif

#define px4_clock_settime system_clock_settime
#define px4_usleep system_usleep
#define px4_sleep system_sleep
#define px4_pthread_cond_timedwait system_pthread_cond_timedwait
