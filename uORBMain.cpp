/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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

#include <string.h>

#include "uORBManager.hpp"
#include "uORB.h"
#include "uORBCommon.hpp"
#include "topic_header/cpuload.h"

#include <px4_log.h>
#include <px4_module.h>

static uORB::DeviceMaster *g_dev = nullptr;
static void usage()
{
  PRINT_MODULE_DESCRIPTION(
      R"DESCR_STR(
### Description
uORB is the internal pub-sub messaging system, used for communication between modules.

It is typically started as one of the very first modules and most other modules depend on it.

### Implementation
No thread or work queue is needed, the module start only makes sure to initialize the shared global state.
Communication is done via shared memory.
The implementation is asynchronous and lock-free, ie. a publisher does not wait for a subscriber and vice versa.
This is achieved by having a separate buffer between a publisher and a subscriber.

The code is optimized to minimize the memory footprint and the latency to exchange messages.

The interface is based on file descriptors: internally it uses `read`, `write` and `ioctl`. Except for the
publications, which use `orb_advert_t` handles, so that they can be used from interrupts as well (on NuttX).

Messages are defined in the `/msg` directory. They are converted into C/C++ code at build-time.

If compiled with ORB_USE_PUBLISHER_RULES, a file with uORB publication rules can be used to configure which
modules are allowed to publish which topics. This is used for system-wide replay.

### Examples
Monitor topic publication rates. Besides `top`, this is an important command for general system inspection:
$ uorb top
)DESCR_STR");

  PRINT_MODULE_USAGE_NAME("uorb", "communication");
  PRINT_MODULE_USAGE_COMMAND("start");
  PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Print topic statistics");
  PRINT_MODULE_USAGE_COMMAND_DESCR("top", "Monitor topic publication rates");
  PRINT_MODULE_USAGE_PARAM_FLAG('a', "print all instead of only currently publishing topics", true);
  PRINT_MODULE_USAGE_PARAM_FLAG('1', "run only once, then exit", true);
  PRINT_MODULE_USAGE_ARG("<filter1> [<filter2>]", "topic(s) to match (implies -a)", true);
}

void *adviser_cpuload(void *) {
  struct cpuload_s cpuload {};
  static orb_advert_t cpuload_pub = nullptr;

  cpuload.timestamp = hrt_absolute_time();
  cpuload.load = 1.0f;
  cpuload.ram_usage = 1.0f;

  if (cpuload_pub == nullptr) {
    LOG_TRACE("before advertise");
    cpuload_pub = orb_advertise(ORB_ID(cpuload), &cpuload);
    LOG_TRACE("after advertise");
    // cpuload_pub = orb_advertise_queue(ORB_ID(cpuload), &cpuload, 2);
  }
  usleep(2 * 1000 * 1000);
  for (int i = 0; i < 10; i++) {
    usleep(1 * 1000 * 1000);
    cpuload.timestamp = hrt_absolute_time();
    cpuload.load++;
    cpuload.ram_usage++;
    orb_publish(ORB_ID(cpuload), cpuload_pub, &cpuload);
  }
  return nullptr;
}

void* cpuload_update_poll(void*) {
  bool updated;
  struct cpuload_s container{};
  int cpuload_sub = orb_subscribe(ORB_ID(cpuload));

  px4_pollfd_struct_t fds[] = {
      {.fd = cpuload_sub, .events = POLLIN},
  };

  memset(&container, 0, sizeof(container));
  int error_counter = 0;

  for (int i = 0; i < 15; i ++) {
    LOG_INFO("TOPIC: cpuload #%d", i);
    int timeout_ms = 5000;
    int poll_ret = px4_poll(fds, 1, timeout_ms);
    if (poll_ret < 0) {
      /* this is seriously bad - should be an emergency */
      if (error_counter < 10 || error_counter % 50 == 0) {
        /* use a counter to prevent flooding (and slowing us down) */
        PX4_ERR("ERROR return value from poll(): %d", poll_ret);
      }
      error_counter++;
    } else if (poll_ret == 0) {
      /* this means none of our providers is giving us data */
      PX4_ERR("Got no data within %d second", timeout_ms);
    } else {
      orb_copy(ORB_ID(cpuload), cpuload_sub, &container);
      LOG_TOKEN(container.timestamp);
      LOG_TOKEN(container.load);
      LOG_TOKEN(container.ram_usage);
    }
  }
  return nullptr;
}

int main(int argc, char *argv[])
{
  if (argc < 2) {
    usage();
    return -EINVAL;
  }

  /*
   * Start/load the driver.
   */
  if (!strcmp(argv[1], "start")) {

    if (g_dev != nullptr) {
      PX4_WARN("already loaded");
      /* user wanted to start uorb, its already running, no error */
      return 0;
    }

    if (!uORB::Manager::initialize()) {
      PX4_ERR("uorb manager alloc failed");
      return -ENOMEM;
    }

    /* create the driver */
    g_dev = uORB::Manager::get_instance()->get_device_master();

    if (g_dev == nullptr) {
      return -errno;
    }

    pthread_t pthread1, pthread2, pthread3, pthread4;
    pthread_create(&pthread1, nullptr, adviser_cpuload, nullptr);
    pthread_create(&pthread2, nullptr, cpuload_update_poll, nullptr);
//    pthread_create(&pthread3, nullptr, cpuload_update_poll, nullptr);
//    pthread_create(&pthread4, nullptr, cpuload_update_poll, nullptr);

    pthread_join(pthread1, nullptr);
    pthread_join(pthread2, nullptr);
//    pthread_join(pthread3, nullptr);
//    pthread_join(pthread4, nullptr);

    return OK;
  }

  /*
   * Print driver information.
   */
  if (!strcmp(argv[1], "status")) {
    if (g_dev != nullptr) {
      g_dev->printStatistics(true);

    } else {
      PX4_INFO("uorb is not running");
    }

    return OK;
  }

  if (!strcmp(argv[1], "top")) {
    if (g_dev != nullptr) {
      g_dev->showTop(argv + 2, argc - 2);

    } else {
      PX4_INFO("uorb is not running");
    }

    return OK;
  }

  usage();
  return -EINVAL;
}
