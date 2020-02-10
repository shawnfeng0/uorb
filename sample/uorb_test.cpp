//
// Created by fs on 2020-01-15.
//

#include <pthread.h>
#include <uORB/topics/cpuload.h>
#include <unistd.h>

#include <Publication.hpp>
#include <SubscriptionPoll.hpp>

#include "base/drv_hrt.h"
#include "sample/ulog/src/ulog.h"

void *adviser_cpuload(void *) {
  uORB::PublicationData<cpuload_s> cpuload_pub(ORB_ID(cpuload));

  for (int i = 0; i < 10; i++) {
    cpuload_pub.get().timestamp = hrt_absolute_time();
    cpuload_pub.get().load++;
    cpuload_pub.get().ram_usage++;
    if (!cpuload_pub.update()) {
      LOG_WARN("publish error");
    }
    usleep(1 * 1000 * 1000);
  }
  LOG_WARN("Publication over.");
  return nullptr;
}

void *cpuload_update_poll(void *arg) {
  uORB::SubscriptionPollData<cpuload_s> cpu_load_sub_data(ORB_ID(cpuload));

  uint32_t sleep_time_s = (arg) ? *(int32_t *)arg : 0;

  LOG_INFO("orb_subcribe, cycle: %d", sleep_time_s);

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#endif

  orb_pollfd_t fds[] = {
      {.fd = cpu_load_sub_data.get_fd(), .events = POLLIN},
  };

  for (int i = 0; i < 20; i++) {
    sleep(sleep_time_s);
    LOG_INFO("TOPIC: cpuload #%d", i);
    int timeout_ms = 300;
    if (orb_poll(fds, ARRAY_SIZE(fds), timeout_ms)) {
      cpu_load_sub_data.update();
      const struct cpuload_s &cpu_loader = cpu_load_sub_data.get();
      LOG_MULTI_TOKEN(cpu_loader.timestamp, cpu_loader.load,
                      cpu_loader.ram_usage);
    } else {
      /* this means none of our providers is giving us data */
      LOG_ERROR("Got no data within %d milliseconds", timeout_ms);
      break;
    }
  }
  LOG_WARN("subscription over");
  return nullptr;
}

void uorb_sample() {
  // One publishing thread, three subscription threads
  pthread_t pthread_id;
  pthread_create(&pthread_id, nullptr, adviser_cpuload, nullptr);
  static uint32_t sleep_time_s_1 = 1;
  pthread_create(&pthread_id, nullptr, cpuload_update_poll, &sleep_time_s_1);
  static uint32_t sleep_time_s_2 = 2;
  pthread_create(&pthread_id, nullptr, cpuload_update_poll, &sleep_time_s_2);
  static uint32_t sleep_time_s_3 = 3;
  pthread_create(&pthread_id, nullptr, cpuload_update_poll, &sleep_time_s_3);
}

int main(int argc, char *argv[]) {
  uorb_sample();

  // Wait for all threads to finish
  pthread_exit(nullptr);
}
