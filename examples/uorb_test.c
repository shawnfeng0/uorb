//
// Created by fs on 2020-01-15.
//

#include <pthread.h>
#include <unistd.h>
#include <uorb/topics/cpuload.h>

#include "ulog/ulog.h"
#include "uorb/base/abs_time.h"

void *adviser_cpuload(void *arg) {
  struct cpuload_s cpuload;
  orb_advert_t cpu_load_pub = orb_advertise_queue(ORB_ID(cpuload), NULL, 3);

  for (int i = 0; i < 10; i++) {
    cpuload.timestamp = orb_absolute_time();
    cpuload.load++;
    cpuload.ram_usage++;
    if (!orb_publish(ORB_ID(cpuload), cpu_load_pub, &cpuload)) {
      LOGGER_ERROR("Publish error");
    }
    usleep(1 * 1000 * 1000);
  }

  orb_unadvertise(&cpu_load_pub);

  LOGGER_WARN("Publication over.");
  return NULL;
}

void *cpuload_update_poll(void *arg) {
  int sleep_time = *(int *)arg;
  sleep(sleep_time);

  orb_subscriber_t cpu_load_sub_data = orb_subscribe(ORB_ID(cpuload));

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#endif

  while (true) {
    struct orb_pollfd pollfds[] = {{.fd = cpu_load_sub_data, .events = POLLIN}};
    int timeout = 2000;
    if (0 < orb_poll(pollfds, ARRAY_SIZE(pollfds), timeout)) {
      struct cpuload_s cpu_loader;
      orb_copy(ORB_ID(cpuload), cpu_load_sub_data, &cpu_loader);
      LOGGER_MULTI_TOKEN(cpu_loader.timestamp, cpu_loader.load,
                         cpu_loader.ram_usage);
    } else {
      LOGGER_WARN("Got no data within %d milliseconds", timeout);
      break;
    }
  }

  orb_unsubscribe(&cpu_load_sub_data);

  LOGGER_WARN("subscription over");
  return NULL;
}

void uorb_sample() {
  // One publishing thread, three subscription threads
  pthread_t pthread_id;
  pthread_create(&pthread_id, NULL, adviser_cpuload, NULL);
  static uint32_t sleep_time_s_1 = 1;
  pthread_create(&pthread_id, NULL, cpuload_update_poll, &sleep_time_s_1);
  static uint32_t sleep_time_s_2 = 2;
  pthread_create(&pthread_id, NULL, cpuload_update_poll, &sleep_time_s_2);
  static uint32_t sleep_time_s_3 = 3;
  pthread_create(&pthread_id, NULL, cpuload_update_poll, &sleep_time_s_3);
}

int main(int argc, char *argv[]) {
  uorb_sample();

  // Wait for all threads to finish
  pthread_exit(NULL);
}
