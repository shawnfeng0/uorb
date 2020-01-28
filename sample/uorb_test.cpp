//
// Created by fs on 2020-01-15.
//

#include <unistd.h>
#include <pthread.h>

#include "../base/drv_hrt.h"
#include "../base/orb_defines.h"
#include "../base/orb_posix.h"
#include "topic_header/cpuload.h"
#include "ulog/src/ulog.h"

void *adviser_cpuload(void *) {
  struct cpuload_s cpuload {};

  cpuload.timestamp = hrt_absolute_time();
  cpuload.load = 1.0f;
  cpuload.ram_usage = 1.0f;

  orb_advert_t cpuload_pub = orb_advertise_queue(ORB_ID(cpuload), &cpuload, 10);
  if (cpuload_pub == nullptr) {
    LOG_WARN("advertise error");
    return nullptr;
  } else {
    LOG_INFO("advertise successful");
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

void* cpuload_update_poll(void* arg) {
  struct cpuload_s cpu_loader {};
  int cpuload_sub = orb_subscribe(ORB_ID(cpuload));
  if (cpuload_sub == ORB_ERROR) {
    LOG_ERROR("orb_subcribe error: %s", strerror(orb_errno));
    return nullptr;
  }

  uint32_t sleep_time_us = 0;
  if (arg)
    sleep_time_us = *(uint32_t *) arg;

  LOG_INFO("orb_subcribe success, cycle: %.1fs", sleep_time_us / 1000.f / 1000.f);

  orb_pollfd_struct_t fds[] = {
      {.fd = cpuload_sub, .events = POLLIN},
  };

  memset(&cpu_loader, 0, sizeof(cpu_loader));
  int error_counter = 0;

  for (int i = 0; i < 12; i ++) {
    usleep(sleep_time_us);
    LOG_INFO("TOPIC: cpuload #%d", i);
    int timeout_ms = 5000;
    int poll_ret = orb_poll(fds, 1, timeout_ms);
    if (poll_ret < 0) {
      /* this is seriously bad - should be an emergency */
      if (error_counter < 10 || error_counter % 50 == 0) {
        /* use a counter to prevent flooding (and slowing us down) */
        LOG_ERROR("ERROR return value from poll(): %d", poll_ret);
      }
      error_counter++;
    } else if (poll_ret == 0) {
      /* this means none of our providers is giving us data */
      LOG_ERROR("Got no data within %d second", timeout_ms);
    } else {
      orb_copy(ORB_ID(cpuload), cpuload_sub, &cpu_loader);
      LOG_MULTI_TOKEN(cpu_loader.timestamp, cpu_loader.load, cpu_loader.ram_usage);
    }
  }
  return nullptr;
}

void uorb_sample() {
  // One publishing thread, three subscription threads
  pthread_t pthread_id;
  pthread_create(&pthread_id, nullptr, adviser_cpuload, nullptr);
  static uint32_t sleep_time_us_1 = 1 * 1000 * 1000;
  pthread_create(&pthread_id, nullptr, cpuload_update_poll, &sleep_time_us_1);
  static uint32_t sleep_time_us_2 = 1.5 * 1000 * 1000;
  pthread_create(&pthread_id, nullptr, cpuload_update_poll, &sleep_time_us_2);
  static uint32_t sleep_time_us_3 = 2 * 1000 * 1000;
  pthread_create(&pthread_id, nullptr, cpuload_update_poll, &sleep_time_us_3);
}

int main(int argc, char *argv[]) {
  uorb_sample();

  // Wait for all threads to finish
  pthread_exit(nullptr);
}
