//
// Created by fs on 2020-01-15.
//

#include "uORB.h"
#include <pthread.h>
#include <src/drivers/drv_hrt.h>
#include <src/platforms/px4_posix.h>
#include <topic_header/cpuload.h>
#include <unistd.h>

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

void* cpuload_update_poll(void* arg) {
  struct cpuload_s cpu_loader {};
  int cpuload_sub = orb_subscribe(ORB_ID(cpuload));

  uint32_t sleep_time_us = 0;
  if (arg)
    sleep_time_us = *(uint32_t *) arg;

  px4_pollfd_struct_t fds[] = {
      {.fd = cpuload_sub, .events = POLLIN},
  };

  memset(&cpu_loader, 0, sizeof(cpu_loader));
  int error_counter = 0;

  for (int i = 0; i < 12; i ++) {
    usleep(sleep_time_us);
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
      orb_copy(ORB_ID(cpuload), cpuload_sub, &cpu_loader);
      LOG_TOKEN(cpu_loader.timestamp);
      LOG_TOKEN(cpu_loader.load);
      LOG_TOKEN(cpu_loader.ram_usage);
    }
  }
  return nullptr;
}

void uorb_sample() {
  // One publishing thread, three subscription threads
  pthread_t pthread1, pthread2, pthread3, pthread4;
  pthread_create(&pthread1, nullptr, adviser_cpuload, nullptr);
  static uint32_t sleep_time_us_1 = 1 * 1000 * 1000;
  pthread_create(&pthread2, nullptr, cpuload_update_poll, &sleep_time_us_1);
  static uint32_t sleep_time_us_2 = 1.5 * 1000 * 1000;
  pthread_create(&pthread3, nullptr, cpuload_update_poll, &sleep_time_us_2);
  static uint32_t sleep_time_us_3 = 2 * 1000 * 1000;
  pthread_create(&pthread4, nullptr, cpuload_update_poll, &sleep_time_us_3);
}


int uorb_main(int argc, char *argv[]);

extern "C" int uorb_tests_main(int argc, char *argv[]);
extern "C" __EXPORT int cdev_test_main(int argc, char *argv[]);

int main(int argc, char *argv[]) {
  // uorb initial
  char *orb_start_args[] = {(char *) "orb", (char *) "start"};
  uorb_main(sizeof(orb_start_args) / sizeof(orb_start_args[0]), orb_start_args);

  if (argc > 1) {
    if (!strcmp(argv[1], "orb_test")) {
      // uorb unit test
      char *orb_test_args[] = {(char *)"orb_test"};
      uorb_tests_main(sizeof(orb_test_args) / sizeof(orb_test_args[0]),
                      orb_test_args);
    } else if (!strcmp(argv[1], "cdev_test")) {
      // cdev unit test
      char *cdev_start_args[] = {(char *) "cdev", (char *) "start"};
      cdev_test_main(sizeof(cdev_start_args) / sizeof(cdev_start_args[0]),cdev_start_args);
    } else {
      uorb_sample();
    }
  } else {
    uorb_sample();
  }

  // Wait for all threads to finish
  pthread_exit(nullptr);
}
