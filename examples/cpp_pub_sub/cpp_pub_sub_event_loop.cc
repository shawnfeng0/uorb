// uorb::EventLoop example.
//
// Demonstrates:
//   * Subscribe<Topic>(cb)        -- EventLoop owns the subscription.
//   * Multiple publisher threads on different topics.
//   * Quit() from another thread to stop the loop.
//
// Copyright (c) 2021-2025 shawnfeng. All rights reserved.

#include <inttypes.h>
#include <unistd.h>

#include <cstdio>
#include <thread>

#include "uevent/uevent.h"
#include "uorb/publication.h"
#include "uorb/topics/example_string.h"
#include "uorb/topics/sensor_accel.h"
#include "uorb/topics/sensor_gyro.h"
#include "uorb_uevent/uorb_uevent.h"

void thread_publisher_example_string() {
  uorb::PublicationData<uorb::msg::example_string> pub_example_string;
  for (int i = 0; i < 5; i++) {
    auto &data = pub_example_string.data();
    data.timestamp = orb_absolute_time_us();
    snprintf(reinterpret_cast<char *>(data.str), example_string_s::STRING_LENGTH, "%d: %s", i,
             "This is a string message. ");
    if (pub_example_string.Publish() != ORB_OK) {
      printf("Publish example_string error\n");
    }
    usleep(2000 * 1000);  // 2 seconds
  }
  printf("example_string publication over.\n");
}

void thread_publisher_sensor_accel() {
  uorb::PublicationData<uorb::msg::sensor_accel> pub_sensor_accel;
  for (int i = 0; i < 20; i++) {
    auto &accel = pub_sensor_accel.data();
    accel.timestamp = orb_absolute_time_us();
    accel.x = i * 0.1f;
    accel.y = i * 0.2f;
    accel.z = i * 0.3f;
    accel.temperature = 25.0f + i;
    if (pub_sensor_accel.Publish() != ORB_OK) {
      printf("Publish sensor_accel error\n");
    }
    usleep(300 * 1000);  // 300 ms
  }
  printf("sensor_accel publication over.\n");
}

void thread_publisher_sensor_gyro() {
  uorb::PublicationData<uorb::msg::sensor_gyro> pub_sensor_gyro;
  for (int i = 0; i < 10; i++) {
    auto &gyro = pub_sensor_gyro.data();
    gyro.timestamp = orb_absolute_time_us();
    gyro.x = i * 1.1f;
    gyro.y = i * 1.2f;
    gyro.z = i * 1.3f;
    gyro.temperature = 30.0f + i;
    if (pub_sensor_gyro.Publish() != ORB_OK) {
      printf("Publish sensor_gyro error\n");
    }
    usleep(1000 * 1000);  // 1 second
  }
  printf("sensor_gyro publication over.\n");
}

int main() {
  uorb::EventLoop loop;
  if (!loop) {
    printf("EventLoop create failed\n");
    return -1;
  }

  // (1) Loop-owned subscriptions: Subscribe<Topic>(callback).
  loop.Subscribe<uorb::msg::example_string>([](const example_string_s &msg) {
    printf("[example_string] timestamp: %" PRIu64 ", msg: '%s'\n", msg.timestamp, msg.str);
  });
  loop.Subscribe<uorb::msg::sensor_accel>([](const sensor_accel_s &msg) {
    printf("[sensor_accel] timestamp: %" PRIu64 ", accel: (%.2f, %.2f, %.2f), temp: %.2f\n", msg.timestamp, msg.x,
                msg.y, msg.z, msg.temperature);
  });

  // (2) All subscriptions are loop-owned via Subscribe<Topic>(callback).
  loop.Subscribe<uorb::msg::sensor_gyro>([](const sensor_gyro_s &msg) {
    printf("[sensor_gyro] timestamp: %" PRIu64 ", gyro: (%.2f, %.2f, %.2f), temp: %.2f\n", msg.timestamp, msg.x,
                msg.y, msg.z, msg.temperature);
  });

  // Run the event loop on a worker thread. Run() blocks until Quit().
  std::thread loop_thread([&] { loop.Run(); });

  // Publish from multiple threads.
  std::thread pub_thread1(thread_publisher_example_string);
  std::thread pub_thread2(thread_publisher_sensor_accel);
  std::thread pub_thread3(thread_publisher_sensor_gyro);
  pub_thread1.join();
  pub_thread2.join();
  pub_thread3.join();

  // (3) Quit() is thread-safe: it wakes up Run() from outside.
  loop.Quit();
  loop_thread.join();

  return 0;
}
