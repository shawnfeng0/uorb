// uorb::EventLoop example.
//
// Demonstrates:
//   * Subscribe<Topic>(cb)        -- EventLoop owns the subscription.
//   * RegisterCallback(sub, cb)   -- the caller owns the subscription.
//   * Multiple publisher threads on different topics.
//   * Quit() from another thread to stop the loop.
//   * UnregisterCallback() on teardown.
//
// Copyright (c) 2021-2025 shawnfeng. All rights reserved.

#include <inttypes.h>
#include <unistd.h>

#include <cstdio>
#include <thread>

#include "slog.h"
#include "uorb/abs_time.h"
#include "uorb/event_loop.h"
#include "uorb/publication.h"
#include "uorb/subscription.h"
#include "uorb/topics/example_string.h"
#include "uorb/topics/sensor_accel.h"
#include "uorb/topics/sensor_gyro.h"

void thread_publisher_example_string() {
  uorb::PublicationData<uorb::msg::example_string> pub_example_string;
  for (int i = 0; i < 5; i++) {
    auto &data = pub_example_string.get();
    data.timestamp = orb_absolute_time_us();
    snprintf(reinterpret_cast<char *>(data.str), example_string_s::STRING_LENGTH, "%d: %s", i,
             "This is a string message. ");
    if (!pub_example_string.Publish()) {
      LOGGER_ERROR("Publish example_string error");
    }
    usleep(2000 * 1000);  // 2 seconds
  }
  LOGGER_WARN("example_string publication over.");
}

void thread_publisher_sensor_accel() {
  uorb::PublicationData<uorb::msg::sensor_accel> pub_sensor_accel;
  for (int i = 0; i < 20; i++) {
    auto &accel = pub_sensor_accel.get();
    accel.timestamp = orb_absolute_time_us();
    accel.x = i * 0.1f;
    accel.y = i * 0.2f;
    accel.z = i * 0.3f;
    accel.temperature = 25.0f + i;
    if (!pub_sensor_accel.Publish()) {
      LOGGER_ERROR("Publish sensor_accel error");
    }
    usleep(300 * 1000);  // 300 ms
  }
  LOGGER_WARN("sensor_accel publication over.");
}

void thread_publisher_sensor_gyro() {
  uorb::PublicationData<uorb::msg::sensor_gyro> pub_sensor_gyro;
  for (int i = 0; i < 10; i++) {
    auto &gyro = pub_sensor_gyro.get();
    gyro.timestamp = orb_absolute_time_us();
    gyro.x = i * 1.1f;
    gyro.y = i * 1.2f;
    gyro.z = i * 1.3f;
    gyro.temperature = 30.0f + i;
    if (!pub_sensor_gyro.Publish()) {
      LOGGER_ERROR("Publish sensor_gyro error");
    }
    usleep(1000 * 1000);  // 1 second
  }
  LOGGER_WARN("sensor_gyro publication over.");
}

int main() {
  uorb::EventLoop loop;
  if (!loop) {
    LOGGER_ERROR("EventLoop create failed");
    return -1;
  }

  // (1) Loop-owned subscriptions: Subscribe<Topic>(callback).
  loop.Subscribe<uorb::msg::example_string>([](const example_string_s &msg) {
    LOGGER_INFO("[example_string] timestamp: %" PRIu64 ", msg: '%s'", msg.timestamp, msg.str);
  });
  loop.Subscribe<uorb::msg::sensor_accel>([](const sensor_accel_s &msg) {
    LOGGER_INFO("[sensor_accel] timestamp: %" PRIu64 ", accel: (%.2f, %.2f, %.2f), temp: %.2f", msg.timestamp, msg.x,
                msg.y, msg.z, msg.temperature);
  });

  // (2) User-owned subscription: create the Subscription*Data yourself, then
  //     RegisterCallback(). Ownership stays with the caller; it must outlive
  //     the EventLoop (or be unregistered before destruction).
  uorb::SubscriptionData<uorb::msg::sensor_gyro> sub_gyro;
  loop.RegisterCallback(sub_gyro, [](const sensor_gyro_s &msg) {
    LOGGER_INFO("[sensor_gyro] timestamp: %" PRIu64 ", gyro: (%.2f, %.2f, %.2f), temp: %.2f", msg.timestamp, msg.x,
                msg.y, msg.z, msg.temperature);
  });

  // Run the event loop on a worker thread. Loop() blocks until Quit().
  std::thread loop_thread([&] { loop.Loop(); });

  // Publish from multiple threads.
  std::thread pub_thread1(thread_publisher_example_string);
  std::thread pub_thread2(thread_publisher_sensor_accel);
  std::thread pub_thread3(thread_publisher_sensor_gyro);
  pub_thread1.join();
  pub_thread2.join();
  pub_thread3.join();

  // (3) Quit() is thread-safe: it wakes up Loop() from outside.
  loop.Quit();
  loop_thread.join();

  // Unregister the externally owned subscription before it goes out of scope
  // (optional here because the EventLoop will be destroyed next, but good
  // practice in longer-lived programs).
  loop.UnregisterCallback(sub_gyro);

  return 0;
}
