//
// Copyright (c) 2021 shawnfeng. All rights reserved.
//

#include <unistd.h>

#include <iostream>
#include <sstream>
#include <thread>

#include "uevent/uevent.h"
#include "uorb/publication.h"
#include "uorb/publication_multi.h"
#include "uorb/subscription.h"
#include "uorb/subscription_interval.h"
#include "uorb/topics/example_string.h"
#include "uorb/topics/msg_template.h"
#include "uorb/topics/sensor_accel.h"
#include "uorb/topics/sensor_gyro.h"
#include "uorb/topics/uorb_topics.h"
#include "uorb_uevent/uorb_uevent.h"
#include "uorb_tcp_listener.h"

template <const orb_metadata &T>
[[noreturn]] static void thread_publisher() {
  uorb::PublicationData<T> publication_data;

  while (true) {
    auto &data = publication_data.data();

    data.timestamp = orb_absolute_time_us();

    if (publication_data.Publish() != ORB_OK) {
      printf("Publish error\n");
    }

    usleep(1 * 1000 * 1000);
  }
  printf("Publication over.\n");
}

[[noreturn]] static void thread_publisher_sensor_accel() {
  uorb::PublicationData<uorb::msg::sensor_accel> publication_data;

  while (true) {
    auto &data = publication_data.data();

    data.timestamp = orb_absolute_time_us();
    data.timestamp_sample = data.timestamp;
    data.device_id = 10;
    data.x += 1;
    data.y += 2;
    data.z += 3;
    data.temperature += 4;

    if (publication_data.Publish() != ORB_OK) {
      printf("Publish error\n");
    }

    usleep(1 * 1000);
  }
  printf("Publication over.\n");
}

template <const orb_metadata &T>
[[noreturn]] static void thread_subscriber() {
  uorb::SubscriptionData<T> subscription_data;

  int timeout_ms = 2000;

  uevent_t poll = UEVENT_INITIALIZER;
  uevent_create(&poll);
  uevent_source_t src = UEVENT_SOURCE_INITIALIZER;
  uorb_subscriber_create_source(&src, subscription_data.handle());
  uevent_add(&poll, &src, 0);

  while (true) {
    uevent_source_t ready[1] = {UEVENT_SOURCE_INITIALIZER};
    if (0 < uevent_loop(&poll, ready, 1, timeout_ms)) {
      if (subscription_data.Update()) {
        //        auto data = sub_example_string.data();
        //        printf("timestamp: %" PRIu64 "[us]", data.timestamp "\n");
      }
    }
  }
}

int main(int, char *[]) {
  printf("uORB version: %s\n", orb_version() );

  std::thread{thread_publisher_sensor_accel}.detach();

  for (int i = 0; i < 3; ++i)
    std::thread{thread_publisher<uorb::msg::example_string>}.detach();

  for (int i = 0; i < 3; ++i)
    std::thread{thread_publisher<uorb::msg::sensor_accel>}.detach();
  for (int i = 0; i < 3; ++i)
    std::thread{thread_subscriber<uorb::msg::example_string>}.detach();

  for (int i = 0; i < 3; ++i)
    std::thread{thread_subscriber<uorb::msg::sensor_accel>}.detach();

  for (int i = 0; i < 3; ++i)
    std::thread{thread_subscriber<uorb::msg::sensor_gyro>}.detach();

  example_string_s example{};
  example.timestamp = orb_absolute_time_us();
  orb_publisher_publish_once(&uorb::msg::example_string, &example);
  orb_subscriber_copy_once(&uorb::msg::example_string, &example);

  orb_tcp_listener_init(orb_get_topics, 10924);

  // Wait for all threads to finish
  pthread_exit(nullptr);
}
