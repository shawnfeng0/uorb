//
// Copyright (c) 2021 shawnfeng. All rights reserved.
//

#include <unistd.h>

#include <iostream>
#include <sstream>
#include <thread>

#include "slog.h"
#include "uorb/publication.h"
#include "uorb/publication_multi.h"
#include "uorb/subscription.h"
#include "uorb/subscription_interval.h"
#include "uorb/topics/example_string.h"
#include "uorb/topics/msg_template.h"
#include "uorb/topics/sensor_accel.h"
#include "uorb/topics/sensor_gyro.h"
#include "uorb/topics/uorb_topics.h"
#include "uorb_tcp_listener.h"

template <const orb_metadata &T>
[[noreturn]] static void thread_publisher() {
  uorb::PublicationData<T> publication_data;

  while (true) {
    auto &data = publication_data.get();

    data.timestamp = orb_absolute_time_us();

    if (!publication_data.Publish()) {
      LOGGER_ERROR("Publish error");
    }

    usleep(1 * 1000 * 1000);
  }
  LOGGER_WARN("Publication over.");
}

template <const orb_metadata &T>
[[noreturn]] static void thread_subscriber() {
  uorb::SubscriptionData<T> subscription_data;

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#endif

  int timeout_ms = 2000;

  struct orb_pollfd poll_fds[] = {
      {.fd = subscription_data.handle(), .events = POLLIN, .revents = 0}};

  while (true) {
    if (0 < orb_poll(poll_fds, ARRAY_SIZE(poll_fds), timeout_ms)) {
      if (subscription_data.Update()) {
        //        auto data = sub_example_string.get();
        //        LOGGER_INFO("timestamp: %" PRIu64 "[us]", data.timestamp);
      }
    }
  }
}

int main(int, char *[]) {
  LOGGER_INFO("uORB version: %s", orb_version());

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
  orb_publish_anonymous(&uorb::msg::example_string, &example);
  orb_copy_anonymous(&uorb::msg::example_string, &example);

  orb_tcp_listener_init(orb_get_topics);

  // Wait for all threads to finish
  pthread_exit(nullptr);
}
