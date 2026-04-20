// Minimal uorb::EventLoop example.
//
// Demonstrates:
//   * Subscribe<Topic>(cb)  -- EventLoop owns the subscription.
//   * RegisterCallback(sub, cb) -- the caller owns the subscription.
//   * Quit() from another thread to stop the loop.
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

int main() {
  uorb::EventLoop loop;
  if (!loop) {
    LOGGER_ERROR("EventLoop create failed");
    return -1;
  }

  // (1) Loop-owned subscription: Subscribe<Topic>(callback).
  loop.Subscribe<uorb::msg::example_string>([](const example_string_s &msg) {
    LOGGER_INFO("[example_string] t=%" PRIu64 " msg='%s'", msg.timestamp,
                msg.str);
  });

  // (2) User-owned subscription: create the Subscription*Data yourself, then
  //     RegisterCallback(). Ownership stays with the caller; it must outlive
  //     the EventLoop (or be unregistered before destruction).
  uorb::SubscriptionData<uorb::msg::sensor_accel> sub_accel;
  loop.RegisterCallback(sub_accel, [](const sensor_accel_s &msg) {
    LOGGER_INFO("[sensor_accel] t=%" PRIu64 " xyz=(%.2f, %.2f, %.2f)",
                msg.timestamp, msg.x, msg.y, msg.z);
  });

  // Run the event loop on a worker thread. Loop() blocks until Quit().
  std::thread loop_thread([&] { loop.Loop(); });

  // Publish a few messages from the main thread.
  uorb::PublicationData<uorb::msg::example_string> pub_str;
  uorb::PublicationData<uorb::msg::sensor_accel> pub_accel;
  for (int i = 0; i < 5; ++i) {
    pub_str.get().timestamp = orb_absolute_time_us();
    snprintf(reinterpret_cast<char *>(pub_str.get().str),
             example_string_s::STRING_LENGTH, "hello #%d", i);
    pub_str.Publish();

    pub_accel.get().timestamp = orb_absolute_time_us();
    pub_accel.get().x = 0.1f * i;
    pub_accel.get().y = 0.2f * i;
    pub_accel.get().z = 0.3f * i;
    pub_accel.Publish();

    usleep(100 * 1000);  // 100 ms
  }

  // (3) Quit() is thread-safe: it wakes up Loop() from outside.
  loop.Quit();
  loop_thread.join();

  // Unregister the externally owned subscription before it goes out of scope
  // (optional here because the EventLoop will be destroyed next, but good
  // practice in longer-lived programs).
  loop.UnregisterCallback(sub_accel);

  return 0;
}
