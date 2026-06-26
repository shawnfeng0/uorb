// Demonstrates multi-instance publishing and subscribing with the C++ helpers.

#include <inttypes.h>
#include <unistd.h>

#include <cstdio>
#include <thread>

#include "uevent/uevent.h"
#include "uorb/publication_multi.h"
#include "uorb/subscription.h"
#include "uorb/topics/example_string.h"
#include "uorb_uevent/uorb_uevent.h"

void publish_instance(const char *label) {
  uorb::PublicationMultiData<uorb::msg::example_string> publisher;

  for (int message_index = 0; message_index < 5; ++message_index) {
    auto &message = publisher.data();
    message.timestamp = orb_absolute_time_us();
    snprintf(reinterpret_cast<char *>(message.str), example_string_s::STRING_LENGTH,
             "%s message %d", label, message_index);

    if (!publisher.Publish()) {
      printf("Publish %s failed\n", label );
      return;
    }

    printf("Published %s on instance %u\n", label, publisher.instance() );
    usleep(200 * 1000);
  }
}

void subscribe_instance(uint8_t instance) {
  uorb::SubscriptionData<uorb::msg::example_string> subscription(instance);
  uevent_t poll = UEVENT_INITIALIZER;
  uevent_create(&poll);
  uevent_source_t src = UEVENT_SOURCE_INITIALIZER;
  uorb_subscriber_create_source(&src, subscription.handle());
  uevent_add(&poll, &src, 0);

  for (;;) {
    uevent_source_t ready[1] = {UEVENT_SOURCE_INITIALIZER};
    const int poll_result = uevent_loop(&poll, ready, 1, 1000);
    if (poll_result <= 0) {
      break;
    }

    if (subscription.Update()) {
      const auto &message = subscription.data();
      printf("instance %u timestamp: %" PRIu64 ", msg: %s\n", instance,
                  message.timestamp, message.str);
    }
  }

  uevent_remove(&poll, &src);
  uorb_subscriber_destroy_source(&src);
  uevent_destroy(&poll);
}

int main() {
  printf("uORB version: %s\n", orb_version() );

  std::thread first_subscriber(subscribe_instance, 0);
  std::thread second_subscriber(subscribe_instance, 1);
  usleep(100 * 1000);

  std::thread first_publisher(publish_instance, "first");
  std::thread second_publisher(publish_instance, "second");

  first_publisher.join();
  second_publisher.join();
  first_subscriber.join();
  second_subscriber.join();
  return 0;
}
