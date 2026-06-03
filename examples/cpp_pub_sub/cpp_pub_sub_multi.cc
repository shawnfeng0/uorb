// Demonstrates multi-instance publishing and subscribing with the C++ helpers.

#include <inttypes.h>
#include <unistd.h>

#include <cstdio>
#include <thread>

#include "slog.h"
#include "uorb/publication_multi.h"
#include "uorb/subscription.h"
#include "uorb/topics/example_string.h"

void publish_instance(const char *label) {
  uorb::PublicationMultiData<uorb::msg::example_string> publisher;

  for (int message_index = 0; message_index < 5; ++message_index) {
    auto &message = publisher.data();
    message.timestamp = orb_absolute_time_us();
    snprintf(reinterpret_cast<char *>(message.str), example_string_s::STRING_LENGTH,
             "%s message %d", label, message_index);

    if (!publisher.Publish()) {
      LOGGER_ERROR("Publish %s failed", label);
      return;
    }

    LOGGER_INFO("Published %s on instance %u", label, publisher.instance());
    usleep(200 * 1000);
  }
}

void subscribe_instance(uint8_t instance) {
  uorb::SubscriptionData<uorb::msg::example_string> subscription(instance);
  orb_pollfd_t poll_fds[] = {{.fd = subscription.handle()}};

  for (;;) {
    const int poll_result = orb_poll(poll_fds, 1, 1000);
    if (poll_result <= 0) {
      break;
    }

    if (subscription.Update()) {
      const auto &message = subscription.data();
      LOGGER_INFO("instance %u timestamp: %" PRIu64 ", msg: %s", instance,
                  message.timestamp, message.str);
    }
  }
}

int main() {
  LOGGER_INFO("uORB version: %s", orb_version());

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
