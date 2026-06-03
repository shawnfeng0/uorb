// Demonstrates SubscriptionInterval throttling with the C++ helpers.

#include <inttypes.h>
#include <unistd.h>

#include <thread>

#include "slog.h"
#include "uorb/publication.h"
#include "uorb/subscription_interval.h"
#include "uorb/topics/sensor_accel.h"

void publish_accel_samples() {
  uorb::PublicationData<uorb::msg::sensor_accel> publisher;

  for (int sample_index = 0; sample_index < 20; ++sample_index) {
    auto &sample = publisher.data();
    sample.timestamp = orb_absolute_time_us();
    sample.timestamp_sample = sample.timestamp;
    sample.device_id = 1;
    sample.x = sample_index * 0.1f;
    sample.y = sample_index * 0.2f;
    sample.z = sample_index * 0.3f;
    sample.temperature = 25.0f + sample_index;

    if (!publisher.Publish()) {
      LOGGER_ERROR("Publish sensor_accel failed");
      return;
    }

    usleep(100 * 1000);
  }
}

int main() {
  LOGGER_INFO("uORB version: %s", orb_version());

  uorb::SubscriptionInterval<uorb::msg::sensor_accel> subscription(500 * 1000);
  orb_pollfd_t poll_fds[] = {{.fd = subscription.handle()}};

  std::thread publisher(publish_accel_samples);

  for (;;) {
    const int poll_result = orb_poll(poll_fds, 1, 1000);
    if (poll_result <= 0) {
      break;
    }

    sensor_accel_s sample{};
    if (subscription.Update(sample)) {
      LOGGER_INFO("timestamp: %" PRIu64 ", accel: (%.2f, %.2f, %.2f), temp: %.2f",
                  sample.timestamp, sample.x, sample.y, sample.z,
                  sample.temperature);
    }
  }

  publisher.join();
  return 0;
}
