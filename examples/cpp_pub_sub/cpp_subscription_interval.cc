// Demonstrates SubscriptionInterval throttling with the C++ helpers.

#include <inttypes.h>
#include <unistd.h>

#include <thread>

#include "uevent/uevent.h"
#include "uorb/publication.h"
#include "uorb/subscription_interval.h"
#include "uorb/topics/sensor_accel.h"
#include "uorb_uevent/uorb_uevent.h"

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
      printf("Publish sensor_accel failed\n");
      return;
    }

    usleep(100 * 1000);
  }
}

int main() {
  printf("uORB version: %s\n", orb_version() );

  uorb::SubscriptionInterval<uorb::msg::sensor_accel> subscription(500 * 1000);
  uevent_t *poll = uevent_create();
  uevent_source_t *src = uorb_subscription_create_source(subscription.handle());
  uevent_add(poll, src, 0);

  std::thread publisher(publish_accel_samples);

  for (;;) {
    uevent_source_t *ready[1];
    const int poll_result = uevent_loop(poll, ready, 1, 1000);
    if (poll_result <= 0) {
      break;
    }

    sensor_accel_s sample{};
    if (subscription.Update(sample)) {
      printf("timestamp: %" PRIu64 ", accel: (%.2f, %.2f, %.2f), temp: %.2f\n",
                  sample.timestamp, sample.x, sample.y, sample.z,
                  sample.temperature);
    }
  }

  publisher.join();
  uevent_remove(poll, src);
  uorb_subscription_destroy_source(src);
  uevent_destroy(poll);
  return 0;
}
