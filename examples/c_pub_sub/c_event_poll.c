// Demonstrates the C orb_event_poll_* API with two subscriptions.

#include <pthread.h>
#include <unistd.h>

#include "slog.h"
#include "uorb/topics/example_string.h"
#include "uorb/topics/sensor_accel.h"

static void *publish_example_string(void *unused) {
  (void)unused;
  orb_publication_t *publisher = orb_create_publication(ORB_ID(example_string));

  for (int message_index = 0; message_index < 5; ++message_index) {
    struct example_string_s message = {0};
    message.timestamp = orb_absolute_time_us();
    snprintf((char *)message.str, EXAMPLE_STRING_STRING_LENGTH, "string %d", message_index);
    orb_publish(publisher, &message);
    usleep(200 * 1000);
  }

  orb_destroy_publication(&publisher);
  return NULL;
}

static void *publish_sensor_accel(void *unused) {
  (void)unused;
  orb_publication_t *publisher = orb_create_publication(ORB_ID(sensor_accel));

  for (int sample_index = 0; sample_index < 5; ++sample_index) {
    struct sensor_accel_s sample = {0};
    sample.timestamp = orb_absolute_time_us();
    sample.timestamp_sample = sample.timestamp;
    sample.device_id = 1;
    sample.x = sample_index * 0.1f;
    sample.y = sample_index * 0.2f;
    sample.z = sample_index * 0.3f;
    sample.temperature = 25.0f + sample_index;
    orb_publish(publisher, &sample);
    usleep(350 * 1000);
  }

  orb_destroy_publication(&publisher);
  return NULL;
}

int main(void) {
  LOGGER_INFO("uORB version: %s", orb_version());

  orb_subscription_t *string_sub = orb_create_subscription(ORB_ID(example_string));
  orb_subscription_t *accel_sub = orb_create_subscription(ORB_ID(sensor_accel));
  orb_event_poll_t *poll = orb_event_poll_create();

  if (!string_sub || !accel_sub || !poll) {
    LOGGER_ERROR("Failed to create subscriptions or event poll");
    orb_destroy_subscription(&string_sub);
    orb_destroy_subscription(&accel_sub);
    orb_event_poll_destroy(&poll);
    return 1;
  }

  orb_event_poll_add(poll, string_sub);
  orb_event_poll_add(poll, accel_sub);

  pthread_t string_publisher;
  pthread_t accel_publisher;
  pthread_create(&string_publisher, NULL, publish_example_string, NULL);
  pthread_create(&accel_publisher, NULL, publish_sensor_accel, NULL);

  for (;;) {
    orb_subscription_t *ready[2] = {NULL, NULL};
    const int ready_count = orb_event_poll_wait(poll, ready, 2, 1000);
    if (ready_count <= 0) {
      break;
    }

    for (int ready_index = 0; ready_index < ready_count; ++ready_index) {
      if (ready[ready_index] == string_sub) {
        struct example_string_s message;
        orb_copy(string_sub, &message);
        LOGGER_INFO("example_string: %s", message.str);
      } else if (ready[ready_index] == accel_sub) {
        struct sensor_accel_s sample;
        orb_copy(accel_sub, &sample);
        LOGGER_INFO("sensor_accel: (%.2f, %.2f, %.2f), temp: %.2f", sample.x,
                    sample.y, sample.z, sample.temperature);
      }
    }
  }

  pthread_join(string_publisher, NULL);
  pthread_join(accel_publisher, NULL);

  orb_event_poll_remove(poll, string_sub);
  orb_event_poll_remove(poll, accel_sub);
  orb_event_poll_destroy(&poll);
  orb_destroy_subscription(&string_sub);
  orb_destroy_subscription(&accel_sub);
  return 0;
}
