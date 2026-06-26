#include <stdio.h>
// Demonstrates the C uevent_* API with two subscriptions.

#include <pthread.h>
#include <unistd.h>

#include "uevent/uevent.h"
#include "uorb/topics/example_string.h"
#include "uorb/topics/sensor_accel.h"
#include "uorb_uevent/uorb_uevent.h"

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
  printf("uORB version: %s\n", orb_version() );

  orb_subscription_t *string_sub = orb_create_subscription(ORB_ID(example_string));
  orb_subscription_t *accel_sub = orb_create_subscription(ORB_ID(sensor_accel));
  uevent_t *base = uevent_create();

  if (!string_sub || !accel_sub || !base) {
    printf("Failed to create subscriptions or event base\n");
    orb_destroy_subscription(&string_sub);
    orb_destroy_subscription(&accel_sub);
    uevent_destroy(base);
    return 1;
  }

  uevent_source_t *string_src = uorb_subscription_create_source(string_sub);
  uevent_source_t *accel_src = uorb_subscription_create_source(accel_sub);
  if (!string_src || !accel_src) {
    printf("Failed to create event sources\n");
    if (string_src) uorb_subscription_destroy_source(string_src);
    if (accel_src) uorb_subscription_destroy_source(accel_src);
    uevent_destroy(base);
    orb_destroy_subscription(&string_sub);
    orb_destroy_subscription(&accel_sub);
    return 1;
  }
  if (uevent_add(base, string_src, 0) != 0 || uevent_add(base, accel_src, 0) != 0) {
    printf("Failed to add event sources\n");
    uorb_subscription_destroy_source(string_src);
    uorb_subscription_destroy_source(accel_src);
    uevent_destroy(base);
    orb_destroy_subscription(&string_sub);
    orb_destroy_subscription(&accel_sub);
    return 1;
  }

  pthread_t string_publisher;
  pthread_t accel_publisher;
  pthread_create(&string_publisher, NULL, publish_example_string, NULL);
  pthread_create(&accel_publisher, NULL, publish_sensor_accel, NULL);

  for (;;) {
    uevent_source_t *ready[2] = {NULL, NULL};
    const int ready_count = uevent_loop(base, ready, 2, 1000);
    if (ready_count <= 0) {
      break;
    }

    for (int ready_index = 0; ready_index < ready_count; ++ready_index) {
      if (ready[ready_index] == string_src) {
        struct example_string_s message = {0};
        if (orb_copy(string_sub, &message)) {
          printf("example_string: %s\n", message.str );
        }
      } else if (ready[ready_index] == accel_src) {
        struct sensor_accel_s sample = {0};
        if (orb_copy(accel_sub, &sample)) {
          printf("sensor_accel: (%.2f, %.2f, %.2f), temp: %.2f\n", sample.x,
                      sample.y, sample.z, sample.temperature);
        }
      }
    }
  }

  pthread_join(string_publisher, NULL);
  pthread_join(accel_publisher, NULL);

  uevent_remove(base, string_src);
  uevent_remove(base, accel_src);
  uorb_subscription_destroy_source(string_src);
  uorb_subscription_destroy_source(accel_src);
  uevent_destroy(base);
  orb_destroy_subscription(&string_sub);
  orb_destroy_subscription(&accel_sub);
  return 0;
}
