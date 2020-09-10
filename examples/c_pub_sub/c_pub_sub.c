//
// Created by fs on 2020-01-15.
//

#include <pthread.h>
#include <unistd.h>

#include "slog.h"
#include "uorb/topics/example_string.h"

void *thread_publisher(void *arg) {
  struct example_string_s example_string;
  orb_publication_t *pub_example_string =
      orb_create_publication(ORB_ID(example_string), 3);

  for (int i = 0; i < 10; i++) {
    snprintf((char *)example_string.string, EXAMPLE_STRING_STRING_LENGTH,
             "%d: %s", i, "This is a string message.");

    if (!orb_publish(pub_example_string, &example_string)) {
      LOGGER_ERROR("Publish error");
    }
    usleep(1 * 1000 * 1000);
  }

  orb_destroy_publication(&pub_example_string);
  LOGGER_WARN("Publication over.");

  return NULL;
}

void *thread_subscriber(void *unused) {
  orb_subscription_t *sub_example_string =
      orb_create_subscription(ORB_ID(example_string));

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#endif

  while (true) {
    struct orb_pollfd pollfds[] = {
        {.fd = sub_example_string, .events = POLLIN}};
    int timeout = 2000;
    if (0 < orb_poll(pollfds, ARRAY_SIZE(pollfds), timeout)) {
      struct example_string_s example_string;
      orb_copy(sub_example_string, &example_string);
      LOGGER_INFO("Receive msg: \"%s\"", example_string.string);
    } else {
      LOGGER_WARN("Got no data within %d milliseconds", timeout);
      break;
    }
  }

  orb_destroy_subscription(&sub_example_string);

  LOGGER_WARN("subscription over");
  return NULL;
}

int main(int argc, char *argv[]) {
  LOGGER_INFO("uORB version: %s", orb_version());

  // One publishing thread, three subscription threads
  pthread_t pthread_id;
  pthread_create(&pthread_id, NULL, thread_publisher, NULL);
  pthread_create(&pthread_id, NULL, thread_subscriber, NULL);
  pthread_create(&pthread_id, NULL, thread_subscriber, NULL);
  pthread_create(&pthread_id, NULL, thread_subscriber, NULL);

  // Wait for all threads to finish
  pthread_exit(NULL);
}
