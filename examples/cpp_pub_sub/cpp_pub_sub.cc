//
// Created by fs on 9/7/20.
//

#include <pthread.h>
#include <unistd.h>
#include <uorb/publication.h>
#include <uorb/subscription.h>
#include <uorb/topics/example_string.h>

#include "slog.h"

void *thread_publisher(void *arg) {
  uorb::PublicationData<ORB::example_string, 3> pub_example_string;

  for (int i = 0; i < 10; i++) {
    snprintf((char *)pub_example_string.get().string,
             example_string_s::STRING_LENGTH, "%d: %s", i,
             "This is a string message.");

    if (!pub_example_string.publish()) {
      LOGGER_ERROR("Publish error");
    }

    usleep(1 * 1000 * 1000);
  }
  LOGGER_WARN("Publication over.");

  return nullptr;
}

void *thread_subscriber(void *unused) {
  uorb::SubscriptionData<ORB::example_string> sub_example_string;

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#endif

  const int period_ms = 20;
  int timeout_ms = 0;
  while (true) {
    if (sub_example_string.update()) {
      timeout_ms = 0;
      LOGGER_INFO("Receive msg: \"%s\"", sub_example_string.get().string);
    } else {
      timeout_ms += period_ms;
      if (timeout_ms >= 2000) {
        LOGGER_WARN("Got no data within %d milliseconds", 2000);
        break;
      }
    }
    usleep(period_ms * 1000);
  }

  LOGGER_WARN("subscription over");
  return nullptr;
}

int main(int argc, char *argv[]) {
  LOGGER_INFO("uORB version: %s", orb_version());

  // One publishing thread, three subscription threads
  pthread_t pthread_id;
  pthread_create(&pthread_id, nullptr, thread_publisher, nullptr);
  pthread_create(&pthread_id, nullptr, thread_subscriber, nullptr);
  pthread_create(&pthread_id, nullptr, thread_subscriber, nullptr);
  pthread_create(&pthread_id, nullptr, thread_subscriber, nullptr);

  // Wait for all threads to finish
  pthread_exit(nullptr);
}
