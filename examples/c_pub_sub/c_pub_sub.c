#include <stdio.h>
//
// Created by fs on 2020-01-15.
//

#include <pthread.h>
#include <unistd.h>

#include "uevent/uevent.h"
#include "uorb/topics/example_string.h"
#include "uorb_uevent/uorb_uevent.h"

void *thread_publisher(void *arg) {
  (void)arg;
  struct example_string_s example_string;
  orb_publisher_t pub_example_string = ORB_PUBLISHER_INITIALIZER;
  orb_publisher_create(&pub_example_string, ORB_ID(example_string));

  for (int i = 0; i < 10; i++) {
    snprintf((char *)example_string.str, EXAMPLE_STRING_STRING_LENGTH, "%d: %s",
             i, "This is a string message.");

    if (orb_publisher_publish(&pub_example_string, &example_string) != ORB_OK) {
      printf("Publish error\n");
    }
    usleep(1 * 1000 * 1000);
  }

  orb_publisher_destroy(&pub_example_string);
  printf("Publication over.\n");

  return NULL;
}

void *thread_subscriber(void *unused) {
  (void)unused;
  orb_subscriber_t sub_example_string = ORB_SUBSCRIBER_INITIALIZER;
  orb_subscriber_create(&sub_example_string, ORB_ID(example_string));

  uevent_t poll = UEVENT_INITIALIZER;
  uevent_create(&poll);
  uevent_source_t src = UEVENT_SOURCE_INITIALIZER;
  uorb_subscriber_create_source(&src, &sub_example_string);
  uevent_add(&poll, &src, 0);
  int timeout = 2000;

  while (true) {
    uevent_source_t ready[1] = {UEVENT_SOURCE_INITIALIZER};
    if (0 < uevent_loop(&poll, ready, 1, timeout)) {
      struct example_string_s example_string;
      orb_subscriber_copy(&sub_example_string, &example_string);
      printf("Receive msg: \"%s\"\n", example_string.str);
    } else {
      printf("Got no data within %d milliseconds\n", timeout);
      break;
    }
  }

  uevent_remove(&poll, &src);
  uorb_subscriber_destroy_source(&src);
  uevent_destroy(&poll);
  orb_subscriber_destroy(&sub_example_string);

  printf("subscription over\n");
  return NULL;
}

int main() {
  printf("uORB version: %s\n", orb_version());

  // One publishing thread, three subscription threads
  pthread_t pthread_id;
  pthread_create(&pthread_id, NULL, thread_publisher, NULL);
  pthread_detach(pthread_id);
  pthread_create(&pthread_id, NULL, thread_subscriber, NULL);
  pthread_detach(pthread_id);
  pthread_create(&pthread_id, NULL, thread_subscriber, NULL);
  pthread_detach(pthread_id);
  pthread_create(&pthread_id, NULL, thread_subscriber, NULL);
  pthread_detach(pthread_id);

  // Detached threads will exit when they complete their work.
  // pthread_exit(NULL) exits the calling thread (main), allowing
  // the detached threads to continue running until they finish.
  pthread_exit(NULL);
}
