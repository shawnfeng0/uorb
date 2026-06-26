//
// Copyright (c) 2021 shawnfeng. All rights reserved.
//

#include <pthread.h>
#include <unistd.h>

#include "uevent/uevent.h"
#include "uorb/publication.h"
#include "uorb/publication_multi.h"
#include "uorb/subscription.h"
#include "uorb/subscription_interval.h"
#include "uorb/topics/example_string.h"
#include "uorb_uevent/uorb_uevent.h"

void *thread_publisher(void *unused) {
  (void)unused;
  uorb::PublicationData<uorb::msg::example_string> pub_example_string;

  for (int i = 0; i < 10; i++) {
    auto &data = pub_example_string.data();

    data.timestamp = orb_absolute_time_us();
    snprintf(reinterpret_cast<char *>(data.str),
             example_string_s::STRING_LENGTH, "%d: %s", i,
             "This is a string message.");

    if (!pub_example_string.Publish()) {
      printf("Publish error\n");
    }

    usleep(1 * 1000 * 1000);
  }
  printf("Publication over.\n");

  return nullptr;
}

void *thread_subscriber(void *unused) {
  (void)unused;
  uorb::SubscriptionData<uorb::msg::example_string> sub_example_string;

  int timeout_ms = 2000;

  uevent_t *poll = uevent_create();
  uevent_source_t *src = uorb_subscription_create_source(sub_example_string.handle());
  uevent_add(poll, src, 0);

  while (true) {
    uevent_source_t *ready[1];
    if (0 < uevent_loop(poll, ready, 1, timeout_ms)) {
      if (sub_example_string.Update()) {
        auto data = sub_example_string.data();
        printf("timestamp: %" PRIu64 "[us], Receive msg: \"%s\"\n",
                    data.timestamp, data.str);
      }
    } else {
      printf("Got no data within %d milliseconds\n", timeout_ms );
      break;
    }
  }

  uevent_remove(poll, src);
  uorb_subscription_destroy_source(src);
  uevent_destroy(poll);

  printf("subscription over\n");
  return nullptr;
}

int main(int, char *[]) {
  printf("uORB version: %s\n", orb_version() );

  // One publishing thread, three subscription threads
  pthread_t pthread_id;
  pthread_create(&pthread_id, nullptr, thread_publisher, nullptr);
  pthread_detach(pthread_id);

  pthread_create(&pthread_id, nullptr, thread_subscriber, nullptr);
  pthread_detach(pthread_id);

  pthread_create(&pthread_id, nullptr, thread_subscriber, nullptr);
  pthread_detach(pthread_id);

  pthread_create(&pthread_id, nullptr, thread_subscriber, nullptr);
  pthread_detach(pthread_id);

  // Wait for all threads to finish
  pthread_exit(nullptr);
}
