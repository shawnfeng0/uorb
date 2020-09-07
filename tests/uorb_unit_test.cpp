/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "uorb_unit_test.hpp"

#include <gtest/gtest.h>
#include <uorb/abs_time.h>

#include <cerrno>
#include <cstdarg>
#include <cstdio>
#include <thread>
int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
namespace uORBTest {

TEST_F(UnitTest, unadvertise) {
  // try to advertise and see whether we get the right instance
  orb_publication_t *pfd[4]{};  ///< used for test_multi and test_multi_reversed
  unsigned instance_test[4]{};
  orb_test_s t{};

  for (int i = 0; i < 4; ++i) {
    pfd[i] = orb_create_publication_multi(ORB_ID(orb_multitest), &instance_test[i], 1);
    EXPECT_EQ(instance_test[i], i) << "got wrong instance";
    orb_publish(pfd[i], &t);
  }

  for (auto &i : pfd) {
    EXPECT_TRUE(orb_destroy_publication(&i));
  }
}

TEST_F(UnitTest, single_topic) {
  orb_test_s t{};
  orb_test_s u{};
  orb_publication_t *ptopic{};

  t.val = 0;
  ptopic = orb_create_publication_multi(ORB_ID(orb_test), nullptr, 1);

  ASSERT_NE(ptopic, nullptr) << "advertise failed:" << errno;

  orb_publish(ptopic, &t);

  auto sfd = orb_create_subscription_multi(ORB_ID(orb_test), 0);

  ASSERT_NE(sfd, nullptr) << "subscribe failed: " << errno;

  u.val = 1;

  ASSERT_TRUE(orb_copy(sfd, &u))
      << "copy(1) failed: " << errno;

  ASSERT_EQ(u.val, t.val) << "copy(1) mismatch";

  ASSERT_FALSE(orb_check_update(sfd)) << "spurious updated flag";

  t.val = 2;

  ASSERT_TRUE(orb_publish(ptopic, &t)) << "publish failed";

  ASSERT_TRUE(orb_check_update(sfd)) << "missing updated flag";

  ASSERT_TRUE(orb_copy(sfd, &u))
      << "copy(2) failed: " << errno;

  ASSERT_EQ(u.val, t.val) << "copy(2) mismatch";

  ASSERT_TRUE(orb_destroy_subscription(&sfd));

  ASSERT_TRUE(orb_destroy_publication(&ptopic)) << "orb_destroy_publication failed";
}

TEST_F(UnitTest, multi_topic) {
  orb_publication_t *pfd[4]{};  ///< used for test_multi and test_multi_reversed

  /* this routine tests the multi-topic support */
  {
    orb_test_s t{};
    orb_test_s u{};

    unsigned instance0;
    pfd[0] = orb_create_publication_multi(ORB_ID(orb_multitest), &instance0, 1);

    unsigned instance1;
    pfd[1] = orb_create_publication_multi(ORB_ID(orb_multitest), &instance1, 1);

    ASSERT_EQ(instance0, 0) << "mult. id0: " << instance0;

    ASSERT_EQ(instance1, 1) << "mult. id1: " << instance1;

    t.val = 103;

    ASSERT_TRUE(orb_publish(pfd[0], &t))
        << "mult. pub0 fail";

    t.val = 203;

    ASSERT_TRUE(orb_publish(pfd[1], &t))
        << "mult. pub1 fail";

    /* subscribe to both topics and ensure valid data is received */
    auto sfd0 = orb_create_subscription_multi(ORB_ID(orb_multitest), 0);

    ASSERT_TRUE(orb_copy(sfd0, &u))
        << "sub #0 copy failed: " << errno;

    ASSERT_EQ(u.val, 103) << "sub #0 val. mismatch: " << u.val;

    auto sfd1 = orb_create_subscription_multi(ORB_ID(orb_multitest), 1);

    ASSERT_TRUE(orb_copy(sfd1, &u))
        << "sub #1 copy failed: " << errno;

    ASSERT_EQ(u.val, 203) << "sub #1 val. mismatch: " << u.val;

    latency_test<orb_test_s>(ORB_ID(orb_test));

    orb_destroy_subscription(&sfd0);
    orb_destroy_subscription(&sfd1);
  }
  {
    /* For these tests 0 and 1 instances are taken from before, therefore
     * continue with 2 and 3. */

    // try multi-topic support subscribing before publishing

    /* Subscribe first and advertise afterwards. */
    auto sfd2 = orb_create_subscription_multi(ORB_ID(orb_multitest), 2);

    ASSERT_NE(sfd2, nullptr) << "errno: " << errno;

    orb_test_s t{};
    orb_test_s u{};

    t.val = 0;

    unsigned int instance2;
    pfd[2] = orb_create_publication_multi(ORB_ID(orb_multitest), &instance2, 1);
    ASSERT_EQ(instance2, 2) << "mult. id2: " << instance2;

    unsigned int instance3;
    pfd[3] = orb_create_publication_multi(ORB_ID(orb_multitest), &instance3, 1);
    ASSERT_EQ(instance3, 3) << "mult. id3: " << instance3;

    t.val = 204;
    ASSERT_TRUE(orb_publish(pfd[2], &t))
        << "mult. pub0 fail";

    t.val = 304;
    ASSERT_TRUE(orb_publish(pfd[3], &t))
        << "mult. pub1 fail";

    ASSERT_TRUE(orb_copy(sfd2, &u))
        << "sub #2 copy failed: " << errno;

    ASSERT_EQ(u.val, 204) << "sub #3 val. mismatch: " << u.val;

    auto sfd3 = orb_create_subscription_multi(ORB_ID(orb_multitest), 3);

    ASSERT_TRUE(orb_copy(sfd3, &u))
        << "sub #3 copy failed: " << errno;

    ASSERT_EQ(u.val, 304) << "sub #3 val. mismatch: " << u.val;
  }

  // we still have the advertisements from the previous test_multi calls.
  for (auto &i : pfd) {
    ASSERT_TRUE(orb_destroy_publication(&i)) << "orb_destroy_publication failed";
  }
}

TEST_F(UnitTest, multi_topic2_queue_simulation) {
  // test: first subscribe, then advertise

  volatile bool thread_should_exit = false;
  const int num_instances = 3;
  orb_subscription_t *orb_data_fd[num_instances]{};
  int orb_data_next = 0;

  for (unsigned i = 0; i < num_instances; ++i) {
    orb_data_fd[i] = orb_create_subscription_multi(ORB_ID(orb_test_medium_multi), i);
  }

  std::thread pub_test_multi2_main([&]() {
    int data_next_idx = 0;
    const int num_instances = 3;
    orb_publication_t *orb_pub[num_instances]{};
    orb_test_medium_s data_topic{};

    for (int i = 0; i < num_instances; ++i) {
      orb_publication_t *&pub = orb_pub[i];
      unsigned idx = i;
      //		PX4_WARN("advertise %i, t=%" PRIu64, i,
      // orb_absolute_time_us());
      pub = orb_create_publication_multi(ORB_ID(orb_test_medium_multi), &idx, 1);

      if (idx != i) {
        thread_should_exit = true;
        ORB_ERROR("Got wrong instance! should be: %i, but is %i", i, idx);
        return -1;
      }
    }

    usleep(100 * 1000);

    int message_counter = 0;
    int num_messages = 50 * num_instances;

    while (message_counter++ < num_messages) {
      usleep(2);  // make sure the timestamps are different
      orb_publication_t *&pub = orb_pub[data_next_idx];

      data_topic.timestamp = orb_absolute_time_us();
      data_topic.val = data_next_idx;

      orb_publish(pub, &data_topic);
      //		PX4_WARN("publishing msg (idx=%i, t=%" PRIu64 ")",
      // data_next_idx, data_topic.time);

      data_next_idx = (data_next_idx + 1) % num_instances;

      if (data_next_idx == 0) {
        usleep(50 * 1000);
      }
    }

    usleep(100 * 1000);
    thread_should_exit = true;

    for (auto &i : orb_pub) {
      orb_destroy_publication(&i);
    }
    return 0;
  });
  pub_test_multi2_main.detach();

  orb_abstime_us last_time = 0;

  while (!thread_should_exit) {
    usleep(1000);

    auto orb_data_cur_fd = orb_data_fd[orb_data_next];

    if (orb_check_update(orb_data_cur_fd)) {
      orb_test_medium_s msg{};
      orb_copy(orb_data_cur_fd, &msg);

      if (last_time != 0) {
        ASSERT_LT(last_time, msg.timestamp) << "Timestamp not increasing!";
      }
      last_time = msg.timestamp;

      ORB_DEBUG("got message (val=%i, idx=%i, t=%" PRIu64 ")", msg.val,
                orb_data_next, msg.timestamp);
      orb_data_next = (orb_data_next + 1) % num_instances;
    }
  }

  for (auto &i : orb_data_fd) {
    orb_destroy_subscription(&i);
  }
}  // namespace uORBTest

TEST_F(UnitTest, queue) {
  orb_test_medium_s t{};
  orb_test_medium_s u{};
  orb_publication_t *ptopic{nullptr};

  auto sfd = orb_create_subscription_multi(ORB_ID(orb_test_medium_queue), 0);

  ASSERT_NE(sfd, nullptr) << "subscribe failed: " << errno;

  const int queue_size = 8;
  t.val = 0;
  ptopic =
      orb_create_publication_multi(ORB_ID(orb_test_medium_queue), nullptr, queue_size);
  ASSERT_NE(ptopic, nullptr) << "advertise failed: " << errno;

  orb_publish(ptopic, &t);

  ASSERT_TRUE(orb_check_update(sfd)) << "update flag not set";

  ASSERT_TRUE(orb_copy(sfd, &u))
      << "copy(1) failed: " << errno;

  ASSERT_EQ(u.val, t.val) << "copy(1) mismatch";

  ASSERT_FALSE(orb_check_update(sfd)) << "spurious updated flag";

  // no messages in the queue anymore

  //  Testing to write some elements...

  for (int i = 0; i < queue_size - 2; ++i) {
    t.val = i;
    orb_publish(ptopic, &t);
  }

  for (int i = 0; i < queue_size - 2; ++i) {
    ASSERT_TRUE(orb_check_update(sfd)) << "update flag not set, element " << i;
    orb_copy(sfd, &u);
    ASSERT_EQ(u.val, i) << "got wrong element from the queue (got" << u.val
                        << "should be" << i << ")";
  }

  ASSERT_FALSE(orb_check_update(sfd))
      << "update flag set, element " << queue_size;

  //  Testing overflow...
  int overflow_by = 3;

  for (int i = 0; i < queue_size + overflow_by; ++i) {
    t.val = i;
    orb_publish(ptopic, &t);
  }

  for (int i = 0; i < queue_size; ++i) {
    ASSERT_TRUE(orb_check_update(sfd)) << "update flag not set, element " << i;
    orb_copy(sfd, &u);
    ASSERT_EQ(u.val, i + overflow_by)
        << "got wrong element from the queue (got " << u.val << "should be"
        << i + overflow_by << ")";
  }

  ASSERT_FALSE(orb_check_update(sfd))
      << "update flag set, element " << queue_size;

  //  Testing underflow...

  for (int i = 0; i < queue_size; ++i) {
    ASSERT_FALSE(orb_check_update(sfd)) << "update flag set, element " << i;
    orb_copy(sfd, &u);
    ASSERT_EQ(u.val, queue_size + overflow_by - 1)
        << "got wrong element from the queue (got " << u.val << ", should be "
        << queue_size + overflow_by - 1 << ")";
  }

  t.val = 943;
  orb_publish(ptopic, &t);
  ASSERT_TRUE(orb_check_update(sfd)) << "update flag not set, element " << -1;

  orb_copy(sfd, &u);
  ASSERT_EQ(u.val, t.val) << "got wrong element from the queue (got " << u.val
                          << ", should be " << t.val << ")";

  ASSERT_TRUE(orb_destroy_publication(&ptopic));
}

TEST_F(UnitTest, queue_poll_notify) {
  orb_test_medium_s t{};
  orb_subscription_t *sfd;
  volatile int num_messages_sent = 0;
  volatile bool thread_should_exit = false;

  ASSERT_NE(sfd = orb_create_subscription_multi(ORB_ID(orb_test_medium_queue_poll), 0),
            nullptr)
      << "subscribe failed: " << errno;

  std::thread test_queue_thread{[&]() {
    orb_test_medium_s t{};
    orb_publication_t *ptopic{nullptr};
    const int queue_size = 50;
    ptopic = orb_create_publication_multi(ORB_ID(orb_test_medium_queue_poll), nullptr,
                                  queue_size);
    if (ptopic == nullptr) {
      thread_should_exit = true;
    }
    ASSERT_NE(ptopic, nullptr) << "advertise failed: " << errno;

    int message_counter = 0;
    int num_messages = 20 * queue_size;

    while (message_counter < num_messages) {
      // simulate burst
      int burst_counter = 0;

      while (burst_counter++ <
             queue_size / 2 + 7) {  // make interval non-boundary aligned
        orb_publish(ptopic, &t);
        ++t.val;
      }

      message_counter += burst_counter;
      usleep(20 * 1000);  // give subscriber a chance to catch up
    }

    num_messages_sent = t.val;
    usleep(100 * 1000);
    thread_should_exit = true;
    orb_destroy_publication(&ptopic);
  }};
  test_queue_thread.detach();

  int next_expected_val = 0;
  orb_pollfd_t fds[1]{};
  fds[0].fd = sfd;
  fds[0].events = POLLIN;

  while (!thread_should_exit) {
    int poll_ret = orb_poll(fds, 1, 500);
    ASSERT_GE(poll_ret, 0) << "poll error (" << poll_ret << "," << errno << ")";

    if (thread_should_exit) {
      break;
    }

    ASSERT_NE(poll_ret, 0) << "poll timeout";

    if (fds[0].revents & POLLIN) {
      orb_copy(sfd, &t);
      ASSERT_EQ(next_expected_val, t.val) << "copy mismatch";
      ++next_expected_val;
    }
  }

  ASSERT_EQ(num_messages_sent, next_expected_val)
      << "number of sent and received messages mismatch";
}

int uORBTest::UnitTest::test_note(const char *fmt, ...) {
  va_list ap;

  fprintf(stderr, "uORB note: ");
  va_start(ap, fmt);
  vfprintf(stderr, fmt, ap);
  va_end(ap);
  fprintf(stderr, "\n");
  fflush(stderr);
  return true;
}

}  // namespace uORBTest
