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

#include "uorb_unit_test.h"

#include <gtest/gtest.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <thread>
#include <vector>

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
    pfd[i] =
        orb_create_publication_multi(ORB_ID(orb_multitest), &instance_test[i]);
    EXPECT_EQ(instance_test[i], i) << "got wrong instance";
    orb_publish(pfd[i], &t);
  }

  for (auto &i : pfd) {
    EXPECT_TRUE(orb_destroy_publication(&i));
  }
}

TEST_F(UnitTest, publication_multi_reports_eexist_when_instances_are_full) {
  orb_publication_t *publications[ORB_MULTI_MAX_INSTANCES]{};
  unsigned instances[ORB_MULTI_MAX_INSTANCES]{};

  for (unsigned index = 0; index < ORB_MULTI_MAX_INSTANCES; ++index) {
    publications[index] = orb_create_publication_multi(ORB_ID(orb_test_medium), &instances[index]);
    ASSERT_NE(publications[index], nullptr);
    EXPECT_EQ(instances[index], index);
  }

  unsigned extra_instance = 0;
  errno = 0;
  EXPECT_EQ(orb_create_publication_multi(ORB_ID(orb_test_medium), &extra_instance), nullptr);
  EXPECT_EQ(errno, EEXIST);

  for (auto *publication : publications) {
    EXPECT_TRUE(orb_destroy_publication(&publication));
  }
}

TEST_F(UnitTest, subscription_multi_reports_einval_for_invalid_instance) {
  errno = 0;
  orb_subscription_t *subscription = orb_create_subscription_multi(ORB_ID(orb_test), ORB_MULTI_MAX_INSTANCES);
  EXPECT_EQ(subscription, nullptr);
  EXPECT_EQ(errno, EINVAL);
}

TEST_F(UnitTest, rejects_null_arguments) {
  orb_test_s msg{};
  orb_pollfd_t fds[1]{};

  errno = 0;
  EXPECT_EQ(orb_create_publication(nullptr), nullptr);
  EXPECT_EQ(errno, EINVAL);

  errno = 0;
  EXPECT_EQ(orb_create_publication_multi(nullptr, nullptr), nullptr);
  EXPECT_EQ(errno, EINVAL);

  errno = 0;
  EXPECT_EQ(orb_create_subscription(nullptr), nullptr);
  EXPECT_EQ(errno, EINVAL);

  errno = 0;
  EXPECT_EQ(orb_create_subscription_multi(nullptr, 0), nullptr);
  EXPECT_EQ(errno, EINVAL);

  errno = 0;
  EXPECT_FALSE(orb_destroy_publication(nullptr));
  EXPECT_EQ(errno, EINVAL);

  errno = 0;
  EXPECT_FALSE(orb_destroy_subscription(nullptr));
  EXPECT_EQ(errno, EINVAL);

  errno = 0;
  EXPECT_FALSE(orb_publish(nullptr, &msg));
  EXPECT_EQ(errno, EINVAL);

  auto *publication = orb_create_publication(ORB_ID(orb_test));
  ASSERT_NE(publication, nullptr);

  errno = 0;
  EXPECT_FALSE(orb_publish(publication, nullptr));
  EXPECT_EQ(errno, EINVAL);

  errno = 0;
  EXPECT_FALSE(orb_publish_once(nullptr, &msg));
  EXPECT_EQ(errno, EINVAL);

  errno = 0;
  EXPECT_FALSE(orb_publish_once(ORB_ID(orb_test), nullptr));
  EXPECT_EQ(errno, EFAULT);

  errno = 0;
  EXPECT_FALSE(orb_copy(nullptr, &msg));
  EXPECT_EQ(errno, EINVAL);

  auto *subscription = orb_create_subscription(ORB_ID(orb_test));
  ASSERT_NE(subscription, nullptr);

  errno = 0;
  EXPECT_FALSE(orb_copy(subscription, nullptr));
  EXPECT_EQ(errno, EINVAL);

  errno = 0;
  EXPECT_FALSE(orb_copy_once(nullptr, &msg));
  EXPECT_EQ(errno, EINVAL);

  errno = 0;
  EXPECT_FALSE(orb_copy_once(ORB_ID(orb_test), nullptr));

  errno = 0;
  EXPECT_FALSE(orb_check_update(nullptr));
  EXPECT_EQ(errno, EINVAL);

  errno = 0;
  EXPECT_FALSE(orb_exists(nullptr, 0));
  EXPECT_EQ(errno, EINVAL);

  errno = 0;
  EXPECT_EQ(orb_group_count(nullptr), 0U);
  EXPECT_EQ(errno, EINVAL);

  errno = 0;
  EXPECT_FALSE(orb_get_topic_status(nullptr, 0, nullptr));
  EXPECT_EQ(errno, EINVAL);

  errno = 0;
  EXPECT_EQ(orb_poll(nullptr, 1, 0), -1);
  EXPECT_EQ(errno, EINVAL);

  errno = 0;
  EXPECT_EQ(orb_poll(fds, 0, 0), -1);
  EXPECT_EQ(errno, EINVAL);

  errno = 0;
  EXPECT_EQ(orb_poll(fds, 1, 0), -1);
  EXPECT_EQ(errno, EINVAL);

  EXPECT_TRUE(orb_destroy_subscription(&subscription));
  EXPECT_TRUE(orb_destroy_publication(&publication));
}

TEST_F(UnitTest, destroy_resets_handles_and_rejects_repeated_destroy) {
  auto *publication = orb_create_publication(ORB_ID(orb_test));
  ASSERT_NE(publication, nullptr);

  EXPECT_TRUE(orb_destroy_publication(&publication));
  EXPECT_EQ(publication, nullptr);

  errno = 0;
  EXPECT_FALSE(orb_destroy_publication(&publication));
  EXPECT_EQ(errno, EINVAL);

  auto *subscription = orb_create_subscription(ORB_ID(orb_test));
  ASSERT_NE(subscription, nullptr);

  EXPECT_TRUE(orb_destroy_subscription(&subscription));
  EXPECT_EQ(subscription, nullptr);

  errno = 0;
  EXPECT_FALSE(orb_destroy_subscription(&subscription));
  EXPECT_EQ(errno, EINVAL);
}

TEST_F(UnitTest, single_topic) {
  orb_test_s t{};
  orb_test_s u{};
  orb_publication_t *ptopic{};

  t.val = 0;
  ptopic = orb_create_publication_multi(ORB_ID(orb_test), nullptr);

  ASSERT_NE(ptopic, nullptr) << "advertise failed:" << errno;

  orb_publish(ptopic, &t);

  auto sfd = orb_create_subscription_multi(ORB_ID(orb_test), 0);

  ASSERT_NE(sfd, nullptr) << "subscribe failed: " << errno;

  u.val = 1;

  ASSERT_TRUE(orb_copy(sfd, &u)) << "copy(1) failed: " << errno;

  ASSERT_EQ(u.val, t.val) << "copy(1) mismatch";

  ASSERT_FALSE(orb_check_update(sfd)) << "spurious updated flag";

  t.val = 2;

  ASSERT_TRUE(orb_publish(ptopic, &t)) << "publish failed";

  ASSERT_TRUE(orb_check_update(sfd)) << "missing updated flag";
  ASSERT_TRUE(orb_copy(sfd, &u)) << "copy(2) failed: " << errno;

  ASSERT_EQ(u.val, t.val) << "copy(2) mismatch";

  // Publish twice
  ASSERT_TRUE(orb_publish(ptopic, &t)) << "publish failed";
  ASSERT_TRUE(orb_publish(ptopic, &t)) << "publish failed";

  ASSERT_TRUE(orb_check_update(sfd)) << "missing updated flag";
  ASSERT_TRUE(orb_copy(sfd, &u)) << "copy failed";

  ASSERT_FALSE(orb_check_update(sfd)) << "need to fail this time";
  ASSERT_TRUE(orb_copy(sfd, &u)) << "copy failed";

  ASSERT_TRUE(orb_destroy_subscription(&sfd));

  ASSERT_TRUE(orb_destroy_publication(&ptopic));
}

TEST_F(UnitTest, once_pub_sub) {
  orb_test_s pub_data{};

  ASSERT_TRUE(orb_publish_once(ORB_ID(orb_test), &pub_data))
      << "publish(1) failed: " << errno;

  orb_test_s sub_data{};
  sub_data.val = 1;

  ASSERT_TRUE(orb_copy_once(ORB_ID(orb_test), &sub_data))
      << "copy(1) failed: " << errno;

  ASSERT_EQ(sub_data.val, pub_data.val) << "copy(1) mismatch";

  pub_data.val = 2;
  ASSERT_TRUE(orb_publish_once(ORB_ID(orb_test), &pub_data))
      << "publish(2) failed" << errno;

  ASSERT_TRUE(orb_copy_once(ORB_ID(orb_test), &sub_data))
      << "copy(2) failed: " << errno;

  ASSERT_EQ(sub_data.val, pub_data.val) << "copy(2) mismatch";
}

TEST_F(UnitTest, multi_topic) {
  orb_publication_t *pfd[4]{};  ///< used for test_multi and test_multi_reversed

  /* this routine tests the multi-topic support */
  {
    orb_test_s pub_data{};
    orb_test_s sub_data{};

    unsigned instance0;
    pfd[0] = orb_create_publication_multi(ORB_ID(orb_multitest), &instance0);

    unsigned instance1;
    pfd[1] = orb_create_publication_multi(ORB_ID(orb_multitest), &instance1);

    ASSERT_EQ(instance0, 0) << "mult. id0: " << instance0;

    ASSERT_EQ(instance1, 1) << "mult. id1: " << instance1;

    pub_data.val = 103;

    ASSERT_TRUE(orb_publish(pfd[0], &pub_data)) << "mult. pub0 fail";

    pub_data.val = 203;

    ASSERT_TRUE(orb_publish(pfd[1], &pub_data)) << "mult. pub1 fail";

    /* subscribe to both topics and ensure valid data is received */
    auto sfd0 = orb_create_subscription_multi(ORB_ID(orb_multitest), 0);

    ASSERT_TRUE(orb_copy(sfd0, &sub_data)) << "sub #0 copy failed: " << errno;

    ASSERT_EQ(sub_data.val, 103) << "sub #0 val. mismatch: " << sub_data.val;

    auto sfd1 = orb_create_subscription_multi(ORB_ID(orb_multitest), 1);

    ASSERT_TRUE(orb_copy(sfd1, &sub_data)) << "sub #1 copy failed: " << errno;

    ASSERT_EQ(sub_data.val, 203) << "sub #1 val. mismatch: " << sub_data.val;

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

    orb_test_s pub_data{};
    orb_test_s sub_data{};

    pub_data.val = 0;

    unsigned int instance2;
    pfd[2] = orb_create_publication_multi(ORB_ID(orb_multitest), &instance2);
    ASSERT_EQ(instance2, 2) << "mult. id2: " << instance2;

    unsigned int instance3;
    pfd[3] = orb_create_publication_multi(ORB_ID(orb_multitest), &instance3);
    ASSERT_EQ(instance3, 3) << "mult. id3: " << instance3;

    pub_data.val = 204;
    ASSERT_TRUE(orb_publish(pfd[2], &pub_data)) << "mult. pub0 fail";

    pub_data.val = 304;
    ASSERT_TRUE(orb_publish(pfd[3], &pub_data)) << "mult. pub1 fail";

    ASSERT_TRUE(orb_copy(sfd2, &sub_data)) << "sub #2 copy failed: " << errno;
    orb_destroy_subscription(&sfd2);

    ASSERT_EQ(sub_data.val, 204) << "sub #3 val. mismatch: " << sub_data.val;

    auto sfd3 = orb_create_subscription_multi(ORB_ID(orb_multitest), 3);
    ASSERT_TRUE(orb_copy(sfd3, &sub_data)) << "sub #3 copy failed: " << errno;
    orb_destroy_subscription(&sfd3);

    ASSERT_EQ(sub_data.val, 304) << "sub #3 val. mismatch: " << sub_data.val;
  }

  // we still have the advertisements from the previous test_multi calls.
  for (auto &i : pfd) {
    ASSERT_TRUE(orb_destroy_publication(&i))
        << "orb_destroy_publication failed";
  }
}

TEST_F(UnitTest, multi_topic2_queue_simulation) {
  // test: first subscribe, then advertise

  std::atomic_bool thread_should_exit{false};
  const int num_instances = 3;
  orb_subscription_t *orb_data_fd[num_instances]{};
  int orb_data_next = 0;

  for (unsigned i = 0; i < num_instances; ++i) {
    orb_data_fd[i] =
        orb_create_subscription_multi(ORB_ID(orb_test_medium_multi), i);
  }

  std::thread pub_test_multi2_main([&]() {
    int data_next_idx = 0;
    const int num_instances = 3;
    orb_publication_t *orb_pub[num_instances]{};
    orb_test_medium_s data_topic{};

    for (unsigned i = 0; i < num_instances; ++i) {
      orb_publication_t *&pub = orb_pub[i];
      unsigned idx = i;
      pub = orb_create_publication_multi(ORB_ID(orb_test_medium_multi), &idx);

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

  orb_abstime_us last_time = 0;

  while (!thread_should_exit.load()) {
    usleep(1000);

    auto orb_data_cur_fd = orb_data_fd[orb_data_next];

    if (orb_check_update(orb_data_cur_fd)) {
      orb_test_medium_s msg{};
      orb_copy(orb_data_cur_fd, &msg);

      if (last_time != 0) {
        ASSERT_LT(last_time, msg.timestamp) << "Timestamp not increasing!";
      }
      last_time = msg.timestamp;

      orb_data_next = (orb_data_next + 1) % num_instances;
    }
  }

  pub_test_multi2_main.join();

  for (auto &i : orb_data_fd) {
    orb_destroy_subscription(&i);
  }
}  // namespace uORBTest

TEST_F(UnitTest, queue) {
  orb_test_medium_s pub_data{};
  orb_test_medium_s sub_data{};
  orb_publication_t *ptopic{nullptr};

  auto sfd = orb_create_subscription(ORB_ID(orb_test_medium_queue));

  ASSERT_NE(sfd, nullptr) << "subscribe failed: " << errno;

  const int queue_size = ORB_ID(orb_test_medium_queue)->o_queue_size;
  pub_data.val = 0;
  ptopic = orb_create_publication(ORB_ID(orb_test_medium_queue));
  ASSERT_NE(ptopic, nullptr) << "advertise failed: " << errno;

  orb_publish(ptopic, &pub_data);

  ASSERT_TRUE(orb_check_update(sfd)) << "update flag not set";

  ASSERT_TRUE(orb_copy(sfd, &sub_data)) << "copy(1) failed: " << errno;

  ASSERT_EQ(sub_data.val, pub_data.val) << "copy(1) mismatch";

  ASSERT_FALSE(orb_check_update(sfd)) << "spurious updated flag";

  // no messages in the queue anymore

  //  Testing to write some elements...

  for (int i = 0; i < queue_size - 2; ++i) {
    pub_data.val = i;
    orb_publish(ptopic, &pub_data);
  }

  for (int i = 0; i < queue_size - 2; ++i) {
    ASSERT_TRUE(orb_check_update(sfd)) << "update flag not set, element " << i;
    orb_copy(sfd, &sub_data);
    ASSERT_EQ(sub_data.val, i) << "got wrong element from the queue (got"
                               << sub_data.val << "should be" << i << ")";
  }

  ASSERT_FALSE(orb_check_update(sfd))
      << "update flag set, element " << queue_size;

  //  Testing overflow...
  int overflow_by = 3;

  for (int i = 0; i < queue_size + overflow_by; ++i) {
    pub_data.val = i;
    orb_publish(ptopic, &pub_data);
  }

  for (int i = 0; i < queue_size; ++i) {
    ASSERT_TRUE(orb_check_update(sfd)) << "update flag not set, element " << i;
    orb_copy(sfd, &sub_data);
    ASSERT_EQ(sub_data.val, i + overflow_by)
        << "got wrong element from the queue (got " << sub_data.val
        << "should be" << i + overflow_by << ")";
  }

  ASSERT_FALSE(orb_check_update(sfd))
      << "update flag set, element " << queue_size;

  //  Testing underflow...

  for (int i = 0; i < queue_size; ++i) {
    ASSERT_FALSE(orb_check_update(sfd)) << "update flag set, element " << i;
    orb_copy(sfd, &sub_data);
    ASSERT_EQ(sub_data.val, queue_size + overflow_by - 1)
        << "got wrong element from the queue (got " << sub_data.val
        << ", should be " << queue_size + overflow_by - 1 << ")";
  }

  pub_data.val = 943;
  orb_publish(ptopic, &pub_data);
  ASSERT_TRUE(orb_check_update(sfd)) << "update flag not set, element " << -1;

  orb_copy(sfd, &sub_data);
  ASSERT_EQ(sub_data.val, pub_data.val)
      << "got wrong element from the queue (got " << sub_data.val
      << ", should be " << pub_data.val << ")";

  ASSERT_TRUE(orb_destroy_publication(&ptopic));
  ASSERT_TRUE(orb_destroy_subscription(&sfd));
}

TEST_F(UnitTest, wrap_around) {
  orb_test_medium_s pub_data{};
  orb_test_medium_s sub_data{};
  orb_publication_t *ptopic{nullptr};

  auto sfd = orb_create_subscription(ORB_ID(orb_test_medium_wrap_around));

  ASSERT_NE(sfd, nullptr) << "subscribe failed: " << errno;

  const int queue_size = 16;
  pub_data.val = 0;
  ptopic = orb_create_publication(ORB_ID(orb_test_medium_wrap_around));
  ASSERT_NE(ptopic, nullptr) << "advertise failed: " << errno;
  orb_publish(ptopic, &pub_data);

  // Set generation to the location where wrap-around is about to be
  {
    auto node = uorb::DeviceMaster::get_instance().GetDeviceNode(
        *ORB_ID(orb_test_medium_wrap_around), 0);
    ASSERT_NE(node, nullptr);
    set_generation(*node, unsigned(-(queue_size / 2)));

    // Refresh the subscriber's generation
    for (int i = 0; i < queue_size; i++) {
      if (!orb_check_update(sfd)) {
        break;
      }
      orb_copy(sfd, &sub_data);
    }
  }

  orb_publish(ptopic, &pub_data);

  ASSERT_TRUE(orb_check_update(sfd)) << "update flag not set";

  ASSERT_TRUE(orb_copy(sfd, &sub_data)) << "copy(1) failed: " << errno;

  ASSERT_EQ(sub_data.val, pub_data.val) << "copy(1) mismatch";

  ASSERT_FALSE(orb_check_update(sfd)) << "spurious updated flag";

  // no messages in the queue anymore

  //  Testing to write some elements...

  for (int i = 0; i < queue_size - 2; ++i) {
    pub_data.val = i;
    orb_publish(ptopic, &pub_data);
  }

  for (int i = 0; i < queue_size - 2; ++i) {
    ASSERT_TRUE(orb_check_update(sfd)) << "update flag not set, element " << i;
    orb_copy(sfd, &sub_data);
    ASSERT_EQ(sub_data.val, i) << "got wrong element from the queue (got"
                               << sub_data.val << "should be" << i << ")";
  }

  ASSERT_FALSE(orb_check_update(sfd))
      << "update flag set, element " << queue_size;

  //  Testing overflow...
  int overflow_by = 3;

  for (int i = 0; i < queue_size + overflow_by; ++i) {
    pub_data.val = i;
    orb_publish(ptopic, &pub_data);
  }

  for (int i = 0; i < queue_size; ++i) {
    ASSERT_TRUE(orb_check_update(sfd)) << "update flag not set, element " << i;
    orb_copy(sfd, &sub_data);
    ASSERT_EQ(sub_data.val, i + overflow_by)
        << "got wrong element from the queue (got " << sub_data.val
        << "should be" << i + overflow_by << ")";
  }

  ASSERT_FALSE(orb_check_update(sfd))
      << "update flag set, element " << queue_size;

  //  Testing underflow...

  for (int i = 0; i < queue_size; ++i) {
    ASSERT_FALSE(orb_check_update(sfd)) << "update flag set, element " << i;
    orb_copy(sfd, &sub_data);
    ASSERT_EQ(sub_data.val, queue_size + overflow_by - 1)
        << "got wrong element from the queue (got " << sub_data.val
        << ", should be " << queue_size + overflow_by - 1 << ")";
  }

  pub_data.val = 943;
  orb_publish(ptopic, &pub_data);
  ASSERT_TRUE(orb_check_update(sfd)) << "update flag not set, element " << -1;

  orb_copy(sfd, &sub_data);
  ASSERT_EQ(sub_data.val, pub_data.val)
      << "got wrong element from the queue (got " << sub_data.val
      << ", should be " << pub_data.val << ")";

  ASSERT_TRUE(orb_destroy_publication(&ptopic));
  ASSERT_TRUE(orb_destroy_subscription(&sfd));
}

TEST_F(UnitTest, queue_poll_notify) {
  orb_test_medium_s t{};
  std::atomic<int> num_messages_sent{0};
  std::atomic_bool thread_should_exit{false};

  orb_subscription_t *sfd;
  ASSERT_NE(sfd = orb_create_subscription(ORB_ID(orb_test_queue_poll)), nullptr)
      << "subscribe failed: " << errno;

  std::thread test_queue_thread{[&]() {
    orb_test_medium_s pub_data{};
    orb_publication_t *ptopic{nullptr};
    const int queue_size = ORB_ID(orb_test_queue_poll)->o_queue_size;
    ptopic = orb_create_publication(ORB_ID(orb_test_queue_poll));
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
        orb_publish(ptopic, &pub_data);
        ++pub_data.val;
      }

      message_counter += burst_counter;
      usleep(20 * 1000);  // give subscriber a chance to catch up
    }

    num_messages_sent = pub_data.val;
    usleep(100 * 1000);
    thread_should_exit = true;
    orb_destroy_publication(&ptopic);
  }};

  int next_expected_val = 0;
  orb_pollfd_t fds[1]{};
  fds[0].fd = sfd;

  while (!thread_should_exit.load()) {
    int poll_ret = orb_poll(fds, 1, 500);
    ASSERT_GE(poll_ret, 0) << "poll error (" << poll_ret << "," << errno << ")";

    if (thread_should_exit.load()) {
      break;
    }

    ASSERT_NE(poll_ret, 0) << "poll timeout";

    if (fds[0].ready) {
      orb_copy(sfd, &t);
      ASSERT_EQ(next_expected_val, t.val) << "copy mismatch";
      ++next_expected_val;
    }
  }

  test_queue_thread.join();

  ASSERT_TRUE(orb_destroy_subscription(&sfd));

  ASSERT_EQ(num_messages_sent.load(), next_expected_val)
      << "number of sent and received messages mismatch";
}

TEST_F(UnitTest, poll_timeout_semantics) {
  auto sub = orb_create_subscription(ORB_ID(orb_test));
  ASSERT_NE(sub, nullptr);

  // Drain any pre-existing updates from other test cases.
  orb_test_s drain{};
  for (int i = 0; i < 8 && orb_check_update(sub); ++i) {
    ASSERT_TRUE(orb_copy(sub, &drain));
  }

  orb_pollfd_t fds[1]{};
  fds[0].fd = sub;

  const auto zero_start = std::chrono::steady_clock::now();
  EXPECT_EQ(orb_poll(fds, 1, 0), 0);
  const auto zero_elapsed_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now() - zero_start)
          .count();
  EXPECT_LT(zero_elapsed_ms, 20);

  orb_publication_t *pub = orb_create_publication(ORB_ID(orb_test));
  ASSERT_NE(pub, nullptr);

  std::thread publisher([&]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(40));
    orb_test_s msg{};
    msg.val = 999;
    EXPECT_TRUE(orb_publish(pub, &msg));
  });

  const auto neg_start = std::chrono::steady_clock::now();
  EXPECT_EQ(orb_poll(fds, 1, -5), 1);
  const auto neg_elapsed_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now() - neg_start)
          .count();
  EXPECT_GE(neg_elapsed_ms, 25);
  EXPECT_TRUE(fds[0].ready);

  publisher.join();
  EXPECT_TRUE(orb_destroy_publication(&pub));
  EXPECT_TRUE(orb_destroy_subscription(&sub));
}

TEST_F(UnitTest, topic_status_counter_saturation) {
  constexpr int kManyHandles = 300;

  std::vector<orb_publication_t *> pubs;
  pubs.reserve(kManyHandles);
  for (int i = 0; i < kManyHandles; ++i) {
    auto *pub = orb_create_publication(ORB_ID(orb_test));
    ASSERT_NE(pub, nullptr);
    pubs.push_back(pub);
  }

  std::vector<orb_subscription_t *> subs;
  subs.reserve(kManyHandles);
  for (int i = 0; i < kManyHandles; ++i) {
    auto *sub = orb_create_subscription(ORB_ID(orb_test));
    ASSERT_NE(sub, nullptr);
    subs.push_back(sub);
  }

  constexpr uint16_t kMaxTrackedHandles = 127;

  orb_status status{};
  ASSERT_TRUE(orb_get_topic_status(ORB_ID(orb_test), 0, &status));
  EXPECT_EQ(status.publisher_count, kMaxTrackedHandles);
  EXPECT_EQ(status.subscriber_count, kMaxTrackedHandles);

  for (auto *sub : subs) {
    EXPECT_TRUE(orb_destroy_subscription(&sub));
  }
  for (auto *pub : pubs) {
    EXPECT_TRUE(orb_destroy_publication(&pub));
  }
}

}  // namespace uORBTest
