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
#include <uevent/uevent.h>
#include <uorb/publication_multi.h>
#include <uorb/subscription_interval.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <thread>
#include <vector>

namespace uORBTest {

class TestableSubscriptionInterval final : public uorb::SubscriptionInterval<uorb::msg::orb_test> {
 public:
  using uorb::SubscriptionInterval<uorb::msg::orb_test>::CalculateNextUpdateTime;
};

TEST_F(UnitTest, subscription_interval_handles_initial_time_less_than_interval) {
  constexpr uint32_t interval_us = 60 * 1000 * 1000;
  constexpr orb_abstime_us now = 1000;

  EXPECT_EQ(TestableSubscriptionInterval::CalculateNextUpdateTime(0, interval_us, now), now);
}

TEST_F(UnitTest, unadvertise) {
  // try to advertise and see whether we get the right instance
  orb_pub_t pfd[4] = {ORB_PUB_INITIALIZER, ORB_PUB_INITIALIZER,
                               ORB_PUB_INITIALIZER, ORB_PUB_INITIALIZER};
  unsigned instance_test[4]{};
  orb_test_s t{};

  for (int i = 0; i < 4; ++i) {
    EXPECT_EQ(orb_pub_create_multi(&pfd[i], ORB_ID(orb_multitest), &instance_test[i]), ORB_OK);
    EXPECT_EQ(instance_test[i], i) << "got wrong instance";
    orb_pub_publish(pfd[i], &t);
  }

  for (auto &i : pfd) {
    orb_pub_destroy(&i);
  }
}

TEST_F(UnitTest, publication_multi_reports_eexist_when_instances_are_full) {
  orb_pub_t publications[ORB_MULTI_MAX_INSTANCES] = {};
  unsigned instances[ORB_MULTI_MAX_INSTANCES]{};

  for (unsigned index = 0; index < ORB_MULTI_MAX_INSTANCES; ++index) {
    EXPECT_EQ(orb_pub_create_multi(&publications[index], ORB_ID(orb_test_medium), &instances[index]), ORB_OK);
    EXPECT_NE(publications[index]._handle, nullptr);
    EXPECT_EQ(instances[index], index);
  }

  unsigned extra_instance = 0;
  orb_pub_t extra_pub = ORB_PUB_INITIALIZER;
  EXPECT_EQ(orb_pub_create_multi(&extra_pub, ORB_ID(orb_test_medium), &extra_instance), ORB_ERR_EXIST);

  for (auto &publication : publications) {
    orb_pub_destroy(&publication);
  }
}

TEST_F(UnitTest, subscription_multi_reports_einval_for_invalid_instance) {
  orb_sub_t subscription = ORB_SUB_INITIALIZER;
  EXPECT_EQ(orb_sub_create_multi(&subscription, ORB_ID(orb_test), ORB_MULTI_MAX_INSTANCES), ORB_ERR_INVALID);
}

TEST_F(UnitTest, rejects_null_arguments) {
  orb_test_s msg{};

  EXPECT_EQ(orb_pub_create(nullptr, ORB_ID(orb_test)), ORB_ERR_INVALID);
  EXPECT_EQ(orb_pub_create_multi(nullptr, ORB_ID(orb_test), nullptr), ORB_ERR_INVALID);
  EXPECT_EQ(orb_sub_create(nullptr, ORB_ID(orb_test)), ORB_ERR_INVALID);
  EXPECT_EQ(orb_sub_create_multi(nullptr, ORB_ID(orb_test), 0), ORB_ERR_INVALID);

  orb_pub_destroy(nullptr);
  orb_sub_destroy(nullptr);

  orb_pub_t empty_pub = ORB_PUB_INITIALIZER;
  EXPECT_EQ(orb_pub_publish(empty_pub, &msg), ORB_ERR_INVALID);

  orb_pub_t publication = ORB_PUB_INITIALIZER;
  EXPECT_EQ(orb_pub_create(&publication, ORB_ID(orb_test)), ORB_OK);
  ASSERT_NE(publication._handle, nullptr);

  EXPECT_EQ(orb_pub_publish(publication, nullptr), ORB_ERR_INVALID);

  EXPECT_EQ(orb_pub_publish_once(nullptr, &msg), ORB_ERR_INVALID);
  EXPECT_EQ(orb_pub_publish_once(ORB_ID(orb_test), nullptr), ORB_ERR_INVALID);

  orb_sub_t empty_sub = ORB_SUB_INITIALIZER;
  EXPECT_EQ(orb_sub_copy(empty_sub, &msg), ORB_ERR_INVALID);

  orb_sub_t subscription = ORB_SUB_INITIALIZER;
  EXPECT_EQ(orb_sub_create(&subscription, ORB_ID(orb_test)), ORB_OK);
  ASSERT_NE(subscription._handle, nullptr);

  EXPECT_EQ(orb_sub_copy(subscription, nullptr), ORB_ERR_INVALID);

  EXPECT_EQ(orb_sub_copy_once(nullptr, &msg), ORB_ERR_INVALID);
  EXPECT_EQ(orb_sub_copy_once(ORB_ID(orb_test), nullptr), ORB_ERR_INVALID);

  EXPECT_FALSE(orb_sub_check_update(empty_sub));

  EXPECT_FALSE(orb_exists(nullptr, 0));
  EXPECT_EQ(orb_group_count(nullptr), 0U);

  EXPECT_EQ(orb_get_topic_status(nullptr, 0, nullptr), ORB_ERR_INVALID);

  orb_sub_destroy(&subscription);
  orb_pub_destroy(&publication);
}

TEST_F(UnitTest, destroy_resets_handles_and_is_idempotent) {
  orb_pub_t publication = ORB_PUB_INITIALIZER;
  EXPECT_EQ(orb_pub_create(&publication, ORB_ID(orb_test)), ORB_OK);
  ASSERT_NE(publication._handle, nullptr);

  orb_pub_destroy(&publication);
  EXPECT_EQ(publication._handle, nullptr);

  // Repeated destroy is a safe no-op.
  orb_pub_destroy(&publication);
  EXPECT_EQ(publication._handle, nullptr);

  orb_sub_t subscription = ORB_SUB_INITIALIZER;
  EXPECT_EQ(orb_sub_create(&subscription, ORB_ID(orb_test)), ORB_OK);
  ASSERT_NE(subscription._handle, nullptr);

  orb_sub_destroy(&subscription);
  EXPECT_EQ(subscription._handle, nullptr);

  // Repeated destroy is a safe no-op.
  orb_sub_destroy(&subscription);
  EXPECT_EQ(subscription._handle, nullptr);
}

TEST_F(UnitTest, uevent_poll_rejects_invalid_arguments) {
  orb_sub_t subscription = ORB_SUB_INITIALIZER;
  EXPECT_EQ(orb_sub_create(&subscription, ORB_ID(orb_test)), ORB_OK);
  ASSERT_NE(subscription._handle, nullptr);
  uevent_source_t src = orb_sub_event_source(subscription);
  uevent_source_t ready[1] = {UEVENT_SOURCE_INITIALIZER};

  EXPECT_EQ(uevent_add(nullptr, &src, 0), UEVENT_ERR_INVALID);

  uevent_t fake_base = {reinterpret_cast<void *>(0x1)};
  EXPECT_EQ(uevent_add(&fake_base, nullptr, 0), UEVENT_ERR_INVALID);

  EXPECT_EQ(uevent_remove(nullptr, &src), UEVENT_ERR_INVALID);

  EXPECT_EQ(uevent_remove(&fake_base, nullptr), UEVENT_ERR_INVALID);

  EXPECT_EQ(uevent_loop(nullptr, ready, 1, 0), UEVENT_ERR_INVALID);

  uevent_t base = UEVENT_INITIALIZER;
  uevent_create(&base);
  ASSERT_NE(base._handle, nullptr);

  EXPECT_EQ(uevent_loop(&base, nullptr, 1, 0), UEVENT_ERR_INVALID);

  EXPECT_EQ(uevent_loop(&base, ready, 0, 0), UEVENT_ERR_INVALID);

  EXPECT_EQ(uevent_loopbreak(nullptr), UEVENT_ERR_INVALID);

  uevent_destroy(&base);
  orb_sub_destroy(&subscription);
}

TEST_F(UnitTest, single_topic) {
  orb_test_s t{};
  orb_test_s u{};
  orb_pub_t ptopic = ORB_PUB_INITIALIZER;

  t.val = 0;
  EXPECT_EQ(orb_pub_create_multi(&ptopic, ORB_ID(orb_test), nullptr), ORB_OK);

  ASSERT_NE(ptopic._handle, nullptr) << "advertise failed:" << errno;

  orb_pub_publish(ptopic, &t);

  orb_sub_t sfd = ORB_SUB_INITIALIZER;
  EXPECT_EQ(orb_sub_create_multi(&sfd, ORB_ID(orb_test), 0), ORB_OK);

  ASSERT_NE(sfd._handle, nullptr) << "subscribe failed: " << errno;

  u.val = 1;

  ASSERT_EQ(orb_sub_copy(sfd, &u), ORB_OK) << "copy(1) failed: " << errno;

  ASSERT_EQ(u.val, t.val) << "copy(1) mismatch";

  ASSERT_FALSE(orb_sub_check_update(sfd)) << "spurious updated flag";

  t.val = 2;

  ASSERT_EQ(orb_pub_publish(ptopic, &t), ORB_OK) << "publish failed";

  ASSERT_TRUE(orb_sub_check_update(sfd)) << "missing updated flag";
  ASSERT_EQ(orb_sub_copy(sfd, &u), ORB_OK) << "copy(2) failed: " << errno;

  ASSERT_EQ(u.val, t.val) << "copy(2) mismatch";

  // Publish twice
  ASSERT_EQ(orb_pub_publish(ptopic, &t), ORB_OK) << "publish failed";
  ASSERT_EQ(orb_pub_publish(ptopic, &t), ORB_OK) << "publish failed";

  ASSERT_TRUE(orb_sub_check_update(sfd)) << "missing updated flag";
  ASSERT_EQ(orb_sub_copy(sfd, &u), ORB_OK) << "copy failed";

  ASSERT_FALSE(orb_sub_check_update(sfd)) << "need to fail this time";
  ASSERT_EQ(orb_sub_copy(sfd, &u), ORB_OK) << "copy failed";

  orb_sub_destroy(&sfd);

  orb_pub_destroy(&ptopic);
}

TEST_F(UnitTest, once_pub_sub) {
  orb_test_s pub_data{};

  ASSERT_EQ(orb_pub_publish_once(ORB_ID(orb_test), &pub_data), ORB_OK)
      << "publish(1) failed: " << errno;

  orb_test_s sub_data{};
  sub_data.val = 1;

  ASSERT_EQ(orb_sub_copy_once(ORB_ID(orb_test), &sub_data), ORB_OK)
      << "copy(1) failed: " << errno;

  ASSERT_EQ(sub_data.val, pub_data.val) << "copy(1) mismatch";

  pub_data.val = 2;
  ASSERT_EQ(orb_pub_publish_once(ORB_ID(orb_test), &pub_data), ORB_OK)
      << "publish(2) failed" << errno;

  ASSERT_EQ(orb_sub_copy_once(ORB_ID(orb_test), &sub_data), ORB_OK)
      << "copy(2) failed: " << errno;

  ASSERT_EQ(sub_data.val, pub_data.val) << "copy(2) mismatch";
}

TEST_F(UnitTest, orb_copy_once_reads_latest_queued_sample) {
  orb_test_medium_s pub_data{};
  orb_pub_t publication = ORB_PUB_INITIALIZER;
  EXPECT_EQ(orb_pub_create(&publication, ORB_ID(orb_test_medium_queue)), ORB_OK);
  ASSERT_NE(publication._handle, nullptr) << "advertise failed: " << errno;

  const int queue_size = ORB_ID(orb_test_medium_queue)->o_queue_size;
  const int last_value = queue_size * 2 + 3;

  for (int value = 0; value <= last_value; ++value) {
    pub_data.val = value;
    ASSERT_EQ(orb_pub_publish(publication, &pub_data), ORB_OK) << "publish failed: " << errno;
  }

  orb_test_medium_s sub_data{};
  ASSERT_EQ(orb_sub_copy_once(ORB_ID(orb_test_medium_queue), &sub_data), ORB_OK) << "copy failed: " << errno;
  EXPECT_EQ(sub_data.val, last_value);

  orb_pub_destroy(&publication);
}

TEST_F(UnitTest, multi_topic) {
  orb_pub_t pfd[4] = {ORB_PUB_INITIALIZER, ORB_PUB_INITIALIZER,
                               ORB_PUB_INITIALIZER, ORB_PUB_INITIALIZER};

  /* this routine tests the multi-topic support */
  {
    orb_test_s pub_data{};
    orb_test_s sub_data{};

    unsigned instance0;
    EXPECT_EQ(orb_pub_create_multi(&pfd[0], ORB_ID(orb_multitest), &instance0), ORB_OK);

    unsigned instance1;
    EXPECT_EQ(orb_pub_create_multi(&pfd[1], ORB_ID(orb_multitest), &instance1), ORB_OK);

    ASSERT_EQ(instance0, 0) << "mult. id0: " << instance0;

    ASSERT_EQ(instance1, 1) << "mult. id1: " << instance1;

    pub_data.val = 103;

    ASSERT_EQ(orb_pub_publish(pfd[0], &pub_data), ORB_OK) << "mult. pub0 fail";

    pub_data.val = 203;

    ASSERT_EQ(orb_pub_publish(pfd[1], &pub_data), ORB_OK) << "mult. pub1 fail";

    /* subscribe to both topics and ensure valid data is received */
    orb_sub_t sfd0 = ORB_SUB_INITIALIZER;
    EXPECT_EQ(orb_sub_create_multi(&sfd0, ORB_ID(orb_multitest), 0), ORB_OK);

    ASSERT_EQ(orb_sub_copy(sfd0, &sub_data), ORB_OK) << "sub #0 copy failed: " << errno;

    ASSERT_EQ(sub_data.val, 103) << "sub #0 val. mismatch: " << sub_data.val;

    orb_sub_t sfd1 = ORB_SUB_INITIALIZER;
    EXPECT_EQ(orb_sub_create_multi(&sfd1, ORB_ID(orb_multitest), 1), ORB_OK);

    ASSERT_EQ(orb_sub_copy(sfd1, &sub_data), ORB_OK) << "sub #1 copy failed: " << errno;

    ASSERT_EQ(sub_data.val, 203) << "sub #1 val. mismatch: " << sub_data.val;

    latency_test<orb_test_s>(ORB_ID(orb_test));

    orb_sub_destroy(&sfd0);
    orb_sub_destroy(&sfd1);
  }
  {
    /* For these tests 0 and 1 instances are taken from before, therefore
     * continue with 2 and 3. */

    // try multi-topic support subscribing before publishing

    /* Subscribe first and advertise afterwards. */
    orb_sub_t sfd2 = ORB_SUB_INITIALIZER;
    EXPECT_EQ(orb_sub_create_multi(&sfd2, ORB_ID(orb_multitest), 2), ORB_OK);

    ASSERT_NE(sfd2._handle, nullptr) << "errno: " << errno;

    orb_test_s pub_data{};
    orb_test_s sub_data{};

    pub_data.val = 0;

    unsigned int instance2;
    EXPECT_EQ(orb_pub_create_multi(&pfd[2], ORB_ID(orb_multitest), &instance2), ORB_OK);
    ASSERT_EQ(instance2, 2) << "mult. id2: " << instance2;

    unsigned int instance3;
    EXPECT_EQ(orb_pub_create_multi(&pfd[3], ORB_ID(orb_multitest), &instance3), ORB_OK);
    ASSERT_EQ(instance3, 3) << "mult. id3: " << instance3;

    pub_data.val = 204;
    ASSERT_EQ(orb_pub_publish(pfd[2], &pub_data), ORB_OK) << "mult. pub0 fail";

    pub_data.val = 304;
    ASSERT_EQ(orb_pub_publish(pfd[3], &pub_data), ORB_OK) << "mult. pub1 fail";

    ASSERT_EQ(orb_sub_copy(sfd2, &sub_data), ORB_OK) << "sub #2 copy failed: " << errno;
    orb_sub_destroy(&sfd2);

    ASSERT_EQ(sub_data.val, 204) << "sub #3 val. mismatch: " << sub_data.val;

    orb_sub_t sfd3 = ORB_SUB_INITIALIZER;
    EXPECT_EQ(orb_sub_create_multi(&sfd3, ORB_ID(orb_multitest), 3), ORB_OK);
    ASSERT_EQ(orb_sub_copy(sfd3, &sub_data), ORB_OK) << "sub #3 copy failed: " << errno;
    orb_sub_destroy(&sfd3);

    ASSERT_EQ(sub_data.val, 304) << "sub #3 val. mismatch: " << sub_data.val;
  }

  // we still have the advertisements from the previous test_multi calls.
  for (auto &i : pfd) {
    orb_pub_destroy(&i);
  }
}

TEST_F(UnitTest, multi_topic2_queue_simulation) {
  // test: first subscribe, then advertise

  std::atomic_bool thread_should_exit{false};
  const int num_instances = 3;
  orb_sub_t orb_data_fd[3] = {ORB_SUB_INITIALIZER, ORB_SUB_INITIALIZER,
                                        ORB_SUB_INITIALIZER};
  int orb_data_next = 0;

  for (unsigned i = 0; i < num_instances; ++i) {
    orb_sub_create_multi(&orb_data_fd[i], ORB_ID(orb_test_medium_multi), i);
  }

  std::thread pub_test_multi2_main([&]() {
    int data_next_idx = 0;
    const int num_instances = 3;
    orb_pub_t orb_pub[3] = {ORB_PUB_INITIALIZER, ORB_PUB_INITIALIZER,
                                     ORB_PUB_INITIALIZER};
    orb_test_medium_s data_topic{};

    for (unsigned i = 0; i < num_instances; ++i) {
      unsigned idx = i;
      orb_pub_create_multi(&orb_pub[i], ORB_ID(orb_test_medium_multi), &idx);

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
      auto &pub = orb_pub[data_next_idx];

      data_topic.timestamp = orb_absolute_time_us();
      data_topic.val = data_next_idx;

      orb_pub_publish(pub, &data_topic);

      data_next_idx = (data_next_idx + 1) % num_instances;

      if (data_next_idx == 0) {
        usleep(50 * 1000);
      }
    }

    usleep(100 * 1000);
    thread_should_exit = true;

    for (auto &i : orb_pub) {
      orb_pub_destroy(&i);
    }
    return 0;
  });

  orb_abstime_us last_time = 0;

  while (!thread_should_exit.load()) {
    usleep(1000);

    auto &orb_data_cur_fd = orb_data_fd[orb_data_next];

    if (orb_sub_check_update(orb_data_cur_fd)) {
      orb_test_medium_s msg{};
      orb_sub_copy(orb_data_cur_fd, &msg);

      if (last_time != 0) {
        ASSERT_LT(last_time, msg.timestamp) << "Timestamp not increasing!";
      }
      last_time = msg.timestamp;

      orb_data_next = (orb_data_next + 1) % num_instances;
    }
  }

  pub_test_multi2_main.join();

  for (auto &i : orb_data_fd) {
    orb_sub_destroy(&i);
  }
}  // namespace uORBTest

TEST_F(UnitTest, queue) {
  orb_test_medium_s pub_data{};
  orb_test_medium_s sub_data{};
  orb_pub_t ptopic = ORB_PUB_INITIALIZER;

  orb_sub_t sfd = ORB_SUB_INITIALIZER;
  EXPECT_EQ(orb_sub_create(&sfd, ORB_ID(orb_test_medium_queue)), ORB_OK);

  ASSERT_NE(sfd._handle, nullptr) << "subscribe failed: " << errno;

  while (orb_sub_check_update(sfd)) {
    ASSERT_EQ(orb_sub_copy(sfd, &sub_data), ORB_OK) << "drain stale sample failed: " << errno;
  }

  const int queue_size = ORB_ID(orb_test_medium_queue)->o_queue_size;
  pub_data.val = 0;
  EXPECT_EQ(orb_pub_create(&ptopic, ORB_ID(orb_test_medium_queue)), ORB_OK);
  ASSERT_NE(ptopic._handle, nullptr) << "advertise failed: " << errno;

  orb_pub_publish(ptopic, &pub_data);

  ASSERT_TRUE(orb_sub_check_update(sfd)) << "update flag not set";

  ASSERT_EQ(orb_sub_copy(sfd, &sub_data), ORB_OK) << "copy(1) failed: " << errno;

  ASSERT_EQ(sub_data.val, pub_data.val) << "copy(1) mismatch";

  ASSERT_FALSE(orb_sub_check_update(sfd)) << "spurious updated flag";

  // no messages in the queue anymore

  //  Testing to write some elements...

  for (int i = 0; i < queue_size - 2; ++i) {
    pub_data.val = i;
    orb_pub_publish(ptopic, &pub_data);
  }

  for (int i = 0; i < queue_size - 2; ++i) {
    ASSERT_TRUE(orb_sub_check_update(sfd)) << "update flag not set, element " << i;
    orb_sub_copy(sfd, &sub_data);
    ASSERT_EQ(sub_data.val, i) << "got wrong element from the queue (got"
                               << sub_data.val << "should be" << i << ")";
  }

  ASSERT_FALSE(orb_sub_check_update(sfd))
      << "update flag set, element " << queue_size;

  //  Testing overflow...
  int overflow_by = 3;

  for (int i = 0; i < queue_size + overflow_by; ++i) {
    pub_data.val = i;
    orb_pub_publish(ptopic, &pub_data);
  }

  for (int i = 0; i < queue_size; ++i) {
    ASSERT_TRUE(orb_sub_check_update(sfd)) << "update flag not set, element " << i;
    orb_sub_copy(sfd, &sub_data);
    ASSERT_EQ(sub_data.val, i + overflow_by)
        << "got wrong element from the queue (got " << sub_data.val
        << "should be" << i + overflow_by << ")";
  }

  ASSERT_FALSE(orb_sub_check_update(sfd))
      << "update flag set, element " << queue_size;

  //  Testing underflow...

  for (int i = 0; i < queue_size; ++i) {
    ASSERT_FALSE(orb_sub_check_update(sfd)) << "update flag set, element " << i;
    orb_sub_copy(sfd, &sub_data);
    ASSERT_EQ(sub_data.val, queue_size + overflow_by - 1)
        << "got wrong element from the queue (got " << sub_data.val
        << ", should be " << queue_size + overflow_by - 1 << ")";
  }

  pub_data.val = 943;
  orb_pub_publish(ptopic, &pub_data);
  ASSERT_TRUE(orb_sub_check_update(sfd)) << "update flag not set, element " << -1;

  orb_sub_copy(sfd, &sub_data);
  ASSERT_EQ(sub_data.val, pub_data.val)
      << "got wrong element from the queue (got " << sub_data.val
      << ", should be " << pub_data.val << ")";

  orb_pub_destroy(&ptopic);
  orb_sub_destroy(&sfd);
}

TEST_F(UnitTest, wrap_around) {
  orb_test_medium_s pub_data{};
  orb_test_medium_s sub_data{};
  orb_pub_t ptopic = ORB_PUB_INITIALIZER;

  orb_sub_t sfd = ORB_SUB_INITIALIZER;
  EXPECT_EQ(orb_sub_create(&sfd, ORB_ID(orb_test_medium_wrap_around)), ORB_OK);

  ASSERT_NE(sfd._handle, nullptr) << "subscribe failed: " << errno;

  const int queue_size = 16;
  pub_data.val = 0;
  EXPECT_EQ(orb_pub_create(&ptopic, ORB_ID(orb_test_medium_wrap_around)), ORB_OK);
  ASSERT_NE(ptopic._handle, nullptr) << "advertise failed: " << errno;
  orb_pub_publish(ptopic, &pub_data);

  // Set generation to the location where wrap-around is about to be
  {
    auto node = uorb::DeviceMaster::get_instance().GetDeviceNode(
        *ORB_ID(orb_test_medium_wrap_around), 0);
    ASSERT_NE(node, nullptr);
    set_generation(*node, unsigned(-(queue_size / 2)));

    // Refresh the subscriber's generation
    for (int i = 0; i < queue_size; i++) {
      if (!orb_sub_check_update(sfd)) {
        break;
      }
      orb_sub_copy(sfd, &sub_data);
    }
  }

  orb_pub_publish(ptopic, &pub_data);

  ASSERT_TRUE(orb_sub_check_update(sfd)) << "update flag not set";

  ASSERT_EQ(orb_sub_copy(sfd, &sub_data), ORB_OK) << "copy(1) failed: " << errno;

  ASSERT_EQ(sub_data.val, pub_data.val) << "copy(1) mismatch";

  ASSERT_FALSE(orb_sub_check_update(sfd)) << "spurious updated flag";

  // no messages in the queue anymore

  //  Testing to write some elements...

  for (int i = 0; i < queue_size - 2; ++i) {
    pub_data.val = i;
    orb_pub_publish(ptopic, &pub_data);
  }

  for (int i = 0; i < queue_size - 2; ++i) {
    ASSERT_TRUE(orb_sub_check_update(sfd)) << "update flag not set, element " << i;
    orb_sub_copy(sfd, &sub_data);
    ASSERT_EQ(sub_data.val, i) << "got wrong element from the queue (got"
                               << sub_data.val << "should be" << i << ")";
  }

  ASSERT_FALSE(orb_sub_check_update(sfd))
      << "update flag set, element " << queue_size;

  //  Testing overflow...
  int overflow_by = 3;

  for (int i = 0; i < queue_size + overflow_by; ++i) {
    pub_data.val = i;
    orb_pub_publish(ptopic, &pub_data);
  }

  for (int i = 0; i < queue_size; ++i) {
    ASSERT_TRUE(orb_sub_check_update(sfd)) << "update flag not set, element " << i;
    orb_sub_copy(sfd, &sub_data);
    ASSERT_EQ(sub_data.val, i + overflow_by)
        << "got wrong element from the queue (got " << sub_data.val
        << "should be" << i + overflow_by << ")";
  }

  ASSERT_FALSE(orb_sub_check_update(sfd))
      << "update flag set, element " << queue_size;

  //  Testing underflow...

  for (int i = 0; i < queue_size; ++i) {
    ASSERT_FALSE(orb_sub_check_update(sfd)) << "update flag set, element " << i;
    orb_sub_copy(sfd, &sub_data);
    ASSERT_EQ(sub_data.val, queue_size + overflow_by - 1)
        << "got wrong element from the queue (got " << sub_data.val
        << ", should be " << queue_size + overflow_by - 1 << ")";
  }

  pub_data.val = 943;
  orb_pub_publish(ptopic, &pub_data);
  ASSERT_TRUE(orb_sub_check_update(sfd)) << "update flag not set, element " << -1;

  orb_sub_copy(sfd, &sub_data);
  ASSERT_EQ(sub_data.val, pub_data.val)
      << "got wrong element from the queue (got " << sub_data.val
      << ", should be " << pub_data.val << ")";

  orb_pub_destroy(&ptopic);
  orb_sub_destroy(&sfd);
}

TEST_F(UnitTest, queue_poll_notify) {
  orb_test_medium_s t{};
  std::atomic<int> num_messages_sent{0};
  std::atomic_bool thread_should_exit{false};

  orb_sub_t sfd = ORB_SUB_INITIALIZER;
  EXPECT_EQ(orb_sub_create(&sfd, ORB_ID(orb_test_queue_poll)), ORB_OK);
  ASSERT_NE(sfd._handle, nullptr)
      << "subscribe failed: " << errno;

  std::thread test_queue_thread{[&]() {
    orb_test_medium_s pub_data{};
    orb_pub_t ptopic = ORB_PUB_INITIALIZER;
    const int queue_size = ORB_ID(orb_test_queue_poll)->o_queue_size;
    EXPECT_EQ(orb_pub_create(&ptopic, ORB_ID(orb_test_queue_poll)), ORB_OK);
    if (ptopic._handle == nullptr) {
      thread_should_exit = true;
    }
    ASSERT_NE(ptopic._handle, nullptr) << "advertise failed: " << errno;

    int message_counter = 0;
    int num_messages = 20 * queue_size;

    while (message_counter < num_messages) {
      // simulate burst
      int burst_counter = 0;

      while (burst_counter++ <
             queue_size / 2 + 7) {  // make interval non-boundary aligned
        orb_pub_publish(ptopic, &pub_data);
        ++pub_data.val;
      }

      message_counter += burst_counter;
      usleep(20 * 1000);  // give subscriber a chance to catch up
    }

    num_messages_sent = pub_data.val;
    usleep(100 * 1000);
    thread_should_exit = true;
    orb_pub_destroy(&ptopic);
  }};

  int next_expected_val = 0;
  uevent_t poll = UEVENT_INITIALIZER;
  uevent_create(&poll);
  ASSERT_NE(poll._handle, nullptr);
  uevent_source_t src = orb_sub_event_source(sfd);
  ASSERT_NE(src._handle, nullptr);
  ASSERT_EQ(uevent_add(&poll, &src, 0), 0);

  while (!thread_should_exit.load()) {
    uevent_source_t ready[1] = {UEVENT_SOURCE_INITIALIZER};
    int poll_ret = uevent_loop(&poll, ready, 1, 500);
    ASSERT_GE(poll_ret, 0) << "poll error (" << poll_ret << "," << errno << ")";

    if (thread_should_exit.load()) {
      break;
    }

    ASSERT_NE(poll_ret, 0) << "poll timeout";

    if (poll_ret > 0 && ready[0]._handle == src._handle) {
      orb_sub_copy(sfd, &t);
      ASSERT_EQ(next_expected_val, t.val) << "copy mismatch";
      ++next_expected_val;
    }
  }

  ASSERT_EQ(uevent_remove(&poll, &src), 0);
  uevent_destroy(&poll);
  test_queue_thread.join();

  orb_sub_destroy(&sfd);

  ASSERT_EQ(num_messages_sent.load(), next_expected_val)
      << "number of sent and received messages mismatch";
}

TEST_F(UnitTest, poll_timeout_semantics) {
  orb_sub_t sub = ORB_SUB_INITIALIZER;
  EXPECT_EQ(orb_sub_create(&sub, ORB_ID(orb_test)), ORB_OK);
  ASSERT_NE(sub._handle, nullptr);

  // Drain any pre-existing updates from other test cases.
  orb_test_s drain{};
  for (int i = 0; i < 8 && orb_sub_check_update(sub); ++i) {
    ASSERT_EQ(orb_sub_copy(sub, &drain), ORB_OK);
  }

  uevent_t poll = UEVENT_INITIALIZER;
  uevent_create(&poll);
  ASSERT_NE(poll._handle, nullptr);
  uevent_source_t src = orb_sub_event_source(sub);
  ASSERT_NE(src._handle, nullptr);
  ASSERT_EQ(uevent_add(&poll, &src, 0), 0);

  uevent_source_t ready[1] = {UEVENT_SOURCE_INITIALIZER};

  const auto zero_start = std::chrono::steady_clock::now();
  EXPECT_EQ(uevent_loop(&poll, ready, 1, 0), 0);
  const auto zero_elapsed_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now() - zero_start)
          .count();
  EXPECT_LT(zero_elapsed_ms, 20);

  orb_pub_t pub = ORB_PUB_INITIALIZER;
  EXPECT_EQ(orb_pub_create(&pub, ORB_ID(orb_test)), ORB_OK);
  ASSERT_NE(pub._handle, nullptr);

  std::thread publisher([&]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(40));
    orb_test_s msg{};
    msg.val = 999;
    EXPECT_EQ(orb_pub_publish(pub, &msg), ORB_OK);
  });

  const auto neg_start = std::chrono::steady_clock::now();
  EXPECT_EQ(uevent_loop(&poll, ready, 1, -5), 1);
  const auto neg_elapsed_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now() - neg_start)
          .count();
  EXPECT_GE(neg_elapsed_ms, 25);
  EXPECT_EQ(ready[0]._handle, src._handle);

  publisher.join();
  ASSERT_EQ(uevent_remove(&poll, &src), 0);
  uevent_destroy(&poll);
  orb_pub_destroy(&pub);
  orb_sub_destroy(&sub);
}

TEST_F(UnitTest, concurrent_publish_copy_with_multiple_handles) {
  constexpr int kPublisherCount = 2;
  constexpr int kSubscriberCount = 3;
  constexpr int kMessagesPerPublisher = 200;

  orb_pub_t publications[kPublisherCount] = {};
  for (auto &publication : publications) {
    EXPECT_EQ(orb_pub_create(&publication, ORB_ID(orb_test_medium)), ORB_OK);
    ASSERT_NE(publication._handle, nullptr);
  }

  orb_sub_t subscriptions[kSubscriberCount] = {};
  for (auto &subscription : subscriptions) {
    EXPECT_EQ(orb_sub_create(&subscription, ORB_ID(orb_test_medium)), ORB_OK);
    ASSERT_NE(subscription._handle, nullptr);
  }

  std::atomic<int> total_published{0};
  std::atomic<int> total_copied{0};
  std::atomic_bool publishers_done{false};
  std::vector<std::thread> publishers;
  std::vector<std::thread> subscribers;

  for (int publisher_index = 0; publisher_index < kPublisherCount;
       ++publisher_index) {
    publishers.emplace_back([&, publisher_index]() {
      orb_test_medium_s msg{};
      for (int message_index = 0; message_index < kMessagesPerPublisher;
           ++message_index) {
        msg.timestamp = orb_absolute_time_us();
        msg.val = publisher_index * kMessagesPerPublisher + message_index;
        EXPECT_EQ(orb_pub_publish(publications[publisher_index], &msg), ORB_OK);
        total_published.fetch_add(1, std::memory_order_relaxed);
        std::this_thread::yield();
      }
    });
  }

  for (auto &subscription : subscriptions) {
    subscribers.emplace_back([&, subscription_ptr = &subscription]() {
      orb_test_medium_s msg{};
      while (!publishers_done.load(std::memory_order_acquire) ||
             orb_sub_check_update(*subscription_ptr)) {
        if (orb_sub_check_update(*subscription_ptr)) {
          EXPECT_EQ(orb_sub_copy(*subscription_ptr, &msg), ORB_OK);
          total_copied.fetch_add(1, std::memory_order_relaxed);
        } else {
          std::this_thread::yield();
        }
      }
    });
  }

  for (auto &publisher : publishers) {
    publisher.join();
  }
  publishers_done.store(true, std::memory_order_release);

  for (auto &subscriber : subscribers) {
    subscriber.join();
  }

  EXPECT_EQ(total_published.load(), kPublisherCount * kMessagesPerPublisher);
  EXPECT_GT(total_copied.load(), 0);

  for (auto &subscription : subscriptions) {
    orb_sub_destroy(&subscription);
  }
  for (auto &publication : publications) {
    orb_pub_destroy(&publication);
  }
}

TEST_F(UnitTest, subscription_interval_boundary_conditions) {
  // Test with interval = 0 (should always update)
  uorb::SubscriptionInterval<uorb::msg::orb_test> sub0(0, 0);
  orb_pub_t pub = ORB_PUB_INITIALIZER;
  EXPECT_EQ(orb_pub_create(&pub, ORB_ID(orb_test)), ORB_OK);
  ASSERT_NE(pub._handle, nullptr);

  orb_test_s data{};
  data.val = 1;
  ASSERT_EQ(orb_pub_publish(pub, &data), ORB_OK);
  EXPECT_TRUE(sub0.Updated());

  // Test with very large interval (should throttle)
  uorb::SubscriptionInterval<uorb::msg::orb_test> sub_large(1000000, 0);  // 1 second
  data.val = 2;
  ASSERT_EQ(orb_pub_publish(pub, &data), ORB_OK);
  // May or may not update depending on timing, but should not crash

  orb_pub_destroy(&pub);
}

TEST_F(UnitTest, topic_status_counter_saturation) {
  constexpr int kManyHandles = 300;
  constexpr uint16_t kMaxTrackedHandles = 127;

  // Capture counts before creating handles
  orb_status before{};
  ASSERT_EQ(orb_get_topic_status(ORB_ID(orb_test), 0, &before), ORB_OK);

  std::vector<orb_pub_t> pubs(kManyHandles, ORB_PUB_INITIALIZER);
  for (int i = 0; i < kManyHandles; ++i) {
    EXPECT_EQ(orb_pub_create(&pubs[i], ORB_ID(orb_test)), ORB_OK);
    ASSERT_NE(pubs[i]._handle, nullptr);
  }

  std::vector<orb_sub_t> subs(kManyHandles, ORB_SUB_INITIALIZER);
  for (int i = 0; i < kManyHandles; ++i) {
    EXPECT_EQ(orb_sub_create(&subs[i], ORB_ID(orb_test)), ORB_OK);
    ASSERT_NE(subs[i]._handle, nullptr);
  }

  orb_status after{};
  ASSERT_EQ(orb_get_topic_status(ORB_ID(orb_test), 0, &after), ORB_OK);

  // Counters should saturate at 127, not overflow
  EXPECT_LE(after.publisher_count, kMaxTrackedHandles);
  EXPECT_LE(after.subscriber_count, kMaxTrackedHandles);
  // If not already saturated before, we should hit the cap now
  if (before.publisher_count < kMaxTrackedHandles) {
    EXPECT_EQ(after.publisher_count, kMaxTrackedHandles);
  }
  if (before.subscriber_count < kMaxTrackedHandles) {
    EXPECT_EQ(after.subscriber_count, kMaxTrackedHandles);
  }

  for (auto &sub : subs) {
    orb_sub_destroy(&sub);
  }
  for (auto &pub : pubs) {
    orb_pub_destroy(&pub);
  }
}

TEST_F(UnitTest, orb_exists_returns_true_after_publish) {
  orb_pub_t pub = ORB_PUB_INITIALIZER;
  EXPECT_EQ(orb_pub_create(&pub, ORB_ID(orb_test)), ORB_OK);
  ASSERT_NE(pub._handle, nullptr);

  orb_test_s data{};
  data.val = 42;
  ASSERT_EQ(orb_pub_publish(pub, &data), ORB_OK);

  // Topic exists because there is an active publisher
  EXPECT_TRUE(orb_exists(ORB_ID(orb_test), 0));

  orb_pub_destroy(&pub);

  // After destroying the publisher, publisher_count drops to 0
  EXPECT_FALSE(orb_exists(ORB_ID(orb_test), 0));
}

TEST_F(UnitTest, orb_group_count_reports_correct_count) {
  orb_pub_t pubs[3] = {ORB_PUB_INITIALIZER, ORB_PUB_INITIALIZER,
                                ORB_PUB_INITIALIZER};
  unsigned instances[3]{};

  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(orb_pub_create_multi(&pubs[i], ORB_ID(orb_test_medium), &instances[i]), ORB_OK);
    EXPECT_EQ(instances[i], i);
  }

  EXPECT_EQ(orb_group_count(ORB_ID(orb_test_medium)), 3U);

  // Destroy one publisher
  orb_pub_destroy(&pubs[0]);
  EXPECT_EQ(orb_group_count(ORB_ID(orb_test_medium)), 2U);

  // Destroy remaining publishers
  orb_pub_destroy(&pubs[1]);
  orb_pub_destroy(&pubs[2]);
  EXPECT_EQ(orb_group_count(ORB_ID(orb_test_medium)), 0U);
}

TEST_F(UnitTest, orb_get_topic_status_reports_fields) {
  orb_pub_t pub = ORB_PUB_INITIALIZER;
  EXPECT_EQ(orb_pub_create(&pub, ORB_ID(orb_test)), ORB_OK);
  ASSERT_NE(pub._handle, nullptr);

  orb_sub_t sub = ORB_SUB_INITIALIZER;
  EXPECT_EQ(orb_sub_create(&sub, ORB_ID(orb_test)), ORB_OK);
  ASSERT_NE(sub._handle, nullptr);

  orb_test_s data{};
  data.val = 77;
  ASSERT_EQ(orb_pub_publish(pub, &data), ORB_OK);

  orb_status status{};
  ASSERT_EQ(orb_get_topic_status(ORB_ID(orb_test), 0, &status), ORB_OK);
  EXPECT_GE(status.publisher_count, 1);
  EXPECT_GE(status.subscriber_count, 1);
  EXPECT_GT(status.queue_size, 0);

  // Non-existent instance should return ORB_ERR_NO_ENTRY
  orb_status status2{};
  EXPECT_EQ(orb_get_topic_status(ORB_ID(orb_test), 3, &status2), ORB_ERR_NO_ENTRY);

  orb_sub_destroy(&sub);
  orb_pub_destroy(&pub);
}

TEST_F(UnitTest, publication_multi_wrapper_publishes_and_reports_instance) {
  // --- PublicationMultiData with embedded message storage ---
  uorb::PublicationMultiData<uorb::msg::orb_test_medium> pub_data;
  pub_data.data().val = 42;
  ASSERT_EQ(pub_data.Publish(), ORB_OK);
  ASSERT_LE(pub_data.instance(), ORB_MULTI_MAX_INSTANCES - 1);

  // Subscribe to the same instance and verify end-to-end delivery.
  orb_sub_t sub_data = ORB_SUB_INITIALIZER;
  ASSERT_EQ(orb_sub_create_multi(&sub_data, ORB_ID(orb_test_medium),
                                        pub_data.instance()),
            ORB_OK);
  ASSERT_NE(sub_data._handle, nullptr);

  // Drain the initial publish so the next publish is a fresh update.
  orb_test_medium_s recv{};
  if (orb_sub_check_update(sub_data)) {
    ASSERT_EQ(orb_sub_copy(sub_data, &recv), ORB_OK);
  }

  pub_data.data().val = 4242;
  ASSERT_EQ(pub_data.Publish(), ORB_OK);
  ASSERT_TRUE(orb_sub_check_update(sub_data));
  ASSERT_EQ(orb_sub_copy(sub_data, &recv), ORB_OK);
  EXPECT_EQ(recv.val, 4242);
  orb_sub_destroy(&sub_data);

  // --- PublicationMulti without embedded data (external data) ---
  uorb::PublicationMulti<uorb::msg::orb_test_medium> pub_ext;
  orb_test_medium_s ext{};
  ext.val = 99;
  ASSERT_EQ(pub_ext.Publish(ext), ORB_OK);
  ASSERT_LE(pub_ext.instance(), ORB_MULTI_MAX_INSTANCES - 1);

  orb_sub_t sub_ext = ORB_SUB_INITIALIZER;
  ASSERT_EQ(orb_sub_create_multi(&sub_ext, ORB_ID(orb_test_medium),
                                        pub_ext.instance()),
            ORB_OK);
  ASSERT_NE(sub_ext._handle, nullptr);

  // Drain the initial publish.
  if (orb_sub_check_update(sub_ext)) {
    ASSERT_EQ(orb_sub_copy(sub_ext, &recv), ORB_OK);
  }

  ext.val = 9999;
  ASSERT_EQ(pub_ext.Publish(ext), ORB_OK);
  ASSERT_TRUE(orb_sub_check_update(sub_ext));
  ASSERT_EQ(orb_sub_copy(sub_ext, &recv), ORB_OK);
  EXPECT_EQ(recv.val, 9999);
  orb_sub_destroy(&sub_ext);
}

TEST_F(UnitTest, subscription_interval_throttle_behavior) {
  uorb::SubscriptionInterval<uorb::msg::orb_test> sub(100000, 0);  // 100ms interval

  orb_pub_t pub = ORB_PUB_INITIALIZER;
  EXPECT_EQ(orb_pub_create(&pub, ORB_ID(orb_test)), ORB_OK);
  ASSERT_NE(pub._handle, nullptr);

  orb_test_s data{};
  orb_test_s dst{};

  // First publish and update (last_update_ = 0, so interval check passes)
  data.val = 55;
  ASSERT_EQ(orb_pub_publish(pub, &data), ORB_OK);
  EXPECT_TRUE(sub.Updated());
  EXPECT_TRUE(sub.Update(&dst));
  EXPECT_EQ(dst.val, 55);

  // After first Update with last_update_=0, CalculateNextUpdateTime sets
  // last_update_ to (now - interval), so the next Updated() still passes.
  // Consume this update as well to properly anchor last_update_.
  data.val = 56;
  ASSERT_EQ(orb_pub_publish(pub, &data), ORB_OK);
  EXPECT_TRUE(sub.Updated());
  EXPECT_TRUE(sub.Update(&dst));
  EXPECT_EQ(dst.val, 56);

  // Now last_update_ is anchored to the second Update time.
  // Publish new data but throttle should prevent immediate update.
  data.val = 57;
  ASSERT_EQ(orb_pub_publish(pub, &data), ORB_OK);
  EXPECT_FALSE(sub.Updated());

  // After sleeping past the interval, Updated() should return true
  usleep(110 * 1000);  // 110ms > 100ms interval
  EXPECT_TRUE(sub.Updated());

  // And Update should now succeed with the new value
  EXPECT_TRUE(sub.Update(&dst));
  EXPECT_EQ(dst.val, 57);

  orb_pub_destroy(&pub);
}

TEST_F(UnitTest, subscription_interval_runtime_change) {
  uorb::SubscriptionInterval<uorb::msg::orb_test> sub(1000000, 0);  // 1s interval

  orb_pub_t pub = ORB_PUB_INITIALIZER;
  EXPECT_EQ(orb_pub_create(&pub, ORB_ID(orb_test)), ORB_OK);
  ASSERT_NE(pub._handle, nullptr);

  orb_test_s data{};
  orb_test_s dst{};

  // First publish and update (last_update_ = 0)
  data.val = 1;
  ASSERT_EQ(orb_pub_publish(pub, &data), ORB_OK);
  EXPECT_TRUE(sub.Updated());
  EXPECT_TRUE(sub.Update(&dst));
  EXPECT_EQ(dst.val, 1);

  // Second publish and update (still passes due to CalculateNextUpdateTime
  // starting from last_update_=0; anchors last_update_ properly).
  data.val = 2;
  ASSERT_EQ(orb_pub_publish(pub, &data), ORB_OK);
  EXPECT_TRUE(sub.Updated());
  EXPECT_TRUE(sub.Update(&dst));
  EXPECT_EQ(dst.val, 2);

  // Publish new data but 1s interval should now throttle
  data.val = 3;
  ASSERT_EQ(orb_pub_publish(pub, &data), ORB_OK);
  EXPECT_FALSE(sub.Updated());

  // Disable throttling at runtime
  sub.set_interval_us(0);
  EXPECT_TRUE(sub.Updated());

  orb_pub_destroy(&pub);
}

TEST_F(UnitTest, publish_once_without_subscribers) {
  orb_test_s data{};
  data.val = 88;

  // Publish without any subscribers - should still succeed
  EXPECT_EQ(orb_pub_publish_once(ORB_ID(orb_test), &data), ORB_OK);

  // Create a subscriber
  orb_sub_t sub = ORB_SUB_INITIALIZER;
  EXPECT_EQ(orb_sub_create(&sub, ORB_ID(orb_test)), ORB_OK);
  ASSERT_NE(sub._handle, nullptr);

  // copy_once should retrieve the published data
  orb_test_s buffer{};
  EXPECT_EQ(orb_sub_copy_once(ORB_ID(orb_test), &buffer), ORB_OK);
  EXPECT_EQ(buffer.val, 88);

  orb_sub_destroy(&sub);
}

TEST_F(UnitTest, copy_once_on_never_published_topic) {
  static const struct orb_metadata never_published_topic = {
      "orb_never_published_test", sizeof(orb_test_large_s), 0, "", 1, ORB_DURABILITY_DURABLE, {}};
  orb_test_large_s buffer{};
  buffer.val = 12345;  // Set non-zero to detect if overwritten

  // Call copy_once on a topic that has never been published.
  // Should not crash. Returns ORB_ERR_NO_ENTRY since no data has been published.
  orb_err result = orb_sub_copy_once(&never_published_topic, &buffer);
  EXPECT_EQ(result, ORB_ERR_NO_ENTRY);
}

TEST_F(UnitTest, orb_strerror_and_no_entry_semantics) {
  EXPECT_STREQ(orb_strerror(ORB_OK), "OK");
  EXPECT_NE(orb_strerror(ORB_ERR_INVALID), nullptr);
  EXPECT_NE(orb_strerror(ORB_ERR_NO_MEM), nullptr);
  EXPECT_NE(orb_strerror(ORB_ERR_BUSY), nullptr);
  EXPECT_NE(orb_strerror(ORB_ERR_EXIST), nullptr);
  EXPECT_NE(orb_strerror(ORB_ERR_NO_ENTRY), nullptr);
  EXPECT_NE(orb_strerror(ORB_ERR_TIMEOUT), nullptr);
  EXPECT_NE(orb_strerror(ORB_ERR_AGAIN), nullptr);
  EXPECT_STREQ(orb_strerror(ORB_ERR_UNKNOWN), "Unknown error");

  static const struct orb_metadata empty_topic = {
      "orb_empty_copy_test", sizeof(orb_test_s), 0, "", 1, ORB_DURABILITY_DURABLE, {}};
  orb_sub_t sub = ORB_SUB_INITIALIZER;
  ASSERT_EQ(orb_sub_create(&sub, &empty_topic), ORB_OK);
  orb_test_s buf{};
  EXPECT_EQ(orb_sub_copy(sub, &buf), ORB_ERR_NO_ENTRY);
  orb_sub_destroy(&sub);
}

TEST_F(UnitTest, volatile_topic_skips_historical_samples_and_rejects_copy_once) {
  static const struct orb_metadata volatile_topic = {
      "orb_volatile_test", sizeof(orb_test_s), 0, "", 4, ORB_DURABILITY_VOLATILE, {}};

  orb_pub_t pub = ORB_PUB_INITIALIZER;
  ASSERT_EQ(orb_pub_create(&pub, &volatile_topic), ORB_OK);

  // Publish before any subscriber exists.
  orb_test_s sent{};
  sent.val = 100;
  ASSERT_EQ(orb_pub_publish(pub, &sent), ORB_OK);

  // copy_once on a volatile topic must return ORB_ERR_NO_ENTRY.
  orb_test_s recv{};
  recv.val = -1;
  EXPECT_EQ(orb_sub_copy_once(&volatile_topic, &recv), ORB_ERR_NO_ENTRY);
  EXPECT_EQ(recv.val, -1);

  // Late subscriber must NOT see historical publication as an update.
  orb_sub_t sub = ORB_SUB_INITIALIZER;
  ASSERT_EQ(orb_sub_create(&sub, &volatile_topic), ORB_OK);
  EXPECT_FALSE(orb_sub_check_update(sub));

  // New publication after subscribing must be visible and delivered.
  sent.val = 200;
  ASSERT_EQ(orb_pub_publish(pub, &sent), ORB_OK);
  EXPECT_TRUE(orb_sub_check_update(sub));
  ASSERT_EQ(orb_sub_copy(sub, &recv), ORB_OK);
  EXPECT_EQ(recv.val, 200);
  EXPECT_FALSE(orb_sub_check_update(sub));

  orb_sub_destroy(&sub);
  orb_pub_destroy(&pub);
}

TEST_F(UnitTest, concurrent_lookup_exists_and_lifecycle) {
  constexpr int kIterations = 200;
  std::atomic<bool> stop{false};

  std::thread pub_thread([&]() {
    orb_test_s msg{};
    for (int i = 0; i < kIterations; ++i) {
      orb_pub_t pub = ORB_PUB_INITIALIZER;
      ASSERT_EQ(orb_pub_create(&pub, ORB_ID(orb_test)), ORB_OK);
      msg.val = i;
      EXPECT_EQ(orb_pub_publish(pub, &msg), ORB_OK);
      orb_pub_destroy(&pub);
    }
    stop.store(true, std::memory_order_release);
  });

  std::thread sub_thread([&]() {
    orb_test_s msg{};
    while (!stop.load(std::memory_order_acquire)) {
      orb_sub_t sub = ORB_SUB_INITIALIZER;
      ASSERT_EQ(orb_sub_create(&sub, ORB_ID(orb_test)), ORB_OK);
      if (orb_sub_check_update(sub)) {
        EXPECT_EQ(orb_sub_copy(sub, &msg), ORB_OK);
      }
      orb_sub_destroy(&sub);
    }
  });

  std::thread status_thread([&]() {
    while (!stop.load(std::memory_order_acquire)) {
      (void)orb_exists(ORB_ID(orb_test), 0);
      (void)orb_group_count(ORB_ID(orb_test));
      orb_status status{};
      (void)orb_get_topic_status(ORB_ID(orb_test), 0, &status);
    }
  });

  pub_thread.join();
  sub_thread.join();
  status_thread.join();
}

TEST_F(UnitTest, cpp_convenience_publish_copy_and_event_source) {
  orb_test_s sent{};
  sent.val = 7331;
  ASSERT_EQ(uorb::publish<uorb::msg::orb_test>(sent), ORB_OK);

  orb_test_s received{};
  ASSERT_EQ(uorb::copy<uorb::msg::orb_test>(received), ORB_OK);
  EXPECT_EQ(received.val, 7331);

  uorb::Subscription<uorb::msg::orb_test> sub;
  // Drain initial sample
  while (sub.Updated()) {
    sub.Copy(&received);
  }

  uevent_t base = UEVENT_INITIALIZER;
  ASSERT_EQ(uevent_create(&base), UEVENT_OK);
  uevent_source_t src = sub.event_source();
  ASSERT_NE(src._handle, nullptr);
  ASSERT_EQ(uevent_add(&base, &src, 0), UEVENT_OK);

  sent.val = 8442;
  ASSERT_EQ(uorb::publish<uorb::msg::orb_test>(sent), ORB_OK);

  uevent_source_t ready[1] = {UEVENT_SOURCE_INITIALIZER};
  ASSERT_EQ(uevent_loop(&base, ready, 1, 100), 1);
  EXPECT_EQ(ready[0]._handle, src._handle);
  ASSERT_TRUE(sub.Update(&received));
  EXPECT_EQ(received.val, 8442);

  ASSERT_EQ(uevent_remove(&base, &src), UEVENT_OK);
  uevent_destroy(&base);
}

TEST_F(UnitTest, generated_msg_respects_orb_durability_constant) {
  EXPECT_EQ(ORB_ID(orb_test)->o_durability, ORB_DURABILITY_DURABLE);
  EXPECT_EQ(ORB_ID(orb_test_volatile)->o_durability, ORB_DURABILITY_VOLATILE);
  EXPECT_EQ(ORB_ID(orb_test_volatile)->o_queue_size, 4U);

  orb_test_volatile_s sent{};
  sent.val = 11;
  ASSERT_EQ(uorb::publish<uorb::msg::orb_test_volatile>(sent), ORB_OK);

  orb_test_volatile_s recv{};
  EXPECT_EQ(uorb::copy<uorb::msg::orb_test_volatile>(recv), ORB_ERR_NO_ENTRY);

  uorb::Subscription<uorb::msg::orb_test_volatile> sub;
  EXPECT_FALSE(sub.Updated());

  sent.val = 22;
  ASSERT_EQ(uorb::publish<uorb::msg::orb_test_volatile>(sent), ORB_OK);
  EXPECT_TRUE(sub.Updated());
  EXPECT_TRUE(sub.Update(&recv));
  EXPECT_EQ(recv.val, 22);
}

TEST_F(UnitTest, lock_free_copy_never_observes_torn_large_payload) {
  struct OddSizePayload {
    uint64_t seq;
    uint64_t check_xor;
    uint8_t bytes[131];  // intentionally non-multiple of sizeof(size_t)
  };
  static const struct orb_metadata torn_test_topic = {
      "orb_torn_read_test", sizeof(OddSizePayload), 0, "", 1, ORB_DURABILITY_DURABLE, {}};

  orb_pub_t pub = ORB_PUB_INITIALIZER;
  ASSERT_EQ(orb_pub_create(&pub, &torn_test_topic), ORB_OK);

  OddSizePayload init{};
  init.seq = 1;
  init.check_xor = 1 ^ 0xA5A5A5A55A5A5A5AULL;
  std::memset(init.bytes, 1, sizeof(init.bytes));
  ASSERT_EQ(orb_pub_publish(pub, &init), ORB_OK);

  constexpr int kWriters = 2;
  constexpr int kReaders = 4;
  constexpr int kWritesPerThread = 2000;
  std::atomic<bool> writers_done{false};
  std::atomic<int> torn_count{0};
  std::atomic<int> total_reads{0};

  std::vector<std::thread> readers;
  readers.reserve(kReaders);
  for (int r = 0; r < kReaders; ++r) {
    readers.emplace_back([&]() {
      orb_sub_t sub = ORB_SUB_INITIALIZER;
      ASSERT_EQ(orb_sub_create(&sub, &torn_test_topic), ORB_OK);
      OddSizePayload msg{};
      int reads = 0;
      while (!writers_done.load(std::memory_order_acquire) || reads < 10) {
        if (orb_sub_copy(sub, &msg) == ORB_OK) {
          ++reads;
          if ((msg.seq ^ 0xA5A5A5A55A5A5A5AULL) != msg.check_xor) {
            torn_count.fetch_add(1, std::memory_order_relaxed);
          }
          const uint8_t expected_byte = static_cast<uint8_t>(msg.seq & 0xFFu);
          for (size_t i = 0; i < sizeof(msg.bytes); ++i) {
            if (msg.bytes[i] != expected_byte) {
              torn_count.fetch_add(1, std::memory_order_relaxed);
              break;
            }
          }
        }
      }
      total_reads.fetch_add(reads, std::memory_order_relaxed);
      orb_sub_destroy(&sub);
    });
  }

  std::vector<std::thread> writers;
  writers.reserve(kWriters);
  for (int w = 0; w < kWriters; ++w) {
    writers.emplace_back([&, w]() {
      OddSizePayload msg{};
      for (int i = 1; i <= kWritesPerThread; ++i) {
        const uint64_t seq = static_cast<uint64_t>(w) * 1000000ULL + static_cast<uint64_t>(i);
        msg.seq = seq;
        msg.check_xor = seq ^ 0xA5A5A5A55A5A5A5AULL;
        std::memset(msg.bytes, static_cast<int>(seq & 0xFFu), sizeof(msg.bytes));
        EXPECT_EQ(orb_pub_publish(pub, &msg), ORB_OK);
      }
    });
  }

  for (auto &t : writers) t.join();
  writers_done.store(true, std::memory_order_release);
  for (auto &t : readers) t.join();

  EXPECT_EQ(torn_count.load(), 0);
  EXPECT_GT(total_reads.load(), 0);
  orb_pub_destroy(&pub);
}

TEST_F(UnitTest, lock_free_copy_queued_zero_extra_slots_and_preemption_skip) {
  static const struct orb_metadata queued_topic = {
      "orb_lock_free_q4_test", sizeof(orb_test_s), 0, "", 4, ORB_DURABILITY_DURABLE, {}};

  orb_pub_t pub = ORB_PUB_INITIALIZER;
  orb_sub_t sub = ORB_SUB_INITIALIZER;
  ASSERT_EQ(orb_pub_create(&pub, &queued_topic), ORB_OK);
  ASSERT_EQ(orb_sub_create(&sub, &queued_topic), ORB_OK);

  auto *node = uorb::DeviceMaster::get_instance().GetDeviceNode(queued_topic, 0);
  ASSERT_NE(node, nullptr);
  // Option B: queue_size >= 2 uses zero extra physical slots.
  EXPECT_EQ(physical_slots(*node), 4u);

  // Fill all 4 logical slots; verify full depth when writer is idle.
  for (int i = 10; i < 14; ++i) {
    orb_test_s msg{};
    msg.val = i;
    ASSERT_EQ(orb_pub_publish(pub, &msg), ORB_OK);
  }
  for (int i = 10; i < 14; ++i) {
    orb_test_s got{};
    ASSERT_EQ(orb_sub_copy(sub, &got), ORB_OK);
    EXPECT_EQ(got.val, i);
  }

  // Publish 4 more items (20..23), then simulate a low-priority writer preempted
  // mid-Publish of the 5th item (overwriting oldest slot holding 20).
  for (int i = 20; i < 24; ++i) {
    orb_test_s msg{};
    msg.val = i;
    ASSERT_EQ(orb_pub_publish(pub, &msg), ORB_OK);
  }
  simulate_in_progress_write(*node);

  // A high-priority reader preempting the writer must NOT livelock: it skips
  // the in-progress oldest slot (20) and cleanly reads 21, 22, 23.
  orb_test_s got{};
  ASSERT_EQ(orb_sub_copy(sub, &got), ORB_OK);
  EXPECT_EQ(got.val, 21);
  ASSERT_EQ(orb_sub_copy(sub, &got), ORB_OK);
  EXPECT_EQ(got.val, 22);
  ASSERT_EQ(orb_sub_copy(sub, &got), ORB_OK);
  EXPECT_EQ(got.val, 23);

  // Restore slot by completing a real publish and verify normal operation resumes.
  orb_test_s final_msg{};
  final_msg.val = 99;
  ASSERT_EQ(orb_pub_publish(pub, &final_msg), ORB_OK);
  ASSERT_EQ(orb_sub_copy(sub, &got), ORB_OK);
  EXPECT_EQ(got.val, 99);

  orb_sub_destroy(&sub);
  orb_pub_destroy(&pub);
}

TEST_F(UnitTest, topic_enumeration_and_last_publish_time_us) {
  static const struct orb_metadata enum_topic_a = {
      "orb_enum_test_a", sizeof(orb_test_s), 0, "", 2, ORB_DURABILITY_DURABLE, {}};
  static const struct orb_metadata enum_topic_b = {
      "orb_enum_test_b", sizeof(orb_test_s), 0, "", 1, ORB_DURABILITY_VOLATILE, {}};

  // Null visitor returns 0 cleanly.
  EXPECT_EQ(orb_for_each_topic(nullptr, nullptr), 0u);

  orb_sub_t sub_a = ORB_SUB_INITIALIZER;
  ASSERT_EQ(orb_sub_create(&sub_a, &enum_topic_a), ORB_OK);

  // Before first publish, last_publish_time_us must be 0.
  orb_status st_before{};
  ASSERT_EQ(orb_get_topic_status(&enum_topic_a, 0, &st_before), ORB_OK);
  EXPECT_EQ(st_before.last_publish_time_us, 0u);

  const orb_abstime_us t_before = orb_absolute_time_us();
  orb_pub_t pub_a = ORB_PUB_INITIALIZER;
  ASSERT_EQ(orb_pub_create(&pub_a, &enum_topic_a), ORB_OK);
  orb_test_s msg{};
  msg.val = 7;
  ASSERT_EQ(orb_pub_publish(pub_a, &msg), ORB_OK);
  const orb_abstime_us t_after = orb_absolute_time_us();

  orb_status st_after{};
  ASSERT_EQ(orb_get_topic_status(&enum_topic_a, 0, &st_after), ORB_OK);
  EXPECT_GE(st_after.last_publish_time_us, t_before);
  EXPECT_LE(st_after.last_publish_time_us, t_after);

  // Create second topic and verify orb_for_each_topic visits both with accurate snapshots,
  // and safely allows calling other uORB APIs inside the visitor (no DeviceMaster::lock_ held).
  orb_pub_t pub_b = ORB_PUB_INITIALIZER;
  ASSERT_EQ(orb_pub_create(&pub_b, &enum_topic_b), ORB_OK);

  struct VisitCtx {
    const orb_metadata *meta_a;
    const orb_metadata *meta_b;
    bool found_a{false};
    bool found_b{false};
    uint64_t a_last_pub{0};
    size_t stop_after{0};
    size_t count{0};
  } ctx{&enum_topic_a, &enum_topic_b, false, false, 0, 0, 0};

  const size_t total = orb_for_each_topic(
      [](const orb_metadata *meta, unsigned int instance, const orb_status *status,
         void *user_ctx) -> bool {
        auto *c = static_cast<VisitCtx *>(user_ctx);
        ++c->count;
        // Reentrant uORB introspection call inside visitor must not deadlock.
        orb_status check{};
        EXPECT_EQ(orb_get_topic_status(meta, instance, &check), ORB_OK);
        if (meta == c->meta_a && instance == 0) {
          c->found_a = true;
          c->a_last_pub = status->last_publish_time_us;
          EXPECT_EQ(status->queue_size, 2u);
          EXPECT_EQ(status->subscriber_count, 1u);
          EXPECT_EQ(status->publisher_count, 1u);
        } else if (meta == c->meta_b && instance == 0) {
          c->found_b = true;
          EXPECT_EQ(status->last_publish_time_us, 0u);
        }
        return (c->stop_after == 0) || (c->count < c->stop_after);
      },
      &ctx);

  EXPECT_EQ(total, ctx.count);
  EXPECT_GE(total, 2u);
  EXPECT_TRUE(ctx.found_a);
  EXPECT_TRUE(ctx.found_b);
  EXPECT_EQ(ctx.a_last_pub, st_after.last_publish_time_us);

  // Early termination when visitor returns false.
  ctx.count = 0;
  ctx.stop_after = 1;
  const size_t early = orb_for_each_topic(
      [](const orb_metadata *, unsigned int, const orb_status *, void *user_ctx) -> bool {
        auto *c = static_cast<VisitCtx *>(user_ctx);
        ++c->count;
        return c->count < c->stop_after;
      },
      &ctx);
  EXPECT_EQ(early, 1u);

  // Compile-time FNV-1a o_name_hash populated by ORB_DEFINE.
  static_assert(uorb::topic_hash("orb_test") != 0u, "FNV-1a hash should be non-zero");
  EXPECT_EQ(ORB_ID(orb_test)->o_name_hash, uorb::topic_hash("orb_test"));
  EXPECT_EQ(ORB_ID(orb_test_medium)->o_name_hash, uorb::topic_hash("orb_test_medium"));
  EXPECT_NE(ORB_ID(orb_test)->o_name_hash, ORB_ID(orb_test_medium)->o_name_hash);

  orb_sub_destroy(&sub_a);
  orb_pub_destroy(&pub_a);
  orb_pub_destroy(&pub_b);
}

TEST_F(UnitTest, custom_allocator_and_zero_heap_preallocate) {
  static const struct orb_metadata prealloc_topic = {
      "orb_prealloc_test", sizeof(orb_test_s), 0, "", 2, ORB_DURABILITY_DURABLE,
      uorb::topic_hash("orb_prealloc_test"), {}};

  // Half-NULL allocator hooks must be rejected.
  auto dummy_alloc = [](size_t n) -> void * { return std::malloc(n); };
  auto dummy_free = [](void *p) { std::free(p); };
  EXPECT_EQ(orb_set_allocator(dummy_alloc, nullptr), ORB_ERR_INVALID);
  EXPECT_EQ(orb_set_allocator(nullptr, dummy_free), ORB_ERR_INVALID);

  // Invalid preallocate arguments rejected.
  EXPECT_EQ(orb_topic_preallocate(nullptr, 0), ORB_ERR_INVALID);
  EXPECT_EQ(orb_topic_preallocate(&prealloc_topic, ORB_MULTI_MAX_INSTANCES), ORB_ERR_INVALID);

  // Install counting allocator.
  static std::atomic<int> alloc_calls{0};
  static std::atomic<int> free_calls{0};
  alloc_calls.store(0);
  free_calls.store(0);

  ASSERT_EQ(orb_set_allocator(
                [](size_t n) -> void * {
                  alloc_calls.fetch_add(1, std::memory_order_relaxed);
                  return std::malloc(n);
                },
                [](void *p) {
                  free_calls.fetch_add(1, std::memory_order_relaxed);
                  std::free(p);
                }),
            ORB_OK);

  // Pre-allocate DeviceNode + ring buffer: must invoke custom allocator twice.
  ASSERT_EQ(orb_topic_preallocate(&prealloc_topic, 0), ORB_OK);
  EXPECT_EQ(alloc_calls.load(), 2);

  // Second preallocate is idempotent (0 new allocations).
  ASSERT_EQ(orb_topic_preallocate(&prealloc_topic, 0), ORB_OK);
  EXPECT_EQ(alloc_calls.load(), 2);

  orb_pub_t pub = ORB_PUB_INITIALIZER;
  ASSERT_EQ(orb_pub_create(&pub, &prealloc_topic), ORB_OK);
  orb_sub_t sub = ORB_SUB_INITIALIZER;
  ASSERT_EQ(orb_sub_create(&sub, &prealloc_topic), ORB_OK);
  EXPECT_EQ(alloc_calls.load(), 3);  // +1 for ReceiverLocal

  // Hot path: Publish() and Copy() must perform ZERO allocations.
  const int allocs_before_hot_path = alloc_calls.load();
  for (int i = 1; i <= 10; ++i) {
    orb_test_s m{};
    m.val = i;
    ASSERT_EQ(orb_pub_publish(pub, &m), ORB_OK);
    orb_test_s out{};
    ASSERT_EQ(orb_sub_copy(sub, &out), ORB_OK);
    EXPECT_EQ(out.val, i);
  }
  EXPECT_EQ(alloc_calls.load(), allocs_before_hot_path);

  // Destroy subscriber before restoring default allocator so free_calls matches.
  orb_sub_destroy(&sub);
  EXPECT_EQ(free_calls.load(), 1);
  orb_pub_destroy(&pub);

  // Restore default allocator.
  ASSERT_EQ(orb_set_allocator(nullptr, nullptr), ORB_OK);
}

TEST_F(UnitTest, loan_publish_discard_busy_and_raii_wrapper) {
  static const struct orb_metadata loan_topic = {
      "orb_loan_test", sizeof(orb_test_large_s), 0, "", 2, ORB_DURABILITY_DURABLE,
      uorb::topic_hash("orb_loan_test"), {}};

  orb_pub_t pub = ORB_PUB_INITIALIZER;
  orb_sub_t sub = ORB_SUB_INITIALIZER;
  ASSERT_EQ(orb_pub_create(&pub, &loan_topic), ORB_OK);
  ASSERT_EQ(orb_sub_create(&sub, &loan_topic), ORB_OK);

  // 1) Invalid arguments.
  void *raw = nullptr;
  EXPECT_EQ(orb_pub_loan(ORB_PUB_INITIALIZER, &raw), ORB_ERR_INVALID);
  EXPECT_EQ(orb_pub_loan(pub, nullptr), ORB_ERR_INVALID);
  EXPECT_EQ(orb_pub_publish_loaned(pub, nullptr), ORB_ERR_INVALID);

  // 2) C API loan -> busy on second loan -> publish_loaned -> copy.
  ASSERT_EQ(orb_pub_loan(pub, &raw), ORB_OK);
  ASSERT_NE(raw, nullptr);

  void *raw2 = nullptr;
  EXPECT_EQ(orb_pub_loan(pub, &raw2), ORB_ERR_BUSY);
  EXPECT_EQ(raw2, nullptr);

  auto *large = static_cast<orb_test_large_s *>(raw);
  EXPECT_EQ(large->val, 0);  // pre-zeroed
  large->val = 777;
  ASSERT_EQ(orb_pub_publish_loaned(pub, raw), ORB_OK);
  // Publishing same pointer again without re-loaning must fail.
  EXPECT_EQ(orb_pub_publish_loaned(pub, raw), ORB_ERR_INVALID);

  orb_test_large_s received{};
  ASSERT_EQ(orb_sub_copy(sub, &received), ORB_OK);
  EXPECT_EQ(received.val, 777);

  // 3) Discard releases without publishing.
  ASSERT_EQ(orb_pub_loan(pub, &raw), ORB_OK);
  static_cast<orb_test_large_s *>(raw)->val = 888;
  orb_pub_discard_loan(pub, raw);
  EXPECT_FALSE(orb_sub_check_update(sub));

  orb_sub_destroy(&sub);
  orb_pub_destroy(&pub);

  // 4) C++ RAII LoanedMessage: move, auto-discard, and Publish().
  uorb::Publication<uorb::msg::orb_test_large> cpp_pub;
  uorb::Subscription<uorb::msg::orb_test_large> cpp_sub;
  ASSERT_TRUE(cpp_sub.Subscribe());
  // Drain any pre-existing sample.
  if (cpp_sub.Updated()) {
    orb_test_large_s drain{};
    cpp_sub.Copy(drain);
  }

  {
    auto loan = cpp_pub.Loan();
    ASSERT_TRUE(loan);
    // Second concurrent loan must fail (returns empty LoanedMessage).
    EXPECT_FALSE(cpp_pub.Loan());
    loan->val = 1234;
    // Scope exit without Publish() must auto-discard.
  }
  EXPECT_FALSE(cpp_sub.Updated());

  {
    auto loan = cpp_pub.Loan();
    ASSERT_TRUE(loan);
    auto moved = std::move(loan);
    EXPECT_FALSE(loan);
    ASSERT_TRUE(moved);
    moved->val = 4321;
    ASSERT_EQ(moved.Publish(), ORB_OK);
    EXPECT_FALSE(moved);
  }
  orb_test_large_s out{};
  ASSERT_TRUE(cpp_sub.Update(out));
  EXPECT_EQ(out.val, 4321);
}

}  // namespace uORBTest
