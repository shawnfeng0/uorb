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

#pragma once

#include <gtest/gtest.h>
#include <unistd.h>
#include <uevent/uevent.h>
#include <uorb/topics/orb_test.h>
#include <uorb/topics/orb_test_large.h>
#include <uorb/topics/orb_test_medium.h>
#include <uorb/topics/orb_test_queue_poll.h>
#include <uorb/uorb.h>
#include <uorb_uevent/uorb_uevent.h>

#include <cerrno>
#include <cmath>
#include <thread>

#include "device_master.h"
#include "device_node.h"

#define ORB_DEBUG(fmt, ...) printf("[DEBUG] " fmt "\n", ##__VA_ARGS__)
#define ORB_INFO(fmt, ...) printf("[INFO] " fmt "\n", ##__VA_ARGS__)
#define ORB_ERROR(fmt, ...) printf("[ERROR] " fmt "\n", ##__VA_ARGS__)

namespace uORBTest {
class UnitTest;
}

class uORBTest::UnitTest : public testing::Test {
 public:
  // Assist in testing the wrap-around situation
  static void set_generation(uorb::DeviceNode &node, unsigned generation) {
    node.generation_ = generation;
  }

  template <typename S>
  void latency_test(const orb_metadata *T);

  // Disallow copy
  UnitTest(const uORBTest::UnitTest & /*unused*/) = delete;
  void SetUp() override {}
  void TearDown() override {}
  void TestBody() override {}
  UnitTest() = default;
};

template <typename S>
void uORBTest::UnitTest::latency_test(const orb_metadata *T) {
  S pub_data{};
  pub_data.val = 308;
  pub_data.timestamp = orb_absolute_time_us();

  orb_publisher_t pfd0 = ORB_PUBLISHER_INITIALIZER;
  orb_publisher_create(&pfd0, T);
  ASSERT_NE(pfd0._handle, nullptr) << "orb_publisher_create failed: " << errno;

  orb_publisher_publish(&pfd0, &pub_data);

  std::atomic<bool> pub_sub_test_passed{false};
  std::atomic<float> mean_latency{0.0f};

  /* test pub / sub latency */

  // Can'pub_data pass a pointer in args, must be a null terminated
  // array of strings because the strings are copied to
  // prevent access if the caller data goes out of scope
  std::thread pub_sub_latency_thread{[&]() {
    /* poll on test topic and output latency */
    float latency_integral = 0.0f;

    /* wakeup source(s) */
    orb_subscriber_t test_multi_sub = ORB_SUBSCRIBER_INITIALIZER;
    orb_subscriber_t test_multi_sub_medium = ORB_SUBSCRIBER_INITIALIZER;
    orb_subscriber_t test_multi_sub_large = ORB_SUBSCRIBER_INITIALIZER;
    orb_subscriber_create(&test_multi_sub, ORB_ID(orb_test));
    orb_subscriber_create(&test_multi_sub_medium, ORB_ID(orb_test_medium));
    orb_subscriber_create(&test_multi_sub_large, ORB_ID(orb_test_large));

    orb_test_large_s pub_data_large{};

    /* clear all ready flags */
    orb_subscriber_copy(&test_multi_sub, &pub_data_large);
    orb_subscriber_copy(&test_multi_sub_medium, &pub_data_large);
    orb_subscriber_copy(&test_multi_sub_large, &pub_data_large);

    uevent_t poll = UEVENT_INITIALIZER;
    uevent_create(&poll);
    uevent_source_t src1 = UEVENT_SOURCE_INITIALIZER;
    uorb_subscriber_create_source(&src1, &test_multi_sub);
    uevent_source_t src2 = UEVENT_SOURCE_INITIALIZER;
    uorb_subscriber_create_source(&src2, &test_multi_sub_medium);
    uevent_source_t src3 = UEVENT_SOURCE_INITIALIZER;
    uorb_subscriber_create_source(&src3, &test_multi_sub_large);
    uevent_add(&poll, &src1, 0);
    uevent_add(&poll, &src2, 0);
    uevent_add(&poll, &src3, 0);

    const unsigned max_runs = 1000;
    int current_value = pub_data_large.val;
    int num_missed = 0;

    // timings has to be on the heap to keep frame size below 2048 bytes
    auto *timings = new unsigned[max_runs]{};
    unsigned timing_min = 9999999, timing_max = 0;

    for (unsigned i = 0; i < max_runs; i++) {
      /* wait for up to 500ms for data */
      uevent_source_t ready[3] = {UEVENT_SOURCE_INITIALIZER,
                                   UEVENT_SOURCE_INITIALIZER,
                                   UEVENT_SOURCE_INITIALIZER};
      int pret = uevent_loop(&poll, ready, 3, 500);

      for (int j = 0; j < pret; ++j) {
        if (ready[j]._handle == src1._handle) {
          orb_subscriber_copy(&test_multi_sub, &pub_data_large);

        } else if (ready[j]._handle == src2._handle) {
          orb_subscriber_copy(&test_multi_sub_medium, &pub_data_large);

        } else if (ready[j]._handle == src3._handle) {
          orb_subscriber_copy(&test_multi_sub_large, &pub_data_large);
        }
      }

      if (pret < 0) {
        ORB_ERROR("poll error %d, %d", pret, errno);
        continue;
      }

      num_missed += pub_data_large.val - current_value - 1;
      current_value = pub_data_large.val;

      auto elt = (unsigned)orb_elapsed_time_us(pub_data_large.timestamp);
      latency_integral += elt;
      timings[i] = elt;

      if (elt > timing_max) {
        timing_max = elt;
      }

      if (elt < timing_min) {
        timing_min = elt;
      }
    }

    uevent_remove(&poll, &src1);
    uevent_remove(&poll, &src2);
    uevent_remove(&poll, &src3);
    uorb_subscriber_destroy_source(&src1);
    uorb_subscriber_destroy_source(&src2);
    uorb_subscriber_destroy_source(&src3);
    uevent_destroy(&poll);

    orb_subscriber_destroy(&test_multi_sub);
    orb_subscriber_destroy(&test_multi_sub_medium);
    orb_subscriber_destroy(&test_multi_sub_large);

    float std_dev = 0.f;
    float mean = latency_integral / max_runs;

    for (unsigned i = 0; i < max_runs; i++) {
      float diff = (float)timings[i] - mean;
      std_dev += diff * diff;
    }

    delete[] timings;

    ORB_INFO("uORB version: %s", orb_version());
    ORB_INFO("mean:    %8.4f us", static_cast<double>(mean));
    ORB_INFO("std dev: %8.4f us",
             static_cast<double>(sqrtf(std_dev / (max_runs - 1))));
    ORB_INFO("min:     %3i us", timing_min);
    ORB_INFO("max:     %3i us", timing_max);
    ORB_INFO("missed topic updates: %i", num_missed);

    pub_sub_test_passed = true;

    mean_latency = static_cast<float>(latency_integral / max_runs);
  }};
  /* give the test task some data */
  while (!pub_sub_test_passed) {
    ++pub_data.val;
    pub_data.timestamp = orb_absolute_time_us();

    ASSERT_EQ(orb_publisher_publish(&pfd0, &pub_data), ORB_OK) << "mult. pub0 timing fail";

    /* simulate >800 Hz system operation */
    usleep(1000);
  }

  orb_publisher_destroy(&pfd0);

  pub_sub_latency_thread.join();

  ASSERT_LT(mean_latency.load(), 200.0f);
}
