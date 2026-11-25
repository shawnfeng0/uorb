/****************************************************************************
 *
 * uORB + uevent EventSource integration tests.
 *
 ****************************************************************************/

#include <gtest/gtest.h>
#include <uevent/uevent.h>
#include <uorb/topics/orb_test.h>
#include <uorb/topics/orb_test_medium.h>
#include <uorb/uorb.h>

#include <chrono>
#include <thread>

// ---- orb_sub_event_source ----

TEST(UorbEventSourceTest, EventSourceFromSubscriber) {
  orb_sub_t sub = ORB_SUB_INITIALIZER;
  EXPECT_EQ(orb_sub_create(&sub, ORB_ID(orb_test)), ORB_OK);
  ASSERT_NE(sub._handle, nullptr);

  uevent_source_t src = orb_sub_event_source(sub);
  ASSERT_NE(src._handle, nullptr);

  // Calling uevent_source_destroy() on a borrowed subscriber source must be a
  // safe no-op (resets handle without deleting ReceiverLocal).
  uevent_source_destroy(&src);
  EXPECT_EQ(src._handle, nullptr);

  // Subscriber must remain valid and usable afterwards.
  uevent_source_t src2 = orb_sub_event_source(sub);
  EXPECT_NE(src2._handle, nullptr);

  orb_sub_destroy(&sub);
}

TEST(UorbEventSourceTest, EventSourceFromNullSubscriberReturnsEmpty) {
  orb_sub_t empty_sub = ORB_SUB_INITIALIZER;
  uevent_source_t src = orb_sub_event_source(empty_sub);
  EXPECT_EQ(src._handle, nullptr);
}

// ---- Source lifecycle with uevent_add/remove ----

TEST(UorbEventSourceTest, SourceAddRemoveLoop) {
  orb_sub_t sub = ORB_SUB_INITIALIZER;
  EXPECT_EQ(orb_sub_create(&sub, ORB_ID(orb_test)), ORB_OK);
  ASSERT_NE(sub._handle, nullptr);

  uevent_source_t src = orb_sub_event_source(sub);
  ASSERT_NE(src._handle, nullptr);

  uevent_t base = UEVENT_INITIALIZER;
  ASSERT_EQ(uevent_create(&base), 0);
  ASSERT_NE(base._handle, nullptr);

  // Add, loop (no data), remove — should work repeatedly
  for (int i = 0; i < 5; ++i) {
    ASSERT_EQ(uevent_add(&base, &src, 0), 0);
    uevent_source_t ready[1] = {UEVENT_SOURCE_INITIALIZER};
    uevent_loop(&base, ready, 1, 0);
    ASSERT_EQ(uevent_remove(&base, &src), 0);
  }

  uevent_destroy(&base);
  orb_sub_destroy(&sub);
}

// ---- Publish triggers source ready ----

TEST(UorbEventSourceTest, PublishTriggersReady) {
  orb_pub_t pub = ORB_PUB_INITIALIZER;
  EXPECT_EQ(orb_pub_create(&pub, ORB_ID(orb_test)), ORB_OK);
  ASSERT_NE(pub._handle, nullptr);

  orb_sub_t sub = ORB_SUB_INITIALIZER;
  EXPECT_EQ(orb_sub_create(&sub, ORB_ID(orb_test)), ORB_OK);
  ASSERT_NE(sub._handle, nullptr);

  // Drain initial data
  orb_test_s drain{};
  for (int i = 0; i < 8 && orb_sub_check_update(sub); ++i) {
    orb_sub_copy(sub, &drain);
  }

  uevent_source_t src = orb_sub_event_source(sub);
  ASSERT_NE(src._handle, nullptr);

  uevent_t base = UEVENT_INITIALIZER;
  ASSERT_EQ(uevent_create(&base), 0);
  ASSERT_NE(base._handle, nullptr);
  ASSERT_EQ(uevent_add(&base, &src, 0), 0);

  // No data initially
  uevent_source_t ready[1] = {UEVENT_SOURCE_INITIALIZER};
  EXPECT_EQ(uevent_loop(&base, ready, 1, 0), 0);

  // Publish
  orb_test_s msg{};
  msg.val = 42;
  ASSERT_EQ(orb_pub_publish(pub, &msg), ORB_OK);

  // Loop should return the source
  ASSERT_EQ(uevent_loop(&base, ready, 1, 100), 1);
  EXPECT_EQ(ready[0]._handle, src._handle);

  // Copy data
  ASSERT_EQ(orb_sub_copy(sub, &msg), ORB_OK);
  EXPECT_EQ(msg.val, 42);

  // No more data
  EXPECT_EQ(uevent_loop(&base, ready, 1, 0), 0);

  ASSERT_EQ(uevent_remove(&base, &src), 0);
  uevent_destroy(&base);
  orb_sub_destroy(&sub);
  orb_pub_destroy(&pub);
}

// ---- Publish from another thread wakes blocking loop ----

TEST(UorbEventSourceTest, PublishFromThreadWakesBlockingLoop) {
  orb_pub_t pub = ORB_PUB_INITIALIZER;
  EXPECT_EQ(orb_pub_create(&pub, ORB_ID(orb_test)), ORB_OK);
  ASSERT_NE(pub._handle, nullptr);

  orb_sub_t sub = ORB_SUB_INITIALIZER;
  EXPECT_EQ(orb_sub_create(&sub, ORB_ID(orb_test)), ORB_OK);
  ASSERT_NE(sub._handle, nullptr);

  // Drain initial data
  orb_test_s drain{};
  for (int i = 0; i < 8 && orb_sub_check_update(sub); ++i) {
    orb_sub_copy(sub, &drain);
  }

  uevent_source_t src = orb_sub_event_source(sub);
  ASSERT_NE(src._handle, nullptr);

  uevent_t base = UEVENT_INITIALIZER;
  ASSERT_EQ(uevent_create(&base), 0);
  ASSERT_NE(base._handle, nullptr);
  ASSERT_EQ(uevent_add(&base, &src, 0), 0);

  std::thread publisher([&]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    orb_test_s msg{};
    msg.val = 99;
    orb_pub_publish(pub, &msg);
  });

  uevent_source_t ready[1] = {UEVENT_SOURCE_INITIALIZER};
  auto t0 = std::chrono::steady_clock::now();
  ASSERT_EQ(uevent_loop(&base, ready, 1, -1), 1);
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                     std::chrono::steady_clock::now() - t0)
                     .count();
  EXPECT_GE(elapsed, 25);
  EXPECT_EQ(ready[0]._handle, src._handle);

  publisher.join();

  ASSERT_EQ(uevent_remove(&base, &src), 0);
  uevent_destroy(&base);
  orb_sub_destroy(&sub);
  orb_pub_destroy(&pub);
}

// ---- Multiple topics with separate sources ----

TEST(UorbEventSourceTest, MultipleTopicsMultipleSources) {
  orb_pub_t pub1 = ORB_PUB_INITIALIZER;
  orb_pub_t pub2 = ORB_PUB_INITIALIZER;
  EXPECT_EQ(orb_pub_create(&pub1, ORB_ID(orb_test)), ORB_OK);
  EXPECT_EQ(orb_pub_create(&pub2, ORB_ID(orb_test_medium)), ORB_OK);
  ASSERT_NE(pub1._handle, nullptr);
  ASSERT_NE(pub2._handle, nullptr);

  orb_sub_t sub1 = ORB_SUB_INITIALIZER;
  orb_sub_t sub2 = ORB_SUB_INITIALIZER;
  EXPECT_EQ(orb_sub_create(&sub1, ORB_ID(orb_test)), ORB_OK);
  EXPECT_EQ(orb_sub_create(&sub2, ORB_ID(orb_test_medium)), ORB_OK);
  ASSERT_NE(sub1._handle, nullptr);
  ASSERT_NE(sub2._handle, nullptr);

  // Drain initial data
  orb_test_s drain1{};
  orb_test_medium_s drain2{};
  for (int i = 0; i < 8 && orb_sub_check_update(sub1); ++i) orb_sub_copy(sub1, &drain1);
  for (int i = 0; i < 8 && orb_sub_check_update(sub2); ++i) orb_sub_copy(sub2, &drain2);

  uevent_source_t src1 = orb_sub_event_source(sub1);
  uevent_source_t src2 = orb_sub_event_source(sub2);
  ASSERT_NE(src1._handle, nullptr);
  ASSERT_NE(src2._handle, nullptr);

  uevent_t base = UEVENT_INITIALIZER;
  ASSERT_EQ(uevent_create(&base), 0);
  ASSERT_NE(base._handle, nullptr);
  ASSERT_EQ(uevent_add(&base, &src1, 0), 0);
  ASSERT_EQ(uevent_add(&base, &src2, 0), 0);

  // Publish to topic 1 only
  orb_test_s msg1{};
  msg1.val = 111;
  orb_pub_publish(pub1, &msg1);

  uevent_source_t ready[2] = {UEVENT_SOURCE_INITIALIZER};
  int n = uevent_loop(&base, ready, 2, 100);
  ASSERT_EQ(n, 1);
  EXPECT_EQ(ready[0]._handle, src1._handle);
  // Copy to clear readiness
  orb_test_s recv1{};
  orb_sub_copy(sub1, &recv1);

  // Publish to topic 2 only
  orb_test_medium_s msg2{};
  msg2.val = 222;
  orb_pub_publish(pub2, &msg2);

  n = uevent_loop(&base, ready, 2, 100);
  ASSERT_EQ(n, 1);
  EXPECT_EQ(ready[0]._handle, src2._handle);
  // Copy to clear readiness
  orb_test_medium_s recv2{};
  orb_sub_copy(sub2, &recv2);

  // Publish to both
  msg1.val = 333;
  orb_pub_publish(pub1, &msg1);
  msg2.val = 444;
  orb_pub_publish(pub2, &msg2);

  n = uevent_loop(&base, ready, 2, 100);
  ASSERT_EQ(n, 2);

  ASSERT_EQ(uevent_remove(&base, &src1), 0);
  ASSERT_EQ(uevent_remove(&base, &src2), 0);
  uevent_destroy(&base);
  orb_sub_destroy(&sub1);
  orb_sub_destroy(&sub2);
  orb_pub_destroy(&pub1);
  orb_pub_destroy(&pub2);
}

// ---- Full lifecycle: add, publish, copy, remove across cycles ----

TEST(UorbEventSourceTest, FullLifecycleMultipleCycles) {
  orb_pub_t pub = ORB_PUB_INITIALIZER;
  EXPECT_EQ(orb_pub_create(&pub, ORB_ID(orb_test)), ORB_OK);
  ASSERT_NE(pub._handle, nullptr);

  orb_sub_t sub = ORB_SUB_INITIALIZER;
  EXPECT_EQ(orb_sub_create(&sub, ORB_ID(orb_test)), ORB_OK);
  ASSERT_NE(sub._handle, nullptr);

  // Drain initial data
  orb_test_s drain{};
  for (int i = 0; i < 8 && orb_sub_check_update(sub); ++i) orb_sub_copy(sub, &drain);

  uevent_t base = UEVENT_INITIALIZER;
  ASSERT_EQ(uevent_create(&base), 0);
  ASSERT_NE(base._handle, nullptr);

  for (int cycle = 0; cycle < 10; ++cycle) {
    uevent_source_t src = orb_sub_event_source(sub);
    ASSERT_NE(src._handle, nullptr);
    ASSERT_EQ(uevent_add(&base, &src, 0), 0);

    orb_test_s msg{};
    msg.val = cycle * 10;
    ASSERT_EQ(orb_pub_publish(pub, &msg), ORB_OK);

    uevent_source_t ready[1] = {UEVENT_SOURCE_INITIALIZER};
    ASSERT_EQ(uevent_loop(&base, ready, 1, 100), 1);
    EXPECT_EQ(ready[0]._handle, src._handle);

    ASSERT_EQ(orb_sub_copy(sub, &msg), ORB_OK);
    EXPECT_EQ(msg.val, cycle * 10);

    ASSERT_EQ(uevent_remove(&base, &src), 0);
  }

  uevent_destroy(&base);
  orb_sub_destroy(&sub);
  orb_pub_destroy(&pub);
}

// ---- Multi-instance publisher + subscriber ----

TEST(UorbEventSourceTest, MultiInstanceEventSource) {
  unsigned instance0 = 0;
  unsigned instance1 = 0;
  orb_pub_t pub0 = ORB_PUB_INITIALIZER;
  orb_pub_t pub1 = ORB_PUB_INITIALIZER;
  ASSERT_EQ(orb_pub_create_multi(&pub0, ORB_ID(orb_test), &instance0), ORB_OK);
  ASSERT_NE(pub0._handle, nullptr);
  ASSERT_EQ(instance0, 0u);

  ASSERT_EQ(orb_pub_create_multi(&pub1, ORB_ID(orb_test), &instance1), ORB_OK);
  ASSERT_NE(pub1._handle, nullptr);
  ASSERT_EQ(instance1, 1u);

  orb_sub_t sub_multi = ORB_SUB_INITIALIZER;
  ASSERT_EQ(orb_sub_create_multi(&sub_multi, ORB_ID(orb_test), 1), ORB_OK);
  ASSERT_NE(sub_multi._handle, nullptr);

  orb_test_s drain{};
  for (int i = 0; i < 8 && orb_sub_check_update(sub_multi); ++i) {
    orb_sub_copy(sub_multi, &drain);
  }

  uevent_source_t src = orb_sub_event_source(sub_multi);
  ASSERT_NE(src._handle, nullptr);

  uevent_t base = UEVENT_INITIALIZER;
  ASSERT_EQ(uevent_create(&base), 0);
  ASSERT_NE(base._handle, nullptr);
  ASSERT_EQ(uevent_add(&base, &src, 0), 0);

  orb_test_s msg{};
  msg.val = 777;
  ASSERT_EQ(orb_pub_publish(pub1, &msg), ORB_OK);

  uevent_source_t ready[1] = {UEVENT_SOURCE_INITIALIZER};
  ASSERT_EQ(uevent_loop(&base, ready, 1, 100), 1);
  EXPECT_EQ(ready[0]._handle, src._handle);

  orb_test_s recv{};
  ASSERT_EQ(orb_sub_copy(sub_multi, &recv), ORB_OK);
  EXPECT_EQ(recv.val, 777);

  msg.val = 888;
  ASSERT_EQ(orb_pub_publish(pub0, &msg), ORB_OK);
  EXPECT_EQ(uevent_loop(&base, ready, 1, 0), 0);

  ASSERT_EQ(uevent_remove(&base, &src), 0);
  uevent_destroy(&base);
  orb_sub_destroy(&sub_multi);
  orb_pub_destroy(&pub1);
  orb_pub_destroy(&pub0);
}

// ---- Burst publish coalescing: 1000 messages, one event ----

TEST(UorbEventSourceTest, BurstPublishCoalescing) {
  orb_pub_t pub = ORB_PUB_INITIALIZER;
  EXPECT_EQ(orb_pub_create(&pub, ORB_ID(orb_test)), ORB_OK);
  ASSERT_NE(pub._handle, nullptr);

  orb_sub_t sub = ORB_SUB_INITIALIZER;
  EXPECT_EQ(orb_sub_create(&sub, ORB_ID(orb_test)), ORB_OK);
  ASSERT_NE(sub._handle, nullptr);

  orb_test_s drain{};
  for (int i = 0; i < 8 && orb_sub_check_update(sub); ++i) {
    orb_sub_copy(sub, &drain);
  }

  uevent_source_t src = orb_sub_event_source(sub);
  ASSERT_NE(src._handle, nullptr);

  uevent_t base = UEVENT_INITIALIZER;
  ASSERT_EQ(uevent_create(&base), 0);
  ASSERT_NE(base._handle, nullptr);
  ASSERT_EQ(uevent_add(&base, &src, 0), 0);

  const int burst_count = 1000;
  const int last_val = burst_count - 1;
  orb_test_s msg{};
  for (int i = 0; i < burst_count; ++i) {
    msg.val = i;
    ASSERT_EQ(orb_pub_publish(pub, &msg), ORB_OK);
  }

  uevent_source_t ready[1] = {UEVENT_SOURCE_INITIALIZER};
  int n = uevent_loop(&base, ready, 1, 0);
  EXPECT_GE(n, 1);

  orb_test_s recv{};
  ASSERT_EQ(orb_sub_copy(sub, &recv), ORB_OK);
  EXPECT_EQ(recv.val, last_val);

  ASSERT_EQ(uevent_remove(&base, &src), 0);
  uevent_destroy(&base);
  orb_sub_destroy(&sub);
  orb_pub_destroy(&pub);
}

// ---- Fan-out: one publication, 50 subscribers ----

TEST(UorbEventSourceTest, FanOutManySubscribers) {
  constexpr int kN = 50;

  orb_pub_t pub = ORB_PUB_INITIALIZER;
  EXPECT_EQ(orb_pub_create(&pub, ORB_ID(orb_test)), ORB_OK);
  ASSERT_NE(pub._handle, nullptr);

  orb_sub_t subs[kN];
  for (int i = 0; i < kN; ++i) {
    subs[i] = ORB_SUB_INITIALIZER;
    ASSERT_EQ(orb_sub_create(&subs[i], ORB_ID(orb_test)), ORB_OK);
    ASSERT_NE(subs[i]._handle, nullptr);
  }

  orb_test_s drain{};
  for (int i = 0; i < kN; ++i) {
    for (int j = 0; j < 8 && orb_sub_check_update(subs[i]); ++j) {
      orb_sub_copy(subs[i], &drain);
    }
  }

  uevent_source_t srcs[kN];
  for (int i = 0; i < kN; ++i) {
    srcs[i] = orb_sub_event_source(subs[i]);
    ASSERT_NE(srcs[i]._handle, nullptr);
  }

  uevent_t base = UEVENT_INITIALIZER;
  ASSERT_EQ(uevent_create(&base), 0);
  ASSERT_NE(base._handle, nullptr);
  for (int i = 0; i < kN; ++i) {
    ASSERT_EQ(uevent_add(&base, &srcs[i], 0), 0);
  }

  orb_test_s msg{};
  msg.val = 12345;
  ASSERT_EQ(orb_pub_publish(pub, &msg), ORB_OK);

  uevent_source_t ready[kN];
  for (int i = 0; i < kN; ++i) ready[i] = UEVENT_SOURCE_INITIALIZER;
  int n = uevent_loop(&base, ready, kN, 100);
  ASSERT_EQ(n, kN);

  for (int i = 0; i < kN; ++i) {
    bool matched = false;
    for (int j = 0; j < kN; ++j) {
      if (ready[i]._handle == srcs[j]._handle) {
        orb_test_s recv{};
        ASSERT_EQ(orb_sub_copy(subs[j], &recv), ORB_OK);
        EXPECT_EQ(recv.val, 12345);
        matched = true;
        break;
      }
    }
    EXPECT_TRUE(matched);
  }

  for (int i = 0; i < kN; ++i) {
    ASSERT_EQ(uevent_remove(&base, &srcs[i]), 0);
  }
  uevent_destroy(&base);
  for (int i = 0; i < kN; ++i) {
    orb_sub_destroy(&subs[i]);
  }
  orb_pub_destroy(&pub);
}
