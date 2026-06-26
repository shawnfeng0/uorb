/****************************************************************************
 *
 * uorb_uevent bridge API tests — tests the bridge between uORB and uevent.
 *
 ****************************************************************************/

#include <gtest/gtest.h>
#include <uorb/topics/orb_test.h>
#include <uorb/topics/orb_test_medium.h>
#include <uorb/uorb.h>
#include <uorb_uevent/uorb_uevent.h>

#include <atomic>
#include <chrono>
#include <thread>

// ---- uorb_subscriber_create_source / destroy_source ----

TEST(UorbUeventTest, CreateDestroySource) {
  orb_subscriber_t sub = ORB_SUBSCRIBER_INITIALIZER;
  EXPECT_EQ(orb_subscriber_create(&sub, ORB_ID(orb_test)), ORB_OK);
  ASSERT_NE(sub._handle, nullptr);

  uevent_source_t src = UEVENT_SOURCE_INITIALIZER;
  ASSERT_EQ(uorb_subscriber_create_source(&src, &sub), 0);
  ASSERT_NE(src._handle, nullptr);

  uorb_subscriber_destroy_source(&src);
  EXPECT_EQ(orb_subscriber_destroy(&sub), ORB_OK);
}

TEST(UorbUeventTest, CreateSourceFromNullSubscriptionFails) {
  errno = 0;
  uevent_source_t src = UEVENT_SOURCE_INITIALIZER;
  EXPECT_EQ(uorb_subscriber_create_source(&src, nullptr), -1);
  EXPECT_EQ(src._handle, nullptr);
  EXPECT_EQ(errno, EINVAL);
}

TEST(UorbUeventTest, DestroyNullSource) {
  uorb_subscriber_destroy_source(nullptr);  // should not crash
}

// ---- Source lifecycle with uevent_add/remove ----

TEST(UorbUeventTest, SourceAddRemoveLoop) {
  orb_subscriber_t sub = ORB_SUBSCRIBER_INITIALIZER;
  EXPECT_EQ(orb_subscriber_create(&sub, ORB_ID(orb_test)), ORB_OK);
  ASSERT_NE(sub._handle, nullptr);

  uevent_source_t src = UEVENT_SOURCE_INITIALIZER;
  ASSERT_EQ(uorb_subscriber_create_source(&src, &sub), 0);
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

  uorb_subscriber_destroy_source(&src);
  uevent_destroy(&base);
  EXPECT_EQ(orb_subscriber_destroy(&sub), ORB_OK);
}

// ---- Publish triggers source ready ----

TEST(UorbUeventTest, PublishTriggersReady) {
  orb_publisher_t pub = ORB_PUBLISHER_INITIALIZER;
  EXPECT_EQ(orb_publisher_create(&pub, ORB_ID(orb_test)), ORB_OK);
  ASSERT_NE(pub._handle, nullptr);

  orb_subscriber_t sub = ORB_SUBSCRIBER_INITIALIZER;
  EXPECT_EQ(orb_subscriber_create(&sub, ORB_ID(orb_test)), ORB_OK);
  ASSERT_NE(sub._handle, nullptr);

  // Drain initial data
  orb_test_s drain{};
  for (int i = 0; i < 8 && orb_subscriber_check_update(&sub); ++i) {
    orb_subscriber_copy(&sub, &drain);
  }

  uevent_source_t src = UEVENT_SOURCE_INITIALIZER;
  ASSERT_EQ(uorb_subscriber_create_source(&src, &sub), 0);
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
  ASSERT_EQ(orb_publisher_publish(&pub, &msg), ORB_OK);

  // Loop should return the source
  ASSERT_EQ(uevent_loop(&base, ready, 1, 100), 1);
  EXPECT_EQ(ready[0]._handle, src._handle);

  // Copy data
  ASSERT_EQ(orb_subscriber_copy(&sub, &msg), ORB_OK);
  EXPECT_EQ(msg.val, 42);

  // No more data
  EXPECT_EQ(uevent_loop(&base, ready, 1, 0), 0);

  ASSERT_EQ(uevent_remove(&base, &src), 0);
  uorb_subscriber_destroy_source(&src);
  uevent_destroy(&base);
  EXPECT_EQ(orb_subscriber_destroy(&sub), ORB_OK);
  EXPECT_EQ(orb_publisher_destroy(&pub), ORB_OK);
}

// ---- Publish from another thread wakes blocking loop ----

TEST(UorbUeventTest, PublishFromThreadWakesBlockingLoop) {
  orb_publisher_t pub = ORB_PUBLISHER_INITIALIZER;
  EXPECT_EQ(orb_publisher_create(&pub, ORB_ID(orb_test)), ORB_OK);
  ASSERT_NE(pub._handle, nullptr);

  orb_subscriber_t sub = ORB_SUBSCRIBER_INITIALIZER;
  EXPECT_EQ(orb_subscriber_create(&sub, ORB_ID(orb_test)), ORB_OK);
  ASSERT_NE(sub._handle, nullptr);

  // Drain initial data
  orb_test_s drain{};
  for (int i = 0; i < 8 && orb_subscriber_check_update(&sub); ++i) {
    orb_subscriber_copy(&sub, &drain);
  }

  uevent_source_t src = UEVENT_SOURCE_INITIALIZER;
  ASSERT_EQ(uorb_subscriber_create_source(&src, &sub), 0);
  ASSERT_NE(src._handle, nullptr);

  uevent_t base = UEVENT_INITIALIZER;
  ASSERT_EQ(uevent_create(&base), 0);
  ASSERT_NE(base._handle, nullptr);
  ASSERT_EQ(uevent_add(&base, &src, 0), 0);

  std::thread publisher([&]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    orb_test_s msg{};
    msg.val = 99;
    orb_publisher_publish(&pub, &msg);
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
  uorb_subscriber_destroy_source(&src);
  uevent_destroy(&base);
  EXPECT_EQ(orb_subscriber_destroy(&sub), ORB_OK);
  EXPECT_EQ(orb_publisher_destroy(&pub), ORB_OK);
}

// ---- orb_subscriber_set_callback / clear_callback ----

TEST(UorbUeventTest, SetCallbackReceivesPublish) {
  orb_publisher_t pub = ORB_PUBLISHER_INITIALIZER;
  EXPECT_EQ(orb_publisher_create(&pub, ORB_ID(orb_test)), ORB_OK);
  ASSERT_NE(pub._handle, nullptr);

  orb_subscriber_t sub = ORB_SUBSCRIBER_INITIALIZER;
  EXPECT_EQ(orb_subscriber_create(&sub, ORB_ID(orb_test)), ORB_OK);
  ASSERT_NE(sub._handle, nullptr);

  // Drain initial data
  orb_test_s drain{};
  for (int i = 0; i < 8 && orb_subscriber_check_update(&sub); ++i) {
    orb_subscriber_copy(&sub, &drain);
  }

  std::atomic<int> callback_count{0};
  auto cb = [](const void *, orb_callback_ctx ctx) {
    static_cast<std::atomic<int> *>(ctx.ptr)->fetch_add(1);
  };

  ASSERT_EQ(orb_subscriber_set_callback(&sub, cb, orb_callback_ctx{&callback_count}), ORB_OK);

  orb_test_s msg{};
  callback_count = 0;
  orb_publisher_publish(&pub, &msg);
  EXPECT_EQ(callback_count.load(), 1);  // exactly one callback

  ASSERT_EQ(orb_subscriber_clear_callback(&sub), ORB_OK);
  int before = callback_count.load();
  orb_publisher_publish(&pub, &msg);
  EXPECT_EQ(callback_count.load(), before);  // no more callbacks

  EXPECT_EQ(orb_subscriber_destroy(&sub), ORB_OK);
  EXPECT_EQ(orb_publisher_destroy(&pub), ORB_OK);
}

TEST(UorbUeventTest, SetCallbackTwiceFails) {
  orb_subscriber_t sub = ORB_SUBSCRIBER_INITIALIZER;
  EXPECT_EQ(orb_subscriber_create(&sub, ORB_ID(orb_test)), ORB_OK);
  ASSERT_NE(sub._handle, nullptr);

  auto cb = [](const void *, orb_callback_ctx) {};
  ASSERT_EQ(orb_subscriber_set_callback(&sub, cb, {}), ORB_OK);

  EXPECT_EQ(orb_subscriber_set_callback(&sub, cb, {}), ORB_ERR_BUSY);

  ASSERT_EQ(orb_subscriber_clear_callback(&sub), ORB_OK);
  // Now should work again
  ASSERT_EQ(orb_subscriber_set_callback(&sub, cb, {}), ORB_OK);
  ASSERT_EQ(orb_subscriber_clear_callback(&sub), ORB_OK);

  EXPECT_EQ(orb_subscriber_destroy(&sub), ORB_OK);
}

TEST(UorbUeventTest, ClearCallbackWithoutSetIsNoop) {
  orb_subscriber_t sub = ORB_SUBSCRIBER_INITIALIZER;
  EXPECT_EQ(orb_subscriber_create(&sub, ORB_ID(orb_test)), ORB_OK);
  ASSERT_NE(sub._handle, nullptr);
  EXPECT_EQ(orb_subscriber_clear_callback(&sub), ORB_OK);
  EXPECT_EQ(orb_subscriber_destroy(&sub), ORB_OK);
}

// ---- EventLoop C++ wrapper ----

// ---- Multiple topics with separate sources ----

TEST(UorbUeventTest, MultipleTopicsMultipleSources) {
  orb_publisher_t pub1 = ORB_PUBLISHER_INITIALIZER;
  orb_publisher_t pub2 = ORB_PUBLISHER_INITIALIZER;
  EXPECT_EQ(orb_publisher_create(&pub1, ORB_ID(orb_test)), ORB_OK);
  EXPECT_EQ(orb_publisher_create(&pub2, ORB_ID(orb_test_medium)), ORB_OK);
  ASSERT_NE(pub1._handle, nullptr);
  ASSERT_NE(pub2._handle, nullptr);

  orb_subscriber_t sub1 = ORB_SUBSCRIBER_INITIALIZER;
  orb_subscriber_t sub2 = ORB_SUBSCRIBER_INITIALIZER;
  EXPECT_EQ(orb_subscriber_create(&sub1, ORB_ID(orb_test)), ORB_OK);
  EXPECT_EQ(orb_subscriber_create(&sub2, ORB_ID(orb_test_medium)), ORB_OK);
  ASSERT_NE(sub1._handle, nullptr);
  ASSERT_NE(sub2._handle, nullptr);

  // Drain initial data
  orb_test_s drain1{};
  orb_test_medium_s drain2{};
  for (int i = 0; i < 8 && orb_subscriber_check_update(&sub1); ++i) orb_subscriber_copy(&sub1, &drain1);
  for (int i = 0; i < 8 && orb_subscriber_check_update(&sub2); ++i) orb_subscriber_copy(&sub2, &drain2);

  uevent_source_t src1 = UEVENT_SOURCE_INITIALIZER;
  uevent_source_t src2 = UEVENT_SOURCE_INITIALIZER;
  ASSERT_EQ(uorb_subscriber_create_source(&src1, &sub1), 0);
  ASSERT_EQ(uorb_subscriber_create_source(&src2, &sub2), 0);
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
  orb_publisher_publish(&pub1, &msg1);

  uevent_source_t ready[2] = {UEVENT_SOURCE_INITIALIZER};
  int n = uevent_loop(&base, ready, 2, 100);
  ASSERT_EQ(n, 1);
  EXPECT_EQ(ready[0]._handle, src1._handle);
  // Copy to clear readiness
  orb_test_s recv1{};
  orb_subscriber_copy(&sub1, &recv1);

  // Publish to topic 2 only
  orb_test_medium_s msg2{};
  msg2.val = 222;
  orb_publisher_publish(&pub2, &msg2);

  n = uevent_loop(&base, ready, 2, 100);
  ASSERT_EQ(n, 1);
  EXPECT_EQ(ready[0]._handle, src2._handle);
  // Copy to clear readiness
  orb_test_medium_s recv2{};
  orb_subscriber_copy(&sub2, &recv2);

  // Publish to both
  msg1.val = 333;
  orb_publisher_publish(&pub1, &msg1);
  msg2.val = 444;
  orb_publisher_publish(&pub2, &msg2);

  n = uevent_loop(&base, ready, 2, 100);
  ASSERT_EQ(n, 2);

  ASSERT_EQ(uevent_remove(&base, &src1), 0);
  ASSERT_EQ(uevent_remove(&base, &src2), 0);
  uorb_subscriber_destroy_source(&src1);
  uorb_subscriber_destroy_source(&src2);
  uevent_destroy(&base);
  EXPECT_EQ(orb_subscriber_destroy(&sub1), ORB_OK);
  EXPECT_EQ(orb_subscriber_destroy(&sub2), ORB_OK);
  EXPECT_EQ(orb_publisher_destroy(&pub1), ORB_OK);
  EXPECT_EQ(orb_publisher_destroy(&pub2), ORB_OK);
}

// ---- Callback triggered from publisher thread ----

TEST(UorbUeventTest, CallbackTriggeredFromPublisherThread) {
  orb_publisher_t pub = ORB_PUBLISHER_INITIALIZER;
  EXPECT_EQ(orb_publisher_create(&pub, ORB_ID(orb_test)), ORB_OK);
  ASSERT_NE(pub._handle, nullptr);

  orb_subscriber_t sub = ORB_SUBSCRIBER_INITIALIZER;
  EXPECT_EQ(orb_subscriber_create(&sub, ORB_ID(orb_test)), ORB_OK);
  ASSERT_NE(sub._handle, nullptr);

  // Drain initial data
  orb_test_s drain{};
  for (int i = 0; i < 8 && orb_subscriber_check_update(&sub); ++i) orb_subscriber_copy(&sub, &drain);

  std::atomic<int> callback_count{0};
  auto cb = [](const void *, orb_callback_ctx ctx) {
    static_cast<std::atomic<int> *>(ctx.ptr)->fetch_add(1, std::memory_order_relaxed);
  };

  ASSERT_EQ(orb_subscriber_set_callback(&sub, cb, orb_callback_ctx{&callback_count}), ORB_OK);

  // Publish from another thread
  std::thread publisher([&]() {
    orb_test_s msg{};
    for (int i = 0; i < 10; ++i) {
      msg.val = i;
      orb_publisher_publish(&pub, &msg);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  });

  publisher.join();

  // Should have received callbacks
  EXPECT_EQ(callback_count.load(), 10);

  ASSERT_EQ(orb_subscriber_clear_callback(&sub), ORB_OK);
  EXPECT_EQ(orb_subscriber_destroy(&sub), ORB_OK);
  EXPECT_EQ(orb_publisher_destroy(&pub), ORB_OK);
}

// ---- Full lifecycle: create source, add, publish, copy, remove, destroy ----

TEST(UorbUeventTest, FullLifecycleMultipleCycles) {
  orb_publisher_t pub = ORB_PUBLISHER_INITIALIZER;
  EXPECT_EQ(orb_publisher_create(&pub, ORB_ID(orb_test)), ORB_OK);
  ASSERT_NE(pub._handle, nullptr);

  orb_subscriber_t sub = ORB_SUBSCRIBER_INITIALIZER;
  EXPECT_EQ(orb_subscriber_create(&sub, ORB_ID(orb_test)), ORB_OK);
  ASSERT_NE(sub._handle, nullptr);

  // Drain initial data
  orb_test_s drain{};
  for (int i = 0; i < 8 && orb_subscriber_check_update(&sub); ++i) orb_subscriber_copy(&sub, &drain);

  uevent_t base = UEVENT_INITIALIZER;
  ASSERT_EQ(uevent_create(&base), 0);
  ASSERT_NE(base._handle, nullptr);

  // Run 10 cycles of: create source -> add -> publish -> loop -> copy -> remove -> destroy
  for (int cycle = 0; cycle < 10; ++cycle) {
    uevent_source_t src = UEVENT_SOURCE_INITIALIZER;
    ASSERT_EQ(uorb_subscriber_create_source(&src, &sub), 0);
    ASSERT_NE(src._handle, nullptr);
    ASSERT_EQ(uevent_add(&base, &src, 0), 0);

    orb_test_s msg{};
    msg.val = cycle * 10;
    ASSERT_EQ(orb_publisher_publish(&pub, &msg), ORB_OK);

    uevent_source_t ready[1] = {UEVENT_SOURCE_INITIALIZER};
    ASSERT_EQ(uevent_loop(&base, ready, 1, 100), 1);
    EXPECT_EQ(ready[0]._handle, src._handle);

    ASSERT_EQ(orb_subscriber_copy(&sub, &msg), ORB_OK);
    EXPECT_EQ(msg.val, cycle * 10);

    ASSERT_EQ(uevent_remove(&base, &src), 0);
    uorb_subscriber_destroy_source(&src);
  }

  uevent_destroy(&base);
  EXPECT_EQ(orb_subscriber_destroy(&sub), ORB_OK);
  EXPECT_EQ(orb_publisher_destroy(&pub), ORB_OK);
}

// ---- Multi-instance publisher + subscriber via bridge ----

TEST(UorbUeventTest, MultiInstanceBridge) {
  // Create two multi-instance publishers so that instance 1 exists.
  unsigned instance0 = 0;
  unsigned instance1 = 0;
  orb_publisher_t pub0 = ORB_PUBLISHER_INITIALIZER;
  orb_publisher_t pub1 = ORB_PUBLISHER_INITIALIZER;
  ASSERT_EQ(orb_publisher_create_multi(&pub0, ORB_ID(orb_test), &instance0),
            ORB_OK);
  ASSERT_NE(pub0._handle, nullptr);
  ASSERT_EQ(instance0, 0u);

  ASSERT_EQ(orb_publisher_create_multi(&pub1, ORB_ID(orb_test), &instance1),
            ORB_OK);
  ASSERT_NE(pub1._handle, nullptr);
  ASSERT_EQ(instance1, 1u);

  // Subscribe to instance 1.
  orb_subscriber_t sub_multi = ORB_SUBSCRIBER_INITIALIZER;
  ASSERT_EQ(orb_subscriber_create_multi(&sub_multi, ORB_ID(orb_test), 1),
            ORB_OK);
  ASSERT_NE(sub_multi._handle, nullptr);

  // Drain any initial data.
  orb_test_s drain{};
  for (int i = 0; i < 8 && orb_subscriber_check_update(&sub_multi); ++i) {
    orb_subscriber_copy(&sub_multi, &drain);
  }

  uevent_source_t src = UEVENT_SOURCE_INITIALIZER;
  ASSERT_EQ(uorb_subscriber_create_source(&src, &sub_multi), 0);
  ASSERT_NE(src._handle, nullptr);

  uevent_t base = UEVENT_INITIALIZER;
  ASSERT_EQ(uevent_create(&base), 0);
  ASSERT_NE(base._handle, nullptr);
  ASSERT_EQ(uevent_add(&base, &src, 0), 0);

  // Publish to instance 1 — the subscriber should become ready.
  orb_test_s msg{};
  msg.val = 777;
  ASSERT_EQ(orb_publisher_publish(&pub1, &msg), ORB_OK);

  uevent_source_t ready[1] = {UEVENT_SOURCE_INITIALIZER};
  ASSERT_EQ(uevent_loop(&base, ready, 1, 100), 1);
  EXPECT_EQ(ready[0]._handle, src._handle);

  // Copy the data.
  orb_test_s recv{};
  ASSERT_EQ(orb_subscriber_copy(&sub_multi, &recv), ORB_OK);
  EXPECT_EQ(recv.val, 777);

  // Publishing to instance 0 should NOT wake the instance-1 source.
  msg.val = 888;
  ASSERT_EQ(orb_publisher_publish(&pub0, &msg), ORB_OK);
  EXPECT_EQ(uevent_loop(&base, ready, 1, 0), 0);

  ASSERT_EQ(uevent_remove(&base, &src), 0);
  uorb_subscriber_destroy_source(&src);
  uevent_destroy(&base);
  EXPECT_EQ(orb_subscriber_destroy(&sub_multi), ORB_OK);
  EXPECT_EQ(orb_publisher_destroy(&pub1), ORB_OK);
  EXPECT_EQ(orb_publisher_destroy(&pub0), ORB_OK);
}

// ---- Burst publish coalescing: 1000 messages, one event ----

TEST(UorbUeventTest, BurstPublishCoalescing) {
  orb_publisher_t pub = ORB_PUBLISHER_INITIALIZER;
  EXPECT_EQ(orb_publisher_create(&pub, ORB_ID(orb_test)), ORB_OK);
  ASSERT_NE(pub._handle, nullptr);

  orb_subscriber_t sub = ORB_SUBSCRIBER_INITIALIZER;
  EXPECT_EQ(orb_subscriber_create(&sub, ORB_ID(orb_test)), ORB_OK);
  ASSERT_NE(sub._handle, nullptr);

  // Drain initial data.
  orb_test_s drain{};
  for (int i = 0; i < 8 && orb_subscriber_check_update(&sub); ++i) {
    orb_subscriber_copy(&sub, &drain);
  }

  uevent_source_t src = UEVENT_SOURCE_INITIALIZER;
  ASSERT_EQ(uorb_subscriber_create_source(&src, &sub), 0);
  ASSERT_NE(src._handle, nullptr);

  uevent_t base = UEVENT_INITIALIZER;
  ASSERT_EQ(uevent_create(&base), 0);
  ASSERT_NE(base._handle, nullptr);
  ASSERT_EQ(uevent_add(&base, &src, 0), 0);

  // Publish 1000 messages in a tight loop — no sleep between publishes.
  const int burst_count = 1000;
  const int last_val = burst_count - 1;
  orb_test_s msg{};
  for (int i = 0; i < burst_count; ++i) {
    msg.val = i;
    ASSERT_EQ(orb_publisher_publish(&pub, &msg), ORB_OK);
  }

  // With timeout=0, uevent_loop should return at least 1 event.
  uevent_source_t ready[1] = {UEVENT_SOURCE_INITIALIZER};
  int n = uevent_loop(&base, ready, 1, 0);
  EXPECT_GE(n, 1);

  // Copying should yield the *latest* published data.
  orb_test_s recv{};
  ASSERT_EQ(orb_subscriber_copy(&sub, &recv), ORB_OK);
  EXPECT_EQ(recv.val, last_val);

  ASSERT_EQ(uevent_remove(&base, &src), 0);
  uorb_subscriber_destroy_source(&src);
  uevent_destroy(&base);
  EXPECT_EQ(orb_subscriber_destroy(&sub), ORB_OK);
  EXPECT_EQ(orb_publisher_destroy(&pub), ORB_OK);
}

// ---- Fan-out: one publication, 50 subscribers ----

TEST(UorbUeventTest, FanOutManySubscribers) {
  constexpr int kN = 50;

  orb_publisher_t pub = ORB_PUBLISHER_INITIALIZER;
  EXPECT_EQ(orb_publisher_create(&pub, ORB_ID(orb_test)), ORB_OK);
  ASSERT_NE(pub._handle, nullptr);

  // Create kN subscriptions.
  orb_subscriber_t subs[kN];
  for (int i = 0; i < kN; ++i) {
    subs[i] = ORB_SUBSCRIBER_INITIALIZER;
    ASSERT_EQ(orb_subscriber_create(&subs[i], ORB_ID(orb_test)), ORB_OK);
    ASSERT_NE(subs[i]._handle, nullptr);
  }

  // Drain initial data for all subscribers.
  orb_test_s drain{};
  for (int i = 0; i < kN; ++i) {
    for (int j = 0; j < 8 && orb_subscriber_check_update(&subs[i]); ++j) {
      orb_subscriber_copy(&subs[i], &drain);
    }
  }

  // Create kN sources and add them all to one base.
  uevent_source_t srcs[kN];
  for (int i = 0; i < kN; ++i) {
    srcs[i] = UEVENT_SOURCE_INITIALIZER;
    ASSERT_EQ(uorb_subscriber_create_source(&srcs[i], &subs[i]), 0);
    ASSERT_NE(srcs[i]._handle, nullptr);
  }

  uevent_t base = UEVENT_INITIALIZER;
  ASSERT_EQ(uevent_create(&base), 0);
  ASSERT_NE(base._handle, nullptr);
  for (int i = 0; i < kN; ++i) {
    ASSERT_EQ(uevent_add(&base, &srcs[i], 0), 0);
  }

  // Publish one message — every subscriber should be ready.
  orb_test_s msg{};
  msg.val = 12345;
  ASSERT_EQ(orb_publisher_publish(&pub, &msg), ORB_OK);

  uevent_source_t ready[kN];
  for (int i = 0; i < kN; ++i) ready[i] = UEVENT_SOURCE_INITIALIZER;
  int n = uevent_loop(&base, ready, kN, 100);
  ASSERT_EQ(n, kN);

  // Verify each ready source can copy the data.
  for (int i = 0; i < kN; ++i) {
    // Match the ready source handle back to its subscriber.
    bool matched = false;
    for (int j = 0; j < kN; ++j) {
      if (ready[i]._handle == srcs[j]._handle) {
        orb_test_s recv{};
        ASSERT_EQ(orb_subscriber_copy(&subs[j], &recv), ORB_OK);
        EXPECT_EQ(recv.val, 12345);
        matched = true;
        break;
      }
    }
    EXPECT_TRUE(matched);
  }

  // Clean up all sources and subscribers.
  for (int i = 0; i < kN; ++i) {
    ASSERT_EQ(uevent_remove(&base, &srcs[i]), 0);
    uorb_subscriber_destroy_source(&srcs[i]);
  }
  uevent_destroy(&base);
  for (int i = 0; i < kN; ++i) {
    EXPECT_EQ(orb_subscriber_destroy(&subs[i]), ORB_OK);
  }
  EXPECT_EQ(orb_publisher_destroy(&pub), ORB_OK);
}

// ---- set_callback then create_source: uevent_add must fail ----

TEST(UorbUeventTest, SetCallbackThenCreateSource) {
  orb_subscriber_t sub = ORB_SUBSCRIBER_INITIALIZER;
  EXPECT_EQ(orb_subscriber_create(&sub, ORB_ID(orb_test)), ORB_OK);
  ASSERT_NE(sub._handle, nullptr);

  // Set a callback on the subscriber manually.
  auto cb = [](const void *, orb_callback_ctx) {};
  ASSERT_EQ(orb_subscriber_set_callback(&sub, cb, {}), ORB_OK);

  // Creating the source should still succeed — it just copies the handle.
  uevent_source_t src = UEVENT_SOURCE_INITIALIZER;
  ASSERT_EQ(uorb_subscriber_create_source(&src, &sub), 0);
  ASSERT_NE(src._handle, nullptr);

  // uevent_add must fail because bridge_register calls
  // orb_subscriber_set_callback on the (already-bound) subscriber handle,
  // which returns ORB_ERR_BUSY.
  uevent_t base = UEVENT_INITIALIZER;
  ASSERT_EQ(uevent_create(&base), 0);
  ASSERT_NE(base._handle, nullptr);
  EXPECT_EQ(uevent_add(&base, &src, 0), -1);

  // Source was never added, so just destroy it directly.
  uorb_subscriber_destroy_source(&src);
  uevent_destroy(&base);

  // Clear the callback and destroy the subscriber.
  ASSERT_EQ(orb_subscriber_clear_callback(&sub), ORB_OK);
  EXPECT_EQ(orb_subscriber_destroy(&sub), ORB_OK);
}

// ---- Source created from an uninitialized subscriber ----

// Creating a source from an uninitialized subscriber succeeds, but adding it
// to an event base fails because the bridge cannot register a callback on a
// NULL handle.
TEST(UorbUeventTest, CreateSourceWithUninitializedSubscriber) {
  orb_subscriber_t sub = ORB_SUBSCRIBER_INITIALIZER;

  uevent_source_t src = UEVENT_SOURCE_INITIALIZER;
  ASSERT_EQ(uorb_subscriber_create_source(&src, &sub), 0);

  uevent_t base = UEVENT_INITIALIZER;
  ASSERT_EQ(uevent_create(&base), 0);

  // uevent_add must fail: bridge_register calls orb_subscriber_set_callback
  // which returns ORB_ERR_INVALID for a NULL handle.
  EXPECT_EQ(uevent_add(&base, &src, 0), -1);

  // Source was never added, so just destroy it directly.
  uorb_subscriber_destroy_source(&src);
  uevent_destroy(&base);
}

// NOTE: SubscriberDestroyedBeforeSource test intentionally omitted.
// Destroying a subscriber while its source is still in the event base
// creates a dangling pointer in the bridge. During cleanup, uevent_remove
// → bridge_unregister → orb_subscriber_clear_callback dereferences the
// freed ReceiverLocal. This is documented UB, not safely testable without
// mocking the internal allocator.
