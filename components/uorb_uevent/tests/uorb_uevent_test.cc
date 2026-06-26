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

// ---- uorb_subscription_create_source / destroy_source ----

TEST(UorbUeventTest, CreateDestroySource) {
  orb_subscription_t *sub = orb_create_subscription(ORB_ID(orb_test));
  ASSERT_NE(sub, nullptr);

  uevent_source_t *src = uorb_subscription_create_source(sub);
  ASSERT_NE(src, nullptr);

  uorb_subscription_destroy_source(src);
  EXPECT_TRUE(orb_destroy_subscription(&sub));
}

TEST(UorbUeventTest, CreateSourceFromNullSubscriptionFails) {
  errno = 0;
  uevent_source_t *src = uorb_subscription_create_source(nullptr);
  EXPECT_EQ(src, nullptr);
  EXPECT_EQ(errno, EINVAL);
}

TEST(UorbUeventTest, DestroyNullSource) {
  uorb_subscription_destroy_source(nullptr);  // should not crash
}

// ---- Source lifecycle with uevent_add/remove ----

TEST(UorbUeventTest, SourceAddRemoveLoop) {
  orb_subscription_t *sub = orb_create_subscription(ORB_ID(orb_test));
  ASSERT_NE(sub, nullptr);

  uevent_source_t *src = uorb_subscription_create_source(sub);
  ASSERT_NE(src, nullptr);

  uevent_t *base = uevent_create();
  ASSERT_NE(base, nullptr);

  // Add, loop (no data), remove — should work repeatedly
  for (int i = 0; i < 5; ++i) {
    ASSERT_EQ(uevent_add(base, src, 0), 0);
    uevent_source_t *ready[1] = {nullptr};
    uevent_loop(base, ready, 1, 0);
    ASSERT_EQ(uevent_remove(base, src), 0);
  }

  uorb_subscription_destroy_source(src);
  uevent_destroy(base);
  EXPECT_TRUE(orb_destroy_subscription(&sub));
}

// ---- Publish triggers source ready ----

TEST(UorbUeventTest, PublishTriggersReady) {
  orb_publication_t *pub = orb_create_publication(ORB_ID(orb_test));
  ASSERT_NE(pub, nullptr);

  orb_subscription_t *sub = orb_create_subscription(ORB_ID(orb_test));
  ASSERT_NE(sub, nullptr);

  // Drain initial data
  orb_test_s drain{};
  for (int i = 0; i < 8 && orb_check_update(sub); ++i) {
    orb_copy(sub, &drain);
  }

  uevent_source_t *src = uorb_subscription_create_source(sub);
  ASSERT_NE(src, nullptr);

  uevent_t *base = uevent_create();
  ASSERT_NE(base, nullptr);
  ASSERT_EQ(uevent_add(base, src, 0), 0);

  // No data initially
  uevent_source_t *ready[1] = {nullptr};
  EXPECT_EQ(uevent_loop(base, ready, 1, 0), 0);

  // Publish
  orb_test_s msg{};
  msg.val = 42;
  ASSERT_TRUE(orb_publish(pub, &msg));

  // Loop should return the source
  ASSERT_EQ(uevent_loop(base, ready, 1, 100), 1);
  EXPECT_EQ(ready[0], src);

  // Copy data
  ASSERT_TRUE(orb_copy(sub, &msg));
  EXPECT_EQ(msg.val, 42);

  // No more data
  EXPECT_EQ(uevent_loop(base, ready, 1, 0), 0);

  ASSERT_EQ(uevent_remove(base, src), 0);
  uorb_subscription_destroy_source(src);
  uevent_destroy(base);
  EXPECT_TRUE(orb_destroy_subscription(&sub));
  EXPECT_TRUE(orb_destroy_publication(&pub));
}

// ---- Publish from another thread wakes blocking loop ----

TEST(UorbUeventTest, PublishFromThreadWakesBlockingLoop) {
  orb_publication_t *pub = orb_create_publication(ORB_ID(orb_test));
  ASSERT_NE(pub, nullptr);

  orb_subscription_t *sub = orb_create_subscription(ORB_ID(orb_test));
  ASSERT_NE(sub, nullptr);

  // Drain initial data
  orb_test_s drain{};
  for (int i = 0; i < 8 && orb_check_update(sub); ++i) {
    orb_copy(sub, &drain);
  }

  uevent_source_t *src = uorb_subscription_create_source(sub);
  ASSERT_NE(src, nullptr);

  uevent_t *base = uevent_create();
  ASSERT_NE(base, nullptr);
  ASSERT_EQ(uevent_add(base, src, 0), 0);

  std::thread publisher([&]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    orb_test_s msg{};
    msg.val = 99;
    orb_publish(pub, &msg);
  });

  uevent_source_t *ready[1] = {nullptr};
  auto t0 = std::chrono::steady_clock::now();
  ASSERT_EQ(uevent_loop(base, ready, 1, -1), 1);
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                     std::chrono::steady_clock::now() - t0)
                     .count();
  EXPECT_GE(elapsed, 25);
  EXPECT_EQ(ready[0], src);

  publisher.join();

  ASSERT_EQ(uevent_remove(base, src), 0);
  uorb_subscription_destroy_source(src);
  uevent_destroy(base);
  EXPECT_TRUE(orb_destroy_subscription(&sub));
  EXPECT_TRUE(orb_destroy_publication(&pub));
}

// ---- orb_subscription_set_callback / clear_callback ----

TEST(UorbUeventTest, SetCallbackReceivesPublish) {
  orb_publication_t *pub = orb_create_publication(ORB_ID(orb_test));
  ASSERT_NE(pub, nullptr);

  orb_subscription_t *sub = orb_create_subscription(ORB_ID(orb_test));
  ASSERT_NE(sub, nullptr);

  // Drain initial data
  orb_test_s drain{};
  for (int i = 0; i < 8 && orb_check_update(sub); ++i) {
    orb_copy(sub, &drain);
  }

  std::atomic<int> callback_count{0};
  auto cb = [](void *ctx) {
    static_cast<std::atomic<int> *>(ctx)->fetch_add(1);
  };

  ASSERT_TRUE(orb_subscription_set_callback(sub, cb, &callback_count));

  orb_test_s msg{};
  callback_count = 0;
  orb_publish(pub, &msg);
  EXPECT_EQ(callback_count.load(), 1);  // exactly one callback

  ASSERT_TRUE(orb_subscription_clear_callback(sub));
  int before = callback_count.load();
  orb_publish(pub, &msg);
  EXPECT_EQ(callback_count.load(), before);  // no more callbacks

  EXPECT_TRUE(orb_destroy_subscription(&sub));
  EXPECT_TRUE(orb_destroy_publication(&pub));
}

TEST(UorbUeventTest, SetCallbackTwiceFails) {
  orb_subscription_t *sub = orb_create_subscription(ORB_ID(orb_test));
  ASSERT_NE(sub, nullptr);

  auto cb = [](void *) {};
  ASSERT_TRUE(orb_subscription_set_callback(sub, cb, nullptr));

  errno = 0;
  EXPECT_FALSE(orb_subscription_set_callback(sub, cb, nullptr));
  EXPECT_EQ(errno, EBUSY);

  ASSERT_TRUE(orb_subscription_clear_callback(sub));
  // Now should work again
  ASSERT_TRUE(orb_subscription_set_callback(sub, cb, nullptr));
  ASSERT_TRUE(orb_subscription_clear_callback(sub));

  EXPECT_TRUE(orb_destroy_subscription(&sub));
}

TEST(UorbUeventTest, ClearCallbackWithoutSetIsNoop) {
  orb_subscription_t *sub = orb_create_subscription(ORB_ID(orb_test));
  ASSERT_NE(sub, nullptr);
  EXPECT_TRUE(orb_subscription_clear_callback(sub));
  EXPECT_TRUE(orb_destroy_subscription(&sub));
}

// ---- EventLoop C++ wrapper ----

// ---- Multiple topics with separate sources ----

TEST(UorbUeventTest, MultipleTopicsMultipleSources) {
  orb_publication_t *pub1 = orb_create_publication(ORB_ID(orb_test));
  orb_publication_t *pub2 = orb_create_publication(ORB_ID(orb_test_medium));
  ASSERT_NE(pub1, nullptr);
  ASSERT_NE(pub2, nullptr);

  orb_subscription_t *sub1 = orb_create_subscription(ORB_ID(orb_test));
  orb_subscription_t *sub2 = orb_create_subscription(ORB_ID(orb_test_medium));
  ASSERT_NE(sub1, nullptr);
  ASSERT_NE(sub2, nullptr);

  // Drain initial data
  orb_test_s drain1{};
  orb_test_medium_s drain2{};
  for (int i = 0; i < 8 && orb_check_update(sub1); ++i) orb_copy(sub1, &drain1);
  for (int i = 0; i < 8 && orb_check_update(sub2); ++i) orb_copy(sub2, &drain2);

  uevent_source_t *src1 = uorb_subscription_create_source(sub1);
  uevent_source_t *src2 = uorb_subscription_create_source(sub2);
  ASSERT_NE(src1, nullptr);
  ASSERT_NE(src2, nullptr);

  uevent_t *base = uevent_create();
  ASSERT_NE(base, nullptr);
  ASSERT_EQ(uevent_add(base, src1, 0), 0);
  ASSERT_EQ(uevent_add(base, src2, 0), 0);

  // Publish to topic 1 only
  orb_test_s msg1{};
  msg1.val = 111;
  orb_publish(pub1, &msg1);

  uevent_source_t *ready[2] = {nullptr, nullptr};
  int n = uevent_loop(base, ready, 2, 100);
  ASSERT_EQ(n, 1);
  EXPECT_EQ(ready[0], src1);
  // Copy to clear readiness
  orb_test_s recv1{};
  orb_copy(sub1, &recv1);

  // Publish to topic 2 only
  orb_test_medium_s msg2{};
  msg2.val = 222;
  orb_publish(pub2, &msg2);

  n = uevent_loop(base, ready, 2, 100);
  ASSERT_EQ(n, 1);
  EXPECT_EQ(ready[0], src2);
  // Copy to clear readiness
  orb_test_medium_s recv2{};
  orb_copy(sub2, &recv2);

  // Publish to both
  msg1.val = 333;
  orb_publish(pub1, &msg1);
  msg2.val = 444;
  orb_publish(pub2, &msg2);

  n = uevent_loop(base, ready, 2, 100);
  ASSERT_EQ(n, 2);

  ASSERT_EQ(uevent_remove(base, src1), 0);
  ASSERT_EQ(uevent_remove(base, src2), 0);
  uorb_subscription_destroy_source(src1);
  uorb_subscription_destroy_source(src2);
  uevent_destroy(base);
  EXPECT_TRUE(orb_destroy_subscription(&sub1));
  EXPECT_TRUE(orb_destroy_subscription(&sub2));
  EXPECT_TRUE(orb_destroy_publication(&pub1));
  EXPECT_TRUE(orb_destroy_publication(&pub2));
}

// ---- Callback triggered from publisher thread ----

TEST(UorbUeventTest, CallbackTriggeredFromPublisherThread) {
  orb_publication_t *pub = orb_create_publication(ORB_ID(orb_test));
  ASSERT_NE(pub, nullptr);

  orb_subscription_t *sub = orb_create_subscription(ORB_ID(orb_test));
  ASSERT_NE(sub, nullptr);

  // Drain initial data
  orb_test_s drain{};
  for (int i = 0; i < 8 && orb_check_update(sub); ++i) orb_copy(sub, &drain);

  std::atomic<int> callback_count{0};
  auto cb = [](void *ctx) {
    static_cast<std::atomic<int> *>(ctx)->fetch_add(1, std::memory_order_relaxed);
  };

  ASSERT_TRUE(orb_subscription_set_callback(sub, cb, &callback_count));

  // Publish from another thread
  std::thread publisher([&]() {
    orb_test_s msg{};
    for (int i = 0; i < 10; ++i) {
      msg.val = i;
      orb_publish(pub, &msg);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  });

  publisher.join();

  // Should have received callbacks
  EXPECT_EQ(callback_count.load(), 10);

  ASSERT_TRUE(orb_subscription_clear_callback(sub));
  EXPECT_TRUE(orb_destroy_subscription(&sub));
  EXPECT_TRUE(orb_destroy_publication(&pub));
}

// ---- Full lifecycle: create source, add, publish, copy, remove, destroy ----

TEST(UorbUeventTest, FullLifecycleMultipleCycles) {
  orb_publication_t *pub = orb_create_publication(ORB_ID(orb_test));
  ASSERT_NE(pub, nullptr);

  orb_subscription_t *sub = orb_create_subscription(ORB_ID(orb_test));
  ASSERT_NE(sub, nullptr);

  // Drain initial data
  orb_test_s drain{};
  for (int i = 0; i < 8 && orb_check_update(sub); ++i) orb_copy(sub, &drain);

  uevent_t *base = uevent_create();
  ASSERT_NE(base, nullptr);

  // Run 10 cycles of: create source -> add -> publish -> loop -> copy -> remove -> destroy
  for (int cycle = 0; cycle < 10; ++cycle) {
    uevent_source_t *src = uorb_subscription_create_source(sub);
    ASSERT_NE(src, nullptr);
    ASSERT_EQ(uevent_add(base, src, 0), 0);

    orb_test_s msg{};
    msg.val = cycle * 10;
    ASSERT_TRUE(orb_publish(pub, &msg));

    uevent_source_t *ready[1] = {nullptr};
    ASSERT_EQ(uevent_loop(base, ready, 1, 100), 1);
    EXPECT_EQ(ready[0], src);

    ASSERT_TRUE(orb_copy(sub, &msg));
    EXPECT_EQ(msg.val, cycle * 10);

    ASSERT_EQ(uevent_remove(base, src), 0);
    uorb_subscription_destroy_source(src);
  }

  uevent_destroy(base);
  EXPECT_TRUE(orb_destroy_subscription(&sub));
  EXPECT_TRUE(orb_destroy_publication(&pub));
}
