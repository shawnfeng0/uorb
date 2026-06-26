/****************************************************************************
 *
 * Internal tests for uevent::EventPoll behavior.
 *
 ****************************************************************************/

#include <gtest/gtest.h>
#include <uevent/uevent.h>
#include <uorb/topics/orb_test.h>
#include <uorb/uorb.h>
#include <uorb_uevent/uorb_uevent.h>

#include <atomic>
#include <chrono>
#include <thread>

#include "event_poll.h"

namespace uORBTest {

namespace {

uevent::EventSource *as_source(orb_subscription_t *sub) {
  return reinterpret_cast<uevent::EventSource *>(
      uorb_subscription_create_source(sub));
}

// Drain any initial data from a freshly created subscription.
void drain_initial(uevent::EventPoll &poll, uevent::EventSource *src,
                   orb_subscription_t *sub) {
  uevent::EventSource *ready[1] = {nullptr};
  orb_test_s msg{};
  for (int i = 0; i < 8; ++i) {
    int n = poll.Wait(ready, 1, 0);
    if (n <= 0 || ready[0] != src) break;
    orb_copy(sub, &msg);
  }
}

}  // namespace

// ---- WaitZeroReturnsZeroWhenNoReadyData ----
TEST(EventPollInternalTest, WaitZeroReturnsZeroWhenNoReadyData) {
  orb_subscription_t *sub = orb_create_subscription(ORB_ID(orb_test));
  ASSERT_NE(sub, nullptr);

  uevent::EventPoll poll;
  uevent::EventSource *src = as_source(sub);
  ASSERT_TRUE(poll.Add(*src));
  drain_initial(poll, src, sub);

  uevent::EventSource *ready[1] = {nullptr};
  EXPECT_EQ(poll.Wait(ready, 1, 0), 0);

  ASSERT_TRUE(poll.Remove(*src));
  uorb_subscription_destroy_source(reinterpret_cast<uevent_source_t *>(src));
  EXPECT_TRUE(orb_destroy_subscription(&sub));
}

// ---- WaitZeroReturnsReadyReceiverWhenDataArrives ----
TEST(EventPollInternalTest, WaitZeroReturnsReadyReceiverWhenDataArrives) {
  orb_publication_t *pub = orb_create_publication(ORB_ID(orb_test));
  ASSERT_NE(pub, nullptr);

  orb_subscription_t *sub = orb_create_subscription(ORB_ID(orb_test));
  ASSERT_NE(sub, nullptr);

  uevent::EventPoll poll;
  uevent::EventSource *src = as_source(sub);
  ASSERT_TRUE(poll.Add(*src));
  drain_initial(poll, src, sub);

  orb_test_s msg{};
  msg.val = 321;
  ASSERT_TRUE(orb_publish(pub, &msg));

  uevent::EventSource *ready[1] = {nullptr};
  const int n = poll.Wait(ready, 1, 0);
  ASSERT_EQ(n, 1);
  EXPECT_EQ(ready[0], src);

  EXPECT_TRUE(orb_copy(sub, &msg));
  EXPECT_EQ(msg.val, 321);

  ASSERT_TRUE(poll.Remove(*src));
  uorb_subscription_destroy_source(reinterpret_cast<uevent_source_t *>(src));
  EXPECT_TRUE(orb_destroy_subscription(&sub));
  EXPECT_TRUE(orb_destroy_publication(&pub));
}

// ---- WaitReturnsMultipleReadyReceivers ----
TEST(EventPollInternalTest, WaitReturnsMultipleReadyReceivers) {
  orb_publication_t *pub = orb_create_publication(ORB_ID(orb_test));
  ASSERT_NE(pub, nullptr);

  orb_subscription_t *first_sub = orb_create_subscription(ORB_ID(orb_test));
  ASSERT_NE(first_sub, nullptr);
  orb_subscription_t *second_sub = orb_create_subscription(ORB_ID(orb_test));
  ASSERT_NE(second_sub, nullptr);

  uevent::EventPoll poll;
  uevent::EventSource *src1 = as_source(first_sub);
  uevent::EventSource *src2 = as_source(second_sub);
  ASSERT_TRUE(poll.Add(*src1));
  ASSERT_TRUE(poll.Add(*src2));

  { uevent::EventSource *r[2] = {nullptr}; poll.Wait(r, 2, 0); }

  orb_test_s msg{};
  msg.val = 777;
  ASSERT_TRUE(orb_publish(pub, &msg));

  uevent::EventSource *ready[2] = {nullptr, nullptr};
  const int n = poll.Wait(ready, 2, 0);
  ASSERT_EQ(n, 2);
  EXPECT_TRUE((ready[0] == src1 && ready[1] == src2) ||
              (ready[0] == src2 && ready[1] == src1));

  ASSERT_TRUE(poll.Remove(*src1));
  ASSERT_TRUE(poll.Remove(*src2));
  uorb_subscription_destroy_source(reinterpret_cast<uevent_source_t *>(src1));
  uorb_subscription_destroy_source(reinterpret_cast<uevent_source_t *>(src2));
  EXPECT_TRUE(orb_destroy_subscription(&first_sub));
  EXPECT_TRUE(orb_destroy_subscription(&second_sub));
  EXPECT_TRUE(orb_destroy_publication(&pub));
}

// ---- WaitTruncatesReadyOutputToCapacity ----
TEST(EventPollInternalTest, WaitTruncatesReadyOutputToCapacity) {
  orb_publication_t *pub = orb_create_publication(ORB_ID(orb_test));
  ASSERT_NE(pub, nullptr);

  orb_subscription_t *first_sub = orb_create_subscription(ORB_ID(orb_test));
  ASSERT_NE(first_sub, nullptr);
  orb_subscription_t *second_sub = orb_create_subscription(ORB_ID(orb_test));
  ASSERT_NE(second_sub, nullptr);

  uevent::EventPoll poll;
  uevent::EventSource *src1 = as_source(first_sub);
  uevent::EventSource *src2 = as_source(second_sub);
  ASSERT_TRUE(poll.Add(*src1));
  ASSERT_TRUE(poll.Add(*src2));

  { uevent::EventSource *r[2] = {nullptr}; poll.Wait(r, 2, 0); }

  orb_test_s msg{};
  msg.val = 888;
  ASSERT_TRUE(orb_publish(pub, &msg));

  uevent::EventSource *ready[1] = {nullptr};
  ASSERT_EQ(poll.Wait(ready, 1, 0), 1);
  ASSERT_TRUE(ready[0] == src1 || ready[0] == src2);
  EXPECT_TRUE(orb_copy(ready[0] == src1 ? first_sub : second_sub, &msg));

  uevent::EventSource *remaining_ready[1] = {nullptr};
  ASSERT_EQ(poll.Wait(remaining_ready, 1, 0), 1);
  EXPECT_NE(remaining_ready[0], ready[0]);
  EXPECT_TRUE(orb_copy(remaining_ready[0] == src1 ? first_sub : second_sub, &msg));

  ASSERT_TRUE(poll.Remove(*src1));
  ASSERT_TRUE(poll.Remove(*src2));
  uorb_subscription_destroy_source(reinterpret_cast<uevent_source_t *>(src1));
  uorb_subscription_destroy_source(reinterpret_cast<uevent_source_t *>(src2));
  EXPECT_TRUE(orb_destroy_subscription(&first_sub));
  EXPECT_TRUE(orb_destroy_subscription(&second_sub));
  EXPECT_TRUE(orb_destroy_publication(&pub));
}

// ---- WaitAfterStopReturnsMinusOneOnce ----
TEST(EventPollInternalTest, WaitAfterStopReturnsMinusOneOnce) {
  orb_subscription_t *sub = orb_create_subscription(ORB_ID(orb_test));
  ASSERT_NE(sub, nullptr);

  uevent::EventPoll poll;
  uevent::EventSource *src = as_source(sub);
  ASSERT_TRUE(poll.Add(*src));
  drain_initial(poll, src, sub);

  poll.Stop();

  uevent::EventSource *ready[1] = {nullptr};
  EXPECT_EQ(poll.Wait(ready, 1, 0), -1);
  EXPECT_EQ(poll.Wait(ready, 1, 0), 0);

  ASSERT_TRUE(poll.Remove(*src));
  uorb_subscription_destroy_source(reinterpret_cast<uevent_source_t *>(src));
  EXPECT_TRUE(orb_destroy_subscription(&sub));
}

// ---- WaitInterruptedByStopReturnsMinusOne ----
TEST(EventPollInternalTest, WaitInterruptedByStopReturnsMinusOne) {
  orb_subscription_t *sub = orb_create_subscription(ORB_ID(orb_test));
  ASSERT_NE(sub, nullptr);

  uevent::EventPoll poll;
  uevent::EventSource *src = as_source(sub);
  ASSERT_TRUE(poll.Add(*src));
  drain_initial(poll, src, sub);

  std::atomic<int> wait_result{0};
  std::thread waiter([&]() {
    uevent::EventSource *ready[1] = {nullptr};
    wait_result = poll.Wait(ready, 1, -1);
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  poll.Stop();

  waiter.join();
  EXPECT_EQ(wait_result.load(), -1);

  ASSERT_TRUE(poll.Remove(*src));
  uorb_subscription_destroy_source(reinterpret_cast<uevent_source_t *>(src));
  EXPECT_TRUE(orb_destroy_subscription(&sub));
}

// ---- AddPollableIsIdempotentForSamePoll ----
TEST(EventPollInternalTest, AddPollableIsIdempotentForSamePoll) {
  orb_subscription_t *sub = orb_create_subscription(ORB_ID(orb_test));
  ASSERT_NE(sub, nullptr);

  uevent::EventPoll poll;
  uevent::EventSource *src = as_source(sub);
  EXPECT_TRUE(poll.Add(*src));
  EXPECT_TRUE(poll.Add(*src));
  EXPECT_TRUE(poll.Remove(*src));
  EXPECT_FALSE(poll.Remove(*src));
  uorb_subscription_destroy_source(reinterpret_cast<uevent_source_t *>(src));
  EXPECT_TRUE(orb_destroy_subscription(&sub));
}

// ---- RemovePollableRejectsReceiverBoundToAnotherPoll ----
TEST(EventPollInternalTest, RemovePollableRejectsReceiverBoundToAnotherPoll) {
  orb_subscription_t *sub = orb_create_subscription(ORB_ID(orb_test));
  ASSERT_NE(sub, nullptr);

  uevent::EventPoll owner_poll;
  uevent::EventPoll other_poll;
  uevent::EventSource *src = as_source(sub);

  ASSERT_TRUE(owner_poll.Add(*src));
  EXPECT_FALSE(other_poll.Remove(*src));
  EXPECT_EQ(errno, EBUSY);
  ASSERT_TRUE(owner_poll.Remove(*src));
  uorb_subscription_destroy_source(reinterpret_cast<uevent_source_t *>(src));
  EXPECT_TRUE(orb_destroy_subscription(&sub));
}

// ---- DestroySubscriptionAfterUnbinding ----
TEST(EventPollInternalTest, DestroySubscriptionAfterUnbinding) {
  orb_subscription_t *sub = orb_create_subscription(ORB_ID(orb_test));
  ASSERT_NE(sub, nullptr);

  uevent::EventPoll poll;
  uevent::EventSource *src = as_source(sub);
  ASSERT_TRUE(poll.Add(*src));

  ASSERT_TRUE(poll.Remove(*src));
  uorb_subscription_destroy_source(reinterpret_cast<uevent_source_t *>(src));
  EXPECT_TRUE(orb_destroy_subscription(&sub));
  EXPECT_EQ(sub, nullptr);
}

// ---- WaitRejectsZeroMaxReady ----
TEST(EventPollInternalTest, WaitRejectsZeroMaxReady) {
  uevent::EventPoll poll;
  uevent::EventSource *ready[1] = {nullptr};

  errno = 0;
  EXPECT_EQ(poll.Wait(ready, 0, 0), -1);
  EXPECT_EQ(errno, EINVAL);
}

// ---- WaitRejectsNullReadyArray ----
TEST(EventPollInternalTest, WaitRejectsNullReadyArray) {
  uevent::EventPoll poll;

  errno = 0;
  EXPECT_EQ(poll.Wait(nullptr, 1, 0), -1);
  EXPECT_EQ(errno, EINVAL);
}

// ---- WaitRejectsNegativeMaxReady ----
TEST(EventPollInternalTest, WaitRejectsNegativeMaxReady) {
  uevent::EventPoll poll;
  uevent::EventSource *ready[1] = {nullptr};

  errno = 0;
  EXPECT_EQ(poll.Wait(ready, -1, 0), -1);
  EXPECT_EQ(errno, EINVAL);
}

// ---- AddRemovePressureWithActivePublisher ----
TEST(EventPollInternalTest, AddRemovePressureWithActivePublisher) {
  orb_publication_t *pub = orb_create_publication(ORB_ID(orb_test));
  ASSERT_NE(pub, nullptr);

  orb_subscription_t *sub = orb_create_subscription(ORB_ID(orb_test));
  ASSERT_NE(sub, nullptr);

  std::atomic<bool> running{true};
  std::thread publisher([&]() {
    orb_test_s msg{};
    int v = 0;
    while (running.load(std::memory_order_relaxed)) {
      msg.val = ++v;
      orb_publish(pub, &msg);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  });

  uevent::EventPoll poll;
  uevent::EventSource *ready[1] = {nullptr};
  uevent::EventSource *src = as_source(sub);
  ASSERT_NE(src, nullptr);
  orb_test_s msg{};

  for (int i = 0; i < 200; ++i) {
    ASSERT_TRUE(poll.Add(*src));
    const int n = poll.Wait(ready, 1, 20);
    ASSERT_GE(n, 0);
    if (n > 0 && ready[0] == src) {
      ASSERT_TRUE(orb_copy(sub, &msg));
    }
    ASSERT_TRUE(poll.Remove(*src));
  }

  running = false;
  publisher.join();

  uorb_subscription_destroy_source(reinterpret_cast<uevent_source_t *>(src));
  EXPECT_TRUE(orb_destroy_subscription(&sub));
  EXPECT_TRUE(orb_destroy_publication(&pub));
}

// ---- PerEventTimeoutFiresWithoutData ----
TEST(EventPollInternalTest, PerEventTimeoutFiresWithoutData) {
  orb_subscription_t *sub = orb_create_subscription(ORB_ID(orb_test));
  ASSERT_NE(sub, nullptr);

  uevent::EventPoll poll;
  uevent::EventSource *src = as_source(sub);
  ASSERT_TRUE(poll.Add(*src));
  drain_initial(poll, src, sub);

  ASSERT_TRUE(poll.Remove(*src));
  ASSERT_TRUE(poll.Add(*src, 50));

  uevent::EventSource *ready[1] = {nullptr};
  auto t0 = std::chrono::steady_clock::now();
  int n = poll.Wait(ready, 1, -1);
  auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - t0)
                        .count();

  ASSERT_EQ(n, 1);
  EXPECT_EQ(ready[0], src);
  EXPECT_GE(elapsed_ms, 30);
  EXPECT_LE(elapsed_ms, 200);

  EXPECT_EQ(poll.Wait(ready, 1, 0), 0);

  ASSERT_TRUE(poll.Remove(*src));
  uorb_subscription_destroy_source(reinterpret_cast<uevent_source_t *>(src));
  EXPECT_TRUE(orb_destroy_subscription(&sub));
}

// ---- PerEventTimeoutZeroMeansNoTimeout ----
TEST(EventPollInternalTest, PerEventTimeoutZeroMeansNoTimeout) {
  orb_subscription_t *sub = orb_create_subscription(ORB_ID(orb_test));
  ASSERT_NE(sub, nullptr);

  uevent::EventPoll poll;
  uevent::EventSource *src = as_source(sub);
  ASSERT_TRUE(poll.Add(*src, 0));
  drain_initial(poll, src, sub);

  uevent::EventSource *ready[1] = {nullptr};
  EXPECT_EQ(poll.Wait(ready, 1, 0), 0);

  ASSERT_TRUE(poll.Remove(*src));
  uorb_subscription_destroy_source(reinterpret_cast<uevent_source_t *>(src));
  EXPECT_TRUE(orb_destroy_subscription(&sub));
}

// ---- CustomEventSourceFromC ----
TEST(EventPollInternalTest, CustomEventSourceFromC) {
  struct Ctx {
    std::atomic<bool> has_data{false};
  };
  Ctx ctx;

  auto ready_fn = [](void *p) -> bool {
    return static_cast<Ctx *>(p)->has_data.load();
  };

  uevent_source_t *src =
      uevent_source_create(ready_fn, nullptr, nullptr, nullptr, &ctx);
  ASSERT_NE(src, nullptr);

  uevent::EventPoll poll;
  ASSERT_TRUE(poll.Add(*reinterpret_cast<uevent::EventSource *>(src)));

  uevent::EventSource *ready[1] = {nullptr};
  EXPECT_EQ(poll.Wait(ready, 1, 0), 0);

  ctx.has_data = true;
  uevent_source_notify(src);

  ASSERT_EQ(poll.Wait(ready, 1, 100), 1);
  EXPECT_EQ(ready[0], reinterpret_cast<uevent::EventSource *>(src));

  ctx.has_data = false;
  EXPECT_EQ(poll.Wait(ready, 1, 0), 0);

  EXPECT_TRUE(poll.Remove(*reinterpret_cast<uevent::EventSource *>(src)));
  uevent_source_destroy(src);
}

}  // namespace uORBTest
