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
#include <vector>

#include "event_poll.h"

namespace uORBTest {

namespace {

// Drain any initial data from a freshly created subscription.
void drain_initial(uevent::EventPoll &poll, uevent::EventSource *src,
                   orb_subscriber_t *sub) {
  uevent::EventSource *ready[1] = {nullptr};
  orb_test_s msg{};
  for (int i = 0; i < 8; ++i) {
    int n = poll.Wait(ready, 1, 0);
    if (n <= 0 || ready[0] != src) break;
    orb_subscriber_copy(sub, &msg);
  }
}

}  // namespace

// ---- WaitZeroReturnsZeroWhenNoReadyData ----
TEST(EventPollInternalTest, WaitZeroReturnsZeroWhenNoReadyData) {
  orb_subscriber_t sub = ORB_SUBSCRIBER_INITIALIZER;
  EXPECT_EQ(orb_subscriber_create(&sub, ORB_ID(orb_test)), ORB_OK);
  ASSERT_NE(sub._handle, nullptr);

  uevent::EventPoll poll;
  uevent_source_t src = UEVENT_SOURCE_INITIALIZER;
  ASSERT_EQ(uorb_subscriber_create_source(&src, &sub), 0);
  uevent::EventSource *es = reinterpret_cast<uevent::EventSource *>(src._handle);
  ASSERT_TRUE(poll.Add(*es));
  drain_initial(poll, es, &sub);

  uevent::EventSource *ready[1] = {nullptr};
  EXPECT_EQ(poll.Wait(ready, 1, 0), 0);

  ASSERT_TRUE(poll.Remove(*es));
  uorb_subscriber_destroy_source(&src);
  EXPECT_EQ(orb_subscriber_destroy(&sub), ORB_OK);
}

// ---- WaitZeroReturnsReadyReceiverWhenDataArrives ----
TEST(EventPollInternalTest, WaitZeroReturnsReadyReceiverWhenDataArrives) {
  orb_publisher_t pub = ORB_PUBLISHER_INITIALIZER;
  EXPECT_EQ(orb_publisher_create(&pub, ORB_ID(orb_test)), ORB_OK);
  ASSERT_NE(pub._handle, nullptr);

  orb_subscriber_t sub = ORB_SUBSCRIBER_INITIALIZER;
  EXPECT_EQ(orb_subscriber_create(&sub, ORB_ID(orb_test)), ORB_OK);
  ASSERT_NE(sub._handle, nullptr);

  uevent::EventPoll poll;
  uevent_source_t src = UEVENT_SOURCE_INITIALIZER;
  ASSERT_EQ(uorb_subscriber_create_source(&src, &sub), 0);
  uevent::EventSource *es = reinterpret_cast<uevent::EventSource *>(src._handle);
  ASSERT_TRUE(poll.Add(*es));
  drain_initial(poll, es, &sub);

  orb_test_s msg{};
  msg.val = 321;
  ASSERT_EQ(orb_publisher_publish(&pub, &msg), ORB_OK);

  uevent::EventSource *ready[1] = {nullptr};
  const int n = poll.Wait(ready, 1, 0);
  ASSERT_EQ(n, 1);
  EXPECT_EQ(ready[0], es);

  EXPECT_EQ(orb_subscriber_copy(&sub, &msg), ORB_OK);
  EXPECT_EQ(msg.val, 321);

  ASSERT_TRUE(poll.Remove(*es));
  uorb_subscriber_destroy_source(&src);
  EXPECT_EQ(orb_subscriber_destroy(&sub), ORB_OK);
  EXPECT_EQ(orb_publisher_destroy(&pub), ORB_OK);
}

// ---- WaitReturnsMultipleReadyReceivers ----
TEST(EventPollInternalTest, WaitReturnsMultipleReadyReceivers) {
  orb_publisher_t pub = ORB_PUBLISHER_INITIALIZER;
  EXPECT_EQ(orb_publisher_create(&pub, ORB_ID(orb_test)), ORB_OK);
  ASSERT_NE(pub._handle, nullptr);

  orb_subscriber_t first_sub = ORB_SUBSCRIBER_INITIALIZER;
  orb_subscriber_t second_sub = ORB_SUBSCRIBER_INITIALIZER;
  EXPECT_EQ(orb_subscriber_create(&first_sub, ORB_ID(orb_test)), ORB_OK);
  EXPECT_EQ(orb_subscriber_create(&second_sub, ORB_ID(orb_test)), ORB_OK);
  ASSERT_NE(first_sub._handle, nullptr);
  ASSERT_NE(second_sub._handle, nullptr);

  uevent::EventPoll poll;
  uevent_source_t src1 = UEVENT_SOURCE_INITIALIZER;
  uevent_source_t src2 = UEVENT_SOURCE_INITIALIZER;
  ASSERT_EQ(uorb_subscriber_create_source(&src1, &first_sub), 0);
  ASSERT_EQ(uorb_subscriber_create_source(&src2, &second_sub), 0);
  uevent::EventSource *es1 = reinterpret_cast<uevent::EventSource *>(src1._handle);
  uevent::EventSource *es2 = reinterpret_cast<uevent::EventSource *>(src2._handle);
  ASSERT_TRUE(poll.Add(*es1));
  ASSERT_TRUE(poll.Add(*es2));

  { uevent::EventSource *r[2] = {nullptr}; poll.Wait(r, 2, 0); }

  orb_test_s msg{};
  msg.val = 777;
  ASSERT_EQ(orb_publisher_publish(&pub, &msg), ORB_OK);

  uevent::EventSource *ready[2] = {nullptr, nullptr};
  const int n = poll.Wait(ready, 2, 0);
  ASSERT_EQ(n, 2);
  EXPECT_TRUE((ready[0] == es1 && ready[1] == es2) ||
              (ready[0] == es2 && ready[1] == es1));

  ASSERT_TRUE(poll.Remove(*es1));
  ASSERT_TRUE(poll.Remove(*es2));
  uorb_subscriber_destroy_source(&src1);
  uorb_subscriber_destroy_source(&src2);
  EXPECT_EQ(orb_subscriber_destroy(&first_sub), ORB_OK);
  EXPECT_EQ(orb_subscriber_destroy(&second_sub), ORB_OK);
  EXPECT_EQ(orb_publisher_destroy(&pub), ORB_OK);
}

// ---- WaitTruncatesReadyOutputToCapacity ----
TEST(EventPollInternalTest, WaitTruncatesReadyOutputToCapacity) {
  orb_publisher_t pub = ORB_PUBLISHER_INITIALIZER;
  EXPECT_EQ(orb_publisher_create(&pub, ORB_ID(orb_test)), ORB_OK);
  ASSERT_NE(pub._handle, nullptr);

  orb_subscriber_t first_sub = ORB_SUBSCRIBER_INITIALIZER;
  orb_subscriber_t second_sub = ORB_SUBSCRIBER_INITIALIZER;
  EXPECT_EQ(orb_subscriber_create(&first_sub, ORB_ID(orb_test)), ORB_OK);
  EXPECT_EQ(orb_subscriber_create(&second_sub, ORB_ID(orb_test)), ORB_OK);
  ASSERT_NE(first_sub._handle, nullptr);
  ASSERT_NE(second_sub._handle, nullptr);

  uevent::EventPoll poll;
  uevent_source_t src1 = UEVENT_SOURCE_INITIALIZER;
  uevent_source_t src2 = UEVENT_SOURCE_INITIALIZER;
  ASSERT_EQ(uorb_subscriber_create_source(&src1, &first_sub), 0);
  ASSERT_EQ(uorb_subscriber_create_source(&src2, &second_sub), 0);
  uevent::EventSource *es1 = reinterpret_cast<uevent::EventSource *>(src1._handle);
  uevent::EventSource *es2 = reinterpret_cast<uevent::EventSource *>(src2._handle);
  ASSERT_TRUE(poll.Add(*es1));
  ASSERT_TRUE(poll.Add(*es2));

  { uevent::EventSource *r[2] = {nullptr}; poll.Wait(r, 2, 0); }

  orb_test_s msg{};
  msg.val = 888;
  ASSERT_EQ(orb_publisher_publish(&pub, &msg), ORB_OK);

  uevent::EventSource *ready[1] = {nullptr};
  ASSERT_EQ(poll.Wait(ready, 1, 0), 1);
  ASSERT_TRUE(ready[0] == es1 || ready[0] == es2);
  EXPECT_EQ(orb_subscriber_copy(ready[0] == es1 ? &first_sub : &second_sub, &msg), ORB_OK);

  uevent::EventSource *remaining_ready[1] = {nullptr};
  ASSERT_EQ(poll.Wait(remaining_ready, 1, 0), 1);
  EXPECT_NE(remaining_ready[0], ready[0]);
  EXPECT_EQ(orb_subscriber_copy(remaining_ready[0] == es1 ? &first_sub : &second_sub, &msg), ORB_OK);

  ASSERT_TRUE(poll.Remove(*es1));
  ASSERT_TRUE(poll.Remove(*es2));
  uorb_subscriber_destroy_source(&src1);
  uorb_subscriber_destroy_source(&src2);
  EXPECT_EQ(orb_subscriber_destroy(&first_sub), ORB_OK);
  EXPECT_EQ(orb_subscriber_destroy(&second_sub), ORB_OK);
  EXPECT_EQ(orb_publisher_destroy(&pub), ORB_OK);
}

// ---- WaitAfterStopReturnsMinusOneOnce ----
TEST(EventPollInternalTest, WaitAfterStopReturnsMinusOneOnce) {
  orb_subscriber_t sub = ORB_SUBSCRIBER_INITIALIZER;
  EXPECT_EQ(orb_subscriber_create(&sub, ORB_ID(orb_test)), ORB_OK);
  ASSERT_NE(sub._handle, nullptr);

  uevent::EventPoll poll;
  uevent_source_t src = UEVENT_SOURCE_INITIALIZER;
  ASSERT_EQ(uorb_subscriber_create_source(&src, &sub), 0);
  uevent::EventSource *es = reinterpret_cast<uevent::EventSource *>(src._handle);
  ASSERT_TRUE(poll.Add(*es));
  drain_initial(poll, es, &sub);

  poll.Stop();

  uevent::EventSource *ready[1] = {nullptr};
  EXPECT_EQ(poll.Wait(ready, 1, 0), -1);
  EXPECT_EQ(poll.Wait(ready, 1, 0), 0);

  ASSERT_TRUE(poll.Remove(*es));
  uorb_subscriber_destroy_source(&src);
  EXPECT_EQ(orb_subscriber_destroy(&sub), ORB_OK);
}

// ---- WaitInterruptedByStopReturnsMinusOne ----
TEST(EventPollInternalTest, WaitInterruptedByStopReturnsMinusOne) {
  orb_subscriber_t sub = ORB_SUBSCRIBER_INITIALIZER;
  EXPECT_EQ(orb_subscriber_create(&sub, ORB_ID(orb_test)), ORB_OK);
  ASSERT_NE(sub._handle, nullptr);

  uevent::EventPoll poll;
  uevent_source_t src = UEVENT_SOURCE_INITIALIZER;
  ASSERT_EQ(uorb_subscriber_create_source(&src, &sub), 0);
  uevent::EventSource *es = reinterpret_cast<uevent::EventSource *>(src._handle);
  ASSERT_TRUE(poll.Add(*es));
  drain_initial(poll, es, &sub);

  std::atomic<int> wait_result{0};
  std::thread waiter([&]() {
    uevent::EventSource *ready[1] = {nullptr};
    wait_result = poll.Wait(ready, 1, -1);
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  poll.Stop();

  waiter.join();
  EXPECT_EQ(wait_result.load(), -1);

  ASSERT_TRUE(poll.Remove(*es));
  uorb_subscriber_destroy_source(&src);
  EXPECT_EQ(orb_subscriber_destroy(&sub), ORB_OK);
}

// ---- AddPollableIsIdempotentForSamePoll ----
TEST(EventPollInternalTest, AddPollableIsIdempotentForSamePoll) {
  orb_subscriber_t sub = ORB_SUBSCRIBER_INITIALIZER;
  EXPECT_EQ(orb_subscriber_create(&sub, ORB_ID(orb_test)), ORB_OK);
  ASSERT_NE(sub._handle, nullptr);

  uevent::EventPoll poll;
  uevent_source_t src = UEVENT_SOURCE_INITIALIZER;
  ASSERT_EQ(uorb_subscriber_create_source(&src, &sub), 0);
  uevent::EventSource *es = reinterpret_cast<uevent::EventSource *>(src._handle);
  EXPECT_TRUE(poll.Add(*es));
  EXPECT_TRUE(poll.Add(*es));
  EXPECT_TRUE(poll.Remove(*es));
  EXPECT_FALSE(poll.Remove(*es));
  uorb_subscriber_destroy_source(&src);
  EXPECT_EQ(orb_subscriber_destroy(&sub), ORB_OK);
}

// ---- RemovePollableRejectsReceiverBoundToAnotherPoll ----
TEST(EventPollInternalTest, RemovePollableRejectsReceiverBoundToAnotherPoll) {
  orb_subscriber_t sub = ORB_SUBSCRIBER_INITIALIZER;
  EXPECT_EQ(orb_subscriber_create(&sub, ORB_ID(orb_test)), ORB_OK);
  ASSERT_NE(sub._handle, nullptr);

  uevent::EventPoll owner_poll;
  uevent::EventPoll other_poll;
  uevent_source_t src = UEVENT_SOURCE_INITIALIZER;
  ASSERT_EQ(uorb_subscriber_create_source(&src, &sub), 0);
  uevent::EventSource *es = reinterpret_cast<uevent::EventSource *>(src._handle);

  ASSERT_TRUE(owner_poll.Add(*es));
  EXPECT_FALSE(other_poll.Remove(*es));
  EXPECT_EQ(errno, EBUSY);
  ASSERT_TRUE(owner_poll.Remove(*es));
  uorb_subscriber_destroy_source(&src);
  EXPECT_EQ(orb_subscriber_destroy(&sub), ORB_OK);
}

// ---- DestroySubscriptionAfterUnbinding ----
TEST(EventPollInternalTest, DestroySubscriptionAfterUnbinding) {
  orb_subscriber_t sub = ORB_SUBSCRIBER_INITIALIZER;
  EXPECT_EQ(orb_subscriber_create(&sub, ORB_ID(orb_test)), ORB_OK);
  ASSERT_NE(sub._handle, nullptr);

  uevent::EventPoll poll;
  uevent_source_t src = UEVENT_SOURCE_INITIALIZER;
  ASSERT_EQ(uorb_subscriber_create_source(&src, &sub), 0);
  uevent::EventSource *es = reinterpret_cast<uevent::EventSource *>(src._handle);
  ASSERT_TRUE(poll.Add(*es));

  ASSERT_TRUE(poll.Remove(*es));
  uorb_subscriber_destroy_source(&src);
  EXPECT_EQ(orb_subscriber_destroy(&sub), ORB_OK);
  EXPECT_EQ(sub._handle, nullptr);
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
  orb_publisher_t pub = ORB_PUBLISHER_INITIALIZER;
  EXPECT_EQ(orb_publisher_create(&pub, ORB_ID(orb_test)), ORB_OK);
  ASSERT_NE(pub._handle, nullptr);

  orb_subscriber_t sub = ORB_SUBSCRIBER_INITIALIZER;
  EXPECT_EQ(orb_subscriber_create(&sub, ORB_ID(orb_test)), ORB_OK);
  ASSERT_NE(sub._handle, nullptr);

  std::atomic<bool> running{true};
  std::thread publisher([&]() {
    orb_test_s msg{};
    int v = 0;
    while (running.load(std::memory_order_relaxed)) {
      msg.val = ++v;
      orb_publisher_publish(&pub, &msg);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  });

  uevent::EventPoll poll;
  uevent::EventSource *ready[1] = {nullptr};
  uevent_source_t src = UEVENT_SOURCE_INITIALIZER;
  ASSERT_EQ(uorb_subscriber_create_source(&src, &sub), 0);
  uevent::EventSource *es = reinterpret_cast<uevent::EventSource *>(src._handle);
  ASSERT_NE(es, nullptr);
  orb_test_s msg{};

  for (int i = 0; i < 200; ++i) {
    ASSERT_TRUE(poll.Add(*es));
    const int n = poll.Wait(ready, 1, 20);
    ASSERT_GE(n, 0);
    if (n > 0 && ready[0] == es) {
      ASSERT_EQ(orb_subscriber_copy(&sub, &msg), ORB_OK);
    }
    ASSERT_TRUE(poll.Remove(*es));
  }

  running = false;
  publisher.join();

  uorb_subscriber_destroy_source(&src);
  EXPECT_EQ(orb_subscriber_destroy(&sub), ORB_OK);
  EXPECT_EQ(orb_publisher_destroy(&pub), ORB_OK);
}

// ---- PerEventTimeoutFiresWithoutData ----
TEST(EventPollInternalTest, PerEventTimeoutFiresWithoutData) {
  orb_subscriber_t sub = ORB_SUBSCRIBER_INITIALIZER;
  EXPECT_EQ(orb_subscriber_create(&sub, ORB_ID(orb_test)), ORB_OK);
  ASSERT_NE(sub._handle, nullptr);

  uevent::EventPoll poll;
  uevent_source_t src = UEVENT_SOURCE_INITIALIZER;
  ASSERT_EQ(uorb_subscriber_create_source(&src, &sub), 0);
  uevent::EventSource *es = reinterpret_cast<uevent::EventSource *>(src._handle);
  ASSERT_TRUE(poll.Add(*es));
  drain_initial(poll, es, &sub);

  ASSERT_TRUE(poll.Remove(*es));
  ASSERT_TRUE(poll.Add(*es, 50));

  uevent::EventSource *ready[1] = {nullptr};
  auto t0 = std::chrono::steady_clock::now();
  int n = poll.Wait(ready, 1, -1);
  auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - t0)
                        .count();

  ASSERT_EQ(n, 1);
  EXPECT_EQ(ready[0], es);
  EXPECT_GE(elapsed_ms, 30);
  EXPECT_LE(elapsed_ms, 200);

  EXPECT_EQ(poll.Wait(ready, 1, 0), 0);

  ASSERT_TRUE(poll.Remove(*es));
  uorb_subscriber_destroy_source(&src);
  EXPECT_EQ(orb_subscriber_destroy(&sub), ORB_OK);
}

// ---- PerEventTimeoutZeroMeansNoTimeout ----
TEST(EventPollInternalTest, PerEventTimeoutZeroMeansNoTimeout) {
  orb_subscriber_t sub = ORB_SUBSCRIBER_INITIALIZER;
  EXPECT_EQ(orb_subscriber_create(&sub, ORB_ID(orb_test)), ORB_OK);
  ASSERT_NE(sub._handle, nullptr);

  uevent::EventPoll poll;
  uevent_source_t src = UEVENT_SOURCE_INITIALIZER;
  ASSERT_EQ(uorb_subscriber_create_source(&src, &sub), 0);
  uevent::EventSource *es = reinterpret_cast<uevent::EventSource *>(src._handle);
  ASSERT_TRUE(poll.Add(*es, 0));
  drain_initial(poll, es, &sub);

  uevent::EventSource *ready[1] = {nullptr};
  EXPECT_EQ(poll.Wait(ready, 1, 0), 0);

  ASSERT_TRUE(poll.Remove(*es));
  uorb_subscriber_destroy_source(&src);
  EXPECT_EQ(orb_subscriber_destroy(&sub), ORB_OK);
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

  uevent_source_t src = UEVENT_SOURCE_INITIALIZER;
  uevent_source_create(&src, ready_fn, nullptr, nullptr, nullptr, &ctx);
  ASSERT_NE(src._handle, nullptr);

  uevent::EventPoll poll;
  ASSERT_TRUE(poll.Add(*reinterpret_cast<uevent::EventSource *>(src._handle)));

  uevent::EventSource *ready[1] = {nullptr};
  EXPECT_EQ(poll.Wait(ready, 1, 0), 0);

  ctx.has_data = true;
  uevent_source_notify(&src);

  ASSERT_EQ(poll.Wait(ready, 1, 100), 1);
  EXPECT_EQ(ready[0], reinterpret_cast<uevent::EventSource *>(src._handle));

  ctx.has_data = false;
  EXPECT_EQ(poll.Wait(ready, 1, 0), 0);

  EXPECT_TRUE(poll.Remove(*reinterpret_cast<uevent::EventSource *>(src._handle)));
  uevent_source_destroy(&src);
}

// ---- AddSourceAlreadyReady ----
TEST(EventPollInternalTest, AddSourceAlreadyReady) {
  orb_publisher_t pub = ORB_PUBLISHER_INITIALIZER;
  EXPECT_EQ(orb_publisher_create(&pub, ORB_ID(orb_test)), ORB_OK);
  ASSERT_NE(pub._handle, nullptr);

  orb_subscriber_t sub = ORB_SUBSCRIBER_INITIALIZER;
  EXPECT_EQ(orb_subscriber_create(&sub, ORB_ID(orb_test)), ORB_OK);
  ASSERT_NE(sub._handle, nullptr);

  // Clear any stale data so the source is not initially ready.
  orb_test_s tmp{};
  orb_subscriber_copy(&sub, &tmp);

  uevent_source_t src = UEVENT_SOURCE_INITIALIZER;
  ASSERT_EQ(uorb_subscriber_create_source(&src, &sub), 0);
  uevent::EventSource *es = reinterpret_cast<uevent::EventSource *>(src._handle);

  // Publish data BEFORE adding to poll.
  orb_test_s msg{};
  msg.val = 42;
  ASSERT_EQ(orb_publisher_publish(&pub, &msg), ORB_OK);

  // Add should detect the source is already ready and place it in ready_set_.
  uevent::EventPoll poll;
  ASSERT_TRUE(poll.Add(*es));
  EXPECT_TRUE(es->is_ready());

  // Wait should return 1 immediately.
  uevent::EventSource *ready[1] = {nullptr};
  EXPECT_EQ(poll.Wait(ready, 1, 0), 1);
  EXPECT_EQ(ready[0], es);

  ASSERT_TRUE(poll.Remove(*es));
  uorb_subscriber_destroy_source(&src);
  EXPECT_EQ(orb_subscriber_destroy(&sub), ORB_OK);
  EXPECT_EQ(orb_publisher_destroy(&pub), ORB_OK);
}

// ---- WaitStarvationOneAlwaysReady ----
TEST(EventPollInternalTest, WaitStarvationOneAlwaysReady) {
  orb_publisher_t pub = ORB_PUBLISHER_INITIALIZER;
  EXPECT_EQ(orb_publisher_create(&pub, ORB_ID(orb_test)), ORB_OK);
  ASSERT_NE(pub._handle, nullptr);

  orb_subscriber_t sub1 = ORB_SUBSCRIBER_INITIALIZER;
  orb_subscriber_t sub2 = ORB_SUBSCRIBER_INITIALIZER;
  orb_subscriber_t sub3 = ORB_SUBSCRIBER_INITIALIZER;
  EXPECT_EQ(orb_subscriber_create(&sub1, ORB_ID(orb_test)), ORB_OK);
  EXPECT_EQ(orb_subscriber_create(&sub2, ORB_ID(orb_test)), ORB_OK);
  EXPECT_EQ(orb_subscriber_create(&sub3, ORB_ID(orb_test)), ORB_OK);
  ASSERT_NE(sub1._handle, nullptr);
  ASSERT_NE(sub2._handle, nullptr);
  ASSERT_NE(sub3._handle, nullptr);

  uevent_source_t src1 = UEVENT_SOURCE_INITIALIZER;
  uevent_source_t src2 = UEVENT_SOURCE_INITIALIZER;
  uevent_source_t src3 = UEVENT_SOURCE_INITIALIZER;
  ASSERT_EQ(uorb_subscriber_create_source(&src1, &sub1), 0);
  ASSERT_EQ(uorb_subscriber_create_source(&src2, &sub2), 0);
  ASSERT_EQ(uorb_subscriber_create_source(&src3, &sub3), 0);
  uevent::EventSource *es1 = reinterpret_cast<uevent::EventSource *>(src1._handle);
  uevent::EventSource *es2 = reinterpret_cast<uevent::EventSource *>(src2._handle);
  uevent::EventSource *es3 = reinterpret_cast<uevent::EventSource *>(src3._handle);

  uevent::EventPoll poll;
  ASSERT_TRUE(poll.Add(*es1));
  ASSERT_TRUE(poll.Add(*es2));
  ASSERT_TRUE(poll.Add(*es3));

  // Drain all initial data.
  orb_test_s tmp{};
  orb_subscriber_copy(&sub1, &tmp);
  orb_subscriber_copy(&sub2, &tmp);
  orb_subscriber_copy(&sub3, &tmp);
  { uevent::EventSource *r[3] = {}; poll.Wait(r, 3, 0); }

  // Publish: all three subscribers receive the update.
  orb_test_s msg{};
  msg.val = 111;
  ASSERT_EQ(orb_publisher_publish(&pub, &msg), ORB_OK);

  // Consume data on sources 2 and 3 so only source 1 remains ready.
  orb_subscriber_copy(&sub2, &tmp);
  orb_subscriber_copy(&sub3, &tmp);

  // Wait should return only 1 (source 1).
  uevent::EventSource *ready[3] = {nullptr, nullptr, nullptr};
  EXPECT_EQ(poll.Wait(ready, 3, 0), 1);
  EXPECT_EQ(ready[0], es1);

  // Consume source 1.
  orb_subscriber_copy(&sub1, &tmp);

  // Second Wait should return 0 (all consumed).
  EXPECT_EQ(poll.Wait(ready, 3, 0), 0);

  ASSERT_TRUE(poll.Remove(*es1));
  ASSERT_TRUE(poll.Remove(*es2));
  ASSERT_TRUE(poll.Remove(*es3));
  uorb_subscriber_destroy_source(&src1);
  uorb_subscriber_destroy_source(&src2);
  uorb_subscriber_destroy_source(&src3);
  EXPECT_EQ(orb_subscriber_destroy(&sub1), ORB_OK);
  EXPECT_EQ(orb_subscriber_destroy(&sub2), ORB_OK);
  EXPECT_EQ(orb_subscriber_destroy(&sub3), ORB_OK);
  EXPECT_EQ(orb_publisher_destroy(&pub), ORB_OK);
}

// ---- ManySourcesAllReady ----
TEST(EventPollInternalTest, ManySourcesAllReady) {
  orb_publisher_t pub = ORB_PUBLISHER_INITIALIZER;
  EXPECT_EQ(orb_publisher_create(&pub, ORB_ID(orb_test)), ORB_OK);
  ASSERT_NE(pub._handle, nullptr);

  constexpr int kN = 30;
  std::vector<orb_subscriber_t> subs(kN);
  std::vector<uevent_source_t> srcs(kN);
  std::vector<uevent::EventSource *> sources(kN, nullptr);

  for (int i = 0; i < kN; ++i) {
    EXPECT_EQ(orb_subscriber_create(&subs[i], ORB_ID(orb_test)), ORB_OK);
    ASSERT_NE(subs[i]._handle, nullptr);
    ASSERT_EQ(uorb_subscriber_create_source(&srcs[i], &subs[i]), 0);
    sources[i] = reinterpret_cast<uevent::EventSource *>(srcs[i]._handle);
  }

  uevent::EventPoll poll;
  for (int i = 0; i < kN; ++i) {
    ASSERT_TRUE(poll.Add(*sources[i]));
  }

  // Drain initial data on all subscriptions.
  orb_test_s tmp{};
  for (int i = 0; i < kN; ++i) {
    orb_subscriber_copy(&subs[i], &tmp);
  }
  { uevent::EventSource *r[30] = {}; poll.Wait(r, 30, 0); }

  // Publish once: all 30 subscriptions receive the update.
  orb_test_s msg{};
  msg.val = 999;
  ASSERT_EQ(orb_publisher_publish(&pub, &msg), ORB_OK);

  // All 30 should be ready.
  uevent::EventSource *ready[30] = {};
  EXPECT_EQ(poll.Wait(ready, 30, 100), 30);

  // Consume all 30.
  for (int i = 0; i < kN; ++i) {
    orb_subscriber_copy(&subs[i], &tmp);
  }

  // Second Wait should return 0 (all consumed).
  EXPECT_EQ(poll.Wait(ready, 30, 0), 0);

  for (int i = 0; i < kN; ++i) {
    ASSERT_TRUE(poll.Remove(*sources[i]));
  }
  for (int i = 0; i < kN; ++i) {
    uorb_subscriber_destroy_source(&srcs[i]);
    EXPECT_EQ(orb_subscriber_destroy(&subs[i]), ORB_OK);
  }
  EXPECT_EQ(orb_publisher_destroy(&pub), ORB_OK);
}

// ---- RemoveWhileWaitBlocked ----
TEST(EventPollInternalTest, RemoveWhileWaitBlocked) {
  orb_subscriber_t sub = ORB_SUBSCRIBER_INITIALIZER;
  EXPECT_EQ(orb_subscriber_create(&sub, ORB_ID(orb_test)), ORB_OK);
  ASSERT_NE(sub._handle, nullptr);

  uevent::EventPoll poll;
  uevent_source_t src = UEVENT_SOURCE_INITIALIZER;
  ASSERT_EQ(uorb_subscriber_create_source(&src, &sub), 0);
  uevent::EventSource *es = reinterpret_cast<uevent::EventSource *>(src._handle);
  ASSERT_TRUE(poll.Add(*es));
  drain_initial(poll, es, &sub);

  std::atomic<int> wait_result{99};
  std::thread waiter([&]() {
    uevent::EventSource *ready[1] = {nullptr};
    wait_result = poll.Wait(ready, 1, -1);
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  ASSERT_TRUE(poll.Remove(*es));
  // Remove does not notify the condition variable; call Stop to wake
  // the blocked Wait thread.
  poll.Stop();

  waiter.join();
  EXPECT_EQ(wait_result.load(), -1);

  uorb_subscriber_destroy_source(&src);
  EXPECT_EQ(orb_subscriber_destroy(&sub), ORB_OK);
}

// ---- StopCalledTwice ----
TEST(EventPollInternalTest, StopCalledTwice) {
  orb_subscriber_t sub = ORB_SUBSCRIBER_INITIALIZER;
  EXPECT_EQ(orb_subscriber_create(&sub, ORB_ID(orb_test)), ORB_OK);
  ASSERT_NE(sub._handle, nullptr);

  uevent::EventPoll poll;
  uevent_source_t src = UEVENT_SOURCE_INITIALIZER;
  ASSERT_EQ(uorb_subscriber_create_source(&src, &sub), 0);
  uevent::EventSource *es = reinterpret_cast<uevent::EventSource *>(src._handle);
  ASSERT_TRUE(poll.Add(*es));
  drain_initial(poll, es, &sub);

  // Calling Stop twice should be safe.
  poll.Stop();
  poll.Stop();

  // First Wait sees stop_ == true, returns -1, clears stop_.
  uevent::EventSource *ready[1] = {nullptr};
  EXPECT_EQ(poll.Wait(ready, 1, 0), -1);

  // Second Wait: stop_ is now false, no ready data -> returns 0.
  EXPECT_EQ(poll.Wait(ready, 1, 0), 0);

  ASSERT_TRUE(poll.Remove(*es));
  uorb_subscriber_destroy_source(&src);
  EXPECT_EQ(orb_subscriber_destroy(&sub), ORB_OK);
}

// ---- PerEventTimeoutWithConcurrentNotify ----
TEST(EventPollInternalTest, PerEventTimeoutWithConcurrentNotify) {
  orb_publisher_t pub = ORB_PUBLISHER_INITIALIZER;
  EXPECT_EQ(orb_publisher_create(&pub, ORB_ID(orb_test)), ORB_OK);
  ASSERT_NE(pub._handle, nullptr);

  orb_subscriber_t sub = ORB_SUBSCRIBER_INITIALIZER;
  EXPECT_EQ(orb_subscriber_create(&sub, ORB_ID(orb_test)), ORB_OK);
  ASSERT_NE(sub._handle, nullptr);

  uevent::EventPoll poll;
  uevent_source_t src = UEVENT_SOURCE_INITIALIZER;
  ASSERT_EQ(uorb_subscriber_create_source(&src, &sub), 0);
  uevent::EventSource *es = reinterpret_cast<uevent::EventSource *>(src._handle);
  ASSERT_TRUE(poll.Add(*es));
  drain_initial(poll, es, &sub);

  ASSERT_TRUE(poll.Remove(*es));
  // Re-add with a 50 ms per-event timeout.
  ASSERT_TRUE(poll.Add(*es, 50));

  auto t0 = std::chrono::steady_clock::now();

  // Thread publishes after 20 ms, which notifies the poll.
  std::thread notifier([&]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    orb_test_s msg{};
    msg.val = 777;
    orb_publisher_publish(&pub, &msg);
  });

  uevent::EventSource *ready[1] = {nullptr};
  int n = poll.Wait(ready, 1, -1);
  auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - t0)
                        .count();

  notifier.join();

  ASSERT_EQ(n, 1);
  EXPECT_EQ(ready[0], es);
  // Should return well before the 50 ms per-event timeout fires.
  EXPECT_LT(elapsed_ms, 40);

  ASSERT_TRUE(poll.Remove(*es));
  uorb_subscriber_destroy_source(&src);
  EXPECT_EQ(orb_subscriber_destroy(&sub), ORB_OK);
  EXPECT_EQ(orb_publisher_destroy(&pub), ORB_OK);
}

}  // namespace uORBTest
