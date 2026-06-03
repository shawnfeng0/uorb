/****************************************************************************
 *
 * Internal tests for uorb::EventPoll behavior.
 *
 ****************************************************************************/

#include <gtest/gtest.h>
#include <uorb/topics/orb_test.h>
#include <uorb/uorb.h>

#include <atomic>
#include <chrono>
#include <thread>

#include "event_poll.h"
#include "receiver_local.h"

namespace uORBTest {

namespace {

uorb::ReceiverLocal *as_receiver(orb_subscription_t *sub) {
  return reinterpret_cast<uorb::ReceiverLocal *>(sub);
}

void drain_nonblocking(uorb::EventPoll &poll, orb_subscription_t *sub,
                       uorb::ReceiverLocal *receiver) {
  uorb::ReceiverLocal *ready[1] = {nullptr};
  orb_test_s msg{};

  for (int i = 0; i < 8; ++i) {
    const int n = poll.Wait(ready, 1, 0);
    if (n <= 0) {
      break;
    }
    if (ready[0] == receiver) {
      orb_copy(sub, &msg);
    }
  }
}

}  // namespace

TEST(EventPollInternalTest, WaitZeroReturnsZeroWhenNoReadyData) {
  orb_subscription_t *sub = orb_create_subscription(ORB_ID(orb_test));
  ASSERT_NE(sub, nullptr);

  uorb::EventPoll poll;
  uorb::ReceiverLocal *receiver = as_receiver(sub);
  ASSERT_TRUE(poll.AddReceiver(*receiver));

  drain_nonblocking(poll, sub, receiver);

  uorb::ReceiverLocal *ready[1] = {nullptr};
  EXPECT_EQ(poll.Wait(ready, 1, 0), 0);

  EXPECT_TRUE(poll.RemoveReceiver(*receiver));
  EXPECT_TRUE(orb_destroy_subscription(&sub));
}

TEST(EventPollInternalTest, WaitZeroReturnsReadyReceiverWhenDataArrives) {
  orb_publication_t *pub = orb_create_publication(ORB_ID(orb_test));
  ASSERT_NE(pub, nullptr);

  orb_subscription_t *sub = orb_create_subscription(ORB_ID(orb_test));
  ASSERT_NE(sub, nullptr);

  uorb::EventPoll poll;
  uorb::ReceiverLocal *receiver = as_receiver(sub);
  ASSERT_TRUE(poll.AddReceiver(*receiver));

  drain_nonblocking(poll, sub, receiver);

  orb_test_s msg{};
  msg.val = 321;
  ASSERT_TRUE(orb_publish(pub, &msg));

  uorb::ReceiverLocal *ready[1] = {nullptr};
  const int n = poll.Wait(ready, 1, 0);
  ASSERT_EQ(n, 1);
  EXPECT_EQ(ready[0], receiver);

  EXPECT_TRUE(orb_copy(sub, &msg));
  EXPECT_EQ(msg.val, 321);

  EXPECT_TRUE(poll.RemoveReceiver(*receiver));
  EXPECT_TRUE(orb_destroy_subscription(&sub));
  EXPECT_TRUE(orb_destroy_publication(&pub));
}

TEST(EventPollInternalTest, WaitReturnsMultipleReadyReceivers) {
  orb_publication_t *pub = orb_create_publication(ORB_ID(orb_test));
  ASSERT_NE(pub, nullptr);

  orb_subscription_t *first_sub = orb_create_subscription(ORB_ID(orb_test));
  ASSERT_NE(first_sub, nullptr);
  orb_subscription_t *second_sub = orb_create_subscription(ORB_ID(orb_test));
  ASSERT_NE(second_sub, nullptr);

  uorb::EventPoll poll;
  uorb::ReceiverLocal *first_receiver = as_receiver(first_sub);
  uorb::ReceiverLocal *second_receiver = as_receiver(second_sub);
  ASSERT_TRUE(poll.AddReceiver(*first_receiver));
  ASSERT_TRUE(poll.AddReceiver(*second_receiver));

  drain_nonblocking(poll, first_sub, first_receiver);
  drain_nonblocking(poll, second_sub, second_receiver);

  orb_test_s msg{};
  msg.val = 777;
  ASSERT_TRUE(orb_publish(pub, &msg));

  uorb::ReceiverLocal *ready[2] = {nullptr, nullptr};
  const int n = poll.Wait(ready, 2, 0);
  ASSERT_EQ(n, 2);
  EXPECT_TRUE((ready[0] == first_receiver && ready[1] == second_receiver) ||
              (ready[0] == second_receiver && ready[1] == first_receiver));

  EXPECT_TRUE(poll.RemoveReceiver(*first_receiver));
  EXPECT_TRUE(poll.RemoveReceiver(*second_receiver));
  EXPECT_TRUE(orb_destroy_subscription(&first_sub));
  EXPECT_TRUE(orb_destroy_subscription(&second_sub));
  EXPECT_TRUE(orb_destroy_publication(&pub));
}

TEST(EventPollInternalTest, WaitTruncatesReadyOutputToCapacity) {
  orb_publication_t *pub = orb_create_publication(ORB_ID(orb_test));
  ASSERT_NE(pub, nullptr);

  orb_subscription_t *first_sub = orb_create_subscription(ORB_ID(orb_test));
  ASSERT_NE(first_sub, nullptr);
  orb_subscription_t *second_sub = orb_create_subscription(ORB_ID(orb_test));
  ASSERT_NE(second_sub, nullptr);

  uorb::EventPoll poll;
  uorb::ReceiverLocal *first_receiver = as_receiver(first_sub);
  uorb::ReceiverLocal *second_receiver = as_receiver(second_sub);
  ASSERT_TRUE(poll.AddReceiver(*first_receiver));
  ASSERT_TRUE(poll.AddReceiver(*second_receiver));

  drain_nonblocking(poll, first_sub, first_receiver);
  drain_nonblocking(poll, second_sub, second_receiver);

  orb_test_s msg{};
  msg.val = 888;
  ASSERT_TRUE(orb_publish(pub, &msg));

  uorb::ReceiverLocal *ready[1] = {nullptr};
  ASSERT_EQ(poll.Wait(ready, 1, 0), 1);
  ASSERT_TRUE(ready[0] == first_receiver || ready[0] == second_receiver);
  EXPECT_TRUE(orb_copy(ready[0] == first_receiver ? first_sub : second_sub, &msg));

  uorb::ReceiverLocal *remaining_ready[1] = {nullptr};
  ASSERT_EQ(poll.Wait(remaining_ready, 1, 0), 1);
  EXPECT_NE(remaining_ready[0], ready[0]);
  EXPECT_TRUE(orb_copy(remaining_ready[0] == first_receiver ? first_sub : second_sub, &msg));

  EXPECT_TRUE(poll.RemoveReceiver(*first_receiver));
  EXPECT_TRUE(poll.RemoveReceiver(*second_receiver));
  EXPECT_TRUE(orb_destroy_subscription(&first_sub));
  EXPECT_TRUE(orb_destroy_subscription(&second_sub));
  EXPECT_TRUE(orb_destroy_publication(&pub));
}

TEST(EventPollInternalTest, WaitAfterStopReturnsMinusOneImmediately) {
  orb_subscription_t *sub = orb_create_subscription(ORB_ID(orb_test));
  ASSERT_NE(sub, nullptr);

  uorb::EventPoll poll;
  uorb::ReceiverLocal *receiver = as_receiver(sub);
  ASSERT_TRUE(poll.AddReceiver(*receiver));

  poll.Stop();

  uorb::ReceiverLocal *ready[1] = {nullptr};
  EXPECT_EQ(poll.Wait(ready, 1, 0), -1);
  EXPECT_EQ(poll.Wait(ready, 1, 10), -1);

  EXPECT_TRUE(poll.RemoveReceiver(*receiver));
  EXPECT_TRUE(orb_destroy_subscription(&sub));
}

TEST(EventPollInternalTest, WaitInterruptedByStopReturnsMinusOne) {
  orb_subscription_t *sub = orb_create_subscription(ORB_ID(orb_test));
  ASSERT_NE(sub, nullptr);

  uorb::EventPoll poll;
  uorb::ReceiverLocal *receiver = as_receiver(sub);
  ASSERT_TRUE(poll.AddReceiver(*receiver));
  drain_nonblocking(poll, sub, receiver);

  std::atomic<int> wait_result{0};
  std::thread waiter([&]() {
    uorb::ReceiverLocal *ready[1] = {nullptr};
    wait_result = poll.Wait(ready, 1, -1);
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  poll.Stop();

  waiter.join();
  EXPECT_EQ(wait_result.load(), -1);

  EXPECT_TRUE(poll.RemoveReceiver(*receiver));
  EXPECT_TRUE(orb_destroy_subscription(&sub));
}

TEST(EventPollInternalTest, AddReceiverIsIdempotentForSamePoll) {
  orb_subscription_t *sub = orb_create_subscription(ORB_ID(orb_test));
  ASSERT_NE(sub, nullptr);

  uorb::EventPoll poll;
  uorb::ReceiverLocal *receiver = as_receiver(sub);

  EXPECT_TRUE(poll.AddReceiver(*receiver));
  EXPECT_TRUE(poll.AddReceiver(*receiver));
  EXPECT_TRUE(poll.RemoveReceiver(*receiver));
  EXPECT_FALSE(poll.RemoveReceiver(*receiver));
  EXPECT_TRUE(orb_destroy_subscription(&sub));
}

TEST(EventPollInternalTest, RemoveReceiverRejectsReceiverBoundToAnotherPoll) {
  orb_subscription_t *sub = orb_create_subscription(ORB_ID(orb_test));
  ASSERT_NE(sub, nullptr);

  uorb::EventPoll owner_poll;
  uorb::EventPoll other_poll;
  uorb::ReceiverLocal *receiver = as_receiver(sub);

  ASSERT_TRUE(owner_poll.AddReceiver(*receiver));
  EXPECT_FALSE(other_poll.RemoveReceiver(*receiver));
  EXPECT_EQ(errno, EBUSY);
  EXPECT_TRUE(owner_poll.RemoveReceiver(*receiver));
  EXPECT_TRUE(orb_destroy_subscription(&sub));
}

TEST(EventPollInternalTest, DestroySubscriptionRejectsBoundReceiver) {
  orb_subscription_t *sub = orb_create_subscription(ORB_ID(orb_test));
  ASSERT_NE(sub, nullptr);

  uorb::EventPoll poll;
  uorb::ReceiverLocal *receiver = as_receiver(sub);

  ASSERT_TRUE(poll.AddReceiver(*receiver));
  EXPECT_FALSE(orb_destroy_subscription(&sub));
  EXPECT_EQ(errno, EBUSY);
  ASSERT_NE(sub, nullptr);
  EXPECT_TRUE(poll.RemoveReceiver(*receiver));
  EXPECT_TRUE(orb_destroy_subscription(&sub));
}

TEST(EventPollInternalTest, WaitAllowsZeroMaxReadyWithoutWritingReadyArray) {
  orb_publication_t *pub = orb_create_publication(ORB_ID(orb_test));
  ASSERT_NE(pub, nullptr);

  orb_subscription_t *sub = orb_create_subscription(ORB_ID(orb_test));
  ASSERT_NE(sub, nullptr);

  uorb::EventPoll poll;
  uorb::ReceiverLocal *receiver = as_receiver(sub);
  ASSERT_TRUE(poll.AddReceiver(*receiver));

  drain_nonblocking(poll, sub, receiver);

  orb_test_s msg{};
  msg.val = 654;
  ASSERT_TRUE(orb_publish(pub, &msg));

  uorb::ReceiverLocal *ready[1] = {reinterpret_cast<uorb::ReceiverLocal *>(0x1)};
  EXPECT_EQ(poll.Wait(ready, 0, 0), 1);
  EXPECT_EQ(ready[0], reinterpret_cast<uorb::ReceiverLocal *>(0x1));

  EXPECT_TRUE(orb_copy(sub, &msg));
  EXPECT_EQ(msg.val, 654);

  EXPECT_TRUE(poll.RemoveReceiver(*receiver));
  EXPECT_TRUE(orb_destroy_subscription(&sub));
  EXPECT_TRUE(orb_destroy_publication(&pub));
}

TEST(EventPollInternalTest, WaitRejectsNullReadyArray) {
  uorb::EventPoll poll;

  errno = 0;
  EXPECT_EQ(poll.Wait(nullptr, 1, 0), -1);
  EXPECT_EQ(errno, EINVAL);
}

TEST(EventPollInternalTest, WaitRejectsNegativeMaxReady) {
  uorb::EventPoll poll;
  uorb::ReceiverLocal *ready[1] = {nullptr};

  errno = 0;
  EXPECT_EQ(poll.Wait(ready, -1, 0), -1);
  EXPECT_EQ(errno, EINVAL);
}

TEST(EventPollInternalTest, AddRemovePressureWithActivePublisher) {
  orb_publication_t *pub = orb_create_publication(ORB_ID(orb_test));
  ASSERT_NE(pub, nullptr);

  orb_subscription_t *sub = orb_create_subscription(ORB_ID(orb_test));
  ASSERT_NE(sub, nullptr);
  uorb::ReceiverLocal *receiver = as_receiver(sub);

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

  uorb::EventPoll poll;
  uorb::ReceiverLocal *ready[1] = {nullptr};
  orb_test_s msg{};

  for (int i = 0; i < 200; ++i) {
    ASSERT_TRUE(poll.AddReceiver(*receiver));
    const int n = poll.Wait(ready, 1, 20);
    ASSERT_GE(n, 0);
    if (n > 0 && ready[0] == receiver) {
      ASSERT_TRUE(orb_copy(sub, &msg));
    }
    ASSERT_TRUE(poll.RemoveReceiver(*receiver));
  }

  running = false;
  publisher.join();

  EXPECT_TRUE(orb_destroy_subscription(&sub));
  EXPECT_TRUE(orb_destroy_publication(&pub));
}

}  // namespace uORBTest

