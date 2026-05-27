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

