/****************************************************************************
 *
 * Unit tests for uorb::EventLoop.
 *
 ****************************************************************************/

#include <gtest/gtest.h>
#include <uorb/event_loop.h>
#include <uorb/publication.h>
#include <uorb/subscription.h>
#include <uorb/topics/orb_test.h>
#include <uorb/topics/orb_test_medium.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <thread>

namespace uORBTest {

// Upper bound on how many pre-existing updates we try to drain from a freshly
// created subscription before running a test. Other tests in the same binary
// may have already published to the shared topics, so a new subscription can
// see those as "updated" on first poll. The number of prior publications is
// small and bounded, so a small fixed cap is plenty.
static constexpr int kMaxDrainIterations = 4;

// RunOnce() on an EventLoop that has no entries should return 0 immediately,
// regardless of the timeout value.
TEST(EventLoopTest, RunOnceEmptyReturnsZero) {
  uorb::EventLoop loop;
  ASSERT_TRUE(loop);
  EXPECT_EQ(loop.RunOnce(0), 0);
  EXPECT_EQ(loop.RunOnce(10), 0);
}

// Run() should return false immediately when there are no registered entries
// (nothing to wait for).
TEST(EventLoopTest, RunExitsWhenEmpty) {
  uorb::EventLoop loop;
  ASSERT_TRUE(loop);
  EXPECT_FALSE(loop.Run());
}

// Subscribe() creates a subscription owned by the loop and dispatches the
// callback when data is published.
TEST(EventLoopTest, SubscribeReceivesPublishedData) {
  uorb::EventLoop loop;
  ASSERT_TRUE(loop);

  std::atomic<int> received_val{-1};
  std::atomic<int> call_count{0};

  ASSERT_TRUE(loop.Subscribe<uorb::msg::orb_test>([&](const orb_test_s &msg) {
    received_val = msg.val;
    ++call_count;
  }));

  // Drain any pre-existing updates from previous tests that published to
  // this topic.
  for (int i = 0; i < kMaxDrainIterations && loop.RunOnce(0) > 0; ++i) {
  }
  call_count = 0;
  received_val = -1;

  uorb::PublicationData<uorb::msg::orb_test> pub;
  pub.data().val = 42;
  ASSERT_TRUE(pub.Publish());

  // One message is pending; RunOnce should deliver it.
  const int n = loop.RunOnce(1000);
  EXPECT_EQ(n, 1);
  EXPECT_EQ(call_count.load(), 1);
  EXPECT_EQ(received_val.load(), 42);
}

// AddSubscription() binds a callback to a user-owned subscription and
// RemoveSubscription() removes it.
TEST(EventLoopTest, AddAndRemoveExternalSubscription) {
  uorb::EventLoop loop;
  ASSERT_TRUE(loop);

  uorb::SubscriptionData<uorb::msg::orb_test_medium> sub;

  std::atomic<int> received_val{-1};
  ASSERT_TRUE(loop.AddSubscription(sub, [&](const orb_test_medium_s &msg) {
    received_val = msg.val;
  }));

  // Drain any pre-existing updates.
  for (int i = 0; i < kMaxDrainIterations && loop.RunOnce(0) > 0; ++i) {
  }
  received_val = -1;

  // Adding the same subscription twice must be refused.
  EXPECT_FALSE(loop.AddSubscription(
      sub, [](const orb_test_medium_s &) {}));

  uorb::PublicationData<uorb::msg::orb_test_medium> pub;
  pub.data().val = 123;
  ASSERT_TRUE(pub.Publish());

  EXPECT_EQ(loop.RunOnce(1000), 1);
  EXPECT_EQ(received_val.load(), 123);

  // After removal, published data should no longer trigger the callback.
  EXPECT_TRUE(loop.RemoveSubscription(sub));
  // Second removal must return false (already removed).
  EXPECT_FALSE(loop.RemoveSubscription(sub));

  received_val = -1;
  pub.data().val = 456;
  ASSERT_TRUE(pub.Publish());

  // No entries left; RunOnce returns 0 without waiting.
  EXPECT_EQ(loop.RunOnce(50), 0);
  EXPECT_EQ(received_val.load(), -1);
}

// RunOnce() with a timeout and no pending data returns 0.
TEST(EventLoopTest, RunOnceTimeoutNoData) {
  uorb::EventLoop loop;
  ASSERT_TRUE(loop);

  ASSERT_TRUE(loop.Subscribe<uorb::msg::orb_test>([](const orb_test_s &) {}));

  // Other test cases may already have published to this topic. A brand new
  // subscription sees the most recent data as "updated", so drain any pending
  // events first with a non-blocking poll before measuring the timeout.
  for (int i = 0; i < kMaxDrainIterations && loop.RunOnce(0) > 0; ++i) {
  }

  const auto start = std::chrono::steady_clock::now();
  const int n = loop.RunOnce(50);
  const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                           std::chrono::steady_clock::now() - start)
                           .count();
  EXPECT_EQ(n, 0);
  EXPECT_GE(elapsed, 40);  // allow some scheduling slack
}

// Quit() is thread-safe and causes Run() to return true, and RunOnce()
// to return -1 thereafter ("sticky" semantics).
TEST(EventLoopTest, QuitStopsLoopFromOtherThread) {
  uorb::EventLoop loop;
  ASSERT_TRUE(loop);

  // Need at least one entry, otherwise Run() exits immediately with false.
  ASSERT_TRUE(loop.Subscribe<uorb::msg::orb_test>([](const orb_test_s &) {}));

  std::thread quitter([&] {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    loop.Quit();
  });

  EXPECT_TRUE(loop.Run());
  quitter.join();

  // Sticky: subsequent RunOnce must return -1 immediately.
  EXPECT_EQ(loop.RunOnce(0), -1);
  EXPECT_EQ(loop.RunOnce(100), -1);
}

// Quit() before Run() starts must still cause Run() to return true (sticky).
TEST(EventLoopTest, QuitBeforeLoopIsSticky) {
  uorb::EventLoop loop;
  ASSERT_TRUE(loop);
  ASSERT_TRUE(loop.Subscribe<uorb::msg::orb_test>([](const orb_test_s &) {}));

  loop.Quit();
  EXPECT_TRUE(loop.Run());
  EXPECT_EQ(loop.RunOnce(0), -1);
}

// Multiple subscribers on different topics each get their own callback.
TEST(EventLoopTest, MultipleSubscriptionsDispatchIndependently) {
  uorb::EventLoop loop;
  ASSERT_TRUE(loop);

  std::atomic<int> a_calls{0};
  std::atomic<int> b_calls{0};

  ASSERT_TRUE(loop.Subscribe<uorb::msg::orb_test>(
      [&](const orb_test_s &) { ++a_calls; }));
  ASSERT_TRUE(loop.Subscribe<uorb::msg::orb_test_medium>(
      [&](const orb_test_medium_s &) { ++b_calls; }));

  // Drain any pre-existing updates, then reset counters.
  for (int i = 0; i < kMaxDrainIterations && loop.RunOnce(0) > 0; ++i) {
  }
  a_calls = 0;
  b_calls = 0;

  uorb::PublicationData<uorb::msg::orb_test> pub_a;
  uorb::PublicationData<uorb::msg::orb_test_medium> pub_b;

  pub_a.data().val = 1;
  ASSERT_TRUE(pub_a.Publish());
  pub_b.data().val = 2;
  ASSERT_TRUE(pub_b.Publish());

  // Drain up to two events. A single RunOnce may return both, or we may need
  // two calls depending on scheduler behavior.
  int total = 0;
  for (int i = 0; i < 4 && total < 2; ++i) {
    const int n = loop.RunOnce(500);
    ASSERT_GE(n, 0);
    total += n;
  }
  EXPECT_EQ(a_calls.load(), 1);
  EXPECT_EQ(b_calls.load(), 1);
}

// The EventLoop destructor must detach and destroy all owned subscriptions,
// and detach (but not destroy) user-owned subscriptions. If this test leaks
// or double-frees, sanitizers / later tests will catch it.
TEST(EventLoopTest, DestructorCleansUpOwnedAndExternal) {
  uorb::SubscriptionData<uorb::msg::orb_test> external_sub;
  {
    uorb::EventLoop loop;
    ASSERT_TRUE(loop);
    ASSERT_TRUE(loop.Subscribe<uorb::msg::orb_test_medium>(
        [](const orb_test_medium_s &) {}));
    ASSERT_TRUE(
        loop.AddSubscription(external_sub, [](const orb_test_s &) {}));
    // Intentionally do not unregister: destructor should clean up.
  }
  // The external subscription must still be usable after the loop is gone.
  EXPECT_NE(external_sub.handle(), nullptr);
}

TEST(EventLoopTest, SubscriptionCannotBindToMultipleEventPolls) {
  uorb::SubscriptionData<uorb::msg::orb_test> sub;

  orb_event_poll_t *poll_a = orb_event_poll_create();
  orb_event_poll_t *poll_b = orb_event_poll_create();
  ASSERT_NE(poll_a, nullptr);
  ASSERT_NE(poll_b, nullptr);

  ASSERT_TRUE(orb_event_poll_add(poll_a, sub.handle()));

  errno = 0;
  EXPECT_FALSE(orb_event_poll_add(poll_b, sub.handle()));
  EXPECT_EQ(errno, EBUSY);

  EXPECT_TRUE(orb_event_poll_remove(poll_a, sub.handle()));
  EXPECT_TRUE(orb_event_poll_add(poll_b, sub.handle()));
  EXPECT_TRUE(orb_event_poll_remove(poll_b, sub.handle()));

  EXPECT_TRUE(orb_event_poll_destroy(&poll_a));
  EXPECT_TRUE(orb_event_poll_destroy(&poll_b));
}

}  // namespace uORBTest
