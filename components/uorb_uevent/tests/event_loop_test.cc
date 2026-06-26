/****************************************************************************
 *
 * Unit tests for uorb::EventLoop.
 *
 ****************************************************************************/

#include <gtest/gtest.h>
#include <uevent/uevent.h>
#include <uorb/publication.h>
#include <uorb_uevent/uorb_uevent.h>
#include <uorb/subscription.h>
#include <uorb/topics/orb_test.h>
#include <uorb/topics/orb_test_large.h>
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

void DrainPendingEvents(uorb::EventLoop &loop) {
  for (int i = 0; i < kMaxDrainIterations && loop.RunOnce(0) > 0; ++i) {
  }
}

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
  DrainPendingEvents(loop);
  call_count = 0;
  received_val = -1;

  uorb::PublicationData<uorb::msg::orb_test> pub;
  pub.data().val = 42;
  ASSERT_EQ(pub.Publish(), ORB_OK);

  // One message is pending; RunOnce should deliver it.
  const int n = loop.RunOnce(1000);
  EXPECT_EQ(n, 1);
  EXPECT_EQ(call_count.load(), 1);
  EXPECT_EQ(received_val.load(), 42);
}

// RunOnce() with a timeout and no pending data returns 0.
TEST(EventLoopTest, RunOnceTimeoutNoData) {
  uorb::EventLoop loop;
  ASSERT_TRUE(loop);

  ASSERT_TRUE(loop.Subscribe<uorb::msg::orb_test>([](const orb_test_s &) {}));

  // Other test cases may already have published to this topic. A brand new
  // subscription sees the most recent data as "updated", so drain any pending
  // events first with a non-blocking poll before measuring the timeout.
  DrainPendingEvents(loop);

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

// After Run() returns due to Quit(), calling Run() again resets quit_requested_
// and the loop can be restarted.
TEST(EventLoopTest, RunCanBeRestartedAfterQuit) {
  uorb::EventLoop loop;
  ASSERT_TRUE(loop);
  ASSERT_TRUE(loop.Subscribe<uorb::msg::orb_test>([](const orb_test_s &) {}));

  // First cycle: start Run, quit from another thread.
  std::thread quitter1([&] {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    loop.Quit();
  });
  EXPECT_TRUE(loop.Run());
  quitter1.join();

  // RunOnce is still -1 (quit_requested_ is sticky until next Run()).
  EXPECT_EQ(loop.RunOnce(0), -1);

  // Second cycle: Run() resets quit_requested_, so the loop restarts.
  std::thread quitter2([&] {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    loop.Quit();
  });
  EXPECT_TRUE(loop.Run());
  quitter2.join();
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
  DrainPendingEvents(loop);
  a_calls = 0;
  b_calls = 0;

  uorb::PublicationData<uorb::msg::orb_test> pub_a;
  uorb::PublicationData<uorb::msg::orb_test_medium> pub_b;

  pub_a.data().val = 1;
  ASSERT_EQ(pub_a.Publish(), ORB_OK);
  pub_b.data().val = 2;
  ASSERT_EQ(pub_b.Publish(), ORB_OK);

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

// The EventLoop destructor must destroy all owned subscriptions.
// If this test leaks or crashes, sanitizers / later tests will catch it.
TEST(EventLoopTest, DestructorCleansUpOwnedSubscriptions) {
  {
    uorb::EventLoop loop;
    ASSERT_TRUE(loop);
    ASSERT_TRUE(loop.Subscribe<uorb::msg::orb_test_medium>(
        [](const orb_test_medium_s &) {}));
    ASSERT_TRUE(loop.Subscribe<uorb::msg::orb_test>(
        [](const orb_test_s &) {}));
    // Intentionally do not unregister: destructor should clean up.
  }
  // If we reach here without crashing or leaking, the destructor worked.
}

TEST(EventLoopTest, CallbackCanQuitLoop) {
  uorb::EventLoop loop;
  ASSERT_TRUE(loop);

  std::atomic<int> call_count{0};
  ASSERT_TRUE(loop.Subscribe<uorb::msg::orb_test_large>(
      [&](const orb_test_large_s &) {
        ++call_count;
        loop.Quit();
      }));

  // Drain any stale data from previous tests
  DrainPendingEvents(loop);
  call_count = 0;

  uorb::PublicationData<uorb::msg::orb_test_large> pub;
  pub.data().val = 808;
  ASSERT_EQ(pub.Publish(), ORB_OK);

  EXPECT_EQ(loop.RunOnce(1000), 1);
  EXPECT_EQ(call_count.load(), 1);
  EXPECT_EQ(loop.RunOnce(0), -1);
}

TEST(EventLoopTest, SubscriptionCannotBindToMultipleEventPolls) {
  uorb::SubscriptionData<uorb::msg::orb_test> sub;

  uevent_t base_a = UEVENT_INITIALIZER;
  ASSERT_EQ(uevent_create(&base_a), 0);
  ASSERT_NE(base_a._handle, nullptr);

  uevent_t base_b = UEVENT_INITIALIZER;
  ASSERT_EQ(uevent_create(&base_b), 0);
  ASSERT_NE(base_b._handle, nullptr);

  uevent_source_t source = UEVENT_SOURCE_INITIALIZER;
  ASSERT_EQ(uorb_subscriber_create_source(&source, sub.handle()), 0);
  ASSERT_NE(source._handle, nullptr);

  ASSERT_EQ(uevent_add(&base_a, &source, 0), 0);

  errno = 0;
  EXPECT_EQ(uevent_add(&base_b, &source, 0), -1);
  EXPECT_EQ(errno, EBUSY);

  EXPECT_EQ(uevent_remove(&base_a, &source), 0);
  EXPECT_EQ(uevent_add(&base_b, &source, 0), 0);
  EXPECT_EQ(uevent_remove(&base_b, &source), 0);

  uorb_subscriber_destroy_source(&source);
  uevent_destroy(&base_a);
  uevent_destroy(&base_b);
}

// RunOnce uses a fixed-size ready[32] array. When more than 32 subscriptions
// are ready, a single call can dispatch at most 32; subsequent calls pick up
// the remainder.
TEST(EventLoopTest, RunOnceExceedsReadyCapacity) {
  uorb::EventLoop loop;
  ASSERT_TRUE(loop);

  constexpr int kNumSubs = 35;
  std::atomic<int> call_count{0};

  // Subscribe 35 times to the same topic. Each Subscribe() creates an
  // independent subscriber with its own event source.
  for (int i = 0; i < kNumSubs; ++i) {
    ASSERT_TRUE(loop.Subscribe<uorb::msg::orb_test>(
        [&](const orb_test_s &) { ++call_count; }));
  }

  DrainPendingEvents(loop);
  call_count = 0;

  uorb::PublicationData<uorb::msg::orb_test> pub;
  pub.data().val = 1;
  ASSERT_EQ(pub.Publish(), ORB_OK);

  // First RunOnce should return at most 32 (limited by ready[32]).
  int first = loop.RunOnce(100);
  ASSERT_GT(first, 0);
  ASSERT_LE(first, 32);

  // Drain remaining events with non-blocking calls.
  int total = first;
  for (int i = 0; i < 4 && total < kNumSubs; ++i) {
    int n = loop.RunOnce(0);
    if (n <= 0) break;
    total += n;
  }

  EXPECT_EQ(call_count.load(), kNumSubs);
  EXPECT_EQ(total, kNumSubs);
}

// Quit() is idempotent: calling it multiple times must be safe.
TEST(EventLoopTest, QuitCalledMultipleTimes) {
  uorb::EventLoop loop;
  ASSERT_TRUE(loop);
  ASSERT_TRUE(loop.Subscribe<uorb::msg::orb_test>([](const orb_test_s &) {}));

  loop.Quit();
  loop.Quit();
  loop.Quit();

  // Sticky: RunOnce must return -1 immediately.
  EXPECT_EQ(loop.RunOnce(0), -1);
  EXPECT_EQ(loop.RunOnce(100), -1);
}

// Subscribe() does not check quit_requested_, so new entries can be added
// even after Quit(). RunOnce must still return -1 (sticky).
TEST(EventLoopTest, SubscribeAfterQuit) {
  uorb::EventLoop loop;
  ASSERT_TRUE(loop);
  ASSERT_TRUE(loop.Subscribe<uorb::msg::orb_test>([](const orb_test_s &) {}));

  loop.Quit();

  // Subscribe() should still succeed — it doesn't check quit_requested_.
  EXPECT_TRUE(loop.Subscribe<uorb::msg::orb_test>([](const orb_test_s &) {}));

  // RunOnce must return -1 immediately (quit_requested_ is sticky).
  EXPECT_EQ(loop.RunOnce(0), -1);
  EXPECT_EQ(loop.RunOnce(100), -1);
}

// A callback that re-publishes to the same topic. The re-published message
// is delivered on the next RunOnce call.
TEST(EventLoopTest, CallbackPublishesSameTopic) {
  uorb::EventLoop loop;
  ASSERT_TRUE(loop);

  std::atomic<int> call_count{0};
  std::atomic<bool> already_republished{false};

  ASSERT_TRUE(loop.Subscribe<uorb::msg::orb_test>([&](const orb_test_s &) {
    ++call_count;
    if (!already_republished.exchange(true)) {
      uorb::PublicationData<uorb::msg::orb_test> pub;
      pub.data().val = 99;
      pub.Publish();
    }
  }));

  DrainPendingEvents(loop);
  call_count = 0;
  already_republished = false;

  // Publish initial message.
  uorb::PublicationData<uorb::msg::orb_test> pub;
  pub.data().val = 1;
  ASSERT_EQ(pub.Publish(), ORB_OK);

  // First RunOnce dispatches the callback (count=1, re-publishes).
  EXPECT_EQ(loop.RunOnce(1000), 1);
  EXPECT_EQ(call_count.load(), 1);

  // Second RunOnce dispatches the re-published message (count=2).
  EXPECT_EQ(loop.RunOnce(1000), 1);
  EXPECT_EQ(call_count.load(), 2);

  // No more pending data.
  EXPECT_EQ(loop.RunOnce(0), 0);
}

// A callback that re-entrantly calls RunOnce(0) to drain pending events on
// another subscription. This is safe because EventPoll::Wait releases its
// mutex before returning the ready array.
TEST(EventLoopTest, CallbackCallsRunOnce) {
  uorb::EventLoop loop;
  ASSERT_TRUE(loop);

  std::atomic<int> first_calls{0};
  std::atomic<int> second_calls{0};

  ASSERT_TRUE(loop.Subscribe<uorb::msg::orb_test>([&](const orb_test_s &) {
    ++first_calls;
    // Re-entrant call to drain any pending events on the second topic.
    loop.RunOnce(0);
  }));
  ASSERT_TRUE(loop.Subscribe<uorb::msg::orb_test_medium>(
      [&](const orb_test_medium_s &) { ++second_calls; }));

  DrainPendingEvents(loop);
  first_calls = 0;
  second_calls = 0;

  // Publish to both topics.
  uorb::PublicationData<uorb::msg::orb_test> pub_a;
  pub_a.data().val = 1;
  ASSERT_EQ(pub_a.Publish(), ORB_OK);

  uorb::PublicationData<uorb::msg::orb_test_medium> pub_b;
  pub_b.data().val = 2;
  ASSERT_EQ(pub_b.Publish(), ORB_OK);

  // RunOnce dispatches the first callback, which calls RunOnce(0) to
  // dispatch the second callback.
  loop.RunOnce(1000);

  EXPECT_EQ(first_calls.load(), 1);
  EXPECT_EQ(second_calls.load(), 1);
}

// Stress test: 100 subscriptions to the same topic, single publish, all
// callbacks must fire across multiple RunOnce(0) calls.
TEST(EventLoopTest, ManySubscriptionsStress) {
  uorb::EventLoop loop;
  ASSERT_TRUE(loop);

  constexpr int kNumSubs = 100;
  std::atomic<int> call_count{0};

  for (int i = 0; i < kNumSubs; ++i) {
    ASSERT_TRUE(loop.Subscribe<uorb::msg::orb_test>(
        [&](const orb_test_s &) { ++call_count; }));
  }

  DrainPendingEvents(loop);
  call_count = 0;

  uorb::PublicationData<uorb::msg::orb_test> pub;
  pub.data().val = 7;
  ASSERT_EQ(pub.Publish(), ORB_OK);

  // Repeatedly call RunOnce(0) until no more events are ready.
  for (int i = 0; i < 10; ++i) {
    int n = loop.RunOnce(0);
    if (n <= 0) break;
  }

  EXPECT_EQ(call_count.load(), kNumSubs);
}

}  // namespace uORBTest
