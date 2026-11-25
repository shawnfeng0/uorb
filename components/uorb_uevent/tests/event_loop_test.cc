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
#include <uorb/topics/orb_test_volatile.h>

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

// Subscribe() accepts move-only callables (no std::function copyability requirement).
TEST(EventLoopTest, SubscribeAcceptsMoveOnlyCallback) {
  uorb::EventLoop loop;
  ASSERT_TRUE(loop);

  std::unique_ptr<int> state(new int(10));
  int observed = 0;
  ASSERT_TRUE(loop.Subscribe<uorb::msg::orb_test>(
      [ptr = std::move(state), &observed](const orb_test_s &msg) {
        observed = msg.val + *ptr;
      }));

  DrainPendingEvents(loop);
  observed = 0;

  uorb::PublicationData<uorb::msg::orb_test> pub;
  pub.data().val = 32;
  ASSERT_EQ(pub.Publish(), ORB_OK);

  ASSERT_EQ(loop.RunOnce(100), 1);
  EXPECT_EQ(observed, 42);
}

// ---------------------------------------------------------------------------
// AsyncDispatcher tests
// ---------------------------------------------------------------------------

TEST(AsyncDispatcherTest, StartStopRestartAndOffPublisherThreadDelivery) {
  uorb::AsyncDispatcher dispatcher;
  ASSERT_TRUE(dispatcher);
  EXPECT_FALSE(dispatcher.IsRunning());

  std::atomic<int> received_val{0};
  std::atomic<bool> on_different_thread{false};
  const std::thread::id main_tid = std::this_thread::get_id();

  const auto tok = dispatcher.Subscribe<uorb::msg::orb_test_volatile>(
      [&](const orb_test_volatile_s &msg) {
        on_different_thread.store(std::this_thread::get_id() != main_tid,
                                  std::memory_order_release);
        received_val.store(msg.val, std::memory_order_release);
      });
  ASSERT_NE(tok, uorb::AsyncDispatcher::kInvalidToken);

  ASSERT_TRUE(dispatcher.Start());
  EXPECT_TRUE(dispatcher.IsRunning());
  // Start() is idempotent.
  EXPECT_TRUE(dispatcher.Start());

  orb_test_volatile_s sent{};
  sent.val = 123;
  ASSERT_EQ(uorb::publish<uorb::msg::orb_test_volatile>(sent), ORB_OK);

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (received_val.load(std::memory_order_acquire) != 123 &&
         std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  EXPECT_EQ(received_val.load(), 123);
  EXPECT_TRUE(on_different_thread.load());

  // Stop and restart cleanly.
  dispatcher.Stop();
  EXPECT_FALSE(dispatcher.IsRunning());
  dispatcher.Stop();  // idempotent

  ASSERT_TRUE(dispatcher.Start());
  EXPECT_TRUE(dispatcher.IsRunning());

  sent.val = 456;
  ASSERT_EQ(uorb::publish<uorb::msg::orb_test_volatile>(sent), ORB_OK);
  const auto deadline2 = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (received_val.load(std::memory_order_acquire) != 456 &&
         std::chrono::steady_clock::now() < deadline2) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  EXPECT_EQ(received_val.load(), 456);
  EXPECT_TRUE(dispatcher.Unsubscribe(tok));
  EXPECT_FALSE(dispatcher.Unsubscribe(tok));  // second unsubscribe returns false
}

// Re-publishing the same topic 100 times in a chain runs iteratively on the
// dispatcher worker thread with O(1) stack depth and zero deadlocks.
TEST(AsyncDispatcherTest, LongPublishChainRunsIterativelyWithoutDeadlock) {
  uorb::AsyncDispatcher dispatcher;
  ASSERT_TRUE(dispatcher);
  ASSERT_TRUE(dispatcher.Start());

  constexpr int kTarget = 100;
  std::atomic<int> count{0};
  const auto tok = dispatcher.Subscribe<uorb::msg::orb_test_volatile>(
      [&](const orb_test_volatile_s &msg) {
        const int n = count.fetch_add(1, std::memory_order_acq_rel) + 1;
        if (n < kTarget) {
          orb_test_volatile_s next = msg;
          next.val += 1;
          EXPECT_EQ(uorb::publish<uorb::msg::orb_test_volatile>(next), ORB_OK);
        }
      });
  ASSERT_NE(tok, uorb::AsyncDispatcher::kInvalidToken);

  orb_test_volatile_s start_msg{};
  start_msg.val = 1;
  ASSERT_EQ(uorb::publish<uorb::msg::orb_test_volatile>(start_msg), ORB_OK);

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (count.load(std::memory_order_acquire) < kTarget &&
         std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  EXPECT_EQ(count.load(), kTarget);
}

// Cross-thread Unsubscribe() must wait for an in-flight callback to finish before
// returning, and self-Unsubscribe() + dynamic Subscribe() inside a callback must
// work without deadlocking.
TEST(AsyncDispatcherTest, CrossThreadDrainAndSelfUnsubscribeInsideCallback) {
  uorb::AsyncDispatcher dispatcher;
  ASSERT_TRUE(dispatcher);
  ASSERT_TRUE(dispatcher.Start());

  // 1) Cross-thread drain on Unsubscribe()
  std::atomic<bool> entered{false};
  std::atomic<bool> finished{false};
  int *captured_state = new int(77);

  const auto slow_tok = dispatcher.Subscribe<uorb::msg::orb_test_volatile>(
      [&](const orb_test_volatile_s &) {
        entered.store(true, std::memory_order_release);
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        EXPECT_EQ(*captured_state, 77);
        finished.store(true, std::memory_order_release);
      });
  ASSERT_NE(slow_tok, uorb::AsyncDispatcher::kInvalidToken);

  orb_test_volatile_s m{};
  m.val = 1;
  ASSERT_EQ(uorb::publish<uorb::msg::orb_test_volatile>(m), ORB_OK);

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (!entered.load(std::memory_order_acquire) &&
         std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_TRUE(entered.load());
  ASSERT_TRUE(dispatcher.Unsubscribe(slow_tok));
  EXPECT_TRUE(finished.load());
  delete captured_state;

  // 2) Self-Unsubscribe + dynamic Subscribe from inside a callback
  std::atomic<uorb::AsyncDispatcher::Token> self_tok{uorb::AsyncDispatcher::kInvalidToken};
  std::atomic<int> first_calls{0};
  std::atomic<int> second_calls{0};

  const auto tok = dispatcher.Subscribe<uorb::msg::orb_test_volatile>(
      [&](const orb_test_volatile_s &) {
        first_calls.fetch_add(1, std::memory_order_acq_rel);
        const auto t = self_tok.load(std::memory_order_acquire);
        EXPECT_TRUE(dispatcher.Unsubscribe(t));
        const auto next_tok = dispatcher.Subscribe<uorb::msg::orb_test_volatile>(
            [&](const orb_test_volatile_s &) {
              second_calls.fetch_add(1, std::memory_order_acq_rel);
            });
        EXPECT_NE(next_tok, uorb::AsyncDispatcher::kInvalidToken);
      });
  ASSERT_NE(tok, uorb::AsyncDispatcher::kInvalidToken);
  self_tok.store(tok, std::memory_order_release);

  m.val = 2;
  ASSERT_EQ(uorb::publish<uorb::msg::orb_test_volatile>(m), ORB_OK);
  const auto deadline2 = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (first_calls.load(std::memory_order_acquire) < 1 &&
         std::chrono::steady_clock::now() < deadline2) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  EXPECT_EQ(first_calls.load(), 1);

  // Give the worker a moment to complete deferred cleanup of first subscription,
  // then publish again: only the dynamically subscribed callback should fire.
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  m.val = 3;
  ASSERT_EQ(uorb::publish<uorb::msg::orb_test_volatile>(m), ORB_OK);
  const auto deadline3 = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (second_calls.load(std::memory_order_acquire) < 1 &&
         std::chrono::steady_clock::now() < deadline3) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  EXPECT_EQ(first_calls.load(), 1);
  EXPECT_GE(second_calls.load(), 1);
}

// Self-Stop() inside a callback followed directly by Start() must reclaim the
// exited worker thread and restart cleanly; concurrent Start()/Stop() calls
// must serialize without spawning duplicate workers or corrupting state.
TEST(AsyncDispatcherTest, SelfStopInsideCallbackThenRestartAndConcurrentStartStop) {
  uorb::AsyncDispatcher dispatcher;
  ASSERT_TRUE(dispatcher);
  ASSERT_TRUE(dispatcher.Start());

  std::atomic<int> received{0};
  std::atomic<bool> stop_on_next{true};
  const auto tok = dispatcher.Subscribe<uorb::msg::orb_test_volatile>(
      [&](const orb_test_volatile_s &msg) {
        received.store(msg.val, std::memory_order_release);
        if (stop_on_next.exchange(false, std::memory_order_acq_rel)) {
          dispatcher.Stop();
        }
      });
  ASSERT_NE(tok, uorb::AsyncDispatcher::kInvalidToken);

  orb_test_volatile_s m{};
  m.val = 11;
  ASSERT_EQ(uorb::publish<uorb::msg::orb_test_volatile>(m), ORB_OK);

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (dispatcher.IsRunning() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  EXPECT_EQ(received.load(std::memory_order_acquire), 11);
  EXPECT_FALSE(dispatcher.IsRunning());

  // Directly call Start() without an intermediate external Stop(): must join
  // the exited worker and spawn a fresh one.
  ASSERT_TRUE(dispatcher.Start());
  EXPECT_TRUE(dispatcher.IsRunning());

  m.val = 22;
  ASSERT_EQ(uorb::publish<uorb::msg::orb_test_volatile>(m), ORB_OK);
  const auto deadline2 = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (received.load(std::memory_order_acquire) != 22 &&
         std::chrono::steady_clock::now() < deadline2) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  EXPECT_EQ(received.load(std::memory_order_acquire), 22);

  // Concurrent Start()/Stop() stress across multiple threads.
  std::thread t1([&]() {
    for (int i = 0; i < 20; ++i) {
      dispatcher.Start();
      std::this_thread::yield();
    }
  });
  std::thread t2([&]() {
    for (int i = 0; i < 20; ++i) {
      dispatcher.Stop();
      std::this_thread::yield();
    }
  });
  std::thread t3([&]() {
    for (int i = 0; i < 20; ++i) {
      orb_test_volatile_s msg{};
      msg.val = 100 + i;
      uorb::publish<uorb::msg::orb_test_volatile>(msg);
      std::this_thread::yield();
    }
  });
  t1.join();
  t2.join();
  t3.join();

  // Leave in a known running state and verify delivery still works.
  ASSERT_TRUE(dispatcher.Start());
  EXPECT_TRUE(dispatcher.IsRunning());
  m.val = 999;
  ASSERT_EQ(uorb::publish<uorb::msg::orb_test_volatile>(m), ORB_OK);
  const auto deadline3 = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (received.load(std::memory_order_acquire) != 999 &&
         std::chrono::steady_clock::now() < deadline3) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  EXPECT_EQ(received.load(std::memory_order_acquire), 999);

  dispatcher.Stop();
  EXPECT_FALSE(dispatcher.IsRunning());
  EXPECT_TRUE(dispatcher.Unsubscribe(tok));
}

}  // namespace uORBTest
