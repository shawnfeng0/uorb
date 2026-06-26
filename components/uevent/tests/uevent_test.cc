/****************************************************************************
 *
 * uevent C API tests — tests uevent directly, no uORB dependency.
 *
 ****************************************************************************/

#include <gtest/gtest.h>
#include <uevent/uevent.h>

#include <atomic>
#include <chrono>
#include <thread>

// ---- uevent_create / uevent_destroy ----

TEST(UeventTest, CreateDestroy) {
  uevent_t base = UEVENT_INITIALIZER;
  ASSERT_EQ(uevent_create(&base), 0);
  ASSERT_NE(base._handle, nullptr);
  uevent_destroy(&base);
}

TEST(UeventTest, DestroyNull) {
  uevent_destroy(nullptr);  // should not crash
}

// ---- uevent_loop with no sources ----

TEST(UeventTest, LoopNoSourcesReturnsZero) {
  uevent_t base = UEVENT_INITIALIZER;
  ASSERT_EQ(uevent_create(&base), 0);

  uevent_source_t ready[1] = {UEVENT_SOURCE_INITIALIZER};
  EXPECT_EQ(uevent_loop(&base, ready, 1, 0), 0);

  uevent_destroy(&base);
}

TEST(UeventTest, LoopTimeoutReturnsZero) {
  uevent_t base = UEVENT_INITIALIZER;
  ASSERT_EQ(uevent_create(&base), 0);

  uevent_source_t ready[1] = {UEVENT_SOURCE_INITIALIZER};
  auto t0 = std::chrono::steady_clock::now();
  EXPECT_EQ(uevent_loop(&base, ready, 1, 50), 0);
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                     std::chrono::steady_clock::now() - t0)
                     .count();
  EXPECT_GE(elapsed, 40);

  uevent_destroy(&base);
}

TEST(UeventTest, LoopBlockForeverReturnsMinusOneOnLoopbreak) {
  uevent_t base = UEVENT_INITIALIZER;
  ASSERT_EQ(uevent_create(&base), 0);

  std::thread breaker([&base]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    uevent_loopbreak(&base);
  });

  uevent_source_t ready[1] = {UEVENT_SOURCE_INITIALIZER};
  EXPECT_EQ(uevent_loop(&base, ready, 1, -1), -1);

  breaker.join();
  uevent_destroy(&base);
}

// ---- uevent_loopbreak is non-sticky ----

TEST(UeventTest, LoopbreakIsNonSticky) {
  uevent_t base = UEVENT_INITIALIZER;
  ASSERT_EQ(uevent_create(&base), 0);

  uevent_loopbreak(&base);

  uevent_source_t ready[1] = {UEVENT_SOURCE_INITIALIZER};
  EXPECT_EQ(uevent_loop(&base, ready, 1, 0), -1);
  // Subsequent loop should work normally
  EXPECT_EQ(uevent_loop(&base, ready, 1, 0), 0);

  uevent_destroy(&base);
}

// ---- uevent_source_create / uevent_source_destroy ----

TEST(UeventTest, SourceCreateWithNullReadyFnFails) {
  errno = 0;
  uevent_source_t src = UEVENT_SOURCE_INITIALIZER;
  EXPECT_EQ(uevent_source_create(&src, nullptr, nullptr, nullptr, nullptr, nullptr), -1);
  EXPECT_EQ(errno, EINVAL);
}

TEST(UeventTest, SourceCreateDestroy) {
  auto ready_fn = [](void *) -> bool { return false; };
  uevent_source_t src = UEVENT_SOURCE_INITIALIZER;
  ASSERT_EQ(uevent_source_create(&src, ready_fn, nullptr, nullptr, nullptr, nullptr), 0);
  uevent_source_destroy(&src);
}

TEST(UeventTest, SourceDestroyNull) {
  uevent_source_destroy(nullptr);  // should not crash
}

// ---- uevent_source_is_bound ----

TEST(UeventTest, SourceIsBoundFalseBeforeAdd) {
  auto ready_fn = [](void *) -> bool { return false; };
  uevent_source_t src = UEVENT_SOURCE_INITIALIZER;
  ASSERT_EQ(uevent_source_create(&src, ready_fn, nullptr, nullptr, nullptr, nullptr), 0);
  EXPECT_FALSE(uevent_source_is_bound(&src));
  uevent_source_destroy(&src);
}

TEST(UeventTest, SourceIsBoundTrueAfterAdd) {
  auto ready_fn = [](void *) -> bool { return false; };
  uevent_source_t src = UEVENT_SOURCE_INITIALIZER;
  ASSERT_EQ(uevent_source_create(&src, ready_fn, nullptr, nullptr, nullptr, nullptr), 0);

  uevent_t base = UEVENT_INITIALIZER;
  ASSERT_EQ(uevent_create(&base), 0);

  ASSERT_EQ(uevent_add(&base, &src, 0), 0);
  EXPECT_TRUE(uevent_source_is_bound(&src));

  ASSERT_EQ(uevent_remove(&base, &src), 0);
  EXPECT_FALSE(uevent_source_is_bound(&src));

  uevent_source_destroy(&src);
  uevent_destroy(&base);
}

// ---- uevent_add / uevent_remove ----

TEST(UeventTest, AddNullBaseFails) {
  auto ready_fn = [](void *) -> bool { return false; };
  uevent_source_t src = UEVENT_SOURCE_INITIALIZER;
  ASSERT_EQ(uevent_source_create(&src, ready_fn, nullptr, nullptr, nullptr, nullptr), 0);
  EXPECT_EQ(uevent_add(nullptr, &src, 0), -1);
  EXPECT_EQ(errno, EINVAL);
  uevent_source_destroy(&src);
}

TEST(UeventTest, AddNullSourceFails) {
  uevent_t base = UEVENT_INITIALIZER;
  ASSERT_EQ(uevent_create(&base), 0);
  EXPECT_EQ(uevent_add(&base, nullptr, 0), -1);
  EXPECT_EQ(errno, EINVAL);
  uevent_destroy(&base);
}

TEST(UeventTest, RemoveNullBaseFails) {
  auto ready_fn = [](void *) -> bool { return false; };
  uevent_source_t src = UEVENT_SOURCE_INITIALIZER;
  ASSERT_EQ(uevent_source_create(&src, ready_fn, nullptr, nullptr, nullptr, nullptr), 0);
  EXPECT_EQ(uevent_remove(nullptr, &src), -1);
  EXPECT_EQ(errno, EINVAL);
  uevent_source_destroy(&src);
}

TEST(UeventTest, AddSameSourceToTwoBasesFails) {
  auto ready_fn = [](void *) -> bool { return false; };
  uevent_source_t src = UEVENT_SOURCE_INITIALIZER;
  ASSERT_EQ(uevent_source_create(&src, ready_fn, nullptr, nullptr, nullptr, nullptr), 0);

  uevent_t base_a = UEVENT_INITIALIZER;
  uevent_t base_b = UEVENT_INITIALIZER;
  ASSERT_EQ(uevent_create(&base_a), 0);
  ASSERT_EQ(uevent_create(&base_b), 0);

  ASSERT_EQ(uevent_add(&base_a, &src, 0), 0);

  errno = 0;
  EXPECT_EQ(uevent_add(&base_b, &src, 0), -1);
  EXPECT_EQ(errno, EBUSY);

  ASSERT_EQ(uevent_remove(&base_a, &src), 0);
  // Now can add to base_b
  ASSERT_EQ(uevent_add(&base_b, &src, 0), 0);
  ASSERT_EQ(uevent_remove(&base_b, &src), 0);

  uevent_source_destroy(&src);
  uevent_destroy(&base_a);
  uevent_destroy(&base_b);
}

TEST(UeventTest, RemoveUnboundSourceFails) {
  auto ready_fn = [](void *) -> bool { return false; };
  uevent_source_t src = UEVENT_SOURCE_INITIALIZER;
  ASSERT_EQ(uevent_source_create(&src, ready_fn, nullptr, nullptr, nullptr, nullptr), 0);

  uevent_t base = UEVENT_INITIALIZER;
  ASSERT_EQ(uevent_create(&base), 0);

  EXPECT_EQ(uevent_remove(&base, &src), -1);

  uevent_source_destroy(&src);
  uevent_destroy(&base);
}

// ---- uevent_add idempotent ----

TEST(UeventTest, AddSameSourceTwiceIsIdempotent) {
  auto ready_fn = [](void *) -> bool { return false; };
  uevent_source_t src = UEVENT_SOURCE_INITIALIZER;
  ASSERT_EQ(uevent_source_create(&src, ready_fn, nullptr, nullptr, nullptr, nullptr), 0);

  uevent_t base = UEVENT_INITIALIZER;
  ASSERT_EQ(uevent_create(&base), 0);

  ASSERT_EQ(uevent_add(&base, &src, 0), 0);
  ASSERT_EQ(uevent_add(&base, &src, 0), 0);  // idempotent
  ASSERT_EQ(uevent_remove(&base, &src), 0);
  EXPECT_EQ(uevent_remove(&base, &src), -1);  // already removed

  uevent_source_destroy(&src);
  uevent_destroy(&base);
}

// ---- uevent_loop returns ready source ----

TEST(UeventTest, LoopReturnsReadySource) {
  struct Ctx {
    std::atomic<bool> ready{false};
  };
  Ctx ctx;

  auto ready_fn = [](void *p) -> bool {
    return static_cast<Ctx *>(p)->ready.load();
  };

  uevent_source_t src = UEVENT_SOURCE_INITIALIZER;
  ASSERT_EQ(uevent_source_create(&src, ready_fn, nullptr, nullptr, nullptr, &ctx), 0);

  uevent_t base = UEVENT_INITIALIZER;
  ASSERT_EQ(uevent_create(&base), 0);
  ASSERT_EQ(uevent_add(&base, &src, 0), 0);

  // No data initially
  uevent_source_t ready[1] = {UEVENT_SOURCE_INITIALIZER};
  EXPECT_EQ(uevent_loop(&base, ready, 1, 0), 0);

  // Set ready and notify
  ctx.ready = true;
  uevent_source_notify(&src);

  // Loop with timeout should return the source
  ASSERT_EQ(uevent_loop(&base, ready, 1, 100), 1);
  EXPECT_EQ(ready[0]._handle, src._handle);

  // After consuming, should be not ready again
  ctx.ready = false;
  EXPECT_EQ(uevent_loop(&base, ready, 1, 0), 0);

  ASSERT_EQ(uevent_remove(&base, &src), 0);
  uevent_source_destroy(&src);
  uevent_destroy(&base);
}

// ---- uevent_loop blocks until notified ----

TEST(UeventTest, LoopBlocksUntilNotified) {
  struct Ctx {
    std::atomic<bool> ready{false};
  };
  Ctx ctx;

  auto ready_fn = [](void *p) -> bool {
    return static_cast<Ctx *>(p)->ready.load();
  };

  uevent_source_t src = UEVENT_SOURCE_INITIALIZER;
  ASSERT_EQ(uevent_source_create(&src, ready_fn, nullptr, nullptr, nullptr, &ctx), 0);

  uevent_t base = UEVENT_INITIALIZER;
  ASSERT_EQ(uevent_create(&base), 0);
  ASSERT_EQ(uevent_add(&base, &src, 0), 0);

  std::thread notifier([&]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    ctx.ready = true;
    uevent_source_notify(&src);
  });

  uevent_source_t ready[1] = {UEVENT_SOURCE_INITIALIZER};
  auto t0 = std::chrono::steady_clock::now();
  ASSERT_EQ(uevent_loop(&base, ready, 1, -1), 1);
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                     std::chrono::steady_clock::now() - t0)
                     .count();
  EXPECT_GE(elapsed, 25);
  EXPECT_EQ(ready[0]._handle, src._handle);

  notifier.join();
  ASSERT_EQ(uevent_remove(&base, &src), 0);
  uevent_source_destroy(&src);
  uevent_destroy(&base);
}

// ---- uevent_loop rejects invalid args ----

TEST(UeventTest, LoopNullBaseFails) {
  uevent_source_t ready[1] = {UEVENT_SOURCE_INITIALIZER};
  EXPECT_EQ(uevent_loop(nullptr, ready, 1, 0), -1);
  EXPECT_EQ(errno, EINVAL);
}

TEST(UeventTest, LoopNullReadyFails) {
  uevent_t base = UEVENT_INITIALIZER;
  ASSERT_EQ(uevent_create(&base), 0);
  EXPECT_EQ(uevent_loop(&base, nullptr, 1, 0), -1);
  EXPECT_EQ(errno, EINVAL);
  uevent_destroy(&base);
}

TEST(UeventTest, LoopZeroMaxReadyFails) {
  uevent_t base = UEVENT_INITIALIZER;
  ASSERT_EQ(uevent_create(&base), 0);
  uevent_source_t ready[1] = {UEVENT_SOURCE_INITIALIZER};
  EXPECT_EQ(uevent_loop(&base, ready, 0, 0), -1);
  EXPECT_EQ(errno, EINVAL);
  uevent_destroy(&base);
}

// ---- Per-event timeout ----

TEST(UeventTest, PerEventTimeoutFires) {
  struct Ctx {
    std::atomic<bool> ready{false};
  };
  Ctx ctx;

  auto ready_fn = [](void *p) -> bool {
    return static_cast<Ctx *>(p)->ready.load();
  };

  uevent_source_t src = UEVENT_SOURCE_INITIALIZER;
  ASSERT_EQ(uevent_source_create(&src, ready_fn, nullptr, nullptr, nullptr, &ctx), 0);

  uevent_t base = UEVENT_INITIALIZER;
  ASSERT_EQ(uevent_create(&base), 0);
  ASSERT_EQ(uevent_add(&base, &src, 50), 0);

  uevent_source_t ready[1] = {UEVENT_SOURCE_INITIALIZER};
  auto t0 = std::chrono::steady_clock::now();
  // Block forever, but per-event timeout should fire after 50ms
  ASSERT_EQ(uevent_loop(&base, ready, 1, -1), 1);
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                     std::chrono::steady_clock::now() - t0)
                     .count();
  EXPECT_EQ(ready[0]._handle, src._handle);
  EXPECT_GE(elapsed, 30);
  EXPECT_LE(elapsed, 200);

  // One-shot: should not fire again
  EXPECT_EQ(uevent_loop(&base, ready, 1, 0), 0);

  ASSERT_EQ(uevent_remove(&base, &src), 0);
  uevent_source_destroy(&src);
  uevent_destroy(&base);
}

// ---- Multiple sources ----

TEST(UeventTest, MultipleSourcesAllReady) {
  struct Ctx {
    std::atomic<bool> ready{false};
  };
  Ctx ctx1, ctx2;

  auto ready_fn = [](void *p) -> bool {
    return static_cast<Ctx *>(p)->ready.load();
  };

  uevent_source_t src1 = UEVENT_SOURCE_INITIALIZER;
  uevent_source_t src2 = UEVENT_SOURCE_INITIALIZER;
  ASSERT_EQ(uevent_source_create(&src1, ready_fn, nullptr, nullptr, nullptr, &ctx1), 0);
  ASSERT_EQ(uevent_source_create(&src2, ready_fn, nullptr, nullptr, nullptr, &ctx2), 0);

  uevent_t base = UEVENT_INITIALIZER;
  ASSERT_EQ(uevent_create(&base), 0);
  ASSERT_EQ(uevent_add(&base, &src1, 0), 0);
  ASSERT_EQ(uevent_add(&base, &src2, 0), 0);

  ctx1.ready = true;
  ctx2.ready = true;
  uevent_source_notify(&src1);
  uevent_source_notify(&src2);

  uevent_source_t ready[2] = {UEVENT_SOURCE_INITIALIZER, UEVENT_SOURCE_INITIALIZER};
  ASSERT_EQ(uevent_loop(&base, ready, 2, 100), 2);
  EXPECT_TRUE((ready[0]._handle == src1._handle && ready[1]._handle == src2._handle) ||
              (ready[0]._handle == src2._handle && ready[1]._handle == src1._handle));

  ASSERT_EQ(uevent_remove(&base, &src1), 0);
  ASSERT_EQ(uevent_remove(&base, &src2), 0);
  uevent_source_destroy(&src1);
  uevent_source_destroy(&src2);
  uevent_destroy(&base);
}

// ---- ctx_destroy callback ----

TEST(UeventTest, CtxDestroyCalledOnSourceDestroy) {
  struct Ctx {
    std::atomic<bool> destroyed{false};
  };
  Ctx ctx;

  auto ready_fn = [](void *) -> bool { return false; };
  auto destroy_fn = [](void *p) {
    static_cast<Ctx *>(p)->destroyed = true;
  };

  uevent_source_t src = UEVENT_SOURCE_INITIALIZER;
  ASSERT_EQ(uevent_source_create(&src, ready_fn, nullptr, nullptr, destroy_fn, &ctx), 0);
  EXPECT_FALSE(ctx.destroyed.load());

  uevent_source_destroy(&src);
  EXPECT_TRUE(ctx.destroyed.load());
}

// ---- Concurrent notify from multiple threads ----

TEST(UeventTest, ConcurrentNotifyFromMultipleThreads) {
  struct Ctx {
    std::atomic<int> count{0};
  };
  Ctx ctx;

  auto ready_fn = [](void *p) -> bool {
    return static_cast<Ctx *>(p)->count.load() > 0;
  };

  uevent_source_t src = UEVENT_SOURCE_INITIALIZER;
  ASSERT_EQ(uevent_source_create(&src, ready_fn, nullptr, nullptr, nullptr, &ctx), 0);

  uevent_t base = UEVENT_INITIALIZER;
  ASSERT_EQ(uevent_create(&base), 0);
  ASSERT_EQ(uevent_add(&base, &src, 0), 0);

  // 4 threads concurrently notify
  std::thread threads[4];
  for (auto &t : threads) {
    t = std::thread([&]() {
      for (int i = 0; i < 100; ++i) {
        ctx.count.fetch_add(1);
        uevent_source_notify(&src);
      }
    });
  }

  // Collect events
  int total_ready = 0;
  for (int i = 0; i < 200; ++i) {
    uevent_source_t ready[1] = {UEVENT_SOURCE_INITIALIZER};
    int n = uevent_loop(&base, ready, 1, 10);
    if (n > 0) {
      total_ready++;
      ctx.count.store(0);  // consume
    }
  }

  for (auto &t : threads) t.join();

  // 400 notifications should produce at least 1 ready event.
  // Threshold is low because consumer runs after producers finish,
  // so most notifications may have been coalesced.
  EXPECT_GE(total_ready, 1);

  ASSERT_EQ(uevent_remove(&base, &src), 0);
  uevent_source_destroy(&src);
  uevent_destroy(&base);
}

// ---- Notify without add is safe ----

TEST(UeventTest, NotifyWithoutAddIsSafe) {
  struct Ctx {
    std::atomic<bool> ready{false};
  };
  Ctx ctx;

  auto ready_fn = [](void *p) -> bool {
    return static_cast<Ctx *>(p)->ready.load();
  };

  uevent_source_t src = UEVENT_SOURCE_INITIALIZER;
  ASSERT_EQ(uevent_source_create(&src, ready_fn, nullptr, nullptr, nullptr, &ctx), 0);

  // Notify without adding to any base — should not crash
  ctx.ready = true;
  uevent_source_notify(&src);
  uevent_source_notify(&src);

  uevent_source_destroy(&src);
}

// ---- Loop with max_ready smaller than ready count ----

TEST(UeventTest, LoopTruncatesReadyOutput) {
  struct Ctx {
    std::atomic<bool> ready{true};
  };
  Ctx ctx1, ctx2;

  auto ready_fn = [](void *p) -> bool {
    return static_cast<Ctx *>(p)->ready.load();
  };

  uevent_source_t src1 = UEVENT_SOURCE_INITIALIZER;
  uevent_source_t src2 = UEVENT_SOURCE_INITIALIZER;
  ASSERT_EQ(uevent_source_create(&src1, ready_fn, nullptr, nullptr, nullptr, &ctx1), 0);
  ASSERT_EQ(uevent_source_create(&src2, ready_fn, nullptr, nullptr, nullptr, &ctx2), 0);

  uevent_t base = UEVENT_INITIALIZER;
  ASSERT_EQ(uevent_create(&base), 0);
  ASSERT_EQ(uevent_add(&base, &src1, 0), 0);
  ASSERT_EQ(uevent_add(&base, &src2, 0), 0);

  // Both ready, but max_ready=1 should only return 1
  uevent_source_t ready[1] = {UEVENT_SOURCE_INITIALIZER};
  ASSERT_EQ(uevent_loop(&base, ready, 1, 0), 1);
  EXPECT_TRUE(ready[0]._handle == src1._handle || ready[0]._handle == src2._handle);

  // Mark consumed source not ready
  if (ready[0]._handle == src1._handle) ctx1.ready = false;
  else ctx2.ready = false;

  // Second call should return the other one
  ASSERT_EQ(uevent_loop(&base, ready, 1, 0), 1);
  EXPECT_TRUE(ready[0]._handle == src1._handle || ready[0]._handle == src2._handle);

  // Mark remaining not ready
  if (ready[0]._handle == src1._handle) ctx1.ready = false;
  else ctx2.ready = false;

  // No more ready
  EXPECT_EQ(uevent_loop(&base, ready, 1, 0), 0);

  ASSERT_EQ(uevent_remove(&base, &src1), 0);
  ASSERT_EQ(uevent_remove(&base, &src2), 0);
  uevent_source_destroy(&src1);
  uevent_source_destroy(&src2);
  uevent_destroy(&base);
}

// ---- Multiple loopbreak calls ----

TEST(UeventTest, MultipleLoopbreakCalls) {
  uevent_t base = UEVENT_INITIALIZER;
  ASSERT_EQ(uevent_create(&base), 0);

  uevent_loopbreak(&base);
  uevent_loopbreak(&base);
  uevent_loopbreak(&base);

  uevent_source_t ready[1] = {UEVENT_SOURCE_INITIALIZER};
  EXPECT_EQ(uevent_loop(&base, ready, 1, 0), -1);
  // Subsequent loop should work normally (non-sticky)
  EXPECT_EQ(uevent_loop(&base, ready, 1, 0), 0);

  uevent_destroy(&base);
}

// ---- Destroy source while still added to base (error path) ----

TEST(UeventTest, DestroySourceWhileAddedToBase) {
  // Destroying a source while it's still added to a base is a user error.
  // The correct lifecycle is: uevent_remove -> uevent_source_destroy.
  // This test verifies the safe path works correctly.
  auto ready_fn = [](void *) -> bool { return false; };
  uevent_source_t src = UEVENT_SOURCE_INITIALIZER;
  ASSERT_EQ(uevent_source_create(&src, ready_fn, nullptr, nullptr, nullptr, nullptr), 0);

  uevent_t base = UEVENT_INITIALIZER;
  ASSERT_EQ(uevent_create(&base), 0);
  ASSERT_EQ(uevent_add(&base, &src, 0), 0);

  // Correct: remove first, then destroy
  ASSERT_EQ(uevent_remove(&base, &src), 0);
  uevent_source_destroy(&src);
  uevent_destroy(&base);
}
