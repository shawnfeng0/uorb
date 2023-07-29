//
// Created by shawnfeng on 23-5-13.
//
#include "condition_variable.h"

#include <gtest/gtest.h>

#include <chrono>
#include <thread>

#include "uorb/abs_time.h"

#define DEBUG_MARK(mark)                                              \
  printf("%s:%d %" PRIu64 ".%03" PRIu64 "ms " #mark "\r\n", __FILE__, \
         __LINE__, orb_absolute_time_us() / 1000000,                  \
         (orb_absolute_time_us() / 1000) % 1000)

class Timer {
 public:
  Timer() { start_ = std::chrono::high_resolution_clock::now(); }
  ~Timer() = default;

  template <typename T>
  uint64_t elapsed() {
    auto now = std::chrono::high_resolution_clock::now();
    return std::chrono::duration_cast<T>(now - start_).count();
  }
  uint64_t elapsed_us() { return elapsed<std::chrono::microseconds>(); }
  uint64_t elapsed_ms() { return elapsed<std::chrono::milliseconds>(); }

 private:
  std::chrono::time_point<std::chrono::high_resolution_clock> start_;
};

class ThreadBarrier {
 public:
  explicit ThreadBarrier(int num_threads) : num_threads_(num_threads) {}
  ThreadBarrier(const ThreadBarrier &) = delete;
  ThreadBarrier &operator=(const ThreadBarrier &) = delete;

  void wait() {
    uorb::base::LockGuard<uorb::base::Mutex> lock(mutex_);
    if (--num_threads_ == 0) {
      cv_.notify_all();
    } else {
      cv_.wait(mutex_, [&]() { return num_threads_ == 0; });
    }
  }

 private:
  uorb::base::Mutex mutex_;
  uorb::base::ConditionVariable cv_;
  int num_threads_;
};

namespace uorb {
namespace base {
class ConditionVariableTest : public ConditionVariable {
 public:
  using ConditionVariable::get_now_time;
  using ConditionVariable::timespec_get_after;
};
}  // namespace base
}  // namespace uorb

TEST(ConditionVariableTest, get_time) {
  for (int ms = 0; ms < 100000; ms++) {
    using namespace uorb::base;
    auto now = ConditionVariableTest::get_now_time();
    auto now_us = (uint64_t)now.tv_sec * 1000000 + now.tv_nsec / 1000;
    auto after = ConditionVariableTest::timespec_get_after(now, ms);
    auto after_us = (uint64_t)after.tv_sec * 1000000 + after.tv_nsec / 1000;
    EXPECT_EQ(after_us, now_us + ms * 1000);
  }
}

bool wait_for_case(int set_timeout_ms, int wait_timeout_ms,
                   uint64_t *actual_waiting_time) {
  using namespace uorb::base;
  auto cv = std::make_shared<ConditionVariable>();
  auto mutex = std::make_shared<Mutex>();
  auto barrier = std::make_shared<ThreadBarrier>(2);
  auto flag = std::make_shared<bool>(false);
  std::thread t([=]() {
    barrier->wait();
    std::this_thread::sleep_for(std::chrono::milliseconds(set_timeout_ms));
    {
      LockGuard<> lg(*mutex);
      *flag = true;
    }
    cv->notify_all();
  });
  barrier->wait();
  Timer timer;
  bool ret;
  {
    LockGuard<> lg(*mutex);
    ret = cv->wait_for(*mutex, wait_timeout_ms, [&]() { return *flag; });
  }
  if (actual_waiting_time) *actual_waiting_time = timer.elapsed_ms();
  t.join();
  return ret;
}

TEST(ConditionVariableTest, wait_for) {
  {
    uint64_t actual_waiting_time;
    EXPECT_TRUE(wait_for_case(10, 50, &actual_waiting_time));
    EXPECT_GE(actual_waiting_time, 10);
    EXPECT_LE(actual_waiting_time, 50);
  }
  {
    uint64_t actual_waiting_time;
    EXPECT_TRUE(wait_for_case(50, 200, &actual_waiting_time));
    EXPECT_GE(actual_waiting_time, 50);
    EXPECT_LE(actual_waiting_time, 200);
  }
}

TEST(ConditionVariableTest, wait_for_timeout) {
  {
    uint64_t actual_waiting_time;
    EXPECT_FALSE(wait_for_case(100, 10, &actual_waiting_time));
    EXPECT_GE(actual_waiting_time, 10);
    EXPECT_LE(actual_waiting_time, 100);
  }
}
