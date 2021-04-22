//
// Copyright (c) 2021 shawnfeng. All rights reserved.
//
#pragma once

namespace uorb {

namespace detail {

struct CallbackBase {
  virtual void Notify() = 0;

 private:
  virtual void operator()() = 0;
};

}  // namespace detail

template <bool needs_locking = false>
struct Callback : detail::CallbackBase {
 private:
  void operator()() override = 0;
  void Notify() final { operator()(); }
};

template <>
struct Callback<true> : detail::CallbackBase {
 private:
  void operator()() override = 0;
  void Notify() final {
    base::MutexGuard lg(mutex);
    operator()();
  }
  base::Mutex mutex{};
};

struct SemaphoreCallback : public Callback<>,
                           public base::SimpleSemaphore<CLOCK_MONOTONIC> {
  void operator()() override { release(); }
};

}  // namespace uorb
