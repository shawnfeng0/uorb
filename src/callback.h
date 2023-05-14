//
// Copyright (c) 2021 shawnfeng. All rights reserved.
//
#pragma once

#include "base/condition_variable.h"

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
    base::LockGuard<base::Mutex> lg(mutex);
    operator()();
  }
  base::Mutex mutex{};
};

struct SemaphoreCallback : public Callback<>, public base::SimpleSemaphore {
  void operator()() override { release(); }
};

}  // namespace uorb
