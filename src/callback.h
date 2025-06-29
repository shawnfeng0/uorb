//
// Copyright (c) 2021 shawnfeng. All rights reserved.
//
#pragma once

#include "base/condition_variable.h"

namespace uorb {

namespace detail {

struct CallbackBase {
  virtual ~CallbackBase() = default;
  virtual void notify_all() = 0;
};

}  // namespace detail

class UpdateNotifyCallback final : public detail::CallbackBase {
 public:
  void notify_all() override { notifier_.notify_all(); }

  template <typename Predicate>
  void wait(Predicate pred) {
    notifier_.wait(pred);
  }

  template <typename Predicate>
  bool wait_for(uint32_t time_ms, Predicate pred) {
    return notifier_.wait_for(time_ms, pred);
  }

 private:
  base::LiteNotifier notifier_;
};

}  // namespace uorb
