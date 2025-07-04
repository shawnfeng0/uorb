//
// Copyright (c) 2021 shawnfeng. All rights reserved.
//
#pragma once

#include "base/intrusive_list/forward_list.h"

namespace uorb::detail {

struct ReceiverBase {
  UORB_NONCOPYABLE(ReceiverBase);
  ReceiverBase() = default;
  virtual ~ReceiverBase() = default;

  virtual void notify_all() = 0;
  bool operator==(const ReceiverBase& rhs) const { return &rhs == this; }

  intrusive_list::forward_list_node receiver_node{};
};

}  // namespace uorb::detail
