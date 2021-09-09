//
// Created by shawnfeng on 2021/9/9.
//

#pragma once

namespace uorb {
namespace internal {

class Noncopyable {
 public:
  Noncopyable() = default;
  ~Noncopyable() = default;

  Noncopyable(const Noncopyable&) = delete;
  Noncopyable& operator=(const Noncopyable&) = delete;
};

}  // namespace internal
}  // namespace uorb
