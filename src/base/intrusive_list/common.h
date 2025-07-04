//
// Created by shawnfeng on 2021/11/2.
//

#pragma once

#include <cstddef>
#include <cstdint>

namespace intrusive_list::internal {

template <class Type, class Member>
static inline constexpr ptrdiff_t offset_of(const Member Type::*member) {
  return reinterpret_cast<ptrdiff_t>(&(reinterpret_cast<Type *>(0)->*member));
}

template <class Type, class Member>
static inline constexpr Type *owner_of(const Member *ptr,
                                       const Member Type::*member) {
  return reinterpret_cast<Type *>(reinterpret_cast<intptr_t>(ptr) -
                                  offset_of(member));
}

}  // namespace intrusive_list::internal
