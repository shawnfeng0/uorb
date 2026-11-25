#pragma once

#include <uorb/uorb.h>

#include <atomic>
#include <cstddef>
#include <new>
#include <utility>

namespace uorb {
namespace base {

/// Pluggable allocator wrapper for uORB core (default: ::operator new/delete).
/// Allows RTOS applications to inject deterministic static/pool allocators via
/// orb_set_allocator().
class Allocator {
 public:
  static orb_err Set(orb_alloc_fn alloc_fn, orb_free_fn free_fn) {
    if ((alloc_fn == nullptr) != (free_fn == nullptr)) {
      return ORB_ERR_INVALID;
    }
    alloc_fn_.store(alloc_fn, std::memory_order_release);
    free_fn_.store(free_fn, std::memory_order_release);
    return ORB_OK;
  }

  static void *Alloc(size_t size) {
    const orb_alloc_fn fn = alloc_fn_.load(std::memory_order_acquire);
    if (fn) {
      return fn(size);
    }
    return ::operator new(size, std::nothrow);
  }

  static void Free(void *ptr) {
    if (!ptr) return;
    const orb_free_fn fn = free_fn_.load(std::memory_order_acquire);
    if (fn) {
      fn(ptr);
      return;
    }
    ::operator delete(ptr);
  }

  template <typename T, typename... Args>
  static T *New(Args &&...args) {
    void *mem = Alloc(sizeof(T));
    if (!mem) return nullptr;
    return new (mem) T(std::forward<Args>(args)...);
  }

  template <typename T>
  static void Delete(T *ptr) {
    if (!ptr) return;
    ptr->~T();
    Free(ptr);
  }

 private:
  static std::atomic<orb_alloc_fn> alloc_fn_;
  static std::atomic<orb_free_fn> free_fn_;
};

}  // namespace base
}  // namespace uorb
