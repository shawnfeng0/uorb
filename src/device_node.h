#pragma once

#include <uorb/uorb.h>

#include <atomic>

#include "base/intrusive_list/forward_list.h"
#include "base/intrusive_list/list.h"
#include "base/mutex.h"

namespace uORBTest {
class UnitTest;
}

namespace uorb {
namespace detail {

// Function-pointer based callback entry for DeviceNode's publish notification.
// Embedded in ReceiverLocal for O(1) registration and O(1) unregistration.
struct CallbackEntry {
  orb_subscriber_callback_fn on_publish{nullptr};
  orb_callback_ctx ctx{};
  intrusive_list::list_node node{};
};

}  // namespace detail

class DeviceMaster;

/**
 * Per-object device instance.
 */
class DeviceNode {
  friend DeviceMaster;

 public:
  DeviceNode(const DeviceNode &) = delete;
  DeviceNode(DeviceNode &&) = delete;
  DeviceNode &operator=(const DeviceNode &) = delete;
  DeviceNode &operator=(DeviceNode &&) = delete;

  // Publish data to this node.
  bool Publish(const void *data);

  void FillStatus(orb_status *status) const;

  void add_subscriber();
  void remove_subscriber();
  uint16_t subscriber_count() const {
    const uint32_t c = subscriber_count_.load(std::memory_order_acquire);
    return static_cast<uint16_t>(c > kMaxCounterValue ? kMaxCounterValue : c);
  }
  void mark_untracked_subscriber();

  void add_publisher();
  void remove_publisher();
  uint16_t publisher_count() const {
    const uint32_t c = publisher_count_.load(std::memory_order_acquire);
    return static_cast<uint16_t>(c > kMaxCounterValue ? kMaxCounterValue : c);
  }
  void mark_untracked_publisher();

  // Whether meta and instance are the same as the current one
  inline bool IsSameWith(const orb_metadata &meta, uint8_t instance) const {
    return IsSameWith(meta) && (instance_ == instance);
  }

  inline bool IsSameWith(const orb_metadata &meta) const { return &meta_ == &meta; }

  // add item to list of work items to schedule on node update
  bool RegisterCallback(detail::CallbackEntry *entry) {
    if (!entry || !entry->on_publish) {
      return false;
    }

    base::LockGuard<base::Mutex> lg(callback_lock_);
    receiver_list_.push_front(*entry);
    callback_count_.fetch_add(1, std::memory_order_seq_cst);
    return true;
  }

  bool UnregisterCallback(detail::CallbackEntry *entry) {
    if (!entry) {
      return false;
    }

    base::LockGuard<base::Mutex> lg(callback_lock_);
    if (receiver_list_.remove_if_exists(*entry)) {
      callback_count_.fetch_sub(1, std::memory_order_release);
      return true;
    }
    return false;
  }

  // Returns the number of updated data relative to the parameter 'generation'
  unsigned updates_available(unsigned generation) const;
  unsigned initial_generation() const;

  unsigned queue_size() const { return queue_size_; }

  const char *name() const { return meta_.o_name; }
  uint8_t instance() const { return instance_; }

  /**
   * Copies data and the corresponding generation
   * from a node to the buffer provided.
   *
   * @param dst
   *   The buffer into which the data is copied.
   * @param sub_generation
   *   The generation that was copied.
   * @return bool
   *   Returns true if the data was copied.
   */
  bool Copy(void *dst, unsigned *sub_generation) const;

 private:
  friend uORBTest::UnitTest;

  static constexpr uint8_t kMaxCounterValue = 0x7F;

  const orb_metadata &meta_; /**< object metadata information */
  uint8_t *data_{nullptr};   /**< allocated object buffer */

  // Lock ordering:
  //   DeviceMaster::lock_  →  data_lock_  (node lookup)
  //   callback_lock_       →  data_lock_  (callbacks calling Copy)
  // data_lock_ and callback_lock_ are NEVER held simultaneously in Publish().
  mutable base::Mutex data_lock_{};      // protects: data_, generation_ writes
  mutable base::Mutex callback_lock_{};  // protects: receiver_list_

  intrusive_list::list<detail::CallbackEntry, &detail::CallbackEntry::node> receiver_list_;
  intrusive_list::forward_list_node device_list_node_{};

  std::atomic_uint32_t generation_{0};      /**< object generation count */
  std::atomic<uint32_t> callback_count_{0}; /**< number of registered callbacks */
  std::atomic<uint32_t> subscriber_count_{0};
  std::atomic<uint32_t> publisher_count_{0};
  std::atomic<bool> has_published_{false};  /**< true once at least one publish completed */
  std::atomic<bool> has_untracked_subscriber_{false};
  std::atomic<bool> has_untracked_publisher_{false};
  const uint16_t item_size_;   /**< cached object size */
  const uint16_t queue_size_;  /**< maximum number of elements in the queue */
  const uint16_t queue_mask_;  /**< queue_size_ - 1 (queue_size_ is power of 2) */
  const uint8_t instance_;     /**< orb multi instance identifier */

  DeviceNode(const struct orb_metadata &meta, uint8_t instance);
  ~DeviceNode();
};
}  // namespace uorb
