#pragma once

#include <uorb/uorb.h>

#include <atomic>
#include <cerrno>

#include "base/intrusive_list/forward_list.h"
#include "base/mutex.h"

namespace uORBTest {
class UnitTest;
}

namespace uorb {
namespace detail {

// Function-pointer based callback entry for DeviceNode's publish notification.
// Replaces the old virtual ReceiverBase — simpler, no inheritance needed.
struct CallbackEntry {
  void (*on_publish)(const void *msg, void *ctx);
  void *ctx;
  intrusive_list::forward_list_node node{};
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
  uint16_t subscriber_count() const { return subscriber_count_; }
  void mark_untracked_subscriber();

  void add_publisher();
  void remove_publisher();
  uint16_t publisher_count() const { return publisher_count_; }
  void mark_untracked_publisher();

  // Whether meta and instance are the same as the current one
  inline bool IsSameWith(const orb_metadata &meta, uint8_t instance) const {
    return IsSameWith(meta) && (instance_ == instance);
  }

  inline bool IsSameWith(const orb_metadata &meta) const { return &meta_ == &meta; }

  // add item to list of work items to schedule on node update
  bool RegisterCallback(detail::CallbackEntry *entry) {
    if (!entry) {
      errno = EINVAL;
      return false;
    }

    base::LockGuard<base::Mutex> lg(callback_lock_);
    receiver_list_.push_front(*entry);
    return true;
  }

  bool UnregisterCallback(const detail::CallbackEntry *entry) {
    base::LockGuard<base::Mutex> lg(callback_lock_);
    return receiver_list_.remove(*entry);
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

  // Lock order invariant: DeviceMaster::lock_ → DeviceNode::data_lock_ → DeviceNode::callback_lock_
  // Never acquire in reverse order.
  mutable base::Mutex data_lock_{};      // protects: data_, generation_, subscriber_count_,
                                         //          publisher_count_, has_untracked_subscriber_,
                                         //          has_untracked_publisher_
  mutable base::Mutex callback_lock_{};  // protects: receiver_list_

  intrusive_list::forward_list<detail::CallbackEntry, &detail::CallbackEntry::node> receiver_list_;
  intrusive_list::forward_list_node device_list_node_{};

  std::atomic_uint32_t generation_{0}; /**< object generation count */
  const uint16_t queue_size_;          /**< maximum number of elements in the queue */
  uint8_t subscriber_count_ : 7;
  bool has_untracked_subscriber_ : 1;
  uint8_t publisher_count_ : 7;
  bool has_untracked_publisher_ : 1;
  const uint8_t instance_; /**< orb multi instance identifier */

  DeviceNode(const struct orb_metadata &meta, uint8_t instance);
  ~DeviceNode();
};
}  // namespace uorb
