#pragma once

#include <uorb/internal/noncopyable.h>
#include <uorb/uorb.h>

#include <cerrno>

#include "base/condition_variable.h"
#include "base/intrusive_list/forward_list.h"
#include "base/mutex.h"
#include "receiver_base.h"

namespace uORBTest {
class UnitTest;
}

namespace uorb {
class DeviceMaster;

/**
 * Per-object device instance.
 */
class DeviceNode {
  friend DeviceMaster;

 public:
  UORB_NONCOPYABLE(DeviceNode);
  // Publish data to this node.
  bool Publish(const void *data);

  void add_subscriber();
  void remove_subscriber();
  uint8_t subscriber_count() const { return subscriber_count_; }
  bool has_anonymous_subscriber() const { return has_anonymous_subscriber_; }
  void mark_anonymous_subscriber() { has_anonymous_subscriber_ = true; }

  void add_publisher();
  void remove_publisher();
  uint8_t publisher_count() const { return publisher_count_; }
  bool has_anonymous_publisher() const { return has_anonymous_publisher_; }
  void mark_anonymous_publisher() { has_anonymous_publisher_ = true; }

  // Whether meta and instance are the same as the current one
  inline bool IsSameWith(const orb_metadata &meta, uint8_t instance) const {
    return IsSameWith(meta) && (instance_ == instance);
  }

  inline bool IsSameWith(const orb_metadata &meta) const { return &meta_ == &meta; }

  // add item to list of work items to schedule on node update
  bool RegisterCallback(detail::ReceiverBase *callback) {
    if (!callback) {
      errno = EINVAL;
      return false;
    }

    base::LockGuard<base::Mutex> lg(lock_);
    receiver_list_.push_front(*callback);
    return true;
  }

  // remove item from list of work items
  bool UnregisterCallback(const detail::ReceiverBase *callback) {
    base::LockGuard<base::Mutex> lg(lock_);
    return receiver_list_.remove(*callback);
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

  const orb_metadata &meta_; /**< object metadata information */
  const uint8_t instance_;   /**< orb multi instance identifier */

  uint8_t *data_{nullptr};    /**< allocated object buffer */
  const uint16_t queue_size_; /**< maximum number of elements in the queue */
  std::atomic_uint32_t generation_{0};    /**< object generation count */

  mutable base::Mutex lock_{};

  uint8_t subscriber_count_{0};
  bool has_anonymous_subscriber_{false};
  uint8_t publisher_count_{0};
  bool has_anonymous_publisher_{false};

  intrusive_list::forward_list<detail::ReceiverBase, &detail::ReceiverBase::receiver_node> receiver_list_;
  intrusive_list::forward_list_node device_list_node_{};

  DeviceNode(const struct orb_metadata &meta, uint8_t instance);
  ~DeviceNode();
};
}  // namespace uorb
