#pragma once

#include <uorb/uorb.h>

#include <atomic>

#include "base/condition_variable.h"
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
// A stack-allocated entry with on_publish == nullptr is used as an intrusive
// traversal cursor during out-of-lock callback dispatch in Publish().
struct CallbackEntry {
  orb_subscriber_callback_fn on_publish{nullptr};
  orb_callback_ctx ctx{};
  intrusive_list::list_node node{};
  uint32_t active_calls{0};       // protected by DeviceNode::callback_lock_
  bool waiting_unregister{false}; // protected by DeviceNode::callback_lock_
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

  // Pre-allocate ring buffer storage so subsequent Publish() calls perform zero allocations.
  bool EnsureBuffer();

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
  bool RegisterCallback(detail::CallbackEntry *entry);
  bool UnregisterCallback(detail::CallbackEntry *entry);

  // Returns the number of updated data relative to the parameter 'generation'
  unsigned updates_available(unsigned generation) const;
  unsigned initial_generation() const;

  unsigned queue_size() const { return queue_size_; }

  const orb_metadata &meta() const { return meta_; }
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
  static constexpr uint32_t kMaxSyncCallbackDepth = 16;

  void DispatchCallbacks(const void *data);

  // --- Group 1: Writer-only mutex (isolated from lock-free Copy() readers) ---
  // Lock ordering:
  //   DeviceMaster::lock_  →  data_lock_     (node lookup)
  //   callback_lock_       →  NEVER held across user callbacks (out-of-lock dispatch)
  //   Copy()               →  100% lock-free (never acquires data_lock_ or callback_lock_)
  // data_lock_ only serializes concurrent writers in Publish(). Keeping it at
  // the front prevents writer lock-contention cache traffic from invalidating
  // the reader-hot fields in Group 3 (buffer_, slot geometry, generation_).
  mutable base::Mutex data_lock_{};  // protects: buffer_ allocation & writes, generation_ writes
  std::atomic<uint64_t> last_publish_time_us_{0}; /**< monotonic timestamp of last Publish() */

  // --- Group 2: Cold / low-frequency callback & lifecycle fields ---
  // Naturally separates Group 1 (writer mutex) from Group 3 (reader-hot block)
  // across cache lines without adding alignas() padding overhead on MCUs.
  mutable base::Mutex callback_lock_{};  // protects: receiver_list_, active_calls
  base::ConditionVariable callback_cv_{};
  intrusive_list::list<detail::CallbackEntry, &detail::CallbackEntry::node> receiver_list_;
  intrusive_list::forward_list_node device_list_node_{};
  std::atomic<uint32_t> subscriber_count_{0};
  std::atomic<uint32_t> publisher_count_{0};
  std::atomic<uint32_t> callback_count_{0}; /**< number of registered callbacks */
  std::atomic<bool> has_untracked_subscriber_{false};
  std::atomic<bool> has_untracked_publisher_{false};

  // --- Group 3: Reader-hot block (all fields touched by lock-free Copy()) ---
  // Fits in a single cache line on 64B/128B CPUs, isolated from data_lock_.
  const orb_metadata &meta_; /**< object metadata information */

  // Lock-free reader / mutex-serialized writer slot storage (single allocation):
  //   [0 .. seq_offset_ - 1]         : physical_slots_ * payload_words_ std::atomic<size_t>
  //   [seq_offset_ .. total_bytes-1] : physical_slots_ std::atomic<uint32_t> per-slot seq tags
  // physical_slots_ == 2 for queue_size_ == 1 (double buffer) and == queue_size_
  // for queue_size_ >= 2 (zero extra slots).
  std::atomic<uint8_t *> buffer_{nullptr};

  const uint32_t physical_slots_; /**< (queue_size_ == 1) ? 2 : queue_size_ (power of 2, >= 2) */
  const uint32_t physical_mask_;  /**< physical_slots_ - 1 */
  const size_t payload_words_;    /**< ceil(item_size_ / sizeof(size_t)) */
  const size_t seq_offset_;       /**< byte offset of per-slot uint32_t seq array in buffer_ */
  std::atomic_uint32_t generation_{0}; /**< object generation count */
  const uint16_t item_size_;      /**< cached object size */
  const uint16_t queue_size_;     /**< logical maximum number of elements in the queue */
  const uint8_t instance_;        /**< orb multi instance identifier */
  std::atomic<bool> has_published_{false}; /**< true once at least one publish completed */

  DeviceNode(const struct orb_metadata &meta, uint8_t instance);
  ~DeviceNode();
};
}  // namespace uorb
