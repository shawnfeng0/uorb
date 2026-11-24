#include "device_node.h"

#include <cstring>
#include <new>

static inline uint16_t RoundPowOfTwo(uint16_t n) {
  if (n == 0) {
    return 1;
  }

  // Avoid is already a power of 2
  uint32_t value = n - 1;

  // Fill 1
  value |= value >> 1U;
  value |= value >> 2U;
  value |= value >> 4U;
  value |= value >> 8U;

  // Unable to round-up, take the value of round-down
  if (value == UINT16_MAX) {
    value >>= 1U;
  }

  return value + 1;
}

uorb::DeviceNode::DeviceNode(const struct orb_metadata &meta, uint8_t instance)
    : meta_(meta),
      item_size_(meta.o_size),
      queue_size_(RoundPowOfTwo(meta.o_queue_size)),
      queue_mask_(static_cast<uint16_t>(queue_size_ - 1)),
      instance_(instance) {}

uorb::DeviceNode::~DeviceNode() { delete[] data_; }

bool uorb::DeviceNode::Copy(void *dst, unsigned *sub_generation_ptr) const {
  if (!dst || !sub_generation_ptr) {
    return false;
  }

  auto &sub_generation = *sub_generation_ptr;

  base::LockGuard<base::Mutex> lg(data_lock_);

  if (!data_) {
    return false;
  }

  // If queue_size is 4 and cur_generation is 10, then 6, 7, 8, 9 are in the
  // range, and others are not.
  const unsigned cur_gen = generation_.load(std::memory_order_relaxed);

  // The subscriber already read the latest message, but nothing new was
  // published yet. Return the previous message.
  if (cur_gen == sub_generation) {
    sub_generation = cur_gen - 1;
  } else if (cur_gen - sub_generation > queue_size_) {
    // Reader is too far behind: some messages are lost
    sub_generation = cur_gen - queue_size_;
  }

  const size_t offset = static_cast<size_t>(item_size_) * (sub_generation & queue_mask_);
  memcpy(dst, data_ + offset, item_size_);

  ++sub_generation;

  return true;
}

unsigned uorb::DeviceNode::updates_available(unsigned generation) const {
  return generation_.load(std::memory_order_seq_cst) - generation;
}

bool uorb::DeviceNode::Publish(const void *data) {
  if (data == nullptr) {
    return false;
  }

  {
    base::LockGuard<base::Mutex> lg(data_lock_);

    if (nullptr == data_) {
      data_ = new (std::nothrow) uint8_t[static_cast<size_t>(item_size_) * queue_size_];

      /* failed or could not allocate */
      if (nullptr == data_) {
        return false;
      }
    }

    const unsigned cur_gen = generation_.load(std::memory_order_relaxed);
    const size_t offset = static_cast<size_t>(item_size_) * (cur_gen & queue_mask_);
    memcpy(data_ + offset, data, item_size_);

    generation_.store(cur_gen + 1, std::memory_order_seq_cst);
    if (!has_published_.load(std::memory_order_relaxed)) {
      has_published_.store(true, std::memory_order_release);
    }
  }

  // Dekker fast-path: generation_.store(seq_cst) paired with callback_count_.load(seq_cst)
  // vs RegisterCallback's callback_count_.fetch_add(seq_cst) paired with updates_available(seq_cst)
  // guarantees no missed wakeups while avoiding callback_lock_ when no callbacks are registered.
  if (callback_count_.load(std::memory_order_seq_cst) > 0) {
    base::LockGuard<base::Mutex> lg(callback_lock_);
    for (auto &entry : receiver_list_) {
      entry.on_publish(data, entry.ctx);
    }
  }

  return true;
}

void uorb::DeviceNode::add_subscriber() {
  subscriber_count_.fetch_add(1, std::memory_order_relaxed);
}

void uorb::DeviceNode::remove_subscriber() {
  subscriber_count_.fetch_sub(1, std::memory_order_relaxed);
}

void uorb::DeviceNode::mark_untracked_subscriber() {
  if (!has_untracked_subscriber_.load(std::memory_order_relaxed)) {
    has_untracked_subscriber_.store(true, std::memory_order_release);
  }
}

unsigned uorb::DeviceNode::initial_generation() const {
  const bool published = has_published_.load(std::memory_order_acquire);
  const unsigned gen = generation_.load(std::memory_order_acquire);

  if (meta_.o_durability == ORB_DURABILITY_VOLATILE) {
    return gen;
  }

  // Durable topic: if there are any previous publications allow the subscriber to read the latest one
  return (published || gen != 0) ? (gen - 1) : 0;
}

void uorb::DeviceNode::remove_publisher() {
  publisher_count_.fetch_sub(1, std::memory_order_release);
}

void uorb::DeviceNode::add_publisher() {
  publisher_count_.fetch_add(1, std::memory_order_release);
}

void uorb::DeviceNode::mark_untracked_publisher() {
  if (!has_untracked_publisher_.load(std::memory_order_relaxed)) {
    has_untracked_publisher_.store(true, std::memory_order_release);
  }
}

void uorb::DeviceNode::FillStatus(orb_status *status) const {
  status->queue_size = queue_size_;
  status->subscriber_count = static_cast<uint8_t>(subscriber_count());
  status->has_untracked_subscriber = has_untracked_subscriber_.load(std::memory_order_acquire);
  status->publisher_count = static_cast<uint8_t>(publisher_count());
  status->has_untracked_publisher = has_untracked_publisher_.load(std::memory_order_acquire);
  status->latest_data_index = generation_.load(std::memory_order_acquire);
}
