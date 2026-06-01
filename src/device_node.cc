#include "device_node.h"

#include <cerrno>
#include <cstddef>
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
      queue_size_(RoundPowOfTwo(meta.o_queue_size)),
      subscriber_count_(0),
      has_untracked_subscriber_(false),
      publisher_count_(0),
      has_untracked_publisher_(false),
      instance_(instance) {}

uorb::DeviceNode::~DeviceNode() { DestroyQueue(); }

void uorb::DeviceNode::ResetQueueForTesting() {
  DestroyQueue();
  generation_ = 0;
  constructed_slot_count_ = 0;
}

bool uorb::DeviceNode::AllocateQueueStorage() {
  if (!meta_.o_operations || !meta_.o_operations->copy_construct || !meta_.o_operations->copy_assign ||
      !meta_.o_operations->destroy) {
    errno = EINVAL;
    return false;
  }

  try {
    data_ = static_cast<unsigned char *>(::operator new(meta_.o_size * queue_size_));
  } catch (const std::bad_alloc &) {
    errno = ENOMEM;
    return false;
  }

  return true;
}

void uorb::DeviceNode::DestroyQueue() {
  if (data_) {
    const uint16_t constructed_slot_count = ConstructedSlotCount();
    for (uint16_t slot_index = 0; slot_index < constructed_slot_count; ++slot_index) {
      meta_.o_operations->destroy(SlotAddress(slot_index));
    }
  }

  ::operator delete(data_);
  data_ = nullptr;
  constructed_slot_count_ = 0;
}

void *uorb::DeviceNode::SlotAddress(unsigned index) const { return data_ + (meta_.o_size * index); }

bool uorb::DeviceNode::IsSlotConstructed(unsigned slot_index) const { return slot_index < constructed_slot_count_; }

uint16_t uorb::DeviceNode::ConstructedSlotCount() const { return constructed_slot_count_; }

bool uorb::DeviceNode::CopyIntoSlot(unsigned index, const void *data) {
  const auto slot_index = index & (queue_size_ - 1);
  void *slot_address = SlotAddress(slot_index);

  try {
    if (IsSlotConstructed(slot_index)) {
      if (!meta_.o_operations->copy_assign(slot_address, data)) {
        errno = EFAULT;
        return false;
      }
    } else if (!meta_.o_operations->copy_construct(slot_address, data)) {
      errno = EFAULT;
      return false;
    } else if (constructed_slot_count_ < queue_size_) {
      ++constructed_slot_count_;
    }
  } catch (const std::bad_alloc &) {
    errno = ENOMEM;
    return false;
  } catch (...) {
    errno = EFAULT;
    return false;
  }

  return true;
}

bool uorb::DeviceNode::Copy(void *dst, unsigned *sub_generation_ptr) const {
  if (!dst || !sub_generation_ptr || !data_) {
    return false;
  }

  auto &sub_generation = *sub_generation_ptr;

  base::LockGuard<base::Mutex> lg(lock_);

  // If queue_size is 4 and cur_generation is 10, then 6, 7, 8, 9 are in the
  // range, and others are not.

  // The subscriber already read the latest message, but nothing new was
  // published yet. Return the previous message */
  if (generation_ == sub_generation) {
    sub_generation = generation_ - 1;
  } else if (generation_ - sub_generation > queue_size_) {
    // Reader is too far behind: some messages are lost
    sub_generation = generation_ - queue_size_;
  }

  const unsigned slot_index = sub_generation & (queue_size_ - 1);
  if (!IsSlotConstructed(slot_index)) {
    return false;
  }

  try {
    if (!meta_.o_operations->copy_assign(dst, SlotAddress(slot_index))) {
      errno = EFAULT;
      return false;
    }
  } catch (const std::bad_alloc &) {
    errno = ENOMEM;
    return false;
  } catch (...) {
    errno = EFAULT;
    return false;
  }

  ++sub_generation;

  return true;
}

unsigned uorb::DeviceNode::updates_available(unsigned generation) const { return generation_ - generation; }

bool uorb::DeviceNode::Publish(const void *data) {
  if (data == nullptr) {
    errno = EFAULT;
    return false;
  }

  base::LockGuard<base::Mutex> lg(lock_);

  if (nullptr == data_ && !AllocateQueueStorage()) {
    return false;
  }

  if (!CopyIntoSlot(generation_, data)) {
    return false;
  }

  generation_++;

  for (auto &receiver : receiver_list_) {
    receiver.notify_all();
  }

  return true;
}

void uorb::DeviceNode::add_subscriber() {
  base::LockGuard<base::Mutex> lg(lock_);
  if (subscriber_count_ < kMaxCounterValue) {
    subscriber_count_++;
  }
}

void uorb::DeviceNode::remove_subscriber() {
  base::LockGuard<base::Mutex> lg(lock_);
  if (subscriber_count_ > 0) {
    subscriber_count_--;
  }
}

unsigned uorb::DeviceNode::initial_generation() const {
  base::LockGuard<base::Mutex> lg(lock_);

  // If there any previous publications allow the subscriber to read them
  return generation_ - (data_ ? 1 : 0);
}

void uorb::DeviceNode::remove_publisher() {
  base::LockGuard<base::Mutex> lg(lock_);
  if (publisher_count_ > 0) {
    publisher_count_--;
  }
}

void uorb::DeviceNode::add_publisher() {
  base::LockGuard<base::Mutex> lg(lock_);
  if (publisher_count_ < kMaxCounterValue) {
    publisher_count_++;
  }
}
