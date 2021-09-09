#include "device_node.h"

#include <cerrno>
#include <cstring>

static inline bool IsInRange(unsigned left, unsigned value, unsigned right) {
  if (right == left) {
    // When the two limits are equal, the interval is considered to have only
    // one data.
    return value == left;
  } else if (right > left) {
    // Normal
    return (left <= value) && (value <= right);
  } else {
    // Maybe the data overflowed and a wraparound occurred
    return (left <= value) || (value <= right);
  }
}

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

uorb::DeviceNode::DeviceNode(const struct orb_metadata &meta, uint8_t instance,
                             unsigned int queue_size)
    : meta_(meta),
      instance_(instance),
      queue_size_(
          RoundPowOfTwo(queue_size > UINT16_MAX ? UINT16_MAX : queue_size)) {}

uorb::DeviceNode::~DeviceNode() { delete[] data_; }

bool uorb::DeviceNode::Copy(void *dst, unsigned *sub_generation_ptr) const {
  if (!dst || !sub_generation_ptr || !data_) {
    return false;
  }

  auto &sub_generation = *sub_generation_ptr;

  base::ReaderLockGuard lg(lock_);

  /* The subscriber already read the latest message, but nothing new was
   * published yet. Return the previous message */
  if (generation_ == sub_generation) {
    --sub_generation;
  }

  // Before the queue is filled, if the incoming sub_generation points to
  // unpublished data, invalid data will be obtained. Such incorrect usage
  // should not be handled.
  const unsigned queue_start = generation_ - queue_size_;

  // If queue_size is 3 and cur_generation is 10, then 7, 8, 9 are in the
  // range, and others are not.
  if (!IsInRange(queue_start, sub_generation, generation_ - 1)) {
    // Reader is too far behind: some messages are lost
    sub_generation = queue_start;
  }

  memcpy(dst, data_ + (meta_.o_size * (sub_generation % queue_size_)),
         meta_.o_size);

  ++sub_generation;

  return true;
}

unsigned uorb::DeviceNode::updates_available(unsigned generation) const {
  return generation_ - generation;
}

bool uorb::DeviceNode::Publish(const void *data) {
  if (data == nullptr) {
    errno = EFAULT;
    return false;
  }

  base::WriterLockGuard lg(lock_);

  if (nullptr == data_) {
    data_ = new uint8_t[meta_.o_size * queue_size_];

    /* failed or could not allocate */
    if (nullptr == data_) {
      errno = ENOMEM;
      return false;
    }
  }

  memcpy(data_ + (meta_.o_size * (generation_ % queue_size_)),
         (const char *)data, meta_.o_size);

  generation_++;

  for (auto callback : callbacks_) {
    (*callback).Notify();
  }

  return true;
}

void uorb::DeviceNode::add_subscriber() {
  base::WriterLockGuard lg(lock_);
  subscriber_count_++;
}

void uorb::DeviceNode::remove_subscriber() {
  base::WriterLockGuard lg(lock_);
  subscriber_count_--;
}

bool uorb::DeviceNode::set_queue_size(unsigned int queue_size) {
  base::WriterLockGuard lg(lock_);

  if (data_ || queue_size_ > queue_size) {
    return false;
  }

  queue_size_ =
      RoundPowOfTwo(queue_size > UINT16_MAX ? UINT16_MAX : queue_size);
  return true;
}

unsigned uorb::DeviceNode::initial_generation() const {
  base::WriterLockGuard lg(lock_);

  // If there any previous publications allow the subscriber to read them
  return generation_ - (data_ ? 1 : 0);
}

void uorb::DeviceNode::remove_publisher() {
  base::WriterLockGuard lg(lock_);
  publisher_count_--;
}

void uorb::DeviceNode::add_publisher() {
  base::WriterLockGuard lg(lock_);
  publisher_count_++;
}
