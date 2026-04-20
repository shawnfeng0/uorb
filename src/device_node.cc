#include "device_node.h"

#include <cerrno>
#include <cstring>

uorb::DeviceNode::DeviceNode(const struct orb_metadata &meta, uint8_t instance)
    : meta_(meta), instance_(instance), queue_size_(meta.o_queue_size ? meta.o_queue_size : 1) {}

uorb::DeviceNode::~DeviceNode() { delete[] data_; }

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

  // Modulo is used instead of bitwise-AND to support non-power-of-two queue
  // sizes.  This trades a small per-copy CPU cost for exact memory allocation
  // (no rounding-up waste).
  memcpy(dst, data_ + (meta_.o_size * (sub_generation % queue_size_)), meta_.o_size);

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

  if (nullptr == data_) {
    data_ = new uint8_t[meta_.o_size * queue_size_];

    /* failed or could not allocate */
    if (nullptr == data_) {
      errno = ENOMEM;
      return false;
    }
  }

  memcpy(data_ + (meta_.o_size * (generation_ % queue_size_)), (const char *)data, meta_.o_size);

  generation_++;

  for (auto &receiver : receiver_list_) {
    receiver.notify_all();
  }

  return true;
}

void uorb::DeviceNode::add_subscriber() {
  base::LockGuard<base::Mutex> lg(lock_);
  subscriber_count_++;
}

void uorb::DeviceNode::remove_subscriber() {
  base::LockGuard<base::Mutex> lg(lock_);
  subscriber_count_--;
}

unsigned uorb::DeviceNode::initial_generation() const {
  base::LockGuard<base::Mutex> lg(lock_);

  // If there any previous publications allow the subscriber to read them
  return generation_ - (data_ ? 1 : 0);
}

void uorb::DeviceNode::remove_publisher() {
  base::LockGuard<base::Mutex> lg(lock_);
  publisher_count_--;
}

void uorb::DeviceNode::add_publisher() {
  base::LockGuard<base::Mutex> lg(lock_);
  publisher_count_++;
}
