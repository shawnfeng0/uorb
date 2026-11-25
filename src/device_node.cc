#include "device_node.h"

#include <cstring>
#include <new>
#include <type_traits>

#include "base/allocator.h"

static_assert(alignof(std::atomic<size_t>) >= alignof(std::atomic<uint32_t>),
              "payload alignment must satisfy trailing uint32_t seq array alignment");
static_assert(sizeof(std::atomic<size_t>) % alignof(std::atomic<uint32_t>) == 0,
              "payload byte stride must keep trailing uint32_t seq array aligned");
static_assert(std::is_trivially_destructible<std::atomic<size_t>>::value,
              "atomic<size_t> must be trivially destructible");
static_assert(std::is_trivially_destructible<std::atomic<uint32_t>>::value,
              "atomic<uint32_t> must be trivially destructible");

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

static inline void WritePayloadAtomic(std::atomic<size_t> *dst_words,
                                      const void *src,
                                      size_t byte_size) {
  const auto *bytes = static_cast<const uint8_t *>(src);
  const size_t full_words = byte_size / sizeof(size_t);
  for (size_t i = 0; i < full_words; ++i) {
    size_t w;
    std::memcpy(&w, bytes + i * sizeof(size_t), sizeof(size_t));
    dst_words[i].store(w, std::memory_order_relaxed);
  }
  const size_t rem = byte_size % sizeof(size_t);
  if (rem > 0) {
    size_t w = 0;
    std::memcpy(&w, bytes + full_words * sizeof(size_t), rem);
    dst_words[full_words].store(w, std::memory_order_relaxed);
  }
}

static inline void ReadPayloadAtomic(void *dst,
                                     const std::atomic<size_t> *src_words,
                                     size_t byte_size) {
  auto *bytes = static_cast<uint8_t *>(dst);
  const size_t full_words = byte_size / sizeof(size_t);
  for (size_t i = 0; i < full_words; ++i) {
    const size_t w = src_words[i].load(std::memory_order_relaxed);
    std::memcpy(bytes + i * sizeof(size_t), &w, sizeof(size_t));
  }
  const size_t rem = byte_size % sizeof(size_t);
  if (rem > 0) {
    const size_t w = src_words[full_words].load(std::memory_order_relaxed);
    std::memcpy(bytes + full_words * sizeof(size_t), &w, rem);
  }
}

uorb::DeviceNode::DeviceNode(const struct orb_metadata &meta, uint8_t instance)
    : meta_(meta),
      physical_slots_(RoundPowOfTwo(meta.o_queue_size) == 1u
                          ? 2u
                          : static_cast<uint32_t>(RoundPowOfTwo(meta.o_queue_size))),
      physical_mask_(physical_slots_ - 1u),
      payload_words_((static_cast<size_t>(meta.o_size) + sizeof(size_t) - 1u) / sizeof(size_t)),
      seq_offset_(static_cast<size_t>(physical_slots_) * payload_words_ *
                  sizeof(std::atomic<size_t>)),
      item_size_(meta.o_size),
      queue_size_(RoundPowOfTwo(meta.o_queue_size)),
      instance_(instance) {}

uorb::DeviceNode::~DeviceNode() {
  base::Allocator::Free(buffer_.load(std::memory_order_relaxed));
  base::Allocator::Free(loan_buffer_.load(std::memory_order_relaxed));
}

bool uorb::DeviceNode::Copy(void *dst, unsigned *sub_generation_ptr) const {
  if (!dst || !sub_generation_ptr) {
    return false;
  }

  const uint8_t *buf = buffer_.load(std::memory_order_acquire);
  if (!buf) {
    return false;
  }

  const auto *payload_base = reinterpret_cast<const std::atomic<size_t> *>(buf);
  const auto *seq_base = reinterpret_cast<const std::atomic<uint32_t> *>(buf + seq_offset_);

  bool has_conflict = false;
  uint32_t conflict_cur_gen = 0;

  for (;;) {
    const uint32_t cur_gen = generation_.load(std::memory_order_acquire);
    if (cur_gen == 0 && !has_published_.load(std::memory_order_acquire)) {
      return false;
    }

    // If queue_size is 4 and cur_generation is 10, then 6, 7, 8, 9 are in the
    // logical readable range.
    uint32_t desired_gen = *sub_generation_ptr;
    if (cur_gen == desired_gen) {
      desired_gen = cur_gen - 1u;
    } else if (cur_gen - desired_gen > queue_size_) {
      desired_gen = cur_gen - queue_size_;
    }

    // When physical_slots_ == queue_size_ (queue_size_ >= 2), the writer's
    // in-progress slot aliases the oldest entry (cur_gen - queue_size_).
    // If we already observed that slot being overwritten for this cur_gen,
    // advance by 1 so a high-priority reader preempting a low-priority writer
    // on a single-core RTOS reads the next clean slot without spinning.
    if (has_conflict && cur_gen == conflict_cur_gen &&
        (cur_gen - desired_gen == physical_slots_)) {
      ++desired_gen;
    }

    const size_t slot = desired_gen & physical_mask_;

    // Each generation g encodes a unique even tag ((g << 1) + 2) and odd
    // in-progress tag ((g << 1) + 1). Matching seq1 == seq2 == expected_seq
    // simultaneously proves:
    //   1) desired_gen finished publishing before ReadPayloadAtomic (acquire on seq1),
    //   2) no newer write touched this slot during ReadPayloadAtomic (acquire fence + seq2),
    //   3) no ABA slot reuse occurred before seq1 (no second generation_ load).
    const uint32_t expected_seq = (desired_gen << 1u) + 2u;
    const uint32_t seq1 = seq_base[slot].load(std::memory_order_acquire);
    if (seq1 != expected_seq) {
      has_conflict = (cur_gen - desired_gen == physical_slots_);
      conflict_cur_gen = cur_gen;
      continue;
    }

    ReadPayloadAtomic(dst, payload_base + slot * payload_words_, item_size_);

    // Prevent payload loads from reordering past the closing seq2 validation.
    std::atomic_thread_fence(std::memory_order_acquire);
    const uint32_t seq2 = seq_base[slot].load(std::memory_order_relaxed);
    if (seq2 != expected_seq) {
      has_conflict = (cur_gen - desired_gen == physical_slots_);
      conflict_cur_gen = cur_gen;
      continue;
    }

    *sub_generation_ptr = desired_gen + 1u;
    return true;
  }
}

unsigned uorb::DeviceNode::updates_available(unsigned generation) const {
  return generation_.load(std::memory_order_seq_cst) - generation;
}

bool uorb::DeviceNode::EnsureBuffer() {
  if (buffer_.load(std::memory_order_acquire) != nullptr) {
    return true;
  }

  base::LockGuard<base::Mutex> lg(data_lock_);
  uint8_t *buf = buffer_.load(std::memory_order_relaxed);
  if (buf != nullptr) {
    return true;
  }

  const size_t total_bytes =
      seq_offset_ + static_cast<size_t>(physical_slots_) * sizeof(std::atomic<uint32_t>);
  void *raw = base::Allocator::Alloc(total_bytes);
  if (nullptr == raw) {
    return false;
  }

  buf = static_cast<uint8_t *>(raw);
  auto *payload_init = reinterpret_cast<std::atomic<size_t> *>(buf);
  const size_t total_payload_words = static_cast<size_t>(physical_slots_) * payload_words_;
  for (size_t i = 0; i < total_payload_words; ++i) {
    new (payload_init + i) std::atomic<size_t>(0);
  }
  auto *seq_init = reinterpret_cast<std::atomic<uint32_t> *>(buf + seq_offset_);
  for (uint32_t i = 0; i < physical_slots_; ++i) {
    new (seq_init + i) std::atomic<uint32_t>(0);
  }

  buffer_.store(buf, std::memory_order_release);
  return true;
}

orb_err uorb::DeviceNode::Loan(void **out_msg) {
  if (!out_msg) {
    return ORB_ERR_INVALID;
  }
  *out_msg = nullptr;

  if (!EnsureBuffer()) {
    return ORB_ERR_NO_MEM;
  }

  uint8_t *lbuf = loan_buffer_.load(std::memory_order_acquire);
  if (!lbuf) {
    base::LockGuard<base::Mutex> lg(data_lock_);
    lbuf = loan_buffer_.load(std::memory_order_relaxed);
    if (!lbuf) {
      void *raw = base::Allocator::Alloc(item_size_);
      if (!raw) {
        return ORB_ERR_NO_MEM;
      }
      lbuf = static_cast<uint8_t *>(raw);
      loan_buffer_.store(lbuf, std::memory_order_release);
    }
  }

  bool expected = false;
  if (!loan_in_use_.compare_exchange_strong(expected, true, std::memory_order_acq_rel)) {
    return ORB_ERR_BUSY;
  }

  std::memset(lbuf, 0, item_size_);
  *out_msg = lbuf;
  return ORB_OK;
}

orb_err uorb::DeviceNode::PublishLoaned(void *loaned_msg) {
  uint8_t *lbuf = loan_buffer_.load(std::memory_order_acquire);
  if (!loaned_msg || loaned_msg != lbuf || !loan_in_use_.load(std::memory_order_acquire)) {
    return ORB_ERR_INVALID;
  }

  const bool ok = Publish(lbuf);
  loan_in_use_.store(false, std::memory_order_release);
  return ok ? ORB_OK : ORB_ERR_NO_MEM;
}

void uorb::DeviceNode::DiscardLoan(void *loaned_msg) {
  uint8_t *lbuf = loan_buffer_.load(std::memory_order_acquire);
  if (loaned_msg && loaned_msg == lbuf) {
    loan_in_use_.store(false, std::memory_order_release);
  }
}

bool uorb::DeviceNode::Publish(const void *data) {
  if (data == nullptr) {
    return false;
  }

  if (buffer_.load(std::memory_order_acquire) == nullptr) {
    if (!EnsureBuffer()) {
      return false;
    }
  }

  {
    base::LockGuard<base::Mutex> lg(data_lock_);

    uint8_t *buf = buffer_.load(std::memory_order_relaxed);
    auto *payload_base = reinterpret_cast<std::atomic<size_t> *>(buf);
    auto *seq_base = reinterpret_cast<std::atomic<uint32_t> *>(buf + seq_offset_);

    const uint32_t cur_gen = generation_.load(std::memory_order_relaxed);
    const size_t slot = cur_gen & physical_mask_;

    // Per-slot SeqLock write protocol (single writer serialized by data_lock_):
    // 1) Mark odd + release fence so no payload store precedes the odd marker.
    const uint32_t odd_seq = (cur_gen << 1u) + 1u;
    const uint32_t even_seq = (cur_gen << 1u) + 2u;
    seq_base[slot].store(odd_seq, std::memory_order_relaxed);
    std::atomic_thread_fence(std::memory_order_release);

    // 2) Write payload using relaxed atomic word stores (C++14/TSan data-race free).
    WritePayloadAtomic(payload_base + slot * payload_words_, data, item_size_);

    // 3) Mark even with release semantics before advancing generation_.
    seq_base[slot].store(even_seq, std::memory_order_release);

    last_publish_time_us_.store(orb_absolute_time_us(), std::memory_order_release);
    generation_.store(cur_gen + 1u, std::memory_order_seq_cst);
    if (!has_published_.load(std::memory_order_relaxed)) {
      has_published_.store(true, std::memory_order_release);
    }
  }

  // Dekker fast-path: generation_.store(seq_cst) paired with waiter_count_.load(seq_cst)
  // vs RegisterWaiter's waiter_count_.fetch_add(seq_cst) paired with updates_available(seq_cst)
  // guarantees no missed wakeups while avoiding waiter_lock_ when no waiters are registered.
  if (waiter_count_.load(std::memory_order_seq_cst) > 0) {
    NotifyWaiters();
  }

  return true;
}

bool uorb::DeviceNode::RegisterWaiter(detail::WaiterEntry *entry) {
  if (!entry || !entry->source) {
    return false;
  }

  base::LockGuard<base::Mutex> lg(waiter_lock_);
  waiter_list_.push_front(*entry);
  waiter_count_.fetch_add(1, std::memory_order_seq_cst);
  return true;
}

bool uorb::DeviceNode::UnregisterWaiter(detail::WaiterEntry *entry) {
  if (!entry) {
    return false;
  }

  base::LockGuard<base::Mutex> lg(waiter_lock_);
  if (!waiter_list_.remove_if_exists(*entry)) {
    return false;
  }
  waiter_count_.fetch_sub(1, std::memory_order_release);
  return true;
}

void uorb::DeviceNode::NotifyWaiters() {
  base::LockGuard<base::Mutex> lg(waiter_lock_);
  for (auto &entry : waiter_list_) {
    entry.source->notify_waiters();
  }
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
  status->subscriber_count = subscriber_count();
  status->has_untracked_subscriber = has_untracked_subscriber_.load(std::memory_order_acquire);
  status->publisher_count = publisher_count();
  status->has_untracked_publisher = has_untracked_publisher_.load(std::memory_order_acquire);
  status->latest_data_index = generation_.load(std::memory_order_acquire);
  status->last_publish_time_us = last_publish_time_us_.load(std::memory_order_acquire);
}
