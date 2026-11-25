#pragma once

#include <uorb/publication.h>
#include <uorb/uorb.h>

namespace uorb {

/**
 * Lightweight C++ publisher wrapper for multi-instance topics.
 *
 * Use this when multiple independent publishers need to publish the same topic
 * type on different instances. The allocated instance can be read through
 * instance() after the first successful Publish().
 */
template <const orb_metadata &meta>
class PublicationMulti {
 public:
  using ValueType = typename msg::TypeMap<meta>::type;
  using Type = ValueType;

  PublicationMulti() noexcept = default;
  PublicationMulti(const PublicationMulti &) = delete;
  PublicationMulti(PublicationMulti &&) = delete;
  PublicationMulti &operator=(const PublicationMulti &) = delete;
  PublicationMulti &operator=(PublicationMulti &&) = delete;
  ~PublicationMulti() { if (handle_._handle) orb_pub_destroy(&handle_); }

  /// Eagerly allocate a topic instance and create the publication handle (idempotent).
  orb_err Advertise() {
    if (handle_._handle) {
      return ORB_OK;
    }
    return orb_pub_create_multi(&handle_, &meta, &instance_);
  }

  /// Return the underlying C publication handle (advertising lazily if needed).
  orb_pub_t handle() {
    return (Advertise() == ORB_OK) ? handle_ : orb_pub_t{nullptr};
  }

  /**
   * Publish the struct
   * @param data The uORB message struct we are updating.
   * @return ORB_OK on success, error code otherwise.
   */
  orb_err Publish(const ValueType &data) {
    orb_err err = Advertise();
    if (err != ORB_OK) return err;

    return orb_pub_publish(handle_, &data);
  }

  /// Borrow a topic-owned message buffer for stack-free in-place construction.
  LoanedMessage<meta> Loan() {
    if (Advertise() != ORB_OK) {
      return LoanedMessage<meta>{};
    }
    void *raw = nullptr;
    if (orb_pub_loan(handle_, &raw) != ORB_OK) {
      return LoanedMessage<meta>{};
    }
    return LoanedMessage<meta>(handle_, static_cast<ValueType *>(raw));
  }

  unsigned instance() const { return instance_; }

 private:
  orb_pub_t handle_ = ORB_PUB_INITIALIZER;
  unsigned instance_{0};
};

/**
 * Multi-instance publisher wrapper with embedded message storage.
 *
 * This is the recommended C++ helper for multi-instance publishing: fill
 * data(), call Publish(), and use instance() to learn the assigned instance.
 */
template <const orb_metadata &meta>
class PublicationMultiData : public PublicationMulti<meta> {
 public:
  using ValueType = typename msg::TypeMap<meta>::type;
  using Type = ValueType;

  PublicationMultiData() noexcept = default;

  using PublicationMulti<meta>::Publish;

  ValueType &data() { return data_; }
  const ValueType &data() const { return data_; }

  // Publishes the embedded struct.
  orb_err Publish() { return PublicationMulti<meta>::Publish(data_); }

 private:
  ValueType data_{};
};

}  // namespace uorb
