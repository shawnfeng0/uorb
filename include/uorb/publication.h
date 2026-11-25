#pragma once

#include <uorb/uorb.h>

namespace uorb {

/**
 * Move-only RAII wrapper for a loaned message buffer obtained via
 * Publication::Loan() or PublicationMulti::Loan().
 *
 * Allows filling large messages in-place without allocating them on the thread
 * stack. Automatically discards the loan on destruction if Publish() was not
 * called.
 */
template <const orb_metadata &meta>
class LoanedMessage {
 public:
  using ValueType = typename msg::TypeMap<meta>::type;
  using Type = ValueType;

  LoanedMessage() noexcept = default;
  LoanedMessage(orb_pub_t pub, Type *msg) noexcept : pub_(pub), msg_(msg) {}

  LoanedMessage(const LoanedMessage &) = delete;
  LoanedMessage &operator=(const LoanedMessage &) = delete;

  LoanedMessage(LoanedMessage &&other) noexcept : pub_(other.pub_), msg_(other.msg_) {
    other.pub_ = ORB_PUB_INITIALIZER;
    other.msg_ = nullptr;
  }

  LoanedMessage &operator=(LoanedMessage &&other) noexcept {
    if (this != &other) {
      Discard();
      pub_ = other.pub_;
      msg_ = other.msg_;
      other.pub_ = ORB_PUB_INITIALIZER;
      other.msg_ = nullptr;
    }
    return *this;
  }

  ~LoanedMessage() { Discard(); }

  explicit operator bool() const noexcept { return msg_ != nullptr; }
  Type *get() noexcept { return msg_; }
  Type *operator->() noexcept { return msg_; }
  Type &operator*() noexcept { return *msg_; }

  orb_err Publish() {
    if (!msg_) return ORB_ERR_INVALID;
    Type *m = msg_;
    orb_pub_t p = pub_;
    msg_ = nullptr;
    pub_ = ORB_PUB_INITIALIZER;
    return orb_pub_publish_loaned(p, m);
  }

  void Discard() {
    if (msg_) {
      orb_pub_discard_loan(pub_, msg_);
      msg_ = nullptr;
      pub_ = ORB_PUB_INITIALIZER;
    }
  }

 private:
  orb_pub_t pub_ = ORB_PUB_INITIALIZER;
  Type *msg_{nullptr};
};

/**
 * Lightweight C++ publisher wrapper for a single topic instance.
 *
 * Use Publication when the message storage is managed by the caller. For the
 * common case where the publisher owns one reusable message object, prefer
 * PublicationData.
 */
template <const orb_metadata &meta>
class Publication {
 public:
  using ValueType = typename msg::TypeMap<meta>::type;
  using Type = ValueType;

  Publication() noexcept = default;
  Publication(const Publication &) = delete;
  Publication(Publication &&) = delete;
  Publication &operator=(const Publication &) = delete;
  Publication &operator=(Publication &&) = delete;
  ~Publication() { if (handle_._handle) orb_pub_destroy(&handle_); }

  /// Eagerly create the underlying publication handle (idempotent).
  orb_err Advertise() {
    if (handle_._handle) {
      return ORB_OK;
    }
    return orb_pub_create(&handle_, &meta);
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
  /// Returns an empty LoanedMessage (operator bool() == false) on error / busy.
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

 private:
  orb_pub_t handle_ = ORB_PUB_INITIALIZER;
};

/**
 * Publisher wrapper with embedded message storage.
 *
 * This is the recommended C++ helper for regular single-instance publishing:
 * fill data(), then call Publish().
 */
template <const orb_metadata &meta>
class PublicationData : public Publication<meta> {
 public:
  using ValueType = typename msg::TypeMap<meta>::type;
  using Type = ValueType;

  PublicationData() noexcept = default;

  using Publication<meta>::Publish;

  ValueType &data() { return data_; }
  const ValueType &data() const { return data_; }

  // Publishes the embedded struct.
  orb_err Publish() { return Publication<meta>::Publish(data_); }

 private:
  ValueType data_{};
};

}  // namespace uorb
