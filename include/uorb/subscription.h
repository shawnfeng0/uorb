#pragma once

#include <uorb/publication.h>
#include <uorb/uorb.h>

namespace uorb {

namespace detail {

/// CRTP base for subscription wrappers.
///
/// Derived classes can hide Updated() and Copy() to customize behavior.
/// Update() dispatches through CRTP so it calls the Derived versions
/// of Updated() and Copy() without virtual overhead.
template <const orb_metadata &meta, typename Derived>
class SubscriptionBase {
 public:
  using ValueType = typename msg::TypeMap<meta>::type;

  SubscriptionBase(const SubscriptionBase &) = delete;
  SubscriptionBase(SubscriptionBase &&) = delete;
  SubscriptionBase &operator=(const SubscriptionBase &) = delete;
  SubscriptionBase &operator=(SubscriptionBase &&) = delete;

  explicit SubscriptionBase(uint8_t instance = 0) noexcept : instance_(instance) {}

  ~SubscriptionBase() { if (handle_._handle) orb_subscriber_destroy(&handle_); }

  bool Subscribe() {
    if (handle_._handle) {
      return true;
    }
    return orb_subscriber_create_multi(&handle_, &meta, instance_) == ORB_OK;
  }

  orb_subscriber_t *handle() { return Subscribe() ? &handle_ : nullptr; }

  /// Check if there is a new update.
  bool Updated() { return Subscribe() && orb_subscriber_check_update(&handle_); }

  /// Update the struct if new data is available.
  /// Dispatches to Derived::Updated() and Derived::Copy() via CRTP.
  bool Update(ValueType *dst) {
    return static_cast<Derived &>(*this).Updated() &&
           static_cast<Derived &>(*this).Copy(dst);
  }
  bool Update(ValueType &dst) { return Update(&dst); }

  /// Copy the struct unconditionally.
  bool Copy(ValueType *dst) { return Subscribe() && orb_subscriber_copy(&handle_, dst) == ORB_OK; }
  bool Copy(ValueType &dst) { return Copy(&dst); }

 protected:
  const uint8_t instance_{0};
  orb_subscriber_t handle_ = ORB_SUBSCRIBER_INITIALIZER;
};

}  // namespace detail

/**
 * Lightweight C++ subscription wrapper.
 *
 * Use Subscription when the destination message storage is managed by the
 * caller. For the common case where the subscription owns one reusable message
 * object, prefer SubscriptionData. Use EventLoop when callbacks are preferable
 * to manual polling.
 */
template <const orb_metadata &meta>
class Subscription : public detail::SubscriptionBase<meta, Subscription<meta>> {
  using Base = detail::SubscriptionBase<meta, Subscription<meta>>;

 public:
  explicit Subscription(uint8_t instance = 0) noexcept : Base(instance) {}
};

/**
 * Subscription wrapper with embedded message storage.
 *
 * This is the recommended C++ helper for regular manual subscription loops:
 * poll/check for updates, call Update(), then read data().
 */
template <const orb_metadata &meta>
class SubscriptionData : public Subscription<meta> {
 public:
  using ValueType = typename msg::TypeMap<meta>::type;

  explicit SubscriptionData(uint8_t instance = 0) noexcept : Subscription<meta>(instance) {}

  using Subscription<meta>::Update;

  // update the embedded struct.
  bool Update() { return Subscription<meta>::Update(&data_); }

  const ValueType &data() const { return data_; }

 private:
  ValueType data_{};
};

}  // namespace uorb
