#pragma once

#include <uorb/publication.h>
#include <uorb/uorb.h>

namespace uorb {

/**
 * Lightweight C++ subscription wrapper.
 *
 * Use Subscription when the destination message storage is managed by the
 * caller. For the common case where the subscription owns one reusable message
 * object, prefer SubscriptionData. Use EventLoop when callbacks are preferable
 * to manual polling.
 */
template <const orb_metadata &meta>
class Subscription {
 public:
  using ValueType = typename msg::TypeMap<meta>::type;

  Subscription(const Subscription &) = delete;
  Subscription(Subscription &&) = delete;
  Subscription &operator=(const Subscription &) = delete;
  Subscription &operator=(Subscription &&) = delete;

  /**
   * Constructor
   *
   * @param meta The uORB metadata (usually from the ORB_ID() macro) for the
   * topic.
   * @param instance The instance for multi sub.
   */
  explicit Subscription(uint8_t instance = 0) noexcept : instance_(instance) {}

  virtual ~Subscription() { handle_ &&orb_destroy_subscription(&handle_); }

  bool Subscribe() {
    if (handle_) {
      return true;
    }
    return (handle_ = orb_create_subscription_multi(&meta, instance_));
  }

  orb_subscription_t *handle() { return Subscribe() ? handle_ : nullptr; }

  /**
   * Check if there is a new update.
   */
  virtual bool Updated() { return Subscribe() && orb_check_update(handle_); }

  /**
   * Update the struct
   * @param data The uORB message struct we are updating.
   */
  virtual bool Update(ValueType *dst) { return Updated() && Copy(dst); }
  virtual bool Update(ValueType &dst) { return Update(&dst); }

  /**
   * Copy the struct
   * @param data The uORB message struct we are updating.
   */
  virtual bool Copy(ValueType *dst) { return Subscribe() && orb_copy(handle_, dst); }
  virtual bool Copy(ValueType &dst) { return Copy(&dst); }

 protected:
  const uint8_t instance_{0};
  orb_subscription_t *handle_{nullptr};
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

  ~SubscriptionData() override = default;

  using Subscription<meta>::Update;

  // update the embedded struct.
  bool Update() { return Subscription<meta>::Update(&data_); }

  const ValueType &data() const { return data_; }

 private:
  ValueType data_{};
};

}  // namespace uorb
