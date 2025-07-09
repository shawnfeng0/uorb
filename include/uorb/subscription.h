#pragma once

#include <uorb/internal/noncopyable.h>
#include <uorb/publication.h>
#include <uorb/uorb.h>

namespace uorb {

template <const orb_metadata &meta>
class Subscription {
 protected:
  const uint8_t instance_{0};
  orb_subscription_t *handle_{nullptr};

  /**
   * Check if there is a new update.
   */
  virtual bool Updated() { return Subscribed() && orb_check_update(handle_); }

 public:
  using ValueType = typename msg::TypeMap<meta>::type;

  UORB_NONCOPYABLE(Subscription);

  /**
   * Constructor
   *
   * @param meta The uORB metadata (usually from the ORB_ID() macro) for the
   * topic.
   * @param instance The instance for multi sub.
   */
  explicit Subscription(uint8_t instance = 0) noexcept : instance_(instance) {}

  virtual ~Subscription() { handle_ &&orb_destroy_subscription(&handle_); }

  bool Subscribed() {
    // check if already subscribed
    if (handle_) {
      return true;
    }
    return (handle_ = orb_create_subscription_multi(&meta, instance_));
  }

  decltype(handle_) handle() { return Subscribed() ? handle_ : nullptr; }

  /**
   * Update the struct
   * @param data The uORB message struct we are updating.
   */
  virtual bool Update(ValueType *dst) { return Updated() && Copy(dst); }

  /**
   * Copy the struct
   * @param data The uORB message struct we are updating.
   */
  virtual bool Copy(ValueType *dst) { return Subscribed() && orb_copy(handle_, dst); }
};

// Subscription wrapper class with data
template <const orb_metadata &T>
class SubscriptionData : public Subscription<T> {
 public:
  using ValueType = typename msg::TypeMap<T>::type;

  explicit SubscriptionData(uint8_t instance = 0) noexcept : Subscription<T>(instance) {}

  ~SubscriptionData() override = default;

  using Subscription<T>::Update;

  // update the embedded struct.
  bool Update() { return Subscription<T>::Update(&data_); }

  const ValueType &get() const { return data_; }

 private:
  ValueType data_{};
};

}  // namespace uorb
