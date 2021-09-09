#pragma once

#include <uorb/internal/noncopyable.h>
#include <uorb/publication.h>
#include <uorb/uorb.h>

namespace uorb {

template <const orb_metadata &meta>
class Subscription : internal::Noncopyable {
  using Type = typename msg::TypeMap<meta>::type;

 protected:
  const uint8_t instance_{0};
  orb_subscription_t *handle_{nullptr};

  /**
   * Check if there is a new update.
   */
  virtual bool Updated() { return Subscribed() && orb_check_update(handle_); }

 public:
  /**
   * Constructor
   *
   * @param meta The uORB metadata (usually from the ORB_ID() macro) for the
   * topic.
   * @param instance The instance for multi sub.
   */
  explicit Subscription(uint8_t instance = 0) noexcept : instance_(instance) {}

  ~Subscription() { handle_ &&orb_destroy_subscription(&handle_); }

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
  virtual bool Update(Type *dst) { return Updated() && Copy(dst); }

  /**
   * Copy the struct
   * @param data The uORB message struct we are updating.
   */
  virtual bool Copy(Type *dst) {
    return Subscribed() && orb_copy(handle_, dst);
  }
};

// Subscription wrapper class with data
template <const orb_metadata &T>
class SubscriptionData : public Subscription<T> {
  using Type = typename msg::TypeMap<T>::type;

 public:
  explicit SubscriptionData(uint8_t instance = 0) noexcept
      : Subscription<T>(instance) {}

  ~SubscriptionData() = default;

  // update the embedded struct.
  bool Update() { return Subscription<T>::Update(&data_); }

  const Type &get() const { return data_; }

 private:
  Type data_{};
};

}  // namespace uorb
