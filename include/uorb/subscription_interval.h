#pragma once

#include <uorb/subscription.h>
#include <uorb/uorb.h>

namespace uorb {

/**
 * Subscription wrapper that throttles updates by a minimum time interval.
 *
 * Use this when a publisher may update faster than the consumer needs. The
 * subscription still tracks the latest topic data, but Update() returns true no
 * more often than the configured interval.
 */
template <const orb_metadata &meta>
class SubscriptionInterval : public Subscription<meta> {
 private:
  template <typename Tp>
  static constexpr Tp constrain(Tp val, Tp min_val, Tp max_val) {
    return (val < min_val) ? min_val : ((val > max_val) ? max_val : val);
  }

 public:
  using ValueType = typename msg::TypeMap<meta>::type;

  /**
   * Constructor
   *
   * @param meta The uORB metadata (usually from the ORB_ID() macro) for the
   * topic.
   * @param interval The requested maximum update interval in microseconds.
   * @param instance The instance for multi sub.
   */
  explicit SubscriptionInterval(uint32_t interval_us = 0, uint8_t instance = 0) noexcept
      : Subscription<meta>(instance), interval_us_(interval_us) {}

  ~SubscriptionInterval() override = default;

  /**
   * Check if there is a new update.
   * */
  bool Updated() override {
    return Subscription<meta>::Updated() &&
           (orb_elapsed_time_us(last_update_) >= interval_us_);
  }

  /**
   * Copy the struct if updated.
   * @param dst The destination pointer where the struct will be copied.
   * @return true only if topic was updated and copied successfully.
   */
  bool Update(ValueType *dst) override {
    if (Updated()) {
      return Copy(dst);
    }

    return false;
  }
  bool Update(ValueType &dst) override { return Update(&dst); }

  /**
   * Copy the struct
   * @param dst The destination pointer where the struct will be copied.
   * @return true only if topic was copied successfully.
   */
  bool Copy(ValueType *dst) override {
    if (Subscription<meta>::Copy(dst)) {
      last_update_ = CalculateNextUpdateTime(last_update_, interval_us_, orb_absolute_time_us());
      return true;
    }

    return false;
  }
  bool Copy(ValueType &dst) override { return Copy(&dst); }

  uint32_t interval_us() const { return interval_us_; }
  uint32_t interval_ms() const { return interval_us_ / 1000; }
  void set_interval_us(uint32_t interval_us) { interval_us_ = interval_us; }
  void set_interval_ms(uint32_t interval_ms) { interval_us_ = interval_ms * 1000; }

 protected:
  static constexpr orb_abstime_us CalculateNextUpdateTime(orb_abstime_us last_update,
                                                          uint32_t interval_us,
                                                          orb_abstime_us now) {
    const orb_abstime_us interval = interval_us;
    const orb_abstime_us minimum_update_time = now > interval ? now - interval : now;

    return constrain(last_update + interval, minimum_update_time, now);
  }

  orb_abstime_us last_update_{0};  // last update in microseconds
  uint32_t interval_us_{0};        // maximum update interval in microseconds
};

}  // namespace uorb
