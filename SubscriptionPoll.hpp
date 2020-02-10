//
// Created by fs on 2/9/20.
//

#ifndef UORB__SUBSCRIPTION_POLL_HPP_
#define UORB__SUBSCRIPTION_POLL_HPP_

#include <base/orb_posix.h>
#include <base/orb_defines.h>
#include <uORB.h>
namespace uORB {
class SubscriptionPoll {
 public:
  /**
   * Constructor
   *
   * @param meta The uORB metadata (usually from the ORB_ID() macro) for the
   * topic.
   * @param instance The instance for multi sub.
   */
  explicit SubscriptionPoll(const orb_metadata *meta, uint8_t instance = 0)
      : meta_(meta), instance_(instance) {
    subscribe();
  }
  bool subscribe() {
    if (valid())
      return true;
    else {
      fd_ = orb_subscribe_multi(meta_, instance_);
      return valid();
    }
  }

  int get_fd() const { return fd_; }

  ~SubscriptionPoll() { unsubscribe(); }

  void unsubscribe() {
    if (valid()) orb_unsubscribe(fd_);
  }

  bool valid() const { return fd_ >= 0; }
  bool advertised() { return orb_exists(meta_, instance_) == ORB_OK; }

  bool poll(int timeout_ms) {
    orb_pollfd_t pollfd = {.fd = fd_, .events = POLLIN};
    if (valid_auto()) {
      return orb_poll(&pollfd, 1, timeout_ms) > 0;
    }
    return false;
  }

  bool updated() {
    bool updated = false;
    if (valid_auto()) {
      orb_check(fd_, &updated);
      return updated;
    }

    return false;
  }

  bool update(void *dst) {
    if (updated())
      return copy(dst);
    return false;
  }

  /**
   * Copy the struct
   * @param data The uORB message struct we are updating.
   */
  bool copy(void *dst) {
    if (valid_auto()) {
      return orb_copy(meta_, fd_, dst) == ORB_OK;
    }

    return false;
  }

  uint8_t get_instance() const { return instance_; }
  orb_id_t get_topic() const { return meta_; }

 private:
  int fd_{-1};
  const orb_metadata *meta_;
  uint8_t instance_;

  bool valid_auto() {
    // If invalid, try to initialize and check again if valid
    return valid() ? true : (subscribe() && valid());
  }
};

// Subscription wrapper class with data
template <class T>
class SubscriptionPollData : public SubscriptionPoll {
 public:
  /**
   * Constructor
   *
   * @param meta The uORB metadata (usually from the ORB_ID() macro) for the
   * topic.
   * @param instance The instance for multi sub.
   */
  explicit SubscriptionPollData(const orb_metadata *meta, uint8_t instance = 0)
      : SubscriptionPoll(meta, instance) {
    copy(&_data);
  }

  ~SubscriptionPollData() = default;

  // no copy, assignment, move, move assignment
  SubscriptionPollData(const SubscriptionPollData &) = delete;
  SubscriptionPollData &operator=(const SubscriptionPollData &) = delete;
  SubscriptionPollData(SubscriptionPollData &&) = delete;
  SubscriptionPollData &operator=(SubscriptionPollData &&) = delete;

  // update the embedded struct.
  bool update() { return SubscriptionPoll::update(&_data); }

  const T &get() const { return _data; }

 private:
  T _data{};
};

}  // namespace uORB

#endif  // UORB__SUBSCRIPTION_POLL_HPP_
