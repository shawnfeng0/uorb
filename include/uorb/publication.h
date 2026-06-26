#pragma once

#include <uorb/uorb.h>

namespace uorb {

/**
 * Lightweight C++ publisher wrapper for a single topic instance.
 *
 * Use Publication when the message storage is managed by the caller. For the
 * common case where the publisher owns one reusable message object, prefer
 * PublicationData.
 */
template <const orb_metadata &meta>
class Publication {
  using Type = typename msg::TypeMap<meta>::type;

 public:
  Publication() noexcept = default;
  Publication(const Publication &) = delete;
  Publication(Publication &&) = delete;
  Publication &operator=(const Publication &) = delete;
  Publication &operator=(Publication &&) = delete;
  ~Publication() { if (handle_._handle) orb_publisher_destroy(&handle_); }

  /**
   * Publish the struct
   * @param data The uORB message struct we are updating.
   */
  bool Publish(const Type &data) {
    if (!handle_._handle) {
      orb_publisher_create(&handle_, &meta);
    }

    return handle_._handle && orb_publisher_publish(&handle_, &data) == ORB_OK;
  }

 private:
  orb_publisher_t handle_ = ORB_PUBLISHER_INITIALIZER;
};

/**
 * Publisher wrapper with embedded message storage.
 *
 * This is the recommended C++ helper for regular single-instance publishing:
 * fill data(), then call Publish().
 */
template <const orb_metadata &meta>
class PublicationData : public Publication<meta> {
  using Type = typename msg::TypeMap<meta>::type;

 public:
  PublicationData() noexcept = default;

  using Publication<meta>::Publish;

  Type &data() { return data_; }
  const Type &data() const { return data_; }

  // Publishes the embedded struct.
  bool Publish() { return Publication<meta>::Publish(data_); }

 private:
  Type data_{};
};

}  // namespace uorb
