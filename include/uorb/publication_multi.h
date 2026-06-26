#pragma once

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
  using Type = typename msg::TypeMap<meta>::type;

 public:
  PublicationMulti() noexcept = default;
  PublicationMulti(const PublicationMulti &) = delete;
  PublicationMulti(PublicationMulti &&) = delete;
  PublicationMulti &operator=(const PublicationMulti &) = delete;
  PublicationMulti &operator=(PublicationMulti &&) = delete;
  ~PublicationMulti() { if (handle_._handle) orb_publisher_destroy(&handle_); }

  /**
   * Publish the struct
   * @param data The uORB message struct we are updating.
   * @return ORB_OK on success, error code otherwise.
   */
  orb_err Publish(const Type &data) {
    if (!handle_._handle) {
      orb_err err = orb_publisher_create_multi(&handle_, &meta, &instance_);
      if (err != ORB_OK) return err;
    }

    return orb_publisher_publish(&handle_, &data);
  }

  unsigned instance() const { return instance_; }

 private:
  orb_publisher_t handle_ = ORB_PUBLISHER_INITIALIZER;
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
  using Type = typename msg::TypeMap<meta>::type;

 public:
  PublicationMultiData() noexcept = default;

  using PublicationMulti<meta>::Publish;

  Type &data() { return data_; }
  const Type &data() const { return data_; }

  // Publishes the embedded struct.
  orb_err Publish() { return PublicationMulti<meta>::Publish(data_); }

 private:
  Type data_{};
};

}  // namespace uorb
