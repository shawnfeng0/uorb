#pragma once

#include <uorb/uorb.h>

namespace uorb {

/**
 * uORB publication wrapper class
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
  ~Publication() { handle_ &&orb_destroy_publication(&handle_); }

  /**
   * Publish the struct
   * @param data The uORB message struct we are updating.
   */
  bool Publish(const Type &data) {
    if (!handle_) {
      handle_ = orb_create_publication(&meta);
    }

    return handle_ && orb_publish(handle_, &data);
  }

 private:
  orb_publication_t *handle_{nullptr};
};

/**
 * The publication class with data embedded.
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
