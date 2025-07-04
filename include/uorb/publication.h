#pragma once

#include <uorb/internal/noncopyable.h>
#include <uorb/uorb.h>

namespace uorb {

/**
 * uORB publication wrapper class
 */
template <const orb_metadata &meta>
class Publication {
  using Type = typename msg::TypeMap<meta>::type;

 public:
  UORB_NONCOPYABLE(Publication);
  Publication() noexcept = default;
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
template <const orb_metadata &T>
class PublicationData : public Publication<T> {
  using Type = typename msg::TypeMap<T>::type;

 public:
  PublicationData() noexcept = default;

  Type &get() { return data_; }
  auto set(const Type &data) -> decltype(*this) {
    data_ = data;
    return *this;
  }

  // Publishes the embedded struct.
  bool Publish() { return Publication<T>::Publish(data_); }

 private:
  Type data_{};
};

}  // namespace uorb
