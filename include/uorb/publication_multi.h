#pragma once

#include <uorb/uorb.h>

namespace uorb {

/**
 * uORB publication wrapper class
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
  ~PublicationMulti() { handle_ &&orb_destroy_publication(&handle_); }

  /**
   * Publish the struct
   * @param data The uORB message struct we are updating.
   */
  bool Publish(const Type &data) {
    if (!handle_) {
      handle_ = orb_create_publication_multi(&meta, &instance_);
    }

    return handle_ && orb_publish(handle_, &data);
  }

  unsigned instance() const { return instance_; }

 private:
  orb_publication_t *handle_{nullptr};
  unsigned instance_{0};
};

/**
 * The publication class with data embedded.
 */
template <const orb_metadata &T>
class PublicationMultiData : public PublicationMulti<T> {
  using Type = typename msg::TypeMap<T>::type;

 public:
  PublicationMultiData() noexcept = default;

  using PublicationMulti<T>::Publish;

  Type &data() { return data_; }
  const Type &data() const { return data_; }

  // Publishes the embedded struct.
  bool Publish() { return PublicationMulti<T>::Publish(data_); }

 private:
  Type data_{};
};

}  // namespace uorb
