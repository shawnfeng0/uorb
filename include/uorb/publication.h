#pragma once

#include <uorb/internal/noncopyable.h>
#include <uorb/uorb.h>

namespace uorb {

namespace internal {

template <typename U>
class DefaultQueueSize {
 private:
  template <typename T>
  static constexpr uint8_t get_queue_size(decltype(T::ORB_QUEUE_LENGTH) *) {
    return T::ORB_QUEUE_LENGTH;
  }

  template <typename T>
  static constexpr uint8_t get_queue_size(...) {
    return 1;
  }

 public:
  static constexpr unsigned value = get_queue_size<U>(nullptr);
};

}  // namespace internal

/**
 * uORB publication wrapper class
 */
template <const orb_metadata &meta,
          uint16_t queue_size = internal::DefaultQueueSize<
              typename msg::TypeMap<meta>::type>::value>
class Publication : internal::Noncopyable {
  using Type = typename msg::TypeMap<meta>::type;

 public:
  Publication() noexcept = default;
  ~Publication() { handle_ &&orb_destroy_publication(&handle_); }

  /**
   * Publish the struct
   * @param data The uORB message struct we are updating.
   */
  bool Publish(const Type &data) {
    if (!handle_) {
      handle_ = orb_create_publication(&meta, queue_size);
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
