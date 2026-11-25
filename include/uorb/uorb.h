/**
 * @file uorb.h
 * API for the uORB lightweight object request broker.
 */

#pragma once

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <time.h>

#include <uevent/uevent.h>

#ifdef __cplusplus
#include <atomic>
#include <type_traits>
#endif

/**
 * Error codes for uORB operations.
 *
 * All error codes are negative so callers can test with `err < 0` or
 * `err != ORB_OK`. They are intentionally not mapped to errno values
 * to avoid namespace collisions with system errno.
 */
typedef enum {
  ORB_OK = 0,
  ORB_ERR_INVALID = -1,
  ORB_ERR_NO_MEM = -2,
  ORB_ERR_BUSY = -3,
  ORB_ERR_EXIST = -4,
  ORB_ERR_NO_ENTRY = -5,
  ORB_ERR_TIMEOUT = -6,
  ORB_ERR_AGAIN = -7,
  ORB_ERR_UNKNOWN = -8,
} orb_err;

/**
 * Maximum number of multi topic instances (overridable at compile time;
 * all translation units must use the same value).
 */
#ifndef ORB_MULTI_MAX_INSTANCES
#define ORB_MULTI_MAX_INSTANCES 4
#endif

/**
 * Topic durability policy.
 *
 * - ORB_DURABILITY_DURABLE (default): state topics where late subscribers
 *   immediately see the most recently published sample as an update, and
 *   orb_subscriber_copy_once() returns the latest sample.
 * - ORB_DURABILITY_VOLATILE: event/stream topics where historical samples are
 *   not delivered to late subscribers (orb_subscriber_check_update() starts
 *   false until a new publication arrives, and orb_subscriber_copy_once()
 *   returns ORB_ERR_NO_ENTRY).
 */
typedef enum {
  ORB_DURABILITY_DURABLE = 0,
  ORB_DURABILITY_VOLATILE = 1,
} orb_durability_t;

/**
 * Object metadata.
 */
struct orb_metadata {
  const char *o_name;               /**< unique object name */
  const uint16_t o_size;            /**< object size */
  const uint16_t o_size_no_padding; /**< object size w/o padding (external logger contract, not used by library) */
  const char *o_fields;             /**< semicolon separated list of fields (with type) */
  uint16_t o_queue_size;            /**< maximum number of queued samples */
  uint8_t o_durability;             /**< orb_durability_t (default: ORB_DURABILITY_DURABLE) */
  uint32_t o_name_hash;             /**< compile-time FNV-1a 32-bit hash of o_name */
#ifdef __cplusplus
  mutable std::atomic<void *> instances[ORB_MULTI_MAX_INSTANCES]{};
#else
  void *instances[ORB_MULTI_MAX_INSTANCES];
#endif
};

/**
 * The status of a topic
 */
struct orb_status {
  uint16_t queue_size;  // Queue size

  uint8_t subscriber_count;       // Number of tracked subscribers
  bool has_untracked_subscriber;  // Whether orb_subscriber_copy_once() has been used
  uint8_t publisher_count;        // Number of tracked publishers
  bool has_untracked_publisher;   // Whether orb_publisher_publish_once() has been used

  unsigned latest_data_index;     // The latest data index
  uint64_t last_publish_time_us;  // Monotonic timestamp [us] of the last Publish() (0 if never published)
};

#ifdef __cplusplus
namespace uorb {
/// Compile-time FNV-1a 32-bit hash of a NUL-terminated topic name.
constexpr uint32_t topic_hash(const char *str, uint32_t h = 2166136261u) {
  return *str ? topic_hash(str + 1, (h ^ static_cast<uint8_t>(*str)) * 16777619u) : h;
}
namespace msg {
template <const orb_metadata &>
struct TypeMap;
}
}  // namespace uorb
#endif

/**
 * Generates a pointer to the uORB metadata structure for
 * a given topic.
 *
 * The topic must have been declared previously in scope
 * with ORB_DECLARE().
 *
 * @param _name		The name of the topic.
 */
#define ORB_ID(_name) __orb_##_name

/**
 * Declare (prototype) the uORB metadata for a topic (used by code generators).
 *
 * @param _name		The name of the topic.
 */
#if defined(__cplusplus)
#define ORB_DECLARE(_name, _struct)       \
  namespace uorb {                        \
  namespace msg {                         \
  extern const struct orb_metadata _name; \
  template <>                             \
  struct TypeMap<_name> {                 \
    using type = _struct;                 \
  };                                      \
  }                                       \
  }                                       \
  extern "C" const struct orb_metadata *__orb_##_name
#else
#define ORB_DECLARE(_name, _struct) extern const struct orb_metadata *__orb_##_name
#endif

/**
 * Define (instantiate) the uORB metadata for a topic.
 *
 * The uORB metadata is used to help ensure that updates and
 * copies are accessing the right data.
 *
 * Note that there must be no more than one instance of this macro
 * for each topic.
 *
 * Simple topics can omit logger metadata:
 *   ORB_DEFINE(my_topic, struct my_topic_s, 1);
 *
 * Topics with logger metadata can use the full form:
 *   ORB_DEFINE(my_topic, struct my_topic_s, 12, "uint64_t timestamp;int32_t val", 1);
 *
 * @param _name		The name of the topic.
 * @param _struct	The structure the topic provides.
 * @param _size_no_padding	Struct size w/o padding at the end
 * @param _fields	All fields in a semicolon separated list
 *                      e.g: "float[3] position;bool armed"
 * @param _queue_size	The maximum number of queued samples.
 */
#ifdef __cplusplus
#define ORB_CHECK_TOPIC_TYPE(_struct)                                                             \
  static_assert(std::is_trivially_copyable<_struct>::value,                                       \
                "uORB topic type must be trivially copyable; std::string, std::vector, "          \
                "custom copy operations, and custom destructors are unsupported");                \
  static_assert(std::is_standard_layout<_struct>::value,                                          \
                "uORB topic type must use standard layout; virtual functions, mixed access "      \
                "control, and non-standard object layout are unsupported")
#define ORB_DEFINE_3(_name, _struct, _queue_size) \
  ORB_DEFINE_6(_name, _struct, 0, "", _queue_size, ORB_DURABILITY_DURABLE)
#define ORB_DEFINE_4(_name, _struct, _queue_size, _durability) \
  ORB_DEFINE_6(_name, _struct, 0, "", _queue_size, _durability)
#define ORB_DEFINE_5(_name, _struct, _size_no_padding, _fields, _queue_size) \
  ORB_DEFINE_6(_name, _struct, _size_no_padding, _fields, _queue_size, ORB_DURABILITY_DURABLE)
#define ORB_DEFINE_6(_name, _struct, _size_no_padding, _fields, _queue_size, _durability)                                    \
  ORB_CHECK_TOPIC_TYPE(_struct);                                                                                             \
  const struct orb_metadata uorb::msg::_name = {                                                                             \
      #_name, sizeof(_struct), _size_no_padding, _fields, _queue_size, static_cast<uint8_t>(_durability),                    \
      uorb::topic_hash(#_name), {}};                                                                                         \
  const struct orb_metadata *__orb_##_name = &uorb::msg::_name;                                                              \
  struct hack
#define ORB_DEFINE_SELECT(_1, _2, _3, _4, _5, _6, NAME, ...) NAME
#define ORB_DEFINE(...) \
  ORB_DEFINE_SELECT(__VA_ARGS__, ORB_DEFINE_6, ORB_DEFINE_5, ORB_DEFINE_4, ORB_DEFINE_3)(__VA_ARGS__)
#else
#if defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L
#define ORB_DEFINE(...) _Static_assert(0, "ORB_DEFINE is only supported in C++ translation units")
#else
#define ORB_DEFINE(...) typedef char ORB_DEFINE_is_only_supported_in_Cplusplus_translation_units[-1]
#endif
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Public C API index:
 *
 * - Metadata: ORB_ID(), ORB_DECLARE(), ORB_DEFINE(), orb_metadata.
 * - Publication: orb_publisher_create(), orb_publisher_create_multi(),
 *   orb_publisher_destroy(), orb_publisher_publish(), orb_publisher_publish_once(),
 *   orb_publisher_publish_auto(), orb_publisher_loan(), orb_publisher_publish_loaned(),
 *   orb_publisher_discard_loan().
 * - Subscription: orb_subscriber_create(),
 *   orb_subscriber_create_multi(), orb_subscriber_destroy(), orb_subscriber_copy(),
 *   orb_subscriber_copy_once(), orb_subscriber_check_update(),
 *   orb_subscriber_event_source().
 * - Introspection: orb_exists(), orb_group_count(),
 *   orb_get_topic_status(), orb_for_each_topic(), orb_version(), orb_strerror().
 * - Memory / RTOS: orb_set_allocator(), orb_topic_preallocate().
 * - Time: orb_absolute_time_us(), orb_elapsed_time_us().
 */

/**
 * ORB topic advertiser handle
 *
 * Opaque handle with private void* pointer.
 * Publication handles are owned by the caller. Each handle returned from
 * orb_publisher_create() or orb_publisher_create_multi() must be released
 * with orb_publisher_destroy(). C++ publication wrappers own their handles
 * exclusively and destroy them in their destructors.
 */
typedef struct {
  void* _handle;
} orb_publisher_t;

#define ORB_PUBLISHER_INITIALIZER {NULL}

/**
 * ORB topic subscriber handle
 *
 * Opaque handle with private void* pointer.
 */
typedef struct {
  void* _handle;
} orb_subscriber_t;

#define ORB_SUBSCRIBER_INITIALIZER {NULL}

/**
 * Create a publication handle for topic instance 0.
 *
 * @see orb_publisher_create_multi()
 * @return ORB_OK on success, error code otherwise
 */
orb_err orb_publisher_create(orb_publisher_t *pub, const struct orb_metadata *meta);

/**
 * Advertise as the publisher of a topic.
 *
 * This performs the initial advertisement of a topic; it creates the topic
 * node if required and publishes the initial data.
 *
 * Any number of advertisers may publish to a topic; publications are atomic
 * but co-ordination between publishers is not provided by the ORB.
 *
 * If instance is NULL, the device is a single instance, and each call will
 * return the same instance. Otherwise, create an independent instance of the
 * topic (each instance has its own buffer), and each call will generate an
 * independent instance (up to ORB_MULTI_MAX_INSTANCES), which is useful for
 * scenarios where multiple publishers publish the same topic.
 *
 * @param meta    The uORB metadata (usually from the ORB_ID() macro) for the
 * topic.
 * @param instance  Pointer to an integer which will yield the instance ID
 * (0-based) of the publication. This is an output parameter and will be set to
 * the newly created instance, ie. 0 for the first advertiser, 1 for the next
 * and so on. If NULL, only instance 0 will be returned.
 * @return ORB_OK on success, error code otherwise
 */
orb_err orb_publisher_create_multi(orb_publisher_t *pub, const struct orb_metadata *meta, unsigned int *instance);

/**
 * Destroy a publication handle (idempotent; NULL-safe).
 *
 * @param pub Pointer to the publication handle; its _handle will be reset to NULL.
 */
void orb_publisher_destroy(orb_publisher_t *pub);

/**
 * Publish new data to a topic.
 *
 * The data is atomically published to the topic and any waiting subscribers
 * will be notified.  Subscribers that are not waiting can check the topic
 * for updates using orb_subscriber_check_update().
 *
 * @param pub   The publication handle (passed by value).
 * @param data  A pointer to the data to be published.
 *              The length must correspond to the topic structure.
 * @return ORB_OK on success, error code otherwise
 */
orb_err orb_publisher_publish(orb_publisher_t pub, const void *data);

/**
 * Borrow a pre-zeroed, topic-owned message buffer for in-place construction.
 *
 * Avoids stack allocation for large message structs on small RTOS stacks and
 * holds no DeviceNode mutex while the caller fills the buffer. At most one
 * loan per topic instance may be active at a time (returns ORB_ERR_BUSY if
 * another loan is already checked out).
 *
 * Every successful loan MUST be returned via orb_publisher_publish_loaned() or
 * orb_publisher_discard_loan().
 *
 * @param pub     The publication handle (passed by value).
 * @param out_msg Output pointer receiving the loaned message buffer.
 * @return ORB_OK on success, ORB_ERR_BUSY / ORB_ERR_NO_MEM / ORB_ERR_INVALID on error
 */
orb_err orb_publisher_loan(orb_publisher_t pub, void **out_msg);

/**
 * Publish a message buffer previously obtained from orb_publisher_loan() and
 * release the loan.
 *
 * @param pub        The publication handle (passed by value).
 * @param loaned_msg The buffer pointer returned by orb_publisher_loan().
 * @return ORB_OK on success, ORB_ERR_INVALID / ORB_ERR_NO_MEM on error
 */
orb_err orb_publisher_publish_loaned(orb_publisher_t pub, void *loaned_msg);

/**
 * Release a loaned message buffer without publishing (idempotent; NULL-safe).
 *
 * @param pub        The publication handle (passed by value).
 * @param loaned_msg The buffer pointer returned by orb_publisher_loan().
 */
void orb_publisher_discard_loan(orb_publisher_t pub, void *loaned_msg);

/**
 * Publish data on topic instance 0 without creating a publication handle.
 *
 * This API does not contribute to the tracked publisher count. Prefer
 * orb_publisher_create() + orb_publisher_publish() for normal publishers.
 *
 * @param meta  The uORB metadata (usually from the ORB_ID() macro) for the
 * topic.
 * @param data @see orb_publisher_publish()
 * @return @see orb_publisher_publish()
 */
orb_err orb_publisher_publish_once(const struct orb_metadata *meta, const void *data);

/**
 * Advertise as the publisher of a topic.
 *
 * This performs the initial advertisement of a topic; it creates the topic
 * node if required and publishes the initial data.
 *
 * @see orb_publisher_create_multi() for meaning of the individual parameters
 */
static inline orb_err orb_publisher_publish_auto(const struct orb_metadata *meta, orb_publisher_t *pub, const void *data,
                                    unsigned int *instance) {
  if (!meta || !pub || !data) {
    return ORB_ERR_INVALID;
  }

  if (!pub->_handle) {
    orb_err err = orb_publisher_create_multi(pub, meta, instance);
    if (err != ORB_OK) {
      return err;
    }
  }
  return orb_publisher_publish(*pub, data);
}

/**
 * Create a subscription handle for topic instance 0.
 *
 * @see orb_subscriber_create_multi()
 * @return ORB_OK on success, error code otherwise
 */
orb_err orb_subscriber_create(orb_subscriber_t *sub, const struct orb_metadata *meta);

/**
 * Subscribe to a multi-instance of a topic.
 *
 * The returned value is a subscriber handle that can be passed to
 * orb_subscriber_event_source() to wait for updates via uevent, as well as
 * orb_subscriber_copy() and orb_subscriber_check_update().
 *
 * If there were any publications of the topic prior to the subscription,
 * orb_subscriber_check_update() right after orb_subscriber_create() will return true.
 *
 * Subscription will succeed even if the topic has not been advertised;
 * in this case the topic will have a timestamp of zero, it will never
 * signal a poll() event, checking will always return false and it cannot
 * be copied. When the topic is subsequently advertised, poll, check,
 * stat and copy calls will react to the initial publication that is
 * performed as part of the advertisement.
 *
 * Subscription will fail if the topic is not known to the system, i.e.
 * there is nothing in the system that has declared the topic and thus it
 * can never be published.
 *
 * If a publisher publishes multiple instances the subscriber should
 * subscribe to each instance with orb_subscriber_create
 * (@see orb_publisher_create_multi()).
 *
 * @note Topic nodes are persistent: once created by the first publisher or
 * subscriber, a DeviceNode lives for the lifetime of the process. This is
 * intentional for embedded use cases where topics are statically defined.
 *
 * @param sub       Pointer to the subscription handle to be created.
 * @param meta      The uORB metadata (usually from the ORB_ID() macro)
 *                  for the topic.
 * @param instance  The instance of the topic. Instance 0 matches the
 *                  topic of the orb_subscriber_create() call, higher indices
 *                  are for topics created with orb_publisher_create_multi().
 * @return ORB_OK on success, error code otherwise
 */
orb_err orb_subscriber_create_multi(orb_subscriber_t *sub, const struct orb_metadata *meta, unsigned instance);

/**
 * Destroy a subscription handle (idempotent; NULL-safe).
 *
 * @param sub Pointer to the subscription handle; its _handle will be reset to NULL.
 */
void orb_subscriber_destroy(orb_subscriber_t *sub);

/**
 * Fetch data from a topic.
 *
 * This is the only operation that will reset the internal marker that
 * indicates that a topic has been updated for a subscriber. Once poll
 * or orb_subscriber_check_update() returns indicating that an update is available, this
 * call must be used to update the subscription.
 *
 * @param sub     A handle returned from orb_subscriber_create (passed by value).
 * @param buffer  Pointer to the buffer receiving the data.
 *                The length must correspond to the topic structure.
 * @return ORB_OK on success, error code otherwise
 */
orb_err orb_subscriber_copy(orb_subscriber_t sub, void *buffer);

/**
 * Copy data from topic instance 0 without creating a subscription handle.
 *
 * This API does not contribute to the tracked subscriber count. Prefer
 * orb_subscriber_create() + orb_subscriber_copy() for normal subscribers.
 *
 * @param meta    The uORB metadata (usually from the ORB_ID() macro) for the
 *                topic.
 * @param buffer @see orb_subscriber_copy()
 * @return @see orb_subscriber_copy()
 */
orb_err orb_subscriber_copy_once(const struct orb_metadata *meta, void *buffer);

/**
 * Check whether a topic has been published to since the last orb_subscriber_copy.
 *
 * This check can be used to determine whether to copy the topic when
 * not using poll(), or to avoid the overhead of calling poll() when the
 * topic is likely to have updated.
 *
 * Updates are tracked on a per-handle basis; this call will continue to
 * return true until orb_subscriber_copy is called using the same handle.
 *
 * @param sub  A handle returned from orb_subscriber_create.
 * @return true if the topic has been updated since the last time it was copied
 * using this handle.
 */
bool orb_subscriber_check_update(orb_subscriber_t sub);

/**
 * Obtain a borrowed uevent_source_t view for a uORB subscriber handle.
 *
 * The returned event source is owned by the subscriber handle (zero extra
 * allocation) and can be added directly to a uevent loop via uevent_add().
 * When new data is published to the subscriber's topic, the event source is
 * woken and reported as ready in uevent_loop().
 *
 * Lifetime: remove the source from its uevent base via uevent_remove() before
 * calling orb_subscriber_destroy(). Calling uevent_source_destroy() on a
 * borrowed subscriber source is unnecessary (and safely ignored).
 *
 * @param sub subscription handle (passed by value)
 * @return initialized uevent_source_t, or UEVENT_SOURCE_INITIALIZER if sub is invalid
 */
uevent_source_t orb_subscriber_event_source(orb_subscriber_t sub);

/**
 * Check if a topic has already been created and published (advertised)
 *
 * @param meta      ORB topic metadata.
 * @param instance  ORB instance
 * @return true if the topic exists, false otherwise.
 */
bool orb_exists(const struct orb_metadata *meta, unsigned int instance);

/**
 * Get the number of published instances of a topic group
 *
 * @param meta  ORB topic metadata.
 * @return      The number of published instances of this topic
 */
unsigned int orb_group_count(const struct orb_metadata *meta);

/**
 * Get the status of a topic (number of publishers, subscribers, etc.)
 *
 * @param meta      ORB topic metadata
 * @param instance  ORB instance
 * @param status    [out] The topic status.
 * @return ORB_OK on success, error code otherwise
 */
orb_err orb_get_topic_status(const struct orb_metadata *meta, unsigned int instance, struct orb_status *status);

/**
 * Visitor callback for orb_for_each_topic().
 *
 * Invoked once per instantiated topic instance with no internal locks held.
 * Return true to continue iteration, or false to stop early.
 *
 * @param meta     Topic metadata pointer
 * @param instance Topic instance index (0 .. ORB_MULTI_MAX_INSTANCES - 1)
 * @param status   Snapshot of the topic instance's current status
 * @param ctx      User context pointer passed to orb_for_each_topic()
 * @return true to continue, false to stop
 */
typedef bool (*orb_topic_visitor_fn)(const struct orb_metadata *meta,
                                     unsigned int instance,
                                     const struct orb_status *status,
                                     void *ctx);

/**
 * Iterate over all currently instantiated topic instances in the process.
 *
 * Thread-safe: snapshots the persistent node list under DeviceMaster::lock_,
 * then invokes visitor outside any internal locks so the visitor may safely
 * call other uORB APIs.
 *
 * @param visitor Callback invoked for each topic instance (must not be NULL)
 * @param ctx     User context pointer forwarded to visitor
 * @return Number of topic instances visited (0 if visitor is NULL or none exist)
 */
size_t orb_for_each_topic(orb_topic_visitor_fn visitor, void *ctx);

/**
 * Custom allocator function signatures for deterministic / RTOS memory pools.
 */
typedef void *(*orb_alloc_fn)(size_t size);
typedef void (*orb_free_fn)(void *ptr);

/**
 * Install custom allocator hooks for all internal uORB allocations
 * (DeviceNode, ring buffers, and subscriber handles).
 *
 * Must be called before creating handles (or after destroying all active
 * subscriber handles before restoring). Pass NULL for both to restore the
 * default allocator. Passing only one NULL returns ORB_ERR_INVALID.
 *
 * @param alloc_fn Allocation function (must return at least max_align_t alignment)
 * @param free_fn  Deallocation function
 * @return ORB_OK on success, ORB_ERR_INVALID otherwise
 */
orb_err orb_set_allocator(orb_alloc_fn alloc_fn, orb_free_fn free_fn);

/**
 * Pre-allocate the DeviceNode and ring buffer for a topic instance.
 *
 * Calling this during startup guarantees that subsequent orb_publisher_publish()
 * calls perform zero dynamic memory allocations (deterministic RT latency).
 *
 * @param meta     Topic metadata (ORB_ID(...))
 * @param instance Topic instance (0 .. ORB_MULTI_MAX_INSTANCES - 1)
 * @return ORB_OK on success, ORB_ERR_INVALID or ORB_ERR_NO_MEM on error
 */
orb_err orb_topic_preallocate(const struct orb_metadata *meta, unsigned int instance);

/**
 * Get orb version string
 * @return version string
 */
const char *orb_version(void);

/**
 * Convert an orb_err code to a human-readable string.
 * @param err Error code
 * @return Static null-terminated description string (never NULL)
 */
const char *orb_strerror(orb_err err);

/**
 * Absolute time, in microsecond units.
 *
 * Absolute time is measured from some arbitrary epoch shortly after
 * system startup.  It should never wrap or go backwards.
 */
typedef uint64_t orb_abstime_us;

/**
 * Get absolute time in [us] (does not wrap).
 */
static inline orb_abstime_us orb_absolute_time_us(void) {
  struct timespec ts = {0, 0};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (orb_abstime_us)(ts.tv_sec) * 1000000 + ts.tv_nsec / 1000;
}

/**
 * Compute the delta between a timestamp taken in the past and now.
 */
static inline orb_abstime_us orb_elapsed_time_us(const orb_abstime_us then) {
  return orb_absolute_time_us() - then;
}

#ifdef __cplusplus
}

namespace uorb {

/**
 * Type-safe C++ convenience wrapper around orb_publisher_publish_once().
 */
template <const orb_metadata &meta>
inline orb_err publish(const typename msg::TypeMap<meta>::type &data) {
  return orb_publisher_publish_once(&meta, &data);
}

/**
 * Type-safe C++ convenience wrapper around orb_subscriber_copy_once().
 */
template <const orb_metadata &meta>
inline orb_err copy(typename msg::TypeMap<meta>::type &out) {
  return orb_subscriber_copy_once(&meta, &out);
}

}  // namespace uorb
#endif
