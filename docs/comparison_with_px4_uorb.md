# Difference from uORB of PX4 Autopilot

The main difference lies in the implementation of the bottom layer, and the application program interface changes relatively little. It also passed the uORB unit test of PX4 Autopilot.

## API interface difference(v1.11)

* Unified publication and subscription type: `orb_publisher_t*` and `orb_subscriber_t*`
* Pass pointers when unpublishing and unsubscribing to avoid wild pointers(Reference from zmq)
* Use ``bool`` type (include in ``<stdbool.h>``) to indicate whether the operation is successful
* Add independent `uevent` event loop library (`uevent/uevent.h`, `uevent_*` APIs), instead of `px4_poll` in PX4 Autopilot
* Add EventLoop/EventPoll APIs (`uorb_uevent/uorb_uevent.h`, `uorb_subscriber_create_source`) for callback/event-loop style dispatch
* Configure the topic's queue size in the topic's metadata, not at the time of publishing, which is good for a single topic with multiple publishers.

| PX4 uORB                                                     | Current uORB                                                 |
| :----------------------------------------------------------- | :----------------------------------------------------------- |
| ~~orb_advert_t orb_advertise(const struct orb_metadata \*meta, const void \*data)~~ |                                                              |
| **orb_advert_t** orb_advertise_queue(const struct orb_metadata \*meta, const void \*data~~, unsigned int queue_size~~) | **orb_publisher_t** \*orb_publisher_create(const struct orb_metadata \*meta) |
| ~~orb_advert_t orb_advertise_multi(const struct orb_metadata \*meta, const void \*data, int \*instance)~~ |                                                              |
| **orb_advert_t** orb_advertise_multi_queue(const struct orb_metadata \*meta, ~~const void \*data~~, int \*instance~~, unsigned queue_size~~) | **orb_publisher_t** \*orb_publisher_create_multi(const struct orb_metadata \*meta, unsigned int \*instance) |
| **int** orb_publisher_publish(~~const struct orb_metadata \*meta,~~ orb_advert_t handle, const void \*data) | **bool** orb_publisher_publish(**orb_publisher_t \*handle**, const void *data) |
| **int** orb_unadvertise(orb_advert_t **handle**)             | **bool** orb_publisher_destroy(**orb_publisher_t \*\*handle_ptr**) |
| **int** orb_subscribe(const struct orb_metadata \*meta)      | **orb_subscriber_t ***orb_subscriber_create(const struct orb_metadata *meta) |
| **int** orb_subscribe_multi(const struct orb_metadata \*meta, unsigned instance) | **orb_subscriber_t ***orb_subscriber_create_multi(const struct orb_metadata *meta, unsigned instance) |
| **int** orb_unsubscribe(**int handle**)                      | bool orb_subscriber_destroy(**orb_subscriber_t handle_ptr\*\***) |
| **int** orb_subscriber_copy(~~const struct orb_metadata \*meta,~~ **int handle**, void \*buffer) | **bool** orb_subscriber_copy(**orb_subscriber_t *handle**, void *buffer) |
| **int** orb_check(**int handle** ~~, bool \*updated~~)       | **bool** orb_subscriber_check_update(**orb_subscriber_t *handle**)    |
| ~~int orb_set_interval(int handle, unsigned interval)~~      |                                                              |
| ~~int orb_get_interval(int handle, unsigned \*interval)~~    |                                                              |
| int px4_poll(**px4_pollfd_struct_t** \*fds, unsigned int nfds, int timeout) | uevent_* APIs (`uevent_create`, `uevent_add`, `uevent_loop`, etc.) |

## PX4-compatible API shim (`uorb_px4_compat`)

For users migrating from PX4's uORB, a standalone compatibility layer is
provided in `components/px4_compat/`.  Link against the CMake target
`uorb_px4_compat` and include `<uorb_compat/px4_compat.h>` to use the
familiar PX4-style C API without changing your application code.

The shim maps the PX4 API surface directly onto the native uORB API:

| PX4 uORB                                                                    | uorb_px4_compat shim                                        |
| :-------------------------------------------------------------------------- | :---------------------------------------------------------- |
| `orb_advert_t orb_advertise(meta, data)`                                    | ✅ provided                                                 |
| `orb_advert_t orb_advertise_queue(meta, data, queue_size)`                  | ✅ provided (queue_size ignored — set in ORB_DEFINE)        |
| `orb_advert_t orb_advertise_multi(meta, data, instance)`                    | ✅ provided                                                 |
| `orb_advert_t orb_advertise_multi_queue(meta, data, instance, queue_size)`  | ✅ provided (queue_size ignored)                            |
| `int orb_publish(meta, handle, data)`                                       | ✅ provided                                                 |
| `int orb_unadvertise(handle)`                                               | ✅ provided                                                 |
| `int orb_subscribe(meta)`                                                   | ✅ provided (returns int fd)                                |
| `int orb_subscribe_multi(meta, instance)`                                   | ✅ provided (returns int fd)                                |
| `int orb_unsubscribe(fd)`                                                   | ✅ provided                                                 |
| `int orb_copy(meta, fd, buffer)`                                            | ✅ provided                                                 |
| `int orb_check(fd, &updated)`                                               | ✅ provided                                                 |
| `int orb_set_interval(fd, interval_ms)`                                     | ✅ stored (throttling not enforced by core library)         |
| `int orb_get_interval(fd, &interval_ms)`                                    | ✅ provided                                                 |
| `int px4_poll(fds, nfds, timeout_ms)`                                       | ✅ provided (condvar-based; uORB fds only, not OS fds)      |

**Limitations:**
- `px4_pollfd_struct_t::fd` values are internal handles managed by the shim.
  They are *not* OS-level file descriptors and cannot be mixed with real fds
  in a POSIX `poll()` call.
- `orb_set_interval()` stores the value but the core library does not enforce
  message throttling; callers must gate copies themselves.
- At most `ORB_PX4_COMPAT_MAX_SUBSCRIBERS` (default 128) concurrent
  subscriptions are supported.

## Difference in implementation

* Delete the ``o_id ``(``ORB_ID enum``) field in ``orb_metadata``. This field has no meaning and is very redundant. It is newly added in PX4-v1.11. 
* In C++, ``orb_metadata`` can be obtained directly through the orb message type, which can simplify the constructors of ``PublicationData`` and ``SubscriptionData``, and there is no need to pass in parameters such as ``ORB_ID(msg_name)`` to simplify the code and avoid errors.
