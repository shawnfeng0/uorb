# Difference from uORB of PX4 Autopilot

The main difference lies in the implementation of the bottom layer, and the application program interface changes relatively little. It also passed the uORB unit test of PX4 Autopilot.

## API interface difference(v1.11)

* Unified publication and subscription type: `orb_publication_t*` and `orb_subscription_t*`
* Pass pointers when unpublishing and unsubscribing to avoid wild pointers(Reference from zmq)
* Use ``bool`` type (include in ``<stdbool.h>``) to indicate whether the operation is successful
* Add independent ``orb_poll`` function, instead of ``px4_poll`` in PX4 Autopilot

| PX4 uORB                                                     | Current uORB                                                 |
| :----------------------------------------------------------- | :----------------------------------------------------------- |
| ~~orb_advert_t orb_advertise(const struct orb_metadata \*meta, const void \*data)~~ |                                                              |
| **orb_advert_t** orb_advertise_queue(const struct orb_metadata \*meta, const void \*data, unsigned int queue_size) | **orb_publication_t** \*orb_create_publication(const struct orb_metadata \*meta, unsigned int queue_size) |
| ~~orb_advert_t orb_advertise_multi(const struct orb_metadata \*meta, const void \*data, int \*instance)~~ |                                                              |
| **orb_advert_t** orb_advertise_multi_queue(const struct orb_metadata \*meta, ~~const void \*data~~, int \*instance, unsigned queue_size) | **orb_publication_t** \*orb_create_publication_multi(const struct orb_metadata \*meta, unsigned int \*instance, unsigned int queue_size) |
| **int** orb_publish(~~const struct orb_metadata \*meta,~~ orb_advert_t handle, const void \*data) | **bool** orb_publish(**orb_publication_t \*handle**, const void *data) |
| **int** orb_unadvertise(orb_advert_t **handle**)             | **bool** orb_destroy_publication(**orb_publication_t \*\*handle_ptr**) |
| **int** orb_subscribe(const struct orb_metadata \*meta)      | **orb_subscription_t ***orb_create_subscription(const struct orb_metadata *meta) |
| **int** orb_subscribe_multi(const struct orb_metadata \*meta, unsigned instance) | **orb_subscription_t ***orb_create_subscription_multi(const struct orb_metadata *meta, unsigned instance) |
| **int** orb_unsubscribe(**int handle**)                      | bool orb_destroy_subscription(**orb_subscription_t handle_ptr\*\***) |
| **int** orb_copy(~~const struct orb_metadata \*meta,~~ **int handle**, void \*buffer) | **bool** orb_copy(**orb_subscription_t *handle**, void *buffer) |
| **int** orb_check(**int handle** ~~, bool \*updated~~)       | **bool** orb_check_update(**orb_subscription_t *handle**)    |
| ~~int orb_set_interval(int handle, unsigned interval)~~      |                                                              |
| ~~int orb_get_interval(int handle, unsigned \*interval)~~    |                                                              |
| int px4_poll(**px4_pollfd_struct_t** \*fds, unsigned int nfds, int timeout) | int orb_poll(**struct orb_pollfd** \*fds, unsigned int nfds, int timeout_ms) |

## Difference in implementation

* Use read-write locks instead of regular locks to reduce latency in multi-subscription situations
* The queue size is set to a power of 2, and this implementation avoids the problem of limiting the number of messages published to ``UINTMAX_MAX``
* Delete the ``o_id ``(``ORB_ID enum``) field in ``orb_metadata``. This field has no meaning and is very redundant. It is newly added in PX4-v1.11. 
* Delete the ``o_size_no_padding`` and ``o_fields`` fields, keep it simple, and do not plan to use logger.
* In C++, ``orb_metadata`` can be obtained directly through the orb message type, which can simplify the constructors of ``PublicationData`` and ``SubscriptionData``, and there is no need to pass in parameters such as ``ORB_ID(msg_name)`` to simplify the code and avoid errors.
