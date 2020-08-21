# uORB

uORB was originally a messaging middleware in PX4 Autopilot, run on Nuttx rtos or Linux:

* [Introduction of uORB in PX4](https://dev.px4.io/master/en/middleware/uorb.html)

The project was re-implemented based on POSIX based on the original API, some redundant software layers were removed, and only most of the core functions were retained.

## Development status

* Unit test passed, basically available
* Documentation and engineering are being improved

## Getting started

TODO

## Difference from uORB of PX4 Autopilot

The main difference lies in the implementation of the bottom layer, and the application program interface changes relatively little. It also passed the uORB unit test of PX4 Autopilot.

### API interface difference

* The handle returned by the orb_subscribe() function uses ``orb_subscriber_t`` instead of ``int``.
* Pass pointers when unpublishing and unsubscribing to avoid wild pointers(Reference from zmq)
* Use ``bool`` type (include in ``<stdbool.h>``) to indicate whether the operation is successful
* Remove the concept of uORB priority(``ORB_PRIO``) completely
* Add independent ``orb_poll`` function, instead of ``px4_poll`` in PX4 Autopilot

| PX4 uORB                                                                                                                                                     | Current uORB                                                                                                                              |
| :----------------------------------------------------------------------------------------------------------------------------------------------------------- | :---------------------------------------------------------------------------------------------------------------------------------------- |
| orb_advert_t orb_advertise(const struct orb_metadata \*meta, const void \*data)                                                                              | orb_advert_t orb_advertise(const struct orb_metadata \*meta, const void \*data)                                                           |
| orb_advert_t orb_advertise_queue(const struct orb_metadata \*meta, const void \*data, unsigned int queue_size)                                               | orb_advert_t orb_advertise_queue(const struct orb_metadata \*meta, const void \*data, unsigned int queue_size)                            |
| orb_advert_t orb_advertise_multi(const struct orb_metadata \*meta, const void \*data, int \*instance ~~,enum ORB_PRIO priority~~)                            | orb_advert_t orb_advertise_multi(const struct orb_metadata \*meta, const void \*data, unsigned int \*instance)                            |
| orb_advert_t orb_advertise_multi_queue(const struct orb_metadata \*meta, const void \*data, int \*instance, ~~enum ORB_PRIO priority,~~ unsigned queue_size) | orb_advert_t orb_advertise_multi_queue(const struct orb_metadata \*meta, const void \*data, unsigned int \*instance, unsigned queue_size) |
| **int** orb_publish(const struct orb_metadata \*meta, orb_advert_t handle, const void \*data)                                                                | **bool** orb_publish(const struct orb_metadata \*meta, orb_advert_t handle, const void \*data)                                            |
| **int** orb_unadvertise(orb_advert_t **handle**)                                                                                                             | **bool** orb_unadvertise(orb_advert_t **\*handle_ptr**)                                                                                   |
| **int** orb_subscribe(const struct orb_metadata \*meta)                                                                                                      | **orb_subscriber_t** orb_subscribe(const struct orb_metadata \*meta)                                                                      |
| **int** orb_subscribe_multi(const struct orb_metadata \*meta, unsigned instance)                                                                             | **orb_subscriber_t** orb_subscribe_multi(const struct orb_metadata *meta, unsigned instance)                                              |
| **int** orb_unsubscribe(**int handle**)                                                                                                                      | **bool** orb_unsubscribe(**orb_subscriber_t \*handle_ptr**)                                                                               |
| **int** orb_copy(const struct orb_metadata \*meta, **int handle**, void \*buffer)                                                                            | **bool** orb_copy(const struct orb_metadata \*meta, **orb_subscriber_t handle**, void \*buffer)                                           |
| **int** orb_check(**int handle** ~~, bool \*updated~~)                                                                                                       | **bool** orb_check **_updated**(**orb_subscriber_t handle**)                                                                              |
| ~~int orb_priority(int handle, enum ORB_PRIO \*priority)~~                                                                                                   |                                                                                                                                           |
| **int** orb_set_interval(**int handle**, unsigned interval)                                                                                                  | **bool** orb_set_interval(**orb_subscriber_t handle**, unsigned interval_ms)                                                              |
| **int** orb_get_interval(**int handle**, unsigned \*interval)                                                                                                | **unsigned** int orb_get_interval(**orb_subscriber_t handle**)                                                                            |

### Difference in implementation

* Use read-write locks instead of regular locks to reduce latency in multi-subscription situations
* The queue size is set to a power of 2, and this implementation avoids the problem of limiting the number of messages published to ``UINTMAX_MAX``
* Delete the ``o_id ``(``ORB_ID enum``) field in ``orb_metadata``. This field has no meaning and is very redundant. It is newly added in PX4-v1.11. 
* Delete the ``o_size_no_padding`` and ``o_fields`` fields, keep it simple, and do not plan to use logger.
* In C++, ``orb_metadata`` can be obtained directly through the orb message type, which can simplify the constructors of ``PublicationData`` and ``SubscriptionData``, and there is no need to pass in parameters such as ``ORB_ID(msg_name)`` to simplify the code and avoid errors.
