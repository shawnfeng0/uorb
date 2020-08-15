/****************************************************************************
 *
 *   Copyright (c) 2012-2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

/**
 * @file uorb.h
 * API for the uORB lightweight object broker.
 */

#include <stdbool.h>
#include <stdint.h>

#include "uorb/base/errno.h"
#include "uorb/base/visibility.h"

/**
 * Object metadata.
 */
struct orb_metadata {
  const char *o_name;    /**< unique object name */
  const uint16_t o_size; /**< object size */
};

/**
 * Maximum number of multi topic instances
 */
#define ORB_MULTI_MAX_INSTANCES 4

/**
 * Generates a pointer to the uORB metadata structure for
 * a given topic.
 *
 * The topic must have been declared previously in scope
 * with ORB_DECLARE().
 *
 * @param _name		The name of the topic.
 */
#define ORB_ID(_name) &__orb_##_name

/**
 * Declare (prototype) the uORB metadata for a topic (used by code generators).
 *
 * @param _name		The name of the topic.
 */
#if defined(__cplusplus)
#define ORB_DECLARE(_name) \
  extern "C" const struct orb_metadata __orb_##_name __EXPORT
#else
#define ORB_DECLARE(_name) \
  extern const struct orb_metadata __orb_##_name __EXPORT
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
 * @param _name		The name of the topic.
 * @param _struct	The structure the topic provides.
 * @param _orb_id_enum	ORB ID enum e.g.: ORB_ID::vehicle_status
 */
#define ORB_DEFINE(_name, _struct)                                     \
  const struct orb_metadata __orb_##_name = {#_name, sizeof(_struct)}; \
  struct hack

__BEGIN_DECLS

/**
 * ORB topic advertiser handle, and hide implementation details.
 *
 * Advertiser handles are global; once obtained they can be shared freely
 * and do not need to be closed or released.
 *
 * This permits publication from interrupt context and other contexts where
 * a file-descriptor-based handle would not otherwise be in scope for the
 * publisher.
 *
 * Why not use void* but named struct* ?
 * void * will be implicitly converted with other void* types, and no warning
 * can be given when using the wrong API. And struct* can give warnings.
 */
typedef struct orb_advert *orb_advert_t;

/**
 * ORB topic subscriber handle, and hide implementation details.
 *
 * Why not use void* but named struct* ?
 * void * will be implicitly converted with other void* types, and no warning
 * can be given when using the wrong API. And struct* can give warnings.
 */
typedef struct orb_subscriber *orb_subscriber_t;

#ifndef POLLIN
#define POLLIN (0x01u)
#endif

/**
 * The auxiliary data structure used to use orb_poll, similar to the poll()
 * function of POSIX.
 */
struct orb_pollfd {
  orb_subscriber_t fd;  // A handle returned from orb_subscribe.
  unsigned events;      // The input event flags
  unsigned revents;     // The output event flags
};

typedef struct orb_pollfd orb_pollfd_struct_t;

/**
 * return orb_advertise_multi_queue(meta, data, nullptr, 1);
 * @see orb_advertise_multi_queue()
 */
extern orb_advert_t orb_advertise(const struct orb_metadata *meta,
                                  const void *data) __EXPORT;

/**
 * return orb_advertise_multi_queue(meta, data, nullptr, queue_size);
 * @see orb_advertise_multi_queue()
 */
extern orb_advert_t orb_advertise_queue(const struct orb_metadata *meta,
                                        const void *data,
                                        unsigned int queue_size) __EXPORT;

/**
 * return orb_advertise_multi_queue(meta, data, instance);
 * @see orb_advertise_multi_queue()
 */
extern orb_advert_t orb_advertise_multi(const struct orb_metadata *meta,
                                        const void *data,
                                        unsigned int *instance) __EXPORT;

/**
 * Advertise as the publisher of a topic.
 *
 * This performs the initial advertisement of a topic; it creates the topic
 * node if required and publishes the initial data.
 *
 * Any number of advertisers may publish to a topic; publications are atomic
 * but co-ordination between publishers is not provided by the ORB.
 *
 * If instance is nullptr, the device is a single instance, and each call will
 * return the same instance.
 * Otherwise, create an independent instance of the topic (each instance has its
 * own buffer), and each call will generate an independent instance (up to
 * ORB_MULTI_MAX_INSTANCES), which is useful for scenarios where multiple
 * publishers publish the same topic.
 *
 * @param meta    The uORB metadata (usually from the ORB_ID() macro) for the
 * topic.
 * @param data    A pointer to the initial data to be published. For topics
 * updated by interrupt handlers, the advertisement must be performed from
 * non-interrupt context.
 * @param instance  Pointer to an integer which will yield the instance ID
 * (0-based) of the publication. This is an output parameter and will be set to
 * the newly created instance, ie. 0 for the first advertiser, 1 for the next
 * and so on.
 * NOTE: If it is NULL, only 0 instances will be returned, which means that if
 * there are other 0 instance publishers (by passing in NULL, or the first
 * instance), the data of multiple publishers will be sent to the same 0
 * Instance.
 * @param queue_size  Maximum number of buffered elements. If this is 1, no
 * queuing is used.
 * @return NULL on error(No memory or too many instances), otherwise returns an
 * object pointer that can be used to publish to the topic.
 */
extern orb_advert_t orb_advertise_multi_queue(const struct orb_metadata *meta,
                                              const void *data,
                                              unsigned int *instance,
                                              unsigned int queue_size) __EXPORT;

/**
 * Unadvertise a topic.
 *
 * @param handle The pointer of the handle returned from orb_advertise_xxx will
 * be destroyed and set to NULL. (In order to prevent wild pointers from
 * appearing, learn from zmq)
 * @return true on success
 */
extern bool orb_unadvertise(orb_advert_t *handle_ptr) __EXPORT;

/**
 * Publish new data to a topic.
 *
 * The data is atomically published to the topic and any waiting subscribers
 * will be notified.  Subscribers that are not waiting can check the topic
 * for updates using orb_check.
 *
 * @param meta    The uORB metadata (usually from the ORB_ID() macro) for the
 * topic.
 * @param handle  The handle returned from orb_advertise.
 * @param data    A pointer to the data to be published.
 * @return    true on success, false with orb_errno set accordingly.
 */
extern bool orb_publish(const struct orb_metadata *meta, orb_advert_t handle,
                        const void *data) __EXPORT;

/**
 * Advertise as the publisher of a topic.
 *
 * This performs the initial advertisement of a topic; it creates the topic
 * node if required and publishes the initial data.
 *
 * @see orb_advertise_multi() for meaning of the individual parameters
 */
static inline bool orb_publish_auto(const struct orb_metadata *meta,
                                    orb_advert_t *handle, const void *data,
                                    unsigned int *instance) {
  if (!meta || !handle) {
    orb_errno = EINVAL;
    return false;
  }

  if (!*handle) {
    *handle = orb_advertise_multi(meta, data, instance);
    return *handle;

  } else {
    return orb_publish(meta, *handle, data);
  }
}

/**
 * return orb_subscribe_multi(meta, 0);
 * @see orb_subscribe_multi()
 */
extern orb_subscriber_t orb_subscribe(const struct orb_metadata *meta) __EXPORT;

/**
 * Subscribe to a multi-instance of a topic.
 *
 * The returned value is a subscriber handle that can be passed to orb_poll()
 * in order to wait for updates to a topic, as well as orb_copy(),
 * orb_check().
 *
 * If there were any publications of the topic prior to the subscription,
 * an orb_check right after orb_subscribe_multi will return true.
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
 * subscribe to each instance with orb_subscribe_multi
 * (@see orb_advertise_multi()).
 *
 * @param meta    The uORB metadata (usually from the ORB_ID() macro)
 *      for the topic.
 * @param instance  The instance of the topic. Instance 0 matches the
 *      topic of the orb_subscribe() call, higher indices
 *      are for topics created with orb_advertise_multi().
 * @return    NULL on error, otherwise returns a handle
 *      that can be used to read and update the topic.
 */
extern orb_subscriber_t orb_subscribe_multi(const struct orb_metadata *meta,
                                            unsigned instance) __EXPORT;

/**
 * Unsubscribe from a topic.
 *
 * @param handle The pointer of the handle returned from orb_subscribe will be
 * destroyed and set to NULL. (In order to prevent wild pointers from appearing,
 * learn from zmq)
 * @return true on success.
 */
extern bool orb_unsubscribe(orb_subscriber_t *handle_ptr) __EXPORT;

/**
 * Fetch data from a topic.
 *
 * This is the only operation that will reset the internal marker that
 * indicates that a topic has been updated for a subscriber. Once poll
 * or check return indicating that an update is available, this call
 * must be used to update the subscription.
 *
 * @param meta    The uORB metadata (usually from the ORB_ID() macro)
 *      for the topic.
 * @param handle  A handle returned from orb_subscribe.
 * @param buffer  Pointer to the buffer receiving the data, or NULL
 *      if the caller wants to clear the updated flag without
 *      using the data.
 * @return    true on success, false otherwise with orb_errno set accordingly.
 */
extern bool orb_copy(const struct orb_metadata *meta, orb_subscriber_t handle,
                     void *buffer) __EXPORT;

/**
 * Check whether a topic has been published to since the last orb_copy.
 *
 * This check can be used to determine whether to copy the topic when
 * not using poll(), or to avoid the overhead of calling poll() when the
 * topic is likely to have updated.
 *
 * Updates are tracked on a per-handle basis; this call will continue to
 * return true until orb_copy is called using the same handle.
 *
 * @param handle  A handle returned from orb_subscribe.
 * @return true if the topic has been updated since the last time it was copied
 * using this handle.
 */
extern bool orb_check_updated(orb_subscriber_t handle) __EXPORT;

/**
 * Check if a topic has already been created and published (advertised)
 *
 * @param meta    ORB topic metadata.
 * @param instance  ORB instance
 * @return true if the topic exists, false otherwise.
 */
extern bool orb_exists(const struct orb_metadata *meta,
                       unsigned int instance) __EXPORT;

/**
 * Get the number of published instances of a topic group
 *
 * @param meta    ORB topic metadata.
 * @return    The number of published instances of this topic
 */
extern unsigned int orb_group_count(const struct orb_metadata *meta) __EXPORT;

/**
 * Set the minimum interval between which updates are seen for a subscription.
 *
 * If this interval is set, the subscriber will not see more than one update
 * within the period.
 *
 * Specifically, the first time an update is reported to the subscriber a timer
 * is started. The update will continue to be reported via poll and orb_check,
 * but once fetched via orb_copy another update will not be reported until the
 * timer expires.
 *
 * This feature can be used to pace a subscriber that is watching a topic that
 * would otherwise update too quickly.
 *
 * @param handle  A handle returned from orb_subscribe.
 * @param interval  An interval period in milliseconds.
 * @return    OK on success, PX4_ERROR otherwise with ERRNO set accordingly.
 */
extern bool orb_set_interval(orb_subscriber_t handle,
                             unsigned interval_ms) __EXPORT;

/**
 * Get the minimum interval between which updates are seen for a subscription.
 *
 * @see orb_set_interval()
 *
 * @param handle  A handle returned from orb_subscribe.
 * @return  The interval period in milliseconds.
 */
extern unsigned int orb_get_interval(orb_subscriber_t handle) __EXPORT;

/**
 * Similar to the poll() function of POSIX.
 *
 * If none of the defined events have occurred on any selected handle, poll()
 * shall wait at least timeout milliseconds for an event to occur on any of the
 * selected handles.
 *
 * @param fds       A set of subscriber handles.
 * @param nfds      The number of orb_pollfd structures in the fds array.
 * @param timeout_ms   Maximum waiting time for poll.
 * If the value of timeout is 0, poll() shall return immediately.
 * If the value of timeout is −1, poll() shall block until a requested event
 * occurs or until the call is interrupted.
 *
 * @return  Upon successful completion, poll() shall return a
 * non-negative value. A positive value indicates the total number of file
 * descriptors that have been selected (that is, handle for which the
 * revents member is non-zero). A value of 0 indicates  that the call timed out
 * and no handle have been selected. Upon failure, poll() shall return
 * −1 and set orb_errno to indicate the error.
 */
extern int orb_poll(struct orb_pollfd *fds, unsigned int nfds,
                    int timeout_ms) __EXPORT;

__END_DECLS
