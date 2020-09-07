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
 * API for the uORB lightweight object request broker.
 */

#include <stdbool.h>
#include <stdint.h>

#include "uorb/base/orb_errno.h"

#if __GNUC__ >= 4
#ifdef __EXPORT
#undef __EXPORT
#endif
#define __EXPORT __attribute__((visibility("default")))
#ifdef __PRIVATE
#undef __PRIVATE
#endif
#define __PRIVATE __attribute__((visibility("hidden")))
#else
#define __EXPORT
#define __PRIVATE
#endif

/**
 * Object metadata.
 */
struct orb_metadata {
  const char *o_name;    /**< unique object name */
  const uint16_t o_size; /**< object size */
  const uint16_t
      o_size_no_padding; /**< object size w/o padding at the end (for logger) */
  const char *o_fields;  /**< semicolon separated list of fields (with type) */
};

#ifdef __cplusplus
namespace uorb {
namespace msg {
template <const orb_metadata &>
struct TypeMap;
}
}  // namespace uorb
#endif

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
#define ORB_ID(_name) __orb_##_name()

/**
 * Declare (prototype) the uORB metadata for a topic (used by code generators).
 *
 * @param _name		The name of the topic.
 */
#if defined(__cplusplus)
#define ORB_DECLARE(_name)                \
  namespace uorb {                        \
  namespace msg {                         \
  extern const struct orb_metadata _name; \
  }                                       \
  }                                       \
  extern "C" const struct orb_metadata *__orb_##_name() __EXPORT
#else
#define ORB_DECLARE(_name) \
  extern const struct orb_metadata *__orb_##_name() __EXPORT
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
 * @param _size_no_padding	Struct size w/o padding at the end
 * @param _fields	All fields in a semicolon separated list
 *                      e.g: "float[3] position;bool armed"
 */
#define ORB_DEFINE(_name, _struct, _size_no_padding, _fields)              \
  namespace uorb {                                                         \
  namespace msg {                                                          \
  const struct orb_metadata _name = {#_name, sizeof(_struct),              \
                                     _size_no_padding, _fields};           \
  }                                                                        \
  }                                                                        \
  const struct orb_metadata *__orb_##_name() { return &uorb::msg::_name; } \
  struct hack

#ifdef __cplusplus
extern "C" {
#endif

/**
 * ORB topic advertiser handle
 *
 * "struct orb_publication" does not exist, it is only defined to hide the
 * implementation and avoid the implicit conversion of "void*" types.
 *
 * Advertiser handles are global; once obtained they can be shared freely and do
 * not need to be closed or released.
 */
typedef struct hack_orb_publication orb_publication_t;

/**
 * ORB topic subscriber handle
 *
 * "struct orb_subscriber" does not exist, it is only defined to hide the
 * implementation and avoid the implicit conversion of "void*" types.
 */
typedef struct hack_orb_subscription orb_subscription_t;

#ifndef POLLIN
#define POLLIN (0x01u)
#endif

/**
 * The auxiliary data structure used to use orb_poll, similar to the poll()
 * function of POSIX.
 *
 * Only supports POLLIN function.
 */
struct orb_pollfd {
  orb_subscription_t *fd;  // A handle returned from orb_create_subscription.
  unsigned events;         // The input event flags
  unsigned revents;        // The output event flags
};

typedef struct orb_pollfd orb_pollfd_t;

/**
 * return orb_create_publication_multi(meta, nullptr, queue_size);
 * @see orb_create_publication_multi()
 */
orb_publication_t *orb_create_publication(const struct orb_metadata *meta,
                                          unsigned int queue_size) __EXPORT;

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
 * return the same instance. Otherwise, create an independent instance of the
 * topic (each instance has its own buffer), and each call will generate an
 * independent instance (up to ORB_MULTI_MAX_INSTANCES), which is useful for
 * scenarios where multiple publishers publish the same topic.
 *
 * @param meta    The uORB metadata (usually from the ORB_ID() macro) for the
 * topic.
 *
 * @param instance  Pointer to an integer which will yield the instance ID
 * (0-based) of the publication. This is an output parameter and will be set to
 * the newly created instance, ie. 0 for the first advertiser, 1 for the next
 * and so on.
 * WARN: If it is NULL, only 0 instances will be returned, which means that if
 * there are other 0 instance publishers (by passing in NULL, or the first
 * instance), the data of multiple publishers will be sent to the same instance.
 *
 * @param queue_size  Maximum number of buffered elements. If queue_size < 2, no
 * queuing is used. Will round up to a power of 2 inside.
 *
 * @return NULL on error(No memory or too many instances), otherwise returns an
 * ORB topic advertiser handle that can be used to publish to the topic.
 */
orb_publication_t *orb_create_publication_multi(
    const struct orb_metadata *meta, unsigned int *instance,
    unsigned int queue_size) __EXPORT;

/**
 * Unadvertise a topic.
 *
 * @param handle The pointer of the handle returned from orb_advertise_xxx will
 * be destroyed and set to NULL. (In order to prevent wild pointers from
 * appearing, reference zmq project)
 * @return true on success
 */
bool orb_destroy_publication(orb_publication_t **handle_ptr) __EXPORT;

/**
 * Publish new data to a topic.
 *
 * The data is atomically published to the topic and any waiting subscribers
 * will be notified.  Subscribers that are not waiting can check the topic
 * for updates using orb_check.
 *
 * @param handle  The handle returned from orb_advertise.
 * @param data    A pointer to the data to be published.
 * @return    true on success, false with orb_errno set accordingly.
 */
bool orb_publish(orb_publication_t *handle, const void *data) __EXPORT;

/**
 * Advertise as the publisher of a topic.
 *
 * This performs the initial advertisement of a topic; it creates the topic
 * node if required and publishes the initial data.
 *
 * @see orb_advertise_multi() for meaning of the individual parameters
 */
static inline bool orb_publish_auto(const struct orb_metadata *meta,
                                    orb_publication_t **handle_ptr,
                                    const void *data, unsigned int *instance) {
  if (!meta || !handle_ptr) {
    orb_errno = EINVAL;
    return false;
  }

  if (!*handle_ptr) {
    *handle_ptr = orb_create_publication_multi(meta, instance, 1);
    if (!*handle_ptr) {
      return false;
    }
  }
  return orb_publish(*handle_ptr, data);
}

/**
 * return orb_create_subscription_multi(meta, 0);
 * @see orb_create_subscription_multi()
 */
orb_subscription_t *orb_create_subscription(
    const struct orb_metadata *meta) __EXPORT;

/**
 * Subscribe to a multi-instance of a topic.
 *
 * The returned value is a subscriber handle that can be passed to orb_poll()
 * in order to wait for updates to a topic, as well as orb_copy(),
 * orb_check().
 *
 * If there were any publications of the topic prior to the subscription,
 * an orb_check right after orb_create_subscription will return true.
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
 * subscribe to each instance with orb_create_subscription
 * (@see orb_advertise_multi()).
 *
 * @param meta    The uORB metadata (usually from the ORB_ID() macro)
 *      for the topic.
 * @param instance  The instance of the topic. Instance 0 matches the
 *      topic of the orb_create_subscription() call, higher indices
 *      are for topics created with orb_advertise_multi().
 * @return    NULL on error, otherwise returns a subscriber handle
 *      that can be used to read and update the topic.
 */
orb_subscription_t *orb_create_subscription_multi(
    const struct orb_metadata *meta, unsigned instance) __EXPORT;

/**
 * Unsubscribe from a topic.
 *
 * @param handle The pointer of the handle returned from orb_create_subscription
 * will be destroyed and set to NULL. (In order to prevent wild pointers from
 * appearing, reference zmq project)
 * @return true on success.
 */
bool orb_destroy_subscription(orb_subscription_t **handle_ptr) __EXPORT;

/**
 * Fetch data from a topic.
 *
 * This is the only operation that will reset the internal marker that
 * indicates that a topic has been updated for a subscriber. Once poll
 * or check return indicating that an update is available, this call
 * must be used to update the subscription.
 *
 * @param handle  A handle returned from orb_create_subscription.
 * @param buffer  Pointer to the buffer receiving the data.
 * @return    true on success, false otherwise with orb_errno set accordingly.
 */
bool orb_copy(orb_subscription_t *handle, void *buffer) __EXPORT;

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
 * @param handle  A handle returned from orb_create_subscription.
 * @return true if the topic has been updated since the last time it was copied
 * using this handle.
 */
bool orb_check_update(orb_subscription_t *handle) __EXPORT;

/**
 * If the message is updated, copy the message.
 * See orb_check_update() and orb_copy().
 */
static inline bool orb_check_and_copy(orb_subscription_t *handle,
                                      void *buffer) {
  return orb_check_update(handle) && orb_copy(handle, buffer);
}

/**
 * Check if a topic has already been created and published (advertised)
 *
 * @param meta    ORB topic metadata.
 * @param instance  ORB instance
 * @return true if the topic exists, false otherwise.
 */
bool orb_exists(const struct orb_metadata *meta,
                unsigned int instance) __EXPORT;

/**
 * Get the number of published instances of a topic group
 *
 * @param meta    ORB topic metadata.
 * @return    The number of published instances of this topic
 */
unsigned int orb_group_count(const struct orb_metadata *meta) __EXPORT;

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
int orb_poll(struct orb_pollfd *fds, unsigned int nfds,
             int timeout_ms) __EXPORT;

/**
 * Get orb version string
 * @return version string
 */
const char *orb_version(void);

#ifdef __cplusplus
}
#endif
