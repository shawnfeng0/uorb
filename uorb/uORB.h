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
 * @file uORB.h
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
  const uint16_t
      o_size_no_padding; /**< object size w/o padding at the end (for logger) */
  const char *o_fields;  /**< semicolon separated list of fields (with type) */
};

typedef const struct orb_metadata *orb_id_t;

#define ORB_OK (0)
#define ORB_ERROR (-1)

/**
 * Maximum number of multi topic instances
 */
#define ORB_MULTI_MAX_INSTANCES \
  4  // This must be < 10 (because it's the last char of the node path)

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
 * @param _size_no_padding	Struct size w/o padding at the end
 * @param _fields	All fields in a semicolon separated list e.g: "float[3]
 * position;bool armed"
 * @param _orb_id_enum	ORB ID enum e.g.: ORB_ID::vehicle_status
 */
#define ORB_DEFINE(_name, _struct, _size_no_padding, _fields)            \
  const struct orb_metadata __orb_##_name = {#_name, sizeof(_struct),    \
                                             _size_no_padding, _fields}; \
  struct hack

__BEGIN_DECLS

/**
 * ORB topic advertiser handle.
 *
 * Advertiser handles are global; once obtained they can be shared freely
 * and do not need to be closed or released.
 *
 * This permits publication from interrupt context and other contexts where
 * a file-descriptor-based handle would not otherwise be in scope for the
 * publisher.
 */
typedef struct {} * orb_advert_t;

/**
 * TODO:
 */
typedef struct {} * orb_subscriber_t;

/**
 * TODO:
 * @param meta
 * @param data
 * @return
 */
extern orb_advert_t orb_advertise(const struct orb_metadata *meta,
                                  const void *data) __EXPORT;

/**
 * TODO:
 * @param meta
 * @param data
 * @param queue_size
 * @return
 */
extern orb_advert_t orb_advertise_queue(const struct orb_metadata *meta,
                                        const void *data,
                                        unsigned int queue_size) __EXPORT;

/**
 * TODO:
 * @param meta
 * @param data
 * @param instance
 * @return
 */
extern orb_advert_t orb_advertise_multi(const struct orb_metadata *meta,
                                        const void *data,
                                        unsigned int *instance) __EXPORT;

/**
 * TODO:
 * @param meta
 * @param data
 * @param instance
 * @param queue_size
 * @return
 */
extern orb_advert_t orb_advertise_multi_queue(const struct orb_metadata *meta,
                                              const void *data,
                                              unsigned int *instance,
                                              unsigned int queue_size) __EXPORT;

/**
 * TODO:
 * @param handle_ptr
 * @return
 */
extern int orb_unadvertise(orb_advert_t *handle_ptr) __EXPORT;

/**
 * TODO:
 * @param meta
 * @param handle
 * @param data
 * @return
 */
extern int orb_publish(const struct orb_metadata *meta, orb_advert_t handle,
                       const void *data) __EXPORT;

/**
 * Advertise as the publisher of a topic.
 *
 * This performs the initial advertisement of a topic; it creates the topic
 * node in /obj if required and publishes the initial data.
 *
 * @see uORB::Manager::orb_advertise_multi() for meaning of the individual
 * parameters
 */
static inline int orb_publish_auto(const struct orb_metadata *meta,
                                   orb_advert_t *handle, const void *data,
                                   unsigned int *instance) {
  if (!meta || !handle) {
    orb_errno = EINVAL;
    return ORB_ERROR;
  }

  if (!*handle) {
    *handle = orb_advertise_multi(meta, data, instance);
    return (*handle) ? ORB_OK : ORB_ERROR;

  } else {
    return orb_publish(meta, *handle, data);
  }
}

/**
 * TODO:
 * @param meta
 * @return
 */
extern orb_subscriber_t orb_subscribe(const struct orb_metadata *meta) __EXPORT;

/**
 * TODO:
 * @param meta
 * @param instance
 * @return
 */
extern orb_subscriber_t orb_subscribe_multi(const struct orb_metadata *meta,
                                            unsigned instance) __EXPORT;

/**
 * TODO:
 * @param handle_ptr
 * @return
 */
extern int orb_unsubscribe(orb_subscriber_t *handle_ptr) __EXPORT;

/**
 * TODO:
 * @param meta
 * @param handle
 * @param buffer
 * @return
 */
extern int orb_copy(const struct orb_metadata *meta, orb_subscriber_t handle,
                    void *buffer) __EXPORT;

/**
 * TODO:
 * @param handle
 * @param updated
 * @return
 */
extern int orb_check(orb_subscriber_t handle, bool *updated) __EXPORT;

/**
 * TODO:
 * @param meta
 * @param instance
 * @return
 */
extern int orb_exists(const struct orb_metadata *meta, int instance) __EXPORT;

/**
 * Get the number of published instances of a topic group
 *
 * @param meta    ORB topic metadata.
 * @return    The number of published instances of this topic
 */
extern int orb_group_count(const struct orb_metadata *meta) __EXPORT;

/**
 * TODO:
 * @param handle
 * @param interval_ms
 * @return
 */
extern int orb_set_interval(orb_subscriber_t handle,
                            unsigned interval_ms) __EXPORT;

/**
 * TODO:
 * @param handle
 * @param interval
 * @return
 */
extern int orb_get_interval(orb_subscriber_t handle,
                            unsigned *interval) __EXPORT;

__END_DECLS
