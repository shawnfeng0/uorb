/**
 * @file px4_compat.h
 *
 * PX4-compatible uORB API shim.
 *
 * This header provides a thin compatibility layer over the native uORB API
 * so that code originally written for PX4 Autopilot's uORB can be adapted
 * with minimal changes.  Link against the `uorb_px4_compat` CMake target to
 * use this header.
 *
 * Key differences from the native uORB API that this shim hides:
 *  - Publisher handle: `orb_advert_t` is a plain `void *` (matching PX4)
 *    instead of the native `orb_publisher_t` struct.
 *  - Subscriber handle: integer file-descriptor style (`int`) instead of
 *    the native `orb_subscriber_t` struct.
 *  - Return values: functions return `int` (0 on success, -1 on error),
 *    matching PX4, instead of `orb_err`.
 *  - `px4_poll()` and `px4_pollfd_struct_t` provide a POSIX-poll-style
 *    blocking wait over uORB subscriptions.
 *
 * Limitations:
 *  - `orb_set_interval()` / `orb_get_interval()` store the value but do
 *    not throttle message delivery (interval filtering is not implemented
 *    in the core library).
 *  - `px4_pollfd_struct_t::fd` values are internal identifiers managed by
 *    this shim; they are *not* OS-level file descriptors and cannot be
 *    mixed with real fds in a POSIX `poll()` call.
 *  - At most `ORB_PX4_COMPAT_MAX_SUBSCRIBERS` (default 128) concurrent
 *    subscriptions are supported.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <uorb/uorb.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -----------------------------------------------------------------------
 * Publisher handle
 * ----------------------------------------------------------------------- */

/** PX4-compatible publisher handle.  NULL means invalid / unadvertised. */
typedef void *orb_advert_t;

/** Sentinel value for an invalid / uninitialised advertiser handle. */
#define ORB_ADVERT_INVALID NULL

/* -----------------------------------------------------------------------
 * Poll support
 * ----------------------------------------------------------------------- */

/** Events bitmask: data is available to read. */
#ifndef POLLIN
#define POLLIN 0x0001
#endif

/**
 * PX4-compatible poll file-descriptor structure.
 *
 * `fd` is an integer subscription handle returned by orb_subscribe() or
 * orb_subscribe_multi().  Set `events` to POLLIN before calling px4_poll();
 * after the call `revents` will be set to POLLIN for each fd that has new
 * data.
 */
typedef struct {
  int fd;      /**< subscription handle (from orb_subscribe / orb_subscribe_multi) */
  short events;   /**< requested events (set to POLLIN) */
  short revents;  /**< returned events (POLLIN if data available) */
} px4_pollfd_struct_t;

/* -----------------------------------------------------------------------
 * Publication API  (PX4 style)
 * ----------------------------------------------------------------------- */

/**
 * Advertise a topic and publish the first message.
 *
 * Equivalent to PX4's orb_advertise().
 *
 * @param meta    Topic metadata (ORB_ID macro).
 * @param data    Pointer to the initial data to publish.
 * @return        Valid advertiser handle on success, ORB_ADVERT_INVALID on error.
 */
orb_advert_t orb_advertise(const struct orb_metadata *meta, const void *data);

/**
 * Advertise a topic with an explicit queue size.
 *
 * The queue_size parameter is accepted for API compatibility but is ignored
 * at runtime – queue size is configured in the topic's ORB_DEFINE macro.
 *
 * @param meta       Topic metadata.
 * @param data       Initial data to publish.
 * @param queue_size Desired queue depth (ignored, see note above).
 * @return           Valid advertiser handle on success, ORB_ADVERT_INVALID on error.
 */
orb_advert_t orb_advertise_queue(const struct orb_metadata *meta, const void *data,
                                 unsigned int queue_size);

/**
 * Advertise a multi-instance topic.
 *
 * @param meta      Topic metadata.
 * @param data      Initial data to publish.
 * @param instance  [out] Instance index assigned to this advertiser (0-based).
 * @return          Valid advertiser handle on success, ORB_ADVERT_INVALID on error.
 */
orb_advert_t orb_advertise_multi(const struct orb_metadata *meta, const void *data,
                                 int *instance);

/**
 * Advertise a multi-instance topic with an explicit queue size.
 *
 * @param meta       Topic metadata.
 * @param data       Initial data to publish.
 * @param instance   [out] Instance index assigned.
 * @param queue_size Desired queue depth (ignored, see orb_advertise_queue()).
 * @return           Valid advertiser handle on success, ORB_ADVERT_INVALID on error.
 */
orb_advert_t orb_advertise_multi_queue(const struct orb_metadata *meta, const void *data,
                                       int *instance, unsigned int queue_size);

/**
 * Publish new data to an already-advertised topic.
 *
 * @param meta    Topic metadata (used for type safety; may be NULL).
 * @param handle  Advertiser handle from orb_advertise / orb_advertise_multi.
 * @param data    Data to publish.
 * @return        0 on success, -1 on error.
 */
int orb_publish(const struct orb_metadata *meta, orb_advert_t handle, const void *data);

/**
 * Unadvertise a topic and release the publisher handle.
 *
 * @param handle  Pointer to the advertiser handle; set to ORB_ADVERT_INVALID on success.
 * @return        0 on success, -1 on error.
 */
int orb_unadvertise(orb_advert_t *handle);

/* -----------------------------------------------------------------------
 * Subscription API  (PX4 style)
 * ----------------------------------------------------------------------- */

/**
 * Subscribe to topic instance 0.
 *
 * @param meta  Topic metadata.
 * @return      Non-negative integer subscription handle, or -1 on error.
 */
int orb_subscribe(const struct orb_metadata *meta);

/**
 * Subscribe to a specific multi-instance topic.
 *
 * @param meta      Topic metadata.
 * @param instance  Instance index (0-based).
 * @return          Non-negative integer subscription handle, or -1 on error.
 */
int orb_subscribe_multi(const struct orb_metadata *meta, unsigned instance);

/**
 * Unsubscribe and release a subscription handle.
 *
 * @param handle  Subscription handle from orb_subscribe / orb_subscribe_multi.
 * @return        0 on success, -1 on error.
 */
int orb_unsubscribe(int handle);

/**
 * Copy data from a topic into the provided buffer.
 *
 * Marks the subscription as "up to date" for the purpose of orb_check().
 *
 * @param meta    Topic metadata (used for type safety; may be NULL).
 * @param handle  Subscription handle.
 * @param buffer  Destination buffer.
 * @return        0 on success, -1 on error.
 */
int orb_copy(const struct orb_metadata *meta, int handle, void *buffer);

/**
 * Check whether new data has been published since the last orb_copy().
 *
 * @param handle   Subscription handle.
 * @param updated  [out] Set to true if new data is available.
 * @return         0 on success, -1 on error.
 */
int orb_check(int handle, bool *updated);

/**
 * Set the minimum publication interval for a subscription.
 *
 * The value is stored per-handle for API compatibility.  The core library
 * does not enforce interval throttling; callers must gate their own copies.
 *
 * @param handle       Subscription handle.
 * @param interval_ms  Minimum interval in milliseconds.
 * @return             0 on success, -1 on error.
 */
int orb_set_interval(int handle, unsigned interval_ms);

/**
 * Get the stored minimum publication interval for a subscription.
 *
 * @param handle       Subscription handle.
 * @param interval_ms  [out] Current interval in milliseconds.
 * @return             0 on success, -1 on error.
 */
int orb_get_interval(int handle, unsigned *interval_ms);

/* -----------------------------------------------------------------------
 * Poll API  (PX4 style)
 * ----------------------------------------------------------------------- */

/**
 * Wait for one or more subscriptions to have new data, with a timeout.
 *
 * Modelled after POSIX poll(2).  Each entry in @p fds must have its `fd`
 * field set to a valid subscription handle and its `events` field set to
 * POLLIN.  After the call `revents` is set to POLLIN for each fd that has
 * data, or 0 if it does not.
 *
 * @param fds        Array of poll descriptors.
 * @param nfds       Number of entries in @p fds.
 * @param timeout_ms Timeout in milliseconds.  0 returns immediately;
 *                   negative blocks indefinitely.
 * @return           Number of fds that became ready (>= 0), or -1 on error.
 */
int px4_poll(px4_pollfd_struct_t *fds, unsigned int nfds, int timeout_ms);

#ifdef __cplusplus
}
#endif
