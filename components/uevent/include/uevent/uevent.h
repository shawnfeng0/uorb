/**
 * @file uevent.h
 * Generic event loop C API. Independent of uORB — knows nothing about
 * topics, subscriptions, or publications.
 */

#pragma once

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Event base handle (the event loop itself).
 * Opaque handle with private void* pointer.
 */
typedef struct {
  void* _handle;
} uevent_t;

#define UEVENT_INITIALIZER {NULL}

/**
 * User-defined event payload carried by uevent_source_t (similar to epoll_data_t).
 * Set before uevent_add() and returned unchanged in ready[i].data by uevent_loop().
 */
typedef union {
  void *ptr;
  uint32_t u32;
  uint64_t u64;
} uevent_data_t;

/**
 * Event source handle (anything that can signal readiness).
 * Opaque handle with private void* pointer and caller-owned user data.
 */
typedef struct {
  void *_handle;
  uevent_data_t data;
} uevent_source_t;

#define UEVENT_SOURCE_INITIALIZER {NULL, {NULL}}

/**
 * Callback to check whether an event source has data ready.
 * @param ctx user context pointer
 * @return true if data is ready
 */
typedef bool (*uevent_ready_fn)(void *ctx);

/**
 * Callback invoked when the source is added to an event base (via uevent_add).
 * Use this to start receiving notifications (e.g., register a callback).
 * @param ctx user context pointer
 * @return true on success, false on error
 */
typedef bool (*uevent_register_fn)(void *ctx);

/**
 * Callback invoked when the source is removed from an event base (via uevent_remove).
 * Use this to stop receiving notifications and clean up.
 * @param ctx user context pointer
 * @return true on success
 */
typedef bool (*uevent_unregister_fn)(void *ctx);

/**
 * Callback type called when the event source is destroyed.
 * Use this to free the user context if needed.
 * @param ctx user context pointer
 */
typedef void (*uevent_ctx_destroy_fn)(void *ctx);

/**
 * Create an event base.
 * @param ev event base handle to initialize
 * @return 0 on success, -1 on error
 */
int uevent_create(uevent_t *ev);

/**
 * Destroy an event base and free all resources.
 *
 * All event sources must be removed (via uevent_remove()) before calling
 * this function. The required lifecycle is:
 *   uevent_remove(&ev, &src) -> uevent_source_destroy(&src) -> uevent_destroy(&ev)
 *
 * @param ev event base handle (may be NULL)
 */
void uevent_destroy(uevent_t *ev);

/**
 * Wait for event sources to become ready.
 *
 * @param ev        event base handle
 * @param ready     output array of ready event source handles
 * @param max_ready max number of output handles (must be > 0)
 * @param timeout_ms timeout in ms (0: return immediately, <0: block)
 * @return number of ready sources (>= 0), or -1 on error/loopbreak
 */
int uevent_loop(uevent_t *ev, uevent_source_t *ready, int max_ready, int timeout_ms);

/**
 * Break the current uevent_loop() call (non-sticky).
 *
 * Thread-safe. Causes the blocked uevent_loop() to return -1.
 * Subsequent loop calls behave normally.
 *
 * @param ev event base handle
 * @return 0 on success, -1 if ev is NULL
 */
int uevent_loopbreak(uevent_t *ev);

/**
 * Create a custom event source.
 *
 * @param src           event source handle to initialize
 * @param ready_fn      callback to check if data is ready (must not be NULL)
 * @param register_fn   callback called on uevent_add (may be NULL)
 * @param unregister_fn callback called on uevent_remove (may be NULL)
 * @param ctx_destroy_fn callback called when source is destroyed (may be NULL)
 * @param ctx           user context pointer passed to all callbacks
 * @return 0 on success, -1 on error
 */
int uevent_source_create(uevent_source_t *src,
                         uevent_ready_fn ready_fn,
                         uevent_register_fn register_fn,
                         uevent_unregister_fn unregister_fn,
                         uevent_ctx_destroy_fn ctx_destroy_fn,
                         void *ctx);

/**
 * Destroy an event source.
 *
 * The source must be removed from the event base (via uevent_remove())
 * before calling this function.
 *
 * @param src event source handle (may be NULL)
 */
void uevent_source_destroy(uevent_source_t *src);

/**
 * Signal that data is available on an event source.
 *
 * Thread-safe. Call this from any thread when new data arrives.
 * Wakes the blocked uevent_loop() and causes the source to be
 * reported as ready (subject to the ready callback returning true).
 *
 * @param src event source handle
 */
void uevent_source_notify(uevent_source_t *src);

/**
 * Add an event source to the event base.
 *
 * @param ev        event base handle
 * @param src       event source handle
 * @param timeout_ms per-event timeout in ms (0 = no timeout, >0 = fire after N ms)
 * @return 0 on success, -1 on error (errno = EBUSY if already bound)
 */
int uevent_add(uevent_t *ev, uevent_source_t *src, int timeout_ms);

/**
 * Remove an event source from the event base.
 * @param ev  event base handle
 * @param src event source handle
 * @return 0 on success, -1 on error
 */
int uevent_remove(uevent_t *ev, uevent_source_t *src);

/**
 * Check if an event source is currently bound to an event base.
 * @param src event source handle
 * @return true if bound, false otherwise
 */
bool uevent_source_is_bound(uevent_source_t *src);

/**
 * Create a manual (software-triggered) event source.
 *
 * Thread-safe. Useful for waking a blocked uevent_loop() from another thread
 * without writing custom ready/register callbacks. Remove via uevent_remove()
 * and destroy via uevent_source_destroy().
 *
 * @param src event source handle to initialize
 * @return 0 on success, -1 on error
 */
int uevent_manual_source_create(uevent_source_t *src);

/**
 * Trigger a manual event source (marks ready and wakes any waiting loop).
 * Thread-safe.
 *
 * @param src manual event source handle
 * @return 0 on success, -1 if src is invalid or not a manual source (errno = EINVAL)
 */
int uevent_manual_source_trigger(uevent_source_t *src);

/**
 * Reset a manual event source to the not-ready state.
 * Thread-safe.
 *
 * @param src manual event source handle
 * @return 0 on success, -1 if src is invalid or not a manual source (errno = EINVAL)
 */
int uevent_manual_source_reset(uevent_source_t *src);

/**
 * Create a built-in monotonic timer event source (zero extra threads or OS timerfd).
 *
 * When added to an event base via uevent_add(), the source fires after period_ms.
 * If periodic is true, it automatically re-arms for the next period on each firing
 * (catching up if delayed to avoid callback storms). Remove via uevent_remove()
 * and destroy via uevent_source_destroy().
 *
 * @param src       event source handle to initialize
 * @param period_ms timer interval in milliseconds (must be > 0)
 * @param periodic  true for repeating timer, false for one-shot timer
 * @return 0 on success, -1 on error (errno = EINVAL or ENOMEM)
 */
int uevent_timer_source_create(uevent_source_t *src, uint32_t period_ms, bool periodic);

#ifdef __cplusplus
}
#endif
