/**
 * @file uevent.h
 * Generic event loop C API. Independent of uORB — knows nothing about
 * topics, subscriptions, or publications.
 */

#pragma once

#include <errno.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Opaque handle to an event base (the event loop itself).
 */
typedef struct uevent_loop uevent_t;

/**
 * Opaque handle to an event source (anything that can signal readiness).
 */
typedef struct uevent_source uevent_source_t;

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
 * @return event base handle, or NULL on error
 */
uevent_t *uevent_create(void);

/**
 * Destroy an event base and free all resources.
 *
 * All event sources must be removed (via uevent_remove()) before calling
 * this function. The required lifecycle is:
 *   uevent_remove(base, source) → uevent_source_destroy(source) → uevent_destroy(base)
 *
 * @param base event base handle (may be NULL)
 */
void uevent_destroy(uevent_t *base);

/**
 * Wait for event sources to become ready.
 *
 * @param base      event base handle
 * @param ready     output array of ready event source handles
 * @param max_ready max number of output handles (must be > 0)
 * @param timeout_ms timeout in ms (0: return immediately, <0: block)
 * @return number of ready sources (>= 0), or -1 on error/loopbreak
 */
int uevent_loop(uevent_t *base, uevent_source_t *ready[], int max_ready, int timeout_ms);

/**
 * Break the current uevent_loop() call (non-sticky).
 *
 * Thread-safe. Causes the blocked uevent_loop() to return -1.
 * Subsequent loop calls behave normally.
 *
 * @param base event base handle
 * @return 0 on success, -1 if base is NULL
 */
int uevent_loopbreak(uevent_t *base);

/**
 * Create a custom event source.
 *
 * @param ready_fn       callback to check if data is ready (must not be NULL)
 * @param register_fn    callback called on uevent_add (may be NULL)
 * @param unregister_fn  callback called on uevent_remove (may be NULL)
 * @param ctx_destroy_fn callback called when source is destroyed (may be NULL)
 * @param ctx            user context pointer passed to all callbacks
 * @return event source handle, or NULL on error
 */
uevent_source_t *uevent_source_create(uevent_ready_fn ready_fn,
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
 * @param source event source handle (may be NULL)
 */
void uevent_source_destroy(uevent_source_t *source);

/**
 * Signal that data is available on an event source.
 *
 * Thread-safe. Call this from any thread when new data arrives.
 * Wakes the blocked uevent_loop() and causes the source to be
 * reported as ready (subject to the ready callback returning true).
 *
 * @param source event source handle
 */
void uevent_source_notify(uevent_source_t *source);

/**
 * Add an event source to the event base.
 *
 * @param base       event base handle
 * @param source     event source handle
 * @param timeout_ms per-event timeout in ms (0 = no timeout, >0 = fire after N ms)
 * @return 0 on success, -1 on error (errno = EBUSY if already bound)
 */
int uevent_add(uevent_t *base, uevent_source_t *source, int timeout_ms);

/**
 * Remove an event source from the event base.
 * @param base   event base handle
 * @param source event source handle
 * @return 0 on success, -1 on error
 */
int uevent_remove(uevent_t *base, uevent_source_t *source);

/**
 * Check if an event source is currently bound to an event base.
 * @param source event source handle
 * @return true if bound, false otherwise
 */
bool uevent_source_is_bound(uevent_source_t *source);

#ifdef __cplusplus
}
#endif
