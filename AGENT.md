# AGENT.md — AI Engineering Notes for uORB

This document captures critical constraints, gotchas, and design decisions
that are not obvious from reading the code alone. Read this before making
changes.

## 1. Architecture

Three libraries, each independently buildable:

```
uorb (core pub/sub)          — src/, include/uorb/
  ├── uevent (event loop)     — components/uevent/
  └── uorb_uevent (bridge)    — components/uorb_uevent/
```

- **uorb**: Topic-based publish/subscribe. No dependency on uevent.
- **uevent**: Generic event loop (poll sources, wait for readiness). No
  dependency on uorb. Can be used standalone.
- **uorb_uevent**: Bridges uORB subscriptions into uevent sources. Depends
  on both.

The C API surface lives in headers: `include/uorb/uorb.h`,
`components/uevent/include/uevent/uevent.h`,
`components/uorb_uevent/include/uorb_uevent/uorb_uevent.h`.

## 2. Build System

- **C++14** required (`CMAKE_CXX_STANDARD 14`, no extensions).
- **CMake 3.5** minimum (do not use newer-only syntax).
- **pthread** is the only threading dependency.
- Tests use GoogleTest (fetched via CMake FetchContent).
- Build with: `cmake -DBUILD_TESTS=ON ...`

## 3. RTOS Constraints

This codebase targets embedded/RTOS platforms. The following are
**forbidden** because they are unavailable or unreliable on RTOS:

- `std::mutex`, `std::lock_guard`, `std::unique_lock` — use `base::Mutex`
  / `base::LockGuard` / `base::UniqueLock` (wrappers around
  `pthread_mutex_t`), or raw `pthread_mutex_t` directly.
- `std::this_thread::yield()`, `std::this_thread::sleep_for()` — use
  `sched_yield()` or condition variable waits.
- `std::condition_variable` — use `base::ConditionVariable`
  (wraps `pthread_cond_t`).
- `std::shared_ptr`, `std::function` — use raw pointers and function
  pointers.

The two internal mutex headers (one per library) both wrap
`pthread_mutex_t` with `PTHREAD_MUTEX_INITIALIZER` support for static
storage.

## 4. Per-Library Base Headers (`base/` vs `uevent_base/`)

To keep `uorb` and `uevent` independently buildable with zero shared
dependencies, each library keeps its own private synchronization helpers
under distinct include prefixes:

- `src/base/` — namespace `uorb::base` (`#include "base/mutex.h"`)
- `components/uevent/src/uevent_base/` — namespace `uevent::base`
  (`#include "uevent_base/mutex.h"`)

Using distinct directory names prevents include-path collisions when test
or bridge targets put both `src/` and `components/uevent/src/` on the
include path. `uevent_source.h` (public header) still uses raw
`pthread_mutex_t` directly via `<pthread.h>` so public headers do not
depend on private `uevent_base/` headers.

## 5. Lock Ordering

### DeviceNode (uorb core)

```
DeviceMaster::lock_  →  data_lock_     (node lookup)
callback_lock_       →  data_lock_     (callbacks calling Copy)
```

- `data_lock_` and `callback_lock_` are **NEVER held simultaneously** in
  `Publish()`. Publish writes data under `data_lock_`, releases it, then
  invokes callbacks under `callback_lock_`.
- Callbacks executing under `callback_lock_` may call `Copy()` which
  acquires `data_lock_`. This is safe: `callback_lock_ → data_lock_`.
- **Do NOT** call `Publish()` on the same topic from a callback — that
  would acquire `data_lock_` then `callback_lock_`, creating a lock-order
  violation.

### EventPoll (uevent)

```
notify_mu_  →  EventPoll::mu_   (notify_waiters calls NotifyReady)
```

- `Remove()` releases `mu_` before calling `ClearWakeup()` (which acquires
  `notify_mu_`). This avoids AB-BA deadlock.
- `ClearWakeup()` must be called **outside** `EventPoll::mu_`.

## 6. EventPoll Destruction Race (Critical)

**Problem**: A publisher thread calls `notify_waiters()` which loads
`wakeup_` (an `EventPoll*`). If the main thread destroys the EventPoll
concurrently, the publisher dereferences freed memory.

**Fix**: `notify_mu_` (a `pthread_mutex_t` on `EventSource`) serializes
`notify_waiters()` and `ClearWakeup()`:

1. `notify_waiters()`: acquires `notify_mu_`, loads `wakeup_`, calls
   `NotifyReady()`, releases `notify_mu_`.
2. `ClearWakeup()`: acquires `notify_mu_`, sets `wakeup_` to nullptr,
   releases `notify_mu_`.

When `ClearWakeup()` returns, no `notify_waiters()` call is in flight,
so the EventPoll can be safely freed. `ClearWakeup()` is called from
`RemoveWakeup()` which is called from `~EventPoll()` and `Remove()`.

`Remove()` does a two-phase cleanup: remove from lists under `mu_`,
call `ClearWakeup()` outside `mu_`, then re-check `ready_queue_` under
`mu_` (to clean entries from a racing `NotifyReady`).

## 7. Callback API

```c
typedef union {
    void *ptr;
    uint32_t u32;
    uint64_t u64;
} orb_callback_ctx;

typedef void (*orb_subscriber_callback_fn)(const void *msg, orb_callback_ctx ctx);

orb_err orb_subscriber_set_callback(orb_subscriber_t *sub,
                                    orb_subscriber_callback_fn cb,
                                    orb_callback_ctx ctx);
```

- `msg` is the published message pointer, valid only during the
  synchronous callback (it's the caller's source buffer, not a copy).
- `ctx` uses the union so callers can pass either a pointer or an
  integer, similar to POSIX `union sigval`.
- **Callback safety**: may call `orb_subscriber_copy()` on the same
  topic (uses `data_lock_`, no conflict with `callback_lock_`). Must
  NOT call `orb_publisher_publish()` on the same topic.
- `set_callback`, `clear_callback`, and `destroy` on the same subscriber
  must not be called concurrently with each other.

### Internal flow

```
Publish(data)
  → write to ring buffer under data_lock_
  → release data_lock_
  → if callback_count_ > 0 (Dekker seq_cst fast-path):
      for each CallbackEntry under callback_lock_:
        entry.on_publish(data, entry.ctx)   // direct call to user callback
```

## 8. DeviceNode Lifecycle

DeviceNodes are **persistent**: once created by the first publisher or
subscriber, they live for the lifetime of the process. This is
intentional for embedded use cases where topics are statically defined.
Subscribers can subscribe before any publisher exists.

## 9. Opaque Handle Pattern

All public C API handles use the struct-with-void-pointer pattern:

```c
typedef struct { void* _handle; } orb_publisher_t;
typedef struct { void* _handle; } orb_subscriber_t;
typedef struct { void* _handle; } uevent_t;
typedef struct { void* _handle; } uevent_source_t;
```

This avoids ABI issues and allows C callers to stack-allocate handles.
Initializers (`ORB_PUBLISHER_INITIALIZER`, etc.) zero-initialize the
handle. Functions check `_handle != NULL` before use.

## 10. Error Codes

`orb_err` values are **negative** for errors, zero for success. Do not
map them to `errno` values. Callers test with `err != ORB_OK` or
`err < 0`.

## 11. Testing

- GoogleTest for unit and integration tests.
- **Valgrind is not available on Apple Silicon (arm64 macOS)**. Use:
  - AddressSanitizer (ASan): `cmake -DCMAKE_BUILD_TYPE=Debug \
    -DCMAKE_CXX_FLAGS="-fsanitize=address -fno-omit-frame-pointer" \
    -DCMAKE_EXE_LINKER_FLAGS="-fsanitize=address" ...`
  - macOS `leaks` tool: `leaks --atExit -- ./test_binary`
  - ThreadSanitizer (TSan): same pattern with `-fsanitize=thread`
- Tests live in `src/base/condition_variable_test.cc`, `tests/` (uorb core),
  `components/uevent/tests/`, and `components/uorb_uevent/tests/`, and are built
  into a single unified test executable `uorb_test` (run via `ctest` or
  `./build/tests/uorb_test`).
- When adding tests that create uevent sources via
  `uorb_subscriber_create_source()`, always pair with
  `uorb_subscriber_destroy_source()` to avoid leaking `UorbEventSource`.

## 12. Common Pitfalls

1. **Forgetting to destroy handles**: Every `*_create` must be paired
   with `*_destroy`. For uevent sources, `uevent_remove()` must be
   called before `uevent_source_destroy()`.

2. **Do not include private `uevent_base/` headers from public headers**:
   Public headers like `uevent_source.h` must use raw `pthread_mutex_t`
   via `<pthread.h>` rather than private `uevent_base/mutex.h`.

3. **Using std primitives**: See RTOS constraints above. Always use
   `base::` wrappers or raw pthread.

4. **Locking in callbacks**: Callbacks run under `callback_lock_`. Do
   not acquire `callback_lock_` again (deadlock). Do not call Publish
   on the same topic (lock-order violation). Calling Copy is safe.

5. **ClearWakeup inside mu_**: Never call `ClearWakeup()` while holding
   `EventPoll::mu_`. It internally calls `notify_waiters()` which
   calls `NotifyReady()` which acquires `mu_` — AB-BA deadlock.

6. **Redundant .load() under lock**: When reading an atomic variable
   that is only modified under the same lock, use the operator directly
   (e.g., `generation_` not `generation_.load()`) to avoid implying
   the value might change externally.
