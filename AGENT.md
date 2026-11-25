# AGENT.md — AI Engineering Notes for uORB

This document captures critical constraints, gotchas, and design decisions
that are not obvious from reading the code alone. Read this before making
changes.

## 1. Architecture

Two cleanly layered libraries:

```
uorb (pub/sub + EventLoop/AsyncDispatcher) — src/, include/uorb/
  └── depends on uevent (generic event/wait primitive) — components/uevent/
```

- **uevent**: Generic event/wait primitive (poll sources, timers, wait for readiness). Zero dependency on uorb; zero threads. Can be used standalone.
- **uorb**: Topic-based publish/subscribe built directly on `uevent::EventSource`. Spawns zero implicit threads and exposes **no raw synchronous callback hooks** (`ReceiverLocal` inherits `uevent::EventSource` directly and is exposed as a borrowed view via `orb_subscriber_event_source()`). C++ callback dispatch is provided via `uorb::EventLoop` (user-driven, zero threads) and `uorb::AsyncDispatcher` (explicit `Start()` worker thread).

The C API surface lives in headers: `include/uorb/uorb.h` and `components/uevent/include/uevent/uevent.h`.

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
- `thread_local` — use `pthread_key_t` (`pthread_getspecific` /
  `pthread_setspecific`) with stack-allocated state so RTOS toolchains
  without C++11 TLS runtime support work cleanly.

The two internal mutex headers (one per library) both wrap
`pthread_mutex_t` with `PTHREAD_MUTEX_INITIALIZER` support for static
storage.

## 4. Per-Library Base Headers (`base/` vs `uevent_base/`)

Each library keeps its own private synchronization helpers under distinct include prefixes so test targets putting both `src/` and `components/uevent/src/` on the include path never collide:

- `src/base/` — namespace `uorb::base` (`#include "base/mutex.h"`, `"base/allocator.h"`)
- `components/uevent/src/uevent_base/` — namespace `uevent::base`
  (`#include "uevent_base/mutex.h"`, `"uevent_base/condition_variable.h"`)

`uevent_source.h` (public header) uses raw `pthread_mutex_t` directly via `<pthread.h>` so public headers do not depend on private `uevent_base/` headers.

## 5. Lock Ordering

### DeviceNode (uorb core)

```
DeviceMaster::lock_  →  data_lock_        (node lookup)
DeviceNode::Publish:
  1. data_lock_      →  (write slot, release)
  2. waiter_lock_    →  EventSource::notify_mu_  →  EventPoll::mu_  (O(1) wakeup only; NO user code)
```

- `data_lock_` only serializes concurrent writers in `Publish()`. `Copy()` is
  completely **lock-free** using per-slot 4-byte `std::atomic<uint32_t>`
  SeqLock tags (`physical_slots = (queue_size == 1) ? 2 : queue_size`, single
  allocation with `std::atomic<size_t>` payload words), so readers never block
  writers or each other.
- `data_lock_` and `waiter_lock_` are **NEVER held simultaneously**.
- `uorb` exposes **no raw synchronous user callback hook**: `DeviceNode::NotifyWaiters()`
  only calls `EventSource::notify_waiters()` on bound subscribers (which never
  invokes user code, never re-enters `DeviceNode`, and obeys the strict DAG
  `waiter_lock_ → notify_mu_ → EventPoll::mu_`).
- All user callbacks execute asynchronously off the publisher path via
  `uorb::EventLoop` (`<uorb/event_loop.h>`, caller thread) or
  `uorb::AsyncDispatcher` (`<uorb/async_dispatcher.h>`, dedicated worker thread)
  with zero uORB locks held.

### AsyncDispatcher (`<uorb/async_dispatcher.h>`)

```
AsyncDispatcher::mu_  →  EventSource::notify_mu_ / DeviceNode::waiter_lock_ / EventPoll::mu_  (Subscribe)
WorkerLoop: EventPoll::mu_ (released) → AsyncDispatcher::mu_ (released) → user callback (NO locks held)
```

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

## 7. Event Notification Model (No Raw Sync Callbacks)

`uorb` intentionally exposes **no raw synchronous callback registration API**.
Instead, every `orb_subscriber_t` (`ReceiverLocal`) directly inherits
`uevent::EventSource` and exposes a zero-allocation borrowed view:

```c
uevent_source_t orb_subscriber_event_source(orb_subscriber_t sub);
```

- Callers add this source to a `uevent_t` loop via `uevent_add()` and remove it
  via `uevent_remove()` before calling `orb_subscriber_destroy()`.
- `ReceiverLocal::is_owned_by_uevent()` returns `false`, so accidentally calling
  `uevent_source_destroy()` on a borrowed subscriber source is a safe no-op.
- C++ callers use `uorb::EventLoop` (`<uorb/event_loop.h>`, user-driven thread)
  or `uorb::AsyncDispatcher` (`<uorb/async_dispatcher.h>`, explicit worker
  thread) for type-safe callbacks.

### Internal flow

```
Publish(data)
  → write to ring buffer under data_lock_
  → release data_lock_
  → if waiter_count_ > 0 (Dekker seq_cst fast-path):
      NotifyWaiters():
        lock waiter_lock_
        for each bound ReceiverLocal: entry.source->notify_waiters()
        unlock waiter_lock_
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
- Tests live in `tests/` (uorb core, event source integration, EventLoop,
  AsyncDispatcher, EventPoll internals) and `components/uevent/tests/`
  (`uevent_test.cc`, `condition_variable_test.cc`), and are built into a single
  unified test executable `uorb_test` (run via `ctest` or `./build/tests/uorb_test`).
- Always call `uevent_remove()` on a subscriber's borrowed event source before
  calling `orb_subscriber_destroy()`.

## 12. Common Pitfalls

1. **Forgetting to destroy handles**: Every `*_create` must be paired
   with `*_destroy`. For uevent sources, `uevent_remove()` must be
   called before `uevent_source_destroy()` (or `orb_subscriber_destroy()` for
   borrowed subscriber sources).

2. **Do not include private `uevent_base/` headers from public headers**:
   Public headers like `uevent_source.h` must use raw `pthread_mutex_t`
   via `<pthread.h>` rather than private `uevent_base/mutex.h`.

3. **Using std primitives**: See RTOS constraints above. Always use
   `base::` wrappers or raw pthread.

4. **Do not expose raw synchronous publish callbacks in `uorb`**: Keeping
   `uorb` strictly event-source based (`EventSource::notify_waiters()`) prevents
   user-code reentrancy, AB-BA deadlocks, and publisher stalls by construction.
   All user callbacks go through `uorb::EventLoop` or `uorb::AsyncDispatcher`.

5. **ClearWakeup inside mu_**: Never call `ClearWakeup()` while holding
   `EventPoll::mu_`. It internally calls `notify_waiters()` which
   calls `NotifyReady()` which acquires `mu_` — AB-BA deadlock.

6. **Redundant .load() under lock**: When reading an atomic variable
   that is only modified under the same lock, use the operator directly
   (e.g., `generation_` not `generation_.load()`) to avoid implying
   the value might change externally.
