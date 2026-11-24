# uORB Architecture

This document describes the architecture of the uORB (micro Object Request Broker) library, including its component structure, dependencies, and design principles.

## Overview

uORB is a lightweight publish/subscribe messaging system designed for real-time embedded systems. It provides a clean separation between the core pub/sub mechanism and event loop functionality, allowing users to choose which components they need.

## Component Structure

The library is organized into three independent components:

```
┌─────────────────────────────────────────────────────────┐
│                    uorb (Core Library)                   │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐     │
│  │   Device    │  │   Device    │  │   uorb.cc   │     │
│  │   Master    │  │    Node     │  │  (C API)    │     │
│  └─────────────┘  └─────────────┘  └─────────────┘     │
└─────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────┐
│                 uevent (Event Loop Library)              │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐     │
│  │   Event     │  │   Event     │  │   uevent    │     │
│  │    Base     │  │   Source    │  │   (C API)   │     │
│  └─────────────┘  └─────────────┘  └─────────────┘     │
└─────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────┐
│            uorb_uevent (Bridge Library)                  │
│  ┌─────────────────────────────────────────────────┐    │
│  │  EventLoop / AsyncDispatcher (C++ wrappers)     │    │
│  │  uorb_subscriber_create_source() (C API)       │    │
│  └─────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────┘
```

## Component Details

### 1. uorb (Core Library)

The core pub/sub library, completely independent of any event loop implementation.

**Source files:**
- `src/device_master.cc` - Manages topic devices
- `src/device_node.cc` - Individual topic device implementation
- `src/uorb.cc` - C API implementation

**Headers:**
- `include/uorb/uorb.h` - Public C API
- `include/uorb/publication.h` - C++ Publication wrapper
- `include/uorb/publication_multi.h` - C++ PublicationMulti wrapper
- `include/uorb/subscription.h` - C++ Subscription wrapper
- `include/uorb/subscription_interval.h` - C++ SubscriptionInterval wrapper

**Dependencies:**
- None (standalone)

**Key features:**
- Thread-safe publish/subscribe
- Multiple topic instances
- Queue-based message buffering
- Callback notifications on publish

### 2. uevent (Event Loop Library)

A generic event loop library, completely independent of uORB. Can be used with any event source.

**Source files:**
- `components/uevent/src/uevent.cc` - Event loop implementation
- `components/uevent/src/uevent_source.cc` - Event source implementation

**Headers:**
- `components/uevent/include/uevent/uevent.h` - Public C API
- `components/uevent/include/uevent/uevent_source.h` - Event source C API

**Dependencies:**
- None (standalone)

**Key features:**
- Event base management (create, destroy, loop, loopbreak)
- Event source registration (add, remove)
- Event source notifications
- Event source readiness checking
- Thread-safe event source notifications

**API:**
```c
// Event base management
int uevent_create(uevent_t *base);
void uevent_destroy(uevent_t *base);
int uevent_loop(uevent_t *base, uevent_source_t *ready, int max_ready, int timeout_ms);
int uevent_loopbreak(uevent_t *base);

// Event source management
int uevent_source_create(uevent_source_t *src, uevent_ready_fn ready_fn, uevent_register_fn register_fn, uevent_unregister_fn unregister_fn, uevent_ctx_destroy_fn ctx_destroy_fn, void *ctx);
void uevent_source_destroy(uevent_source_t *source);
void uevent_source_notify(uevent_source_t *source);
int uevent_add(uevent_t *base, uevent_source_t *source, int timeout_ms);
int uevent_remove(uevent_t *base, uevent_source_t *source);
bool uevent_source_is_bound(uevent_source_t *source);
```

### 3. uorb_uevent (Bridge Library)

A bridge library that connects uORB subscriptions with the uevent event loop. This is the only component that knows about both uorb and uevent.

**Source files:**
- `components/uorb_uevent/src/uorb_uevent.cc` - Bridge implementation

**Headers:**
- `components/uorb_uevent/include/uorb_uevent/uorb_uevent.h` - Bridge API & `EventLoop` (includes `async_dispatcher.h`)
- `components/uorb_uevent/include/uorb_uevent/async_dispatcher.h` - Thread-safe `AsyncDispatcher` (background `uevent` worker thread)

**Dependencies:**
- uorb (core library)
- uevent (event loop library)

**Key features:**
- `uorb_subscriber_create_source()` - Creates an event source that wraps a uORB subscription
- `EventLoop` C++ class - Single-threaded event loop providing type-safe callback dispatch on the caller's thread
- `AsyncDispatcher` C++ class - Thread-safe asynchronous dispatcher running callbacks on a dedicated POSIX worker thread (zero STL containers, `-fno-exceptions -fno-rtti` safe)

**API:**
```c
// Bridge C API
int uorb_subscriber_create_source(uevent_source_t *src, orb_subscriber_t *sub);
void uorb_subscriber_destroy_source(uevent_source_t *source);
```

```cpp
// C++ EventLoop class
class EventLoop {
 public:
  EventLoop();
  ~EventLoop();
  
  template <const orb_metadata &meta, typename Callback>
  bool Subscribe(Callback &&cb);
  
  int RunOnce(int timeout_ms = -1);
  bool Run();
  void Quit();
};
```

## Dependency Graph

```
    ┌─────────┐     ┌─────────┐
    │  uorb   │     │ uevent  │
    │ (Core   │     │ (Event  │
    │ pub/sub)│     │  loop)  │
    └────┬────┘     └────┬────┘
         │               │
         │               │
         └───────┬───────┘
                 │
            ┌────┴───────┐
            │uorb_uevent │  (Bridge library)
            └────────────┘
```

**Key principles:**
- **Zero circular dependencies**: Each component only depends on components below it
- **Clear boundaries**: Each component has well-defined public APIs
- **Independent compilation**: Each component can be built and tested independently
- **Flexible usage**: Users can choose to use only uorb, only uevent, or both via uorb_uevent

## Directory Structure

```
uorb/
├── CMakeLists.txt                    # Main CMake configuration
├── include/
│   └── uorb/                         # uorb public headers
│       ├── uorb.h
│       ├── publication.h
│       ├── publication_multi.h
│       ├── subscription.h
│       └── subscription_interval.h
├── src/                              # uorb source files
│   ├── base/                         # Base utilities
│   │   ├── mutex.h
│   │   ├── condition_variable.h
│   │   └── intrusive_list/           # Intrusive list implementation
│   ├── device_master.cc
│   ├── device_master.h
│   ├── device_node.cc
│   ├── device_node.h
│   └── uorb.cc
├── components/
│   ├── uevent/                       # uevent component
│   │   ├── CMakeLists.txt
│   │   ├── include/
│   │   │   └── uevent/               # uevent public headers
│   │   │       ├── uevent.h
│   │   │       └── uevent_source.h
│   │   ├── src/                      # uevent source files
│   │   │   ├── event_poll.h          # Internal EventPoll implementation
│   │   │   ├── uevent.cc
│   │   │   └── uevent_source.cc
│   │   └── tests/                    # uevent tests
│   │       └── uevent_test.cc
│   └── uorb_uevent/                  # uorb_uevent component
│       ├── CMakeLists.txt
│       ├── include/
│       │   └── uorb_uevent/          # uorb_uevent public headers
│       │       └── uorb_uevent.h
│       ├── src/                      # uorb_uevent source files
│       │   └── uorb_uevent.cc
│       └── tests/                    # uorb_uevent tests
│           ├── uorb_uevent_test.cc
│           ├── event_loop_test.cc
│           └── event_poll_internal_test.cc
├── tests/                            # Test files
│   ├── msg/                          # Test message definitions
│   ├── uorb_unit_test.cc
│   ├── uorb_unit_test.h
│   └── device_master_bench.cc
├── examples/                         # Example code
│   ├── c_pub_sub/                    # C examples
│   ├── cpp_pub_sub/                  # C++ examples
│   └── tcp_topic_listener/           # TCP listener example
└── tools/
    └── uorb_tcp_topic_listener_lib/  # TCP listener library
```

## Design Principles

### 1. Zero Circular Dependencies
Each component only depends on components below it in the dependency graph. This ensures clean separation and makes it easy to understand the flow of dependencies.

### 2. Clear Public APIs
Each component exposes a well-defined public API in its `include/` directory. Internal implementation details are kept in the `src/` directory and are not exposed to users.

### 3. Independent Compilation
Each component can be built and tested independently. This makes it easy to:
- Build only the components you need
- Test components in isolation
- Reuse components in other projects

### 4. Flexible Usage
Users can choose which components to use:
- **Only uorb**: For simple pub/sub without event loops
- **Only uevent**: For generic event loop functionality
- **Both via uorb_uevent**: For integrated pub/sub with event loop support

### 5. Thread Safety
- **uorb**: Lock-free `DeviceNode::Copy()` (per-slot 4-byte atomic SeqLock), mutex-serialized writers in `Publish()`, out-of-lock callback dispatch
- **uevent**: Thread-safe event source notifications and concurrent `Add()`/`Remove()`/`Stop()`
- **uorb_uevent**: Single-threaded `EventLoop` (with thread-safe `Quit()`), and fully thread-safe `AsyncDispatcher` (`Start`/`Stop`/`Subscribe`/`Unsubscribe`)

## Notification & Callback Architecture

Executing arbitrary application callbacks synchronously on the publisher's thread while holding internal locks is a well-known source of production bugs in pub/sub libraries:

1. **Same-topic / cross-topic AB-BA deadlocks**: Callback `A` publishes `A` (self-deadlock) or publishes `B` while another thread's callback `B` publishes `A`.
2. **Hidden application-mutex AB-BA deadlocks**: Thread 1 holds application mutex `M` and calls `Publish(A)` while Thread 2's `Publish(A)` callback tries to lock `M`.
3. **Self-unregistration deadlocks / UAF**: A one-shot subscriber destroys itself inside its own callback.
4. **Stack overflow & publisher stalls**: Chained synchronous callbacks (`A -> B -> C -> ...`) grow stack depth linearly on small RTOS thread stacks, and slow consumers stall high-rate producers.

uORB addresses these systematically with a **two-layer defense**:

```
Publisher Thread (Publish)
  │
  ├─ 1. data_lock_ → write slot (atomic SeqLock) → unlock data_lock_
  │     (Copy() is 100% lock-free, never blocks on data_lock_ or callback_lock_)
  │
  └─ 2. Out-of-lock callback dispatch (DeviceNode::DispatchCallbacks)
        ├─ callback_lock_ held ONLY to advance intrusive stack cursor
        ├─ callback_lock_ RELEASED before calling entry->on_publish()
        ├─ Cross-thread UnregisterCallback waits for active_calls == 0
        ├─ Self-unregister detaches current thread's ActiveCallFrame (no deadlock / UAF)
        └─ Depth circuit breaker (kMaxSyncCallbackDepth = 16) prevents cyclic stack overflow
              │
              ├── Recommended (Business Logic): UorbEventSource::OnPublish()
              │     └─ O(1) wakeup signal only (depth == 1, no user code on publisher thread)
              │           └─► uorb::EventLoop (caller thread) OR uorb::AsyncDispatcher (worker thread)
              │                 └─ Copy() + execute business callback with O(1) stack & 0 uORB locks
              │
              └── Low-Level Hook (orb_subscriber_set_callback / Subscription::SetCallback)
                    └─ Intended for lightweight non-blocking wakeup bridges; safe against
                       callback_lock_ deadlocks even if callers re-publish or self-destroy
```

### Layer 1: Out-of-Lock Synchronous Dispatch (`DeviceNode`)
- **Zero lock-inversion deadlocks**: `DeviceNode::Publish()` never holds `data_lock_` or `callback_lock_` while executing user callbacks. It traverses `receiver_list_` using a stack-allocated intrusive cursor (`on_publish == nullptr`), increments `entry->active_calls` under `callback_lock_`, unlocks `callback_lock_`, invokes `fn(data, ctx)`, then re-locks to decrement `active_calls`.
- **Safe concurrent & self teardown**:
  - Another thread calling `UnregisterCallback()` removes the entry from `receiver_list_` and waits on `callback_cv_` until `active_calls == 0`, guaranteeing the callback has exited before callers free captured state or `ReceiverLocal`.
  - The same thread calling `UnregisterCallback()` (or `orb_subscriber_destroy()`) from inside its own callback detaches its thread-local `ActiveCallFrame`s so it neither deadlocks nor triggers use-after-free when the outer frame unwinds.
  - Destroying a sibling subscription on the same topic is also safe because `active_calls` is only held on the single entry currently executing.
- **Stack-overflow circuit breaker**: A thread-local `g_sync_callback_depth` caps recursive synchronous callback fan-out at `kMaxSyncCallbackDepth` (16). Data is still published normally to the lock-free ring buffer; only runaway synchronous cycles are broken.

### Layer 2: Asynchronous Decoupled Execution (`EventLoop` & `AsyncDispatcher`)
For all business callbacks that may publish other topics, acquire application mutexes, or perform non-trivial work, decouple notification from execution:

| Pattern | Runs Callback On | Stack Depth | Publisher Isolation | When to Use |
| :--- | :--- | :---: | :---: | :--- |
| **`uorb::EventLoop`** | Caller's thread (`Run` / `RunOnce`) | O(1) | Full | Module already owns a thread / event loop |
| **`uorb::AsyncDispatcher`** | Dedicated POSIX worker thread | O(1) | Full | Drop-in thread-safe async `Subscribe()` / `Unsubscribe()` without writing a thread loop |
| **`SetCallback` (sync)** | Publisher's thread | Bounded (`<= 16`) | None | Lightweight O(1) wakeup bridges (e.g. `UorbEventSource`) or ultra-low-latency non-blocking hooks |

### Using `uorb::AsyncDispatcher`

```cpp
#include <uorb_uevent/uorb_uevent.h>  // or <uorb_uevent/async_dispatcher.h>

uorb::AsyncDispatcher dispatcher;
dispatcher.Start();

// Thread-safe: safe to call from any thread, before or after Start().
auto token = dispatcher.Subscribe<uorb::msg::orb_test>([](const orb_test_s &msg) {
    // Runs on dispatcher's worker thread with zero uORB locks held.
    // Safe to publish other topics, copy topics, or lock application mutexes.
    printf("Received asynchronously: %d\n", msg.val);
});

// Safe from any thread (blocks until any in-flight invocation finishes)
// or even from inside the callback itself (deferred cleanup).
dispatcher.Unsubscribe(token);
dispatcher.Stop();
```

## Usage Examples

### Using only uorb (pub/sub)

```c
#include <uorb/uorb.h>

// Create a publication
orb_publisher_t pub;
orb_publisher_create(&pub, ORB_ID(orb_test));

// Create a subscription
orb_subscriber_t sub;
orb_subscriber_create(&sub, ORB_ID(orb_test));

// Publish data
orb_test_t data = { .val = 42 };
if (orb_publisher_publish(&pub, &data) != ORB_OK) {
    // handle publish failure
}

// Check for updates
if (orb_subscriber_check_update(&sub)) {
    orb_test_t received;
    orb_subscriber_copy(&sub, &received);
    printf("Received: %d\n", received.val);
}

// Cleanup
orb_subscriber_destroy(&sub);
orb_publisher_destroy(&pub);
```

### Using only uevent (event loop)

```c
#include <uevent/uevent.h>

// Create an event base
uevent_t base = UEVENT_INITIALIZER;
uevent_create(&base);

// Create a custom event source
uevent_source_t source = UEVENT_SOURCE_INITIALIZER;
uevent_source_create(&source, my_ready_fn, my_register_fn, my_unregister_fn, my_ctx_destroy_fn, my_ctx);

// Add to event base
uevent_add(&base, &source, 0);

// Wait for events
uevent_source_t ready[10] = {UEVENT_SOURCE_INITIALIZER};
int n = uevent_loop(&base, ready, 10, 1000);

// Cleanup
uevent_remove(&base, &source);
uevent_source_destroy(&source);
uevent_destroy(&base);
```

### Using uorb_uevent (integrated pub/sub with event loop)

```cpp
#include <uorb_uevent/uorb_uevent.h>

// Create an event loop
uorb::EventLoop loop;

// Subscribe to a topic with a callback
loop.Subscribe<uorb::msg::orb_test>([](const orb_test_s &msg) {
    printf("Received: %d\n", msg.val);
});

// Run the event loop
loop.Run();

// Or run once with timeout
int n = loop.RunOnce(1000);

// Quit the loop
loop.Quit();
```

## Build System

The project uses CMake for building. Key CMake targets:

- `uorb` - Core pub/sub library
- `uevent` - Event loop library
- `uorb_uevent` - Bridge library
- `uorb_test` - Unified unit and integration test suite
- `uorb_bench` - Benchmarks

### Build Options

- `UORB_BUILD_TESTS` - Build tests (default: OFF)
- `UORB_BUILD_EXAMPLES` - Build examples (default: OFF)

### Building

```bash
mkdir build && cd build
cmake .. -DUORB_BUILD_TESTS=ON -DUORB_BUILD_EXAMPLES=ON
make -j$(nproc)
ctest --output-on-failure
```

## Conclusion

The uORB library is designed with a clear separation of concerns, allowing users to choose which components they need. The three-component architecture (uorb, uevent, uorb_uevent) provides flexibility while maintaining clean dependencies and well-defined APIs.
