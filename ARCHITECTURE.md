# uORB Architecture

This document describes the architecture of the uORB (micro Object Request Broker) library, including its two-layer component structure, dependencies, and design principles.

## Overview

uORB is a lightweight publish/subscribe messaging system designed for real-time embedded systems. It is structured as a clean two-layer stack: a standalone generic wait/multiplexing primitive (`uevent`) at the bottom, and the uORB pub/sub library (`uorb`) directly on top.

## Component Structure

The library is organized into two layered components:

```
┌────────────────────────────────────────────────────────────────────┐
│                    uorb (Pub/Sub & Dispatchers)                    │
│  ┌──────────────┐ ┌──────────────┐ ┌────────────┐ ┌─────────────┐  │
│  │ DeviceMaster │ │  DeviceNode  │ │  uorb.cc   │ │ EventLoop / │  │
│  │              │ │ (SeqLock +   │ │ (C API +   │ │ AsyncDisp.  │  │
│  │              │ │  waiters)    │ │ EventSrc)  │ │ (C++ API)   │  │
│  └──────────────┘ └──────────────┘ └────────────┘ └─────────────┘  │
└─────────────────────────────────┬──────────────────────────────────┘
                                  │ depends on
                                  ▼
┌────────────────────────────────────────────────────────────────────┐
│                 uevent (Generic Event/Wait Primitive)              │
│  ┌──────────────┐ ┌──────────────┐ ┌────────────────────────────┐  │
│  │  EventPoll   │ │ EventSource  │ │ uevent_* C API + Timers    │  │
│  └──────────────┘ └──────────────┘ └────────────────────────────┘  │
└────────────────────────────────────────────────────────────────────┘
```

## Component Details

### 1. uevent (Generic Event/Wait Primitive)

A standalone generic event-multiplexing library with zero dependency on uORB and zero internal threads. Analogous to OS `k_poll` / `wait_queue` primitives.

**Source files:**
- `components/uevent/src/uevent.cc` - Event loop C API, manual & timer sources
- `components/uevent/src/uevent_source.cc` - `EventSource` notification & wakeup teardown
- `components/uevent/src/event_poll.h` - Intrusive zero-alloc `EventPoll` implementation

**Headers:**
- `components/uevent/include/uevent/uevent.h` - Public C API
- `components/uevent/include/uevent/uevent_source.h` - `uevent::EventSource` base class

**Dependencies:**
- POSIX `pthread` only (zero dependency on uORB)

**Key features:**
- Event base management (`uevent_create`, `uevent_destroy`, `uevent_loop`, `uevent_loopbreak`)
- Zero-allocation intrusive FIFO ready queue and per-event monotonic deadlines
- Built-in manual-trigger sources and one-shot/periodic timers
- Support for borrowed `EventSource` views (`is_owned_by_uevent() == false`)

### 2. uorb (Pub/Sub & C++ Callback Dispatchers)

The publish/subscribe library built directly on `uevent::EventSource`. Spawns **zero implicit threads** and exposes **no raw synchronous publish callback hooks**.

**Source files:**
- `src/device_master.cc` - Topic registry & lookup
- `src/device_node.cc` - Lock-free SeqLock ring buffer, loan buffer, and `EventSource` waiter list
- `src/uorb.cc` - C API and `ReceiverLocal` (inherits `uevent::EventSource` directly)
- `src/async_dispatcher.cc` - Non-template `uorb::AsyncDispatcher` implementation

**Headers:**
- `include/uorb/uorb.h` - Public C API (including `orb_subscriber_event_source()`)
- `include/uorb/publication.h` - C++ `Publication` / `PublicationData` / `LoanedMessage`
- `include/uorb/publication_multi.h` - C++ `PublicationMulti` / `PublicationMultiData`
- `include/uorb/subscription.h` - C++ `Subscription` / `SubscriptionData`
- `include/uorb/subscription_interval.h` - C++ `SubscriptionInterval`
- `include/uorb/event_loop.h` - Single-threaded, user-driven `uorb::EventLoop` (0 threads)
- `include/uorb/async_dispatcher.h` - Thread-safe `uorb::AsyncDispatcher` (explicit `Start()` worker thread)

**Dependencies:**
- `uevent` + POSIX `pthread`

**Key features:**
- Lock-free `Copy()` (per-slot 4-byte atomic SeqLock) and mutex-serialized `Publish()`
- Stack-free `Loan` API and pluggable RTOS allocator (`orb_set_allocator`, `orb_topic_preallocate`)
- Zero-allocation `orb_subscriber_event_source(sub)` borrowed `uevent_source_t` view
- `EventLoop` (caller-driven, zero threads) and `AsyncDispatcher` (explicit background worker thread)

## Directory Structure

```
uorb/
├── CMakeLists.txt
├── include/
│   └── uorb/                         # uorb public headers
│       ├── uorb.h
│       ├── publication.h
│       ├── publication_multi.h
│       ├── subscription.h
│       ├── subscription_interval.h
│       ├── event_loop.h
│       └── async_dispatcher.h
├── src/                              # uorb source files
│   ├── base/
│   │   ├── allocator.h
│   │   ├── mutex.h
│   │   └── intrusive_list/
│   ├── async_dispatcher.cc
│   ├── device_master.cc / .h
│   ├── device_node.cc / .h
│   └── uorb.cc
├── components/
│   └── uevent/                       # Standalone uevent component
│       ├── CMakeLists.txt
│       ├── include/uevent/
│       │   ├── uevent.h
│       │   └── uevent_source.h
│       ├── src/
│       │   ├── event_poll.h
│       │   ├── uevent.cc
│       │   ├── uevent_source.cc
│       │   └── uevent_base/
│       │       ├── mutex.h
│       │       └── condition_variable.h
│       └── tests/
│           ├── condition_variable_test.cc
│           └── uevent_test.cc
├── tests/                            # Unified test suite & benchmark
│   ├── msg/
│   ├── uorb_unit_test.cc / .h
│   ├── uorb_event_source_test.cc
│   ├── event_loop_test.cc
│   ├── event_poll_internal_test.cc
│   └── device_master_bench.cc
├── examples/
└── tools/
    └── uorb_tcp_topic_listener_lib/
```

## Notification & Callback Architecture

uORB intentionally does **not** expose any raw synchronous publish callback registration API. Executing arbitrary user code on the publisher's thread causes reentrancy bugs, AB-BA deadlocks with application mutexes, stack overflow on small RTOS stacks, and publisher stalls.

Instead, `Publish()` performs only an O(1) wakeup signal on bound `uevent::EventSource` waiters (identical in spirit to Linux `wake_up()` or Zephyr `k_poll_signal_raise()`):

```
Publisher Thread (Publish)
  │
  ├─ 1. data_lock_ → write slot (atomic SeqLock) → unlock data_lock_
  │     (Copy() is 100% lock-free, never blocks on data_lock_ or waiter_lock_)
  │
  └─ 2. if waiter_count_ > 0 (Dekker seq_cst fast-path):
        DeviceNode::NotifyWaiters()
          └─ waiter_lock_ → EventSource::notify_waiters() (O(1) wakeup only; NO user code)
                └─► uevent_loop() / uorb::EventLoop (caller thread, 0 implicit threads)
                    OR uorb::AsyncDispatcher (dedicated worker thread via explicit Start())
                      └─ Copy() + execute user callback with O(1) stack & 0 uORB locks
```

| Pattern | Runs Callback On | Spawns Thread | Stack Depth | When to Use |
| :--- | :--- | :---: | :---: | :--- |
| **`Copy` / `Update` + `uevent_loop`** | Caller's thread | No | O(1) | C / C++ polling or custom event loop integration |
| **`uorb::EventLoop`** (`<uorb/event_loop.h>`) | Caller's thread (`Run` / `RunOnce`) | No | O(1) | Typed topic & timer callbacks on an existing application thread |
| **`uorb::AsyncDispatcher`** (`<uorb/async_dispatcher.h>`) | Dedicated POSIX worker thread | Explicit (`Start()`) | O(1) | Thread-safe async `Subscribe()` / `Unsubscribe()` with configurable priority & stack |

### Using `uorb::AsyncDispatcher`

```cpp
#include <uorb/async_dispatcher.h>

uorb::AsyncDispatcher dispatcher;
dispatcher.Start();

// Thread-safe: safe to call from any thread, before or after Start().
auto sub = dispatcher.SubscribeScoped<uorb::msg::orb_test>([](const orb_test_s &msg) {
    // Runs on dispatcher's worker thread with zero uORB locks held.
    printf("Received asynchronously: %d\n", msg.val);
});

dispatcher.Stop();
```

### Using `uorb::EventLoop` (Zero Threads)

```cpp
#include <uorb/event_loop.h>

uorb::EventLoop loop;
loop.Subscribe<uorb::msg::orb_test>([](const orb_test_s &msg) {
    printf("Received: %d\n", msg.val);
});

loop.Run();  // Runs on caller's thread until loop.Quit()
```

## Build System

Key CMake targets:
- `uevent` - Generic event/wait primitive library (standalone)
- `uorb` - Pub/sub library + `EventLoop` / `AsyncDispatcher` (links `uevent`)
- `uorb_test` - Unified unit and integration test suite
- `uorb_bench` - Benchmarks

```bash
mkdir build && cd build
cmake .. -DUORB_BUILD_TESTS=ON -DUORB_BUILD_EXAMPLES=ON
make -j$(nproc)
ctest --output-on-failure
```
