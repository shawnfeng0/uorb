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
│  │  EventLoop (C++ wrapper)                        │    │
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
- `components/uorb_uevent/include/uorb_uevent/uorb_uevent.h` - Bridge API (includes both uorb.h and uevent.h)

**Dependencies:**
- uorb (core library)
- uevent (event loop library)

**Key features:**
- `uorb_subscriber_create_source()` - Creates an event source that wraps a uORB subscription
- `EventLoop` C++ class - High-level wrapper providing type-safe callback dispatch for uORB subscriptions

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
- **uorb**: Thread-safe publish/subscribe operations
- **uevent**: Thread-safe event source notifications
- **uorb_uevent**: Thread-safe EventLoop operations

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
orb_publisher_publish(&pub, &data);

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
- `uorb_unittest` - Unit tests
- `uorb_test` - Integration tests
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
