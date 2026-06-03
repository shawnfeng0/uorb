# Add uORB to your project

There are two common ways to use uORB in your project:

* [**Build and install uORB**](#build-and-install-uorb). Use this when you want to consume installed headers and libraries from another project.

* [**Build uORB from source**](#build-uorb-from-source). Use this when you want to vendor uORB as a subdirectory and debug it together with your project.

## Build and install uORB

It is recommended that you build uORB outside of the source directory, so generated files stay in the build tree:

```shell
mkdir uorb-build
cd uorb-build
cmake -DCMAKE_C_COMPILER=clang     \
        -DCMAKE_CXX_COMPILER=clang++ \
        -DCMAKE_INSTALL_PREFIX=./install \
        $HOME/uorb                         # we're assuming this is where uorb is.
make
make install
```

The `./install` directory contains a lib directory and an include directory, link the libraries in the lib directory, and then add the include folder to the include path of your project.

Or refer to the x.toolchain.cmake file in the toolchain directory to write a cmake toolchain file for your corresponding platform.

## Build uORB from source

The best way to build from source code is as a subdirectory of the cmake project:

Add the source code to your project.

```shell
git submodule add https://github.com/ShawnFeng0/uorb.git # If you don't use git, just download the source code and unzip it.
```

Link the uORB library in the CMakeLists.txt of your own project.

```cmake
# uORB library
add_subdirectory(uorb)
target_link_libraries(${PROJECT_NAME} PRIVATE uorb) # Link the uorb library and add the include path of uorb
```

This is an example of using this method (**recommended**): [uorb-examples](https://github.com/ShawnFeng0/uorb-examples.git).

# Using uORB

uORB communicates based on topics, not data streams, and each topic has a corresponding structure.

The C interface of uORB is defined in uorb/uorb.h.

## Create uORB topic

The topic is defined by the following macros and needs to include the topic name and the structure corresponding to the topic.

```C++
// uorb/uorb.h
// To declare a topic, usually declare a topic in the header file.
#define ORB_DECLARE(_name, _struct) ...
```

```C++
// uorb/uorb.h
// Define a topic, usually define a topic in the source file, can only be defined once and need to be compiled in the C++ source file (*.cpp/*.cc)
#define ORB_DEFINE(_name, _struct, _queue_size) ...
```

### Use tools to manage uORB topics

In order to manage and modify topics in a unified way, uORB provides a message generation tool. Topics are described with compact `.msg` files and generated into C/C++ headers plus topic metadata sources during the CMake build.

#### Add uORB topics

First, create a folder to store `.msg` topic definitions. Each `.msg` file is generated into a topic header and source file during the CMake build.

```
# msg/example_string.msg

uint64 timestamp # required: time since system start (microseconds)

uint32 STRING_LENGTH = 128
char[128] str

uint16 ORB_QUEUE_SIZE = 3 # optional: queued samples per subscription
```

For the `.msg` format, start with the examples in this project and keep one topic per file.

Use the message generation tool in the `tools/msg` directory of this project to generate `*.h` and `*.cc` topic source files from `*.msg` files. The generated files should be written to the CMake binary directory, not committed into the source tree. It is recommended to follow the `msg/CMakeLists.txt` files under [examples](../examples) and [tests](../tests).

Make sure `tools_root` points at this repository's message generator. If CMake fails during generation, first verify Python dependencies with `pip3 install -r tools/msg/tools/requirements.txt`.

```Cmake
# Need to specify the path of generator and message template, they are in the tools/msg directory of this project.
set(tools_root ${CMAKE_CURRENT_SOURCE_DIR}/../../tools/msg)
```

#### Add uORB topic as a dependency

Add the following statement to `CMakeLists.txt` at the level of the msg directory you created to use the topic library as a dependency.

```cmake
# Generate orb message
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/msg)
target_link_libraries(${PROJECT_NAME} PRIVATE uorb_examples_msgs)

# The topic library exposes its generated binary include directory, so consumers
# can include generated headers such as <uorb/topics/example_string.h>.
```

### Manually manage uORB topics

Although it is not recommended, we also support directly using macros to manually define topics, which may be good for beginners. You need to make sure to declare `ORB_DECLARE` in the topic header file, and define `ORB_DEFINE` in the `C++` source file.

#### Declare uORB topics

Just include the `uorb/uorb.h` header file in your topic header file and declare the topic directly.

```C++
// orb_topic.h
#pragma once

#include "uorb/uorb.h"

struct orb_example_struct_str {
  char str[30];
  int32_t index;
};

ORB_DECLARE(example_struct_str, orb_example_struct_str);

struct orb_example_struct_number {
  int16_t num_int16;
  int32_t num_int32;
  int64_t num_int64;
};

ORB_DECLARE(example_struct_number, orb_example_struct_number);
```

#### Define uORB topics

Similar to the declaration, define uORB topics in the source file after including uorb/uorb.h.

```C++
// orb_topic.cc
#include "orb_topic.h"

ORB_DEFINE(example_struct_str, orb_example_struct_str, 1);

ORB_DEFINE(example_struct_number, orb_example_struct_number, 1);
```

## Publish uORB topic

Include the uORB publication header:

```C++
#include "uorb/publication.h"
```

Include the header file of the topic you want to post:

```c++
// Header file generated by topic generation tool
#include "uorb/topics/example_string.h"

// Or include manually defined topic header files
#include "orb_topic.h"
```

Define a data type for publishing, use `uorb::msg::{topic_name}` to specify the message name (`*.msg` file name or topic name declared with `ORB_DECLARE`):

```c++
uorb::PublicationData<uorb::msg::example_string> pub_example_string;
```

Fill data:

```C++
auto &data = pub_example_string.data();

data.timestamp = orb_absolute_time_us();
snprintf(reinterpret_cast<char *>(data.str), example_string_s::STRING_LENGTH,
         "%d: %s", i, "This is a string message.");
```
Publish data:

```c++
pub_example_string.Publish();
```

Please refer to the complete routine: [examples/cpp_pub_sub/cpp_pub_sub.cc](../examples/cpp_pub_sub/cpp_pub_sub.cc) 

## Subscribe to uORB topic

Include the uORB subscription header:

```c++
#include "uorb/subscription.h"
```

Define variables of the subscription data type:

```c++
uorb::SubscriptionData<uorb::msg::example_string> sub_example_string;
```

Polling for updates:

```c++
if (sub_example_string.Update()) {
  // Data processing...
}
```

Or use `orb_poll()`, which is a function similar to `poll()`(Recommend this usage):

```c++
struct orb_pollfd pollfds[] = {{.fd = sub_example_string.handle()}};

if (0 < orb_poll(pollfds, ARRAY_SIZE(pollfds), timeout_ms)) {
  if (pollfds[0].ready && sub_example_string.Update()) {
    // Data processing...
  }
}
```

Get updated data and processing:

```c++
auto data = sub_example_string.data();
printf("timestamp: %" PRIu64 "[us], Receive msg: \"%s\"\n", data.timestamp,
       data.str);
```

Please refer to the complete routine: [examples/cpp_pub_sub/cpp_pub_sub.cc](../examples/cpp_pub_sub/cpp_pub_sub.cc)

## Callback-based subscription with `EventLoop`

In addition to manual polling / `orb_poll()`, uORB ships a small C++ event loop, `uorb::EventLoop`, that wraps "wait on many subscriptions" + "run a callback per subscription" behind a simple API. It fits well on a dedicated worker thread that only runs subscription callbacks.

Include the header:

```c++
#include "uorb/event_loop.h"
```

### Two ways to register a callback

1. **`EventLoop` owns the subscription** — use `Subscribe<Topic>(callback)`. The `EventLoop` creates and holds the subscription internally, and destroys it when the loop is destroyed.

   ```c++
   uorb::EventLoop loop;
   loop.Subscribe<uorb::msg::example_string>(
       [](const example_string_s &msg) {
         // handle message
       });
   ```

2. **Caller owns the subscription** — build your own `uorb::SubscriptionData` and register it with `AddSubscription(sub, cb)`. The caller must keep the subscription alive at least as long as the `EventLoop`, or call `RemoveSubscription(sub)` before the loop is destroyed.

   ```c++
   uorb::SubscriptionData<uorb::msg::sensor_accel> sub_accel;
   loop.AddSubscription(sub_accel, [](const sensor_accel_s &msg) {
     // handle message
   });
   // ... later ...
   loop.RemoveSubscription(sub_accel);
   ```

### Driving the loop

* `RunOnce(timeout_ms)` blocks for at most `timeout_ms` milliseconds and dispatches the ready callbacks once; `timeout_ms = -1` blocks until either an event arrives or `Quit()` is called. Returns the number of events dispatched; `-1` on error or when `Quit()` has been requested; `0` when there are no registered subscriptions.
* `Run()` calls `RunOnce(-1)` in a loop until `Quit()` is requested (returns `true`) or it sees an error / an empty loop (returns `false`).
* `Quit()` is **thread-safe** and may be called from any thread to request loop shutdown. It is *sticky*: once called, subsequent `RunOnce()` calls return `-1` immediately and the `EventLoop` cannot be restarted.

### Threading model

* `Quit()` may be called from any thread.
* All other member functions (`AddSubscription` / `RemoveSubscription` / `Subscribe` / `RunOnce` / `Run`) must be invoked from a single thread — typically the thread that runs the loop.

### Full examples

See [examples/cpp_pub_sub/cpp_pub_sub_event_loop.cc](../examples/cpp_pub_sub/cpp_pub_sub_event_loop.cc) for a runnable example covering both loop-owned `Subscribe<>()` and user-owned `AddSubscription()`, multi-threaded publishers, and a cross-thread `Quit()`.

## Additional examples

* [C event poll](../examples/c_pub_sub/c_event_poll.c): waits on multiple C subscriptions with `orb_event_poll_*`.
* [C++ multi-instance publishing](../examples/cpp_pub_sub/cpp_pub_sub_multi.cc): publishes the same topic type on independent instances.
* [C++ subscription interval](../examples/cpp_pub_sub/cpp_subscription_interval.cc): throttles a fast publisher with `SubscriptionInterval`.
