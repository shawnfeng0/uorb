# Add uORB to your project

There are two ways use uORB in your project:

* [**Use the uorb pre-built library binaries and headers**](#use-the-uorb-pre-built-library). Use this approach if you just want to use a stable version of the uorb library in your project.

* [**Build uorb from source**](#build-uorb-from-source). Use this approach if you would like to debug or want the latest modifications.

## Use the uORB pre-built library

It is recommended that you build uORB outside of the source directory, to avoid having issues with CMake generated files polluting the source directory:

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
#define ORB_SIMPLE_DEFINE(_name, _struct) ...
```

### Use tools to manage uORB topics

In order to manage and modify topics in a unified way, we use a message generation tool, which uses a way similar to ROS to define topics.

I am lazy : ). I am not familiar with the code generation tool at the moment, so the generation tool is still using the official PX4 (may be cut later), but the uORB topic template has been modified.

#### Add uORB topics

First, create a folder to store uORB topics (the original topics are defined using .msg files, and it will generate *.h/*.cc topic files). Then add a topic file.

```
# msg/example_string.msg

uint64 timestamp # time since system start (microseconds)

uint32 STRING_LENGTH = 128
int8[128] string
```

For the format of msg file, please refer to ros and PX4 or the examples in this project.

Use the message generation tool in the `tools/msg` directory of this project to generate `*.h` and `*.cc` topic source files from `*.msg` files. There are many tool parameters. It is currently recommended to refer to the [examples](../examples) directory of this project or the independent [uorb examples](https://github.com/ShawnFeng0/uorb-examples.git) project `msg/CMakeLists.txt` realizes message generation.

It should be noted that the address of the setting generation tool path in msg/CMakeLists.txt must be correct.

```Cmake
# Need to specify the path of generator and message template, they are in the tools/msg directory of this project.
set(tools_root ${CMAKE_CURRENT_SOURCE_DIR}/../../tools/msg)
```

#### Add uORB topic as a dependency

Add the following statement to `CMakeLists.txt` at the level of the msg directory you created to use the topic library as a dependency.

```cmake
# Generate orb message
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/msg)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/msg)

# The name of your topic library. It is a library generated after compiling all [topic].cc files. Please refer to the example implementation for details.
link_libraries(uorb_examples_msgs)
```

### Manually manage uORB topics

Although it is not recommended, we also support directly using macros to manually define topics, which may be good for beginners. You need to make sure to declare `ORB_DECLARE` in the topic header file, and define `ORB_SIMPLE_DEFINE` in the `C++` source file.

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

ORB_SIMPLE_DEFINE(example_struct_str, orb_example_struct_str);

ORB_SIMPLE_DEFINE(example_struct_number, orb_example_struct_number);
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
auto &data = pub_example_string.get();

data.timestamp = orb_absolute_time_us(); // orb_absolute_time_us() from "uorb/abs_time.h"
snprintf((char *)data.string, example_string_s::STRING_LENGTH, "%d: %s", i,
             "This is a string message.");
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
uorb::SubscriptionDatauorb::msg::example_string sub_example_string;
```

Polling for updates:

```c++
if (sub_example_string.Update()) {
  // Data processing...
}
```

Or use `orb_poll()`, which is a function similar to `poll()`(Recommend this usage):

```c++
struct orb_pollfd pollfds[] = {
  {.fd = sub_example_string.handle(), .events = POLLIN}};

if (0 < orb_poll(pollfds, ARRAY_SIZE(pollfds), timeout_ms)) {
  if (sub_example_string.Update()) {
    // Data processing...
  }
}
```

Get updated data and processing:

```c++
auto data = sub_example_string.get();
printf("timestamp: %" PRIu64 "[us], Receive msg: "%s"\n", data.timestamp,
       data.string);
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

2. **Caller owns the subscription** — build your own `uorb::SubscriptionData` and register it with `RegisterCallback(sub, cb)`. The caller must keep the subscription alive at least as long as the `EventLoop`, or call `UnregisterCallback(sub)` before the loop is destroyed.

   ```c++
   uorb::SubscriptionData<uorb::msg::sensor_accel> sub_accel;
   loop.RegisterCallback(sub_accel, [](const sensor_accel_s &msg) {
     // handle message
   });
   // ... later ...
   loop.UnregisterCallback(sub_accel);
   ```

### Driving the loop

* `PollOnce(timeout_ms)` blocks for at most `timeout_ms` milliseconds and dispatches the ready callbacks once; `timeout_ms = -1` blocks until either an event arrives or `Quit()` is called. Returns the number of events dispatched; `-1` on error or when `Quit()` has been requested; `0` when there are no registered subscriptions.
* `Loop()` calls `PollOnce(-1)` in a loop until `Quit()` is requested (returns `true`) or it sees an error / an empty loop (returns `false`).
* `Quit()` is **thread-safe** and may be called from any thread to request loop shutdown. It is *sticky*: once called, subsequent `PollOnce()` calls return `-1` immediately and the `EventLoop` cannot be restarted.

### Threading model

* `Quit()` may be called from any thread.
* All other member functions (`RegisterCallback` / `UnregisterCallback` / `Subscribe` / `PollOnce` / `Loop`) must be invoked from a single thread — typically the thread that runs the loop.

### Full examples

Minimal example: [examples/cpp_pub_sub/cpp_pub_sub_event_loop.cc](../examples/cpp_pub_sub/cpp_pub_sub_event_loop.cc)

Multi-thread / multi-topic example: [examples/cpp_pub_sub/cpp_pub_sub_event_poll.cc](../examples/cpp_pub_sub/cpp_pub_sub_event_poll.cc)
