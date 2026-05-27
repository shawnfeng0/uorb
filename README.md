# uORB

![Tests](https://github.com/ShawnFeng0/uorb/actions/workflows/tests.yml/badge.svg)

English | [中文](README_CN.md)

The uORB is an asynchronous publish() / subscribe() messaging API used for inter-thread communication. "UORB" is the
abbreviation of Micro Object Request Broker.

uORB was originally a messaging middleware in PX4 Autopilot, run on Nuttx rtos or Linux:

* [Introduction of uORB in PX4](https://dev.px4.io/master/en/middleware/uorb.html)

The project was re-implemented based on POSIX based on the original API, some redundant software layers were removed,
and only most of the core functions were retained. Some of these improvements and fixes have been merged upstream of
PX4.

* [Comparison with uORB of PX4](docs/comparison_with_px4_uorb.md)

## Features

* Based on POSIX, good compatibility
* Real-time response mechanism similar to poll() function
* A flexible and easy-to-use C++ interface, which can operate topics like local structures

## Dependencies

Compiling the uorb library requires the support of c++11, and most compilations are currently supported.

Officially supported build environments are POSIX platforms (Linux/macOS) with GCC/Clang.
Native Windows/MSVC builds are currently not supported.

We have a message generator that can easily generate message meta-data. These libraries are needed to make it work:

```shell
pip3 install -r tools/msg/tools/requirements.txt
```

## Documentation

* [Getting Started Guide](docs/getting_started.md)
* API reference (TODO)
* [Changelog](CHANGELOG.md)

## Examples

[uorb-examples](https://github.com/ShawnFeng0/uorb-examples.git)

Minimal multi-instance error handling pattern:

```cpp
unsigned instance = 0;
orb_publication_t *pub = orb_create_publication_multi(ORB_ID(orb_test), &instance);
if (!pub) {
  // check errno and bail out
  return;
}

orb_subscription_t *sub = orb_create_subscription_multi(ORB_ID(orb_test), instance);
if (!sub) {
  orb_destroy_publication(&pub);
  return;
}

orb_test_s msg{};
if (!orb_publish(pub, &msg)) {
  // handle publish failure
}

orb_destroy_subscription(&sub);
orb_destroy_publication(&pub);
```

## Tools

### uorb topic listener

uorb also has a [topic listener library](tools/uorb_tcp_topic_listener_lib). It is responsible for starting a tcp server, which is convenient for developers to monitor uorb topic data in real time outside the process.

Here is an [example](examples/tcp_topic_listener) of using this listener.
