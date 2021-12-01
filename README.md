# uORB

![Tests](https://github.com/ShawnFeng0/uorb/workflows/Tests/badge.svg)

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

We have a message generator that can easily generate message meta-data. These libraries are needed to make it work:

```shell
pip3 install -r ${UORB_REPO_PATH}/tools/msg/tools/requirements.txt
```

## Documentation

* [Getting Started Guide](docs/getting_started.md)
* API reference (TODO)
* [Changelog](CHANGELOG.md)

## Examples

[uorb-examples](https://github.com/ShawnFeng0/uorb-examples.git)

## Tools

### uorb topic listener

uorb also has a [topic listener library](tools/uorb_tcp_topic_listener_lib). It is responsible for starting a tcp server, which is convenient for developers to monitor uorb topic data in real time outside the process.

Here is an [example](examples/tcp_topic_listener) of using this listener.
