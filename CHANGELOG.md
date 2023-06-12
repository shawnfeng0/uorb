# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/), and this project adheres
to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

[Unreleased]: https://github.com/ShawnFeng0/uorb/compare/v0.3.0...HEAD

## [0.3.0] - 2023-06-13
[0.3.0]: https://github.com/ShawnFeng0/uorb/compare/v0.2.3...v0.3.0

### Added

- Support custom uorb listener port
- Add ORB_QUEUE_SIZE to the orb_metadata structure
- Automatically configures the queue according to the topic's metadata
- Support macos platform
  - Make ConditionVariable support macos platform
  - Add unit test for ConditionVariable
  - Remove the functional interface of the wait_until class in ConditionVariable, which is not actually a commonly used interface

### Changed

- Make the tcp listener library use the uorb::listener namespace
- ORB_ID(name) uses a global pointer variable instead of a global function
- msg_gen: Add dependent python libraries to reduce usage complexity
- compile: Simplify cmake scripts
  - Add cmake policy configuration file: cmake/policy.cmake 
  - Add CMP0135 cmake strategy to prevent warning when downloading google test library 
  - Add googletest download script: cmake/googletest.cmake
  - Add a cmake macro to get the git version: cmake/git_version.cmake

### Fixed

- topic_listener: when NDEBUG is defined, the judgment in assert will be optimized away

## [0.2.3] - 2021-10-10
[0.2.3]: https://github.com/ShawnFeng0/uorb/compare/v0.2.2...v0.2.3

### Added

- Add `orb_get_topic_status()` interface to get topics status, such as the number of publish subscribers, publish index, etc.
- [optional] Add topics tcp listener library. The tcp client (such as `netcat`) can be used outside the application to connect to the topics listener server to obtain the status of uorb's topics, especially the topic data (10HZ) can be obtained in real time.

## [0.2.2] - 2021-09-09

[0.2.2]: https://github.com/ShawnFeng0/uorb/compare/v0.2.1...v0.2.2

### Fixed

- orb_poll is invalid when using -1 time to wait (64bit system)
- An anonymous publisher alone cannot publish a message
- Make the parameters of the orb_elapsed_time_us function only use values instead of pointers
- Subscription_interval.h compile error
- To build using modern cmake syntax, only target_link_libraries(xxx uorb) is required, and the header file path is no
  longer required.
- The number of publishers is incorrect
- When the queue length is 1, the subscriber index is not updated to the latest in time

### Added

- Add time constant literals (_s, _ms, _us)
- Add MIT license

### Removed

- Delete unused file: base/semaphore.h

## [0.2.1] - 2021-02-28

[0.2.1]: https://github.com/ShawnFeng0/uorb/compare/v0.2.0...v0.2.1

### Fixed

- Crash when orb_poll was used for multiple subscriptions

## [0.2.0] - 2021-02-28

[0.2.0]: https://github.com/ShawnFeng0/uorb/compare/v0.1.0...v0.2.0

### Added

- Add interfaces for anonymously publishing and copying topic data
- Add Chinese version of README
- Add DeviceNode::publisher_count_ statistics
- uORB::Publication*: template parameter automatically obtains the queue size according to the type

## [0.1.0] - 2020-10-9

[0.1.0]: https://github.com/ShawnFeng0/uorb/releases/tag/v0.1.0
