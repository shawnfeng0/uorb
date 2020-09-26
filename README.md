# uORB

The uORB is an asynchronous publish() / subscribe() messaging API used for inter-thread communication. "UORB" is the abbreviation of Micro Object Request Broker.

uORB was originally a messaging middleware in PX4 Autopilot, run on Nuttx rtos or Linux:

* [Introduction of uORB in PX4](https://dev.px4.io/master/en/middleware/uorb.html)

The project was re-implemented based on POSIX based on the original API, some redundant software layers were removed, and only most of the core functions were retained.

* [Comparison with uORB of PX4](docs/contrast_with_px4_uorb.md)

## Development status

* Unit test passed, basically available
* Documentation and engineering are being improved

## Getting started

TODO
