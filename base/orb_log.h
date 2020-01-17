/**
 * @file px4_log.h
 * Platform dependant logging/debug implementation
 */

#pragma once

#include "../src/platforms/ulog/src/ulog.h"

#define PX4_INFO(FMT, ...) LOG_INFO(FMT, ##__VA_ARGS__)

#define PX4_INFO_RAW  LOG_RAW

#define PX4_PANIC(FMT, ...) LOG_FATAL(FMT, ##__VA_ARGS__)
#define PX4_ERR(FMT, ...) LOG_ERROR(FMT, ##__VA_ARGS__)
#define PX4_WARN(FMT, ...) LOG_WARN(FMT, ##__VA_ARGS__)
#define PX4_DEBUG(FMT, ...) ((void)0)
