/**
 * @file orb_log.h
 * Platform dependant logging/debug implementation
 */

#pragma once

#if defined(DEBUG_ORB)

#include "../sample/ulog/src/ulog.h"

#define ORB_INFO_RAW LOG_RAW
#define ORB_ERR(FMT, ...) LOG_ERROR(FMT, ##__VA_ARGS__)
#define ORB_WARN(FMT, ...) LOG_WARN(FMT, ##__VA_ARGS__)
#define ORB_INFO(FMT, ...) LOG_INFO(FMT, ##__VA_ARGS__)
#define ORB_DEBUG(FMT, ...) ((void)0)

#else

#define ORB_INFO_RAW(...) ((void)0)
#define ORB_ERR(FMT, ...) ((void)0)
#define ORB_WARN(FMT, ...) ((void)0)
#define ORB_INFO(FMT, ...) ((void)0)
#define ORB_DEBUG(FMT, ...) ((void)0)

#endif
