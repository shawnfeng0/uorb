#pragma once

#include <stdio.h>
#include <string.h>

// Precompiler define to get only filename;
#if !defined(__FILENAME__)
#define __FILENAME__                                       \
  (strrchr(__FILE__, '/')    ? strrchr(__FILE__, '/') + 1  \
   : strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 \
                             : __FILE__)
#endif

#define LOGGER_LEVEL_TRACE "T"
#define LOGGER_LEVEL_DEBUG "D"
#define LOGGER_LEVEL_INFO "I"
#define LOGGER_LEVEL_WARN "W"
#define LOGGER_LEVEL_ERROR "E"
#define LOGGER_LEVEL_FATAL "F"

#define LOGGER_RESET_COLOR "\x1b[0m"
#define LOGGER_TRACE_COLOR "\x1b[37m"
#define LOGGER_DEBUG_COLOR "\x1b[34m"
#define LOGGER_INFO_COLOR "\x1b[32m"
#define LOGGER_WARN_COLOR "\x1b[33m"
#define LOGGER_ERROR_COLOR "\x1b[31m"
#define LOGGER_FATAL_COLOR "\x1b[35m"

#define LOGGER_OUT(level, fmt, ...)                                    \
  do {                                                                 \
    printf("%s/(%s:%d %s) %s" fmt LOGGER_RESET_COLOR "\r\n",           \
           LOGGER_LEVEL_##level, __FILENAME__, __LINE__, __FUNCTION__, \
           LOGGER_##level##_COLOR, ##__VA_ARGS__);                     \
  } while (0)

#define LOGGER_TRACE(fmt, ...) LOGGER_OUT(TRACE, fmt, ##__VA_ARGS__)
#define LOGGER_DEBUG(fmt, ...) LOGGER_OUT(DEBUG, fmt, ##__VA_ARGS__)
#define LOGGER_INFO(fmt, ...) LOGGER_OUT(INFO, fmt, ##__VA_ARGS__)
#define LOGGER_WARN(fmt, ...) LOGGER_OUT(WARN, fmt, ##__VA_ARGS__)
#define LOGGER_ERROR(fmt, ...) LOGGER_OUT(ERROR, fmt, ##__VA_ARGS__)
#define LOGGER_FATAL(fmt, ...) LOGGER_OUT(FATAL, fmt, ##__VA_ARGS__)
