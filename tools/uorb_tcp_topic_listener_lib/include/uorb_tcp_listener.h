//
// Created by shawnfeng on 2021/9/30.
//

#pragma once

#include <stddef.h>

/**
 * @brief Callback function to return theme metadata
 */
typedef const struct orb_metadata *const *(*orb_get_topics_callback)(
    size_t *size);

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Start a tcp shell for the uorb topic
 *
 * port: 10924
 *
 * Use the following script to connect:
 *   stty -echo -icanon && nc localhost 10924
 */
void orb_tcp_listener_init(orb_get_topics_callback callback);

#ifdef __cplusplus
}
#endif
