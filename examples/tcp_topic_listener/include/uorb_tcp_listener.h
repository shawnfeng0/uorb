//
// Created by shawnfeng on 2021/9/30.
//

#pragma once

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
void orb_tcp_listener_init();

#ifdef __cplusplus
}
#endif
