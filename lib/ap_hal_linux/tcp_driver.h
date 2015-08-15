#ifndef TCP_DRIVER_H
#define TCP_DRIVER_H
#include <stdbool.h>

bool flag_tcp_connect;

/**
 * tcp initialization
 */
int tcp_init(void (*myhandler)(int num));

/**
 * tcp accept
 * @return 0: success; -1: error
 */
int tcp_accept();

/**
 * close fd in listening
 */
void tcp_destroy();

/**
 * close fd of connection
 */
void tcp_close();

/**
 * send data with tcp
 * @param  ch     buffer
 * @param  length length of buffer
 * @return        0: success; -1: error
 */
int tcp_send(const uint8_t *ch, uint16_t length);

/**
 * receive data with tcp
 * @param  ch     buffer
 * @param  length length of buffer
 * @return        0: success; -1: error
 */
int tcp_receive(uint8_t *ch, uint16_t length);

#endif
