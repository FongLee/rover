#ifndef TCP_DRIVER_H
#define TCP_DRIVER_H
#include <stdbool.h>

bool flag_tcp_connect;

int tcp_init(void (*myhandler)(int num));

int tcp_accept();
void tcp_destroy();
void tcp_close();
int tcp_send(const uint8_t *ch, uint16_t length);
int tcp_receive(uint8_t *ch, uint16_t length);

#endif
