#ifndef UDP_DRIVER_H
#define UDP_DRIVER_H

#include <stdbool.h>

extern bool flag_udp_init;

int udp_init(char *ip);
void udp_close();
int udp_send(const uint8_t *ch, uint16_t length);
int udp_receive(uint8_t *ch, uint16_t length);

#endif