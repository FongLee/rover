#ifndef UART_DRIVER_H
#define UART_DRIVER_H
#include <stdbool.h>

extern bool flag_uart_init;

int uart_init(const char *serial,int baud) ;
void uart_close();
int read_uart(char *buf, unsigned int n);
int write_uart(char *buf,unsigned int n);
#endif