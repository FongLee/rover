#ifndef UART_DRIVER_H
#define UART_DRIVER_H
#include <stdbool.h>

extern int fd_gps;
extern int fd_ultr;

int uart_init(int *fd_uart, const char *serial,int baud) ;
void uart_close(int fd_uart);
int read_uart(int fd_uart, char *buf, unsigned int n);
int write_uart(int fd_uart, char *buf,unsigned int n);
#endif