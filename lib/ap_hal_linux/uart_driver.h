#ifndef UART_DRIVER_H
#define UART_DRIVER_H
#include <stdbool.h>

extern int fd_gps;
extern int fd_ultr;

/**
 * uart initialize
 * @param  fd_uart [description]
 * @param  serial  [description]
 * @param  baud    0: B115200; 1: B9600
 * @return         0: success; -1: err
 */
int uart_init(int *fd_uart, const char *serial,int baud);

/**
 * close uart
 * @param fd_uart [description]
 */
void uart_close(int *fd_uart);

/**
 * read characters from uart in noblocking ways
 * @param  fd_uart [description]
 * @param  buf     [description]
 * @param  n       [description]
 * @return         [description]
 */
int read_uart(int fd_uart, char *buf, unsigned int n);

/**
 * write characters from uart in noblocking ways
 * @param  fd_uart [description]
 * @param  buf     [description]
 * @param  n       [description]
 * @return         [description]
 */
int write_uart(int fd_uart, char *buf,unsigned int n);

#endif
