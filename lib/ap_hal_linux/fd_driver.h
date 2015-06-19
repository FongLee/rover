#ifndef FD_DRIVER_H
#define FD_DRIVER_H

ssize_t writen(int fd, void *buf, size_t count);
ssize_t readn(int fd, void * buf, size_t count);
int write_timeout(int fd, int seconds, int microsec);
int read_timeout(int fd, int seconds, int microsec);

#endif
