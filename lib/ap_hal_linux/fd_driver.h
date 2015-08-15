#ifndef FD_DRIVER_H
#define FD_DRIVER_H

/**
 * write buf into fd, untial all the buf is sended
 * @param  fd    file descriptor to write
 * @param  buf   buffer to write
 * @param  count length of buffer
 * @return       success: equal to count; err: less than count
 */
ssize_t writen(int fd, void *buf, size_t count);

/**
 * read buf from fd, until length of buf is count
 * @param  fd    file descriptor to read
 * @param  buf   buffer from reading
 * @param  count count of buffer
 * @return       read length : 0-count
 */
ssize_t readn(int fd, void * buf, size_t count);

/**
 * judge whether we can write in a period of time
 * @param  fd       file descriptor to write
 * @param  seconds  maximum waiting time to write (seconds)
 * @param  microsec maximum waiting time to write (microsecond)
 * @return          -1: errno = ETIMEDOUT or else; 0: success,can write now
 */
int write_timeout(int fd, int seconds, int microsec);

/**
 * judge whether we can read in a period of time
 * @param  fd       file descriptor
 * @param  seconds  maximum waiting time to read (seconds)
 * @param  microsec maximum waiting time to read (microsecond)
 * @return          -1: errno = ETIMEDOUT or else; 0: success,can read now
 */
int read_timeout(int fd, int seconds, int microsec);

#endif
