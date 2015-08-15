#ifndef SOCKETLIB_H
#define SOCKETLIB_H

#define MAX_BUF_SIZE	 	1024
#define SOCKET_OK 			0
#define SOCKET_BASE_ERR 	4000
#define SOCKET_PAR_ERR 		(SOCKET_BASE_ERR + 1)
#define SOCKET_TIMEOUT_ERR 	(SOCKET_BASE_ERR + 2)
#define SOCKET_MALLOC_ERR	(SOCKET_BASE_ERR + 3)
#define SOCKET_UNCOON_ERR		(SOCKET_BASE_ERR + 4)


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
 * socket of client initialization
 * @param  sockhandle sock handle structure
 * @param  contime    maximum  of connect time (second)
 * @param  sendtime   maximum  of send time (second)
 * @param  revtime    maximum of receive time (second)
 * @param  connum     maximum number of connect
 * @return            0: success; >0: err, including SOCKET_PAR_ERR SOCKET_MALLOC_ERR
 */
int clt_socket_init(void **sockhandle, int contime, int sendtime, int revtime);

/**
 * build connection
 * @param  sockhandle sock handle structure
 * @param  ip         ip of server
 * @param  port       port of server
 * @param  connfd     file descriptor of connection
 * @return            0: success; >0: err, including SOCKET_BASE_ERR SOCKET_TIMEOUT_ERR
 */
int clt_socket_getcoon(void *sockhandle, char *ip, int port, int *connfd);

/**
 * close fd of connection
 * @param  connfd file descriptor of connection
 * @return        0:success; >0: err, including SOCKET_PAR_ERR
 */
int clt_socket_closeconn(int *connfd);

/**
 * send buf
 * @param  sockhandle sock handle structure
 * @param  connfd     file descriptor of connection
 * @param  buf        buffer to send
 * @param  buflen     length of send buffer
 * @return            0: success; >0: err,including SOCKET_MALLOC_ERR SOCKET_TIMEOUT_ERR SOCKET_BASE_ERR SOCKET_UNCOON_ERR
 */
int clt_socket_send(void *sockhandle, int connfd, char *buf, int buflen);

/**
 * receive buf (buflen long)
 * @param  sockhandle sock handle structure
 * @param  connfd     file descriptor of connection
 * @param  buf        buffer to receive
 * @param  buflen     the actual length of receiving buffer
 * @return            0: success; >0: err,including SOCKET_UNCOON_ERR SOCKET_TIMEOUT_ERR SOCKET_BASE_ERR
 */
int clt_socket_rev(void *sockhandle, int connfd, char *buf, int *buflen);
/**
 * release sockhandle
 * @param  sockhandle sock handle structure
 * @return            0: success; >0: err,including SOCKET_PAR_ERR
 */
int clt_socket_destory(void **sockhandle);

/**
 * socket of server initialization
 * @param  listenfd  file descriptor of listenning connection
 * @param  listennum number of listenning connection
 * @param  port      listenning port of server
 * @return           0: success; >0: err, including SOCKET_PAR_ERR SOCKET_MALLOC_ERR
 */
int srv_socket_init(int *listenfd, int listennum, int port);

/**
 * server accpet connection from client
 * @param  listenfd file descriptor of listenning connection
 * @param  connfd   file descriptor of connection
 * @param  contime  maximum wating time of connecting to client (second)
 * @return          0: success; >0: err, including SOCKET_BASE_ERR SOCKET_TIMEOUT_ERR
 */
int srv_socket_accept(int listenfd, int *connfd, int contime);

/**
 * server send buf
 * @param  connfd   file descriptor of connection
 * @param  buf      buffer to send
 * @param  buflen   length of send buffer
 * @param  sendtime maximum waiting time of sending buffer (second)
 * @return          0: success; >0: err,including SOCKET_MALLOC_ERR SOCKET_TIMEOUT_ERR SOCKET_BASE_ERR SOCKET_UNCOON_ERR
 */
int srv_socket_send(int connfd, char *buf, int buflen, int sendtime);

/**
 * server receive buf
 * @param  connfd  file descriptor of connection
 * @param  buf     buffer to receive
 * @param  buflen  the actual length of receiving buffer
 * @param  revtime maximum waiting time of receiving buffer (second)
 * @return         0: success; >0: err,including SOCKET_UNCOON_ERR SOCKET_TIMEOUT_ERR SOCKET_BASE_ERR
 */
int srv_socket_rev(int connfd, char *buf, int *buflen, int revtime);

/**
 * close connection fd of server
 * @param  connfd file descriptor of connection
 * @return        0: success; >0, err, including SOCKET_PAR_ERR
 */
int srv_socket_close(int *connfd);

/**
 * close listening fd of server
 * @param  listenfd file descriptor of listening
 * @return          0: success; >0, err, including SOCKET_PAR_ERR
 */
int srv_socket_destory(int *listenfd);

#endif
