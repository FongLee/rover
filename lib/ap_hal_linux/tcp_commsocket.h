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
 * socket of client initialization
 * @param  sockhandle [description]
 * @param  contime    [description]
 * @param  sendtime   [description]
 * @param  revtime    [description]
 * @param  connum     [description]
 * @return            0: success; err: SOCKET_PAR_ERR SOCKET_MALLOC_ERR 
 */
int clt_socket_init(void **sockhandle, int contime, int sendtime, int revtime);

/**
 * build connection  
 * @param  sockhandle [description]
 * @param  ip         [description]
 * @param  port       [description]
 * @param  connfd     [description]
 * @return            0: success; err: SOCKET_BASE_ERR SOCKET_TIMEOUT_ERR
 */
int clt_socket_getcoon(void *sockhandle, char *ip, int port, int *connfd);

/**
 * close fd
 * @param  connfd [description]
 * @return        [description]
 */
int clt_socket_closeconn(int *connfd);

/**
 * send buf
 * @param  sockhandle [description]
 * @param  connfd     [description]
 * @param  buf        [description]
 * @param  buflen     [description]
 * @return            0: success; err: SOCKET_MALLOC_ERR SOCKET_TIMEOUT_ERR SOCKET_BASE_ERR SOCKET_UNCOON_ERR
 */
int clt_socket_send(void *sockhandle, int connfd, char *buf, int buflen);

/**
 * receive buf (buflen long) 
 * @param  sockhandle [description]
 * @param  connfd     [description]
 * @param  buf        [description]
 * @param  buflen     the actual length of getting buf
 * @return            0: success; err: SOCKET_UNCOON_ERR SOCKET_TIMEOUT_ERR SOCKET_BASE_ERR
 */
int clt_socket_rev(void *sockhandle, int connfd, char *buf, int *buflen);
/**
 * release sockhandle
 * @param  sockhandle [description]
 * @return            0: success;
 */
int clt_socket_destory(void **sockhandle);

/**
 * socket of server initialization
 * @param  listenfd  [description]
 * @param  listennum [description]
 * @param  port      [description]
 * @return           0: success; err: SOCKET_PAR_ERR SOCKET_MALLOC_ERR
 */
int srv_socket_init(int *listenfd, int listennum, int port);

/**
 * server accpet connection from client
 * @param  listenfd [description]
 * @param  connfd   [description]
 * @param  contime  [description]
 * @return          0: success; err: SOCKET_BASE_ERR SOCKET_TIMEOUT_ERR
 */
int srv_socket_accept(int listenfd, int *connfd, int contime);

/**
 * server send buf
 * @param  connfd   [description]
 * @param  buf      [description]
 * @param  buflen   [description]
 * @param  sendtime [description]
 * @return          0: success; err: SOCKET_MALLOC_ERR SOCKET_TIMEOUT_ERR SOCKET_BASE_ERR SOCKET_UNCOON_ERR
 */
int srv_socket_send(int connfd, char *buf, int buflen, int sendtime);

/**
 * server receive buf
 * @param  connfd  [description]
 * @param  buf     [description]
 * @param  buflen  [description]
 * @param  revtime [description]
 * @return         0: success; err: SOCKET_UNCOON_ERR SOCKET_TIMEOUT_ERR SOCKET_BASE_ERR
 */
int srv_socket_rev(int connfd, char *buf, int *buflen, int revtime);

/**
 * close connection fd of server
 * @param  connfd [description]
 * @return        0: success; err: SOCKET_PAR_ERR
 */
int srv_socket_close(int *connfd);

/**
 * close listening fd of server 
 * @param  listenfd [description]
 * @return          0: success; err: SOCKET_PAR_ERR
 */
int srv_socket_destory(int *listenfd);

#endif