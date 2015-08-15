#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include "commsocket.h"

typedef struct
{
	int contime;
	int sendtime;
	int revtime;
} handle_t ;


/**
 * set file descriptor nonblock
 * @param  fd file descriptor
 * @return    0: normal ; -1: err
 */
static int active_nonblock(int fd)
{
	int res;
	int flags = fcntl(fd, F_GETFL);
	if (flags < 0)
		return -1;
	flags = flags | O_NONBLOCK;
	res = fcntl(fd, F_SETFL, flags);
	if (res < 0)
		return -1;
	return 0;
}

/**
 * cancel file descriptor nonblock
 * @param  fd file descriptor
 * @return    0: normal ; -1: err
 */
static int deactive_nonblock(int fd)
{
	int res;
	int flags = fcntl(fd, F_GETFL);
	if (flags < 0)
		return -1;
	flags = flags & ~O_NONBLOCK;
	res = fcntl(fd, F_SETFL, flags);
	if (res < 0)
		return -1;
	return 0;
}

/**
 * connect in a period of time
 * @param  fd      file descriptor of connection to server
 * @param  addr    address of server (struct sockaddr)
 * @param  seconds maximum  of connect time (second)
 *                 0 means : connect is in block mode
 * @return         0: normal ; -1: err ETIMEDOUT or else
 */
static int connect_timeout(int fd, struct sockaddr *addr,  int seconds)
{
	if (addr == NULL || seconds < 0)
		return -1;

	int res = 0;
	socklen_t len = sizeof(struct sockaddr);
	// len = sizeof(addr);
	if (seconds > 0)
		active_nonblock(fd);

	res = connect(fd, addr, len);
	if (res < 0 && errno == EINPROGRESS)
	{

		struct timeval mytime;
		mytime.tv_sec = seconds;
		mytime.tv_usec = 0;
		fd_set wset;
		FD_ZERO(&wset);
		FD_SET(fd, &wset);
		fd_set rset;
		FD_ZERO(&rset);
		FD_SET(fd, &rset);
		do
		{
			res = select(fd + 1, &rset, &wset, NULL, &mytime);
		} while (res < 0 && errno == EINTR);

		if (res < 0)
		{
			res = -1;
		}

		else if (res ==0)
		{
			errno = ETIMEDOUT;
			res = -1;
		}
		else if (res > 0)
		{
			int err;
			socklen_t err_len = sizeof(err);

			if (FD_ISSET(fd, &wset))
			{
				res = getsockopt(fd, SOL_SOCKET, SO_ERROR, &err, &err_len);
				if (res < 0)
				{
					res = -1;
				}

				else if (err > 0)
				{
					errno = err;
					res = -1;
				}
			}
		}

	}

	if (seconds > 0)
		deactive_nonblock(fd);

	return res;
}

/**
 * accept in a period of time
 * @param  fd      file descriptor of listenning connection
 * @param  addr    address from client (struct sockadd)
 * @param  seconds maximum wating time of connecting to client (second), 0 means : accpet in block mode
 * @return         0: normal, ; -1: err, including ETIMEDOUT or else
 */
static int accept_timeout(int fd, struct sockaddr *addr,  int seconds)
{
	if (seconds < 0 || fd < 0)
		return -1;

	int res = 0;
	socklen_t len = sizeof(struct sockaddr);

	if (seconds > 0)
	{
		struct timeval mytime;
		mytime.tv_sec = seconds;
		mytime.tv_usec = 0;
		fd_set rset;
		FD_ZERO(&rset);
		FD_SET(fd, &rset);
		do
		{
			res = select(fd + 1, &rset, NULL, NULL, &mytime);
		} while (res < 0 && errno == EINTR);

		if (res < 0)
		{
			return -1;
		}

		else if (res ==0)
		{
			errno = ETIMEDOUT;

			return -1;
		}

	}

	if (addr != NULL)
		res = accept(fd, addr, &len);
	else
		res = accept(fd, NULL, NULL);

	return res;
}

/**
 * write buf into fd, untial all the buf is sended
 * @param  fd    file descriptor to write
 * @param  buf   buffer to write
 * @param  count length of buffer
 * @return       success: equal to count; err: less than count
 */
ssize_t writen(int fd, void *buf, size_t count)
{
	if (buf == NULL || count < 0 )
		return -1;
	char *newchar;
	newchar = (char *)buf;
	ssize_t ret;
	size_t left;
	left = count;
	while(left>0)
	{
		ret = write(fd, newchar, left);
		if (ret < 0)
		{
			if (errno == EINTR)
				continue;
			else return -1;
		}

		left = left - ret;
		newchar = newchar + ret;

	}
	return count;

}

/**
 * read buf from fd, until length of buf is count
 * @param  fd    file descriptor to read
 * @param  buf   buffer from reading
 * @param  count count of buffer
 * @return       read length : 0-count
 */
ssize_t readn(int fd, void * buf, size_t count)
{
	char *newchar = (char *)buf;
	size_t left;
	ssize_t res;

	left = count;
	while(left > 0)
	{
		res = read(fd, newchar, left);
		if ( res == 0 )
			break;
		if (res < 0)
		{
			if (errno == EINTR)
				continue;
			else
				return -1;
		}
		left = left - res;
		newchar = newchar + res;
	}
	return count - left;

}

/**
 * judge whether we can write in a period of time
 * @param  fd      file descriptor to write
 * @param  seconds maximum waiting time to write (second),
 *                 0 means : write  of next instructions will in block mode
 * @return         -1: errno = ETIMEDOUT or else; 0: success,can write now
 */
static int write_timeout(int fd, int seconds)
{
	int res = 0;

	if (seconds < 0)
		res = -1;

	if (seconds > 0)
	{
		fd_set myset;
		struct timeval mytime;
		FD_ZERO(&myset);
		FD_SET(fd, &myset);

		mytime.tv_sec = seconds;
		mytime.tv_usec = 0;
		do
		{
			res = select (fd + 1, NULL , &myset, NULL, &mytime);
		} while(res < 0 && (errno == EINTR));
		if (res < 0)
		{
			res = -1;
		}
		else if (res == 0)
		{
			errno = ETIMEDOUT;
			res = -1;
		}
		else
		{
			if (FD_ISSET(fd, &myset))
			{
				res = 0;
			}
			else
				res = -1;
		}
	}

	return res;

}

/**
 * judge whether we can write in a period of time
 * @param  fd      file descriptor
 * @param  seconds maximum waiting time to read (second)
 * @return         -1: errno = ETIMEDOUT or else; 0: success,can read now
 */
static int read_timeout(int fd, int seconds)
{
	int res = 0;

	if (seconds < 0)
		res = -1;

	if (seconds > 0)
	{
		struct timeval mytime;
		fd_set myset;
		FD_ZERO(&myset);
		FD_SET(fd, &myset);
		mytime.tv_sec = seconds;
		mytime.tv_usec = 0;
		do
		{
			res = select (fd + 1, &myset, NULL, NULL, &mytime);
		} while(res < 0 && errno == EINTR);

		if (res < 0)
		{
			res = -1;
		}
		else if (res == 0)
		{
			errno = ETIMEDOUT;
			res = -1;
		}
		else
		{
			//when client send FIN , fd changes
	 		if (FD_ISSET(fd, &myset))
			{
				res = 0;
			}
			else
				res = -1;
		}
	}

	return res;

}

/**
 * socket of client initialization
 * @param  sockhandle sock handle structure
 * @param  contime    maximum  of connect time (second)
 * @param  sendtime   maximum  of send time (second)
 * @param  revtime    maximum of receive time (second)
 * @param  connum     maximum number of connect
 * @return            0: success; >0: err, including SOCKET_PAR_ERR SOCKET_MALLOC_ERR
 */
int clt_socket_init(void **sockhandle, int contime, int sendtime, int revtime)
{
	int res;
	if (sockhandle == NULL || contime < 0 || sendtime < 0 || revtime < 0)
	{
		res = SOCKET_PAR_ERR;
		return res;
	}

	handle_t * tmp;
	tmp = (handle_t *) malloc(sizeof(handle_t));
	if ( tmp == NULL)
	{
		fprintf(stdout, "func : malloc err.\n");
		res = SOCKET_MALLOC_ERR;
		return res;
	}
	tmp->contime = contime;
	tmp->sendtime = sendtime;
	tmp->revtime = revtime;

	*sockhandle = tmp;

	res = SOCKET_OK;
	return res;

}

/**
 * build connection
 * @param  sockhandle sock handle structure
 * @param  ip         ip of server
 * @param  port       port of server
 * @param  connfd     file descriptor of connection
 * @return            0: success; >0: err, including SOCKET_BASE_ERR SOCKET_TIMEOUT_ERR
 */
int clt_socket_getcoon(void *sockhandle, char *ip, int port, int *connfd)
{
	int res;

	handle_t * tmp;
	tmp = (handle_t *) sockhandle;

	if ((res = socket(AF_INET, SOCK_STREAM, 0)) < -1)
	{
	//perror("socket err:");
		fprintf(stdout, "func : socket err :%s\n", strerror(errno));

		res = SOCKET_BASE_ERR;
		return res;
	}
	*connfd = res;

	struct sockaddr_in ser_addr;
	ser_addr.sin_family = AF_INET;
	ser_addr.sin_port = htons(port);
	ser_addr.sin_addr.s_addr = inet_addr(ip);
	if (connect_timeout(*connfd, (struct sockaddr *)&ser_addr, tmp->contime) < 0)
	{
		if (errno == ETIMEDOUT)
		{
			fprintf(stdout, "connect time out\n");
			res = SOCKET_TIMEOUT_ERR;
			close(*connfd);
			return res;
		}
		fprintf(stdout, "func : connect_timeout err :%s\n", strerror(errno));
		res = SOCKET_BASE_ERR;
		close(*connfd);
		return res;
	}

	res = SOCKET_OK;
	return res;

}

/**
 * close fd of connection
 * @param  connfd file descriptor of connection
 * @return        0:success; >0: err, including SOCKET_PAR_ERR
 */
int clt_socket_closeconn(int *connfd)
{
	int res;
	if (*connfd < 0)
	{
		res = SOCKET_PAR_ERR;
		return res;
	}

	close(*connfd);
 	*connfd = -1;
	res = SOCKET_OK;
	return res;
}



/**
 * send buf
 * @param  sockhandle sock handle structure
 * @param  connfd     file descriptor of connection
 * @param  buf        buffer to send
 * @param  buflen     length of send buffer
 * @return            0: success; >0: err,including SOCKET_MALLOC_ERR SOCKET_TIMEOUT_ERR SOCKET_BASE_ERR SOCKET_UNCOON_ERR
 */
int clt_socket_send(void *sockhandle, int connfd, char *buf, int buflen)
{
	int res;

	handle_t * tmp;
	tmp = (handle_t *)sockhandle;

	char *newdata;
	newdata = (char *)malloc(sizeof(char) * (buflen + 4));
	if ( newdata == NULL)
	{
		fprintf(stdout, "func : malloc err: %s\n", strerror(errno));
		res = SOCKET_MALLOC_ERR;
		return res;
	}

	int nlen = htonl(buflen);
	memcpy(newdata, &nlen, 4);
	memcpy(newdata + 4, buf, buflen);

	res = write_timeout(connfd, tmp->sendtime);
	if (res < 0)
	{
		if (errno == ETIMEDOUT)
		{
			//fprintf(stdout, "write time out\n");
			free(newdata);
			res = SOCKET_TIMEOUT_ERR;
			return res;
		}
		else
		{
			//fprintf(stdout, "select error\n");
			fprintf(stdout, "write_timeout err: %s\n", strerror(errno));
			free(newdata);
			res = SOCKET_BASE_ERR;
			return res;
		}

	}
	//can write now
	else if (res == 0)
	{
		int wrlen;
		wrlen = writen(connfd, newdata, buflen + 4);
		if (wrlen < (buflen + 4))
		{
			free(newdata);
			res = SOCKET_UNCOON_ERR;
			return res;
		}

	}
	free(newdata);
	res = SOCKET_OK;
	return res;

}

/**
 * receive buf (buflen long)
 * @param  sockhandle sock handle structure
 * @param  connfd     file descriptor of connection
 * @param  buf        buffer to receive
 * @param  buflen     the actual length of receiving buffer
 * @return            0: success; >0: err,including SOCKET_UNCOON_ERR SOCKET_TIMEOUT_ERR SOCKET_BASE_ERR
 */
int clt_socket_rev(void *sockhandle, int connfd, char *buf, int *buflen)
{
	int res;

	handle_t * tmp;
	tmp = (handle_t *)sockhandle;
	int rdlen = 0;

	res = read_timeout(connfd, tmp->revtime);
	if (res < 0)
	{
		//if get the errno equals to ETIMEDOUT
		if (errno == ETIMEDOUT)
		{
			//fprintf(stdout, "read time out\n");
			res = SOCKET_TIMEOUT_ERR;
			return res;
		}
		else
		{
			fprintf(stdout, "read_timeout err: %s\n", strerror(errno));
		}
		//continue;

	}
	else if (res == 0)
	{
		res = readn(connfd, &rdlen, 4);
		if (res < 0)
		{
			fprintf(stderr, "readn err: %s\n", strerror(errno));
			res = SOCKET_BASE_ERR;
			return res;

		}
		else if (res < 4)
		{
			//tcp is closed
			*buflen = 0;
			res = SOCKET_UNCOON_ERR;
			return res;
		}

		rdlen = ntohl(rdlen);
	}



	res = read_timeout(connfd, tmp->sendtime);
	if (res < 0)
	{
		//if get the errno equals to ETIMEDOUT
		if (errno == ETIMEDOUT)
		{
			//fprintf(stdout, "read time out\n");
			res = SOCKET_TIMEOUT_ERR;
			return res;
		}
		else
		{
			fprintf(stdout, "read_timeout err: %s\n", strerror(errno));
		}
		//continue;

	}
	//can read now
	else if (res == 0)
	{
		res = readn(connfd, buf, rdlen);
		//res = read(new_fd, rec_buf, sizeof(rec_buf));
		if (res < 0)
		{
			fprintf(stderr, "readn err: %s\n", strerror(errno));
			res = SOCKET_BASE_ERR;
			 return res;
		}
		//tcp is closed
		else if (res < rdlen)
		{
			//fprintf(stdout, "client is closed\n");
			*buflen = rdlen;
			res = SOCKET_UNCOON_ERR;
			return res;
		}
		//read success
		else if (res == rdlen)
		{
			//res = SOCKET_OK;
			*buflen = rdlen;
			//return res;
		}

	}

	res = SOCKET_OK;
	return res;
}

/**
 * release sockhandle
 * @param  sockhandle sock handle structure
 * @return            0: success; >0: err,including SOCKET_PAR_ERR
 */
int clt_socket_destory(void **sockhandle)
{
	int res;
	handle_t *tmp;
	tmp = (handle_t *)*sockhandle;

	if (tmp == NULL)
	{
		res = SOCKET_PAR_ERR;
		return res;
	}

	free(tmp);
	tmp = NULL;
	res = SOCKET_OK;
	return res;


}


////////////////////////////////////////////////////////////////

/**
 * socket of server initialization
 * @param  listenfd  file descriptor of listenning connection
 * @param  listennum number of listenning connection
 * @param  port      listenning port of server
 * @return           0: success; >0: err, including SOCKET_PAR_ERR SOCKET_MALLOC_ERR
 */
int srv_socket_init(int *listenfd, int listennum, int port)
{
	int res;
	if (listenfd == NULL || listennum < 0 || port < 0)
	{
		res = SOCKET_PAR_ERR;
		return res;
	}

	int socket_fd;
	if ((socket_fd = socket(AF_INET, SOCK_STREAM, 0)) == -1)
	{
		fprintf(stderr, "socket err : %s\n", strerror(errno));

	}
	*listenfd = socket_fd;

	int optval = 1;
	if (setsockopt(socket_fd, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval)) < 0)
	{
		fprintf(stderr, "setsocketopt err : %s\n", strerror(errno));
		res = SOCKET_BASE_ERR;
		return res;
	}

	struct sockaddr_in ser_addr;
	ser_addr.sin_family = AF_INET;
	ser_addr.sin_port = htons(port);
	ser_addr.sin_addr.s_addr = INADDR_ANY;

	if (bind(socket_fd, (struct sockaddr *)&ser_addr, sizeof(ser_addr)) < 0)
	{
		fprintf(stderr, "bind err : %s\n", strerror(errno));
		res = SOCKET_BASE_ERR;
		return res;
	}

	if (listen(socket_fd, listennum) < 0)
	{
		fprintf(stderr, "listen err : %s\n", strerror(errno));
		res = SOCKET_BASE_ERR;
		return res;
	}

	res = SOCKET_OK;
	return res;
}

/**
 * server accpet connection from client
 * @param  listenfd file descriptor of listenning connection
 * @param  connfd   file descriptor of connection
 * @param  contime  maximum wating time of connecting to client (second)
 * @return          0: success; >0: err, including SOCKET_BASE_ERR SOCKET_TIMEOUT_ERR
 */
int srv_socket_accept(int listenfd, int *connfd, int contime)
{
	int res;
	struct sockaddr_in client_addr;
	//socklen_t addrlen;
	//*connfd = accept(listenfd, (struct sockaddr *)&client_addr, &addrlen);
	//int accept_timeout(int fd, struct sockaddr *addr, unsigned int seconds)
	*connfd = accept_timeout(listenfd, (struct sockaddr *)&client_addr, contime);
	if (*connfd < 0)
	{
		if (errno == ETIMEDOUT)
		{
			fprintf(stderr, "accept time out\n");
			res = SOCKET_TIMEOUT_ERR;
			return res;
		}
		fprintf(stderr, "accept err : %s\n", strerror(errno));
		res = SOCKET_BASE_ERR;
		return res;
	}

	res = SOCKET_OK;
	return res;
}

/**
 * server send buf
 * @param  connfd   file descriptor of connection
 * @param  buf      buffer to send
 * @param  buflen   length of send buffer
 * @param  sendtime maximum waiting time of sending buffer (second)
 * @return          0: success; >0: err,including SOCKET_MALLOC_ERR SOCKET_TIMEOUT_ERR SOCKET_BASE_ERR SOCKET_UNCOON_ERR
 */
int srv_socket_send(int connfd, char *buf, int buflen, int sendtime)
{
	int res;

	char *newdata;
	newdata = (char *)malloc(sizeof(char) * (buflen + 4));
	if ( newdata == NULL)
	{
		fprintf(stdout, "func : malloc err: %s\n", strerror(errno));
		res = SOCKET_MALLOC_ERR;
		return res;
	}

	int nlen = htonl(buflen);
	memcpy(newdata, &nlen, 4);
	memcpy(newdata + 4, buf, buflen);

	res = write_timeout(connfd, sendtime);
	if (res < 0)
	{
		if (errno == ETIMEDOUT)
		{
			//fprintf(stdout, "write time out\n");
			free(newdata);
			res = SOCKET_TIMEOUT_ERR;
			return res;
		}
		else
		{
			//fprintf(stdout, "select error\n");
			fprintf(stdout, "write_timeout err: %s\n", strerror(errno));
			free(newdata);
			res = SOCKET_BASE_ERR;
			return res;
		}

	}
	//can write now
	else if (res == 0)
	{
		int wrlen;
		wrlen = writen(connfd, newdata, buflen + 4);
		if (wrlen < (buflen + 4))
		{
			free(newdata);
			res = SOCKET_UNCOON_ERR;
			return res;
		}

	}

	free(newdata);
	res = SOCKET_OK;
	return res;
}

/**
 * server receive buf
 * @param  connfd  file descriptor of connection
 * @param  buf     buffer to receive
 * @param  buflen  the actual length of receiving buffer
 * @param  revtime maximum waiting time of receiving buffer (second)
 * @return         0: success; >0: err,including SOCKET_UNCOON_ERR SOCKET_TIMEOUT_ERR SOCKET_BASE_ERR
 */
int srv_socket_rev(int connfd, char *buf, int *buflen, int revtime)
{
 	int res;

	int rdlen = 0;

	res = read_timeout(connfd, revtime);
	if (res < 0)
	{
		//if get the errno equals to ETIMEDOUT
		if (errno == ETIMEDOUT)
		{
			//fprintf(stdout, "read time out\n");
			res = SOCKET_TIMEOUT_ERR;
			return res;
		}
		else
		{
			fprintf(stdout, "read_timeout err: %s\n", strerror(errno));
		}
		//continue;

	}
	else if (res == 0)
	{
		res = readn(connfd, &rdlen, 4);
		if (res < 0)
		{
			fprintf(stderr, "readn err: %s\n", strerror(errno));
			res = SOCKET_BASE_ERR;
			return res;

		}
		else if (res < 4)
		{
			//tcp is closed
			*buflen = 0;
			res = SOCKET_UNCOON_ERR;
			return res;
		}

		rdlen = ntohl(rdlen);
	}

	res = read_timeout(connfd, revtime);
	if (res < 0)
	{
		//if get the errno equals to ETIMEDOUT
		if (errno == ETIMEDOUT)
		{
			//fprintf(stdout, "read time out\n");
			res = SOCKET_TIMEOUT_ERR;
			return res;
		}
		else
		{
			fprintf(stdout, "read_timeout err: %s\n", strerror(errno));
		}
		//continue;

	}
	//can read now
	else if (res == 0)
	{
		res = readn(connfd, buf, rdlen);
		//res = read(new_fd, rec_buf, sizeof(rec_buf));
		if (res < 0)
		{
			fprintf(stderr, "readn err: %s\n", strerror(errno));
			res = SOCKET_BASE_ERR;
			return res;
		}
		//tcp is closed
		else if (res < rdlen)
		{
			//fprintf(stdout, "client is closed\n");
			*buflen = rdlen;
			res = SOCKET_UNCOON_ERR;
			return res;
		}
		//read success
		else if (res == rdlen)
		{
			//res = SOCKET_OK;
			*buflen = rdlen;
			//return res;
		}

	}

	res = SOCKET_OK;
	return res;
}

/**
 * close connection fd of server
 * @param  connfd file descriptor of connection
 * @return        0: success; >0, err, including SOCKET_PAR_ERR
 */
int srv_socket_close(int *connfd)
{
	int res;
	if (*connfd < 0)
	{
		res = SOCKET_PAR_ERR;
		return res;
	}

	close(*connfd);
 	*connfd = -1;
	res = SOCKET_OK;
	return res;
}

/**
 * close listening fd of server
 * @param  listenfd file descriptor of listening
 * @return          0: success; >0, err, including SOCKET_PAR_ERR
 */
int srv_socket_destory(int *listenfd)
{
	int res;
	if (*listenfd < 0)
	{
		res = SOCKET_PAR_ERR;
		return res;
	}

	close(*listenfd);
 	*listenfd = -1;
	res = SOCKET_OK;
	return res;
}
