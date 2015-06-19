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

/**
 * write buf into fd, untial all the buf is sended
 * @param  fd    [description]
 * @param  buf   [description]
 * @param  count [description]
 * @return       success: euqal to count; err: less than count
 */
ssize_t writen(int fd, void *buf, size_t count)
{
	if (buf == NULL || count < 0 )
		return -1;
	char *newchar;
	newchar = buf;
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
 * @param  fd    [description]
 * @param  buf   [description]
 * @param  count [description]
 * @return       read length 0-count
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
 * @param  fd      [description]
 * @param  seconds [description]
 * @return         -1: errno = ETIMEDOUT or else; 0: success,can write now
 */
int write_timeout(int fd, int seconds, int microsec)
{
	fd_set myset;
	struct timeval mytime;
	int res;

	FD_ZERO(&myset);
	FD_SET(fd, &myset);

	mytime.tv_sec = seconds;
	mytime.tv_usec = microsec;
	do
	{
		res = select (fd + 1, NULL , &myset, NULL, &mytime);
	} while(res < 0 && (errno == EINTR));
	if (res < 0)
	{
		return  -1;
	}
	else if (res == 0)
	{
		errno = ETIMEDOUT;
		return -1;
	}
	else
	{
		if (FD_ISSET(fd, &myset))
		{
			return 0;
		}
		return -1;
	}

}

/**
 * judge whether we can write in a period of time
 * @param  fd      [description]
 * @param  seconds [description]
 * @return         -1: errno = ETIMEDOUT or else; 0: success,can read now
 */
int read_timeout(int fd, int seconds, int microsec)
{
	fd_set myset;
	struct timeval mytime;
	int res;

	FD_ZERO(&myset);
	FD_SET(fd, &myset);

	mytime.tv_sec = seconds;
	mytime.tv_usec = microsec;
	do
	{
		res = select (fd + 1, &myset, NULL, NULL, &mytime);
	} while(res < 0 && errno == EINTR);

	if (res < 0)
	{
		return  -1;
	}
	else if (res == 0)
	{
		errno = ETIMEDOUT;
		return -1;
	}
	else
	{
		//when client send FIN , fd changes
 		if (FD_ISSET(fd, &myset))
		{
			return 0;
		}
		return -1;
	}

}
