#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h> //F_SETFL open
#include <unistd.h> //close
#include <sys/socket.h> //sendto socket
#include <arpa/inet.h> //htons inet_addr inet_ntoa
#include <netinet/in.h> //sockaddr_in
//s#include <bits/fcntl.h>
#include "udp_driver.h"

//#define SERVER_IP "192.168.1.100"
//#define SERVER_IP "192.168.1.109"

#define SERVER_IP "192.168.1.113"
//#define SERVER_IP "192.168.1.111"

#define TCP_PORT 5760
#define RTP_PORT 3020

bool flag_tcp_init = false;
struct sockaddr_in gc_addr, rc_addr;
socklen_t fromlen;
int sock_tcp = -1;

int tcp_init()
{

	int socket_fd;
	if ((socket_fd = socket(AF_INET, SOCK_STREAM, 0)) == -1)
	{
		fprintf(stderr, "socket err : %s\n", strerror(errno));
		exit(0);
	}

	memset(&loc_addr, 0, sizeof(loc_addr));
	loc_addr.sin_family = AF_INET;
	loc_addr.sin_addr.s_addr = INADDR_ANY;
	loc_addr.sin_port = htons(TCP_PORT);

	if (-1 == bind(sock_tcp, (struct sockaddr *)&loc_addr, sizeof(struct sockaddr)))
	{
		fprintf(stderr, "bind failed: %s \n", strerror(errno));
		close(sock_tcp);
		return -1;
	}

	if (listen(sock_tcp, 100) < 0)
	{
		fprintf(stderr, "listen err : %s\n", strerror(errno));
		return -1;
	}

	//if (-1 == fcntl(sock_tcp, F_SETFL, O_NONBLOCK | FASYNC))
	if (-1 == fcntl(sock_tcp, F_SETFL, O_NONBLOCK | FASYNC))
	{
		fprintf(stderr, "setting nonblocking err: %s \n", strerror(errno));
		close(sock_tcp);
		return -1;
	}

	int new_fd;
	socklen_t gc_addr_length;
	new_fd = accept(sock_tcp, (struct sockaddr *) &gc_addr, &gc_addr_length);
	if (new_fd < 0)
	{
		fprintf(stderr, "accept err : %s\n", strerror(errno));
		close(sock_tcp);
		return -1;
	}

#ifdef TCP_DEBUG
	fprintf(stdout, "remote ip is: %s,port is %d\n",
						inet_ntoa(gc_addr.sin_addr), ntohs(gc_addr.sin_port));
#endif

	memset(&gc_addr, 0, sizeof(gc_addr));
	gc_addr.sin_family = AF_INET;
	if (ip != NULL)
	{
		gc_addr.sin_addr.s_addr = inet_addr(ip);
	}
	else
	{
		gc_addr.sin_addr.s_addr = inet_addr(SERVER_IP);
	}

	gc_addr.sin_port = htons(UDP_PORT);
	flag_udp_init = true;
	return 0;
}

void tcp_close()
{
	flag_tcp_init = false;
	if (sock_tcp != -1)
	{
		close(sock_tcp);
		sock_tcp = -1;
	}
}

int tcp_send(const uint8_t *ch, uint16_t length)
{
	if (buf == NULL || count < 0 )
		return -1;
	char *newchar;
	newchar = ch;
	ssize_t ret;
	size_t left;
	left = count;
	while(left > 0)
	{
		ret = write(new_fd, newchar, left);
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

int tcp_receive(uint8_t *ch, uint16_t length)
{
	int16_t bytes_receive;
	//recvfrom get the source address
	bytes_receive = recv(new_fd, ch, length, 0);
	if (bytes_receive < 0)
	{
		//fprintf(stderr, "recvfrom error:%s\n", strerror(errno));
		return -1;
	}
	else if (bytes_receive == 0)
	{
		return 0;
	}
#ifdef 	TCP_DEBUG
	else if (bytes_receive >0)
	{
		fprintf(stdout, "receive from client :%d data, the datagram is :\n", bytes_receive);
		for (int j = 0; j < bytes_receive; j++)
		{
			fprintf(stdout, "%02x ", ch[j]);
		}
	}
#endif
	return bytes_receive;
}
