#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h> //F_SETFL open
#include <unistd.h> //close
#include <sys/socket.h> //sendto socket
#include <arpa/inet.h> //htons inet_addr inet_ntoa
#include <netinet/in.h> //sockaddr_in
#include <signal.h>
#include <sys/types.h>

//s#include <bits/fcntl.h>
#include "tcp_driver.h"

//#define SERVER_IP "192.168.1.100"
//#define SERVER_IP "192.168.1.109"

#define SERVER_IP "192.168.1.113"
//#define SERVER_IP "192.168.1.111"

#define TCP_PORT 5760
//#define RTP_PORT 3020

bool flag_tcp_init = false;
bool flag_tcp_connect = false;
int sock_tcp = -1;
int new_sock_fd = -1;
int sock_accept_tcp = -1;

int tcp_init(void (*myhandler)(int num))
{

	//int socket_fd;
	if ((sock_tcp = socket(AF_INET, SOCK_STREAM, 0)) == -1)
	{
		fprintf(stderr, "socket err : %s\n", strerror(errno));
		return -1;
	}
	int optval = 1;
	if (setsockopt(sock_tcp, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval)) < 0)
	{
		fprintf(stderr, "setsockopt : %s\n", strerror(errno));
		return -1;
	}

	int res;
	struct sigaction myact;
	myact.sa_handler = myhandler;
	res = sigaction(SIGUSR1, &myact, NULL);
	if (res < 0)
		return -1;
	
	struct sockaddr_in loc_addr;
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

	if (listen(sock_tcp, 10) < 0)
	{
		fprintf(stderr, "listen err : %s\n", strerror(errno));
		return -1;
	}

	//if (-1 == fcntl(sock_tcp, F_SETFL, O_NONBLOCK | FASYNC))
	if (-1 == fcntl(sock_tcp, F_SETFL, O_NONBLOCK))
	{
		fprintf(stderr, "setting nonblocking err: %s \n", strerror(errno));
		close(sock_tcp);
		return -1;
	}
	flag_tcp_init = true;
	return 0;
}

int tcp_accept()
{

	socklen_t client_addr_length;
	struct sockaddr_in client_addr;
	new_sock_fd = accept(sock_tcp, (struct sockaddr *) &client_addr, &client_addr_length);
	if (new_sock_fd < 0)
	{
		//fprintf(stderr, "accept err : %s\n", strerror(errno));
		//close(sock_tcp);
		return -1;
	}
	sock_accept_tcp = new_sock_fd;
	
	if (-1 == fcntl(sock_accept_tcp, F_SETFL, O_NONBLOCK))
	{
		//fprintf(stderr, "setting nonblocking err: %s \n", strerror(errno));
		close(sock_tcp);
		return -1;
	}

	flag_tcp_connect = true;

#ifdef TCP_DEBUG
	fprintf(stdout, "remote ip is: %s,port is %d\n",
						inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));
#endif
	return 0;
}

void tcp_destroy()
{
	flag_tcp_init = false;
	if (sock_tcp != -1)
	{
		close(sock_tcp);
		sock_tcp = -1;
	}
}

void tcp_close()
{
	flag_tcp_connect = false;
	if (sock_accept_tcp != -1)
	{
		close(sock_accept_tcp);
		sock_accept_tcp = -1;
	}
}

int tcp_send(const uint8_t *ch, uint16_t length)
{
	if (ch == NULL || length < 0 )
		return -1;
	uint8_t *newchar;
	//uint16_t count;
	newchar = ch;
	ssize_t ret;
	size_t left;
	left = length;
	while(left > 0)
	{
		ret = write(sock_accept_tcp, newchar, left);
		if (ret < 0)
		{
			if (errno == EINTR)
				continue;
			else return -1;
		}

		left = left - ret;
		newchar = newchar + ret;

	}
	return length;
}

int tcp_receive(uint8_t *ch, uint16_t length)
{
	int16_t bytes_receive;
	//recvfrom get the source address
	bytes_receive = recv(sock_accept_tcp, ch, length, 0);
	if (bytes_receive < 0)
	{
		//fprintf(stderr, "recvfrom error:%s\n", strerror(errno));
		return -1;
	}
	else if (bytes_receive == 0)
	{
		kill(getpid(), SIGUSR1); 
		tcp_close();
		
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
