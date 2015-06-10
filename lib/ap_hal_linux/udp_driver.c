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

#define UDP_PORT 14550
#define RTP_PORT 3020

bool flag_udp_init = false;
struct sockaddr_in gc_addr, rc_addr;
socklen_t fromlen;
int sock_udp = -1;

int udp_init(char *ip)
{
	
	struct sockaddr_in loc_addr;
	sock_udp = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (sock_udp == -1)
	{
		fprintf(stderr, "sock initialize failed: %s \n", strerror(errno));
		return -1;
	}
	memset(&loc_addr, 0, sizeof(loc_addr));
	loc_addr.sin_family = AF_INET;
	loc_addr.sin_addr.s_addr = INADDR_ANY;
	loc_addr.sin_port = htons(UDP_PORT);
	if (-1 == bind(sock_udp, (struct sockaddr *)&loc_addr, sizeof(struct sockaddr)))
	{
		fprintf(stderr, "bind failed: %s \n", strerror(errno));
		close(sock_udp);
		return -1;
	}
	//if (-1 == fcntl(sock_udp, F_SETFL, O_NONBLOCK | FASYNC))
	if (-1 == fcntl(sock_udp, F_SETFL, O_NONBLOCK | FASYNC))
	{
		fprintf(stderr, "setting nonblocking err: %s \n", strerror(errno));
		close(sock_udp);
		return -1;
	}
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

void udp_close()
{
	flag_udp_init = false;
	if (sock_udp != -1)
	{
		close(sock_udp);
		sock_udp = -1;
	}
}

int udp_send(const uint8_t *ch, uint16_t length)
{
	int16_t bytes_send = 0;
	bytes_send = sendto(sock_udp, ch, length, 0, (struct sockaddr *)&gc_addr,
								sizeof(struct sockaddr_in));

	if (bytes_send == -1)
	{

#ifdef UDP_DEBUG
		fprintf(stderr, "udp send err: %s \n", strerror(errno));
#endif
		return -1;
	}


	return bytes_send;
}

int udp_receive(uint8_t *ch, uint16_t length)
{
	int16_t bytes_receive;
	//recvfrom get the source address
	bytes_receive = recvfrom(sock_udp, ch, length, 0, (struct sockaddr *)&rc_addr, &fromlen);

#ifdef UDP_DEBUG
	if (bytes_receive < 0)
	{
		fprintf(stderr, "recvfrom error:%s\n", strerror(errno));
	}
	else
	{
		fprintf(stdout, "remote ip is: %s,port is %d\n",
						inet_ntoa(rc_addr.sin_addr), ntohs(rc_addr.sin_port));
		fprintf(stdout, "bytes received:%d datagram:\n", bytes_receive);
		for (int j = 0; j < bytes_receive; j++)
		{
			fprintf(stdout, "%02x ", ch[j]);
		}
		fprintf(stdout, "\n");
	}
#endif
	return bytes_receive;
}
