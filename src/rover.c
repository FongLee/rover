#include <stdio.h>
#include <pthread.h>
#include <errno.h>
#include <string.h>
#include <signal.h>
#include <stdbool.h>
#include <getopt.h>

#include "scheduler.h"
#include "my_timer.h"
#include "settings.h"
#include "communication.h"
#include "task.h"
#include "imu.h"
#include "ap_gps.h"
#include "ap_control.h"
#include "ap_ultrasonic.h"
#include "rtpsend.h"
#include "tcp_driver.h"
#ifdef MEMWATCH
#include "memwatch.h"
#endif

/**
 * register signal handler function
 * @return 0: seccess; -1: error
 */
int register_sig_handler();

/**
 * signal handler function
 * @param sig signa number
 */
void sig_main_handler(int sig);


int main_done = 0;
pthread_t transfer_thread;
pthread_t read_imu_thread;
pthread_t read_gps_thread;
pthread_t read_lowsensor_thread;
pthread_t control_thread;

pthread_t camera_thread;
pthread_t ultrasonic_thread;

/**
 * usage of programe
 * @param argv_0 name of programe
 */
void usage(char *argv_0)
{
	fprintf(stdout, "usage: %s [iptions]\n", argv_0);
	fprintf(stdout, "-p <ip address> the ip address of server\n");
	fprintf(stdout, "-h show the help\n");
	fprintf(stdout, "example: %s -p \"192.168.1.113\"\n", argv_0);
	exit(1);
}

/**
 * initialzaiton of rover
 * @return 0: success
 */
int  rover_init()
{
	scheduler_init();
	scheduler_begin(timer_update);
	global_data_reset_param_defaults();
	register_sig_handler();

	return 0;

}

/**
 * main programe
 * @param  argc
 * @param  argv
 * @return      0: success
 */
int main(int argc, char **argv)
{
	int opt;
	int len;
	char * ip_addr = NULL;

	while ( (opt = getopt(argc, argv, "p:hc")) != -1)
	{
		switch (opt)
		{
			case 'p':
			len = 1 + strlen(optarg);
			ip_addr = (char *) malloc(len);
			if (ip_addr == NULL)
			{
				fprintf(stderr, "malloc err: %s\n", strerror(errno));
			}
			strcpy(ip_addr, optarg);
			fprintf(stdout, "remote ip addr is %s\n", ip_addr);
			break;
			case 'h':
				usage(argv[0]);
				break;

			case 'c':
				flag_control_mode=2;
				break;

			default:
				usage(argv[0]);
				break;

		}
	}
	rover_init();

	int res;


	void *thread_result;

	if (mpu9150_init(i2c_bus_imu, sample_rate, yaw_mix_factor) == 0)
	{
		flag_imu_init = true;
		res = pthread_create(&read_imu_thread, NULL, task_read_imu, 0);
		if (res != 0)
		{
			fprintf(stderr, "task:read imu failed:%s\n", strerror(errno));
		}
	}

	if (gps_init() == 0 && ultrasonic_init()==0)
	{
		flag_gps_init = true;
		flag_control_init = true;
		res = pthread_create(&read_lowsensor_thread, NULL, task_read_lowsensor, 0);
		if (res != 0)
		{
			fprintf(stderr, "task:read_lowsensor failed:%s\n", strerror(errno));
		}
	}

	if (control_init() == 0)
	{
		flag_control_init = true;
		res = pthread_create(&control_thread, NULL, task_control, 0);
		if (res != 0)
		{
			fprintf(stderr, "task:control failed:%s\n", strerror(errno));
		}
	}

#ifdef TCP
	if (communication_init(sig_main_handler) == 0)
	{
		flag_communication_init = true;
		while(1)
		{
			delay_ms(1000);

			//only a connection can work
			if (! flag_communication_connect  && (tcp_accept() == 0) )
			{
				flag_communication_connect = true;

				res = pthread_create(&transfer_thread, NULL, task_transfer, 0);
				if (res != 0)
				{
					fprintf(stderr, "task:transfer failed:%s\n", strerror(errno));
				}
			}
		}

	}
#endif

#ifdef UDP
	if (communication_init(ip_addr) == 0)
	{
		flag_communication_init = true;
		flag_communication_connect = true;
		res = pthread_create(&transfer_thread, NULL, task_transfer, 0);
		if (res != 0)
		{
			fprintf(stderr, "task:transfer failed:%s\n", strerror(errno));
		}
	}

 	if(camera_init(ip_addr)==0)
 	{

 		res = pthread_create(&camera_thread, NULL, task_camera, 0);
		if (res != 0)
		{
			fprintf(stderr, "task:camera failed:%s\n", strerror(errno));
		}
 	}

	while(main_done == 0)
	{
		delay_ms(1000);
	}
#endif

	return 0;
}

/**
 * register signal handler function
 * @return 0: seccess; -1: error
 */
int register_sig_handler()
{
	struct sigaction sia;

	bzero(&sia, sizeof sia);
	sia.sa_handler = sig_main_handler;

	if (sigaction(SIGINT, &sia, NULL) < 0) {
		perror("sigaction(SIGINT)");
		//exit(1);
		return -1;
	}
	if (sigaction(SIGPIPE, &sia, NULL) < 0) {
		perror("sigaction(SIGINT)");
		//exit(1);
		return -1;
	}
	return 0;
}

/**
 * signal handler function
 * @param sig signa number
 */
void sig_main_handler(int sig)
{

	int res;
	if (sig == SIGINT)
	{
		sigint_handler(sig);

		//tcp_close();

		//tcp_destroy();
		delay_ms(1000);
#ifdef UDP
		main_done = 1;
#endif
		exit(0);

	}
	//receive from tcp_receive function when connection is closed
	if (sig == SIGUSR1)
	{

#ifdef SIG_DEBUG
		fprintf(stdout, "get SIGUSR1 signal\n");
#endif

		res = pthread_cancel(transfer_thread);
		if (res != 0)
			fprintf(stderr, "pthread_cancel err: %s\n", strerror(errno));
		flag_communication_connect = false;
	}

	if (sig == SIGPIPE)
	{
#ifdef SIG_DEBUG
		fprintf(stdout, "get SIGPIPE signal\n");
#endif
	}

}

