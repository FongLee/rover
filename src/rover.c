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
#ifdef MEMWATCH
#include "memwatch.h"
#endif


int register_sig_handler();

void sigint_main_handler(int sig);


int main_done = 0;

void usage(char *argv_0)
{
	fprintf(stdout, "usage: %s [iptions]\n", argv_0);
	fprintf(stdout, "-p <ip address> the ip address of server\n");
	fprintf(stdout, "-h show the help\n");
	fprintf(stdout, "example: %s -p \"192.168.1.113\"\n", argv_0);
	exit(1);
}

int  rover_init()
{
	scheduler_init();
	scheduler_begin(timer_update);
	global_data_reset_param_defaults();
	register_sig_handler();

	return 0;

}
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
	pthread_t transfer_thread;
	pthread_t read_imu_thread;
	pthread_t read_gps_thread;
	pthread_t control_thread;

	pthread_t camera_thread;
	pthread_t ultrasonic_thread;

	void *thread_result;

	
	if (communication_init(ip_addr) == 0)
	{
		flag_communication_init = true;
		res = pthread_create(&transfer_thread, NULL, task_transfer, 0);
		if (res != 0)
		{
			fprintf(stderr, "task:transfer failed:%s\n", strerror(errno));
		}
	}

	if (mpu9150_init(i2c_bus_imu, sample_rate, yaw_mix_factor) == 0)
	{
		flag_imu_init = true;
		res = pthread_create(&read_imu_thread, NULL, task_read_imu, 0);
		if (res != 0)
		{
			fprintf(stderr, "task:read imu failed:%s\n", strerror(errno));
		}
	}


	if (gps_init() == 0)
	{
		flag_gps_init = true;
		res = pthread_create(&read_gps_thread, NULL, task_read_gps, 0);
		if (res != 0)
		{
			fprintf(stderr, "task:read gps failed:%s\n", strerror(errno));
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



 	if(camera_init(ip_addr)==0)
 	{

 		res = pthread_create(&camera_thread, NULL, task_camera, 0);
		if (res != 0)
		{
			fprintf(stderr, "task:camera failed:%s\n", strerror(errno));
		}
 	}
	
	
 	if(ultrasonic_init()==0)
 	{
 		res = pthread_create(&ultrasonic_thread, NULL, task_read_ultrasonic, 0);
		if (res != 0)
		{
			fprintf(stderr, "task:ultrasonic failed:%s\n", strerror(errno));
		}
 	}




	//while(main_done == 0)
	//{
		//sleep(1);
	//}



	res = pthread_join(read_imu_thread, &thread_result);
	return 0;
}

int register_sig_handler()
{
	struct sigaction sia;

	bzero(&sia, sizeof sia);
	sia.sa_handler = sigint_main_handler;

	if (sigaction(SIGINT, &sia, NULL) < 0) {
		perror("sigaction(SIGINT)");
		//exit(1);
		return -1;
	}
	return 0;
}

void sigint_main_handler(int sig)
{
	sigint_handler(sig);
	main_done = 1;


}

