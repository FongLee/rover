#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdbool.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "ap_control.h"

#define ACC 0
#define STREER 1
#define STOP  3

int fd_control = -1;
bool flag_control_init = false;
int flag_control_mode = 0;
int flag_control_avoid;
int channel_steer;
int channel_throttle;

int control_init()
{


	fd_control=open("/dev/pwm",O_WRONLY | O_CREAT | O_APPEND);
	if(fd_control < 0)
	{
		fprintf(stderr, "PWM failed:%s\n", strerror(errno));
		return -1;
	}

	//ioctl(fd,0,10);

	return 0;
}

void control_close()
{
	close(fd_control);
}

void setting_moto(int throttle)
{
	if(flag_control_mode ==1 &&throttle>=1400&&throttle<=1600)
	{
		ioctl(fd_control,ACC,20);
		flag_control_mode = 0;
		fprintf(stdout, "output low pwm\n");
	}
	if(flag_control_mode == 2 && throttle>=1000&&throttle<=1100)
	{
		ioctl(fd_control,ACC,10);
		fprintf(stdout, "output high pwm\n");
		flag_control_mode = 1;
		
	}

}

void moto_control(int throttle, int steering)
{
	static int before_steering;
	static int before_acc;

	steering=steering-100;
	//fprintf(stdout, "%d", steering);

	if(before_steering != steering / 100)
	{
		ioctl(fd_control,STREER,steering / 100);
		before_steering=steering / 100;
	}


	if(throttle >=1000 && throttle <=1500)
	{
		ioctl(fd_control,ACC,20);
	}
	else
	{
		throttle=3500 - throttle;
		if(before_acc != throttle / 100)
		{
			ioctl(fd_control,ACC,throttle/100);
			before_acc=throttle/100;
		}
	}

}

