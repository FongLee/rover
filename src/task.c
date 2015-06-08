#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "communication.h"
#include "settings.h"
#include "my_timer.h"
#include "imu.h"
#include "mpu9150.h"
#include "ap_gps.h"
#include "ap_control.h"
#include "rtpsend.h"
#include "ap_ultrasonic.h"
#include "scheduler.h"

void *task_transfer()
{

	while(1)
	{
		usleep(5000);
		if (send_system_state_now)
		{
			send_system_state_now = false;
			if (global_data.param[PARAM_SYSTEM_SEND_STATE])
			{
				communication_system_state_send();
			}

		}

		if (send_params_now)
		{
			send_params_now = false;
			communication_parameter_send();

		}

		if (send_gps_now)
		{
			send_gps_now = false;
			communication_gps_send();


		}

		if (receive_now)
		{
			receive_now = false;
			communication_receive();

		}

		if (send_imu_now)
		{
			send_imu_now = false;
			communication_imu_send();
		}
	}
}

void *task_read_imu()
{
	uint32_t now;
	int opt;
	int verbose = 1;
	//int len = 0;

	//char *mag_cal_file = NULL;
	//char *accel_cal_file = NULL;

	int mag_mode = -1;
	done =0;
	//register_sig_handler();
	mpu9150_set_debug(verbose);

	//no calibrate
	if (mag_mode == -1)
	{
		set_cal(0);
		set_cal(1);
	}

	// if (accel_cal_file)
	// {
	// 	free(accel_cal_file);
	// }
	// if (mag_cal_file)
	// {
	// 	free(mag_cal_file);
	// }

	while(done == 0)
	{

		usleep(5000);
		if (mag_mode == 1)
		{
			fprintf(stdout, "mag start to calibration\n");
			if (mag_calibration()== 0)
			{
				fprintf(stdout, "mag calibration succeed\n");
				mag_mode =-1;
				mpu9150_init(i2c_bus_imu, sample_rate, yaw_mix_factor);
				if (mpu9150_init(i2c_bus_imu, sample_rate, yaw_mix_factor) == -1)
				{
					exit(1);
				}
				set_cal(0);
				set_cal(1);
			}
		}
		else if (mag_mode == 0)
		{
			fprintf(stdout, "acc start to calibration\n");
			if (acc_calibration()== 0)
			{
				fprintf(stdout, "acc calibration succeed\n");
				mag_mode =-1;
				mpu9150_init(i2c_bus_imu, sample_rate, yaw_mix_factor);
				if (mpu9150_init(i2c_bus_imu, sample_rate, yaw_mix_factor) == -1)
				{
					exit(1);
				}
				set_cal(0);
				set_cal(1);
			}
		}

		else
		{
			if (read_imu_now)
			{
				//get_ms(&now);
				//fprintf(stdout, "time is %lu\n", (long unsigned int)now);
				read_imu_now = false;
				if (mpu9150_read(&mpu) == 0)
				{
					//print_fused_euler_angles(&mpu);
					//fprintf(stdout, "read mpu9150 dmpTimestamp is %llu\n", (long long unsigned int)mpu.dmpTimestamp);
					//fprintf(stdout, "read mpu9150 magTimestamp is %llu\n", (long long unsigned int)mpu.magTimestamp);
					send_imu_now  = true;
				}
			}

		}

	}
	mpu9150_exit();
	return NULL;

}

void *task_read_gps()
{

	while(1)
	{
		usleep(10000);
		if (read_gps_now)
		{
			read_gps_now = false;
			gps_parse();
			send_gps_now = true;

		}
	}
}

void *task_control()
{
	while(1)
	{
		usleep(10000);
		if (begin_control)
		{
			begin_control = false;
			if (flag_control_mode != 0 )
			{
				setting_moto(channel_throttle);

				fprintf(stdout, "control mode is in the calibraton\n"); 
			}
			else 
			{
				if(flag_control_avoid==1)
				{
					if(channel_throttle>1700)
					moto_control(1700, channel_steer);
					else
						moto_control(channel_throttle, channel_steer);
					fprintf(stdout, "The distance is too close \n"); 
				}

				else

					moto_control(channel_throttle, channel_steer);
				fprintf(stdout, "control mode is in the control \n"); 
			}
		}

	}

}

void *task_read_ultrasonic()
{

	while(1)
	{
		usleep(50);
		if (read_ultrasonic_now)
		{
			read_ultrasonic_now = false;
			
			ultrasonic_read();
			fprintf(stdout,"distance=%f\n",distance);
			if(distance>10000)
			{
				flag_control_avoid=2;//超量程或者是0
			}
			if(distance<2000&&distance>0)
			{
				flag_control_avoid=1;//很近
			}
			else
			{
				flag_control_avoid=0;
			}
			uint64_t time1 = 0;
			get_us(&time1);

			// fprintf(fp_dis, "distance=%f time=%ld\n",distance,time1);
		

		}
	}
}
void *task_camera()
{
while(1)
{

//这一段涉及到异步IO

  // fd_set fds;
  // struct timeval tv;
 //  int r;

 //  FD_ZERO (&fds);//将指定的文件描述符集清空
 //  FD_SET (fd, &fds);//在文件描述符集合中增加一个新的文件描述符

   /* Timeout. */
 //  tv.tv_sec = 2;
 //  tv.tv_usec = 0;

 //  r = select (fd + 1, &fds, NULL, NULL, &tv);//判断是否可读（即摄像头是否准备好），tv是定时

 //  if (-1 == r) {
 //   if (EINTR == errno)
   //  continue;
 //   printf ("select err\n");
    //                    }
 //  if (0 == r) {
  //  fprintf (stderr, "select timeout\n");
  //  exit (EXIT_FAILURE);
   //                     }
	printf("read to send");
   if(read_frame ())
  {
    H264_Encode(nv12buffer,fp_h264);
    rtpSend(pRtpSession,oinfo.StrmVirAddr,oinfo.dataSize);

  }
    


} 

}
