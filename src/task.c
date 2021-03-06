#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>

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
#include "ap_math.h"
#include "ap_navigation.h"


/**
 * transfer task
 * @return 0: success; -1: error
 */
void *task_transfer()
{

	int res;

	res = pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);
	if (res != 0)
		return (void *)-1;

	res = pthread_detach(pthread_self());
	if (res != 0)
		return (void *)-1;

	while(1)
	{
		usleep(5000);
		if(!flag_communication_connect)
			break;

		if (send_system_state_now && flag_communication_connect)
		{
			send_system_state_now = false;
			if (global_data.param[PARAM_SYSTEM_SEND_STATE])
			{
				communication_system_state_send();
			}

		}

		if (send_params_now && flag_communication_connect)
		{
			send_params_now = false;
			communication_parameter_send();

		}

		if (send_gps_now && flag_communication_connect)
		{
			send_gps_now = false;
			communication_gps_send();


		}

		if (receive_now && flag_communication_connect)
		{
			receive_now = false;
			communication_receive();

		}

		if (send_imu_now && flag_communication_connect)
		{
			send_imu_now = false;
			communication_imu_send();
		}
	}
	return (void *) 0;
}

/**
 * read imu task
 * @return 0: success; -1: error
 */
void *task_read_imu()
{
	int res;
	res = pthread_detach(pthread_self());
	if (res != 0)
		return (void *)-1;

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

	while(done == 0)
	{

		usleep(5000);
		//fprintf(stdout, "task_read_imu thread ID id %d", getpid());
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
					//fprintf(stdout, "\n");
					//fprintf(stdout, "%f %f %f\n", mpu.fusedEuler[VEC3_X] * RAD_TO_DEG, mpu.fusedEuler[VEC3_Y] * RAD_TO_DEG, mpu.fusedEuler[VEC3_Z] * RAD_TO_DEG);

#ifdef USE_NAVIGATION				
					inertial_navigation(&nav_data, &mpu, &gps_data, USE_MPU);
#endif

					send_imu_now  = true;
				}
			}

		}

	}
	mpu9150_exit();
	return NULL;

}

/**
 * read low sensor task including GPS and ultrasonic distance measurement
 * @return 0: success; -1: error
 */
void *task_read_lowsensor()
{
	int res;
	res = pthread_detach(pthread_self());
	if (res != 0)
		return (void *)-1;
	float distance;

	while(1)
	{
		delay_ms(50);
		res = gps_parse(&gps_data);
		if (res == 0)
		{
			send_gps_now = true;
		}

		res = ultrasonic_read(&distance);
		if (res == 0)
		{
			if(distance > 10000)
			{
				flag_control_avoid = 2;//out of range or read value is zero
			}
			if(distance < 2000 && distance > 0)
			{
				flag_control_avoid = 1;//very close
			}
			else
			{
				flag_control_avoid= 0;
			}

#ifdef ULTRA_DEBUG
			fprintf(stdout,"distance=%f\n",distance);
#endif
		}

	}


}

/**
 * read gps task
 * @return 0: success; -1: error
 */
void *task_read_gps()
{
	int res;
	res = pthread_detach(pthread_self());
	if (res != 0)
		return (void *)-1;

	while(1)
	{
		//usleep(10000);
		delay_ms(500);
		if (read_gps_now)
		{
			read_gps_now = false;
			gps_parse(&gps_data);
			if(gps_data.flag_gps_glitching == false)
			{
				
#ifdef 	GPS_DEBUG
				fprintf(stdout, "get gps\n");
#endif
				
#ifdef USE_NAVIGATION
				inertial_navigation(&nav_data, &mpu, &gps_data, USE_GPS);				
#endif	

				send_gps_now = true;
			}
			

		}
	}
}

/**
 * control throttle and steer task
 * @return 0: success; -1: error
 */
void *task_control()
{
	int res;
	res = pthread_detach(pthread_self());
	if (res != 0)
		return (void *)-1;

	while(1)
	{
		delay_ms(10);
		if (begin_control)
		{
			begin_control = false;
			if (flag_control_mode != 0 )
			{
				setting_moto(channel_throttle);
#ifdef CONTROL_DEBUG
				fprintf(stdout, "control mode is in the calibraton\n");
#endif
			}
			else
			{

				if(flag_control_avoid==1)
				{
					if(channel_throttle>1700)
					moto_control(1700, channel_steer);
					else
						moto_control(channel_throttle, channel_steer);
#ifdef ULTRA_DEBUG

					fprintf(stdout, "The distance is too close \n");
#endif
				}

				else

					moto_control(channel_throttle, channel_steer);
#ifdef CONTROL_DEBUG
				fprintf(stdout, "control mode is in the control \n");
#endif
			}
		}

	}

}

/**
 *  ultrasonic distance measure task
 * @return 0: success; -1: error
 */
void *task_read_ultrasonic()
{

	int res;
	res = pthread_detach(pthread_self());
	if (res != 0)
		return (void *)-1;
	int distance;
	while(1)
	{
		delay_ms(50);
		if (read_ultrasonic_now)
		{
			read_ultrasonic_now = false;

			res = ultrasonic_read(&distance);
			if (res == 0)
			{

#ifdef ULTRA_DEBUG
				fprintf(stdout,"distance=%f\n",distance);
#endif
				if(distance>10000)
				{
					flag_control_avoid=2;//exceed the range or zero
				}
				if(distance<2000&&distance>0)
				{
					flag_control_avoid=1;//very close
				}
				else
				{
					flag_control_avoid=0;
				}
			}

		}
	}
}

/**
 * read video task
 * @return [description]
 */
void *task_camera()
{
	int res;
	res = pthread_detach(pthread_self());
	if (res != 0)
		return (void *)-1;

	while(1)
	{
		//delay_us(50);

	  	if(read_frame ())
	  	{
			H264_Encode(nv12buffer,fp_h264);
	    	rtpSend(pRtpSession,oinfo.StrmVirAddr,oinfo.dataSize);

	 	}
	}

}
