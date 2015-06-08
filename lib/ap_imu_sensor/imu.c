
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <getopt.h>
#include <errno.h>
#include <stdbool.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include "imu.h"
#include "i2c_driver.h"
#include "local_defaults.h"
#include "scheduler.h"
#include "matrix.h"
#include "mpu_calibration.h"

#ifdef MEMWATCH
#include "memwatch.h"
#endif

int done = 0;


mpudata_t mpu;

int timer_read_data = 20;
int timer_print_mpu = 1000;
bool flag_read_imu = false;
bool flag_print_mpu = false;
bool flag_imu_init = false;
int i2c_bus_imu = DEFAULT_I2C_BUS;
int sample_rate = DEFAULT_SAMPLE_RATE_HZ;
//int sample_rate = 100;
int yaw_mix_factor = DEFAULT_YAW_MIX_FACTOR;

//int mag_mode;

//begin imu calibration
int mag_calibration()
{
	int group = 0;
	int mag_num = 0;
	char input[10];
	char buffer[32];
	long int sum[3] = {0};
	int fd ;
	MAT *measure_mag;
	VEC *cal_bias;
	VEC *cal_scale;

	measure_mag  = m_get(6, 3);
	cal_bias = v_get(3);
	cal_scale = v_get(3);

	while(done == 0 && group < 6)
	{
		fprintf(stdout, "please input \"mag\" to collect %dst the data\n", group + 1);
		bzero(input, 10);
		gets(input);
		if (strcmp(input, "mag") == 0)
		{
			mag_num  = 0;

			fprintf(stdout, "begin collect the %dst group data\n", group + 1);
			while (done == 0 && mag_num < 200)
			{
				if (mpu9150_read_mag(&mpu) == 0)
				{
					//print_mag(&mpu);
					printf("raw:X %d    Y %d    Z %d\n",
						mpu.rawMag[0],
						mpu.rawMag[1],
						mpu.rawMag[2]);

					//fflush(stdout);
					mag_num++;
					sum[0] += mpu.rawMag[0];
					sum[1] += mpu.rawMag[1];
					sum[2] += mpu.rawMag[2];
				}

				delay_ms(20);
			}

			if (mag_num == 200)
			{
				measure_mag->me[group][0] = (double) sum[0] / mag_num;
				measure_mag->me[group][1] = (double) sum[1] / mag_num;
				measure_mag->me[group][2] = (double) sum[2] / mag_num;
				sum[0] = 0;
				sum[1] = 0;
				sum[2] = 0;
				fprintf(stdout, "\n");
				printf("average:X %f    Y %f    Z %f\n",
						measure_mag->me[group][0],
						measure_mag->me[group][1],
						measure_mag->me[group][2]);
				group++;
			}

		}

	}

	if (group == 6)
	{
		imu_calibration(measure_mag, cal_bias, cal_scale);
		fd = open("magcal.txt", O_WRONLY | O_CREAT | O_TRUNC, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
		if (fd < 0)
		{
			fprintf(stderr, "open calfile failed\n");
			return -1;
		}

		for(int i = 0; i < 3; i++)
		{
			sprintf(buffer, "%f\n", cal_bias->ve[i]);
			write(fd, buffer, strlen(buffer));
		}
		for(int i = 0; i < 3; i++)
		{
			sprintf(buffer, "%f\n", cal_scale->ve[i]);
			write(fd, buffer, strlen(buffer));
		}

		close(fd);
		v_free(cal_bias);
		v_free(cal_scale);
		m_free(measure_mag);
		return 0;
	}
	else
	{
		v_free(cal_bias);
		v_free(cal_scale);
		m_free(measure_mag);
		return -1;
	}

}

int acc_calibration()
{
	int group = 0;
	int mag_num = 0;
	char input[10];
	char buffer[32];
	long int sum[3] = {0};
	int fd ;
	MAT *measure_mag;
	VEC *cal_bias;
	VEC *cal_scale;

	measure_mag  = m_get(6, 3);
	cal_bias = v_get(3);
	cal_scale = v_get(3);

	while(done == 0 && group < 6)
	{
		fprintf(stdout, "please input \"acc\" to collect %dst the data\n", group + 1);
		bzero(input, 10);
		gets(input);
		if (strcmp(input, "acc") == 0)
		{
			mag_num  = 0;

			fprintf(stdout, "begin collect the %dst group data\n", group + 1);
			if (mpu9150_init(i2c_bus_imu, sample_rate, yaw_mix_factor) == -1)
			{
				goto err;
			}
			while (done == 0 && mag_num < 200)
			{
				if (mpu9150_read_dmp(&mpu) == 0)
				{
					//print_mag(&mpu);
					printf("raw accel:X %d    Y %d    Z %d\n",
						mpu.rawAccel[0],
						mpu.rawAccel[1],
						mpu.rawAccel[2]);

					//fflush(stdout);
					mag_num++;
					sum[0] += mpu.rawAccel[0];
					sum[1] += mpu.rawAccel[1];
					sum[2] += mpu.rawAccel[2];
				}

				delay_ms(20);
			}

			if (mag_num == 200)
			{
				measure_mag->me[group][0] = (double) sum[0] / mag_num;
				measure_mag->me[group][1] = (double) sum[1] / mag_num;
				measure_mag->me[group][2] = (double) sum[2] / mag_num;
				sum[0] = 0;
				sum[1] = 0;
				sum[2] = 0;
				fprintf(stdout, "\n");
				printf("average:X %f    Y %f    Z %f\n",
						measure_mag->me[group][0],
						measure_mag->me[group][1],
						measure_mag->me[group][2]);
				group++;
			}

		}

	}

	if (group == 6)
	{
		m_output(measure_mag);
		imu_calibration(measure_mag, cal_bias, cal_scale);
		fd = open("accelcal.txt", O_WRONLY | O_CREAT | O_TRUNC, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
		if (fd < 0)
		{
			fprintf(stderr, "open calfile failed\n");
			return -1;
		}

		for(int i = 0; i < 3; i++)
		{
			sprintf(buffer, "%f\n", cal_bias->ve[i]);
			write(fd, buffer, strlen(buffer));
		}
		for(int i = 0; i < 3; i++)
		{
			sprintf(buffer, "%f\n", cal_scale->ve[i]);
			write(fd, buffer, strlen(buffer));
		}

		close(fd);
		v_free(cal_bias);
		v_free(cal_scale);
		m_free(measure_mag);
		return 0;
	}
	else
	{
		goto err;

	}
err:
	v_free(cal_bias);
	v_free(cal_scale);
	m_free(measure_mag);
	return -1;

}

void print_raw_mag(mpudata_t *mpu)
{
	printf("X:%5d    Y:%5d    Z:%5d",
			mpu->rawMag[0],
			mpu->rawMag[1],
			mpu->rawMag[2]);
}

void print_raw_accel(mpudata_t *mpu)
{
	printf("X:%5d    Y:%5d    Z:%5d",
		mpu->rawAccel[0],
		mpu->rawAccel[1],
		mpu->rawAccel[2]);
}
void print_fused_euler_angles(mpudata_t *mpu)
{
	printf("X: %0.2f Y: %0.2f Z: %0.2f\n",
			mpu->fusedEuler[VEC3_X] * RAD_TO_DEGREE,
			mpu->fusedEuler[VEC3_Y] * RAD_TO_DEGREE,
			mpu->fusedEuler[VEC3_Z] * RAD_TO_DEGREE);
}

void print_fused_quaternions(mpudata_t *mpu)
{
	printf("W: %0.2f X: %0.2f Y: %0.2f Z: %0.2f\n",
			mpu->fusedQuat[QUAT_W],
			mpu->fusedQuat[QUAT_X],
			mpu->fusedQuat[QUAT_Y],
			mpu->fusedQuat[QUAT_Z]);
}

void print_calibrated_accel(mpudata_t *mpu)
{
	printf("X: %5f Y: %5f Z: %5f\n",
			mpu->calibratedAccel[VEC3_X],
			mpu->calibratedAccel[VEC3_Y],
			mpu->calibratedAccel[VEC3_Z]);

}

void print_calibrated_mag(mpudata_t *mpu)
{
	printf("X: %5f Y: %5f Z: %5f\n",
			mpu->calibratedMag[VEC3_X],
			mpu->calibratedMag[VEC3_Y],
			mpu->calibratedMag[VEC3_Z]);
}

int set_cal(int mag)
{
	int i;
	FILE *f;
	char buff[32];
	float val[6];

	caldata_t cal;

		if (mag) {
			f = fopen("./magcal.txt", "r");

			if (!f) {
				printf("Default magcal.txt not found\n");
				return 0;
			}
		}
		else {
			f = fopen("./accelcal.txt", "r");

			if (!f) {
				printf("Default accelcal.txt not found\n");
				return 0;
			}
		}
	memset(buff, 0, sizeof(buff));

	if (mag)
	{
		for (i = 0; i < 6; i++)
		{
			if (!fgets(buff, 20, f))
			{
				fprintf(stderr, "not enough lines in the calibration file\n");
			}

			val[i] = atof(buff);

		}
		fclose(f);
		if (i != 6)
			return -1;
		cal.bias[0] = val[0];
		cal.bias[1] = val[1];
		cal.bias[2] = val[2];

		cal.scale[0] = val[3];
		cal.scale[1] = val[4];
		cal.scale[2] = val[5];
		mpu9150_set_mag_cal(&cal);
	}
	else
	{
		for (i = 0; i < 6; i++)
		{
			if (!fgets(buff, 20, f))
			{
				printf("Not enough lines in calibration file\n");
				break;
			}
			puts(buff);
			val[i] = atof(buff);

			if (val[i] < 0.000001 && val[i] > -0.000001)
			{
				printf("Invalid cal value: %s\n", buff);
				break;
			}
		}

		fclose(f);

		if (i != 6)
			return -1;

		cal.bias[0] = val[0];
		cal.bias[1] = val[1];
		cal.bias[2] = val[2];

		cal.scale[0] = val[3];
		cal.scale[1] = val[4];
		cal.scale[2] = val[5];

		mpu9150_set_accel_cal(&cal);
	}

	return 0;
}



void sigint_handler(int sig)
{
	done = 1;
}
