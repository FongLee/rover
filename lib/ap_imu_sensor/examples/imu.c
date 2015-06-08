
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

#include "mpu9150.h"
#include "i2c_driver.h"
#include "local_defaults.h"
#include "scheduler.h"
#include "matrix.h"
#include "mpu_calibration.h"

int set_cal(int mag);
void read_loop(unsigned int sample_rate);
void print_fused_euler_angles(mpudata_t *mpu);
void print_fused_quaternions(mpudata_t *mpu);
void print_calibrated_accel(mpudata_t *mpu);
void print_calibrated_mag(mpudata_t *mpu);
void register_sig_handler();
void sigint_handler(int sig);
int mag_calibration();
int acc_calibration();

int done = 0;

mpudata_t mpu;

int timer_read_data = 20;
int timer_print_mpu = 1000;
bool flag_read_imu = false;
bool flag_print_mpu = false;
int i2c_bus_imu = DEFAULT_I2C_BUS;
int sample_rate = DEFAULT_SAMPLE_RATE_HZ;
//int sample_rate = 100;
int yaw_mix_factor = DEFAULT_YAW_MIX_FACTOR;

int mag_mode;

void timer_update(union sigval v)
{
	timer_read_data--;
	timer_print_mpu--;
	if (timer_read_data <= 0)
	{
		flag_read_imu = true;
		timer_read_data = 20; //20ms
	}
	if (timer_print_mpu <=0)
	{
		flag_print_mpu = true;
		timer_print_mpu = 1000; //1s
	}


}

void usage(char *argv_0)
{
	printf("\nUsage: %s [options]\n", argv_0);
	printf("  -b <i2c-bus>          The I2C bus number where the IMU is. The default is 1 to use /dev/i2c-1.\n");
	printf("  -s <sample-rate>      The IMU sample rate in Hz. Range 2-50, default 10.\n");
	printf("  -y <yaw-mix-factor>   Effect of mag yaw on fused yaw data.\n");
	printf("                           0 = gyro only\n");
	printf("                           1 = mag only\n");
	printf("                           > 1 scaled mag adjustment of gyro data\n");
	printf("                           The default is 4.\n");
	printf("  -a 					Accelerometer calibration\n");
	printf("  -m 					Magnetometer calibration\n");
	printf("  -v                    Verbose messages\n");
	printf("  -h                    Show this help\n");

	printf("\nExample: %s -b3 -s20 -y10\n\n", argv_0);

	exit(1);
}

int main(int argc, char **argv )
{

	uint32_t now;
	int opt;
	int verbose = 1;
	//int len = 0;

	//char *mag_cal_file = NULL;
	//char *accel_cal_file = NULL;

	mag_mode = -1;
	done =0;
	while ( (opt = getopt(argc, argv, "b:s:y:amvh")) != -1)
	{
		switch (opt)
		{
			case 'b':
				i2c_bus_imu = strtoul(optarg, NULL, 0);
				if (errno == EINVAL)
				{
					usage(argv[0]);
				}
				if (i2c_bus_imu < MIN_I2C_BUS || i2c_bus_imu > MAX_I2C_BUS)
				{
					usage(argv[0]);
				}
				break;
			case 's':
				sample_rate = strtoul(optarg, NULL, 0);
				if (errno == EINVAL)
				{
					usage(argv[0]);
				}
				if (sample_rate < MIN_SAMPLE_RATE || sample_rate > MAX_SAMPLE_RATE)
				{
					usage(argv[0]);
				}
				break;
			case 'a':
				// len = 1 + strlen(optarg);
				// accel_cal_file = (char *)malloc(len);
				// if (!accel_cal_file)
				// {
				// 	fprintf(stderr, "malloc err: %s\n", strerror(errno));
				// }
				// strcpy(accel_cal_file, optarg);
				if (mag_mode != -1)
				{
					usage(argv[0]);
				}
				mag_mode = 0;
				break;
			case 'm':
				// len = 1 + strlen(optarg);
				// mag_cal_file = (char *)malloc(len);
				// if (!mag_cal_file)
				// {
				// 	fprintf(stderr, "malloc err : %s\n", strerror(errno));
				// }
				// strcpy(mag_cal_file, optarg);
				// fprintf(stdout, "mag_cal_file is %s\n", mag_cal_file);
				fprintf(stdout, "mag test\n");
				if (mag_mode != -1)
				{
					usage(argv[0]);
				}
				mag_mode = 1;
				break;
			case 'y':
				yaw_mix_factor = strtoul(optarg, NULL, 0);
				if (errno == EINVAL)
				{
					usage(argv[0]);
				}
				if (yaw_mix_factor < 0 || yaw_mix_factor > 100)
				{
					usage(argv[0]);
				}
				break;
			case 'v':
				verbose = 1;
				break;
			case 'h':
			default:
				usage(argv[0]);
				break;
		}
	}
	scheduler_init();
	scheduler_begin(timer_update);
	register_sig_handler();
	mpu9150_set_debug(verbose);
	if (mpu9150_init(i2c_bus_imu, sample_rate, yaw_mix_factor) == -1)
	{
		exit(1);
	}
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
			if (flag_read_imu)
			{
				//get_ms(&now);
				//fprintf(stdout, "time is %lu\n", (long unsigned int)now);
				flag_read_imu = false;
				if (mpu9150_read(&mpu) == 0)
				{
					//print_fused_euler_angles(&mpu);
					//fprintf(stdout, "read mpu9150 dmpTimestamp is %lu\n", mpu.dmpTimestamp);
					//fprintf(stdout, "read mpu9150 magTimestamp is %lu\n", mpu.magTimestamp);
				}
			}
			if (flag_print_mpu)
			{
				get_ms(&now);
				//fprintf(stdout, "time is %lu\n", (long unsigned int)now);

				fprintf(stdout, "\n");
				fprintf(stdout, "\n");
				fprintf(stdout, "fused_euler_angles is :\n");
				print_fused_euler_angles(&mpu);
				fprintf(stdout, "rawAccel is :\n");
				printf("\rX: %05d Y: %05d Z: %05d\n",
					mpu.rawAccel[VEC3_X],
					mpu.rawAccel[VEC3_Y],
					mpu.rawAccel[VEC3_Z]);
				fprintf(stdout, "calibrated_accel is :\n");
				print_calibrated_accel(&mpu);
				fprintf(stdout, "g is %f m/s\n", sqrt(mpu.calibratedAccel[0] * mpu.calibratedAccel[0] +mpu.calibratedAccel[1] *mpu.calibratedAccel[1] + mpu.calibratedAccel[2] * mpu.calibratedAccel[2]));
				fprintf(stdout, "rawMag is :\n");
				printf("X: %05d Y: %05d Z: %05d\n",
					mpu.rawMag[VEC3_X],
					mpu.rawMag[VEC3_Y],
					mpu.rawMag[VEC3_Z]);
				fprintf(stdout, "calibrated_mag is :\n");
				print_calibrated_mag(&mpu);
				flag_print_mpu = false;

			}
		}

	}
	mpu9150_exit();
	return 0;
}

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

void register_sig_handler()
{
	struct sigaction sia;

	bzero(&sia, sizeof sia);
	sia.sa_handler = sigint_handler;

	if (sigaction(SIGINT, &sia, NULL) < 0) {
		perror("sigaction(SIGINT)");
		exit(1);
	}
}

void sigint_handler(int sig)
{
	done = 1;
}
