////////////////////////////////////////////////////////////////////////////
//
//  This file is part of linux-mpu9150
//
//  Copyright (c) 2013 Pansenti, LLC
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of
//  this software and associated documentation files (the "Software"), to deal in
//  the Software without restriction, including without limitation the rights to use,
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
//  Software, and to permit persons to whom the Software is furnished to do so,
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

// Date			Author			Notes
// 17/12/2015     FinnickLee     	change

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <stdbool.h>
#include <sys/time.h> //gettimeofday

#include <sys/times.h> //times


#include <time.h> //clock_gettime

#include "i2c_driver.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "mpu9150.h"
#include "matrix_kalman.h"
//#include "matrix_kalman.h"
#include "kalman.h"
#include "math.h"
#include "ap_math.h"
#include "scheduler.h"
#include "ap_ahrs.h"
#include "ekf.h"

#ifdef MEMWATCH
#include "memwatch.h"
#endif

#define GRO_FACTOR 	16.38
#define Q_ANGLE 	0.001
#define Q_BIAS 		0.003
//#define Q_QUAT 		0.1
#define Q_QUAT 		0.01
#define R_ACCEL 	0.3
#define R_MAG 		3.0

int kalman_init(void);
static int data_ready();
static void calibrate_data(mpudata_t *mpu);
static void tilt_compensate(quaternion_t magQ, quaternion_t unfusedQ);
static int data_fusion(mpudata_t *mpu);
static unsigned short inv_row_2_scale(const signed char *row);
static unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx);
static void run_self_test(void);

int data_fusion_kalman(mpudata_t *mpu);
int data_fusion_kalman_three(mpudata_t *mpu);
int data_fusion_kalman_three_improve(mpudata_t *mpu);
int data_fusion_ekf(mpudata_t *mpu);


int debug_on;
int yaw_mixing_factor;

int use_accel_cal;
caldata_t accel_cal_data;

int use_mag_cal;
caldata_t mag_cal_data;

kalman_t *f_yaw;
kalman_t *kalman_three_dim;
ekf_t 	*kalman_ekf;

#define DELT_T 			0.01




/**
 * kalman apply in yaw estimate
 * @return  0:success
 */
int kalman_init(void)
{
	alloc_kalman_filter(&f_yaw, 2, 1, 1);

	set_matrix(f_yaw->state_transition,
				1.0,  	-DELT_T,
				0.0, 		1.0);
	set_matrix(f_yaw->control_input_model, (double)DELT_T, 0.0);
	set_matrix(f_yaw->measure_model, 1.0, 0.0);

	set_matrix(f_yaw->process_noise_covariance,
				Q_ANGLE, 	0.0,
				0.0, 		Q_BIAS);
	//set_matrix(f_yaw->process_noise_covariance,
	//			0.00001, 	0.0,
	//			0.0, 		0.00003);
	// set_matrix(f_yaw->process_noise_covariance,
	//  			10.0, 	0.0,
	//  			0.0, 		10.0);

	//set_matrix(f_yaw->measure_noise_covariance, 0.03);
	set_matrix(f_yaw->measure_noise_covariance, R_MAG);
	//set_matrix(f_yaw->measure_noise_covariance, 50.0);


	float deviation = 1000.0;
	set_matrix(f_yaw->state_estimate, 0.0, 0.0);
	m_ident(f_yaw->covariance_estimate);
	sm_mlt(deviation * deviation, f_yaw->covariance_estimate, f_yaw->covariance_estimate);

	return 0;
}

/**
 * kalman apply in euler(three-dimensional)
 * @return  0:success
 */
int kalman_init_three(void)
{
	alloc_kalman_filter(&kalman_three_dim, 6, 3, 3);

	set_matrix(kalman_three_dim->state_transition,
				1.0, -DELT_T,	0.0, 0.0, 		0.0, 0.0,
				0.0, 1.0, 		0.0, 0.0, 		0.0, 0.0,
				0.0, 0.0, 		1.0, -DELT_T, 	0.0, 0.0,
				0.0 ,0.0, 		0.0, 1.0, 		0.0, 0.0,
				0.0, 0.0, 		0.0, 0.0, 		1.0, -DELT_T,
				0.0, 0.0, 		0.0, 0.0, 		0.0, 1.0);
	set_matrix(kalman_three_dim->control_input_model,
				DELT_T,	0.0, 	0.0,
				0.0, 	0.0, 	0.0,
				0.0, 	DELT_T,	0.0,
				0.0, 	0.0, 	0.0,
				0.0, 	0.0, 	DELT_T,
				0.0, 	0.0, 	0.0);
	set_matrix(kalman_three_dim->measure_model,
				1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
				0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
				0.0, 0.0, 0.0, 0.0, 1.0, 0.0);

	set_matrix(kalman_three_dim->process_noise_covariance,
				Q_ANGLE,0.0,	0.0,	0.0, 	0.0, 	0.0,
				0.0,	Q_BIAS,	0.0, 	0.0, 	0.0, 	0.0,
				0.0, 	0.0, 	Q_ANGLE,0.0, 	0.0, 	0.0,
				0.0, 	0.0, 	0.0, 	Q_BIAS,	0.0, 	0.0,
				0.0, 	0.0, 	0.0, 	0.0, 	Q_ANGLE,0.0,
				0.0, 	0.0, 	0.0, 	0.0, 	0.0,	Q_BIAS);
	sm_mlt(DELT_T, kalman_three_dim->process_noise_covariance, kalman_three_dim->process_noise_covariance);

	//set_matrix(kalman_three_dim->measure_noise_covariance,
	//			0.03, 0.0, 0.0,
	//			0.0, 0.03, 0.0,
	//			0.0, 0.0, 0.03);
	set_matrix(kalman_three_dim->measure_noise_covariance,
				R_ACCEL,0.0, 		0.0,
				0.0, 	R_ACCEL, 	0.0,
				0.0, 	0.0, 		R_MAG);

	float deviation = 1000.0;
	set_matrix(kalman_three_dim->state_estimate,
					0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	//unit matrix
	m_ident(kalman_three_dim->covariance_estimate);
	sm_mlt(deviation * deviation, kalman_three_dim->covariance_estimate, kalman_three_dim->covariance_estimate);

	return 0;
}

int ekf_init(void)
{
	alloc_ekf_filter(&kalman_ekf, 4, 6, 3);
	set_matrix(kalman_ekf->process_noise_covariance,
				Q_QUAT,	0.0,	0.0,	0.0,
				0.0,	Q_QUAT,	0.0, 	0.0,
				0.0, 	0.0, 	Q_QUAT,	0.0,
				0.0, 	0.0, 	0.0, 	Q_QUAT);
	sm_mlt(DELT_T, kalman_ekf->process_noise_covariance, kalman_ekf->process_noise_covariance);
	set_matrix(kalman_ekf->measure_noise_covariance,
				R_ACCEL,0.0,	0.0, 	0.0,	0.0,	0.0,
				0.0,	R_ACCEL,0.0,	0.0,	0.0,	0.0,
				0.0, 	0.0, 	R_ACCEL,0.0,	0.0,	0.0,
				0.0,	0.0,	0.0,	R_MAG,	0.0,	0.0,
				0.0,	0.0, 	0.0,	0.0, 	R_MAG, 	0.0,
				0.0, 	0.0, 	0.0, 	0.0, 	0.0,	R_MAG);

	float deviation = 1000.0;
	set_matrix(kalman_ekf->state_estimate,
					0.0, 0.0, 0.0, 0.0);
	//unit matrix
	m_ident(kalman_ekf->covariance_estimate);
	sm_mlt(deviation * deviation, kalman_ekf->covariance_estimate, kalman_ekf->covariance_estimate);

	return 0;
	
}

/**
 * verify whether print calibration of accelerometer and gyroscope
 * @param on 1: print; 0: no print
 */
void mpu9150_set_debug(int on)
{
	debug_on = on;
}

int mpu9150_init(int i2c_bus, int sample_rate, int mix_factor)
{
	signed char gyro_orientation[9] = { 1, 0, 0,
                                        0, 1, 0,
                                        0, 0, 1 };
	// signed char gyro_orientation[9] = { 1, 0, 0,
 //                                        0, -1, 0,
 //                                        0, 0, -1 };

	if (i2c_bus < MIN_I2C_BUS || i2c_bus > MAX_I2C_BUS) {
		printf("Invalid I2C bus %d\n", i2c_bus);
		return -1;
	}

	if (sample_rate < MIN_SAMPLE_RATE || sample_rate > MAX_SAMPLE_RATE) {
		printf("Invalid sample rate %d\n", sample_rate);
		return -1;
	}

	if (mix_factor < 0 || mix_factor > 100) {
		printf("Invalid mag mixing factor %d\n", mix_factor);
		return -1;
	}

	yaw_mixing_factor = mix_factor;

	linux_set_i2c_bus(i2c_bus);

	printf("\nInitializing IMU .");
	fflush(stdout);

	if (mpu_init(NULL)) {
		printf("\nmpu_init() failed\n");
		return -1;
	}

	printf(".");
	fflush(stdout);

	if (mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS)) {
		printf("\nmpu_set_sensors() failed\n");
		return -1;
	}

	printf(".");
	fflush(stdout);

	if (mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL)) {
		printf("\nmpu_configure_fifo() failed\n");
		return -1;
	}

	printf(".");
	fflush(stdout);

	if (mpu_set_sample_rate(sample_rate)) {
		printf("\nmpu_set_sample_rate() failed\n");
		return -1;
	}

	printf(".");
	fflush(stdout);

	if (mpu_set_compass_sample_rate(sample_rate)) {
		printf("\nmpu_set_compass_sample_rate() failed\n");
		return -1;
	}

	printf(".");
	fflush(stdout);

	if (dmp_load_motion_driver_firmware()) {
		printf("\ndmp_load_motion_driver_firmware() failed\n");
		return -1;
	}

	printf(".");
	fflush(stdout);

	if (dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation))) {
		printf("\ndmp_set_orientation() failed\n");
		return -1;
	}

	printf(".");
	fflush(stdout);
	/*
	DMP_FEATURE_LP_QUAT and DMP_FEATURE_6X_LP_QUAT are mutually
 *  exclusive.
 *  DMP_FEATURE_SEND_RAW_GYRO and DMP_FEATURE_SEND_CAL_GYRO are also
 *  mutually exclusive.
 *  */
  	if (dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL
						| DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL)) {
		printf("\ndmp_enable_feature() failed\n");
		return -1;
	}

	printf(".");
	fflush(stdout);

	if (dmp_set_fifo_rate(sample_rate)) {
		printf("\ndmp_set_fifo_rate() failed\n");
		return -1;
	}
	//test in it self
	// printf(".");
	// fflush(stdout);
	// run_self_test();

	printf(".");
	fflush(stdout);

	if (mpu_set_dmp_state(1)) {
		printf("\nmpu_set_dmp_state(1) failed\n");
		return -1;
	}

	printf(".");
	fflush(stdout);

#ifdef 	KALMAN_THREE_APPLLY
	kalman_init_three();
#elif 	KALMAN_APPLLY
	kalman_init();
#elif 	EKF_APPLLY
	ekf_init();
#endif

	printf(" done\n\n");

	return 0;
}

void mpu9150_exit()
{
	// turn off the DMP on exit
		if (mpu_set_dmp_state(0))
			printf("mpu_set_dmp_state(0) failed\n");

	//free memory of kalman filter
	free_kalman_filter(&f_yaw);

	// TODO: Should turn off the sensors too
}


void run_self_test(void)
{
    int result;
	// char test_packet[4] = {0};
    long gyro[3], accel[3];
	//
    result = mpu_run_self_test(gyro, accel);
    fprintf(stdout, "\n");
    fprintf(stdout, "result is %d\n", result);
    if(result == 0x07) {
        fprintf(stdout, "test result is good \n");
    }
    unsigned char i=0;
    for (i=0; i < 3; i++) {
        fprintf(stdout,"gyro is %ld, accel is %ld\n",gyro[i], accel[i]);
    }
    //fprintf(stdout,"gyro is %d, accel is %d",gyro[i], accel[i]);
  if (result == 0x7)
		//if (result == 0x3)
		{
        /* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP.
         */
        //fprintf(stdout, "\nself test passed\n");
        float sens;
        unsigned short accel_sens;
        mpu_get_gyro_sens(&sens);
        gyro[0] = (long)(gyro[0] * sens);
        gyro[1] = (long)(gyro[1] * sens);
        gyro[2] = (long)(gyro[2] * sens);
        dmp_set_gyro_bias(gyro);
        mpu_get_accel_sens(&accel_sens);
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        dmp_set_accel_bias(accel);
    }
}

void mpu9150_set_accel_cal(caldata_t *cal)
{
	int i;
	long bias[3];

	if (!cal) {
		use_accel_cal = 0;
		return;
	}

	memcpy(&accel_cal_data, cal, sizeof(caldata_t));


	if (debug_on) {
		printf("\naccel cal (bias : scale)\n");

		for (i = 0; i < 3; i++)
			printf("%f : %f\n", accel_cal_data.bias[i], accel_cal_data.scale[i]);
	}

	//must multiplee 4.906,do not know why
	bias[0] = (long)(accel_cal_data.bias[0] * 4.096);
	bias[1] = (long)(accel_cal_data.bias[1] * 4.096) ;
	bias[2] = (long)(accel_cal_data.bias[2] * 4.096);
	for (int i = 0; i < 3; i++)
	{
		fprintf(stdout, "bias is %ld\n", bias[i]);
	}
	mpu_set_accel_bias(bias);

	use_accel_cal = 1;
}

void mpu9150_set_mag_cal(caldata_t *cal)
{
	int i;

	if (!cal) {
		use_mag_cal = 0;
		return;
	}

	memcpy(&mag_cal_data, cal, sizeof(mag_caldata_t));

	if (debug_on) {
		printf("\nmag cal (bias : scale)\n");

		for (i = 0; i < 3; i++)
			printf("%f : %f\n", mag_cal_data.bias[i], mag_cal_data.scale[i]);
	}

	use_mag_cal = 1;
}

int mpu9150_read_dmp(mpudata_t *mpu)
{
	short sensors;
	unsigned char more;

	if (!data_ready())
		return -1;

	if (dmp_read_fifo(mpu->rawGyro, mpu->rawAccel, mpu->rawQuat, &mpu->dmpTimestamp, &sensors, &more) < 0) {

#ifdef MPU9150_DEBUG_ERROR

		printf("dmp_read_fifo() failed\n");
#endif

		return -1;
	}

#ifdef MPU9150_DEBUG
	fprintf(stdout, "sensors is %d\n", sensors);
	fprintf(stdout, "more is %d\n", more);
#endif
	while (more) {
		// Fell behind, reading again
		if (dmp_read_fifo(mpu->rawGyro, mpu->rawAccel, mpu->rawQuat, &mpu->dmpTimestamp, &sensors, &more) < 0) {

#ifdef MPU9150_DEBUG_ERROR
			printf("dmp_read_fifo() failed\n");
#endif

			return -1;
		}

#ifdef MPU9150_DEBUG
	fprintf(stdout, "sensors is %d\n", sensors);
	fprintf(stdout, "more is %d\n", more);
#endif

	}

	return 0;
}

int mpu9150_read_mag(mpudata_t *mpu)
{
	if (mpu_get_compass_reg(mpu->rawMag, &mpu->magTimestamp) < 0) {
		printf("mpu_get_compass_reg() failed\n");
		return -1;
	}

	return 0;
}

int mpu9150_read(mpudata_t *mpu)
{
	if (mpu9150_read_dmp(mpu) != 0)
		return -1;

	if (mpu9150_read_mag(mpu) != 0)
		return -1;

	calibrate_data(mpu);

#ifdef MPU9150_DEBUG
	static uint64_t tmp_time;
	get_ns(&tmp_time);
#endif


#ifdef AHRS_APPLLY
	update_ahrs(mpu);
#elif KALMAN_THREE_APPLLY
	//data_fusion_kalman_three_improve(mpu);
	data_fusion_kalman_three(mpu);
#elif EKF_APPLLY
	data_fusion_ekf(mpu);
#elif KALMAN_APPLLY
	data_fusion_kalman(mpu);
#else 
	data_fusion(mpu);
#endif

#ifdef MPU9150_DEBUG
	static uint64_t tmp_time2;
	get_ns(&tmp_time2);
	fprintf(stdout, "\n");
	fprintf(stdout, "fusion time is %lu ns\n", ( unsigned long)(tmp_time2 - tmp_time));
#endif

	return 0;
}

/**
 * verify whethe data in mpu9150 fifo is ready
 * @return
 */
int data_ready()
{
	short status;

	if (mpu_get_int_status(&status) < 0) {
		printf("mpu_get_int_status() failed\n");
		return 0;
	}

	// debug
	//if (status != 0x0103)
	//	fprintf(stderr, "%04X\n", status);

	return (status == (MPU_INT_STATUS_DATA_READY | MPU_INT_STATUS_DMP | MPU_INT_STATUS_DMP_0));
}


void calibrate_data(mpudata_t *mpu)
{
	if (use_mag_cal) 
	{
		static float mag_data[3];
		mag_data[VEC3_X] = ((float)mpu->rawMag[0] + mag_cal_data.bias[0] ) * mag_cal_data.scale[0];
		mag_data[VEC3_Y] = ((float)mpu->rawMag[1] + mag_cal_data.bias[1] ) * mag_cal_data.scale[1];
		mag_data[VEC3_Z] = ((float)mpu->rawMag[2] + mag_cal_data.bias[2] ) * mag_cal_data.scale[2];

#ifdef AHRS_APPLLY
		//Front-Left-Up body coordinate system
		mpu->calibratedMag[VEC3_X] =  mag_data[VEC3_Y];
		mpu->calibratedMag[VEC3_Y] =  mag_data[VEC3_X];
		mpu->calibratedMag[VEC3_Z] =  -mag_data[VEC3_Z];
#elif KALMAN_THREE_APPLLY
		mpu->calibratedMag[VEC3_X] =  mag_data[VEC3_Y];
		mpu->calibratedMag[VEC3_Y] =  mag_data[VEC3_X];
		mpu->calibratedMag[VEC3_Z] =  -mag_data[VEC3_Z];
#elif EKF_APPLLY
		mpu->calibratedMag[VEC3_X] =  mag_data[VEC3_Y];
		mpu->calibratedMag[VEC3_Y] =  mag_data[VEC3_X];
		mpu->calibratedMag[VEC3_Z] =  -mag_data[VEC3_Z];
#elif KALMAN_APPLLY
		//Front-Right-Down body coordinate system
		mpu->calibratedMag[VEC3_Y] = -mag_data[0];
		mpu->calibratedMag[VEC3_X] =  mag_data[1];
		mpu->calibratedMag[VEC3_Z] =  mag_data[2];
#else 
		//Front-Left-Up body coordinate system
	 	mpu->calibratedMag[VEC3_X] =  mag_data[VEC3_Y];
		mpu->calibratedMag[VEC3_Y] =  mag_data[VEC3_X];
		mpu->calibratedMag[VEC3_Z] =  -mag_data[VEC3_Z];
#endif

	}
	else 
	{
		//Front-Left-Up body coordinate system
		mpu->calibratedMag[VEC3_Y] = (float)mpu->rawMag[VEC3_X];
		mpu->calibratedMag[VEC3_X] = (float)mpu->rawMag[VEC3_Y];
		mpu->calibratedMag[VEC3_Z] = -(float)mpu->rawMag[VEC3_Z];
	}

	if (use_accel_cal) 
	{

#ifdef AHRS_APPLLY
    	mpu->calibratedAccel[VEC3_X] = ((float)mpu->rawAccel[VEC3_X]) * accel_cal_data.scale[VEC3_X];
      	mpu->calibratedAccel[VEC3_Y] = ((float)mpu->rawAccel[VEC3_Y]) * accel_cal_data.scale[VEC3_Y];
      	mpu->calibratedAccel[VEC3_Z] = ((float)mpu->rawAccel[VEC3_Z])* accel_cal_data.scale[VEC3_Z];
#elif KALMAN_THREE_APPLLY
		mpu->calibratedAccel[VEC3_X] = ((float)mpu->rawAccel[VEC3_X]) * accel_cal_data.scale[VEC3_X];
		mpu->calibratedAccel[VEC3_Y] = ((float)mpu->rawAccel[VEC3_Y]) * accel_cal_data.scale[VEC3_Y];
		mpu->calibratedAccel[VEC3_Z] = ((float)mpu->rawAccel[VEC3_Z])* accel_cal_data.scale[VEC3_Z];
#elif EKF_APPLLY
		mpu->calibratedAccel[VEC3_X] = ((float)mpu->rawAccel[VEC3_X]) * accel_cal_data.scale[VEC3_X];
		mpu->calibratedAccel[VEC3_Y] = ((float)mpu->rawAccel[VEC3_Y]) * accel_cal_data.scale[VEC3_Y];
		mpu->calibratedAccel[VEC3_Z] = ((float)mpu->rawAccel[VEC3_Z])* accel_cal_data.scale[VEC3_Z];
#elif KALMAN_APPLLY
		mpu->calibratedAccel[VEC3_X] = -((float)mpu->rawAccel[VEC3_X]) * accel_cal_data.scale[VEC3_X];
		mpu->calibratedAccel[VEC3_Y] = ((float)mpu->rawAccel[VEC3_Y]) * accel_cal_data.scale[VEC3_Y];
		mpu->calibratedAccel[VEC3_Z] = ((float)mpu->rawAccel[VEC3_Z])* accel_cal_data.scale[VEC3_Z];
#else
		mpu->calibratedAccel[VEC3_X] = ((float)mpu->rawAccel[VEC3_X]) * accel_cal_data.scale[VEC3_X];
		mpu->calibratedAccel[VEC3_Y] = ((float)mpu->rawAccel[VEC3_Y]) * accel_cal_data.scale[VEC3_Y];
		mpu->calibratedAccel[VEC3_Z] = ((float)mpu->rawAccel[VEC3_Z])* accel_cal_data.scale[VEC3_Z];
#endif

	}
	else 
	{
		mpu->calibratedAccel[VEC3_X] = (float)mpu->rawAccel[VEC3_X];
		mpu->calibratedAccel[VEC3_Y] = (float)mpu->rawAccel[VEC3_Y];
		mpu->calibratedAccel[VEC3_Z] = (float)mpu->rawAccel[VEC3_Z];
	}

#ifdef AHRS_APPLLY
	mpu->rawGyro[VEC3_X] = mpu->rawGyro[VEC3_X];
	mpu->rawGyro[VEC3_Y] = mpu->rawGyro[VEC3_Y];
	mpu->rawGyro[VEC3_Z] = mpu->rawGyro[VEC3_Z];
#elif KALMAN_THREE_APPLLY
	mpu->rawGyro[VEC3_X] = mpu->rawGyro[VEC3_X];
	mpu->rawGyro[VEC3_Y] = mpu->rawGyro[VEC3_Y];
	mpu->rawGyro[VEC3_Z] = mpu->rawGyro[VEC3_Z];
#elif EKF_APPLLY
	mpu->rawGyro[VEC3_X] = mpu->rawGyro[VEC3_X];
	mpu->rawGyro[VEC3_Y] = mpu->rawGyro[VEC3_Y];
	mpu->rawGyro[VEC3_Z] = mpu->rawGyro[VEC3_Z];
#elif KALMAN_APPLLY
	mpu->rawGyro[VEC3_X] = mpu->rawGyro[VEC3_X];
	mpu->rawGyro[VEC3_Y] = -mpu->rawGyro[VEC3_Y];
	mpu->rawGyro[VEC3_Z] = -mpu->rawGyro[VEC3_Z];
#else 
	mpu->rawGyro[VEC3_X] = mpu->rawGyro[VEC3_X];
	mpu->rawGyro[VEC3_Y] = mpu->rawGyro[VEC3_Y];
	mpu->rawGyro[VEC3_Z] = mpu->rawGyro[VEC3_Z];
#endif


}

void tilt_compensate(quaternion_t magQ, quaternion_t unfusedQ)
{
	quaternion_t unfusedConjugateQ;
	quaternion_t tempQ;

	quaternionConjugate(unfusedQ, unfusedConjugateQ);
	quaternionMultiply(magQ, unfusedConjugateQ, tempQ);
	quaternionMultiply(unfusedQ, tempQ, magQ);
}

/**
 * data fusion to estimate euler using DMP in MPU9150
 * @param  mpu
 * @return     0:success
 */

int data_fusion(mpudata_t *mpu)
{
	quaternion_t dmpQuat;
	vector3d_t dmpEuler;
	dmpQuat[QUAT_W] = (float)mpu->rawQuat[QUAT_W];
	dmpQuat[QUAT_X] = (float)mpu->rawQuat[QUAT_X];
	dmpQuat[QUAT_Y] = (float)mpu->rawQuat[QUAT_Y];
	dmpQuat[QUAT_Z] = (float)mpu->rawQuat[QUAT_Z];

	quaternionNormalize(dmpQuat);
	quaternionToEuler(dmpQuat, dmpEuler);

	mpu->fusedEuler[VEC3_X] = dmpEuler[VEC3_X];
	mpu->fusedEuler[VEC3_Y] = -dmpEuler[VEC3_Y];
	mpu->fusedEuler[VEC3_Z] = -dmpEuler[VEC3_Z];

	if (mpu->fusedEuler[VEC3_Z] < 0)
	{
		mpu->fusedEuler[VEC3_Z] += 2 * PI;
	}

	return 0;
}


/**
 * kalman filter apply in data fusion to estimate euler
 * @param  mpu
 * @return     0:success
 */
int data_fusion_kalman_three(mpudata_t *mpu)
{


	//static quaternion_t dmp_quat;
	static vector3d_t measure_euler;
	float tmp_norm;
	tmp_norm = inv_sqrt(mpu->calibratedAccel[VEC3_X] * mpu->calibratedAccel[VEC3_X]
							+ mpu->calibratedAccel[VEC3_Y] * mpu->calibratedAccel[VEC3_Y]
							+ mpu->calibratedAccel[VEC3_Z] * mpu->calibratedAccel[VEC3_Z]);
	//pitch
	measure_euler[VEC3_Y] = -asin(mpu->calibratedAccel[VEC3_X] * tmp_norm);
	//roll
	measure_euler[VEC3_X] = atan2(mpu->calibratedAccel[VEC3_Y], mpu->calibratedAccel[VEC3_Z]);

	static double hn_y, hn_x;
	static double mag[3];
	mag[VEC3_X] = mpu->calibratedMag[VEC3_X];
	mag[VEC3_Y] = mpu->calibratedMag[VEC3_Y];
	mag[VEC3_Z] = mpu->calibratedMag[VEC3_Z];

	hn_y = mag[VEC3_Y] * cos(measure_euler[VEC3_X]) - mag[VEC3_Z] * sin(measure_euler[VEC3_X]);
	hn_x = mag[VEC3_X] * cos(measure_euler[VEC3_Y])
				+ mag[VEC3_Y] * sin(measure_euler[VEC3_X]) * sin(measure_euler[VEC3_Y])
				+ mag[VEC3_Z] * cos(measure_euler[VEC3_X]) * sin(measure_euler[VEC3_Y]);
	measure_euler[VEC3_Z] = (double) atan2(-hn_y, hn_x);
	// change the range of mag_angle into 0 - 2 * PI,it is necessary
	if(measure_euler[VEC3_Z]  < 0)
	{
		measure_euler[VEC3_Z]  += 2 * PI;
	}
	set_matrix(kalman_three_dim->measure,
				measure_euler[VEC3_X], measure_euler[VEC3_Y], measure_euler[VEC3_Z]);
	
	static double delt_t = 0.0;
	static vector3d_t last_gyro;
	static uint64_t last_kalman_time;
	static bool kalman_time_init = false;
	if (kalman_time_init)
	{
		delt_t = (double)(mpu->magTimestamp - last_kalman_time);
		last_kalman_time = mpu->magTimestamp;
		set_matrix(kalman_three_dim->control_input,
				(double)last_gyro[VEC3_X] / GRO_FACTOR / 180 * PI,
				(double)last_gyro[VEC3_Y] / GRO_FACTOR / 180 * PI,
				(double)last_gyro[VEC3_Z] / GRO_FACTOR / 180 * PI);
	
		last_gyro[VEC3_X] = mpu->rawGyro[VEC3_X];
		last_gyro[VEC3_Y] = mpu->rawGyro[VEC3_Y];
		last_gyro[VEC3_Z] = mpu->rawGyro[VEC3_Z];
		
#ifdef KALMAN_DEBUG
		fprintf(stdout, "delt_t is %f\n", delt_t * 1e-3);
#endif

		set_matrix(kalman_three_dim->state_transition,
				1.0, -delt_t * 1e-3, 0.0, 0.0,            0.0, 0.0,
				0.0, 1.0, 		     0.0, 0.0,            0.0, 0.0,
				0.0, 0.0,            1.0, -delt_t * 1e-3, 0.0, 0.0,
				0.0, 0.0,            0.0, 1.0,            0.0, 0.0,
				0.0, 0.0,            0.0, 0.0,            1.0, -delt_t * 1e-3,
				0.0, 0.0,            0.0, 0.0,            0.0, 1.0);

		set_matrix(kalman_three_dim->control_input_model,
						delt_t * 1e-3, 	0.0, 			0.0,
						0.0,           	0.0, 			0.0,
						0.0,      		delt_t * 1e-3, 	0.0,
						0.0, 			0.0, 			0.0,
						0.0, 			0.0, 			delt_t * 1e-3,
						0.0, 			0.0, 			0.0);
		set_matrix(kalman_three_dim->process_noise_covariance,
					Q_ANGLE,0.0,	0.0,	0.0,	0.0,	0.0,
					0.0,	Q_BIAS, 0.0,	0.0,	0.0,	0.0,
					0.0,	0.0,	Q_ANGLE,0.0,	0.0,	0.0,
					0.0,	0.0,	0.0,	Q_BIAS, 0.0,	0.0,
					0.0,	0.0,	0.0,	0.0,	Q_ANGLE,0.0,
					0.0,	0.0,	0.0,	0.0,	0.0,	Q_BIAS);
		sm_mlt(delt_t * 1e-3, kalman_three_dim->process_noise_covariance, kalman_three_dim->process_noise_covariance);
	}
	else
	{
		last_kalman_time = mpu->magTimestamp;
		kalman_time_init = true ;
		last_gyro[VEC3_X] = mpu->rawGyro[VEC3_X];
		last_gyro[VEC3_Y] = mpu->rawGyro[VEC3_Y];
		last_gyro[VEC3_Z] = mpu->rawGyro[VEC3_Z];
		kalman_three_dim->state_estimate->me[0][0] = measure_euler[VEC3_X];
		kalman_three_dim->state_estimate->me[2][0] = measure_euler[VEC3_Y];
		kalman_three_dim->state_estimate->me[4][0] = measure_euler[VEC3_Z];
		
		return -1;
	}

#ifdef KALMAN_DEBUG
	fprintf(stdout, "measure euler, x : %f, y : %f, z : %f\n", measure_euler[VEC3_X] * RAD_TO_DEG, measure_euler[VEC3_Y] * RAD_TO_DEG, measure_euler[VEC3_Z] * RAD_TO_DEG);
	fprintf(stdout, "\n");
	fprintf(stdout, "measure is : \n");
	m_output(kalman_three_dim->measure);
	fprintf(stdout, "\n");
	fprintf(stdout, "control_input is \n");
	m_output(kalman_three_dim->control_input);
#endif

	predict(kalman_three_dim);

	update(kalman_three_dim);

#ifdef KALMAN_DEBUG
	fprintf(stdout, "covariance_estimate is :\n");
	m_output(kalman_three_dim->covariance_estimate);
#endif

	mpu->fusedEuler[VEC3_X] = kalman_three_dim->state_estimate->me[0][0];
	mpu->fusedEuler[VEC3_Y] = -kalman_three_dim->state_estimate->me[2][0];
	mpu->fusedEuler[VEC3_Z] = -kalman_three_dim->state_estimate->me[4][0] + 2 * PI;
	//the range of mpu->fuseEuler is -PI to PI
	if (mpu->fusedEuler[VEC3_Z] < 0)
	{
		mpu->fusedEuler[VEC3_Z] += 2 * PI;
	}
	return 0;

}


/**
 * kalman filter apply in data fusion to estimate euler, input in an average filter
 * @param  mpu
 * @return     0:success
 */
int data_fusion_kalman_three_improve(mpudata_t *mpu)
{
	static double last_kalman_time = 0;
	static bool kalman_time_init = false;
	static double delt_t = 0.0;
	static vector3d_t last_gyro;
	if (kalman_time_init)
	{
		delt_t = (double)(mpu->magTimestamp - last_kalman_time);
		last_kalman_time = mpu->magTimestamp;

		set_matrix(kalman_three_dim->control_input,
				(double)last_gyro[VEC3_X] / GRO_FACTOR / 180 * PI,
				(double)last_gyro[VEC3_Y] / GRO_FACTOR / 180 * PI,
				(double)last_gyro[VEC3_Z] / GRO_FACTOR / 180 * PI);
		last_gyro[VEC3_X] = mpu->rawGyro[VEC3_X];
		last_gyro[VEC3_Y] = mpu->rawGyro[VEC3_Y];
		last_gyro[VEC3_Z] = mpu->rawGyro[VEC3_Z];

		set_matrix(kalman_three_dim->state_transition,
				1.0, -delt_t * 1e-3, 0.0, 0.0,            0.0, 0.0,
				0.0, 1.0, 		     0.0, 0.0,            0.0, 0.0,
				0.0, 0.0,            1.0, -delt_t * 1e-3, 0.0, 0.0,
				0.0, 0.0,            0.0, 1.0,            0.0, 0.0,
				0.0, 0.0,            0.0, 0.0,            1.0, -delt_t * 1e-3,
				0.0, 0.0,            0.0, 0.0,            0.0, 1.0);

		set_matrix(kalman_three_dim->control_input_model,
						delt_t * 1e-3, 	0.0, 			0.0,
						0.0,           	0.0, 			0.0,
						0.0,      		delt_t * 1e-3, 	0.0,
						0.0, 			0.0, 			0.0,
						0.0, 			0.0, 			delt_t * 1e-3,
						0.0, 			0.0, 			0.0);

		set_matrix(kalman_three_dim->process_noise_covariance,
					Q_ANGLE,0.0,	0.0,	0.0,	0.0,	0.0,
					0.0,	Q_BIAS, 0.0,	0.0,	0.0,	0.0,
					0.0,	0.0,	Q_ANGLE,0.0,	0.0,	0.0,
					0.0,	0.0,	0.0,	Q_BIAS, 0.0,	0.0,
					0.0,	0.0,	0.0,	0.0,	Q_ANGLE,0.0,
					0.0,	0.0,	0.0,	0.0,	0.0,	Q_BIAS);
		sm_mlt(delt_t * 1e-3, kalman_three_dim->process_noise_covariance, kalman_three_dim->process_noise_covariance);
	}
	else
	{
		last_kalman_time = mpu->magTimestamp;
		kalman_time_init = true ;
		last_gyro[VEC3_X] = mpu->rawGyro[VEC3_X];
		last_gyro[VEC3_Y] = mpu->rawGyro[VEC3_Y];
		last_gyro[VEC3_Z] = mpu->rawGyro[VEC3_Z];
		return -1;
	}

	predict(kalman_three_dim);

	//static quaternion_t dmp_quat;
	static vector3d_t measure_euler;
	static vector3d_t measure_euler_sum;
	float tmp_norm;
	tmp_norm = inv_sqrt(mpu->calibratedAccel[VEC3_X] * mpu->calibratedAccel[VEC3_X]
							+ mpu->calibratedAccel[VEC3_Y] * mpu->calibratedAccel[VEC3_Y]
							+ mpu->calibratedAccel[VEC3_Z] * mpu->calibratedAccel[VEC3_Z]);
	//pitch
	measure_euler[VEC3_Y] = -asin(mpu->calibratedAccel[VEC3_X] * tmp_norm);
	//roll
	measure_euler[VEC3_X] = atan2(mpu->calibratedAccel[VEC3_Y], mpu->calibratedAccel[VEC3_Z]);

	static double hn_y, hn_x;
	static double mag[3];
	mag[VEC3_X] = mpu->calibratedMag[VEC3_X];
	mag[VEC3_Y] = mpu->calibratedMag[VEC3_Y];
	mag[VEC3_Z] = mpu->calibratedMag[VEC3_Z];
	hn_y = mag[VEC3_Y] * cos(measure_euler[VEC3_X]) - mag[VEC3_Z] * sin(measure_euler[VEC3_X]);
	hn_x = mag[VEC3_X] * cos(measure_euler[VEC3_Y])
				+ mag[VEC3_Y] * sin(measure_euler[VEC3_X]) * sin(measure_euler[VEC3_Y])
				+ mag[VEC3_Z] * cos(measure_euler[VEC3_X]) * sin(measure_euler[VEC3_Y]);
	measure_euler[VEC3_Z] = (double) atan2(-hn_y, hn_x);
	// change the range of mag_angle into 0 - 2 * PI,it is necessary
	if(measure_euler[VEC3_Z]  < 0)
	{
		measure_euler[VEC3_Z]  += 2 * PI;
	}
	static int num_measure = 0;
	const int NUM_TIME = 5;
	if(num_measure == NUM_TIME)
	{
		num_measure = 0;
		measure_euler[VEC3_X] = measure_euler_sum[VEC3_X] / NUM_TIME;
		measure_euler[VEC3_Y] = measure_euler_sum[VEC3_Y] / NUM_TIME;
		measure_euler[VEC3_Z] = measure_euler_sum[VEC3_Z] / NUM_TIME;
		set_matrix(kalman_three_dim->measure,
						measure_euler[VEC3_X], measure_euler[VEC3_Y], measure_euler[VEC3_Z]);
		update(kalman_three_dim);
		measure_euler_sum[VEC3_X] = 0;
		measure_euler_sum[VEC3_Y] = 0;
		measure_euler_sum[VEC3_Z] = 0;
	}
	else if(num_measure < NUM_TIME)
	{
		num_measure++;
		measure_euler_sum[VEC3_X] += measure_euler[VEC3_X];
		measure_euler_sum[VEC3_Y] += measure_euler[VEC3_Y];
		measure_euler_sum[VEC3_Z] += measure_euler[VEC3_Z];
		kalman_three_dim->state_estimate->me[0][0] = kalman_three_dim->state_predict->me[0][0];
		kalman_three_dim->state_estimate->me[2][0] = kalman_three_dim->state_predict->me[2][0];
		kalman_three_dim->state_estimate->me[4][0] = kalman_three_dim->state_predict->me[4][0];

	}
	mpu->fusedEuler[VEC3_X] = kalman_three_dim->state_estimate->me[0][0];
	mpu->fusedEuler[VEC3_Y] = -kalman_three_dim->state_estimate->me[2][0];
	mpu->fusedEuler[VEC3_Z] = -kalman_three_dim->state_estimate->me[4][0] + 2 * PI;

	//the range of mpu->fuseEuler is -PI to PI
	if (mpu->fusedEuler[VEC3_Z] < 0)
	{
		mpu->fusedEuler[VEC3_Z] += 2 * PI;
	}

	return 0;

}



/**
 * ekf filter apply in data fusion to estimate euler
 * @param  mpu
 * @return     0:success
 */
int data_fusion_ekf(mpudata_t *mpu)
{

	static vector3d_t measure_euler;
	static float tmp_norm;
	static double delt_t = 0.0;
	static vector3d_t last_gyro;
	static uint64_t last_kalman_time;
	static bool kalman_time_init = false;
	static quaternion_t quat;
	static float tmp_x, tmp_y, tmp_z;
	static float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;  
	static float hx, hy, bx, bz;
	static float ax, ay, az;
	static float mx, my, mz;
	static float wx, wy, wz;
	static float vx, vy, vz;
	static float q0, q1, q2, q3;
	quat[QUAT_W] = kalman_ekf->state_estimate->me[0][0];
	quat[QUAT_X] = kalman_ekf->state_estimate->me[1][0];
	quat[QUAT_Y] = kalman_ekf->state_estimate->me[2][0];
	quat[QUAT_Z] = kalman_ekf->state_estimate->me[3][0];
	if (kalman_time_init)
	{
		delt_t = (double)(mpu->magTimestamp - last_kalman_time);
		last_kalman_time = mpu->magTimestamp;
		
#ifdef KALMAN_DEBUG
		fprintf(stdout, "delt_t is %f\n", delt_t * 1e-3);
#endif

		tmp_x = last_gyro[VEC3_X] * delt_t * 1e-3 / 2.0;
		tmp_y = last_gyro[VEC3_Y] * delt_t * 1e-3 / 2.0;
		tmp_z = last_gyro[VEC3_Z] * delt_t * 1e-3 / 2.0;
		last_gyro[VEC3_X] = mpu->rawGyro[VEC3_X] / GRO_FACTOR / 180 * PI;
		last_gyro[VEC3_Y] = mpu->rawGyro[VEC3_Y] / GRO_FACTOR / 180 * PI;
		last_gyro[VEC3_Z] = mpu->rawGyro[VEC3_Z] / GRO_FACTOR / 180 * PI;
		set_matrix(kalman_ekf->state_transition,
				1.0, 	-tmp_x,	-tmp_y,	-tmp_z,
				tmp_x, 	1.0,	tmp_z, 	tmp_y,
				tmp_y, 	-tmp_z, 1.0, 	tmp_x,
				tmp_z, 	tmp_y, 	-tmp_x, 1.0);

		set_matrix(kalman_ekf->process_noise_covariance,
					Q_QUAT,	0.0,	0.0,	0.0,
					0.0,	Q_QUAT, 0.0,	0.0,
					0.0,	0.0,	Q_QUAT,	0.0,
					0.0,	0.0,	0.0,	Q_QUAT);
		sm_mlt(delt_t * 1e-3, kalman_ekf->process_noise_covariance, kalman_ekf->process_noise_covariance);

		quat[QUAT_W] = quat[QUAT_W] - quat[QUAT_X] * tmp_x - quat[QUAT_Y] * tmp_y - quat[QUAT_Z] * tmp_z;
		quat[QUAT_X] = quat[QUAT_X] + quat[QUAT_W] * tmp_x + quat[QUAT_Y] * tmp_z + quat[QUAT_Z] * tmp_y;
		quat[QUAT_Y] = quat[QUAT_Y] + quat[QUAT_W] * tmp_y - quat[QUAT_X] * tmp_z + quat[QUAT_Z] * tmp_x;
		quat[QUAT_Z] = quat[QUAT_Z] + quat[QUAT_W] * tmp_z + quat[QUAT_X] * tmp_y - quat[QUAT_Y] * tmp_x;
		quaternionNormalize(quat);
		set_matrix(kalman_ekf->state_predict, 
					quat[QUAT_W], quat[QUAT_X], quat[QUAT_Y], quat[QUAT_Z]);
#ifdef 	KALMAN_DEBUG
		quaternionToEuler(quat, measure_euler);
		fprintf(stdout, "predict euler, x : %f, y : %f, z : %f\n", measure_euler[VEC3_X] * RAD_TO_DEG, measure_euler[VEC3_Y] * RAD_TO_DEG, measure_euler[VEC3_Z] * RAD_TO_DEG); 
#endif	
		// Normalise accelerometer measurement
		ax = mpu->calibratedAccel[VEC3_X];
		ay = mpu->calibratedAccel[VEC3_Y];
		az = mpu->calibratedAccel[VEC3_Z];
		tmp_norm = inv_sqrt(ax * ax + ay * ay + az * az);
		ax *= tmp_norm;
		ay *= tmp_norm;
		az *= tmp_norm;     

		// Normalise magnetometer measurement
		mx = mpu->calibratedMag[VEC3_X];
		my = mpu->calibratedMag[VEC3_Y];
		mz = mpu->calibratedMag[VEC3_Z];
		tmp_norm = inv_sqrt(mx * mx + my * my + mz * mz);
		mx *= tmp_norm;
		my *= tmp_norm;
		mz *= tmp_norm; 
		set_matrix(kalman_ekf->measure,
					ax, ay, az, mx, my, mz);
		
		q0 = quat[QUAT_W];
		q1 = quat[QUAT_X];
		q2 = quat[QUAT_Y];
		q3 = quat[QUAT_Z];

		// Auxiliary variables to avoid repeated arithmetic
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3; 

        // Reference direction of Earth's magnetic field
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

		vx = 2 * (q1q3 - q0q2);
		vy = 2 * (q0q1 + q2q3);
		vz = 2 * (q0q0 - 0.5f + q3q3);
		wx = 2 * (bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2));
        wy = 2 * (bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3));
        wz = 2 * (bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2)); 
		set_matrix(kalman_ekf->tmp_measure_dimension, 
					vx, vy, vz, wx, wy, wz);
		set_matrix(kalman_ekf->measure_model, 
					-2 * q2, 					2 * q3, 				-2 * q0, 					2 * q1,
					2 * q1, 					2 * q0, 				2 * q3,						2 * q2,
					2 * q0, 					-2 * q1, 				-2 * q2, 					2 * q3, 
					2 * (q0 * bx - q2 * bz), 	2 * (q1 * bx + q3 * bz), 2 * (-q2 * bx - q0 * bz), 	2 * (-q3 * bx + q1 * bz),
					2 * (-q3 * bx + q1 * bz), 	2 * (q2 * bx + q0 * bz), 2 * (q1 * bx + q3 * bz), 	2 * (-q0 * bx + q2 * bz),
					2 * (q2 * bx + q0 * bz), 	2 * (q3 * bx - q1 * bz), 2 * (q0 * bx - q2 * bz), 	2 * (q1 * bx + q3 * bz));

	}
	else
	{


		tmp_norm = inv_sqrt(mpu->calibratedAccel[VEC3_X] * mpu->calibratedAccel[VEC3_X]
								+ mpu->calibratedAccel[VEC3_Y] * mpu->calibratedAccel[VEC3_Y]
								+ mpu->calibratedAccel[VEC3_Z] * mpu->calibratedAccel[VEC3_Z]);
		//pitch
		measure_euler[VEC3_Y] = -asin(mpu->calibratedAccel[VEC3_X] * tmp_norm);
		//roll
		measure_euler[VEC3_X] = atan2(mpu->calibratedAccel[VEC3_Y], mpu->calibratedAccel[VEC3_Z]);
		
		static double hn_y, hn_x;
		static double mag[3];
		mag[VEC3_X] = mpu->calibratedMag[VEC3_X];
		mag[VEC3_Y] = mpu->calibratedMag[VEC3_Y];
		mag[VEC3_Z] = mpu->calibratedMag[VEC3_Z];
		
		hn_y = mag[VEC3_Y] * cos(measure_euler[VEC3_X]) - mag[VEC3_Z] * sin(measure_euler[VEC3_X]);
		hn_x = mag[VEC3_X] * cos(measure_euler[VEC3_Y])
					+ mag[VEC3_Y] * sin(measure_euler[VEC3_X]) * sin(measure_euler[VEC3_Y])
					+ mag[VEC3_Z] * cos(measure_euler[VEC3_X]) * sin(measure_euler[VEC3_Y]);
		measure_euler[VEC3_Z] = (double) atan2(-hn_y, hn_x);

		last_kalman_time = mpu->magTimestamp;
		kalman_time_init = true ;
		last_gyro[VEC3_X] = mpu->rawGyro[VEC3_X] / GRO_FACTOR / 180 * PI;
		last_gyro[VEC3_Y] = mpu->rawGyro[VEC3_Y] / GRO_FACTOR / 180 * PI;
		last_gyro[VEC3_Z] = mpu->rawGyro[VEC3_Z] / GRO_FACTOR / 180 * PI;
#ifdef 	KALMAN_DEBUG
		fprintf(stdout, "initial alignment euler, x : %f, y : %f, z : %f\n", measure_euler[VEC3_X] * RAD_TO_DEG, measure_euler[VEC3_Y] * RAD_TO_DEG, measure_euler[VEC3_Z] * RAD_TO_DEG); 
#endif
		eulerToQuaternion(measure_euler, quat);
		kalman_ekf->state_estimate->me[0][0] = quat[QUAT_W];
		kalman_ekf->state_estimate->me[1][0] = quat[QUAT_X];
		kalman_ekf->state_estimate->me[2][0] = quat[QUAT_Y];
		kalman_ekf->state_estimate->me[3][0] = quat[QUAT_Z];
		return -1;
	}

#ifdef KALMAN_DEBUG
	fprintf(stdout, "\n");
	fprintf(stdout, "measure is : \n");
	m_output(kalman_ekf->measure);
#endif

	predict_ekf(kalman_ekf);

	update_ekf(kalman_ekf);
	
#ifdef KALMAN_DEBUG
	fprintf(stdout, "covariance_estimate is :\n");
	m_output(kalman_ekf->covariance_estimate);
#endif

	quat[QUAT_W] = kalman_ekf->state_estimate->me[0][0];
	quat[QUAT_X] = kalman_ekf->state_estimate->me[1][0];
	quat[QUAT_Y] = kalman_ekf->state_estimate->me[2][0];
	quat[QUAT_Z] = kalman_ekf->state_estimate->me[3][0];
	quaternionToEuler(quat,mpu->fusedEuler);
	mpu->fusedEuler[VEC3_Y] = -mpu->fusedEuler[VEC3_Y];
	mpu->fusedEuler[VEC3_Z] = -mpu->fusedEuler[VEC3_Z];
	//the range of mpu->fuseEuler is -PI to PI
	if (mpu->fusedEuler[VEC3_Z] < 0)
	{
		mpu->fusedEuler[VEC3_Z] += 2 * PI;
	}
	return 0;

}


/**kalman filter apply in data fusion to estimate yaw
 * @param  mpu handler
 * @return     0:success
 */
int data_fusion_kalman(mpudata_t *mpu)
{
	static quaternion_t dmp_quat;
	static vector3d_t dmp_euler;
	static uint64_t last_mag_time;
	static bool mag_time_init = false;

	dmp_quat[0] = (float) mpu->rawQuat[0];
	dmp_quat[1] = (float) mpu->rawQuat[1];
	dmp_quat[2] = (float) mpu->rawQuat[2];
	dmp_quat[3] = (float) mpu->rawQuat[3];

	quaternionNormalize(dmp_quat);
	quaternionToEuler(dmp_quat, dmp_euler);

	mpu->fusedEuler[VEC3_X] = dmp_euler[VEC3_X];
	mpu->fusedEuler[VEC3_Y] = -dmp_euler[VEC3_Y];
	//mpu->fusedEuler[VEC3_Z] = -dmp_euler[VEC3_Z];

	static double hn_y, hn_x, mag_angle;
	static double mag[3];
	mag[VEC3_X] = mpu->calibratedMag[VEC3_X];
	mag[VEC3_Y] = mpu->calibratedMag[VEC3_Y];
	mag[VEC3_Z] = mpu->calibratedMag[VEC3_Z];

	hn_y = mag[VEC3_Y] * cos(dmp_euler[VEC3_X]) - mag[VEC3_Z] * sin(dmp_euler[VEC3_X]);
	hn_x = mag[VEC3_X] * cos(dmp_euler[VEC3_Y])
			+ mag[VEC3_Y] * sin(dmp_euler[VEC3_X]) * sin(dmp_euler[VEC3_Y])
			+ mag[VEC3_Z] * cos(dmp_euler[VEC3_X]) * sin(dmp_euler[VEC3_Y]);
	mag_angle = (double) atan2(-hn_y, hn_x);
	// change the range of mag_angle into 0 - 2 * PI,it is necessary
	if(mag_angle < 0)
	{
		mag_angle += 2 * PI;
	}

	static double delt_t = 0.0;
	if (mag_time_init)
	{
		delt_t = (double)(mpu->magTimestamp - last_mag_time);
		last_mag_time = mpu->magTimestamp;
		//fprintf(stdout, "delt_t is %f\n", delt_t * 1e-3);
		set_matrix(f_yaw->state_transition,
						1.0,  	-delt_t * 1e-3,
						0.0, 		1.0);
		set_matrix(f_yaw->control_input_model, delt_t * 1e-3, 0.0);
	}
	else
	{
		last_mag_time = mpu->magTimestamp;
		mag_time_init = true ;
	}

	set_matrix(f_yaw->measure,  mag_angle);

#ifdef KALMAN_DEBUG
	fprintf(stdout, "\n");
	fprintf(stdout, "measure is : \n");
	m_output(f_yaw->measure);
#endif

	set_matrix(f_yaw->control_input, (double)mpu->rawGyro[VEC3_Z] / 16.4 / 180 * PI);

#ifdef KALMAN_DEBUG
	fprintf(stdout, "\n");
	fprintf(stdout, "control_input is \n");
	m_output(f_yaw->control_input);
#endif
	predict(f_yaw);
	update(f_yaw);

#ifdef KALMAN_DEBUG
	fprintf(stdout, "covariance_estimate is :\n");
	m_output(f_yaw->covariance_estimate);
#endif
	//the range of mpu->fuseEuler is -PI to PI
	if (f_yaw->state_estimate->me[0][0] >=  PI)
	{
		mpu->fusedEuler[VEC3_Z] = f_yaw->state_estimate->me[0][0] - 2 * PI;
	}
	else
	{
		mpu->fusedEuler[VEC3_Z] = f_yaw->state_estimate->me[0][0];
	}

	//}

	return 0;
}



/* These next two functions convert the orientation matrix (see
 * gyro_orientation) to a scalar representation for use by the DMP.
 * NOTE: These functions are borrowed from InvenSense's MPL.
 */
unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx)
{
    unsigned short scalar;
    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */
    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;
    return scalar;
}

