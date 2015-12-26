#include <stdio.h>
#include <stdbool.h>
#include "ap_ahrs.h"
#include "ap_math.h"
#include "mpu9150.h"
#include "quaternion.h"
#include "MahonyAHRS.h"

#define GRO_FACTOR 16.38

bool flag_ahrs_init = false;

static int ahrs_init(mpudata_t *mpu);

int update_ahrs(mpudata_t *mpu)
{
	static float tmp_norm;
	static float ax, ay, az;
	ax = (float)mpu->calibratedAccel[VEC3_X];
	ay = (float)mpu->calibratedAccel[VEC3_Y];
	az = (float)mpu->calibratedAccel[VEC3_Z];

	static float mx, my, mz;
	mx = mpu->calibratedMag[VEC3_X];
	my = mpu->calibratedMag[VEC3_Y];
	mz = mpu->calibratedMag[VEC3_Z];
	if (flag_ahrs_init == false)
	{
		ahrs_init(mpu);
	}

	// fprintf(stdout, "\n");
	// fprintf(stdout, "before fusedEuler x is %f\n", mpu->fusedEuler[0] * RAD_TO_DEG);
	// fprintf(stdout, "before fusedEuler y is %f\n", mpu->fusedEuler[1] * RAD_TO_DEG);
	// fprintf(stdout, "before fusedEuler z is %f\n", mpu->fusedEuler[2] * RAD_TO_DEG);

	// fprintf(stdout, "\n");
	// fprintf(stdout, "raw q0 is %f\n", q0);
	// fprintf(stdout, "raw q1 is %f\n", q1);
	// fprintf(stdout, "raw q2 is %f\n", q2);
	// fprintf(stdout, "raw q3 is %f\n", q3);

	static float gx, gy, gz;
	gx = mpu->rawGyro[VEC3_X] / GRO_FACTOR * DEG_TO_RAD;
	gy = mpu->rawGyro[VEC3_Y] / GRO_FACTOR * DEG_TO_RAD;
	gz = mpu->rawGyro[VEC3_Z] / GRO_FACTOR * DEG_TO_RAD;

	MahonyAHRSupdate(gx, gy, gz, ax, ay, az, mx, my, mz);

	mpu->fusedQuat[0] = q0;
	mpu->fusedQuat[1] = q1;
	mpu->fusedQuat[2] = q2;
	mpu->fusedQuat[3] = q3;

	// fprintf(stdout, "\n");
	// fprintf(stdout, "q0 is %f\n", q0);
	// fprintf(stdout, "q1 is %f\n", q1);
	// fprintf(stdout, "q2 is %f\n", q2);
	// fprintf(stdout, "q3 is %f\n", q3);
	quaternionToEuler(mpu->fusedQuat,mpu->fusedEuler);

	//change to Front-Right-Down body coordinate system
	mpu->fusedEuler[VEC3_Y] = -mpu->fusedEuler[VEC3_Y];
	mpu->fusedEuler[VEC3_Z] = -mpu->fusedEuler[VEC3_Z];
	
	if (mpu->fusedEuler[VEC3_Z] < 0)
	{
		mpu->fusedEuler[VEC3_Z] += 2 * PI;
	}
	return 0;
}

int ahrs_init(mpudata_t *mpu)
{
	float init_Roll, init_Pitch, init_Yaw;

	float ax, ay, az;
	ax = (float)mpu->calibratedAccel[0];
	ay = (float)mpu->calibratedAccel[1];
	az = (float)mpu->calibratedAccel[2];

	float mx, my, mz;
	mx = mpu->calibratedMag[0];
	my = mpu->calibratedMag[1];
	mz = mpu->calibratedMag[2];

	float tmp_norm;
	tmp_norm = inv_sqrt(ax * ax + ay * ay + az * az);
	ax *= tmp_norm;
	ay *= tmp_norm;
	az *= tmp_norm;

	init_Roll  =  atan2(ay, az);
	init_Pitch = -asin(ax);              //init_Pitch = asin(ax / 1);
	float hy, hx;
	hy = my*cos(init_Pitch) - mz*sin(init_Pitch);
	hx = mx*cos(init_Roll) + my*sin(init_Roll)*sin(init_Pitch) + mz*sin(init_Roll)*cos(init_Pitch);
	init_Yaw   = atan2(-hy,hx);                       //atan2(-my, mx);

#ifdef AHRS_DEBUG
	fprintf(stdout, "\n");
	fprintf(stdout, "roll is %f degree\n", init_Roll * RAD_TO_DEG);
	fprintf(stdout, "pitch is %f degree\n", init_Pitch* RAD_TO_DEG);
	fprintf(stdout, "yaw is %f degree\n", init_Yaw* RAD_TO_DEG);
#endif

	q0 = cos(0.5*init_Roll)*cos(0.5*init_Pitch)*cos(0.5*init_Yaw) + sin(0.5*init_Roll)*sin(0.5*init_Pitch)*sin(0.5*init_Yaw);  //w
	q1 = sin(0.5*init_Roll)*cos(0.5*init_Pitch)*cos(0.5*init_Yaw) - cos(0.5*init_Roll)*sin(0.5*init_Pitch)*sin(0.5*init_Yaw);  //x   绕x轴旋转是roll
	q2 = cos(0.5*init_Roll)*sin(0.5*init_Pitch)*cos(0.5*init_Yaw) + sin(0.5*init_Roll)*cos(0.5*init_Pitch)*sin(0.5*init_Yaw);  //y   绕y轴旋转是pitch
	q3 = cos(0.5*init_Roll)*cos(0.5*init_Pitch)*sin(0.5*init_Yaw) - sin(0.5*init_Roll)*sin(0.5*init_Pitch)*cos(0.5*init_Yaw);  //z   绕z轴旋转是Yaw
	//Yaw = init_Yaw * 57.3;

	tmp_norm = inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= tmp_norm;
	q1 *= tmp_norm;
	q2 *= tmp_norm;
	q3 *= tmp_norm;
	flag_ahrs_init = true;

	return 0;
}
