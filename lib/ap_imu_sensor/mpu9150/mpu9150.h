
#ifndef MPU9150_H
#define MPU9150_H

#include <stdint.h>

#include "quaternion.h"

#define MAG_SENSOR_RANGE 	4096
#define ACCEL_SENSOR_RANGE 	32000

// Somewhat arbitrary limits here. The values are samples per second.
// The MIN comes from the way we are timing our loop in imu and imucal.
// That's easily worked around, but no one probably cares.
// The MAX comes from the compass. This could be avoided with separate
// sample rates for the compass and the accel/gyros which can handle
// faster sampling rates. This is a TODO item to see if it's useful.
// There are some practical limits on the speed that come from a 'userland'
// implementation like this as opposed to a kernel or 'bare-metal' driver.
#define MIN_SAMPLE_RATE 2
#define MAX_SAMPLE_RATE 100

typedef struct {
	float bias[3];
	float scale[3];

} caldata_t;

typedef struct
{
	float bias[3];
	float scale[3];
}mag_caldata_t;

typedef struct {
	short rawGyro[3];
	short rawAccel[3];
	long rawQuat[4];
	//unsigned long dmpTimestamp;
	uint64_t dmpTimestamp;
	short rawMag[3];
	//unsigned long magTimestamp;
	uint64_t  magTimestamp;
	//short calibratedAccel[3];
	//short calibratedMag[3];
	float calibratedAccel[3];
	float calibratedMag[3];

	quaternion_t fusedQuat;
	vector3d_t fusedEuler;

	float lastDMPYaw;
	float lastYaw;
} mpudata_t;


void mpu9150_set_debug(int on);
int mpu9150_init(int i2c_bus, int sample_rate, int yaw_mixing_factor);
void mpu9150_exit();
int mpu9150_read(mpudata_t *mpu);
int mpu9150_read_dmp(mpudata_t *mpu);
int mpu9150_read_mag(mpudata_t *mpu);
void mpu9150_set_accel_cal(caldata_t *cal);
//void mpu9150_set_mag_cal(caldata_t *cal);
void mpu9150_set_mag_cal(caldata_t *cal);

#endif /* MPU9150_H */

