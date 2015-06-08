#ifndef MPU_CALIBRATION_H
#define MPU_CALIBRATION_H

#include "matrix.h"
int imu_calibration(MAT *mea, VEC *cal_bias, VEC *cal_factor);

#endif
