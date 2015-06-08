#ifndef IMU_H
#define IMU_H

#include "mpu9150.h"

extern int done;

extern mpudata_t mpu;
extern int i2c_bus_imu;
extern int sample_rate;
extern int yaw_mix_factor;
extern bool flag_imu_init;

int mag_calibration();
int acc_calibration();
void print_raw_mag(mpudata_t *mpu);
void print_raw_accel(mpudata_t *mpu);
void print_fused_euler_angles(mpudata_t *mpu);
void print_fused_quaternions(mpudata_t *mpu);
void print_calibrated_accel(mpudata_t *mpu);
void print_calibrated_mag(mpudata_t *mpu);
int set_cal(int mag);
//void register_sig_handler();
void sigint_handler(int sig);


#endif
