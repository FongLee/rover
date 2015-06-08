#ifndef AP_AHRS_H
#define AP_AHRS_H
#include "mpu9150.h"
//#include "location.h"

int update_ahrs(mpudata_t *mpu);

/**
 * update duler angle in centi_degree values
 */
// void update_euler_angles();

// int get_position(struct location *loc);

// void set_home(struct location *loc);

// int ahrs_healthy(void);

// void update_ahrs_ekf();

#endif
