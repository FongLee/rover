
#ifndef AP_NAVIGATION
#define AP_NAVIGATION

//#include <stdint.h>
#include <stdbool.h>
#include <nmea/nmea.h>
#include <pthread.h>

#include "mpu9150.h"
#include "matrix.h"
#include "kalman.h"
#include "ap_gps.h"

#define USE_MPU 		0
#define USE_GPS 		1


typedef struct
{
	VEC *w_nie;
	VEC *w_nen;
	VEC *w_nin;
	VEC *v_n;
	VEC *last_quaternion;
	VEC *last_velocity;
	VEC *velocity_nav;

	VEC *last_f_b;
	VEC *last_f_n;

	MAT *w_n_skew;
	MAT *last_c_bn;
	MAT *c_nb;

	VEC *delt_velocity;
	VEC *last_acce;
	uint64_t last_use_mpu_nav_time;
	bool flag_use_mpu_init;
	bool flag_nav_failure;
	VEC *origin;
	VEC *location;

	kalman_t *kalman_navigation;
	nmeaPOS home_position;
	nmeaPOS current_position;
	bool flag_get_home;
	double speed;

	pthread_mutex_t mutex;

}nav_data_t;

extern nav_data_t nav_data;


int navigation_init(nav_data_t * nav);

int navigation_close(nav_data_t * nav);

int inertial_navigation(nav_data_t * nav, mpudata_t * mpu, gps_data_t  *gps, int flag);

#endif
