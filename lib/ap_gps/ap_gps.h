#ifndef AP_GPS_H
#define AP_GPS_H
#include <stdint.h>
#include <stdbool.h>
#include <nmea/nmea.h>
#include "ap_location.h"
#include "matrix.h"

typedef struct
{
	nmeaINFO info;
	nmeaINFO prior_info;
	nmeaPARSER parser;
	nmeaPOS position;

	VEC *velocity_body;

	int fd_gps;
	uint64_t gps_timestamp;

	bool flag_get_first_data;
	bool flag_gps_glitching;
	bool flag_gps_init ;

} gps_data_t;

gps_data_t gps_data;


/**
 * gps initialization
 * @return 0: success; -1: err
 */
int gps_init(gps_data_t *gps_data);

/**
 * close gps
 */
void gps_end(gps_data_t *gps_data);

/**
 * parse the data of gps
 * @return 0: success; -1: err
 */
int  gps_parse(gps_data_t *gps_data);

/**
 * get quality gps (0 = Invalid; 1 = Fix; 2 = Differential,
 * 					3 = Sensitive)
 * @return  quality indicator of gps
 */
int  gps_quality(gps_data_t *gps_data);

/**
 * get speed of obeject in m/s
 * @return speed of obeject
 */
int gps_op_mode(gps_data_t *gps_data);

/**
 * get mode of gps
 * @return mode of gps
 */
double ground_speed(gps_data_t *gps_data);

/**
 * get speed of obeject in cm/s
 * @return speed of obeject
 */
long ground_speed_cm(gps_data_t *gps_data);

/**
 * get ground course in centidegrees
 * @return ground course
 */
long ground_course_cd(gps_data_t *gps_data);

/**
 * get satalite of gps
 * @return number of satellite in use
 */
int num_sats(gps_data_t *gps_data);


int get_3d_velocity(gps_data_t *gps_data, MAT * c_bn, VEC *velocity_nav);

void check_gps(gps_data_t *gps_data, nmeaPOS *prior_location, uint64_t last_gps_timestamp);

#endif
