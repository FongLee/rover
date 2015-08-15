#ifndef AP_GPS_H
#define AP_GPS_H
#include <stdint.h>
#include <stdbool.h>
#include <nmea/nmea.h>
#include "ap_location.h"
#include "matrix.h"

extern nmeaINFO info;
//parameter
extern float radius_cm;
extern float accel_max_cmss;
extern bool flag_gps_init ;
extern struct location gps_loc;
extern unsigned long last_good_update;
extern unsigned long last_good_lat;
extern unsigned long last_good_lon;
extern VEC *last_good_vel;
extern uint64_t gps_timestamp;
extern struct location gps_loc;

/**
 * gps initialization
 * @return 0: success; -1: err
 */
int gps_init();

/**
 * close gps
 */
void gps_end();

/**
 * parse the data of gps
 * @return 0: success; -1: err
 */
int  gps_parse();

/**
 * get quality gps (0 = Invalid; 1 = Fix; 2 = Differential,
 * 					3 = Sensitive)
 * @return  quality indicator of gps
 */
int  gps_quality();

/**
 * get speed of obeject in m/s
 * @return speed of obeject
 */
float ground_speed();

/**
 * get mode of gps
 * @return mode of gps
 */
int gps_op_mode();

/**
 * get speed of obeject in cm/s
 * @return speed of obeject
 */
long ground_speed_cm();

/**
 * get ground course in centidegrees
 * @return ground course
 */
long ground_course_cd();

/**
 * get satalite of gps
 * @return number of satellite in use
 */
int num_sats();

/**
 * get location of gps
 * @param loc location to return
 */
void ground_location(struct location *loc);

/**
 * get velocity in 3d of object
 * @param v speed of vector
 */
void check_position();

/**
 * check positon of object
 */
void fill_3d_velocity(VEC *v);

#endif
