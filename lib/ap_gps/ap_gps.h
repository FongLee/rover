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
int gps_init();
void gps_end();
void gps_parse();
int  gps_quality();
float ground_speed();
int gps_op_mode();
long ground_speed_cm();
long ground_course_cd();
int num_sats();
void ground_location(struct location *loc);
void check_position();
void fill_3d_velocity(VEC *v);

#endif