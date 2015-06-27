#ifndef MY_TIMER_H
#define MY_TIMER_H

#include <stdbool.h>

//#include <signal.h>
extern bool send_system_state_now;
extern bool receive_now;
extern bool send_params_now;
extern bool send_gps_now;
extern bool send_imu_now;
extern bool read_imu_now;
extern bool read_laser_range_now;
extern bool read_gps_now;
extern bool update_current_mode;
extern bool set_servos_now;
extern bool navigate;
extern bool begin_control;

extern bool read_ultrasonic_now;

/**
 * set kinds of timer
 */
void timer_data_defaluts();

/**
 * this function is called by timer upon timer expiration
 */
void timer_update();

#endif
