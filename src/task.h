#ifndef TASK_H
#define TASK_H

/**
 * transfer task
 * @return 0: success; -1: error
 */
void *task_transfer();

/**
 * read imu task
 * @return 0: success; -1: error
 */
void *task_read_imu();

/**
 * read gps task
 * @return 0: success; -1: error
 */
void *task_read_gps();

/**
 * control throttle and steer task
 * @return 0: success; -1: error
 */
void *task_control();

/**
 * read low sensor task including GPS and ultrasonic distance measurement
 * @return 0: success; -1: error
 */
void *task_read_lowsensor();

/**
 * read video task
 * @return [description]
 */
void *task_camera();

/**
 *  ultrasonic distance measure task
 * @return 0: success; -1: error
 */
void *task_read_ultrasonic();

#endif
