#ifndef TASK_H
#define TASK_H



void *task_transfer();
void *task_read_imu();
void *task_read_gps();
void *task_control();
void *read_lowsensor();

void *task_camera();
void *task_read_ultrasonic();

#endif
