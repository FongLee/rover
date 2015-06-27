#ifndef AP_ULTRASONIC_H
#define AP_ULTRASONIC_H

#include "uart_driver.h"
#include "stdio.h"
#include "errno.h"
#include "string.h"
//float distance;
//FILE *fp_dis;


int ultrasonic_init();
int ultrasonic_read(float *distance);
int ultrasonic_close();


#endif
