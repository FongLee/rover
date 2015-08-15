#ifndef LINUX_GLUE_H
#define LINUX_GLUE_H

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "inv_mpu.h"
#include "scheduler.h"

#define MIN_I2C_BUS 0
#define MAX_I2C_BUS 7

static inline int reg_int_cb(struct int_param_s *int_param)
{
	return 0;
}

#define i2c_write	linux_i2c_write
#define i2c_read	linux_i2c_read
#define delay_ms	linux_delay_ms
#define get_ms		linux_get_ms
#define log_i		printf
#define log_e		printf
#define min(a, b) 	((a < b) ? a : b)

void __no_operation(void);

/**
 * set i2c device
 * @param bus number of i2c device
 */
void linux_set_i2c_bus(int bus);

/**
 * write data to i2c device
 * @param  slave_addr address of slave
 * @param  reg_addr   address of register
 * @param  length     length of data
 * @param  data       data
 * @return            0: success; -1: error
 */
int linux_i2c_write(unsigned char slave_addr, unsigned char reg_addr,
       unsigned char length, unsigned char const *data);

/**
 * read data from i2c device
 * @param  slave_addr address of slave
 * @param  reg_addr   address of register
 * @param  length     length of data
 * @param  data       data
 * @return            0: success; -1: error
 */
int linux_i2c_read(unsigned char slave_addr, unsigned char reg_addr,
       unsigned char length, unsigned char *data);


#endif

