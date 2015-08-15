#ifndef SCHEDULER_H
#define SCHEDULER_H
#include <stdint.h>

#define delay_us    linux_delay_microseconds
#define delay_ms	linux_delay_ms
#define get_ms		linux_get_ms
#define get_us 		linux_get_us
#define get_ns 		linux_get_ns

/**
 * scheduler initialization
 * @return 0: success
 */
int scheduler_init();

/**
 * start scheduler
 */
int scheduler_begin();

/**
 * get  elapsed time(ms)
 * @param  count elapsed time of return value
 * @return       0: success; -1: error
 */
int linux_get_ms(uint64_t *count);

/**
 * get  elapsed time(us)
 * @param  count elapsed time of return value
 * @return       0: success; -1: error
 */
int linux_get_us(uint64_t *count);

/**
 * time delay (microseconds)
 * @param usec time count
 */
void linux_delay_microseconds(uint64_t usec);

/**
 * time delay (ms)
 * @param usec time count
 */
void linux_delay_ms(uint64_t num_ms);

/**
 * get  elapsed time(ns)
 * @param  count elapsed time of return value
 * @return       0: success; -1: error
 */
int linux_get_ns(uint64_t *count);


#endif
