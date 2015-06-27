#ifndef SCHEDULER_H
#define SCHEDULER_H
#include <stdint.h>

#define delay_us    linux_delay_microseconds
#define delay_ms	linux_delay_ms
#define get_ms		linux_get_ms
#define get_us 		linux_get_us
#define get_ns 		linux_get_ns
//extern void timer_update(union sigval v);
int scheduler_init();
int scheduler_begin();
int linux_get_ms(uint64_t *count);
int linux_get_us(uint64_t *count);
void linux_delay_microseconds(uint64_t usec);
void linux_delay_ms(uint64_t num_ms);
int linux_get_ns(uint64_t *count);


#endif
