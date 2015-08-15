#include <stdio.h>
#include <signal.h>
#include <time.h>
#include <errno.h>
#include <string.h>
#include <sys/time.h>
#include "scheduler.h"

struct timeval start_time;
struct timespec start_ntime;

/**
 * scheduler initialization
 * @return 0: success
 */
int scheduler_init()
{
	gettimeofday(&start_time, NULL);
	clock_gettime(CLOCK_MONOTONIC, &start_ntime);

	return 0;
}

/**
 * start scheduler
 */
int scheduler_begin(void timer_update(union sigval v))
{
	struct sigevent evp;
	struct itimerspec ts;
	timer_t timer;
	int ret;

	memset(&evp, 0, sizeof(evp));
	evp.sigev_value.sival_ptr = &timer;
	evp.sigev_value.sival_int = 3;
	evp.sigev_notify = SIGEV_THREAD;
	evp.sigev_notify_function = timer_update;
	ret = timer_create(CLOCK_MONOTONIC, &evp, &timer);
	if (ret) {
		fprintf(stderr, "\ntimer_create:%s\n", strerror(errno));
		return -1;
	}
	ts.it_interval.tv_sec = 0;
	ts.it_interval.tv_nsec  = 1000000;
	ts.it_value.tv_sec = 0;
	ts.it_value.tv_nsec  = 1000000;

	ret = timer_settime(timer, TIMER_ABSTIME, &ts, NULL);
	if (ret) {
		fprintf(stderr, "\ntimer_settime err:%s\n", strerror(errno));
		return -1;
	}
	return 0;
}

/**
 * get  elapsed time(ms)
 * @param  count elapsed time of return value
 * @return       0: success; -1: error
 */
int linux_get_ms(uint64_t *count)
{
	static struct timespec t;
	if (!count)
		return -1;
	if (clock_gettime(CLOCK_MONOTONIC, &t) < 0) {
		//perror("gettimeofday");
		fprintf(stderr, "\nclock_gettime err:%s\n", strerror(errno));
		return -1;
	}

	*count = (uint64_t)(t.tv_sec * 1e3 - start_ntime.tv_sec * 1e3) + (uint64_t)(t.tv_nsec / 1e6  - start_ntime.tv_nsec / 1e6);

	return 0;
}

/**
 * get  elapsed time(us)
 * @param  count elapsed time of return value
 * @return       0: success; -1: error
 */
int linux_get_us(uint64_t *count)
{
	struct timespec t;

	if (!count)
		return -1;

	if (clock_gettime(CLOCK_MONOTONIC, &t) < 0)
	{
		fprintf(stderr, "\nclock_gettime err:%s\n", strerror(errno));
		return -1;
	}

	*count = (uint64_t)(t.tv_sec * 1e6 - start_ntime.tv_sec * 1e6) + (uint64_t)(t.tv_nsec / 1e3  - start_ntime.tv_nsec / 1e3);

	return 0;
}

/**
 * get  elapsed time(ns)
 * @param  count elapsed time of return value
 * @return       0: success; -1: error
 */
int linux_get_ns(uint64_t *count)
{
	struct timespec t;

	if (!count)
		return -1;

	if (clock_gettime(CLOCK_MONOTONIC, &t) < 0)
	{
		fprintf(stderr, "\nclock_gettime err:%s\n", strerror(errno));
		return -1;
	}

	*count = (uint64_t)(t.tv_sec * 1e9 - start_ntime.tv_sec * 1e9) +  (uint64_t)(t.tv_nsec - start_ntime.tv_nsec);

	return 0;
}

/**
 * time delay (microseconds)
 * @param usec time count
 */
void linux_delay_microseconds(uint64_t usec)
{
	struct timespec  ts;
	ts.tv_sec = 0;
	ts.tv_nsec = usec * 1000UL;
	while (nanosleep(&ts, &ts) == -1);
}

/**
 * time delay (ms)
 * @param usec time count
 */
void linux_delay_ms(uint64_t num_ms)
{
	struct timespec  ts;
	ts.tv_sec = num_ms / 1000;
	ts.tv_nsec = (num_ms % 1000) * 1000000;
	while (nanosleep(&ts, &ts) == -1);
}
