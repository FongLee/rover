#include <stdio.h>
#include <signal.h>
#include <time.h>
#include <errno.h>
#include <string.h>
#include <sys/time.h>
#include "scheduler.h"

struct timeval start_time;
struct timespec start_ntime;
//volatile uint32_t boot_time_ms = 0;

//extern void timer_update(union sigval v);

int scheduler_init()
{
	gettimeofday(&start_time, NULL);
	clock_gettime(CLOCK_MONOTONIC, &start_ntime);
	//*start_time = (uint32_t)tp.tv_sec * 1000 + (uint32_t)tp.tv_usec * 0.001;
	return 0;
}

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
	/* reset/start timers */
	// timer[TIMER_SONAR] = SONAR_TIMER_COUNT;
	// timer[TIMER_SYSTEM_STATE] = SYSTEM_STATE_COUNT;
	// timer[TIMER_RECEIVE] = SYSTEM_STATE_COUNT / 2;
	// timer[TIMER_PARAMS] = PARAMS_COUNT;
	// timer[TIMER_IMAGE] = global_data.param[PARAM_VIDEO_RATE];
	//timer[3] = (unsigned int )100;

	ret = timer_settime(timer, TIMER_ABSTIME, &ts, NULL);
	if (ret) {
		fprintf(stderr, "\ntimer_settime err:%s\n", strerror(errno));
		return -1;
	}
	return 0;
}

// void timer_update(union sigval v)
// {
// 	//boot_time_ms++;
// }


// int get_ms(uint32_t *count)
// {
// 	struct timeval t;
// 	if (!count)
// 		return -1;
// 	if (gettimeofday(&t, NULL) < 0) {
// 		//perror("gettimeofday");
// 		fprintf(stderr, "\ngettimeofday err:%s\n", strerror(errno));
// 		return -1;
// 	}
// 	*count = (t.tv_sec * 1000) + (t.tv_usec / 1000) - (start_time.tv_sec * 1000) - (start_time.tv_usec / 1000);
// 	return 0;
// }

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
	//*count = (t.tv_sec * 1000) + (t.tv_usec / 1000) - (start_time.tv_sec * 1000) - (start_time.tv_usec / 1000);
	*count = (uint64_t)(t.tv_sec * 1e3 - start_ntime.tv_sec * 1e3) + (uint64_t)(t.tv_nsec / 1e6  - start_ntime.tv_nsec / 1e6);
	//*count = (t.tv_sec * 1e3) + (t.tv_nsec * 1e6);
	return 0;
}

int linux_get_us(uint64_t *count)
{
	struct timespec t;

	if (!count)
		return -1;

	if (clock_gettime(CLOCK_MONOTONIC, &t) < 0) {
		//perror("gettimeofday");
		fprintf(stderr, "\nclock_gettime err:%s\n", strerror(errno));
		return -1;
	}

	*count = (uint64_t)(t.tv_sec * 1e6 - start_ntime.tv_sec * 1e6) + (uint64_t)(t.tv_nsec / 1e3  - start_ntime.tv_nsec / 1e3);
	//*count = (t.tv_sec * 1e6) + (t.tv_nsec * 1e3);
	return 0;
}

int linux_get_ns(uint64_t *count)
{
	struct timespec t;

	if (!count)
		return -1;

	if (clock_gettime(CLOCK_MONOTONIC, &t) < 0) {
		//perror("gettimeofday");
		fprintf(stderr, "\nclock_gettime err:%s\n", strerror(errno));
		return -1;
	}

	*count = (uint64_t)(t.tv_sec * 1e9 - start_ntime.tv_sec * 1e9) +  (uint64_t)(t.tv_nsec - start_ntime.tv_nsec);
	//*count = (t.tv_sec * 1e6) + (t.tv_nsec * 1e3);
	return 0;
}

void delay_microseconds(uint64_t usec)
{
	struct timespec  ts;
	ts.tv_sec = 0;
	ts.tv_nsec = usec * 1000UL;
	while (nanosleep(&ts, &ts) == -1);
}

void linux_delay_ms(uint64_t num_ms)
{
	struct timespec  ts;
	ts.tv_sec = num_ms / 1000;
	ts.tv_nsec = (num_ms % 1000) * 1000000;
	while (nanosleep(&ts, &ts) == -1);
	//return nanosleep(&ts, NULL);
}
