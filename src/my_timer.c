//#include <stdbool.h>
#include <pthread.h>

#include "scheduler.h"
#include "my_timer.h"
#include "settings.h"

#define TIMER_DATA_LENGTH 15

bool send_system_state_now = false;
bool receive_now = false;
bool send_params_now = false;
bool send_gps_now = false;
bool send_imu_now = false;
bool read_imu_now = false;
bool read_laser_range_now = false;
bool read_gps_now = false;
bool update_current_mode = false;
bool set_servos_now = false;
bool navigate = false;
bool begin_control = false;

enum
{
	TIMER_SYSTEM_STATE,
	TIMER_RECEIVE,
	TIMER_SEND_PARAMS,
	TIMER_READ_IMU,
	TIMER_READ_GPS,
	TIMER_READ_LR,

	NTIMERS
} timer_state;

enum
{
	SYSTEM_STATE_COUNT = 1000,
	RECEIVE_COUNT = 50,
	PARAMS_COUNT = 100,
	IMU_COUNT = 10,
	GPS_COUNT = 100,
	LR_COUNT = 1000
} timer_count;

struct
{
	int timer[NTIMERS];
	char timer_name[NTIMERS][TIMER_DATA_LENGTH];

} timer_data;


void timer_data_defaluts()
{
	timer_data.timer[TIMER_SYSTEM_STATE] = SYSTEM_STATE_COUNT;

	timer_data.timer[TIMER_RECEIVE] = RECEIVE_COUNT;

	timer_data.timer[TIMER_SEND_PARAMS] = PARAMS_COUNT;

	timer_data.timer[TIMER_READ_IMU] = IMU_COUNT;

	timer_data.timer[TIMER_READ_GPS] = GPS_COUNT;

	timer_data.timer[TIMER_READ_LR] = LR_COUNT;

}

void timer_update()
{
	unsigned i = 0;
	for ( i = 0; i < NTIMERS; i++)
	{
		if (timer_data.timer[i] > 0)
			timer_data.timer[i]--;
	}
	if (timer_data.timer[TIMER_SYSTEM_STATE] == 0)
	{
		send_system_state_now = true;
		timer_data.timer[TIMER_SYSTEM_STATE] = SYSTEM_STATE_COUNT;
	}
	if (timer_data.timer[TIMER_RECEIVE] == 0)
	{
		receive_now = true;
		timer_data.timer[TIMER_RECEIVE] = RECEIVE_COUNT;
	}
	if (timer_data.timer[TIMER_SEND_PARAMS] == 0)
	{
		send_params_now = true;
		timer_data.timer[TIMER_SEND_PARAMS] =PARAMS_COUNT;
	}

	if (timer_data.timer[TIMER_READ_IMU] == 0)
	{
		//pthread_mutex_lock(&lock_read_imu);
		read_imu_now = true;
		//pthread_mutex_unlock(&lock_read_imu);
		//pthread_cond_signal(&ready_read_imu);
		
		timer_data.timer[TIMER_READ_IMU] = IMU_COUNT;
	}

	if (timer_data.timer[TIMER_READ_GPS] == 0)
	{
		read_gps_now = true;
		timer_data.timer[TIMER_READ_GPS] = GPS_COUNT;
	}

	if (timer_data.timer[TIMER_READ_LR] == 0)
	{
		read_laser_range_now = true;
		timer_data.timer[TIMER_READ_LR] = LR_COUNT;
	}
}
