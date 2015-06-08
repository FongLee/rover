#ifndef SETTINGS_H
#define SETTINGS_H

#include <pthread.h>


#define ONBOARD_PARAM_NAME_LENGTH 15

typedef enum
{
	READ_ONLY = 0,
	READ_WRITE = 1,
}para_access_t;

enum
{
	PARAM_SYSTEM_ID = 0,
	PARAM_COMPONENT_ID,
	PARAM_SYSTEM_TYPE,
	PARAM_AUTOPILOT_TYPE,
	PARAM_SW_VERSION,
	PARAM_SYSTEM_SEND_STATE,
	ONBOARD_PARAM_COUNT

} global_param_id;

struct global_struct
{
	float param[ONBOARD_PARAM_COUNT];
	char param_name[ONBOARD_PARAM_COUNT][ONBOARD_PARAM_NAME_LENGTH];
	para_access_t param_access[ONBOARD_PARAM_COUNT];
};

struct global_struct global_data;


extern pthread_mutex_t lock_read_imu;
extern pthread_cond_t ready_read_imu;


void global_data_reset_param_defaults(void);
//void global_data_reset(void);

#endif
