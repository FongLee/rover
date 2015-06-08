#include <string.h>


#include "mavlink.h"
#include "settings.h"

pthread_mutex_t lock_read_imu = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t ready_read_imu = PTHREAD_COND_INITIALIZER;


void global_data_reset_param_defaults(void)
{
	global_data.param[PARAM_SYSTEM_ID] = 81;
	strcpy(global_data.param_name[PARAM_SYSTEM_ID], "SYS_ID");
	global_data.param_access[PARAM_SYSTEM_ID] = READ_WRITE;

	global_data.param[PARAM_COMPONENT_ID] = 50;
	strcpy(global_data.param_name[PARAM_COMPONENT_ID], "SYS_COMP_ID");
	global_data.param_access[PARAM_COMPONENT_ID] = READ_WRITE;

	global_data.param[PARAM_AUTOPILOT_TYPE] = MAV_AUTOPILOT_GENERIC;
	strcpy(global_data.param_name[PARAM_AUTOPILOT_TYPE], "SYS_AP_TYPE");
	global_data.param_access[PARAM_AUTOPILOT_TYPE] = READ_WRITE;

	global_data.param[PARAM_SW_VERSION] = 1000;
	strcpy(global_data.param_name[PARAM_SW_VERSION], "SYS_SW_VER");
	global_data.param_access[PARAM_SW_VERSION] = READ_WRITE;

	global_data.param[PARAM_SYSTEM_SEND_STATE] = 1;
	strcpy(global_data.param_name[PARAM_SYSTEM_SEND_STATE], "SYS_SEND_STATE");
	global_data.param_access[PARAM_SYSTEM_SEND_STATE] = READ_WRITE;

}
