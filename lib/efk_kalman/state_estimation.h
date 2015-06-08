#ifndef STATE_ESTIMATION_H
#define STATE_ESTIMATION_H

typedef enum
{
	FILTERRESULT_UNINITIALISED = -1,
	FILTERRESULT_OK = 0,
	FILTERRESULT_WARNING  = 1,
	FILTERRESULT_CRITICAL = 2,
	FILTERRESULT_ERROR = 3,
} filter_result_t;


typedef enum
{
	SENSOR_UPDATE_gyro = 1 << 0,
	SENSOR_UPDATE_accel = 1 << 1,
	SENSOR_UPDATE_mag = 1 << 2,
	SENSOR_UPDATE_attitude = 1 << 3,
	SENSOR_UPDATE_pos = 1 << 4,
	SENSOR_UPDATE_vel = 1 << 5,
	SENSOR_UPDATE_baro = 1 << 6,
} sensor_updates_t;

#define MAG_STATUS_OK 1
#define MAG_STATUS_AUX 2
#define MAG_STATUS_INVALID 0

typedef struct
{
	float gyro[3];
	float accel[3];
	float mag[3];
	float attitude[4];
	float pos[3];
	float vel[3];
	float baro[1];
	sensor_updates_t updated;
} state_estimation_t;

typedef struct state_filter_struct
{
	int32_t (*init) (struct state_filter_struct *self);
	filter_result_t (*filter)(struct state_filter_struct *self, state_estimation_t *state);
	void *local_data;
} state_filter_t;

#endif
