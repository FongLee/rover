
#include <stdbool.h>

#include "quaternion.h"
#include "ap_math.h"

#define DT_INIT 1.0f/50

#define IMPORT_SENSOR_IF_UPDATED(shortname, num) \
	if (IS_SET(state->updated, SENSOR_UPDATE_##shortname)) { \
		uint8_t t; \
		for (t = 0; t < num; t++) { \
			this->work.shortname[t] = state->shortname[t]; \
			} \
		}

struct
{
	float roll;
	float pitch;
	float yaw;

} attitude_state;

struct data
{
    bool usepos;
    int32_t init_stage;
    state_estimation work;
    bool inited;
};

static bool initialized = 0;

static filter_result filter(state_filter *self, state_estimation *state)
{
    struct data *this = (struct data *)self->local_data;
    const float zeros[3] = {0.0f, 0.0f, 0.0f};

    float dt = DT_INIT ;
    uint16_t sensors = 0;

    this->work.updated |= state->updated;

    IMPORT_SENSOR_IF_UPDATED(gyro, 3);
    IMPORT_SENSOR_IF_UPDATED(accel, 3);
    IMPORT_SENSOR_IF_UPDATED(mag, 3);
    IMPORT_SENSOR_IF_UPDATED(baro, 1);
    IMPORT_SENSOR_IF_UPDATED(pos, 3);
    IMPORT_SENSOR_IF_UPDATED(vel, 3);

   	if (!(IS_SET(this->work.updated, SENSOR_UPDATE_accel) && IS_SET(this->work.updated, SENSOR_UPDATE_gyro)))
   	{
   		UNSET_MASK(state->updated, SENSOR_UPDATE_pos);
   		UNSET_MASK(state->updated, SENSOR_UPDATA_vel);
   		UNSET_MASK(state->updated, SENSOR_UPDATE_attitude);
   		UNSET_MASK(state->updated, SENSOR_UPDATE_gyro);
   		return FILTERRESULT_OK;
   	}

   	if (!this->inited && IS_SET(this->work.updated, SENSOR_UPDATE_mag) && IS_SET(this->work.update, SENSOR_UPDATE_pos))
   	{
   		if (this->init_stage == 0)
   		{
	   		ins_gps_init();
	   		float ax, ay, az;
	   		float mx, my, mz;
	   		float tmp_norm;

	   		ax = this->work.accel[0];
	   		ay = this->work.accel[1];
	   		az = this->work.accel[2];
	   		tmp_norm = inv_sqrt(ax * ax + ay * ay + az * az);
	   		ax *= tmp_norm;
	   		ay *= tmp_norm;
	   		az *= tmp_norm;

	   		mx = this->work.mag[0];
	   		my = this->work.mag[1];
	   		mz = this->work.mag[2];
	   		tmp_norm = inv_sqrt(mx * mx + my * my + mz * mz);
	   		mx *= tmp_norm;
	   		my *= tmp_norm;
	   		mz *= tmp_norm;
	   		attitude_state.roll = atan2(ay, az);
	   		attitude_state.pitch = -asin(ax);

	   		float hy, hx;
			hy = my*cos(attitude_state.pitch) - mz*sin(attitude_state.pitch);
			hx = mx*cos(attitude_state.roll) + my*sin(attitude_state.roll)*sin(attitude_state.pitch) + mz*sin(attitude_state.roll)*cos(attitude_state.pitch);
			attitude_state.yaw   = atan2(-hy,hx);
			eulerToQuaternion(&attitude_state.roll, this->work.attitude);

			ins_set_state(this->work.pos, (float *)zeros, this->work.attitude, (float *)zeros, (float *)zeros);
   		}
	   	else
	   	{
	   		float gyro[3] =
	   		{
	   			this->work.gyro[0] * DEG_TO_RAD, this->work.gyro[1] * DEG_TO_RAD, this->work.gyro[2]
	   		};
	   		ins_state_prediction(gyro, this->work.accel, dt);
	   		state->attitude[0] = nav.q[0];
	   		state->attitude[1] = nav.q[1];
	   		state->attitude[2] = nav.q[2];
	   		state->attitude[3] = nav.q[3];
	   		state->gyro[0] -= nav.gyro_bias[0] * RAD_TO_DEG;
	   		state->gyro[1] -= nav.gyro_bias[1] * RAD_TO_DEG;
	   		state->gyro[2] -= nav.gyro_bias[2] * RAD_TO_DEG;
	   		state->pos[0] = nav.pos[0];
	   		state->pos[1] = nav.pos[1];
	   		state->pos[2] = nav.pos[2];
	   		state->vel[0] = nav.vel[0];
	   		state->vel[1] = nav.vel[1];
	   		state->vel[2] = nav.vel[2];
	   		state->updated |= SENSOR_UPDATE_attitude | SENSOR_UPDATE_pos | SENSOR_UPDATE_vel;
	   	}
   		this->init_stage ++;
   		if (this->init_stage > 10)
   		{
   			this->inited = true;
   		}

   		return FILTERRESULT_OK;
   	}
	float gyro[3] =
	{
		this->work.gyro[0] * DEG_TO_RAD, this->work.gyro[1] * DEG_TO_RAD, this->work.gyro[2]
	};
	ins_state_prediction(gyro, this->work.accel, dt);
	state->attitude[0] = nav.q[0];
	state->attitude[1] = nav.q[1];
	state->attitude[2] = nav.q[2];
	state->attitude[3] = nav.q[3];
	state->gyro[0] -= nav.gyro_bias[0] * RAD_TO_DEG;
	state->gyro[1] -= nav.gyro_bias[1] * RAD_TO_DEG;
	state->gyro[2] -= nav.gyro_bias[2] * RAD_TO_DEG;
	state->pos[0] = nav.pos[0];
	state->pos[1] = nav.pos[1];
	state->pos[2] = nav.pos[2];
	state->vel[0] = nav.vel[0];
	state->vel[1] = nav.vel[1];
	state->vel[2] = nav.vel[2];
	state->updated |= SENSOR_UPDATE_attitude | SENSOR_UPDATE_pos | SENSOR_UPDATE_vel;
	ins_covariance_prediction(dt);

	if (IS_SET(this->work.updated, SENSOR_UPDATE_mag))
	{
		sensors |= MAG_SENSORS;
	}

	if (IS_SET(this->work.updated, SENSOR_UPDATE_pos))
	{
		sensors |= POS_SENSORS;
	}

	if (IS_SET(this->work.updated, SENSOR_UPDATE_vel))
	{
		sensors |= HORIZ_SENSORS | VERT_SENSORS;
	}

	if (sensors)
	{
		ins_correction(this->work.mag, this->work.pos, this->work.vel, this->work.baro[0], sensors);
	}

	this->work.updated = 0;
	return FILTERRESULT_OK;


}
