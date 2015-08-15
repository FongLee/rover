#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>

#include "mavlink_bridge_header.h"
#include "mavlink.h"
#include "settings.h"
#include "scheduler.h"
#include "udp_driver.h"
#include "ap_math.h"
#include "imu.h"
#include "ap_gps.h"
#include "ap_control.h"
#include "my_timer.h"
#include "tcp_driver.h"


mavlink_system_t mavlink_system;

static uint32_t m_parameter_i = 0;
bool flag_communication_init = false;
bool flag_communication_connect = false;

#ifdef UDP
/**
 * communication initialization with UDP
 * @return 0: success; -1: error
 */
int communication_init(char *ip)
{


	mavlink_system.sysid = global_data.param[PARAM_SYSTEM_ID];
	mavlink_system.compid = global_data.param[PARAM_COMPONENT_ID];
	if (udp_init(ip) != 0)
	 	return -1;
	else
	{
		//flag_communication_init = true;
		return 0;
	}

}
#endif

#ifdef TCP
/**
 * communication initialization with tcp
 * @return 0: success; -1: error
 */
int communication_init(void (*handler)(int num))
{


	mavlink_system.sysid = global_data.param[PARAM_SYSTEM_ID];
	mavlink_system.compid = global_data.param[PARAM_COMPONENT_ID];
	if (tcp_init(handler) != 0)
	 	return -1;
	else
	{
		//flag_communication_init = true;
		return 0;
	}

}
#endif

/**
 * send system state
 */
void communication_system_state_send(void)
{
	mavlink_msg_heartbeat_send(MAVLINK_COMM_0, global_data.param[PARAM_SYSTEM_TYPE], global_data.param[PARAM_AUTOPILOT_TYPE], 0, 0, 0);
}

/**
 * send parameter
 */
void communication_parameter_send(void)
{
	if (m_parameter_i < ONBOARD_PARAM_COUNT)
	{
		mavlink_msg_param_value_send(MAVLINK_COMM_0,
						global_data.param_name[m_parameter_i],
						global_data.param[m_parameter_i], MAVLINK_TYPE_FLOAT, ONBOARD_PARAM_COUNT, m_parameter_i);
		m_parameter_i++;
	}
}

/**
 * send imu data
 */
void communication_imu_send()
{
	mavlink_msg_scaled_imu_send(MAVLINK_COMM_0,
								mpu.dmpTimestamp,
								mpu.calibratedAccel[VEC3_X] / 16384.0 * 1000,
							 	mpu.calibratedAccel[VEC3_Y] / 16384.0 * 1000,
							 	mpu.calibratedAccel[VEC3_Z] / 16384.0 * 1000,
								mpu.rawGyro[VEC3_X] / 16.38 / 180 * 3.14 * 1000,
								mpu.rawGyro[VEC3_Y] / 16.38 / 180 * 3.14 * 1000 ,
								mpu.rawGyro[VEC3_Z] / 16.38 / 180 * 3.14 * 1000 , 65535, 65535, 65535);

	//fprintf(stdout, "tmp_yaw is %f \n", tmp_yaw);
	mavlink_msg_attitude_send(MAVLINK_COMM_0,
								mpu.dmpTimestamp,
								mpu.fusedEuler[VEC3_X],
								mpu.fusedEuler[VEC3_Y],
								mpu.fusedEuler[VEC3_Z], 65535, 65535, 65535);
}

/**
 * send gps data
 */
void communication_gps_send()
{

	// mavlink_msg_gps_raw_int_send(MAVLINK_COMM_0, gps_timestamp* 1000, info.fix, info.lat*100000, info.lon*100000, info.elv * 1000,
	// 								info.HDOP * 100, info.VDOP * 100, info.speed / 3.6 * 100, info.direction * 100, info.satinfo.inview);

	mavlink_msg_gps_raw_int_send(MAVLINK_COMM_0, gps_timestamp* 1000, info.fix, gps_loc.lat, gps_loc.lng, gps_loc.alt * 10,
									info.HDOP * 100, info.VDOP * 100, info.speed / 3.6 * 100, info.direction * 100, info.satinfo.inview);

}

/**
 * handle received mavlink message data
 * @param chan MAVLINK_COMM_0
 * @param msg  [description]
 */
void handle_mavlink_message(mavlink_channel_t chan, mavlink_message_t *msg)
{
	switch (msg->msgid)
	{
		case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
		{
			mavlink_param_request_read_t set;
			mavlink_msg_param_request_read_decode(msg, &set);

			if ((uint8_t) set.target_system == (uint8_t) global_data.param[PARAM_SYSTEM_ID]
					&& (uint8_t) set.target_component == (uint8_t) global_data.param[PARAM_COMPONENT_ID])
			{

			}
		}
		break;

		case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
		{
			m_parameter_i = 0;
		}
		break;

		case MAVLINK_MSG_ID_PARAM_SET:
		{
			mavlink_param_set_t set;
			mavlink_msg_param_set_decode(msg, &set);

			if ((uint8_t) set.target_system == (uint8_t) global_data.param[PARAM_SYSTEM_ID]
					&& (uint8_t) set.target_component == (uint8_t) global_data.param[PARAM_COMPONENT_ID])
			{
				char *key = (char *) set.param_id;
				for (int i = 0; i < ONBOARD_PARAM_COUNT; i++)
				{
					bool match = true;
					for (int j = 0; j < ONBOARD_PARAM_NAME_LENGTH; j++)
					{
						if ((char) (global_data.param_name[i][j])
							!= (char) (key[j]))
						{
							match = false;
						}
						if ((char) global_data.param_name[i][j] == '\0')
						{
							break;
						}
					}
					if (match)
					{
						if (global_data.param[i] != set.param_value
								&& !isnan(set.param_value)
								&& !isinf(set.param_value)
								&& global_data.param_access[i])
						{
							global_data.param[i] = set.param_value;

						}
						else
						{
							mavlink_msg_param_value_send(MAVLINK_COMM_0,
									global_data.param_name[i],
									global_data.param[i], MAVLINK_TYPE_FLOAT, ONBOARD_PARAM_COUNT, m_parameter_i);

						}
					}
				}
			}

		}
		break;

		case MAVLINK_MSG_ID_PING:
		{
			mavlink_ping_t ping;
			mavlink_msg_ping_decode(msg, &ping);
			if (ping.target_system == 0 && ping.target_component == 0)
			{
				uint64_t r_timestamp;
				get_us(&r_timestamp);
				mavlink_msg_ping_send(chan, ping.seq, msg->sysid, msg->compid, r_timestamp);
			}
		}
		break;

		case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
		{
			mavlink_rc_channels_override_t rc_channels_override;
			mavlink_msg_rc_channels_override_decode(msg, &rc_channels_override);
			if ((uint8_t) rc_channels_override.target_system == (uint8_t) global_data.param[PARAM_SYSTEM_ID]
					&& (uint8_t) rc_channels_override.target_component == (uint8_t) global_data.param[PARAM_COMPONENT_ID])
			{

#ifdef CONTROL_DEBUG
				fprintf(stdout, "\n");
				fprintf(stdout, "RC channel 1: %d\n", rc_channels_override.chan1_raw);
				fprintf(stdout, "RC channel 2: %d\n", rc_channels_override.chan2_raw);
				fprintf(stdout, "RC channel 3: %d\n", rc_channels_override.chan3_raw);
				fprintf(stdout, "RC channel 4: %d\n", rc_channels_override.chan4_raw);
#endif
				channel_throttle = rc_channels_override.chan3_raw;
				channel_steer = rc_channels_override.chan4_raw;
				begin_control = true;
			}
		}
		break;

		default:
			break;

	}
}

/**
 * receive message from computer
 * @return  >0: receive buf's length; -1: err; 0: tcp is closed
 */
int communication_receive(void)
{
	mavlink_message_t msg;
	mavlink_status_t status = {0};
	int i = 0;
	int rec_size = 0;
	uint8_t buf_receive[500];
	memset(buf_receive, 0, 500);

#ifdef TCP
	rec_size = tcp_receive(buf_receive, 500);
#endif

#ifdef UDP
	rec_size = udp_receive(buf_receive, 500);
#endif

	if (rec_size < 0)
	{
		return -1;
	}
	else if (rec_size == 0)
	{
		return 0;
	}
	else
	{
		for (i = 0; i < rec_size; i++)
		{
			if (mavlink_parse_char(MAVLINK_COMM_0, buf_receive[i], &msg, &status))
			{

#ifdef COMMU_DEBUG
				fprintf(stdout, "\n");
				fprintf(stdout, "received packed,sys: %d, comp: %d, LEN : %d, MSG ID: %d\n", msg.sysid, msg.compid, msg.len, msg.msgid);
#endif
				handle_mavlink_message(MAVLINK_COMM_0, &msg);
			}
		}
	}
	return rec_size;
}

/**
 * send message in mavlink protocol
 * @param chan   channel number
 * @param ch     buffer
 * @param length length of buffer
 */
void mavlink_send_uart_bytes(mavlink_channel_t chan, const uint8_t *ch, uint16_t length)
{
	int send_size = 0;
	if (chan == MAVLINK_COMM_0)
	{

#ifdef TCP
		send_size = tcp_send(ch, length);
#endif

#ifdef UDP
		send_size = udp_send(ch, length);
#endif

#ifdef COMMU_DEBUG
		if (send_size < 0)
		{
			fprintf(stderr, "sending packet err: %s\n", strerror(errno));
		}

#endif
	}
}

