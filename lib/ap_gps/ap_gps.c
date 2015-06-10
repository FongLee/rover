#include <nmea/nmea.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
//#include <stdlib.h>
#include "ap_gps.h"
#include "uart_driver.h"
#include "scheduler.h"
#include "ap_math.h"

//#define UART_DEVICE "/dev/ttySAC3"
#define UART_DEVICE "/dev/ttySAC3"
#define MAXBUF_UART 200
#define GPS_MIN_SPEED 3
//#define GPS_DEBUG
//struct location
nmeaINFO info;
nmeaPARSER parser;
VEC *gps_vel;
struct location gps_loc;
unsigned long last_good_update;
unsigned long last_good_lat;
unsigned long last_good_lon;
VEC *last_good_vel;
uint64_t gps_timestamp;

float radius_cm = 1000;
float accel_max_cmss = 200;

bool flag_gps_glitching = false;
bool flag_gps_init = false;
bool flag_gps = false;

int gps_init()
{

	if (uart_init(&fd_gps, UART_DEVICE,0) != 0)

	{
		return -1;
	}
	nmea_zero_INFO(&info);
	last_good_vel = v_get(2);
	gps_vel = v_get(3);
  	if (nmea_parser_init(&parser) == 0)
  	{
  		return -1;
  	}
  	// if(pthread_mutex_init(&lock_nmea,NULL) != 0)
  	// {
   //  //exit(EXIT_FAILURE);
   //  return -1;
  	// }
    return 0;

}

void gps_end()
{
	uart_close(fd_gps);
	nmea_parser_destroy(&parser);
}

void gps_parse()
{
	static char gps_buf[MAXBUF_UART];
	static int len = 0;
	memset(gps_buf,0,MAXBUF_UART);
	len = read_uart(fd_gps, gps_buf, MAXBUF_UART);
//fprintf(stdout, "len is %d\n" , len);
#ifdef 	GPS_DEBUG
	//gps_buf[len] = '\0';
	fprintf(stdout, "read_uart length is %d\n" , len);
	fprintf(stdout, "read_uart buf is %s\n" , gps_buf);
#endif

	if ( len > 0)
	{
		nmea_parse(&parser, gps_buf, len, &info);
		get_ms(&gps_timestamp);
#ifdef 	GPS_DEBUG
        /*dispaly the parsed data*/
		fprintf(stdout, "gps_timestamp is %lu\n", (unsigned long )gps_timestamp);
        fprintf(stdout, "latitude:%f,longitude:%f\n",info.lat,info.lon);
        fprintf(stdout, "the satellite being used:%d,the visible satellite:%d\n",info.satinfo.inuse,info.satinfo.inview);
        fprintf(stdout, "altitude:%f m\n", info.elv);
        fprintf(stdout, "speed:%f km/h\n", info.speed);
        fprintf(stdout, "direction:%f degree\n", info.direction);
#endif

		if (gps_op_mode() >= NMEA_FIX_2D)
		{
			ground_location(&gps_loc);
			if (ground_speed() >= GPS_MIN_SPEED)
			{
				fill_3d_velocity(gps_vel);
			}
		}
	}
}

int  gps_quality()
{
	return info.sig;
}

int gps_op_mode()
{
	return info.fix;
}
//speed in m/s
float ground_speed()
{
    return info.speed * 0.27778;	// k/h to m/s
}

//speed in cm/s
long ground_speed_cm()
{
    return (long)(ground_speed() * 100);
}

//ground course in centidegrees
long ground_course_cd()
{
	return (long)(info.direction * 100);
}

void ground_location(struct location *loc)
{
	static long altitude = 0;
	static long lattitude = 0;
	static long longitude = 0;
	altitude = info.elv * 100;
	lattitude = (long)(((int)info.lat / 100 +  \
					 (info.lat - (int)info.lat / 100 * 100) / 60.0f) * 1e7);
	longitude = (long)(((int)info.lon / 100 +  \
					 (info.lon - (int)info.lon / 100 * 100) / 60.0f) * 1e7);
	loc->alt = info.elv * 100;	//Altitude in centimeters (meters * 100)
	loc->lat = lattitude;		//Lattitude * 10**7
	loc->lng = longitude;		//Longitude * 10**7
}


int num_sats()
{
	return info.satinfo.inuse;
}

void fill_3d_velocity(VEC *v)
{
	float  gps_heading = radians(ground_course_cd());
	v->ve[0] = ground_speed() * cos(gps_heading);
	v->ve[1] = ground_speed() *sin(gps_heading);
	v->ve[2] = 0;
}

void check_position()
{
	uint64_t now ;
	struct location cur_loc;
	float  sane_dt;
	float distance_cm;
	float accel_based_distance;
	bool all_ok;

	get_ms(&now);
	if (gps_op_mode() < NMEA_FIX_2D)
	{
		flag_gps_glitching = true;
		return ;
	}
	if (!flag_gps)
	{
		last_good_update = now;
		last_good_lat = gps_loc.lat;
		last_good_lon = gps_loc.lng;
		last_good_vel->ve[0] = gps_vel->ve[0];
		last_good_vel->ve[1] = gps_vel->ve[1];
		flag_gps_init= true;
		flag_gps_glitching = false;
		return ;
	}
	sane_dt = (now - last_good_update) / 1000.0f;
	cur_loc.lat = last_good_lat;
	cur_loc.lng = last_good_lon;
	distance_cm = get_distance_cm(&cur_loc, &gps_loc);
	if (distance_cm <= radius_cm)
	{
		all_ok = true;
	}
	else
	{
		accel_based_distance = 0.5f * accel_max_cmss * sane_dt * sane_dt;
		all_ok = (distance_cm <= accel_based_distance);
	}

	if (all_ok)
	{
		last_good_update = now;
		last_good_lat = gps_loc.lat;
		last_good_lon = gps_loc.lng;
		last_good_vel->ve[0] = gps_vel->ve[0];
		last_good_vel->ve[1] = gps_vel->ve[1];
	}
	flag_gps_glitching = !all_ok;
	return ;

}

