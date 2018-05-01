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
#define UART_DEVICE 	"/dev/ttySAC3"
#define MAXBUF_UART 	1024
#define GPS_MIN_SPEED 	3
#define RADIUS 			1.0
#define ACCEL_MAX 		2.0

/**
 * gps initialization
 * @return 0: success; -1: err
 */
int gps_init(gps_data_t *gps_data)
{
	
	//use rate in 9600
	gps_data->fd_gps = -1;
	if (uart_init(&(gps_data->fd_gps), UART_DEVICE,1) != 0)
	{
		return -1;
	}
	
	nmea_zero_INFO(&(gps_data->info));
	nmea_zero_INFO(&(gps_data->prior_info));
  	if (nmea_parser_init(&(gps_data->parser)) == 0)
  	{
  		return -1;
  	}

	gps_data->velocity_body = v_get(3);

	gps_data->flag_get_first_data =false;
	gps_data->flag_gps_glitching = false;
	gps_data->flag_gps_init = false;
	gps_data->gps_timestamp = 0;
	
    return 0;

}

/**
 * close gps
 */
void gps_end(gps_data_t *gps_data)
{
	uart_close(&(gps_data->fd_gps));
	nmea_parser_destroy(&(gps_data->parser));

	v_free(gps_data->velocity_body);
}

/**
 * parse the data of gps
 * @return 0: success; -1: err
 */
int  gps_parse(gps_data_t *gps_data)
{
	char gps_buf[MAXBUF_UART];
	nmeaPOS prior_position;
	uint64_t last_gps_timestamp = 0;
	int len = 0;
	
	memset(gps_buf,0,MAXBUF_UART);
	len = read_uart(gps_data->fd_gps, gps_buf, MAXBUF_UART);

#ifdef 	GPS_DEBUG
	char newbuf[MAXBUF_UART + 1];
	memset(newbuf, 0, MAXBUF_UART + 1);
	memcpy(newbuf, gps_buf, len);
	newbuf[len] = '\0';
	fprintf(stdout, "read_uart length is %d\n" , len);
	if(len > 0)
	{		
		fprintf(stdout, "read_uart buf is %s\n" , newbuf);
	}
#endif

	if ( len <= 0)
	{
		gps_data->flag_gps_glitching = true;
		return -1;
	}	

	else if ( len > 0)
	{

		memcpy(&(gps_data->prior_info), &(gps_data->info), sizeof(nmeaINFO));
		gps_data->info.sig = NMEA_SIG_BAD;
    	gps_data->info.fix = NMEA_FIX_BAD;
		nmea_parse(&(gps_data->parser), gps_buf, len, &(gps_data->info));

		if(gps_data->info.sig == NMEA_SIG_BAD || gps_data->info.fix == NMEA_FIX_BAD)
		{
			gps_data->flag_gps_glitching = true;
			return -1;
		}

		
		last_gps_timestamp = gps_data->gps_timestamp;
		memcpy( &prior_position, &(gps_data->position), sizeof(nmeaPOS));

		get_ms(&(gps_data->gps_timestamp));
		nmea_info2pos(&(gps_data->info),&(gps_data->position));
		gps_data->velocity_body->ve[0] = ground_speed(gps_data);
		//if GPS is unvaluable for a long time , it is not work
		//check_gps(gps_data, &prior_position, last_gps_timestamp);
		gps_data->flag_gps_glitching = false;
#ifdef 	GPS_DEBUG
        /*dispaly the parsed data*/
		fprintf(stdout, "gps_timestamp is %lu\n", (unsigned long )(gps_data->gps_timestamp));
		fprintf(stdout, "gps_quality is :%d \n", gps_quality(gps_data));
		fprintf(stdout, "Operating mode is :%d \n", gps_op_mode(gps_data));
		fprintf(stdout, "latitude:%f degree,longitude:%f degree\n",nmea_radian2degree(gps_data->position.lat),nmea_radian2degree(gps_data->position.lon));
		fprintf(stdout, "latitude:%f reg,longitude:%f reg\n", gps_data->position.lat, gps_data->position.lon);
		fprintf(stdout, "the satellite being used:%d,the visible satellite:%d\n", gps_data->info.satinfo.inuse,gps_data->info.satinfo.inview);
        fprintf(stdout, "altitude:%f m\n", gps_data->info.elv);
        fprintf(stdout, "speed:%f km/h\n", gps_data->info.speed);
        fprintf(stdout, "direction:%f degree\n", gps_data->info.direction);
#endif

		return 0;
	}
}


/**
 * get quality gps (0 = Invalid; 1 = Fix; 2 = Differential,
 * 					3 = Sensitive)
 * @return  quality indicator of gps
 */
int  gps_quality(gps_data_t *gps_data)
{
	return gps_data->info.sig;
}

/**
 * get mode of gps
 * @return mode of gps
 */
int gps_op_mode(gps_data_t *gps_data)
{
	return gps_data->info.fix;
}


/**
 * get speed of obeject in m/s
 * @return speed of obeject
 */
double ground_speed(gps_data_t *gps_data)
{
    return gps_data->info.speed / NMEA_TUS_MS; 	// k/h to m/s
}

/**
 * get speed of obeject in cm/s
 * @return speed of obeject
 */
long ground_speed_cm(gps_data_t *gps_data)
{
    return (long)(ground_speed(gps_data) * 100);
}


/**
 * get ground course in centidegrees
 * @return ground course
 */
long ground_course_cd(gps_data_t *gps_data)
{
	return (long)(gps_data->info.direction * 100);
}


/**
 * get satalite of gps
 * @return number of satellite in use
 */
int num_sats(gps_data_t *gps_data)
{
	return gps_data->info.satinfo.inuse;
}

/**
 * get velocity in 3d of object
 * @param v speed of vector
 */
int get_3d_velocity(gps_data_t *gps_data, MAT * c_bn, VEC *velocity_nav)
{
	if(gps_data == NULL || c_bn == NULL || velocity_nav == NULL)
	{
		return -1;
	}

	mv_mlt(c_bn, gps_data->velocity_body, velocity_nav);
	
	return 0;
}	

/**
 * check positon of object
 */
void check_gps(gps_data_t *gps_data, nmeaPOS *prior_position, uint64_t last_gps_timestamp)
{
	uint64_t now_time = 0;
	double  sane_dt = 0.0;
	double distance = 0.0;
	double accel_based_distance = 0.0;
	bool all_ok = false;
	
	if (gps_op_mode(gps_data) < NMEA_FIX_2D || gps_quality(gps_data) == NMEA_SIG_BAD)
	{
		gps_data->flag_gps_glitching = true;
		return ;
	}
	
	now_time = gps_data->gps_timestamp;
	sane_dt = (now_time - last_gps_timestamp) / 1000.0f;
	distance = nmea_distance(&(gps_data->position), prior_position);

#ifdef GPS_DEBUG
	fprintf(stdout, "between distance is %f\n", distance); 
#endif

	if (distance <= RADIUS)
	{
		all_ok = true;
	}
	else
	{
		accel_based_distance = 0.5f * ACCEL_MAX * sane_dt * sane_dt;
		all_ok = (distance <= accel_based_distance);
	}

	gps_data->flag_gps_glitching = !all_ok;

#ifdef GPS_DEBUG
	fprintf(stdout, "flag_gps_glitching is %d\n", gps_data->flag_gps_glitching); 
#endif	
	return ;

}

