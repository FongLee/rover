#include <stdio.h>
#include <unistd.h>
#include <stdint.h>

#include "ap_gps.h"
//#include "ap_location.h"
#include "scheduler.h"

int main()
{
	scheduler_init();
	uint64_t timer = 0;
	get_ms(&timer);
	fprintf(stdout, "\ntime stamp is :%lu \n", (unsigned long)timer);
	gps_init(&gps_data);
	while(1)
	{
		gps_parse(&gps_data);

		fprintf(stdout, "gps time stamp is  :%lu ms\n", (unsigned long)gps_data.gps_timestamp);
		delay_ms(1000);
		fprintf(stdout, "gps_quality is :%d \n", gps_quality(&gps_data));
		fprintf(stdout, "Operating mode is :%d \n", gps_op_mode(&gps_data));
        fprintf(stdout, "latitude:%f degree,longitude:%f degree\n",nmea_radian2degree(gps_data.position.lat),nmea_radian2degree(gps_data.position.lon));
        fprintf(stdout, "the satellite being used:%d\n",num_sats(&gps_data));
        fprintf(stdout, "altitude:%f m\n", gps_data.info.elv);
        fprintf(stdout, "speed:%f m/s\n", ground_speed(&gps_data));
        fprintf(stdout, "speed:%ld cm/s\n", ground_speed_cm(&gps_data));
        fprintf(stdout, "direction:%ld centidegrees\n", ground_course_cd(&gps_data));

	}
}