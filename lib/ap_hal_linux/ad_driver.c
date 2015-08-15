#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <sys/ioctl.h> //ioctl
#include <unistd.h>	//close,read
#include <fcntl.h> //open
#include "ad_driver.h"
#include "scheduler.h"

#define ADC_SET_CHANNEL         0xc000fa01
#define ADC_SET_ADCTSC          0xc000fa02
#define CHANNELCOUNT 6

int fd_ad = -1;
bool flag_ad_init = false;
uint32_t last_good_time_ad = 0;

/**
 * ad initialization
 * @return 0: success; -1: error
 */
int ad_init()
{
	int channels[CHANNELCOUNT] = {0,1,4,5,6,7};
	int channel_ad;
	fd_ad= open("/dev/adc", 0);

	if (fd_ad < 0)
	{
		fprintf(stderr, "open ADC device err:%s\n", strerror(errno));
        return -1;
    }

    channel_ad = channels[0];
    if (ioctl(fd_ad, ADC_SET_CHANNEL, channel_ad) < 0)
    {
		fprintf(stderr, "can not set channel_ad for /dev/adc: %s\n", strerror(errno));
    	close(fd_ad);
    	return -1;
    }
    flag_ad_init = true;
#ifdef AD_DEBUG
		fprintf(stdout, "ad init succeed\n");
#endif
    return 0;
}

/**
 * close ad
 */
void ad_close()
{
	flag_ad_init = false;
	if (fd_ad != -1)
	{
		close(fd_ad);
		fd_ad = -1;
	}
}

/**
 * read voltage
 * @param  ad_vol data
 * @return        0: success; -1: error
 */
int read_ad_vol(float *ad_vol)
{
	uint32_t now;
	get_ms(&now);
	char buffer_ad[30];
	int len = 0;
	int ad_value;
	if (!flag_ad_init)
	{
		return -1;
	}
	len = read(fd_ad, buffer_ad, sizeof(buffer_ad) - 1);
	if (len > 0)
	{
		buffer_ad[len] = '\0';
#ifdef AD_DEBUG
		fprintf(stdout, "value of ad is:%s\n", buffer_ad);
#endif
		sscanf(buffer_ad, "%d", &ad_value);
		*ad_vol = ad_value * 3.3f / 4096.0f;
#ifdef AD_DEBUG
		fprintf(stdout, "vol of ad is:%f\n", *ad_vol);
#endif
		last_good_time_ad = now;
		return 0;
	}
	else
	{
		return -1;
	}
}

/**
 * read value of ad
 * @param  ad_value data
 * @return          0: success; -1: error
 */
int read_ad_value(int *ad_value)
{
	uint32_t now;
	get_ms(&now);
	char buffer_ad[30];
	int len = 0;
	if (!flag_ad_init)
	{
		return -1;
	}
	len = read(fd_ad, buffer_ad, sizeof(buffer_ad) - 1);
	if (len > 0)
	{
		buffer_ad[len] = '\0';
		sscanf(buffer_ad, "%d", ad_value);
#ifdef AD_DEBUG
		fprintf(stdout, "value of ad is:%d\n",  *ad_value);
#endif
		last_good_time_ad = now;
		return 0;
	}
	else
	{
		return -1;
	}
}
