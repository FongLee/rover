#ifndef AD_DRIVER_H
#define AD_DRIVER_H

/**
 * ad initialization
 * @return 0: success; -1: error
 */
int ad_init();

/**
 * close ad
 */
void ad_close();

/**
 * read voltage
 * @param  ad_vol data
 * @return        0: success; -1: error
 */
int read_ad_vol(float *ad_vol);

/**
 * read value of ad
 * @param  ad_value data
 * @return          0: success; -1: error
 */
int read_ad_value(int *ad_value);

#endif
