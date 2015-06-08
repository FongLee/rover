
#ifndef LOCAL_DEFAULTS_H
#define LOCAL_DEFAULTS_H

// To avoid having to pass the same command line switches when running
// the test apps, you can specify the defaults for your platform here.

// RPi I2C bus 
#define DEFAULT_I2C_BUS 0

// Gumstix Overo
// #define DEFAULT_I2C_BUS 3

// Gumstix Duovero
// #define DEFAULT_I2C_BUS 2


// platform independent

#define DEFAULT_SAMPLE_RATE_HZ	100

#define DEFAULT_YAW_MIX_FACTOR 4

#endif /* LOCAL_DEFAULTS_H */

