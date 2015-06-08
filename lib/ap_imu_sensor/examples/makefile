#
# Makefile for eMPL Linux userland implementation
#

CC = arm-linux-gcc
#CC = gcc
CFLAGS = -Wall -fsingle-precision-constant -std=gnu99

# add -DI2C_DEBUG -DMPU9150_DEBUG  -DKALMAN_DEBUG  -DAHRS_DEBUG for debugging
DEFS = -DEMPL_TARGET_LINUX -DMPU9150 -DAK8975_SECONDARY

EMPLDIR = ../eMPL
HALDIR = ../../ap_hal_linux
MPUDIR = ../mpu9150
KALDIR = ../../kalman
MATRIXDIR = ../../mesch12b
MATHDIR = ../../ap_math
AHRSDIR = ../../ap_ahrs

OBJS = inv_mpu.o \
       inv_mpu_dmp_motion_driver.o \
       i2c_driver.o \
       scheduler.o \
       mpu9150.o \
       quaternion.o \
       vector3d.o \
       kalman.o \
       matrix_kalman.o \
       ap_ahrs.o \
       ap_math.o \
       mpu_calibration.o \
       $(MATRIXDIR)/meschach.a \


INCS = -I ../ -I $(EMPLDIR) -I $(HALDIR) -I $(MPUDIR) \
		-I $(KALDIR) -I $(MATRIXDIR) -I $(MATHDIR) \
		-I $(AHRSDIR)

.PHONY: all imu imucal cal_test

all: imu imucal cal_test

imu: $(OBJS) imu.o
	$(CC) $(CFLAGS) imu.o $(OBJS)  -lm -lrt -o $@

imucal: $(OBJS) imucal.o
	$(CC) $(CFLAGS) $(OBJS) imucal.o -lm -lrt   -o $@

cal_test: $(OBJS) test_cal.o
	$(CC) $(CFLAGS)  test_cal.o $(OBJS) -lm -lrt -o $@

imu.o: imu.c
	$(CC) $(CFLAGS) $(INCS)  $(DEFS)  -c $<

imucal.o: imucal.c
	$(CC) $(CFLAGS) $(INCS) -I $(MPUDIR) $(DEFS) -c $<

test_cal.o: test_cal.c
	$(CC) $(CFLAGS) $(INCS)  $(DEFS)  -c $<

mpu9150.o: $(MPUDIR)/mpu9150.c
	$(CC) $(CFLAGS) $(DEFS) $(INCS) -c $<

quaternion.o: $(MPUDIR)/quaternion.c
	$(CC) $(CFLAGS) $(DEFS) -c $<

vector3d.o: $(MPUDIR)/vector3d.c
	$(CC) $(CFLAGS) $(DEFS) -c $<

i2c_driver.o: $(HALDIR)/i2c_driver.c
	$(CC) $(CFLAGS) $(DEFS) $(INCS) -c $<

scheduler.o: $(HALDIR)/scheduler.c
	$(CC) $(CFLAGS) $(DEFS) $(INCS) -c $<

inv_mpu_dmp_motion_driver.o: $(EMPLDIR)/inv_mpu_dmp_motion_driver.c
	$(CC) $(CFLAGS) $(DEFS) $(INCS) -c $<

inv_mpu.o: $(EMPLDIR)/inv_mpu.c
	$(CC) $(CFLAGS) $(DEFS) $(INCS) -c $<

kalman.o: $(KALDIR)/kalman.c
	$(CC) $(CFLAGS) $(DEFS) $(INCS) -c $<

matrix_kalman.o: $(KALDIR)/matrix_kalman.c
	$(CC) $(CFLAGS) $(DEFS) $(INCS) -c $<

$(MATRIXDIR)/meschach.a:
	cd $(MATRIXDIR) && make

ap_ahrs.o: $(AHRSDIR)/ap_ahrs.c
	$(CC) $(CFLAGS) $(DEFS) $(INCS) -c $<

ap_math.o: $(MATHDIR)/ap_math.c
	$(CC) $(CFLAGS) $(DEFS) $(INCS) -c $<

mpu_calibration.o: $(MPUDIR)/mpu_calibration.c
	$(CC) $(CFLAGS) $(DEFS) $(INCS) -c $<

kbhit.o: $(HALDIR)/kbhit.c
	$(CC) $(CFLAGS) $(DEFS) $(INCS) -c $<

clean:
	rm -f *.o imu imucal cal_test
	cd $(MATRIXDIR) && make clean
