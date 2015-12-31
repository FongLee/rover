#./install/rover
.PHONY: all rover

#before

all:  rover


CC = arm-linux-gcc
#-pg
CFLAGS = -Wall -std=gnu99 -c -g
#-pg -lefence -g -static
LDFLAGS = -g


#add  -DGPS_DEBUG -DUDP_DEBUG -DCOMMU_DEBUG -DMPU9150_DEBUG_ERROR
#-DI2C_DEBUG -DMPU9150_DEBUG  -DKALMAN_DEBUG  -DAHRS_DEBUG  -DMEMWATCH
#-DMEMWATCH_STDIO -DULTRA_DEBUG  -DTCP_DEBUG -DCONTROL_DEBUG  -DSIG_DEBUG  for debugging
#add -DTCP -DUDP for using tcp or udp
# add -DAHRS_APPLLY -DKALMAN_THREE_APPLLY -DKALMAN_APPLLY -DEKF_APPLLY -DEKF_SEVEN_APPLLY -DKF_EKF
DEFS = -DEMPL_TARGET_LINUX -DMPU9150 -DAK8975_SECONDARY -DUDP -DEKF_APPLLY

INSTALL_PATH = ./install
SRCDIR = ./src
MAVLINKDIR = ./lib/mavlink
MAVLINKDIR2 = ./lib/mavlink/common
HALDIR = ./lib/ap_hal_linux
IMUDIR = ./lib/ap_imu_sensor
KALDIR = ./lib/kalman
EKFDIR = ./lib/efk_kalman
MATRIXDIR = ./lib/mesch12b
MATHDIR = ./lib/ap_math
AHRSDIR = ./lib/ap_ahrs
NMEADIR = ./lib/nmealib
GPSDIR = ./lib/ap_gps
CONTROLDIR = ./lib/ap_control
CAMDIR = ./lib/camera
MEMDIR= ./lib/memwatch
FENCEDIR = ./lib/electric_fence
ULRDIR=./lib/ap_ultrasonic
ORTPDIR=./lib/ortp

OBJS = $(SRCDIR)/communication.o \
		$(SRCDIR)/settings.o \
		$(SRCDIR)/my_timer.o \
		$(SRCDIR)/task.o \
		$(SRCDIR)/rover.o \
		$(HALDIR)/udp_driver.o \
		$(HALDIR)/tcp_driver.o \
		$(HALDIR)/scheduler.o \
		$(HALDIR)/i2c_driver.o \
		$(HALDIR)/uart_driver.o \
		$(MATHDIR)/ap_math.o \
		$(MATHDIR)/ap_location.o \
		$(IMUDIR)/eMPL/inv_mpu.o \
		$(IMUDIR)/eMPL/inv_mpu_dmp_motion_driver.o \
       	$(IMUDIR)/mpu9150/mpu9150.o \
       	$(IMUDIR)/mpu9150/quaternion.o \
       	$(IMUDIR)/mpu9150/vector3d.o \
       	$(IMUDIR)/mpu9150/mpu_calibration.o \
       	$(IMUDIR)/imu.o \
       	$(KALDIR)/kalman.o \
       	$(KALDIR)/matrix_kalman.o \
       	$(EKFDIR)/ekf.o \
       	$(GPSDIR)/ap_gps.o \
       	$(CONTROLDIR)/ap_control.o \
       	$(MATRIXDIR)/meschach.a \
       	$(NMEADIR)/lib/libnmea.a \
       	$(CAMDIR)/capture.o \
       	$(CAMDIR)/Encoder.o \
       	$(CAMDIR)/rtpsend.o \
       	$(CAMDIR)/SsbSipMfcDecAPI.o \
       	$(CAMDIR)/SsbSipMfcEncAPI.o \
       	$(ULRDIR)/ap_ultrasonic.o\
		$(MEMDIR)/memwatch.o \
		$(AHRSDIR)/ap_ahrs.o \
		$(AHRSDIR)/MahonyAHRS.o
		#$(ORTPDIR)/lib/libortp.so
       	#$(FENCEDIR)/lib/libefence.a
       	#ap_ahrs.o \
#change to -lpthread is incorrect -lrt
LIBS = -pthread  -lm  -lrt -L $(FENCEDIR)/lib -L $(ORTPDIR)/lib -lortp


INCS = -I $(MAVLINKDIR) -I $(MAVLINKDIR2) -I $(SRCDIR)  -I $(HALDIR) \
		-I $(IMUDIR)/eMPL -I $(IMUDIR)/mpu9150 -I $(IMUDIR) -I $(KALDIR) \
		-I $(MATRIXDIR) -I $(MATHDIR) -I $(AHRSDIR) -I $(NMEADIR) \
		-I $(NMEADIR)/include    -I $(GPSDIR) -I $(CONTROLDIR) \
		-I $(CAMDIR)  -I $(ULRDIR) \
		-I $(ORTPDIR)/include \
		-I $(MEMDIR) -I $(EKFDIR)

#NOTE:need root authority, file path can not obtain chinese character

empty:=
space:=$(empty) $(empty)

before: fence ortp

fence:
	cd $(FENCEDIR) && make  LIB_INSTALL_DIR=./lib MAN_INSTALL_DIR=./man  CC=arm-linux-gcc AR=arm-linux-ar install
ortp:
	cd $(ORTPDIR)/ortp-0.24.2 && \
	./configure CC=arm-linux-gcc --prefix=$(subst $(space),\ ,$(shell cd $(ORTPDIR) ; pwd)) --host=arm-linux -target=arm-linux && \
	make && make install

rover: $(OBJS)
	$(CC) $(INCS) $^    $(LDFLAGS) $(LIBS)  -o $@
	#$(CC) $(INCS) $^    -lefence   $(LIBS)  -o $@

%.o: %.c
	$(CC) $(CFLAGS) $(INCS) $(DEFS) $< -o $@

$(MATRIXDIR)/meschach.a:
	cd $(MATRIXDIR) && make CC=arm-linux-gcc CFLAGS='-O -g'

$(NMEADIR)/lib/libnmea.a:
	cd $(NMEADIR) && make CC=arm-linux-gcc CFLAGS=-g

$(FENCEDIR)/lib/libefence.a:
	cd $(FENCEDIR) && make  LIB_INSTALL_DIR=./lib MAN_INSTALL_DIR=./man  CC=arm-linux-gcc AR=arm-linux-ar install

install: rover
	mv rover $(INSTALL_PATH)
clean:
	rm -f $(OBJS)
	cd $(MATRIXDIR) && make clean
	cd $(NMEADIR) && make clean
	cd $(FENCEDIR) && make clean
	cd $(IMUDIR)/examples && make clean
