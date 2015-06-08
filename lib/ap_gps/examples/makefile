
#webcam_server webcam_shower
#CC=gcc
#LD=ld
CC=arm-linux-gcc
LD=arm-linux-ld
CFLAGS=	-c -Wall -std=gnu99

DEFS = -DGPS_DEBUG
#DEFS =

GPSDIR = ..
HALDIR = ../../ap_hal_linux
MATHDIR = ../../ap_math
MATRIXDIR = ../../mesch12b
NMEADIR = ../../nmealib
#CXXFLAGS=	-c -g -O0 -fPIC -I/opt/webcam/ffmpeg/include -I/usr/include -L/opt/webcam/ffmpeg/lib
OBJS=	 gps_test.o  $(GPSDIR)/ap_gps.o $(HALDIR)/uart_driver.o  \
			$(HALDIR)/scheduler.o $(MATHDIR)/ap_location.o \
			$(MATHDIR)/ap_math.o $(MATRIXDIR)/meschach.a

LIBS=	   -Llib -lnmea -lrt -lm

INCS = -I $(NMEADIR)/include -I $(HALDIR) -I $(GPSDIR)  \
			-I $(MATHDIR) -I $(MATRIXDIR) \
			-L $(NMEADIR)/lib

.PHONY: all all-before gps_test

all: all-before gps_test

#all-before:
#	cd ../../nmealib && $(MAKE)

%.o: %.c
	$(CC)  $(CFLAGS) $(INCS) $<  $(DEFS) -o $@

../../mesch12b/meschach.a:
	cd ../../mesch12b/ && $(MAKE)

gps_test: $(OBJS)
	$(CC) $(INCS)  $^ $(LIBS)    -o $@

clean:
	rm -f $(OBJS)
	rm -f gps_test

