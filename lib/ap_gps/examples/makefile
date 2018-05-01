
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
OBJS=	 gps_test.o  ap_gps.o uart_driver.o  \
			scheduler.o  ap_math.o\
			 $(MATRIXDIR)/meschach.a $(NMEADIR)/lib/libnmea.a

LIBS=	  -lrt -lm

INCS = -I $(NMEADIR)/include -I $(HALDIR) -I $(GPSDIR)  \
			-I $(MATHDIR) -I $(MATRIXDIR) \

.PHONY: all gps_test

all: gps_test


%.o: %.c
	$(CC)  $(CFLAGS) $(INCS) $<  $(DEFS) -o $@

ap_gps.o: $(GPSDIR)/ap_gps.c
	$(CC)  $(CFLAGS) $(INCS) $<  $(DEFS) -o $@

uart_driver.o: $(HALDIR)/uart_driver.c
	$(CC)  $(CFLAGS) $(INCS) $<  $(DEFS) -o $@


ap_math.o : $(MATHDIR)/ap_math.c
	$(CC)  $(CFLAGS) $(INCS) $<  $(DEFS) -o $@

scheduler.o: $(HALDIR)/scheduler.c
	$(CC)  $(CFLAGS) $(INCS) $<  $(DEFS) -o $@
	
$(MATRIXDIR)/meschach.a: 
	cd $(MATRIXDIR) && make CC=$(CC) CFLAGS='-O -g'

$(NMEADIR)/lib/libnmea.a:
	cd $(NMEADIR) && make CC=$(CC) CFLAGS=-g

gps_test: $(OBJS)
	$(CC) $(INCS)  $^ $(LIBS)    -o $@


clean:
	rm -f $(OBJS)
	rm -f gps_test

