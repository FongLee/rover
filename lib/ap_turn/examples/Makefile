
#webcam_server webcam_shower
#CC=gcc
#LD=ld
CC=arm-linux-gcc
LD=arm-linux-ld
CFLAGS=	-c -Wall -std=gnu99 

#DEFS = -DGPS_DEBUG
DEFS = 

MOTODIR = ..
#HALDIR = ../../ap_hal_linux
#MATHDIR = ../../ap_math
#MATRIXDIR = ../../mesch12b
#NMEADIR = ../../nmealib
#CXXFLAGS=	-c -g -O0 -fPIC -I/opt/webcam/ffmpeg/include -I/usr/include -L/opt/webcam/ffmpeg/lib
OBJS=	 moto_test.o  $(MOTODIR)/ap_turn.o 
#LIBS=	   -Llib -lnmea -lrt -lm 

INCS =  -I $(MOTODIR)  
			

.PHONY: all all-before moto_test

all: all-before moto_test	

#all-before:
#	cd ../../nmealib && $(MAKE)

%.o: %.c 
	$(CC)  $(CFLAGS) $(INCS) $<  $(DEFS) -o $@  

#../../mesch12b/meschach.a:
#	cd ../../mesch12b/ && $(MAKE) 

moto_test: $(OBJS)	
	$(CC) $(INCS)  $^    -o $@

clean:
	rm -f $(OBJS)
	rm -f moto_test
	