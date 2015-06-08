
#webcam_server webcam_shower
#CC=gcc
#LD=ld
CC=gcc
LD=ld
CFLAGS=	-c -Wall -std=gnu99 


#DEFS = -DKALMAN_DEBUG
DEFS =

MATRIXDIR = ../mesch12b
#CXXFLAGS=	-c -g -O0 -fPIC -I/opt/webcam/ffmpeg/include -I/usr/include -L/opt/webcam/ffmpeg/lib
OBJS=	 kalman.o  matrix_kalman.o kalman_test.o $(MATRIXDIR)/meschach.a

LIBS=	-lm 

INCS = -I $(MATRIXDIR) 

.PHONY: all  kalman

all:  kalman	

#all-before:
#	cd ../../nmealib && $(MAKE)

%.o: %.c 
	$(CC)  $(CFLAGS) $(INCS) $<  $(DEFS) -o $@  

$(MATRIXDIR)/meschach.a:
	cd $(MATRIXDIR) && $(MAKE) 

kalman: $(OBJS)	
	$(CC) $(INCS)  $^ $(LIBS)    -o $@

clean:
	rm -f $(OBJS)
	rm -f kalman
	cd $(MATRIXDIR) && make clean
	