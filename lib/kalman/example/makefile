
#webcam_server webcam_shower
#CC=gcc
#LD=ld
CC=gcc
LD=ld
CFLAGS=	-c -Wall -std=gnu99


#DEFS = -DKALMAN_DEBUG
DEFS = -DKALMAN_DEBUG

MATRIXDIR = ../../mesch12b
KALMANDIR = ..
#CXXFLAGS=	-c -g -O0 -fPIC -I/opt/webcam/ffmpeg/include -I/usr/include -L/opt/webcam/ffmpeg/lib
OBJS=	 $(KALMANDIR)/kalman.o  $(KALMANDIR)/matrix_kalman.o kalman_example.o $(MATRIXDIR)/meschach.a

LIBS=	-lm

INCS = -I $(MATRIXDIR) -I $(KALMANDIR)

.PHONY: all  

all:  example

#all-before:
#	cd ../../nmealib && $(MAKE)

%.o: %.c
	$(CC)  $(CFLAGS) $(INCS) $^  $(DEFS) -o $@

$(MATRIXDIR)/meschach.a:
	cd $(MATRIXDIR) && $(MAKE) CC=gcc

example: $(OBJS)
	$(CC) $(INCS)  $^ $(LIBS)    -o $@

clean:
	rm -f $(OBJS)
	rm -f kalman
	cd $(MATRIXDIR) && make clean

