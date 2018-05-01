all: navigation

MESCH_DIR = ../mesch12b
CC = gcc
LD = ld
CFLAGS = -c -Wall 
OBJS = navigation.o $(MESCH_DIR)/meschach.a
LIBS = -lm 
INCS = -I$(MESCH_DIR)

%.o:%.c
	$(CC) $(CFLAGS) $(INCS) $< -o $@

navigation:$(OBJS) 
	$(CC) $(INCS) $^  $(LIBS) -o $@
#cc -O -DHAVE_CONFIG_H -I./mesch12b -o matrix_test matrix_test.c  meschach.a -lm

clean:
	rm -f *.o
	rm -f navigation