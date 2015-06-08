.PHONY: all
all: test

CC = gcc
CFLAGS = -c -std=gnu99 -Wall

MATRIXDIR = ../../mesch12b
KALMANDIR = ../
CUNITIDR = ../../cunit/local

OBJS = test_kalman.o test_run.o $(KALMANDIR)/matrix_kalman.o \
		$(MATRIXDIR)/meschach.a

INCS = -I $(MATRIXDIR) -I $(KALMANDIR) -I $(CUNITIDR)/include/CUnit
#-static is necessary
LIBS = -L $(CUNITIDR)/lib -lm -lcunit

test: $(OBJS)
	$(CC) $(INCS) $^ $(LIBS) -static -o $@

%.o: %.c
	$(CC) $(CFLAGS) $(INCS) $<  -o $@

$(MATRIXDIR)/meschach.a:
	cd $(MATRIXDIR) && make CC=gcc

clean:
	rm -f $(OBJS)
	rm -f test
	cd $(MATRIXDIR) && make clean
