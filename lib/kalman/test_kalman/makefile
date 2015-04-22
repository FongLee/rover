all: test

CC = gcc
CFLAGS = -c -std=gnu99 -Wall

TESTDIR = .
KALMANDIR = ..
MATRIXDIR = ../../mesch12b
CUNITDIR = /opt/cunit/local

OBJS =  $(KALMANDIR)/kalman.o $(KALMANDIR)/matrix_kalman.o test_kalman.o test_run.o   $(MATRIXDIR)/meschach.a
INCS = -I $(KALMANDIR) -I $(MATRIXDIR) -I $(CUNITDIR)/include/CUnit  
LIBS = -lm -L $(CUNITDIR)/lib -lcunit

test: $(OBJS)
	$(CC)  $(INCS) $^ -static $(LIBS) -o $@

%.o: %.c
	$(CC) $(CFLAGS) $(INCS) $^ -o $@ 

$(MATRIXDIR)/meschach.a:
	cd $(MATRIXDIR) && make CC=gcc

clean:
	rm -f $(OBJS)
	rm -f test
	cd $(MATRIXDIR) && make clean
