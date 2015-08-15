BIN = kbhit
all: kbhit

CC = gcc
CFLAGS = -c -g -Wall -std=gnu99
INCS = -I ../
HALDIR = ..

OBJS = $(HALDIR)/kbhit_linux.o

%.o: %.c
	$(CC) $(INCS) $(CFLAGS) $< -o $@

kbhit:  $(OBJS) test_kbhit.o
	$(CC) $(INCS) $^ -o $@

clean:
	rm -f *.o $(BIN) $(OBJS)
