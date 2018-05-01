all: location_test

BIN = location_test math_test
CC =gcc

CFLAGS = -std=gnu99 -c

OBJS = ap_math.o ap_location.o

MATHDIR = ..
MATRIXDIR = ../../mesch12b

INCS = -I $(MATHDIR) -I $(MATRIXDIR)

LIBS = -lm -lrt

%.o: %.c
	$(CC) $(CFLAGS) $(INCS) $< -o $@

ap_math.o: $(MATHDIR)/ap_math.c
	$(CC) $(CFLAGS) $(INCS) $< -o $@

ap_location.o: $(MATHDIR)/ap_location.c
	$(CC) $(CFLAGS) $(INCS)  $< -o $@

math_test: $(OBJS) math_test.o
	$(CC) $(INCS)  $^ $(LIBS) -o $@

location_test: $(OBJS) location_test.o
	$(CC) $(INCS)  $^ $(LIBS) -o $@

clean:
	rm -f $(OBJS) $(BIN)
