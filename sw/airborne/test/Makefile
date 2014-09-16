
# Quiet compilation
Q=@


CC = gcc
CFLAGS = -std=c99 -I.. -I../../include -Wall
#CFLAGS += -DDEBUG
LDFLAGS = -lm


test_matrix: test_matrix.c
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

test_geodetic: test_geodetic.c ../math/pprz_geodetic_float.c ../math/pprz_geodetic_double.c ../math/pprz_geodetic_int.c ../math/pprz_trig_int.c
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

test_algebra: test_algebra.c ../math/pprz_trig_int.c ../math/pprz_algebra_int.c ../math/pprz_algebra_float.c ../math/pprz_algebra_double.c
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

test_bla: test_bla.c ../math/pprz_trig_int.c ../math/pprz_algebra_int.c ../math/pprz_algebra_float.c ../math/pprz_algebra_double.c
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

%.exe : %.c
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

clean:
	$(Q)rm -f *~ test_matrix test_geodetic test_algebra test_bla *.exe
