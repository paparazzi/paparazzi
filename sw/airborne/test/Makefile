

CC = gcc
CFLAGS = -I.. -I../../include -I../booz -I../../booz -DFLOAT_T=double -Wall
LDFLAGS = -lm

test_geodetic: test_geodetic.c ../math/pprz_geodetic_float.c ../math/pprz_geodetic_int.c ../math/pprz_trig_int.c
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

test_algebra: test_algebra.c ../math/pprz_trig_int.c
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

test_sqrt: test_sqrt.c
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

test_fmul: test_fmul.c
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

%.exe : %.c
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

clean:
	rm -f *~ test_geodetic test_algebra *.exe
