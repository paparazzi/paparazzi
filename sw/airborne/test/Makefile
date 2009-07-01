

CC = gcc
CFLAGS = -I.. -I../../include -I../booz -I../../booz -DFLOAT_T=double -Wall
LDFLAGS = -lm

test_geodetic: test_geodetic.c ../pprz_geodetic_float.c ../pprz_geodetic_int.c ../booz/booz_trig_int.c
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

test_algebra: test_algebra.c ../booz/booz_trig_int.c
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

test_sqrt: test_sqrt.c
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

test_fmul: test_fmul.c
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

%.exe : %.c
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

clean:
	rm -f *~ test_geodetic *.exe
