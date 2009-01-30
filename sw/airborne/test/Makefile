

CC = gcc
CFLAGS = -I.. -DFLOAT_T=double -Wall
LDFLAGS = -lm

test_geodetic: test_geodetic.c ../pprz_geodetic_float.c
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)



clean:
	rm -f *~ test_geodetic
