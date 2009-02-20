

CC = gcc
CFLAGS = -I.. -I../../include -DFLOAT_T=double -Wall
LDFLAGS = -lm

test_geodetic: test_geodetic.c ../pprz_geodetic_float.c
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

test_fmul: test_fmul.c
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)



clean:
	rm -f *~ test_geodetic
