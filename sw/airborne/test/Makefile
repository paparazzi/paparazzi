
# Quiet compilation
Q=@


CC = gcc
CFLAGS = -std=c99 -I.. -I../../include -Wall
#CFLAGS += -DDEBUG
CFLAGS += -DCA_N_U=6
CFLAGS += -DCA_N_V=4
LDFLAGS = -lm


test_matrix: test_matrix.c
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

test_geodetic: test_geodetic.c ../math/pprz_geodetic_float.c ../math/pprz_geodetic_double.c ../math/pprz_geodetic_int.c ../math/pprz_trig_int.c
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

test_algebra: test_algebra.c ../math/pprz_trig_int.c ../math/pprz_algebra_int.c ../math/pprz_algebra_float.c ../math/pprz_algebra_double.c
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

test_bla: test_bla.c ../math/pprz_trig_int.c ../math/pprz_algebra_int.c ../math/pprz_algebra_float.c ../math/pprz_algebra_double.c
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

test_geo: test_geo_conversions.c ../math/pprz_trig_int.c ../math/pprz_algebra_int.c ../math/pprz_algebra_float.c ../math/pprz_algebra_double.c ../math/pprz_geodetic_int.c ../math/pprz_geodetic_float.c ../math/pprz_geodetic_double.c
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

test_alloc: test_alloc.c ../math/wls/wls_alloc.c ../math/qr_solve/r8lib_min.c ../math/qr_solve/qr_solve.c
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS) -DWLS_N_U=8 -DWLS_N_V=4

test_filter: test_filter.c
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

test_tt: test_tilt_twist.c ../math/pprz_algebra_float.c
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

%.exe : %.c
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

clean:
	$(Q)rm -f *~ test_matrix test_geodetic test_algebra test_bla test_alloc *.exe
