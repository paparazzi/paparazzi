
CC = gcc
CFLAGS = -I.. -I/home/poine/work/savannah/paparazzi3/sw/include -DFLOAT_T=double -Wall
LDFLAGS = -lm



test_vg_ref: test_vg_ref.c
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

test_vg_adpt: test_vg_adpt.c
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)


