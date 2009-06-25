
CC = gcc
CFLAGS = -Wall -I.. -I../.. -I../../test/ -I../../../include -Wall
LDFLAGS = -lm



test_vg_ref: test_vg_ref.c
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

test_vg_adpt: test_vg_adpt.c
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)


test_deadband: test_deadband.c
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

test_scaling: test_scaling.c
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)


