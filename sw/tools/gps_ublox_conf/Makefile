
# Quiet compilation
Q=@

CC=gcc
MODCFLAGS=-Wall

all:		ublox_conf

clean:
	$(Q)rm -f core *.o ublox_conf

ublox_conf:	ublox_conf.c Makefile
	$(CC) $(MODCFLAGS) ublox_conf.c -o ublox_conf

