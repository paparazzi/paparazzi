

CC = gcc
CFLAGS=-g -O2 -Wall `pkg-config gtk+-2.0 --cflags`
LDFLAGS=`pkg-config gtk+-2.0 --libs` `pcre-config --libs` -lglibivy -lm -lgtkdatabox

ir_calib : main.c calibrator.c gui.c
	$(CC) $(CFLAGS) -g -o $@ $^ $(LDFLAGS)

clean:
	rm -f *~ ir_calib