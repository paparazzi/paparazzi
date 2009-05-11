gpsd_ivy: gpsd_ivy.c
	gcc -g -O2 -Wall `pkg-config glib-2.0 --cflags` -o $@ $^ `pkg-config glib-2.0 --libs`  `pcre-config --libs` -lglibivy -lgps

