

CC = gcc
CFLAGS=-g -O2 -Wall `pkg-config glib-2.0 --cflags`
LDFLAGS=`pkg-config glib-2.0 --libs` `pcre-config --libs` -lglibivy -lm

ir_calib : main.c
	$(CC) $(CFLAGS) -g -o $@ $^ $(LDFLAGS)