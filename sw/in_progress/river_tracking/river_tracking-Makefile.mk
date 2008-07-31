#Makefile written by Mitchel Humpherys

#if pkg-config --cflags opencv --libs opencv
#doesn't return something useful
#make sure PKG_CONFIG_PATH is set correctly
#Also, RTHOME must be set to the absolute path of the river_tracking
#directory in your environment.
RTSOURCE = $(RTHOME)/src
RTBIN = $(RTHOME)/bin
INCLUDES = `pkg-config --cflags --libs opencv gtk+-2.0` -I$(RTSOURCE) -lglibivy

CPP = g++ -g -Wno-deprecated
CC = gcc -g
CFLAGS = -c $(INCLUDES)

all : main

main : src/river_track.c
	$(CC) src/river_track.c -o bin/river_track $(INCLUDES)

clean :
	cd $(RTHOME) rm -f *.o
	cd $(RTSOURCE); rm -f *.o;
	cd $(RTBIN); rm -f river_track;
