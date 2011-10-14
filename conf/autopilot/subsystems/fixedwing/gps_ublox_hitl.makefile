# Hey Emacs, this is a -*- makefile -*-

# UBlox Hardware In The Loop


ap.CFLAGS += -DUSE_GPS -DUBX -DGPS_USE_LATLONG
ap.CFLAGS += -DGPS_TYPE_H=\"subsystems/gps/gps_ubx.h\"
ap.srcs   +=  $(SRC_SUBSYSTEMS)/gps/gps_ubx.c $(SRC_SUBSYSTEMS)/gps.c
