# Hey Emacs, this is a -*- makefile -*-

# ARDrone 2 Flightrecorder GPS unit

GPS_LED ?= none

ap.CFLAGS += -DUSE_GPS -DUSE_GPS_ARDRONE2

ifneq ($(GPS_LED),none)
  ap.CFLAGS += -DGPS_LED=$(GPS_LED)
endif

ap.CFLAGS += -DGPS_TYPE_H=\"subsystems/gps/gps_ardrone2.h\"
ap.srcs += $(SRC_SUBSYSTEMS)/gps/gps_ardrone2.c
ap.srcs += $(SRC_SUBSYSTEMS)/gps.c

nps.CFLAGS += -DUSE_GPS
nps.CFLAGS += -DGPS_TYPE_H=\"subsystems/gps/gps_sim_nps.h\"
nps.srcs += $(SRC_SUBSYSTEMS)/gps/gps_sim_nps.c
nps.srcs += $(SRC_SUBSYSTEMS)/gps.c
