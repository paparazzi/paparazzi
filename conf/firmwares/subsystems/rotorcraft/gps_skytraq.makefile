# Hey Emacs, this is a -*- makefile -*-

GPS_LED ?= none

ap.CFLAGS += -DUSE_GPS
ap.CFLAGS += -DGPS_LINK=$(GPS_PORT)
ap.CFLAGS += -DUSE_$(GPS_PORT)
ap.CFLAGS += -D$(GPS_PORT)_BAUD=$(GPS_BAUD)

ifneq ($(GPS_LED),none)
  ap.CFLAGS += -DGPS_LED=$(GPS_LED)
endif

ap.srcs += $(SRC_SUBSYSTEMS)/gps.c
ap.CFLAGS += -DGPS_TYPE_H=\"subsystems/gps/gps_skytraq.h\"
ap.srcs += $(SRC_SUBSYSTEMS)/gps/gps_skytraq.c


nps.CFLAGS += -DUSE_GPS
nps.srcs += $(SRC_SUBSYSTEMS)/gps.c
