# Hey Emacs, this is a -*- makefile -*-

GPS_LED ?= none
UBX_GPS_PORT_LOWER=$(shell echo $(GPS_PORT) | tr A-Z a-z)

ap.srcs += $(SRC_SUBSYSTEMS)/gps.c
ap.CFLAGS += -DGPS_TYPE_H=\"subsystems/gps/gps_ubx.h\"
ap.srcs += $(SRC_SUBSYSTEMS)/gps/gps_ubx.c

ap.CFLAGS += -DUSE_$(GPS_PORT) -D$(GPS_PORT)_BAUD=$(GPS_BAUD)
ap.CFLAGS += -DUSE_GPS -DGPS_LINK=$(UBX_GPS_PORT_LOWER)

ifneq ($(GPS_LED),none)
  ap.CFLAGS += -DGPS_LED=$(GPS_LED)
endif

nps.CFLAGS += -DUSE_GPS
nps.srcs += $(SRC_SUBSYSTEMS)/gps.c
nps.CFLAGS += -DGPS_TYPE_H=\"subsystems/gps/gps_sim_nps.h\"
nps.srcs += $(SRC_SUBSYSTEMS)/gps/gps_sim_nps.c
