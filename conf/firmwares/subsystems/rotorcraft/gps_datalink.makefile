# Hey Emacs, this is a -*- makefile -*-

GPS_LED ?= none

ifneq ($(GPS_LED),none)
  ap.CFLAGS += -DGPS_LED=$(GPS_LED)
endif

ifdef SECONDARY_GPS
ifneq (,$(findstring $(SECONDARY_GPS), datalink))
# this is the secondary GPS
ap.CFLAGS += -DGPS_SECONDARY_TYPE_H=\"subsystems/gps/gps_datalink.h\"
ap.CFLAGS += -DSECONDARY_GPS=gps_datalink
else
ap.CFLAGS += -DGPS_TYPE_H=\"subsystems/gps/gps_datalink.h\"
ap.CFLAGS += -DPRIMARY_GPS=gps_datalink
endif
else
# plain old single GPS usage
ap.CFLAGS += -DGPS_TYPE_H=\"subsystems/gps/gps_datalink.h\"
endif

ap.srcs += $(SRC_SUBSYSTEMS)/gps.c
ap.srcs += $(SRC_SUBSYSTEMS)/gps/gps_datalink.c

ap.CFLAGS += -DUSE_GPS -DGPS_DATALINK

nps.CFLAGS += -DUSE_GPS
nps.CFLAGS += -DGPS_TYPE_H=\"subsystems/gps/gps_sim_nps.h\"
nps.srcs += $(SRC_SUBSYSTEMS)/gps/gps_sim_nps.c
nps.srcs += $(SRC_SUBSYSTEMS)/gps.c
