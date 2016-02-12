# Hey Emacs, this is a -*- makefile -*-

GPS_LED ?= none

ifneq ($(GPS_LED),none)
  ap.CFLAGS += -DGPS_LED=$(GPS_LED)
endif

ifdef SECONDARY_GPS
ifneq (,$(findstring $(SECONDARY_GPS), udp))
# this is the secondary GPS
ap.CFLAGS += -DGPS_SECONDARY_TYPE_H=\"subsystems/gps/gps_udp.h\"
ap.CFLAGS += -DSECONDARY_GPS=gps_udp
else
ap.CFLAGS += -DGPS_TYPE_H=\"subsystems/gps/gps_udp.h\"
ap.CFLAGS += -DPRIMARY_GPS=gps_udp
endif
else
# plain old single GPS usage
ap.CFLAGS += -DGPS_TYPE_H=\"subsystems/gps/gps_udp.h\"
endif

ap.CFLAGS += -DUSE_GPS
ap.srcs += $(SRC_SUBSYSTEMS)/gps.c
ap.srcs += $(SRC_SUBSYSTEMS)/gps/gps_udp.c

nps.CFLAGS += -DUSE_GPS
nps.srcs += $(SRC_SUBSYSTEMS)/gps.c
nps.CFLAGS += -DGPS_TYPE_H=\"subsystems/gps/gps_sim_nps.h\"
nps.srcs += $(SRC_SUBSYSTEMS)/gps/gps_sim_nps.c
