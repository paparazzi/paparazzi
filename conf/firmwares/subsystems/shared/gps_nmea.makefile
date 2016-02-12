# Hey Emacs, this is a -*- makefile -*-

# NMEA GPS unit

GPS_LED ?= none
NMEA_GPS_PORT ?= $(GPS_PORT)
NMEA_GPS_BAUD ?= $(GPS_BAUD)

NMEA_GPS_PORT_LOWER=$(shell echo $(NMEA_GPS_PORT) | tr A-Z a-z)

ap.CFLAGS += -DUSE_GPS
ap.CFLAGS += -DNMEA_GPS_LINK=$(NMEA_GPS_PORT_LOWER)
ap.CFLAGS += -DUSE_$(NMEA_GPS_PORT)
ap.CFLAGS += -D$(NMEA_GPS_PORT)_BAUD=$(NMEA_GPS_BAUD)

ifneq ($(GPS_LED),none)
  ap.CFLAGS += -DGPS_LED=$(GPS_LED)
endif

ifdef SECONDARY_GPS
ifneq (,$(findstring $(SECONDARY_GPS), nmea))
# this is the secondary GPS
ap.CFLAGS += -DGPS_SECONDARY_TYPE_H=\"subsystems/gps/gps_nmea.h\"
ap.CFLAGS += -DSECONDARY_GPS=gps_nmea
else
ap.CFLAGS += -DGPS_TYPE_H=\"subsystems/gps/gps_nmea.h\"
ap.CFLAGS += -DPRIMARY_GPS=gps_nmea
endif
else
# plain old single GPS usage
ap.CFLAGS += -DGPS_TYPE_H=\"subsystems/gps/gps_nmea.h\"
endif

ap.srcs += $(SRC_SUBSYSTEMS)/gps/gps_nmea.c
ap.srcs += $(SRC_SUBSYSTEMS)/gps.c

sim.CFLAGS += -DUSE_GPS
sim.CFLAGS += -DGPS_TYPE_H=\"subsystems/gps/gps_sim.h\"
sim.srcs += $(SRC_SUBSYSTEMS)/gps/gps_sim.c
sim.srcs += $(SRC_SUBSYSTEMS)/gps.c

nps.CFLAGS += -DUSE_GPS
nps.CFLAGS += -DGPS_TYPE_H=\"subsystems/gps/gps_sim_nps.h\"
nps.srcs += $(SRC_SUBSYSTEMS)/gps/gps_sim_nps.c
nps.srcs += $(SRC_SUBSYSTEMS)/gps.c
