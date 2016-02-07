# Hey Emacs, this is a -*- makefile -*-

# Furuno NMEA GPS unit

GPS_LED ?= none
FURUNO_GPS_PORT_LOWER=$(shell echo $(GPS_PORT) | tr A-Z a-z)

ap.CFLAGS += -DUSE_GPS
ap.CFLAGS += -DNMEA_GPS_LINK=$(FURUNO_GPS_PORT_LOWER)
ap.CFLAGS += -DUSE_$(GPS_PORT)
ap.CFLAGS += -D$(GPS_PORT)_BAUD=$(GPS_BAUD)
ap.CFLAGS += -DNMEA_PARSE_PROP

ifneq ($(GPS_LED),none)
  ap.CFLAGS += -DGPS_LED=$(GPS_LED)
endif

ap.CFLAGS += -DPRIMARY_GPS_TYPE_H=\"subsystems/gps/gps_nmea.h\"
ap.srcs   += $(SRC_SUBSYSTEMS)/gps/gps_nmea.c $(SRC_SUBSYSTEMS)/gps/gps_furuno.c

$(TARGET).srcs += $(SRC_SUBSYSTEMS)/gps.c

sim.CFLAGS += -DUSE_GPS
sim.CFLAGS += -DPRIMARY_GPS_TYPE_H=\"subsystems/gps/gps_sim.h\"
sim.srcs += $(SRC_SUBSYSTEMS)/gps/gps_sim.c

nps.CFLAGS += -DUSE_GPS
nps.srcs += $(SRC_SUBSYSTEMS)/gps.c
nps.CFLAGS += -DPRIMARY_GPS_TYPE_H=\"subsystems/gps/gps_sim_nps.h\"
nps.srcs += $(SRC_SUBSYSTEMS)/gps/gps_sim_nps.c