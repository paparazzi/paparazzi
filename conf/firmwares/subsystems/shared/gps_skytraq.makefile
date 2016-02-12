# Hey Emacs, this is a -*- makefile -*-

GPS_LED ?= none
SKYTRAQ_GPS_PORT ?= $(GPS_PORT)
SKYTRAQ_GPS_BAUD ?= $(GPS_BAUD)

SKYTRAQ_GPS_PORT_LOWER=$(shell echo $(SKYTRAQ_GPS_PORT) | tr A-Z a-z)

ap.CFLAGS += -DUSE_GPS
ap.CFLAGS += -DSKYTRAQ_GPS_LINK=$(SKYTRAQ_SKYTRAQ_GPS_PORT_LOWER)
ap.CFLAGS += -DUSE_$(SKYTRAQ_GPS_PORT)
ap.CFLAGS += -D$(SKYTRAQ_GPS_PORT)_BAUD=$(SKYTRAQ_GPS_BAUD)

ifneq ($(GPS_LED),none)
  ap.CFLAGS += -DGPS_LED=$(GPS_LED)
endif

ifdef SECONDARY_GPS
ifneq (,$(findstring $(SECONDARY_GPS), skytraq))
# this is the secondary GPS
ap.CFLAGS += -DGPS_SECONDARY_TYPE_H=\"subsystems/gps/gps_skytraq.h\"
ap.CFLAGS += -DSECONDARY_GPS=gps_skytraq
else
ap.CFLAGS += -DGPS_TYPE_H=\"subsystems/gps/gps_skytraq.h\"
ap.CFLAGS += -DPRIMARY_GPS=gps_skytraq
endif
else
# plain old single GPS usage
ap.CFLAGS += -DGPS_TYPE_H=\"subsystems/gps/gps_skytraq.h\"
endif

ap.srcs += $(SRC_SUBSYSTEMS)/gps/gps_skytraq.c
ap.srcs += $(SRC_SUBSYSTEMS)/gps.c

sim.CFLAGS += -DUSE_GPS
sim.CFLAGS += -DGPS_TYPE_H=\"subsystems/gps/gps_sim.h\"
sim.srcs += $(SRC_SUBSYSTEMS)/gps/gps_sim.c
sim.srcs += $(SRC_SUBSYSTEMS)/gps.c

nps.CFLAGS += -DUSE_GPS
nps.CFLAGS += -DGPS_TYPE_H=\"subsystems/gps/gps_sim_nps.h\"
nps.srcs += $(SRC_SUBSYSTEMS)/gps/gps_sim_nps.c
nps.srcs += $(SRC_SUBSYSTEMS)/gps.c
