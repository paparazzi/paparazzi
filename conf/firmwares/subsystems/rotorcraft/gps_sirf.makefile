# Hey Emacs, this is a -*- makefile -*-

# Sirf GPS unit

GPS_LED ?= none
SIRF_GPS_PORT ?= $(GPS_PORT)
SIRF_GPS_BAUD ?= $(GPS_BAUD)

SIRF_GPS_PORT_LOWER=$(shell echo $(SIRF_GPS_PORT) | tr A-Z a-z)

ap.CFLAGS += -DUSE_GPS
ap.CFLAGS += -DSIRF_GPS_LINK=$(SIRF_GPS_PORT_LOWER)
ap.CFLAGS += -DUSE_$(SIRF_GPS_PORT)
ap.CFLAGS += -D$(SIRF_GPS_PORT)_BAUD=$(SIRF_GPS_BAUD)

ifneq ($(GPS_LED),none)
  ap.CFLAGS += -DGPS_LED=$(GPS_LED)
endif

ifdef SECONDARY_GPS
ifneq (,$(findstring $(SECONDARY_GPS), sirf))
# this is the secondary GPS
ap.CFLAGS += -DGPS_SECONDARY_TYPE_H=\"subsystems/gps/gps_sirf.h\"
ap.CFLAGS += -DSECONDARY_GPS=gps_sirf
else
ap.CFLAGS += -DGPS_TYPE_H=\"subsystems/gps/gps_sirf.h\"
ap.CFLAGS += -DPRIMARY_GPS=gps_sirf
endif
else
# plain old single GPS usage
ap.CFLAGS += -DGPS_TYPE_H=\"subsystems/gps/gps_sirf.h\"
endif

ap.srcs += $(SRC_SUBSYSTEMS)/gps/gps_sirf.c
ap.srcs += $(SRC_SUBSYSTEMS)/gps.c

nps.CFLAGS += -DUSE_GPS
nps.CFLAGS += -DGPS_TYPE_H=\"subsystems/gps/gps_sim_nps.h\"
nps.srcs += $(SRC_SUBSYSTEMS)/gps/gps_sim_nps.c
nps.srcs += $(SRC_SUBSYSTEMS)/gps.c
