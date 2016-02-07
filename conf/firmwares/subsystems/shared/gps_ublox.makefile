# Hey Emacs, this is a -*- makefile -*-
# UBlox LEA

GPS_LED ?= none
UBX_GPS_PORT ?= $(GPS_PORT)
UBX_GPS_BAUD ?= $(GPS_BAUD)

UBX_GPS_PORT_LOWER=$(shell echo $(UBX_GPS_PORT) | tr A-Z a-z)

ap.CFLAGS += -DUSE_GPS -DUBX
ap.CFLAGS += -DUBX_GPS_LINK=$(UBX_GPS_PORT_LOWER)
ap.CFLAGS += -DUSE_$(UBX_GPS_PORT)
ap.CFLAGS += -D$(UBX_GPS_PORT)_BAUD=$(UBX_GPS_BAUD)

ifneq ($(GPS_LED),none)
  ap.CFLAGS += -DGPS_LED=$(GPS_LED)
endif

ifdef SECONDARY_GPS
ifneq (,$(findstring $(SECONDARY_GPS), ublox))
# this is the secondary GPS
ap.CFLAGS += -DGPS_SECONDARY_TYPE_H=\"subsystems/gps/gps_ubx.h\"
ap.CFLAGS += -DSECONDARY_GPS=gps_ubx
else
ap.CFLAGS += -DGPS_TYPE_H=\"subsystems/gps/gps_ubx.h\"
ap.CFLAGS += -DPRIMARY_GPS=gps_ubx
endif
else
# plain old single GPS usage
ap.CFLAGS += -DGPS_TYPE_H=\"subsystems/gps/gps_ubx.h\"
endif

ap.srcs += $(SRC_SUBSYSTEMS)/gps/gps_ubx.c
ap.srcs += $(SRC_SUBSYSTEMS)/gps.c

sim.CFLAGS += -DUSE_GPS
sim.CFLAGS += -DGPS_TYPE_H=\"subsystems/gps/gps_sim.h\"
sim.srcs += $(SRC_SUBSYSTEMS)/gps/gps_sim.c
sim.srcs += $(SRC_SUBSYSTEMS)/gps.c

nps.CFLAGS += -DUSE_GPS
nps.CFLAGS += -DGPS_TYPE_H=\"subsystems/gps/gps_sim_nps.h\"
nps.srcs += $(SRC_SUBSYSTEMS)/gps/gps_sim_nps.c
nps.srcs += $(SRC_SUBSYSTEMS)/gps.c
