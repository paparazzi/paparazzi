# Hey Emacs, this is a -*- makefile -*-

# Swift-Nav Piksi RTK module

GPS_LED ?= none
PIKSI_GPS_PORT ?= $(GPS_PORT)
PIKSI_GPS_BAUD ?= $(GPS_BAUD)

PIKSI_GPS_PORT_LOWER=$(shell echo $(PIKSI_GPS_PORT) | tr A-Z a-z)

ap.CFLAGS += -DUSE_GPS
ap.CFLAGS += -DUSE_$(PIKSI_GPS_PORT) -D$(PIKSI_GPS_PORT)_BAUD=B115200
ap.CFLAGS += -DPIKSI_GPS_LINK=$(PIKSI_GPS_PORT_LOWER)

ifneq ($(GPS_LED),none)
  ap.CFLAGS += -DGPS_LED=$(GPS_LED)
endif

ifdef SECONDARY_GPS
ifneq (,$(findstring $(SECONDARY_GPS), piksi))
# this is the secondary GPS
ap.CFLAGS += -DGPS_SECONDARY_TYPE_H=\"subsystems/gps/gps_piksi.h\"
ap.CFLAGS += -DSECONDARY_GPS=gps_piksi
else
ap.CFLAGS += -DGPS_TYPE_H=\"subsystems/gps/gps_piksi.h\"
ap.CFLAGS += -DPRIMARY_GPS=gps_piksi
endif
else
# plain old single GPS usage
ap.CFLAGS += -DGPS_TYPE_H=\"subsystems/gps/gps_piksi.h\"
endif

ap.srcs += $(SRC_SUBSYSTEMS)/gps.c
ap.srcs += $(SRC_SUBSYSTEMS)/gps/gps_piksi.c

# libsbp
ap.CFLAGS += -I$(PAPARAZZI_SRC)/sw/ext/libsbp/c/include
ap.srcs		+= $(PAPARAZZI_SRC)/sw/ext/libsbp/c/src/sbp.c $(PAPARAZZI_SRC)/sw/ext/libsbp/c/src/edc.c

sim.CFLAGS += -DUSE_GPS
sim.CFLAGS += -DGPS_TYPE_H=\"subsystems/gps/gps_sim.h\"
sim.srcs += $(SRC_SUBSYSTEMS)/gps/gps_sim.c
sim.srcs += $(SRC_SUBSYSTEMS)/gps.c

nps.CFLAGS += -DUSE_GPS
nps.CFLAGS += -DGPS_TYPE_H=\"subsystems/gps/gps_sim_nps.h\"
nps.srcs += $(SRC_SUBSYSTEMS)/gps/gps_sim_nps.c
nps.srcs += $(SRC_SUBSYSTEMS)/gps.c
