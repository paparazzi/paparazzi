# Hey Emacs, this is a -*- makefile -*-

# Swift-Nav Piksi RTK module

GPS_LED ?= none

GPS_PORT_LOWER=$(shell echo $(GPS_PORT) | tr A-Z a-z)

ap.CFLAGS += -DUSE_GPS -DGPS_USE_LATLONG
ap.CFLAGS += -DUSE_$(GPS_PORT) -D$(GPS_PORT)_BAUD=B115200
ap.CFLAGS += -DGPS_LINK=$(GPS_PORT_LOWER)

ifneq ($(GPS_LED),none)
  ap.CFLAGS += -DGPS_LED=$(GPS_LED)
endif

$(TARGET).srcs += $(SRC_SUBSYSTEMS)/gps.c

ap.CFLAGS += -DGPS_TYPE_H=\"subsystems/gps/gps_piksi.h\"
ap.srcs   += $(SRC_SUBSYSTEMS)/gps/gps_piksi.c

# libswiftnav
ap.CFLAGS += -I$(PAPARAZZI_SRC)/sw/ext/libswiftnav/include
ap.srcs		+= $(PAPARAZZI_SRC)/sw/ext/libswiftnav/src/sbp.c $(PAPARAZZI_SRC)/sw/ext/libswiftnav/src/edc.c

sim.CFLAGS += -DUSE_GPS -DGPS_USE_LATLONG
sim.CFLAGS += -DGPS_TYPE_H=\"subsystems/gps/gps_sim.h\"
sim.srcs += $(SRC_SUBSYSTEMS)/gps/gps_sim.c

nps.CFLAGS += -DUSE_GPS -DGPS_USE_LATLONG
nps.CFLAGS += -DGPS_TYPE_H=\"subsystems/gps/gps_sim_nps.h\"
nps.srcs += $(SRC_SUBSYSTEMS)/gps/gps_sim_nps.c

