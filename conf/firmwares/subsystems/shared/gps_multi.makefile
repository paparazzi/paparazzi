# Hey Emacs, this is a -*- makefile -*-

GPS_LED ?= none
GPS_PRIMARY_PORT_LOWER=$(shell echo $(GPS_PRIMARY_PORT) | tr A-Z a-z)
GPS_SECONDARY_PORT_LOWER=$(shell echo $(GPS_SECONDARY_PORT) | tr A-Z a-z)

ap.CFLAGS += -DUSE_GPS
ap.CFLAGS += -DUSE_MULTI_GPS
ap.CFLAGS += -DUSE_$(GPS_PRIMARY_PORT) -D$(GPS_PRIMARY_PORT)_BAUD=$(GPS_PRIMARY_BAUD)
ap.CFLAGS += -DUSE_$(GPS_SECONDARY_PORT) -D$(GPS_SECONDARY_PORT)_BAUD=$(GPS_SECONDARY_BAUD)
ap.CFLAGS += -DGPS_PRIMARY_PORT=$(GPS_PRIMARY_PORT_LOWER) 
ap.CFLAGS += -DGPS_SECONDARY_PORT=$(GPS_SECONDARY_PORT_LOWER)

ap.CFLAGS += -DGPS_TYPE_H=\"subsystems/gps/gps_multi.h\"

ap.CFLAGS += -DSECONDARY_GPS_TYPE_H=\"subsystems/gps/gps_multi/gps_multi_piksi.h\"
ap.CFLAGS += -DPRIMARY_GPS_TYPE_H=\"subsystems/gps/gps_multi/gps_multi_ubx.h\"

ap.srcs += $(SRC_SUBSYSTEMS)/gps/gps_multi/gps_multi_piksi.c
ap.srcs += $(SRC_SUBSYSTEMS)/gps/gps_multi/gps_multi_ubx.c
ap.srcs += $(SRC_SUBSYSTEMS)/gps/gps_multi.c
ap.srcs += $(SRC_SUBSYSTEMS)/gps.c

ap.CFLAGS += -DGPS_PRIMARY_$(GPS_PRIMARY_TYPE)
ap.CFLAGS += -DGPS_SECONDARY_$(GPS_SECONDARY_TYPE)

ifeq (PIKSI,$(filter PIKSI,$(GPS_PRIMARY_TYPE) $(GPS_SECONDARY_TYPE)))
  # libsbp
  ap.CFLAGS += -I$(PAPARAZZI_SRC)/sw/ext/libsbp/c/include
  ap.srcs		+= $(PAPARAZZI_SRC)/sw/ext/libsbp/c/src/sbp.c $(PAPARAZZI_SRC)/sw/ext/libsbp/c/src/edc.c
endif

ifneq ($(GPS_LED),none)
  ap.CFLAGS += -DGPS_LED=$(GPS_LED)
endif

nps.CFLAGS += -DUSE_GPS
nps.srcs += $(SRC_SUBSYSTEMS)/gps.c
nps.CFLAGS += -DGPS_TYPE_H=\"subsystems/gps/gps_sim_nps.h\"
nps.srcs += $(SRC_SUBSYSTEMS)/gps/gps_sim_nps.c
