# Hey Emacs, this is a -*- makefile -*-

# Mediatek MT3329, DIYDrones V1.4/1.6 protocol

GPS_LED ?= none
MTK_GPS_PORT_LOWER=$(shell echo $(GPS_PORT) | tr A-Z a-z)

ap.CFLAGS += -DUSE_GPS -DGPS_CONFIGURE -DGPS_USE_LATLONG
ap.CFLAGS += -DGPS_LINK=$(MTK_GPS_PORT_LOWER)
ap.CFLAGS += -DUSE_$(GPS_PORT)
ap.CFLAGS += -D$(GPS_PORT)_BAUD=$(GPS_BAUD)

ifneq ($(GPS_LED),none)
  ap.CFLAGS += -DGPS_LED=$(GPS_LED)
endif

ap.CFLAGS += -DGPS_TYPE_H=\"subsystems/gps/gps_mtk.h\"
ap.srcs   += $(SRC_SUBSYSTEMS)/gps/gps_mtk.c

$(TARGET).srcs += $(SRC_SUBSYSTEMS)/gps.c

sim.CFLAGS += -DUSE_GPS -DGPS_USE_LATLONG
sim.CFLAGS += -DGPS_TYPE_H=\"subsystems/gps/gps_sim.h\"
sim.srcs += $(SRC_SUBSYSTEMS)/gps/gps_sim.c

nps.CFLAGS += -DUSE_GPS -DGPS_USE_LATLONG
nps.srcs += $(SRC_SUBSYSTEMS)/gps.c
nps.CFLAGS += -DGPS_TYPE_H=\"subsystems/gps/gps_sim_nps.h\"
nps.srcs += $(SRC_SUBSYSTEMS)/gps/gps_sim_nps.c
