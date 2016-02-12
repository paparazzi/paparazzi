# Hey Emacs, this is a -*- makefile -*-

# Mediatek MT3329, DIYDrones V1.4/1.6 protocol

GPS_LED ?= none
MTK_GPS_PORT ?= $(GPS_PORT)
MTK_GPS_BAUD ?= $(GPS_BAUD)

MTK_GPS_PORT_LOWER=$(shell echo $(MTK_GPS_PORT) | tr A-Z a-z)

ap.CFLAGS += -DUSE_GPS -DGPS_CONFIGURE
ap.CFLAGS += -DMTK_GPS_LINK=$(MTK_GPS_PORT_LOWER)
ap.CFLAGS += -DUSE_$(MTK_GPS_PORT)
ap.CFLAGS += -D$(MTK_GPS_PORT)_BAUD=$(MTK_GPS_BAUD)

ifneq ($(GPS_LED),none)
  ap.CFLAGS += -DGPS_LED=$(GPS_LED)
endif

ifdef SECONDARY_GPS
ifneq (,$(findstring $(SECONDARY_GPS), mtk mediatek))
# this is the secondary GPS
ap.CFLAGS += -DGPS_SECONDARY_TYPE_H=\"subsystems/gps/gps_mtk.h\"
ap.CFLAGS += -DSECONDARY_GPS=gps_mtk
else
ap.CFLAGS += -DGPS_TYPE_H=\"subsystems/gps/gps_mtk.h\"
ap.CFLAGS += -DPRIMARY_GPS=gps_mtk
endif
else
# plain old single GPS usage
ap.CFLAGS += -DGPS_TYPE_H=\"subsystems/gps/gps_mtk.h\"
endif

ap.srcs   += $(SRC_SUBSYSTEMS)/gps/gps_mtk.c
ap.srcs += $(SRC_SUBSYSTEMS)/gps.c

sim.CFLAGS += -DUSE_GPS
sim.CFLAGS += -DGPS_TYPE_H=\"subsystems/gps/gps_sim.h\"
sim.srcs += $(SRC_SUBSYSTEMS)/gps/gps_sim.c
sim.srcs += $(SRC_SUBSYSTEMS)/gps.c

nps.CFLAGS += -DUSE_GPS
nps.CFLAGS += -DGPS_TYPE_H=\"subsystems/gps/gps_sim_nps.h\"
nps.srcs += $(SRC_SUBSYSTEMS)/gps/gps_sim_nps.c
nps.srcs += $(SRC_SUBSYSTEMS)/gps.c
