# Hey Emacs, this is a -*- makefile -*-

ap.CFLAGS += -DUSE_GPS -DHITL
ap.CFLAGS += -DGPS_TYPE=\"subsystems/gps/gps_sim_hitl.h\"
ap.srcs += $(SRC_SUBSYSTEMS)/gps/gps_sim_hitl.c

ap.srcs += $(SRC_SUBSYSTEMS)/gps.c

ifneq ($(GPS_LED),none)
  ap.CFLAGS += -DGPS_LED=$(GPS_LED)
endif

nps.CFLAGS += -DUSE_GPS
nps.srcs += $(SRC_SUBSYSTEMS)/gps.c
nps.CFLAGS += -DGPS_TYPE=\"subsystems/gps/gps_sim_nps.h\"
nps.srcs += $(SRC_SUBSYSTEMS)/gps/gps_sim_nps.c
