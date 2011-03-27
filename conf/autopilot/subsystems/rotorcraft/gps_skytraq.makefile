
ap.srcs += $(SRC_SUBSYSTEMS)/gps.c
ap.CFLAGS += -DGPS_TYPE_H=\"subsystems/gps/gps_skytraq.h\"
ap.srcs += $(SRC_SUBSYSTEMS)/gps/gps_skytraq.c

ap.CFLAGS += -DUSE_$(GPS_PORT) -D$(GPS_PORT)_BAUD=$(GPS_BAUD)
ap.CFLAGS += -DUSE_GPS -DGPS_LINK=$(GPS_PORT)

ifneq ($(GPS_LED),none)
  ap.CFLAGS += -DGPS_LED=$(GPS_LED)
endif

sim.CFLAGS += -DUSE_GPS
sim.srcs += $(SRC_SUBSYSTEMS)/gps.c
