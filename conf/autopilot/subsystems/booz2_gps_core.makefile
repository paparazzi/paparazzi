
ap.srcs += $(SRC_BOOZ)/booz2_gps.c
ap.CFLAGS += -DUSE_GPS -DGPS_LINK=gps0

#sim.CFLAGS += -DUSE_GPS
#sim.srcs += $(SRC_BOOZ)/booz2_gps.c
