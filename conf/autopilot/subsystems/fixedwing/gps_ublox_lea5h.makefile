# UBlox LEA 5H


ap.CFLAGS += -DUSE_GPS -DUBX -DGPS_USE_LATLONG
ap.CFLAGS += -DGPS_LINK=$(GPS_PORT)
ap.CFLAGS += -DUSE_$(GPS_PORT)
ap.CFLAGS += -D$(GPS_PORT)_BAUD=$(GPS_BAUD)

ifneq ($(GPS_LED),none)
  ap.CFLAGS += -DGPS_LED=$(GPS_LED)
endif

ap.srcs   += $(SRC_FIXEDWING)/gps_ubx.c

$(TARGET).srcs += $(SRC_FIXEDWING)/gps.c $(SRC_FIXEDWING)/latlong.c
