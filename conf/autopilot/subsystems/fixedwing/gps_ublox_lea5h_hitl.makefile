# UBlox LEA 5H


ap.CFLAGS += -DUSE_GPS -DUBX -DGPS_USE_LATLONG

ap.srcs   += $(SRC_FIXEDWING)/gps_ubx.c $(SRC_FIXEDWING)/gps.c $(SRC_FIXEDWING)/latlong.c
