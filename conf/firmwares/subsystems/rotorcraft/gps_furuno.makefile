# Hey Emacs, this is a -*- makefile -*-

# Furuno NMEA GPS unit


ap.CFLAGS += -DUSE_GPS
ap.CFLAGS += -DGPS_LINK=$(GPS_PORT)
ap.CFLAGS += -DUSE_$(GPS_PORT)
ap.CFLAGS += -D$(GPS_PORT)_BAUD=$(GPS_BAUD)
ap.CFLAGS += -DNMEA_PARSE_PROP

ifneq ($(GPS_LED),none)
  ap.CFLAGS += -DGPS_LED=$(GPS_LED)
endif

ap.CFLAGS += -DGPS_TYPE_H=\"subsystems/gps/gps_nmea.h\"
ap.srcs   += $(SRC_SUBSYSTEMS)/gps/gps_nmea.c $(SRC_SUBSYSTEMS)/gps/gps_furuno.c

$(TARGET).srcs += $(SRC_SUBSYSTEMS)/gps.c

nps.CFLAGS += -DUSE_GPS
nps.CFLAGS += -DGPS_TYPE_H=\"subsystems/gps/gps_sim.h\"
nps.srcs += $(SRC_SUBSYSTEMS)/gps/gps_sim_nps.c

