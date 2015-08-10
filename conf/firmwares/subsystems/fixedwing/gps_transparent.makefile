# GPS Transparent
# 2014, Michal Podhradsky, michal.podhradsky@aggiemail.usu.edu
# Utah State University, http://aggieair.usu.edu/
GPS_LED ?= none

ap.srcs += $(SRC_SUBSYSTEMS)/gps.c
ap.CFLAGS += -DGPS_TYPE_H=\"subsystems/gps/gps_transparent.h\"
ap.srcs += $(SRC_SUBSYSTEMS)/gps/gps_transparent.c
ap.CFLAGS += -DUSE_GPS

ifneq ($(GPS_LED),none)
  ap.CFLAGS += -DGPS_LED=$(GPS_LED)
endif
