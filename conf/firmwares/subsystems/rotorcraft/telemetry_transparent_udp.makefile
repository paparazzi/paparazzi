
#serial UDP

ifeq ($(TARGET), ap)
include $(CFG_SHARED)/telemetry_transparent_udp.makefile
endif

ap.srcs += $(SRC_FIRMWARE)/datalink.c $(SRC_FIRMWARE)/rotorcraft_telemetry.c
