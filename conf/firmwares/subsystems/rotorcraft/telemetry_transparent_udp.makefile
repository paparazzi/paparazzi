
#serial UDP

ifeq ($(TARGET), ap)
include $(CFG_SHARED)/telemetry_transparent_udp.makefile
endif

ap.srcs += subsystems/datalink/datalink.c $(SRC_FIRMWARE)/rotorcraft_datalink.c $(SRC_FIRMWARE)/rotorcraft_telemetry.c
