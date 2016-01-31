
#serial USB (e.g. /dev/ttyACM0)

ifeq ($(TARGET), ap)
include $(CFG_SHARED)/telemetry_transparent_usb.makefile
endif

ap.srcs += subsystems/datalink/datalink.c
ap.srcs += $(SRC_FIRMWARE)/rotorcraft_datalink.c $(SRC_FIRMWARE)/rotorcraft_telemetry.c
