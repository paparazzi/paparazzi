# Hey Emacs, this is a -*- makefile -*-

#serial USB (e.g. /dev/ttyACM0)

ifeq ($(TARGET), ap)
include $(CFG_SHARED)/telemetry_transparent_usb.makefile
endif

ap.srcs += $(SRC_FIRMWARE)/datalink.c $(SRC_FIRMWARE)/ap_downlink.c $(SRC_FIRMWARE)/fbw_downlink.c
# avoid fbw_telemetry_mode error
ap.srcs += $(SRC_FIRMWARE)/fbw_downlink.c

fbw.srcs += $(SRC_FIRMWARE)/fbw_downlink.c

