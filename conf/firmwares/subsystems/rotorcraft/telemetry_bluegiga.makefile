#
# The bluegiga module as telemetry downlink/uplink for rotorcraft
#

ifeq ($(TARGET), ap)
include $(CFG_SHARED)/telemetry_bluegiga.makefile
endif

ap.srcs += $(SRC_FIRMWARE)/datalink.c $(SRC_FIRMWARE)/rotorcraft_telemetry.c
