#
# The bluegiga module as telemetry downlink/uplink for rotorcraft
#

ifeq ($(TARGET), ap)
include $(CFG_SHARED)/telemetry_bluegiga.makefile
endif

ap.srcs += subsystems/datalink/datalink.c
ap.srcs += $(SRC_FIRMWARE)/rotorcraft_datalink.c $(SRC_FIRMWARE)/rotorcraft_telemetry.c
