#
# The superbitRF module as telemetry downlink/uplink
#

# include generic part
ifeq ($(TARGET), ap)
include $(CFG_SHARED)/telemetry_superbitrf.makefile
endif

# add rotorcraft specific files
ap.srcs += $(SRC_FIRMWARE)/datalink.c $(SRC_FIRMWARE)/rotorcraft_telemetry.c
