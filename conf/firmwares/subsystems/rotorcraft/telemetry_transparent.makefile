#
# Expected from board file or overriden as xml param :
#
# MODEM_PORT
# MODEM_BAUD
#

ifeq ($(TARGET), ap)
include $(CFG_SHARED)/telemetry_transparent.makefile
endif

ap.srcs += $(SRC_FIRMWARE)/rotorcraft_datalink.c $(SRC_FIRMWARE)/rotorcraft_telemetry.c
