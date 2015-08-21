#
# Expected from board file or overriden as xml param :
#

ifeq ($(TARGET), ap)
include $(CFG_SHARED)/telemetry_bluegiga.makefile
endif

ap.srcs += $(SRC_FIRMWARE)/rotorcraft_telemetry.c
