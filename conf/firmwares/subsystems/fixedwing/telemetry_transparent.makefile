# Hey Emacs, this is a -*- makefile -*-

# include shared part for ap
ifeq ($(TARGET),ap)
include $(CFG_SHARED)/telemetry_transparent.makefile
endif

ap.srcs += $(SRC_FIRMWARE)/datalink.c $(SRC_FIRMWARE)/ap_downlink.c

# avoid fbw_telemetry_mode error
ap.srcs += $(SRC_FIRMWARE)/fbw_downlink.c

fbw.srcs += $(SRC_FIRMWARE)/fbw_downlink.c

