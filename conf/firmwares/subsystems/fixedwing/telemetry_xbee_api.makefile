# Hey Emacs, this is a -*- makefile -*-

# XBee modems in API mode

# include shared part for ap
ifeq ($(TARGET),ap)
include $(CFG_SHARED)/telemetry_xbee_api.makefile
endif

ap.srcs += $(SRC_FIRMWARE)/fixedwing_datalink.c $(SRC_FIRMWARE)/ap_downlink.c

# avoid fbw_telemetry_mode error
ap.srcs += $(SRC_FIRMWARE)/fbw_downlink.c

fbw.srcs += $(SRC_FIRMWARE)/fbw_downlink.c
