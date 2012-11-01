# Hey Emacs, this is a -*- makefile -*-

# W5100 ethernet chip.

ap.CFLAGS += -DDOWNLINK -DDOWNLINK_FBW_DEVICE=W5100 -DDOWNLINK_AP_DEVICE=W5100 -DUSE_SPI1
ap.CFLAGS += -DDOWNLINK_TRANSPORT=W5100Transport -DDATALINK=W5100
ap.srcs += subsystems/datalink/downlink.c subsystems/datalink/w5100.c
ap.srcs += $(SRC_FIRMWARE)/datalink.c

