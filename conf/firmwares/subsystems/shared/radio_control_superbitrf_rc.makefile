#
# Makefile for shared radio_control superbitrf subsystem
#

RADIO_CONTROL_LED ?= none

RC_CFLAGS += -DRADIO_CONTROL -DRADIO_CONTROL_TYPE_SUPERBITRF -DRADIO_CONTROL_TYPE_H=\"subsystems/radio_control/superbitrf_rc.h\"
RC_CFLAGS += -DUSE_SUPERBITRF -DUSE_SPI2 -DUSE_SPI_SLAVE2

ifneq ($(RADIO_CONTROL_LED),none)
RC_CFLAGS += -DRADIO_CONTROL_LED=$(RADIO_CONTROL_LED)
endif

RC_SRCS += peripherals/cyrf6936.c \
		   $(SRC_SUBSYSTEMS)/datalink/superbitrf.c\
           $(SRC_SUBSYSTEMS)/radio_control.c \
           $(SRC_SUBSYSTEMS)/radio_control/superbitrf_rc.c

ap.CFLAGS += $(RC_CFLAGS)
ap.srcs   += $(RC_SRCS)

test_radio_control.CFLAGS += $(RC_CFLAGS)
test_radio_control.srcs   += $(RC_SRCS)
