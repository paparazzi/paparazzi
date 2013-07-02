#
# Makefile for shared radio_control superbitrf subsystem
#

ap.CFLAGS += -DRADIO_CONTROL -DRADIO_CONTROL_TYPE_SUPERBITRF -DRADIO_CONTROL_TYPE_H=\"subsystems/radio_control/superbitrf.h\"
ap.CFLAGS += -DUSE_SPI1 -DUSE_SPI_SLAVE1

ifneq ($(RADIO_CONTROL_LED),none)
ap.CFLAGS += -DRADIO_CONTROL_LED=$(RADIO_CONTROL_LED)
endif

ap.srcs += peripherals/cyrf6936.c \
           $(SRC_SUBSYSTEMS)/radio_control.c \
           $(SRC_SUBSYSTEMS)/radio_control/superbitrf.c 
