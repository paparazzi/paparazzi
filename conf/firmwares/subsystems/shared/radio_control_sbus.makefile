#
# Makefile for shared radio_control SBUS subsystem
#

RADIO_CONTROL_LED ?= none

ifneq ($(RADIO_CONTROL_LED),none)
	RC_CFLAGS += -DRADIO_CONTROL_LED=$(RADIO_CONTROL_LED)
endif

$(TARGET).CFLAGS	+= -DRADIO_CONTROL

# convert SBUS_PORT to upper and lower case strings:
SBUS_PORT_UPPER=$(shell echo $(SBUS_PORT) | tr a-z A-Z)
SBUS_PORT_LOWER=$(shell echo $(SBUS_PORT) | tr A-Z a-z)

RC_CFLAGS += -DUSE_$(SBUS_PORT_UPPER) -D$(SBUS_PORT_UPPER)_BAUD=B100000
RC_CFLAGS += -DSBUS_UART_DEV=$(SBUS_PORT_LOWER)
RC_CFLAGS += -DRADIO_CONTROL_TYPE_H=\"subsystems/radio_control/sbus.h\"
RC_CFLAGS += -DRADIO_CONTROL_TYPE_SBUS
RC_SRCS	+= $(SRC_SUBSYSTEMS)/radio_control.c
RC_SRCS	+= $(SRC_SUBSYSTEMS)/radio_control/sbus.c
RC_SRCS	+= $(SRC_SUBSYSTEMS)/radio_control/sbus_common.c

ap.CFLAGS += $(RC_CFLAGS)
ap.srcs   += $(RC_SRCS)

test_radio_control.CFLAGS += $(RC_CFLAGS)
test_radio_control.srcs   += $(RC_SRCS)
