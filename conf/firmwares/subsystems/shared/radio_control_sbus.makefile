#
# Makefile for shared radio_control SBUS subsystem
#

RADIO_CONTROL_LED ?= none

ifneq ($(RADIO_CONTROL_LED),none)
	ap.CFLAGS += -DRADIO_CONTROL_LED=$(RADIO_CONTROL_LED)
endif

$(TARGET).CFLAGS	+= -DRADIO_CONTROL

# convert SBUS_PORT to upper and lower case strings:
SBUS_PORT_UPPER=$(shell echo $(SBUS_PORT) | tr a-z A-Z)
SBUS_PORT_LOWER=$(shell echo $(SBUS_PORT) | tr A-Z a-z)

$(TARGET).CFLAGS += -DUSE_$(SBUS_PORT_UPPER) -D$(SBUS_PORT_UPPER)_BAUD=B100000
$(TARGET).CFLAGS += -DSBUS_UART_DEV=$(SBUS_PORT_LOWER)
$(TARGET).CFLAGS += -DRADIO_CONTROL_TYPE_H=\"subsystems/radio_control/sbus.h\"
$(TARGET).CFLAGS += -DRADIO_CONTROL_TYPE_SBUS
$(TARGET).srcs	+= $(SRC_SUBSYSTEMS)/radio_control.c
$(TARGET).srcs	+= $(SRC_SUBSYSTEMS)/radio_control/sbus.c

