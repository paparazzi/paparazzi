#
# Makefile for shared radio_control SBUS subsystem
#

# Uart polarity needs to be reversed:
# define RC_POLARITY_GPIO_PORT <SET_TO_SBUS_RX_PIN_GPIO>
# define RC_POLARITY_GPIO_PIN  <SET_TO_SBUS_RX_PIN_GPIO_PIN>

#

RADIO_CONTROL_LED ?= none

ifneq ($(RADIO_CONTROL_LED),none)
	ap.CFLAGS += -DRADIO_CONTROL_LED=$(RADIO_CONTROL_LED)
endif

$(TARGET).CFLAGS	+= -DRADIO_CONTROL


# convert SBUS_PORT to upper and lower case strings:
SBUS1_PORT_UPPER=$(shell echo $(SBUS1_PORT) | tr a-z A-Z)
SBUS1_PORT_LOWER=$(shell echo $(SBUS1_PORT) | tr A-Z a-z)
SBUS2_PORT_UPPER=$(shell echo $(SBUS2_PORT) | tr a-z A-Z)
SBUS2_PORT_LOWER=$(shell echo $(SBUS2_PORT) | tr A-Z a-z)

$(TARGET).CFLAGS += -DUSE_$(SBUS1_PORT_UPPER) -D$(SBUS1_PORT_UPPER)_BAUD=B100000
$(TARGET).CFLAGS += -DUSE_$(SBUS2_PORT_UPPER) -D$(SBUS2_PORT_UPPER)_BAUD=B100000
$(TARGET).CFLAGS += -DSBUS1_UART_DEV=$(SBUS1_PORT_LOWER)
$(TARGET).CFLAGS += -DSBUS2_UART_DEV=$(SBUS2_PORT_LOWER)
$(TARGET).CFLAGS += -DRADIO_CONTROL_TYPE_H=\"subsystems/radio_control/sbus_dual.h\"
$(TARGET).CFLAGS += -DRADIO_CONTROL_TYPE_SBUS
$(TARGET).srcs	+= $(SRC_SUBSYSTEMS)/radio_control.c
$(TARGET).srcs	+= $(SRC_SUBSYSTEMS)/radio_control/sbus_dual.c
$(TARGET).srcs	+= $(SRC_SUBSYSTEMS)/radio_control/sbus_common.c

