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

RC_CFLAGS += -DRADIO_CONTROL_TYPE_H=\"subsystems/radio_control/sbus_dual.h\"

RC_FBW_CFLAGS += -DRADIO_CONTROL

# convert SBUS_PORT to upper and lower case strings:
SBUS1_PORT_UPPER=$(shell echo $(SBUS1_PORT) | tr a-z A-Z)
SBUS1_PORT_LOWER=$(shell echo $(SBUS1_PORT) | tr A-Z a-z)
SBUS2_PORT_UPPER=$(shell echo $(SBUS2_PORT) | tr a-z A-Z)
SBUS2_PORT_LOWER=$(shell echo $(SBUS2_PORT) | tr A-Z a-z)

RC_FBW_CFLAGS += -DUSE_$(SBUS1_PORT_UPPER) -D$(SBUS1_PORT_UPPER)_BAUD=B100000
RC_FBW_CFLAGS += -DUSE_$(SBUS2_PORT_UPPER) -D$(SBUS2_PORT_UPPER)_BAUD=B100000
RC_FBW_CFLAGS += -DSBUS1_UART_DEV=$(SBUS1_PORT_LOWER)
RC_FBW_CFLAGS += -DSBUS2_UART_DEV=$(SBUS2_PORT_LOWER)
RC_FBW_CFLAGS += -DRADIO_CONTROL_TYPE_SBUS
RC_SRCS	+= $(SRC_SUBSYSTEMS)/radio_control.c
RC_SRCS	+= $(SRC_SUBSYSTEMS)/radio_control/sbus_dual.c
RC_SRCS	+= $(SRC_SUBSYSTEMS)/radio_control/sbus_common.c


ifeq (,$(findstring $(SEPARATE_FBW),1 TRUE))
# Single MCU's run RC on ap target
$(TARGET).CFLAGS += $(RC_CFLAGS) $(RC_FBW_CFLAGS)
$(TARGET).srcs   += $(RC_SRCS)
else
# Dual MCU case
fbw.CFLAGS += $(RC_CFLAGS) $(RC_FBW_CFLAGS)
fbw.srcs   += $(RC_SRCS)
# define RADIO_CONTROL_TYPE for ap in dual_mcu case to get defines
# but don't add source files
ap.CFLAGS += $(RC_CFLAGS)
endif
