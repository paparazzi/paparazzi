#
# Makefile for shared radio_control SBUS subsystem
#

RADIO_CONTROL_LED ?= none

ifneq ($(RADIO_CONTROL_LED),none)
	RC_CFLAGS += -DRADIO_CONTROL_LED=$(RADIO_CONTROL_LED)
endif

RC_CFLAGS += -DRADIO_CONTROL_TYPE_H=\"subsystems/radio_control/sbus.h\"

RC_FBW_CFLAGS += -DRADIO_CONTROL

# convert SBUS_PORT to upper and lower case strings:
SBUS_PORT_UPPER=$(shell echo $(SBUS_PORT) | tr a-z A-Z)
SBUS_PORT_LOWER=$(shell echo $(SBUS_PORT) | tr A-Z a-z)

RC_FBW_CFLAGS += -DUSE_$(SBUS_PORT_UPPER) -D$(SBUS_PORT_UPPER)_BAUD=B100000
RC_FBW_CFLAGS += -DSBUS_UART_DEV=$(SBUS_PORT_LOWER)
RC_FBW_CFLAGS += -DRADIO_CONTROL_TYPE_SBUS
RC_SRCS	+= $(SRC_SUBSYSTEMS)/radio_control.c
RC_SRCS	+= $(SRC_SUBSYSTEMS)/radio_control/sbus.c
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
