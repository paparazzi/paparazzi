#
# Makefile for shared radio_control SBUS subsystem
#

$(TARGET).CFLAGS	+= -DRADIO_CONTROL
ifneq ($(RADIO_CONTROL_LED),none)
	ap.CFLAGS += -DRADIO_CONTROL_LED=$(RADIO_CONTROL_LED)
endif
$(TARGET).CFLAGS += -DUSE_$(SBUS_PORT) -DSBUS_LINK=$(SBUS_PORT) -D$(SBUS_PORT)_BAUD=B100000
$(TARGET).CFLAGS 	+= -DRADIO_CONTROL_TYPE_H=\"subsystems/radio_control/sbus.h\"
$(TARGET).CFLAGS 	+= -DRADIO_CONTROL_TYPE_SBUS
$(TARGET).srcs	+= $(SRC_SUBSYSTEMS)/radio_control.c
$(TARGET).srcs	+= $(SRC_SUBSYSTEMS)/radio_control/sbus.c

