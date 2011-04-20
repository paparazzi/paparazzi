
NORADIO = False

ifeq ($(BOARD),classix)
  ifeq ($(TARGET),ap)
    NORADIO = True
  endif
endif


ifeq ($(NORADIO), False)
  $(TARGET).CFLAGS	+= -DRADIO_CONTROL
	$(TARGET).CFLAGS 	+= -DRADIO_CONTROL_TYPE_H=\"radio_control/rc_datalink.h\"
	$(TARGET).CFLAGS 	+= -DRADIO_CONTROL_TYPE_DATALINK
  $(TARGET).srcs		+= $(SRC_SUBSYSTEMS)/radio_control.c
  $(TARGET).srcs		+= $(SRC_SUBSYSTEMS)/radio_control/rc_datalink.c
# arch only with sim target for compatibility (empty functions)
	sim.srcs					+= $(SRC_ARCH)/subsystems/radio_control/rc_datalink.c
endif
