# Hey Emacs, this is a -*- makefile -*-


NORADIO = False

ifeq ($(BOARD),classix)
  ifeq ($(TARGET),ap)
    NORADIO = True
  endif
endif

RADIO_CONTROL_DATALINK_LED ?= none
RADIO_CONTROL_LED ?= none

ifeq ($(NORADIO), False)
  ifneq ($(RADIO_CONTROL_DATALINK_LED),none)
    RC_CFLAGS += -DRADIO_CONTROL_DATALINK_LED=$(RADIO_CONTROL_DATALINK_LED)
  endif
  ifneq ($(RADIO_CONTROL_LED),none)
    RC_CFLAGS += -DRADIO_CONTROL_LED=$(RADIO_CONTROL_LED)
    fbw.CFLAGS += -DRADIO_CONTROL_LED=$(RADIO_CONTROL_LED)
  endif
  $(TARGET).CFLAGS += -DRADIO_CONTROL
  $(TARGET).CFLAGS += -DRADIO_CONTROL_TYPE_H=\"radio_control/rc_datalink.h\"
  $(TARGET).CFLAGS += -DRADIO_CONTROL_TYPE_DATALINK
  $(TARGET).srcs   += $(SRC_SUBSYSTEMS)/radio_control.c
  $(TARGET).srcs   += $(SRC_SUBSYSTEMS)/radio_control/rc_datalink.c
# arch only with sim target for compatibility (empty functions)
  sim.srcs += $(SRC_ARCH)/subsystems/radio_control/rc_datalink.c

  ap.CFLAGS += $(RC_CFLAGS)
  ap.srcs   += $(RC_SRCS)

  test_radio_control.CFLAGS += $(RC_CFLAGS)
  test_radio_control.srcs   += $(RC_SRCS)
endif
