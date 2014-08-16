#
# Makefile for shared radio_control ppm subsystem
#

NORADIO = False
RADIO_CONTROL_LED ?= none

ifeq ($(BOARD),classix)
  ifeq ($(TARGET),ap)
    NORADIO = True
  endif
endif

ifeq ($(NORADIO), False)
  $(TARGET).CFLAGS	+= -DRADIO_CONTROL
  ifneq ($(RADIO_CONTROL_LED),none)
    ap.CFLAGS += -DRADIO_CONTROL_LED=$(RADIO_CONTROL_LED)
    fbw.CFLAGS += -DRADIO_CONTROL_LED=$(RADIO_CONTROL_LED)
    test_radio_control.CFLAGS += -DRADIO_CONTROL_LED=$(RADIO_CONTROL_LED)
  endif
  $(TARGET).CFLAGS 	+= -DRADIO_CONTROL_TYPE_H=\"subsystems/radio_control/ppm.h\"
  $(TARGET).CFLAGS 	+= -DRADIO_CONTROL_TYPE_PPM
  $(TARGET).srcs	+= $(SRC_SUBSYSTEMS)/radio_control.c
  $(TARGET).srcs	+= $(SRC_SUBSYSTEMS)/radio_control/ppm.c
  $(TARGET).srcs 	+= $(SRC_ARCH)/subsystems/radio_control/ppm_arch.c

  ifeq ($(ARCH),stm32)
    ifdef RADIO_CONTROL_PPM_PIN
      ifeq ($(RADIO_CONTROL_PPM_PIN),$(filter $(RADIO_CONTROL_PPM_PIN),PA_10 UART1_RX))
        $(TARGET).CFLAGS += -DPPM_CONFIG=1
      else ifeq ($(RADIO_CONTROL_PPM_PIN),$(filter $(RADIO_CONTROL_PPM_PIN),PA_01 SERVO6))
        $(TARGET).CFLAGS += -DPPM_CONFIG=2
      endif
    endif
  endif
endif

# dummy stuff so you don't have to unload superbitrf.xml settings file for simulators
nps.srcs += $(SRC_ARCH)/subsystems/datalink/superbitrf.c
sim.srcs += $(SRC_ARCH)/subsystems/datalink/superbitrf.c
