#
# Makefile for shared radio_control ppm susbsytem
#

NORADIO = False

ifeq ($(BOARD),classix)
  ifeq ($(TARGET),ap)
    NORADIO = True
  endif
endif

ifeq ($(NORADIO), False)
  $(TARGET).CFLAGS	+= -DRADIO_CONTROL
  ifneq ($(RADIO_CONTROL_LED),none)
    ap.CFLAGS += -DRADIO_CONTROL_LED=$(RADIO_CONTROL_LED)
  endif
  $(TARGET).CFLAGS 	+= -DRADIO_CONTROL_TYPE_H=\"subsystems/radio_control/ppm.h\"
  $(TARGET).CFLAGS 	+= -DRADIO_CONTROL_TYPE_PPM
  $(TARGET).srcs	+= $(SRC_SUBSYSTEMS)/radio_control.c
  $(TARGET).srcs	+= $(SRC_SUBSYSTEMS)/radio_control/ppm.c
  $(TARGET).srcs 	+= $(SRC_ARCH)/subsystems/radio_control/ppm_arch.c

  ifeq ($(ARCH),stm32)
# default to PA.01 (Servo 6 on Lisa/M) if not already defined
        RADIO_CONTROL_PPM_PIN ?= PA_01
    ifeq ($(RADIO_CONTROL_PPM_PIN),$(filter $(RADIO_CONTROL_PPM_PIN),PA_10 UART1_RX))
      ap.CFLAGS += -DUSE_PPM_TIM1
      fbw.CFLAGS += -DUSE_PPM_TIM1
    else ifeq ($(RADIO_CONTROL_PPM_PIN),$(filter $(RADIO_CONTROL_PPM_PIN),PA_01 SERVO6))
      ap.CFLAGS += -DUSE_PPM_TIM2
      fbw.CFLAGS += -DUSE_PPM_TIM2
    else
        $(error unknown configuration for RADIO_CONTROL_PPM_PIN)
    endif
endif
endif
