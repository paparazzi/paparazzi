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
    ifeq ($(RADIO_CONTROL_PPM_PIN),UART1_RX)
      $(TARGET).CFLAGS += -DUSE_TIM1_IRQ
      fbw.CFLAGS += -DUSE_TIM1_IRQ
    else ifeq ($(RADIO_CONTROL_PPM_PIN),SERVO6)
      $(TARGET).CFLAGS += -DUSE_TIM2_IRQ
      fbw.CFLAGS += -DUSE_TIM2_IRQ
    else
        $(error unknown configuration for RADIO_CONTROL_PPM_PIN)
    endif
  endif
endif
