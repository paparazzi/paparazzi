
NORADIO = False

ifeq ($(BOARD),classix)
  ifeq ($(TARGET),ap)
    NORADIO = True
  endif
endif



ifeq ($(ARCH),stm32)
  ap.CFLAGS += -DRADIO_CONTROL
ifdef RADIO_CONTROL_LED
  ap.CFLAGS += -DRADIO_CONTROL_LED=$(RADIO_CONTROL_LED)
endif
  ap.CFLAGS += -DRADIO_CONTROL_TYPE_H=\"subsystems/radio_control/ppm.h\"
  ap.CFLAGS += -DRADIO_CONTROL_TYPE_PPM
  ap.srcs += $(SRC_SUBSYSTEMS)/radio_control.c \
             $(SRC_SUBSYSTEMS)/radio_control/ppm.c \
             $(SRC_ARCH)/subsystems/radio_control/ppm_arch.c
  ap.CFLAGS += -DUSE_TIM2_IRQ

  NORADIO = True
endif

ifeq ($(NORADIO), False)
  $(TARGET).CFLAGS	+= -DRADIO_CONTROL
  $(TARGET).CFLAGS 	+= -DRADIO_CONTROL_TYPE_H=\"subsystems/radio_control/ppm.h\"
  $(TARGET).CFLAGS 	+= -DRADIO_CONTROL_TYPE_PPM
  $(TARGET).srcs	+= $(SRC_SUBSYSTEMS)/radio_control.c
  $(TARGET).srcs	+= $(SRC_SUBSYSTEMS)/radio_control/ppm.c
  ifneq ($(ARCH),jsbsim)
    $(TARGET).srcs 	+= $(SRC_ARCH)/subsystems/radio_control/ppm_arch.c
  endif
endif
