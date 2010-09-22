
NORADIO = False

ifeq ($(BOARD),classix)
  ifeq ($(TARGET),ap)
    NODRADIO = True
  endif
endif



ifeq ($(NORADIO), False)
  $(TARGET).CFLAGS 	+= -DRADIO_CONTROL
  $(TARGET).srcs 	+= $(SRC_FIXEDWING)/radio_control.c
  ifneq ($(ARCH),jsbsim)
    $(TARGET).srcs 	+= $(SRC_ARCH)/ppm_hw.c
  endif
endif


ifeq ($(ARCH),stm32)
  ap.CFLAGS  += -I$(SRC_FIXEDWING)/booz/
  ap.CFLAGS  += -I$(SRC_FIXEDWING)/booz/arch/stm32/

  ap.CFLAGS += -DUSE_RADIO_CONTROL -DRADIO_CONTROL_LED=$(RADIO_CONTROL_LED)
  ap.CFLAGS += -DRADIO_CONTROL_TYPE_H=\"radio_control/booz_radio_control_ppm.h\"
  ap.CFLAGS += -DRADIO_CONTROL_TYPE_PPM
  ap.srcs += $(SRC_FIXEDWING)/booz/booz_radio_control.c                    \
             $(SRC_FIXEDWING)/booz/radio_control/booz_radio_control_ppm.c  \
             $(SRC_FIXEDWING)/booz/arch/stm32/radio_control/booz_radio_control_ppm_arch.c
  ap.CFLAGS += -DUSE_TIM2_IRQ

endif
