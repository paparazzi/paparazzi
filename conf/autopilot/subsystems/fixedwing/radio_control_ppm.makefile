
NORADIO = False

ifeq ($(BOARD),classix)
  ifeq ($(TARGET),ap)
    NODRADIO = True
  endif
endif



ifeq ($(NORADIO), False)
  $(TARGET).CFLAGS 	+= -DRADIO_CONTROL
  $(TARGET).srcs 	+= $(SRC_FIXEDWING)/radio_control.c $(SRC_ARCH)/ppm_hw.c
endif


