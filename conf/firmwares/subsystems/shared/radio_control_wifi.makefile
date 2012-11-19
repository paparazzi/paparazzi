#
# Makefile for shared radio_control ppm susbsytem
#

NORADIO = False

ifeq ($(NORADIO), False)
  $(TARGET).CFLAGS	+= -DRADIO_CONTROL
#  ifneq ($(RADIO_CONTROL_LED),none)
#    ap.CFLAGS += -DRADIO_CONTROL_LED=$(RADIO_CONTROL_LED)
#  endif
  $(TARGET).CFLAGS 	+= -DRADIO_CONTROL_TYPE_H=\"subsystems/radio_control/wifi.h\"
  $(TARGET).CFLAGS 	+= -DRADIO_CONTROL_TYPE_WIFI
  $(TARGET).srcs	+= $(SRC_SUBSYSTEMS)/radio_control.c
  $(TARGET).srcs	+= $(SRC_SUBSYSTEMS)/radio_control/wifi.c
  $(TARGET).srcs 	+= $(SRC_ARCH)/subsystems/radio_control/wifi_arch.c

endif
