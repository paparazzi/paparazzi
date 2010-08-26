#
# Makefile for radio_control susbsytem in rotorcraft firmware
#
ifndef RADIO_CONTROL_SPEKTRUM_MODEL
RADIO_CONTROL_SPEKTRUM_MODEL=\"booz/radio_control/booz_radio_control_spektrum_dx7se.h\"
endif

ap.CFLAGS += -DUSE_RADIO_CONTROL -DRADIO_CONTROL_BIND_IMPL_FUNC=radio_control_spektrum_try_bind
ap.CFLAGS += -DRADIO_CONTROL_TYPE_H=\"booz/radio_control/booz_radio_control_spektrum.h\"
ap.CFLAGS += -DRADIO_CONTROL_SPEKTRUM_MODEL_H=$(RADIO_CONTROL_SPEKTRUM_MODEL)
ap.CFLAGS += -DRADIO_CONTROL_LED=$(RADIO_CONTROL_LED)
ap.CFLAGS += -DRADIO_CONTROL_LINK=$(RADIO_CONTROL_LINK)
#ap.CFLAGS += -DUSE_$(RADIO_CONTROL_LINK) -D$(RADIO_CONTROL_LINK)_BAUD=B115200
ap.CFLAGS += -DOVERRIDE_$(RADIO_CONTROL_LINK)_IRQ_HANDLER

ap.srcs += $(SRC_BOOZ)/booz_radio_control.c \
           $(SRC_BOOZ)/radio_control/booz_radio_control_spektrum.c \
	   $(SRC_BOOZ_ARCH)/radio_control/booz_radio_control_spektrum_arch.c

