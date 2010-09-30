#
# Makefile for radio_control susbsytem in rotorcraft firmware
#
ifndef RADIO_CONTROL_SPEKTRUM_MODEL
RADIO_CONTROL_SPEKTRUM_MODEL=\"booz/radio_control/booz_radio_control_spektrum_dx7se.h\"
endif

ap.CFLAGS += -DUSE_RADIO_CONTROL -DRADIO_CONTROL_BIND_IMPL_FUNC=radio_control_spektrum_try_bind
ap.CFLAGS += -DRADIO_CONTROL_TYPE_H=\"booz/radio_control/booz_radio_control_spektrum.h\"
ifeq ($(BOARD), booz)
ap.CFLAGS += -DRADIO_CONTROL_SPEKTRUM_MODEL_H=$(RADIO_CONTROL_SPEKTRUM_MODEL)
endif
ifdef RADIO_CONTROL_LED
ap.CFLAGS += -DRADIO_CONTROL_LED=$(RADIO_CONTROL_LED)
endif
ap.CFLAGS += -DRADIO_CONTROL_SPEKTRUM_PRIMARY_PORT=$(RADIO_CONTROL_SPEKTRUM_PRIMARY_PORT)
ap.CFLAGS += -DOVERRIDE_$(RADIO_CONTROL_SPEKTRUM_PRIMARY_PORT)_IRQ_HANDLER -DUSE_TIM6_IRQ

ap.srcs += $(SRC_BOOZ)/booz_radio_control.c \
           $(SRC_BOOZ)/radio_control/booz_radio_control_spektrum.c \
	   $(SRC_BOOZ_ARCH)/radio_control/booz_radio_control_spektrum_arch.c

