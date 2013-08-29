#
# Makefile for shared radio_control spektrum susbsytem
#
ifndef RADIO_CONTROL_SPEKTRUM_MODEL
RADIO_CONTROL_SPEKTRUM_MODEL=\"subsystems/radio_control/spektrum_dx7se.h\"
endif

ap.CFLAGS += -DRADIO_CONTROL -DRADIO_CONTROL_BIND_IMPL_FUNC=radio_control_spektrum_try_bind
ap.CFLAGS += -DRADIO_CONTROL_TYPE_H=\"subsystems/radio_control/spektrum.h\"
ifeq ($(ARCH), lpc21)
ap.CFLAGS += -DRADIO_CONTROL_SPEKTRUM_MODEL_H=$(RADIO_CONTROL_SPEKTRUM_MODEL)
endif
ifneq ($(RADIO_CONTROL_LED),none)
ap.CFLAGS += -DRADIO_CONTROL_LED=$(RADIO_CONTROL_LED)
endif
ap.CFLAGS += -DRADIO_CONTROL_SPEKTRUM_PRIMARY_PORT=SPEKTRUM_$(RADIO_CONTROL_SPEKTRUM_PRIMARY_PORT)
ap.CFLAGS += -DOVERRIDE_$(RADIO_CONTROL_SPEKTRUM_PRIMARY_PORT)_IRQ_HANDLER

# enable the second spektrum port if so configured
ifdef USE_SECONDARY_SPEKTRUM_RECEIVER
ifneq ($(USE_SECONDARY_SPEKTRUM_RECEIVER),0)
ap.CFLAGS += -DRADIO_CONTROL_SPEKTRUM_SECONDARY_PORT=SPEKTRUM_$(RADIO_CONTROL_SPEKTRUM_SECONDARY_PORT)
ap.CFLAGS += -DOVERRIDE_$(RADIO_CONTROL_SPEKTRUM_SECONDARY_PORT)_IRQ_HANDLER
endif
endif

ap.srcs += $(SRC_SUBSYSTEMS)/radio_control.c \
           $(SRC_SUBSYSTEMS)/radio_control/spektrum.c \
           $(SRC_ARCH)/subsystems/radio_control/spektrum_arch.c
