#
# Makefile for radio_control susbsytem in rotorcraft firmware
#

RADIO_CONTROL_LED ?= none

ifndef RADIO_CONTROL_SPEKTRUM_MODEL
RADIO_CONTROL_SPEKTRUM_MODEL=\"subsystems/radio_control/spektrum_dx7se.h\"
endif

stm_passthrough.CFLAGS += -DRADIO_CONTROL -DRADIO_CONTROL_BIND_IMPL_FUNC=radio_control_spektrum_try_bind
stm_passthrough.CFLAGS += -DRADIO_CONTROL_TYPE_H=\"subsystems/radio_control/spektrum.h\"
stm_passthrough.CFLAGS += -DRADIO_CONTROL_SPEKTRUM_MODEL_H=$(RADIO_CONTROL_SPEKTRUM_MODEL)
stm_passthrough.CFLAGS += -DRADIO_CONTROL_LINK=$(RADIO_CONTROL_LINK)
stm_passthrough.CFLAGS += -DUSE_$(RADIO_CONTROL_LINK) -D$(RADIO_CONTROL_LINK)_BAUD=B115200
stm_passthrough.srcs += $(SRC_SUBSYSTEMS)/radio_control.c                        \
                        $(SRC_SUBSYSTEMS)/radio_control/spektrum.c \
                        $(SRC_ARCH)/subsystems/radio_control/spektrum_arch.c

ifneq ($(RADIO_CONTROL_LED,none))
stm_passthrough.CFLAGS += -DRADIO_CONTROL_LED=$(RADIO_CONTROL_LED)
endif
