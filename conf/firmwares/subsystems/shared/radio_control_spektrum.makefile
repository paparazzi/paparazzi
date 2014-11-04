#
# Makefile for shared radio_control spektrum susbsytem
#

RADIO_CONTROL_LED ?= none

ifneq ($(RADIO_CONTROL_LED),none)
RC_CFLAGS += -DRADIO_CONTROL_LED=$(RADIO_CONTROL_LED)
endif

RC_CFLAGS += -DRADIO_CONTROL_TYPE_H=\"subsystems/radio_control/spektrum.h\"

ifeq ($(ARCH), lpc21)
ifndef RADIO_CONTROL_SPEKTRUM_MODEL
RADIO_CONTROL_SPEKTRUM_MODEL=\"subsystems/radio_control/spektrum_dx7se.h\"
endif
RC_CFLAGS += -DRADIO_CONTROL_SPEKTRUM_MODEL_H=$(RADIO_CONTROL_SPEKTRUM_MODEL)
endif

RC_FBW_CFLAGS += -DRADIO_CONTROL -DRADIO_CONTROL_BIND_IMPL_FUNC=radio_control_spektrum_try_bind

RC_FBW_CFLAGS += -DRADIO_CONTROL_SPEKTRUM_PRIMARY_PORT=SPEKTRUM_$(RADIO_CONTROL_SPEKTRUM_PRIMARY_PORT)
RC_FBW_CFLAGS += -DOVERRIDE_$(RADIO_CONTROL_SPEKTRUM_PRIMARY_PORT)_IRQ_HANDLER

# enable the second spektrum port if so configured
ifdef USE_SECONDARY_SPEKTRUM_RECEIVER
ifneq ($(USE_SECONDARY_SPEKTRUM_RECEIVER),0)
RC_FBW_CFLAGS += -DRADIO_CONTROL_SPEKTRUM_SECONDARY_PORT=SPEKTRUM_$(RADIO_CONTROL_SPEKTRUM_SECONDARY_PORT)
RC_FBW_CFLAGS += -DOVERRIDE_$(RADIO_CONTROL_SPEKTRUM_SECONDARY_PORT)_IRQ_HANDLER
endif
endif

RC_SRCS += $(SRC_SUBSYSTEMS)/radio_control.c \
           $(SRC_SUBSYSTEMS)/radio_control/spektrum.c \
           $(SRC_ARCH)/subsystems/radio_control/spektrum_arch.c


ifeq (,$(findstring $(SEPARATE_FBW),1 TRUE))
# Single MCU's run RC on ap target
$(TARGET).CFLAGS += $(RC_CFLAGS) $(RC_FBW_CFLAGS)
$(TARGET).srcs   += $(RC_SRCS)
else
# Dual MCU case
fbw.CFLAGS += $(RC_CFLAGS) $(RC_FBW_CFLAGS)
fbw.srcs   += $(RC_SRCS)
# define RADIO_CONTROL_TYPE for ap in dual_mcu case to get defines
# but don't add source files
ap.CFLAGS += $(RC_CFLAGS)
endif

test_radio_control.CFLAGS += $(RC_CFLAGS) $(RC_FBW_CFLAGS)
test_radio_control.srcs   += $(RC_SRCS)
