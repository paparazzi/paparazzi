# Hey Emacs, this is a -*- makefile -*-


RADIO_CONTROL_DATALINK_LED ?= none
RADIO_CONTROL_LED ?= none

ifneq ($(RADIO_CONTROL_DATALINK_LED),none)
RC_CFLAGS += -DRADIO_CONTROL_DATALINK_LED=$(RADIO_CONTROL_DATALINK_LED)
endif

ifneq ($(RADIO_CONTROL_LED),none)
RC_CFLAGS += -DRADIO_CONTROL_LED=$(RADIO_CONTROL_LED)
RC_CFLAGS += -DRADIO_CONTROL_LED=$(RADIO_CONTROL_LED)
endif

RC_CFLAGS += -DRADIO_CONTROL_TYPE_H=\"radio_control/rc_datalink.h\"

RC_FBW_CFLAGS += -DRADIO_CONTROL
RC_FBW_CFLAGS += -DRADIO_CONTROL_TYPE_DATALINK
RC_SRCS   += $(SRC_SUBSYSTEMS)/radio_control.c
RC_SRCS   += $(SRC_SUBSYSTEMS)/radio_control/rc_datalink.c


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

# arch only with sim target for compatibility (empty functions)
sim.srcs += $(SRC_ARCH)/subsystems/radio_control/rc_datalink.c
