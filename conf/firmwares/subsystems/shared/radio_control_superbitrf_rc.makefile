#
# Makefile for shared radio_control superbitrf subsystem
#

RADIO_CONTROL_LED ?= none
ifneq ($(RADIO_CONTROL_LED),none)
RC_CFLAGS += -DRADIO_CONTROL_LED=$(RADIO_CONTROL_LED)
endif

RC_CFLAGS += -DRADIO_CONTROL_TYPE_H=\"subsystems/radio_control/superbitrf_rc.h\"

RC_FBW_CFLAGS += -DRADIO_CONTROL -DRADIO_CONTROL_TYPE_SUPERBITRF
RC_FBW_CFLAGS += -DUSE_SUPERBITRF -DUSE_SPI2 -DUSE_SPI_SLAVE2

RC_SRCS += peripherals/cyrf6936.c \
		   $(SRC_SUBSYSTEMS)/datalink/superbitrf.c\
           $(SRC_SUBSYSTEMS)/radio_control.c \
           $(SRC_SUBSYSTEMS)/radio_control/superbitrf_rc.c


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
