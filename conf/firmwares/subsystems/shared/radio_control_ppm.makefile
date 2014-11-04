#
# Makefile for shared radio_control ppm subsystem
#

RADIO_CONTROL_LED ?= none

ifneq ($(RADIO_CONTROL_LED),none)
RC_CFLAGS += -DRADIO_CONTROL_LED=$(RADIO_CONTROL_LED)
endif

RC_CFLAGS += -DRADIO_CONTROL_TYPE_H=\"subsystems/radio_control/ppm.h\"

RC_FBW_CFLAGS += -DRADIO_CONTROL
RC_FBW_CFLAGS += -DRADIO_CONTROL_TYPE_PPM

RC_SRCS   += $(SRC_SUBSYSTEMS)/radio_control.c
RC_SRCS   += $(SRC_SUBSYSTEMS)/radio_control/ppm.c
RC_SRCS   += $(SRC_ARCH)/subsystems/radio_control/ppm_arch.c

ifeq ($(ARCH),stm32)
  ifdef RADIO_CONTROL_PPM_PIN
    ifeq ($(RADIO_CONTROL_PPM_PIN),$(filter $(RADIO_CONTROL_PPM_PIN),PA_10 UART1_RX))
      $(TARGET).CFLAGS += -DPPM_CONFIG=1
    else ifeq ($(RADIO_CONTROL_PPM_PIN),$(filter $(RADIO_CONTROL_PPM_PIN),PA_01 SERVO6))
      $(TARGET).CFLAGS += -DPPM_CONFIG=2
    endif
  endif
endif


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

# dummy stuff so you don't have to unload superbitrf.xml settings file for simulators
nps.srcs += $(SRC_ARCH)/subsystems/datalink/superbitrf.c
sim.srcs += $(SRC_ARCH)/subsystems/datalink/superbitrf.c
