# Hey Emacs, this is a -*- makefile -*-
#
# fixedwing.makefile
#
#


CFG_SHARED=$(PAPARAZZI_SRC)/conf/autopilot/subsystems/shared
CFG_FIXEDWING=$(PAPARAZZI_SRC)/conf/autopilot/subsystems/fixedwing


SRC_FIXEDWING=.
SRC_ARCH=arch/$(ARCH)
SRC_FIXEDWING_TEST=$(SRC_FIXEDWING)/

SRC_FIRMWARE=firmwares/fixedwing
SRC_SUBSYSTEMS=subsystems
SRC_MODULES=modules

FIXEDWING_INC = -I$(SRC_FIRMWARE) -I$(SRC_FIXEDWING)



# Standard Fixed Wing Code
include $(CFG_FIXEDWING)/autopilot.makefile

# automatically include correct actuators for the ap target
ifeq ($(BOARD),classix)
  ACTUATOR_TARGET = fbw
else
  ACTUATOR_TARGET = ap
endif

ifeq ($(TARGET),$(ACTUATOR_TARGET))

  ifeq ($(ACTUATORS),)

    ifeq ($(BOARD),lisa_l)
      include $(CFG_SHARED)/actuators_direct.makefile
    endif
    ifeq ($(BOARD),lisa_m)
      include $(CFG_SHARED)/actuators_direct.makefile
    endif

  else
    include $(CFG_SHARED)/$(ACTUATORS).makefile
  endif

endif



