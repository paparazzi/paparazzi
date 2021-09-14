# Hey Emacs, this is a -*- makefile -*-
#
# fixedwing.makefile
#
#


CFG_SHARED=$(PAPARAZZI_SRC)/conf/firmwares/subsystems/shared
CFG_FIXEDWING=$(PAPARAZZI_SRC)/conf/firmwares/subsystems/fixedwing


# Standard Fixed Wing Code
include $(CFG_FIXEDWING)/autopilot.makefile

# automatically include correct actuators for the ap target
ACTUATOR_TARGET = ap
ACTUATORS ?= none

ifeq ($(TARGET),$(ACTUATOR_TARGET))

  ifneq ($(ACTUATORS),none)
    include $(CFG_SHARED)/$(ACTUATORS).makefile
  endif

endif



