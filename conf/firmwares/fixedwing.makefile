# Hey Emacs, this is a -*- makefile -*-
#
# fixedwing.makefile
#
#


CFG_SHARED=$(PAPARAZZI_SRC)/conf/firmwares/subsystems/shared
CFG_FIXEDWING=$(PAPARAZZI_SRC)/conf/firmwares/subsystems/fixedwing


SRC_FIXEDWING=.
SRC_ARCH=arch/$(ARCH)
SRC_FIXEDWING_TEST=$(SRC_FIXEDWING)/

SRC_BOARD=boards/$(BOARD)
SRC_FIRMWARE=firmwares/fixedwing
SRC_SUBSYSTEMS=subsystems
SRC_MODULES=modules

FIXEDWING_INC = -DFIXEDWING_FIRMWARE -I$(SRC_FIRMWARE) -I$(SRC_FIXEDWING) -I$(SRC_BOARD)

VPATH += $(PAPARAZZI_HOME)/var/share
VPATH += $(PAPARAZZI_HOME)/sw/ext

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



