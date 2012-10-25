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

FIXEDWING_INC = -I$(SRC_FIRMWARE) -I$(SRC_FIXEDWING) -I$(SRC_BOARD)



# Standard Fixed Wing Code
include $(CFG_FIXEDWING)/autopilot.makefile

# automatically include correct actuators for the ap target
ifeq ($(BOARD),classix)
  ACTUATOR_TARGET = fbw
else
  ACTUATOR_TARGET = ap
endif

ifeq ($(TARGET),$(ACTUATOR_TARGET))

  ifneq ($(ACTUATORS),)
    include $(CFG_SHARED)/$(ACTUATORS).makefile
  endif

endif



