# Hey Emacs, this is a -*- makefile -*-
#
# Copyright (C) 2010 The Paparazzi Team
#
# This file is part of Paparazzi.
#
# Paparazzi is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
# any later version.
#
# Paparazzi is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with Paparazzi; see the file COPYING.  If not, write to
# the Free Software Foundation, 59 Temple Place - Suite 330,
# Boston, MA 02111-1307, USA.
#
#

CFG_SHARED=$(PAPARAZZI_SRC)/conf/firmwares/subsystems/shared
CFG_ROTORCRAFT=$(PAPARAZZI_SRC)/conf/firmwares/subsystems/rotorcraft

SRC_BOARD=boards/$(BOARD)
SRC_FIRMWARE=firmwares/rotorcraft
SRC_SUBSYSTEMS=subsystems
SRC_MODULES=modules

SRC_ARCH=arch/$(ARCH)

ROTORCRAFT_INC = -DROTORCRAFT_FIRMWARE -I$(SRC_FIRMWARE) -I$(SRC_BOARD)

ap.ARCHDIR = $(ARCH)


VPATH += $(PAPARAZZI_HOME)/var/share
VPATH += $(PAPARAZZI_HOME)/sw/ext

######################################################################
##
## COMMON ROTORCRAFT ALL TARGETS (AP + NPS)
##

$(TARGET).CFLAGS += $(ROTORCRAFT_INC)
$(TARGET).CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
$(TARGET).CFLAGS += -DPERIPHERALS_AUTO_INIT
$(TARGET).srcs   += mcu.c
$(TARGET).srcs   += $(SRC_ARCH)/mcu_arch.c

# frequency of main periodic
PERIODIC_FREQUENCY ?= 512
$(TARGET).CFLAGS += -DPERIODIC_FREQUENCY=$(PERIODIC_FREQUENCY)

ifdef AHRS_PROPAGATE_FREQUENCY
$(TARGET).CFLAGS += -DAHRS_PROPAGATE_FREQUENCY=$(AHRS_PROPAGATE_FREQUENCY)
endif

ifdef AHRS_CORRECT_FREQUENCY
$(TARGET).CFLAGS += -DAHRS_CORRECT_FREQUENCY=$(AHRS_CORRECT_FREQUENCY)
endif

ifdef AHRS_MAG_CORRECT_FREQUENCY
$(TARGET).CFLAGS += -DAHRS_MAG_CORRECT_FREQUENCY=$(AHRS_MAG_CORRECT_FREQUENCY)
endif


#
# Systime
#
$(TARGET).srcs += mcu_periph/sys_time.c $(SRC_ARCH)/mcu_periph/sys_time_arch.c
ifeq ($(ARCH), linux)
# seems that we need to link against librt for glibc < 2.17
$(TARGET).LDFLAGS += -lrt
endif


#
# Math functions
#
ifneq ($(TARGET), fbw)
$(TARGET).srcs += math/pprz_geodetic_int.c math/pprz_geodetic_float.c math/pprz_geodetic_double.c math/pprz_trig_int.c math/pprz_orientation_conversion.c math/pprz_algebra_int.c math/pprz_algebra_float.c math/pprz_algebra_double.c math/pprz_stat.c

$(TARGET).srcs += subsystems/settings.c
$(TARGET).srcs += $(SRC_ARCH)/subsystems/settings_arch.c
endif

$(TARGET).srcs += subsystems/actuators.c
$(TARGET).srcs += subsystems/commands.c

ifneq ($(TARGET), fbw)
$(TARGET).srcs += state.c

#
# BARO_BOARD (if existing/configured)
#
include $(CFG_SHARED)/baro_board.makefile


else
$(TARGET).CFLAGS += -DFBW=1
endif

#
# Main
#
ifeq ($(RTOS), chibios)
$(TARGET).srcs += $(SRC_FIRMWARE)/main_chibios.c
else # No RTOS
$(TARGET).srcs += $(SRC_FIRMWARE)/main.c
endif # RTOS
ifneq ($(TARGET), fbw)
$(TARGET).srcs += $(SRC_FIRMWARE)/main_ap.c
$(TARGET).srcs += autopilot.c
$(TARGET).srcs += $(SRC_FIRMWARE)/autopilot_firmware.c
$(TARGET).srcs += $(SRC_FIRMWARE)/autopilot_utils.c
$(TARGET).srcs += $(SRC_FIRMWARE)/autopilot_guided.c
ifeq ($(USE_GENERATED_AUTOPILOT), TRUE)
$(TARGET).srcs += $(SRC_FIRMWARE)/autopilot_generated.c
$(TARGET).CFLAGS += -DUSE_GENERATED_AUTOPILOT=1
else
$(TARGET).srcs += $(SRC_FIRMWARE)/autopilot_static.c
endif
else
$(TARGET).srcs += $(SRC_FIRMWARE)/main_fbw.c
endif # TARGET == fbw



######################################################################
##
## COMMON HARDWARE SUPPORT FOR ALL TARGETS
##

ifneq ($(TARGET), fbw)
$(TARGET).srcs += mcu_periph/i2c.c
$(TARGET).srcs += mcu_periph/softi2c.c
$(TARGET).srcs += $(SRC_ARCH)/mcu_periph/i2c_arch.c
endif

include $(CFG_SHARED)/uart.makefile


#
# Electrical subsystem / Analog Backend
#
$(TARGET).CFLAGS += -DUSE_ADC
$(TARGET).srcs   += $(SRC_ARCH)/mcu_periph/adc_arch.c
$(TARGET).srcs   += subsystems/electrical.c


######################################################################
##
## HARDWARE SUPPORT FOR ALL NON-SIMULATION TARGETS (ap)
##

# baro has variable offset amplifier on booz board
ifeq ($(BOARD), booz)
ns_CFLAGS += -DUSE_DAC
ns_srcs   += $(SRC_ARCH)/mcu_periph/dac_arch.c
endif

#
# Interrupts
#
ifeq ($(ARCH), lpc21)
ns_srcs += $(SRC_ARCH)/armVIC.c
endif

ifeq ($(ARCH), stm32)
ns_srcs += $(SRC_ARCH)/mcu_periph/gpio_arch.c
endif

ifeq ($(ARCH), chibios)
ns_srcs       += $(SRC_ARCH)/mcu_periph/gpio_arch.c
endif

#
# LEDs
#
ns_CFLAGS += -DUSE_LED
ifneq ($(SYS_TIME_LED),none)
ns_CFLAGS += -DSYS_TIME_LED=$(SYS_TIME_LED)
endif

ifeq ($(ARCH), stm32)
ns_srcs += $(SRC_ARCH)/led_hw.c
endif

ifeq ($(BOARD), ardrone)
ns_srcs += $(SRC_BOARD)/gpio_ardrone.c
endif

#
# add other subsystems to rotorcraft firmware in airframe file:
#
# telemetry
# radio_control
# actuators
# imu
# gps
# ahrs
# ins
#


######################################################################
##
## Final Target Allocations
##

ap.CFLAGS 		+= $(ns_CFLAGS)
ap.srcs 		+= $(ns_srcs)
fbw.CFLAGS 		+= $(ns_CFLAGS)
fbw.srcs 		+= $(ns_srcs)

######################################################################
##
## include firmware independent nps makefile and add rotorcraft specifics
##
ifneq ($(TARGET), hitl)
  include $(CFG_SHARED)/nps.makefile
else
  include $(CFG_SHARED)/hitl.makefile
endif

nps.srcs += nps/nps_autopilot_rotorcraft.c
