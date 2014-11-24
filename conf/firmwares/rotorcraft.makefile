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

ROTORCRAFT_INC = -I$(SRC_FIRMWARE) -I$(SRC_BOARD)


ap.ARCHDIR = $(ARCH)


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

#
# Systime
#
$(TARGET).srcs += mcu_periph/sys_time.c $(SRC_ARCH)/mcu_periph/sys_time_arch.c


#
# Math functions
#
$(TARGET).srcs += math/pprz_geodetic_int.c math/pprz_geodetic_float.c math/pprz_geodetic_double.c math/pprz_trig_int.c math/pprz_orientation_conversion.c math/pprz_algebra_int.c math/pprz_algebra_float.c math/pprz_algebra_double.c

$(TARGET).srcs += subsystems/settings.c
$(TARGET).srcs += $(SRC_ARCH)/subsystems/settings_arch.c

$(TARGET).srcs += subsystems/actuators.c
$(TARGET).srcs += subsystems/commands.c

$(TARGET).srcs += state.c

#
# BARO_BOARD (if existing/configured)
#
include $(CFG_SHARED)/baro_board.makefile


$(TARGET).srcs += $(SRC_FIRMWARE)/stabilization.c
$(TARGET).srcs += $(SRC_FIRMWARE)/stabilization/stabilization_none.c
$(TARGET).srcs += $(SRC_FIRMWARE)/stabilization/stabilization_rate.c

$(TARGET).srcs += $(SRC_FIRMWARE)/guidance/guidance_h.c
$(TARGET).srcs += $(SRC_FIRMWARE)/guidance/guidance_h_ref.c
$(TARGET).srcs += $(SRC_FIRMWARE)/guidance/guidance_v.c
$(TARGET).srcs += $(SRC_FIRMWARE)/guidance/guidance_v_ref.c
$(TARGET).srcs += $(SRC_FIRMWARE)/guidance/guidance_v_adapt.c

include $(CFG_ROTORCRAFT)/navigation.makefile

$(TARGET).srcs += $(SRC_FIRMWARE)/main.c
$(TARGET).srcs += $(SRC_FIRMWARE)/autopilot.c

######################################################################
##
## COMMON HARDWARE SUPPORT FOR ALL TARGETS
##

$(TARGET).srcs += mcu_periph/i2c.c
$(TARGET).srcs += $(SRC_ARCH)/mcu_periph/i2c_arch.c


#
# Electrical subsystem / Analog Backend
#
ifneq ($(ARCH), linux)
$(TARGET).CFLAGS += -DUSE_ADC
$(TARGET).srcs   += $(SRC_ARCH)/mcu_periph/adc_arch.c
$(TARGET).srcs   += subsystems/electrical.c
endif


######################################################################
##
## HARDWARE SUPPORT FOR ALL NON-SIMULATION TARGETS (ap)
##

# baro has variable offset amplifier on booz board
ifeq ($(BOARD), booz)
ns_CFLAGS += -DUSE_DAC
ns_srcs   += $(SRC_ARCH)/mcu_periph/dac_arch.c
else ifeq ($(BOARD)$(BOARD_TYPE), ardronesdk)
ns_srcs   += $(SRC_BOARD)/electrical_dummy.c
else ifeq ($(BOARD)$(BOARD_TYPE), ardroneraw)
ns_srcs   += $(SRC_BOARD)/electrical_raw.c
else ifeq ($(BOARD), bebop)
ns_srcs   += $(SRC_BOARD)/electrical.c
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

ifeq ($(BOARD)$(BOARD_TYPE), ardroneraw)
ns_srcs += $(SRC_BOARD)/gpio_ardrone.c
endif


ns_srcs += mcu_periph/uart.c
ns_srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c
ifeq ($(ARCH), linux)
ns_srcs += $(SRC_ARCH)/serial_port.c
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
