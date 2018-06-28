# Hey Emacs, this is a -*- makefile -*-
#
# Copyright (C) 2018 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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

SRC_BOARD=boards/$(BOARD)
SRC_FIRMWARE=firmwares/rover
SRC_SUBSYSTEMS=subsystems
SRC_MODULES=modules

SRC_ARCH=arch/$(ARCH)

ROTORCRAFT_INC = -I$(SRC_FIRMWARE) -I$(SRC_BOARD)

ap.ARCHDIR = $(ARCH)


VPATH += $(PAPARAZZI_HOME)/var/share

######################################################################
##
## COMMON ROVER ALL TARGETS (AP)
##

$(TARGET).CFLAGS += $(ROTORCRAFT_INC)
$(TARGET).CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
$(TARGET).CFLAGS += -DPERIPHERALS_AUTO_INIT
$(TARGET).srcs   += mcu.c
$(TARGET).srcs   += $(SRC_ARCH)/mcu_arch.c

# frequency of main periodic
PERIODIC_FREQUENCY ?= 100
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
$(TARGET).srcs += math/pprz_geodetic_int.c math/pprz_geodetic_float.c math/pprz_geodetic_double.c math/pprz_trig_int.c math/pprz_orientation_conversion.c math/pprz_algebra_int.c math/pprz_algebra_float.c math/pprz_algebra_double.c math/pprz_stat.c

$(TARGET).srcs += subsystems/settings.c
$(TARGET).srcs += $(SRC_ARCH)/subsystems/settings_arch.c

$(TARGET).srcs += subsystems/actuators.c
$(TARGET).srcs += subsystems/commands.c

$(TARGET).srcs += state.c

#
# BARO_BOARD (if existing/configured)
#
include $(CFG_SHARED)/baro_board.makefile

#
# Main
#
# based on ChibiOS
#
ifeq ($(RTOS), chibios)
$(TARGET).srcs += $(SRC_FIRMWARE)/main_chibios.c
endif # RTOS
$(TARGET).srcs += $(SRC_FIRMWARE)/main_ap.c
$(TARGET).srcs += autopilot.c
$(TARGET).srcs += $(SRC_FIRMWARE)/autopilot_firmware.c
$(TARGET).srcs += $(SRC_FIRMWARE)/autopilot_utils.c
ifeq ($(USE_GENERATED_AUTOPILOT), TRUE)
$(TARGET).srcs += $(SRC_FIRMWARE)/autopilot_generated.c
$(TARGET).CFLAGS += -DUSE_GENERATED_AUTOPILOT=1
else
$(error "Rover firmware should use generated autopilot")
endif



######################################################################
##
## COMMON HARDWARE SUPPORT FOR ALL TARGETS
##

$(TARGET).srcs += mcu_periph/i2c.c
$(TARGET).srcs += $(SRC_ARCH)/mcu_periph/i2c_arch.c

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
ifeq ($(ARCH), stm32)
ns_srcs += $(SRC_ARCH)/mcu_periph/gpio_arch.c
endif

ifeq ($(ARCH), chibios)
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

######################################################################
##
## Final Target Allocations
##

ap.CFLAGS 		+= $(ns_CFLAGS)
ap.srcs 		+= $(ns_srcs)

