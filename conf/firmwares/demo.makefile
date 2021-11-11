# Hey Emacs, this is a -*- makefile -*-
#
# Copyright (C) 2013 The Paparazzi Team
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

################################################################################
#
#
#  Some demo programs
#
################################################################################


SRC_ARCH=arch/$(ARCH)
SRC_BOARD=boards/$(BOARD)
SRC_SUBSYSTEMS=subsystems
SRC_MODULES=modules

CFG_SHARED=$(PAPARAZZI_SRC)/conf/firmwares/subsystems/shared

VPATH += $(PAPARAZZI_HOME)/var/share

#
# common test
#
# configuration
#   SYS_TIME_LED
#   MODEM_PORT
#   MODEM_BAUD
#
PERIODIC_FREQUENCY ?= 512

COMMON_DEMO_CFLAGS  = -I$(SRC_BOARD) -DBOARD_CONFIG=$(BOARD_CFG)
COMMON_DEMO_CFLAGS += -DPERIPHERALS_AUTO_INIT
COMMON_DEMO_SRCS    = mcu.c $(SRC_ARCH)/mcu_arch.c
ifneq ($(SYS_TIME_LED),none)
  COMMON_DEMO_CFLAGS += -DSYS_TIME_LED=$(SYS_TIME_LED)
endif
COMMON_DEMO_CFLAGS += -DPERIODIC_FREQUENCY=$(PERIODIC_FREQUENCY)
COMMON_DEMO_SRCS   += mcu_periph/sys_time.c $(SRC_ARCH)/mcu_periph/sys_time_arch.c
ifeq ($(ARCH), linux)
# seems that we need to link agains librt for glibc < 2.17
$(TARGET).LDFLAGS += -lrt
endif

COMMON_DEMO_CFLAGS += -DUSE_LED

ifeq ($(ARCH), stm32)
COMMON_DEMO_SRCS += $(SRC_ARCH)/led_hw.c
COMMON_DEMO_SRCS += $(SRC_ARCH)/mcu_periph/gpio_arch.c
endif

COMMON_DEMO_SRCS += mcu_periph/i2c.c mcu_periph/softi2c.c $(SRC_ARCH)/mcu_periph/i2c_arch.c
COMMON_DEMO_SRCS += mcu_periph/uart.c
COMMON_DEMO_SRCS += $(SRC_ARCH)/mcu_periph/uart_arch.c
ifeq ($(ARCH), linux)
COMMON_DEMO_SRCS += $(SRC_ARCH)/serial_port.c
endif

#
# Demo AHRS and actuators
#
# required subsystems:
#   - telemetry
#   - imu
#   - ahrs
#   - actuators
#
# configuration
#   SYS_TIME_LED
#   MODEM_PORT
#   MODEM_BAUD
#
demo_ahrs_actuators.ARCHDIR = $(ARCH)
demo_ahrs_actuators.CFLAGS += $(COMMON_DEMO_CFLAGS)
demo_ahrs_actuators.srcs   += $(COMMON_DEMO_SRCS)
demo_ahrs_actuators.srcs   += mcu_periph/i2c.c $(SRC_ARCH)/mcu_periph/i2c_arch.c
demo_ahrs_actuators.srcs   += subsystems/settings.c $(SRC_ARCH)/subsystems/settings_arch.c
demo_ahrs_actuators.srcs   += subsystems/commands.c modules/actuators/actuators.c
demo_ahrs_actuators.srcs   += state.c
demo_ahrs_actuators.srcs   += math/pprz_geodetic_int.c math/pprz_geodetic_float.c math/pprz_geodetic_double.c math/pprz_trig_int.c math/pprz_orientation_conversion.c math/pprz_algebra_int.c math/pprz_algebra_float.c math/pprz_algebra_double.c
demo_ahrs_actuators.srcs   += firmwares/demo/demo_ahrs_actuators.c

ifeq ($(TARGET), demo_ahrs_actuators)
  ifeq ($(ACTUATORS),)
    $(error ACTUATORS not configured, if your board file has no default, configure in your airframe file)
  else
    include $(CFG_SHARED)/$(ACTUATORS).makefile
  endif
endif
