#
# Copyright (C) 2013 AggieAir, A Remote Sensing Unmanned Aerial System for Scientific Applications
# Utah State University, http://aggieair.usu.edu/
#
# Michal Podhradsky (michal.podhradsky@aggiemail.usu.edu)
# Calvin Coopmans (c.r.coopmans@ieee.org)
#
# credits to ENAC team for initial work on ChibiOs and Paparazzi integration
#
# This file is part of paparazzi.
#
# paparazzi is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
# any later version.
#
# paparazzi is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with paparazzi; see the file COPYING.  If not, write to
# the Free Software Foundation, 59 Temple Place - Suite 330,
# Boston, MA 02111-1307, USA.
#

# Board definition
BOARD=lisa_mx
BOARD_CFG=\"boards/$(BOARD)/board.h\"#actual chibios defines
# Arch definition
ARCH=chibios# for backwards compatibility with the paparazzi make process
ARCH_DIR=chibios
SRC_ARCH=arch/$(ARCH_DIR)
$(TARGET).ARCHDIR = $(ARCH)

## FPU on F4
USE_FPU=no
# FPU somehow screws up floating point computation, for example vff

#
# default LED configuration
#
BARO_LED           ?= 5
RADIO_CONTROL_LED  ?= 4
GPS_LED            ?= 3
AHRS_ALIGNER_LED   ?= 2
SYS_TIME_LED       ?= 1

#
# default uart configuration
#
MODEM_PORT ?= UART2
MODEM_BAUD ?= B57600

GPS_PORT ?= UART5
GPS_BAUD ?= B115200


# default flash mode is via usb dfu bootloader (luftboot)
# other possibilities: DFU-UTIL, JTAG, SWD, STLINK, SERIAL
FLASH_MODE ?= SWD

# no luftboot for f4
HAS_LUFTBOOT ?= 0
ifeq (,$(findstring $(HAS_LUFTBOOT),0 FALSE))
$(TARGET).CFLAGS+=-DLUFTBOOT
endif

##############################################################################
# Architecture or project specific options
#
# Define project name here (target)
PROJECT = ap

# Project specific files and paths (see Makefile.chibios for details)
CHIBIOS_BOARD_PLATFORM = STM32F4xx/platform.mk
CHIBIOS_BOARD_PORT = ARMCMx/STM32F4xx/port.mk
CHIBIOS_BOARD_LINKER = STM32F407xG.ld

##############################################################################
# Compiler settings
#
MCU  = cortex-m4

# -----------------------------------------------------------------------
# include Makefile.chibios instead of Makefile.stm32
$(TARGET).MAKEFILE = chibios


#
# default actuator configuration
#
# you can use different actuators by adding a configure option to your firmware section
# e.g. <configure name="ACTUATORS" value="actuators_ppm/>
# and by setting the correct "driver" attribute in servo section
# e.g. <servo driver="Ppm">
#
ACTUATORS ?= actuators_pwm
