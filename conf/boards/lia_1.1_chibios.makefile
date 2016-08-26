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
BOARD=lia
BOARD_VERSION=1.1_chibios
BOARD_CFG=\"boards/$(BOARD)/chibios/board.h\"#actual chibios defines
# Arch definition
ARCH=chibios# for backwards compatibility with the paparazzi make process
ARCH_DIR=chibios
SRC_ARCH=arch/$(ARCH_DIR)
$(TARGET).ARCHDIR = $(ARCH)

# since it s Cortex M3
HARD_FLOAT=no

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
GPS_BAUD ?= B38400


# default flash mode is via usb dfu bootloader
# possibilities: DFU, SWD, JTAG
FLASH_MODE ?= DFU
STLINK ?= n
DFU_UTIL ?= n

ifndef NO_LUFTBOOT
$(TARGET).CFLAGS+=-DLUFTBOOT
$(TARGET).LDFLAGS+=-Wl,-Ttext=0x8002000
endif

##############################################################################
# Architecture or project specific options
#
# Define project name here (target)
PROJECT = ap

# Project specific files and paths (see Makefile.chibios for details)
CHIBIOS_BOARD_PLATFORM = STM32F1xx/platform_f105_f107.mk
CHIBIOS_BOARD_PORT = ARMCMx/STM32F1xx/port.mk
CHIBIOS_BOARD_LINKER = STM32F107xC.ld

##############################################################################
# Compiler settings
#
MCU  = cortex-m3

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
