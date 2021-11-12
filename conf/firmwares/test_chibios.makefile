# Hey Emacs, this is a -*- makefile -*-
#
# Copyright (C) 2010 The Paparazzi Team
#
# modified by AggieAir, A Remote Sensing Unmanned Aerial System for Scientific Applications
# Utah State University, http://aggieair.usu.edu/
#
# Michal Podhradsky (michal.podhradsky@aggiemail.usu.edu)
# Calvin Coopmans (c.r.coopmans@ieee.org)
# 2015
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
################################################################################
#
#
#  Common test programs for ChibiOS boards
#
################################################################################
SRC_ARCH=arch/$(ARCH)
SRC_BOARD=boards/$(BOARD)
SRC_MODULES=modules

CFG_SHARED=$(PAPARAZZI_SRC)/conf/firmwares

# Enable debug mode by default for test programs
RTOS_DEBUG ?= 1

#
# common test
#
# configuration
#   SYS_TIME_LED
#   MODEM_PORT
#   MODEM_BAUD
#
PERIODIC_FREQUENCY ?= 500

COMMON_TEST_CFLAGS  = -I$(SRC_ARCH) -I$(SRC_BOARD) -DBOARD_CONFIG=$(BOARD_CFG)
COMMON_TEST_CFLAGS += -DUSE_CHIBIOS_RTOS
COMMON_TEST_CFLAGS += -DPERIPHERALS_AUTO_INIT
COMMON_TEST_SRCS    = mcu.c $(SRC_ARCH)/mcu_arch.c
ifneq ($(SYS_TIME_LED),none)
  COMMON_TEST_CFLAGS += -DSYS_TIME_LED=$(SYS_TIME_LED)
endif
COMMON_TEST_CFLAGS += -DPERIODIC_FREQUENCY=$(PERIODIC_FREQUENCY)
COMMON_TEST_SRCS   += mcu_periph/sys_time.c $(SRC_ARCH)/mcu_periph/sys_time_arch.c

COMMON_TEST_CFLAGS += -DUSE_LED
COMMON_TEST_SRCS += $(SRC_ARCH)/mcu_periph/gpio_arch.c

# pprz downlink/datalink
COMMON_TELEMETRY_CFLAGS = -DDOWNLINK -DDOWNLINK_TRANSPORT=pprz_tp -DDATALINK=PPRZ
COMMON_TELEMETRY_SRCS   = modules/datalink/downlink.c modules/datalink/pprz_transport.c modules/datalink/pprz_dl.c

COMMON_TELEMETRY_MODEM_PORT_LOWER=$(shell echo $(MODEM_PORT) | tr A-Z a-z)
COMMON_TELEMETRY_CFLAGS += -DUSE_$(MODEM_PORT) -D$(MODEM_PORT)_BAUD=$(MODEM_BAUD)
COMMON_TELEMETRY_CFLAGS += -DPPRZ_UART=$(MODEM_PORT)
COMMON_TELEMETRY_CFLAGS += -DDOWNLINK_DEVICE=$(COMMON_TELEMETRY_MODEM_PORT_LOWER)
COMMON_TELEMETRY_SRCS  += mcu_periph/uart.c
COMMON_TELEMETRY_SRCS  += $(SRC_ARCH)/mcu_periph/uart_arch.c

LED_DEFINES ?= -DLED_RED=4 -DLED_GREEN=3


#
# test sys_time
#
test_sys_time_timer.ARCHDIR = $(ARCH)
test_sys_time_timer.CFLAGS += $(COMMON_TEST_CFLAGS) $(LED_DEFINES)
test_sys_time_timer.srcs   += $(COMMON_TEST_SRCS)
test_sys_time_timer.srcs   += test/mcu_periph/chibios_test_sys_time_timer.c

test_sys_time_usleep.ARCHDIR = $(ARCH)
test_sys_time_usleep.CFLAGS += $(COMMON_TEST_CFLAGS) $(LED_DEFINES)
test_sys_time_usleep.srcs   += $(COMMON_TEST_SRCS)
test_sys_time_usleep.srcs   += test/mcu_periph/chibios_test_sys_time_usleep.c

#
# test gpio
#
test_sys_gpio.ARCHDIR = $(ARCH)
test_sys_gpio.CFLAGS += $(COMMON_TEST_CFLAGS)
test_sys_gpio.srcs   += $(COMMON_TEST_SRCS)
test_sys_gpio.srcs   += test/mcu_periph/chibios_test_gpio.c

#
# test led
#
test_led.ARCHDIR = $(ARCH)
test_led.CFLAGS += $(COMMON_TEST_CFLAGS)
test_led.srcs   += $(COMMON_TEST_SRCS)
test_led.srcs   += test/chibios_test_led.c

#
# test shell
# shows the basic functionality of ChibiOS shell
# might be useful later for implementing a console/terminal
# functionality such as is in Pixhawk
#
test_shell.ARCHDIR = $(ARCH)
test_shell.CFLAGS += $(COMMON_TEST_CFLAGS) $(LED_DEFINES)
test_shell.srcs   += $(COMMON_TEST_SRCS)
test_shell.srcs += mcu_periph/uart.c
test_shell.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c
test_shell.srcs += test/chibios_test_shell.c

#
# test serial ports
#
test_serial.ARCHDIR = $(ARCH)
test_serial.CFLAGS += $(COMMON_TEST_CFLAGS) $(LED_DEFINES)
test_serial.srcs   += $(COMMON_TEST_SRCS)
test_serial.srcs += mcu_periph/uart.c
test_serial.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c
test_serial.srcs += test/mcu_periph/chibios_test_serial.c

#
# test_telemetry : Sends ALIVE telemetry messages
#
# configuration
#   MODEM_PORT :
#   MODEM_BAUD :
#
test_telemetry.ARCHDIR = $(ARCH)
test_telemetry.CFLAGS += $(COMMON_TEST_CFLAGS) $(LED_DEFINES)
test_telemetry.srcs   += $(COMMON_TEST_SRCS)
test_telemetry.CFLAGS += $(COMMON_TELEMETRY_CFLAGS)
test_telemetry.srcs   += $(COMMON_TELEMETRY_SRCS)
test_telemetry.srcs   += test/chibios_test_telemetry.c
