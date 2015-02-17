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
SRC_SUBSYSTEMS=subsystems
SRC_MODULES=modules

CFG_SHARED=$(PAPARAZZI_SRC)/conf/firmwares/subsystems/shared

#
# common test
#
# configuration
#   SYS_TIME_LED
#   MODEM_PORT
#   MODEM_BAUD
#
PERIODIC_FREQUENCY ?= 500

COMMON_TEST_CFLAGS  = -I$(SRC_BOARD) -DBOARD_CONFIG=$(BOARD_CFG)
COMMON_TEST_CFLAGS += -DUSE_CHIBIOS_RTOS
COMMON_TEST_CFLAGS += -DPERIPHERALS_AUTO_INIT
COMMON_TEST_SRCS    = mcu.c $(SRC_ARCH)/mcu_arch.c
ifneq ($(SYS_TIME_LED),none)
  COMMON_TEST_CFLAGS += -DSYS_TIME_LED=$(SYS_TIME_LED)
endif
COMMON_TEST_CFLAGS += -DPERIODIC_FREQUENCY=$(PERIODIC_FREQUENCY)
COMMON_TEST_SRCS   += mcu_periph/sys_time.c $(SRC_ARCH)/mcu_periph/sys_time_arch.c

COMMON_TEST_CFLAGS += -DUSE_LED

COMMON_TELEMETRY_MODEM_PORT_LOWER=$(shell echo $(MODEM_PORT) | tr A-Z a-z)
COMMON_TELEMETRY_CFLAGS  = -DUSE_$(MODEM_PORT) -D$(MODEM_PORT)_BAUD=$(MODEM_BAUD)
COMMON_TELEMETRY_CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=pprz_tp -DDOWNLINK_DEVICE=$(COMMON_TELEMETRY_MODEM_PORT_LOWER)
COMMON_TELEMETRY_CFLAGS += -DDATALINK=PPRZ  -DPPRZ_UART=$(MODEM_PORT)
COMMON_TELEMETRY_SRCS    = mcu_periph/uart.c
COMMON_TELEMETRY_SRCS   += $(SRC_ARCH)/mcu_periph/uart_arch.c
COMMON_TELEMETRY_SRCS   += subsystems/datalink/downlink.c subsystems/datalink/pprz_transport.c


#
# test sys_time
#
LED_DEFINES ?= -DLED_RED=2 -DLED_GREEN=3

test_sys_time_timer.ARCHDIR = $(ARCH)
test_sys_time_timer.CFLAGS += $(COMMON_TEST_CFLAGS) $(LED_DEFINES)
test_sys_time_timer.srcs   += $(COMMON_TEST_SRCS)
test_sys_time_timer.srcs   += test/mcu_periph/chibios_test_sys_time_timer.c

