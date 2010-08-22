# Hey Emacs, this is a -*- makefile -*-
#
# $Id$
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

################################################################################
#
#
#  Test program for the booz board
#
#
#
#
# every "firmware" makefile should have a description of available targets
# possible options for each of them, susbsystems and associated params for each of them
# 
#
#
#
################################################################################

ARCHI=arm7

SRC_ARCH=$(ARCHI)
SRC_BOOZ=booz
SRC_BOOZ_ARCH=$(SRC_BOOZ)/arch/lpc21
SRC_BOARD=boards/$(BOARD)

BOARD_CFG=\"boards/booz_1.0.h\"

#
# default configuration expected from board files
#
# SYS_TIME_LED = 1  
# MODEM_PORT   = UART1
# MODEM_BAUD   = B57600


#
# test_telemetry : Sends ALIVE telemetry messages
#
# used configuration
#   SYS_TIME_LED   :
#   MODEM_PORT     :
#   MODEM_BAUD     :
#
test_telemetry.ARCHDIR   = $(ARCHI)
test_telemetry.ARCH      = arm7tdmi
test_telemetry.TARGET    = test_telemetry
test_telemetry.TARGETDIR = test_telemetry
test_telemetry.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_telemetry.CFLAGS += -DPERIPHERALS_AUTO_INIT
test_telemetry.srcs   += test/test_telemetry.c \
	                 $(SRC_ARCH)/armVIC.c
test_telemetry.CFLAGS += -DUSE_LED
test_telemetry.CFLAGS += -DUSE_SYS_TIME
test_telemetry.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))'
test_telemetry.CFLAGS += -DSYS_TIME_LED=$(SYS_TIME_LED)
test_telemetry.srcs   += sys_time.c $(SRC_ARCH)/sys_time_hw.c
test_telemetry.CFLAGS += -DUSE_$(MODEM_PORT)
test_telemetry.CFLAGS += -D$(MODEM_PORT)_BAUD=$(MODEM_BAUD)
test_telemetry.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=$(MODEM_PORT)
test_telemetry.srcs   += downlink.c pprz_transport.c
test_telemetry.srcs   += $(SRC_ARCH)/uart_hw.c


#
# test_baro : reads barometers and sends values over telemetry
#
# configuration
#   SYS_TIME_LED
#   MODEM_PORT
#   MODEM_BAUD
#
test_baro.ARCHDIR   = $(ARCHI)
test_baro.ARCH      = arm7tdmi
test_baro.TARGET    = test_baro
test_baro.TARGETDIR = test_baro
test_baro.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_baro.CFLAGS += -I$(SRC_BOOZ) -I$(SRC_BOOZ_ARCH) -I$(SRC_ARCH) -I$(SRC_BOARD)
test_baro.CFLAGS += -DPERIPHERALS_AUTO_INIT
test_baro.srcs    = $(SRC_BOARD)/test_baro.c \
                    $(SRC_ARCH)/armVIC.c
test_baro.CFLAGS += -DUSE_LED
test_baro.CFLAGS += -DUSE_SYS_TIME
test_baro.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
test_baro.CFLAGS += -DSYS_TIME_LED=$(SYS_TIME_LED)
test_baro.srcs   += sys_time.c $(SRC_ARCH)/sys_time_hw.c
test_baro.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=$(MODEM_PORT)
test_baro.srcs   += downlink.c pprz_transport.c
test_baro.CFLAGS += -DUSE_$(MODEM_PORT) -D$(MODEM_PORT)_BAUD=$(MODEM_BAUD)
test_baro.srcs   += $(SRC_ARCH)/uart_hw.c
test_baro.srcs   += $(SRC_BOARD)/baro_board.c
test_baro.CFLAGS += -DBOOZ2_ANALOG_BARO_PERIOD='SYS_TICS_OF_SEC((1./100.))'
test_baro.CFLAGS += -DADC0_VIC_SLOT=2
test_baro.CFLAGS += -DADC1_VIC_SLOT=3
test_baro.srcs += $(SRC_BOOZ)/booz2_analog.c \
                  $(SRC_BOOZ_ARCH)/booz2_analog_hw.c
# tell me why this shit needs to know battery !!!!
test_baro.CFLAGS += -DBOOZ2_ANALOG_BATTERY_PERIOD='SYS_TICS_OF_SEC((1./10.))'
test_baro.srcs += $(SRC_BOOZ)/booz2_battery.c


#
# test_spektrum :
#
# TODO
# 
# 
# 
#



