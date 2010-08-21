# Hey Emacs, this is a -*- makefile -*-
#
# $Id: lisa_test_progs.makefile 5420 2010-08-17 14:05:21Z poine $
# Copyright (C) 2010 Antoine Drouin
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
#  Test program for the lisa_L board
#
#
################################################################################

ARCHI=stm32
SRC_ARCH=$(ARCHI)
SRC_LISA=lisa
SRC_LISA_ARCH=$(SRC_LISA)/arch/$(ARCHI)
SRC_BOOZ=booz
SRC_BOOZ_ARCH=$(SRC_BOOZ)/arch/$(ARCHI)

BOARD_CFG=\"boards/lisa_0.99.h\"

#
# transmits baro readings on uart2
#
test_baro.ARCHDIR   = $(ARCHI)
test_baro.TARGET    = test_baro
test_baro.TARGETDIR = test_baro
test_baro.CFLAGS = -I$(SRC_LISA) -I$(SRC_ARCH) -DPERIPHERALS_AUTO_INIT
test_baro.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_baro.srcs = $(SRC_LISA)/test_baro2.c      \
                 $(SRC_ARCH)/stm32_exceptions.c   \
                 $(SRC_ARCH)/stm32_vector_table.c
test_baro.CFLAGS += -DUSE_LED
test_baro.srcs   += $(SRC_ARCH)/led_hw.c
test_baro.CFLAGS += -DUSE_SYS_TIME
test_baro.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
test_baro.CFLAGS += -DSYS_TIME_LED=1
test_baro.srcs   += sys_time.c $(SRC_ARCH)/sys_time_hw.c
test_baro.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart2 
test_baro.srcs   += downlink.c pprz_transport.c
test_baro.CFLAGS += -DUSE_UART2 -DUART2_BAUD=B57600
test_baro.srcs   += $(SRC_ARCH)/uart_hw.c
test_baro.srcs   += $(SRC_LISA)/lisa_baro.c
test_baro.CFLAGS += -DUSE_I2C2
test_baro.srcs   += i2c.c $(SRC_ARCH)/i2c_hw.c
