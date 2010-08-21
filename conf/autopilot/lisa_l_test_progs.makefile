# Hey Emacs, this is a -*- makefile -*-
#
# $Id: lisa_test_progs.makefile 5420 2010-08-17 14:05:21Z poine $
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
#  Test program for the lisa_L board
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

ARCHI=stm32
SRC_ARCH=$(ARCHI)
SRC_LISA=lisa
SRC_LISA_ARCH=$(SRC_LISA)/arch/$(ARCHI)
SRC_BOOZ=booz
SRC_BOOZ_ARCH=$(SRC_BOOZ)/arch/$(ARCHI)

BOARD_CFG=\"boards/lisa_l_1.0.h\"


#
# default configuration expected from the board files
#
# MODEM_PORT = UART2
# MODEM_BAUD = B57600


#
# test_telemetry : Sends ALIVE telemetry messages
#
# configuration
#   MODEM_PORT :
#   MODEM_BAUD :
#
test_telemetry.ARCHDIR = $(ARCHI)
test_telemetry.TARGET = test_telemetry
test_telemetry.TARGETDIR = test_telemetry
test_telemetry.CFLAGS += -I$(SRC_LISA) -I$(SRC_ARCH) -I$(PAPARAZZI_SRC)/conf -DPERIPHERALS_AUTO_INIT
test_telemetry.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_telemetry.srcs = test/test_telemetry.c            \
                      $(SRC_ARCH)/stm32_exceptions.c   \
                      $(SRC_ARCH)/stm32_vector_table.c
test_telemetry.CFLAGS += -DUSE_LED
test_telemetry.srcs += $(SRC_ARCH)/led_hw.c
test_telemetry.CFLAGS += -DUSE_SYS_TIME
test_telemetry.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
test_telemetry.CFLAGS += -DSYS_TIME_LED=1
test_telemetry.srcs   += sys_time.c $(SRC_ARCH)/sys_time_hw.c
test_telemetry.CFLAGS += -DUSE_$(MODEM_PORT)
test_telemetry.CFLAGS += -D$(MODEM_PORT)_BAUD=$(MODEM_BAUD)
test_telemetry.srcs += downlink.c pprz_transport.c
test_telemetry.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=$(MODEM_PORT)
test_telemetry.srcs   += $(SRC_ARCH)/uart_hw.c


#
# test_baro : reads barometers and sends values over telemetry
#
# configuration
#   MODEM_PORT :
#   MODEM_BAUD :
#
test_baro.ARCHDIR   = $(ARCHI)
test_baro.TARGET    = test_baro
test_baro.TARGETDIR = test_baro
test_baro.CFLAGS = -I$(SRC_LISA) -I$(SRC_ARCH) -I$(PAPARAZZI_SRC)/conf -DPERIPHERALS_AUTO_INIT
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
test_baro.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=$(MODEM_PORT)
test_baro.srcs   += downlink.c pprz_transport.c
test_baro.CFLAGS += -DUSE_$(MODEM_PORT) -D$(MODEM_PORT)_BAUD=$(MODEM_BAUD)
test_baro.srcs   += $(SRC_ARCH)/uart_hw.c
test_baro.srcs   += $(SRC_LISA)/lisa_baro.c
test_baro.CFLAGS += -DUSE_I2C2
test_baro.srcs   += i2c.c $(SRC_ARCH)/i2c_hw.c


#
# test_spektrum : sends RADIO_CONTROL messages on telemetry
#
# configuration
#   MODEM_PORT :
#   MODEM_BAUD :
#   RADIO_CONROL_LINK :
#
#
test_spektrum.ARCHDIR = $(ARCHI)
test_spektrum.TARGET = test_spektrum
test_spektrum.TARGETDIR = test_spektrum
test_spektrum.CFLAGS += -I$(SRC_ARCH) -I$(SRC_BOOZ) -I$(SRC_BOOZ_ARCH) -I$(PAPARAZZI_SRC)/conf -DPERIPHERALS_AUTO_INIT
test_spektrum.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG) 
test_spektrum.srcs   += $(SRC_BOOZ_TEST)/booz2_test_radio_control.c \
                        $(SRC_ARCH)/stm32_exceptions.c              \
                        $(SRC_ARCH)/stm32_vector_table.c

test_spektrum.CFLAGS += -DUSE_LED
test_spektrum.srcs   += $(SRC_ARCH)/led_hw.c
test_spektrum.CFLAGS += -DUSE_SYS_TIME
test_spektrum.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))'
test_spektrum.CFLAGS += -DSYS_TIME_LED=1
test_spektrum.srcs   += sys_time.c $(SRC_ARCH)/sys_time_hw.c
test_spektrum.CFLAGS += -DUSE_$(MODEM_PORT) -D$(MODEM_PORT)_BAUD=$(MODEM_BAUD)
test_spektrum.srcs   += $(SRC_ARCH)/uart_hw.c
test_spektrum.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=$(MODEM_PORT)
test_spektrum.srcs   += downlink.c pprz_transport.c
test_spektrum.CFLAGS += -DUSE_RADIO_CONTROL
test_spektrum.CFLAGS += -DRADIO_CONTROL_LED=$(RADIO_CONTROL_LED)
test_spektrum.CFLAGS += -DRADIO_CONTROL_TYPE_H=\"radio_control/booz_radio_control_spektrum.h\"
test_spektrum.CFLAGS += -DRADIO_CONTROL_SPEKTRUM_MODEL_H=\"radio_control/booz_radio_control_spektrum_dx7se.h\"
test_spektrum.CFLAGS += -DRADIO_CONTROL_LINK=$(RADIO_CONTROL_LINK)
test_spektrum.CFLAGS += -DUSE_$(RADIO_CONTROL_LINK) -D$(RADIO_CONTROL_LINK)_BAUD=B115200
test_spektrum.srcs   += $(SRC_BOOZ)/booz_radio_control.c                                 \
                        $(SRC_BOOZ)/radio_control/booz_radio_control_spektrum.c          \
	                $(SRC_BOOZ_ARCH)/radio_control/booz_radio_control_spektrum_arch.c

