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

ARCH=lpc21

SRC_ARCH=arch/$(ARCH)
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
test_telemetry.ARCHDIR   = $(ARCH)
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
test_telemetry.srcs   += $(SRC_ARCH)/mcu_periph/uart_arch.c


#
# test_baro : reads barometers and sends values over telemetry
#
# configuration
#   SYS_TIME_LED
#   MODEM_PORT
#   MODEM_BAUD
#
test_baro.ARCHDIR   = $(ARCH)
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
test_baro.srcs   += $(SRC_ARCH)/mcu_periph/uart_arch.c
test_baro.srcs   += $(SRC_BOARD)/baro_board.c
test_baro.CFLAGS += -DBOOZ2_ANALOG_BARO_PERIOD='SYS_TICS_OF_SEC((1./100.))'
test_baro.CFLAGS += -DADC0_VIC_SLOT=2
test_baro.CFLAGS += -DADC1_VIC_SLOT=3
test_baro.srcs += $(SRC_BOOZ)/booz2_analog.c \
				  $(SRC_BOOZ_ARCH)/booz2_analog_hw.c
# tell me why this shit needs to know battery !!!!
test_baro.CFLAGS += -DBOOZ2_ANALOG_BATTERY_PERIOD='SYS_TICS_OF_SEC((1./10.))'
test_baro.srcs += $(SRC_FIRMWARE)/battery.c


#
# test_rc_spektrum :
#
# TODO
#
#
#
#



#
# test rc ppm
#
# configuration
#   SYS_TIME_LED
#   MODEM_PORT
#   MODEM_BAUD
#   RADIO_CONTROL_LED
#
test_rc_ppm.ARCHDIR   = $(ARCH)

test_rc_ppm.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_rc_ppm.CFLAGS += -I$(SRC_BOOZ) -I$(SRC_BOOZ_ARCH) -I$(SRC_BOARD)
test_rc_ppm.CFLAGS += -DPERIPHERALS_AUTO_INIT
test_rc_ppm.srcs   += $(SRC_BOOZ)/test/booz2_test_radio_control.c
test_rc_ppm.CFLAGS += -DUSE_LED
test_rc_ppm.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))'
test_rc_ppm.CFLAGS += -DTIME_LED=$(SYS_TIME_LED)
test_rc_ppm.srcs   += sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c

test_rc_ppm.CFLAGS += -DUSE_$(MODEM_PORT) -D$(MODEM_PORT)_BAUD=$(MODEM_BAUD)
test_rc_ppm.srcs   += $(SRC_ARCH)/mcu_periph/uart_arch.c
test_rc_ppm.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=$(MODEM_PORT)
test_rc_ppm.srcs   += downlink.c pprz_transport.c
test_rc_ppm.CFLAGS += -DRADIO_CONTROL -DRADIO_CONTROL_LED=$(RADIO_CONTROL_LED)
test_rc_ppm.CFLAGS += -DRADIO_CONTROL_TYPE_H=\"subsystems/radio_control/ppm.h\"
test_rc_ppm.CFLAGS += -DRADIO_CONTROL_TYPE_PPM
test_rc_ppm.srcs   += subsystmes/radio_control.c \
					  subsystems/radio_control/ppm.c \
					  $(SRC_ARCH)/subsystems/radio_control/ppm_arch.c



#
# simple test of mikrokopter motor controllers
#
test_esc_mkk_simple.ARCHDIR = $(ARCH)
test_esc_mkk_simple.CFLAGS = -I$(SRC_LISA) -I$(ARCH) -DPERIPHERALS_AUTO_INIT
test_esc_mkk_simple.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_esc_mkk_simple.srcs = test/test_esc_mkk_simple.c		\
						   $(SRC_ARCH)/armVIC.c
test_esc_mkk_simple.CFLAGS += -DUSE_LED
test_esc_mkk_simple.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
test_esc_mkk_simple.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
test_esc_mkk_simple.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c
test_esc_mkk_simple.CFLAGS += -DACTUATORS_MKK_DEV=i2c0
test_esc_mkk_simple.CFLAGS += -DUSE_I2C0
test_esc_mkk_simple.CFLAGS += -DI2C0_SCLL=150 -DI2C0_SCLH=150 -DI2C0_VIC_SLOT=10
test_esc_mkk_simple.srcs += i2c.c $(SRC_ARCH)/i2c_hw.c


#
# test actuators mkk
#
test_actuators_mkk.ARCHDIR = $(ARCH)
test_actuators_mkk.CFLAGS = -I$(SRC_BOOZ) -I$(SRC_BOOZ_ARCH) -DPERIPHERALS_AUTO_INIT
test_actuators_mkk.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_actuators_mkk.srcs = test/test_actuators.c \
						  $(SRC_ARCH)/armVIC.c

test_actuators_mkk.CFLAGS += -DUSE_LED

test_actuators_mkk.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=$(SYS_TIME_LED)
test_actuators_mkk.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
test_actuators_mkk.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c

test_actuators_mkk.CFLAGS += -DUSE_$(MODEM_PORT) -D$(MODEM_PORT)_BAUD=$(MODEM_BAUD)
test_actuators_mkk.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c

test_actuators_mkk.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=$(MODEM_PORT)
test_actuators_mkk.srcs += downlink.c pprz_transport.c

test_actuators_mkk.srcs += $(SRC_FIRMWARE)/commands.c
test_actuators_mkk.srcs += $(SRC_FIRMWARE)/actuators/actuators_mkk.c
test_actuators_mkk.CFLAGS += -DACTUATORS_MKK_DEVICE=i2c0
test_actuators_mkk.srcs += $(SRC_FIRMWARE)/actuators/supervision.c
test_actuators_mkk.CFLAGS += -DACTUATORS_MKK_DEV=i2c0
test_actuators_mkk.CFLAGS += -DUSE_I2C0
test_actuators_mkk.srcs += i2c.c $(SRC_ARCH)/i2c_hw.c
test_actuators_mkk.CFLAGS += -DUSE_I2C0 -DI2C0_SCLL=150 -DI2C0_SCLH=150 -DI2C0_VIC_SLOT=10


#
# test ami601
#
test_ami601.ARCHDIR = $(ARCH)
test_ami601.CFLAGS = -I$(SRC_BOOZ) -I$(SRC_BOOZ_ARCH) -DPERIPHERALS_AUTO_INIT
test_ami601.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_ami601.srcs = test/peripherals/test_ami601.c \
				   $(SRC_ARCH)/armVIC.c

test_ami601.CFLAGS += -DUSE_LED

test_ami601.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=$(SYS_TIME_LED)
test_ami601.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
test_ami601.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c

test_ami601.CFLAGS += -DUSE_$(MODEM_PORT) -D$(MODEM_PORT)_BAUD=$(MODEM_BAUD)
test_ami601.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c

test_ami601.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=$(MODEM_PORT)
test_ami601.srcs += downlink.c pprz_transport.c

test_ami601.CFLAGS += -DUSE_AMI601
test_ami601.srcs += peripherals/ami601.c
test_ami601.CFLAGS += -DUSE_I2C1  -DI2C1_SCLL=150 -DI2C1_SCLH=150 -DI2C1_VIC_SLOT=11
test_ami601.srcs += i2c.c $(SRC_ARCH)/i2c_hw.c
