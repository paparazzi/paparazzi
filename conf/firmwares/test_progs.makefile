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
#  Common test programs for the all LPC21 and STM32 boards
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
PERIODIC_FREQUENCY ?= 512

COMMON_TEST_CFLAGS  = -I$(SRC_BOARD) -DBOARD_CONFIG=$(BOARD_CFG)
COMMON_TEST_CFLAGS += -DPERIPHERALS_AUTO_INIT
COMMON_TEST_SRCS    = mcu.c $(SRC_ARCH)/mcu_arch.c
COMMON_TEST_CFLAGS += -DUSE_SYS_TIME
ifneq ($(SYS_TIME_LED),none)
  COMMON_TEST_CFLAGS += -DSYS_TIME_LED=$(SYS_TIME_LED)
endif
COMMON_TEST_CFLAGS += -DPERIODIC_FREQUENCY=$(PERIODIC_FREQUENCY)
COMMON_TEST_SRCS   += mcu_periph/sys_time.c $(SRC_ARCH)/mcu_periph/sys_time_arch.c

COMMON_TEST_CFLAGS += -DUSE_LED

ifeq ($(ARCH), lpc21)
COMMON_TEST_SRCS += $(SRC_ARCH)/armVIC.c
else ifeq ($(ARCH), stm32)
COMMON_TEST_SRCS += $(SRC_ARCH)/led_hw.c
COMMON_TEST_SRCS += $(SRC_ARCH)/mcu_periph/gpio_arch.c
endif

COMMON_TELEMETRY_CFLAGS  = -DUSE_$(MODEM_PORT) -D$(MODEM_PORT)_BAUD=$(MODEM_BAUD)
COMMON_TELEMETRY_CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=$(MODEM_PORT)
COMMON_TELEMETRY_CFLAGS += -DDefaultPeriodic='&telemetry_Main'
COMMON_TELEMETRY_CFLAGS += -DDATALINK=PPRZ  -DPPRZ_UART=$(MODEM_PORT)
COMMON_TELEMETRY_SRCS    = mcu_periph/uart.c
COMMON_TELEMETRY_SRCS   += $(SRC_ARCH)/mcu_periph/uart_arch.c
COMMON_TELEMETRY_SRCS   += subsystems/datalink/downlink.c subsystems/datalink/pprz_transport.c
COMMON_TELEMETRY_SRCS   += subsystems/datalink/telemetry.c

#COMMON_TEST_SRCS   += math/pprz_trig_int.c



#
# test sys_time
#
ifeq ($(BOARD), lisa_m)
ifeq ($(BOARD_VERSION), 2.0)
LED_DEFINES = -DLED_BLUE=3 -DLED_RED=4 -DLED_GREEN=5
endif
endif
LED_DEFINES ?= -DLED_RED=2 -DLED_GREEN=3

test_sys_time_timer.ARCHDIR = $(ARCH)
test_sys_time_timer.CFLAGS += $(COMMON_TEST_CFLAGS) $(LED_DEFINES)
test_sys_time_timer.srcs   += $(COMMON_TEST_SRCS)
test_sys_time_timer.srcs   += test/mcu_periph/test_sys_time_timer.c

test_sys_time_usleep.ARCHDIR = $(ARCH)
test_sys_time_usleep.CFLAGS += $(COMMON_TEST_CFLAGS) $(LED_DEFINES)
test_sys_time_usleep.srcs   += $(COMMON_TEST_SRCS)
test_sys_time_usleep.srcs   += test/mcu_periph/test_sys_time_usleep.c


#
# test_telemetry : Sends ALIVE telemetry messages
#
# configuration
#   MODEM_PORT :
#   MODEM_BAUD :
#
test_telemetry.ARCHDIR = $(ARCH)
test_telemetry.CFLAGS += $(COMMON_TEST_CFLAGS)
test_telemetry.srcs   += $(COMMON_TEST_SRCS)
test_telemetry.CFLAGS += $(COMMON_TELEMETRY_CFLAGS)
test_telemetry.srcs   += $(COMMON_TELEMETRY_SRCS)
test_telemetry.srcs   += test/test_telemetry.c


#
# test ms2100 mag
#
test_ms2100.ARCHDIR = $(ARCH)
test_ms2100.CFLAGS += $(COMMON_TEST_CFLAGS)
test_ms2100.srcs   += $(COMMON_TEST_SRCS)
test_ms2100.CFLAGS += $(COMMON_TELEMETRY_CFLAGS)
test_ms2100.srcs   += $(COMMON_TELEMETRY_SRCS)

test_ms2100.srcs   += test/peripherals/test_ms2100.c
test_ms2100.srcs   += peripherals/ms2100.c $(SRC_ARCH)/peripherals/ms2100_arch.c
test_ms2100.CFLAGS += -DUSE_SPI -DSPI_MASTER
test_ms2100.srcs   += mcu_periph/spi.c $(SRC_ARCH)/mcu_periph/spi_arch.c
ifeq ($(ARCH), lpc21)
test_ms2100.CFLAGS += -DUSE_SPI1
test_ms2100.CFLAGS += -DUSE_SPI_SLAVE1
test_ms2100.CFLAGS += -DMS2100_SLAVE_IDX=1
test_ms2100.CFLAGS += -DMS2100_SPI_DEV=spi1
test_ms2100.CFLAGS += -DMS2100_DRDY_VIC_SLOT=12
else ifeq ($(ARCH), stm32)
test_ms2100.CFLAGS += -DUSE_SPI2
test_ms2100.CFLAGS += -DUSE_SPI_SLAVE4
test_ms2100.CFLAGS += -DMS2100_SLAVE_IDX=4
test_ms2100.CFLAGS += -DMS2100_SPI_DEV=spi2
endif


#
# test lis302dl accelerometer
#
test_lis302dl.ARCHDIR = $(ARCH)
test_lis302dl.CFLAGS += $(COMMON_TEST_CFLAGS)
test_lis302dl.srcs   += $(COMMON_TEST_SRCS)
test_lis302dl.CFLAGS += $(COMMON_TELEMETRY_CFLAGS)
test_lis302dl.srcs   += $(COMMON_TELEMETRY_SRCS)

test_lis302dl.srcs   += test/peripherals/test_lis302dl_spi.c
test_lis302dl.srcs   += peripherals/lis302dl_spi.c
test_lis302dl.CFLAGS += -DUSE_SPI -DSPI_MASTER
test_lis302dl.srcs   += mcu_periph/spi.c $(SRC_ARCH)/mcu_periph/spi_arch.c

test_lis302dl.CFLAGS += -DUSE_SPI1
test_lis302dl.CFLAGS += -DUSE_SPI_SLAVE2
test_lis302dl.CFLAGS += -DLIS302DL_SLAVE_IDX=2
test_lis302dl.CFLAGS += -DLIS302DL_SPI_DEV=spi1


#
# test PWM actuators by changing the value for each channel via settings
#
test_actuators_pwm.ARCHDIR = $(ARCH)
test_actuators_pwm.CFLAGS += $(COMMON_TEST_CFLAGS)
test_actuators_pwm.srcs   += $(COMMON_TEST_SRCS)
test_actuators_pwm.CFLAGS += $(COMMON_TELEMETRY_CFLAGS)
test_actuators_pwm.srcs   += $(COMMON_TELEMETRY_SRCS)
test_actuators_pwm.srcs   += test/test_actuators_pwm.c
test_actuators_pwm.srcs   += $(SRC_ARCH)/subsystems/actuators/actuators_pwm_arch.c $(SRC_ARCH)/subsystems/actuators/actuators_shared_arch.c


#
# test PWM actuators by simply moving each one from 1ms to 2ms
#
test_actuators_pwm_sin.ARCHDIR = $(ARCH)
test_actuators_pwm_sin.CFLAGS += $(COMMON_TEST_CFLAGS)
test_actuators_pwm_sin.srcs   += $(COMMON_TEST_SRCS)
test_actuators_pwm_sin.srcs   += test/test_actuators_pwm_sin.c
test_actuators_pwm_sin.srcs   += $(SRC_ARCH)/subsystems/actuators/actuators_pwm_arch.c $(SRC_ARCH)/subsystems/actuators/actuators_shared_arch.c


#
# simple test of mikrokopter motor controllers
#
test_esc_mkk_simple.ARCHDIR = $(ARCH)
test_esc_mkk_simple.CFLAGS += $(COMMON_TEST_CFLAGS)
test_esc_mkk_simple.srcs   += $(COMMON_TEST_SRCS)

test_esc_mkk_simple.srcs   += test/test_esc_mkk_simple.c
test_esc_mkk_simple.srcs   += mcu_periph/i2c.c $(SRC_ARCH)/mcu_periph/i2c_arch.c
test_esc_mkk_simple.CFLAGS += -DUSE_I2C2
test_esc_mkk_simple.CFLAGS += -DACTUATORS_MKK_DEV=i2c2


#
# simple test of asctec v1 motor controllers
#
test_esc_asctecv1_simple.ARCHDIR = $(ARCH)
test_esc_asctecv1_simple.CFLAGS += $(COMMON_TEST_CFLAGS)
test_esc_asctecv1_simple.srcs   += $(COMMON_TEST_SRCS)

test_esc_asctecv1_simple.srcs   += test/test_esc_asctecv1_simple.c
test_esc_asctecv1_simple.srcs   += mcu_periph/i2c.c $(SRC_ARCH)/mcu_periph/i2c_arch.c
test_esc_asctecv1_simple.CFLAGS += -DUSE_I2C1


#
# Test manual : a simple test with rc and servos
# add the desired actuators and radio_control subsystem to this target
#
test_manual.ARCHDIR = $(ARCH)
test_manual.CFLAGS += $(COMMON_TEST_CFLAGS)
test_manual.srcs   += $(COMMON_TEST_SRCS)
test_manual.CFLAGS += $(COMMON_TELEMETRY_CFLAGS)
test_manual.srcs   += $(COMMON_TELEMETRY_SRCS)

test_manual.srcs   += subsystems/commands.c
test_manual.srcs   += subsystems/actuators.c
test_manual.srcs   += test/test_manual.c

ifeq ($(TARGET), test_manual)
  ifeq ($(ACTUATORS),)
    $(error ACTUATORS not configured, if your board file has no default, configure in your airframe file)
  else
    include $(CFG_SHARED)/$(ACTUATORS).makefile
  endif
endif
