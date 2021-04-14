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
#  Common test programs for the all STM32 boards
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

COMMON_TEST_CFLAGS  = -I$(SRC_BOARD) -DBOARD_CONFIG=$(BOARD_CFG)
COMMON_TEST_CFLAGS += -DPERIPHERALS_AUTO_INIT
COMMON_TEST_SRCS    = mcu.c $(SRC_ARCH)/mcu_arch.c
ifneq ($(SYS_TIME_LED),none)
  COMMON_TEST_CFLAGS += -DSYS_TIME_LED=$(SYS_TIME_LED)
endif
COMMON_TEST_CFLAGS += -DPERIODIC_FREQUENCY=$(PERIODIC_FREQUENCY)
COMMON_TEST_SRCS   += mcu_periph/sys_time.c $(SRC_ARCH)/mcu_periph/sys_time_arch.c
ifeq ($(ARCH), linux)
# seems that we need to link agains librt for glibc < 2.17
$(TARGET).LDFLAGS += -lrt
endif

COMMON_TEST_CFLAGS += -DUSE_LED

ifeq ($(ARCH), stm32)
COMMON_TEST_SRCS += $(SRC_ARCH)/led_hw.c
COMMON_TEST_SRCS += $(SRC_ARCH)/mcu_periph/gpio_arch.c
endif

# pprz downlink/datalink
COMMON_TELEMETRY_CFLAGS = -DDOWNLINK -DDOWNLINK_TRANSPORT=pprz_tp -DDATALINK=PPRZ
COMMON_TELEMETRY_SRCS   = subsystems/datalink/downlink.c pprzlink/src/pprz_transport.c modules/datalink/pprz_dl.c

# check if we are using UDP
ifneq (,$(findstring UDP, $(MODEM_DEV)))
include $(CFG_SHARED)/udp.makefile
MODEM_PORT_OUT    ?= 4242
MODEM_PORT_IN     ?= 4243
MODEM_BROADCAST   ?= TRUE
UDP_MODEM_PORT_LOWER=$(shell echo $(MODEM_DEV) | tr A-Z a-z)

COMMON_TELEMETRY_CFLAGS += -DUSE_$(MODEM_DEV) -D$(MODEM_DEV)_PORT_OUT=$(MODEM_PORT_OUT) -D$(MODEM_DEV)_PORT_IN=$(MODEM_PORT_IN)
COMMON_TELEMETRY_CFLAGS += -D$(MODEM_DEV)_BROADCAST=$(MODEM_BROADCAST) -D$(MODEM_DEV)_HOST=$(MODEM_HOST)
COMMON_TELEMETRY_CFLAGS += -DPPRZ_UART=$(UDP_MODEM_PORT_LOWER)
COMMON_TELEMETRY_CFLAGS += -DDOWNLINK_DEVICE=$(UDP_MODEM_PORT_LOWER)
else
ifneq (,$(findstring usb, $(MODEM_DEV)))
# via USB
COMMON_TELEMETRY_CFLAGS += -DUSE_USB_SERIAL
COMMON_TELEMETRY_CFLAGS += -DPPRZ_UART=usb_serial
COMMON_TELEMETRY_CFLAGS += -DDOWNLINK_DEVICE=usb_serial
ifeq ($(ARCH), stm32)
COMMON_TELEMETRY_SRCS += $(SRC_ARCH)/usb_ser_hw.c
else
ifneq ($(ARCH), sim)
$(error telemetry_transparent_usb currently only implemented for the stm32)
endif
endif
else
# via UART
#ifeq ($(MODEM_PORT),)
#$(error MODEM_PORT not defined)
#endif
COMMON_TELEMETRY_MODEM_PORT_LOWER=$(shell echo $(MODEM_PORT) | tr A-Z a-z)
COMMON_TELEMETRY_CFLAGS += -DUSE_$(MODEM_PORT) -D$(MODEM_PORT)_BAUD=$(MODEM_BAUD)
COMMON_TELEMETRY_CFLAGS += -DPPRZ_UART=$(COMMON_TELEMETRY_MODEM_PORT_LOWER)
COMMON_TELEMETRY_CFLAGS += -DDOWNLINK_DEVICE=$(COMMON_TELEMETRY_MODEM_PORT_LOWER)
COMMON_TELEMETRY_SRCS  += mcu_periph/uart.c
COMMON_TELEMETRY_SRCS  += $(SRC_ARCH)/mcu_periph/uart_arch.c
ifeq ($(ARCH), linux)
COMMON_TELEMETRY_SRCS  += $(SRC_ARCH)/serial_port.c
endif
endif
endif #UART

#COMMON_TEST_SRCS   += math/pprz_trig_int.c



#
# test sys_time
#
ifeq ($(BOARD), lisa_m)
ifeq ($(BOARD_VERSION), 2.0)
LED_DEFINES = -DLED_BLUE=3 -DLED_RED=4 -DLED_GREEN=5
endif
endif
ifeq ($(BOARD), navstik)
LED_DEFINES = -DLED_RED=1 -DLED_GREEN=2
endif
ifeq ($(BOARD), cc3d)
LED_DEFINES = -DLED_BLUE=1
endif
ifeq ($(BOARD), naze32)
LED_DEFINES = -DLED_RED=1 -DLED_GREEN=2
endif
//LED_DEFINES ?= -DLED_RED=2 -DLED_GREEN=3

test_sys_time_timer.ARCHDIR = $(ARCH)
test_sys_time_timer.CFLAGS += $(COMMON_TEST_CFLAGS) $(LED_DEFINES)
test_sys_time_timer.srcs   += $(COMMON_TEST_SRCS)
test_sys_time_timer.srcs   += test/mcu_periph/test_sys_time_timer.c

test_sys_time_usleep.ARCHDIR = $(ARCH)
test_sys_time_usleep.CFLAGS += $(COMMON_TEST_CFLAGS) $(LED_DEFINES)
test_sys_time_usleep.srcs   += $(COMMON_TEST_SRCS)
test_sys_time_usleep.srcs   += test/mcu_periph/test_sys_time_usleep.c


test_gpio.ARCHDIR = $(ARCH)
test_gpio.CFLAGS += $(COMMON_TEST_CFLAGS)
test_gpio.srcs   += $(COMMON_TEST_SRCS)
test_gpio.srcs   += test/mcu_periph/test_gpio.c


#
# test uart
#
# required configuration:
#   -DUSE_UARTx
#   -DUARTx_BAUD=B57600
#
test_uart.ARCHDIR = $(ARCH)
test_uart.CFLAGS += $(COMMON_TEST_CFLAGS)
test_uart.srcs   += $(COMMON_TEST_SRCS)

#test_uart.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600
#test_uart.CFLAGS += -DUSE_UART2 -DUART2_BAUD=B57600
#test_uart.CFLAGS += -DUSE_UART3 -DUART3_BAUD=B57600
#test_uart.CFLAGS += -DUSE_UART5 -DUART5_BAUD=B57600
test_uart.srcs += mcu_periph/uart.c
test_uart.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c
test_uart.srcs += test/mcu_periph/test_uart.c
ifeq ($(ARCH), linux)
test_uart.srcs += $(SRC_ARCH)/serial_port.c
endif


#
# test uart_echo
#
# required configuration:
#   -DUSE_UARTx
#   -DUARTx_BAUD=B57600
#
test_uart_echo.ARCHDIR = $(ARCH)
test_uart_echo.CFLAGS += $(COMMON_TEST_CFLAGS)
test_uart_echo.srcs   += $(COMMON_TEST_SRCS)
test_uart_echo.srcs += mcu_periph/uart.c
test_uart_echo.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c
test_uart_echo.srcs += test/mcu_periph/test_uart_echo.c
ifeq ($(ARCH), linux)
test_uart_echo.srcs += $(SRC_ARCH)/serial_port.c
endif

#
# test uart_send
#
# required configuration:
#   -DUSE_UARTx
#   -DUARTx_BAUD=B57600
#
test_uart_send.ARCHDIR = $(ARCH)
test_uart_send.CFLAGS += $(COMMON_TEST_CFLAGS)
test_uart_send.srcs   += $(COMMON_TEST_SRCS)
test_uart_send.srcs += mcu_periph/uart.c
test_uart_send.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c
test_uart_send.srcs += test/mcu_periph/test_uart_send.c
ifeq ($(ARCH), linux)
test_uart_send.srcs += $(SRC_ARCH)/serial_port.c
endif

#
# test uart_recv
#
# required configuration:
#   -DUSE_UARTx
#   -DUARTx_BAUD=B57600
#
test_uart_recv.ARCHDIR = $(ARCH)
test_uart_recv.CFLAGS += $(COMMON_TEST_CFLAGS)
test_uart_recv.srcs   += $(COMMON_TEST_SRCS)
test_uart_recv.srcs += mcu_periph/uart.c
test_uart_recv.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c
test_uart_recv.srcs += test/mcu_periph/test_uart_recv.c
ifeq ($(ARCH), linux)
test_uart_recv.srcs += $(SRC_ARCH)/serial_port.c
endif

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
# test_datalink : Sends ALIVE and pong telemetry messages
#
# configuration
#   MODEM_PORT :
#   MODEM_BAUD :
#
test_datalink.ARCHDIR = $(ARCH)
test_datalink.CFLAGS += $(COMMON_TEST_CFLAGS)
test_datalink.srcs   += $(COMMON_TEST_SRCS)
test_datalink.CFLAGS += $(COMMON_DATALINK_CFLAGS)
test_datalink.CFLAGS += $(COMMON_TELEMETRY_CFLAGS)
test_datalink.srcs   += $(COMMON_DATALINK_SRCS)
test_datalink.srcs   += $(COMMON_TELEMETRY_SRCS)
test_datalink.srcs   += test/test_datalink.c


#
# test_math_trig_compressed: Test math trigonometric using compressed data
#
# configuration
#   MODEM_PORT :
#   MODEM_BAUD :
#
test_math_trig_compressed.ARCHDIR = $(ARCH)
test_math_trig_compressed.CFLAGS += $(COMMON_TEST_CFLAGS)
test_math_trig_compressed.srcs   += $(COMMON_TEST_SRCS)
test_math_trig_compressed.CFLAGS += $(COMMON_TELEMETRY_CFLAGS)
test_math_trig_compressed.CFLAGS += -DPPRZ_TRIG_INT_TEST
test_math_trig_compressed.srcs   += $(COMMON_TELEMETRY_SRCS)
test_math_trig_compressed.srcs   += test/test_math_trig_compressed.c math/pprz_trig_int.c


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
ifeq ($(ARCH), stm32)
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


##
## test can interface
##
test_can.ARCHDIR = $(ARCH)
test_can.CFLAGS += $(COMMON_TEST_CFLAGS)
test_can.srcs   += $(COMMON_TEST_SRCS)
test_can.CFLAGS += $(COMMON_TELEMETRY_CFLAGS)
test_can.srcs   += $(COMMON_TELEMETRY_SRCS)
test_can.srcs   += test/test_can.c
test_can.srcs   += mcu_periph/can.c $(SRC_ARCH)/mcu_periph/can_arch.c


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
# only add this so it doesn't fail to build if you also have setup_actuators.xml settings file loaded
# remove me again when we have auto loading of settings according to subsystem/module/target...
test_actuators_pwm.srcs   += subsystems/actuators.c


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


#
# test_baro_board : reads barometers and sends values over telemetry
#
# configuration
#   SYS_TIME_LED
#   MODEM_PORT
#   MODEM_BAUD
#
test_baro_board.ARCHDIR = $(ARCH)
test_baro_board.CFLAGS += $(COMMON_TEST_CFLAGS)
test_baro_board.srcs   += $(COMMON_TEST_SRCS)
test_baro_board.CFLAGS += $(COMMON_TELEMETRY_CFLAGS)
test_baro_board.srcs   += $(COMMON_TELEMETRY_SRCS)
test_baro_board.srcs += test/test_baro_board.c
test_baro_board.srcs += mcu_periph/i2c.c $(SRC_ARCH)/mcu_periph/i2c_arch.c
ifeq ($(TARGET),test_baro_board)
include $(CFG_SHARED)/baro_board.makefile
endif
test_baro_board.CFLAGS += $(BARO_BOARD_CFLAGS)
test_baro_board.srcs += $(BARO_BOARD_SRCS)
ifeq ($(IMU_INIT),1)
test_baro_board.srcs += test/test_baro_board_imu.c
endif


#
# test_adc
#
# configuration
#   SYS_TIME_LED
#   MODEM_PORT
#   MODEM_BAUD
#
test_adc.ARCHDIR = $(ARCH)
test_adc.CFLAGS += $(COMMON_TEST_CFLAGS)
test_adc.srcs   += $(COMMON_TEST_SRCS)
test_adc.CFLAGS += $(COMMON_TELEMETRY_CFLAGS)
test_adc.srcs   += $(COMMON_TELEMETRY_SRCS)
test_adc.srcs   += $(SRC_ARCH)/mcu_periph/adc_arch.c
test_adc.srcs   += test/mcu_periph/test_adc.c


#
# test_imu
#
# add imu subsystem to test_imu target!
#
# configuration
#   SYS_TIME_LED
#   MODEM_PORT
#   MODEM_BAUD
#
test_imu.ARCHDIR = $(ARCH)
test_imu.CFLAGS += $(COMMON_TEST_CFLAGS)
test_imu.srcs   += $(COMMON_TEST_SRCS)
test_imu.CFLAGS += $(COMMON_TELEMETRY_CFLAGS)
test_imu.srcs   += $(COMMON_TELEMETRY_SRCS)
test_imu.srcs   += mcu_periph/i2c.c $(SRC_ARCH)/mcu_periph/i2c_arch.c
test_imu.srcs   += state.c
test_imu.srcs   += test/subsystems/test_imu.c
test_imu.srcs   += math/pprz_geodetic_int.c math/pprz_geodetic_float.c math/pprz_geodetic_double.c math/pprz_trig_int.c math/pprz_orientation_conversion.c math/pprz_algebra_int.c math/pprz_algebra_float.c math/pprz_algebra_double.c


#
# test_ahrs
#
# add imu and ahrs subsystems to test_ahrs target!
#
# configuration
#   SYS_TIME_LED
#   MODEM_PORT
#   MODEM_BAUD
#
test_ahrs.ARCHDIR = $(ARCH)
test_ahrs.CFLAGS += $(COMMON_TEST_CFLAGS)
test_ahrs.srcs   += $(COMMON_TEST_SRCS)
test_ahrs.CFLAGS += $(COMMON_TELEMETRY_CFLAGS)
test_ahrs.srcs   += $(COMMON_TELEMETRY_SRCS)
test_ahrs.srcs   += subsystems/datalink/telemetry.c
test_ahrs.CFLAGS += -DPERIODIC_TELEMETRY
test_ahrs.srcs   += mcu_periph/i2c.c $(SRC_ARCH)/mcu_periph/i2c_arch.c
test_ahrs.srcs   += test/subsystems/test_ahrs.c
test_ahrs.srcs   += state.c
test_ahrs.srcs   += math/pprz_geodetic_int.c math/pprz_geodetic_float.c math/pprz_geodetic_double.c math/pprz_trig_int.c math/pprz_orientation_conversion.c math/pprz_algebra_int.c math/pprz_algebra_float.c math/pprz_algebra_double.c


#
# test_radio_control
#
# add appropriate radio_control subsystem to target!
#
test_radio_control.ARCHDIR = $(ARCH)
test_radio_control.CFLAGS += $(COMMON_TEST_CFLAGS)
test_radio_control.srcs   += $(COMMON_TEST_SRCS)
test_radio_control.CFLAGS += $(COMMON_TELEMETRY_CFLAGS)
test_radio_control.srcs   += $(COMMON_TELEMETRY_SRCS)
test_radio_control.srcs   += test/subsystems/test_radio_control.c


#
# test_settings :
#
# configuration
#   MODEM_PORT :
#   MODEM_BAUD :
#
test_settings.ARCHDIR = $(ARCH)
test_settings.CFLAGS += $(COMMON_TEST_CFLAGS)
test_settings.srcs   += $(COMMON_TEST_SRCS)
test_settings.CFLAGS += $(COMMON_TELEMETRY_CFLAGS)
test_settings.srcs   += $(COMMON_TELEMETRY_SRCS)
test_settings.srcs   += subsystems/settings.c
test_settings.srcs   += $(SRC_ARCH)/subsystems/settings_arch.c
test_settings.srcs   += test/subsystems/test_settings.c
test_settings.CFLAGS += -DUSE_PERSISTENT_SETTINGS


#
# test_module
#
# configuration
#   SYS_TIME_LED
#   MODEM_PORT
#   MODEM_BAUD
#
test_module.ARCHDIR = $(ARCH)
test_module.CFLAGS += $(COMMON_TEST_CFLAGS)
test_module.srcs   += $(COMMON_TEST_SRCS)
test_module.CFLAGS += $(COMMON_TELEMETRY_CFLAGS)
test_module.srcs   += $(COMMON_TELEMETRY_SRCS)
test_module.srcs   += mcu_periph/i2c.c $(SRC_ARCH)/mcu_periph/i2c_arch.c
test_module.srcs   += test/test_module.c


test_eigen.ARCHDIR = $(ARCH)
test_eigen.CFLAGS += $(COMMON_TEST_CFLAGS)
test_eigen.CXXFLAGS += -I$(PAPARAZZI_SRC)/sw/ext/eigen -Wno-shadow
test_eigen.srcs   += $(COMMON_TEST_SRCS)
test_eigen.srcs   += test/test_eigen.cpp
test_eigen.srcs   += pprz_syscalls.c
test_eigen.LDFLAGS += -lstdc++

