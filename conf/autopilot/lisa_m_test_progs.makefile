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
#  Test program for the Lisa/M board
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

ARCH=stm32
SRC_ARCH=arch/$(ARCH)
SRC_LISA=lisa
SRC_LISA_ARCH=$(SRC_LISA)/arch/$(ARCH)
#SRC_ROTORCRAFT=rotorcraft
SRC_BOARD=boards/$(BOARD)

SRC_FIRMWARE=firmwares/rotorcraft
SRC_SUBSYSTEMS=subsystems
SRC_AIRBORNE=.

#
# default configuration expected from the board files
#
SYS_TIME_LED = 1
# MODEM_PORT   = UART2
# MODEM_BAUD   = B57600

#
# test leds
#
test_led.ARCHDIR = $(ARCH)
test_led.CFLAGS += -I$(SRC_LISA) -I$(ARCH) -DPERIPHERALS_AUTO_INIT
test_led.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_led.srcs += $(SRC_AIRBORNE)/mcu.c \
                 $(SRC_ARCH)/mcu_arch.c \
                 $(SRC_LISA)/test_led.c           \
                 $(SRC_ARCH)/stm32_exceptions.c   \
                 $(SRC_ARCH)/stm32_vector_table.c
test_led.CFLAGS += -DUSE_LED
test_led.srcs += $(SRC_ARCH)/led_hw.c

#
# test uart
#
test_uart_lisam.ARCHDIR = $(ARCH)
test_uart_lisam.CFLAGS = -I$(SRC_LISA) -I$(ARCH) -DPERIPHERALS_AUTO_INIT
test_uart_lisam.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_uart_lisam.srcs = $(SRC_AIRBORNE)/mcu.c \
                 $(SRC_ARCH)/mcu_arch.c \
                 $(SRC_LISA)/test_uart_lisam.c         \
                 $(SRC_ARCH)/stm32_exceptions.c  \
                 $(SRC_ARCH)/stm32_vector_table.c
test_uart_lisam.CFLAGS += -DUSE_LED -DUSE_UART
test_uart_lisam.srcs += $(SRC_ARCH)/led_hw.c
test_uart_lisam.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=$(SYS_TIME_LED)
test_uart_lisam.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
test_uart_lisam.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c
test_uart_lisam.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600
test_uart_lisam.CFLAGS += -DUSE_UART2 -DUART2_BAUD=B57600
test_uart_lisam.CFLAGS += -DUSE_UART3 -DUART3_BAUD=B57600
test_uart_lisam.CFLAGS += -DUSE_UART5 -DUART5_BAUD=B57600
test_uart_lisam.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c


##
## test servos
##
#
#SRC_BOOZ_ARCH=$(SRC_BOOZ)/arch/$(ARCH)
#
test_servos.ARCHDIR = $(ARCH)
test_servos.CFLAGS  = -I$(SRC_FIRMWARE)/actuators/arch/$(ARCH) -I$(SRC_LISA) -I$(ARCH) -DPERIPHERALS_AUTO_INIT
test_servos.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_servos.LDFLAGS += -lm
test_servos.srcs += $(SRC_AIRBORNE)/mcu.c \
                    $(SRC_ARCH)/mcu_arch.c \
                    $(SRC_LISA)/test_servos.c   \
                    $(SRC_ARCH)/stm32_exceptions.c   \
                    $(SRC_ARCH)/stm32_vector_table.c
test_servos.CFLAGS += -DUSE_LED
test_servos.srcs += $(SRC_ARCH)/led_hw.c
test_servos.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=$(SYS_TIME_LED) -DUSE_SERVOS_7AND8
test_servos.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))'
test_servos.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c

test_servos.srcs += $(SRC_FIRMWARE)/actuators/actuators_pwm.c $(SRC_FIRMWARE)/actuators/arch/$(ARCH)/actuators_pwm_arch.c
#
#
##
## test_telemetry : Sends ALIVE telemetry messages
##
## configuration
##   MODEM_PORT :
##   MODEM_BAUD :
##
test_telemetry.ARCHDIR = $(ARCH)
test_telemetry.CFLAGS += -I$(SRC_LISA) -I$(SRC_ARCH) -DPERIPHERALS_AUTO_INIT
test_telemetry.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_telemetry.srcs = $(SRC_AIRBORNE)/mcu.c \
                 $(SRC_ARCH)/mcu_arch.c \
                 test/test_telemetry.c            \
                 $(SRC_ARCH)/stm32_exceptions.c   \
                 $(SRC_ARCH)/stm32_vector_table.c
test_telemetry.CFLAGS += -DUSE_LED
test_telemetry.srcs += $(SRC_ARCH)/led_hw.c
test_telemetry.CFLAGS += -DUSE_SYS_TIME
test_telemetry.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
test_telemetry.CFLAGS += -DSYS_TIME_LED=$(SYS_TIME_LED)
test_telemetry.srcs   += sys_time.c $(SRC_ARCH)/sys_time_hw.c
test_telemetry.CFLAGS += -DUSE_$(MODEM_PORT)
test_telemetry.CFLAGS += -D$(MODEM_PORT)_BAUD=$(MODEM_BAUD)
test_telemetry.srcs += downlink.c pprz_transport.c
test_telemetry.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=$(MODEM_PORT)
test_telemetry.srcs   += $(SRC_ARCH)/mcu_periph/uart_arch.c
#
#
##
## test_baro : reads barometers and sends values over telemetry
##
## configuration
##   SYS_TIME_LED
##   MODEM_PORT
##   MODEM_BAUD
##
#test_baro.ARCHDIR = $(ARCH)
#test_baro.CFLAGS  = -I$(SRC_LISA) -I$(SRC_ARCH) -I$(SRC_BOARD) -DPERIPHERALS_AUTO_INIT
#test_baro.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
#test_baro.srcs = $(SRC_AIRBORNE)/mcu.c \
#                 $(SRC_ARCH)/mcu_arch.c \
#                 $(SRC_BOARD)/test_baro.c         \
#                 $(SRC_ARCH)/stm32_exceptions.c   \
#                 $(SRC_ARCH)/stm32_vector_table.c
#test_baro.CFLAGS += -DUSE_LED
#test_baro.srcs   += $(SRC_ARCH)/led_hw.c
#test_baro.CFLAGS += -DUSE_SYS_TIME
#test_baro.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
#test_baro.CFLAGS += -DSYS_TIME_LED=$(SYS_TIME_LED)
#test_baro.srcs   += sys_time.c $(SRC_ARCH)/sys_time_hw.c
#test_baro.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=$(MODEM_PORT)
#test_baro.srcs   += downlink.c pprz_transport.c
#test_baro.CFLAGS += -DUSE_$(MODEM_PORT) -D$(MODEM_PORT)_BAUD=$(MODEM_BAUD)
#test_baro.srcs   += $(SRC_ARCH)/mcu_periph/uart_arch.c
#test_baro.srcs   += $(SRC_BOARD)/baro_board.c
#test_baro.CFLAGS += -DUSE_I2C2
#test_baro.srcs   += mcu_periph/i2c.c $(SRC_ARCH)/mcu_periph/i2c_arch.c
#
#
##
## test_rc_spektrum : sends RADIO_CONTROL messages on telemetry
##
## configuration
##   SYS_TIME_LED
##   MODEM_PORT
##   MODEM_BAUD
##   RADIO_CONTROL_LED
##   RADIO_CONROL_SPEKTRUM_PRIMARY_PORT
##
test_rc_spektrum.ARCHDIR   = $(ARCH)

test_rc_spektrum.CFLAGS += -I$(SRC_ARCH) -I$(SRC_BOOZ) -I$(SRC_BOOZ_ARCH) -DPERIPHERALS_AUTO_INIT
test_rc_spektrum.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_rc_spektrum.srcs   += $(SRC_AIRBORNE)/mcu.c \
                 $(SRC_ARCH)/mcu_arch.c \
                 booz/test/booz2_test_radio_control.c \
               $(SRC_ARCH)/stm32_exceptions.c              \
               $(SRC_ARCH)/stm32_vector_table.c

test_rc_spektrum.CFLAGS += -DUSE_LED
test_rc_spektrum.srcs   += $(SRC_ARCH)/led_hw.c
test_rc_spektrum.CFLAGS += -DUSE_SYS_TIME
test_rc_spektrum.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))'
test_rc_spektrum.CFLAGS += -DSYS_TIME_LED=$(SYS_TIME_LED)
test_rc_spektrum.srcs   += sys_time.c $(SRC_ARCH)/sys_time_hw.c
test_rc_spektrum.CFLAGS += -DUSE_$(MODEM_PORT) -D$(MODEM_PORT)_BAUD=$(MODEM_BAUD)
test_rc_spektrum.srcs   += $(SRC_ARCH)/mcu_periph/uart_arch.c
test_rc_spektrum.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=$(MODEM_PORT)
test_rc_spektrum.srcs   += downlink.c pprz_transport.c
test_rc_spektrum.CFLAGS += -DRADIO_CONTROL
ifdef RADIO_CONTROL_LED
test_rc_spektrum.CFLAGS += -DRADIO_CONTROL_LED=$(RADIO_CONTROL_LED)
endif
test_rc_spektrum.CFLAGS += -DRADIO_CONTROL_BIND_IMPL_FUNC=radio_control_spektrum_try_bind
test_rc_spektrum.CFLAGS += -DRADIO_CONTROL_TYPE_H=\"subsystems/radio_control/spektrum.h\"
test_rc_spektrum.CFLAGS += -DRADIO_CONTROL_SPEKTRUM_PRIMARY_PORT=$(RADIO_CONTROL_SPEKTRUM_PRIMARY_PORT)
test_rc_spektrum.CFLAGS += -DRADIO_CONTROL_SPEKTRUM_SECONDARY_PORT=$(RADIO_CONTROL_SPEKTRUM_SECONDARY_PORT)
test_rc_spektrum.CFLAGS += -DOVERRIDE_$(RADIO_CONTROL_SPEKTRUM_PRIMARY_PORT)_IRQ_HANDLER -DUSE_TIM6_IRQ
test_rc_spektrum.CFLAGS += -DOVERRIDE_$(RADIO_CONTROL_SPEKTRUM_SECONDARY_PORT)_IRQ_HANDLER
test_rc_spektrum.srcs   += $(SRC_SUBSYSTEMS)/radio_control.c                                 \
               subsystems/radio_control/spektrum.c          \
               $(SRC_ARCH)/subsystems/radio_control/spektrum_arch.c


##
## test_rc_ppm
##
## configuration
##   SYS_TIME_LED
##   MODEM_PORT
##   MODEM_BAUD
##   RADIO_CONTROL_LED
##
#test_rc_ppm.ARCHDIR   = $(ARCH)
#
#test_rc_ppm.CFLAGS += -I$(SRC_BOOZ) -I$(SRC_BOOZ_ARCH) -I$(SRC_BOARD)
#test_rc_ppm.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
#test_rc_ppm.CFLAGS += -DPERIPHERALS_AUTO_INIT
#test_rc_ppm.srcs   += $(SRC_AIRBORNE)/mcu.c \
#                 $(SRC_ARCH)/mcu_arch.c \
#                 $(SRC_BOOZ)/test/booz2_test_radio_control.c \
#                      $(SRC_ARCH)/stm32_exceptions.c              \
#                      $(SRC_ARCH)/stm32_vector_table.c
#
#test_rc_ppm.CFLAGS += -DUSE_LED
#test_rc_ppm.srcs   += $(SRC_ARCH)/led_hw.c
#test_rc_ppm.CFLAGS += -DUSE_SYS_TIME
#test_rc_ppm.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))'
#test_rc_ppm.CFLAGS += -DSYS_TIME_LED=$(SYS_TIME_LED)
#test_rc_ppm.srcs   += sys_time.c $(SRC_ARCH)/sys_time_hw.c
#test_rc_ppm.CFLAGS += -DUSE_$(MODEM_PORT) -D$(MODEM_PORT)_BAUD=$(MODEM_BAUD)
#test_rc_ppm.srcs   += $(SRC_ARCH)/mcu_periph/uart_arch.c
#test_rc_ppm.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=$(MODEM_PORT)
#test_rc_ppm.srcs   += downlink.c pprz_transport.c
#test_rc_ppm.CFLAGS += -DRADIO_CONTROL
#test_rc_ppm.CFLAGS += -DRADIO_CONTROL_LED=$(RADIO_CONTROL_LED)
#test_rc_ppm.CFLAGS += -DRADIO_CONTROL_TYPE_H=\"subsystems/radio_control/ppm.h\"
#test_rc_ppm.CFLAGS += -DRADIO_CONTROL_TYPE_PPM
#test_rc_ppm.srcs   += $(SRC_SUBSYSTEMS)/radio_control.c \
#                      $(SRC_SUBSYSTEMS)/radio_control/ppm.c \
#                      $(SRC_ARCH)/subsystems/radio_control/ppm_arch.c
#test_rc_ppm.CFLAGS += -DUSE_TIM2_IRQ
#
##
## test_adc
##
## configuration
##   SYS_TIME_LED
##   MODEM_PORT
##   MODEM_BAUD
##
#test_adc.ARCHDIR = $(ARCH)
#test_adc.CFLAGS = -I$(SRC_LISA) -I$(ARCH) -DPERIPHERALS_AUTO_INIT
#test_adc.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
#
#test_adc.srcs = $(SRC_AIRBORNE)/mcu.c \
#                 $(SRC_ARCH)/mcu_arch.c \
#                 $(SRC_LISA)/test_adc.c \
#                $(SRC_ARCH)/stm32_exceptions.c \
#                $(SRC_ARCH)/stm32_vector_table.c
#
#test_adc.CFLAGS += -DUSE_LED
#test_adc.srcs   += $(SRC_ARCH)/led_hw.c
#
#test_adc.CFLAGS += -DUSE_SYS_TIME
#test_adc.CFLAGS +=-DSYS_TIME_LED=$(SYS_TIME_LED)
#test_adc.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
#test_adc.srcs   += sys_time.c $(SRC_ARCH)/sys_time_hw.c
#
#test_adc.CFLAGS += -DUSE_$(MODEM_PORT) -D$(MODEM_PORT)_BAUD=$(MODEM_BAUD)
#test_adc.srcs   += $(SRC_ARCH)/mcu_periph/uart_arch.c
#test_adc.CFLAGS += -DDATALINK=PPRZ -DPPRZ_UART=$(MODEM_PORT)
#
#test_adc.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=$(MODEM_PORT)
#test_adc.srcs   += downlink.c pprz_transport.c
#
#test_adc.srcs   += $(SRC_ARCH)/mcu_periph/adc_arch.c
#test_adc.CFLAGS += -DUSE_AD1 -DUSE_AD1_1 -DUSE_AD1_2 -DUSE_AD1_3 -DUSE_AD1_4
#test_adc.CFLAGS += -DUSE_ADC1_2_IRQ_HANDLER
#
##
## test IMU b2
##
## configuration
##   SYS_TIME_LED
##   MODEM_PORT
##   MODEM_BAUD
##
#test_imu_b2.ARCHDIR = $(ARCH)
#test_imu_b2.CFLAGS  = -I$(SRC_LISA) -I$(ARCH) -I$(SRC_BOOZ) -I$(SRC_BOOZ_ARCH) -DPERIPHERALS_AUTO_INIT
#test_imu_b2.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
#test_imu_b2.srcs += $(SRC_AIRBORNE)/mcu.c \
#                 $(SRC_ARCH)/mcu_arch.c \
#                 $(SRC_BOOZ_TEST)/booz_test_imu.c \
#                    $(SRC_ARCH)/stm32_exceptions.c   \
#                    $(SRC_ARCH)/stm32_vector_table.c
#
#test_imu_b2.CFLAGS += -DUSE_LED
#test_imu_b2.srcs += $(SRC_ARCH)/led_hw.c
#
#test_imu_b2.CFLAGS += -DUSE_SYS_TIME
#test_imu_b2.CFLAGS += -DSYS_TIME_LED=$(SYS_TIME_LED)
#test_imu_b2.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))'
#test_imu_b2.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c
#
#test_imu_b2.CFLAGS += -DUSE_$(MODEM_PORT) -D$(MODEM_PORT)_BAUD=$(MODEM_BAUD)
#test_imu_b2.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c
#
#test_imu_b2.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart2
#test_imu_b2.srcs += downlink.c pprz_transport.c
#
#test_imu_b2.srcs += math/pprz_trig_int.c
#
#test_imu_b2.CFLAGS += -DIMU_TYPE_H=\"subsystems/imu/imu_b2.h\"
#test_imu_b2.CFLAGS += -DIMU_B2_MAG_TYPE=IMU_B2_MAG_MS2001 -DIMU_B2_VERSION_1_1
#test_imu_b2.srcs += $(SRC_SUBSYSTEMS)/imu.c
#test_imu_b2.CFLAGS += -DMAX_1168_DRDY_PORT=$(MAX_1168_DRDY_PORT)
#test_imu_b2.CFLAGS += -DMAX_1168_DRDY_PORT_SOURCE=$(MAX_1168_DRDY_PORT_SOURCE)
#test_imu_b2.CFLAGS += -DUSE_SPI2 -DUSE_DMA1_C4_IRQ -DUSE_EXTI2_IRQ -DUSE_SPI2_IRQ
#test_imu_b2.srcs += $(SRC_SUBSYSTEMS)/imu/imu_b2.c $(SRC_ARCH)/subsystems/imu/imu_b2_arch.c
#test_imu_b2.srcs += peripherals/max1168.c $(SRC_ARCH)/peripherals/max1168_arch.c
#test_imu_b2.srcs += peripherals/ms2001.c  $(SRC_ARCH)/peripherals/ms2001_arch.c
#
##
## test IMU b2 1.2
##
## configuration
##   SYS_TIME_LED
##   MODEM_PORT
##   MODEM_BAUD
##
#test_imu_b2_2.ARCHDIR = $(ARCH)
#test_imu_b2_2.CFLAGS  = -I$(SRC_LISA) -I$(ARCH) -I$(SRC_BOOZ) -I$(SRC_BOOZ_ARCH) -DPERIPHERALS_AUTO_INIT
#test_imu_b2_2.CFLAGS +=  -DBOARD_CONFIG=$(BOARD_CFG)
#test_imu_b2_2.srcs += $(SRC_AIRBORNE)/mcu.c \
#                 $(SRC_ARCH)/mcu_arch.c \
#                 $(SRC_BOOZ_TEST)/booz_test_imu.c \
#                    $(SRC_ARCH)/stm32_exceptions.c   \
#                    $(SRC_ARCH)/stm32_vector_table.c
#
#test_imu_b2_2.CFLAGS += -DUSE_LED
#test_imu_b2_2.srcs += $(SRC_ARCH)/led_hw.c
#
#test_imu_b2_2.CFLAGS += -DUSE_SYS_TIME
#test_imu_b2_2.CFLAGS += -DSYS_TIME_LED=$(SYS_TIME_LED)
#test_imu_b2_2.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))'
#test_imu_b2_2.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c
#
#test_imu_b2_2.CFLAGS += -DUSE_$(MODEM_PORT) -D$(MODEM_PORT)_BAUD=$(MODEM_BAUD)
#test_imu_b2_2.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c
#
#test_imu_b2_2.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart2
#test_imu_b2_2.srcs += downlink.c pprz_transport.c
#
#test_imu_b2_2.srcs += math/pprz_trig_int.c
#
#test_imu_b2_2.CFLAGS += -DIMU_TYPE_H=\"subsystems/imu/imu_b2.h\"
#test_imu_b2_2.CFLAGS += -DIMU_B2_MAG_TYPE=IMU_B2_MAG_HMC5843 -DIMU_B2_VERSION_1_2
#test_imu_b2_2.srcs += $(SRC_SUBSYSTEMS)/imu.c
#test_imu_b2_2.CFLAGS += -DMAX_1168_DRDY_PORT=$(MAX_1168_DRDY_PORT)
#test_imu_b2_2.CFLAGS += -DMAX_1168_DRDY_PORT_SOURCE=$(MAX_1168_DRDY_PORT_SOURCE)
#test_imu_b2_2.CFLAGS += -DUSE_SPI2 -DUSE_DMA1_C4_IRQ -DUSE_EXTI2_IRQ -DUSE_SPI2_IRQ
#test_imu_b2_2.srcs += $(SRC_SUBSYSTEMS)/imu/imu_b2.c $(SRC_ARCH)/subsystems/imu/imu_b2_arch.c
#test_imu_b2_2.srcs += peripherals/max1168.c $(SRC_ARCH)/peripherals/max1168_arch.c
#test_imu_b2_2.CFLAGS += -DUSE_I2C2
#test_imu_b2_2.srcs += mcu_periph/i2c.c $(SRC_ARCH)/mcu_periph/i2c_arch.c
#test_imu_b2_2.srcs += peripherals/hmc5843.c $(SRC_ARCH)/peripherals/hmc5843_arch.c
#test_imu_b2_2.CFLAGS += -DUSE_EXTI9_5_IRQ    # Mag Int on PB5
#
#
##
## test_imu_aspirin : test aspirin imu
##
## configuration
##   MODEM_PORT :
##   MODEM_BAUD :
##
test_imu_aspirin.ARCHDIR = $(ARCH)
test_imu_aspirin.CFLAGS += -I$(SRC_LISA) -I$(SRC_ARCH) -DPERIPHERALS_AUTO_INIT
test_imu_aspirin.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_imu_aspirin.srcs = $(SRC_AIRBORNE)/mcu.c \
                 $(SRC_ARCH)/mcu_arch.c \
                 $(SRC_ARCH)/stm32_exceptions.c   \
                 $(SRC_ARCH)/stm32_vector_table.c\
								 booz/test/booz_test_imu.c 

test_imu_aspirin.CFLAGS += -DUSE_LED
test_imu_aspirin.srcs   += $(SRC_ARCH)/led_hw.c
test_imu_aspirin.CFLAGS += -DUSE_SYS_TIME
test_imu_aspirin.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
test_imu_aspirin.CFLAGS += -DSYS_TIME_LED=$(SYS_TIME_LED)
test_imu_aspirin.srcs   += sys_time.c $(SRC_ARCH)/sys_time_hw.c
test_imu_aspirin.CFLAGS += -DUSE_$(MODEM_PORT)
test_imu_aspirin.CFLAGS += -D$(MODEM_PORT)_BAUD=$(MODEM_BAUD)
test_imu_aspirin.srcs   += downlink.c pprz_transport.c
test_imu_aspirin.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=$(MODEM_PORT)
test_imu_aspirin.srcs   += $(SRC_ARCH)/mcu_periph/uart_arch.c
test_imu_aspirin.srcs   += math/pprz_trig_int.c
test_imu_aspirin.CFLAGS += -DIMU_TYPE_H=\"imu/imu_aspirin.h\" -DIMU_OVERRIDE_CHANNELS
test_imu_aspirin.srcs += $(SRC_SUBSYSTEMS)/imu.c             \
                        $(SRC_SUBSYSTEMS)/imu/imu_aspirin.c \
                        $(SRC_ARCH)/subsystems/imu/imu_aspirin_arch.c
test_imu_aspirin.srcs += peripherals/hmc5843.c $(SRC_ARCH)/peripherals/hmc5843_arch.c

test_imu_aspirin.CFLAGS += -DUSE_I2C2
test_imu_aspirin.srcs += mcu_periph/i2c.c $(SRC_ARCH)/mcu_periph/i2c_arch.c
test_imu_aspirin.CFLAGS += -DUSE_EXTI15_10_IRQ  # Gyro Int on PC14
test_imu_aspirin.CFLAGS += -DUSE_EXTI9_5_IRQ    # Mag Int on PB5
test_imu_aspirin.CFLAGS += -DUSE_EXTI2_IRQ      # Accel Int on PD2
test_imu_aspirin.CFLAGS += -DUSE_DMA1_C4_IRQ    # SPI2 Rx DMA
#
##
## test hmc5843
##
#test_hmc5843.ARCHDIR = $(ARCH)
#test_hmc5843.CFLAGS = -I$(SRC_FIRMWARE) -I$(SRC_LISA) -I$(ARCH) -Ibooz -DPERIPHERALS_AUTO_INIT
#test_hmc5843.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
#test_hmc5843.srcs = $(SRC_AIRBORNE)/mcu.c \
#                 $(SRC_ARCH)/mcu_arch.c \
#                 lisa/test/lisa_test_hmc5843.c         \
#                    $(SRC_ARCH)/stm32_exceptions.c   \
#                    $(SRC_ARCH)/stm32_vector_table.c
#test_hmc5843.CFLAGS += -DUSE_LED
#test_hmc5843.srcs += $(SRC_ARCH)/led_hw.c
#test_hmc5843.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=$(SYS_TIME_LED)
#test_hmc5843.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
#test_hmc5843.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c
#
#test_hmc5843.CFLAGS += -DUSE_$(MODEM_PORT) -D$(MODEM_PORT)_BAUD=$(MODEM_BAUD)
#test_hmc5843.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c
#
#test_hmc5843.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=$(MODEM_PORT)
#test_hmc5843.srcs += downlink.c pprz_transport.c
#
#test_hmc5843.CFLAGS += -DUSE_I2C2
#test_hmc5843.srcs += mcu_periph/i2c.c $(SRC_ARCH)/mcu_periph/i2c_arch.c
#test_hmc5843.CFLAGS += -DIMU_OVERRIDE_CHANNELS
#test_hmc5843.CFLAGS += -DUSE_EXTI9_5_IRQ   # Mag Int on PB5
#
#
##
## test ITG3200
##
#test_itg3200.ARCHDIR = $(ARCH)
#test_itg3200.CFLAGS  = -I$(SRC_FIRMWARE) -I$(SRC_LISA) -I$(ARCH) -I$(SRC_BOOZ) -I$(SRC_BOOZ_ARCH) -DPERIPHERALS_AUTO_INIT
#test_itg3200.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
#test_itg3200.srcs += $(SRC_AIRBORNE)/mcu.c \
#                 $(SRC_ARCH)/mcu_arch.c \
#                 lisa/test/lisa_test_itg3200.c \
#                       $(SRC_ARCH)/stm32_exceptions.c   \
#                       $(SRC_ARCH)/stm32_vector_table.c
#
#test_itg3200.CFLAGS += -DUSE_LED
#test_itg3200.srcs += $(SRC_ARCH)/led_hw.c
#
#test_itg3200.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=$(SYS_TIME_LED)
#test_itg3200.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))'
#test_itg3200.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c
#
#test_itg3200.CFLAGS +=  -DUSE_$(MODEM_PORT) -D$(MODEM_PORT)_BAUD=$(MODEM_BAUD)
#test_itg3200.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c
#
#test_itg3200.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=$(MODEM_PORT)
#test_itg3200.srcs += downlink.c pprz_transport.c
#
#test_itg3200.CFLAGS += -DUSE_I2C2
#test_itg3200.srcs += mcu_periph/i2c.c $(SRC_ARCH)/mcu_periph/i2c_arch.c
#test_itg3200.CFLAGS += -DUSE_EXTI15_10_IRQ   # Gyro Int on PC14
#
#
##
## test adxl345 with DMA
##
#test_adxl345.ARCHDIR = $(ARCH)
#test_adxl345.CFLAGS  = -I$(SRC_FIRMWARE) -I$(SRC_LISA) -I$(ARCH) -I$(SRC_BOOZ) -I$(SRC_BOOZ_ARCH) -DPERIPHERALS_AUTO_INIT
#test_adxl345.CFLAGS +=  -DBOARD_CONFIG=$(BOARD_CFG)
#test_adxl345.srcs += $(SRC_AIRBORNE)/mcu.c \
#                 $(SRC_ARCH)/mcu_arch.c \
#                 lisa/test/lisa_test_adxl345_dma.c \
#                     $(SRC_ARCH)/stm32_exceptions.c   \
#		     $(SRC_ARCH)/stm32_vector_table.c
#
#test_adxl345.CFLAGS += -DUSE_LED
#test_adxl345.srcs += $(SRC_ARCH)/led_hw.c
#
#test_adxl345.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
#test_adxl345.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))'
#test_adxl345.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c
#
#test_adxl345.CFLAGS += -DUSE_UART2 -DUART2_BAUD=B57600
#test_adxl345.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c
#
#test_adxl345.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart2
#test_adxl345.srcs += downlink.c pprz_transport.c
#
#test_adxl345.CFLAGS += -DUSE_EXTI2_IRQ   # Accel Int on PD2
#test_adxl345.CFLAGS += -DUSE_DMA1_C4_IRQ # SPI2 Rx DMA
#
#
#
##
## simple test of mikrokopter motor controllers
##
#test_esc_mkk_simple.ARCHDIR = $(ARCH)
#test_esc_mkk_simple.CFLAGS  = -I$(SRC_FIRMWARE) -I$(SRC_LISA) -I$(ARCH) -DPERIPHERALS_AUTO_INIT
#test_esc_mkk_simple.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
#test_esc_mkk_simple.srcs    = $(SRC_AIRBORNE)/mcu.c \
#                 $(SRC_ARCH)/mcu_arch.c \
#                 test/test_esc_mkk_simple.c		\
#			      $(SRC_ARCH)/stm32_exceptions.c   \
#			      $(SRC_ARCH)/stm32_vector_table.c
#test_esc_mkk_simple.CFLAGS += -DUSE_LED
#test_esc_mkk_simple.srcs   += $(SRC_ARCH)/led_hw.c
#test_esc_mkk_simple.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=$(SYS_TIME_LED)
#test_esc_mkk_simple.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
#test_esc_mkk_simple.srcs   += sys_time.c $(SRC_ARCH)/sys_time_hw.c
#test_esc_mkk_simple.CFLAGS += -DUSE_I2C2
#test_esc_mkk_simple.srcs   += mcu_periph/i2c.c $(SRC_ARCH)/mcu_periph/i2c_arch.c
#test_esc_mkk_simple.CFLAGS += -DACTUATORS_MKK_DEV=i2c2
#
#
##
## simple test of asctec v1 motor controllers
##
#test_esc_asctecv1_simple.ARCHDIR = $(ARCH)
#test_esc_asctecv1_simple.CFLAGS = -I$(SRC_FIRMWARE) -I$(SRC_LISA) -I$(ARCH) -DPERIPHERALS_AUTO_INIT
#test_esc_asctecv1_simple.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
#test_esc_asctecv1_simple.srcs = $(SRC_AIRBORNE)/mcu.c \
#                 $(SRC_ARCH)/mcu_arch.c \
#                 test/test_esc_asctecv1_simple.c  \
#                                $(SRC_ARCH)/stm32_exceptions.c   \
#                                $(SRC_ARCH)/stm32_vector_table.c
#test_esc_asctecv1_simple.CFLAGS += -DUSE_LED
#test_esc_asctecv1_simple.srcs += $(SRC_ARCH)/led_hw.c
#test_esc_asctecv1_simple.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
#test_esc_asctecv1_simple.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
#test_esc_asctecv1_simple.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c
#test_esc_asctecv1_simple.CFLAGS += -DUSE_I2C1
#test_esc_asctecv1_simple.srcs += mcu_periph/i2c.c $(SRC_ARCH)/mcu_periph/i2c_arch.c
#
#
##
## test actuators mkk
##
#test_actuators_mkk.ARCHDIR = $(ARCH)
#test_actuators_mkk.CFLAGS = -I$(SRC_LISA) -I$(ARCH) -I$(SRC_BOOZ) -DPERIPHERALS_AUTO_INIT
#test_actuators_mkk.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
#test_actuators_mkk.srcs = $(SRC_AIRBORNE)/mcu.c \
#                 $(SRC_ARCH)/mcu_arch.c \
#                 test/test_actuators.c \
#                          $(SRC_ARCH)/stm32_exceptions.c   \
#                          $(SRC_ARCH)/stm32_vector_table.c
#
#test_actuators_mkk.CFLAGS += -DUSE_LED
#test_actuators_mkk.srcs += $(SRC_ARCH)/led_hw.c
#
#test_actuators_mkk.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
#test_actuators_mkk.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
#test_actuators_mkk.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c
#
#test_actuators_mkk.CFLAGS += -DUSE_UART2 -DUART2_BAUD=B57600
#test_actuators_mkk.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c
#
#test_actuators_mkk.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart2
#test_actuators_mkk.srcs += downlink.c pprz_transport.c
#
#test_actuators_mkk.srcs += $(SRC_BOOZ)/booz2_commands.c
#test_actuators_mkk.srcs += $(SRC_FIRMWARE)/actuators/actuators_mkk.c
#test_actuators_mkk.CFLAGS += -DACTUATORS_MKK_DEVICE=i2c1
#test_actuators_mkk.srcs += $(SRC_FIRMWARE)/actuators/supervision.c
#test_actuators_mkk.CFLAGS += -DUSE_I2C1
#test_actuators_mkk.srcs += mcu_periph/i2c.c $(SRC_ARCH)/mcu_periph/i2c_arch.c
#
##
## test actuators asctecv1
##
#test_actuators_asctecv1.ARCHDIR = $(ARCH)
#test_actuators_asctecv1.CFLAGS = -I$(SRC_LISA) -I$(ARCH) -I$(SRC_BOOZ) -DPERIPHERALS_AUTO_INIT
#test_actuators_asctecv1.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
#test_actuators_asctecv1.srcs = $(SRC_AIRBORNE)/mcu.c \
#                 $(SRC_ARCH)/mcu_arch.c \
#                 test/test_actuators.c            \
#                               $(SRC_ARCH)/stm32_exceptions.c   \
#                               $(SRC_ARCH)/stm32_vector_table.c
#
#test_actuators_asctecv1.CFLAGS += -DUSE_LED
#test_actuators_asctecv1.srcs += $(SRC_ARCH)/led_hw.c
#
#test_actuators_asctecv1.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
#test_actuators_asctecv1.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
#test_actuators_asctecv1.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c
#
#test_actuators_asctecv1.CFLAGS += -DUSE_UART2 -DUART2_BAUD=B57600
#test_actuators_asctecv1.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c
#
#test_actuators_asctecv1.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart2
#test_actuators_asctecv1.srcs += downlink.c pprz_transport.c
#
#test_actuators_asctecv1.srcs += $(SRC_BOOZ)/booz2_commands.c
#test_actuators_asctecv1.CFLAGS += -DACTUATORS_ASCTEC_DEVICE=i2c1
#test_actuators_asctecv1.srcs += $(SRC_FIRMWARE)/actuators/actuators_asctec.c
#test_actuators_asctecv1.CFLAGS += -DUSE_I2C1
#test_actuators_asctecv1.srcs += mcu_periph/i2c.c $(SRC_ARCH)/mcu_periph/i2c_arch.c
#
#
##
## test bmp085
##
#test_bmp085.ARCHDIR = $(ARCH)
#test_bmp085.CFLAGS = -I$(SRC_FIRMWARE) -I$(SRC_LISA) -I$(ARCH) -DPERIPHERALS_AUTO_INIT
#test_bmp085.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
#test_bmp085.srcs = $(SRC_AIRBORNE)/mcu.c \
#                 $(SRC_ARCH)/mcu_arch.c \
#                 lisa/test/lisa_test_bmp085.c     \
#           $(SRC_ARCH)/stm32_exceptions.c   \
#           $(SRC_ARCH)/stm32_vector_table.c
#test_bmp085.CFLAGS += -DUSE_LED
#test_bmp085.srcs += $(SRC_ARCH)/led_hw.c
#test_bmp085.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=$(SYS_TIME_LED)
#test_bmp085.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
#test_bmp085.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c
#
#test_bmp085.CFLAGS += -DUSE_$(MODEM_PORT) -D$(MODEM_PORT)_BAUD=$(MODEM_BAUD)
#test_bmp085.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c
#
#test_bmp085.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=$(MODEM_PORT)
#test_bmp085.srcs += downlink.c pprz_transport.c
#
#test_bmp085.CFLAGS += -DUSE_I2C2
#test_bmp085.srcs += mcu_periph/i2c.c $(SRC_ARCH)/mcu_periph/i2c_arch.c
##test_bmp085.CFLAGS += -DIMU_OVERRIDE_CHANNELS
##test_bmp085.CFLAGS += -DUSE_EXTI9_5_IRQ   # Mag Int on PB5
#
#
#
##
## Test manual : a simple test with rc and servos - I want to fly lisa/M
##
#test_manual.ARCHDIR = $(ARCH)
#test_manual.CFLAGS  = -I$(SRC_FIRMWARE) -I$(ARCH) -DPERIPHERALS_AUTO_INIT
#test_manual.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
#test_manual.srcs    = $(SRC_AIRBORNE)/mcu.c \
#                 $(SRC_ARCH)/mcu_arch.c \
#                 test/test_manual.c               \
#              $(SRC_ARCH)/stm32_exceptions.c   \
#              $(SRC_ARCH)/stm32_vector_table.c
#test_manual.CFLAGS += -DUSE_LED
#test_manual.srcs   += $(SRC_ARCH)/led_hw.c
#test_manual.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=$(SYS_TIME_LED)
#test_manual.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
#test_manual.srcs   += sys_time.c $(SRC_ARCH)/sys_time_hw.c
#
#test_manual.CFLAGS += -DUSE_$(MODEM_PORT) -D$(MODEM_PORT)_BAUD=$(MODEM_BAUD)
#test_manual.srcs   += $(SRC_ARCH)/mcu_periph/uart_arch.c
#
#test_manual.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=$(MODEM_PORT)
#test_manual.srcs   += downlink.c pprz_transport.c
#
#test_manual.srcs += $(SRC_BOOZ)/booz2_commands.c
#
#test_manual.CFLAGS += -I$(SRC_FIRMWARE)/actuators/arch/$(ARCH)
##test_manual.srcs   += $(SRC_FIRMWARE)/actuators/actuators_pwm.c
#test_manual.srcs   += $(SRC_FIRMWARE)/actuators/arch/$(ARCH)/actuators_pwm_arch.c
#test_manual.srcs   += $(SRC_FIRMWARE)/actuators/actuators_heli.c
#
#
#test_manual.CFLAGS += -I$(SRC_BOOZ) -I$(SRC_BOOZ)/arch/$(ARCH)
#test_manual.CFLAGS += -DRADIO_CONTROL
#ifdef RADIO_CONTROL_LED
#test_manual.CFLAGS += -DRADIO_CONTROL_LED=$(RADIO_CONTROL_LED)
#endif
#test_manual.CFLAGS += -DRADIO_CONTROL_BIND_IMPL_FUNC=radio_control_spektrum_try_bind
#test_manual.CFLAGS += -DRADIO_CONTROL_TYPE_H=\"subsystems/radio_control/spektrum.h\"
#test_manual.CFLAGS += -DRADIO_CONTROL_SPEKTRUM_PRIMARY_PORT=$(RADIO_CONTROL_SPEKTRUM_PRIMARY_PORT)
#test_manual.CFLAGS += -DOVERRIDE_$(RADIO_CONTROL_SPEKTRUM_PRIMARY_PORT)_IRQ_HANDLER -DUSE_TIM6_IRQ
#test_manual.srcs   += $(SRC_SUBSYSTEMS)/radio_control.c                                 \
#                  subsystems/radio_control/spektrum.c          \
#                  $(SRC_ARCH)/subsystems/radio_control/spektrum_arch.c
#
#
#
##
## tunnel
##
#tunnel.ARCHDIR = $(ARCH)
#tunnel.CFLAGS += -I$(SRC_LISA) -I$(ARCH) -DPERIPHERALS_AUTO_INIT
#tunnel.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
#tunnel.srcs   += $(SRC_AIRBORNE)/mcu.c \
#                 $(SRC_ARCH)/mcu_arch.c \
#                 $(SRC_LISA)/tunnel_hw.c          \
#                 $(SRC_ARCH)/stm32_exceptions.c   \
#                 $(SRC_ARCH)/stm32_vector_table.c
#tunnel.CFLAGS += -DUSE_LED
#tunnel.srcs   += $(SRC_ARCH)/led_hw.c
#tunnel.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=$(SYS_TIME_LED)
#tunnel.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
#tunnel.srcs   += sys_time.c $(SRC_ARCH)/sys_time_hw.c
