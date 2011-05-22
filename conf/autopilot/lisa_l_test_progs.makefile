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

ARCH=stm32
SRC_ARCH=arch/$(ARCH)
SRC_LISA=lisa
SRC_LISA_ARCH=$(SRC_LISA)/arch/$(ARCH)
SRC_BOOZ=booz
SRC_BOOZ_ARCH=$(SRC_BOOZ)/arch/$(ARCH)
SRC_BOOZ_TEST = $(SRC_BOOZ)/test
#SRC_ROTORCRAFT=rotorcraft
SRC_BOARD=boards/$(BOARD)

SRC_FIRMWARE=firmwares/rotorcraft
SRC_SUBSYSTEMS=subsystems
SRC_AIRBORNE=.

#
# default configuration expected from the board files
#
# SYS_TIME_LED = 1
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
test_uart.ARCHDIR = $(ARCH)
test_uart.CFLAGS = -I$(SRC_LISA) -I$(ARCH) -DPERIPHERALS_AUTO_INIT
test_uart.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_uart.srcs = $(SRC_AIRBORNE)/mcu.c \
                 $(SRC_ARCH)/mcu_arch.c \
                 $(SRC_LISA)/test_uart.c         \
                 $(SRC_ARCH)/stm32_exceptions.c  \
                 $(SRC_ARCH)/stm32_vector_table.c
test_uart.CFLAGS += -DUSE_LED -DUSE_UART
test_uart.srcs += $(SRC_ARCH)/led_hw.c
test_uart.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=$(SYS_TIME_LED)
test_uart.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
test_uart.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c
test_uart.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600
test_uart.CFLAGS += -DUSE_UART2 -DUART2_BAUD=B57600
test_uart.CFLAGS += -DUSE_UART3 -DUART3_BAUD=B57600
test_uart.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c



#
# test servos
#

SRC_BOOZ_ARCH=$(SRC_BOOZ)/arch/$(ARCH)

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
test_servos.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=$(SYS_TIME_LED)
test_servos.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))'
test_servos.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c

test_servos.srcs += $(SRC_FIRMWARE)/actuators/actuators_pwm.c $(SRC_FIRMWARE)/actuators/arch/$(ARCH)/actuators_pwm_arch.c


#
# test_telemetry : Sends ALIVE telemetry messages
#
# configuration
#   MODEM_PORT :
#   MODEM_BAUD :
#
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
# test_baro : reads barometers and sends values over telemetry
#
# configuration
#   SYS_TIME_LED
#   MODEM_PORT
#   MODEM_BAUD
#
test_baro.ARCHDIR = $(ARCH)
test_baro.CFLAGS  = -I$(SRC_LISA) -I$(SRC_ARCH) -I$(SRC_BOARD) -DPERIPHERALS_AUTO_INIT
test_baro.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_baro.srcs = $(SRC_AIRBORNE)/mcu.c \
                 $(SRC_ARCH)/mcu_arch.c \
                 $(SRC_BOARD)/test_baro.c         \
                 $(SRC_ARCH)/stm32_exceptions.c   \
                 $(SRC_ARCH)/stm32_vector_table.c
test_baro.CFLAGS += -DUSE_LED
test_baro.srcs   += $(SRC_ARCH)/led_hw.c
test_baro.CFLAGS += -DUSE_SYS_TIME
test_baro.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
test_baro.CFLAGS += -DSYS_TIME_LED=$(SYS_TIME_LED)
test_baro.srcs   += sys_time.c $(SRC_ARCH)/sys_time_hw.c
test_baro.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=$(MODEM_PORT)
test_baro.srcs   += downlink.c pprz_transport.c
test_baro.CFLAGS += -DUSE_$(MODEM_PORT) -D$(MODEM_PORT)_BAUD=$(MODEM_BAUD)
test_baro.srcs   += $(SRC_ARCH)/mcu_periph/uart_arch.c
test_baro.srcs   += $(SRC_BOARD)/baro_board.c
test_baro.CFLAGS += -DUSE_I2C2
test_baro.srcs   += mcu_periph/i2c.c $(SRC_ARCH)/mcu_periph/i2c_arch.c


#
# test_rc_spektrum : sends RADIO_CONTROL messages on telemetry
#
# configuration
#   SYS_TIME_LED
#   MODEM_PORT
#   MODEM_BAUD
#   RADIO_CONTROL_LED
#   RADIO_CONROL_SPEKTRUM_PRIMARY_PORT
#
test_rc_spektrum.ARCHDIR   = $(ARCH)

test_rc_spektrum.CFLAGS += -I$(SRC_ARCH) -I$(SRC_BOOZ) -I$(SRC_BOOZ_ARCH) -DPERIPHERALS_AUTO_INIT
test_rc_spektrum.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_rc_spektrum.srcs   += $(SRC_AIRBORNE)/mcu.c                \
                           $(SRC_ARCH)/mcu_arch.c               \
                           test/subsystems/test_radio_control.c \
                           $(SRC_ARCH)/stm32_exceptions.c       \
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
test_rc_spektrum.CFLAGS += -DOVERRIDE_$(RADIO_CONTROL_SPEKTRUM_PRIMARY_PORT)_IRQ_HANDLER -DUSE_TIM6_IRQ
test_rc_spektrum.srcs   += $(SRC_SUBSYSTEMS)/radio_control.c                                 \
               subsystems/radio_control/spektrum.c          \
               $(SRC_ARCH)/subsystems/radio_control/spektrum_arch.c


#
# test_rc_ppm
#
# configuration
#   SYS_TIME_LED
#   MODEM_PORT
#   MODEM_BAUD
#   RADIO_CONTROL_LED
#
test_rc_ppm.ARCHDIR   = $(ARCH)

test_rc_ppm.CFLAGS += -I$(SRC_BOOZ) -I$(SRC_BOOZ_ARCH) -I$(SRC_BOARD)
test_rc_ppm.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_rc_ppm.CFLAGS += -DPERIPHERALS_AUTO_INIT
test_rc_ppm.srcs   += $(SRC_AIRBORNE)/mcu.c \
                 $(SRC_ARCH)/mcu_arch.c \
                 $(SRC_BOOZ)/test/booz2_test_radio_control.c \
                      $(SRC_ARCH)/stm32_exceptions.c              \
                      $(SRC_ARCH)/stm32_vector_table.c

test_rc_ppm.CFLAGS += -DUSE_LED
test_rc_ppm.srcs   += $(SRC_ARCH)/led_hw.c
test_rc_ppm.CFLAGS += -DUSE_SYS_TIME
test_rc_ppm.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))'
test_rc_ppm.CFLAGS += -DSYS_TIME_LED=$(SYS_TIME_LED)
test_rc_ppm.srcs   += sys_time.c $(SRC_ARCH)/sys_time_hw.c
test_rc_ppm.CFLAGS += -DUSE_$(MODEM_PORT) -D$(MODEM_PORT)_BAUD=$(MODEM_BAUD)
test_rc_ppm.srcs   += $(SRC_ARCH)/mcu_periph/uart_arch.c
test_rc_ppm.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=$(MODEM_PORT)
test_rc_ppm.srcs   += downlink.c pprz_transport.c
test_rc_ppm.CFLAGS += -DRADIO_CONTROL
test_rc_ppm.CFLAGS += -DRADIO_CONTROL_LED=$(RADIO_CONTROL_LED)
test_rc_ppm.CFLAGS += -DRADIO_CONTROL_TYPE_H=\"subsystems/radio_control/ppm.h\"
test_rc_ppm.CFLAGS += -DRADIO_CONTROL_TYPE_PPM
test_rc_ppm.srcs   += $(SRC_SUBSYSTEMS)/radio_control.c \
                      $(SRC_SUBSYSTEMS)/radio_control/ppm.c \
                      $(SRC_ARCH)/subsystems/radio_control/ppm_arch.c
test_rc_ppm.CFLAGS += -DUSE_TIM2_IRQ

#
# test_adc
#
# configuration
#   SYS_TIME_LED
#   MODEM_PORT
#   MODEM_BAUD
#
test_adc.ARCHDIR = $(ARCH)
test_adc.CFLAGS = -I$(SRC_LISA) -I$(ARCH) -DPERIPHERALS_AUTO_INIT
test_adc.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)

test_adc.srcs =  $(SRC_LISA)/test_adc.c \
		 $(SRC_AIRBORNE)/mcu.c \
                 $(SRC_ARCH)/mcu_arch.c \
                 $(SRC_ARCH)/stm32_exceptions.c \
                 $(SRC_ARCH)/stm32_vector_table.c

test_adc.CFLAGS += -DUSE_LED
test_adc.srcs   += $(SRC_ARCH)/led_hw.c

test_adc.CFLAGS += -DUSE_SYS_TIME
test_adc.CFLAGS +=-DSYS_TIME_LED=$(SYS_TIME_LED)
test_adc.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
test_adc.srcs   += sys_time.c $(SRC_ARCH)/sys_time_hw.c

test_adc.CFLAGS += -DUSE_$(MODEM_PORT) -D$(MODEM_PORT)_BAUD=$(MODEM_BAUD)
test_adc.srcs   += $(SRC_ARCH)/mcu_periph/uart_arch.c
test_adc.CFLAGS += -DDATALINK=PPRZ -DPPRZ_UART=$(MODEM_PORT)

test_adc.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=$(MODEM_PORT)
test_adc.srcs   += downlink.c pprz_transport.c

test_adc.srcs   += $(SRC_ARCH)/mcu_periph/adc_arch.c
test_adc.CFLAGS += -DUSE_AD1 -DUSE_AD1_1 -DUSE_AD1_2 -DUSE_AD1_3 -DUSE_AD1_4
#test_adc.CFLAGS += -DUSE_AD1 -DUSE_AD1_3
test_adc.CFLAGS += -DUSE_ADC1_2_IRQ_HANDLER



#
# common test 
#
# configuration
#   SYS_TIME_LED
#   MODEM_PORT
#   MODEM_BAUD
#
PERIODIC_FREQUENCY = 512

COMMON_TEST_CFLAGS  = -I$(SRC_FIRMWARE) -I$(ARCH) -DPERIPHERALS_AUTO_INIT
COMMON_TEST_CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
COMMON_TEST_SRCS    = $(SRC_AIRBORNE)/mcu.c            \
                      $(SRC_ARCH)/mcu_arch.c           \
                      $(SRC_ARCH)/stm32_exceptions.c   \
                      $(SRC_ARCH)/stm32_vector_table.c
COMMON_TEST_CFLAGS += -DUSE_LED
COMMON_TEST_SRCS   += $(SRC_ARCH)/led_hw.c
COMMON_TEST_CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=$(SYS_TIME_LED)
COMMON_TEST_CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./$(PERIODIC_FREQUENCY).))' 
COMMON_TEST_CFLAGS += -DPERIODIC_FREQUENCY=$(PERIODIC_FREQUENCY)
COMMON_TEST_SRCS   += sys_time.c $(SRC_ARCH)/sys_time_hw.c
COMMON_TEST_CFLAGS += -DUSE_$(MODEM_PORT) -D$(MODEM_PORT)_BAUD=$(MODEM_BAUD)
COMMON_TEST_SRCS   += $(SRC_ARCH)/mcu_periph/uart_arch.c
COMMON_TEST_CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=$(MODEM_PORT)
COMMON_TEST_SRCS   += downlink.c pprz_transport.c
COMMON_TEST_SRCS   += math/pprz_trig_int.c


#
# test IMU b2 v1.1
#
IMU_B2_CFLAGS  = -DIMU_TYPE_H=\"subsystems/imu/imu_b2.h\"
IMU_B2_CFLAGS += -DIMU_B2_MAG_TYPE=IMU_B2_MAG_MS2100 -DIMU_B2_VERSION_1_1
IMU_B2_SRCS    = $(SRC_SUBSYSTEMS)/imu.c
IMU_B2_CFLAGS += -DMAX_1168_DRDY_PORT=$(MAX_1168_DRDY_PORT)
IMU_B2_CFLAGS += -DMAX_1168_DRDY_PORT_SOURCE=$(MAX_1168_DRDY_PORT_SOURCE)
IMU_B2_CFLAGS += -DUSE_SPI2 -DUSE_DMA1_C4_IRQ -DUSE_EXTI2_IRQ -DUSE_SPI2_IRQ
IMU_B2_SRCS   += $(SRC_SUBSYSTEMS)/imu/imu_b2.c $(SRC_ARCH)/subsystems/imu/imu_b2_arch.c
IMU_B2_SRCS   += peripherals/max1168.c $(SRC_ARCH)/peripherals/max1168_arch.c
IMU_B2_SRCS   += peripherals/ms2100.c  $(SRC_ARCH)/peripherals/ms2100_arch.c

test_imu_b2.ARCHDIR = $(ARCH)
test_imu_b2.srcs    = test/subsystems/test_imu.c
test_imu_b2.CFLAGS  = $(COMMON_TEST_CFLAGS)
test_imu_b2.srcs   += $(COMMON_TEST_SRCS)
test_imu_b2.CFLAGS += $(IMU_B2_CFLAGS)
test_imu_b2.srcs   += $(IMU_B2_SRCS)




#
# test IMU b2 v1.2
#
IMU_B2_2_CFLAGS  = -DIMU_TYPE_H=\"subsystems/imu/imu_b2.h\"
IMU_B2_2_CFLAGS += -DIMU_B2_MAG_TYPE=IMU_B2_MAG_HMC5843 -DIMU_B2_VERSION_1_2
IMU_B2_2_SRCS    = $(SRC_SUBSYSTEMS)/imu.c
IMU_B2_2_CFLAGS += -DMAX_1168_DRDY_PORT=$(MAX_1168_DRDY_PORT)
IMU_B2_2_CFLAGS += -DMAX_1168_DRDY_PORT_SOURCE=$(MAX_1168_DRDY_PORT_SOURCE)
IMU_B2_2_CFLAGS += -DUSE_SPI2 -DUSE_DMA1_C4_IRQ -DUSE_EXTI2_IRQ -DUSE_SPI2_IRQ
IMU_B2_2_SRCS   += $(SRC_SUBSYSTEMS)/imu/imu_b2.c $(SRC_ARCH)/subsystems/imu/imu_b2_arch.c
IMU_B2_2_SRCS   += peripherals/max1168.c $(SRC_ARCH)/peripherals/max1168_arch.c
IMU_B2_2_CFLAGS += -DUSE_I2C2
IMU_B2_2_SRCS   += mcu_periph/i2c.c $(SRC_ARCH)/mcu_periph/i2c_arch.c
IMU_B2_2_SRCS   += peripherals/hmc5843.c $(SRC_ARCH)/peripherals/hmc5843_arch.c
IMU_B2_2_CFLAGS += -DUSE_EXTI9_5_IRQ    # Mag Int on PB5

test_imu_b2_2.ARCHDIR = $(ARCH)
test_imu_b2_2.srcs    = test/subsystems/test_imu.c
test_imu_b2_2.CFLAGS  = $(COMMON_TEST_CFLAGS)
test_imu_b2_2.srcs   += $(COMMON_TEST_SRCS)
test_imu_b2_2.CFLAGS += $(IMU_B2_2_CFLAGS)
test_imu_b2_2.srcs   += $(IMU_B2_2_SRCS)





#
# test IMU aspirin
#
IMU_ASPIRIN_CFLAGS = -DIMU_TYPE_H=\"imu/imu_aspirin.h\" -DIMU_OVERRIDE_CHANNELS
IMU_ASPIRIN_SRCS   = $(SRC_SUBSYSTEMS)/imu.c             \
                     $(SRC_SUBSYSTEMS)/imu/imu_aspirin.c \
                     $(SRC_ARCH)/subsystems/imu/imu_aspirin_arch.c
IMU_ASPIRIN_SRCS   += peripherals/hmc5843.c $(SRC_ARCH)/peripherals/hmc5843_arch.c
IMU_ASPIRIN_CFLAGS += -DUSE_I2C2
IMU_ASPIRIN_SRCS   += mcu_periph/i2c.c $(SRC_ARCH)/mcu_periph/i2c_arch.c
IMU_ASPIRIN_CFLAGS += -DUSE_EXTI15_10_IRQ  # Gyro Int on PC14
IMU_ASPIRIN_CFLAGS += -DUSE_EXTI9_5_IRQ    # Mag Int on PB5
IMU_ASPIRIN_CFLAGS += -DUSE_EXTI2_IRQ      # Accel Int on PD2
IMU_ASPIRIN_CFLAGS += -DUSE_DMA1_C4_IRQ    # SPI2 Rx DMA

test_imu_aspirin.ARCHDIR = $(ARCH)
test_imu_aspirin.srcs    = test/subsystems/test_imu.c
test_imu_aspirin.CFLAGS  = $(COMMON_TEST_CFLAGS)
test_imu_aspirin.srcs   += $(COMMON_TEST_SRCS)
test_imu_aspirin.CFLAGS += $(IMU_ASPIRIN_CFLAGS)
test_imu_aspirin.srcs   += $(IMU_ASPIRIN_SRCS)


#
# test AHRS
#
test_ahrs.ARCHDIR = $(ARCH)
test_ahrs.srcs    = test/subsystems/test_ahrs.c
test_ahrs.CFLAGS  = $(COMMON_TEST_CFLAGS)
test_ahrs.srcs   += $(COMMON_TEST_SRCS)
test_ahrs.CFLAGS += $(IMU_ASPIRIN_CFLAGS)
test_ahrs.srcs   += $(IMU_ASPIRIN_SRCS)

#AHRS = ice
AHRS = icq
#AHRS = flq
#AHRS = fcr
#AHRS = fcr2
#AHRS = fcq

test_ahrs.srcs += subsystems/ahrs.c                      \
                  subsystems/ahrs/ahrs_aligner.c

ifeq ($(AHRS), ice)
test_ahrs.CFLAGS += -DFACE_REINJ_1=1024
test_ahrs.CFLAGS += -DAHRS_TYPE_H=\"subsystems/ahrs/ahrs_int_cmpl_euler.h\"
test_ahrs.srcs += subsystems/ahrs/ahrs_int_cmpl_euler.c  \
                  lisa/plug_sys.c
endif

ifeq ($(AHRS), icq)
#test_ahrs.CFLAGS += -DAHRS_TYPE=\"ICQ\"
test_ahrs.CFLAGS += -DAHRS_PROPAGATE_FREQUENCY=512
test_ahrs.CFLAGS += -DAHRS_TYPE_H=\"subsystems/ahrs/ahrs_int_cmpl.h\"
test_ahrs.srcs +=subsystems/ahrs/ahrs_int_cmpl.c 
endif

ifeq ($(AHRS), flq)
test_ahrs.CFLAGS += -DAHRS_TYPE_H=\"subsystems/ahrs/ahrs_float_lkf_quat.h\"
test_ahrs.CFLAGS += -DAHRS_H_X=0.51562740288882 -DAHRS_H_Y=-0.05707735220832 -DAHRS_H_Z=0.85490967783446
test_ahrs.srcs += subsystems/ahrs/ahrs_float_lkf_quat.c
endif

ifeq ($(AHRS), fcr)
test_ahrs.CFLAGS += -DAHRS_TYPE_H=\"subsystems/ahrs/ahrs_float_dcm.h\"
test_ahrs.CFLAGS += -DINS_ROLL_NEUTRAL_DEFAULT=0
test_ahrs.CFLAGS += -DINS_PITCH_NEUTRAL_DEFAULT=0
test_ahrs.CFLAGS += -DAHRS_PROPAGATE_FREQUENCY=512
test_ahrs.CFLAGS += -DDCM_UPDATE_AFTER_PROPAGATE
test_ahrs.srcs += subsystems/ahrs/ahrs_float_dcm.c
endif

ifeq ($(AHRS), fcr2)
test_ahrs.CFLAGS += -DAHRS_TYPE_H=\"subsystems/ahrs/ahrs_float_cmpl_rmat.h\"
test_ahrs.CFLAGS += -DAHRS_H_X=0.51562740288882 -DAHRS_H_Y=-0.05707735220832 -DAHRS_H_Z=0.85490967783446
test_ahrs.CFLAGS += -DAHRS_PROPAGATE_FREQUENCY=512
test_ahrs.srcs += subsystems/ahrs/ahrs_float_cmpl_rmat.c
endif

ifeq ($(AHRS), fcq)
test_ahrs.CFLAGS += -DAHRS_TYPE_H=\"subsystems/ahrs/ahrs_float_cmpl_rmat.h\"
test_ahrs.CFLAGS += -DAHRS_H_X=0.51562740288882 -DAHRS_H_Y=-0.05707735220832 -DAHRS_H_Z=0.85490967783446
test_ahrs.CFLAGS += -DAHRS_PROPAGATE_FREQUENCY=512
test_ahrs.srcs += subsystems/ahrs/ahrs_float_cmpl_quat.c
endif








#
# test hmc5843
#
test_hmc5843.ARCHDIR = $(ARCH)
test_hmc5843.CFLAGS = -I$(SRC_FIRMWARE) -I$(SRC_LISA) -I$(ARCH) -Ibooz -DPERIPHERALS_AUTO_INIT
test_hmc5843.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_hmc5843.srcs = $(SRC_AIRBORNE)/mcu.c \
                 $(SRC_ARCH)/mcu_arch.c \
                 lisa/test/lisa_test_hmc5843.c         \
                    $(SRC_ARCH)/stm32_exceptions.c   \
                    $(SRC_ARCH)/stm32_vector_table.c
test_hmc5843.CFLAGS += -DUSE_LED
test_hmc5843.srcs += $(SRC_ARCH)/led_hw.c
test_hmc5843.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=$(SYS_TIME_LED)
test_hmc5843.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
test_hmc5843.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c

test_hmc5843.CFLAGS += -DUSE_$(MODEM_PORT) -D$(MODEM_PORT)_BAUD=$(MODEM_BAUD)
test_hmc5843.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c

test_hmc5843.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=$(MODEM_PORT)
test_hmc5843.srcs += downlink.c pprz_transport.c

test_hmc5843.CFLAGS += -DUSE_I2C2
test_hmc5843.srcs += mcu_periph/i2c.c $(SRC_ARCH)/mcu_periph/i2c_arch.c
test_hmc5843.CFLAGS += -DIMU_OVERRIDE_CHANNELS
test_hmc5843.CFLAGS += -DUSE_EXTI9_5_IRQ   # Mag Int on PB5


#
# test ITG3200
#
test_itg3200.ARCHDIR = $(ARCH)
test_itg3200.CFLAGS  = -I$(SRC_FIRMWARE) -I$(SRC_LISA) -I$(ARCH) -I$(SRC_BOOZ) -I$(SRC_BOOZ_ARCH) -DPERIPHERALS_AUTO_INIT
test_itg3200.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_itg3200.srcs += $(SRC_AIRBORNE)/mcu.c \
                 $(SRC_ARCH)/mcu_arch.c \
                 lisa/test/lisa_test_itg3200.c \
                       $(SRC_ARCH)/stm32_exceptions.c   \
                       $(SRC_ARCH)/stm32_vector_table.c

test_itg3200.CFLAGS += -DUSE_LED
test_itg3200.srcs += $(SRC_ARCH)/led_hw.c

test_itg3200.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=$(SYS_TIME_LED)
test_itg3200.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))'
test_itg3200.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c

test_itg3200.CFLAGS +=  -DUSE_$(MODEM_PORT) -D$(MODEM_PORT)_BAUD=$(MODEM_BAUD)
test_itg3200.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c

test_itg3200.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=$(MODEM_PORT)
test_itg3200.srcs += downlink.c pprz_transport.c

test_itg3200.CFLAGS += -DUSE_I2C2
test_itg3200.srcs += mcu_periph/i2c.c $(SRC_ARCH)/mcu_periph/i2c_arch.c
test_itg3200.CFLAGS += -DUSE_EXTI15_10_IRQ   # Gyro Int on PC14


#
# test adxl345 with DMA
#
test_adxl345.ARCHDIR = $(ARCH)
test_adxl345.CFLAGS  = -I$(SRC_FIRMWARE) -I$(SRC_LISA) -I$(ARCH) -I$(SRC_BOOZ) -I$(SRC_BOOZ_ARCH) -DPERIPHERALS_AUTO_INIT
test_adxl345.CFLAGS +=  -DBOARD_CONFIG=$(BOARD_CFG)
test_adxl345.srcs += $(SRC_AIRBORNE)/mcu.c \
                 $(SRC_ARCH)/mcu_arch.c \
                 lisa/test/lisa_test_adxl345_dma.c \
                     $(SRC_ARCH)/stm32_exceptions.c   \
		     $(SRC_ARCH)/stm32_vector_table.c

test_adxl345.CFLAGS += -DUSE_LED
test_adxl345.srcs += $(SRC_ARCH)/led_hw.c

test_adxl345.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
test_adxl345.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))'
test_adxl345.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c

test_adxl345.CFLAGS += -DUSE_UART2 -DUART2_BAUD=B57600
test_adxl345.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c

test_adxl345.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart2
test_adxl345.srcs += downlink.c pprz_transport.c

test_adxl345.CFLAGS += -DUSE_EXTI2_IRQ   # Accel Int on PD2
test_adxl345.CFLAGS += -DUSE_DMA1_C4_IRQ # SPI2 Rx DMA



#
# simple test of mikrokopter motor controllers
#
test_esc_mkk_simple.ARCHDIR = $(ARCH)
test_esc_mkk_simple.CFLAGS  = -I$(SRC_FIRMWARE) -I$(SRC_LISA) -I$(ARCH) -DPERIPHERALS_AUTO_INIT
test_esc_mkk_simple.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_esc_mkk_simple.srcs    = $(SRC_AIRBORNE)/mcu.c \
                 $(SRC_ARCH)/mcu_arch.c \
                 test/test_esc_mkk_simple.c		\
			      $(SRC_ARCH)/stm32_exceptions.c   \
			      $(SRC_ARCH)/stm32_vector_table.c
test_esc_mkk_simple.CFLAGS += -DUSE_LED
test_esc_mkk_simple.srcs   += $(SRC_ARCH)/led_hw.c
test_esc_mkk_simple.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=$(SYS_TIME_LED)
test_esc_mkk_simple.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
test_esc_mkk_simple.srcs   += sys_time.c $(SRC_ARCH)/sys_time_hw.c
test_esc_mkk_simple.CFLAGS += -DUSE_I2C2
test_esc_mkk_simple.srcs   += mcu_periph/i2c.c $(SRC_ARCH)/mcu_periph/i2c_arch.c
test_esc_mkk_simple.CFLAGS += -DACTUATORS_MKK_DEV=i2c2


#
# simple test of asctec v1 motor controllers
#
test_esc_asctecv1_simple.ARCHDIR = $(ARCH)
test_esc_asctecv1_simple.CFLAGS = -I$(SRC_FIRMWARE) -I$(SRC_LISA) -I$(ARCH) -DPERIPHERALS_AUTO_INIT
test_esc_asctecv1_simple.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_esc_asctecv1_simple.srcs = $(SRC_AIRBORNE)/mcu.c \
                 $(SRC_ARCH)/mcu_arch.c \
                 test/test_esc_asctecv1_simple.c  \
                                $(SRC_ARCH)/stm32_exceptions.c   \
                                $(SRC_ARCH)/stm32_vector_table.c
test_esc_asctecv1_simple.CFLAGS += -DUSE_LED
test_esc_asctecv1_simple.srcs += $(SRC_ARCH)/led_hw.c
test_esc_asctecv1_simple.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
test_esc_asctecv1_simple.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
test_esc_asctecv1_simple.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c
test_esc_asctecv1_simple.CFLAGS += -DUSE_I2C1
test_esc_asctecv1_simple.srcs += mcu_periph/i2c.c $(SRC_ARCH)/mcu_periph/i2c_arch.c


#
# test actuators mkk
#
test_actuators_mkk.ARCHDIR = $(ARCH)
test_actuators_mkk.CFLAGS = -I$(SRC_LISA) -I$(ARCH) -I$(SRC_BOOZ) -DPERIPHERALS_AUTO_INIT
test_actuators_mkk.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_actuators_mkk.srcs = $(SRC_AIRBORNE)/mcu.c \
                 $(SRC_ARCH)/mcu_arch.c \
                 test/test_actuators.c \
                          $(SRC_ARCH)/stm32_exceptions.c   \
                          $(SRC_ARCH)/stm32_vector_table.c

test_actuators_mkk.CFLAGS += -DUSE_LED
test_actuators_mkk.srcs += $(SRC_ARCH)/led_hw.c

test_actuators_mkk.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
test_actuators_mkk.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
test_actuators_mkk.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c

test_actuators_mkk.CFLAGS += -DUSE_UART2 -DUART2_BAUD=B57600
test_actuators_mkk.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c

test_actuators_mkk.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart2
test_actuators_mkk.srcs += downlink.c pprz_transport.c

test_actuators_mkk.srcs += $(SRC_FIRMWARE)/commands.c
test_actuators_mkk.srcs += $(SRC_FIRMWARE)/actuators/actuators_mkk.c
test_actuators_mkk.CFLAGS += -DACTUATORS_MKK_DEVICE=i2c1
test_actuators_mkk.srcs += $(SRC_FIRMWARE)/actuators/supervision.c
test_actuators_mkk.CFLAGS += -DUSE_I2C1
test_actuators_mkk.srcs += mcu_periph/i2c.c $(SRC_ARCH)/mcu_periph/i2c_arch.c

#
# test actuators asctecv1
#
test_actuators_asctecv1.ARCHDIR = $(ARCH)
test_actuators_asctecv1.CFLAGS = -I$(SRC_LISA) -I$(ARCH) -I$(SRC_BOOZ) -DPERIPHERALS_AUTO_INIT
test_actuators_asctecv1.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_actuators_asctecv1.srcs = $(SRC_AIRBORNE)/mcu.c \
                 $(SRC_ARCH)/mcu_arch.c \
                 test/test_actuators.c            \
                               $(SRC_ARCH)/stm32_exceptions.c   \
                               $(SRC_ARCH)/stm32_vector_table.c

test_actuators_asctecv1.CFLAGS += -DUSE_LED
test_actuators_asctecv1.srcs += $(SRC_ARCH)/led_hw.c

test_actuators_asctecv1.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
test_actuators_asctecv1.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
test_actuators_asctecv1.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c

test_actuators_asctecv1.CFLAGS += -DUSE_UART2 -DUART2_BAUD=B57600
test_actuators_asctecv1.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c

test_actuators_asctecv1.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart2
test_actuators_asctecv1.srcs += downlink.c pprz_transport.c

test_actuators_asctecv1.srcs += $(SRC_FIRMWARE)/commands.c
test_actuators_asctecv1.CFLAGS += -DACTUATORS_ASCTEC_DEVICE=i2c1
test_actuators_asctecv1.srcs += $(SRC_FIRMWARE)/actuators/actuators_asctec.c
test_actuators_asctecv1.CFLAGS += -DUSE_I2C1
test_actuators_asctecv1.srcs += mcu_periph/i2c.c $(SRC_ARCH)/mcu_periph/i2c_arch.c


#
# test bmp085
#
test_bmp085.ARCHDIR = $(ARCH)
test_bmp085.CFLAGS = -I$(SRC_FIRMWARE) -I$(SRC_LISA) -I$(ARCH) -DPERIPHERALS_AUTO_INIT
test_bmp085.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_bmp085.srcs = $(SRC_AIRBORNE)/mcu.c \
                 $(SRC_ARCH)/mcu_arch.c \
                 lisa/test/lisa_test_bmp085.c     \
           $(SRC_ARCH)/stm32_exceptions.c   \
           $(SRC_ARCH)/stm32_vector_table.c
test_bmp085.CFLAGS += -DUSE_LED
test_bmp085.srcs += $(SRC_ARCH)/led_hw.c
test_bmp085.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=$(SYS_TIME_LED)
test_bmp085.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
test_bmp085.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c

test_bmp085.CFLAGS += -DUSE_$(MODEM_PORT) -D$(MODEM_PORT)_BAUD=$(MODEM_BAUD)
test_bmp085.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c

test_bmp085.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=$(MODEM_PORT)
test_bmp085.srcs += downlink.c pprz_transport.c

test_bmp085.CFLAGS += -DUSE_I2C2
test_bmp085.srcs += mcu_periph/i2c.c $(SRC_ARCH)/mcu_periph/i2c_arch.c
#test_bmp085.CFLAGS += -DIMU_OVERRIDE_CHANNELS
#test_bmp085.CFLAGS += -DUSE_EXTI9_5_IRQ   # Mag Int on PB5




#
# Test manual : a simple test with rc and servos - I want to fly lisa/M
#
test_manual.ARCHDIR = $(ARCH)
test_manual.CFLAGS  = -I$(SRC_FIRMWARE) -I$(ARCH) -DPERIPHERALS_AUTO_INIT
test_manual.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_manual.srcs    = $(SRC_AIRBORNE)/mcu.c \
                 $(SRC_ARCH)/mcu_arch.c \
                 test/test_manual.c               \
              $(SRC_ARCH)/stm32_exceptions.c   \
              $(SRC_ARCH)/stm32_vector_table.c
test_manual.CFLAGS += -DUSE_LED
test_manual.srcs   += $(SRC_ARCH)/led_hw.c
test_manual.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=$(SYS_TIME_LED)
test_manual.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
test_manual.srcs   += sys_time.c $(SRC_ARCH)/sys_time_hw.c

test_manual.CFLAGS += -DUSE_$(MODEM_PORT) -D$(MODEM_PORT)_BAUD=$(MODEM_BAUD)
test_manual.srcs   += $(SRC_ARCH)/mcu_periph/uart_arch.c

test_manual.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=$(MODEM_PORT)
test_manual.srcs   += downlink.c pprz_transport.c

test_manual.srcs += $(SRC_FIRMWARE)/commands.c

test_manual.CFLAGS += -I$(SRC_FIRMWARE)/actuators/arch/$(ARCH)
#test_manual.srcs   += $(SRC_FIRMWARE)/actuators/actuators_pwm.c
test_manual.srcs   += $(SRC_FIRMWARE)/actuators/arch/$(ARCH)/actuators_pwm_arch.c
test_manual.srcs   += $(SRC_FIRMWARE)/actuators/actuators_heli.c


test_manual.CFLAGS += -I$(SRC_BOOZ) -I$(SRC_BOOZ)/arch/$(ARCH)
test_manual.CFLAGS += -DRADIO_CONTROL
ifdef RADIO_CONTROL_LED
test_manual.CFLAGS += -DRADIO_CONTROL_LED=$(RADIO_CONTROL_LED)
endif
test_manual.CFLAGS += -DRADIO_CONTROL_BIND_IMPL_FUNC=radio_control_spektrum_try_bind
test_manual.CFLAGS += -DRADIO_CONTROL_TYPE_H=\"subsystems/radio_control/spektrum.h\"
test_manual.CFLAGS += -DRADIO_CONTROL_SPEKTRUM_PRIMARY_PORT=$(RADIO_CONTROL_SPEKTRUM_PRIMARY_PORT)
test_manual.CFLAGS += -DOVERRIDE_$(RADIO_CONTROL_SPEKTRUM_PRIMARY_PORT)_IRQ_HANDLER -DUSE_TIM6_IRQ
test_manual.srcs   += $(SRC_SUBSYSTEMS)/radio_control.c                                 \
                  subsystems/radio_control/spektrum.c          \
                  $(SRC_ARCH)/subsystems/radio_control/spektrum_arch.c



#
# tunnel sw
#
tunnel_sw.ARCHDIR = $(ARCH)
tunnel_sw.CFLAGS += -I$(SRC_LISA) -I$(ARCH) -DPERIPHERALS_AUTO_INIT
tunnel_sw.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
tunnel_sw.srcs   += $(SRC_AIRBORNE)/mcu.c            \
                    $(SRC_ARCH)/mcu_arch.c           \
                    $(SRC_LISA)/tunnel_hw.c          \
                    $(SRC_ARCH)/stm32_exceptions.c   \
                    $(SRC_ARCH)/stm32_vector_table.c
tunnel_sw.CFLAGS += -DUSE_LED
tunnel_sw.srcs   += $(SRC_ARCH)/led_hw.c
tunnel_sw.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=$(SYS_TIME_LED)
tunnel_sw.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
tunnel_sw.srcs   += sys_time.c $(SRC_ARCH)/sys_time_hw.c


#
# tunnel hw
#
tunnel_hw.ARCHDIR = $(ARCH)
tunnel_hw.CFLAGS += -I$(SRC_LISA) -I$(ARCH) -DPERIPHERALS_AUTO_INIT
tunnel_hw.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
tunnel_hw.srcs   += lisa/test/lisa_tunnel.c          \
                    $(SRC_AIRBORNE)/mcu.c            \
                    $(SRC_ARCH)/mcu_arch.c           \
                    $(SRC_ARCH)/stm32_exceptions.c   \
                    $(SRC_ARCH)/stm32_vector_table.c
tunnel_hw.CFLAGS += -DUSE_LED
tunnel_hw.srcs   += $(SRC_ARCH)/led_hw.c
tunnel_hw.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=$(SYS_TIME_LED)
tunnel_hw.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
tunnel_hw.srcs   += sys_time.c $(SRC_ARCH)/sys_time_hw.c
tunnel_hw.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600
tunnel_hw.CFLAGS += -DUSE_UART2 -DUART2_BAUD=B57600
tunnel_hw.srcs   += $(SRC_ARCH)/mcu_periph/uart_arch.c



#
# test_settings :
#
# configuration
#   MODEM_PORT :
#   MODEM_BAUD :
#
test_settings.ARCHDIR = $(ARCH)
test_settings.CFLAGS += -I$(SRC_LISA) -I$(SRC_ARCH) -DPERIPHERALS_AUTO_INIT
test_settings.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_settings.srcs =  test/subsystems/test_settings.c  \
		      $(SRC_AIRBORNE)/mcu.c            \
                      $(SRC_ARCH)/mcu_arch.c           \
                      $(SRC_ARCH)/stm32_exceptions.c   \
                      $(SRC_ARCH)/stm32_vector_table.c
test_settings.CFLAGS += -DUSE_LED
test_settings.srcs   += $(SRC_ARCH)/led_hw.c
test_settings.CFLAGS += -DUSE_SYS_TIME
test_settings.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
test_settings.CFLAGS += -DSYS_TIME_LED=$(SYS_TIME_LED)
test_settings.srcs   += sys_time.c $(SRC_ARCH)/sys_time_hw.c
test_settings.CFLAGS += -DUSE_$(MODEM_PORT)
test_settings.CFLAGS += -D$(MODEM_PORT)_BAUD=$(MODEM_BAUD)
test_settings.srcs   += downlink.c pprz_transport.c
test_settings.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=$(MODEM_PORT)
test_settings.srcs   += $(SRC_ARCH)/mcu_periph/uart_arch.c
test_settings.CFLAGS += -DDATALINK=PPRZ -DPPRZ_UART=$(MODEM_PORT)
test_settings.srcs   += subsystems/settings.c
test_settings.srcs   += $(SRC_ARCH)/subsystems/settings_arch.c
test_settings.CFLAGS += -DUSE_PERSISTENT_SETTINGS
