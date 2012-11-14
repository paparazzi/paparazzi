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
#  Test program for the Lisa/M and Lisa/L boards
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
                      $(SRC_ARCH)/mcu_arch.c
COMMON_TEST_CFLAGS += -DUSE_SYS_TIME
ifneq ($(SYS_TIME_LED),none)
  COMMON_TEST_CFLAGS += -DSYS_TIME_LED=$(SYS_TIME_LED)
endif
COMMON_TEST_CFLAGS += -DPERIODIC_FREQUENCY=$(PERIODIC_FREQUENCY)
COMMON_TEST_SRCS   += mcu_periph/sys_time.c $(SRC_ARCH)/mcu_periph/sys_time_arch.c

COMMON_TEST_CFLAGS += -DUSE_LED
COMMON_TEST_SRCS   += $(SRC_ARCH)/led_hw.c

COMMON_TELEMETRY_CFLAGS  = -DUSE_$(MODEM_PORT) -D$(MODEM_PORT)_BAUD=$(MODEM_BAUD)
COMMON_TELEMETRY_CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=$(MODEM_PORT)
COMMON_TELEMETRY_SRCS    = mcu_periph/uart.c
COMMON_TELEMETRY_SRCS   += $(SRC_ARCH)/mcu_periph/uart_arch.c
COMMON_TELEMETRY_SRCS   += subsystems/datalink/downlink.c subsystems/datalink/pprz_transport.c

#COMMON_TEST_SRCS   += math/pprz_trig_int.c


#
# test leds
#
test_led.ARCHDIR = $(ARCH)
test_led.CFLAGS  = $(COMMON_TEST_CFLAGS)
test_led.srcs    = $(COMMON_TEST_SRCS)

test_led.CFLAGS += -I$(SRC_LISA)
test_led.srcs   += $(SRC_LISA)/test_led.c


#
# test sys_time
#
test_sys_time.ARCHDIR = $(ARCH)
test_sys_time.CFLAGS  = $(COMMON_TEST_CFLAGS)
test_sys_time.srcs    = $(COMMON_TEST_SRCS)

test_sys_time.CFLAGS += -DLED_RED=2 -DLED_BLUE=3
test_sys_time.srcs   += $(SRC_AIRBORNE)/test/mcu_periph/test_sys_time.c



#
# test uart
#
test_uart.ARCHDIR = $(ARCH)
test_uart.CFLAGS  = $(COMMON_TEST_CFLAGS)
test_uart.srcs    = $(COMMON_TEST_SRCS)

test_uart.CFLAGS += -I$(SRC_LISA) -DUSE_UART
test_uart.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600
test_uart.CFLAGS += -DUSE_UART2 -DUART2_BAUD=B57600
test_uart.CFLAGS += -DUSE_UART3 -DUART3_BAUD=B57600
test_uart.srcs += mcu_periph/uart.c
test_uart.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c
ifeq ($(BOARD), lisa_m)
  test_uart.srcs += $(SRC_LISA)/test_uart_lisam.c
  test_uart.CFLAGS += -DUSE_UART5 -DUART5_BAUD=B57600
else ifeq ($(BOARD), lisa_l)
  test_uart.srcs += $(SRC_LISA)/test_uart_lisal.c
endif


#
# test servos
#
test_servos.ARCHDIR = $(ARCH)
test_servos.CFLAGS  = $(COMMON_TEST_CFLAGS)
test_servos.srcs    = $(COMMON_TEST_SRCS)

test_servos.CFLAGS  += -I$(SRC_LISA)
test_servos.LDFLAGS += -lm
test_servos.srcs    += $(SRC_LISA)/test_servos.c
test_servos.srcs    += $(SRC_ARCH)/subsystems/actuators/actuators_pwm_arch.c
ifeq ($(BOARD), lisa_m)
  test_servos.CFLAGS += -DUSE_SERVOS_7AND8
endif


#
# test_telemetry : Sends ALIVE telemetry messages
#
# configuration
#   MODEM_PORT :
#   MODEM_BAUD :
#
test_telemetry.ARCHDIR = $(ARCH)
test_telemetry.CFLAGS  = $(COMMON_TEST_CFLAGS)
test_telemetry.srcs    = $(COMMON_TEST_SRCS)
test_telemetry.CFLAGS += $(COMMON_TELEMETRY_CFLAGS)
test_telemetry.srcs   += $(COMMON_TELEMETRY_SRCS)

test_telemetry.CFLAGS += -I$(SRC_LISA)
test_telemetry.srcs   += test/test_telemetry.c


#
# test_baro : reads barometers and sends values over telemetry
#
# configuration
#   SYS_TIME_LED
#   MODEM_PORT
#   MODEM_BAUD
#
test_baro.ARCHDIR = $(ARCH)
test_baro.CFLAGS  = $(COMMON_TEST_CFLAGS)
test_baro.srcs    = $(COMMON_TEST_SRCS)
test_baro.CFLAGS += $(COMMON_TELEMETRY_CFLAGS)
test_baro.srcs   += $(COMMON_TELEMETRY_SRCS)

test_baro.CFLAGS += -I$(SRC_LISA) -I$(SRC_BOARD)
test_baro.srcs   += $(SRC_BOARD)/test_baro.c
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
test_rc_spektrum.CFLAGS  = $(COMMON_TEST_CFLAGS)
test_rc_spektrum.srcs    = $(COMMON_TEST_SRCS)
test_rc_spektrum.CFLAGS += $(COMMON_TELEMETRY_CFLAGS)
test_rc_spektrum.srcs   += $(COMMON_TELEMETRY_SRCS)

test_rc_spektrum.srcs   += test/subsystems/test_radio_control.c
test_rc_spektrum.CFLAGS += -DRADIO_CONTROL
ifneq ($(RADIO_CONTROL_LED),none)
  test_rc_spektrum.CFLAGS += -DRADIO_CONTROL_LED=$(RADIO_CONTROL_LED)
endif
test_rc_spektrum.CFLAGS += -DRADIO_CONTROL_BIND_IMPL_FUNC=radio_control_spektrum_try_bind
test_rc_spektrum.CFLAGS += -DRADIO_CONTROL_TYPE_H=\"subsystems/radio_control/spektrum.h\"
test_rc_spektrum.CFLAGS += -DRADIO_CONTROL_SPEKTRUM_PRIMARY_PORT=$(RADIO_CONTROL_SPEKTRUM_PRIMARY_PORT)
test_rc_spektrum.CFLAGS += -DRADIO_CONTROL_SPEKTRUM_SECONDARY_PORT=$(RADIO_CONTROL_SPEKTRUM_SECONDARY_PORT)
test_rc_spektrum.CFLAGS += -DOVERRIDE_$(RADIO_CONTROL_SPEKTRUM_PRIMARY_PORT)_IRQ_HANDLER -DUSE_TIM6_IRQ
test_rc_spektrum.CFLAGS += -DOVERRIDE_$(RADIO_CONTROL_SPEKTRUM_SECONDARY_PORT)_IRQ_HANDLER
test_rc_spektrum.srcs   += $(SRC_SUBSYSTEMS)/radio_control.c
test_rc_spektrum.srcs   += $(SRC_SUBSYSTEMS)/radio_control/spektrum.c
test_rc_spektrum.srcs   += $(SRC_ARCH)/subsystems/radio_control/spektrum_arch.c


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
test_rc_ppm.CFLAGS  = $(COMMON_TEST_CFLAGS)
test_rc_ppm.srcs    = $(COMMON_TEST_SRCS)
test_rc_ppm.CFLAGS += $(COMMON_TELEMETRY_CFLAGS)
test_rc_ppm.srcs   += $(COMMON_TELEMETRY_SRCS)

test_rc_ppm.srcs   += test/subsystems/test_radio_control.c
test_rc_ppm.CFLAGS += -DRADIO_CONTROL
ifneq ($(RADIO_CONTROL_LED),none)
  test_rc_ppm.CFLAGS += -DRADIO_CONTROL_LED=$(RADIO_CONTROL_LED)
endif
test_rc_ppm.CFLAGS += -DRADIO_CONTROL_TYPE_H=\"subsystems/radio_control/ppm.h\"
test_rc_ppm.CFLAGS += -DRADIO_CONTROL_TYPE_PPM
test_rc_ppm.srcs   += $(SRC_SUBSYSTEMS)/radio_control.c
test_rc_ppm.srcs   += $(SRC_SUBSYSTEMS)/radio_control/ppm.c
test_rc_ppm.srcs   += $(SRC_ARCH)/subsystems/radio_control/ppm_arch.c
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
test_adc.CFLAGS  = $(COMMON_TEST_CFLAGS)
test_adc.srcs    = $(COMMON_TEST_SRCS)
test_adc.CFLAGS += $(COMMON_TELEMETRY_CFLAGS)
test_adc.srcs   += $(COMMON_TELEMETRY_SRCS)

test_adc.CFLAGS += -I$(SRC_LISA)
test_adc.srcs   += $(SRC_LISA)/test_adc.c
test_adc.srcs   += $(SRC_ARCH)/mcu_periph/adc_arch.c
test_adc.CFLAGS += -DUSE_AD1 -DUSE_AD1_1 -DUSE_AD1_2 -DUSE_AD1_3 -DUSE_AD1_4
test_adc.CFLAGS += -DUSE_ADC1_2_IRQ_HANDLER



#
# test IMU b2 v1.1
#
IMU_B2_CFLAGS  = -DIMU_TYPE_H=\"subsystems/imu/imu_b2.h\"
IMU_B2_CFLAGS += -DIMU_B2_MAG_TYPE=IMU_B2_MAG_MS2100 -DIMU_B2_VERSION_1_1
IMU_B2_SRCS    = $(SRC_SUBSYSTEMS)/imu.c
IMU_B2_SRCS   += math/pprz_trig_int.c
IMU_B2_CFLAGS += -DUSE_SPI -DSPI_MASTER -DUSE_SPI1 -DIMU_B2_MAG_TYPE=IMU_B2_MAG_MS2100 -DIMU_B2_VERSION_1_1 -DUSE_SPI_SLAVE1
IMU_B2_SRCS   += $(SRC_SUBSYSTEMS)/imu/imu_b2.c
IMU_B2_SRCS   += peripherals/max1168.c $(SRC_ARCH)/peripherals/max1168_arch.c
IMU_B2_SRCS   += peripherals/ms2100.c  $(SRC_ARCH)/peripherals/ms2100_arch.c
IMU_B2_SRCS   += mcu_periph/spi.c $(SRC_ARCH)/mcu_periph/spi_arch.c

test_imu_b2.ARCHDIR = $(ARCH)
test_imu_b2.srcs    = test/subsystems/test_imu.c
test_imu_b2.CFLAGS  = $(COMMON_TEST_CFLAGS)
test_imu_b2.srcs   += $(COMMON_TEST_SRCS)
test_imu_b2.CFLAGS += $(COMMON_TELEMETRY_CFLAGS)
test_imu_b2.srcs   += $(COMMON_TELEMETRY_SRCS)
test_imu_b2.CFLAGS += $(IMU_B2_CFLAGS)
test_imu_b2.srcs   += $(IMU_B2_SRCS)




#
# test IMU b2 v1.2
#
IMU_B2_2_CFLAGS  = -DIMU_TYPE_H=\"subsystems/imu/imu_b2.h\"
IMU_B2_2_CFLAGS += -DIMU_B2_MAG_TYPE=IMU_B2_MAG_HMC5843 -DIMU_B2_VERSION_1_2
IMU_B2_2_SRCS    = $(SRC_SUBSYSTEMS)/imu.c
IMU_B2_2_SRCS   += math/pprz_trig_int.c
IMU_B2_2_CFLAGS += -DUSE_SPI -DSPI_MASTER -DUSE_SPI2
IMU_B2_2_SRCS   += $(SRC_SUBSYSTEMS)/imu/imu_b2.c $(SRC_ARCH)/subsystems/imu/imu_b2_arch.c
IMU_B2_2_SRCS   += peripherals/max1168.c $(SRC_ARCH)/peripherals/max1168_arch.c
IMU_B2_2_CFLAGS += -DUSE_I2C2
IMU_B2_2_SRCS   += mcu_periph/i2c.c $(SRC_ARCH)/mcu_periph/i2c_arch.c
IMU_B2_2_SRCS   += peripherals/hmc5843.c $(SRC_ARCH)/peripherals/hmc5843_arch.c
IMU_B2_2_SRCS   += mcu_periph/spi.c $(SRC_ARCH)/mcu_periph/spi_arch.c

test_imu_b2_2.ARCHDIR = $(ARCH)
test_imu_b2_2.srcs    = test/subsystems/test_imu.c
test_imu_b2_2.CFLAGS  = $(COMMON_TEST_CFLAGS)
test_imu_b2_2.srcs   += $(COMMON_TEST_SRCS)
test_imu_b2_2.CFLAGS += $(COMMON_TELEMETRY_CFLAGS)
test_imu_b2_2.srcs   += $(COMMON_TELEMETRY_SRCS)
test_imu_b2_2.CFLAGS += $(IMU_B2_2_CFLAGS)
test_imu_b2_2.srcs   += $(IMU_B2_2_SRCS)





#
# test IMU aspirin
#
IMU_ASPIRIN_CFLAGS = -DIMU_TYPE_H=\"imu/imu_aspirin.h\" -DIMU_ASPIRIN_VERSION_1_0
IMU_ASPIRIN_SRCS   = $(SRC_SUBSYSTEMS)/imu.c             \
                     $(SRC_SUBSYSTEMS)/imu/imu_aspirin.c \
                     $(SRC_ARCH)/subsystems/imu/imu_aspirin_arch.c
IMU_ASPIRIN_SRCS   += math/pprz_trig_int.c
IMU_ASPIRIN_SRCS   += peripherals/hmc5843.c $(SRC_ARCH)/peripherals/hmc5843_arch.c
IMU_ASPIRIN_CFLAGS += -DUSE_I2C2
IMU_ASPIRIN_SRCS   += mcu_periph/i2c.c $(SRC_ARCH)/mcu_periph/i2c_arch.c

test_imu_aspirin.ARCHDIR = $(ARCH)
test_imu_aspirin.srcs    = test/subsystems/test_imu.c
test_imu_aspirin.CFLAGS  = $(COMMON_TEST_CFLAGS)
test_imu_aspirin.srcs   += $(COMMON_TEST_SRCS)
test_imu_aspirin.CFLAGS += $(COMMON_TELEMETRY_CFLAGS)
test_imu_aspirin.srcs   += $(COMMON_TELEMETRY_SRCS)
test_imu_aspirin.CFLAGS += $(IMU_ASPIRIN_CFLAGS)
test_imu_aspirin.srcs   += $(IMU_ASPIRIN_SRCS)


#
# test AHRS
#
test_ahrs.ARCHDIR = $(ARCH)
test_ahrs.srcs    = test/subsystems/test_ahrs.c
test_ahrs.CFLAGS  = $(COMMON_TEST_CFLAGS)
test_ahrs.srcs   += $(COMMON_TEST_SRCS)
test_ahrs.CFLAGS += $(COMMON_TELEMETRY_CFLAGS)
test_ahrs.srcs   += $(COMMON_TELEMETRY_SRCS)
test_ahrs.CFLAGS += $(IMU_ASPIRIN_CFLAGS)
test_ahrs.srcs   += $(IMU_ASPIRIN_SRCS)

#AHRS = ice
AHRS = icq
#AHRS = flq
#AHRS = fcr
#AHRS = fcr2
#AHRS = fcq

test_ahrs.srcs += $(SRC_SUBSYSTEMS)/ahrs.c
test_ahrs.srcs += $(SRC_SUBSYSTEMS)/ahrs/ahrs_aligner.c

ifeq ($(AHRS), ice)
test_ahrs.CFLAGS += -DFACE_REINJ_1=1024
test_ahrs.CFLAGS += -DAHRS_TYPE_H=\"subsystems/ahrs/ahrs_int_cmpl_euler.h\"
test_ahrs.srcs += subsystems/ahrs/ahrs_int_cmpl_euler.c  \
                  lisa/plug_sys.c
endif

ifeq ($(AHRS), icq)
#test_ahrs.CFLAGS += -DAHRS_TYPE=\"ICQ\"
test_ahrs.CFLAGS += -DAHRS_PROPAGATE_FREQUENCY=512
test_ahrs.CFLAGS += -DAHRS_TYPE_H=\"subsystems/ahrs/ahrs_int_cmpl_quat.h\"
test_ahrs.srcs +=subsystems/ahrs/ahrs_int_cmpl_quat.c
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
test_ahrs.CFLAGS += -DAHRS_TYPE_H=\"subsystems/ahrs/ahrs_float_cmpl.h\"
test_ahrs.CFLAGS += -DAHRS_H_X=0.51562740288882 -DAHRS_H_Y=-0.05707735220832 -DAHRS_H_Z=0.85490967783446
test_ahrs.CFLAGS += -DAHRS_PROPAGATE_FREQUENCY=512
test_ahrs.srcs += subsystems/ahrs/ahrs_float_cmpl.c
endif

ifeq ($(AHRS), fcq)
test_ahrs.CFLAGS += -DAHRS_TYPE_H=\"subsystems/ahrs/ahrs_float_cmpl.h\"
test_ahrs.CFLAGS += -DAHRS_H_X=0.51562740288882 -DAHRS_H_Y=-0.05707735220832 -DAHRS_H_Z=0.85490967783446
test_ahrs.CFLAGS += -DAHRS_PROPAGATE_FREQUENCY=512
test_ahrs.srcs += subsystems/ahrs/ahrs_float_cmpl_quat.c
endif








#
# test hmc5843
#
test_hmc5843.ARCHDIR = $(ARCH)
test_hmc5843.CFLAGS  = $(COMMON_TEST_CFLAGS)
test_hmc5843.srcs    = $(COMMON_TEST_SRCS)
test_hmc5843.CFLAGS += $(COMMON_TELEMETRY_CFLAGS)
test_hmc5843.srcs   += $(COMMON_TELEMETRY_SRCS)

test_hmc5843.CFLAGS += -I$(SRC_LISA)
test_hmc5843.srcs   += lisa/test/lisa_test_hmc5843.c
test_hmc5843.CFLAGS += -DUSE_I2C2
test_hmc5843.srcs   += mcu_periph/i2c.c $(SRC_ARCH)/mcu_periph/i2c_arch.c


#
# test ITG3200
#
test_itg3200.ARCHDIR = $(ARCH)
test_itg3200.CFLAGS  = $(COMMON_TEST_CFLAGS)
test_itg3200.srcs    = $(COMMON_TEST_SRCS)
test_itg3200.CFLAGS += $(COMMON_TELEMETRY_CFLAGS)
test_itg3200.srcs   += $(COMMON_TELEMETRY_SRCS)

test_itg3200.CFLAGS += -I$(SRC_LISA)
test_itg3200.srcs   += lisa/test/lisa_test_itg3200.c
test_itg3200.CFLAGS += -DUSE_I2C2
test_itg3200.srcs   += mcu_periph/i2c.c $(SRC_ARCH)/mcu_periph/i2c_arch.c


#
# test adxl345 with DMA
#
test_adxl345.ARCHDIR = $(ARCH)
test_adxl345.CFLAGS  = $(COMMON_TEST_CFLAGS)
test_adxl345.srcs    = $(COMMON_TEST_SRCS)
test_adxl345.CFLAGS += $(COMMON_TELEMETRY_CFLAGS)
test_adxl345.srcs   += $(COMMON_TELEMETRY_SRCS)

test_adxl345.CFLAGS += -I$(SRC_LISA)
test_adxl345.srcs   += lisa/test/lisa_test_adxl345_dma.c



#
# simple test of mikrokopter motor controllers
#
test_esc_mkk_simple.ARCHDIR = $(ARCH)
test_esc_mkk_simple.CFLAGS  = $(COMMON_TEST_CFLAGS)
test_esc_mkk_simple.srcs    = $(COMMON_TEST_SRCS)
test_esc_mkk_simple.CFLAGS += $(COMMON_TELEMETRY_CFLAGS)
test_esc_mkk_simple.srcs   += $(COMMON_TELEMETRY_SRCS)

test_esc_mkk_simple.srcs   += test/test_esc_mkk_simple.c
test_esc_mkk_simple.CFLAGS += -DUSE_I2C2
test_esc_mkk_simple.srcs   += mcu_periph/i2c.c $(SRC_ARCH)/mcu_periph/i2c_arch.c
test_esc_mkk_simple.CFLAGS += -DACTUATORS_MKK_DEV=i2c2


#
# simple test of asctec v1 motor controllers
#
test_esc_asctecv1_simple.ARCHDIR = $(ARCH)
test_esc_asctecv1_simple.CFLAGS  = $(COMMON_TEST_CFLAGS)
test_esc_asctecv1_simple.srcs    = $(COMMON_TEST_SRCS)
test_esc_asctecv1_simple.CFLAGS += $(COMMON_TELEMETRY_CFLAGS)
test_esc_asctecv1_simple.srcs   += $(COMMON_TELEMETRY_SRCS)

test_esc_asctecv1_simple.srcs   += test/test_esc_asctecv1_simple.c
test_esc_asctecv1_simple.CFLAGS += -DUSE_I2C1
test_esc_asctecv1_simple.srcs   += mcu_periph/i2c.c $(SRC_ARCH)/mcu_periph/i2c_arch.c


#
# test actuators mkk
#
test_actuators_mkk.ARCHDIR = $(ARCH)
test_actuators_mkk.CFLAGS  = $(COMMON_TEST_CFLAGS)
test_actuators_mkk.srcs    = $(COMMON_TEST_SRCS)
test_actuators_mkk.CFLAGS += $(COMMON_TELEMETRY_CFLAGS)
test_actuators_mkk.srcs   += $(COMMON_TELEMETRY_SRCS)

test_actuators_mkk.srcs   += test/test_actuators.c
test_actuators_mkk.srcs   += subsystems/commands.c
test_actuators_mkk.srcs   += $(SRC_FIRMWARE)/actuators/actuators_mkk.c
test_actuators_mkk.CFLAGS += -DACTUATORS_MKK_DEVICE=i2c1
test_actuators_mkk.srcs   += $(SRC_FIRMWARE)/actuators/supervision.c
test_actuators_mkk.CFLAGS += -DUSE_I2C1
test_actuators_mkk.srcs   += mcu_periph/i2c.c $(SRC_ARCH)/mcu_periph/i2c_arch.c


#
# test actuators asctecv1
#
test_actuators_asctecv1.ARCHDIR = $(ARCH)
test_actuators_asctecv1.CFLAGS  = $(COMMON_TEST_CFLAGS)
test_actuators_asctecv1.srcs    = $(COMMON_TEST_SRCS)
test_actuators_asctecv1.CFLAGS += $(COMMON_TELEMETRY_CFLAGS)
test_actuators_asctecv1.srcs   += $(COMMON_TELEMETRY_SRCS)

test_actuators_asctecv1.srcs   += test/test_actuators.c
test_actuators_asctecv1.srcs   += subsystems/commands.c
test_actuators_asctecv1.CFLAGS += -DACTUATORS_ASCTEC_DEVICE=i2c1
test_actuators_asctecv1.srcs   += $(SRC_FIRMWARE)/actuators/actuators_asctec.c
test_actuators_asctecv1.CFLAGS += -DUSE_I2C1
test_actuators_asctecv1.srcs   += mcu_periph/i2c.c $(SRC_ARCH)/mcu_periph/i2c_arch.c


##
## test bmp085
##
#test_bmp085.ARCHDIR = $(ARCH)
#test_bmp085.CFLAGS  = $(COMMON_TEST_CFLAGS)
#test_bmp085.srcs    = $(COMMON_TEST_SRCS)
#test_bmp085.CFLAGS += $(COMMON_TELEMETRY_CFLAGS)
#test_bmp085.srcs   += $(COMMON_TELEMETRY_SRCS)
#
#test_bmp085.CFLAGS += -I$(SRC_LISA)
#test_bmp085.srcs   += lisa/test/lisa_test_bmp085.c
#test_bmp085.CFLAGS += -DUSE_I2C2
#test_bmp085.srcs   += mcu_periph/i2c.c $(SRC_ARCH)/mcu_periph/i2c_arch.c



##
## Test manual : a simple test with rc and servos - I want to fly lisa/M
##
#test_manual.ARCHDIR = $(ARCH)
#test_manual.CFLAGS  = $(COMMON_TEST_CFLAGS)
#test_manual.srcs    = $(COMMON_TEST_SRCS)
#test_manual.CFLAGS += $(COMMON_TELEMETRY_CFLAGS)
#test_manual.srcs   += $(COMMON_TELEMETRY_SRCS)
#
#test_manual.srcs   += test/test_manual.c
#test_manual.srcs   += subsystems/commands.c
##test_manual.srcs   += subsystems/actuators/actuators_pwm.c
#test_manual.srcs   += $(SRC_ARCH)/subsystems/actuators/actuators_pwm_arch.c
#
#test_manual.CFLAGS += -DRADIO_CONTROL
#ifneq ($(RADIO_CONTROL_LED),none)
#test_manual.CFLAGS += -DRADIO_CONTROL_LED=$(RADIO_CONTROL_LED)
#endif
#test_manual.CFLAGS += -DRADIO_CONTROL_BIND_IMPL_FUNC=radio_control_spektrum_try_bind
#test_manual.CFLAGS += -DRADIO_CONTROL_TYPE_H=\"subsystems/radio_control/spektrum.h\"
#test_manual.CFLAGS += -DRADIO_CONTROL_SPEKTRUM_PRIMARY_PORT=$(RADIO_CONTROL_SPEKTRUM_PRIMARY_PORT)
#test_manual.CFLAGS += -DOVERRIDE_$(RADIO_CONTROL_SPEKTRUM_PRIMARY_PORT)_IRQ_HANDLER -DUSE_TIM6_IRQ
#test_manual.srcs   += $(SRC_SUBSYSTEMS)/radio_control.c
#test_manual.srcs   += $(SRC_SUBSYSTEMS)/radio_control/spektrum.c
#test_manual.srcs   += $(SRC_ARCH)/subsystems/radio_control/spektrum_arch.c

##
## test can interface
##
test_can.ARCHDIR = $(ARCH)
test_can.CFLAGS  = $(COMMON_TEST_CFLAGS)
test_can.srcs    = $(COMMON_TEST_SRCS)
test_can.CFLAGS += $(COMMON_TELEMETRY_CFLAGS)
test_can.srcs   += $(COMMON_TELEMETRY_SRCS)
test_can.CFLAGS += -I$(SRC_LISA) -I$(SRC_BOARD)

test_can.CFLAGS += -I$(SRC_LISA)
test_can.srcs   += lisa/test_can.c
test_can.srcs   += mcu_periph/can.c $(SRC_ARCH)/mcu_periph/can_arch.c
