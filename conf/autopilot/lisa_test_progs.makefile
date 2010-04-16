# Hey Emacs, this is a -*- makefile -*-
#
# $Id$
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
#  Those babies run on the overo
#
#
################################################################################


overo_test_spi.ARCHDIR = omap
overo_test_spi.srcs=$(SRC_FMS)/overo_test_spi.c


overo_test_link_stm.ARCHDIR = omap
overo_test_link_stm.srcs  = $(SRC_FMS)/overo_test_link.c
overo_test_link_stm.srcs += $(SRC_FMS)/fms_spi_link.c


overo_test_telemetry.ARCHDIR  = omap
overo_test_telemetry.CFLAGS  += -I$(ACINCLUDE) -I. -I$(PAPARAZZI_HOME)/var/include
overo_test_telemetry.srcs     = $(SRC_FMS)/overo_test_telemetry.c
overo_test_telemetry.CFLAGS  += -DDOWNLINK -DDOWNLINK_TRANSPORT=UdpTransport
overo_test_telemetry.srcs    += $(SRC_FMS)/udp_transport.c downlink.c
overo_test_telemetry.srcs    += $(SRC_FMS)/fms_network.c
overo_test_telemetry.LDFLAGS += -levent





################################################################################
#
#
#  Those babies run on the stm32
#
#
################################################################################

ARCHI=stm32
SRC_LISA=lisa
SRC_ARCH=$(ARCHI)

#BOARD_CFG=\"boards/olimex_stm32-h103.h\"
BOARD_CFG=\"boards/lisa_0.99.h\"
#FLASH_MODE = SERIAL
FLASH_MODE = JTAG

#
# test leds
#
test_led.ARCHDIR = $(ARCHI)
test_led.TARGET = test_led
test_led.TARGETDIR = test_led
test_led.CFLAGS += -I$(SRC_LISA) -I$(ARCHI) -DPERIPHERALS_AUTO_INIT
#test_led.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_led.srcs += $(SRC_LISA)/test_led.c       	\
                 $(SRC_ARCH)/led_hw.c     	\
                 $(SRC_LISA)/exceptions.c     	\
                 $(SRC_LISA)/vector_table.c
test_led.CFLAGS += -DUSE_LED


#
# test leds2
#
test_led2.ARCHDIR = $(ARCHI)
test_led2.TARGET = test_led2
test_led2.TARGETDIR = test_led2
test_led2.CFLAGS += -I$(SRC_LISA) -I$(ARCHI) -DPERIPHERALS_AUTO_INIT
test_led2.CFLAGS += -DBOARD_CONFIG=\"boards/lisa_0.99.h\"
test_led2.srcs += $(SRC_LISA)/test_led2.c              \
                 $(SRC_LISA)/exceptions.c              \
                 $(SRC_LISA)/vector_table.c
test_led2.CFLAGS += -DUSE_LED


#
# test periodic
#
test_periodic.ARCHDIR = $(ARCHI)
test_periodic.TARGET = test_periodic
test_periodic.TARGETDIR = test_periodic
test_periodic.CFLAGS += -I$(SRC_LISA) -I$(ARCHI) -DPERIPHERALS_AUTO_INIT
test_periodic.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_periodic.srcs += $(SRC_LISA)/test_periodic.c  \
                      $(SRC_LISA)/exceptions.c     \
                      $(SRC_LISA)/vector_table.c
test_periodic.CFLAGS += -DUSE_LED
test_periodic.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
test_periodic.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
test_periodic.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c


#
# test uart
#
test_uart.ARCHDIR = $(ARCHI)
test_uart.TARGET = test_uart
test_uart.TARGETDIR = test_uart
test_uart.CFLAGS = -I$(SRC_LISA) -I$(ARCHI) -DPERIPHERALS_AUTO_INIT
test_uart.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_uart.srcs = $(SRC_LISA)/test_uart.c         \
                      $(SRC_LISA)/exceptions.c   \
                      $(SRC_LISA)/vector_table.c
test_uart.CFLAGS += -DUSE_LED
test_uart.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
test_uart.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
test_uart.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c
test_uart.CFLAGS += -DUSE_UART2 -DUART2_BAUD=B57600
test_uart.srcs += $(SRC_ARCH)/uart_hw.c



#
# test telemetry1
#
test_telemetry1.ARCHDIR = $(ARCHI)
test_telemetry1.TARGET = test_telemetry1
test_telemetry1.TARGETDIR = test_telemetry1
test_telemetry1.CFLAGS = -I$(SRC_LISA) -I$(ARCHI) -DPERIPHERALS_AUTO_INIT
test_telemetry1.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_telemetry1.srcs = $(SRC_LISA)/test_telemetry.c \
                      $(SRC_LISA)/exceptions.c      \
                      $(SRC_LISA)/vector_table.c
test_telemetry1.CFLAGS += -DUSE_LED
test_telemetry1.srcs += $(SRC_ARCH)/led_hw.c
test_telemetry1.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
test_telemetry1.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
test_telemetry1.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c
test_telemetry1.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600
test_telemetry1.srcs += $(SRC_ARCH)/uart_hw.c
test_telemetry1.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart1 
test_telemetry1.srcs += downlink.c pprz_transport.c


#
# test telemetry2
#
test_telemetry2.ARCHDIR = $(ARCHI)
test_telemetry2.TARGET = test_telemetry2
test_telemetry2.TARGETDIR = test_telemetry2
test_telemetry2.CFLAGS = -I$(SRC_LISA) -I$(ARCHI) -DPERIPHERALS_AUTO_INIT
test_telemetry2.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_telemetry2.srcs = $(SRC_LISA)/test_telemetry.c \
                       $(SRC_LISA)/exceptions.c     \
                       $(SRC_LISA)/vector_table.c
test_telemetry2.CFLAGS += -DUSE_LED
test_telemetry2.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
test_telemetry2.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
test_telemetry2.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c
test_telemetry2.CFLAGS += -DUSE_UART2 -DUART2_BAUD=B57600
test_telemetry2.srcs += $(SRC_ARCH)/uart_hw.c
test_telemetry2.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart2 
test_telemetry2.srcs += downlink.c pprz_transport.c


#
# test telemetry3
#
test_telemetry3.ARCHDIR = $(ARCHI)
test_telemetry3.TARGET = test_telemetry3
test_telemetry3.TARGETDIR = test_telemetry3
test_telemetry3.CFLAGS = -I$(SRC_LISA) -I$(ARCHI) -DPERIPHERALS_AUTO_INIT
test_telemetry3.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_telemetry3.srcs = $(SRC_LISA)/test_telemetry.c \
                       $(SRC_LISA)/exceptions.c     \
                       $(SRC_LISA)/vector_table.c
test_telemetry3.CFLAGS += -DUSE_LED
test_telemetry3.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
test_telemetry3.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
test_telemetry3.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c
test_telemetry3.CFLAGS += -DUSE_UART3 -DUART3_BAUD=B57600
test_telemetry3.srcs += $(SRC_ARCH)/uart_hw.c
test_telemetry3.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart3 
test_telemetry3.srcs += downlink.c pprz_transport.c

#
# test datalink
#
test_datalink.ARCHDIR = $(ARCHI)
test_datalink.TARGET = test_datalink
test_datalink.TARGETDIR = test_datalink
test_datalink.CFLAGS = -I$(SRC_LISA) -I$(ARCHI) -DPERIPHERALS_AUTO_INIT
test_datalink.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_datalink.srcs = $(SRC_LISA)/test_datalink.c \
                     $(SRC_LISA)/exceptions.c    \
                     $(SRC_LISA)/vector_table.c
test_datalink.CFLAGS += -DUSE_LED
test_datalink.CFLAGS += -DUSE_SYS_TIME # -DSYS_TIME_LED=1
test_datalink.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
test_datalink.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c
test_datalink.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600
test_datalink.srcs += $(SRC_ARCH)/uart_hw.c
test_datalink.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart1 
test_datalink.srcs += downlink.c pprz_transport.c

test_datalink.CFLAGS += -DDATALINK=PPRZ -DPPRZ_UART=Uart1


#
# test rc 2.4
#

SRC_BOOZ = booz
SRC_BOOZ_TEST = $(SRC_BOOZ)/test

test_rc_24.ARCHDIR = $(ARCHI)
test_rc_24.TARGET = test_rc_24
test_rc_24.TARGETDIR = test_rc_24
test_rc_24.CFLAGS += -I$(SRC_LISA) -I$(ARCHI) -I$(SRC_BOOZ) -DPERIPHERALS_AUTO_INIT
test_rc_24.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG) 
test_rc_24.srcs += $(SRC_BOOZ_TEST)/booz2_test_radio_control.c \
                   $(SRC_LISA)/exceptions.c                     \
                   $(SRC_LISA)/vector_table.c
test_rc_24.CFLAGS += -DUSE_LED
test_rc_24.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
test_rc_24.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))'
test_rc_24.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c

test_rc_24.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600
test_rc_24.srcs += $(SRC_ARCH)/uart_hw.c
test_rc_24.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart1 
test_rc_24.srcs += downlink.c pprz_transport.c

test_rc_24.CFLAGS += -DUSE_RADIO_CONTROL -DRADIO_CONTROL_LED=1
test_rc_24.CFLAGS += -DRADIO_CONTROL_TYPE_H=\"booz_radio_control_spektrum.h\"
test_rc_24.CFLAGS += -DRADIO_CONTROL_SPEKTRUM_MODEL_H=\"booz_radio_control_spektrum_dx7se.h\"
test_rc_24.CFLAGS += -DUSE_UART2 -DUART2_BAUD=B115200
test_rc_24.CFLAGS += -DRADIO_CONTROL_LINK=Uart2
test_rc_24.srcs += $(SRC_BOOZ)/booz_radio_control.c \
                   $(SRC_BOOZ)/booz_radio_control_spektrum.c \
#                   $(SRC_ARCH)/uart_hw.c


#
# test servos
#

SRC_BOOZ_ARCH=$(SRC_BOOZ)/arch/$(ARCHI)

test_servos.ARCHDIR = $(ARCHI)
test_servos.TARGET = test_servos
test_servos.TARGETDIR = test_servos
test_servos.CFLAGS  = -I$(SRC_LISA) -I$(ARCHI) -I$(SRC_BOOZ) -I$(SRC_BOOZ_ARCH) -DPERIPHERALS_AUTO_INIT
test_servos.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_servos.LDFLAGS += -lm
test_servos.srcs += $(SRC_LISA)/test_servos.c 	\
                    $(SRC_LISA)/exceptions.c    \
                    $(SRC_LISA)/vector_table.c
test_servos.CFLAGS += -DUSE_LED
test_servos.srcs += $(SRC_ARCH)/led_hw.c
test_servos.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
test_servos.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))'
test_servos.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c

test_servos.srcs += $(SRC_BOOZ)/actuators/booz_actuators_pwm.c $(SRC_BOOZ_ARCH)/actuators/booz_actuators_pwm_hw.c


#test_servos.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600
#test_servos.srcs += $(SRC_ARCH)/uart_hw.c
#test_servos.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart1 
#test_servos.srcs += downlink.c pprz_transport.c





#
# test Max1168
#
test_max1168.ARCHDIR = $(ARCHI)
test_max1168.TARGET = test_max1168
test_max1168.TARGETDIR = test_max1168
test_max1168.CFLAGS = -I$(SRC_LISA) -I$(ARCHI) -I$(SRC_BOOZ) -I$(SRC_BOOZ_ARCH) -DPERIPHERALS_AUTO_INIT
test_max1168.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_max1168.srcs = $(SRC_LISA)/test/lisa_test_max1168.c \
                    $(SRC_LISA)/exceptions.c             \
                    $(SRC_LISA)/vector_table.c

test_max1168.CFLAGS += -DUSE_LED
test_max1168.srcs += $(SRC_ARCH)/led_hw.c

test_max1168.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
test_max1168.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
test_max1168.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c

test_max1168.CFLAGS += -DUSE_SPI2 -DUSE_EXTI2_IRQ -DUSE_DMA1_C4_IRQ -DMAX1168_HANDLES_DMA_IRQ
test_max1168.srcs   += $(SRC_BOOZ)/peripherals/booz_max1168.c \
                       $(SRC_BOOZ_ARCH)/peripherals/booz_max1168_arch.c

test_max1168.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600
test_max1168.srcs += $(SRC_ARCH)/uart_hw.c

test_max1168.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart1 
test_max1168.srcs += downlink.c pprz_transport.c

#
# test ms2001
#
test_ms2001.ARCHDIR = $(ARCHI)
test_ms2001.TARGET = test_ms2001
test_ms2001.TARGETDIR = test_ms2001
test_ms2001.CFLAGS = -I$(SRC_LISA) -I$(ARCHI) -I$(SRC_BOOZ) -I$(SRC_BOOZ_ARCH) -DPERIPHERALS_AUTO_INIT
test_ms2001.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_ms2001.srcs = $(SRC_LISA)/test/lisa_test_ms2001.c \
                   $(SRC_LISA)/exceptions.c            \
                   $(SRC_LISA)/vector_table.c

test_ms2001.CFLAGS += -DUSE_LED
test_ms2001.srcs += $(SRC_ARCH)/led_hw.c

test_ms2001.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
test_ms2001.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
test_ms2001.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c

test_ms2001.CFLAGS += -DUSE_SPI2
test_ms2001.CFLAGS += -DUSE_DMA1_C4_IRQ -DMS2001_HANDLES_DMA_IRQ
test_ms2001.CFLAGS += -DUSE_SPI2_IRQ -DMS2001_HANDLES_SPI_IRQ
test_ms2001.srcs   += $(SRC_BOOZ)/peripherals/booz_ms2001.c \
                      $(SRC_BOOZ_ARCH)/peripherals/booz_ms2001_arch.c

test_ms2001.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600
test_ms2001.srcs += $(SRC_ARCH)/uart_hw.c

test_ms2001.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart1 
test_ms2001.srcs += downlink.c pprz_transport.c

#
# test IMU b2
#
test_imu_b2.ARCHDIR = $(ARCHI)
test_imu_b2.TARGET = test_imu_b2
test_imu_b2.TARGETDIR = test_imu_b2
test_imu_b2.CFLAGS  =  -I$(SRC_LISA) -I$(ARCHI) -I$(SRC_BOOZ) -I$(SRC_BOOZ_ARCH) -DPERIPHERALS_AUTO_INIT
test_imu_b2.CFLAGS +=  -DBOARD_CONFIG=$(BOARD_CFG)
test_imu_b2.srcs += $(SRC_BOOZ_TEST)/booz2_test_imu_b2.c \
                    $(SRC_LISA)/exceptions.c             \
                    $(SRC_LISA)/vector_table.c

test_imu_b2.CFLAGS += -DUSE_LED
test_imu_b2.srcs += $(SRC_ARCH)/led_hw.c

test_imu_b2.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
test_imu_b2.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))'
test_imu_b2.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c

test_imu_b2.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600
test_imu_b2.srcs += $(SRC_ARCH)/uart_hw.c

test_imu_b2.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart1 
test_imu_b2.srcs += downlink.c pprz_transport.c

test_imu_b2.srcs += math/pprz_trig_int.c

test_imu_b2.CFLAGS += -DBOOZ_IMU_TYPE_H=\"imu/booz_imu_b2.h\"
test_imu_b2.CFLAGS += -DIMU_B2_MAG_TYPE=IMU_B2_MAG_MS2001
test_imu_b2.srcs += $(SRC_BOOZ)/booz_imu.c
test_imu_b2.CFLAGS += -DUSE_SPI2 -DUSE_DMA1_C4_IRQ -DUSE_EXTI2_IRQ -DUSE_SPI2_IRQ
test_imu_b2.srcs += $(SRC_BOOZ)/imu/booz_imu_b2.c $(SRC_BOOZ_ARCH)/imu/booz_imu_b2_arch.c
test_imu_b2.srcs += $(SRC_BOOZ)/peripherals/booz_max1168.c $(SRC_BOOZ_ARCH)/peripherals/booz_max1168_arch.c
test_imu_b2.srcs += $(SRC_BOOZ)/peripherals/booz_ms2001.c  $(SRC_BOOZ_ARCH)/peripherals/booz_ms2001_arch.c






#
# test motor controllers
#
test_mc.ARCHDIR = $(ARCHI)
test_mc.TARGET = test_mc
test_mc.TARGETDIR = test_mc
test_mc.CFLAGS = -I$(SRC_LISA) -I$(ARCHI) -DPERIPHERALS_AUTO_INIT
test_mc.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_mc.srcs = $(SRC_LISA)/test_mc.c      \
               $(SRC_LISA)/exceptions.c   \
               $(SRC_LISA)/vector_table.c
test_mc.CFLAGS += -DUSE_LED
test_mc.srcs += $(SRC_ARCH)/led_hw.c
test_mc.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
test_mc.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
test_mc.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c


#
# test motor controllers with interrupts
#
test_mc2.ARCHDIR = $(ARCHI)
test_mc2.TARGET = test_mc
test_mc2.TARGETDIR = test_mc
test_mc2.CFLAGS = -I$(SRC_LISA) -I$(ARCHI) -DPERIPHERALS_AUTO_INIT
test_mc2.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_mc2.srcs = $(SRC_LISA)/test_mc2.c     \
                $(SRC_LISA)/exceptions.c   \
                $(SRC_LISA)/vector_table.c
test_mc2.CFLAGS += -DUSE_LED
test_mc2.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
test_mc2.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
test_mc2.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c
#test_mc2.CFLAGS += -DACTUATORS=\"actuators_buss_twi_blmc_hw.h\" -DUSE_BUSS_TWI_BLMC
#test_mc2.srcs += $(SRC_BOOZ_ARCH)/actuators_buss_twi_blmc_hw.c actuators.c
test_mc2.CFLAGS += -DUSE_I2C1
test_mc2.srcs += i2c.c $(SRC_ARCH)/i2c_hw.c


#
# test motor controllers asctec
#
test_mc3.ARCHDIR = $(ARCHI)
test_mc3.TARGET = test_mc3
test_mc3.TARGETDIR = test_mc3
test_mc3.CFLAGS = -I$(SRC_LISA) -I$(ARCHI) -DPERIPHERALS_AUTO_INIT
test_mc3.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_mc3.srcs = $(SRC_LISA)/test_mc3.c      \
               $(SRC_LISA)/exceptions.c   \
               $(SRC_LISA)/vector_table.c
test_mc3.CFLAGS += -DUSE_LED
test_mc3.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
test_mc3.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
test_mc3.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c




#
# test baro
#
test_baro.ARCHDIR = $(ARCHI)
test_baro.TARGET = test_baro
test_baro.TARGETDIR = test_baro
test_baro.CFLAGS = -I$(SRC_LISA) -I$(ARCHI) -DPERIPHERALS_AUTO_INIT
test_baro.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_baro.srcs = $(SRC_LISA)/test_baro.c      \
               $(SRC_LISA)/exceptions.c   \
               $(SRC_LISA)/vector_table.c
test_baro.CFLAGS += -DUSE_LED
test_baro.srcs += $(SRC_ARCH)/led_hw.c
test_baro.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
test_baro.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
test_baro.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c

test_baro.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600
test_baro.srcs += $(SRC_ARCH)/uart_hw.c

test_baro.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart1 
test_baro.srcs += downlink.c pprz_transport.c







#
# test spi slave
#
test_spi_slave.ARCHDIR = $(ARCHI)
test_spi_slave.TARGET = test_spi_slave
test_spi_slave.TARGETDIR = test_spi_slave
test_spi_slave.CFLAGS = -I$(SRC_LISA) -I$(ARCHI) -DPERIPHERALS_AUTO_INIT
test_spi_slave.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_spi_slave.srcs = $(SRC_LISA)/test_spi_slave.c      \
                      $(SRC_LISA)/exceptions.c          \
                      $(SRC_LISA)/vector_table.c
test_spi_slave.CFLAGS += -DUSE_LED
test_spi_slave.srcs += $(SRC_ARCH)/led_hw.c
test_spi_slave.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
test_spi_slave.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
test_spi_slave.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c
test_spi_slave.CFLAGS += -DUSE_SPI1_IRQ
test_spi_slave.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600
test_spi_slave.srcs += $(SRC_ARCH)/uart_hw.c
test_spi_slave.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart1 
test_spi_slave.srcs += downlink.c pprz_transport.c


#
# test spi slave2
#
test_spi_slave2.ARCHDIR = $(ARCHI)
test_spi_slave2.TARGET = test_spi_slave2
test_spi_slave2.TARGETDIR = test_spi_slave2
test_spi_slave2.CFLAGS = -I$(SRC_LISA) -I$(ARCHI) -DPERIPHERALS_AUTO_INIT
test_spi_slave2.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_spi_slave2.srcs = $(SRC_LISA)/test_spi_slave2.c      \
                      $(SRC_LISA)/exceptions.c          \
                      $(SRC_LISA)/vector_table.c
test_spi_slave2.CFLAGS += -DUSE_LED
test_spi_slave2.srcs += $(SRC_ARCH)/led_hw.c
test_spi_slave2.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
test_spi_slave2.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
test_spi_slave2.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c
test_spi_slave2.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600
test_spi_slave2.srcs += $(SRC_ARCH)/uart_hw.c
test_spi_slave2.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart1 
test_spi_slave2.srcs += downlink.c pprz_transport.c



#
# test overo com
#
test_ovc.ARCHDIR = $(ARCHI)
test_ovc.TARGET = test_ovc
test_ovc.TARGETDIR = test_ovc
test_ovc.CFLAGS += -I$(SRC_LISA) -I$(ARCHI) -DPERIPHERALS_AUTO_INIT
test_ovc.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_ovc.srcs += $(SRC_LISA)/test_ovc.c       \
                 $(SRC_LISA)/exceptions.c     \
                 $(SRC_LISA)/vector_table.c
test_ovc.CFLAGS += -DUSE_LED
test_ovc.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
test_ovc.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
test_ovc.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c

test_ovc.CFLAGS += -DUSE_UART2 -DUART2_BAUD=B57600
test_ovc.srcs += $(SRC_ARCH)/uart_hw.c


