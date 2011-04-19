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

SRC_CSC=csc

# test spi link between overo and stm32
overo_test_spi_link.ARCHDIR = omap
overo_test_spi_link.CFLAGS  += -I$(ACINCLUDE) -I. -I$(PAPARAZZI_HOME)/var/include
overo_test_spi_link.CFLAGS  += -DOVERO_LINK_MSG_UP=AutopilotMessageFoo -DOVERO_LINK_MSG_DOWN=AutopilotMessageFoo
overo_test_spi_link.srcs  = $(SRC_FMS)/overo_test_spi_link.c
overo_test_spi_link.srcs += $(SRC_FMS)/fms_spi_link.c



# test network based telemetry on overo
overo_test_telemetry.ARCHDIR  = omap
overo_test_telemetry.CFLAGS  += -I$(ACINCLUDE) -I. -I$(PAPARAZZI_HOME)/var/include
overo_test_telemetry.srcs     = $(SRC_FMS)/overo_test_telemetry.c
overo_test_telemetry.CFLAGS  += -DDOWNLINK -DDOWNLINK_TRANSPORT=UdpTransport
overo_test_telemetry.srcs    += $(SRC_FMS)/udp_transport.c downlink.c
overo_test_telemetry.srcs    += $(SRC_FMS)/fms_network.c
overo_test_telemetry.LDFLAGS += -levent

# test network based telemetry on overo (using udp_transport2/messages2)
overo_test_telemetry2.ARCHDIR  = omap
overo_test_telemetry2.CFLAGS  += -I$(ACINCLUDE) -I. -I$(PAPARAZZI_HOME)/var/include
overo_test_telemetry2.srcs     = $(SRC_FMS)/overo_test_telemetry2.c
overo_test_telemetry2.CFLAGS  += -DDOWNLINK -DDOWNLINK_TRANSPORT=UdpTransport
overo_test_telemetry2.srcs    += $(SRC_FMS)/udp_transport2.c downlink.c
overo_test_telemetry2.srcs    += $(SRC_FMS)/fms_network.c
overo_test_telemetry2.LDFLAGS += -levent

# test gps on overo
overo_test_gps.ARCHDIR  = omap
overo_test_gps.CFLAGS  += -I$(ACINCLUDE) -I. -I$(PAPARAZZI_HOME)/var/include
overo_test_gps.srcs     = $(SRC_FMS)/overo_test_gps.c
overo_test_gps.CFLAGS  += -DFMS_PERIODIC_FREQ=500
overo_test_gps.srcs    += $(SRC_FMS)/fms_periodic.c
overo_test_gps.CFLAGS  += -DDOWNLINK -DDOWNLINK_TRANSPORT=UdpTransport
overo_test_gps.srcs    += $(SRC_FMS)/udp_transport2.c downlink.c
overo_test_gps.srcs    += $(SRC_FMS)/fms_network.c
overo_test_gps.LDFLAGS += -levent


# test periodic tasks on the overo
overo_test_periodic.ARCHDIR  = omap
overo_test_periodic.CFLAGS  += -I$(ACINCLUDE) -I. -I$(PAPARAZZI_HOME)/var/include
overo_test_periodic.srcs     = $(SRC_FMS)/overo_test_periodic.c
overo_test_periodic.CFLAGS  += -DFMS_PERIODIC_FREQ=10
overo_test_periodic.srcs    += $(SRC_FMS)/fms_periodic.c
overo_test_periodic.srcs    += $(SRC_FMS)/fms_serial_port.c
overo_test_periodic.LDFLAGS += -lrt
overo_test_periodic.CFLAGS  += -DDOWNLINK -DDOWNLINK_TRANSPORT=UdpTransport
overo_test_periodic.srcs    += $(SRC_FMS)/udp_transport.c downlink.c
overo_test_periodic.srcs    += $(SRC_FMS)/fms_network.c
overo_test_periodic.LDFLAGS += -levent
overo_test_periodic.CFLAGS  += -DOVERO_LINK_MSG_UP=AutopilotMessageBethUp -DOVERO_LINK_MSG_DOWN=AutopilotMessageBethDown
overo_test_periodic.srcs    += $(SRC_FMS)/fms_spi_link.c

# test passthrough , aka using stm32 as io processor
# this demonstrates
#   -link with io processor
#   -periodic event
#   -telemetry and datalink
#
overo_test_passthrough.ARCHDIR  = omap
overo_test_passthrough.LDFLAGS += -levent -lm
overo_test_passthrough.CFLAGS  += -I$(ACINCLUDE) -I. -I$(PAPARAZZI_HOME)/var/include
overo_test_passthrough.CFLAGS  += -DOVERO_LINK_MSG_UP=AutopilotMessagePTUp -DOVERO_LINK_MSG_DOWN=AutopilotMessagePTDown
overo_test_passthrough.srcs     = $(SRC_FMS)/overo_test_passthrough.c
overo_test_passthrough.CFLAGS  += -DFMS_PERIODIC_FREQ=512
overo_test_passthrough.srcs    += $(SRC_FMS)/fms_periodic.c
overo_test_passthrough.srcs    += $(SRC_FMS)/fms_spi_link.c
overo_test_passthrough.srcs    += $(SRC_FMS)/fms_gs_com.c
overo_test_passthrough.CFLAGS  += -DDOWNLINK -DDOWNLINK_TRANSPORT=UdpTransport
overo_test_passthrough.srcs    += $(SRC_FMS)/udp_transport2.c downlink.c
overo_test_passthrough.srcs    += $(SRC_FMS)/fms_network.c



################################################################################
#
#
#  Those babies run on the stm32
#
#
################################################################################

ARCH=stm32
SRC_ARCH=$(ARCH)
SRC_LISA=lisa
SRC_LISA_ARCH=$(SRC_LISA)/arch/$(ARCH)
SRC_BOOZ=booz
SRC_BOOZ_ARCH=$(SRC_BOOZ)/arch/$(ARCH)

SRC_FIRMWARE=firmwares/rotorcraft
SRC_SUBSYSTEMS=subsystems

#BOARD_CFG=\"boards/olimex_stm32-h103.h\"
BOARD_CFG=\"boards/lisa_l_1.0.h\"
#FLASH_MODE = SERIAL
FLASH_MODE = JTAG

#
# test leds
#
test_led.ARCHDIR = $(ARCH)
test_led.CFLAGS += -I$(SRC_LISA) -I$(ARCH) -DPERIPHERALS_AUTO_INIT
test_led.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_led.srcs += $(SRC_LISA)/test_led.c         \
                 $(SRC_ARCH)/led_hw.c       \
                 $(SRC_ARCH)/stm32_exceptions.c   \
                 $(SRC_ARCH)/stm32_vector_table.c
test_led.CFLAGS += -DUSE_LED


#
# test leds2
#
test_led2.ARCHDIR = $(ARCH)
test_led2.CFLAGS += -I$(SRC_LISA) -I$(ARCH) -DPERIPHERALS_AUTO_INIT
test_led2.CFLAGS += -DBOARD_CONFIG=\"boards/lisa_0.99.h\"
test_led2.srcs += $(SRC_LISA)/test_led2.c              \
                 $(SRC_LISA)/exceptions.c              \
                 $(SRC_LISA)/vector_table.c
test_led2.CFLAGS += -DUSE_LED


#
# test periodic
#
test_periodic.ARCHDIR = $(ARCH)
test_periodic.CFLAGS += -I$(SRC_LISA) -I$(ARCH) -DPERIPHERALS_AUTO_INIT
test_periodic.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_periodic.srcs += $(SRC_LISA)/test_periodic.c  \
                      $(SRC_ARCH)/stm32_exceptions.c     \
                      $(SRC_ARCH)/stm32_vector_table.c
test_periodic.CFLAGS += -DUSE_LED
test_periodic.srcs += $(SRC_ARCH)/led_hw.c
test_periodic.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
test_periodic.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
test_periodic.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c


#
# test uart
#
test_uart.ARCHDIR = $(ARCH)
test_uart.CFLAGS = -I$(SRC_LISA) -I$(ARCH) -DPERIPHERALS_AUTO_INIT
test_uart.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_uart.srcs = $(SRC_LISA)/test_uart.c         \
                      $(SRC_LISA)/exceptions.c   \
                      $(SRC_LISA)/vector_table.c
test_uart.CFLAGS += -DUSE_LED
test_uart.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
test_uart.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
test_uart.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c
test_uart.CFLAGS += -DUSE_UART2 -DUART2_BAUD=B57600
test_uart.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c



#
# test telemetry1
#
test_telemetry1.ARCHDIR = $(ARCH)
test_telemetry1.CFLAGS = -I$(SRC_LISA) -I$(ARCH) -DPERIPHERALS_AUTO_INIT
test_telemetry1.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_telemetry1.srcs = $(SRC_LISA)/test_telemetry.c \
                       $(SRC_ARCH)/stm32_exceptions.c   \
                       $(SRC_ARCH)/stm32_vector_table.c
test_telemetry1.CFLAGS += -DUSE_LED
test_telemetry1.srcs += $(SRC_ARCH)/led_hw.c
test_telemetry1.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
test_telemetry1.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
test_telemetry1.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c
test_telemetry1.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600
test_telemetry1.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c
test_telemetry1.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart1
test_telemetry1.srcs += downlink.c pprz_transport.c


#
# test telemetry2
#
test_telemetry2.ARCHDIR = $(ARCH)
test_telemetry2.CFLAGS = -I$(SRC_LISA) -I$(ARCH) -DPERIPHERALS_AUTO_INIT
test_telemetry2.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_telemetry2.srcs = $(SRC_LISA)/test_telemetry.c \
                       $(SRC_ARCH)/stm32_exceptions.c   \
                       $(SRC_ARCH)/stm32_vector_table.c
test_telemetry2.CFLAGS += -DUSE_LED
test_telemetry2.srcs += $(SRC_ARCH)/led_hw.c
test_telemetry2.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
test_telemetry2.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
test_telemetry2.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c
test_telemetry2.CFLAGS += -DUSE_UART2 -DUART2_BAUD=B57600
test_telemetry2.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c
test_telemetry2.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart2
test_telemetry2.srcs += downlink.c pprz_transport.c


#
# test telemetry3
#
test_telemetry3.ARCHDIR = $(ARCH)
test_telemetry3.CFLAGS = -I$(SRC_LISA) -I$(ARCH) -DPERIPHERALS_AUTO_INIT
test_telemetry3.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_telemetry3.srcs = $(SRC_LISA)/test_telemetry.c \
                       $(SRC_ARCH)/stm32_exceptions.c   \
                       $(SRC_ARCH)/stm32_vector_table.c
test_telemetry3.CFLAGS += -DUSE_LED
test_telemetry3.srcs += $(SRC_ARCH)/led_hw.c
test_telemetry3.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
test_telemetry3.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
test_telemetry3.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c
test_telemetry3.CFLAGS += -DUSE_UART3 -DUART3_BAUD=B57600
test_telemetry3.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c
test_telemetry3.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart3
test_telemetry3.srcs += downlink.c pprz_transport.c

#
# test datalink
#
test_datalink.ARCHDIR = $(ARCH)
test_datalink.CFLAGS = -I$(SRC_LISA) -I$(ARCH) -DPERIPHERALS_AUTO_INIT
test_datalink.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_datalink.srcs = $(SRC_LISA)/test_datalink.c \
                     $(SRC_ARCH)/stm32_exceptions.c   \
                     $(SRC_ARCH)/stm32_vector_table.c
test_datalink.CFLAGS += -DUSE_LED
test_datalink.srcs += $(SRC_ARCH)/led_hw.c
test_datalink.CFLAGS += -DUSE_SYS_TIME  -DSYS_TIME_LED=1
test_datalink.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
test_datalink.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c
test_datalink.CFLAGS += -DUSE_UART2 -DUART2_BAUD=B57600
test_datalink.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c
test_datalink.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart2
test_datalink.srcs += downlink.c pprz_transport.c
test_datalink.CFLAGS += -DDATALINK=PPRZ -DPPRZ_UART=Uart2
#test_datalink.srcs += $(SRC_FIRMWARE)/datalink.c

#
# tunnel
#
tunnel.ARCHDIR = $(ARCH)
tunnel.CFLAGS  = -I$(SRC_LISA) -I$(ARCH) -DPERIPHERALS_AUTO_INIT
tunnel.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
tunnel.srcs += $(SRC_LISA)/test/lisa_tunnel.c \
           $(SRC_ARCH)/stm32_exceptions.c  \
               $(SRC_ARCH)/stm32_vector_table.c
tunnel.CFLAGS += -DUSE_LED
tunnel.srcs += $(SRC_ARCH)/led_hw.c
tunnel.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
tunnel.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))'
tunnel.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c

#tunnel.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B9600
#tunnel.CFLAGS += -DUSE_UART2 -DUART2_BAUD=B9600

#tunnel.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B38400
#tunnel.CFLAGS += -DUSE_UART2 -DUART2_BAUD=B38400

tunnel.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600
tunnel.CFLAGS += -DUSE_UART2 -DUART2_BAUD=B57600

#tunnel.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B115200
#tunnel.CFLAGS += -DUSE_UART2 -DUART2_BAUD=B115200
tunnel.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c



#
# test float
#
test_float.ARCHDIR = $(ARCH)
test_float.CFLAGS = -I$(SRC_LISA) -I$(ARCH) -DPERIPHERALS_AUTO_INIT
test_float.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_float.srcs = $(SRC_LISA)/test_float.c \
                       $(SRC_ARCH)/stm32_exceptions.c   \
                       $(SRC_ARCH)/stm32_vector_table.c
test_float.CFLAGS += -DUSE_LED
test_float.srcs += $(SRC_ARCH)/led_hw.c
test_float.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
test_float.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
test_float.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c
test_float.CFLAGS += -DUSE_UART2 -DUART2_BAUD=B57600
test_float.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c
test_float.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart2
test_float.srcs += downlink.c pprz_transport.c
test_float.srcs += lisa/plug_sys.c

#
# test bswap
#
test_bswap.ARCHDIR = $(ARCH)
test_bswap.CFLAGS = -I$(SRC_LISA) -I$(ARCH)
test_bswap.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_bswap.srcs = $(SRC_LISA)/test/test_bswap.c \



#
# test rc 2.4
#

SRC_BOOZ = booz
SRC_BOOZ_TEST = $(SRC_BOOZ)/test

test_rc_24.ARCHDIR = $(ARCH)
test_rc_24.CFLAGS += -I$(SRC_LISA) -I$(ARCH) -I$(SRC_BOOZ) -DPERIPHERALS_AUTO_INIT
test_rc_24.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_rc_24.srcs += $(SRC_BOOZ_TEST)/booz2_test_radio_control.c \
                   $(SRC_ARCH)/stm32_exceptions.c   \
                   $(SRC_ARCH)/stm32_vector_table.c

test_rc_24.CFLAGS += -DUSE_LED
test_rc_24.srcs += $(SRC_ARCH)/led_hw.c

test_rc_24.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
test_rc_24.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))'
test_rc_24.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c

test_rc_24.CFLAGS += -DUSE_UART2 -DUART2_BAUD=B57600
test_rc_24.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c
test_rc_24.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart2
test_rc_24.srcs += downlink.c pprz_transport.c

test_rc_24.CFLAGS += -DRADIO_CONTROL -DRADIO_CONTROL_LED=2
test_rc_24.CFLAGS += -DRADIO_CONTROL_TYPE_H=\"subsystems/radio_control/spektrum.h\"
test_rc_24.CFLAGS += -DRADIO_CONTROL_SPEKTRUM_MODEL_H=\"subsystems/radio_control/spektrum_dx7se.h\"
test_rc_24.CFLAGS += -DUSE_UART3 -DUART3_BAUD=B115200
test_rc_24.CFLAGS += -DRADIO_CONTROL_LINK=Uart3
test_rc_24.srcs += $(SRC_SUBSYSTEMS)/radio_control.c \
                   $(SRC_BOOZ)/subsystems/radio_control/spektrum.c
#                  $(SRC_ARCH)/mcu_periph/uart_arch.c


#
# test servos
#

SRC_BOOZ_ARCH=$(SRC_BOOZ)/arch/$(ARCH)

test_servos.ARCHDIR = $(ARCH)
test_servos.CFLAGS  = -I$(SRC_LISA) -I$(ARCH) -I$(SRC_FIRMWARE)/actuators/arch/$(ARCH) -DPERIPHERALS_AUTO_INIT
test_servos.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_servos.LDFLAGS += -lm
test_servos.srcs += $(SRC_LISA)/test_servos.c   \
                    $(SRC_ARCH)/stm32_exceptions.c   \
                    $(SRC_ARCH)/stm32_vector_table.c
test_servos.CFLAGS += -DUSE_LED
test_servos.srcs += $(SRC_ARCH)/led_hw.c
test_servos.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
test_servos.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))'
test_servos.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c

test_servos.srcs += $(SRC_FIRMWARE)/actuators/actuators_pwm.c $(SRC_FIRMWARE)/actuators/arch/$(ARCH)/actuators_pwm_arch.c


#test_servos.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600
#test_servos.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c
#test_servos.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart1
#test_servos.srcs += downlink.c pprz_transport.c






#
# test IMU b2
#
test_imu_b2.ARCHDIR = $(ARCH)
test_imu_b2.CFLAGS  = -I$(SRC_FIRMWARE) -I$(SRC_SUBSYSTEMS)/imu/arch/$(ARCH) -I$(SRC_LISA) -I$(ARCH) -I$(SRC_BOOZ) -I$(SRC_BOOZ_ARCH) -DPERIPHERALS_AUTO_INIT
test_imu_b2.CFLAGS +=  -DBOARD_CONFIG=$(BOARD_CFG)
test_imu_b2.srcs += $(SRC_BOOZ_TEST)/booz_test_imu.c \
                    $(SRC_ARCH)/stm32_exceptions.c   \
                    $(SRC_ARCH)/stm32_vector_table.c

test_imu_b2.CFLAGS += -DUSE_LED
test_imu_b2.srcs += $(SRC_ARCH)/led_hw.c

test_imu_b2.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
test_imu_b2.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))'
test_imu_b2.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c

test_imu_b2.CFLAGS += -DUSE_UART2 -DUART2_BAUD=B57600
test_imu_b2.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c

test_imu_b2.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart2
test_imu_b2.srcs += downlink.c pprz_transport.c

test_imu_b2.srcs += math/pprz_trig_int.c

test_imu_b2.CFLAGS += -DIMU_TYPE_H=\"imu/imu_b2.h\"
test_imu_b2.CFLAGS += -DIMU_B2_MAG_TYPE=IMU_B2_MAG_MS2100
test_imu_b2.srcs += $(SRC_SUBSYSTEMS)/imu.c
test_imu_b2.CFLAGS += -DUSE_SPI2 -DUSE_DMA1_C4_IRQ -DUSE_EXTI2_IRQ -DUSE_SPI2_IRQ
test_imu_b2.srcs += $(SRC_SUBSYSTEMS)/imu/imu_b2.c $(SRC_SUBSYSTEMS)/imu/arch/$(ARCH)/imu_b2_arch.c
test_imu_b2.srcs += peripherals/max1168.c $(SRC_ARCH)/peripherals/max1168_arch.c
test_imu_b2.srcs += peripherals/ms2100.c  $(SRC_ARCH)/peripherals/ms2100_arch.c


#
# test IMU crista
#
test_imu_crista.ARCHDIR = $(ARCH)
test_imu_crista.CFLAGS  = -I$(SRC_FIRMWARE) -I$(SRC_SUBSYSTEMS)/imu/arch/$(ARCH) -I$(SRC_LISA) -I$(ARCH) -I$(SRC_BOOZ) -I$(SRC_BOOZ_ARCH) -DPERIPHERALS_AUTO_INIT
test_imu_crista.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_imu_crista.srcs += $(SRC_BOOZ_TEST)/booz_test_imu.c \
                    $(SRC_ARCH)/stm32_exceptions.c   \
                    $(SRC_ARCH)/stm32_vector_table.c

test_imu_crista.CFLAGS += -DUSE_LED
test_imu_crista.srcs += $(SRC_ARCH)/led_hw.c

test_imu_crista.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
test_imu_crista.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))'
test_imu_crista.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c

test_imu_crista.CFLAGS += -DUSE_UART2 -DUART2_BAUD=B57600
test_imu_crista.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c

test_imu_crista.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart2
test_imu_crista.srcs += downlink.c pprz_transport.c

test_imu_crista.srcs += math/pprz_trig_int.c

test_imu_crista.CFLAGS += -DIMU_TYPE_H=\"imu/imu_crista.h\" -DIMU_OVERRIDE_CHANNELS
test_imu_crista.srcs += $(SRC_SUBSYSTEMS)/imu.c             \
                        $(SRC_SUBSYSTEMS)/imu/imu_crista.c \
                        $(SRC_SUBSYSTEMS)/imu/arch/$(ARCH)/imu_crista_arch.c
test_imu_crista.CFLAGS += -DUSE_DMA1_C4_IRQ


#
# test IMU aspirin
#
test_imu_aspirin.ARCHDIR = $(ARCH)
test_imu_aspirin.CFLAGS  = -I$(SRC_FIRMWARE) -I$(SRC_SUBSYSTEMS)/imu/arch/$(ARCH) -I$(SRC_LISA) -I$(ARCH) -I$(SRC_BOOZ) -I$(SRC_BOOZ_ARCH) -DPERIPHERALS_AUTO_INIT
test_imu_aspirin.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_imu_aspirin.srcs += $(SRC_BOOZ_TEST)/booz_test_imu.c \
                    $(SRC_ARCH)/stm32_exceptions.c   \
                    $(SRC_ARCH)/stm32_vector_table.c

test_imu_aspirin.CFLAGS += -DUSE_LED
test_imu_aspirin.srcs += $(SRC_ARCH)/led_hw.c

test_imu_aspirin.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
test_imu_aspirin.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))'
test_imu_aspirin.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c

test_imu_aspirin.CFLAGS += -DUSE_UART2 -DUART2_BAUD=B57600
test_imu_aspirin.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c

test_imu_aspirin.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart2
test_imu_aspirin.srcs += downlink.c pprz_transport.c

test_imu_aspirin.srcs += math/pprz_trig_int.c

test_imu_aspirin.CFLAGS += -DIMU_TYPE_H=\"imu/imu_aspirin.h\" -DIMU_OVERRIDE_CHANNELS
test_imu_aspirin.srcs += $(SRC_SUBSYSTEMS)/imu.c             \
                        $(SRC_SUBSYSTEMS)/imu/imu_aspirin.c \
                        $(SRC_SUBSYSTEMS)/imu/arch/$(ARCH)/imu_aspirin_arch.c

test_imu_aspirin.CFLAGS += -DUSE_I2C2
test_imu_aspirin.srcs += i2c.c $(SRC_ARCH)/i2c_hw.c
test_imu_aspirin.CFLAGS += -DUSE_EXTI15_10_IRQ  # Gyro Int on PC14
test_imu_aspirin.CFLAGS += -DUSE_EXTI9_5_IRQ    # Mag Int on PB5
test_imu_aspirin.CFLAGS += -DUSE_EXTI2_IRQ      # Accel Int on PD2
test_imu_aspirin.CFLAGS += -DUSE_DMA1_C4_IRQ    # SPI2 Rx DMA

#test_imu_aspirin.CFLAGS += -DI2C2_STOP_HANDLER=OnI2CDone -DI2C2_STOP_HANDLER_HEADER=\"imu/imu_aspirin_arch.h\"
#test_imu_aspirin.CFLAGS += -DUSE_DMA1_C4_IRQ




#
# test motor controllers
#
test_mc.ARCHDIR = $(ARCH)
test_mc.CFLAGS = -I$(SRC_LISA) -I$(ARCH) -DPERIPHERALS_AUTO_INIT
test_mc.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_mc.srcs = $(SRC_LISA)/test_mc.c      \
               $(SRC_ARCH)/stm32_exceptions.c   \
               $(SRC_ARCH)/stm32_vector_table.c
test_mc.CFLAGS += -DUSE_LED
test_mc.srcs += $(SRC_ARCH)/led_hw.c
test_mc.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
test_mc.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
test_mc.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c


#
# test motor controllers with interrupts
#
test_mc2.ARCHDIR = $(ARCH)
test_mc2.CFLAGS = -I$(SRC_LISA) -I$(ARCH) -DPERIPHERALS_AUTO_INIT
test_mc2.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_mc2.srcs = $(SRC_LISA)/test_mc2.c           \
                $(SRC_ARCH)/stm32_exceptions.c   \
                $(SRC_ARCH)/stm32_vector_table.c
test_mc2.CFLAGS += -DUSE_LED
test_mc2.srcs += $(SRC_ARCH)/led_hw.c
test_mc2.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
test_mc2.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
test_mc2.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c
test_mc2.CFLAGS += -DUSE_I2C1
test_mc2.srcs += i2c.c $(SRC_ARCH)/i2c_hw.c


#
# test motor controllers asctec with interrupts
#
test_mc_asctec_v1_simple.ARCHDIR = $(ARCH)
test_mc_asctec_v1_simple.CFLAGS = -I$(SRC_LISA) -I$(ARCH) -DPERIPHERALS_AUTO_INIT
test_mc_asctec_v1_simple.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_mc_asctec_v1_simple.srcs = $(SRC_LISA)/test/test_mc_asctec_v1_simple.c  \
                                $(SRC_ARCH)/stm32_exceptions.c               \
                                $(SRC_ARCH)/stm32_vector_table.c
test_mc_asctec_v1_simple.CFLAGS += -DUSE_LED
test_mc_asctec_v1_simple.srcs += $(SRC_ARCH)/led_hw.c
test_mc_asctec_v1_simple.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
test_mc_asctec_v1_simple.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
test_mc_asctec_v1_simple.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c
test_mc_asctec_v1_simple.CFLAGS += -DUSE_I2C1
test_mc_asctec_v1_simple.srcs += i2c.c $(SRC_ARCH)/i2c_hw.c
test_mc_asctec_v1_simple.CFLAGS += -DUSE_UART2 -DUART2_BAUD=B57600
test_mc_asctec_v1_simple.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c

test_mc_asctec_v1_simple.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart2
test_mc_asctec_v1_simple.srcs += downlink.c pprz_transport.c


#
# test motor controllers asctec2 with interrupts
#
test_mc5.ARCHDIR = $(ARCH)
test_mc5.CFLAGS = -I$(SRC_LISA) -I$(ARCH) -DPERIPHERALS_AUTO_INIT
test_mc5.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_mc5.srcs = $(SRC_LISA)/test_mc5.c           \
                $(SRC_ARCH)/stm32_exceptions.c   \
                $(SRC_ARCH)/stm32_vector_table.c
test_mc5.CFLAGS += -DUSE_LED
test_mc5.srcs += $(SRC_ARCH)/led_hw.c
test_mc5.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
test_mc5.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
test_mc5.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c
test_mc5.CFLAGS += -DUSE_I2C1
test_mc5.srcs += i2c.c $(SRC_ARCH)/i2c_hw.c




#
# test actuators mkk
#
test_actuators_mkk.ARCHDIR = $(ARCH)
test_actuators_mkk.CFLAGS = -I$(SRC_LISA) -I$(ARCH) -I$(SRC_BOOZ) -DPERIPHERALS_AUTO_INIT
test_actuators_mkk.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_actuators_mkk.srcs = $(SRC_LISA)/test/lisa_test_actuators_mkk.c \
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

test_actuators_mkk.srcs += $(SRC_BOOZ)/booz2_commands.c
test_actuators_mkk.srcs += $(SRC_FIRMWARE)/actuators/actuators_mkk.c
test_actuators_mkk.CFLAGS += -DACTUATORS_MKK_DEVICE=i2c1  -DUSE_TIM2_IRQ
#test_actuators_mkk.CFLAGS += -DACTUATORS_ASCTEC_V2_PROTOCOL -DACTUATORS_ASCTEC_DEVICE=i2c1
#test_actuators_mkk.srcs += $(SRC_FIRMWARE)/actuators/actuators_asctec.c
test_actuators_mkk.srcs += $(SRC_FIRMWARE)/actuators/supervision.c
test_actuators_mkk.CFLAGS += -DUSE_I2C1
test_actuators_mkk.srcs += i2c.c $(SRC_ARCH)/i2c_hw.c


#
# test actuators asctec
#
test_actuators_asctec.ARCHDIR = $(ARCH)
test_actuators_asctec.CFLAGS = -I$(SRC_LISA) -I$(ARCH) -I$(SRC_BOOZ) -DPERIPHERALS_AUTO_INIT
test_actuators_asctec.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_actuators_asctec.srcs = $(SRC_LISA)/test/lisa_test_actuators_mkk.c \
                          $(SRC_ARCH)/stm32_exceptions.c   \
                          $(SRC_ARCH)/stm32_vector_table.c

test_actuators_asctec.CFLAGS += -DUSE_LED
test_actuators_asctec.srcs += $(SRC_ARCH)/led_hw.c

test_actuators_asctec.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
test_actuators_asctec.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
test_actuators_asctec.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c

test_actuators_asctec.CFLAGS += -DUSE_UART2 -DUART2_BAUD=B57600
test_actuators_asctec.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c

test_actuators_asctec.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart2
test_actuators_asctec.srcs += downlink.c pprz_transport.c

test_actuators_asctec.srcs += $(SRC_BOOZ)/booz2_commands.c
test_actuators_asctec.srcs += $(SRC_FIRMWARE)/actuators/actuators_asctec.c
test_actuators_asctec.CFLAGS += -DACTUATORS_ASCTEC_DEVICE=i2c1
# -DBOOZ_START_DELAY=3
#  -DUSE_TIM2_IRQ
test_actuators_asctec.CFLAGS += -DUSE_I2C1
test_actuators_asctec.srcs += i2c.c $(SRC_ARCH)/i2c_hw.c






#
# test motor controllers asctec
#
test_mc3.ARCHDIR = $(ARCH)
test_mc3.CFLAGS = -I$(SRC_LISA) -I$(ARCH) -DPERIPHERALS_AUTO_INIT
test_mc3.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_mc3.srcs = $(SRC_LISA)/test_mc3.c      \
                $(SRC_LISA)/exceptions.c    \
                $(SRC_LISA)/vector_table.c
test_mc3.CFLAGS += -DUSE_LED
test_mc3.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
test_mc3.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
test_mc3.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c




#
# test baro
#
test_baro.ARCHDIR = $(ARCH)
test_baro.CFLAGS = -I$(SRC_LISA) -I$(ARCH) -DPERIPHERALS_AUTO_INIT
test_baro.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_baro.srcs = $(SRC_LISA)/test_baro.c      \
                 $(SRC_ARCH)/stm32_exceptions.c   \
                 $(SRC_ARCH)/stm32_vector_table.c
test_baro.CFLAGS += -DUSE_LED
test_baro.srcs += $(SRC_ARCH)/led_hw.c
test_baro.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
test_baro.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
test_baro.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c

test_baro.CFLAGS += -DUSE_UART2 -DUART2_BAUD=B57600
test_baro.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c

test_baro.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart2
test_baro.srcs += downlink.c pprz_transport.c


#
# test baro with interrupts
#
test_baro2.ARCHDIR = $(ARCH)
test_baro2.CFLAGS = -I$(SRC_LISA) -I$(ARCH) -DPERIPHERALS_AUTO_INIT
test_baro2.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_baro2.srcs = $(SRC_LISA)/test_baro2.c      \
                 $(SRC_ARCH)/stm32_exceptions.c   \
                 $(SRC_ARCH)/stm32_vector_table.c
test_baro2.CFLAGS += -DUSE_LED
test_baro2.srcs += $(SRC_ARCH)/led_hw.c
test_baro2.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
test_baro2.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
test_baro2.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c

test_baro2.CFLAGS += -DUSE_UART2 -DUART2_BAUD=B57600
test_baro2.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c

test_baro2.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart2
test_baro2.srcs += downlink.c pprz_transport.c

test_baro2.srcs += $(SRC_LISA)/lisa_baro.c
test_baro2.CFLAGS += -DUSE_I2C2
test_baro2.srcs += i2c.c $(SRC_ARCH)/i2c_hw.c


#
# another baro test with interrupts
#
test_baro3.ARCHDIR = $(ARCH)
test_baro3.CFLAGS = -I$(SRC_LISA) -I$(ARCH) -DPERIPHERALS_AUTO_INIT
test_baro3.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_baro3.srcs = $(SRC_LISA)/test_baro3.c      \
                  $(SRC_ARCH)/stm32_exceptions.c   \
                  $(SRC_ARCH)/stm32_vector_table.c
test_baro3.CFLAGS += -DUSE_LED
test_baro3.srcs += $(SRC_ARCH)/led_hw.c
test_baro3.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
test_baro3.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
test_baro3.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c

test_baro3.CFLAGS += -DUSE_UART2 -DUART2_BAUD=B57600
test_baro3.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c

test_baro3.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart2
test_baro3.srcs += downlink.c pprz_transport.c

test_baro3.CFLAGS += -DUSE_I2C2
test_baro3.srcs += i2c.c $(SRC_ARCH)/i2c_hw.c



#
# test spi slave ( hardcoded SPI without DMA )
#
test_spi_slave.ARCHDIR = $(ARCH)
test_spi_slave.CFLAGS = -I$(SRC_LISA) -I$(ARCH) -DPERIPHERALS_AUTO_INIT
test_spi_slave.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_spi_slave.srcs = $(SRC_LISA)/test_spi_slave.c     \
                      $(SRC_ARCH)/stm32_exceptions.c   \
                      $(SRC_ARCH)/stm32_vector_table.c
test_spi_slave.CFLAGS += -DUSE_LED
test_spi_slave.srcs += $(SRC_ARCH)/led_hw.c
test_spi_slave.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
test_spi_slave.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
test_spi_slave.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c
test_spi_slave.CFLAGS += -DUSE_SPI1_IRQ
test_spi_slave.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600
test_spi_slave.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c
test_spi_slave.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart1
test_spi_slave.srcs += downlink.c pprz_transport.c


#
# test spi slave2  ( hardcoded SPI with DMA )
#
test_spi_slave2.ARCHDIR = $(ARCH)
test_spi_slave2.CFLAGS = -I$(SRC_LISA) -I$(ARCH) -DPERIPHERALS_AUTO_INIT
test_spi_slave2.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_spi_slave2.srcs = $(SRC_LISA)/test_spi_slave2.c    \
                       $(SRC_ARCH)/stm32_exceptions.c   \
                       $(SRC_ARCH)/stm32_vector_table.c
test_spi_slave2.CFLAGS += -DUSE_LED
test_spi_slave2.srcs += $(SRC_ARCH)/led_hw.c
test_spi_slave2.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
test_spi_slave2.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
test_spi_slave2.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c
test_spi_slave2.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600
test_spi_slave2.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c
test_spi_slave2.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart1
test_spi_slave2.srcs += downlink.c pprz_transport.c



#
# test spi link between overo and stm32
#
stm_test_spi_link.ARCHDIR = $(ARCH)
stm_test_spi_link.CFLAGS += -Ilisa -Ilisa/arch/$(ARCH) -I$(ARCH) -DPERIPHERALS_AUTO_INIT
stm_test_spi_link.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
stm_test_spi_link.srcs += lisa/stm_test_spi_link.c       \
                      $(SRC_ARCH)/stm32_exceptions.c   \
                  $(SRC_ARCH)/stm32_vector_table.c

stm_test_spi_link.CFLAGS += -DUSE_LED
stm_test_spi_link.srcs += $(SRC_ARCH)/led_hw.c

stm_test_spi_link.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
stm_test_spi_link.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
stm_test_spi_link.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c

stm_test_spi_link.CFLAGS += -DUSE_UART2 -DUART2_BAUD=B57600
stm_test_spi_link.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c

stm_test_spi_link.CFLAGS += -DUSE_OVERO_LINK -DOVERO_LINK_MSG_UP=AutopilotMessageFoo -DOVERO_LINK_MSG_DOWN=AutopilotMessageFoo
stm_test_spi_link.CFLAGS += -DOVERO_LINK_LED_OK=3 -DOVERO_LINK_LED_KO=2 -DUSE_DMA1_C2_IRQ
stm_test_spi_link.srcs += lisa/lisa_overo_link.c lisa/arch/stm32/lisa_overo_link_arch.c





#
# test static
#
test_static.ARCHDIR = $(ARCH)
test_static.CFLAGS = -I$(SRC_LISA) -I$(ARCH) -DPERIPHERALS_AUTO_INIT
test_static.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_static.srcs = $(SRC_LISA)/test_static.c \
                   $(SRC_ARCH)/stm32_exceptions.c   \
                   $(SRC_ARCH)/stm32_vector_table.c



#
# test SC18IS600
#
test_sc18is600.ARCHDIR = $(ARCH)
test_sc18is600.CFLAGS  =  -I$(SRC_LISA) -I$(ARCH) -I$(SRC_BOOZ) -I$(SRC_BOOZ_ARCH) -DPERIPHERALS_AUTO_INIT
test_sc18is600.CFLAGS +=  -DBOARD_CONFIG=$(BOARD_CFG)
test_sc18is600.srcs += lisa/test/lisa_test_sc18is600.c \
                       $(SRC_ARCH)/stm32_exceptions.c   \
                       $(SRC_ARCH)/stm32_vector_table.c

test_sc18is600.CFLAGS += -DUSE_LED
test_sc18is600.srcs += $(SRC_ARCH)/led_hw.c

test_sc18is600.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
test_sc18is600.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./128.))'
test_sc18is600.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c

test_sc18is600.CFLAGS += -DUSE_UART2 -DUART2_BAUD=B57600
test_sc18is600.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c

test_sc18is600.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart2
test_sc18is600.srcs += downlink.c pprz_transport.c

test_sc18is600.srcs += math/pprz_trig_int.c

test_sc18is600.CFLAGS += -DUSE_EXTI2_IRQ  -DUSE_DMA1_C4_IRQ
test_sc18is600.srcs += peripherals/sc18is600.c \
                       $(SRC_ARCH)/peripherals/sc18is600_arch.c


#
# test Max1168
#
test_max1168.ARCHDIR = $(ARCH)
test_max1168.CFLAGS = -I$(SRC_LISA) -I$(ARCH) -I$(SRC_BOOZ) -I$(SRC_BOOZ_ARCH) -DPERIPHERALS_AUTO_INIT
test_max1168.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_max1168.srcs = $(SRC_LISA)/test/lisa_test_max1168.c \
                    $(SRC_ARCH)/stm32_exceptions.c   \
                    $(SRC_ARCH)/stm32_vector_table.c

test_max1168.CFLAGS += -DUSE_LED
test_max1168.srcs += $(SRC_ARCH)/led_hw.c

test_max1168.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
test_max1168.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
test_max1168.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c

test_max1168.CFLAGS += -DUSE_SPI2 -DUSE_EXTI2_IRQ -DUSE_DMA1_C4_IRQ -DMAX1168_HANDLES_DMA_IRQ
test_max1168.srcs   += peripherals/max1168.c \
                       $(SRC_ARCH)/peripherals/max1168_arch.c

test_max1168.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600
test_max1168.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c

test_max1168.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart1
test_max1168.srcs += downlink.c pprz_transport.c

#
# test ms2100
#
test_ms2100.ARCHDIR = $(ARCH)
test_ms2100.CFLAGS = -I$(SRC_LISA) -I$(ARCH) -I$(SRC_BOOZ) -I$(SRC_BOOZ_ARCH) -DPERIPHERALS_AUTO_INIT
test_ms2100.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_ms2100.srcs = $(SRC_LISA)/test/lisa_test_ms2100.c \
                   $(SRC_ARCH)/stm32_exceptions.c   \
                   $(SRC_ARCH)/stm32_vector_table.c

test_ms2100.CFLAGS += -DUSE_LED
test_ms2100.srcs += $(SRC_ARCH)/led_hw.c

test_ms2100.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
test_ms2100.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
test_ms2100.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c

test_ms2100.CFLAGS += -DUSE_SPI2
test_ms2100.CFLAGS += -DUSE_DMA1_C4_IRQ -DMS2100_HANDLES_DMA_IRQ
test_ms2100.CFLAGS += -DUSE_SPI2_IRQ -DMS2100_HANDLES_SPI_IRQ
test_ms2100.srcs   += peripherals/ms2100.c \
                      $(SRC_ARCH)/peripherals/ms2100_arch.c

test_ms2100.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600
test_ms2100.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c

test_ms2100.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart1
test_ms2100.srcs += downlink.c pprz_transport.c

#
# test adxl345
#
test_adxl345.ARCHDIR = $(ARCH)
test_adxl345.CFLAGS  =  -I$(SRC_LISA) -I$(ARCH) -I$(SRC_BOOZ) -I$(SRC_BOOZ_ARCH) -DPERIPHERALS_AUTO_INIT
test_adxl345.CFLAGS +=  -DBOARD_CONFIG=$(BOARD_CFG)
test_adxl345.srcs += lisa/test/lisa_test_adxl345.c \
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

test_adxl345.CFLAGS += -DUSE_EXTI2_IRQ   # Acc  Int on PD2

#
# test adxl345 with DMA
#
test_adxl345_dma.ARCHDIR = $(ARCH)
test_adxl345_dma.CFLAGS  =  -I$(SRC_LISA) -I$(ARCH) -I$(SRC_BOOZ) -I$(SRC_BOOZ_ARCH) -DPERIPHERALS_AUTO_INIT
test_adxl345_dma.CFLAGS +=  -DBOARD_CONFIG=$(BOARD_CFG)
test_adxl345_dma.srcs += lisa/test/lisa_test_adxl345_dma.c \
                       $(SRC_ARCH)/stm32_exceptions.c   \
                       $(SRC_ARCH)/stm32_vector_table.c

test_adxl345_dma.CFLAGS += -DUSE_LED
test_adxl345_dma.srcs += $(SRC_ARCH)/led_hw.c

test_adxl345_dma.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
test_adxl345_dma.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))'
test_adxl345_dma.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c

test_adxl345_dma.CFLAGS += -DUSE_UART2 -DUART2_BAUD=B57600
test_adxl345_dma.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c

test_adxl345_dma.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart2
test_adxl345_dma.srcs += downlink.c pprz_transport.c

test_adxl345_dma.CFLAGS += -DUSE_EXTI2_IRQ   # Accel Int on PD2
test_adxl345_dma.CFLAGS += -DUSE_DMA1_C4_IRQ # SPI2 Rx DMA




#
# test ITG3200
#
test_itg3200.ARCHDIR = $(ARCH)
test_itg3200.CFLAGS  =  -I$(SRC_LISA) -I$(ARCH) -I$(SRC_BOOZ) -I$(SRC_BOOZ_ARCH) -DPERIPHERALS_AUTO_INIT
test_itg3200.CFLAGS +=  -DBOARD_CONFIG=$(BOARD_CFG)
test_itg3200.srcs += lisa/test/lisa_test_itg3200.c \
                       $(SRC_ARCH)/stm32_exceptions.c   \
                       $(SRC_ARCH)/stm32_vector_table.c

test_itg3200.CFLAGS += -DUSE_LED
test_itg3200.srcs += $(SRC_ARCH)/led_hw.c

test_itg3200.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
test_itg3200.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))'
test_itg3200.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c

test_itg3200.CFLAGS += -DUSE_UART2 -DUART2_BAUD=B57600
test_itg3200.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c

test_itg3200.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart2
test_itg3200.srcs += downlink.c pprz_transport.c

test_itg3200.CFLAGS += -DUSE_I2C2
test_itg3200.srcs += i2c.c $(SRC_ARCH)/i2c_hw.c
test_itg3200.CFLAGS += -DUSE_EXTI15_10_IRQ   # Gyro Int on PC14


#
# test hmc5843
#
test_hmc5843.ARCHDIR = $(ARCH)
test_hmc5843.CFLAGS = -I$(SRC_LISA) -I$(ARCH) -Ibooz -DPERIPHERALS_AUTO_INIT
test_hmc5843.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_hmc5843.srcs = lisa/test/lisa_test_hmc5843.c         \
                    $(SRC_ARCH)/stm32_exceptions.c   \
                    $(SRC_ARCH)/stm32_vector_table.c
test_hmc5843.CFLAGS += -DUSE_LED
test_hmc5843.srcs += $(SRC_ARCH)/led_hw.c
test_hmc5843.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
test_hmc5843.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
test_hmc5843.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c

test_hmc5843.CFLAGS += -DUSE_UART2 -DUART2_BAUD=B57600
test_hmc5843.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c

test_hmc5843.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart2
test_hmc5843.srcs += downlink.c pprz_transport.c

test_hmc5843.CFLAGS += -DUSE_I2C2
test_hmc5843.srcs += i2c.c $(SRC_ARCH)/i2c_hw.c
test_hmc5843.CFLAGS += -DIMU_OVERRIDE_CHANNELS
test_hmc5843.CFLAGS += -DUSE_EXTI9_5_IRQ   # Mag Int on PB5





#
# test Aspirin ( rewired ) no sc18is600
#
test_aspirin.ARCHDIR = $(ARCH)
test_aspirin.CFLAGS  =  -I$(SRC_LISA) -I$(ARCH) -I$(SRC_BOOZ) -I$(SRC_BOOZ_ARCH) -DPERIPHERALS_AUTO_INIT
test_aspirin.CFLAGS +=  -DBOARD_CONFIG=$(BOARD_CFG)
test_aspirin.srcs += lisa/test/lisa_test_aspirin.c \
                       $(SRC_ARCH)/stm32_exceptions.c   \
                       $(SRC_ARCH)/stm32_vector_table.c

test_aspirin.CFLAGS += -DUSE_LED
test_aspirin.srcs += $(SRC_ARCH)/led_hw.c

test_aspirin.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
test_aspirin.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))'
test_aspirin.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c

test_aspirin.CFLAGS += -DUSE_UART2 -DUART2_BAUD=B57600
test_aspirin.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c

test_aspirin.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart2
test_aspirin.srcs += downlink.c pprz_transport.c

test_aspirin.CFLAGS += -DUSE_I2C2
test_aspirin.srcs += i2c.c $(SRC_ARCH)/i2c_hw.c
test_aspirin.CFLAGS += -DUSE_EXTI2_IRQ   # Gyro Int
test_aspirin.CFLAGS += -DUSE_EXTI3_IRQ   # Mag  Int
test_aspirin.CFLAGS += -DUSE_EXTI4_IRQ   # Acc  Int




#
#
# Passing  STM32 telemetry through WIFI
#
#

ptw.ARCHDIR = stm32
ptw.CFLAGS += -I$(SRC_LISA) -I$(SRC_BOOZ) -I$(SRC_BOOZ_ARCH) -DPERIPHERALS_AUTO_INIT
ptw.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
ptw.srcs = $(SRC_LISA)/test/lisa_test_stm_wifi_telemetry.c \
          $(SRC_ARCH)/stm32_exceptions.c   \
          $(SRC_ARCH)/stm32_vector_table.c

# Leds
ptw.CFLAGS += -DUSE_LED
ptw.srcs += $(SRC_ARCH)/led_hw.c

# Sys time
ptw.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
ptw.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
ptw.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c

# Link Overo
ptw.CFLAGS += -DUSE_OVERO_LINK -DOVERO_LINK_MSG_UNION=AutopilotMessageTW
ptw.CFLAGS += -DOVERO_LINK_LED_OK=3 -DOVERO_LINK_LED_KO=4 -DUSE_DMA1_C2_IRQ
ptw.srcs += lisa/lisa_overo_link.c lisa/arch/stm32/lisa_overo_link_arch.c

# Telemetry
ptw.CFLAGS += -DUSE_OVERO_LINK_TELEMETRY
#ptw.CFLAGS += -DUSE_UART2 -DUART2_BAUD=B57600
#ptw.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c
ptw.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=OveroLinkTelemetry
ptw.srcs += downlink.c pprz_transport.c

# IMU
ptw.CFLAGS += -DIMU_TYPE_H=\"imu/imu_b2.h\"
ptw.CFLAGS += -DIMU_B2_MAG_TYPE=IMU_B2_MAG_MS2100
ptw.srcs += $(SRC_SUBSYSTEMS)/imu.c
ptw.CFLAGS += -DUSE_SPI2 -DUSE_DMA1_C4_IRQ -DUSE_EXTI2_IRQ -DUSE_SPI2_IRQ
ptw.srcs += $(SRC_SUBSYSTEMS)/imu/imu_b2.c $(SRC_SUBSYSTEMS)/imu/arch/$(ARCH)/imu_b2_arch.c
ptw.srcs += peripherals/max1168.c $(SRC_ARCH)/peripherals/max1168_arch.c
ptw.srcs += peripherals/ms2100.c  $(SRC_ARCH)/peripherals/ms2100_arch.c
ptw.srcs += math/pprz_trig_int.c

ptw.srcs += $(SRC_BOOZ)/booz2_commands.c

# Radio control
ptw.CFLAGS += -DRADIO_CONTROL
ptw.CFLAGS += -DRADIO_CONTROL_TYPE_H=\"subsystems/radio_control/spektrum.h\"
ptw.CFLAGS += -DRADIO_CONTROL_SPEKTRUM_MODEL_H=\"subsystems/radio_control/spektrum_dx7se.h\"
ptw.srcs += $(SRC_SUBSYSTEMS)/radio_control.c \
           $(SRC_BOOZ)/subsystems/radio_control/spektrum.c
ptw.CFLAGS += -DRADIO_CONTROL_LED=6
ptw.CFLAGS += -DUSE_UART3 -DUART3_BAUD=B115200
ptw.CFLAGS += -DRADIO_CONTROL_LINK=Uart3
ptw.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c

# Actuators
ptw.srcs += $(SRC_FIRMWARE)/actuators/supervision.c
ptw.srcs += $(SRC_FIRMWARE)/actuators/actuators_mkk.c
#ptw.srcs += $(SRC_BOOZ_ARCH)/actuators/actuators_mkk_arch.c
ptw.srcs += i2c.c $(SRC_ARCH)/i2c_hw.c
ptw.CFLAGS += -DACTUATORS_MKK_DEVICE=i2c1  -DUSE_TIM2_IRQ
ptw.CFLAGS += -DUSE_I2C1

#
# test csc servo
#
test_csc_servo.ARCHDIR = $(ARCH)
test_csc_servo.CFLAGS = -I $(SRC_CSC) -I$(SRC_LISA) -I$(ARCH) -DPERIPHERALS_AUTO_INIT
test_csc_servo.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_csc_servo.srcs = $(SRC_CSC)/csc_protocol.c \
        $(SRC_LISA)/test_csc_servo.c      \
        $(SRC_ARCH)/stm32_exceptions.c   \
        $(SRC_ARCH)/stm32_vector_table.c
test_csc_servo.CFLAGS += -DUSE_LED
test_csc_servo.srcs += $(SRC_ARCH)/led_hw.c
test_csc_servo.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
test_csc_servo.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
test_csc_servo.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c

test_csc_servo.CFLAGS += -DUSE_UART2 -DUART2_BAUD=B57600
test_csc_servo.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c
test_csc_servo.CFLAGS += -DDATALINK=PPRZ -DPPRZ_UART=Uart2

test_csc_servo.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart2
test_csc_servo.srcs += downlink.c pprz_transport.c

# setting CAN prescaler to generate 3MHz time quanta, drift compensiation to 1
# time quanta, bit section 1 to 3 time quanta and bit section 2 to 4 time quanta
# resulting in a 375kHz CAN bitrate expected by the CSC.
test_csc_servo.CFLAGS += \
    -DUSE_CAN1 \
    -DUSE_USB_LP_CAN1_RX0_IRQ \
    -DCAN_PRESCALER=12 \
    -DCAN_SJW_TQ=CAN_SJW_1tq \
    -DCAN_BS1_TQ=CAN_BS1_3tq \
    -DCAN_BS2_TQ=CAN_BS2_4tq \
    -DCAN_ERR_RESUME=DISABLE
test_csc_servo.srcs += can.c $(SRC_ARCH)/can_hw.c




#
# test GPS
#
test_gps.ARCHDIR = $(ARCH)
test_gps.CFLAGS = -I$(ARCH) -DPERIPHERALS_AUTO_INIT
test_gps.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG) -I$(SRC_BOOZ) -I$(SRC_BOOZ_ARCH)
test_gps.srcs += $(SRC_BOOZ_TEST)/booz2_test_gps.c \
         $(SRC_ARCH)/stm32_exceptions.c    \
         $(SRC_ARCH)/stm32_vector_table.c
test_gps.CFLAGS += -DUSE_LED
test_gps.srcs += $(SRC_ARCH)/led_hw.c
test_gps.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
test_gps.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))' -DTIME_LED=1
test_gps.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c

test_gps.CFLAGS += -DUSE_UART2 -DUART2_BAUD=B57600
test_gps.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c

test_gps.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart2
test_gps.srcs += downlink.c pprz_transport.c

test_gps.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B38400
test_gps.CFLAGS += -DGPS_LINK=Uart1 -DGPS_LED=3
test_gps.srcs += $(SRC_BOOZ)/booz_gps.c
#test_gps.CFLAGS += -DBOOZ_GPS_TYPE_H=\"gps/booz_gps_ubx.h\"
#test_gps.srcs += $(SRC_BOOZ)/gps/booz_gps_ubx.c
test_gps.CFLAGS += -DBOOZ_GPS_TYPE_H=\"gps/booz_gps_skytraq.h\"
test_gps.srcs += $(SRC_BOOZ)/gps/booz_gps_skytraq.c



#
# test ADC
#
# test_adc.ARCHDIR = $(ARCH)
# test_adc.CFLAGS = -I$(ARCH) -DPERIPHERALS_AUTO_INIT
# test_adc.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG) -I$(SRC_BOOZ) -I$(SRC_BOOZ_ARCH)
# test_adc.srcs += $(SRC_LISA)/test/lisa_test_adc.c \
#        $(SRC_ARCH)/stm32_exceptions.c   \
#        $(SRC_ARCH)/stm32_vector_table.c
# test_adc.CFLAGS += -DUSE_LED
# test_adc.srcs += $(SRC_ARCH)/led_hw.c
# test_adc.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
# test_adc.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))' -DTIME_LED=1
# test_adc.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c
#
# test_adc.CFLAGS += -DUSE_UART2 -DUART2_BAUD=B57600
# test_adc.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c
#
# test_adc.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart2
# test_adc.srcs += downlink.c pprz_transport.c
#
# test_adc.srcs += $(SRC_ARCH)/adc_hw.c

#
# test adc
#
test_adc.ARCHDIR = $(ARCH)
test_adc.CFLAGS = -I$(SRC_LISA) -I$(ARCH) -DPERIPHERALS_AUTO_INIT
test_adc.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_adc.srcs = $(SRC_ARCH)/adc_hw.c \
        $(SRC_LISA)/test_adc.c \
        $(SRC_ARCH)/stm32_exceptions.c \
        $(SRC_ARCH)/stm32_vector_table.c
test_adc.CFLAGS += -DUSE_LED
test_adc.srcs += $(SRC_ARCH)/led_hw.c
test_adc.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
test_adc.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
test_adc.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c

test_adc.CFLAGS += -DUSE_UART2 -DUART2_BAUD=B57600
test_adc.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c
test_adc.CFLAGS += -DDATALINK=PPRZ -DPPRZ_UART=Uart2

test_adc.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart2
test_adc.srcs += downlink.c pprz_transport.c

test_adc.CFLAGS += -DUSE_AD1 -DUSE_AD1_1 -DUSE_AD1_2 -DUSE_AD1_3 -DUSE_AD1_4 -DUSE_ADC1_2_IRQ_HANDLER

################################################################################
#
#
#  Hardware test suite
#
#
################################################################################
test_board.ARCHDIR = $(ARCH)
test_board.CFLAGS = -I$(SRC_LISA) -I$(ARCH) -I$(SRC_BOOZ) -I$(SRC_BOOZ_ARCH) -DPERIPHERALS_AUTO_INIT
test_board.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_board.srcs = $(SRC_LISA)/test/test_board.c       \
                     $(SRC_ARCH)/stm32_exceptions.c   \
                     $(SRC_ARCH)/stm32_vector_table.c
test_board.CFLAGS += -DUSE_LED
test_board.srcs += $(SRC_ARCH)/led_hw.c
test_board.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
test_board.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
test_board.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c

test_board.CFLAGS += -DUSE_UART2 -DUART2_BAUD=B57600
test_board.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c
test_board.CFLAGS += -DDATALINK=PPRZ -DPPRZ_UART=Uart2

test_board.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart2
test_board.srcs += downlink.c pprz_transport.c

test_board.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600
test_board.CFLAGS += -DUSE_UART3 -DUART3_BAUD=B57600

test_board.srcs += $(SRC_LISA)/lisa_baro.c
test_board.CFLAGS += -DUSE_I2C2
test_board.srcs += i2c.c $(SRC_ARCH)/i2c_hw.c

test_board.CFLAGS += -DUSE_I2C1

test_board.srcs += $(SRC_FIRMWARE)/actuators/actuators_pwm.c $(SRC_FIRMWARE)/actuators/arch/$(ARCH)/actuators_pwm_arch.c




################################################################################
#
#
#  Tools for IMUs comparison
#
#
################################################################################

#
# Spits every samples of one axis of gyro on IMU aspirin
#
hs_gyro_aspirin.ARCHDIR = $(ARCH)
hs_gyro_aspirin.CFLAGS  =  -I$(SRC_LISA) -I$(ARCH) -I$(SRC_BOOZ) -I$(SRC_BOOZ_ARCH) -DPERIPHERALS_AUTO_INIT
hs_gyro_aspirin.CFLAGS +=  -DBOARD_CONFIG=$(BOARD_CFG)
hs_gyro_aspirin.srcs += lisa/test/hs_gyro.c \
                    $(SRC_ARCH)/stm32_exceptions.c   \
                    $(SRC_ARCH)/stm32_vector_table.c

hs_gyro_aspirin.CFLAGS += -DUSE_LED
hs_gyro_aspirin.srcs += $(SRC_ARCH)/led_hw.c

hs_gyro_aspirin.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
hs_gyro_aspirin.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))'
hs_gyro_aspirin.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c

hs_gyro_aspirin.CFLAGS += -DUSE_UART2 -DUART2_BAUD=B57600
hs_gyro_aspirin.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c

hs_gyro_aspirin.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart2
hs_gyro_aspirin.srcs += downlink.c pprz_transport.c

hs_gyro_aspirin.srcs += math/pprz_trig_int.c

hs_gyro_aspirin.CFLAGS += -DIMU_TYPE_H=\"imu/imu_aspirin.h\" -DIMU_OVERRIDE_CHANNELS
hs_gyro_aspirin.srcs += $(SRC_SUBSYSTEMS)/imu.c             \
                        $(SRC_SUBSYSTEMS)/imu/imu_aspirin.c \
                        $(SRC_SUBSYSTEMS)/imu/arch/$(ARCH)/imu_aspirin_arch.c

hs_gyro_aspirin.CFLAGS += -DUSE_I2C2
hs_gyro_aspirin.srcs += i2c.c $(SRC_ARCH)/i2c_hw.c
hs_gyro_aspirin.CFLAGS += -DUSE_EXTI15_10_IRQ  # Gyro Int on PC14
hs_gyro_aspirin.CFLAGS += -DUSE_EXTI9_5_IRQ    # Mag Int on PB5
hs_gyro_aspirin.CFLAGS += -DUSE_EXTI2_IRQ      # Accel Int on PD2
hs_gyro_aspirin.CFLAGS += -DUSE_DMA1_C4_IRQ    # SPI2 Rx DMA


#
# Spits every samples of one axis of gyro on IMU b2
#
hs_gyro_b2.ARCHDIR = $(ARCH)
hs_gyro_b2.CFLAGS  =  -I$(SRC_LISA) -I$(ARCH) -I$(SRC_BOOZ) -I$(SRC_BOOZ_ARCH) -DPERIPHERALS_AUTO_INIT
hs_gyro_b2.CFLAGS +=  -DBOARD_CONFIG=$(BOARD_CFG)
hs_gyro_b2.srcs += lisa/test/hs_gyro.c \
                    $(SRC_ARCH)/stm32_exceptions.c   \
                    $(SRC_ARCH)/stm32_vector_table.c

hs_gyro_b2.CFLAGS += -DUSE_LED
hs_gyro_b2.srcs += $(SRC_ARCH)/led_hw.c

hs_gyro_b2.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
hs_gyro_b2.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))'
hs_gyro_b2.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c

hs_gyro_b2.CFLAGS += -DUSE_UART2 -DUART2_BAUD=B57600
hs_gyro_b2.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c

hs_gyro_b2.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart2
hs_gyro_b2.srcs += downlink.c pprz_transport.c

hs_gyro_b2.srcs += math/pprz_trig_int.c

hs_gyro_b2.CFLAGS += -DIMU_TYPE_H=\"imu/imu_b2.h\"
hs_gyro_b2.CFLAGS += -DIMU_B2_MAG_TYPE=IMU_B2_MAG_MS2100
hs_gyro_b2.srcs += $(SRC_SUBSYSTEMS)/imu.c
hs_gyro_b2.CFLAGS += -DUSE_SPI2 -DUSE_DMA1_C4_IRQ -DUSE_EXTI2_IRQ -DUSE_SPI2_IRQ
hs_gyro_b2.srcs += $(SRC_SUBSYSTEMS)/imu/imu_b2.c $(SRC_SUBSYSTEMS)/imu/arch/$(ARCH)/imu_b2_arch.c
hs_gyro_b2.srcs += peripherals/max1168.c $(SRC_ARCH)/peripherals/max1168_arch.c
hs_gyro_b2.srcs += peripherals/ms2100.c  $(SRC_ARCH)/peripherals/ms2100_arch.c

#
# Spits every samples of one axis of gyro on IMU crista
#
hs_gyro_crista.ARCHDIR = $(ARCH)
hs_gyro_crista.CFLAGS  =  -I$(SRC_LISA) -I$(ARCH) -I$(SRC_BOOZ) -I$(SRC_BOOZ_ARCH) -DPERIPHERALS_AUTO_INIT
hs_gyro_crista.CFLAGS +=  -DBOARD_CONFIG=$(BOARD_CFG)
hs_gyro_crista.srcs += lisa/test/hs_gyro.c \
                    $(SRC_ARCH)/stm32_exceptions.c   \
                    $(SRC_ARCH)/stm32_vector_table.c

hs_gyro_crista.CFLAGS += -DUSE_LED
hs_gyro_crista.srcs += $(SRC_ARCH)/led_hw.c

hs_gyro_crista.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
hs_gyro_crista.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))'
hs_gyro_crista.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c

hs_gyro_crista.CFLAGS += -DUSE_UART2 -DUART2_BAUD=B57600
hs_gyro_crista.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c

hs_gyro_crista.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart2
hs_gyro_crista.srcs += downlink.c pprz_transport.c

hs_gyro_crista.srcs += math/pprz_trig_int.c

hs_gyro_crista.CFLAGS += -DIMU_TYPE_H=\"imu/imu_crista.h\" -DIMU_OVERRIDE_CHANNELS
hs_gyro_crista.srcs += $(SRC_SUBSYSTEMS)/imu.c             \
                        $(SRC_SUBSYSTEMS)/imu/imu_crista.c \
                        $(SRC_SUBSYSTEMS)/imu/arch/$(ARCH)/imu_crista_arch.c
hs_gyro_crista.CFLAGS += -DUSE_DMA1_C4_IRQ

hs_gyro_crista.CFLAGS += -DMEASURED_SENSOR=gyro_unscaled.p -DMEASURED_SENSOR_NB=0
