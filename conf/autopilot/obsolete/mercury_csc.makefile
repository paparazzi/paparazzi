#
# $Id$
#
# Copyright (C) 2009 Antoine Drouin
#
# This file is part of paparazzi.
#
# paparazzi is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
# any later version.
#
# paparazzi is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with paparazzi; see the file COPYING.  If not, write to
# the Free Software Foundation, 59 Temple Place - Suite 330,
# Boston, MA 02111-1307, USA.
#
#

ARCH=lpc21

FLASH_MODE = ISP
LPC21ISP_PORT = /dev/ttyUSB0

LPC21ISP_BAUD = 38400
LPC21ISP_XTAL = 12000


LPC21ISP_CONTROL = -control

LDSCRIPT=$(SRC_ARCH)/LPC2129-ROM.ld

BOARD_CFG = \"csc_board_v1_0.h\"



SRC_CSC=csc
SRC_CSC_ARCH=$(SRC_CSC)/$(ARCH)

ap.ARCHDIR = $(ARCH)

ap.CFLAGS += -I$(SRC_CSC) -I$(SRC_CSC_ARCH)
ap.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
ap.srcs += $(SRC_CSC)/mercury_csc_main.c
ap.CFLAGS += -DUSE_LED -DTIME_LED=1

ap.CFLAGS += -DCSC_BOARD_ID=$(CSC_ID)

ap.CFLAGS += -DUSE_PWM_INPUT
ap.CFLAGS += -DUSE_PWM_INPUT1
ap.CFLAGS += -DUSE_PWM_INPUT2
ap.CFLAGS += -DUSE_VANE_SENSOR
# ap.CFLAGS += -DUSE_BAT_MONITOR

ap.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))' -DTIMER0_VIC_SLOT=1
ap.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c

#ap.srcs += $(SRC_ARCH)/adc_hw.c
#ap.CFLAGS += -DADC -DUSE_AD0 -DUSE_AD0_0 -DUSE_AD0_1


#ap.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600 -DUART1_VIC_SLOT=6
#ap.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport \
#					  -DDOWNLINK_DEVICE=Uart0
#ap.srcs += downlink.c pprz_transport.c


ap.CFLAGS += -DAP_LINK_CAN -DCAN_LED=2
ap.CFLAGS += -DUSE_CAN1 #-DCAN1_BTR=CANBitrate125k_3MHz
ap.CFLAGS +=  -DCAN1_VIC_SLOT=3 -DCAN1_ERR_VIC_SLOT=7
ap.srcs += $(SRC_CSC)/csc_can.c
ap.CFLAGS += -DUSE_UART0 -DUART0_BAUD=B57600 -DUART0_VIC_SLOT=5
#ap.CFLAGS += -DUSE_CAN2 -DCAN2_BTR=CANBitrate125k_2MHz -DCAN2_VIC_SLOT=4

ap.CFLAGS += -DACTUATORS=\"servos_direct_hw.h\"
ap.srcs += actuators.c
ap.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c
ap.srcs += $(SRC_ARCH)/servos_direct_hw.c
ap.srcs += $(SRC_CSC)/csc_servos.c
ap.srcs += $(SRC_CSC)/csc_bat_monitor.c
ap.srcs += $(SRC_CSC)/csc_ap_link.c
ap.srcs += $(SRC_CSC)/csc_adc.c

ap.CFLAGS += -DERROR_LED=4

#
#
# test uart
#
test_uart.ARCHDIR = $(ARCH)


test_uart.CFLAGS += -I$(SRC_CSC)
test_uart.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_uart.srcs += $(SRC_CSC)/csc_test_uart.c
test_uart.CFLAGS += -DUSE_LED

# -DTIME_LED=1
test_uart.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))' -DTIMER0_VIC_SLOT=1
test_uart.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c

test_uart.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c

test_uart.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600 -DUART1_VIC_SLOT=6
test_uart.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport \
					  -DDOWNLINK_DEVICE=Uart1
test_uart.srcs += downlink.c pprz_transport.c


#
# TEST CAN1
#

test_can1.ARCHDIR = $(ARCH)


test_can1.CFLAGS += -I$(SRC_CSC)
test_can1.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_can1.srcs += $(SRC_CSC)/test_can1.c
test_can1.CFLAGS += -DUSE_LED

# -DTIME_LED=1
test_can1.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))' -DTIMER0_VIC_SLOT=1
test_can1.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c

test_can1.CFLAGS += -DUSE_CAN1 -DCAN1_BTR=CANBitrate125k_2MHz
test_can1.CFLAGS +=  -DCAN1_VIC_SLOT=3 -DCAN1_ERR_VIC_SLOT=7
test_can1.srcs += $(SRC_CSC)/csc_can.c
test_can1.CFLAGS += -DCSC_BOARD_ID=0
