#
# $Id$
#
# Copyright (C) 2009 Antoine Drouin, Allen H. Ibara
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

ARCHI=arm7

FLASH_MODE = ISP
LPC21ISP_PORT = /dev/ttyUSB0

LPC21ISP_BAUD = 38400
LPC21ISP_XTAL = 12000


LPC21ISP_CONTROL = -control

LDSCRIPT=$(SRC_ARCH)/LPC2129-ROM.ld

BOARD_CFG = \"csc_board_v1_0.h\"

PERIODIC_FREQ = 40

SRC_CSC=csc

ap.ARCHDIR = $(ARCHI)

ap.CFLAGS += -I$(SRC_CSC)
ap.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
ap.srcs += $(SRC_CSC)/csc_ap_main.c
ap.CFLAGS += -DUSE_LED -DTIME_LED=1

ap.CFLAGS += -DCSC_BOARD_ID=$(CSC_ID)

ap.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./40.))' -DTIMER0_VIC_SLOT=1
ap.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c

ap.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c
ap.srcs += $(SRC_ARCH)/adc_hw.c
ap.CFLAGS += -DADC -DUSE_AD0 -DUSE_AD0_0 -DUSE_AD0_1


ap.CFLAGS += -DUSE_UART0 -DUART0_BAUD=B230400 -DUART0_VIC_SLOT=5
ap.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600 -DUART1_VIC_SLOT=6
ap.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport \
	                  -DDOWNLINK_DEVICE=Uart1
ap.srcs += downlink.c pprz_transport.c $(SRC_CSC)/csc_telemetry.c
ap.CFLAGS += -DDATALINK=PPRZ -DPPRZ_UART=Uart1
ap.srcs += $(SRC_CSC)/csc_datalink.c

ap.srcs += $(SRC_CSC)/csc_xsens.c
ap.CFLAGS += -DXSENS1_LINK=Uart0

ap.CFLAGS += -DAP_LINK_CAN -DCAN_LED=2
ap.CFLAGS += -DUSE_CAN1 -DCAN1_BTR=CANBitrate125k_2MHz
ap.CFLAGS +=  -DCAN1_VIC_SLOT=3 -DCAN1_ERR_VIC_SLOT=7
ap.srcs += $(SRC_CSC)/csc_can.c
#ap.CFLAGS += -DUSE_CAN2 -DCAN2_BTR=CANBitrate125k_2MHz -DCAN2_VIC_SLOT=4

ap.srcs += $(SRC_CSC)/csc_ap_link.c

#ap.srcs += $(SRC_CSC)/csc_servos.c
#ap.CFLAGS += -DPWM_SERVO_5 -DPWM_SERVO_0
ap.srcs += $(SRC_ARCH)/servos_csc.c

ap.srcs += $(SRC_CSC)/csc_adc.c
ap.CFLAGS += -DACTUATORS=\"servos_csc.h\"
ap.srcs += commands.c actuators.c

ap.CFLAGS += -DRADIO_CONTROL
ap.srcs += radio_control.c $(SRC_ARCH)/ppm_hw.c

ap.srcs += $(SRC_CSC)/csc_autopilot.c

ap.CFLAGS += -DERROR_LED=4

#
#
# test uart
#
test_uart.ARCHDIR = $(ARCHI)


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

