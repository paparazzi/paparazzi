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

ARCHI=arm7

FLASH_MODE = ISP
LPC21ISP_PORT = /dev/ttyUSB0

#LPC21ISP_BAUD = 115200
#LPC21ISP_XTAL = 14746
LPC21ISP_BAUD = 38400
LPC21ISP_XTAL = 12000


LPC21ISP_CONTROL = -control

LDSCRIPT=$(SRC_ARCH)/LPC2129-ROM.ld

BOARD_CFG = \"csc_board_v1_0.h\"



SRC_CSC=csc

main.ARCHDIR = $(ARCHI)
main.ARCH = arm7tdmi
main.TARGET = main
main.TARGETDIR = main


main.CFLAGS += -I$(SRC_CSC)
main.CFLAGS += -DCONFIG=$(BOARD_CFG)
main.srcs += $(SRC_CSC)/csc_main.c
main.CFLAGS += -DLED

main.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))' -DTIME_LED=1 -DTIMER0_VIC_SLOT=1
main.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c

main.CFLAGS += -DUSE_UART0 -DUART0_BAUD=B57600 -DUART0_VIC_SLOT=5
main.srcs += $(SRC_ARCH)/uart_hw.c

main.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart0
main.srcs += downlink.c pprz_transport.c

main.CFLAGS += -DUSE_CAN1 -DCAN1_BTR=CANBitrate125k_12MHz -DCAN1_VIC_SLOT=3
main.srcs += $(SRC_CSC)/csc_can.c

main.CFLAGS += -DUSE_CAN2 -DCAN2_BTR=CANBitrate125k_12MHz -DCAN2_VIC_SLOT=4

main.srcs += $(SRC_CSC)/csc_servos.c

