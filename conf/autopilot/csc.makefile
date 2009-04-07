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
LPC21ISP_PORT = /dev/ttyUSB1
LPC21ISP_BAUD = 115200
LPC21ISP_XTAL = 14746
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
main.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))'
# -DTIME_LED=1
main.CFLAGS += -DLED
main.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c

