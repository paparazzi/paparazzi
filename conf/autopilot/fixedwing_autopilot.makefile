#
# $Id: booz2_autopilot.makefile 4827 2010-04-21 08:02:18Z poine $
#  
# Copyright (C) 2008 Antoine Drouin
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

ap.ARCHDIR = $(ARCHI)
# this is supposedly ignored by the stm32 makefile
ap.ARCH = arm7tdmi
ap.TARGET = ap
ap.TARGETDIR = ap


ap.CFLAGS += $(FIXEDWING_INC)
ap.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
ap.srcs    = $(SRC_FIXEDWING)/main.c

ifeq ($(ARCHI), stm32) 
ap.srcs += lisa/plug_sys.c
endif

#
# Interrupts
#
ifeq ($(ARCHI), arm7)
ap.srcs += $(SRC_ARCH)/armVIC.c
else ifeq ($(ARCHI), stm32) 
ap.srcs += $(SRC_ARCH)/stm32_exceptions.c
ap.srcs += $(SRC_ARCH)/stm32_vector_table.c
endif

#
# LEDs
#
ap.CFLAGS += -DUSE_LED -DLED -DTIME_LED=1
ifeq ($(ARCHI), stm32) 
ap.srcs += $(SRC_ARCH)/led_hw.c
endif

#
# Systime
#
ap.CFLAGS += -DUSE_SYS_TIME
ap.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c
#ap.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
ap.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./60.))'
ifeq ($(ARCHI), stm32) 
ap.CFLAGS += -DSYS_TIME_LED=1  -DPERIPHERALS_AUTO_INIT
endif


#
# FlyByWire Main
#
ap.CFLAGS += -DFBW
ap.srcs += $(SRC_FIXEDWING)/main_fbw.c

#
# AutoPilot Main
#
ap.CFLAGS += -DAP
ap.srcs += $(SRC_FIXEDWING)/main_ap.c
ap.srcs += $(SRC_FIXEDWING)/estimator.c
ap.CFLAGS += -DNAV
ap.srcs += $(SRC_FIXEDWING)/nav.c $(SRC_FIXEDWING)/fw_h_ctl.c $(SRC_FIXEDWING)/fw_v_ctl.c
ap.srcs += $(SRC_FIXEDWING)/nav_survey_rectangle.c $(SRC_FIXEDWING)/nav_line.c

#
# InterMCU & Commands
#

ap.CFLAGS += -DINTER_MCU
ap.srcs += $(SRC_FIXEDWING)/inter_mcu.c 
ap.srcs += $(SRC_FIXEDWING)/commands.c

#
# UARTS
#
ap.srcs += $(SRC_ARCH)/uart_hw.c


ifeq ($(ARCHI), arm7)
ap.CFLAGS += -DADC
ap.srcs += $(SRC_ARCH)/adc_hw.c
else ifeq ($(ARCHI), stm32) 
ap.srcs += lisa/lisa_analog_plug.c
endif


#
# GPS choice
# 
# include booz2_gps.makefile
# or
# nothing
#


#
# AHRS choice
#
# include booz2_ahrs_cmpl.makefile
# or
# include booz2_ahrs_lkf.makefile
#


#
# INS choice
#
# include booz2_ins_hff.makefile
# or
# nothing
#


#
# FMS  choice
#
# include booz2_fms_test_signal.makefile
# or
# include booz2_fms_datalink.makefile
# or 
# nothing
#
