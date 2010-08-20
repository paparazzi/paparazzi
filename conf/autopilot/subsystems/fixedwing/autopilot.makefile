#
# $Id: booz2_autopilot.makefile 4827 2010-04-21 08:02:18Z poine $
#
# Copyright (C) 2008 Antoine Drouin
#
# This file is part of paparazzi.
#tin
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

# All targets need the board config
$(TARGET).CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)

$(TARGET).CFLAGS += -DWIND_INFO -DTRAFFIC_INFO

sim.CFLAGS += -DSITL -DAP -DFBW -DRADIO_CONTROL -DINTER_MCU -DDOWNLINK -DDOWNLINK_TRANSPORT=IvyTransport -DINFRARED -DLED
sim.srcs += latlong.c radio_control.c downlink.c commands.c gps.c inter_mcu.c infrared.c estimator.c sys_time.c main_fbw.c main_ap.c datalink.c $(SRC_ARCH)/ppm_hw.c $(SRC_ARCH)/sim_gps.c $(SRC_ARCH)/sim_ir.c $(SRC_ARCH)/sim_ap.c $(SRC_ARCH)/ivy_transport.c $(SRC_ARCH)/sim_adc_generic.c $(SRC_ARCH)/led_hw.c



ap.CFLAGS += $(FIXEDWING_INC)
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


