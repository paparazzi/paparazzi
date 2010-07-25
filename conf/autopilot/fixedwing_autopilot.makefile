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
ap.CFLAGS += -DUSE_LED
ifeq ($(ARCHI), stm32) 
ap.srcs += $(SRC_ARCH)/led_hw.c
endif

#
# Systime
#
ap.CFLAGS += -DUSE_SYS_TIME
ap.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c
ap.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./60.))'
ifeq ($(ARCHI), stm32) 
ap.CFLAGS += -DSYS_TIME_LED=1
endif

#
# FlyByWire
#
ap.CFLAGS += -DFBW
ap.srcs += $(SRC_FIXEDWING)/main_fbw.c

#
# Telemetry/Datalink
#
# ap.srcs += $(SRC_ARCH)/uart_hw.c
# ap.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport
# ap.srcs += $(SRC_FIXEDWING)/booz2_telemetry.c \
# 	   downlink.c \
#            pprz_transport.c
# ap.CFLAGS += -DDATALINK=PPRZ
# ap.srcs += $(SRC_FIXEDWING)/booz2_datalink.c

# ifeq ($(ARCHI), arm7)
# ap.CFLAGS += -DUSE_UART1  -DUART1_VIC_SLOT=6  -DUART1_BAUD=MODEM_BAUD
# ap.CFLAGS += -DDOWNLINK_DEVICE=Uart1 
# ap.CFLAGS += -DPPRZ_UART=Uart1
# else ifeq ($(ARCHI), stm32) 
# ap.CFLAGS += -DUSE_UART2 -DUART2_BAUD=MODEM_BAUD
# ap.CFLAGS += -DDOWNLINK_DEVICE=Uart2 
# ap.CFLAGS += -DPPRZ_UART=Uart2
# endif


# ap.srcs += $(SRC_FIXEDWING)/booz2_commands.c

#
# Radio control choice
#
# include booz2_radio_control_ppm.makefile
# or
# include booz2_radio_control_spektrum.makefile
#

#
# Actuator choice
#
# include booz2_actuators_buss.makefile
# or
# include booz2_actuators_asctec.makefile
#

#
# IMU choice
#
# include booz2_imu_b2v1.makefile
# or
# include booz2_imu_b2v1_1.makefile
# or
# include booz2_imu_crista.makefile
#


# ifeq ($(ARCHI), arm7)
# ap.CFLAGS += -DBOOZ2_ANALOG_BARO_LED=2 -DBOOZ2_ANALOG_BARO_PERIOD='SYS_TICS_OF_SEC((1./100.))'
# ap.srcs += $(SRC_FIXEDWING)/booz2_analog_baro.c
# 
# ap.CFLAGS += -DBOOZ2_ANALOG_BATTERY_PERIOD='SYS_TICS_OF_SEC((1./10.))'
# ap.srcs += $(SRC_FIXEDWING)/booz2_battery.c
# 
# ap.CFLAGS += -DADC0_VIC_SLOT=2
# ap.CFLAGS += -DADC1_VIC_SLOT=3
# ap.srcs += $(SRC_FIXEDWING)/booz2_analog.c \
#            $(SRC_FIXEDWING_ARCH)/booz2_analog_hw.c
# else ifeq ($(ARCHI), stm32) 
# ap.srcs += lisa/lisa_analog_plug.c
# endif


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

# ap.srcs += $(SRC_FIXEDWING)/booz2_autopilot.c

# ap.srcs += math/pprz_trig_int.c
# ap.srcs += $(SRC_FIXEDWING)/booz_stabilization.c
# ap.srcs += $(SRC_FIXEDWING)/stabilization/booz_stabilization_rate.c


# ap.CFLAGS += -DSTABILISATION_ATTITUDE_TYPE_INT
# ap.CFLAGS += -DSTABILISATION_ATTITUDE_H=\"stabilization/booz_stabilization_attitude_int.h\"
# ap.CFLAGS += -DSTABILISATION_ATTITUDE_REF_H=\"stabilization/booz_stabilization_attitude_ref_euler_int.h\"
# ap.srcs += $(SRC_FIXEDWING)/stabilization/booz_stabilization_attitude_ref_euler_int.c
# ap.srcs += $(SRC_FIXEDWING)/stabilization/booz_stabilization_attitude_euler_int.c

# ap.CFLAGS += -DUSE_NAVIGATION
# ap.srcs += $(SRC_FIXEDWING)/guidance/booz2_guidance_h.c
# ap.srcs += $(SRC_FIXEDWING)/guidance/booz2_guidance_v.c

# ap.srcs += $(SRC_FIXEDWING)/booz2_ins.c
# ap.srcs += math/pprz_geodetic_int.c math/pprz_geodetic_float.c math/pprz_geodetic_double.c

#
# INS choice
#
# include booz2_ins_hff.makefile
# or
# nothing
#

#  vertical filter float version
# ap.srcs += $(SRC_FIXEDWING)/ins/booz2_vf_float.c
# ap.CFLAGS += -DUSE_VFF -DDT_VFILTER="(1./512.)"

# ap.srcs += $(SRC_FIXEDWING)/booz2_navigation.c


#
# FMS  choice
#
# include booz2_fms_test_signal.makefile
# or
# include booz2_fms_datalink.makefile
# or 
# nothing
#
