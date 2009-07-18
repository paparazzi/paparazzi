#
# $Id$
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

ARCHI=arm7

FLASH_MODE = IAP
BOARD_CFG = \"booz2_board_v1_0.h\"

SRC_BOOZ=booz
SRC_BOOZ_ARCH=$(SRC_BOOZ)/arm7

ap.ARCHDIR = $(ARCHI)
ap.ARCH = arm7tdmi
ap.TARGET = ap
ap.TARGETDIR = ap


ap.CFLAGS += -I$(SRC_BOOZ) -I$(SRC_BOOZ_ARCH)
ap.CFLAGS += -DCONFIG=$(BOARD_CFG)
ap.srcs += $(SRC_BOOZ)/booz2_main.c

ap.CFLAGS += -DPERIPHERALS_AUTO_INIT

ap.CFLAGS += -DUSE_LED

ap.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c
ap.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))'
# -DTIME_LED=1

ap.CFLAGS += -DUSE_UART1  -DUART1_VIC_SLOT=6  -DUART1_BAUD=MODEM_BAUD
ap.srcs += $(SRC_ARCH)/uart_hw.c

ap.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart1 
ap.srcs += $(SRC_BOOZ)/booz2_telemetry.c \
	   downlink.c \
           pprz_transport.c

ap.CFLAGS += -DDATALINK=PPRZ -DPPRZ_UART=Uart1
ap.srcs += $(SRC_BOOZ)/booz2_datalink.c

ap.srcs += $(SRC_BOOZ)/booz2_commands.c

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
# include booz2_imu_crista.makefile
#



ap.CFLAGS += -DBOOZ2_ANALOG_BARO_LED=2 -DBOOZ2_ANALOG_BARO_PERIOD='SYS_TICS_OF_SEC((1./100.))'
ap.srcs += $(SRC_BOOZ)/booz2_analog_baro.c

ap.CFLAGS += -DBOOZ2_ANALOG_BATTERY_PERIOD='SYS_TICS_OF_SEC((1./10.))'
ap.srcs += $(SRC_BOOZ)/booz2_battery.c

ap.CFLAGS += -DADC0_VIC_SLOT=2
ap.CFLAGS += -DADC1_VIC_SLOT=3
ap.srcs += $(SRC_BOOZ)/booz2_analog.c $(SRC_BOOZ_ARCH)/booz2_analog_hw.c

ap.srcs += $(SRC_BOOZ)/booz2_gps.c
ap.CFLAGS += -DUSE_UART0 -DUART0_BAUD=B38400 -DUART0_VIC_SLOT=5
ap.CFLAGS += -DUSE_GPS -DGPS_LINK=Uart0 -DGPS_LED=4



ap.srcs += $(SRC_BOOZ)/booz2_autopilot.c

ap.CFLAGS += -DAHRS_ALIGNER_LED=3
ap.srcs += $(SRC_BOOZ)/booz_ahrs_aligner.c
ap.srcs += $(SRC_BOOZ)/booz2_filter_attitude_cmpl_euler.c
ap.srcs += $(SRC_BOOZ)/booz_trig_int.c
ap.srcs += $(SRC_BOOZ)/booz2_stabilization.c
ap.srcs += $(SRC_BOOZ)/booz2_stabilization_rate.c
ap.srcs += $(SRC_BOOZ)/booz2_stabilization_attitude.c

ap.srcs += $(SRC_BOOZ)/booz2_guidance_h.c
ap.srcs += $(SRC_BOOZ)/booz2_guidance_v.c
ap.srcs += $(SRC_BOOZ)/booz2_ins.c
ap.srcs += pprz_geodetic_int.c pprz_geodetic_float.c
ap.srcs += $(SRC_BOOZ)/booz2_hf_float.c
#  vertical filter float version
ap.srcs += $(SRC_BOOZ)/booz2_vf_float.c
ap.CFLAGS += -DUSE_VFF -DDT_VFILTER="(1./512.)" -DFLOAT_T=float

ap.srcs += $(SRC_BOOZ)/booz2_navigation.c



ap.CFLAGS += -DHS_YAW

ap.srcs += $(SRC_BOOZ)/booz2_fms.c

#ap.CFLAGS += -DBOOZ2_FMS_TYPE=BOOZ2_FMS_TYPE_DATALINK
#ap.srcs += $(SRC_BOOZ)/booz2_fms_datalink.c

ap.CFLAGS += -DBOOZ2_FMS_TYPE=BOOZ2_FMS_TYPE_TEST_SIGNAL
ap.srcs += $(SRC_BOOZ)/booz2_fms_test_signal.c
