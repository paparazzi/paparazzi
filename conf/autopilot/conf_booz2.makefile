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

BOOZ=booz
BOOZ_PRIV=booz_priv
BOOZ_PRIV_ARCH=booz_priv/arm7
BOOZ_ARCH=booz/arm7

#
# SITL Simulator
#

SIM_TYPE = BOOZ

sim.ARCHDIR = $(ARCHI)
sim.ARCH = sitl
sim.TARGET = sim
sim.TARGETDIR = sim


sim.CFLAGS +=  `pkg-config glib-2.0 --cflags` -I /usr/include/meschach
sim.LDFLAGS += `pkg-config glib-2.0 --libs` -lm -lmeschach -lpcre -lglibivy

sim.CFLAGS += -I$(BOOZ) -I../simulator -DFLOAT_T=float
sim.CFLAGS += -DBSM_PARAMS=\"booz_sensors_model_params_booz2.h\"

sim.srcs = $(BOOZ_PRIV)/main_sim.c                   \
	   $(SIMDIR)/booz_flight_model.c             \
           $(SIMDIR)/booz_flight_model_utils.c       \
           $(SIMDIR)/booz_sensors_model.c            \
	   $(SIMDIR)/booz_sensors_model_utils.c      \
           $(SIMDIR)/booz_sensors_model_accel.c      \
           $(SIMDIR)/booz_sensors_model_gyro.c       \
           $(SIMDIR)/booz_sensors_model_mag.c        \
           $(SIMDIR)/booz_sensors_model_rangemeter.c \
           $(SIMDIR)/booz_sensors_model_baro.c       \
           $(SIMDIR)/booz_sensors_model_gps.c        \
#           $(SIMDIR)/booz_wind_model.c               \

sim.CFLAGS += -DSITL
sim.CFLAGS += -DBOOZ_CONTROLLER_MCU
sim.CFLAGS += -DCONFIG=\"conf_booz.h\"

sim.srcs += $(BOOZ_PRIV)/booz_imu_int.c
sim.srcs += $(BOOZ_PRIV)/booz_estimator_int.c
sim.srcs += $(BOOZ_PRIV)/booz_nav_filter_int.c
sim.srcs += $(BOOZ_PRIV)/booz_cmp_flt_quat_int.c
sim.srcs += $(BOOZ_PRIV)/booz_guidance_int.c
sim.srcs += $(BOOZ_PRIV)/booz_stabilization_int.c
sim.srcs += $(BOOZ_PRIV)/booz_supervision_int.c



# test imu
test_imu.ARCHDIR = $(ARCHI)
test_imu.ARCH = arm7tdmi
test_imu.TARGET = test_imu
test_imu.TARGETDIR = test_imu

test_imu.CFLAGS += -DCONFIG=\"tiny_1_1.h\" -I$(BOOZ_PRIV_ARCH)
test_imu.srcs += $(BOOZ_PRIV)/test_imu.c
test_imu.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./1000.))' -DTIME_LED=1
test_imu.CFLAGS += -DLED
test_imu.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c

test_imu.CFLAGS += -DUSE_UART0 -DUART0_BAUD=B57600
test_imu.srcs += $(SRC_ARCH)/uart_hw.c

test_imu.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart0 
test_imu.srcs += downlink.c pprz_transport.c

test_imu.CFLAGS += -DFLOAT_T=float
test_imu.srcs += $(BOOZ_PRIV)/booz_imu_int.c $(BOOZ_PRIV_ARCH)/booz_imu_int_hw.c