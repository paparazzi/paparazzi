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
sim.srcs += $(BOOZ_PRIV)/booz_a_la_mkk.c

#
# Controller MCU
#
ctl.ARCHDIR = $(ARCHI)
ctl.ARCH = arm7tdmi
ctl.TARGET = ctl
ctl.TARGETDIR = ctl

ctl.CFLAGS += -DCONFIG=\"booz2_board.h\" -I$(BOOZ_ARCH) -I$(BOOZ_PRIV) -I$(BOOZ_PRIV_ARCH)
ctl.srcs += $(BOOZ_PRIV)/booz2_main.c
ctl.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./500.))' -DTIME_LED=1
ctl.CFLAGS += -DLED
ctl.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c

ctl.CFLAGS += -DUSE_UART0 -DUART0_BAUD=B57600
ctl.srcs += $(SRC_ARCH)/uart_hw.c

ctl.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart0 
ctl.srcs += $(BOOZ_PRIV)/booz2_telemetry.c downlink.c pprz_transport.c

ctl.CFLAGS += -DDATALINK=PPRZ -DPPRZ_UART=Uart0
ctl.srcs += $(BOOZ)/booz_datalink.c

ctl.srcs += commands.c

ctl.CFLAGS += -DRADIO_CONTROL -DRADIO_CONTROL_TYPE=RC_FUTABA -DRC_LED=4
ctl.srcs += radio_control.c $(SRC_ARCH)/ppm_hw.c

ctl.CFLAGS += -DACTUATORS=\"actuators_buss_twi_blmc_hw.h\" -DUSE_BUSS_TWI_BLMC
ctl.srcs += $(BOOZ_ARCH)/actuators_buss_twi_blmc_hw.c actuators.c
ctl.CFLAGS += -DI2C_SCLL=150 -DI2C_SCLH=150 -DI2C_VIC_SLOT=10
ctl.srcs += i2c.c $(SRC_ARCH)/i2c_hw.c

ctl.srcs += $(BOOZ_PRIV)/booz2_imu_v3.c $(BOOZ_PRIV_ARCH)/booz2_imu_v3_hw.c
ctl.CFLAGS += -DFLOAT_T=float
ctl.srcs += $(BOOZ_PRIV)/booz2_imu.c

ctl.CFLAGS += -DBOOZ2_ANALOG_BARO_LED=5 -DBOOZ2_ANALOG_BARO_PERIOD='SYS_TICS_OF_SEC((1./100.))'
ctl.srcs += $(BOOZ_PRIV)/booz2_analog_baro.c

ctl.srcs += $(BOOZ_PRIV)/booz2_autopilot.c
ctl.srcs += $(BOOZ_PRIV)/booz_stabilization_euler_int.c
ctl.srcs += $(BOOZ_PRIV)/booz_supervision_int.c


#
# IMU V3 MCU
#

imu.ARCHDIR = $(ARCHI)
imu.ARCH = arm7tdmi
imu.TARGET = imu
imu.TARGETDIR = imu

imu.CFLAGS += -DCONFIG=\"pprz_imu.h\" -I$(BOOZ) -I$(BOOZ_ARCH) -I$(BOOZ_PRIV) -I$(BOOZ_PRIV_ARCH)
imu.srcs += $(BOOZ_PRIV)/imu_v3_main.c
imu.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./500.))' -DTIME_LED=1
imu.CFLAGS += -DLED
imu.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c

imu.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600
imu.srcs += $(SRC_ARCH)/uart_hw.c

imu.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart1 
imu.srcs += downlink.c pprz_transport.c $(BOOZ_PRIV)/imu_v3_telemetry.c

imu.srcs += $(BOOZ_PRIV)/imu_v3_sensors.c $(BOOZ_PRIV_ARCH)/imu_v3_sensors_hw.c
imu.CFLAGS += -DADC -DUSE_AD0 -DUSE_AD0_1 -DUSE_AD0_2 -DUSE_AD0_3
imu.srcs += $(SRC_ARCH)/adc_hw.c
imu.srcs += max1167.c  $(SRC_ARCH)/max1167_hw.c 
imu.srcs += micromag.c $(SRC_ARCH)/micromag_hw.c

imu.srcs += $(BOOZ_PRIV)/imu_v3_client_link.c $(BOOZ_PRIV_ARCH)/imu_v3_client_link_hw.c


# test leds
test_led.ARCHDIR = $(ARCHI)
test_led.ARCH = arm7tdmi
test_led.TARGET = test_led
test_led.TARGETDIR = test_led

test_led.CFLAGS += -DCONFIG=\"booz2_board.h\" -I$(BOOZ_PRIV_ARCH)
test_led.srcs += $(BOOZ_PRIV)/test_led.c
test_led.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./1000.))' -DTIME_LED=1
test_led.CFLAGS += -DLED
test_led.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c


# test modem
test_modem.ARCHDIR = $(ARCHI)
test_modem.ARCH = arm7tdmi
test_modem.TARGET = test_modem
test_modem.TARGETDIR = test_modem

test_modem.CFLAGS += -DCONFIG=\"tiny_1_1.h\" -I$(BOOZ_PRIV_ARCH)
test_modem.srcs += $(BOOZ_PRIV)/test_modem.c
test_modem.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./1000.))' -DTIME_LED=1
test_modem.CFLAGS += -DLED
test_modem.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c

test_modem.CFLAGS += -DUSE_UART0 -DUART0_BAUD=B9600
test_modem.srcs += $(SRC_ARCH)/uart_hw.c

test_modem.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart0 
test_modem.srcs += downlink.c pprz_transport.c

test_modem.CFLAGS += -DBOOZ_ANALOG_BARO_LED=2 -DBOOZ_ANALOG_BARO_PERIOD='SYS_TICS_OF_SEC((1./100.))'
test_modem.srcs += $(BOOZ_PRIV)/booz_analog_baro.c


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