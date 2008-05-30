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

TL=coaxial
BOOZ=booz
BOOZ_ARCH=booz/arm7

#
# filter CPU
#

flt.ARCHDIR = $(ARCHI)
flt.ARCH = arm7tdmi
flt.TARGET = flt
flt.TARGETDIR = flt

flt.CFLAGS += -I$(TL) -I$(BOOZ) -I$(BOOZ_ARCH)

flt.CFLAGS += -DBOOZ_FILTER_MCU -DCONFIG=\"pprz_imu.h\"
flt.srcs = $(BOOZ)/booz_filter_main.c

flt.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./250.))'
flt.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c

flt.CFLAGS += -DLED

flt.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600
flt.srcs += $(SRC_ARCH)/uart_hw.c

flt.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart1 
flt.srcs += downlink.c pprz_transport.c $(BOOZ)/booz_filter_telemetry.c

flt.CFLAGS += -DADC -DUSE_AD0 -DUSE_AD0_1 -DUSE_AD0_2 -DUSE_AD0_3 -DUSE_AD0_4
flt.srcs += $(SRC_ARCH)/adc_hw.c

flt.srcs += $(BOOZ)/booz_imu.c $(BOOZ_ARCH)/booz_imu_hw.c
flt.srcs += max1167.c  $(SRC_ARCH)/max1167_hw.c 
flt.srcs += micromag.c $(SRC_ARCH)/micromag_hw.c
flt.srcs += scp1000.c  $(SRC_ARCH)/scp1000_hw.c

flt.srcs += $(BOOZ)/booz_still_detection.c

flt.CFLAGS += -DFLOAT_T=float

flt.CFLAGS += -DBOOZ_AHRS_TYPE=BOOZ_AHRS_MULTITILT
flt.srcs += $(BOOZ)/ahrs_multitilt.c $(BOOZ)/booz_ahrs.c

#flt.CFLAGS += -DBOOZ_AHRS_TYPE=BOOZ_AHRS_QUATERNION
#flt.srcs +=  $(BOOZ)/ahrs_quat_fast_ekf.c booz_ahrs.c

#flt.CFLAGS += -DBOOZ_AHRS_TYPE=BOOZ_AHRS_EULER
#flt.srcs +=  $(BOOZ)/ahrs_euler_fast_ekf.c booz_ahrs.c

#flt.CFLAGS += -DBOOZ_AHRS_TYPE=BOOZ_AHRS_COMP_FILTER
#flt.srcs +=  $(BOOZ)/ahrs_comp_filter.c booz_ahrs.c

flt.srcs += $(BOOZ)/booz_ins.c
flt.CFLAGS += -DDT_VFILTER="(1./250.)"
flt.srcs += $(TL)/tl_vfilter.c

flt.CFLAGS += -DUSE_UART0 -DUART0_BAUD=B38400
flt.CFLAGS += -DGPS -DUBX -DGPS_LINK=Uart0 -DDOWNLINK_GPS_DEVICE=Uart1 -DGPS_BAUD=38400
flt.srcs += gps_ubx.c gps.c latlong.c

flt.srcs += $(BOOZ)/booz_inter_mcu.c
flt.srcs += $(BOOZ)/booz_link_mcu.c $(BOOZ_ARCH)/booz_link_mcu_hw.c


#
# controller CPU
#

ctl.ARCHDIR = $(ARCHI)
ctl.ARCH = arm7tdmi
ctl.TARGET = ctl
ctl.TARGETDIR = ctl

ctl.CFLAGS += -I$(BOOZ) -I$(BOOZ_ARCH)

ctl.CFLAGS += -DBOOZ_CONTROLLER_MCU -DCONFIG=\"conf_booz.h\"
ctl.srcs = $(BOOZ)/booz_controller_main.c

ctl.CFLAGS += -DLED

ctl.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./250.))' -DTIME_LED=1
ctl.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c

ctl.srcs += $(SRC_ARCH)/armVIC.c

ctl.CFLAGS += -DUSE_ADC_BAT
ctl.srcs += $(BOOZ)/booz_energy.c $(SRC_ARCH)/adc_hw.c

ctl.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600
ctl.srcs += $(SRC_ARCH)/uart_hw.c

ctl.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart1 
ctl.srcs += $(BOOZ)/booz_controller_telemetry.c downlink.c pprz_transport.c 

ctl.CFLAGS += -DDATALINK=PPRZ -DPPRZ_UART=Uart1
ctl.srcs += $(BOOZ)/booz_datalink.c

ctl.CFLAGS += -DACTUATORS=\"actuators_buss_twi_blmc_hw.h\" -DUSE_BUSS_TWI_BLMC
ctl.srcs += $(BOOZ_ARCH)/actuators_buss_twi_blmc_hw.c actuators.c
ctl.CFLAGS += -DI2C_SCLL=150 -DI2C_SCLH=150 -DI2C_VIC_SLOT=10
ctl.srcs += i2c.c $(SRC_ARCH)/i2c_hw.c

ctl.CFLAGS += -DRADIO_CONTROL -DRADIO_CONTROL_TYPE=RC_FUTABA -DRC_LED=4
ctl.srcs += radio_control.c $(SRC_ARCH)/ppm_hw.c

ctl.srcs += $(BOOZ)/booz_inter_mcu.c
ctl.CFLAGS += -DFLOAT_T=float
ctl.srcs += $(BOOZ)/booz_link_mcu.c $(BOOZ_ARCH)/booz_link_mcu_hw.c

ctl.srcs += commands.c
ctl.CFLAGS += -DNAV_VERTICAL
ctl.srcs += $(BOOZ)/booz_estimator.c      \
            $(BOOZ)/booz_control.c        \
            $(BOOZ)/booz_nav.c  	  \
            $(BOOZ)/booz_nav_hover.c  	  \
            $(BOOZ)/booz_autopilot.c

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

sim.CFLAGS += -I$(BOOZ) -I$(TL)
sim.CFLAGS += -DBSM_PARAMS=\"booz_sensors_model_params.h\"

sim.srcs = $(SIMDIR)/main_booz_sim.c                 \
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
           $(SIMDIR)/booz_wind_model.c               \
           $(SIMDIR)/booz_flightgear.c               \
           $(SIMDIR)/booz_joystick.c                 \

sim.CFLAGS += -DSITL
sim.CFLAGS += -DBOOZ_CONTROLLER_MCU
sim.CFLAGS += -DCONFIG=\"conf_booz.h\"

sim.srcs += $(BOOZ)/booz_controller_main.c

sim.srcs += sys_time.c

sim.CFLAGS += -DUSE_ADC_BAT
sim.srcs += $(BOOZ)/booz_energy.c

sim.CFLAGS += -DRADIO_CONTROL 
sim.srcs += radio_control.c \
	    $(SRC_ARCH)/ppm_hw.c

sim.CFLAGS += -DACTUATORS=\"actuators_buss_twi_blmc_hw.h\" 
sim.srcs += actuators.c \
            $(SRC_ARCH)/actuators_buss_twi_blmc_hw.c \
            i2c.c $(SRC_ARCH)/i2c_hw.c

sim.CFLAGS += -DDOWNLINK 
sim.srcs += $(BOOZ)/booz_controller_telemetry.c \
            downlink.c
sim.CFLAGS += -DDOWNLINK_TRANSPORT=IvyTransport
sim.srcs += $(SRC_ARCH)/ivy_transport.c

sim.srcs += $(BOOZ)/booz_inter_mcu.c
sim.srcs += $(BOOZ)/booz_link_mcu.c $(SRC_ARCH)/booz_link_mcu_hw.c

sim.srcs += $(BOOZ)/booz_estimator.c
sim.srcs += $(BOOZ)/booz_control.c
sim.CFLAGS += -DNAV_VERTICAL
sim.srcs += $(BOOZ)/booz_nav.c
sim.srcs += $(BOOZ)/booz_nav_hover.c

sim.srcs += $(BOOZ)/booz_autopilot.c
sim.srcs += commands.c


sim.CFLAGS += -DBOOZ_FILTER_MCU
sim.srcs += $(BOOZ)/booz_filter_main.c

sim.CFLAGS += -DADC_CHANNEL_AX=1 -DADC_CHANNEL_AY=2 -DADC_CHANNEL_AZ=3 -DADC_CHANNEL_BAT=4
sim.srcs  += $(SRC_ARCH)/adc_hw.c


sim.srcs += $(BOOZ)/booz_filter_telemetry.c

sim.srcs += $(BOOZ)/booz_imu.c $(SRC_ARCH)/booz_imu_hw.c
sim.srcs += max1167.c  $(SRC_ARCH)/max1167_hw.c 
sim.srcs += micromag.c $(SRC_ARCH)/micromag_hw.c
sim.srcs += scp1000.c  $(SRC_ARCH)/scp1000_hw.c

sim.srcs += $(BOOZ)/booz_still_detection.c

sim.CFLAGS += -DFLOAT_T=float
#sim.CFLAGS += -DBOOZ_AHRS_TYPE=BOOZ_AHRS_MULTITILT
#sim.srcs += $(BOOZ)/ahrs_multitilt.c $(BOOZ)/booz_ahrs.c

#sim.CFLAGS += -DBOOZ_AHRS_TYPE=BOOZ_AHRS_QUATERNION 
#sim.CFLAGS += -DEKF_UPDATE_DISCRETE
#sim.CFLAGS += -DEKF_PREDICT_ONLY
#sim.srcs += $(BOOZ)/ahrs_quat_fast_ekf.c

sim.CFLAGS += -DBOOZ_AHRS_TYPE=BOOZ_AHRS_COMP_FILTER
sim.srcs += $(BOOZ)/ahrs_comp_filter.c $(BOOZ)/booz_ahrs.c

sim.srcs += gps.c latlong.c $(SRC_ARCH)/gps_hw.c

sim.srcs += $(BOOZ)/booz_ins.c
sim.CFLAGS += -DDT_VFILTER="(1./250.)"
sim.srcs += $(TL)/tl_vfilter.c




##
##
##
## IMU test_gyros
##
##
##
test_gyros.ARCHDIR = $(ARCHI)
test_gyros.ARCH = arm7tdmi
test_gyros.TARGET = test_gyros
test_gyros.TARGETDIR = test_gyros

test_gyros.CFLAGS += -DCONFIG=\"pprz_imu.h\"  -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./250.))'
test_gyros.srcs = $(SRC_ARCH)/booz_test_gyros.c sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c

test_gyros.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600
test_gyros.srcs += $(SRC_ARCH)/uart_hw.c

test_gyros.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart1 
test_gyros.srcs += downlink.c pprz_transport.c

test_gyros.srcs += max1167.c  $(SRC_ARCH)/max1167_hw.c
test_gyros.srcs += micromag.c $(SRC_ARCH)/micromag_hw.c
test_gyros.srcs += scp1000.c  $(SRC_ARCH)/scp1000_hw.c

##
##
##
## IMU test_mm
##
##
##
test_mm.ARCHDIR = $(ARCHI)
test_mm.ARCH = arm7tdmi
test_mm.TARGET = test_mm
test_mm.TARGETDIR = test_mm

test_mm.CFLAGS += -DCONFIG=\"pprz_imu.h\"  -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./20.))'
test_mm.srcs = $(SRC_ARCH)/booz_test_micromag.c sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c

test_mm.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600
test_mm.srcs += $(SRC_ARCH)/uart_hw.c

test_mm.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart1 
test_mm.srcs += downlink.c pprz_transport.c

test_mm.srcs += micromag.c $(SRC_ARCH)/micromag_hw.c

##
##
##
## IMU test_scp
##
##
##
test_scp.ARCHDIR = $(ARCHI)
test_scp.ARCH = arm7tdmi
test_scp.TARGET = test_scp
test_scp.TARGETDIR = test_scp

test_scp.CFLAGS += -DCONFIG=\"pprz_imu.h\"  -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./20.))'
test_scp.srcs = $(SRC_ARCH)/booz_test_scp.c sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c

test_scp.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600
test_scp.srcs += $(SRC_ARCH)/uart_hw.c

test_scp.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart1 
test_scp.srcs += downlink.c pprz_transport.c

test_scp.srcs += scp1000.c $(SRC_ARCH)/scp1000_hw.c


##
##
##
## IMU test_imu
##
##
##
test_imu.ARCHDIR = $(ARCHI)
test_imu.ARCH = arm7tdmi
test_imu.TARGET = test_imu
test_imu.TARGETDIR = test_imu

test_imu.CFLAGS += -DBOOZ_DEBUG
test_imu.srcs += booz_debug.c
test_imu.CFLAGS += -DCONFIG=\"pprz_imu.h\"  -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./250.))'
test_imu.srcs += booz_test_imu.c sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c

test_imu.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600
test_imu.srcs += $(SRC_ARCH)/uart_hw.c

test_imu.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart1 
test_imu.srcs += downlink.c pprz_transport.c

test_imu.srcs += booz_imu.c $(SRC_ARCH)/booz_imu_hw.c
test_imu.srcs += max1167.c  $(SRC_ARCH)/max1167_hw.c 
test_imu.srcs += micromag.c $(SRC_ARCH)/micromag_hw.c
test_imu.srcs += scp1000.c  $(SRC_ARCH)/scp1000_hw.c
test_imu.CFLAGS += -DADC -DUSE_AD0 -DUSE_AD0_1 -DUSE_AD0_2 -DUSE_AD0_3 -DUSE_AD0_4
test_imu.srcs += $(SRC_ARCH)/adc_hw.c


##
##
##
## IMU test_still_detection
##
##
##
test_sd.ARCHDIR = $(ARCHI)
test_sd.ARCH = arm7tdmi
test_sd.TARGET = test_sd
test_sd.TARGETDIR = test_sd

test_sd.CFLAGS += -DBOOZ_DEBUG
test_sd.srcs += booz_debug.c
test_sd.CFLAGS += -DCONFIG=\"pprz_imu.h\"  -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./250.))'
test_sd.srcs += booz_test_still_detection.c 
test_sd.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c

test_sd.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600
test_sd.srcs += $(SRC_ARCH)/uart_hw.c

test_sd.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart1 
test_sd.srcs += downlink.c pprz_transport.c

test_sd.srcs += booz_imu.c $(SRC_ARCH)/booz_imu_hw.c
test_sd.srcs += max1167.c  $(SRC_ARCH)/max1167_hw.c 
test_sd.srcs += micromag.c $(SRC_ARCH)/micromag_hw.c
test_sd.srcs += scp1000.c  $(SRC_ARCH)/scp1000_hw.c
test_sd.CFLAGS += -DADC -DUSE_AD0 -DUSE_AD0_1 -DUSE_AD0_2 -DUSE_AD0_3 -DUSE_AD0_4
test_sd.srcs += $(SRC_ARCH)/adc_hw.c

test_sd.srcs += booz_still_detection.c


##
##
##
## IMU test_vfilter
##
##
##
test_vf.ARCHDIR = $(ARCHI)
test_vf.ARCH = arm7tdmi
test_vf.TARGET = test_vf
test_vf.TARGETDIR = test_vf

test_vf.CFLAGS += -DCONFIG=\"pprz_imu.h\"  -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./250.))'
test_vf.srcs = booz_test_vfilter.c sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c

test_vf.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600
test_vf.srcs += $(SRC_ARCH)/uart_hw.c

test_vf.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart1 
test_vf.srcs += downlink.c pprz_transport.c

test_vf.srcs += booz_imu.c $(SRC_ARCH)/booz_imu_hw.c
test_vf.srcs += max1167.c  $(SRC_ARCH)/max1167_hw.c 
test_vf.srcs += micromag.c $(SRC_ARCH)/micromag_hw.c
test_vf.srcs += scp1000.c  $(SRC_ARCH)/scp1000_hw.c
test_vf.CFLAGS += -DADC -DUSE_AD0 -DUSE_AD0_1 -DUSE_AD0_2 -DUSE_AD0_3 -DUSE_AD0_4
test_vf.srcs += $(SRC_ARCH)/adc_hw.c

test_vf.CFLAGS += -DDT_VFILTER="(1./250.)"
test_vf.srcs += tl_vfilter.c


##
##
## IMU test GPS filter
##
##
tf.ARCHDIR = $(ARCHI)
tf.ARCH = arm7tdmi
tf.TARGET = tf
tf.TARGETDIR = tf

tf.CFLAGS += -DCONFIG=\"pprz_imu.h\"  -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./60.))'
tf.srcs = booz_tf_main.c sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c

tf.CFLAGS += -DLED

tf.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600
tf.srcs += $(SRC_ARCH)/uart_hw.c

tf.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart1 
tf.srcs += downlink.c pprz_transport.c

tf.CFLAGS += -DADC -DUSE_AD0 -DUSE_AD0_1 -DUSE_AD0_2 -DUSE_AD0_3 -DUSE_AD0_4
tf.srcs += $(SRC_ARCH)/adc_hw.c

tf.srcs += micromag.c $(SRC_ARCH)/micromag_hw.c

tf.CFLAGS += -DGPS -DUBX -DUSE_UART0 -DGPS_LINK=Uart0 -DUART0_BAUD=B38400 -DDOWNLINK_GPS_DEVICE=Uart1 -DGPS_BAUD=38400
tf.srcs += gps_ubx.c gps.c latlong.c


##
##
##
## IMU test_link_mcu_imu
##
##
##
test_link_mcu_imu.ARCHDIR = $(ARCHI)
test_link_mcu_imu.ARCH = arm7tdmi
test_link_mcu_imu.TARGET = test_link_mcu_imu
test_link_mcu_imu.TARGETDIR = test_link_mcu_imu

test_link_mcu_imu.CFLAGS += -DBOOZ_DEBUG
test_link_mcu_imu.srcs += booz_debug.c
test_link_mcu_imu.CFLAGS += -DBOOZ_FILTER_MCU -DFLOAT_T=float

test_link_mcu_imu.CFLAGS += -DCONFIG=\"pprz_imu.h\"  -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./250.))'
test_link_mcu_imu.srcs += booz_test_link_mcu_imu.c sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c

test_link_mcu_imu.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600
test_link_mcu_imu.srcs += $(SRC_ARCH)/uart_hw.c

test_link_mcu_imu.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart1 
test_link_mcu_imu.srcs += downlink.c pprz_transport.c

test_link_mcu_imu.srcs += booz_inter_mcu.c
test_link_mcu_imu.srcs += booz_link_mcu.c $(SRC_ARCH)/booz_link_mcu_hw.c


##
##
##
## CTL test_link_mcu_ctl
##
##
##
test_link_mcu_ctl.ARCHDIR = $(ARCHI)
test_link_mcu_ctl.ARCH = arm7tdmi
test_link_mcu_ctl.TARGET = test_link_mcu_ctl
test_link_mcu_ctl.TARGETDIR = test_link_mcu_ctl

test_link_mcu_ctl.CFLAGS += -DBOOZ_DEBUG
test_link_mcu_ctl.srcs += booz_debug.c
test_link_mcu_ctl.CFLAGS += -DBOOZ_CONTROLLER_MCU -DFLOAT_T=float

test_link_mcu_ctl.CFLAGS += -DCONFIG=\"pprz_imu.h\"  -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./250.))'
test_link_mcu_ctl.srcs += booz_test_link_mcu_ctl.c sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c

test_link_mcu_ctl.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600
test_link_mcu_ctl.srcs += $(SRC_ARCH)/uart_hw.c

test_link_mcu_ctl.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart1 
test_link_mcu_ctl.srcs += downlink.c pprz_transport.c

test_link_mcu_ctl.srcs += booz_inter_mcu.c
test_link_mcu_ctl.srcs += booz_link_mcu.c $(SRC_ARCH)/booz_link_mcu_hw.c


##
##
##
## CTL test_motor
##
##
##
test_motor.ARCHDIR = $(ARCHI)
test_motor.ARCH = arm7tdmi
test_motor.TARGET = test_motor
test_motor.TARGETDIR = test_motor

test_motor.CFLAGS += -DBOOZ_DEBUG
test_motor.srcs += booz_debug.c
test_motor.CFLAGS += -DBOOZ_CONTROLLER_MCU -DFLOAT_T=float

test_motor.CFLAGS += -DCONFIG=\"pprz_imu.h\"  -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./250.))'
test_motor.srcs += booz_test_motor.c sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c

test_motor.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600
test_motor.srcs += $(SRC_ARCH)/uart_hw.c

test_motor.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart1 
test_motor.srcs += downlink.c pprz_transport.c





#
# simple emtpy demo
#
demo1.ARCHDIR = $(ARCHI)
demo1.ARCH = arm7tdmi
demo1.TARGET = demo1
demo1.TARGETDIR = demo1

demo1.CFLAGS += -DCONFIG=\"conf_demo.h\"
demo1.srcs = main_demo1.c