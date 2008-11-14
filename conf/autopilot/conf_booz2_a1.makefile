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
BOOZ_PRIV_TEST=booz_priv/test
BOOZ_ARCH=booz/arm7


ap.ARCHDIR = $(ARCHI)
ap.ARCH = arm7tdmi
ap.TARGET = ap
ap.TARGETDIR = ap

#ap.CFLAGS += -DKILL_MOTORS

ap.CFLAGS += -DCONFIG=\"booz2_v1_0.h\" -I$(BOOZ_ARCH) -I$(BOOZ_PRIV) -I$(BOOZ_PRIV_ARCH)
ap.srcs += $(BOOZ_PRIV)/booz2_main.c
ap.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))' -DTIME_LED=1
ap.CFLAGS += -DLED
ap.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c

ap.CFLAGS += -DUSE_UART0 -DUART0_BAUD=B57600
ap.srcs += $(SRC_ARCH)/uart_hw.c

ap.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart0 
ap.srcs += $(BOOZ_PRIV)/booz2_telemetry.c downlink.c pprz_transport.c

ap.CFLAGS += -DDATALINK=PPRZ -DPPRZ_UART=Uart0
ap.srcs += $(BOOZ)/booz_datalink.c

ap.srcs += commands.c

ap.CFLAGS += -DRADIO_CONTROL -DRADIO_CONTROL_TYPE=RC_FUTABA -DRC_LED=4
ap.srcs += radio_control.c $(SRC_ARCH)/ppm_hw.c

ap.CFLAGS += -DACTUATORS=\"actuators_buss_twi_blmc_hw.h\" -DUSE_BUSS_TWI_BLMC
ap.srcs += $(BOOZ_ARCH)/actuators_buss_twi_blmc_hw.c actuators.c
ap.CFLAGS += -DUSE_I2C0 -DI2C0_SCLL=150 -DI2C0_SCLH=150 -DI2C0_VIC_SLOT=10
ap.srcs += i2c.c $(SRC_ARCH)/i2c_hw.c

#ap.srcs += $(BOOZ_PRIV)/booz2_imu_v3.c $(BOOZ_PRIV_ARCH)/booz2_imu_v3_hw.c
ap.srcs += $(BOOZ_PRIV)/booz2_imu_crista.c $(BOOZ_PRIV_ARCH)/booz2_imu_crista_hw.c
ap.CFLAGS += -DFLOAT_T=float
ap.srcs += $(BOOZ_PRIV)/booz2_imu.c

ap.CFLAGS += -DBOOZ2_ANALOG_BARO_LED=5 -DBOOZ2_ANALOG_BARO_PERIOD='SYS_TICS_OF_SEC((1./100.))'
ap.srcs += $(BOOZ_PRIV)/booz2_analog_baro.c


ap.CFLAGS += -DUSE_I2C1  -DI2C1_SCLL=150 -DI2C1_SCLH=150 -DI2C1_VIC_SLOT=11 -DI2C1_BUF_LEN=16
#ap.srcs += i2c.c $(SRC_ARCH)/i2c_hw.c
ap.CFLAGS += -DUSE_AMI601
ap.srcs += AMI601.c


ap.srcs += $(BOOZ_PRIV)/booz2_autopilot.c

ap.CFLAGS += -DFILTER_ALIGNER_LED=6
ap.srcs += $(BOOZ_PRIV)/booz2_filter_aligner2.c
ap.srcs += $(BOOZ_PRIV)/booz2_filter_attitude_cmpl_euler.c
ap.srcs += $(BOOZ_PRIV)/booz_trig_int.c

ap.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B38400 -DUART1_VIC_SLOT=6
ap.CFLAGS += -DGPS_LINK=Uart1 -DGPS_LED=7
ap.srcs += $(BOOZ_PRIV)/booz2_gps.c
ap.srcs += $(BOOZ_PRIV)/booz2_guidance_h.c
ap.srcs += $(BOOZ_PRIV)/booz2_guidance_v.c

#ap.CFLAGS += -DNAV_ENAC
ap.srcs += $(BOOZ_PRIV)/booz2_navigation.c


ap.srcs += $(BOOZ_PRIV)/booz2_stabilization.c
ap.srcs += $(BOOZ_PRIV)/booz2_stabilization_rate.c
ap.srcs += $(BOOZ_PRIV)/booz2_stabilization_attitude.c
ap.srcs += $(BOOZ_PRIV)/booz_supervision_int.c

# Joystick
ap.CFLAGS += -DBOOZ2_STICK



# test leds
test_led.ARCHDIR = $(ARCHI)
test_led.ARCH = arm7tdmi
test_led.TARGET = test_led
test_led.TARGETDIR = test_led

test_led.CFLAGS += -DCONFIG=\"booz2_v1_0.h\" -I$(BOOZ_PRIV_ARCH)
test_led.srcs += $(BOOZ_PRIV)/test_led.c
test_led.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./1024.))' -DTIME_LED=1
test_led.CFLAGS += -DLED
test_led.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c



#
# tunnel
#

tunnel.ARCHDIR = $(ARCHI)
tunnel.ARCH = arm7tdmi
tunnel.TARGET = tunnel
tunnel.TARGETDIR = tunnel

tunnel.CFLAGS += -DCONFIG=\"booz2_v1_0.h\" -I$(BOOZ_PRIV_ARCH)
tunnel.srcs += $(BOOZ_PRIV_TEST)/booz2_tunnel.c
tunnel.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))' -DTIME_LED=1
tunnel.CFLAGS += -DLED
tunnel.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c

tunnel.CFLAGS += -DUSE_UART0 -DUART0_BAUD=B57600
tunnel.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B38400
tunnel.srcs += $(SRC_ARCH)/uart_hw.c

#
# test GPS
#
test_gps.ARCHDIR = $(ARCHI)
test_gps.ARCH = arm7tdmi
test_gps.TARGET = test_gps
test_gps.TARGETDIR = test_gps

test_gps.CFLAGS += -DCONFIG=\"booz2_v1_0.h\" -I$(BOOZ_PRIV) -I$(BOOZ_PRIV_ARCH)
test_gps.srcs += $(BOOZ_PRIV_TEST)/booz2_test_gps.c
test_gps.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))' -DTIME_LED=1
test_gps.CFLAGS += -DLED
test_gps.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c

test_gps.CFLAGS += -DUSE_UART0 -DUART0_BAUD=B57600
test_gps.srcs += $(SRC_ARCH)/uart_hw.c

test_gps.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B38400
test_gps.CFLAGS += -DGPS_LINK=Uart1 -DGPS_LED=7
test_gps.srcs += $(BOOZ_PRIV)/booz2_gps.c





#
# test modem
#
test_modem.ARCHDIR = $(ARCHI)
test_modem.ARCH = arm7tdmi
test_modem.TARGET = test_modem
test_modem.TARGETDIR = test_modem

test_modem.CFLAGS += -DCONFIG=\"tiny_1_1.h\" -I$(BOOZ_PRIV_ARCH)
test_modem.srcs += $(BOOZ_PRIV)/test_modem.c
test_modem.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./1024.))' -DTIME_LED=1
test_modem.CFLAGS += -DLED
test_modem.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c

test_modem.CFLAGS += -DUSE_UART0 -DUART0_BAUD=B9600
test_modem.srcs += $(SRC_ARCH)/uart_hw.c

test_modem.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart0 
test_modem.srcs += downlink.c pprz_transport.c

test_modem.CFLAGS += -DBOOZ_ANALOG_BARO_LED=2 -DBOOZ_ANALOG_BARO_PERIOD='SYS_TICS_OF_SEC((1./100.))'
test_modem.srcs += $(BOOZ_PRIV)/booz_analog_baro.c


#
# test AMI
#
test_ami.ARCHDIR = $(ARCHI)
test_ami.ARCH = arm7tdmi
test_ami.TARGET = test_ami
test_ami.TARGETDIR = test_ami

test_ami.CFLAGS += -DCONFIG=\"booz2_v1_0.h\" -I$(BOOZ_PRIV_ARCH)
test_ami.srcs += $(BOOZ_PRIV_TEST)/booz2_test_ami.c
test_ami.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./50.))' -DTIME_LED=1
test_ami.CFLAGS += -DLED
test_ami.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c

test_ami.CFLAGS += -DUSE_UART0 -DUART0_BAUD=B57600
test_ami.srcs += $(SRC_ARCH)/uart_hw.c

test_ami.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart0 
test_ami.srcs += downlink.c pprz_transport.c

test_ami.CFLAGS += -DUSE_I2C1  -DI2C1_SCLL=150 -DI2C1_SCLH=150 -DI2C1_VIC_SLOT=11 -DI2C1_BUF_LEN=16
test_ami.srcs += i2c.c $(SRC_ARCH)/i2c_hw.c
test_ami.CFLAGS += -DUSE_AMI601
test_ami.srcs += AMI601.c


#
# test crista
#
test_crista.ARCHDIR = $(ARCHI)
test_crista.ARCH = arm7tdmi
test_crista.TARGET = test_crista
test_crista.TARGETDIR = test_crista

test_crista.CFLAGS += -DCONFIG=\"booz2_v1_0.h\" -I$(BOOZ_PRIV) -I$(BOOZ_PRIV_ARCH)
test_crista.srcs += $(BOOZ_PRIV_TEST)/booz2_test_crista.c
test_crista.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))' -DTIME_LED=1
test_crista.CFLAGS += -DLED
test_crista.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c

test_crista.CFLAGS += -DUSE_UART0 -DUART0_BAUD=B57600
test_crista.srcs += $(SRC_ARCH)/uart_hw.c

test_crista.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart0 
test_crista.srcs += downlink.c pprz_transport.c

test_crista.CFLAGS += -DFLOAT_T=float
test_crista.srcs += $(BOOZ_PRIV)/booz2_imu.c
test_crista.srcs += $(BOOZ_PRIV)/booz2_imu_crista.c $(BOOZ_PRIV_ARCH)/booz2_imu_crista_hw.c


