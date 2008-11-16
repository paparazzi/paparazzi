#
# $id$
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


#
# tunnel hw
#
tunnel.ARCHDIR = $(ARCHI)
tunnel.ARCH = arm7tdmi
tunnel.TARGET = tunnel
tunnel.TARGETDIR = tunnel

tunnel.CFLAGS += -DCONFIG=$(BOARD_CFG) -I$(BOOZ_PRIV_ARCH)
tunnel.srcs += $(BOOZ_PRIV_TEST)/booz2_tunnel.c
tunnel.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))' -DTIME_LED=1
tunnel.CFLAGS += -DLED
tunnel.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c

tunnel.CFLAGS += -DUSE_UART0 -DUART0_BAUD=B57600
#tunnel.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600
tunnel.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B38400
#tunnel.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B19200
#tunnel.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B9600
tunnel.srcs += $(SRC_ARCH)/uart_hw.c


#
# tunnel bit banging
#
tunnel_bb.ARCHDIR = $(ARCHI)
tunnel_bb.ARCH = arm7tdmi
tunnel_bb.TARGET = tunnel_bb
tunnel_bb.TARGETDIR = tunnel_bb

tunnel_bb.CFLAGS += -DCONFIG=$(BOARD_CFG) -I$(BOOZ_PRIV_ARCH)
tunnel_bb.srcs += $(BOOZ_PRIV_TEST)/booz2_tunnel_bb.c
tunnel_bb.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))' -DTIME_LED=1
tunnel_bb.CFLAGS += -DLED
tunnel_bb.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c

#
# test leds
#
test_led.ARCHDIR = $(ARCHI)
test_led.ARCH = arm7tdmi
test_led.TARGET = test_led
test_led.TARGETDIR = test_led

test_led.CFLAGS += -DCONFIG=$(BOARD_CFG) $(BOOZ_CFLAGS)
test_led.srcs += $(BOOZ_PRIV)/test_led.c
test_led.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))' -DTIME_LED=1
test_led.CFLAGS += -DLED
test_led.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c



#
# test GPS
#
test_gps.ARCHDIR = $(ARCHI)
test_gps.ARCH = arm7tdmi
test_gps.TARGET = test_gps
test_gps.TARGETDIR = test_gps

test_gps.CFLAGS += -DCONFIG=$(BOARD_CFG) -I$(BOOZ_PRIV) -I$(BOOZ_PRIV_ARCH)
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

test_modem.CFLAGS += -DCONFIG=$(BOARD_CFG) -I$(BOOZ_PRIV_ARCH)
test_modem.srcs += $(BOOZ_PRIV)/test_modem.c
test_modem.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))' -DTIME_LED=1
test_modem.CFLAGS += -DLED
test_modem.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c

test_modem.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600
test_modem.srcs += $(SRC_ARCH)/uart_hw.c

test_modem.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart1 
test_modem.srcs += downlink.c pprz_transport.c

#test_modem.CFLAGS += -DBOOZ_ANALOG_BARO_LED=2 -DBOOZ_ANALOG_BARO_PERIOD='SYS_TICS_OF_SEC((1./100.))'
#test_modem.srcs += $(BOOZ_PRIV)/booz_analog_baro.c


#
# test AMI
#
test_ami.ARCHDIR = $(ARCHI)
test_ami.ARCH = arm7tdmi
test_ami.TARGET = test_ami
test_ami.TARGETDIR = test_ami

test_ami.CFLAGS += -DCONFIG=$(BOARD_CFG) -I$(BOOZ_PRIV_ARCH)
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

test_crista.CFLAGS += -DCONFIG=$(BOARD_CFG) -I$(BOOZ_PRIV) -I$(BOOZ_PRIV_ARCH)
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


#
# test MAX1168
#
test_max1168.ARCHDIR = $(ARCHI)
test_max1168.ARCH = arm7tdmi
test_max1168.TARGET = test_max1168
test_max1168.TARGETDIR = test_max1168

test_max1168.CFLAGS += -DCONFIG=$(BOARD_CFG) -I$(BOOZ_PRIV) -I$(BOOZ_PRIV_ARCH)
test_max1168.srcs += $(BOOZ_PRIV_TEST)/booz2_test_max1168.c
test_max1168.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))' -DTIME_LED=1
test_max1168.CFLAGS += -DLED
test_max1168.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c

test_max1168.CFLAGS += -DUSE_UART0 -DUART0_BAUD=B57600
test_max1168.srcs += $(SRC_ARCH)/uart_hw.c

test_max1168.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart0 
test_max1168.srcs += downlink.c pprz_transport.c

test_max1168.CFLAGS += -I$(BOOZ)
test_max1168.srcs += $(BOOZ)/booz_debug.c

test_max1168.CFLAGS += -DMAX1168_EOC_VIC_SLOT=8 -DSSP_VIC_SLOT=9
test_max1168.srcs += $(BOOZ_PRIV)/booz2_max1168.c $(BOOZ_PRIV_ARCH)/booz2_max1168_hw.c




#
# test MICROMAG
#
test_micromag.ARCHDIR = $(ARCHI)
test_micromag.ARCH = arm7tdmi
test_micromag.TARGET = test_micromag
test_micromag.TARGETDIR = test_micromag

test_micromag.CFLAGS += -DCONFIG=$(BOARD_CFG) -I$(BOOZ_PRIV) -I$(BOOZ_PRIV_ARCH)
test_micromag.srcs += $(BOOZ_PRIV_TEST)/booz2_test_micromag.c
test_micromag.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))' -DTIME_LED=1
test_micromag.CFLAGS += -DLED
test_micromag.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c

test_micromag.CFLAGS += -DUSE_UART0 -DUART0_BAUD=B57600
test_micromag.srcs += $(SRC_ARCH)/uart_hw.c

test_micromag.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart0 
test_micromag.srcs += downlink.c pprz_transport.c

test_micromag.CFLAGS += -I$(BOOZ)
test_micromag.srcs += $(BOOZ)/booz_debug.c

test_micromag.CFLAGS += -DMICROMAG_DRDY_VIC_SLOT=8 -DSSP_VIC_SLOT=9
test_micromag.srcs += micromag.c $(SRC_ARCH)/micromag_hw.c











#
# test IMU b2
#
test_b2.ARCHDIR = $(ARCHI)
test_b2.ARCH = arm7tdmi
test_b2.TARGET = test_b2
test_b2.TARGETDIR = test_b2

test_b2.CFLAGS += -DCONFIG=$(BOARD_CFG) -I$(BOOZ_PRIV) -I$(BOOZ_PRIV_ARCH)
test_b2.srcs += $(BOOZ_PRIV_TEST)/booz2_test_b2.c
test_b2.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))' -DTIME_LED=1
test_b2.CFLAGS += -DLED
test_b2.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c

test_b2.CFLAGS += -DUSE_UART0 -DUART0_BAUD=B57600
test_b2.srcs += $(SRC_ARCH)/uart_hw.c

test_b2.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart0 
test_b2.srcs += downlink.c pprz_transport.c

test_b2.CFLAGS += -DFLOAT_T=float
test_b2.srcs += $(BOOZ_PRIV)/booz2_imu.c
test_b2.CFLAGS += -DMAX1168_EOC_VIC_SLOT=8 -DSSP_VIC_SLOT=9
test_b2.srcs += $(BOOZ_PRIV)/booz2_imu_b2.c $(BOOZ_PRIV_ARCH)/booz2_imu_b2_hw.c


#
# test RC
#
test_rc.ARCHDIR = $(ARCHI)
test_rc.ARCH = arm7tdmi
test_rc.TARGET = test_rc
test_rc.TARGETDIR = test_rc

test_rc.CFLAGS += -DCONFIG=$(BOARD_CFG) -I$(BOOZ_PRIV) -I$(BOOZ_PRIV_ARCH)
test_rc.srcs += $(BOOZ_PRIV_TEST)/booz2_test_rc.c
test_rc.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))' -DTIME_LED=1
test_rc.CFLAGS += -DLED
test_rc.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c

test_rc.CFLAGS += -DUSE_UART0 -DUART0_BAUD=B57600
test_rc.srcs += $(SRC_ARCH)/uart_hw.c

test_rc.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart0 
test_rc.srcs += downlink.c pprz_transport.c

test_rc.CFLAGS += -I$(BOOZ)
test_rc.srcs += $(BOOZ)/booz_debug.c

test_rc.CFLAGS += -DRADIO_CONTROL -DRADIO_CONTROL_TYPE=RC_FUTABA -DRC_LED=4
test_rc.srcs += radio_control.c $(SRC_ARCH)/ppm_hw.c


#
# test MC
#
test_mc.ARCHDIR = $(ARCHI)
test_mc.ARCH = arm7tdmi
test_mc.TARGET = test_mc
test_mc.TARGETDIR = test_mc

test_mc.CFLAGS += -DCONFIG=$(BOARD_CFG) -I$(BOOZ_PRIV) -I$(BOOZ_PRIV_ARCH) -I$(BOOZ_ARCH)
test_mc.srcs += $(BOOZ_PRIV_TEST)/booz2_test_mc.c
test_mc.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))' -DTIME_LED=1
test_mc.CFLAGS += -DLED
test_mc.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c

test_mc.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600
test_mc.srcs += $(SRC_ARCH)/uart_hw.c

test_mc.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart1 
test_mc.srcs += downlink.c pprz_transport.c

test_mc.CFLAGS += -I$(BOOZ)
test_mc.srcs += $(BOOZ)/booz_debug.c


test_mc.CFLAGS += -DACTUATORS=\"actuators_buss_twi_blmc_hw.h\" -DUSE_BUSS_TWI_BLMC
test_mc.srcs += $(BOOZ_ARCH)/actuators_buss_twi_blmc_hw.c actuators.c
test_mc.CFLAGS += -DUSE_I2C0 -DI2C0_SCLL=150 -DI2C0_SCLH=150 -DI2C0_VIC_SLOT=10
test_mc.srcs += i2c.c $(SRC_ARCH)/i2c_hw.c


#
# test 24 bits baro
#
test_baro_24.ARCHDIR = $(ARCHI)
test_baro_24.ARCH = arm7tdmi
test_baro_24.TARGET = test_baro_24
test_baro_24.TARGETDIR = test_baro_24

test_baro_24.CFLAGS += -DCONFIG=$(BOARD_CFG) $(BOOZ_CFLAGS)
test_baro_24.srcs += $(BOOZ_PRIV_TEST)/booz2_test_baro_24.c
test_baro_24.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./50.))' -DTIME_LED=1
test_baro_24.CFLAGS += -DLED
test_baro_24.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c

test_baro_24.CFLAGS += -DUSE_UART0 -DUART0_BAUD=B57600
test_baro_24.srcs += $(SRC_ARCH)/uart_hw.c

test_baro_24.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart0 
test_baro_24.srcs += downlink.c pprz_transport.c

test_baro_24.CFLAGS += -DUSE_I2C1  -DI2C1_SCLL=150 -DI2C1_SCLH=150 -DI2C1_VIC_SLOT=11 -DI2C1_BUF_LEN=16
test_baro_24.srcs += i2c.c $(SRC_ARCH)/i2c_hw.c
#test_baro_24.CFLAGS += -DUSE_AMI601
test_baro_24.srcs += $(BOOZ_PRIV)/booz2_baro_24.c

