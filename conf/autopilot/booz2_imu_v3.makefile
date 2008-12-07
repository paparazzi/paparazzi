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




#
# IMU V3 MCU
#

imu.ARCHDIR = $(ARCHI)
imu.ARCH = arm7tdmi
imu.TARGET = imu
imu.TARGETDIR = imu

imu.CFLAGS += -DCONFIG=\"pprz_imu.h\" -I$(BOOZ) -I$(BOOZ_ARCH) -I$(BOOZ_PRIV) -I$(BOOZ_PRIV_ARCH)
imu.srcs += $(BOOZ_PRIV)/imu_v3_main.c
imu.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))'
# -DTIME_LED=1
imu.CFLAGS += -DLED
imu.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c

imu.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600 -DUART1_VIC_SLOT=6
imu.srcs += $(SRC_ARCH)/uart_hw.c

imu.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart1 
imu.srcs   += downlink.c pprz_transport.c $(BOOZ_PRIV)/imu_v3_telemetry.c

imu.CFLAGS += -DIMU_SENSORS_SPI1_VIC_SLOT=7
imu.srcs   += $(BOOZ_PRIV)/imu_v3_sensors.c $(BOOZ_PRIV_ARCH)/imu_v3_sensors_hw.c
imu.CFLAGS += -DADC -DUSE_AD0 -DUSE_AD0_1 -DUSE_AD0_2 -DUSE_AD0_3 -DAD0_VIC_SLOT=2
imu.srcs   += $(SRC_ARCH)/adc_hw.c
imu.CFLAGS += -DMAX1167_EOC_VIC_SLOT=8
imu.srcs   += max1167.c  $(SRC_ARCH)/max1167_hw.c 
imu.CFLAGS += -DMICROMAG_DRDY_VIC_SLOT=9
imu.srcs   += micromag.c $(SRC_ARCH)/micromag_hw.c

imu.CFLAGS += -DIMU_CLIENT_LINK_SPI0_VIC_SLOT=3
imu.srcs   += $(BOOZ_PRIV)/imu_v3_client_link.c $(BOOZ_PRIV_ARCH)/imu_v3_client_link_hw.c



#
# IMU V3 MCU tests
#

#
# test micromag
#
imu_test_micromag.ARCHDIR = $(ARCHI)
imu_test_micromag.ARCH = arm7tdmi
imu_test_micromag.TARGET = imu_test_micromag
imu_test_micromag.TARGETDIR = imu_test_micromag

imu_test_micromag.CFLAGS += -DCONFIG=\"pprz_imu.h\" -I$(BOOZ) -I$(BOOZ_ARCH) -I$(BOOZ_PRIV) -I$(BOOZ_PRIV_ARCH)
imu_test_micromag.srcs += $(BOOZ_PRIV_TEST)/imu_v3_test_micromag.c
imu_test_micromag.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))'
# -DTIME_LED=1
imu_test_micromag.CFLAGS += -DLED
imu_test_micromag.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c

imu_test_micromag.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600 -DUART1_VIC_SLOT=6
imu_test_micromag.srcs += $(SRC_ARCH)/uart_hw.c

imu_test_micromag.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart1 
imu_test_micromag.srcs   += downlink.c pprz_transport.c

imu_test_micromag.CFLAGS += -DMICROMAG_DRDY_VIC_SLOT=9
imu_test_micromag.srcs   += micromag.c $(SRC_ARCH)/micromag_hw.c





#
# Controller MCU tests
#

#
# test GPS, aka tunnel
#

tunnel.ARCHDIR = $(ARCHI)
tunnel.ARCH = arm7tdmi
tunnel.TARGET = tunnel
tunnel.TARGETDIR = tunnel

tunnel.CFLAGS += -DCONFIG=\"booz2_board_proto.h\" -I$(BOOZ_PRIV_ARCH)
tunnel.srcs += $(BOOZ_PRIV_TEST)/booz2_tunnel.c
tunnel.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))' -DTIME_LED=1
tunnel.CFLAGS += -DLED
tunnel.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c

tunnel.CFLAGS += -DUSE_UART0 -DUART0_BAUD=B57600
tunnel.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B38400
tunnel.srcs += $(SRC_ARCH)/uart_hw.c



