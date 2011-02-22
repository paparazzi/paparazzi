#
# $Id$
#
# Copyright (C) 2009 Antoine Drouin, Allen H. Ibara
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

FLASH_MODE = ISP
LPC21ISP_PORT = /dev/ttyUSB0

LPC21ISP_BAUD = 38400
LPC21ISP_XTAL = 12000


LPC21ISP_CONTROL = -control

LDSCRIPT=$(SRC_ARCH)/LPC2129-ROM.ld

BOARD_CFG = \"csc_board_v1_0.h\"

PERIODIC_FREQ = 512

SRC_CSC=csc
SRC_CSC_ARCH=$(SRC_CSC)/$(ARCHI)
SRC_BOOZ=booz
SRC_SUBSYSTEMS=subsystems

ap.ARCHDIR = $(ARCHI)

ap.CFLAGS += -I$(SRC_CSC) -I$(SRC_BOOZ) -I$(SRC_CSC_ARCH)
ap.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
ap.srcs += $(SRC_CSC)/mercury_main.c
ap.CFLAGS += -DUSE_LED -DTIME_LED=1
ap.CFLAGS += -DAHRS_ALIGNER_LED=2

ap.CFLAGS += -DCSC_BOARD_ID=$(CSC_ID)

ap.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))' -DTIMER0_VIC_SLOT=1
ap.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c

ap.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c
ap.srcs += $(SRC_ARCH)/adc_hw.c
ap.CFLAGS += -DADC -DUSE_AD0 -DUSE_AD0_0 -DUSE_AD0_1


ap.CFLAGS += -DUSE_UART0 -DUART0_BAUD=B230400 -DUART0_VIC_SLOT=5
ap.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600 -DUART1_VIC_SLOT=6
ap.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport \
                      -DDOWNLINK_DEVICE=Uart1
ap.srcs += downlink.c pprz_transport.c $(SRC_CSC)/csc_telemetry.c
ap.CFLAGS += -DDATALINK=PPRZ -DPPRZ_UART=Uart1
ap.srcs += $(SRC_CSC)/csc_datalink.c

ap.CFLAGS += -DPPRZ_TRIG_CONST=const
ap.srcs += $(SRC_CSC)/mercury_xsens.c $(SRC_SUBSYSTEMS)/imu.c math/pprz_trig_int.c
ap.CFLAGS += -DXSENS1_LINK=Uart0 -DIMU_TYPE_H=\"mercury_xsens.h\"

ap.srcs += $(SRC_BOOZ)/ahrs/ahrs_cmpl_euler.c $(SRC_BOOZ)/ahrs/ahrs_aligner.c
ap.CFLAGS += -DFLOAT_T=float

ap.srcs += $(SRC_FIRMWARE)/stabilization.c
ap.srcs += $(SRC_FIRMWARE)/stabilization/stabilization_rate.c

ap.CFLAGS += -DSTABILISATION_ATTITUDE_TYPE_INT
ap.CFLAGS += -DSTABILISATION_ATTITUDE_H=\"stabilization/stabilization_attitude_int.h\"
ap.CFLAGS += -DSTABILISATION_ATTITUDE_REF_H=\"stabilization/stabilization_attitude_ref_euler_int.h\"
ap.srcs += $(SRC_FIRMWARE)/stabilization/stabilization_attitude_ref_euler_int.c
ap.srcs += $(SRC_FIRMWARE)/stabilization/stabilization_attitude_euler_int.c


# AHI copied from booz w/ light modifications for vertical control
ap.CFLAGS += -DUSE_VFF -DDT_VFILTER="(1./512.)" -DFLOAT_T=float
ap.srcs += $(SRC_CSC)/csc_booz2_ins.c $(SRC_CSC)/csc_booz2_hf_float.c $(SRC_CSC)/csc_booz2_vf_float.c
ap.srcs += $(SRC_CSC)/csc_booz2_guidance_v.c


ap.CFLAGS += -DAP_LINK_CAN # -DCAN_LED=2
ap.CFLAGS += -DUSE_CAN1 -DCAN1_BTR=CANBitrate125k_3MHz
ap.CFLAGS +=  -DCAN1_VIC_SLOT=3 -DCAN1_ERR_VIC_SLOT=7
ap.srcs += $(SRC_CSC)/csc_can.c
#ap.CFLAGS += -DUSE_CAN2 -DCAN2_BTR=CANBitrate125k_2MHz -DCAN2_VIC_SLOT=4

ap.srcs += $(SRC_CSC)/csc_ap_link.c

ap.srcs += $(SRC_CSC)/csc_adc.c

ap.srcs += $(SRC_ARCH)/servos_csc.c
ap.CFLAGS += -DACTUATORS=\"servos_csc.h\"
ap.srcs += commands.c actuators.c

ap.srcs += $(SRC_CSC_ARCH)/props_csc.c

ap.CFLAGS += -DRADIO_CONTROL
ap.srcs += radio_control.c $(SRC_ARCH)/ppm_hw.c

ap.srcs += $(SRC_CSC)/mercury_ap.c
ap.srcs += $(SRC_CSC)/mercury_supervision.c


ap.CFLAGS += -DERROR_LED=4

#
#
# test uart
#
test_uart.ARCHDIR = $(ARCHI)


test_uart.CFLAGS += -I$(SRC_CSC)
test_uart.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_uart.srcs += $(SRC_CSC)/csc_test_uart.c
test_uart.CFLAGS += -DUSE_LED

# -DTIME_LED=1
test_uart.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))' -DTIMER0_VIC_SLOT=1
test_uart.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c

test_uart.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c

test_uart.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600 -DUART1_VIC_SLOT=6
test_uart.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport \
                      -DDOWNLINK_DEVICE=Uart1
test_uart.srcs += downlink.c pprz_transport.c
