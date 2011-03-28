#
# $id$
#
# Copyright (C) 2008 Antoine Drouin (poinix@gmail.com)
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

include $(PAPARAZZI_SRC)/conf/autopilot/booz2_common.makefile

#
# test_led : blinks all leds
#
test_led.ARCHDIR = $(ARCH)

test_led.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG) $(BOOZ_CFLAGS)
test_led.CFLAGS += -DPERIPHERALS_AUTO_INIT
test_led.srcs   += $(SRC_BOOZ_TEST)/booz2_test_led.c
test_led.CFLAGS += -DUSE_LED
test_led.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))' -DTIME_LED=1
test_led.srcs   += sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c

#
# test_downlink : sends a TIME message
#
test_downlink.ARCHDIR = $(ARCH)

test_downlink.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG) $(BOOZ_CFLAGS)
test_downlink.CFLAGS += -DPERIPHERALS_AUTO_INIT
test_downlink.srcs   += $(SRC_BOOZ_TEST)/booz2_test_downlink.c
test_downlink.CFLAGS += -DUSE_LED
test_downlink.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./10.))' -DTIME_LED=1
test_downlink.srcs   += sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c

test_downlink.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600
test_downlink.srcs   += $(SRC_ARCH)/mcu_periph/uart_arch.c

test_downlink.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart1
test_downlink.srcs   += downlink.c pprz_transport.c


#
# test_max1168 : samples gyros and mags, sends values
#
test_max1168.ARCHDIR = $(ARCH)

test_max1168.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG) -I$(SRC_BOOZ) -I$(SRC_BOOZ_ARCH)
test_max1168.CFLAGS += -DPERIPHERALS_AUTO_INIT
test_max1168.srcs   += $(SRC_BOOZ_TEST)/booz2_test_max1168.c
test_max1168.CFLAGS += -DUSE_LED
test_max1168.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))' -DTIME_LED=1
test_max1168.srcs   += sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c

test_max1168.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600
test_max1168.srcs   += $(SRC_ARCH)/mcu_periph/uart_arch.c

test_max1168.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart1
test_max1168.srcs   += downlink.c pprz_transport.c

test_max1168.CFLAGS += -DMAX1168_EOC_VIC_SLOT=8 -DSSP_VIC_SLOT=9
test_max1168.srcs   += peripherals/max1168.c \
					   $(SRC_ARCH)/peripherals/max1168_arch.c

#
# test MICROMAG
#
test_micromag.ARCHDIR = $(ARCH)

test_micromag.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG) -I$(SRC_BOOZ) -I$(SRC_BOOZ_ARCH)
test_micromag.srcs += $(SRC_BOOZ_TEST)/booz2_test_micromag.c
test_micromag.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))' -DTIME_LED=1
test_micromag.CFLAGS += -DUSE_LED
test_micromag.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c

test_micromag.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600
test_micromag.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c

test_micromag.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart1
test_micromag.srcs += downlink.c pprz_transport.c

#test_micromag.CFLAGS += -I$(BOOZ)
#test_micromag.srcs += pprz_debug.c

test_micromag.CFLAGS += -DMICROMAG_DRDY_VIC_SLOT=8 -DSSP_VIC_SLOT=9
test_micromag.srcs += micromag.c $(SRC_ARCH)/micromag_hw.c



#
# tunnel hw
#
tunnel.ARCHDIR = $(ARCH)

tunnel.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG) $(BOOZ_CFLAGS)
tunnel.srcs += $(SRC_BOOZ_TEST)/booz2_tunnel.c
tunnel.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))' -DTIME_LED=1
tunnel.CFLAGS += -DUSE_LED
tunnel.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c
tunnel.srcs += mcu.c $(SRC_ARCH)/mcu_arch.c

tunnel.CFLAGS += -DUSE_UART0 -DUART0_BAUD=B38400
tunnel.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600
tunnel.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c


#
# tunnel bit banging
#
tunnel_bb.ARCHDIR = $(ARCH)

tunnel_bb.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG) $(BOOZ_CFLAGS)
tunnel_bb.srcs += $(SRC_BOOZ_TEST)/booz2_tunnel_bb.c
tunnel_bb.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))' -DTIME_LED=1
tunnel_bb.CFLAGS += -DUSE_LED
tunnel_bb.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c


#
# usb tunnels
#
usb_tunnel_0.ARCHDIR = $(ARCH)
usb_tunnel_0.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
usb_tunnel_0.CFLAGS += -DUSE_UART0 -DUART0_BAUD=B38400 -DPERIPHERALS_AUTO_INIT
usb_tunnel_0.CFLAGS += -DUSE_USB_LINE_CODING -DUSE_USB_SERIAL -DUSE_LED
usb_tunnel_0.srcs += $(SRC_ARCH)/usb_tunnel.c $(SRC_ARCH)/usb_ser_hw.c $(SRC_ARCH)/mcu_periph/uart_arch.c
usb_tunnel_0.srcs += $(SRC_ARCH)/lpcusb/usbhw_lpc.c $(SRC_ARCH)/lpcusb/usbinit.c
usb_tunnel_0.srcs += $(SRC_ARCH)/lpcusb/usbcontrol.c $(SRC_ARCH)/lpcusb/usbstdreq.c
usb_tunnel_0.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c
usb_tunnel_0.srcs += mcu.c $(SRC_ARCH)/mcu_arch.c

usb_tunnel_1.ARCHDIR = $(ARCH)
usb_tunnel_1.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
usb_tunnel_1.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B38400 -DPERIPHERALS_AUTO_INIT
usb_tunnel_1.CFLAGS += -DUSE_USB_LINE_CODING -DUSE_USB_SERIAL -DUSE_LED
usb_tunnel_1.srcs += $(SRC_ARCH)/usb_tunnel.c $(SRC_ARCH)/usb_ser_hw.c $(SRC_ARCH)/mcu_periph/uart_arch.c
usb_tunnel_1.srcs += $(SRC_ARCH)/lpcusb/usbhw_lpc.c $(SRC_ARCH)/lpcusb/usbinit.c
usb_tunnel_1.srcs += $(SRC_ARCH)/lpcusb/usbcontrol.c $(SRC_ARCH)/lpcusb/usbstdreq.c
usb_tunnel_1.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c
usb_tunnel_1.srcs += mcu.c $(SRC_ARCH)/mcu_arch.c


#
# test GPS
#
test_gps.ARCHDIR = $(ARCH)

test_gps.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG) -I$(SRC_BOOZ) -I$(SRC_BOOZ_ARCH)
test_gps.srcs += $(SRC_BOOZ_TEST)/booz2_test_gps.c
test_gps.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))' -DTIME_LED=1
test_gps.CFLAGS += -DUSE_LED
test_gps.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c

test_gps.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600
test_gps.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c

test_gps.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart1
test_gps.srcs += downlink.c pprz_transport.c

test_gps.CFLAGS += -DUSE_UART0 -DUART0_BAUD=B38400
test_gps.CFLAGS += -DGPS_LINK=Uart0 -DGPS_LED=2
test_gps.srcs += $(SRC_BOOZ)/booz2_gps.c





#
# test modem
#
test_modem.ARCHDIR = $(ARCH)

test_modem.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG) $(BOOZ_CFLAGS)
test_modem.srcs += $(SRC_BOOZ_TEST)/booz2_test_modem.c
test_modem.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))' -DTIME_LED=1
test_modem.CFLAGS += -DUSE_LED
test_modem.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c

test_modem.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600
test_modem.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c

test_modem.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart1
test_modem.srcs += downlink.c pprz_transport.c

#test_modem.CFLAGS += -DBOOZ_ANALOG_BARO_LED=2 -DBOOZ_ANALOG_BARO_PERIOD='SYS_TICS_OF_SEC((1./100.))'
#test_modem.srcs += $(BOOZ_PRIV)/booz_analog_baro.c


#
# test USB telemetry
#
test_usb.ARCHDIR = $(ARCH)

test_usb.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG) $(BOOZ_CFLAGS)
test_usb.srcs += $(SRC_BOOZ_TEST)/booz2_test_usb.c
test_usb.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))'
# -DTIME_LED=1
test_usb.CFLAGS += -DUSE_LED
test_usb.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c
test_usb.srcs += mcu.c $(SRC_ARCH)/mcu_arch.c

#test_usb.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600
#test_usb.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c
#test_usb.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart1
#test_usb.srcs += downlink.c pprz_transport.c

test_usb.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DUSE_USB_SERIAL
test_usb.CFLAGS += -DDOWNLINK_DEVICE=UsbS -DPPRZ_UART=UsbS -DDATALINK=PPRZ
test_usb.srcs += downlink.c $(SRC_ARCH)/mcu_periph/uart_arch.c $(SRC_ARCH)/usb_ser_hw.c pprz_transport.c
# $(SRC_FIRMWARE)/datalink.c
test_usb.srcs += $(SRC_ARCH)/lpcusb/usbhw_lpc.c $(SRC_ARCH)/lpcusb/usbcontrol.c
test_usb.srcs += $(SRC_ARCH)/lpcusb/usbstdreq.c $(SRC_ARCH)/lpcusb/usbinit.c






#
# test AMI
#
test_ami.ARCHDIR = $(ARCH)

test_ami.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG) $(BOOZ_CFLAGS)
test_ami.srcs += $(SRC_BOOZ_TEST)/booz2_test_ami.c
test_ami.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./50.))' -DTIME_LED=1
test_ami.CFLAGS += -DUSE_LED
test_ami.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c

test_ami.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600
test_ami.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c

test_ami.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart1
test_ami.srcs += downlink.c pprz_transport.c

test_ami.CFLAGS += -DUSE_I2C1  -DI2C1_SCLL=150 -DI2C1_SCLH=150 -DI2C1_VIC_SLOT=11 -DI2C1_BUF_LEN=16
test_ami.srcs += i2c.c $(SRC_ARCH)/i2c_hw.c
test_ami.CFLAGS += -DUSE_AMI601
test_ami.srcs += AMI601.c


#
# test crista
#
test_crista.ARCHDIR = $(ARCH)

test_crista.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG) -I$(SRC_BOOZ) -I$(SRC_BOOZ_ARCH)
test_crista.srcs += $(SRC_BOOZ_TEST)/booz2_test_crista.c
test_crista.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))' -DTIME_LED=1
test_crista.CFLAGS += -DUSE_LED
test_crista.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c

test_crista.CFLAGS += -DUSE_UART0 -DUART0_BAUD=B57600
test_crista.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c

test_crista.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart0
test_crista.srcs += downlink.c pprz_transport.c

test_crista.CFLAGS += -DFLOAT_T=float -DBOOZ2_IMU_TYPE=\"booz2_imu_crista.h\"
test_crista.srcs += $(SRC_BOOZ)/booz2_imu.c
test_crista.srcs += $(SRC_BOOZ)/booz2_imu_crista.c $(SRC_BOOZ_ARCH)/booz2_imu_crista_hw.c




#
# test MICROMAG
#
test_micromag2.ARCHDIR = $(ARCH)

test_micromag2.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG) -I$(SRC_BOOZ) -I$(SRC_BOOZ_ARCH)
test_micromag2.srcs += $(SRC_BOOZ_TEST)/booz2_test_micromag_2.c
test_micromag2.CFLAGS += -DSSP_VIC_SLOT=9
test_micromag2.CFLAGS += -DMICROMAG_DRDY_VIC_SLOT=8
test_micromag2.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./50.))' -DTIME_LED=1
test_micromag2.CFLAGS += -DUSE_LED
test_micromag2.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c

test_micromag2.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600
test_micromag2.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c

test_micromag2.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart1
test_micromag2.srcs += downlink.c pprz_transport.c






#
# test IMU b2
#
test_imu_b2.ARCHDIR = $(ARCH)

test_imu_b2.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG) -I$(SRC_BOOZ) -I$(SRC_BOOZ_ARCH)
test_imu_b2.srcs += $(SRC_BOOZ_TEST)/booz2_test_imu_b2.c
test_imu_b2.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))' -DTIME_LED=1
test_imu_b2.CFLAGS += -DUSE_LED
test_imu_b2.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c

test_imu_b2.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600
test_imu_b2.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c

test_imu_b2.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart1
test_imu_b2.srcs += downlink.c pprz_transport.c

test_imu_b2.srcs += $(SRC_BOOZ)/booz_trig_int.c

test_imu_b2.CFLAGS += -DBOOZ2_IMU_TYPE=\"booz2_imu_b2.h\"
test_imu_b2.CFLAGS += -DSSP_VIC_SLOT=9
test_imu_b2.srcs += $(SRC_BOOZ)/booz2_imu_b2.c $(SRC_BOOZ_ARCH)/booz2_imu_b2_hw.c
test_imu_b2.CFLAGS += -DMAX1168_EOC_VIC_SLOT=8
test_imu_b2.srcs += $(SRC_BOOZ)/booz2_max1168.c $(SRC_BOOZ_ARCH)/booz2_max1168_hw.c
test_imu_b2.CFLAGS += -DFLOAT_T=float
test_imu_b2.srcs += $(SRC_BOOZ)/booz2_imu.c


#
# test rc spektrum
#

test_rc_spektrum.ARCHDIR = $(ARCH)

test_rc_spektrum.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG) -I$(SRC_BOOZ) $(BOOZ_CFLAGS)
test_rc_spektrum.CFLAGS += -DPERIPHERALS_AUTO_INIT
test_rc_spektrum.srcs += $(SRC_BOOZ_TEST)/booz2_test_radio_control.c
test_rc_spektrum.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))' -DTIME_LED=1
test_rc_spektrum.CFLAGS += -DUSE_LED
test_rc_spektrum.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c

#test_rc_spektrum.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600
#test_rc_spektrum.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c
#test_rc_spektrum.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart1
#test_rc_spektrum.srcs += downlink.c pprz_transport.c
test_rc_spektrum.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DUSE_USB_SERIAL
test_rc_spektrum.CFLAGS += -DDOWNLINK_DEVICE=UsbS -DPPRZ_UART=UsbS -DDATALINK=PPRZ
test_rc_spektrum.srcs += downlink.c $(SRC_ARCH)/usb_ser_hw.c pprz_transport.c
test_rc_spektrum.srcs += $(SRC_ARCH)/lpcusb/usbhw_lpc.c $(SRC_ARCH)/lpcusb/usbcontrol.c
test_rc_spektrum.srcs += $(SRC_ARCH)/lpcusb/usbstdreq.c $(SRC_ARCH)/lpcusb/usbinit.c

test_rc_spektrum.CFLAGS += -DRADIO_CONTROL -DRADIO_CONTROL_LED=1
test_rc_spektrum.CFLAGS += -DRADIO_CONTROL_TYPE_H=\"booz_radio_control_spektrum.h\"
test_rc_spektrum.CFLAGS += -DRADIO_CONTROL_SPEKTRUM_MODEL_H=\"booz_radio_control_spektrum_dx7se.h\"
test_rc_spektrum.CFLAGS += -DUSE_UART0 -DUART0_BAUD=B115200
test_rc_spektrum.CFLAGS += -DRADIO_CONTROL_LINK=Uart0
test_rc_spektrum.srcs += $(SRC_SUBSYSTEMS)/radio_control.c \
						 $(SRC_SUBSYSTEMS)/radio_control_spektrum.c \
						 $(SRC_ARCH)/mcu_periph/uart_arch.c

#
# test rc ppm
#

test_rc_ppm.ARCHDIR = $(ARCH)

test_rc_ppm.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG) -I$(SRC_BOOZ) -I$(SRC_BOOZ_ARCH) $(BOOZ_CFLAGS)
test_rc_ppm.CFLAGS += -DPERIPHERALS_AUTO_INIT
test_rc_ppm.srcs += $(SRC_BOOZ_TEST)/booz2_test_radio_control.c
test_rc_ppm.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))' -DTIME_LED=1
test_rc_ppm.CFLAGS += -DUSE_LED
test_rc_ppm.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c

#test_rc_ppm.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600
#test_rc_ppm.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c
#test_rc_ppm.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart1
#test_rc_ppm.srcs += downlink.c pprz_transport.c
test_rc_ppm.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DUSE_USB_SERIAL
test_rc_ppm.CFLAGS += -DDOWNLINK_DEVICE=UsbS -DPPRZ_UART=UsbS -DDATALINK=PPRZ
test_rc_ppm.srcs += downlink.c $(SRC_ARCH)/usb_ser_hw.c pprz_transport.c
test_rc_ppm.srcs += $(SRC_ARCH)/lpcusb/usbhw_lpc.c $(SRC_ARCH)/lpcusb/usbcontrol.c
test_rc_ppm.srcs += $(SRC_ARCH)/lpcusb/usbstdreq.c $(SRC_ARCH)/lpcusb/usbinit.c

test_rc_ppm.CFLAGS += -DRADIO_CONTROL -DRADIO_CONTROL_LED=1
test_rc_ppm.CFLAGS += -DRADIO_CONTROL_TYPE_H=\"booz_radio_control_ppm.h\"
test_rc_ppm.CFLAGS += -DRADIO_CONTROL_TYPE_PPM
test_rc_ppm.srcs += $(SRC_SUBSYSTEMS)/radio_control.c \
					$(SRC_BOOZ)/$(IMPL)/booz_radio_control_ppm.c \
					$(SRC_BOOZ)/$(IMPL)/$(ARCH)/booz_radio_control_ppm_arch.c \


#
# test MC
#
test_mc.ARCHDIR = $(ARCH)

test_mc.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG) -I$(SRC_BOOZ) -I$(SRC_BOOZ_ARCH)
test_mc.srcs += $(SRC_BOOZ_TEST)/booz2_test_mc.c
test_mc.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))' -DTIME_LED=1
test_mc.CFLAGS += -DUSE_LED
test_mc.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c

test_mc.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600
test_mc.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c

test_mc.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart1
test_mc.srcs += downlink.c pprz_transport.c

test_mc.CFLAGS += -DACTUATORS=\"actuators_buss_twi_blmc_hw.h\" -DUSE_BUSS_TWI_BLMC
test_mc.srcs += $(SRC_BOOZ_ARCH)/actuators_buss_twi_blmc_hw.c actuators.c
test_mc.CFLAGS += -DUSE_I2C0 -DI2C0_SCLL=150 -DI2C0_SCLH=150 -DI2C0_VIC_SLOT=10
test_mc.srcs += i2c.c $(SRC_ARCH)/i2c_hw.c



#
# test BUSS BLDC
#
test_buss_bldc.ARCHDIR = $(ARCH)

test_buss_bldc.CFLAGS += -DPERIPHERALS_AUTO_INIT
test_buss_bldc.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG) -I$(SRC_BOOZ) -I$(SRC_BOOZ_ARCH)
test_buss_bldc.srcs += $(SRC_BOOZ_TEST)/booz2_test_buss_bldc.c
test_buss_bldc.CFLAGS += -DUSE_LED
test_buss_bldc.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))' -DTIME_LED=1
test_buss_bldc.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c

test_buss_bldc.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600
test_buss_bldc.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c

test_buss_bldc.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart1
test_buss_bldc.srcs += downlink.c pprz_transport.c

test_buss_bldc.CFLAGS += -DUSE_I2C0 -DI2C0_SCLL=150 -DI2C0_SCLH=150 -DI2C0_VIC_SLOT=10
test_buss_bldc.srcs += i2c.c $(SRC_ARCH)/i2c_hw.c



#
# test asctec BLMC
#
test_amc.ARCHDIR = $(ARCH)

test_amc.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG) -I$(SRC_BOOZ) -I$(SRC_BOOZ_ARCH)
test_amc.srcs += $(SRC_BOOZ_TEST)/booz2_test_amc.c
test_amc.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))' -DTIME_LED=1
test_amc.CFLAGS += -DUSE_LED
test_amc.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c

test_amc.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600
test_amc.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c

test_amc.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart1
test_amc.srcs += downlink.c pprz_transport.c
test_amc.CFLAGS += -DDATALINK=PPRZ -DPPRZ_UART=Uart1
test_amc.srcs += $(SRC_FIRMWARE)/datalink.c

test_amc.CFLAGS += -DACTUATORS=\"actuators_asctec_twi_blmc_hw.h\"
test_amc.srcs += $(SRC_BOOZ_ARCH)/actuators_asctec_twi_blmc_hw.c actuators.c
test_amc.CFLAGS += -DUSE_I2C0 -DI2C0_SCLL=150 -DI2C0_SCLH=150 -DI2C0_VIC_SLOT=10
test_amc.srcs += i2c.c $(SRC_ARCH)/i2c_hw.c

test_amc.CFLAGS += -DFLOAT_T=float
#-DBOOZ2_IMU_TYPE=\"booz2_imu_crista.h\"



#
# test_mkk_bldc :
#
test_mkk_bldc.ARCHDIR = $(ARCH)

test_mkk_bldc.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG) $(BOOZ_CFLAGS)
test_mkk_bldc.CFLAGS += -DPERIPHERALS_AUTO_INIT
test_mkk_bldc.srcs   += $(SRC_BOOZ_TEST)/booz2_test_buss_bldc_hexa.c
test_mkk_bldc.CFLAGS += -DUSE_LED
test_mkk_bldc.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))' -DTIME_LED=1
test_mkk_bldc.srcs   += sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c
test_mkk_bldc.CFLAGS += -DUSE_I2C0 -DI2C0_SCLL=150 -DI2C0_SCLH=150 -DI2C0_VIC_SLOT=10
test_mkk_bldc.srcs += i2c.c $(SRC_ARCH)/i2c_hw.c


#
# test 24 bits baro
#
test_baro_24.ARCHDIR = $(ARCH)

test_baro_24.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG) -I$(SRC_BOOZ) $(BOOZ_CFLAGS)
test_baro_24.srcs += $(SRC_BOOZ_TEST)/booz2_test_baro_24.c
test_baro_24.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./5.))' -DTIME_LED=1
test_baro_24.CFLAGS += -DUSE_LED
test_baro_24.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c


test_baro_24.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600
test_baro_24.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c

test_baro_24.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart1
test_baro_24.srcs += downlink.c pprz_transport.c

test_baro_24.CFLAGS += -DUSE_I2C1  -DI2C1_SCLL=150 -DI2C1_SCLH=150 -DI2C1_VIC_SLOT=11 -DI2C1_BUF_LEN=16
test_baro_24.srcs += i2c.c $(SRC_ARCH)/i2c_hw.c
test_baro_24.srcs += $(SRC_BOOZ)/booz2_baro_24.c

#
# test_coders : blinks all leds
#
test_coder.ARCHDIR = $(ARCH)

test_coder.CFLAGS += -DBOARD_CONFIG=\"boards/olimex_lpc_h2148.h\" $(BOOZ_CFLAGS)
test_coder.CFLAGS += -DPERIPHERALS_AUTO_INIT
test_coder.srcs   += $(SRC_BOOZ_TEST)/booz2_test_coder.c
test_coder.CFLAGS += -DUSE_LED
test_coder.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))' -DTIME_LED=1
test_coder.srcs   += sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c
