#
# setup.makefile
#
#


CFG_SETUP=$(PAPARAZZI_SRC)/conf/autopilot/subsystems/SETUP


SRC_SETUP=.
SRC_SETUP_ARCH=$(SRC_SETUP)/$(ARCH)
SRC_SETUP_TEST=$(SRC_SETUP)/

SETUP_INC = -I$(SRC_SETUP) -I$(SRC_SETUP_ARCH)


# for the usb_tunnel we need to set PCLK higher
# so we just use tiny_2_1_1_usb.h as board file for the usb tunnels
ifeq ($(TARGET), usb_tunnel_0)
  usb_tunnel_0.CFLAGS += -DBOARD_CONFIG=\"tiny_2_1_1_usb.h\"
else
  ifeq ($(TARGET), usb_tunnel_1)
    usb_tunnel_1.CFLAGS += -DBOARD_CONFIG=\"tiny_2_1_1_usb.h\"
  else
    $(TARGET).CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
  endif
endif


# a test program to tunnel between both uart
tunnel.CFLAGS += -DFBW -DLED
tunnel.srcs += $(SRC_ARCH)/uart_tunnel.c


# a configuration program to access both uart through usb
usb_tunnel_0.CFLAGS += -DFBW -DUSE_UART0 -DUART0_BAUD=B115200
usb_tunnel_0.CFLAGS += -DUSE_USB_LINE_CODING -DUSE_USB_SERIAL -DLED
usb_tunnel_0.srcs += $(SRC_ARCH)/usb_tunnel.c $(SRC_ARCH)/usb_ser_hw.c $(SRC_ARCH)/uart_hw.c
usb_tunnel_0.srcs += $(SRC_ARCH)/lpcusb/usbhw_lpc.c $(SRC_ARCH)/lpcusb/usbinit.c
usb_tunnel_0.srcs += $(SRC_ARCH)/lpcusb/usbcontrol.c $(SRC_ARCH)/lpcusb/usbstdreq.c
usb_tunnel_0.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c

usb_tunnel_1.CFLAGS += -DFBW -DUSE_UART1 -DUART1_BAUD=B115200
usb_tunnel_1.CFLAGS += -DUSE_USB_LINE_CODING -DUSE_USB_SERIAL -DLED
usb_tunnel_1.srcs += $(SRC_ARCH)/usb_tunnel.c $(SRC_ARCH)/usb_ser_hw.c $(SRC_ARCH)/uart_hw.c
usb_tunnel_1.srcs += $(SRC_ARCH)/lpcusb/usbhw_lpc.c $(SRC_ARCH)/lpcusb/usbinit.c
usb_tunnel_1.srcs += $(SRC_ARCH)/lpcusb/usbcontrol.c $(SRC_ARCH)/lpcusb/usbstdreq.c
usb_tunnel_1.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c



# a test program to setup actuators
setup_actuators.CFLAGS += -DFBW -DLED -DTIME_LED=1 -DACTUATORS=\"servos_4017_hw.h\" -DSERVOS_4017
setup_actuators.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600 -DDOWNLINK_DEVICE=Uart1 -DPPRZ_UART=Uart1
setup_actuators.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDATALINK=PPRZ
setup_actuators.CFLAGS += -DDOWNLINK_FBW_DEVICE=Uart1 -DDOWNLINK_AP_DEVICE=Uart1
setup_actuators.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c pprz_transport.c downlink.c actuators.c setup_actuators.c $(SRC_ARCH)/uart_hw.c $(SRC_ARCH)/servos_4017_hw.c main.c
