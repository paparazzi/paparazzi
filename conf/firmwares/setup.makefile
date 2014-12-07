#
# setup.makefile
#
#

SRC_ARCH=arch/$(ARCH)
SRC_BOARD=boards/$(BOARD)
SRC_SUBSYSTEMS=subsystems
SRC_MODULES=modules

CFG_SHARED=$(PAPARAZZI_SRC)/conf/firmwares/subsystems/shared

SRC_FIRMWARE=firmwares/setup

#
# common setup
#
# configuration
#   SYS_TIME_LED
#   MODEM_PORT
#   MODEM_BAUD
#
PERIODIC_FREQUENCY ?= 512

COMMON_SETUP_CFLAGS  = -I$(SRC_BOARD) -DBOARD_CONFIG=$(BOARD_CFG)
COMMON_SETUP_CFLAGS += -DPERIPHERALS_AUTO_INIT
COMMON_SETUP_SRCS    = mcu.c $(SRC_ARCH)/mcu_arch.c
ifneq ($(SYS_TIME_LED),none)
  COMMON_SETUP_CFLAGS += -DSYS_TIME_LED=$(SYS_TIME_LED)
endif
COMMON_SETUP_CFLAGS += -DPERIODIC_FREQUENCY=$(PERIODIC_FREQUENCY)
COMMON_SETUP_SRCS   += mcu_periph/sys_time.c $(SRC_ARCH)/mcu_periph/sys_time_arch.c

COMMON_SETUP_CFLAGS += -DUSE_LED

ifeq ($(ARCH), lpc21)
COMMON_SETUP_SRCS += $(SRC_ARCH)/armVIC.c
else ifeq ($(ARCH), stm32)
COMMON_SETUP_SRCS += $(SRC_ARCH)/led_hw.c
COMMON_SETUP_SRCS += $(SRC_ARCH)/mcu_periph/gpio_arch.c
endif



#
# a test program to tunnel between both uart
#
tunnel.ARCHDIR = $(ARCH)
tunnel.CFLAGS += $(COMMON_SETUP_CFLAGS)
tunnel.srcs   += $(COMMON_SETUP_SRCS)
tunnel.srcs += $(SRC_ARCH)/uart_tunnel.c


#
# usb tunnel
#
# a configuration program to access a uart through usb
#
# configuration:
#    TUNNEL_PORT (defaults to GPS_PORT)
#    TUNNEL_BAUD
#
usb_tunnel.ARCHDIR = $(ARCH)
usb_tunnel.CFLAGS += $(COMMON_SETUP_CFLAGS)
usb_tunnel.srcs   += $(COMMON_SETUP_SRCS)
usb_tunnel.srcs   += mcu_periph/uart.c $(SRC_ARCH)/mcu_periph/uart_arch.c

TUNNEL_PORT ?= GPS_PORT
TUNNEL_PORT_LOWER=$(shell echo $(TUNNEL_PORT) | tr A-Z a-z)
TUNNEL_PORT_UPPER=$(shell echo $(TUNNEL_PORT) | tr a-z A-Z)
TUNNEL_BAUD ?= B115200

usb_tunnel.CFLAGS += -DUSE_$(TUNNEL_PORT_UPPER) -D$(TUNNEL_PORT_UPPER)_BAUD=$(TUNNEL_BAUD)
usb_tunnel.CFLAGS += -DUSB_TUNNEL_UART=$(TUNNEL_PORT_LOWER)
usb_tunnel.CFLAGS += -DUSE_USB_LINE_CODING -DUSE_USB_SERIAL
usb_tunnel.srcs += $(SRC_FIRMWARE)/usb_tunnel.c $(SRC_ARCH)/usb_ser_hw.c

TUNNEL_RX_LED ?= 2
TUNNEL_TX_LED ?= 3
ifneq ($(TUNNEL_TX_LED),none)
  usb_tunnel.CFLAGS += -DTUNNEL_TX_LED=$(TUNNEL_TX_LED)
endif
ifneq ($(TUNNEL_RX_LED),none)
 usb_tunnel.CFLAGS += -DTUNNEL_RX_LED=$(TUNNEL_RX_LED)
endif

ifeq ($(ARCH), lpc21)
# for the usb_tunnel we need to set PCLK higher with the flag USE_USB_HIGH_PCLK
usb_tunnel.CFLAGS += -DUSE_USB_HIGH_PCLK
usb_tunnel.srcs += $(SRC_ARCH)/lpcusb/usbhw_lpc.c $(SRC_ARCH)/lpcusb/usbinit.c
usb_tunnel.srcs += $(SRC_ARCH)/lpcusb/usbcontrol.c $(SRC_ARCH)/lpcusb/usbstdreq.c
endif



#
# setup actuators
#
setup_actuators.ARCHDIR = $(ARCH)
setup_actuators.CFLAGS += $(COMMON_SETUP_CFLAGS)
setup_actuators.srcs   += $(COMMON_SETUP_SRCS)

setup_actuators.srcs   += mcu_periph/uart.c $(SRC_ARCH)/mcu_periph/uart_arch.c
setup_actuators.CFLAGS += -DUSE_$(MODEM_PORT)
setup_actuators.CFLAGS += -D$(MODEM_PORT)_BAUD=$(MODEM_BAUD)
SETUP_ACTUATORS_MODEM_PORT_LOWER=$(shell echo $(MODEM_PORT) | tr A-Z a-z)
setup_actuators.CFLAGS += -DDOWNLINK -DDOWNLINK_DEVICE=$(SETUP_ACTUATORS_MODEM_PORT_LOWER) -DPPRZ_UART=$(MODEM_PORT)
setup_actuators.CFLAGS += -DDOWNLINK_TRANSPORT=pprz_tp -DDATALINK=PPRZ
setup_actuators.srcs += subsystems/datalink/downlink.c subsystems/datalink/pprz_transport.c

setup_actuators.srcs   += subsystems/actuators.c
setup_actuators.srcs   += $(SRC_FIRMWARE)/setup_actuators.c

ifeq ($(TARGET), setup_actuators)
  ifeq ($(ACTUATORS),)
    $(error ACTUATORS not configured, if your board file has no default, configure in your airframe file)
  else
    include $(CFG_SHARED)/$(ACTUATORS).makefile
  endif
endif
