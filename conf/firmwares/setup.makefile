#
# setup.makefile
#
#


CFG_SHARED=$(PAPARAZZI_SRC)/conf/firmwares/subsystems/shared
#CFG_SETUP=$(PAPARAZZI_SRC)/conf/firmwares/subsystems/setup

SRC_ARCH=arch/$(ARCH)
SRC_FIRMWARE=firmwares/setup
SRC_LISA=lisa

SETUP_INC = -I$(SRC_FIRMWARE)

$(TARGET).CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)

# a test program to tunnel between both uart
tunnel.CFLAGS += -DUSE_LED
tunnel.srcs += $(SRC_ARCH)/uart_tunnel.c
tunnel.srcs += mcu.c $(SRC_ARCH)/mcu_arch.c
ifeq ($(ARCH), stm32)
tunnel.ARCHDIR = $(ARCH)
tunnel.CFLAGS += -I$(ARCH) -DPERIPHERALS_AUTO_INIT
tunnel.srcs   += $(SRC_ARCH)/mcu_periph/gpio_arch.c $(SRC_ARCH)/led_hw.c
ifneq ($(SYS_TIME_LED),none)
tunnel.CFLAGS += -DSYS_TIME_LED=$(SYS_TIME_LED)
endif
tunnel.CFLAGS += -DPERIODIC_FREQUENCY='512.'
tunnel.CFLAGS += -DUSE_SYS_TIME
tunnel.srcs   += mcu_periph/sys_time.c $(SRC_ARCH)/mcu_periph/sys_time_arch.c
endif


# for the usb_tunnel we need to set PCLK higher with the flag USE_USB_HIGH_PCLK

# a configuration program to access both uart through usb
ifeq ($(ARCH), lpc21)
usb_tunnel_0.CFLAGS += -DUSE_UART0 -DUART0_BAUD=B115200 -DPERIPHERALS_AUTO_INIT
usb_tunnel_0.CFLAGS += -DUSE_USB_LINE_CODING -DUSE_USB_SERIAL -DUSE_LED -DUSE_USB_HIGH_PCLK
usb_tunnel_0.srcs += $(SRC_ARCH)/usb_tunnel.c $(SRC_ARCH)/usb_ser_hw.c mcu_periph/uart.c $(SRC_ARCH)/mcu_periph/uart_arch.c
usb_tunnel_0.srcs += $(SRC_ARCH)/lpcusb/usbhw_lpc.c $(SRC_ARCH)/lpcusb/usbinit.c
usb_tunnel_0.srcs += $(SRC_ARCH)/lpcusb/usbcontrol.c $(SRC_ARCH)/lpcusb/usbstdreq.c
usb_tunnel_0.srcs += mcu_periph/sys_time.c $(SRC_ARCH)/mcu_periph/sys_time_arch.c $(SRC_ARCH)/armVIC.c
usb_tunnel_0.srcs += mcu.c $(SRC_ARCH)/mcu_arch.c

usb_tunnel_1.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B115200 -DPERIPHERALS_AUTO_INIT
usb_tunnel_1.CFLAGS += -DUSE_USB_LINE_CODING -DUSE_USB_SERIAL -DUSE_LED -DUSE_USB_HIGH_PCLK
usb_tunnel_1.srcs += $(SRC_ARCH)/usb_tunnel.c $(SRC_ARCH)/usb_ser_hw.c mcu_periph/uart.c $(SRC_ARCH)/mcu_periph/uart_arch.c
usb_tunnel_1.srcs += $(SRC_ARCH)/lpcusb/usbhw_lpc.c $(SRC_ARCH)/lpcusb/usbinit.c
usb_tunnel_1.srcs += $(SRC_ARCH)/lpcusb/usbcontrol.c $(SRC_ARCH)/lpcusb/usbstdreq.c
usb_tunnel_1.srcs += mcu_periph/sys_time.c $(SRC_ARCH)/mcu_periph/sys_time_arch.c $(SRC_ARCH)/armVIC.c
usb_tunnel_1.srcs += mcu.c $(SRC_ARCH)/mcu_arch.c
else
ifeq ($(TARGET),usb_tunnel_0)
$(error usb_tunnel_0 currently only implemented for the lpc21)
else ifeq ($(TARGET),usb_tunnel_1)
$(error usb_tunnel_1 currently only implemented for the lpc21)
endif
endif



PERIODIC_FREQUENCY ?= 512

ifeq ($(TARGET), setup_actuators)
  ifeq ($(ACTUATORS),)
    $(error ACTUATORS not configured, if your board file has no default, configure in your airframe file)
  else
    include $(CFG_SHARED)/$(ACTUATORS).makefile
  endif
endif

# a test program to setup actuators
setup_actuators.CFLAGS += -DUSE_LED -DPERIPHERALS_AUTO_INIT
setup_actuators.srcs   += mcu.c $(SRC_ARCH)/mcu_arch.c

setup_actuators.CFLAGS += -DUSE_SYS_TIME
ifneq ($(SYS_TIME_LED),none)
setup_actuators.CFLAGS += -DSYS_TIME_LED=$(SYS_TIME_LED)
endif
setup_actuators.srcs   += mcu_periph/sys_time.c $(SRC_ARCH)/mcu_periph/sys_time_arch.c

setup_actuators.CFLAGS += -DUSE_$(MODEM_PORT)
setup_actuators.CFLAGS += -D$(MODEM_PORT)_BAUD=$(MODEM_BAUD)
setup_actuators.srcs   += mcu_periph/uart.c $(SRC_ARCH)/mcu_periph/uart_arch.c

setup_actuators.CFLAGS += -DDOWNLINK -DDOWNLINK_DEVICE=$(MODEM_PORT) -DPPRZ_UART=$(MODEM_PORT)
setup_actuators.CFLAGS += -DDOWNLINK_TRANSPORT=PprzTransport -DDATALINK=PPRZ
setup_actuators.srcs += subsystems/datalink/downlink.c subsystems/datalink/pprz_transport.c
# we actually don't really use the generated periodic telemetry in this firmware,
# but still needed to register e.g. the UART_ERRORS message #if DOWNLINK
setup_actuators.CFLAGS += -DDefaultPeriodic='&telemetry_Main'
setup_actuators.srcs += subsystems/datalink/telemetry.c

setup_actuators.CFLAGS += -DPERIODIC_FREQUENCY=$(PERIODIC_FREQUENCY)
setup_actuators.srcs   += subsystems/actuators.c
setup_actuators.srcs   += $(SRC_FIRMWARE)/setup_actuators.c

ifeq ($(ARCH), lpc21)
setup_actuators.srcs += $(SRC_ARCH)/armVIC.c
else ifeq ($(ARCH), stm32)
setup_actuators.ARCHDIR = $(ARCH)
setup_actuators.CFLAGS += -I$(ARCH)
setup_actuators.srcs   += $(SRC_ARCH)/led_hw.c
setup_actuators.srcs   += $(SRC_ARCH)/mcu_periph/gpio_arch.c
endif

