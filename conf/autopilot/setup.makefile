#
# setup.makefile
#
#


CFG_SHARED=$(PAPARAZZI_SRC)/conf/autopilot/subsystems/shared
#CFG_SETUP=$(PAPARAZZI_SRC)/conf/autopilot/subsystems/setup

SRC_ARCH=arch/$(ARCH)
SRC_FIRMWARE=firmwares/setup
SRC_LISA=lisa

SETUP_INC = -I$(SRC_FIRMWARE)

$(TARGET).CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)

# a test program to tunnel between both uart
ifeq ($(ARCH), lpc21)
tunnel.CFLAGS += -DUSE_LED
tunnel.srcs += $(SRC_ARCH)/uart_tunnel.c
tunnel.srcs += mcu.c $(SRC_ARCH)/mcu_arch.c
else ifeq ($(ARCH), stm32)
tunnel.ARCHDIR = $(ARCH)
tunnel.CFLAGS += -I$(SRC_LISA) -I$(ARCH) -DPERIPHERALS_AUTO_INIT
tunnel.srcs   += mcu.c \
                 $(SRC_ARCH)/mcu_arch.c \
                 $(SRC_LISA)/tunnel_hw.c          \
                 $(SRC_ARCH)/stm32_exceptions.c   \
                 $(SRC_ARCH)/stm32_vector_table.c
tunnel.CFLAGS += -DUSE_LED
tunnel.srcs   += $(SRC_ARCH)/led_hw.c
tunnel.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=$(SYS_TIME_LED)
tunnel.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
tunnel.srcs   += sys_time.c $(SRC_ARCH)/sys_time_hw.c
endif


# for the usb_tunnel we need to set PCLK higher with the flag USE_USB_HIGH_PCLK

# a configuration program to access both uart through usb
ifeq ($(ARCH), lpc21)
usb_tunnel_0.CFLAGS += -DUSE_UART0 -DUART0_BAUD=B115200 -DPERIPHERALS_AUTO_INIT
usb_tunnel_0.CFLAGS += -DUSE_USB_LINE_CODING -DUSE_USB_SERIAL -DUSE_LED -DUSE_USB_HIGH_PCLK
usb_tunnel_0.srcs += $(SRC_ARCH)/usb_tunnel.c $(SRC_ARCH)/usb_ser_hw.c mcu_periph/uart.c $(SRC_ARCH)/mcu_periph/uart_arch.c
usb_tunnel_0.srcs += $(SRC_ARCH)/lpcusb/usbhw_lpc.c $(SRC_ARCH)/lpcusb/usbinit.c
usb_tunnel_0.srcs += $(SRC_ARCH)/lpcusb/usbcontrol.c $(SRC_ARCH)/lpcusb/usbstdreq.c
usb_tunnel_0.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c
usb_tunnel_0.srcs += mcu.c $(SRC_ARCH)/mcu_arch.c

usb_tunnel_1.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B115200 -DPERIPHERALS_AUTO_INIT
usb_tunnel_1.CFLAGS += -DUSE_USB_LINE_CODING -DUSE_USB_SERIAL -DUSE_LED -DUSE_USB_HIGH_PCLK
usb_tunnel_1.srcs += $(SRC_ARCH)/usb_tunnel.c $(SRC_ARCH)/usb_ser_hw.c mcu_periph/uart.c $(SRC_ARCH)/mcu_periph/uart_arch.c
usb_tunnel_1.srcs += $(SRC_ARCH)/lpcusb/usbhw_lpc.c $(SRC_ARCH)/lpcusb/usbinit.c
usb_tunnel_1.srcs += $(SRC_ARCH)/lpcusb/usbcontrol.c $(SRC_ARCH)/lpcusb/usbstdreq.c
usb_tunnel_1.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c
usb_tunnel_1.srcs += mcu.c $(SRC_ARCH)/mcu_arch.c
else
ifeq ($(TARGET),usb_tunnel_0)
$(error usb_tunnel_0 currently only implemented for the lpc21)
else ifeq ($(TARGET),usb_tunnel_1)
$(error usb_tunnel_1 currently only implemented for the lpc21)
endif
endif



ifeq ($(TARGET), setup_actuators)
  ifeq ($(ACTUATORS),)
    ifeq ($(BOARD),tiny)
      ifeq ($(BOARD_VERSION),1.1)
        include $(CFG_SHARED)/actuators_4015.makefile
      else
        ifeq ($(BOARD_VERSION),0.99)
          include $(CFG_SHARED)/actuators_4015.makefile
        else
          include $(CFG_SHARED)/actuators_4017.makefile
        endif
      endif
    endif
    ifeq ($(BOARD),twog)
      include $(CFG_SHARED)/actuators_4017.makefile
    endif

    ifeq ($(BOARD),lisa_l)
      include $(CFG_SHARED)/actuators_direct.makefile
    endif

  else
    include $(CFG_SHARED)/$(ACTUATORS).makefile
  endif
endif


# a test program to setup actuators
ifeq ($(ARCH), lpc21)
setup_actuators.CFLAGS += -DFBW -DUSE_LED -DTIME_LED=1
setup_actuators.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600 -DDOWNLINK_DEVICE=Uart1 -DPPRZ_UART=Uart1
setup_actuators.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDATALINK=PPRZ
setup_actuators.CFLAGS += -DDOWNLINK_FBW_DEVICE=Uart1 -DDOWNLINK_AP_DEVICE=Uart1
setup_actuators.CFLAGS += $(SETUP_INC) -Ifirmwares/fixedwing
setup_actuators.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c pprz_transport.c downlink.c $(SRC_FIRMWARE)/setup_actuators.c mcu_periph/uart.c $(SRC_ARCH)/mcu_periph/uart_arch.c firmwares/fixedwing/main.c mcu.c $(SRC_ARCH)/mcu_arch.c
else ifeq ($(TARGET),setup_actuators)
$(error setup_actuators currently only implemented for the lpc21)
endif
