
#serial USB (e.g. /dev/ttyACM0)

$(TARGET).CFLAGS += -DDOWNLINK -DDOWNLINK_DEVICE=usb_serial -DPPRZ_UART=usb_serial
$(TARGET).CFLAGS += -DDOWNLINK_TRANSPORT=pprz_tp -DDATALINK=PPRZ -DUSE_USB_SERIAL
$(TARGET).CFLAGS += -DPERIODIC_TELEMETRY
$(TARGET).srcs += subsystems/datalink/downlink.c subsystems/datalink/datalink.c pprzlink/src/pprz_transport.c subsystems/datalink/telemetry.c

ifeq ($(ARCH), lpc21)
$(TARGET).srcs += $(SRC_ARCH)/usb_ser_hw.c $(SRC_ARCH)/lpcusb/usbhw_lpc.c $(SRC_ARCH)/lpcusb/usbcontrol.c
$(TARGET).srcs += $(SRC_ARCH)/lpcusb/usbstdreq.c $(SRC_ARCH)/lpcusb/usbinit.c
else
ifeq ($(ARCH), stm32)
$(TARGET).srcs += $(SRC_ARCH)/usb_ser_hw.c
else
ifneq ($(ARCH), sim)
$(error telemetry_transparent_usb currently only implemented for the lpc21 and stm32)
endif
endif
endif
