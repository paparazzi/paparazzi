# Hey Emacs, this is a -*- makefile -*-


#serial USB (e.g. /dev/ttyACM0)

ifeq ($(ARCH), lpc21)
ap.CFLAGS += -DDOWNLINK -DDOWNLINK_DEVICE=UsbS -DPPRZ_UART=UsbS
ap.CFLAGS += -DDOWNLINK_TRANSPORT=PprzTransport -DDATALINK=PPRZ -DUSE_USB_SERIAL -DUSE_USB_HIGH_PCLK
ap.srcs += subsystems/datalink/downlink.c subsystems/datalink/pprz_transport.c
ap.srcs += $(SRC_FIRMWARE)/datalink.c
ap.srcs += $(SRC_ARCH)/usb_ser_hw.c $(SRC_ARCH)/lpcusb/usbhw_lpc.c $(SRC_ARCH)/lpcusb/usbcontrol.c
ap.srcs += $(SRC_ARCH)/lpcusb/usbstdreq.c $(SRC_ARCH)/lpcusb/usbinit.c
else
ifneq ($(ARCH), sim)
$(error telemetry_transparent_usb currently only implemented for the lpc21)
endif
endif

