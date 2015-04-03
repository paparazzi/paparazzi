# Hey Emacs, this is a -*- makefile -*-

#serial USB (e.g. /dev/ttyACM0)

telemetry_CFLAGS = -DDOWNLINK -DDOWNLINK_DEVICE=usb_serial -DPPRZ_UART=usb_serial
telemetry_CFLAGS += -DDOWNLINK_TRANSPORT=pprz_tp -DDATALINK=PPRZ -DUSE_USB_SERIAL
telemetry_srcs = subsystems/datalink/downlink.c subsystems/datalink/pprz_transport.c subsystems/datalink/telemetry.c
telemetry_srcs += $(SRC_FIRMWARE)/datalink.c $(SRC_FIRMWARE)/ap_downlink.c $(SRC_FIRMWARE)/fbw_downlink.c
# avoid fbw_telemetry_mode error
telemetry_srcs += $(SRC_FIRMWARE)/fbw_downlink.c

ap.CFLAGS += $(telemetry_CFLAGS)
ap.srcs += $(telemetry_srcs)

fbw.srcs += $(SRC_FIRMWARE)/fbw_downlink.c

ifeq ($(ARCH), lpc21)
ap.CFLAGS += -DUSE_USB_HIGH_PCLK
ap.srcs += $(SRC_ARCH)/usb_ser_hw.c $(SRC_ARCH)/lpcusb/usbhw_lpc.c $(SRC_ARCH)/lpcusb/usbcontrol.c
ap.srcs += $(SRC_ARCH)/lpcusb/usbstdreq.c $(SRC_ARCH)/lpcusb/usbinit.c
else
ifeq ($(ARCH), stm32)
ap.srcs += $(SRC_ARCH)/usb_ser_hw.c
else
ifneq ($(ARCH), sim)
$(error telemetry_transparent_usb currently only implemented for the lpc21 and stm32)
endif
endif
endif
