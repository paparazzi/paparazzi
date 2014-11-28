
#serial USB (e.g. /dev/ttyACM0)

ifeq ($(ARCH), lpc21)
ap.CFLAGS += -DDOWNLINK -DPERIODIC_TELEMETRY -DDOWNLINK_DEVICE=usb_serial -DPPRZ_UART=UsbS
ap.CFLAGS += -DDOWNLINK_TRANSPORT=pprz_tp -DDATALINK=PPRZ -DUSE_USB_SERIAL -DDefaultPeriodic='&telemetry_Main'
ap.srcs += subsystems/datalink/downlink.c subsystems/datalink/pprz_transport.c subsystems/datalink/telemetry.c
ap.srcs += $(SRC_FIRMWARE)/datalink.c $(SRC_FIRMWARE)/rotorcraft_telemetry.c
ap.srcs += $(SRC_ARCH)/usb_ser_hw.c $(SRC_ARCH)/lpcusb/usbhw_lpc.c $(SRC_ARCH)/lpcusb/usbcontrol.c
ap.srcs += $(SRC_ARCH)/lpcusb/usbstdreq.c $(SRC_ARCH)/lpcusb/usbinit.c
else
ifeq ($(ARCH), stm32)
ap.CFLAGS += -DDOWNLINK -DPERIODIC_TELEMETRY -DDOWNLINK_DEVICE=usb_serial -DPPRZ_UART=UsbS
ap.CFLAGS += -DDOWNLINK_TRANSPORT=pprz_tp -DDATALINK=PPRZ -DUSE_USB_SERIAL -DDefaultPeriodic='&telemetry_Main'
ap.srcs += subsystems/datalink/downlink.c subsystems/datalink/pprz_transport.c subsystems/datalink/telemetry.c
ap.srcs += $(SRC_FIRMWARE)/datalink.c $(SRC_FIRMWARE)/rotorcraft_telemetry.c
ap.srcs += $(SRC_ARCH)/usb_ser_hw.c
else
ifneq ($(ARCH), sim)
$(error telemetry_transparent_usb currently only implemented for the lpc21 and stm32)
endif
endif
endif
