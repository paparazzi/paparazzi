#
# setup.makefile
#
#


CFG_SHARED=$(PAPARAZZI_SRC)/conf/firmwares/subsystems/shared

SRC_ARCH=arch/$(ARCH)
SRC_FIRMWARE=firmwares/logger

SETUP_INC = -I$(SRC_FIRMWARE)

$(TARGET).CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)

# default config
ifndef SPI_CHANNEL
SPI_CHANNEL = 1
endif

ifndef UART0_BAUD
UART0_BAUD = B9600
endif

ifndef UART1_BAUD
UART1_BAUD = B9600
endif

ifndef LOG_STOP_KEY
LOG_STOP_KEY = 7
endif


# a configuration program to access both uart through usb
ifeq ($(ARCH), lpc21)


ap.CFLAGS += -DUSE_LED -DPERIPHERALS_AUTO_INIT
ap.srcs = mcu_periph/sys_time.c $(SRC_ARCH)/mcu_periph/sys_time_arch.c $(SRC_ARCH)/armVIC.c $(SRC_FIRMWARE)/main_logger.c

# PPRZ message format is default
ifndef LOG_MSG_FMT
LOG_MSG_FMT = LOG_PPRZ
endif

#set the speed
ap.CFLAGS += -DUSE_UART0 -DUART0_BAUD=$(UART0_BAUD) -DUSE_UART0_TX=FALSE -DPERIPHERALS_AUTO_INIT
ap.CFLAGS += -DUSE_UART1 -DUART1_BAUD=$(UART1_BAUD) -DUSE_UART1_TX=FALSE
ap.CFLAGS += -DLOG_STOP_KEY=$(LOG_STOP_KEY)
ap.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c
ap.srcs += mcu_periph/uart.c
ap.srcs += $(SRC_ARCH)/mcu_arch.c
ap.srcs += mcu.c

#set SPI interface for SD card (0 or 1)
ap.CFLAGS += -DHW_ENDPOINT_LPC2000_SPINUM=$(SPI_CHANNEL)

#message format pprz/xbee
ap.CFLAGS += -D$(LOG_MSG_FMT)

#LPC2148 USB hw module needs at least 18MHz PCLK
ap.CFLAGS += -DUSE_USB_HIGH_PCLK

#efsl
ap.CFLAGS += -I$(SRC_ARCH)/efsl/inc -I$(SRC_ARCH)/efsl/conf

ap.srcs += $(SRC_ARCH)/efsl/src/efs.c $(SRC_ARCH)/efsl/src/plibc.c
ap.srcs += $(SRC_ARCH)/efsl/src/disc.c $(SRC_ARCH)/efsl/src/partition.c
ap.srcs += $(SRC_ARCH)/efsl/src/time.c $(SRC_ARCH)/efsl/src/fs.c
ap.srcs += $(SRC_ARCH)/efsl/src/fat.c $(SRC_ARCH)/efsl/src/file.c
ap.srcs += $(SRC_ARCH)/efsl/src/dir.c $(SRC_ARCH)/efsl/src/ls.c
ap.srcs += $(SRC_ARCH)/efsl/src/mkfs.c $(SRC_ARCH)/efsl/src/debug.c
ap.srcs += $(SRC_ARCH)/efsl/src/ioman.c $(SRC_ARCH)/efsl/src/ui.c
ap.srcs += $(SRC_ARCH)/efsl/src/extract.c
ap.srcs += $(SRC_ARCH)/efsl/src/interfaces/lpc2000_spi.c
ap.srcs += $(SRC_ARCH)/efsl/src/interfaces/sd.c

#usb mass storage
ap.CFLAGS += -DUSE_USB_MSC
ap.CFLAGS += -I$(SRC_ARCH)/lpcusb -I$(SRC_ARCH)/lpcusb/examples

ap.srcs += $(SRC_ARCH)/usb_msc_hw.c
ap.srcs += $(SRC_ARCH)/lpcusb/usbhw_lpc.c $(SRC_ARCH)/lpcusb/usbcontrol.c
ap.srcs += $(SRC_ARCH)/lpcusb/usbstdreq.c $(SRC_ARCH)/lpcusb/usbinit.c
ap.srcs += $(SRC_ARCH)/lpcusb/examples/msc_bot.c
ap.srcs += $(SRC_ARCH)/lpcusb/examples/msc_scsi.c
ap.srcs += $(SRC_ARCH)/lpcusb/examples/blockdev_sd.c
ifeq ($(SPI_CHANNEL), 1)
ap.srcs += $(SRC_ARCH)/lpcusb/examples/lpc2000_spi.c
else
ap.srcs += $(SRC_ARCH)/lpcusb/examples/lpc2000_spi0.c
endif

else
$(error logger currently only implemented for the lpc21)
endif


