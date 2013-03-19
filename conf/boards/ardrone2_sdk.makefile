# Hey Emacs, this is a -*- makefile -*-
#
# ardrone2.makefile
#
# http://paparazzi.enac.fr/wiki/AR.Drone_2_-_Specifications
#

BOARD=ardrone
BOARD_VERSION=2
BOARD_TYPE=sdk
BOARD_CFG=\"boards/$(BOARD)$(BOARD_VERSION)_$(BOARD_TYPE).h\"

ARCH=omap_ardrone2
$(TARGET).ARCHDIR = $(ARCH)

# -----------------------------------------------------------------------
USER=foobar
HOST=192.168.1.1
SUB_DIR=sdk
FTP_DIR=/data/video
TARGET_DIR=$(FTP_DIR)/$(SUB_DIR)
# -----------------------------------------------------------------------

# The GPS sensor is connected trough USB, we have to fix this
ifndef GPS_PORT
GPS_PORT=UART1
endif

ifndef GPS_BAUD
GPS_BAUD=B38400
endif

# This is a (temporary) fix for uart_arch.c to compile with a device name
$(TARGET).CFLAGS += -DUART1_DEV=\"/dev/ttyUSB0\"
# -----------------------------------------------------------------------
