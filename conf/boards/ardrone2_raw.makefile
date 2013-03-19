# Hey Emacs, this is a -*- makefile -*-
#
# ardrone2.makefile
#
# http://paparazzi.enac.fr/wiki/AR.Drone_2_-_Specifications
#

BOARD=ardrone
BOARD_VERSION=2
BOARD_TYPE=raw
BOARD_CFG=\"boards/$(BOARD)$(BOARD_VERSION)_$(BOARD_TYPE).h\"

ARCH=omap_ardrone2
$(TARGET).ARCHDIR = $(ARCH)

# -----------------------------------------------------------------------
USER=foobar
HOST=192.168.1.1
SUB_DIR=raw
FTP_DIR=/data/video
TARGET_DIR=$(FTP_DIR)/$(SUB_DIR)
# -----------------------------------------------------------------------

# Do we need to disable modem? We don't have a modem.
#ifndef MODEM_PORT
#MODEM_PORT=UART0
#endif

#ifndef MODEM_BAUD
#MODEM_BAUD=B57600
#endif

# The GPS sensor is connected trough USB, we have to fix this
ifndef GPS_PORT
GPS_PORT=UART1
endif

ifndef GPS_BAUD
GPS_BAUD=B38400
endif

# This is a (temporary) fix for uart_arch.c to compile with a device name
#$(TARGET).CFLAGS += -DUART0_DEV=\"/dev/ttyO3\"
$(TARGET).CFLAGS += -DUART1_DEV=\"/dev/ttyACM0\"

# for telemetry
$(TARGET).CFLAGS += -DARDRONE_NAVDATA

# -----------------------------------------------------------------------

# default LED configuration

ifndef RADIO_CONTROL_LED
RADIO_CONTROL_LED  = none
endif

ifndef BARO_LED
BARO_LED = none
endif

ifndef AHRS_ALIGNER_LED
AHRS_ALIGNER_LED = none
endif

ifndef GPS_LED
GPS_LED = none
endif

ifndef SYS_TIME_LED
SYS_TIME_LED = none
endif

# -----------------------------------------------------------------------
