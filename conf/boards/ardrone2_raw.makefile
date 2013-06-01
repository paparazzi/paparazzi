# Hey Emacs, this is a -*- makefile -*-
#
# ardrone2_raw.makefile
#
# http://paparazzi.enac.fr/wiki/AR.Drone_2_-_Specifications
#

BOARD=ardrone
BOARD_VERSION=2
BOARD_TYPE=raw
BOARD_CFG=\"boards/$(BOARD)$(BOARD_VERSION)_$(BOARD_TYPE).h\"

ARCH=omap
$(TARGET).ARCHDIR = $(ARCH)

# -----------------------------------------------------------------------
USER=foobar
HOST=192.168.1.1
SUB_DIR=raw
FTP_DIR=/data/video
TARGET_DIR=$(FTP_DIR)/$(SUB_DIR)
# -----------------------------------------------------------------------

# The GPS sensor is connected trough USB so we have to define the device
GPS_PORT           ?= UART1
GPS_BAUD           ?= B57600

# Here we define what the UART1_DEV device mapping
$(TARGET).CFLAGS   += -DUART1_DEV=\"/dev/ttyUSB0\"
#$(TARGET).CFLAGS  += -DUART0_DEV=\"/dev/ttyO3\"

# for distinction between RAW and SDK version
$(TARGET).CFLAGS +=-DARDRONE2_RAW

# -----------------------------------------------------------------------

# default LED configuration
RADIO_CONTROL_LED  ?= none
BARO_LED           ?= none
AHRS_ALIGNER_LED   ?= none
GPS_LED            ?= none
SYS_TIME_LED       ?= none
