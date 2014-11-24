# Hey Emacs, this is a -*- makefile -*-
#
# ardrone2_raw.makefile
#
# http://wiki.paparazziuav.org/wiki/AR.Drone_2_-_Specifications
#

BOARD=ardrone
BOARD_VERSION=2
BOARD_TYPE=raw
BOARD_CFG=\"boards/$(BOARD)$(BOARD_VERSION)_$(BOARD_TYPE).h\"

ARCH=linux
$(TARGET).ARCHDIR = $(ARCH)
# include conf/Makefile.ardrone2 (with specific upload rules) instead of only Makefile.linux:
ap.MAKEFILE = ardrone2

# -----------------------------------------------------------------------
USER=foobar
HOST?=192.168.1.1
SUB_DIR=raw
FTP_DIR=/data/video
TARGET_DIR=$(FTP_DIR)/$(SUB_DIR)
# -----------------------------------------------------------------------
ARDRONE2_START_PAPARAZZI ?= 0
ARDRONE2_WIFI_MODE ?= 0
ARDRONE2_SSID ?= ardrone2_paparazzi
ARDRONE2_IP_ADDRESS_BASE ?= 192.168.1.
ARDRONE2_IP_ADDRESS_PROBE ?= 1
# -----------------------------------------------------------------------

# The GPS sensor is connected trough USB so we have to define the device
GPS_PORT           ?= UART1
GPS_BAUD           ?= B57600

# The datalink default uses UDP
MODEM_HOST         ?= \"192.168.1.255\"

# Here we define what the UART1_DEV device mapping
$(TARGET).CFLAGS   += -DUART1_DEV=\"/dev/ttyUSB0\"
#$(TARGET).CFLAGS  += -DUART0_DEV=\"/dev/ttyO3\"

# for distinction between RAW and SDK version
$(TARGET).CFLAGS +=-DARDRONE2_RAW

# -----------------------------------------------------------------------

# default LED configuration
RADIO_CONTROL_LED			?= 6
BARO_LED           			?= none
AHRS_ALIGNER_LED   			?= 5
GPS_LED            			?= 3
SYS_TIME_LED       			?= 0
