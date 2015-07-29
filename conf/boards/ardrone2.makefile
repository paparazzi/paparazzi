# Hey Emacs, this is a -*- makefile -*-
#
# ardrone2.makefile
#
# http://wiki.paparazziuav.org/wiki/AR.Drone_2_-_Specifications
#

BOARD=ardrone
BOARD_VERSION=2
BOARD_CFG=\"boards/$(BOARD)$(BOARD_VERSION).h\"

ARCH=linux
$(TARGET).ARCHDIR = $(ARCH)
# include conf/Makefile.ardrone2 (with specific upload rules) instead of only Makefile.linux:
ap.MAKEFILE = ardrone2

# -----------------------------------------------------------------------
USER=foobar
HOST?=192.168.1.1
SUB_DIR=paparazzi
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
MODEM_HOST         ?= 192.168.1.255

# handle linux signals by hand
$(TARGET).CFLAGS += -DUSE_LINUX_SIGNAL -D_GNU_SOURCE

# board specific init function
$(TARGET).srcs +=  $(SRC_BOARD)/board.c

# Link static (Done for GLIBC)
$(TARGET).CFLAGS += -DLINUX_LINK_STATIC
$(TARGET).LDFLAGS += -static

# limit main loop to 1kHz so ap doesn't need 100% cpu
#$(TARGET).CFLAGS += -DLIMIT_EVENT_POLLING

# -----------------------------------------------------------------------

# default LED configuration
RADIO_CONTROL_LED			?= 6
BARO_LED           			?= none
AHRS_ALIGNER_LED   			?= 5
GPS_LED            			?= 3
SYS_TIME_LED       			?= 0
