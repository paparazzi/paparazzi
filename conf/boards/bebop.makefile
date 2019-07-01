# Hey Emacs, this is a -*- makefile -*-
#
# bebop.makefile
#
# http://wiki.paparazziuav.org/wiki/Bebop
#

BOARD=bebop
BOARD_CFG=\"boards/$(BOARD).h\"

ARCH=linux
$(TARGET).ARCHDIR = $(ARCH)
# include conf/Makefile.bebop (with specific upload rules) instead of only Makefile.linux:
ap.MAKEFILE = bebop

# -----------------------------------------------------------------------
USER=foobar
HOST?=192.168.42.1
SUB_DIR=internal_000/paparazzi
FTP_DIR=/data/ftp
TARGET_DIR=$(FTP_DIR)/$(SUB_DIR)
# -----------------------------------------------------------------------

# The datalink default uses UDP
MODEM_HOST         ?= 192.168.42.255

# The GPS sensor is connected internally
GPS_PORT           ?= UART1
GPS_BAUD           ?= B230400

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
RADIO_CONTROL_LED   ?= none
BARO_LED            ?= none
AHRS_ALIGNER_LED    ?= 1
GPS_LED             ?= none
SYS_TIME_LED        ?= 0
