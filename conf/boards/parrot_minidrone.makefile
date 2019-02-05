# Hey Emacs, this is a -*- makefile -*-
#
# parrot_minidrone.makefile
#
# https://wiki.paparazziuav.org/wiki/Ap.parrot_minidrone
#

BOARD=parrot_minidrone
BOARD_VERSION=
BOARD_CFG=\"boards/$(BOARD).h\"

ARCH=linux
$(TARGET).ARCHDIR = $(ARCH)
# include conf/Makefile.parrot_minidrone (with specific upload rules) instead of only Makefile.linux:
ap.MAKEFILE = parrot_minidrone

FLOAT_ABI =
ARCH_CFLAGS = -march=armv5

# -----------------------------------------------------------------------
USER=foobar
HOST?=192.168.3.1
SUB_DIR=paparazzi
FTP_DIR=/data/edu
TARGET_DIR=$(FTP_DIR)/$(SUB_DIR)
# -----------------------------------------------------------------------

# The datalink default uses UDP
MODEM_HOST         ?= 192.168.3.255

# If an external GPS sensor is used via USB to serial interface
GPS_PORT           ?= UART2
GPS_BAUD           ?= B38400

# handle linux signals by hand
$(TARGET).CFLAGS += -DUSE_LINUX_SIGNAL -D_GNU_SOURCE

# board specific init function
$(TARGET).srcs += $(SRC_BOARD)/board.c

# Link static (Done for GLIBC)
$(TARGET).CFLAGS += -DLINUX_LINK_STATIC
$(TARGET).LDFLAGS += -static

# -----------------------------------------------------------------------

# default LED configuration
RADIO_CONTROL_LED  ?= none
BARO_LED           ?= none
AHRS_ALIGNER_LED   ?= none
GPS_LED            ?= none
SYS_TIME_LED       ?= 0
