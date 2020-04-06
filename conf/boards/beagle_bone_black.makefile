# Hey Emacs, this is a -*- makefile -*-
#
# beagle_bone_black.makefile
#

BOARD=beagle_bone_black
BOARD_CFG=\"boards/$(BOARD).h\"

ARCH=linux
$(TARGET).ARCHDIR = $(ARCH)
# include conf/Makefile.bbb (to set hard-float, etc) instead of only Makefile.linux
$(TARGET).MAKEFILE = bbb

# -----------------------------------------------------------------------

# The GPS sensor is connected trough USB so we have to define the device
GPS_PORT           ?= UART1
GPS_BAUD           ?= B57600

# The datalink default uses UDP
#MODEM_DEV          ?= UDP0
MODEM_HOST         ?= 192.168.1.255

#
# UART4 (RX P9_11, TX P9_13)
#
MODEM_PORT ?= UART4
MODEM_BAUD ?= B57600


# handle linux signals by hand (CTRL-C twice to stop)
#$(TARGET).CFLAGS += -DUSE_LINUX_SIGNAL
$(TARGET).CFLAGS += -D_GNU_SOURCE

# -----------------------------------------------------------------------

# default LED configuration
RADIO_CONTROL_LED			?= none
BARO_LED           			?= none
AHRS_ALIGNER_LED   			?= none
GPS_LED            			?= none
SYS_TIME_LED       			?= none
